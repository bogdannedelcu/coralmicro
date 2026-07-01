# s237 — PyTorch-route c2f_thick: export-route EdgeTPU speed sweep (Coral USB stick)

Anchor: `TD-S10-A10/B10` (paper §4.2 performance) · EXP-s237 · follows
[s236](../s236_model_speed_sweep_640x480/) (TF-native cover_v1 sweep) and uses
its offline parser `iter15_offline_exe/dump_exe`.

## What this is

MLflow experiment `sentai_revision_cover_v1` (exp 5) got a **new run**
`cover__c2f_thick_PYTORCH` (run `d62bb783`, 2026-06-26) — a cross-framework
parity study of the revision champion `yolov5n_p3p4_c2f_thick` (751,260 params),
**PyTorch yolov5 `export.py` route vs the TF-native fork** used in s236. See
`../pytorch_vs_tf_report.md`. The run ships THREE edgetpu-compiled variants of the
same architecture, differing only in the export path / activation. This benchmarks
their **pure TPU inference latency on the physical Coral USB Accelerator** (full
`libedgetpu`, host-side), to pick the fastest deployment route.

Input is **480×640×3 uint8** for all (the deployed cover_v1 size).

## Models (run d62bb783)

| tag | MLflow artifact | export route |
|---|---|---|
| `A_main`    | `edgetpu/best-int8_edgetpu.tflite`                       | PyTorch `export.py` (Keras rebuild), detection decode bundled **on-TPU** → output `(1,6000,6)` |
| `B_silu`    | `relu_experiment/best-int8-SILU_edgetpu.tflite`         | SiLU-activation experiment variant |
| `C_onnx2tf` | `onnx2tf_route/best_classic_full_integer_quant_edgetpu.tflite` | onnx2tf export route; emits raw grid maps **and** decoded output (3 tensors) |

## Method

`bench_usbcoral.py` (run with `venv-coral39/bin/python3`): loads each model on the
USB TPU via `pycoral.make_interpreter`, 20 warmup + **200 timed** `invoke()` calls
on a fixed mid-range input (content does not affect TPU timing). Reports the
invoke-only latency (params/instructions are resident after the first call).

```bash
venv-coral39/bin/python3 bench_usbcoral.py     # writes results.csv
```

## Results (Coral USB Accelerator, full libedgetpu)

| model | output | size | mean | median | p95 | min | **FPS** |
|---|---|---:|---:|---:|---:|---:|---:|
| **B_silu** (SiLU)        | `(1,6000,6)`            | 1.06 MB | **35.3 ms** | 35.4 | 36.5 | 33.8 | **28.3** |
| **A_main** (PyTorch route) | `(1,6000,6)`          | 1.22 MB | 69.1 ms | 69.2 | 70.0 | 66.7 | 14.5 |
| **C_onnx2tf**            | 3× (raw+decoded)        | 1.42 MB | 79.9 ms | 80.1 | 82.8 | 72.5 | 12.5 |

## Why B is 2× faster — it avoids the host-scratch spill

`dump_exe` (offline darwinn-executable parse) on the inference executable:

| model | `scratch_size_bytes` | instruction bytes | dma_hints | latency |
|---|---:|---:|---:|---:|
| A_main    | **153,600** | 310,416 | 11 (spill OUTFEED/FENCE/INFEED) | 69 ms |
| B_silu    | **0**       | 150,752 | 5  (no spill)                   | 35 ms |
| C_onnx2tf | **153,600** | 364,176 | 12 (spill + 3 outputs)          | 80 ms |

`A_main` and `C_onnx2tf` are wide enough that their intermediate activations do
**not** fit in the Apex on-chip SRAM, so the compiler emits a **host-mediated
activation spill** (`SCRATCH OUTFEED 153600` → `FENCE` → `SCRATCH INFEED 153600`)
plus ~2× the instructions. `B_silu` keeps everything on-chip (`scratch=0`, half the
instructions) → ~2× the throughput. The activation/export choice, not the
architecture, decides whether the model pays the scratch round-trip.

This is the SAME `scratch_size_bytes=153600` spill that crashed the i.MX RT1176
board before the [s236 fix](../s236_model_speed_sweep_640x480/iter16_scratch_fix/)
(commit `8510b1b7`): `A_main` and `C_onnx2tf` would have hung the board pre-fix and
now run; `B_silu` never touches that path.

## Takeaways

- **`B_silu` is the deployment winner on this hardware**: 28.3 FPS vs 14.5 (A) /
  12.5 (C) — 2× the throughput, smallest graph, and `scratch=0` so it is also the
  lightest on the on-board USB2 path.
- The PyTorch `export.py` route (`A_main`) bundles the YOLO grid-decode on-TPU
  (+36 ops vs TF-native, see the report). That convenience costs a scratch spill
  and halves throughput vs the no-spill SiLU build.
- These are **host (USB3, full libedgetpu)** numbers — an upper bound. The on-board
  (RT1176, USB2, custom single-EP port) figure is transfer-bound and lower (s236
  measured ~24 FPS input-bound for the non-spill TF-native models; the spill models
  pay the extra 153,600-byte OUTFEED+INFEED there too).

## Op-level breakdown — where the instructions come from

Op histogram of the INT8 graphs (pre-EdgeTPU-fusion), all same `c2f_thick`
backbone (52 CONV_2D):

| op | A_main | B (low-LOGISTIC) | C_onnx2tf | role |
|---|---:|---:|---:|---|
| CONV_2D     | 52 | 52 | 52 | backbone compute (native systolic — cheap, irreducible) |
| **LOGISTIC** | **56** | **6** | **52** | sigmoid — **transcendental, NOT native**; element-wise pass per call |
| **MUL**      | **62** | **12** | **60** | element-wise; pairs with LOGISTIC for SiLU = `x·σ(x)` |
| ADD         | 14 | 14 | 14 | residual adds |
| CONCAT      | 11 | 11 | 11 | C2f / FPN concat |
| SPLIT(_V)   | 5  | 5  | 0(+2 SPLIT_V) | C2f channel split |
| STRIDED_SLICE | 6 | 6 | 10 | decode-head gather |
| RESHAPE/PAD/QUANTIZE/MAXPOOL/RESIZE/TRANSPOSE | 18 | 18 | 20 | data movement / requant |
| **TOTAL**   | **224** | **124** | **221** | |

**The SiLU→ReLU win, explained.** `A_main`/`C_onnx2tf` use **SiLU** everywhere:
each activation is `MUL(x, LOGISTIC(x))` → ~52 LOGISTIC + ~52 MUL from the backbone
alone. `B` drops to **6 LOGISTIC + 12 MUL** — its backbone activations **fold into
the conv output clamp (ReLU/ReLU6 family) and emit ZERO ops**. That removed ~50
LOGISTIC + ~50 MUL ⇒ instruction stream **310 KB → 151 KB**, i.e. ≈1.6 KB of TPU
instructions per element-wise op.

**Two costs of SiLU, not one:**
1. *Instructions* — sigmoid is transcendental; the EdgeTPU has no systolic sigmoid,
   so each `LOGISTIC` is a full element-wise activation-unit pass (instructions ∝
   tensor size). ReLU6 is free (fused clamp).
2. *Scratch spill* — `x·σ(x)` needs **two live tensors** (`x` and `σ(x)`) at every
   layer ⇒ peak activation memory exceeds on-chip SRAM ⇒ the compiler emits the
   host-scratch round-trip. Measured: SiLU `A_main` → `scratch=153600` (spill);
   ReLU `B` → `scratch=0` (fits on-chip). So **ReLU also eliminates the spill that
   crashed the board** (s236) and is responsible for most of the 2× latency.

## Where to cut next (after SiLU→ReLU)

1. **Move the YOLO decode off-TPU** (export raw grid maps, decode on the M7) — the
   only transcendental left in `B` is the detection head: its **6 LOGISTIC + 12 MUL
   + 6 STRIDED_SLICE + 4 RESHAPE + part of CONCAT**. The TF-native s236 route already
   does this (188→ raw maps). Removes the last sigmoids and the gather/reshape
   plumbing; the decode is trivially cheap on the host.
2. **Requantize** — 5 `QUANTIZE` ops are scale-bridges between mixed-scale tensors;
   keeping consistent int8 scales (avoid int16 islands) removes them.
3. **Structural** — after that, the residue is `CONV_2D` (irreducible) + `CONCAT/
   SPLIT/ADD/PAD/MAXPOOL`. To cut those you change the **architecture**: `C2f→C3`
   (fewer SPLIT/CONCAT), narrower channels, or drop the SPPF MaxPool chain.

**Activation guidance (EdgeTPU instruction cost):** ReLU/ReLU6 = free (fused clamp,
0 ops). LeakyReLU = cheap (MAX+MUL, no transcendental). **Avoid** SiLU/Swish, GELU,
Mish, Tanh — all expand to transcendental passes (LOGISTIC/EXP/TANH) **and** force
the extra live-tensor scratch spill. There are no other transcendental
nonlinearities in YOLOv5n besides the sigmoid; MaxPool/Add/Concat are not
nonlinearities (native data ops).

## Files

- `bench_usbcoral.py` — the benchmark
- `models/` — the 3 edgetpu variants (from MLflow run d62bb783)
- `results.csv` — raw numbers
- `../pytorch_vs_tf_report.md` — the upstream cross-framework parity report
