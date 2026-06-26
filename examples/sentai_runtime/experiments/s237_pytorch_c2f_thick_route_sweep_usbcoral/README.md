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

## Files

- `bench_usbcoral.py` — the benchmark
- `models/` — the 3 edgetpu variants (from MLflow run d62bb783)
- `results.csv` — raw numbers
- `../pytorch_vs_tf_report.md` — the upstream cross-framework parity report
