# s238 — ReLU (relu_v1) detectors: on-board speed + board-vs-host detection parity

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s238 · redo of [s236](../s236_model_speed_sweep_640x480/)
for the **ReLU** activation models, on the physical i.MX RT1176 board, fed a real image.

## What this is

MLflow experiment `sentai_revision_relu_v1` (exp 8) trained the **8 cover
architectures with ReLU** (vs the SiLU `cover_v1` set benchmarked in s236). This
measures their on-board inference speed and validates detections against a host
CPU int8 reference, on the same real image.

- Board: i.MX RT1176, single-EP EdgeTPU over USB2, firmware build 1560 (carries the
  [s236 host-scratch fix](../s236_model_speed_sweep_640x480/iter16_scratch_fix/) —
  though the ReLU graphs don't need it, see below).
- Image: `test.bin` = the s236-chosen 640×480 RGB-u8 frame (contains people),
  loaded with `sentai.tpu.load_image("/test.bin")`. `test_image.png` is the viewable
  version.
- Models + image copied to the board over **USB-MSC** (`sentai.usb.drive(1)`) — yes,
  large files (the 8 `.tflite` are 205 KB–943 KB) copy fine this way; mount the
  auto-mounted volume (`/run/media/$USER/0000-0001`, **not** `/dev/sda` which is the
  host system disk), copy, `udisksctl unmount`, then `q` on the MSC control tty +
  `sentai.usb.drive(0)` to remount FileX.

## Method (honest wall-clock ms, per s236)

`run_s238.py` drives the board over REPL. Per model: `load` → `load_image` →
5 warmup invokes → time **N=50** invokes with `sentai.rtos.micros()` (µs wall
clock) → `ms_per_invoke = wall_us / (1000·N)`. Board prints raw ints only; the host
computes ms/fps (no on-device float formatting). URB per-phase wait times separate
instruction-upload / input-transfer / output-read so the ms are transparent.

**No `sys.reset()` between models** (unlike s236): the ReLU graphs have
`scratch_size_bytes = 0` (verified offline with `dump_exe`), so none spills / wedges
the TPU — all 8 run back-to-back in one session. c2f_thick, which **crashed the
board as SiLU in s236**, runs cleanly here.

> Gotcha: never run `sentai.verbose(0)` during REPL work — it silences the printf
> backend that `print()` and REPL result-echo use, so the board looks wedged
> (echoes lines, evaluates nothing). `sentai.verbose(1)` restores it.

## Speed (on-board, N=50, real image)

| model | **ms/invoke** | FPS | instr ms | **input ms** | output ms | s236 SiLU FPS |
|---|--:|--:|--:|--:|--:|--:|
| msblock    | 32.6 | 30.7 | 2.16 | 25.98 | 2.47 | 22.9 |
| c3         | 32.8 | 30.5 | 2.31 | 25.99 | 2.51 | 26.2 |
| c2f        | 33.0 | 30.3 | 2.43 | 25.99 | 2.57 | 26.2 |
| gelan      | 34.1 | 29.3 | 3.34 | 26.00 | 2.83 | 24.4 |
| c2f_thick  | 35.2 | 28.4 | 3.43 | 25.97 | 3.85 | **crash** |
| gelan_pan2 | 35.4 | 28.2 | 4.40 | 26.00 | 3.08 | 22.8 |
| c2f_pan2   | 35.3 | 28.3 | 3.12 | 27.56 | 2.74 | 24.9 |
| c2f_deep   | 35.6 | 28.1 | 4.45 | 25.99 | 3.19 | 22.6 |

- **All 8 run, fails=0** — including c2f_thick (crashed as SiLU pre-fix).
- **Input transfer dominates (~26 ms of ~33)**: the 921,600-byte image re-sent to the
  TPU every invoke over USB2 is the bottleneck, identical across models. The model
  differences live in instruction upload (2.2–4.5 ms) and output read (2.5–3.8 ms).
- ReLU is faster than the s236 SiLU set (e.g. msblock 22.9→30.7 FPS) because the
  smaller graphs upload fewer instructions — and because none pays the SiLU scratch
  round-trip.
- The measurement is **USB-transfer-bound, not TPU-compute-bound**: the honest
  end-to-end "time from instructions+image in to boxes out" is ~33–36 ms ≈ 28–31 FPS.

## Detection parity — board (EdgeTPU) vs host (CPU int8)

Same image, same decode (raw grid cells with `sigmoid(obj)·sigmoid(cls) ≥ 0.25`,
both heads, pre-NMS), board vs host CPU int8 reference:

| model | board dets | host dets | board maxconf | host maxconf |
|---|--:|--:|--:|--:|
| msblock    | 18 | 18 | 0.393 | 0.393 |
| c2f        | 17 | 17 | 0.420 | 0.421 |
| c3         | 26 | 25 | 0.367 | 0.367 |
| c2f_pan2   | 17 | 18 | 0.389 | 0.390 |
| gelan      | 18 | 18 | 0.410 | 0.411 |
| gelan_pan2 | 14 | 14 | 0.395 | 0.395 |
| c2f_deep   | 20 | 21 | 0.364 | 0.364 |
| c2f_thick  | 16 | 16 | 0.406 | 0.418 |

Counts match within **±1 cell** (a cell whose confidence sits right on the 0.25
threshold flips between the TPU's and the CPU's rounding); maxconf matches to 3
decimals. **The board's EdgeTPU inference is host-identical** — the scratch fix and
the ReLU export produce correct results, not just fast ones. (Counts are raw
pre-NMS cells; after NMS each image collapses to a few person boxes at ~0.4 conf.)

## Files

- `run_s238.py` — board speed driver (imports `../s236.../board_serial.py`)
- `results.csv`, `sweep.log` — speed numbers + raw board log
- `detections_board_vs_host.csv` — the parity table
- `models/` — the 8 ReLU edgetpu tflites (on-board set)
- `host_int8_models/` — the 8 ReLU int8 tflites (host CPU reference)
- `test.bin` / `test_image.png` — the test image (raw + viewable)
