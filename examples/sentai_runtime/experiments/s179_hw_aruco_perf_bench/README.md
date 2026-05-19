# s179 — HW ArUco detect perf bench (320×240 Y8 × 10 on real M7)

## What this proves

`sentai_aruco_detect()` runs in **X µs per frame on real M7 @ 800 MHz**
when fed a 320×240 grayscale frame, measured over 10 in-memory
iterations against a SIM-captured PGM.  Number drives the embedded-
engineering chapter of the thesis and prepares the ground for
`OP-S9-W4` (DWT cycle-counter instrumentation per stage).

This is the **first** real-HW exercise of the post-T22 ARM stack
(W12 + W13 + W14 build) — also doubles as a Lane-A sanity check
that the firmware boots, the MP REPL responds, and the new
`sentai.safety` / `sentai.fr` / `sentai.calib` modules are reachable
from MicroPython (T2 sanity is bundled into `bench_runner.py`).

WBS: `OP-S10-W11` (no algorithm change, infrastructure use) +
companion to `OP-S9-W4` HW timing prep.

## How to run

```bash
cd examples/sentai_runtime/experiments/s179_hw_aruco_perf_bench/
./run.sh
```

The driver:

1. Switches the board to USB MSC mode (`sentai.usb.drive(1)` →
   warm reset → `/dev/sda` enumerates as FAT16).
2. Mounts the FxUser volume via `udisksctl` (no sudo) and copies
   `frame.pgm` → `/aruco_test.pgm`.
3. Sends `q\r\n` to `/dev/ttyACM0` to warm-reset back to default
   mode (REPL + CDC-NCM).
4. Opens REPL, prints `sentai.version()` and `hasattr` matrix for
   the W12/W13/W14 module surface.
5. Calls `sentai.aruco.init()`, sets generic intrinsics + marker
   size, then loops 10 × `sentai.aruco._test_pgm()` reading
   `get_stats()['last_detect_us']` each time.
6. Writes a `results.txt` with min / median / mean / max µs.

If the FS occasionally returns "absent" or a wrong size after the
MSC warm reset (FxUser mount sometimes needs a beat), the verifier
re-polls a few times before aborting with a clear error message.

Use `--skip-upload` to re-run the bench against an already-uploaded
PGM (useful when iterating on the script while the file is stable
on board).

## Inputs

- `frame.pgm` — 320×240 grayscale, copied from
  `s175_pnp_planar_ambiguity/frame_original.pgm` (SIM-rendered scene
  with 4 ArUco markers above the camera, drone at z ≈ 1.05 m).

## Pass criteria

This is a **measurement experiment**, not a pass/fail gate.  Sanity
floor: median `detect_us` should fit inside a 33 ms frame budget
(< 33 000 µs), otherwise the perception loop can't keep up with a
30 FPS camera.

## Outputs

- `results.txt` — version banner + module-availability map + the
  10 iteration timings + kernel breakdown.

## First-run numbers (build #1343, 2026-05-19)

Real M7 @ 800 MHz, single 320×240 synth frame.

| Stage                                       |   Time |
|---------------------------------------------|-------:|
| Raw memory scan (BW floor)                  |  830 µs |
| Naive 7×7 scalar mean                       | 12.2 ms |
| Bradley integral-image (current production) | 16.3 ms |
| **PXP HW scale + CPU compare**              | **2.8 ms** |
| Sobel 3×3 \|dx\|+\|dy\|                     | 3.8 ms |
| **End-to-end detect (build #1347, pre-SIMD)** | **24 ms** |
| **End-to-end detect (build #1353, +CMSIS-DSP)** | **22 ms** |

End-to-end detect at 22 ms / frame = 45.5 FPS equivalent, or 66 %
of the 33.3 ms slot at 30 FPS.  Fits with headroom for camera
ISR + flow + safety + FR + RTOS scheduler jitter.

The CMSIS-DSP path is byte-identical to the scalar reference
(0 mismatches at block ∈ {7, 23, 51, 101, 201} via
`sentai.aruco._verify_threshold(block)`).  Interior 4-wide
SIMD compare via inline-asm `usub8` / `sel` wrappers (CMSIS
intrinsics — toolchain `arm_acle.h` doesn't ship them on this
gcc 9.3 SDK).  Border columns stay scalar (per-pixel
`x_factor` clamping).

## Optimization roadmap (filed as separate WBS tasks)

- **OP-S10-W14-T18-S** — Switch `sentai_aruco_detect` threshold from
  Bradley integral-image to the PXP path validated above.  Expected
  win: per-frame detect 24 → ~10 ms (5.8× on the threshold stage).
  Gates: s127 FlowBaseline + s174 yaw frame set must not regress.
- **Task #50** — CMSIS-DSP / `__USADA8` SIMD path for the integral-
  image inner loop, as a fallback if PXP shifts detection accuracy.
- **OP-S10-W15-T5** — Cache-line align the hot ArUco buffers
  (`s_integral`, `s_binary`, `s_labels`) to 32 B on the M7 D-cache.

## Caveats discovered

- `sentai_aruco_test_synth_and_detect` currently returns 0 markers
  on every (marker_id, side_px) combo post the T18 cv2-parity work.
  Real captured frames still detect 90% 4/4 (s174 yaw mission), so
  the regression is specific to the synthetic geometry — likely a
  contour winding / convex / perim gate that's tuned for noisy real
  frames and rejects perfect synth edges.  Filed as a separate bug.
- `s_stats.last_detect_us` is never written by `sentai_aruco_detect`;
  the MP binding exposes the field but it always reads 0.  Bench
  uses `sentai.rtos.ticks_ms()` instead — millisecond precision is
  adequate for a 20+ ms budget.  µs-precision DWT wiring filed
  separately.
- PXP needs `sentai.camera.init()` to have run before its register
  block is out of SFTRST/CLKGATE.  The bench now kicks camera init
  before the kernel bench; without it the PXP path reads 127 ms
  (busy-wait DWT timeout, not a real PXP cycle count).

## Cross-refs

- `examples/sentai_runtime/aruco_bench.cc` — kernel-level bench used
  for the sub-stage timing column above.
- `diary/2026-05-19.md` Lane A — operator scope brief.
