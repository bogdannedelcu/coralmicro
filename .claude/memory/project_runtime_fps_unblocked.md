---
name: VGA45/60 runtime fps init UNBLOCKED (build #98x)
description: Root cause was tHsSettle_EscClk lookup keyed on compile-time DEMO_CAMERA_FRAME_RATE; fixed by keying on g_runtime_fps. All 3 fps × 3 ratios = 100/100.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
VGA30/45/60 all run 100/100 correct via runtime
`sentai.camera.init(1, fps)` + `sys.reset()` self-correcting REPL
idiom.  Bench `_t_fps_bench.py` validates 100 frames per ratio at
1:1 / 2:1 / 3:1 with cam_id verification (`peek5_b40` 5-row sample
classifier vs sensor test pattern).

**Root cause of the historical wedge:**
`libs/camera/camera_support.c` `BOARD_InitMipiCsi` looked up
`tHsSettle_EscClk` keyed on `DEMO_CAMERA_FRAME_RATE` (compile-time
macro = 30) instead of `g_runtime_fps`.  At runtime fps=45/60 the
OV5640 PLL was reprogrammed via `cameraConfig.framePerSec =
g_runtime_fps`, but the CSI receiver's DPHY lane-settling window
stayed locked to the 30 fps row → first MIPI sync mis-sampled →
warm-up `select()` flips never saw clean EOF → drain timeout →
fallback mutex deadlock vs CSI ISR mid-recovery → REPL wedge → 3
boot loops → RECOVERY_MODE.

**Fix:** key the table lookup on `g_runtime_fps`.  Same row stays
correct for compile-time builds since `g_runtime_fps` initialises
to `DEMO_CAMERA_FRAME_RATE`.

**Why to apply:** When adding new resolutions or fps rows, ensure
ALL receiver-side lookups (HsSettle, future T-CLK-PRE, etc.) are
keyed on `g_runtime_fps`, not the macro.  The OV5640 driver's
`framePerSec` argument already does the right thing; the bug class
is the CSI receiver side forgetting to follow.

**Result table** (single-grab `peek5_b40` × 100 frames per ratio):
- VGA30: 1:1 13fps, 2:1 19fps, 3:1 21fps (loop)
- VGA45: 1:1 23fps, 2:1 30fps, 3:1 34fps
- VGA60: 1:1 30fps, 2:1 40fps, 3:1 45fps
All 100/100 correct, 0 scrambled, 0 wrong, parity within rounding.
Loop fps tracks sensor period linearly; p99 ≈ p50 + 1 sensor period.

**sys.reset() persistence:** Persistent flash REQUIRED for runtime
fps switching via sys.reset() — `--ram` flash gets discarded by ROM
bootloader on NVIC reset.  Use `flashtool.py -e sentai_runtime`
(no --ram) when validating fps switching.
