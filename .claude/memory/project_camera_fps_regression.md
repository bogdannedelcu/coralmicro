---
name: Camera VGA30 delivers 18 fps not 30 (regression in current build)
description: OV5640 + CSI delivers 18.5 fps default, 28 fps with init(1, 45). Was 45 fps in build #1077. Out-of-scope for flow.
type: project
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
**Symptom:** `sentai.camera.frame_count()` measured ISR rate on
build #1112 baseline (no flow changes):
| init args | measured ISR fps |
|---|---:|
| `init(1)` (g_runtime_fps default = 30) | 18.5 |
| `init(1, 30)` explicit | 18.5 |
| `init(1, 45)` explicit | 28.0 |

**History:** experiment.md "VGA45 pipeline FPS" session shows the
TPU pipeline hitting 45.9 fps on the same hardware in build #1077.
Now we measure 28 fps even with explicit init.  Regression is
upstream of any frame consumer.

**Where to look:**
- OV5640 register table for VGA/30 in `fsl_ov5640.c` may have
  drifted from a working version
- PLL / divider config in `BOARD_InitCameraResource`
  (`libs/camera/camera_support.c`)
- Some default ratio / alt-mode that halves per-cam delivery

**Why:** Flow stack (and any other camera consumer) is bound by
this rate.  Flow algorithm completes in 2.3 ms / frame and would
sustain ~400 fps if camera delivered.  No fix needed in flow.

**How to apply:** Don't try to "fix" flow throughput by changing
flow code -- it's already at the camera ceiling.  A separate
session needs to bisect the OV5640 driver vs the working VGA45
baseline build (~#1077).

**Re-init crashes:** `sentai.camera.init()` after an existing
init returns -11 (camera already initialized).  One init per
boot.  Re-init pathway is not re-entrant on this HAL.
