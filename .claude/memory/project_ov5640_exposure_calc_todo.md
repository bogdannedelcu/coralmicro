---
name: TODO — OV5640 exposure register auto-calculation
description: Compute OV5640 exposure (AEC/AGC) registers from sensor frame period so exposure scales correctly when changing fps/resolution (open at 2026-04-25).
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
**TODO:** When the runtime sets a new sensor configuration (resolution
+ fps), the OV5640 exposure registers (AEC max-frame, AGC ceiling, etc.)
must be RE-COMPUTED so the exposure budget matches the new frame
period.  Today they're left at the default the driver wrote at init
time, which is correct for VGA45 but mismatched for any other fps.

**Why this matters:**
- At higher fps (shorter frame period), max exposure rows drop → image
  gets darker.
- At lower fps (longer frame period), AEC can over-expose if max isn't
  raised.
- The current `s_ov5640MipiClockConfigs` table sets PLL but does NOT
  touch exposure regs — so SXGA15 uses the same exposure window as
  VGA45 = wrong by 3×.

**Where to compute:**
- `fsl_ov5640.c` already has `OV5640_AeCfg(...)` (or similar) that
  writes `AEC_MAX_EXPO_*` registers (0x3500..0x3503) and the per-row
  exposure time (HTS × line_count).  Check existing entry points.
- Caller side: in our `BOARD_InitCamera` (libs/camera/camera_support.c),
  after `CAMERA_DEVICE_Init`, write the recomputed exposure regs based
  on `cameraConfig.framePerSec` + `cameraConfig.resolution`.

**Formula sketch** (from OV5640 datasheet §8.2.1.2):
```
frame_period_us = 1_000_000 / fps
max_exposure_rows = HTS_per_line × VTS  (= total clock cycles per frame)
exposure_max_us = (max_exposure_rows × tROW)
```
HTS, VTS depend on resolution mode (already in resolution_param array).
The AEC algorithm caps exposure at the per-frame budget; if we don't
update the cap, low-light frames look black.

**How to apply:** when implementing the runtime camera config refactor
(`sentai.camera.init(streaming, w, h, fps)` per agent.md plan), include
exposure auto-recalc as part of `BOARD_InitCamera(w, h, fps)`.

**Test:** capture JPEGs at SXGA15 vs VGA45 in same lighting; brightness
should be roughly equal post-fix.

Linked context:
- agent.md "Runtime camera config" plan (Phase B)
- camera_support.c csi2rxHsSettle table (where new fps rows are added)
- fsl_ov5640.c s_ov5640MipiClockConfigs (PLL config per fps)
