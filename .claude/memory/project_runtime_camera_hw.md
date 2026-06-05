---
name: Runtime camera HW config — set_hw() works once, then hangs
description: 2026-04-24. Added sentai.camera.set_hw(w,h,fps) runtime API. First reinit works; second or with pipeline.start fails — CAMERA_RECEIVER_Init / CAMERA_DEVICE_Init aren't fully idempotent.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
## Shipped API

**Preferred: single-init-per-boot via init params**
`sentai.camera.init(streaming=1, w, h, fps)` — params optional; when all 3
are given, applies them before HW init.  Returns 0=ok, -1=invalid combo.

```python
# main.py example
import sentai
sentai.camera.init(1, 640, 480, 30)  # VGA @ 30 FPS
```

After the first init in a session, subsequent re-inits may be unstable
(driver state isn't fully idempotent).  To switch config: call
`sentai.sys.reset()` (warm reboot) then init with new params.

**Legacy runtime setter** (works ONCE reliably):
`sentai.camera.set_hw(w, h, fps)` → 0=ok, -1=invalid combo, -2=camera still running.

`sentai.camera.hw_config()` → `(w, h, fps)` tuple with current values.

Valid tuples (per OV5640 MIPI clock table, `fsl_ov5640.c:s_ov5640MipiClockConfigs`):
- VGA (640, 480): 15, 30, **45** (sentai-added)
- QVGA (320, 240): 15, 30
- 720P / 1080P entries exist in driver but framebuffers are sized for VGA — won't fit.

Workflow:
```python
sentai.camera.stop()
sentai.camera.set_hw(640, 480, 30)
sentai.camera.init(1)
```

## Implementation

`libs/camera/camera_support.{h,c}`:
- Added globals `uint8_t g_camera_frame_rate`, `uint16_t g_camera_hw_w`, `uint16_t g_camera_hw_h`
- `BOARD_InitCamera()` reads these globals instead of `DEMO_CAMERA_*` constants — called at each `cam->Enable()` (inside `HandleEnableRequest` in camera.cc).
- Also HS-settle lookup uses globals (else MIPI timing off).

`examples/sentai_runtime/modsentai_camera.c`:
- `mod_sentai_cam_set_hw` validates tuple, refuses while camera live, writes globals.
- `mod_sentai_cam_hw_config` returns tuple.

## Works / Fails

Works:
- First boot → VGA45 default init → read hw_config = (640,480,45) ✓
- `stop() → set_hw(640,480,30) → init()` → hw_config = (640,480,30), frame counter advancing ✓ (tested interactively)

Fails:
- Second reinit in same session (e.g. VGA30 → VGA15) → camera driver hangs
- Or `set_hw → init → pipeline.start + ratio(1,1)` matrix → hangs mid-matrix
- Pattern: subsequent `CAMERA_DEVICE_Init` / `CAMERA_RECEIVER_Init` doesn't cleanly
  reset OV5640 SCCB state or CSI peripheral state.

## Why: What to apply

If future sessions need runtime FPS iteration across many values:
1. **Safer**: 3 firmware builds with `DEMO_CAMERA_FRAME_RATE = 45/30/15` respectively.
   Reflash between matrices.  Per-build takes ~2 min.
2. **Fix set_hw properly**: inspect what state survives `cam->Disable + SetPower(false)`.
   Likely need explicit MIPI CSI2RX deinit + re-init via `BOARD_InitMipiCsi()` or
   direct register reset.  Look at `CAMERA_DEVICE_Deinit`, `CAMERA_RECEIVER_Deinit`
   (do they exist in SDK?).

Before proposing either:
- Read experiment.md latest — runtime reinit status may have been worked on.
- Check camera.cc HandleEnableRequest to see if it already re-calls BOARD_InitCamera
  (it DOES — so why reinit #2 hangs remains open).

## Measured data — alt 1:1 @ VGA45 (camera default)

Matrix of `pipeline.prep_fps(pf) × pipeline.invokes_per_frame(ipf)`:

| pf  | ipf | FPS   | fails |
|-----|-----|-------|-------|
| 0   | 1   | 20.3  | 0     |
| 30  | 1   | 20.7  | 0     |
| 30  | 2   | 34.0  | 0     |
| 15  | 4   | 44.3  | 0     |

- Baseline alt 1:1 ceiling ~20 FPS (2 cameras × 22 ms invoke serialized)
- `ipf=4` gives 44 FPS total TPU but same 11 camera frames/sec × 4 invoke each
- Useful for multi-patch workloads, NOT for fresh-frame-per-detection

V30 / V15 alt 1:1 matrices unmeasured due to runtime reinit hang.

## Related

- `DEMO_CAMERA_BUFFER_BPP = 4` (XRGB8888 in RAM) is HW-enforced by NXP
  MIPI→CSI pipeline — can't be reduced to 2 for RGB565.  See experiment.md.
- `camera.ratio(1,1)` pair with `switch_drain(1)` = 19.5 FPS baseline from
  experiment.md §9.  Our 20.3 matches.
