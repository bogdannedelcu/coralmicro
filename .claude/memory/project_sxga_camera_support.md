---
name: SXGA (1280×960) camera mode — driver + linker shipped, 15 FPS inits OK
description: 2026-04-24. Added SXGA 1280×960 support to OV5640 driver (ISP binned 2x2). `sentai.camera.init(1, 1280, 960, 15)` returns 0 with clean init sequence. 30 FPS clock config added but untested. Post-init REPL may wedge — needs investigation.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
## What shipped

1. **OV5640 driver** (`third_party/nxp/rt1176-sdk/components/video/camera/device/ov5640/fsl_ov5640.c`):
   - `resolutionParam` entry for `FSL_VIDEO_RESOLUTION(1280, 960)` — register
     values for 2x2 binning from QSXGA, output 1280×960.  Cloned framing
     from VGA (same binning) but with outW=0x0500, outH=0x03c0.
   - `s_ov5640MipiClockConfigs` entries for SXGA @ 15 (pllCtrl1=0x41)
     and SXGA @ 30 (pllCtrl1=0x21), both with pllCtrl2=0x54, vfifoCtrl0C=0x20,
     pclkDiv=0x04, pclkPeriod=0x0a (cloned from 720P row + 1080P/15).

2. **CSI HS-settle** (`libs/camera/camera_support.c`):
   - Added `FSL_VIDEO_RESOLUTION(1280, 960)` @ 15 → 0x17,
     @ 30 → 0x12.

3. **Linker** (`examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`):
   - `m_ncamera` 16 MB → **24 MB** (ORIGIN=0x82000000, LENGTH=0x01800000).
     Covers 4 × 4.9 MB SXGA framebuffers + 2.25 MB .pxp_ring.
   - `.ncamera (NOLOAD)` — skips zero-init at boot (was too slow with
     19.6 MB).

4. **Compile-time framebuffer size** (`libs/camera/camera_support.h`):
   - New `DEMO_CAMERA_MAX_HEIGHT=960`, `DEMO_CAMERA_MAX_WIDTH=1280`
     used by `camera_support.c:framebuffers[]` array.
   - Legacy `DEMO_CAMERA_HEIGHT=480`, `DEMO_CAMERA_WIDTH=640` kept as
     RUNTIME DEFAULTS (what `g_camera_hw_w/h` initialise to).

5. **MP validators** (`examples/sentai_runtime/modsentai_camera.c`):
   - Both `set_hw()` and `init(streaming, w, h, fps)` accept
     `(1280, 960, 15|30)`.

## Supported modes (summary)

| Resolution | FPS options | Notes |
|---|---|---|
| QVGA (320×240) | 15, 30 | |
| VGA (640×480) | 15, 30, **45** (sentai-added) | |
| **SXGA (1280×960)** | **15, 30** (sentai-added 2026-04-24) | 2×2 binned, same sensor read as VGA |

## Test status

**SXGA @ 15 init**: VERIFIED WORKING on fresh flash.
Output sequence:
```
CAMERA_RECEIVER_Init = 0
OV5640 power-on + reset pins toggled
CAMERA_DEVICE_Init = 0
CAMERA_DEVICE_Start = 0
CAMERA_RECEIVER_Start = 0
rc = 0
```

**Known issue**: post-init REPL commands may wedge. Possible causes:
- SXGA CSI DMA bandwidth (36.9 MP/s @ 30, 18.4 MP/s @ 15) may saturate
  SEMC enough to starve MP task (which runs at lower priority).
- Need to confirm frame_count advances before running pipeline.
- SXGA @ 30 clock config is aggressive — untested.

## Why: What to apply

For next session continuing SXGA experiment:
1. Test `init(1, 1280, 960, 15)` on fresh flash, then verify
   `frame_count()` advances (should see ~15 per second) BEFORE
   trying pipeline.
2. If SXGA streams OK, run pipeline at SXGA — expected to be
   significantly slower than VGA due to:
   - PXP scaling 1280×960 → 512×512 takes longer (more source pixels)
   - SDRAM traffic 4× higher for camera write (but binned, so sensor
     read is same as VGA — SDRAM WRITE is 4× VGA)
3. Compare against VGA@45 baseline (42 FPS pipeline).
4. If SXGA @ 30 works, measure delta vs @ 15.

Do NOT delete SXGA support if experiment shows slower pipeline —
it's a valid mode for detection-at-distance use cases.

## Refactor (2026-04-24): compile-time DEMO_CAMERA_* → runtime g_camera_hw_*

Hot path uses runtime dims so SXGA captures the right buffer size:
- `camera.cc HandleReturnRequest`: DCACHE range = `g_camera_hw_h × (g_camera_hw_w + LINE_PADDING) × BPP`
- `detection_task.cc PrepTask`: `sentai_pxp_scale(raw, g_camera_hw_w, g_camera_hw_h, ...)` (both V23 and legacy branches)
- `flow_task.cc`: same
- `camera_support.c BOARD_PxpConfig`: PXP scaler sizes use runtime
- `camera_support.c csi2rxHsSettle lookup`: runtime w/h/fps match

REPL helpers in `sentai_runtime.cc` (`sentai_cam_capture_rgb`, `_jpeg`,
`_to_tensor`, snapshots) KEPT at compile-time `DEMO_CAMERA_WIDTH/HEIGHT`
(VGA) to save ITCM space.  **Bug**: these REPL paths are wrong when
user is in SXGA mode — they'd pass VGA dims to PXP on SXGA source
→ garbage output.  Acceptable trade-off since these are for 1-shot
debug, not pipeline operation.  If needed, move them to
`sentai_slow_bridge.cc` to reclaim ITCM for runtime dims.

Bounds checks (`sentai_cam_set_res`, `_capture_*`, `_snapshot_*`)
updated to use `DEMO_CAMERA_MAX_WIDTH/HEIGHT` (1280/960) so SXGA
dims aren't rejected.

## Last-known state 2026-04-24

Post-refactor build compiles.  Board boots, REPL works, init(1,640,480,45)
returns 0.  But subsequent pipeline tests wedge REPL after ~1 run.  Same
flakiness as earlier runtime-reinit bugs.

Possible root cause: the `.ncamera` 24 MB BSS zero-init at boot leaves
`framebuffers` addresses + MPU relationship different from pre-refactor,
and something in the NXP CSI driver is latching bad state after first
pipeline start.  Next session: investigate whether the CSI IRQ/queue
state is proper after pipeline.stop at new buffer layout.

## Framebuffer allocation size

```
DEMO_CAMERA_BUFFER_COUNT × MAX_H × MAX_W × BPP = 4 × 960 × 1280 × 4 = 19.66 MB
```

Plus `.pxp_ring` 2.25 MB = 21.9 MB.  `m_ncamera` 24 MB region has 2.1 MB headroom.
