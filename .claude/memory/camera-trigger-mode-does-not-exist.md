---
name: camera-trigger-mode-does-not-exist
description: 2026-09-04 (EXP-s245). Camera trigger mode is dead code and GPIO8_IO27 is not routed — dual-camera capture must go through the free-running path.
metadata: 
  node_type: memory
  type: project
  originSessionId: d5a3f91c-f734-4950-82fe-b796eef65638
  modified: 2026-09-04T07:07:55.050Z
---

**Do not try to build a hardware-triggered camera capture path on this board.**
EXP-s245 (`experiments/s245_camera_trigger_hw_probe/`, builds 1595/1596) closed
this with measurements, not reading.

**1. Trigger mode is inherited dead code from upstream coralmicro.**
- `CameraMode::kTrigger` is defined and documented (`libs/camera/camera.h:42`)
- `CameraTask::Trigger()` is an **empty stub** (`libs/camera/camera.cc:746`)
- the `kTrigger` gates in `GetFrame()` are **commented out** (`camera.cc:490-510`)
- `HandleEnableRequest(mode)` **ignores `mode`** and always calls
  `CAMERA_RECEIVER_Start` (`camera.cc:896`)

So `sentai.camera.init(streaming=0)` accepts the argument and silently gives
streaming anyway. Anything relying on `streaming=0` meaning "trigger" is wrong.

**2. `Gpio::kCameraTrigger` (GPIO8_IO27, pad `GPIO_EMC_B2_17`) is not routed.**
- iter2: the pin follows both internal weak pulls, 5/5 stable runs → nothing
  drives the net. Validated against the adjacent known-routed `kEdgeTpuPgood`,
  which correctly held against the pull-down.
- iter3: making either OV5640 drive its output pads (all implemented `0x3016`
  bits) never moved the pin → no sensor output pad reaches it.
- Electrically safe to re-mux: `SEMC_DM03` is unused because the SDRAM is
  `kSEMC_PortSize16Bit` (`board_hardware.c:76`). SDRAM canary verified clean.
- Unexcludable by construction: a FREX *input* never drives, so it is invisible
  to a listening test. Settling that needs the schematic or a scope.

**3. What DOES exist: a sensor-side software trigger.** The OV5640 FREX block
(`0x3B00-0x3B0C`) is live and writable on both sensors, including `0x3B08`
FREX_REQUEST. Value is **determinism and cross-camera sync, not throughput** —
the frame still traverses MIPI and I²C paces the request. Untested as of s245.

**Where the FPS headroom actually is.** s243 measured 1:1 alternation at
21.6 fps aggregate — exactly half the 45 fps sensor rate — while
`CSI_IRQHandler` flips the MUX inside VBLANK with a ~1500× timing margin, so the
next DMA buffer should already be clean. Roughly one good frame per flip is
discarded. Suspects, all in source: `g_cam_dirty_consecutive_n`, `switch_drain`,
and the `vTaskDelay(1 ms)` quantization in both wait loops. Worth up to **2×**.
That is s245 iter4, not yet run. Respect [[csi-isr-is-timing-sensitive]] — the
plan is to hang a high-priority task off the `vTaskNotifyGiveFromISR` rendezvous
the ISR already performs, adding **zero** instructions to the ISR body.

**Two measurement traps hit while proving this** (both produced confident wrong
answers before being caught):
- A host parser matched the REPL's **echo** of `pin_probe(1)` instead of the
  result, so every read returned `None` and the script still printed a verdict.
  Always abort on a failed baseline read — an instrument reading nothing cannot
  produce a negative result.
- Judging a pin on its **no-pull** reading manufactures false positives: a
  floating CMOS input holds residual charge and drifts. Judge on the pull-up and
  pull-down columns, which a genuinely driven pad forces.
