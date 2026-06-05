---
name: Runtime fps switch blocked — needs BOARD_InitCamera split
description: 2026-04-26 attempts at runtime fps switching (Disable+Enable, SetPower cycle, OV5640 PLL direct-write, software reset) all wedge the CSI receiver. Root cause: NXP HAL doesn't tolerate any MIPI lane disturbance + CAMERA_RECEIVER_Init isn't re-entrant.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
Tried four implementations of `sentai.camera.set_fps(fps)` to allow runtime fps switching without rebuild+reflash. All wedged the CSI receiver / USB CDC. Diagnosis: any approach that briefly drops the OV5640 MIPI lane signal causes the receiver to enter a fault state it can't recover from without full BOARD_InitCamera, but BOARD_InitCamera isn't re-entrant on this NXP HAL drop.

**Approaches tried (all wedge):**

1. `Disable() + SetPower(false) + sleep + SetPower(true) + Enable()` — full power cycle. Wedges on second iteration (matches the "first reinit works; second hangs" note in `project_runtime_camera_hw.md`).
2. `Disable() + sleep + Enable()` — soft reconfigure, no power cycle. Wedges immediately on Enable() because HandleEnableRequest re-runs CAMERA_RECEIVER_Init which is NOT re-entrant in fsl_csi.c.
3. **OV5640 PLL direct-write via I²C** (write 0x3008=0x42 standby → write 0x3035/0x3036/0x4837 → write 0x3008=0x02). I²C writes themselves succeed (no `[CAM] W` errors), but the brief MIPI-clock pause when entering standby causes USB CDC to drop on the host side — board re-enumerates a moment later but the consumer process has died.
4. **OV5640 software reset** (write 0x3103=0x11 + 0x3008=0x82) — same MIPI disturbance + needs full re-init sequence afterward, equivalent to approach #2.

**Why all four wedge:** the RT1176 CSI2RX D-PHY is configured ONCE during BOARD_InitCamera with a specific T-HSSETTLE / lane-rate setup that matches the OV5640's lane rate at boot fps.  Any change to the OV5640's lane rate (or any pause in the MIPI HS-mode signal) puts the D-PHY into a state the NXP HAL expects to clear via CAMERA_RECEIVER_Init — which isn't safely callable a second time.

**The real fix (deferred — multi-day refactor):**
Split `BOARD_InitCamera` in `libs/camera/camera_support.c` into:
- `BOARD_InitCamera_OneTime()` — RECEIVER_Init, BOARD_InitMipiCsi, NVIC_SetPriority. Called once at boot.
- `BOARD_InitCamera_PerFps()` — CAMERA_DEVICE_Init, CAMERA_DEVICE_Start, SubmitEmptyBuffer. Re-callable each time fps changes; also re-issues the CSI2RX D-PHY config with the correct T-HSSETTLE for the new lane rate.

Then `sentai_cam_set_fps` would: cam->Disable() → write g_runtime_fps → BOARD_InitCamera_PerFps() → cam->Enable(). Cleanly re-tunes both ends of the lane.

**For now the workflow remains:**
- Edit `DEMO_CAMERA_FRAME_RATE` in `libs/camera/camera_support.h:107` (default = 30).
- `make -C build -j8 sentai_runtime`.
- `python3 scripts/flashtool.py -e sentai_runtime --ram` (RAM flash is fast, no wedge).
- Verify with `diag/_t_cam_init_diag.py`: register dump at boot confirms the patch values applied.

**Validated (build #957/977, two consecutive boots each fps):**
- VGA30: 0x3036=0x38, 0x4837=0x14 → matches patched table.
- VGA45: 0x3036=0x54, 0x4837=0x0D → matches.
- VGA60: 0x3036=0x70, 0x4837=0x0C → matches.

**Don't repeat:**
- Don't add a `set_fps()` API that touches OV5640 registers via I²C while CSI is streaming. The MIPI disruption WILL wedge the receiver every time, regardless of how briefly.
- Don't rely on `cam->Disable() + cam->Enable()` to reconfigure — HandleEnableRequest re-runs the full BOARD_InitCamera which has the second-call hang.
- The proper fix is the BOARD_InitCamera split above; nothing else will be reliable.
