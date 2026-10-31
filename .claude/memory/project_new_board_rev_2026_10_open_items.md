---
name: new-board-rev-2026-10-open-items
description: "New board revision (schematic 10/30/2026) — CAM1 moved I2C5→I2C1 (FlexPWM1 pin must stay low), mic/SD changes; board enumerates as SDP (1fc9:013d) on plain replug and persistent flash does not boot — only --ram loads run. User partition in SAFE MODE."
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-30T21:36:53.073Z
---

Operator-stated 2026-10-30 (session with the first SD-card bring-up on the
new hardware).  Schematic PDF from the electronics colleague:
`~/.claude/uploads/.../ad21b268-Coral_Dev_Board_Micro.pdf` (19 pages, Altium).

**Hardware deltas vs the Google Coral Dev Board Micro (per operator + sheet 6):**
- **Camera 1 is on LPI2C1 (pads GPIO_AD_32/33), camera 2 on LPI2C6; PMIC
  alone on I2C5.**  DONE 2026-10-30 (build 1622, uncommitted):
  `main_freertos_m7.cc` creates the LPI2C1 handle (24 MHz source clock,
  pads open-drain 0x10) and passes it as cam_id 0 to `CameraTask::Init`.
  **Gotcha that cost 5 builds:** the CAM1 I2C isolation switch U9
  (74LVC2G66) is enabled by VDD_1V8_CAM1 but pulled off by Q7 whose gate
  is `FLEXPWM1_PWM0_A` = **GPIO_AD_00**; that pad powers up with its
  internal pull-up ON (SW_PAD_CTL 0x0E → pin HIGH) → camera 1 NAKs every
  SCCB transfer (LPI2C status 902 = Nak).  Fix: pad 0x06 (pull-down) +
  GPIO2.31 output LOW before camera init (GPIO_AD_01/PWM0_B likewise for
  the add-on I2C switch).  `CameraTask::Detect()` gives a FALSE POSITIVE
  for a dead bus (Read() fills the buffer from the other camera first and
  Detect checks the value) — use `sentai.camera.reg_read(cam, 0x300A)`
  per bus.  Both cameras verified: ID 0x5640, 30 fps, JPEG captures to SD.
  `sentai.camera.power_rails(on)` toggles PMIC LDO1–4 (new binding).
- **Peripheral inventory vs drivers (schematic text + sheets 3–16, 2026-10-30):**
  | part | sheet | driver | status |
  | PMIC MCP16701 (addr 0x5B, CRC-8) | 3/4 | `libs/pmic` | same part; camera LDO1–4 default ON (cameras work without `power_rails`) |
  | LDO TCR2LN18 → VDD_1V8_INT (mic + IMU), CONTROL = `VDD_1V8_INT_EN` GPIO_AD_21/GPIO9.20 | 4 | `main_freertos_m7.cc` writes **0** ("Enable" comment) | POLARITY TO VERIFY — active-high on this LDO |
  | 2× OV5640 via PI3WVR626 MIPI mux, DSC1001 24 MHz + PL133-27 buffer | 9 | `libs/camera` | OK after I2C1 + PWM0_A fix |
  | T5838 PDM mic: CLK GPIO_LPSR_00, DATA GPIO_LPSR_01, **THSEL = GPIO_AD_19**, WAKE = GPIO_AD_18 (→WAKEUP via Q1) | 12 | `libs/t5838` + `libs/audio` | pin map to re-check (firmware THRS on GPIO10.6) |
  | LIS2DUX12TR accel on I2C5 @0x19, INT1→WAKEUP | 16 | `libs/lis2du12` (WHO_AM_I 0x45) | **UPGRADE NEEDED**: LIS2DUX12 WHO_AM_I 0x47, different reg map (ST lis2dux12-pid); `sentai.imu.init()` → -1 today |
  | SDRAM 512 Mbit 16-bit 166 MHz (U18) | 11 | SEMC config | OK (RAM builds run) |
  | PTN5150A USB-C CC + SN74LVC1T45 (PORT from GPIO_AD_08, ID → GPIO_AD_09) | 13 | none needed (UFP default) | device mode enumerates OK |
  | Edge TPU module LBPB0ZZ1WV, load switch TPU_POW_EN EMC_B2_15, RST EMC_B2_14 | 14 | `libs/tpu` | unchanged; not exercised on this board yet |
  | QSPI NAND 1 Gbit (U17) | 10 | flashtool / LittleFS | OK; boot blocked by BOOT_MODE straps |
  | JTAG/SWD pads shared with DMIC lines | 6 | — | DAP power-up fails (see J-Link note) |
- microSD: see [[sdcard-usdhc1-bringup]] (done).
- Sheet 16 note: "7s hold USER = force power cycle", USER = BOOT_MODE
  selector (pressed → Serial Downloader).

**Boot anomaly — ROOT CAUSE FOUND 2026-10-30 (EXP-s248, build 1609):**
`sentai.sys.peek32(0x40C0400C)` → `SRC_SBMR2 = 0x00000009`: **BMOD = 00
"Boot From Fuses", BT_FUSE_SEL = 0** → ROM falls back to Serial Downloader.
`SRC_SBMR1 = 0`.  Flash is fine (valid NFCB at NAND page 0, firmware table
blocks 4/8, 1 Gbit / 2 KB pages / 128 KB blocks).  Board needs
BOOT_MODE[1:0] = `10` at POR (BOOT_MODE1 = GPIO_LPSR_03 pulled high — SW1 /
R29 / R163 on sheets 15–16); original Coral has that.  Fuse programming
(BT_FUSE_SEL) is the irreversible alternative — operator decision only.
The flashloader cannot read peripheral space (`kStatusMemoryRangeInvalid`)
and refuses SPI-NAND reads past block 0 (`kStatus_InvalidArgument`) — use
`peek32` from firmware for registers.

**Boot anomaly (original symptom log):**
- On plain USB replug (USER *not* pressed) the board enumerates as ROM SDP
  `1fc9:013d`, never as firmware `1fc9:c0a1`.
- `flashtool.py -e sentai_runtime` (persistent) completes rc=0, then the
  board disappears from USB entirely; after POR it is back in SDP.
- `--ram` loads boot fine (builds 1602, 1605 verified via `sentai.version()`).
- `sentai.fs.exists('/')` → False, `/log` missing ⇒ FileX user partition in
  SAFE MODE (agent.md §2 rule 12).  Not formatted; operator consent needed.
- Hypotheses: flash image not written/valid on this unit (new NAND/FlexSPI
  routing on the rev?), or BOOT_MODE straps read "serial downloader".
  J-Link PLUS (1366:0101) is attached for when we dig in — see
  [[jlink-headless-only]].

**J-Link on this board rev (2026-10-30, read-only probe attempt):**
- VTref 1.63–1.85 V, `Found SW-DP with ID 0x6BA02477` (Cortex-M7 DP answers),
  but **`Failed to power up DAP`** every time (4000 and 1000 kHz) → no memory
  or register reads possible.  Unresolved: debug-domain power-up handshake
  fails on the new rev (JTAG_MOD level? debug pads on the add-on header?).
- **JLinkExe's automatic fallback "Executing connect under reset" RESET the
  board** → the `--ram` image was lost and the board dropped off USB
  (needs POR + `--ram` again).  Do NOT run JLinkExe `connect` against this
  board while a RAM-loaded image is the only thing running; expect to
  re-flash `--ram` after any J-Link attempt until DAP power-up is solved.
- Sheet 6: SWD_DIO = JTAG_TMS (shared pad `DMIC_DATA3__JTAG_TMS`), SWD_CLK =
  JTAG_TCK (`DMIC_DATA2__JTAG_TCK`), JTAG_MOD on `DMIC_DATA1` pad — the
  debug pins are shared with the digital-mic lines on this rev.

**Tooling that worked:** `venv-flash/bin/python scripts/flashtool.py`
(system python lacks progress/hexformat/hid).  `.venv-flash/` is a duplicate.

**How to apply:** every test on this board must be re-loaded with `--ram`
after a power cycle; do not assume persistent flash runs.  Before any CAM1
work, implement the I2C1 move + FlexPWM1 guard first.

**EMU (Renode) storage targets broken by the SD migration (2026-10-31):**
`emu/CMakeLists.txt` links `fx_user_fs.cc` + `fx_nand_driver.cc` in five
targets; `fx_user_fs.cc` now needs `FxSd*` → undefined references.  WBS
OP-S10-W22-T8.  `cmake -S . -B build-emu -DSENTAI_ARM_EMU=ON
-DSENTAI_SKIP_SDK_PATCHES=ON` configures; note every EMU build bumps
`build_version.txt` (restore it afterwards so the counter matches the board).

**Init audit 2026-10-31 (OP-S10-W22-T9, agent/board_rev_2026_10_init_audit.md):**
BOOT straps: SW1/R163/R164 DNP; sheet-16 Q4/D6 button circuit is inverted
(open = serial downloader, pressed = internal boot) and pull-ups hang on PMIC
VDD_1V8 → test: hold USER while resetting.  VDD_1V8_INT_EN goes through Q9
inverter: 0 = mic+IMU LDO ON (default, survives SNVS), 1 = OFF; R13 DNP so
the rail is LDO-fed (the "fed from PMIC via R13" comment is wrong).  SDK
debug console is LPUART6 = PILOT/Crazyflie connector (DbgConsole_Printf in
cdc_acm.cc leaks onto the link).  PwmInit() re-muxes AD_00/01 every call
(would open the CAM1 I2C switch).  WAKEUP pad (T8) = GPIO13.0 falling IRQ
with no handler; mic WAKE (Q1) and IMU INT1 (Q5) pull it low → the SNVS
power-down wake path exists in hardware; firmware has it only in the
disabled camera_streaming_http example (commit 0997099b).  Operator has a
reference board with burnt fuses that boots from flash → RAM-load and dump
SBMR/OCOTP for comparison when asked.
