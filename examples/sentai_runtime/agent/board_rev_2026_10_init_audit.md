# Board rev 2026-10 — hardware vs firmware initialisation audit

**WBS:** OP-S10-W22-T9 (new board rev bring-up: init audit).  **Date:** 2026-10-31.
**Inputs:** schematic `Coral_Dev_Board_Micro.pdf` (19 sheets, 2026-10-30), BOM
`board_rev_2026_10_bom.csv` (same folder), firmware at commit `1e6877cc`
(build 1648 RAM-loaded), the firmware init inventory produced for this audit
(gpio.cc tables, pin_mux.c, main_freertos_m7.cc, libs/sdcard, libs/tpu,
libs/pmic, libs/camera, libs/t5838, libs/lis2dux12, libs/audio).
**Status:** PROPOSAL — nothing below is implemented yet; each item names the
change and the test that proves it.  Items marked **HW** need the
electronics colleague; items marked **FW** are firmware only; **DECISION**
items need the operator.

## 0. Summary

| # | Finding | Severity | Owner |
|---|---|---|---|
| F1 | Boot straps: the USER-button circuit drives BOOT_MODE[1:0] inverted vs its own table; the external boot DIP switch SW1 (and R163/R164) is DNP; both strap pull-ups hang on PMIC `VDD_1V8`, which may not be up when the ROM samples the pins → board always enters the serial downloader | BLOCKER for flash boot | HW (+ a 1-minute button test) |
| F2 | SDK debug console (`DbgConsole_Init`) runs on **LPUART6**, which on this rev is the PILOT (Crazyflie) connector J3; error paths emit text onto the autopilot link | HIGH | FW |
| F3 | `VDD_1V8_INT_EN` logic is inverted by Q9: writing 1 **turns the mic+IMU LDO off**; the rail is ON by default (R59 pull-up to VSYS_DA) and stays on in SNVS power-down — correct behaviour today, wrong comment in firmware, and a landmine for any future "power-save" code | MEDIUM (doc) / HIGH (if touched) | FW doc |
| F4 | `PwmInit()` re-muxes GPIO_AD_00/AD_01 to FLEXPWM1 on every call (`muxed` never set); any use of `LedSet(Led::kTpu)` or the PWM API would open the CAM1 I²C isolation switch U9 → camera 1 NAKs | HIGH (latent) | FW |
| F5 | Legacy Coral pin_mux still runs: GPIO_AD_12..23 muxed as parallel-CSI pads at boot (AD_18 = THSEL, AD_19 tied to MIC_CLK, AD_21 = 1V8_INT_EN, AD_12/16 = SD power/VSELECT) and only some are re-muxed later; GPIO_SD_B1_03 (USDHC1_DATA1) muxed as GPIO "THRS"; `GPIO_SD_B1_05` "MicClkFeedback" is USDHC1_DATA3 | MEDIUM | FW cleanup |
| F6 | `PinNameToIRQ` has no entries for kCamPwrDn…kMicClkFeedback → `GpioConfigureInterrupt()` on them enables IRQ 0 (`DMA0_DMA16_IRQn`, the PDM mic DMA) at priority 5 | MEDIUM (latent) | FW |
| F7 | WAKEUP pad (T8) is configured as GPIO13.0 falling-edge input with the port interrupt enabled but no handler: every mic-WAKE / IMU-INT1 edge raises an IRQ that is silently discarded; no GPC/SNVS wake path exists | MEDIUM (sleep prerequisite) | FW |
| F8 | TPU power-on waits for PGOOD with **no timeout**; `TPU_BOOT_FAIL` (EMC_B2_13) and `MCM_SD_ALARM` (SNVS_05) are never read (SNVS_05 is still wired as "camera int" rising-edge IRQ) | MEDIUM | FW |
| F9 | Gpio table collisions: `kAA`/`kCamReset` both GPIO3.25 (GPIO_AD_26 = CAM1_RESET) with different ALT macros and opposite init levels; `kAB` module/macro mismatch; Wi-Fi/BT-era names on camera pads (DISP_B2_07 "WL_HOST_WAKE" = CAM_MUX, DISP_B2_15 "BT_HOST_WAKE" = CAM2_PWDN) | LOW (cleanup) | FW |
| F10 | TPU LED (LED5, GPIO_AD_02 → Q3) is muxed to FLEXPWM1_PWM1_A with no driver running → undefined/dim; board-status LED Q10 is ON by default by design | LOW | FW |
| F11 | USB-C CC controller PTN5150A PORT/ID (GPIO_AD_08/09) untouched: device-only (UFP) today; fine, documented | INFO | — |
| F12 | JTAG: `JTAG_MOD` (LPSR_13) has R37 DNP + R38 4.7 k pull-down → SWD mode selected correctly; LPSR_10..12 are muxed away from JTAG at boot (TRST/TDO/TDI), TCK/TMS left default; "Failed to power up DAP" (2026-10-30) is still unexplained | INFO | HW/FW later |
| F14 | `sentai.sdcard.init()` (→ `SDCard::Init` → `SD_Init`) has no bound when no card answers: on the reference board it parked the REPL task for good (Ctrl-C cannot interrupt native code, the watchdog keeps kicking).  Boot mount is protected by `FxUserBootMount`'s 20 s task deadline; the REPL/REPL-driven paths are not | MEDIUM | FW |
| F19 | Manufacturer files received 2026-10-31: `board_rev_2026_10_owl_r2_bom.csv` (OWL-R2 BOM, 77 rows: U8 LIS2DUX12TR, U19 T5838, U1 MCP16701, U2 TCR2LN33, U3 TCR2LN18, U6 NX3L1T3157 (SD IO-rail selector, S has no pull), U12 FDN306P (SD power switch), J2 Molex 47219-2001 microSD) and `board_rev_2026_10_owl_r2_pnp.csv` (placement). The microSD card itself is not in the BOM (`SD64G`, 64 GB; MID/OID readable from the CID if needed) | INFO | — |
| F18 | **LIS2DUX12 answers at I2C 0x18, not 0x19, and powers up in deep power-down** (2026-10-31, first true power cycle of unit 58eb): `imu.init()` failed with WHO_AM_I 0x00 at 0x19 while the mic on the same 1V8_INT rail worked; an I2C5 scan showed 0x10, 0x1F, 0x5B (PMIC) only; after the wake write to EN_DEVICE_CONFIG (NAKed but executed, AN5909 §3.1.1) the part appeared at **0x18** and WHO_AM_I = 0x47. Earlier sessions passed at 0x19 only because the part had never been power-cycled (state carried across RAM loads) | MEDIUM (IMU dead after every cold power-up until fixed) | FW **SHIPPED build 1678**: driver tries 0x19 then 0x18, each with the wake write + 25 ms; `sentai.imu.scan()/reg_read()/reg_write()` diagnostics. **Rail test (same day):** GPIO9.20 = 1 → the whole I2C5 bus disappears (even the PMIC at 0x5B) and the mic reads 0 → **1 = LDO OFF, 0 = ON, F3 polarity confirmed**; I2C5 pull-ups and the mic are on VDD_1V8_INT. Back to 0 → bus and mic return, the IMU is again in deep power-down (needs the wake write) → the "SA0 follows the rail" idea is refuted; why earlier sessions saw 0x19 stays open. HW question for the colleague: SA0 strap on U8. FW follow-up: `imu.read()` failures after a rail cycle should trigger a re-init |
| F17 | **microSD stopped answering after the persistent-flash session** (2026-10-31, after EXP-s253): every `SD_Init()` blocks in `SD_CardInit` after CMD8 (USDHC1 idle: `PRES_STATE 0xFF888088`, CMD/DATA lines high, `INT_STATUS 0`, `RSP0 0`), the known-good build 1664 rebuilt from commit 35c3f695 fails identically, a 3 s `SD_PWREN_B` power-off does not recover it. Suspect: the elf_loader (old-board pinout) ran on this unit during the persistent flash and drove the SD rail/VSELECT or pads into a state the card did not come back from. **Resolved by a physical reseat of card and board (USB replug): mounts again at boot, `init_stage` 10.** So it was a card/rail state that the firmware's 30 ms `SD_PWREN_B` pulse (and even a 3 s one) could not clear — the card needs a true VDD collapse | MEDIUM (recurs only after an elf_loader run on this unit) | **Proposal (electronics colleague, 2026-10-31):** the card identifies in DS mode and runs HS at 3.3 V (no UHS-I 1.8 V switch in SentAI); during the persistent flash the elf_loader's legacy pin_mux (F5) takes GPIO_AD_12 (SD_PWREN_B) and AD_16 (SD1_VSELECT) away for minutes, so the card may sit on the 1.8 V rail or on a half-collapsed VDD. FW: (a) in `ConfigureSDHost` discharge for ≥ 500 ms with CMD/DATA driven low as GPIOs so VDD really collapses, (b) keep VSELECT at 3.3 V whenever the pad is ours, (c) stop the legacy pin_mux from touching AD_12/16 on this rev (F5). Not implemented yet — operator decision |
| F16 | **CSI input wedge on MUX flip** (EXP-s251 iter9/iter12, builds 1654/1658): during 1:1 alternation at the as-found phase (+400 µs, cam0 margin ~130 µs) the CSI stopped receiving frames (CR3.FRMCNT frozen, SR without overrun, CSI2RX IRQ_STATUS 0x13d = CRC + 2-bit ECC + ULPS + SoT + SoT-sync + ERRCONTROL, PPI_ERRSOT_HS lane 0, PPI_ERRSOTSYNC_HS lane 1); both sensors still streaming and answering on I2C. NOT recovered by sensor standby toggle, MUX switch, or `camera.stop()+init()` (which re-runs CSI2RX_Init incl. PHY PD cycle and the sensor PWDN/RESET pulse); only a SoC reset recovers. 2 of 10 cold boots during the genlock measurement | HIGH (dual-camera; latent in production alternation, same ISR flip path) | FW: genlock_auto default OFF; flip-free phase measurement via STROBE pads (W23-T10); deeper recovery (VIDEO_MUX / PGMC BPC4 / CSR re-init) to be tried on the next wedged unit |
| F15 | NXP CSI driver keeps its empty-buffer linked list INSIDE the frame buffers; a DMA write into a still-linked buffer turns the list head into pixel data → BusFault in the CSI ISR re-arm (caught by the new crash record during EXP-s251; likely the same defect behind the iter3 LOCKUP) | HIGH (latent, dual-camera) | FW — **SHIPPED** SDK patch 0005, build 1657 |
| F13 | SD card runs at 3.3 V signalling (SD1_VSELECT → U6 → NVCC_SD = 3V3); the hardware supports 1.8 V UHS-I switching, unused (16 MB/s read today) | INFO (future speed) | FW later |

## 1. Pin / net map (nets that firmware drives or should know)

| Net (schematic) | Pad | Sheet | Firmware today | Verdict |
|---|---|---|---|---|
| USER_BUTTON | GPIO_SNVS_00 (GPIO13.3) | 16 | kUserButton, falling-edge IRQ → `btn_usb` → USB storage mode reset | OK; note SW2 also toggles BOOT_MODE0/1 (inputs at runtime, harmless) |
| BRD_STAT_LED (orange, Q10) | GPIO_SNVS_02 (GPIO13.5) | 16 | kStatusLed out/0 | OK (gate high = LED on; R172 100 k keeps it ON when the pin floats) |
| USER_LED (green, Q2) | GPIO_SNVS_03 (GPIO13.6) | 16 | kUserLed out/0 | OK |
| LED_TPU (white, Q3) | GPIO_AD_02 / FLEXPWM1_PWM1_A | 16 | pin_mux: PWM1_A, config 0; `LedSet(kTpu)` → PWM (no caller) | F10 |
| BOOT_MODE0 / BOOT_MODE1 | GPIO_LPSR_02 / 03 | 15, 16 | not touched (ROM straps) | F1 |
| POR_B | PMIC DA_GPIO1 / SW3 | 5, 16 | — | OK |
| ONOFF | U10 (R52 100 k PU) | 5 | — | not wired to a button on this rev (power diagram shows "Button MCU ON/OFF"; BOM has only SW2/SW3) — DECISION for sleep |
| WAKEUP | T8 (R25 100 k PU; Q1 mic WAKE, Q5 IMU INT1 pull low) | 5, 12, 16 | pin_mux GPIO13.0 falling IRQ, no handler | F7 |
| PMIC_ON_REQ | U9 → R6 → DA_GPIO4_SYS_EN (R7 100 k to 1V8_INT, R9 1 M to GND) | 4, 5 | SNVS default; nothing muxed | needed for SNVS power-down (see §3) |
| VDD_1V8_INT_EN | GPIO_AD_21 (GPIO9.20) → Q9 gate (R61 430 k PD) → U3 CONTROL (R59 430 k PU to VSYS_DA) | 4 | written 0 ("held off") | F3: 0 = rail ON (correct), comment wrong; R12 populated, **R13 DNP** |
| SD_PWREN_B | GPIO_AD_12 (GPIO3.11) → R77 → U12 FDN306P gate | 16 | sdcard.cc: 1 then 0 (active low) | OK; pin_mux first makes it CSI PIXCLK (F5) |
| SD1_VSELECT | GPIO_AD_16 (GPIO3.15) → U6 NX3L1T3157 S | 6 | sdcard.cc: 0 → Y0 = VDD_3V3 | OK (F13 for 1.8 V later) |
| USDHC1 CMD/CLK/D0-3 | GPIO_SD_B1_00..05 | 6 | sdcard.cc muxes all six | OK; pin_mux earlier muxes B1_03 and B1_05 as GPIO (F5) |
| TPU_POW_EN | GPIO_EMC_B2_15 (GPIO8.25) → U21 EN (R39 4.7 k PD) | 14 | kEdgeTpuPmic | OK |
| TPU_RST_L | GPIO_EMC_B2_14 (GPIO8.24) | 14 | kEdgeTpuReset | OK |
| MCM PGOOD | GPIO_EMC_B2_16 (GPIO8.26) | 7, 14 | kEdgeTpuPgood, polled without timeout | F8 |
| TPU_BOOT_FAIL | GPIO_EMC_B2_13 (GPIO8.23) | 14 | muxed, never read | F8 |
| MCM_SD_ALARM | GPIO_SNVS_05 (GPIO13.8) | 14 | kCameraInt rising-edge IRQ (legacy name) | F8 |
| MCM_LDO_EN (U22 1V8) | R95 10 k PU to MCM_3V3 | 14 | — | OK (follows TPU_POW_EN) |
| CAM1_RESET / CAM2_RESET | GPIO_AD_26 / GPIO_AD_11 | 6, 9 | kCamReset / kCamReset2 | OK (F9 table hygiene) |
| CAM1_PWDN / CAM2_PWDN | GPIO_DISP_B2_09 / B2_15 | 6, 9 | kCamPwrDn / kCamPwrDn2 | OK |
| CAM_MUX (PI3WVR626 SEL) | GPIO_DISP_B2_07 | 6, 9 | kCamMux (+ ISR flip) | OK |
| CAM1 I²C isolation (U9 1E/2E) | VDD_1V8_CAM1 via R57, pulled off by Q7 ← FLEXPWM1_PWM0_A (GPIO_AD_00) | 6 | pad 0x06 + GPIO2.31 low (build 1622) | OK; F4 can undo it |
| Add-on I²C1 isolation (U11) | Q6 ← FLEXPWM1_PWM0_B (GPIO_AD_01) | 6 | pad 0x06 + GPIO3.0 low | OK |
| I2C1 SCL/SDA (CAM1 + add-on) | GPIO_AD_32/33 | 6 | LPI2C1 handle, pad 0x10 | OK |
| I2C5 (PMIC via U13, IMU, MCM via U10, add-on J9) | GPIO_LPSR_04/05 | 6 | LPI2C5 | OK (U13 enable = VDD_1V8, U10 enable = MCM_1V8) |
| I2C6 (CAM2) | GPIO_LPSR_06/07 | 6 | LPI2C6 | OK |
| MIC_CLK / DMIC_DATA0 | GPIO_LPSR_00 / 01 | 6, 12 | PDM, config 3 | OK (48000 samples / 3 s measured) |
| T5838 THSEL | GPIO_AD_18 (TP4) | 12 | libs/t5838 (GPIO3.17), not used at boot | OK (F5: pin_mux muxes AD_18 as CSI_DATA07 first) |
| GPIO_AD_19 | R54 0 R to MIC_CLK_R | 12 | pin_mux CSI_DATA06 (input) | never drive as output (F5) |
| T5838 WAKE | Q1 → WAKEUP | 12 | — | F7 |
| LIS2DUX12 INT1 | Q5 → WAKEUP (R56 100 k) | 16 | wake-up engine routed to INT1 by `SetInt2WakeUpThreshold`; events polled at 50 Hz | F7 |
| LPUART6 TX/RX (PILOT J3 via TXB0302 U15, OE = R89 to 1V8) | GPIO_EMC_B1_40/41 | 7, 16 | SDK DbgConsole **and** Crazyflie bridge | F2 |
| LPUART1 TX/RX (J4 header via U14, enable BT_CFG_IO_EN) | GPIO_AD_24/25, enable GPIO_AD_05 | 6, 16 | kLpuart1SwitchEnable out/1; UART unused | candidate debug console (F2) |
| BT_CFG1/2 boot-config straps | GPIO_DISP_B1_06..11, DISP_B2_00..05 | 6, 15 | pin_mux muxes B1_06..11 as ENET RGMII (unused) | OK (47 k pull-downs R151-R162 → BOOT_CFG = 0 = QSPI NAND defaults) |
| JTAG_MOD | GPIO_LPSR_13 (R37 DNP, R38 4.7 k PD) | 6 | not touched | SWD mode (F12) |
| USB_PORT_RT / USB_OTG1_ID | GPIO_AD_08 / 09 (PTN5150A) | 13 | not touched | F11 |
| Legacy Wi-Fi/BT: WL_REG_ON, BT_REG_ON, WL_ANT_SEL, BT_DEV_WAKE, WIFI_POWER_EN, ETHPHY_RST_B, CRYPTO_RST_N | AD_34, AD_35, DISP_B2_06, DISP_B2_14, SNVS_09, EMC_B2_03, LPSR_08 | 6, 7 | driven out/0 or out/1 at GpioInit | unconnected on this rev; harmless, cleanup later |

## 2. Findings in detail

### F1 — Boot straps (sheets 15 + 16, BOM)

Sheet 15 "External Boot Switch": SW1 (DIP) with R164 100 Ω (BOOT_MODE0) and
R163 4.7 k (BOOT_MODE1) to `DCDC_1V8_OUT`.  **SW1, R163 and R164 are not in
the BOM** (the only switches are SW2 USER and SW3 RESET).  The on-chip
default is a 25-50 kΩ pull-down on both BOOT_MODE pins.

Sheet 16 "Buttons": R29 4.7 k (VDD_1V8) → node N; N → R31 100 Ω → BOOT_MODE0;
N → Q4 gate; Q4 drain = BOOT_MODE1 (R30 4.7 k pull-up to VDD_1V8); D6 anode
at N, cathode at the SW2 node.  Therefore:

| SW2 | N | BOOT_MODE1 | BOOT_MODE0 | ROM result | table on the sheet says |
|---|---|---|---|---|---|
| open | high | **0** (Q4 on) | **1** | 01 = Serial Downloader | 10 Internal Boot |
| pressed | low (through D6) | 1 (R30) | 0 | 10 = Internal Boot | 01 Serial Downloader |

The circuit implements the inverse of its own table.  In addition both
pull-ups reference PMIC `VDD_1V8`; `SRC_SBMR2 = 0x9` read in EXP-s248
(BMOD = 00 "boot from fuses", BT_FUSE_SEL = 0) is consistent with *both*
pins reading low when the ROM sampled them, i.e. `VDD_1V8` not yet up at
POR release (the DCDC_1V8_OUT reference used on sheet 15 is the on-chip
regulator and is up at that point, which is why the original design used it).

**Test (operator, 1 minute):** hold USER (SW2) while pressing/releasing
RESET (SW3).  If the board enumerates as `1fc9:c0a1` (firmware) instead of
`1fc9:013d`, the inversion is confirmed and the flash image already there
runs.  Do not hold USER for 7 s (PMIC power-cycle note on sheet 16).

**HW fix (colleague):** make the open state read `10`: e.g. populate R163 to
`DCDC_1V8_OUT` for BOOT_MODE1 and change the Q4/D6 arrangement so the button
pulls BOOT_MODE1 low and BOOT_MODE0 high only when pressed (or drop Q4/R31
and use SW1's footprint).  Strap pull-ups must reference `DCDC_1V8_OUT` or
`VDD_SNVS`, not a PMIC rail.

**Reference board:** the operator has a unit with fuses burnt that boots
from flash.  Plan: RAM-load build ≥ 1648 on it (RAM load does not touch
flash) and read `SRC_SBMR1/2`, `SRC_SRSR` and the OCOTP BOOT_CFG / BT_FUSE_SEL
shadow registers with `sentai.sys.peek32`; compare with this unit.  That
gives the exact fuse words if the fuse route is ever chosen.

### F2 — SDK debug console on the PILOT UART

`third_party/modified/nxp/rt1176-sdk/board.h:23` → `BOARD_DEBUG_UART_INSTANCE_M7 6`;
`BOARD_InitDebugConsole()` runs in `BOARD_InitHardware()`.  On this rev
LPUART6 (GPIO_EMC_B1_40/41) goes through U15 (TXB0302, OE tied high) to J3
"Conector interfata PILOT" — the Crazyflie link that `sentai_crazy` also
uses (`modsentai_hal.cc:277` sets the LPUART6 baud rate).  `DbgConsole_Printf`
is still called in `libs/cdc_acm/cdc_acm.cc` (5 sites: send/recv failures,
unhandled events), `libs/base/utils.cc` and `camera_support.c` (clock
errors).  Each emits ASCII at the debug-console baud onto the autopilot UART.

**FW proposal:** (a) move the SDK debug console to LPUART1 (GPIO_AD_24/25,
J4 header; its isolation switch U14 is already enabled by `BT_CFG_IO_EN` =
GPIO_AD_05 out/1), or (b) disable `DbgConsole` on M7 and route those 9 call
sites to `printf` (CDC-ACM) / SERR.  (b) is the smaller change and removes
a second UART clock; (a) keeps a wired console for boot-before-USB
forensics.  **DECISION:** (a) or (b).  Test: scope / loopback J3 during a
CDC-ACM error (unplug mid-print) — no bytes; Crazyflie link ping unaffected.

### F3 — VDD_1V8_INT (mic + IMU rail)

Sheet 4 at 400 dpi: U3 TCR2LN18 CONTROL ← node C; R59 430 k from C to
VSYS_DA (pull-up); Q9 (DMN2550) drain = C, source = GND, gate =
`VDD_1V8_INT_EN` with R61 430 k pull-down.  TCR2LN datasheet: CONTROL high
= ON, low/open = OFF.  So:

- GPIO low or floating → Q9 off → CONTROL high → **LDO ON** (default).
- GPIO high → Q9 on → CONTROL low → **LDO OFF**.

R12 (0 Ω) is populated, **R13 is DNP** → the rail is LDO-fed, not PMIC-fed.
Firmware writes 0 (`main_freertos_m7.cc` VDD_1V8_INT_EN block) → rail ON,
which is why the IMU answers and the mic streams.  The comment written on
2026-10-31 ("rail fed from PMIC VDD_1V8_AON via R13, pin held LOW on
purpose") is wrong and must be replaced with the inverter description.
Because the default (pin high-Z) is ON, the rail survives SNVS power-down,
which is exactly what wake-on-sound / wake-on-motion needs (§3).

**FW proposal:** fix the comment; add a one-line boot print of the rail
state; never write 1 except in an explicit "mic+IMU off" power mode.

### F4 — `PwmInit()` re-muxes the I²C-switch gates

`libs/base/pwm.cc:63-68`: the `muxed` flag is never set, so every
`PwmInit()` muxes GPIO_AD_00/AD_01 to FLEXPWM1_PWM0_A/B.  `LedSet(Led::kTpu)`
uses PWM1 submodule 1 (AD_02) through the same init.  On this rev AD_00
drives Q7 (CAM1 I²C isolation U9) and AD_01 drives Q6 (add-on I²C U11); the
build-1622 fix keeps them GPIO low.  Any future PWM/TPU-LED use silently
reverts them.  No caller exists today (`LedSet(kUser)` only).

**FW proposal:** set `muxed = true` after muxing and mux only the pads of the
requested PWM channel; keep AD_00/AD_01 out of the PWM mux on this board
rev (build flag or board table).  Test: call `LedSet(kTpu)` then
`sentai.camera.reg_read(0, 0x300A)` → still 0x56.

### F5 — Legacy pin_mux ordering and overlaps

`third_party/modified/nxp/rt1176-sdk/pin_mux.c` still carries the Coral
parallel-camera and Wi-Fi/BT muxing:

- 473-495: GPIO_AD_12..23 → VIDEO_MUX_CSI_* (PIXCLK, MCLK, VSYNC, HSYNC,
  DATA02..09).  On this rev: AD_12 = SD_PWREN_B, AD_16 = SD1_VSELECT (both
  re-muxed by `sdcard.cc` after the scheduler starts — so for the first
  ~hundreds of ms the SD power FET gate is a CSI input, i.e. floating, held
  by R88 10 k pull-down → card off: acceptable), AD_18 = THSEL (CSI_DATA07
  input, fine), **AD_19 = CSI_DATA06 input while tied to MIC_CLK** (input,
  fine, but must never become an output), AD_21 = VDD_1V8_INT_EN (CSI_DATA04
  then re-muxed to GPIO9_IO20 at line 519 — fine).
- 93-94: GPIO_SD_B1_03 → GPIO10_IO06 ("T5838 THRS") — that pad is
  USDHC1_DATA1 here; `sdcard.cc:139` re-muxes it.  The GPIO write was
  removed from `main_freertos_m7.cc` in build 1642; the mux line remains.
- 97-98: GPIO_SD_B1_05 → GPIO_MUX4_IO08 ("MIC_CLK_FEEDBACK") — USDHC1_DATA3
  here; re-muxed by `sdcard.cc:141`.
- 498-560: Wi-Fi/BT block names on camera pads (see F9).

**FW proposal:** a board-rev section in pin_mux.c (or a `#if
SENTAI_BOARD_REV_2026_10`) that: leaves AD_12..23 as GPIO inputs with no
pull (except the ones owned by drivers), drops the SD_B1_03 / SD_B1_05 GPIO
muxes, and documents each pad with its real net.  Test: EXP-s249 full pass +
`sentai.camera.init()` + SD mount + mic samples — identical results.

### F6 — Missing IRQ table entries

`gpio.cc` `PinNameToIRQ` stops before kCamPwrDn; the six trailing enums read
0 = `DMA0_DMA16_IRQn`.  `GpioConfigureInterrupt()` on any of them (the
disabled `camera_streaming_http` example does it for kMicClkFeedback) would
`EnableIRQ(0)` and set priority 5 on the PDM DMA IRQ.

**FW proposal:** fill the table (`kCamPwrDn`..`kMicClkFeedback` → their
GPIO port IRQs, or a sentinel that makes `GpioConfigureInterrupt` return
an error), and drop `kMicClkFeedback` (the pad is USDHC1_DATA3 now).

### F7 — WAKEUP pad handling

`pin_mux.c:38-61` sets WAKEUP_DIG as GPIO13.0 input, falling edge, port
interrupt enabled, pad 0x0E.  `gpio.cc` CommonIrqHandler clears GPIO13.0
and ignores it.  With the T5838 AAD or the LIS2DUX12 wake-up engine armed,
every event is an unhandled interrupt.  Nothing in `sentai_runtime` uses the
RT1176 GPC/SNVS wake machinery; the only implementation is in the disabled
`examples/camera_streaming_http/camera_streaming_http.cc` (`shutdown_system()`:
SRTC alarm + `SNVS->LPCR |= LPWUI_EN`, then `DP_EN | TOP`; commit 0997099b
"Make the wakeup on accel working" — validated on the previous board).

**FW proposal:** see §3.

### F8 — TPU power sequencing observability

`libs/tpu/edgetpu_task.cc:150-153` spins on `GpioGet(kEdgeTpuPgood)` with
`taskYIELD()` and no deadline: a failed U21 or a shorted MCM_3V3 hangs the
caller (`EdgeTpuManager`, i.e. `sentai.tpu.load`) forever.  `TPU_BOOT_FAIL`
(GPIO8.23) is muxed and never read; `MCM_SD_ALARM` (GPIO13.8) is wired as
`kCameraInt` with a rising-edge IRQ whose handler does nothing useful.

**FW proposal:** bounded PGOOD wait (200 ms) → `SERR_TPU_PGOOD_TIMEOUT`,
return failure; read TPU_BOOT_FAIL after the reset release and expose it
plus SD_ALARM in `sentai.tpu` stats; rename `kCameraInt` → `kTpuSdAlarm`
and log the edge.  Test: `sentai.tpu.load` with `TPU_POW_EN` forced low via
`poke32` on the GPIO DR → returns an error within 200 ms instead of hanging.

### F9 — Gpio table hygiene

`kAA` = GPIO3.25 with `IOMUXC_GPIO_AD_26_GPIO9_IO25` (ALT 0xA, wrong module),
`kCamReset` = GPIO3.25 with `GPIO_AD_26_GPIO_MUX3_IO25`; GpioInit initialises
the same pin as out/0 (kAA) then out/1 (kCamReset) — a 0 → 1 glitch on
CAM1_RESET at boot.  `kAB` has module GPIO3 with a GPIO9 macro.  Camera pads
carry Wi-Fi/BT names and pad configs in pin_mux.c (`DISP_B2_15` "BT_HOST_WAKE"
0xC = CAM2_PWDN; `DISP_B2_07` "WL_HOST_WAKE" = CAM_MUX; `DISP_B2_06`
"WL_RST#" = WL_ANT_SEL, unconnected).

**FW proposal:** delete kAA/kAB (no users), fix labels, give the camera pads
explicit pad configs in the board-rev section.  Test: `sentai.camera.init()`
+ EXP-s249 R10.

### F10 — LEDs

LED5 (TPU, white) is on GPIO_AD_02 = FLEXPWM1_PWM1_A through Q3; the pad is
muxed to PWM with no PWM running → the gate follows the pad's idle state
(no pull, config 0).  LED4 (board status, orange) is ON by default through
R172 and is driven by `kStatusLed`; LED3 (user, green) by `kUserLed`.

**FW proposal:** drive AD_02 as GPIO low at boot (same pattern as AD_00/01)
and later use it as a TPU-activity indicator through the fixed PWM path
(F4).  **DECISION:** keep the white LED off by default?

### F11 — USB-C

PTN5150A: PORT (GPIO_AD_08) low = UFP default, ID (GPIO_AD_09) needs an MCU
pull-up for OTG; neither is touched.  Device mode works.  Record only.

### F12 — JTAG / SWD

JTAG_MOD = LPSR_13 with R38 4.7 k pull-down (R37 DNP) → SWD mode.  pin_mux
moves LPSR_10/11/12 (TRST/TDO/TDI) to GPIO_MUX6; TCK (LPSR_14) and TMS
(LPSR_15) stay on JTAG; the pads are shared with DMIC_DATA2/3, which on this
rev only go to the B2B connector.  The 2026-10-30 "Failed to power up DAP"
is therefore not a mode-pin problem; next suspects: VTref sensing on the
test points (1.63-1.85 V measured), or the SDK's `BOARD_InitPins` running
before the probe attached.  Not needed for the current bring-up.

### F13 — SD signalling voltage

SD1_VSELECT = 0 → U6 routes VDD_3V3 to NVCC_SD; the card runs DS/HS at
3.3 V (50 MHz, 4-bit, 16 MB/s read in EXP-s250).  The hardware supports
UHS-I 1.8 V switching (U6 Y1 = VDD_1V8).  A later work item, not bring-up.

### F14 — `SD_Init` without a card is unbounded (EXP-s252)

`libs/sdcard/sdcard.cc`: `SdBoardCardDetected()` always returns true ("no
card-detect switch; let SD_Init fail"), `s_cd.type = kSD_DetectCardByGpioCD`,
and `SD_Init()` is called with no deadline.  With an empty socket (or, on
the reference board, no USDHC1 wiring at all) the SDK init never returned.
`FxUserBootMount` already runs the boot-time mount in a worker task with a
20 s deadline, so boot is safe; `sentai.sdcard.init()` and `FxUserRemount`
are not.

**Status (2026-10-31, build 1675): SHIPPED** — `SDCard::InitBounded(deadline_ms)`
runs `SD_Init` in a static worker task (`sd_init`, 2048-word SDRAM stack, no
heap) and waits at most 5 s; `-1` timeout → `SERR_LFX_SD_INIT_TIMEOUT`
(0x0D39), the worker is left running and a later call returns its result;
`-2` while a previous attempt is still inside `SD_Init`.  Used by
`FxSdInit` (boot mount, remount) and `sentai.sdcard.init(deadline_ms)`;
`sentai.sdcard.init_stage()` exposes the progress breadcrumb.  Verified on
the REPL: `init(1)` returns False in 0.3 s with the SERR logged, the REPL
stays alive (before: blocked forever).  The no-card path itself was
exercised unintentionally by F17.

**FW proposal (original):** run `SD_Init` in the same deadline pattern (worker task +
`xSemaphoreTake(…, 5 s)`), or probe the card first with a bounded
`SDMMCHOST` command (CMD0/CMD8 with the host's own transfer timeout) and
return `-ETIMEDOUT` to the REPL; add `SERR_LFX_SD_INIT_TIMEOUT`.  Test:
`sentai.sdcard.init()` with the card removed returns False within 5 s and
the REPL stays alive.

### F15 — NXP CSI driver: empty-buffer list lives inside the frame buffers (EXP-s251, build 1651)

Caught by the new crash-text path (first real field use): `BUS_FAULT
PC=0x0000EA8A` = `CSI_TransferGetEmptyBuffer` (fsl_csi.c:235) called from
`CSI_TransferHandleIRQ` re-arm path, `BFAR = r0 = 0x00604F90` — a pixel
value (XRGB 60/4F/90) dereferenced as a pointer.  The SDK driver keeps its
empty-buffer list as a singly linked list whose "next" word is stored in
the first 4 bytes of each empty frame buffer.  If the CSI DMA writes into a
buffer that is still linked (double submission, the single-buffer branch
at fsl_csi.c:944-948 when the queue is full, or a skipped double-done
IRQ), the list head becomes pixel data and the next re-arm faults inside
the ISR.  It happened 3534 s into a session while both sensors were put in
software standby and restarted (s251 sync-start test); the iter3 LOCKUP
(fault during a fault, no breadcrumb) is probably the same defect caught
one level deeper.  The stale-queue condition of finding 5 (three full
buffers, single-buffer mode) makes the window wider.

**FW proposal (SDK patch 0005):** keep the empty list in a side array
(indices into the known 4-buffer pool) and bounds-check `emptyBuffer`
before dereferencing; on corruption drop to a `SERR_CAM_QUEUE_CORRUPT`
counter and re-initialise the queue instead of faulting.  Plus the stale
guard of s251 proposal C so the queue never sits full.

**Status (2026-10-31, build 1657): SHIPPED as SDK patch
`0005-fsl-csi-empty-list-side-array-f15.patch`.**  `csi_handle_t` now
carries `emptyBufferStack[CSI_DRIVER_ACTUAL_QUEUE_SIZE]` (LIFO) instead of
the `emptyBuffer` list head; `CSI_TransferGetEmptyBuffer` /
`CSI_TransferPutEmptyBuffer` are bounds-checked and call
`coralmicro_csi_on_queue_fault(kind)` (kind 1 = get on empty, 2 = put on
full; ISR-safe counter in `camera_support.c`, M4 stub), and
`CSI_TransferSubmitEmptyBuffer` returns `kStatus_CSI_QueueFull` on
overflow.  `CombinedWatchdogTask` reports a counter change as
`SERR_CAM_CSI_QUEUE_FAULT` (0x0A0F, val = kind<<16 | count);
`sentai.camera.stats()["csi_queue_faults"]`.  The stale guard (s251
proposal C) shipped in the same build.  The queue is NOT re-initialised on
a fault (not needed: nothing can corrupt the side array), it is only
reported.

### F16 — CSI input wedge on MUX flip (EXP-s251 iter9 / iter12, builds 1654 / 1658)

Observed twice in ten cold boots while the C genlock measured the phase by
alternating 1:1 at the as-found offset (+400 µs: the flip towards cam0 has
only ~130 µs of VBLANK margin).  Forensics taken on the wedged unit
(iter12, before any reset):

| probe | value | meaning |
|---|---|---|
| `camera.sensor_frames()` | 182, frozen | CSI FB-done IRQs stopped |
| `CSI->CR3` | `0xB61021` → FRMCNT = 182 | the CSI itself counted no further frame starts |
| `CSI->SR` | `0x80204000` | F1_INT, STATFF_INT, bit 31; no RxFIFO overrun, no DMA-done |
| `CSI2RX IRQ_STATUS` | `0x13D` | CRC err, 2-bit ECC err, ULPS change, ERRSOT_HS, ERRSOTSYNC_HS, ERRCONTROL |
| `CSI2RX PPI_ERRSOT_HS / PPI_ERRSOTSYNC_HS` | `0x1` / `0x2` | SoT error on lane 0, SoT-sync error on lane 1 |
| OV5640 cam0 / cam1 `0x3008 0x380E/F 0x4202 0x300E 0x3A00` | `02 03D8 00 45 58` on both | streaming, VTS 984 (restored), MIPI 2-lane, night mode off |
| recovery tries | sensor standby toggle (cam0, both), `select(1)`, `camera.stop()+init(1,45)` | **none** brought frames back (after re-init FRMCNT reset to 0 and stayed 0; SR showed SOF_INT once) |

Interpretation: a flip that lands outside the new sensor's blanking puts a
mid-packet HS burst on the lanes; the CSI2RX normally resyncs at the next
LP-11, but occasionally the receiver / pixel-link / CSI path stays stuck in
a state that neither CSI2RX_Init (controller SW reset + PHY PD cycle) nor
a sensor PWDN/RESET pulse clears — only a SoC reset does.  Candidate state
not touched by re-init: VIDEO_MUX, PGMC BPC4 D-PHY power domain, the
MIPI_CSI CSR pixel-link master.  The s251 iter3 LOCKUP (no breadcrumb) may
be the same event taken one level deeper.

**Root cause found and fixed the same day (EXP-s251 iter17–iter20, builds
1661–1664).**  A/B on cold boots: patch 0005 reverted → 3/3 clean; patch
applied → 8/9 wedged; patch applied **plus a 20 µs hold before the MUX
flip** → 3/3 clean, the same build with the hold set to 0 → 2/2 wedged.
The flip executed inside the CSI FB-done ISR was racing the OLD sensor's
MIPI **Frame-End short packet**: DMA-done fires when the last line is
written, FE follows within a few µs, and a flip that cuts the FE burst
leaves the CSI2RX / D-PHY in an error state that survives sensor restarts
and `CSI2RX_Init()`.  Which side of the race the flip landed on depended on
the ISR's own latency — patch 0005 removed one non-cacheable SDRAM read
from the re-arm path (a few hundred ns) and turned 1-in-8 into 8-in-9;
s246's "an XOR + store in the ISR broke alternation" was the same race.
The phase (as-found vs aligned) was never the deciding factor.

**Fix (build 1664):** the CSI ISR no longer flips; it arms **GPT1 output
compare 2** at `CNT + g_cam_flip_delay_us` (default 40 µs, ~10× the race
margin, far inside the 543 µs VBLANK even at the as-found phase) and
`GPT1_IRQHandler` executes the flip in `sentai_cam_deferred_flip_isr()`
(ITCM, counters + one GPIO write, GPT1_IRQn at the CSI priority so the two
never preempt each other).  `sentai.camera.flip_delay_us([us])` exposes the
knob (0 = legacy immediate flip, diagnostics only); `stats()` reports
`flips_deferred` / `flips_executed`.  Patch 0005 stays.  With the race gone,
the init-time genlock measurement is safe again; the default remains the
flip-free pre-trim (`genlock_auto` 1) with the on-the-fly `cam_tune` loop,
and mode 2 adds the measured verify.  W23-T10 (STROBE-pad phase
measurement) is no longer needed for safety and is kept only as an
optional future improvement.  If a unit ever wedges again: try `VIDEO_MUX`
re-select, PGMC BPC4 power cycle and `CSI2RX_Deinit()` + `CSI_Deinit()`
before resetting, and record what recovers it.

## 3. Sleep / wake architecture (what the board allows)

Power diagram (sheet 3) and sheet 5: `VDD_SNVS_IN` is fed by U2
(TCR2LN33, always on from VSYS); the PMIC MCP16701 is enabled by
`PMIC_ON_REQ` (RT1176 SNVS output, via R6/R7/R9 to DA_GPIO4_SYS_EN); the
mic+IMU rail is the U3 LDO, ON by default (F3); the WAKEUP pin has a 100 k
pull-up and is pulled low by Q1 (T5838 WAKE) or Q5 (LIS2DUX12 INT1).  The
ONOFF pin has a pull-up and no button on this rev.

Three levels are possible:

| Level | Mechanism | Wakes on | Firmware today | Blocked by |
|---|---|---|---|---|
| L1 idle | FreeRTOS tickless `__WFI()` in GPC RUN mode (GPT2 wake) | any IRQ | exists | — |
| L2 low-power run | stop camera + `TPU_POW_EN = 0` (cuts MCM_3V3, the largest consumer) + CSI/USB clocks gated; CPU stays in RUN/WAIT via GPC | USB, UART6, WAKEUP GPIO IRQ, button | nothing | F7 handler, measurement |
| L3 SNVS power-down | `SNVS->LPCR = DP_EN \| TOP` → PMIC_ON_REQ low → PMIC rails off; only SNVS (RTC, LPGPR) alive; U3 keeps mic+IMU powered | WAKEUP pin (mic AAD WAKE, IMU INT1), SRTC alarm, ONOFF (no button) | only in the disabled example | **F1** (the wake is a cold boot through the ROM → with the current straps it lands in the serial downloader), PMIC_ON_REQ pad mux, LPWUI config |

Proposed firmware shape (after F1 is fixed on hardware):

1. `sentai.sys.wake_reason()` — read `SNVS_LPSR` / `SRC_SRSR` at boot and
   keep it in a breadcrumb (SNVS LPGPR survives the power-down).
2. `sentai.imu.wake_on_motion(threshold_mg)` — LIS2DUX12 wake-up engine on
   INT1 (already implemented in `libs/lis2dux12`, used today only for its
   side effect); `sentai.mic.wake_on_sound(mode, threshold_db)` — T5838
   AAD-A/D1/D2 through the existing `T5838Aad` class (THSEL on GPIO_AD_18,
   bit-banged clock on LPSR_00, restore PDM clock afterwards).
3. `sentai.sys.sleep("snvs", seconds=None)` — arm the sources, optional
   SRTC alarm, flush FS (`FxUserSync`), stop USB cleanly, then
   `shutdown_system()` as in the example (LPWUI_EN, DP_EN | TOP).
4. A WAKEUP GPIO handler for L1/L2 (F7) so events are visible while awake
   (`sentai.sys.wake_events()` counter + dmesg line), and so the IRQ storm
   case is bounded (edge → disable → re-enable after debounce).

Measurements to take before choosing L2 vs L3: board current in L1 with
camera/TPU off (FNIRSI logger already running), L3 current (expected: U2 +
SNVS + U3 + mic AAD + IMU LP ≈ well below 1 mA), wake latency to REPL.

## 4. Proposed order of work (each step = one build + EXP-s249 + its own test)

| Step | Items | Risk | Needs |
|---|---|---|---|
| A | F3 comment fix; F6 IRQ table; F9 table hygiene; F2 option (b) or (a) | low | decision on F2 |
| B | F4 PwmInit fix + F10 AD_02 GPIO low | low | decision on F10 default |
| C | F5 board-rev pin_mux section (AD_12..23, SD_B1_03/05) | medium (touches boot muxing; verify camera/SD/mic after) | — |
| D | F8 TPU PGOOD bound + BOOT_FAIL/SD_ALARM exposure | low | new SERR codes |
| E | F7 WAKEUP handler + `wake_events()`; IMU/mic wake arming bindings | medium | — |
| F | L3 SNVS sleep (`sentai.sys.sleep`) | medium | **F1 hardware fix first** |
| D' | F14 bounded `sentai.sdcard.init()` | low | — |
| — | F1 button test now; reference-board fuse/strap read-out (DONE: EXP-s252 — the reference board boots through straps, BMOD=10, no fuses burnt; it is most likely the earlier revision); HW change request to the colleague | — | operator |

Not proposed (explicitly left alone): PMIC rail sequencing (works, LDO1-4
default on), camera power/reset sequence, SD power/VSELECT, USB-C CC
handling, JTAG.

## 5. Open questions for the operator

1. F1: run the USER-held reset test; if it boots from flash, do we keep that
   as the interim procedure until the colleague fixes the straps?
2. F2: SDK debug console → LPUART1 (J4 header) or disabled?
3. F10: white TPU LED off by default, or TPU-activity indicator?
4. Sleep: is the target L2 (fast, USB stays) or L3 (lowest power, cold boot
   on wake, needs F1)?  Which wake sources matter first: sound, motion, timer?
5. Reference board: when it is plugged in alone, RAM-load the current build
   and dump SBMR/OCOTP for comparison (no flash write).
