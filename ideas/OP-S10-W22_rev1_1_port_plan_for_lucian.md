# OP-S10-W22-T11 — Board rev 1.1 hardware support as a SentAI-free package (for the camera colleague)

**WBS**: OP-S10-W22-T11
**Date**: 2026-11-09
**Status**: DONE 2026-11-09 — branch `lucian/board-rev-1.1` from `6448b25d`, FatFs API kept, tested on 58eb (EXP-s272). Decisions taken by the operator: base = his last commit here, keep FatFs, branch name `lucian/board-rev-1.1`, camera SDK patches included.

## 1. Question

The camera colleague (Lucian) works on a branch of this repository that must stay
free of the SentAI runtime (`examples/sentai_runtime`, MicroPython, `sentai_*`
symbols, experiments, ideas/, .claude/).  He needs the board rev 1.1 (hardware
revision 2026-10, "OWL-R2") support: microSD on USDHC1, the two cameras with CAM1 on
LPI2C1, the LIS2DUX12 accelerometer, the T5838 pin remap, the boot-fuse story and the
tooling.  Can that be delivered onto a new branch started from his branch without
pulling SentAI in?

## 2. Where his work actually is

| Ref | Date | Content | SentAI-free? |
|---|---|---|---|
| `origin/lucian/ov5640-camera-support` (8dc319a6) | 2025-02-13 | OV5640 QVGA/720p/1080p modes | yes, but stale (518 commits behind) |
| `c672d072` (Lucian) | 2025-07-21 | MUX support for dual camera | yes |
| `593fd812` .. `6448b25d` (Lucian, merged by `eb427d1d`) | 2026-02-11 .. 2026-03-23 | LIS2DU12 accel, T5838 audio + AAD wake, PMIC, SD card (`libs/sdcard` + `examples/sdcard_test`), I2C1 test, console forward over TCP/HTTP | **yes** — last commit before `rename to sentai` (edc68d2e, 2026-03-26) |
| `origin/feature/support-custom-coral-micro-board-v1` (ab24db16) | 2026-03-31 | merge of the above + first SentAI/MicroPython/meshproxy commits | no |

Lucian's 2026 work is already in our history (merged 2026-03-23).  The branch on this
remote under his name is the 2025 one; his current head probably lives in his own
fork (his merge `6f5e12ce` pulled from a remote named `bogdan`, i.e. he works
elsewhere).  **Base for the new branch: `6448b25d`** (his last commit here, no
SentAI, vendor examples intact), unless he gives us a newer fork URL.

## 3. What "rev 1.1 support" is in our tree (relative to ab24db16)

| # | Delta | Files (HEAD) | Rev-1.1 specific? | Coupled to SentAI? |
|---|---|---|---|---|
| A | CAM1 on LPI2C1 (handle, 24 MHz source clock, ODE pads 0x10); PMIC alone on I2C5 | `libs/base/main_freertos_m7.cc` (boot block ~L560-600) | yes | block itself no; file has ~70 `sentai_*` refs around it |
| B | FlexPWM1 PWM0_A/B pads (GPIO_AD_00/01) forced low + pull-down so the U9/Q7 I2C isolation switch stays closed; `PwmInit()` must not re-mux them (audit F4) | same block; `libs/base/pwm.cc` untouched (F4 open) | yes | no |
| C | `CameraTask::Init(i2c_handle, i2c_handle2)`, per-bus `Detect(cam_id)`, `DetectedMask()`; cam_mux.h convention text (LPI2C6 instead of LPI2C2) | `libs/camera/camera.{cc,h}`, `libs/camera/cam_mux.h` | yes | small; the rest of the camera diff (CSI ISR hooks, deferred MUX flip, YUV420, SXGA rows, recovery hooks) is SentAI pipeline work, not rev 1.1 |
| D | `LDO_1V8_INT_EN` (GPIO9.20) held 0 (rail fed by PMIC VDD_1V8_AON, audit F3/W22-T6); old THRS drive on GPIO10.6 removed (that pad is USDHC1_DATA1 on rev 1.1) | `main_freertos_m7.cc` ~L540-560 | yes | no |
| E | microSD on USDHC1: `libs/sdcard` rewritten (bounded `InitBounded(deadline)`, init-stage breadcrumbs, SD_PWREN_B / SD1_VSELECT GPIOs, no `CHECK()`); FatFs `Mount/Unmount/Format` removed; `fsl_sd_disk.c` no longer re-runs `SD_Init()` on an initialised host | `libs/sdcard/*`, `third_party/modified/nxp/rt1176-sdk/fatfs/fsl_sd_disk.c`, `ffconf.h` | yes | 2 refs (log + init-stage export) |
| F | elf_loader parks the SD pads during a persistent flash (card off, IO rail 3.3 V; audit F5a/F17) | `apps/elf_loader/elf_loader.cc` | yes | no |
| G | LIS2DUX12 driver (WHO_AM_I 0x47, addr 0x18, deep power-down wake, tap/wake config) + vendored ST `lis2dux12-pid`; `libs/lis2du12` kept for the old board | `libs/lis2dux12/*`, `third_party/st/lis2dux12-pid/*`, `libs/CMakeLists.txt` | yes | driver clean; init lives in `modsentai_hal.cc` (SentAI) → needs a vendor example instead |
| H | T5838 THSEL → GPIO_AD_18 (AD_19 tied to MIC_CLK), audio driver tweaks | `libs/t5838/*`, `libs/audio/*` | yes | 1 ref |
| I | PMIC: camera rails LDO1–4 toggle API; read/write failure no longer reboots | `libs/pmic/pmic.cc` | partly (rails map is rev 1.1) | 1 comment |
| J | LPI2C transfers bounded (`pinLowTimeout_ns` 20 ms) | `libs/base/i2c.cc` | no (hardening) | no |
| K | SEMC BMCR1 arbitration weights | `libs/nxp/rt1176-sdk/board_hardware.c` | no (perf, W24-T12) | no |
| L | Boot eFuses: `BOOT_CFG` 0x4C0 + `BT_FUSE_SEL` tool over blhost (standalone, no REPL), probe notes (`SRC_SBMR2` = 0x9 → serial downloader before fusing) | `experiments/s255_efuse_boot_config/efuse_boot_cfg.py`, s248/s255 READMEs | yes | script imports only `flashtool` |
| M | flashtool: tries both VIDs, default USB IP 10.0.0.1 | `scripts/flashtool.py` | partly | no |
| N | SDK patches: 0001 VGA 30 fps D-PHY, 0005 CSI empty-list BusFault, 0007 SXGA true fps rows (camera, generic); 0002/0006 EHCI (TPU), 0003 CSI hooks (SentAI), 0004 httpd | `patches/coralmicro-rt1176-sdk/*`, `scripts/apply_sdk_patches.sh`, top `CMakeLists.txt` hook | no, but he wants 0001/0005/0007 | 0003 is SentAI-only |
| O | Hardware audit + BOM (F1–F19), schematic pin/net map | `examples/sentai_runtime/agent/board_rev_2026_10_init_audit.md`, `board_rev_2026_10_bom.csv` | yes (docs) | path only |

Not part of the package (SentAI choices): FileX/LevelX + `fx_sd_driver` + `fx_user_fs`
(he has FatFs), `libs/tpu` changes, CDC-NCM/ECM/RNDIS, `msc_ums` raw-SD export,
`console_m7` changes, boot-persist / storage-mode machinery, `check.h` hooks,
`libs/aifes`, everything under `examples/sentai_runtime`, `sim/`, `emu/`, `ideas/`.

## 4. Why cherry-pick does not work

The rev-1.1 commits mix libs with bindings, experiments and memory (`aa453c74` touches
`libs/base/fx_sd_driver.cc` together with `modsentai_sdcard.c`, s247/s248 folders,
`.claude/memory`).  The generic libs at HEAD reference ~40 distinct `sentai_*` symbols
(`sentai_boot_progress_mark`, `sentai_storage_log*`, `sentai_cam_get_raw_with_recovery`,
`SentaiCamMuxSetFromIsr`, `sentai_cam_deferred_flip_isr`, `sentai_logf` …).  Vendor
examples link only thanks to weak fallbacks (`libs/base/sentai_log_weak.cc`).
Cherry-picking would drag the names and the hooks into his tree.  **This is a port:
re-apply the hardware deltas by hand as small, named commits with neutral symbol
names.**

## 5. Proposed branch and commit plan

Branch `PentruLucian` (operator's name; or `lucian/board-rev-1.1`) from `6448b25d`.
Commit subjects carry `OP-S10-W22-T11:` and one delta each:

1. `docs: board rev 1.1 hardware audit, BOM, pin/net map` (O; copied to `docs/board_rev_1_1/`, SentAI references edited out).
2. `sdk patches 0001/0005/0007 + apply script + CMake hook` (N minus 0002/0003/0004/0006).
3. `libs/sdcard: USDHC1 bring-up for rev 1.1` (E + F): keep Lucian's FatFs `Mount/Unmount/Format` API, add `InitBounded`, SD_PWREN_B / VSELECT handling, `fsl_sd_disk.c` fix, elf_loader pad parking.  `examples/sdcard_test` must still build and run.
4. `libs/camera + boot: CAM1 on LPI2C1, FlexPWM1 pad guard, two-handle Init, per-bus Detect` (A, B, C, D).  `camera_streaming_http` with both cameras is the test.  Fix audit F4 in `pwm.cc` while there (do not re-mux AD_00/01).
5. `libs/lis2dux12 + ST vendored lib + examples/lis2dux12_test` (G): the test example replaces the SentAI init path (addr 0x18, power-down wake, WHO_AM_I 0x47).
6. `libs/t5838 + audio: THSEL on GPIO_AD_18` (H), `libs/pmic: camera rail API, no reboot on I2C failure` (I), `libs/base/i2c: bounded transfers` (J).
7. `tools: efuse_boot_cfg.py (standalone) + flashtool VID fallback` (L, M) under `scripts/`, with the s248/s255 findings as `docs/board_rev_1_1/boot_fuses.md` (SRC_SBMR2 reading, what the fuses do, `SW2` recovery).
8. Optional: `board_hardware.c SEMC BMCR1` (K) — only if he runs camera + USB loads.

Validation on hardware: build the vendor examples tree from `PentruLucian`, load with
`flashtool.py --ram` (never persistent) on unit 85b1 so 58eb keeps the SentAI image:
`sdcard_test` (read/write blocks), `camera_streaming_http` (both sensors answer
`0x300A` = 0x56 on their own bus, MUX switch), `lis2dux12_test` (WHO_AM_I 0x47),
`blink_led` from flash boot on the fused unit (proves fuses + elf_loader pad parking).

Effort: about one day of porting plus half a day on the bench.

## 6. Decisions needed from the operator

1. Base: `6448b25d` (his last commit in this repo) or a newer head from his fork (URL).
2. SD card API: keep his FatFs `Mount/Unmount/Format` (recommended — his examples use
   it) and add our fixes, or replace with our raw block-only driver.
3. Branch name: `PentruLucian` as said, or `lucian/board-rev-1.1`.
4. Whether to include the camera SDK patches 0001/0005/0007 (recommended: 0001 fixes
   VGA 30 fps D-PHY sync, 0005 a BusFault after sensor restart; they are generic).
