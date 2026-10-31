# s247 — microSD bring-up on the new board revision (USDHC1)

**WBS**: `OP-S10-W22-T1` (new board rev 2026-10 bring-up) · EXP-s247
**Started**: 2026-10-30
**Status**: 🟢 PASS — (1) block-level bring-up (build 1608), (2) **user FS
migrated to the card**: FileX FAT32 + fault-tolerant journal over
`fx_sd_driver`, mounted at boot in 0.8 s, `/log/boot.log` on the card
(build 1616), (3) **both cameras captured to the card and fetched over
HTTP** (build 1622) — see "Phase 2/3" below

## Why this experiment exists

The electronics colleague delivered a new schematic (Altium, 10/30/2026).
Among the changes: the microSD socket moved to **USDHC1** with GPIO-controlled
card supply and pad-rail selection, and he warned that the inherited Google
`libs/sdcard` "did not set the SD voltages correctly".  SD-card support had
never been wired into the SentAI runtime.  Goal: talk to the card from the
REPL, read-only, and identify what is on it — without touching the cameras
(not connected during this session).

## Hardware (schematic sheet 16 "Interface", sheet 6 "RT1176 PART2")

| Net | Pad | Role |
|---|---|---|
| `SD1_CMD/CLK/D0..D3` | `GPIO_SD_B1_00..05` | USDHC1 (Google code used USDHC2 = spare pads) |
| `SD_PWREN_B` | `GPIO_AD_12` → GPIO3.11 | P-FET U12 gate, VDD_3V3 → VSD_3V3; **active low** |
| `SD1_VSELECT` | `GPIO_AD_16` → GPIO3.15 | NX3L1T3157 U6 select for `NVCC_SD1`; **0 = 3.3 V, 1 = 1.8 V** |
| card detect | — | not wired |

The card is only ever supplied 3.3 V → signalling stays 3.3 V, UHS-I (1.8 V)
switching disabled, bus capped at 50 MHz.

## What changed in the firmware

- `libs/sdcard/sdcard.cc`: USDHC1, the two GPIOs above, power cycle
  (off 30 ms → on 30 ms) before `SD_Init`, `ioVoltage = nullptr`,
  `maxFreq = 50 MHz`, FatFs drive `"2:"` (= `SDDISK`) with FRESULT logging.
- `third_party/modified/nxp/rt1176-sdk/fatfs/ffconf.h`: `FF_VOLUMES 1 → 3`.
  With `FF_MULTI_PARTITION=0` logical drive N is physical drive N and the SD
  glue only answers `pdrv == SDDISK (2)`; with one volume every `f_mount`
  landed on the (not compiled) RAM disk → `FR_NOT_READY`.
- `third_party/modified/nxp/rt1176-sdk/fatfs/fsl_sd_disk.c` (new, copy of the
  vendor file): `sd_disk_initialize()` returns ready when `g_sd` was already
  brought up by `SDCard::Init()`.  The vendor flow re-ran `SD_Init()` on the
  copied struct (same `sdmmchost_t`) and **deadlocked the MicroPython task**
  (REPL silent, board still enumerated; recovered with `--ram` reload).
- `examples/sentai_runtime/sentai_sdcard_glue.cc` + `bindings/modsentai_sdcard.c`:
  `sentai.sdcard.{init, deinit, info, probe, hexdump, mount, unmount, ls,
  fs_info}` — read-only, scalars/strings only, sector data stays in C
  (NonCacheable 512 B buffer).  Registered in `modsentai.c`; ARM-only.
- Linker: new `.sdcard` output section → SDRAM (17.4 KB).  Without it
  `region m_text overflowed` (ITCM `.text` is at 240.8 / 259 KB).
- QSTRs regenerated (`/qstr-regen`).

## How to run

```bash
venv-flash/bin/python scripts/flashtool.py -e sentai_runtime --ram   # this board does not boot from flash yet
python3 - <<'E'
import serial,time
s=serial.Serial('/dev/ttyACM0',115200,timeout=0.5)
def cmd(c,w=2): s.write(c.encode()+b'\r\n'); time.sleep(w); print(s.read(65536).decode(errors='replace'))
s.write(b'\x03\r\n'); time.sleep(0.5); s.read(4096)
cmd('sentai.verbose(1)')
cmd('print(sentai.sdcard.init())', 5)
cmd('print(sentai.sdcard.info())')
cmd('print(sentai.sdcard.probe())')
cmd('print(sentai.sdcard.mount())', 6)
cmd('print(sentai.sdcard.fs_info())', 10)
cmd('print(sentai.sdcard.ls("/"))', 5)
E
```

## Pass criteria

1. `init()` → True, `SD_Init` status 0x0 (no voltage-switch attempt).
2. `probe()` identifies the partition table / FS from LBA0 without writing.
3. `mount()` → True and `ls("/")` returns > 0 entries matching what a host
   PC shows for the same card.
4. REPL stays responsive throughout (no WDOG reset, no silent hang).

## Result (build 1608, 2026-10-30 21:14)

See [repl_transcript.log](repl_transcript.log).

- `info()` → `(124735488, 512, 60906, True, 'SD64G')` — 64 GB SDXC.
- `probe()` → `LBA0=MBR part0 type=0x07 start=32768 sectors=124702720 exFAT
  bytes/sec=512 sec/clus=256`.
- `mount()` → `"/sd" -> FatFs drive "2:" fs_type=4` (exFAT).
- `fs_info()` → `('exFAT', '', 60874, 36882)` MB total / free.
- `ls("/")` → 15 entries (`DCIM`, `EVENT`, `MISC`, `coral`, `System Volume
  Information`, car-infotainment and GoPro leftovers).  Card was clearly
  reused from a consumer device; no SentAI data on it yet.

All four criteria met.  Nothing was written to the card.

## Dead-ends logged

1. **Build 1606 — `mount()` False, `FRESULT 12`**: FatFs drive mapping
   (`FF_VOLUMES=1` vs `SDDISK=2`).  Fixed in ffconf.h.
2. **Build 1607 — `mount()` hangs the REPL**: double `SD_Init` from the
   vendor `fsl_sd_disk.c`.  Fixed with the modified copy.
3. **Hidden `verbose(0)` trap** (agent.md §2 rule 8): one probe run printed
   nothing because `verbose(0)` also silences Python `print`.
4. **J-Link read-only probe failed**: `Found SW-DP 0x6BA02477` but `Failed to
   power up DAP`; JLinkExe's "connect under reset" fallback reset the board
   and killed the RAM image.  Open — see memory
   `project_new_board_rev_2026_10_open_items.md`.

## Phase 2 — `sentai.fs` moves to the card (operator decision 2026-10-30)

Design: FileX **without LevelX** (the card has its own FTL), fault-tolerant
journal ON, FAT32 / 32 KB clusters, superfloppy when formatted by the board;
MBR cards handled (partition 0).  `libs/base/fx_sd_driver.{h,cc}` is the
FX_MEDIA driver + MSC block layer; `fx_user_fs.cc` keeps its API, mutex and
SAFE-MODE semantics.  Mount moved **post-scheduler** into `app_main()`
(`FxUserBootMount(20000)`), never in `real_main()`.  FatFs removed from
`libs/sdcard`; `sentai.sdcard.*` is block-level diag only.

Dead-ends (all logged in `repl_transcript.log`):

| build | symptom | root cause | fix |
|---|---|---|---|
| 1612 | `fx_media_format` → `FX_IO_ERROR` in 27 ms; board reset later | `SDCard::WriteBlocks` compared `SD_PollingCardStatusBusy()` against `kStatus_Success`; the SDK reports success as `kStatus_SDMMC_CardStatusIdle` (1844). Every good write was reported failed. Also the format wrote the VBR at the OLD exFAT partition offset (32768). | drop the redundant poll (SD_WriteBlocks already waits); `FxSdResetPartition()` before format |
| 1613 | boot never reaches the REPL (console stops after `[fx_sd] card ready`) | `fx_media_open` walked FAT chains built from the half-formatted card and never returned — unbounded op on the boot path | `FxUserBootMount()` runs the mount in a task with a 20 s deadline → SAFE MODE; `validate_boot_sector()` rejects foreign/corrupt BPBs before `fx_media_open`; `lba_range_ok()` in the driver |
| 1614 | — | — | exFAT card → SAFE MODE in 0.7 s, REPL alive ✓ |
| 1616 | — | — | format 2.3 s, mount 0.8 s, write/read/ls/sync ✓, `/log/boot.log` on card ✓ |

embeded.md §14 review (embeded-reviewer agent) on the diff: 1 CRITICAL (reap
path used `vTaskDelete`/`vSemaphoreDelete` on a task that may own the SDK
host mutex → replaced by an SD abort flag + bounded unwind), 3 MAJOR (double
partition offset on PC-formatted cards → single offset + BPB check 10; MSC
medium-present without UNIT ATTENTION → `NotifyMediaChanged()`; no SD-lost
state → `FX_SD_MAX_CONSECUTIVE_ERRORS` → SAFE MODE), minors partially
addressed (format with no card now returns False; new codes 0x0D35–0x0D37).

## Phase 3 — cameras on the new board (build 1617–1622)

- Camera 1 moved to **LPI2C1** (`main_freertos_m7.cc`: `I2cInitController(kI2c1)`,
  24 MHz source clock, pads GPIO_AD_32/33 open-drain); camera 2 stays on LPI2C6.
- Build 1617–1621: camera 1 **NAKed every transfer** (`LPI2C status 902 =
  kStatus_LPI2C_Nak`) while `Detect()` reported success — false positive:
  `CameraTask::Read()` fills the same buffer from both buses and `Detect()`
  checks the value, not the status.  `CAMERA_DEVICE_Init = 1` because the
  driver aborts on the first failed write.
- Root cause (read back with `sentai.sys.peek32`): pad `GPIO_AD_00`
  (`FLEXPWM1_PWM0_A`) had `SW_PAD_CTL = 0x0E` = internal **pull-up**, pin
  level 1 → Q7 on → isolation switch U9 disabled → LPI2C1 cut from camera 1.
  Exactly the electronics colleague's warning.  Fix: pad pull-down (0x06) +
  GPIO2.31 driven low (same for GPIO_AD_01 / add-on switch) before camera
  init.  Build 1622: both cameras ID `0x5640`, 30 fps, 39 frames in 1.3 s.
- Captures: `cam0.jpg` (26 234 B) and `cam1.jpg` (26 968 B), 640×480 q85,
  95 ms capture + 34 ms SD write each; fetched intact over CDC-NCM
  `http://10.0.0.1/api/raw/...` after `sentai.usb.ip(1)` (REPL `repr()`
  transfer of >6 KB chunks drops bytes — CDC TX ring; use HTTP or ≤512 B).
- `sentai.camera.power_rails(on)` (PMIC LDO1–4) added for bring-up; whether
  it is required is checked in `camera_both_capture.log` (fresh-boot test).

## Open items (not part of this experiment)

- Board boots to ROM SDP on plain replug; persistent flash image does not run.
- `SENTAI_API.md` section for `sentai.sdcard` not yet written.
- Write path (`sentai.sdcard.write`/FR logging to SD) intentionally absent.
