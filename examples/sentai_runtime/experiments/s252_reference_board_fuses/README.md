# EXP-s252 — boot configuration of a flash-booting board vs the bring-up unit

**WBS:** OP-S10-W22-T4 / T9 (board rev 2026-10 bring-up: boot straps).
**Date:** 2026-10-31.  **Firmware:** build 1648/1649 RAM-loaded on both
boards (flash and fuses untouched; the reference board returns to its own
flash image, build 1601, at the next reset).

## Question

The bring-up unit of the 2026-10 revision always enters the ROM serial
downloader (EXP-s248: `SRC_SBMR2 = 0x9`, BMOD = 00 "boot from fuses",
BT_FUSE_SEL = 0).  The operator has a board that boots from flash and was
believed to have its boot fuses burnt.  Which registers differ?

## Method

`dump_boot_config.py` reads, through `sentai.sys.peek32`: `SRC_SCR/SRMR/
SBMR1/SBMR2/SRSR`, `SRC_GPR1..20`, and all 144 OCOTP fuse shadow words
(`OCOTP_BASE + 0x800 + 16*n`; the OCOTP clock is on, CTRL reads 0x8AD).
`--diff a.csv b.csv` prints the registers that differ.

## Result

| register | reference board | bring-up unit | meaning |
|---|---|---|---|
| `SRC_SBMR2` | `0x02000009` | `0x00000009` | BMOD[25:24] = **10 Internal Boot** vs **00 Boot-from-fuses**; BT_FUSE_SEL (bit 4) = **0 on both** |
| `SRC_SBMR1` | `0x000004C0` | `0x00000000` | BOOT_CFG straps sampled: bits 6, 7, 10 = exactly the sheet-15 resistor table (Serial NAND, FlexSPI1, 400 ns); 0 on the unit because in BMOD 00 the pins are not sampled |
| `FUSE022` @0x960 (BOOT_CFG) | `0xFFF70008` | `0xFFF70008` | identical — **no boot fuses are burnt on either board** |
| `FUSE016/017` (UNIQUE_ID) | `829342A9 092FC00E` | `829358EB 0D1DB80E` | per-chip id (= USB iSerial) |
| `FUSE002, 007, 018, 028-034, 036` | differ | differ | per-chip trim / lock words, not boot related |

Files: `good_board.csv`, `bringup_unit.csv`.

## Conclusion

The reference board boots because its **BOOT_MODE[1:0] straps read `10`**,
not because of fuses.  The boot fuses are identical on both boards.  The
difference is purely on the BOOT_MODE1 (`GPIO_LPSR_03`) strap at POR —
see the init audit (`agent/board_rev_2026_10_init_audit.md`, F1): on the
2026-10 schematic SW1/R163/R164 are DNP and the USER-button circuit Q4/D6
inverts the table.  The reference board therefore has different population
or is an earlier revision.  Indicators that it is an **earlier revision**
(to be confirmed by the operator): with build 1649 loaded, camera init
returns -1 (camera 0 is expected on LPI2C1 on the new rev), `sentai.imu.
init()` returns None (LIS2DUX12 driver; the old board has a LIS2DU12),
there is no microSD, and `sentai.sdcard.init()` **blocks the REPL
indefinitely** (see below).

## Side finding — `sentai.sdcard.init()` without a card hangs the REPL

On the reference board `sentai.sdcard.init()` printed the host-clock line
and never returned; Ctrl-C has no effect (native code), the watchdog keeps
kicking (hw_wdog is alive, USB enumerated), so the board sits there until a
reset.  The boot-time mount is protected (`FxUserBootMount` runs the mount
in its own task with a 20 s deadline) but the REPL binding calls
`SDCard::Init()` directly.  Recorded as audit item F14 (bounded SD init /
card-presence probe before `SD_Init`).

## Reproduce

```bash
# board alone on USB, build >= 1648 RAM-loaded
python3 dump_boot_config.py --label <name>
python3 dump_boot_config.py --diff good_board.csv bringup_unit.csv
```
