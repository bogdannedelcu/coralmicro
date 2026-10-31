# EXP-s255 — boot eFuses for "boot from fuses" (BOOT_MODE = 00) — rehearsal on a rev-1.0 board

**WBS:** OP-S10-W22-T4 / T7 (board rev 2026-10 bring-up: flash boot).  **Date:** 2026-10-31.
**Why:** both rev-1.1 units read BOOT_MODE[1:0] = `00` at POR (missing BOOT_MODE1 pull-up,
s248) and the ROM falls into the Serial Downloader because BT_FUSE_SEL = 0.  The RM
(IMXRT1170RM Rev.3 §10 "System Boot") describes exactly this as the production flow:
leave BOOT_MODE at `00`, provision the boot device over SDP, then blow BT_FUSE_SEL +
the BOOT_CFG fuses so `00` means "boot from fuses".  Fuses are permanent, so the
operator asked for a rehearsal on a retired rev-1.0 board first ("does it brick?").

## Fuse words (primary source: IMXRT1170RM Rev.3, Table 26-6 "FlexSPI (Serial NAND) boot fusemap", Table 10-11, §10.3)

| OCOTP word | blhost index | value programmed | meaning |
|---|--:|--:|---|
| 0x940 (BOOT_CFG1 [7:0], BOOT_CFG2 [15:8]) | 0x14 | `0x000004C0` | BOOT_CFG1 = 0xC0: bits 7:6 = `11` Serial NAND via FlexSPI, 50 MHz safe freq, 12-bit column, hold by read-status, FCB search stride 64 pages; BOOT_CFG2 = 0x04: CS interval 400 ns, 1 search, FlexSPI1 — identical to `SRC_SBMR1` of every strap-booting board |
| 0x960 bit 4 | 0x16 | `0x00000010` | BT_FUSE_SEL = 1 |
| 0x9A0, 0xC80 | 0x1A, 0x48 | untouched (0) | PORTA / primary pin group, default NAND timing |

Semantics confirmed in the RM text: with BT_FUSE_SEL = 1 the GPIO BOOT_CFG
overrides are ignored in every mode (`10` and `00` both boot from the fuses);
BOOT_MODE `01` (SW2 "BOOT" on rev 1.1) still forces the Serial Downloader, so a
wrong BOOT_CFG is recoverable over SDP/RAM-load but not by wiring the straps.

## Tool

`efuse_boot_cfg.py` — reuses `scripts/flashtool.py`: resets a running firmware
into ROM SDP (CDC 1200-baud/DTR trick, found by VID/PID), loads the flashloader,
then `blhost efuse-read-once` on the four words; only `--burn I-UNDERSTAND`
programs 0x940 then 0x960, each followed by a read-back, then `blhost reset`.

## Result — board 42a9 (rev 1.0, straps BOOT_MODE `10`, BOOT_CFG `0x4C0`)

| step | observation |
|---|---|
| before | `efuse-read-once`: 0x940 = 0, 0x960 = 0x08 (bit 3 factory), 0x9A0 = 0, 0xC80 = 0; board booted Blink from flash (`flashtool.py -e blink_led`, persistent) |
| burn (`burn_board42a9.log`) | 0x940 → read-back `0x000004C0` OK; 0x960 → read-back `0x00000018` OK |
| after reset | enumerates as the Blink image from flash → **boots, not bricked** |
| from firmware (RAM-loaded 1682) | `SRC_SBMR2 = 0x02000019` (BMOD 10, **BT_FUSE_SEL = 1**), `SRC_SBMR1 = 0x4C0` now sourced from the fuses; shadows 0x940 = `FB3F04C0`, 0x960 = `FFE70018` |

Because BT_FUSE_SEL = 1 makes the ROM take BOOT_CFG from the fuses even in
mode `10`, this boot is a genuine test of the programmed values: the fuse
configuration is correct for our serial NAND.  What this board cannot show is
mode `00` itself (its BOOT_MODE1 strap is high); that is exactly the case of
the rev-1.1 units, where the fuse burn removes the need for the R163 pull-up.

## Plan for rev 1.1

Unit 85b1 first (no SD / no cameras; NAND holds v1.1 build 1679), same two
words, then power-cycle: expected `1fc9:c0a1` "SentAI v1.1 build #1679 board
85b1" with BOOT_MODE `00`.  SW2 + RST must still give `1fc9:013d` (recovery
path).  Then unit 58eb.

## Result — unit 85b1 (rev 1.1, BOOT_MODE straps `00`) — 2026-10-31

Operator ran the burn (`burn_board85b1.log`): 0x940 → `0x000004C0` OK, 0x960 →
`0x00000018` OK, `blhost reset`.  **The board enumerated directly from the NAND:
`1fc9:c0a1` "autonomous.ro SentAI v1.1 build #1679 board 85b1"** — the first
rev-1.1 boot from flash, with no hardware change (`00` = boot from fuses).
Follow-up checks on 85b1 (operator + lsusb): persistent flash of build 1682
→ boots 1682 from the NAND by itself; **USB replug (true POR) → `1fc9:c0a1`
build #1682**; **SW2 held + RST → `1fc9:013d`** (Serial Downloader, recovery
path intact); plain RST → back to the firmware.  The health indicator
(STAT LED code 1 = no SD card, `sys.health()`) runs on the flash-booted
build.  Next: unit 58eb.

## Result — unit 58eb (rev 1.1, cameras + microSD) — 2026-10-31

Operator ran the burn (`burn_board58eb.log`): 0x940 → `0x000004C0` OK, 0x960 →
`0x00000018` OK; after `blhost reset` the board enumerated from the NAND as
build #1666 (what s253 had left there).  Persistent flash of build 1682 →
boots 1682 by itself.  **s249 regression 15/15 on the flash-booted firmware**
(`SBMR2 = 0x00000019`); `sys.health()`: SD, both cameras and the TPU present,
genlock verified (phase 3 µs, pre-trim 12 lines at the default 30 fps).  One
finding: the boot-time IMU probe failed on the cold start (code 4) while the
same init passed seconds later → the driver now retries the wake over three
rounds (build 1683).

Both rev-1.1 units now boot from flash with BOOT_MODE `00`; the strap fix
(R163) is unnecessary.  Recovery stays SW2 + RST → Serial Downloader.
