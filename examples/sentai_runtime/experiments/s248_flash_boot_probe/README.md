# s248 — why does the new board rev boot into ROM SDP instead of flash?

**WBS**: `OP-S10-W22-T4` (new board rev 2026-10 bring-up) · EXP-s248 ·
follows [s247](../s247_sdcard_usdhc1_bringup/) (same board, same session)
**Started**: 2026-10-30
**Status**: 🟢 ROOT CAUSE FOUND (software side complete) — ROM samples
**BOOT_MODE[1:0] = 00 ("Boot From Fuses") with BT_FUSE_SEL = 0** → falls
back to Serial Downloader.  Flash content is fine.  Fix is on the board:
BOOT_MODE must read `10` (internal boot) at POR — check SW1 / BOOT_MODE1
pull-up with the electronics colleague.

## Result (build 1609, `sentai.sys.peek32`, straps latched at the last POR — USER not pressed)

```
SRC_SBMR2 = 0x00000009  → BMOD = 00 (Boot From Fuses), BT_FUSE_SEL = 0, SEC_CONFIG = 01 (open)
SRC_SBMR1 = 0x00000000  → BOOT_CFG1..4 all 0x00 (GPIO BOOT_CFG straps ignored in fuse mode anyway)
```
With BMOD=00 the ROM boots from the OTP BOOT_CFG fuses; with BT_FUSE_SEL=0
those fuses are "not programmed" and the ROM enters Serial Downloader —
exactly the `1fc9:013d` we see on every power-up.  The original Coral board
pulls BOOT_MODE1 high (sheet 16 table: OPEN → `10` = Internal Boot); on this
unit BOOT_MODE1 reads **low** at POR.

Options:
1. **Board (preferred, reversible):** make BOOT_MODE[1:0] = `10` at POR —
   SW1 position / populate R29 (BOOT_MODE1 4.7 k to VDD_1V8) / R163, and
   confirm DCDC_1V8_OUT is up before POR_B releases.
2. **OTP fuses (irreversible, operator decision only):** program
   BT_FUSE_SEL=1 + BOOT_CFG fuses for FlexSPI1 Serial NAND so `00` means
   internal boot from fuses.  Not done; not recommended on a dev unit.

## Symptom

- Plain USB replug (USER **not** pressed) → board enumerates as ROM Serial
  Downloader `1fc9:013d`, never as firmware `1fc9:c0a1`.
- `flashtool.py -e sentai_runtime` (persistent) completes rc=0; afterwards the
  board drops off USB; after POR it is back in SDP.
- `--ram` loads run fine (builds 1602–1608 verified via `sentai.version()`).
- J-Link cannot attach (`Failed to power up DAP`, see s247 README) → no
  register reads that way.

## Hypotheses

| # | Hypothesis | How to test | Result |
|---|---|---|---|
| H1 | Flash never written / FCB missing | flashloader `read-memory 0 0x1000 … 0x101` | ❌ refuted — valid FCB at page 0 (see below) |
| H2 | NAND geometry mismatch (new part) | `get-property 0x19 0x101` | ❌ refuted — 128 MB, 2 KB pages, 128 KB blocks = flashtool's .bd geometry |
| H3 | BOOT_MODE[1:0] straps wrong — SW1/R163/R29 on sheets 15/16 | read `SRC_SBMR2.BMOD` from firmware | ✅ **CONFIRMED (variant)** — BMOD = 00 "fuses" with BT_FUSE_SEL = 0 → serial downloader; expected `10` |
| H4 | BOOT_CFG straps wrong (e.g. FlexSPI2 / NOR instead of Serial NAND) | read `SRC_SBMR1.BOOT_CFG1..4` | ◻ moot — SBMR1 = 0 (not sampled in fuse mode); re-check after H3 is fixed |
| H5 | Boot-image copies at blocks 4/8 corrupt | flashloader read at 0x80000 / 0x100000 | ⚠ blocked — `kStatus_InvalidArgument` for any address past block 0 (flashloader SPI-NAND address semantics unclear) |

## Evidence so far (probe_run1.log, blhost via flashloader K2.8.0)

```
SPI NAND Attributes: Start Address = 0x00000000  Total Size = 128 MB  Page Size = 2 KB  Block Size = 128 KB
page 0: 11d8bdb4 4243464e 00000001 00000080 | 0040 0002 0003 … | @0x40: 0100 0000 0100 0000 0200 0000 0100 0000
```
Decoded as the RT1170 Serial-NAND FCB: fingerprint `NFCB`, version 1,
firmware table `[0] = {startPage 0x100 (block 4), 0x100 pages}`,
`[1] = {startPage 0x200 (block 8), 0x100 pages}` — matches
`load spinand bootImageFile > 0x4` / `> 0x8` in `flashtool.CreateSbFile`.
Bytes 0x400.. of page 0 are 0xFF (unused remainder of the FCB page).

## Schematic facts (sheet 15 "Boot", sheet 10 "Flash Boot")

- BOOT_CFG resistor array default: FlexSPI1, 400 ns CS interval,
  FCB/DBBT search count 1, **primary boot device 11 = Serial NAND**, high
  speed, 12-bit column, hold by Read Status, 64 pages/block.
  "STRONG PULLUPS REQUIRED" note.
- External boot switch SW1 on BOOT_MODE[1:0] (`GPIO_LPSR_02/03`), R163 4.7 k
  pull-up to DCDC_1V8_OUT, R164 100 Ω.  Table: 00 fuses, **01 serial
  downloader**, 10 internal boot, 11 reserved.  On-chip default: 25–50 k PD.
- Sheet 16: USER button also drives BOOT_MODE0/1 through D6/Q4
  (pressed → 0/1 = serial downloader).
- Sheet 10: U17 "1V8 QSPI Flash", 8-pin (CS#, SO/IO1, WP#/IO2, VSS, SI/IO0,
  SCK, HOLD#/IO3, VCC) — pinout identical for QSPI NAND and NOR; no part
  number on the sheet.  The flashloader talked to it as SPI NAND, so NAND.

## Tools

- `probe_boot.py` — SDP → flashloader → SRC register read attempt (fails:
  flashloader refuses peripheral space, `kStatusMemoryRangeInvalid`) →
  NAND configure + page-0 dump.  Reuses `scripts/flashtool.py` helpers.
- `nand_page0.bin` — raw FCB page.
- Next: `sentai.sys.peek32(addr)` (new binding) to read
  `SRC_SBMR1 0x40C04008`, `SRC_SBMR2 0x40C0400C`, `SRC_SRSR 0x40C04010`
  from the RAM-loaded firmware — these latch the straps at POR.

## Side observation

While idle on the RAM image (build 1608) for ~10 min between steps, the board
was found back in SDP without any host action → something reset it (WDOG?
brownout?).  Not investigated here; see `/log/boot_prev.log` once the user
partition is out of SAFE MODE.

## Side finding — REPL `SyntaxError: invalid syntax for integer` (FIXED, build 1628)

The `peek32` probe line itself would not parse:
`print("SBMR1=0x%08X ..." % (p(0x40C04008), ...))` →
`SyntaxError: invalid syntax for integer with base 16`, although the echo
showed the input intact.  Bisect (builds 1627/1628, `/dev/ttyACM0`):

| shape | result |
|---|---|
| `print("%X %X %X" % (p(0x40C04008), ...))` | OK |
| `print("SBMR1=0x%08X ... zzz", 0x40C04008)` (long string first) | FAIL base 16 |
| same with `1086341128` | FAIL base 10 |
| `print(0x40C04008, "SBMR1=0x%08X ... zzz")` (literal first) | OK |
| `print("0x%08X" % 0x40C04008)` (short string first) | OK |

Trigger = literal > 2^30 (small-int overflow → long-int path) **and** a
longer token earlier on the line.  Root cause in
`py/objint_longlong.c:mp_obj_new_int_from_str_len()`: `strtoll()` on the
lexer's non-NUL-terminated `vstr` ignored `len`, reading the stale tail of
the previous token ("8X SBMR2…" → extra hex digit → `str != top` →
value_error).  Fix = `patches/micropython/0001-…honour-len.patch`
(bounded loop, `_` separators, `OverflowError`, `neg` applied).  Verified:
original line prints `SBMR1=0x00000000 SBMR2=0x00000009 SRSR=0x00000000`;
`int("-1086341128")` now negative (was positive);
`print(99999999999999999999)` → `OverflowError` (was silent garbage).

Second REPL defect fixed in the same build: CRLF pastes ended indented
blocks after the first body line (`'\r'` ended the line, the following
`'\n'` was read as an empty line).  `repl_readline` now swallows the `\n`
of a CRLF pair (one-byte pushback otherwise); a 3-line `for` body pasted
with `\r\n` runs to completion.

## Update 2026-10-31 — SW2 "BOOT" test (OWL Rev2 top silk: SW2 BOOT, SW3 RST)

Operator held SW2 while pulsing SW3, then the firmware (RAM-loaded, SRC
registers untouched by the RAM load) read the straps latched at that reset:

```
SRC_SBMR2 = 0x01000009  → BMOD = 01 (Serial Downloader forced), BT_FUSE_SEL = 0
SRC_SBMR1 = 0x000004C0  → BOOT_CFG sampled: Serial NAND, FlexSPI1 — identical to the flash-booting reference board (s252)
```

So SW2 pulls **BOOT_MODE0** high (the classic "enter bootloader" button), and
without it the pins read `00`.  Internal boot needs `10`: **BOOT_MODE1 high,
BOOT_MODE0 low**.  The BOOT_CFG straps are already correct, so the only
missing part is the pull-up on BOOT_MODE1 (GPIO_LPSR_03; R163/R164/R29 DNP on
the 2026-10 schematic).  Populating it should make the board boot the build
that is on the NAND (`/default.elf` = 1666 since s253) with no firmware change.

**Second unit (85b1, 2026-10-31):** `SBMR2 = 0x00000009`, `SBMR1 = 0`, boot
fuses `0xFFF70008` — identical to 58eb.  Two of two rev-1.1 boards: the
BOOT_MODE1 pull-up is missing on the revision, not on one unit.

## eFuse option — analysis (2026-10-31, on request; NOT executed)

**What we know (measured, both rev-1.1 units + the old reference board):**

| item | value | meaning |
|---|---|---|
| OCOTP word 0x960 (shadow FUSE022) | `0xFFF70008` on all three boards | bit 4 = **BT_FUSE_SEL = 0** (confirmed location: [NXP community](https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/Checking-Fuses/m-p/1719041)); bit 3 set and the upper half `0xFFF7` are factory/unknown bits, identical everywhere → leave alone |
| OCOTP 0x940 / 0x950 (FUSE020/021) | `0xFFFF0000` | lower halves 0 → no BOOT_CFG fuses programmed |
| `SRC_SBMR1` on the reference board | `0x000004C0` | the strap-sampled BOOT_CFG the ROM *did* boot from: BOOT_CFG1 = `0xC0` (bit 7 = NAND boot, bit 6 = option), BOOT_CFG2 = `0x04` |
| `SRC_SBMR2` with SW2 held | BMOD `01` | Serial Downloader stays reachable through SW2 |

**What a fuse fix would be:** `BT_FUSE_SEL = 1` (0x960 bit 4) **plus** the
BOOT_CFG fuses set to exactly what the straps give on the reference board
(`BOOT_CFG1 = 0xC0`, `BOOT_CFG2 = 0x04`).  BT_FUSE_SEL alone is wrong for
us: with BOOT_CFG fuses at 0 the ROM boots FlexSPI **NOR**
([Embedded Artists](https://www.manualslib.com/manual/2200790/Embedded-Artists-Imx-Rt1176.html?page=12), [NXP community](https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/Regarding-Boot-Configuration-Settings-on-MIMXRT1170/m-p/1906794/highlight/true?profile.language=en)) and we have serial NAND.
The fuse word that holds BOOT_CFG1/2 is 0x940 by our reading of the RM; it
was **not** confirmed by a primary source in this session — IMXRT1170RM
"System Boot → Fusemap" (table 8-x) must be checked bit by bit before any
burn ([forum reading of BOOT_CFG1[7] = NAND](https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/Unable-to-boot-from-NAND-flash-in-imxrt1170-eval-board/m-p/1817553?profile.language=en)).

**Procedure if ever done (one unit first, 85b1):** flashloader via
flashtool, `blhost efuse-read-once 0x16` and `0x14` to confirm the current
words, then `efuse-program-once 0x14 <BOOT_CFG word>` and only after a
read-back `efuse-program-once 0x16 0x10` (BT_FUSE_SEL) — fuses are OR-only,
one shot, no undo ([blhost UG](https://www.NXP.com/docs/en/user-guide/MCUBLHOSTUG.pdf), [EA how-to](https://www.embeddedartists.com/wp-content/uploads/2020/03/iMX_Howto_Program_Fuses.pdf)).  Never touch SDP_DISABLE / SEC_CONFIG / DIR_BT_DIS bits.

**Risk / consequence:** irreversible; boot device locked to serial NAND on
FlexSPI1 for the life of the chip (no SD-card or NOR boot experiments later);
a wrong BOOT_CFG value is *recoverable* (ROM falls back to the Serial
Downloader, and SW2 = BOOT_MODE0 forces it) as long as SDP stays enabled.
**Recommendation:** the hardware fix (R163 to DCDC_1V8_OUT, optionally drop
Q4) is reversible, free and already fully characterised — do that first;
keep the fuse route for units that cannot be reworked, and then only after
the fusemap bits are confirmed in the RM and the exact words were tried on
one unit.
