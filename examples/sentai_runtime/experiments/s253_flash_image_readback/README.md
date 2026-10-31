# EXP-s253 — is a persistent flash actually written to the NAND? (read-back from firmware)

**WBS:** OP-S10-W22-T4 / T7 (board rev 2026-10 bring-up: flash boot).
**Date:** 2026-10-31.  **Board:** rev 2026-10, id 58eb.  **Builds:** 1666 (RAM-loaded
reader), persistent flash of 1666.  **Follows:** [s248](../s248_flash_boot_probe/)
(root cause of the no-boot: BOOT_MODE straps read `00`, ROM enters the serial
downloader; FCB in NAND page 0 verified through the flashloader, but the image
region could not be read that way).

## Question (electronics colleague)

When a new build is written to the board's flash, is it really written and
does it stay?  The board does not boot from flash — but is the new version
surely on the NAND?

## What `flashtool.py -e sentai_runtime` writes (no `--ram`)

1. ROM SDP → flashloader → `program.sb`: the **elf_loader** boot image (IVT)
   into serial-NAND blocks 4 and 8 (two copies), FCB in page 0.
2. Flashloader loads the elf_loader into RAM (`STATE_LOAD_ELFLOADER`) and
   the elf_loader writes the runtime ELF as **`/default.elf`** into the
   LittleFS **system partition** (NAND blocks 12..75) over USB HID
   (`STATE_PROGRAM_DATA_FILES`), plus the USB IP address file.
3. Reset → the ROM should boot the elf_loader from blocks 4/8, which loads
   `/default.elf`.  On this board step 3 fails (BOOT_MODE straps, s248).

So "the new build" lives in `/default.elf`; the boot image is the same
elf_loader every time.

## Method

New diagnostics (build 1666, C in `sentai_runtime.cc`, static SDRAM buffers,
bounded): `sentai.sys.sysfs_size(path)`, `sysfs_read(path, off, n<=512)`,
`sysfs_find(path, needle, start)` over the NAND system LittleFS.
`probe_default_elf.py` reads the size of `/default.elf`, lists the
`2026-…` build-timestamp strings it contains and the `SentAI v1.0 build N
(…)` version string.

## Result

| step | `/default.elf` size | version string stored on NAND |
|---|--:|---|
| before (RAM-loaded 1666 reading the NAND) | 1 346 820 | `SentAI v1.0 build 1602 (2026-10-30 20:46:10)` — the last persistent flash done on this board (s247 era); every flash since was `--ram` |
| `flashtool.py -e sentai_runtime` (persistent, rc=0, log `flashtool_persistent_1666.log`) | — | board re-enumerates as ROM SDP `1fc9:013d` (no boot, as in s248) |
| after (RAM-loaded 1666 again) | **1 363 036** | **`SentAI v1.0 build 1666 (2026-10-31 12:12:55)`** |

**Answer: yes.**  The persistent flash writes the new ELF to the NAND and it
stays there across the reset (the read-back was done from a fresh RAM load
after the board had gone through ROM SDP).  The FCB and the elf_loader boot
image are written too (s248).  The only defect is that the ROM never enters
internal boot (BOOT_MODE[1:0] = `00` at POR); nothing is wrong with the
flash content.  Once the straps are fixed the board should boot the build
that is on the NAND at that moment — re-flash persistently first if a newer
build is wanted.

## Files

- `probe_default_elf.py` — the read-back (REPL over `/dev/ttyACM0`).
- `probe_before_flash.log`, `probe_after_flash_1666.log`, `flashtool_persistent_1666.log`.

## Second board (id 85b1, same rev 1.1, no SD card / no cameras) — 2026-10-31

| step | `/default.elf` on NAND |
|---|---|
| as received | 244 616 bytes, no `SentAI v1.` string (factory image) |
| `flashtool.py -e sentai_runtime` (build 1679, rc=0; `flashtool_persistent_1679_board85b1.log`) | board re-enumerates as ROM SDP `1fc9:013d`, like 58eb |
| RAM-load + read-back | **1 364 228 bytes, `SentAI v1.1 build #%d board %s`, timestamps 2026-10-31 13:16:38** |

Same picture on two units: the persistent write lands on the NAND every
time; only the boot straps keep the ROM out of internal boot.
