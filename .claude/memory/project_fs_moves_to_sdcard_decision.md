---
name: fs-moves-to-sdcard-decision
description: Operator decision 2026-10-30 — the SentAI user filesystem moves EXCLUSIVELY to the microSD card (FatFs); the FileX/LevelX user partition on NAND is dropped; NAND keeps only the LittleFS system partition that elf_loader boots from.
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-30T20:44:11.501Z
---

Operator-stated 2026-10-30, right after the SD card bring-up PASSED
([[sdcard-usdhc1-bringup]]): (paraphrased) now that we have the SD card we can drop the two-partition
NAND format: LittleFS stays on flash and the SD card is mounted as the
only SentAI user FS.

**Target layout:**
- NAND (1 Gbit, FlexSPI1): ROM FCB/DBBT + boot image copies (blocks 4/8,
  written by flashtool) + **LittleFS system partition** (holds the
  `sentai_runtime` ELF + data files that `elf_loader` loads).  Unchanged.
- **No FileX/LevelX user partition on NAND any more.**
- microSD (USDHC1, FatFs, exFAT/FAT32): everything `sentai.fs.*` exposes —
  `/main.py`, `/lib/diag`, `/diags/sNNN_*`, `/log/boot*.log`, `/models`,
  flight-recorder output, captures.

**Why:** the NAND user partition has been a recurring source of pain
(SAFE MODE after brownouts, FileX/LevelX sync subtleties, 1–2 KB/s REPL
uploads, MSC mode switching) and is in SAFE MODE on the new board right
now; a removable 64 GB card is faster to fill from a PC and trivially
inspectable.

**Decided 2026-10-30 (operator confirmed) + IMPLEMENTED build 1612 (uncommitted):**
FileX **without LevelX** on the SD card, fault-tolerant journal ON, FAT32 /
32 KB clusters, superfloppy (no MBR) when formatted by the board; MBR cards
are handled (partition 0).  New `libs/base/fx_sd_driver.{h,cc}` (FX_MEDIA
driver + MSC block layer, 32 B-alignment bounce buffer); `fx_user_fs.cc`
keeps API/mutex/SAFE-MODE; `fx_nand_driver.cc` + LevelX out of the link;
`libs/sdcard` is block-level only (FatFs removed — one FAT stack per card).
**Mount moved post-scheduler into `app_main()`** (USDHC needs FreeRTOS);
`real_main` no longer calls `LfsUserInit`.  Storage mode exposes the RAW
card over MSC (512 B LBAs, count from card) and arms "medium present" only
after the block layer is up (`sentai_msc_set_unit_ready`).  New codes
`SERR_LFX_SD_*` 0x0D2E..0x0D32.  Verified: boots with an exFAT card →
SAFE MODE, USB/REPL fine.  Order agreed: A camera→I2C1, B this FS move,
C capture→SD, D EXP-s249 I/O benchmark.

**Operator's explicit worries (answered in the implementation):**
1. Boot with broken/missing card → `FxSdInit` fails → SAFE MODE, logs
   stay in the SDRAM NOLOAD ring, never `CHECK()` on the SD path.
2. USB mass-storage must survive → yes, now the whole card is the LUN.

**How to apply (constraints the operator set):**
- Read the whole boot path first (`libs/base/main_freertos_m7.cc`,
  `FxUserInit`/`LfsUserInit`, `micropython_task.c` main.py sequencing,
  camera init in `sentai_runtime.cc`) and do not break FreeRTOS start-up or
  camera init.  Anti-brick rules (agent.md §2 rule 1, embeded.md §10) still
  bind: USB CDC must come up even with no card inserted → SD mount failure
  must degrade to SAFE MODE, never CHECK()/hang.
- `sentai.fs.*` API surface stays the same for MicroPython callers; the
  backend behind `FxUser*`/`LfsUser*` compat shims becomes FatFs-on-SD.
- SD writes go through the single-writer task discipline the project
  already uses (lfs_task / fs_task) — no FatFs calls from multiple tasks
  without the existing serialization.
- SIM build must keep working (`sim/sentai_fs_sim_backend.c` is the SIM
  side of `sentai.fs`).
- Related: [[new-board-rev-2026-10-open-items]], [[safe-mode-brick-fix]],
  [[lfs-task-fix]], [[usb-mode-switch]].
