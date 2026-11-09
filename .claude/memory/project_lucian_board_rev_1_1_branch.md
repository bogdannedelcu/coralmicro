---
name: project_lucian_board_rev_1_1_branch
description: "Branch lucian/board-rev-1.1 (SentAI-free port of the rev-1.1 hardware support for the camera colleague) — where it lives, how it was tested, what is still open (2026-11-09; FatFs verified, card re-formatted + restored)"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-09T07:23:30.738Z
---

OP-S10-W22-T11 (2026-11-09): the rev-1.1 (OWL-R2) hardware support was delivered for the camera colleague Lucian on branch `lucian/board-rev-1.1` (pushed to origin), started from his last commit `6448b25d` (2026-03-23, no SentAI runtime). Worktree: `/home/bogdan/work/coralmicro-lucian` (Make generator, toolchain symlinked from the main checkout, SDK patches 0001+0007 merged into one file and 0005 with a weak `coralmicro_csi_on_queue_fault` default). His 2026 work (accel, audio, PMIC, SD, I2C1) was already merged in our history; the branch on the remote under his name is from 2025.

Decisions (operator): base = his last commit here, keep his FatFs `Mount/Unmount/Format` API (now on drive "2:", FF_VOLUMES 3), branch name `lucian/board-rev-1.1`, camera SDK patches included, no `sentai_*` identifiers (only provenance notes in `docs/board_rev_1_1/`).

Testing recipe (EXP-s272): build in the worktree, load with the MAIN repo's `venv-flash/bin/python scripts/flashtool.py --build_dir /home/bogdan/work/coralmicro-lucian/build --example <x> --ram` (58eb keeps SentAI in NAND; `blhost -u 0x15a2,0x0073 -- reset` after loading the flashloader boots it back). Vendor examples enumerate as **18d1:9308** — udev rule added to `scripts/99-coral-micro.rules` but must be installed with sudo (auto-mode denied it); until then `sudo chmod 666 /dev/ttyACM0`. The vendor CDC console drops output with no host attached and **pyserial loses lines** — capture with raw-mode `cat` (`experiments/s272_.../capture_console.py`). To leave a RAM-loaded vendor example: open the port at 1200 baud with DTR low → ROM SDP.

FatFs path verified too (operator accepted the format; card backed up with `sentai.usb.drive(1)` + udisks + rsync to `/home/bogdan/work/sdcard_backups/58eb_20261109/files/` and restored — guard the mountpoint, an empty one aimed rsync at host `/`). FileX card = FAT32 without the `BS_FilSysType32` string → FatFs FR_NO_FILESYSTEM; FatFs `f_mkfs` makes an MBR partition that FileX mounts fine, so 58eb's card is now FatFs-formatted with the old content restored. Open: camera 2 image path (USER button in `camera_streaming_http`); the LIS2DUX12 answered at 0x19 here but 0x18 on the 2026-10-31 cold power-up — both addresses are probed.

**Why:** a second developer works on this hardware without the SentAI stack; the port is a hand port (cherry-pick drags ~40 `sentai_*` symbols), so future rev-1.1 fixes must be applied twice.
**How to apply:** when touching rev-1.1 hardware code (SD/USDHC1, LPI2C1 camera path, LIS2DUX12, T5838 pins, boot fuses), mirror the change onto `lucian/board-rev-1.1` with neutral names. Related: [[project_new_board_rev_2026_10_open_items]], [[project_sdcard_usdhc1_bringup]].
