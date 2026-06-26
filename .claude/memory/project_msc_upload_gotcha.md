---
name: MSC large-file upload corrupts FileX if you create subdirs or exit with 'q'
description: How to upload models to the board via USB MSC without corrupting the FileX user partition (cost a reformat on 2026-06-26)
metadata:
  type: project
---
Uploading `.tflite` models (≥10 KB) to the board MUST go via USB MSC, not the
chunked REPL uploader. Two mistakes corrupted the FileX volume into SAFE_MODE
(`sentai.fs.exists('/')` -> False, even root `/` unreadable) on 2026-06-26:

1. **Do NOT create a new subdirectory** (e.g. `/models/`) on the FAT volume
   from the host. The board's FileX chokes on a host-created subdir entry at
   next mount. **Put model files at ROOT** (like the legacy `yolo_1_512`).
2. **Do NOT exit MSC with `printf 'q'`** then poke around — on build 1546 'q'
   leaves the board in "flash busy" and does NOT cleanly hand flash back to
   FileX. **Exit with `sentai.usb.drive(0)`** (it re-enumerates USB; reconnect
   after). The agent.md §3.1.1 'q' recipe is stale for this build.

**Clean MSC upload recipe (verified working):**
```
# enter MSC (serial drops - expected)
sentai.usb.drive(1)
# board volume = /dev/sdc, by-id usb-SENTAI_FLASH_STORAGE_* (NOT host sda/sdb!)
# udisks auto-mounts it RO at /run/media/$USER/0000-0001 -> unmount that first
udisksctl unmount -b /dev/sdc          # or sudo umount
sudo mount -o rw /dev/sdc /mnt/sentai_user
sudo cp *.tflite /mnt/sentai_user/     # ROOT, no subdir
sync && sudo umount /mnt/sentai_user
# exit MSC cleanly (NOT 'q'):
sentai.usb.drive(0)                     # re-enumerates; reconnect REPL
# verify: sentai.fs.ls('/') shows the models
```

ALWAYS identify the board volume by its `SENTAI_FLASH_STORAGE` by-id label -
on this host `/dev/sda` (894 GB) and `/dev/sdb` (931 GB) are the REAL system
disks; the board is `/dev/sdc` (~55 MB).

Recovery if corrupted (dev only, operator policy = data loss OK): the format
binding on build 1546 is **`sentai.fs.format()`** (returns True), NOT the
`sentai.diag.fx_format(0xDEADBEEF)` from the stale fx-reformat-dev skill.
Wipes user partition only (system/firmware untouched). See [[reference_host_coral_testing]].
