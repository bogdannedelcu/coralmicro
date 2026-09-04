---
name: reflash-corrupts-filex-littlefs-mismatch
description: "CORRECTED — flashtool does NOT corrupt FileX; the SAFE_MODE seen all session was wedge-reset corruption, not the flash"
metadata: 
  node_type: memory
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

**CORRECTION (2026-07-02, verified empirically).** Earlier this session I claimed
`flashtool.py` corrupts the FileX user partition (elfloader LittleFS-formats where
FileX lives). **That was WRONG — the flash does NOT touch FileX.** Verified: synced
the volume, flashed build #1577 with the board idle, and RIGHT AFTER the flash
`fs.exists("/")==True` and the model (`c2f_thick.tflite`, 942848 B) was still
present + intact. FileX survives a flash cleanly.

Why the earlier diagnosis was wrong — the NAND partition layout (block units):
- Boot images: blocks 4–12 (always erased by the SB file).
- **LittleFS SYSTEM: blocks 12–76** (`kFilesystemBaseBlock=12`, `kSystemBlockCount=64`,
  filesystem.cc). The elfloader links this SAME filesystem.cc, so its
  `LfsInit(force_format=true)` formats ONLY blocks 12–76.
- **FileX USER: blocks 76–524** (`FX_NAND_USER_BASE_BLOCK=76`, count 448,
  fx_nand_driver.h). NOT erased by the SB file (`erase spinand` covers only 4–12
  and optionally 12–76), NOT touched by the elfloader LittleFS format.

So neither the erase nor the LittleFS format reaches the FileX NAND blocks. The
real cause of the repeated SAFE_MODE this session: **involuntary WEDGE-RESETS**
(the board watchdog-resetting mid-FileX-op, so the volume was never synced →
corruption). The mount then SAFE_MODEs by design (fx_user_fs.cc only auto-formats
on virgin-NAND signatures LX_SYSTEM_INVALID_FORMAT/LX_NO_PAGES; any other error is
treated as potential real-data corruption → SAFE_MODE, never silent format). I was
re-flashing an already-corrupt (wedged) board and blaming the flash.

**Takeaways:**
- Flashing is SAFE for FileX. Just `sentai.fs.sync()` before flashing (or ensure the
  board is idle) and the model + user data persist across the flash. No reformat
  needed if the volume was healthy.
- FileX corruption in this repo comes from involuntary resets (wedges), not the
  toolchain. With stable detection ([[tpu-usb-wedge-mechanism]] correction: full-rate
  detection is stable on a healthy board), wedge-resets don't happen, so FileX stays
  intact.
- Dev recovery when a wedge DID corrupt it: `sentai.fs.format()` + MSC re-upload
  (see [[fx-reformat-dev]]).

Possible future hardening (low priority now that detection is stable): FileX/LevelX
journaling or auto-sync so an involuntary reset can't corrupt the volume.
