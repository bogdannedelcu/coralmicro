---
name: fx-reformat-dev
description: Recover from a corrupted FileX volume during development — reformat the user partition and re-upload files via USB MSC. Use when sentai.fs.exists("/") returns False (SAFE_MODE) or writes are failing silently.
---

# /fx-reformat-dev

**Operator policy (2026-05-21)**: during development, **data loss on the user partition is OK**.  If the FileX volume is corrupted or in SAFE_MODE, reformat without trying to recover.  Re-upload files via USB MSC.  Field policy is different — see §Field rules at the end.

## Detect corruption

```python
import sentai
sentai.fs.exists("/")
#  False  →  SAFE_MODE (g_mounted == false)
#  True   →  MOUNTED, healthy
sentai.diag.dmesg()[-2000:]    # tail of boot log
# Look for: "*** MOUNT FAIL (lx=N, fx=N) -- SAFE MODE ***"
#          or  SERR_LFX_MOUNT_FAIL_SAFE (0x0D29)
```

Other symptoms:
- `sentai.fs.write(...)` returns 0 / -1 silently.
- `/api/ls` returns `{"error":"lfs_busy"}` repeatedly (≠ corruption, but worth ruling in).
- Boot log shows `SERR_LFX_MOUNT_RETRY_OK` (recovered) — NOT corruption; ignore.
- Boot log shows `SERR_LFX_FIRST_BOOT_FORMAT` — virgin NAND, already formatted.

## Reformat (DEV-ONLY)

```python
sentai.diag.fx_format(0xDEADBEEF)   # magic constant — refuses anything else
```

This wipes EVERYTHING on the user partition: models, calib JSON, diag sessions, gallery, persistent state.  Then re-mounts a fresh empty volume.  After this returns, `sentai.fs.exists("/")` should be True.

**System partition is separate** (LittleFS, agent.md §11 "Filesystem layout") and NOT touched by `fx_format`.

## Re-upload files via USB MSC

Once the volume is mounted fresh, switch the board into Mass-Storage mode to drag-drop files from the host.  Faster than the chunked REPL uploader for ≥100 KB files (e.g. `.tflite` models).

### From REPL

```python
sentai.usb.drive(1)       # mode-switch to USB MSC
# board re-enumerates; /dev/sda appears on host
```

### From host (after MSC mode)

```bash
# Wait for /dev/sda to appear (auto-detect, up to 15 s)
for i in $(seq 1 15); do [ -b /dev/sda ] && break; sleep 1; done

# Mount and copy
sudo mkdir -p /mnt/sentai_user
sudo mount /dev/sda /mnt/sentai_user
sudo cp ~/path/to/model.tflite /mnt/sentai_user/
sudo cp ~/path/to/cam_calib.json /mnt/sentai_user/system/
sudo umount /mnt/sentai_user

# Tell the board to exit MSC and warm-reboot to default mode
printf 'q\r\n' > /dev/ttyACM0

# Wait for REPL to come back
for i in $(seq 1 20); do
  python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200,timeout=0.5); s.write(b'\r\n'); print(s.read(100))" 2>/dev/null | grep -q '>>>' && break
  sleep 1
done
```

### For driver scripts (<100 KB)

The chunked REPL uploader is better than MSC:
```bash
python3 examples/sentai_runtime/diag/_host_upload_repl.py --file path/to/_t_driver.py
```
Auto-syncs at end; no mode switching.

## Mutual exclusion (load-bearing)

MSC mode and FileX are MUTUALLY EXCLUSIVE at runtime.  In `sentai.usb.drive(1)` mode:
- FileX is unmounted.  REPL `sentai.fs.*` returns errors.
- Host sees `/dev/sda`; reads/writes go directly to NAND via USB-MSC.

Exiting MSC (send `q` on `/dev/ttyACM0`) triggers warm reset back to default mode.  Files written from host become visible from REPL after exit, and vice versa.

## Field rules (NOT dev)

In production / field deploy, `fx_format(0xDEADBEEF)` IS a real data-loss event.  Decision rule from `agent.md` §12:
- **Data worth recovering**: connect JTAG, dump NAND, attempt offline LX recovery.
- **Data NOT worth recovering**: `fx_format(0xDEADBEEF)`.

The board NEVER decides for you — that's the contract.  In dev, the operator decides "wipe it"; in field, the operator decides via human-in-the-loop.

## Reject patterns

- Auto-formatting on mount failure without explicit operator action (would silently destroy user data in field).
- Skipping the MSC `q` exit and just power-cycling (FileX won't sync MSC writes; data loss).
- Trying to use `sentai.fs.*` while in MSC mode.

## See also

- `agent.md` §12 "Constraints" + "Mount states MOUNTED vs SAFE MODE"
- `agent.md` §4 "How-to: survive a stuck REPL" (the `sentai.usb.drive(1)` recipe origin)
- `[[fx-sync-protocol]]` skill — sync rules to AVOID needing this reformat
