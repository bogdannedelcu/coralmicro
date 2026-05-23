---
name: fx-sync-protocol
description: FileX / LevelX sync rules for power-fail durability. Use when committing important data to user partition before a power event, or when reviewing code that writes to /system or /diags.
---

# /fx-sync-protocol

The user partition has THREE cache layers between API and NAND.  `fx_file_close` flushes only one.  `sentai.fs.sync()` (build #1223+) flushes all three.

```
N1  FileX logical-sector cache  (16 KB SDRAM, g_fx_media_memory)
N2  FileX FAT/dir cache
N3  LevelX log + wear-level mapping table  (32 KB SDRAM, g_lx_memory_buffer)
```

Pre-#1223, sync flushed N1+N2 but NOT N3 — `sync()` returned True while LX log-page indirection sat in SDRAM, and a power-cycle left the FAT pointing at unprogrammed NAND pages.  Build #1223+: `FxUserSync` does `fx_media_flush` + `_lx_nand_flash_close` + `lx_nand_flash_open` so all 3 hit NAND.  Cost: 200–500 ms per call.

## Durability rules

| Event | Survives without sync? | Why |
|---|---|---|
| `sentai.sys.reset()` (NVIC) | ✅ YES | SDRAM preserved across NVIC reset; next boot's mount reads back cached pages |
| Soft reboot (`flashtool.py`) | ✅ YES | Same — SDRAM preserved |
| WDOG1 timeout | ✅ YES | Same |
| Power cycle | ❌ NO | SDRAM lost |
| Brownout | ❌ NO | Same |
| Pulling USB on bus-powered board | ❌ NO | Same |

## Recipe — call sync after important writes

```python
sentai.fs.write("/system/cam_calib.json", payload)
sentai.fs.write("/system/flow_gains.txt", "kp=0.39\nkd=0.0\n")
sentai.fs.sync()              # ~300 ms, flushes all 3 cache layers
```

## Recipe — host-side chunked upload auto-syncs

`diag/_host_upload_repl.py` auto-calls `sentai.fs.sync()` after the last chunk.  Ad-hoc REPL writes do NOT.  If you write via raw `sentai.fs.write/append` in your own script, add an explicit sync.

## Reject patterns

- Adding `fx_media_flush` after every small write — Phase 3.2 regression, 41–139× slowdown on 256-byte writes.  One sync at the END of a batch.
- Assuming `sentai.fs.write(...)` returning True means durable.  It means accepted to cache, not on NAND.
- Skipping sync because "we just call sys.reset() anyway".  True today; false the moment a tester unplugs the USB cable.

## Verify

After sync, mount the volume from host (USB MSC) and confirm files are intact:
```python
sentai.usb.drive(1)             # mode-switch to MSC
# wait for /dev/sda on host, browse files; data must be present
```

## See also

- `agent.md` §12 "Constraints" item 4 (sync semantics + 3 cache layers)
- `embeded.md` §6.2 SOUP (FileX/LevelX versioning)
- `embeded.md` §1.4 safe state (sync at safe-state entry where applicable)
