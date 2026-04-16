# SentAI Runtime — Boot Process

## Overview

The SentAI runtime follows a deterministic boot sequence on the Coral Micro
(NXP i.MX RT1176, Cortex-M7).  Every `printf` emitted between power-on and
the MicroPython REPL prompt is captured to `/log/boot.log` on the LittleFS
user partition.

## Boot Sequence

```text
app_main()
 ├─ boot_log_init()            — activate RAM capture (16 KB buffer)
 ├─ logf("SentAI build #…")   — first logged line
 ├─ boot_log_fs_init()         — rotate boot.log → boot_old.log, open new file
 ├─ User-button task created   — ISR → task notification → usb.drive(0)
 ├─ micropython_start_repl_task()
 │    └─ micropython_repl_task()
 │         ├─ mp_embed_init()
 │         ├─ import sentai
 │         ├─ [main.py execution]   ← safe-boot + timeout protected
 │         ├─ sentai_boot_log_stop()— final flush & close boot.log
 │         └─ REPL loop (>>> prompt)
 └─ Main()
      ├─ EdgeTPU open
      └─ vTaskSuspend() — parks forever
```

## Boot Logging (`/log/boot.log`)

| Aspect | Detail |
|--------|--------|
| **Mechanism** | Global `_write()` override captures all `stdout`/`stderr` |
| **Buffer** | 16 KB in SDRAM, flushed when nearly full or at REPL start |
| **Rotation** | On each boot: `boot.log` → `boot_old.log` (keeps last 2 boots) |
| **Scope** | From `boot_log_init()` until `sentai_boot_log_stop()` |
| **Includes** | Driver messages, TPU init, `main.py` output (`print()`), errors |

### What gets logged

- Build version and timestamp
- Filesystem and TPU initialization status
- `main.py` execution output (any `print()` calls)
- `main.py` result: `Finished OK` or `FAILED`
- All C-level `printf` from any FreeRTOS task during boot

### What does NOT get logged

- REPL session input/output (boot log is closed before `>>>`)
- Output after `sentai_boot_log_stop()` is called

## Safe Boot Protection

Prevents boot loops caused by a crashing `main.py`.

1. Before running `main.py`, a counter is written to `/log/.boot_pending`.
2. If `main.py` completes (even with a Python exception), the counter is cleared.
3. If a hard fault causes a reboot, the counter persists and increments.
4. After **3 consecutive hard crashes**, `main.py` is skipped (safe mode).
5. A 30-second timeout also interrupts `main.py` via `KeyboardInterrupt`.

### Recovery from safe mode

```python
>>> sentai.fs.remove('/log/.boot_pending')   # re-enable main.py
>>> sentai.fs.remove('/main.py')             # or delete the bad script
```

## Key Source Files

| File | Role |
|------|------|
| `sentai_runtime.cc` | `app_main`, boot log system, `_write()` override |
| `micropython_task.c` | REPL task, `main.py` execution, safe boot logic |
| `build_version.h` | `BUILD_VERSION`, `BUILD_TIMESTAMP` macros |
