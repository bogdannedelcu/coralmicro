---
name: flashtool persistent works without manual SDP if board alive
description: When the SentAI board's USB CDC is up (REPL accessible), flashtool.py -e sentai_runtime auto-resets it into SDP — no need to ask the user to press buttons
type: feedback
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
When the SentAI board is alive on USB (`/dev/ttyACM*` enumerated as `1fc9:c0a1`), `python3 scripts/flashtool.py -e sentai_runtime` (persistent flash, no `--ram`) **auto-resets the board into SDP via HID** before flashing. The flow visible in flashtool's stdout:

```
STATE_CHECK_FOR_CORAL_MICRO       <- detects board alive
STATE_RESET_TO_SDP                <- HID reset command
STATE_CHECK_FOR_SDP
STATE_LOAD_FLASHLOADER
STATE_PROGRAM
STATE_PROGRAM_DATA_FILES
STATE_RESET_TO_FLASH
"Flashing to flash storage complete, the device is restarting"
```

**Don't ask the user to "put the board in SDP" by pressing USER+RESET unless the board is actually bricked** (no `1fc9:c0a1`, no `1fc9:013d`, no `/dev/ttyACM*`). The user prefers zero-intervention flashing whenever possible.

**Why:** Build/test cycles get faster. The user explicitly stated this preference 2026-05-10 mid-session — preference verified by them as the right call (build 1226 deploy after pas 2 fix).

**How to apply:**

| Board state                              | Recipe                                                      |
|------------------------------------------|-------------------------------------------------------------|
| `lsusb` shows `1fc9:c0a1` (runtime alive) | `python3 scripts/flashtool.py -e sentai_runtime` — auto-SDP |
| `lsusb` shows `1fc9:013d` (already SDP)   | Same command, just skips the auto-reset step                |
| `lsusb` shows nothing or `18d1:9307`      | Ask user for physical USER+RESET (board is truly stuck)     |

`--ram` flash is for ephemeral RAM-only loads (lost on `sys.reset()` because NVIC reset returns to ROM bootloader → persistent flash). Use it only for unwedge probes within a single boot cycle, never as the default.
