---
name: Storage-mode watchdog reset bug fix
description: Network watchdog killed board ~2m30s after entering sentai.usb.drive(1); fixed in build #587 by skipping dead/warn logic while storage-mode active.
type: project
originSessionId: 45aa0c5d-06e1-46bc-b44e-2efd77a32986
---
In builds ≤ #586 the board silently re-enumerated ~2m30s after `sentai.usb.drive(1)`, interrupting active USB MSC sessions. Not a crash — the network watchdog fired because:

- `start_network_watchdog()` is called on entry to storage mode (`sentai_runtime.cc:997`).
- `CombinedWatchdogTask` monitors `g_http_last_activity` and `g_repl_last_activity`.
- In storage mode both are off — nothing updates either timestamp.
- Thresholds: `kWarningThresholdMs=60000`, `kDeadThresholdMs=120000`. At 60s idle → WARN (E:0501), at 120s → DEAD (E:05F0), stop kicking WDOG1. WDOG1 fires 30s later + 35s NVIC fallback ⇒ total ~150s ≈ 2m30s.

**Fix (build #587):** `CombinedWatchdogTask` now checks `sentai_storage_mode_active()` at the top of its loop; in storage mode it just refreshes WDOG1 (still catches CPU lockup) and skips all dead/warn logic. User exits storage mode themselves via `drive(0)`, `'q'` byte on `/dev/ttyACM0`, user button, or hardware reset.

**Verified:** 4m1s sustained storage-mode session with zero re-enumeration, /dev/sda stable throughout. Before fix: always reset at 150±5 s.

**How to apply:** If similar "board restarts after N minutes in mode X" reports surface, audit every long-lived mode (recovery mode, future modes) to ensure either the activity timestamps get refreshed, or the watchdog's dead-logic is bypassed with a similar early-continue guard. WDOG1 must keep being refreshed — dropping kicks entirely gives up CPU-lockup protection.
