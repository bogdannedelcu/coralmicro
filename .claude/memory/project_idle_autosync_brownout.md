---
name: Idle-driven auto-sync vs USB→battery brownout (build #1226)
description: Phase 3.2 dropped per-write fx_media_flush; #1226 adds idle-driven FxUserMaybeIdleSync from CombinedWatchdogTask to bound unflushed-data window without re-introducing per-write cost
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Build #1226 (2026-05-10) reduces the brownout vulnerability window when the operator switches the board's power source USB→battery (Crazyflie BL deck VBAT ~3.7V vs the board's 5V need can dip into brownout briefly during the handover).

**Trade-off recap:** Phase 3.2 (build #1074) removed per-write `fx_media_flush` from FxUserWriteFile/AppendFile/Remove for 41-139× small-write speed-up. Writes then live ONLY in FileX/LevelX SDRAM caches (~32 KB) until an explicit `FxUserSync()`. If brownout interrupts a NAND program in-flight (LevelX log-page write, FAT update), the page is left ECC-corrupted and the next mount fails with `[nand] READ FAIL page=N` → SAFE MODE. Pre-#1225 SAFE MODE bricked USB CDC (see `project_safe_mode_brick_fix.md`); #1225 fixed that, but the underlying corruption was still routine.

**Fix mechanism:** counter `g_unflushed_writes` + timestamp `g_last_write_ms` updated in `FxUserWriteFile/AppendFile/Remove/Rename/MakeDirs` via `note_write()`. New `FxUserMaybeIdleSync()` called once per 5 s tick from `CombinedWatchdogTask` checks: mounted AND unflushed > 0 AND last-write older than 2 s → calls `FxUserSync()` synchronously (~200-500 ms). Counter reset on successful sync.

**Net effect:** unflushed-data window shrinks from "indefinite until next reset" to ~7 s on an idle board. **Zero impact on hot-path write throughput** because the 2 s idle gate skips the sync while writes are still in flight. Critical writes that demand stronger guarantees still call `sentai.fs.sync()` explicitly per agent.md rule #11.

**Limits this fix does NOT address:**
- Brownout caught EXACTLY mid-NAND-program (microsecond window) is still possible, just much less likely.
- Hot-path bursts > 5 s long with no idle gap will hold writes in cache the whole time.
- LevelX wear-level metadata corruption from a brownout during `_lx_nand_flash_close` itself.

**How to apply:** When designing FS APIs that batch writes for perf, pair the optimisation with an idle-driven flush mechanism — the worst-case unflushed window must be bounded to a few seconds, not "until next explicit sync". The pattern is generic: counter + timestamp + watchdog-task tick.

**Testing protocol** (manual, requires hardware): write a few files via REPL, wait 10 s, physically pull USB and reconnect. Mount should succeed; files should be intact. Without #1226 this would race the brownout window.
