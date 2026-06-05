---
name: LFS Task Architecture Fix
description: Root cause and fixes for random USB NCM hangs and HTTP GET retry protocol overhead in SentAI firmware
type: project
originSessionId: 93909a81-a37a-4af3-92c6-afc40ae13899
---
Dedicated LFS task added to eliminate USB NCM stalls + fast-path fix so HTTP GET succeeds on the first attempt.

**Original hang bug (fixed build #510+):** `tcpip_thread` (FreeRTOS priority 4, highest) called `lfs_dir_open()` / `lfs_file_read()` directly. Those wait on the LFS-internal mutex with `portMAX_DELAY`. When MP was mid-flash-write (~700ms), `tcpip_thread` blocked → no TCP ACKs → USB NCM "transmit queue timed out" → board appeared hung.

**Task-based async fix (build #510–#583):**
- Dedicated `sentai_lfs_task.cc` / `sentai_lfs_task.h` (priority 2) owns all GET work (ls, raw)
- `tcpip_thread` posts to a queue and returns `{"error":"lfs_busy"}` immediately — never blocks
- Browser retries on lfs_busy after 600ms (`browser.html`)
- `sentai_lfs_lock/unlock` (2000ms timeout) — app-level mutex used by: modsentai_fs (all MP fs_*), `boot_log_flush`, `crash_log_write`, `sentai_get_last_crash_log_path`, lfs_task itself
- POST (write/mkdir/rm) stays in tcpip_thread, calls lfs_* directly — relies on LFS internal mutex per call (~700ms worst case, within USB NCM tolerance)

**Fast-path fix (build #585):** Task-based async made *every* GET require ≥2 HTTP round-trips even when the filesystem was idle — pure protocol overhead. New `sentai_lfs_try_serve`:
1. If slot already holds the result for this request → serve
2. Take `s_lfs_mutex` with 500ms timeout.  If acquired, run DoLs/DoRaw **inline in tcpip_thread** and serve on this same HTTP response — 1 round-trip, no `lfs_busy`.  Once we hold `s_lfs_mutex`, no other owner is in `lfs_*`, so the internal mutex is free and we can't stall on flash I/O.
3. Fall back to `lfs_task` async only if the mutex is still contended after 500ms (true burst-of-writes case).

500ms stall on tcpip_thread is well below the USB NCM watchdog (~5s). Measured: GET `/api/ls/` drops from 2 attempts × 600ms ≈ 1200ms to a single 230–380ms request; during 8 back-to-back MP flash writes, 30/30 concurrent HTTP GETs still first-try OK (latency 380–524ms).

**Sources of `s_lfs_mutex` contention — none are periodic:**
- `boot_log_flush_to_file` — only fires while `g_boot_log_active`, which is cleared by `boot_log_stop()` called from `micropython_task.c:466` just before the REPL starts. After boot, the RAM log is no longer drained to LFS, so no periodic write.
- `crash_log_write` — only on faults / `HTTP_HANG`.
- `sentai_get_last_crash_log_path` — read-only, on-demand (diag).
- MicroPython fs ops — only when a REPL/user script runs `sentai.fs.*`.
- POST handlers (write/mkdir/rm) — do NOT take `s_lfs_mutex`, rely on the LFS-internal mutex alone. A concurrent fast-path GET + POST can theoretically race on the internal mutex, but the stall is bounded (~700ms worst case).
There is NO timer, NO health task, NO background logger that touches LFS after boot. `lfs_task` itself parks on `xQueueReceive`. So on an idle board the fast-path acquires `s_lfs_mutex` instantly and the observed latency (230-900ms depending on directory size) is pure LittleFS metadata-walk time on NAND+FlexSPI, not contention. The task-based async design originally added the ≥2-attempt protocol overhead even when LFS was idle — that was the real bug the fast path eliminates.

**Earlier lwIP bug also fixed (build #584):** lwIP's `httpd_default_filenames` (`third_party/nxp/rt1176-sdk/middleware/lwip/src/apps/http/httpd.c:154`) rewrites trailing-slash URIs by appending `index.shtml` / `ssi` / `shtm` / `html` / `htm`. So `GET /api/ls/` arrived as `/api/ls/index.shtml` → `DoLs("/index.shtml")` → LFS_ERR_NOENT → `[]` instead of the root listing. Fix in `sentai_httpd.cc` strips any of the five suffixes from the ls path before dispatch.

**How to apply:** If random hangs recur, check (1) new `lfs_*` calls added in tcpip_thread context without going through `sentai_lfs_try_serve`, (2) new tasks using `lfs_*` without `sentai_lfs_lock`, (3) any call to `sentai_lfs_lock` that uses `portMAX_DELAY`. If HTTP GET starts returning `lfs_busy` for every request again, suspect the fast-path's 500ms timeout being consistently exceeded — likely means someone is holding `s_lfs_mutex` for too long (find the long-running lfs caller rather than extending the timeout).
