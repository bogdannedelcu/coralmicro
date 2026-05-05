// sentai_fs_task.cc — Dedicated FS task for thread-safe filesystem GET access.
// (Phase 2: user partition is FileX/LevelX.  Helpers were renamed from
//  sentai_lfs_* to sentai_fs_* to remove "LFS" confusion.  System partition
//  is still LittleFS but is not served by this task.)
//
// Root cause of random board hangs (pre-fix):
//   tcpip_thread (priority 4, highest) called lfs_dir_open() / lfs_file_read()
//   directly. Those functions internally acquire g_lfs_user_mutex with
//   portMAX_DELAY, so when MicroPython was mid-flash-write (~700ms),
//   tcpip_thread blocked → no TCP ACKs → USB NCM "transmit queue timed out".
//
// Fix:
//   All GET operations (ls, raw) are processed by lfs_task (priority 2).
//   tcpip_thread NEVER calls lfs_*() for reads — it just checks slot state
//   and returns lfs_busy if data is not ready. The browser retries after 600ms.
//
//   POST operations (write, mkdir, rm) still run in tcpip_thread. Each
//   individual lfs_* call serializes via internal g_lfs_user_mutex; worst-case
//   lfs_file_close stall (~700ms) is within the USB NCM watchdog tolerance (~5s)
//   and POST is user-initiated / infrequent.

#include "sentai_fs_task.h"

#include "libs/base/filesystem.h"
#include "libs/base/fx_user_fs.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/queue.h"
#include "third_party/freertos_kernel/include/semphr.h"

#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define LFS_TASK_PRIORITY  2    // above mp_repl(1), below hw_wdog(3)
#define LFS_TASK_STACK     512  // words = 2KB

// Queue depth 4 (Phase 3.4): under concurrent REPL writes + HTTP /api/ls
// burst, depth=1 saturates immediately and causes ~50% lfs_busy responses.
// The lfs_task drains in FIFO order; each entry is small (~300 B) so 4
// pending requests cost ~1.2 KB queue memory.  Beyond 4 the wait becomes
// long enough that the browser-side 600 ms retry kicks in anyway.
#define LFS_QUEUE_DEPTH    4

// ---------------------------------------------------------------------------
// Response buffer (SDRAM, 256 KB)
// ---------------------------------------------------------------------------
static constexpr size_t kRespBufSize = 256 * 1024;
static uint8_t g_resp_buf[kRespBufSize] __attribute__((section(".sdram_bss")));

// ---------------------------------------------------------------------------
// Application-level LFS mutex (priority-inheritance aware)
// Shared with: crash_log_write, boot_log_flush, modsentai_fs, lfs_task itself.
// ---------------------------------------------------------------------------
static SemaphoreHandle_t s_lfs_mutex = nullptr;

// ---------------------------------------------------------------------------
// Request type and queue
// ---------------------------------------------------------------------------
typedef struct {
    sentai_fs_req_type_t type;
    char path[256];
} LfsRequest;

static QueueHandle_t s_req_queue = nullptr;

// ---------------------------------------------------------------------------
// Slot state — tracks where the current GET request is in the pipeline.
//
// Written by tcpip_thread (FsOpenCustom/FsCloseCustom via sentai_fs_try_serve
// and sentai_fs_resp_done) and by lfs_task (when result is ready).
//
// On single-core Cortex-M7, 32-bit aligned enum stores are atomic. We use
// __DMB() before SLOT_READY to ensure s_slot_len is visible to all observers.
// ---------------------------------------------------------------------------
typedef enum {
    SLOT_IDLE    = 0,  // no request in flight; buf may be overwritten
    SLOT_PENDING = 1,  // lfs_task is processing; do not touch buf
    SLOT_READY   = 2,  // buf contains result; serve on next matching request
    SLOT_SERVING = 3,  // lwIP is streaming from buf; block new requests
} SlotState;

static volatile SlotState          s_slot_state = SLOT_IDLE;
static volatile size_t             s_slot_len   = 0;
static char                        s_slot_path[256] = {};
static sentai_fs_req_type_t       s_slot_type  = FS_REQ_LS;

// Diagnostic counters — tell us EXACTLY why `lfs_busy` was returned on
// each failing request.  Exposed via sentai.diag.lfs_stats() so a host
// script can do (GET /api/ls → lfs_busy → REPL lfs_stats) to know which
// branch of the state machine stalled.  NASA/JPL §I — every failure
// mode must be observable.
typedef struct {
    volatile uint32_t served_ready_cached;   // Step 1: cached READY + path match
    volatile uint32_t served_fast_raw;       // Step 3: RAW fast path via mutex
    volatile uint32_t served_after_wait;     // Step 4: PENDING → READY within wait
    volatile uint32_t busy_serving_prev;     // Step 2: previous response still streaming
    volatile uint32_t busy_fast_raw_mutex;   // Step 3: mutex contended, RAW fallback to slow
    volatile uint32_t busy_slow_timeout;     // Step 4: wait timed out (lfs_task still working)
    volatile uint32_t busy_slow_state_drift; // Step 4: state left PENDING (queue full? task died?)
    volatile uint32_t busy_path_mismatch;    // Step 4: READY for a DIFFERENT path than ours
    volatile uint32_t busy_not_inited;       // no queue yet (pre-boot)
    volatile uint32_t enqueue_ok;            // xQueueSend succeeded
    volatile uint32_t enqueue_fail;          // xQueueSend failed (queue full)
} lfs_stats_t;

static lfs_stats_t s_lfs_stats = {};

// ---------------------------------------------------------------------------
// JSON helpers — used only by DoLs (lfs_task context)
// ---------------------------------------------------------------------------
static void BufAppend(size_t* pos, const char* s) {
    size_t len = strlen(s);
    size_t space = kRespBufSize - *pos;
    if (len > space) len = space;
    memcpy(g_resp_buf + *pos, s, len);
    *pos += len;
}

static void BufAppendJsonStr(size_t* pos, const char* s) {
    if (*pos >= kRespBufSize) return;
    g_resp_buf[(*pos)++] = '"';
    for (; *s && *pos < kRespBufSize - 1; ++s) {
        if (*s == '"' || *s == '\\') g_resp_buf[(*pos)++] = '\\';
        g_resp_buf[(*pos)++] = (uint8_t)*s;
    }
    if (*pos < kRespBufSize) g_resp_buf[(*pos)++] = '"';
}

// ---------------------------------------------------------------------------
// DoLs / DoRaw — called ONLY from lfs_task (safe to block on FS ops)
// ---------------------------------------------------------------------------
struct LsCallbackCtx {
    size_t pos;
    bool first;
};

static int ls_dir_cb(const FxDirEntry* e, void* user) {
    LsCallbackCtx* ctx = static_cast<LsCallbackCtx*>(user);
    if (!ctx->first && ctx->pos < kRespBufSize) g_resp_buf[ctx->pos++] = ',';
    ctx->first = false;
    if (ctx->pos < kRespBufSize) g_resp_buf[ctx->pos++] = '{';
    BufAppend(&ctx->pos, "\"name\":");
    BufAppendJsonStr(&ctx->pos, e->name);
    char tmp[64];
    snprintf(tmp, sizeof(tmp), ",\"type\":\"%s\",\"size\":%lu",
             e->is_dir ? "dir" : "file",
             (unsigned long)e->size);
    BufAppend(&ctx->pos, tmp);
    if (ctx->pos < kRespBufSize) g_resp_buf[ctx->pos++] = '}';
    return 0;
}

static size_t DoLs(const char* path) {
    if (!FxUserIsMounted()) { g_resp_buf[0] = '['; g_resp_buf[1] = ']'; return 2; }
    LsCallbackCtx ctx = { 0, true };
    g_resp_buf[ctx.pos++] = '[';
    int rc = FxUserListDir(path, ls_dir_cb, &ctx);
    if (rc < 0) {
        g_resp_buf[0] = '['; g_resp_buf[1] = ']'; return 2;
    }
    if (ctx.pos < kRespBufSize) g_resp_buf[ctx.pos++] = ']';
    return ctx.pos;
}

static size_t DoRaw(const char* path) {
    if (!FxUserIsMounted()) return 0;
    ssize_t sz = FxUserSize(path);
    if (sz <= 0) return 0;
    size_t to_read = ((size_t)sz < kRespBufSize) ? (size_t)sz : kRespBufSize;
    return FxUserReadFile(path, g_resp_buf, to_read);
}

// ---------------------------------------------------------------------------
// LFS task — owns all blocking lfs_* GET operations
// ---------------------------------------------------------------------------
static void LfsTaskFn(void* /*arg*/) {
    LfsRequest req;
    for (;;) {
        // Block until a request arrives (yields CPU — correct FreeRTOS pattern)
        xQueueReceive(s_req_queue, &req, portMAX_DELAY);

        // Acquire application-level mutex.
        // Blocking is safe here: this is NOT tcpip_thread.
        // Priority inheritance boosts any lower-priority holder (e.g. mp_repl)
        // to our priority (2) while we wait, keeping the system responsive.
        if (s_lfs_mutex) xSemaphoreTake(s_lfs_mutex, portMAX_DELAY);

        size_t len = 0;
        if (req.type == FS_REQ_LS) {
            len = DoLs(req.path);
        } else if (req.type == FS_REQ_RAW) {
            len = DoRaw(req.path);
        }

        if (s_lfs_mutex) xSemaphoreGive(s_lfs_mutex);

        // Publish result. Memory barrier ensures s_slot_len is visible before SLOT_READY.
        s_slot_len = len;
        __asm volatile("dmb 0xF" : : : "memory");
        s_slot_state = SLOT_READY;
    }
}

// ---------------------------------------------------------------------------
// Public API — called from sentai_httpd.cc (tcpip_thread context)
// ---------------------------------------------------------------------------

static size_t EnqueueLfsRequest(sentai_fs_req_type_t type, const char* path) {
    LfsRequest req;
    req.type = type;
    strncpy(req.path, path, sizeof(req.path) - 1);
    req.path[sizeof(req.path) - 1] = '\0';

    // Only publish the slot's path/type + state **after** xQueueSend
    // accepts the request.  Pre-publishing on a failed send used to
    // leave the invariant broken (path pointed to the new URI, state
    // still IDLE), and every retry re-entered this path, failed to
    // enqueue (queue already full with a prior in-flight request),
    // and returned lfs_busy forever.  NASA/JPL §E — atomic publish-on-
    // success.
    if (xQueueSend(s_req_queue, &req, 0) != pdTRUE) {
        s_lfs_stats.enqueue_fail++;
        return 0;
    }
    s_slot_type = type;
    strncpy(s_slot_path, path, sizeof(s_slot_path) - 1);
    s_slot_path[sizeof(s_slot_path) - 1] = '\0';
    s_slot_state = SLOT_PENDING;
    s_lfs_stats.enqueue_ok++;
    return 0;
}

size_t sentai_fs_try_serve(sentai_fs_req_type_t type, const char* path) {
    if (!s_req_queue) { s_lfs_stats.busy_not_inited++; return 0; }

    // Step 1 — if the slot already holds the result for THIS exact request
    // (either from an earlier async attempt or a concurrent burst), serve it.
    if (s_slot_state == SLOT_READY) {
        if (s_slot_type == type && strcmp(s_slot_path, path) == 0) {
            size_t len = s_slot_len;
            s_slot_state = SLOT_IDLE;
            if (len > 0) {
                s_slot_state = SLOT_SERVING;
                s_lfs_stats.served_ready_cached++;
                return len;
            }
            return (size_t)-1;  // 404 / empty
        }
        // Stale result for a different path — discard it.
        s_slot_state = SLOT_IDLE;
    }

    // Step 2 — a previous response is still being streamed from g_resp_buf.
    // Reusing the buffer now would corrupt it; tell the client to retry.
    if (s_slot_state == SLOT_SERVING) {
        s_lfs_stats.busy_serving_prev++;
        return 0;
    }

    // Step 3 — FAST PATH (RAW only).  Take the LFS mutex and service the
    // request inline from tcpip_thread so a single GET resolves in one HTTP
    // round-trip.  RAW reads are bounded by kRespBufSize (256 KB) and have a
    // predictable cost on LittleFS (sequential block reads from NAND); safe
    // for tcpip_thread context.
    //
    // LS requests are INTENTIONALLY excluded from the fast path: dir walks
    // on LittleFS have pathological worst cases that were seen to block
    // tcpip_thread well past the USB NCM transmit timeout (observed: root
    // ls hanging > 30 s on an aged filesystem, causing the network
    // watchdog to fire at the 2-min idle threshold → hard reset loop).
    // The "1 round-trip" saving on ls is not worth risking a whole-device
    // reset — per embeded.md, every tcpip_thread-executed path must be
    // strictly bounded.  LS therefore ALWAYS goes through lfs_task (slow
    // path) below; the browser retries on `lfs_busy` after 600 ms and the
    // subsequent request hits the SLOT_READY cache.
    if (type == FS_REQ_RAW) {
        constexpr TickType_t FAST_PATH_WAIT_MS = 500;
        if (s_lfs_mutex &&
            xSemaphoreTake(s_lfs_mutex, pdMS_TO_TICKS(FAST_PATH_WAIT_MS)) == pdTRUE) {
            size_t len = DoRaw(path);
            xSemaphoreGive(s_lfs_mutex);
            if (len > 0) {
                s_slot_state = SLOT_SERVING;
                s_lfs_stats.served_fast_raw++;
                return len;
            }
            return (size_t)-1;  // 404 / empty
        }
        s_lfs_stats.busy_fast_raw_mutex++;
    }

    // Step 4 — SLOW PATH.  Either this is an LS request (always deferred)
    // or the fast-path mutex was contended.  Hand the request to lfs_task,
    // then block tcpip_thread for up to kSlowPathWaitMs waiting for the
    // task to publish SLOT_READY.  Historically this path returned 0
    // immediately (→ `{"error":"lfs_busy"}` body) and relied on the
    // browser to re-issue the GET after 600 ms — which was fine for the
    // interactive file-browser but terrible for curl and host automation
    // that don't retry.  The bounded wait collapses the two-round-trip
    // dance into one: the lfs_task finishes a typical LS in a few ms,
    // so the common case serves the result on the first request.
    //
    // NASA/JPL §B: the wait is explicitly bounded below the USB-NCM
    // watchdog ceiling (2 min) with plenty of margin; if a pathological
    // LS truly exceeds the budget we still return lfs_busy so the
    // caller can retry rather than hang tcpip_thread.
    /* Phase 3.4: longer slow-path wait reduces lfs_busy responses by
     * giving the dedicated FS task more time to publish SLOT_READY
     * before tcpip_thread gives up.  Old value 500 ms (matched the
     * browser retry budget) caused ~50% busy under contention; 1500 ms
     * stays well within the USB-NCM 2 min watchdog. */
    constexpr TickType_t kSlowPathWaitMs = 1500;
    if (s_slot_state == SLOT_IDLE) {
        EnqueueLfsRequest(type, path);
    }
    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(kSlowPathWaitMs);
    while (s_slot_state == SLOT_PENDING &&
           (int32_t)(deadline - xTaskGetTickCount()) > 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (s_slot_state == SLOT_READY) {
        if (s_slot_type == type && strcmp(s_slot_path, path) == 0) {
            size_t len = s_slot_len;
            s_slot_state = SLOT_IDLE;
            if (len > 0) {
                s_slot_state = SLOT_SERVING;
                s_lfs_stats.served_after_wait++;
                return len;
            }
            return (size_t)-1;  // 404 / empty
        }
        s_lfs_stats.busy_path_mismatch++;
        return 0;
    }
    // Still PENDING after the bounded wait — lfs_task is slow or stuck.
    if (s_slot_state == SLOT_PENDING) {
        s_lfs_stats.busy_slow_timeout++;
    } else {
        s_lfs_stats.busy_slow_state_drift++;
    }
    return 0;
}

// ------------------------------------------------------------------
// Diagnostic snapshot — exposed to MicroPython via sentai.diag.lfs_stats().
// Every "lfs_busy" response increments exactly one of the busy_* counters,
// so after a failing GET the host can query this struct to learn which
// arm of the state machine returned early.
// ------------------------------------------------------------------
extern "C" void sentai_fs_stats_get(uint32_t* out, int max_fields) {
    if (max_fields < 11 || !out) return;
    out[0]  = s_lfs_stats.served_ready_cached;
    out[1]  = s_lfs_stats.served_fast_raw;
    out[2]  = s_lfs_stats.served_after_wait;
    out[3]  = s_lfs_stats.busy_serving_prev;
    out[4]  = s_lfs_stats.busy_fast_raw_mutex;
    out[5]  = s_lfs_stats.busy_slow_timeout;
    out[6]  = s_lfs_stats.busy_slow_state_drift;
    out[7]  = s_lfs_stats.busy_path_mismatch;
    out[8]  = s_lfs_stats.busy_not_inited;
    out[9]  = s_lfs_stats.enqueue_ok;
    out[10] = s_lfs_stats.enqueue_fail;
}

uint8_t* sentai_fs_resp_buf(void) { return g_resp_buf; }

void sentai_fs_resp_done(void) {
    s_slot_state = SLOT_IDLE;
}

// ---------------------------------------------------------------------------
// sentai_fs_lock / sentai_fs_unlock
//
// Used by: crash_log_write, boot_log_flush, sentai_get_last_crash_log_path,
//          modsentai_fs (MP fs ops), lfs_task (already holds mutex).
//
// 2000ms timeout: callers that fail should log an error and skip the operation
// rather than blocking their task indefinitely.
//
// NOT for tcpip_thread POST operations — those call lfs_* directly and rely on
// the internal g_lfs_user_mutex for per-call serialization.
// ---------------------------------------------------------------------------
extern "C" int sentai_fs_lock(void) {
    if (!s_lfs_mutex) return 1;  // Pre-init: single-threaded boot, allow.
    return xSemaphoreTake(s_lfs_mutex, pdMS_TO_TICKS(2000)) == pdTRUE ? 1 : 0;
}

extern "C" void sentai_fs_unlock(void) {
    if (s_lfs_mutex) xSemaphoreGive(s_lfs_mutex);
}

// ---------------------------------------------------------------------------
// sentai_fs_task_start — call once from sentai_runtime before sentai_httpd_start
// ---------------------------------------------------------------------------
extern "C" void sentai_fs_task_start(void) {
    static bool started = false;
    if (started) return;
    started = true;

    s_lfs_mutex = xSemaphoreCreateMutex();
    s_req_queue  = xQueueCreate(LFS_QUEUE_DEPTH, sizeof(LfsRequest));

    xTaskCreate(LfsTaskFn, "lfs_task", LFS_TASK_STACK, nullptr,
                LFS_TASK_PRIORITY, nullptr);

    printf("[fs_task] started (priority=%d buf=%uKB)\r\n",
           LFS_TASK_PRIORITY, (unsigned)(kRespBufSize / 1024));
}
