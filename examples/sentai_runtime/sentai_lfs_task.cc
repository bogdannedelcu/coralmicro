// sentai_lfs_task.cc — Dedicated LFS task for thread-safe filesystem GET access.
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

#include "sentai_lfs_task.h"

#include "libs/base/filesystem.h"
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

// Queue depth 1: only one GET request in flight at a time.
// Browser retries until data is ready, so depth > 1 just wastes memory.
#define LFS_QUEUE_DEPTH    1

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
    sentai_lfs_req_type_t type;
    char path[256];
} LfsRequest;

static QueueHandle_t s_req_queue = nullptr;

// ---------------------------------------------------------------------------
// Slot state — tracks where the current GET request is in the pipeline.
//
// Written by tcpip_thread (FsOpenCustom/FsCloseCustom via sentai_lfs_try_serve
// and sentai_lfs_resp_done) and by lfs_task (when result is ready).
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
static sentai_lfs_req_type_t       s_slot_type  = LFS_REQ_LS;

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
// DoLs / DoRaw — called ONLY from lfs_task (safe to block on lfs_*)
// ---------------------------------------------------------------------------
static size_t DoLs(const char* path) {
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) { g_resp_buf[0] = '['; g_resp_buf[1] = ']'; return 2; }
    lfs_dir_t dir;
    if (lfs_dir_open(lfs, &dir, path) < 0) {
        g_resp_buf[0] = '['; g_resp_buf[1] = ']'; return 2;
    }
    size_t pos = 0;
    g_resp_buf[pos++] = '[';
    lfs_info info;
    bool first = true;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
        if (info.name[0] == '.' &&
            (info.name[1] == '\0' ||
             (info.name[1] == '.' && info.name[2] == '\0'))) continue;
        if (!first && pos < kRespBufSize) g_resp_buf[pos++] = ',';
        first = false;
        if (pos < kRespBufSize) g_resp_buf[pos++] = '{';
        BufAppend(&pos, "\"name\":");
        BufAppendJsonStr(&pos, info.name);
        char tmp[64];
        snprintf(tmp, sizeof(tmp), ",\"type\":\"%s\",\"size\":%lu",
                 info.type == LFS_TYPE_DIR ? "dir" : "file",
                 (unsigned long)info.size);
        BufAppend(&pos, tmp);
        if (pos < kRespBufSize) g_resp_buf[pos++] = '}';
    }
    lfs_dir_close(lfs, &dir);
    if (pos < kRespBufSize) g_resp_buf[pos++] = ']';
    return pos;
}

static size_t DoRaw(const char* path) {
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return 0;
    lfs_info info;
    if (lfs_stat(lfs, path, &info) < 0 ||
        info.type != LFS_TYPE_REG || info.size == 0) return 0;
    size_t to_read = (info.size < kRespBufSize) ? info.size : kRespBufSize;
    lfs_file_t f;
    if (lfs_file_open(lfs, &f, path, LFS_O_RDONLY) < 0) return 0;
    lfs_ssize_t n = lfs_file_read(lfs, &f, g_resp_buf, to_read);
    lfs_file_close(lfs, &f);
    return (n > 0) ? (size_t)n : 0;
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
        if (req.type == LFS_REQ_LS) {
            len = DoLs(req.path);
        } else if (req.type == LFS_REQ_RAW) {
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

static size_t EnqueueLfsRequest(sentai_lfs_req_type_t type, const char* path) {
    LfsRequest req;
    req.type = type;
    strncpy(req.path, path, sizeof(req.path) - 1);
    req.path[sizeof(req.path) - 1] = '\0';
    s_slot_type = type;
    strncpy(s_slot_path, path, sizeof(s_slot_path) - 1);
    s_slot_path[sizeof(s_slot_path) - 1] = '\0';
    if (xQueueSend(s_req_queue, &req, 0) == pdTRUE) {
        s_slot_state = SLOT_PENDING;
    }
    return 0;
}

size_t sentai_lfs_try_serve(sentai_lfs_req_type_t type, const char* path) {
    if (!s_req_queue) return 0;

    // Step 1 — if the slot already holds the result for THIS exact request
    // (either from an earlier async attempt or a concurrent burst), serve it.
    if (s_slot_state == SLOT_READY) {
        if (s_slot_type == type && strcmp(s_slot_path, path) == 0) {
            size_t len = s_slot_len;
            s_slot_state = SLOT_IDLE;
            if (len > 0) { s_slot_state = SLOT_SERVING; return len; }
            return (size_t)-1;  // 404 / empty
        }
        // Stale result for a different path — discard it.
        s_slot_state = SLOT_IDLE;
    }

    // Step 2 — a previous response is still being streamed from g_resp_buf.
    // Reusing the buffer now would corrupt it; tell the client to retry.
    if (s_slot_state == SLOT_SERVING) return 0;

    // Step 3 — FAST PATH.  Take the LFS mutex and service the request inline
    // from tcpip_thread so a single GET resolves in one HTTP round-trip.
    //
    // The short wait (FAST_PATH_WAIT_MS) catches the common case where MP
    // is partway through a brief flash op — by the time we'd be about to
    // send lfs_busy, the mutex is usually free.  The wait is an order of
    // magnitude below the USB NCM transmit-timeout so tcpip_thread's brief
    // stall here is invisible to the host.
    //
    // The mutex is the app-level gate used by MP, boot_log_flush, crash_log,
    // and lfs_task itself.  Once we own it, none of those is inside lfs_*
    // and LFS's internal mutex is free — so the lfs_* calls below will not
    // stall on top of our own wait.
    constexpr TickType_t FAST_PATH_WAIT_MS = 500;
    if (s_lfs_mutex &&
        xSemaphoreTake(s_lfs_mutex, pdMS_TO_TICKS(FAST_PATH_WAIT_MS)) == pdTRUE) {
        size_t len = (type == LFS_REQ_LS) ? DoLs(path) : DoRaw(path);
        xSemaphoreGive(s_lfs_mutex);
        if (len > 0) { s_slot_state = SLOT_SERVING; return len; }
        return (size_t)-1;  // 404 / empty
    }

    // Step 4 — SLOW PATH.  Mutex contended (MP mid-write, lfs_task busy,
    // boot_log_flush in progress…).  Hand the request to lfs_task so we
    // don't block tcpip_thread, and tell the client to retry.  By the next
    // retry the fast path will usually succeed, or the queued result will
    // be waiting in SLOT_READY.
    if (s_slot_state == SLOT_IDLE) {
        EnqueueLfsRequest(type, path);
    }
    return 0;
}

uint8_t* sentai_lfs_resp_buf(void) { return g_resp_buf; }

void sentai_lfs_resp_done(void) {
    s_slot_state = SLOT_IDLE;
}

// ---------------------------------------------------------------------------
// sentai_lfs_lock / sentai_lfs_unlock
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
extern "C" int sentai_lfs_lock(void) {
    if (!s_lfs_mutex) return 1;  // Pre-init: single-threaded boot, allow.
    return xSemaphoreTake(s_lfs_mutex, pdMS_TO_TICKS(2000)) == pdTRUE ? 1 : 0;
}

extern "C" void sentai_lfs_unlock(void) {
    if (s_lfs_mutex) xSemaphoreGive(s_lfs_mutex);
}

// ---------------------------------------------------------------------------
// sentai_lfs_task_start — call once from sentai_runtime before sentai_httpd_start
// ---------------------------------------------------------------------------
extern "C" void sentai_lfs_task_start(void) {
    static bool started = false;
    if (started) return;
    started = true;

    s_lfs_mutex = xSemaphoreCreateMutex();
    s_req_queue  = xQueueCreate(LFS_QUEUE_DEPTH, sizeof(LfsRequest));

    xTaskCreate(LfsTaskFn, "lfs_task", LFS_TASK_STACK, nullptr,
                LFS_TASK_PRIORITY, nullptr);

    printf("[lfs_task] started (priority=%d buf=%uKB)\r\n",
           LFS_TASK_PRIORITY, (unsigned)(kRespBufSize / 1024));
}
