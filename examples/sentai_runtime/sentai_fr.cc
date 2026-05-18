// sentai_fr.cc — OP-S10-W13-T2 implementation.
//
// Multi-channel flight recorder.  Producers push items (best-effort,
// O(1), non-blocking).  Single recorder task drains queues → disk.
//
// First scope: frames channel fully implemented (operator priority for
// SafetyArucoBaseline debug); events + scalars implemented in simple
// CSV/text form per operator spec ("event type si TEXT, poate coma
// separated... nu vreau mai complicat de atat").  Kernel channel is a
// stub (no-op) until T-future.
//
// SIM-only initially; ARM port (FxUser sinks) deferred.

#include "sentai_fr.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

// ── Threading model (operator spec 2026-05-18) ─────────────────────
// "sentai.fr trebuie sa fie independent de alte task-uri si nu trebuie
//  sa le incurce, e doar un jurnal de zbor"; followup: "nu decupla
//  de FreeRTOS".
//
// Producers acquire `s_lock` (BINARY semaphore — NOT a PI-mutex) for a
// brief slot memcpy and release.  The worker (in sentai_fr_task.cc)
// calls `sentai_fr_drain_round()` from this TU, which acquires the
// same lock briefly per item, copies into a static drain-side scratch,
// releases, then does I/O outside the lock.
//
// Why a binary semaphore, not xSemaphoreCreateMutex?  A FreeRTOS
// MUTEX is priority-inheriting; producer + drain contention tripped
// `xTaskPriorityDisinherit pxTCB == pxCurrentTCB` (FreeRTOS tasks.c).
// PI also entangles tasks' priorities — the opposite of "independent".
// A binary semaphore behaves as a non-PI mutex; held only briefly,
// no priority inversion is possible in practice.
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <time.h>
#define FR_HAVE_FREERTOS 1

// ============================================================================
// Per-channel slot pools (static, .sdram_bss when on ARM).
// ============================================================================
namespace {

// ── Frames pool ────────────────────────────────────────────────────────
// On ARM the pool defaults are 8 × 320×240 (~614 KB) — must live in
// SDRAM because m_data is only 224 KB.  The explicit `"aw",%nobits`
// assembler attributes match the NOLOAD type of the linker section
// (otherwise GCC emits .sdram_bss as PROGBITS and the assembler warns).
// SIM build keeps the pool in default .bss with no linker constraints.
#ifdef __arm__
#define SENTAI_FR_POOL_ATTR  __attribute__((section(".sdram_bss,\"aw\",%nobits @")))
#else
#define SENTAI_FR_POOL_ATTR
#endif
struct FrameSlot {
    uint8_t   data[SENTAI_FR_FRAMES_BYTES];
    int       w, h;
    int       n_dets;
    uint32_t  seq;
    uint32_t  ts_ms;
};
static FrameSlot s_frame_pool[SENTAI_FR_FRAMES_SLOTS] SENTAI_FR_POOL_ATTR;

// ── Events pool ────────────────────────────────────────────────────────
// 256 × 120 B ≈ 30 KB.  ARM: routed to .sdram_bss (m_data is 224 KB and
// shared with the rest of the firmware's DTCM .bss).
struct EventSlot {
    uint32_t  ts_ms;
    char      type[SENTAI_FR_EVENT_TYPE_LEN];
    char      text[SENTAI_FR_EVENT_TEXT_LEN];
};
static EventSlot s_event_pool[SENTAI_FR_EVENTS_SLOTS] SENTAI_FR_POOL_ATTR;

// ── Scalars pool ───────────────────────────────────────────────────────
// 1024 × 32 B = 32 KB.  ARM: routed to .sdram_bss (same reason as events).
struct ScalarSlot {
    uint32_t  ts_ms;
    char      label[SENTAI_FR_SCALAR_LABEL_LEN];
    double    value;
};
static ScalarSlot s_scalar_pool[SENTAI_FR_SCALARS_SLOTS] SENTAI_FR_POOL_ATTR;

// ── Per-channel runtime state (mutex-protected) ────────────────────────
struct ChanState {
    sentai_fr_channel_stats_t stats{};
    uint32_t   head = 0;        // next slot to WRITE (producer)
    uint32_t   tail = 0;        // next slot to READ (recorder)
    // (queue depth = head - tail; capacity = pool size; modular)
};
static ChanState s_chans[SENTAI_FR_CH__COUNT];

inline uint32_t pool_size_for(sentai_fr_channel_t c) {
    switch (c) {
    case SENTAI_FR_CH_FRAMES:  return SENTAI_FR_FRAMES_SLOTS;
    case SENTAI_FR_CH_EVENTS:  return SENTAI_FR_EVENTS_SLOTS;
    case SENTAI_FR_CH_SCALARS: return SENTAI_FR_SCALARS_SLOTS;
    default: return 0;
    }
}

// ── Binary semaphore (FreeRTOS, no priority inheritance) ───────────────
// Held briefly (single slot memcpy or snapshot).  Worker (in
// sentai_fr_task.cc) polls via vTaskDelay — no condition variable.
static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock = nullptr;
inline void mu_init() {
    if (!s_lock) {
        s_lock = xSemaphoreCreateBinaryStatic(&s_lock_buf);
        if (s_lock) xSemaphoreGive(s_lock);   // start "available"
    }
}
inline void mu_lock()   { if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY); }
inline void mu_unlock() { if (s_lock) xSemaphoreGive(s_lock); }
// Producers used to signal the worker via a counting semaphore; that
// mechanism was removed when the worker switched to a 20 ms polling
// schedule (see sentai_fr_task.cc).  Keep a no-op so existing
// push_* sites still compile.
inline void wake_post() {}

struct MuGuard { MuGuard(){ mu_lock(); } ~MuGuard(){ mu_unlock(); } };

// Recorder thread lives in sentai_fr_task.cc (split for clarity,
// mirrors sentai_safety / sentai_safety_task).  Public callers reach
// it via sentai_fr_task_start / sentai_fr_task_stop declared in
// sentai_fr_task.h.

static FILE* s_events_fp  = nullptr;
static FILE* s_scalars_fp = nullptr;

// ── Helpers ────────────────────────────────────────────────────────────
// mkdir is a POSIX-only call.  ARM build uses FileX / NXP HAL — caller
// is responsible for ensuring the FX directory exists on the user
// partition.  We treat the host-side mkdir as a no-op on ARM and
// trust the FileX volume layout.
inline bool mkdir_p(const char* path) {
    if (!path || !*path) return false;
#ifdef __arm__
    (void)path;
    return true;
#else
    struct stat st;
    if (stat(path, &st) == 0) return S_ISDIR(st.st_mode);
    return mkdir(path, 0755) == 0;
#endif
}

inline void copy_str_(char* dst, size_t cap, const char* src) {
    if (cap == 0) return;
    if (!src) { dst[0] = 0; return; }
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = 0;
}

inline void sanitise_csv_(char* s) {
    // Replace control chars + commas with spaces so the CSV stays
    // line-oriented even on weird input.
    for (; *s; ++s) {
        if (*s == ',' || *s == '\n' || *s == '\r') *s = ' ';
    }
}

inline FILE* open_append_(const char* path, const char* header) {
    FILE* fp = fopen(path, "ab");
    if (!fp) return nullptr;
    // Write header only if file is empty (new).
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    if (sz == 0 && header) {
        fputs(header, fp);
        fputc('\n', fp);
        fflush(fp);
    }
    return fp;
}

}  // namespace

// ============================================================================
// Public API
// ============================================================================

extern "C" int sentai_fr_init(void) {
    mu_init();
    MuGuard g;
    for (int i = 0; i < SENTAI_FR_CH__COUNT; ++i) s_chans[i] = ChanState{};
    return 0;
}

extern "C" int sentai_fr_open(sentai_fr_channel_t ch, const char* path) {
    if (ch <= 0 || ch >= SENTAI_FR_CH__COUNT) return SENTAI_FR_ERR_UNKNOWN;
    if (!path || !*path) return SENTAI_FR_ERR_PARAMS;
    mu_init();
    MuGuard g;
    ChanState& c = s_chans[ch];
    // Frames: path = dir
    if (ch == SENTAI_FR_CH_FRAMES) {
        if (!mkdir_p(path)) {
            // Continue even if mkdir failed — caller may have pre-created.
            struct stat st;
            if (stat(path, &st) != 0 || !S_ISDIR(st.st_mode)) {
                return SENTAI_FR_ERR_IO;
            }
        }
    } else if (ch == SENTAI_FR_CH_EVENTS) {
        if (s_events_fp) { fclose(s_events_fp); s_events_fp = nullptr; }
        s_events_fp = open_append_(path, "# sentai.fr events  ts_ms,type,text");
        if (!s_events_fp) return SENTAI_FR_ERR_IO;
    } else if (ch == SENTAI_FR_CH_SCALARS) {
        if (s_scalars_fp) { fclose(s_scalars_fp); s_scalars_fp = nullptr; }
        s_scalars_fp = open_append_(path, "# sentai.fr scalars  ts_ms,label,value");
        if (!s_scalars_fp) return SENTAI_FR_ERR_IO;
    } else if (ch == SENTAI_FR_CH_KERNEL) {
        // Stub T1 — kernel mirror added later.
    }
    c.stats.enabled = 1;
    copy_str_(c.stats.path, sizeof(c.stats.path), path);
    c.head = c.tail = 0;
    return SENTAI_FR_OK;
}

extern "C" int sentai_fr_close(sentai_fr_channel_t ch) {
    if (ch <= 0 || ch >= SENTAI_FR_CH__COUNT) return SENTAI_FR_ERR_UNKNOWN;
    MuGuard g;
    s_chans[ch].stats.enabled = 0;
    if (ch == SENTAI_FR_CH_EVENTS && s_events_fp) {
        fflush(s_events_fp); fclose(s_events_fp); s_events_fp = nullptr;
    }
    if (ch == SENTAI_FR_CH_SCALARS && s_scalars_fp) {
        fflush(s_scalars_fp); fclose(s_scalars_fp); s_scalars_fp = nullptr;
    }
    return SENTAI_FR_OK;
}

extern "C" int sentai_fr_push_frame(const uint8_t* gray, int w, int h,
                                      int n_dets, uint32_t seq, uint32_t ts_ms) {
    if (!gray || w <= 0 || h <= 0) return SENTAI_FR_ERR_PARAMS;
    if (w > SENTAI_FR_FRAMES_MAX_W || h > SENTAI_FR_FRAMES_MAX_H)
        return SENTAI_FR_ERR_PARAMS;
    MuGuard g;
    ChanState& c = s_chans[SENTAI_FR_CH_FRAMES];
    if (!c.stats.enabled) return SENTAI_FR_OK;  // silent no-op
    c.stats.pushes_total++;
    uint32_t depth = c.head - c.tail;
    if (depth >= SENTAI_FR_FRAMES_SLOTS) {
        c.stats.drops_full++;
        return SENTAI_FR_ERR_FULL;
    }
    FrameSlot& slot = s_frame_pool[c.head % SENTAI_FR_FRAMES_SLOTS];
    memcpy(slot.data, gray, (size_t)w * (size_t)h);
    slot.w = w; slot.h = h; slot.n_dets = n_dets;
    slot.seq = seq; slot.ts_ms = ts_ms;
    c.head++;
    depth = c.head - c.tail;
    if (depth > c.stats.worst_queue_depth) c.stats.worst_queue_depth = depth;
    c.stats.queue_depth = depth;
    c.stats.pushes_accepted++;
    wake_post();
    return SENTAI_FR_OK;
}

extern "C" int sentai_fr_push_event(const char* type, const char* text) {
    if (!type) type = "";
    if (!text) text = "";
    MuGuard g;
    ChanState& c = s_chans[SENTAI_FR_CH_EVENTS];
    if (!c.stats.enabled) return SENTAI_FR_OK;
    c.stats.pushes_total++;
    uint32_t depth = c.head - c.tail;
    if (depth >= SENTAI_FR_EVENTS_SLOTS) {
        c.stats.drops_full++;
        return SENTAI_FR_ERR_FULL;
    }
    EventSlot& slot = s_event_pool[c.head % SENTAI_FR_EVENTS_SLOTS];
    // Use the FreeRTOS tick / monotonic clock for ts.
#if FR_HAVE_FREERTOS
    slot.ts_ms = (uint32_t)(xTaskGetTickCount() * (1000U / configTICK_RATE_HZ));
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    slot.ts_ms = (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
#endif
    copy_str_(slot.type, sizeof(slot.type), type);
    copy_str_(slot.text, sizeof(slot.text), text);
    sanitise_csv_(slot.type);
    sanitise_csv_(slot.text);
    c.head++;
    depth = c.head - c.tail;
    if (depth > c.stats.worst_queue_depth) c.stats.worst_queue_depth = depth;
    c.stats.queue_depth = depth;
    c.stats.pushes_accepted++;
    wake_post();
    return SENTAI_FR_OK;
}

extern "C" int sentai_fr_push_scalar(const char* label, double value,
                                       uint32_t ts_ms) {
    if (!label) label = "";
    MuGuard g;
    ChanState& c = s_chans[SENTAI_FR_CH_SCALARS];
    if (!c.stats.enabled) return SENTAI_FR_OK;
    c.stats.pushes_total++;
    uint32_t depth = c.head - c.tail;
    if (depth >= SENTAI_FR_SCALARS_SLOTS) {
        c.stats.drops_full++;
        return SENTAI_FR_ERR_FULL;
    }
    ScalarSlot& slot = s_scalar_pool[c.head % SENTAI_FR_SCALARS_SLOTS];
    slot.ts_ms = ts_ms;
    copy_str_(slot.label, sizeof(slot.label), label);
    sanitise_csv_(slot.label);
    slot.value = value;
    c.head++;
    depth = c.head - c.tail;
    if (depth > c.stats.worst_queue_depth) c.stats.worst_queue_depth = depth;
    c.stats.queue_depth = depth;
    c.stats.pushes_accepted++;
    wake_post();
    return SENTAI_FR_OK;
}

extern "C" int sentai_fr_get_stats(sentai_fr_channel_t ch,
                                     sentai_fr_channel_stats_t* out) {
    if (!out) return SENTAI_FR_ERR_PARAMS;
    if (ch <= 0 || ch >= SENTAI_FR_CH__COUNT) return SENTAI_FR_ERR_UNKNOWN;
    MuGuard g;
    *out = s_chans[ch].stats;
    return SENTAI_FR_OK;
}

// ============================================================================
// Recorder task — drains queues to disk.
// ============================================================================
namespace {

// Drain-side scratch slot.  Static BSS — NOT on the task stack
// (FreeRTOS POSIX stack = configMINIMAL_STACK_SIZE * 4 ≈ 16 KB; a
// FrameSlot is ~308 KB max / 76 KB at 320×240 — would overflow the
// task stack instantly).  Only the drain task reads/writes this, so
// no concurrent access; safe outside the channel mutex once snapshot
// has been copied under the lock.  Routed to SDRAM on ARM (same as
// the pool itself — m_data is only 224 KB).
static FrameSlot s_drain_snap_frame SENTAI_FR_POOL_ATTR;

// Drain one frame slot if available.  Returns true on consumed.
bool drain_one_frame_() {
    char dirpath[128];
    {
        MuGuard g;
        ChanState& c = s_chans[SENTAI_FR_CH_FRAMES];
        if (c.tail == c.head || !c.stats.enabled) return false;
        // Copy into static drain-side scratch under the lock; the
        // pool slot becomes safe to recycle as soon as we release.
        s_drain_snap_frame = s_frame_pool[c.tail % SENTAI_FR_FRAMES_SLOTS];
        copy_str_(dirpath, sizeof(dirpath), c.stats.path);
        c.tail++;
        c.stats.queue_depth = c.head - c.tail;
    }
    // I/O outside the mutex.
    const FrameSlot& snap = s_drain_snap_frame;
    uint32_t rel_ms = snap.ts_ms;
    char path[256];
    snprintf(path, sizeof(path), "%s/t%08u_n%d_f%06u.pgm",
              dirpath, (unsigned)rel_ms, snap.n_dets, (unsigned)snap.seq);
    FILE* fp = fopen(path, "wb");
    bool ok = false;
    if (fp) {
        fprintf(fp, "P5\n%d %d\n255\n", snap.w, snap.h);
        size_t expected = (size_t)snap.w * (size_t)snap.h;
        ok = (fwrite(snap.data, 1, expected, fp) == expected);
        fclose(fp);
    }
    {
        MuGuard g;
        ChanState& c = s_chans[SENTAI_FR_CH_FRAMES];
        if (ok) c.stats.writes_ok++;
        else    c.stats.writes_fail++;
    }
    return true;
}

bool drain_one_event_() {
    EventSlot snap;
    FILE* fp;
    {
        MuGuard g;
        ChanState& c = s_chans[SENTAI_FR_CH_EVENTS];
        if (c.tail == c.head || !c.stats.enabled) return false;
        snap = s_event_pool[c.tail % SENTAI_FR_EVENTS_SLOTS];
        c.tail++;
        c.stats.queue_depth = c.head - c.tail;
        fp = s_events_fp;
    }
    if (!fp) return true;
    fprintf(fp, "%u,%s,%s\n",
             (unsigned)snap.ts_ms, snap.type, snap.text);
    fflush(fp);
    MuGuard g;
    s_chans[SENTAI_FR_CH_EVENTS].stats.writes_ok++;
    return true;
}

bool drain_one_scalar_() {
    ScalarSlot snap;
    FILE* fp;
    {
        MuGuard g;
        ChanState& c = s_chans[SENTAI_FR_CH_SCALARS];
        if (c.tail == c.head || !c.stats.enabled) return false;
        snap = s_scalar_pool[c.tail % SENTAI_FR_SCALARS_SLOTS];
        c.tail++;
        c.stats.queue_depth = c.head - c.tail;
        fp = s_scalars_fp;
    }
    if (!fp) return true;
    fprintf(fp, "%u,%s,%.9g\n", (unsigned)snap.ts_ms, snap.label, snap.value);
    fflush(fp);
    MuGuard g;
    s_chans[SENTAI_FR_CH_SCALARS].stats.writes_ok++;
    return true;
}

}  // namespace

// Public drain primitive — called by the worker task in
// sentai_fr_task.cc.  Best-effort: drain up to 8 items PER CHANNEL
// per call so a single noisy channel can't starve the others.
// Returns the total number of items consumed across all channels.
extern "C" uint32_t sentai_fr_drain_round(void) {
    uint32_t n = 0;
    for (int i = 0; i < 8 && drain_one_frame_();  ++i) ++n;
    for (int i = 0; i < 8 && drain_one_event_();  ++i) ++n;
    for (int i = 0; i < 8 && drain_one_scalar_(); ++i) ++n;
    return n;
}

// Internal entry point used by sentai_fr_task.cc::sentai_fr_task_start
// to make sure the lock semaphore is created before the worker takes
// it.  Idempotent.  Public ABI; not exposed via the high-level header.
extern "C" void sentai_fr_internal_mu_init(void) { mu_init(); }
