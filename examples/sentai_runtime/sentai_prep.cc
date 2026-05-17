// sentai_prep.cc — pre-processed camera-frame slot table.
// See sentai_prep.h for the design contract.
//
// Pure storage + atomic-publish layer.  No image processing, no PXP
// calls — those live in the producer (PrepTask on ARM,
// camera_bridge_recv on SIM).  This file is identical on both targets;
// the SENTAI_PREP_BSS macro picks the right linker section.

#include "sentai_prep.h"
#include "sentai_error.h"

#include <stdint.h>
#include <stdio.h>     // SERR_LOG → printf
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #define SENTAI_PREP_BSS    /* nothing — host-side .bss is fine */
  #define SENTAI_PREP_DMB()  /* no barrier needed on x86 single-core */
#else
  #define SENTAI_PREP_BSS    __attribute__((section(".sdram_bss"), aligned(64)))
  // Cortex-M7 Data Memory Barrier — flushes write buffer so the
  // seq increment is visible to other masters (including ISR readers)
  // strictly after the slot buffer write.
  #define SENTAI_PREP_DMB()  __asm volatile ("dmb" ::: "memory")
#endif

// =========================================================================
// Slot static descriptors (compile-time).
// =========================================================================

// Dims — match the design table in sentai_prep.h.
#define GRAY_NATIVE_W   320
#define GRAY_NATIVE_H   240
#define RGB_64_W         64
#define RGB_64_H         64
#define GRAY_64_W        64
#define GRAY_64_H        64

typedef struct {
    sentai_prep_fmt_t fmt;
    int   max_w;
    int   max_h;
    int   buf_size;          // bytes
} slot_desc_t;

static const slot_desc_t s_desc[SENTAI_PREP_SLOT_COUNT] = {
    [SENTAI_PREP_SLOT_GRAY_NATIVE] = {
        SENTAI_PREP_FMT_Y8,     GRAY_NATIVE_W, GRAY_NATIVE_H,
        GRAY_NATIVE_W * GRAY_NATIVE_H,
    },
    [SENTAI_PREP_SLOT_RGB_64] = {
        SENTAI_PREP_FMT_RGB888, RGB_64_W,      RGB_64_H,
        RGB_64_W * RGB_64_H * 3,
    },
    [SENTAI_PREP_SLOT_GRAY_64] = {
        SENTAI_PREP_FMT_Y8,     GRAY_64_W,     GRAY_64_H,
        GRAY_64_W * GRAY_64_H,
    },
};

// =========================================================================
// Per-slot mutable state.
// =========================================================================
typedef struct {
    uint8_t  refcount;        // 0 = disabled, ≥1 = producer fires this slot
    uint8_t  frame_div;       // 1 = every frame, N = every N-th frame
    uint8_t  frame_counter;   // counts up to frame_div-1, then triggers + resets
    uint8_t  _pad;
    uint32_t seq;             // monotonic; bumped by _commit()
    int      cur_w;           // current produced dims (may be < max for scale)
    int      cur_h;
} slot_state_t;

static slot_state_t s_state[SENTAI_PREP_SLOT_COUNT];

// =========================================================================
// Slot buffers — separate static allocations so the linker can place
// them all in .sdram_bss explicitly (no anonymous union sharing).
// =========================================================================
static uint8_t s_buf_gray_native[GRAY_NATIVE_W * GRAY_NATIVE_H] SENTAI_PREP_BSS;
static uint8_t s_buf_rgb_64     [RGB_64_W * RGB_64_H * 3]       SENTAI_PREP_BSS;
static uint8_t s_buf_gray_64    [GRAY_64_W * GRAY_64_H]         SENTAI_PREP_BSS;

static uint8_t* const s_buf[SENTAI_PREP_SLOT_COUNT] = {
    [SENTAI_PREP_SLOT_GRAY_NATIVE] = s_buf_gray_native,
    [SENTAI_PREP_SLOT_RGB_64]      = s_buf_rgb_64,
    [SENTAI_PREP_SLOT_GRAY_64]     = s_buf_gray_64,
};

// =========================================================================
// Module-wide counters.
// =========================================================================
static uint32_t s_frames_total      = 0;
static uint32_t s_frames_with_aux   = 0;
static uint32_t s_producer_overruns = 0;

// Bitmask: bit i = SERR_PREP_TORN_READ was already logged for slot i
// in this session.  Defined here (before init/reset functions that
// touch it) so file order doesn't bite.  Reset by init/reset_stats.
static uint8_t  s_torn_logged       = 0;

// =========================================================================
// Helpers
// =========================================================================
static int slot_id_ok(sentai_prep_slot_id_t id) {
    return (id >= 0) && (id < SENTAI_PREP_SLOT_COUNT);
}

// =========================================================================
// Public API
// =========================================================================

extern "C" void sentai_prep_init(void) {
    memset(s_state, 0, sizeof(s_state));
    for (int i = 0; i < SENTAI_PREP_SLOT_COUNT; ++i) {
        s_state[i].frame_div = 1;     // default = every frame
    }
    s_frames_total      = 0;
    s_frames_with_aux   = 0;
    s_producer_overruns = 0;
    s_torn_logged       = 0;
}

extern "C" int sentai_prep_slot_enable(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) { SERR_LOG(SERR_PREP_BAD_SLOT, (uint32_t)id); return -1; }
    // uint8 saturates at 255; bound the refcount to detect rogue callers.
    if (s_state[id].refcount < 255) {
        s_state[id].refcount++;
    } else {
        // Log once per saturation hit (M4 — silent saturation gone).
        SERR_LOG(SERR_PREP_REFCOUNT_SAT, (uint32_t)id);
    }
    return (int)s_state[id].refcount;
}

extern "C" int sentai_prep_slot_disable(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return -1;
    if (s_state[id].refcount > 0) s_state[id].refcount--;
    return (int)s_state[id].refcount;
}

extern "C" int sentai_prep_slot_is_enabled(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return 0;
    return s_state[id].refcount > 0 ? 1 : 0;
}

extern "C" int sentai_prep_slot_get(sentai_prep_slot_id_t id,
                                     const uint8_t** out_buf,
                                     int* out_w, int* out_h,
                                     uint32_t* out_seq) {
    if (!slot_id_ok(id) || !out_buf || !out_w || !out_h) return -1;
    if (s_state[id].refcount == 0) return -1;
    if (s_state[id].seq == 0)      return -1;   // never produced yet
    *out_buf = s_buf[id];
    *out_w   = s_state[id].cur_w;
    *out_h   = s_state[id].cur_h;
    if (out_seq) *out_seq = s_state[id].seq;
    return 0;
}

// ---- Seqlock-style atomic read protocol (W11 audit C1+M1 fix) ----
//
// Detection of producer-during-consume interleave.  See header for
// the contract.  s_producer_overruns is bumped by _end_read on miss
// (the metric was previously dead code).
//
// SERR_PREP_TORN_READ is logged only on the FIRST torn read per slot
// (s_torn_logged bit) — the per-frame counter is the right place for
// recurrence; the SERR is just a forensics breadcrumb that the
// hazard fired at all in this session.  (s_torn_logged declared
// above near the module-wide counters.)

extern "C" int sentai_prep_slot_begin_read(sentai_prep_slot_id_t id,
                                            const uint8_t** out_buf,
                                            int* out_w, int* out_h,
                                            uint32_t* out_ticket) {
    if (!slot_id_ok(id) || !out_buf || !out_w || !out_h || !out_ticket) return -1;
    if (s_state[id].refcount == 0) return -1;
    const uint32_t seq = s_state[id].seq;
    if (seq == 0) return -1;
    *out_buf    = s_buf[id];
    *out_w      = s_state[id].cur_w;
    *out_h      = s_state[id].cur_h;
    *out_ticket = seq;
    // Ensure subsequent buffer reads happen-after this seq snapshot.
    SENTAI_PREP_DMB();
    return 0;
}

extern "C" int sentai_prep_slot_end_read(sentai_prep_slot_id_t id,
                                          uint32_t ticket) {
    if (!slot_id_ok(id)) return 0;
    SENTAI_PREP_DMB();
    if (s_state[id].seq == ticket) return 1;   // no torn read

    // Torn: producer fired during consume window.  Bump counter and
    // SERR_LOG on first occurrence per slot per session.  Counter is
    // single-writer per slot today (one consumer assumption) — safe.
    s_producer_overruns++;
    const uint8_t bit = (uint8_t)(1u << (int)id);
    if (!(s_torn_logged & bit)) {
        s_torn_logged = (uint8_t)(s_torn_logged | bit);
        SERR_LOG(SERR_PREP_TORN_READ, (uint32_t)id);
    }
    return 0;
}

extern "C" int sentai_prep_slot_set_div(sentai_prep_slot_id_t id, int n) {
    if (!slot_id_ok(id)) return -1;
    if (n < 1) n = 1;
    if (n > 255) n = 255;
    s_state[id].frame_div    = (uint8_t)n;
    s_state[id].frame_counter = 0;
    return 0;
}

// =========================================================================
// Producer-side API
// =========================================================================

extern "C" uint8_t* sentai_prep_slot_begin_write(sentai_prep_slot_id_t id,
                                                  int* out_w, int* out_h) {
    if (!slot_id_ok(id))          return nullptr;
    if (s_state[id].refcount == 0) return nullptr;
    if (out_w) *out_w = s_desc[id].max_w;
    if (out_h) *out_h = s_desc[id].max_h;
    return s_buf[id];
}

extern "C" void sentai_prep_slot_commit(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return;
    // Record produced dims (today always max_w/h; a future producer
    // could shrink for an ROI write).
    s_state[id].cur_w = s_desc[id].max_w;
    s_state[id].cur_h = s_desc[id].max_h;
    SENTAI_PREP_DMB();
    s_state[id].seq++;
}

extern "C" uint32_t sentai_prep_tick_frame(void) {
    s_frames_total++;
    uint32_t will_fire = 0;
    int any = 0;
    for (int i = 0; i < SENTAI_PREP_SLOT_COUNT; ++i) {
        if (s_state[i].refcount == 0) continue;
        uint8_t div = s_state[i].frame_div;
        if (div == 0) div = 1;
        if (++s_state[i].frame_counter >= div) {
            s_state[i].frame_counter = 0;
            will_fire |= (1u << i);
            any = 1;
        }
    }
    if (any) s_frames_with_aux++;
    return will_fire;
}

extern "C" void sentai_prep_get_stats(sentai_prep_stats_t* out) {
    if (!out) return;
    out->frames_total      = s_frames_total;
    out->frames_with_aux   = s_frames_with_aux;
    out->producer_overruns = s_producer_overruns;
    for (int i = 0; i < SENTAI_PREP_SLOT_COUNT; ++i) {
        out->slot_refcount[i] = s_state[i].refcount;
        out->slot_seq[i]      = s_state[i].seq;
    }
}

extern "C" void sentai_prep_reset_stats(void) {
    s_frames_total      = 0;
    s_frames_with_aux   = 0;
    s_producer_overruns = 0;
    s_torn_logged       = 0;
    // Note: do NOT reset per-slot seq — consumers track monotonic seq.
}

extern "C" sentai_prep_fmt_t sentai_prep_slot_fmt(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return SENTAI_PREP_FMT_Y8;
    return s_desc[id].fmt;
}
extern "C" int sentai_prep_slot_max_w(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return 0;
    return s_desc[id].max_w;
}
extern "C" int sentai_prep_slot_max_h(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return 0;
    return s_desc[id].max_h;
}
extern "C" int sentai_prep_slot_buf_size(sentai_prep_slot_id_t id) {
    if (!slot_id_ok(id)) return 0;
    return s_desc[id].buf_size;
}
