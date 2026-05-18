// sentai_safety.cc — OP-S10-W12-T2 implementation.
//
// Pure state machine.  Does NOT call sentai_aruco_detect / read camera
// frames / run any per-pixel compute.  Receives upstream results via
// `on_aruco_result` (push from sentai_safety_task or any feeder).
//
// Threading: single-writer per check (the feeder; for ArUco that's
// sentai_safety_task).  Multi-reader from MP via snapshot/aborted/reason.
// All shared state guarded by `s_mu`; the abort flag is read via a
// single byte load (atomic on supported arches, racy but harmless
// elsewhere — readers always re-check reason under the mutex).
//
// All public symbols are extern "C" to match the .h.  No exceptions.

#include "sentai_safety.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#if defined(__ARM_ARCH) || defined(SENTAI_HAVE_FREERTOS)
#  include "FreeRTOS.h"
#  include "semphr.h"
#  define SAFETY_HAVE_FREERTOS 1
#else
#  include <pthread.h>
#  define SAFETY_HAVE_FREERTOS 0
#endif

// ============================================================================
// Locking primitive — FreeRTOS mutex on ARM/RTOS, pthread_mutex on POSIX SIM.
// ============================================================================
namespace {

#if SAFETY_HAVE_FREERTOS
static StaticSemaphore_t s_mu_buf;
static SemaphoreHandle_t s_mu = nullptr;
inline void mu_init() {
    if (s_mu == nullptr) s_mu = xSemaphoreCreateMutexStatic(&s_mu_buf);
}
inline void mu_lock()   { if (s_mu) xSemaphoreTake(s_mu, portMAX_DELAY); }
inline void mu_unlock() { if (s_mu) xSemaphoreGive(s_mu); }
#else
static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;
inline void mu_init()   {}
inline void mu_lock()   { pthread_mutex_lock(&s_mu); }
inline void mu_unlock() { pthread_mutex_unlock(&s_mu); }
#endif

struct MuGuard {
    MuGuard()  { mu_lock(); }
    ~MuGuard() { mu_unlock(); }
};

// ============================================================================
// Per-check state.
// ============================================================================
struct ArucoState {
    bool                                enabled        = false;
    sentai_safety_aruco_params_t        params         = {4, 1.0f};
    uint32_t                            last_push_t_ms = 0;
    uint32_t                            last_seq       = 0;
    int                                 last_n_dets    = 0;
    bool                                in_lt_streak   = false;
    uint32_t                            streak_start_ms= 0;
    uint32_t                            n_frames       = 0;
    void reset() { *this = ArucoState{}; }
};

// (Future-stubbed checks — keep state structs minimal to reserve ABI.)
struct AltFloorState {
    bool enabled = false; sentai_safety_alt_floor_params_t p{};
    uint32_t last_push_t_ms = 0; void reset(){ *this = AltFloorState{}; }
};
struct EkfCeilState {
    bool enabled = false; sentai_safety_ekf_ceil_params_t p{};
    uint32_t last_push_t_ms = 0; void reset(){ *this = EkfCeilState{}; }
};
struct BatteryState {
    bool enabled = false; sentai_safety_battery_params_t p{};
    uint32_t last_push_t_ms = 0; void reset(){ *this = BatteryState{}; }
};
struct LinkState {
    bool enabled = false; sentai_safety_link_params_t p{};
    uint32_t last_push_t_ms = 0; void reset(){ *this = LinkState{}; }
};

struct Manager {
    bool                inited      = false;
    // ── Latched abort flag (sticky until clear()) ────────────────────
    volatile uint8_t    aborted     = 0;
    sentai_safety_check_t   abort_kind = SENTAI_SAFETY_CHK_NONE;
    uint32_t            abort_t_ms  = 0;
    char                reason[SENTAI_SAFETY_REASON_LEN] = {0};
    // ── Per-check state ──────────────────────────────────────────────
    ArucoState          aruco;
    AltFloorState       alt_floor;
    EkfCeilState        ekf_ceil;
    BatteryState        battery;
    LinkState           link;
    // ── Event ring ───────────────────────────────────────────────────
    sentai_safety_event_t evt[SENTAI_SAFETY_EVENTS_CAP] = {};
    uint32_t            evt_head    = 0;       // next write index
    uint32_t            evt_count   = 0;       // total writes (for ring math)
};
static Manager S;

// ── Helpers (all called WITH s_mu held unless noted) ───────────────────
inline void log_event_(uint32_t now_ms, sentai_safety_check_t c,
                        sentai_safety_evkind_t kind,
                        const sentai_safety_event_t* tmpl) {
    sentai_safety_event_t& slot = S.evt[S.evt_head % SENTAI_SAFETY_EVENTS_CAP];
    slot.t_ms = now_ms;
    slot.check = static_cast<uint8_t>(c);
    slot.kind  = static_cast<uint8_t>(kind);
    slot._pad  = 0;
    if (tmpl) slot.d = tmpl->d; else memset(&slot.d, 0, sizeof(slot.d));
    ++S.evt_head;
    ++S.evt_count;
}

inline void latch_abort_(sentai_safety_check_t kind, uint32_t now_ms,
                          const char* msg) {
    if (S.aborted) return;   // sticky — first reason wins
    S.aborted    = 1;
    S.abort_kind = kind;
    S.abort_t_ms = now_ms;
    if (msg) {
        size_t n = strlen(msg);
        if (n >= sizeof(S.reason)) n = sizeof(S.reason) - 1;
        memcpy(S.reason, msg, n);
        S.reason[n] = 0;
    }
}

inline void set_active_mask_bit_(int* mask_out, sentai_safety_check_t c, bool v) {
    if (c <= 0 || c >= SENTAI_SAFETY_CHK__COUNT) return;
    if (v) *mask_out |= (1 << c); else *mask_out &= ~(1 << c);
}

inline uint8_t compute_active_mask_() {
    uint8_t m = 0;
    if (S.aruco.enabled)     m |= (1 << SENTAI_SAFETY_CHK_ARUCO);
    if (S.alt_floor.enabled) m |= (1 << SENTAI_SAFETY_CHK_ALT_FLOOR);
    if (S.ekf_ceil.enabled)  m |= (1 << SENTAI_SAFETY_CHK_EKF_CEIL);
    if (S.battery.enabled)   m |= (1 << SENTAI_SAFETY_CHK_BATTERY);
    if (S.link.enabled)      m |= (1 << SENTAI_SAFETY_CHK_LINK);
    return m;
}

}  // namespace

// ============================================================================
// Public API
// ============================================================================

extern "C" int sentai_safety_init(void) {
    mu_init();
    MuGuard g;
    S = Manager{};
    S.inited = true;
    return 0;
}

extern "C" int sentai_safety_clear(void) {
    mu_init();
    MuGuard g;
    if (!S.inited) {
        S = Manager{};
        S.inited = true;
        return 0;
    }
    // Preserve event-log audit by emitting CLEAR FIRST, then wipe.
    uint32_t now_ms = 0;
    // Use the most recent push time as best-effort timestamp if caller
    // didn't pass us a clock; the next push will overwrite anyway.
    if (S.aruco.last_push_t_ms > now_ms) now_ms = S.aruco.last_push_t_ms;
    log_event_(now_ms, SENTAI_SAFETY_CHK_NONE, SENTAI_SAFETY_EV_CLEAR, nullptr);
    // Snapshot events out before reset — actually, we want to RETAIN the
    // CLEAR event in the ring for audit, so we save the ring and restore.
    sentai_safety_event_t  saved_evt[SENTAI_SAFETY_EVENTS_CAP];
    memcpy(saved_evt, S.evt, sizeof(saved_evt));
    uint32_t saved_head  = S.evt_head;
    uint32_t saved_count = S.evt_count;
    S = Manager{};
    S.inited = true;
    memcpy(S.evt, saved_evt, sizeof(saved_evt));
    S.evt_head  = saved_head;
    S.evt_count = saved_count;
    return 0;
}

// ── enable / disable ───────────────────────────────────────────────────
extern "C" int sentai_safety_enable(sentai_safety_check_t check,
                                     const void* params) {
    mu_init();
    MuGuard g;
    if (!S.inited) { S = Manager{}; S.inited = true; }
    switch (check) {
    case SENTAI_SAFETY_CHK_ARUCO: {
        if (!params) return SENTAI_SAFETY_ERR_PARAMS;
        auto* p = static_cast<const sentai_safety_aruco_params_t*>(params);
        if (p->n_min < 1 || p->n_min > 16) return SENTAI_SAFETY_ERR_PARAMS;
        if (!(isfinite(p->max_loss_s)) || p->max_loss_s <= 0.f)
            return SENTAI_SAFETY_ERR_PARAMS;
        // Re-enable: overwrite params + reset streak, but DON'T touch
        // aborted flag (per spec — only clear() resets it).
        S.aruco.enabled        = true;
        S.aruco.params         = *p;
        S.aruco.last_seq       = 0;
        S.aruco.last_push_t_ms = 0;
        S.aruco.last_n_dets    = 0;
        S.aruco.in_lt_streak   = false;
        S.aruco.streak_start_ms= 0;
        log_event_(0, SENTAI_SAFETY_CHK_ARUCO,
                    SENTAI_SAFETY_EV_ENABLE, nullptr);
        return SENTAI_SAFETY_OK;
    }
    case SENTAI_SAFETY_CHK_ALT_FLOOR: S.alt_floor.enabled = true; goto stub_ok;
    case SENTAI_SAFETY_CHK_EKF_CEIL:  S.ekf_ceil.enabled  = true; goto stub_ok;
    case SENTAI_SAFETY_CHK_BATTERY:   S.battery.enabled   = true; goto stub_ok;
    case SENTAI_SAFETY_CHK_LINK:      S.link.enabled      = true; goto stub_ok;
    default: return SENTAI_SAFETY_ERR_UNKNOWN;
    }
stub_ok:
    log_event_(0, check, SENTAI_SAFETY_EV_ENABLE, nullptr);
    return SENTAI_SAFETY_OK;
}

extern "C" int sentai_safety_disable(sentai_safety_check_t check) {
    mu_init();
    MuGuard g;
    switch (check) {
    case SENTAI_SAFETY_CHK_ARUCO:     S.aruco.enabled     = false; break;
    case SENTAI_SAFETY_CHK_ALT_FLOOR: S.alt_floor.enabled = false; break;
    case SENTAI_SAFETY_CHK_EKF_CEIL:  S.ekf_ceil.enabled  = false; break;
    case SENTAI_SAFETY_CHK_BATTERY:   S.battery.enabled   = false; break;
    case SENTAI_SAFETY_CHK_LINK:      S.link.enabled      = false; break;
    default: return SENTAI_SAFETY_ERR_UNKNOWN;
    }
    log_event_(0, check, SENTAI_SAFETY_EV_DISABLE, nullptr);
    return SENTAI_SAFETY_OK;
}

// ── PUSH endpoint: ArUco result ────────────────────────────────────────
extern "C" int sentai_safety_on_aruco_result(int n_dets,
                                              uint32_t frame_seq,
                                              uint32_t ts_ms) {
    mu_init();
    MuGuard g;
    if (!S.inited || !S.aruco.enabled) return 0;
    if (n_dets < 0) n_dets = 0;
    // Idempotent: same frame_seq twice = ignore the duplicate.
    if (frame_seq != 0 && frame_seq == S.aruco.last_seq) return 0;
    S.aruco.last_seq       = frame_seq;
    S.aruco.last_push_t_ms = ts_ms;
    S.aruco.last_n_dets    = n_dets;
    S.aruco.n_frames++;
    const int n_min = S.aruco.params.n_min;
    const uint32_t max_loss_ms = static_cast<uint32_t>(
                                   S.aruco.params.max_loss_s * 1000.0f);
    if (n_dets < n_min) {
        if (!S.aruco.in_lt_streak) {
            S.aruco.in_lt_streak    = true;
            S.aruco.streak_start_ms = ts_ms;
            sentai_safety_event_t ev{};
            ev.d.aruco_lt.n_dets = n_dets;
            ev.d.aruco_lt.n_min  = n_min;
            log_event_(ts_ms, SENTAI_SAFETY_CHK_ARUCO,
                        SENTAI_SAFETY_EV_LT_N_MIN, &ev);
        } else {
            uint32_t streak_ms = (ts_ms >= S.aruco.streak_start_ms)
                                  ? (ts_ms - S.aruco.streak_start_ms) : 0;
            if (streak_ms >= max_loss_ms && !S.aborted) {
                sentai_safety_event_t ev{};
                ev.d.aruco_abort.streak_ms = streak_ms;
                ev.d.aruco_abort.n_dets    = n_dets;
                log_event_(ts_ms, SENTAI_SAFETY_CHK_ARUCO,
                            SENTAI_SAFETY_EV_ABORT, &ev);
                char msg[SENTAI_SAFETY_REASON_LEN];
                snprintf(msg, sizeof(msg),
                          "aruco: n_dets=%d < %d for %u ms >= %u ms",
                          n_dets, n_min,
                          static_cast<unsigned>(streak_ms),
                          static_cast<unsigned>(max_loss_ms));
                latch_abort_(SENTAI_SAFETY_CHK_ARUCO, ts_ms, msg);
            }
        }
    } else {
        // n_dets >= n_min → reset streak.  Fluke is OK per operator
        // ([[flowbaseline2-4markers-abort]]): "e ok ca fluke sa salveze
        // misiunia, nu e grav".
        if (S.aruco.in_lt_streak) {
            uint32_t streak_ms = (ts_ms >= S.aruco.streak_start_ms)
                                  ? (ts_ms - S.aruco.streak_start_ms) : 0;
            sentai_safety_event_t ev{};
            ev.d.aruco_recover.prev_streak_ms = streak_ms;
            log_event_(ts_ms, SENTAI_SAFETY_CHK_ARUCO,
                        SENTAI_SAFETY_EV_RECOVER, &ev);
        }
        S.aruco.in_lt_streak    = false;
        S.aruco.streak_start_ms = 0;
    }
    return 0;
}

// ── Future-stub push endpoints (T-future implementation) ───────────────
extern "C" int sentai_safety_on_alt_pose(float z_pnp_m, uint32_t ts_ms) {
    (void)z_pnp_m; (void)ts_ms;
    mu_init(); MuGuard g;
    if (S.alt_floor.enabled) S.alt_floor.last_push_t_ms = ts_ms;
    return 0;
}
extern "C" int sentai_safety_on_ekf_state(float ekf_z_m, uint32_t ts_ms) {
    (void)ekf_z_m; (void)ts_ms;
    mu_init(); MuGuard g;
    if (S.ekf_ceil.enabled) S.ekf_ceil.last_push_t_ms = ts_ms;
    return 0;
}
extern "C" int sentai_safety_on_battery(float v_batt_v, uint32_t ts_ms) {
    (void)v_batt_v; (void)ts_ms;
    mu_init(); MuGuard g;
    if (S.battery.enabled) S.battery.last_push_t_ms = ts_ms;
    return 0;
}
extern "C" int sentai_safety_on_link_keepalive(uint32_t ts_ms) {
    mu_init(); MuGuard g;
    if (S.link.enabled) S.link.last_push_t_ms = ts_ms;
    return 0;
}

// ── Tick: stale-feed watchdog ──────────────────────────────────────────
extern "C" int sentai_safety_tick(uint32_t now_ms) {
    mu_init();
    MuGuard g;
    if (!S.inited) return 0;
    const uint32_t stale_ms = static_cast<uint32_t>(
                                SENTAI_SAFETY_STALE_TIMEOUT_S * 1000.0f);
    auto check_stale = [&](bool enabled, uint32_t last_t,
                            sentai_safety_check_t kind, const char* name) {
        if (!enabled || S.aborted) return;
        // Skip stale check until feeder has pushed at least once
        // (otherwise enable() would instantly trip).
        if (last_t == 0) return;
        uint32_t silent_ms = (now_ms >= last_t) ? (now_ms - last_t) : 0;
        if (silent_ms >= stale_ms) {
            sentai_safety_event_t ev{};
            ev.d.stale.silent_ms = silent_ms;
            log_event_(now_ms, kind, SENTAI_SAFETY_EV_STALE, &ev);
            char msg[SENTAI_SAFETY_REASON_LEN];
            snprintf(msg, sizeof(msg),
                      "%s: feeder silent for %u ms (>= %u ms timeout)",
                      name, static_cast<unsigned>(silent_ms),
                      static_cast<unsigned>(stale_ms));
            latch_abort_(kind, now_ms, msg);
        }
    };
    check_stale(S.aruco.enabled,     S.aruco.last_push_t_ms,
                SENTAI_SAFETY_CHK_ARUCO,     "aruco");
    check_stale(S.alt_floor.enabled, S.alt_floor.last_push_t_ms,
                SENTAI_SAFETY_CHK_ALT_FLOOR, "alt_floor");
    check_stale(S.ekf_ceil.enabled,  S.ekf_ceil.last_push_t_ms,
                SENTAI_SAFETY_CHK_EKF_CEIL,  "ekf_ceil");
    check_stale(S.battery.enabled,   S.battery.last_push_t_ms,
                SENTAI_SAFETY_CHK_BATTERY,   "battery");
    check_stale(S.link.enabled,      S.link.last_push_t_ms,
                SENTAI_SAFETY_CHK_LINK,      "link");
    return 0;
}

// ── Read-side ──────────────────────────────────────────────────────────
extern "C" int sentai_safety_snapshot(sentai_safety_snapshot_t* out) {
    if (!out) return SENTAI_SAFETY_ERR_PARAMS;
    mu_init();
    MuGuard g;
    memset(out, 0, sizeof(*out));
    out->aborted     = S.aborted;
    out->active_mask = compute_active_mask_();
    out->abort_kind  = static_cast<uint8_t>(S.abort_kind);
    out->abort_t_ms  = S.abort_t_ms;
    size_t n = strlen(S.reason);
    if (n >= sizeof(out->reason)) n = sizeof(out->reason) - 1;
    memcpy(out->reason, S.reason, n);
    out->reason[n] = 0;
    out->aruco_last_n_dets        = S.aruco.last_n_dets;
    out->aruco_last_push_t_ms     = S.aruco.last_push_t_ms;
    out->aruco_n_frames_processed = S.aruco.n_frames;
    if (S.aruco.in_lt_streak && S.aruco.last_push_t_ms >= S.aruco.streak_start_ms) {
        out->aruco_streak_ms = S.aruco.last_push_t_ms - S.aruco.streak_start_ms;
    } else {
        out->aruco_streak_ms = 0;
    }
    return 0;
}

extern "C" int sentai_safety_is_aborted(void) {
    // Lock-free read on byte; safe per platform memory model assumptions.
    return S.aborted ? 1 : 0;
}

extern "C" const char* sentai_safety_reason(void) {
    // No lock — reason string is written under mutex but readers see
    // either old or new full string; partial overwrite would be at most
    // up to SENTAI_SAFETY_REASON_LEN-1 bytes (always null-terminated).
    return S.reason;
}

extern "C" int sentai_safety_get_events(sentai_safety_event_t* out, int cap) {
    if (!out || cap <= 0) return 0;
    mu_init();
    MuGuard g;
    int n_avail = (S.evt_count < SENTAI_SAFETY_EVENTS_CAP)
                  ? static_cast<int>(S.evt_count)
                  : SENTAI_SAFETY_EVENTS_CAP;
    int n_emit = (n_avail < cap) ? n_avail : cap;
    // Emit oldest-first.  Oldest index = (evt_head - n_avail) mod cap.
    uint32_t start = (S.evt_head + SENTAI_SAFETY_EVENTS_CAP - n_avail)
                      % SENTAI_SAFETY_EVENTS_CAP;
    for (int i = 0; i < n_emit; ++i) {
        uint32_t idx = (start + i) % SENTAI_SAFETY_EVENTS_CAP;
        out[i] = S.evt[idx];
    }
    return n_emit;
}
