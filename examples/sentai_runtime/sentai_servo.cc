// sentai_servo.cc — ObjectsPlan L4 (Stage 4) skeleton implementation.
//
// Pure FSM + trace ring.  No transport bytes are sent in L4; Stage 4.A
// will replace the per-action `record_*` no-ops with real
// sentai_link / sentai_crazy calls.  One .cc compiles for ARM
// (sentai_runtime) and SIM (sentai_sim) — selected via the same
// SENTAI_PLATFORM_SIM / __arm__ guard L2 and L3 use.

#include "sentai_servo.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t srv_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  #define SENTAI_SRV_SDRAM_BSS   /* nothing */
  #define SENTAI_SRV_SDRAM_TEXT  /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t srv_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  #define SENTAI_SRV_SDRAM_BSS   __attribute__((section(".sdram_bss"), aligned(4)))
  // Per [[itcm-budget]]: new ARM code defaults to .sdram_text.  L4 is a
  // pure cold-path FSM (every action is MP-task driven, ≪ kHz), so ITCM
  // placement would waste the tight m_text margin.  noinline keeps the
  // section attribute from being eliminated by LTO/inlining.
  #define SENTAI_SRV_SDRAM_TEXT  __attribute__((section(".sdram_text"), noinline))
#endif

// ---- State (mirror L2/L3: big static buffers in SDRAM) -----------------
typedef struct {
    sentai_servo_status_t s;
    // Ring as flat array; head = next write index; count saturates at DEPTH.
    uint16_t              head;
    sentai_servo_trace_t  ring[SENTAI_SERVO_TRACE_DEPTH];
} servo_fsm_t;

SENTAI_SRV_SDRAM_BSS static servo_fsm_t g_fsm;

// ---- Limits (per ideas/objects_plan.md Stage 4 + L4 handoff) -----------
#define SRV_ALT_MAX_M    30.0f
#define SRV_ALT_MIN_M    0.0f      // open interval — alt must be > 0
#define SRV_STEP_MAX_M   5.0f      // |dx|,|dy|,|dz| absolute cap
#define SRV_DYAW_MAX_RAD 1.5707964f /* π/2 */

// ---- Helpers -----------------------------------------------------------

static inline int srv_finite4(float a, float b, float c, float d) {
    return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d);
}

static SENTAI_SRV_SDRAM_TEXT void srv_ring_push(uint8_t action, int8_t result,
                                                float p0, float p1, float p2, float p3) {
    sentai_servo_trace_t* e = &g_fsm.ring[g_fsm.head];
    e->seq      = g_fsm.s.seq;
    e->t_ms     = srv_now_ms();
    e->action   = action;
    e->result   = result;
    e->backend  = g_fsm.s.backend;
    e->flight   = g_fsm.s.flight;
    e->param[0] = p0;
    e->param[1] = p1;
    e->param[2] = p2;
    e->param[3] = p3;

    g_fsm.head = (uint16_t)((g_fsm.head + 1) % SENTAI_SERVO_TRACE_DEPTH);
    if (g_fsm.s.trace_count < SENTAI_SERVO_TRACE_DEPTH) {
        g_fsm.s.trace_count++;
    } else {
        g_fsm.s.trace_overwrites++;
    }

    g_fsm.s.last_action      = action;
    g_fsm.s.last_result      = result;
    g_fsm.s.t_last_action_ms = e->t_ms;
}

static SENTAI_SRV_SDRAM_TEXT int8_t srv_classify_and_count(int8_t result) {
    if (result == 0) {
        g_fsm.s.actions_ok++;
    } else if (result == -1) {
        g_fsm.s.faults_no_backend++;
    } else if (result == -2) {
        g_fsm.s.faults_not_armed++;
    } else if (result == -3) {
        g_fsm.s.faults_oob++;
    }
    return result;
}

// ---- Public API --------------------------------------------------------

SENTAI_SRV_SDRAM_TEXT int sentai_servo_init(uint8_t backend) {
    if (backend != SERVO_BACKEND_SIM &&
        backend != SERVO_BACKEND_CF2 &&
        backend != SERVO_BACKEND_PX4) {
        // Bad backend — state untouched (do not record; nothing was initialised).
        return -1;
    }
    g_fsm.s.backend = backend;
    g_fsm.s.armed   = 0;
    g_fsm.s.flight  = SERVO_FLIGHT_GROUND;
    g_fsm.s.seq++;
    srv_ring_push(SERVO_ACT_INIT, 0, (float)backend, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_arm(void) {
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        g_fsm.s.seq++;
        srv_ring_push(SERVO_ACT_ARM, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    g_fsm.s.armed = 1;
    g_fsm.s.seq++;
    srv_ring_push(SERVO_ACT_ARM, 0, 0.f, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_disarm(void) {
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        g_fsm.s.seq++;
        srv_ring_push(SERVO_ACT_DISARM, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    g_fsm.s.armed  = 0;
    // Safe abort: if we were AIRBORNE, drop back to GROUND so the FSM
    // can't get stuck claiming flight while the transport is disarmed.
    g_fsm.s.flight = SERVO_FLIGHT_GROUND;
    g_fsm.s.seq++;
    srv_ring_push(SERVO_ACT_DISARM, 0, 0.f, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_takeoff(float alt_m) {
    g_fsm.s.seq++;
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        srv_ring_push(SERVO_ACT_TAKEOFF, -1, alt_m, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    if (!g_fsm.s.armed) {
        srv_ring_push(SERVO_ACT_TAKEOFF, -2, alt_m, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-2);
    }
    if (!isfinite(alt_m) ||
        alt_m <= SRV_ALT_MIN_M || alt_m > SRV_ALT_MAX_M ||
        g_fsm.s.flight != SERVO_FLIGHT_GROUND) {
        srv_ring_push(SERVO_ACT_TAKEOFF, -3, alt_m, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-3);
    }
    g_fsm.s.flight = SERVO_FLIGHT_AIRBORNE;
    srv_ring_push(SERVO_ACT_TAKEOFF, 0, alt_m, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_move(float dx, float dy, float dz, float dyaw) {
    g_fsm.s.seq++;
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        srv_ring_push(SERVO_ACT_MOVE, -1, dx, dy, dz, dyaw);
        return srv_classify_and_count(-1);
    }
    if (!g_fsm.s.armed) {
        srv_ring_push(SERVO_ACT_MOVE, -2, dx, dy, dz, dyaw);
        return srv_classify_and_count(-2);
    }
    if (g_fsm.s.flight != SERVO_FLIGHT_AIRBORNE ||
        !srv_finite4(dx, dy, dz, dyaw) ||
        fabsf(dx)   > SRV_STEP_MAX_M  ||
        fabsf(dy)   > SRV_STEP_MAX_M  ||
        fabsf(dz)   > SRV_STEP_MAX_M  ||
        fabsf(dyaw) > SRV_DYAW_MAX_RAD) {
        srv_ring_push(SERVO_ACT_MOVE, -3, dx, dy, dz, dyaw);
        return srv_classify_and_count(-3);
    }
    srv_ring_push(SERVO_ACT_MOVE, 0, dx, dy, dz, dyaw);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_hover(void) {
    g_fsm.s.seq++;
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        srv_ring_push(SERVO_ACT_HOVER, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    if (!g_fsm.s.armed) {
        srv_ring_push(SERVO_ACT_HOVER, -2, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-2);
    }
    if (g_fsm.s.flight != SERVO_FLIGHT_AIRBORNE) {
        srv_ring_push(SERVO_ACT_HOVER, -3, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-3);
    }
    srv_ring_push(SERVO_ACT_HOVER, 0, 0.f, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_land(void) {
    g_fsm.s.seq++;
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        srv_ring_push(SERVO_ACT_LAND, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    if (!g_fsm.s.armed) {
        srv_ring_push(SERVO_ACT_LAND, -2, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-2);
    }
    if (g_fsm.s.flight != SERVO_FLIGHT_AIRBORNE) {
        srv_ring_push(SERVO_ACT_LAND, -3, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-3);
    }
    g_fsm.s.flight = SERVO_FLIGHT_GROUND;
    srv_ring_push(SERVO_ACT_LAND, 0, 0.f, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT void sentai_servo_status(sentai_servo_status_t* out_status) {
    if (out_status) *out_status = g_fsm.s;
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_trace(sentai_servo_trace_t* out, int max) {
    if (out == NULL || max <= 0) return 0;
    int n = (int)g_fsm.s.trace_count;
    if (n > max) n = max;
    // Oldest entry index: when count < DEPTH, oldest = 0; otherwise head
    // points at the oldest (next slot to overwrite).
    int oldest;
    if (g_fsm.s.trace_count < SENTAI_SERVO_TRACE_DEPTH) {
        oldest = 0;
    } else {
        oldest = g_fsm.head;
    }
    for (int i = 0; i < n; i++) {
        int src = (oldest + i) % SENTAI_SERVO_TRACE_DEPTH;
        out[i] = g_fsm.ring[src];
    }
    return n;
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_clear_trace(void) {
    int n = (int)g_fsm.s.trace_count;
    memset(g_fsm.ring, 0, sizeof(g_fsm.ring));
    g_fsm.head             = 0;
    g_fsm.s.trace_count    = 0;
    // Lifetime counters + FSM state preserved (per concurrency contract).
    return n;
}
