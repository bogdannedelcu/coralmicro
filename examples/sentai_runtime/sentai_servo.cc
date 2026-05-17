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

// Stage 4.A transport headers.  These are declared even on platforms
// where the backend is never exercised — the dispatcher needs the
// prototypes to compile.  On SIM both crazy + link are available.  On
// ARM crazy is available; link is the MAVLink bridge.
#include "sentai_crazy.h"
#include "sentai_crazy_log.h"
// All link.* symbols are weak so ARM (which lacks the SIM MAVLink
// bridge) still links — the PX4 backend stays unreachable on ARM until
// sentai_link.cc gains these wrappers.  Calling a NULL weak symbol is
// guarded inside srv_xport_* via explicit checks.
extern "C" int sentai_link_cmd_arm(int do_arm)             __attribute__((weak));
extern "C" int sentai_link_cmd_takeoff(float altitude_m)   __attribute__((weak));
extern "C" int sentai_link_cmd_land(void)                  __attribute__((weak));
extern "C" int sentai_link_cmd_move(float dx, float dy, float dz, float dyaw)
    __attribute__((weak));
extern "C" int sentai_link_pose(float* x, float* y, float* z, float* yaw)
    __attribute__((weak));
extern "C" int sentai_link_pose_subscribe(int period_ms)
    __attribute__((weak));

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
    // Stage 4.A: per-action durations (HL Commander expects them in
    // seconds; mission constants like WAYPOINT_DUR=6.0 plug in here).
    float                 takeoff_dur;
    float                 move_dur;
    float                 land_dur;
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
    } else if (result == SERVO_FAULT_TX_FAIL) {
        g_fsm.s.faults_oob++;   // shared bucket — transport NAK ~ "bad call"
    }
    return result;
}

// ---- Stage 4.A: transport dispatch helpers ----------------------------
// One function per FSM action; returns 0 on success / fail-loud on
// error.  SIM backend (skeleton) always returns 0 — record-only.
// CF2 + PX4 backends route to sentai_crazy.* / sentai_link.* respectively.

// PX4 link cmd functions return 1=success / 0=failure (long-standing
// convention in sentai_link_sim.cc, predates the servo unification).
// Convert to servo's 0=ok / -1=fail convention so dispatch stays clean.
static inline int srv_link_norm(int link_rc) {
    return (link_rc > 0) ? 0 : -1;
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_arm(void) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM: return 0;
        case SERVO_BACKEND_CF2: return sentai_crazy_arm();
        case SERVO_BACKEND_PX4:
            return sentai_link_cmd_arm ? srv_link_norm(sentai_link_cmd_arm(1)) : -1;
        default: return -1;
    }
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_disarm(void) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM: return 0;
        case SERVO_BACKEND_CF2: return sentai_crazy_disarm();
        case SERVO_BACKEND_PX4:
            return sentai_link_cmd_arm ? srv_link_norm(sentai_link_cmd_arm(0)) : -1;
        default: return -1;
    }
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_takeoff(float alt_m) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM: (void)alt_m; return 0;
        case SERVO_BACKEND_CF2:
            // height, duration, yaw=0, use_current_yaw=1, group=0
            return sentai_crazy_takeoff(alt_m, g_fsm.takeoff_dur, 0.0f, 1, 0);
        case SERVO_BACKEND_PX4:
            return sentai_link_cmd_takeoff ?
                   srv_link_norm(sentai_link_cmd_takeoff(alt_m)) : -1;
        default: return -1;
    }
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_move(float dx, float dy, float dz, float dyaw) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM:
            (void)dx; (void)dy; (void)dz; (void)dyaw; return 0;
        case SERVO_BACKEND_CF2:
            // CF2 HL Commander GO_TO with relative=1 → polynomial trajectory
            // from current pose by (dx,dy,dz) with yaw delta dyaw over move_dur.
            return sentai_crazy_go_to(dx, dy, dz, dyaw, g_fsm.move_dur,
                                       /* relative */ 1,
                                       /* linear   */ 0,
                                       /* group    */ 0);
        case SERVO_BACKEND_PX4:
            return sentai_link_cmd_move ?
                   sentai_link_cmd_move(dx, dy, dz, dyaw) : -1;
        default: return -1;
    }
}

// Absolute waypoint dispatch.  CF2: HL Commander GO_TO with relative=0.
// PX4: SET_POSITION_TARGET_LOCAL_NED frame=LOCAL_NED (1) — needs current
// pose subtracted to compute the relative move via cmd_move().  We do
// the conversion here so the link layer stays single-purpose.
static SENTAI_SRV_SDRAM_TEXT int srv_xport_go_to(float x, float y, float z, float yaw) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM:
            (void)x; (void)y; (void)z; (void)yaw; return 0;
        case SERVO_BACKEND_CF2:
            return sentai_crazy_go_to(x, y, z, yaw, g_fsm.move_dur,
                                       /* relative */ 0,
                                       /* linear   */ 0,
                                       /* group    */ 0);
        case SERVO_BACKEND_PX4: {
            if (!sentai_link_cmd_move || !sentai_link_pose) return -1;
            float cx=0, cy=0, cz=0, cyaw=0;
            if (sentai_link_pose(&cx, &cy, &cz, &cyaw) != 0) return -1;
            return srv_link_norm(sentai_link_cmd_move(
                x - cx, y - cy, z - cz, yaw - cyaw));
        }
        default: return -1;
    }
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_hover(void) {
    // HOVER is a no-op for CF2 (HL Commander already holds position
    // after any GO_TO/TAKEOFF) and for PX4 (LOITER mode is implicit).
    // SIM: record-only.  All backends OK.
    return 0;
}

static SENTAI_SRV_SDRAM_TEXT int srv_xport_land(void) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM: return 0;
        case SERVO_BACKEND_CF2:
            return sentai_crazy_land(0.0f, g_fsm.land_dur, 0.0f, 1, 0);
        case SERVO_BACKEND_PX4:
            return sentai_link_cmd_land ?
                   srv_link_norm(sentai_link_cmd_land()) : -1;
        default: return -1;
    }
}

// Pose subscribe — invoked once at init() so subsequent pose() reads
// are non-blocking.  SIM backend has no pose stream; returns 0 and the
// pose() call later returns -2 (no data yet).
static SENTAI_SRV_SDRAM_TEXT int srv_xport_pose_subscribe(int period_ms) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_SIM: (void)period_ms; return 0;
        case SERVO_BACKEND_CF2: return sentai_crazy_pose_subscribe(period_ms);
        case SERVO_BACKEND_PX4:
            return sentai_link_pose_subscribe ?
                   sentai_link_pose_subscribe(period_ms) : 0;
        default: return -1;
    }
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
    // Defaults if set_durations() was never called.  Tuned empirically
    // for cf2 SITL (per s147-s151 migration notes): slow move dur keeps
    // peak velocity under flow-tracking budget.
    if (g_fsm.takeoff_dur <= 0.0f) g_fsm.takeoff_dur = 2.0f;
    if (g_fsm.move_dur    <= 0.0f) g_fsm.move_dur    = 6.0f;
    if (g_fsm.land_dur    <= 0.0f) g_fsm.land_dur    = 2.0f;
    // Stage 4.A: kick the pose subscription so subsequent pose() reads
    // are warm.  SIM backend is a no-op.  CF2/PX4 hit the wire.
    //
    // Return value DISCARDED (per embeded.md §2 rule 5: justification is
    // mandatory).  Subscription is best-effort — if it fails (cf2 SITL
    // not running, or PX4 not connected yet) the FSM still initialises
    // and missions can re-subscribe later (e.g. after sentai.crazy.init()
    // succeeds in a deferred path).  Failure surfaces via pose_ready()
    // returning 0 and pose() returning None.
    (void)srv_xport_pose_subscribe(100);
    srv_ring_push(SERVO_ACT_INIT, 0, (float)backend, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_set_durations(float takeoff_dur,
                                                     float move_dur,
                                                     float land_dur) {
    if (!isfinite(takeoff_dur) || takeoff_dur <= 0.0f) return -1;
    if (!isfinite(move_dur)    || move_dur    <= 0.0f) return -1;
    if (!isfinite(land_dur)    || land_dur    <= 0.0f) return -1;
    g_fsm.takeoff_dur = takeoff_dur;
    g_fsm.move_dur    = move_dur;
    g_fsm.land_dur    = land_dur;
    return 0;
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_pose(float* out_x, float* out_y,
                                            float* out_z, float* out_yaw) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_NONE: return -1;
        case SERVO_BACKEND_SIM:  return -2;   // no pose stream in SIM skeleton
        case SERVO_BACKEND_CF2:  return sentai_crazy_pose(out_x, out_y, out_z, out_yaw);
        case SERVO_BACKEND_PX4:
            return sentai_link_pose ? sentai_link_pose(out_x, out_y, out_z, out_yaw) : -2;
        default: return -1;
    }
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_pose_ready(void) {
    switch (g_fsm.s.backend) {
        case SERVO_BACKEND_CF2: return sentai_crazy_pose_ready();
        case SERVO_BACKEND_PX4: {
            // pose_ready() check is "have we ever received a frame?".  For
            // PX4 we approximate via a non-NULL out value from pose().
            float x=0,y=0,z=0,yaw=0;
            if (sentai_link_pose && sentai_link_pose(&x,&y,&z,&yaw) == 0) return 1;
            return 0;
        }
        default: return 0;
    }
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_arm(void) {
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        g_fsm.s.seq++;
        srv_ring_push(SERVO_ACT_ARM, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    g_fsm.s.seq++;
    int tx = srv_xport_arm();
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_ARM, SERVO_FAULT_TX_FAIL, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
    }
    g_fsm.s.armed = 1;
    srv_ring_push(SERVO_ACT_ARM, 0, 0.f, 0.f, 0.f, 0.f);
    return srv_classify_and_count(0);
}

SENTAI_SRV_SDRAM_TEXT int sentai_servo_disarm(void) {
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        g_fsm.s.seq++;
        srv_ring_push(SERVO_ACT_DISARM, -1, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(-1);
    }
    g_fsm.s.seq++;
    // Always update FSM even if transport NAKs — disarm is a safety
    // abort and the local-state invariant must hold either way.  A
    // failed TX is still logged (result=-4) so the operator can audit.
    int tx = srv_xport_disarm();
    g_fsm.s.armed  = 0;
    g_fsm.s.flight = SERVO_FLIGHT_GROUND;
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_DISARM, SERVO_FAULT_TX_FAIL, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
    }
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
    int tx = srv_xport_takeoff(alt_m);
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_TAKEOFF, SERVO_FAULT_TX_FAIL, alt_m, 0.f, 0.f, 0.f);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
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
    int tx = srv_xport_move(dx, dy, dz, dyaw);
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_MOVE, SERVO_FAULT_TX_FAIL, dx, dy, dz, dyaw);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
    }
    srv_ring_push(SERVO_ACT_MOVE, 0, dx, dy, dz, dyaw);
    return srv_classify_and_count(0);
}

#define SRV_ABS_MAX_M    30.0f    // |x|,|y|,|z| absolute cap for go_to

SENTAI_SRV_SDRAM_TEXT int sentai_servo_go_to(float x, float y, float z, float yaw) {
    g_fsm.s.seq++;
    if (g_fsm.s.backend == SERVO_BACKEND_NONE) {
        srv_ring_push(SERVO_ACT_GO_TO, -1, x, y, z, yaw);
        return srv_classify_and_count(-1);
    }
    if (!g_fsm.s.armed) {
        srv_ring_push(SERVO_ACT_GO_TO, -2, x, y, z, yaw);
        return srv_classify_and_count(-2);
    }
    if (g_fsm.s.flight != SERVO_FLIGHT_AIRBORNE ||
        !srv_finite4(x, y, z, yaw) ||
        fabsf(x) > SRV_ABS_MAX_M ||
        fabsf(y) > SRV_ABS_MAX_M ||
        fabsf(z) > SRV_ABS_MAX_M) {
        srv_ring_push(SERVO_ACT_GO_TO, -3, x, y, z, yaw);
        return srv_classify_and_count(-3);
    }
    int tx = srv_xport_go_to(x, y, z, yaw);
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_GO_TO, SERVO_FAULT_TX_FAIL, x, y, z, yaw);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
    }
    srv_ring_push(SERVO_ACT_GO_TO, 0, x, y, z, yaw);
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
    int tx = srv_xport_hover();
    if (tx < 0) {
        srv_ring_push(SERVO_ACT_HOVER, SERVO_FAULT_TX_FAIL, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
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
    int tx = srv_xport_land();
    if (tx < 0) {
        // Failed land bytes — keep AIRBORNE so the operator/mission can
        // retry.  TX_FAIL on land is recoverable; do not flip to GROUND
        // optimistically (caller must read the result code).
        srv_ring_push(SERVO_ACT_LAND, SERVO_FAULT_TX_FAIL, 0.f, 0.f, 0.f, 0.f);
        return srv_classify_and_count(SERVO_FAULT_TX_FAIL);
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
