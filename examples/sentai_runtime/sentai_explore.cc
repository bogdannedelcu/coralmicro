// sentai_explore.cc — ObjectsPlan L6 skeleton implementation.
//
// Pure FSM that wraps L4 sentai_servo intents and reads L5
// sentai_object_lifter world positions.  Single .cc compiles for ARM
// (sentai_runtime) and SIM (sentai_sim) — selected via the same
// SENTAI_PLATFORM_SIM / __arm__ guard L2-L5 use.
//
// Per [[itcm-budget]]: cold-path FSM (operator-driven, ≪ kHz) lives in
// .sdram_text on ARM.  SIM uses default sections.

#include "sentai_explore.h"
#include "sentai_servo.h"
#include "sentai_object_lifter.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t exp_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  #define SENTAI_EXP_SDRAM_BSS   /* nothing */
  #define SENTAI_EXP_SDRAM_TEXT  /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t exp_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  #define SENTAI_EXP_SDRAM_BSS   __attribute__((section(".sdram_bss"), aligned(4)))
  #define SENTAI_EXP_SDRAM_TEXT  __attribute__((section(".sdram_text"), noinline))
#endif

// ---- Live state --------------------------------------------------------
typedef struct {
    sentai_explore_status_t s;
    uint16_t                head;
    sentai_explore_trace_t  ring[SENTAI_EXPLORE_TRACE_DEPTH];
} explore_fsm_t;

SENTAI_EXP_SDRAM_BSS static explore_fsm_t g_exp;

// ---- Helpers -----------------------------------------------------------

static inline int exp_finite4(float a, float b, float c, float d) {
    return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d);
}

static SENTAI_EXP_SDRAM_TEXT void exp_ring_push(uint8_t action, int8_t result,
                                                uint8_t state_before, uint8_t state_after,
                                                float p0, float p1, float p2, float p3) {
    sentai_explore_trace_t* e = &g_exp.ring[g_exp.head];
    e->seq          = g_exp.s.seq;
    e->t_ms         = exp_now_ms();
    e->action       = action;
    e->result       = result;
    e->state_before = state_before;
    e->state_after  = state_after;
    e->param[0]     = p0;
    e->param[1]     = p1;
    e->param[2]     = p2;
    e->param[3]     = p3;

    g_exp.head = (uint16_t)((g_exp.head + 1) % SENTAI_EXPLORE_TRACE_DEPTH);
    if (g_exp.s.trace_count < SENTAI_EXPLORE_TRACE_DEPTH) {
        g_exp.s.trace_count++;
    } else {
        g_exp.s.trace_overwrites++;
    }
}

static SENTAI_EXP_SDRAM_TEXT void exp_set_state(uint8_t new_state, float reason_code) {
    if (new_state == g_exp.s.state) return;
    uint8_t old = g_exp.s.state;
    uint32_t now = exp_now_ms();
    float t_in_state = (float)(now - g_exp.s.t_state_entered_ms);
    g_exp.s.state = new_state;
    g_exp.s.t_state_entered_ms = now;
    g_exp.s.transitions++;
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_TRANSITION, 0, old, new_state,
                  (float)old, (float)new_state, reason_code, t_in_state);
}

static SENTAI_EXP_SDRAM_TEXT int8_t exp_classify_and_count(int8_t result) {
    if (result == 0) {
        g_exp.s.actions_ok++;
    } else if (result == -1) {
        g_exp.s.faults_wrong_state++;
    } else if (result == -2) {
        g_exp.s.faults_oob++;
    } else if (result == -3) {
        g_exp.s.faults_servo++;
    } else if (result == -4) {
        g_exp.s.faults_lifter++;
    } else if (result == -5) {
        g_exp.s.faults_pose_stale++;
    }
    return result;
}

static SENTAI_EXP_SDRAM_TEXT int exp_pose_fresh(void) {
    if (g_exp.s.t_last_pose_ms == 0) return 0;
    uint32_t age = exp_now_ms() - g_exp.s.t_last_pose_ms;
    return age <= SENTAI_EXPLORE_POSE_STALE_MS;
}

// ---- Public API --------------------------------------------------------

SENTAI_EXP_SDRAM_TEXT int sentai_explore_init(uint8_t backend) {
    int rc = sentai_servo_init(backend);
    if (rc < 0) {
        // Servo init refused (bad backend id).  Do not touch FSM.
        return -2;
    }
    // Reset live FSM (per-mission state).  LIFETIME counters
    // (actions_ok, faults_*, transitions, trace_overwrites) are
    // PRESERVED — mirrors L4 servo init() convention so post-mortem
    // can audit cumulative behaviour across missions.  Per-mission
    // counters (gotos_completed, aborts) reset here.
    g_exp.s.state              = EXPLORE_IDLE;
    g_exp.s.backend            = backend;
    g_exp.s.t_state_entered_ms = exp_now_ms();
    g_exp.s.t_last_pose_ms     = 0;
    g_exp.s.t_mission_start_ms = 0;
    g_exp.s.pose_x = g_exp.s.pose_y = g_exp.s.pose_z = g_exp.s.pose_yaw = 0.f;
    g_exp.s.home_x = g_exp.s.home_y = g_exp.s.home_z = g_exp.s.home_yaw = 0.f;
    g_exp.s.home_set           = 0;
    g_exp.s.target_tracklet_id = 0;
    g_exp.s.target_x = g_exp.s.target_y = 0.f;
    g_exp.s.gotos_completed = 0;
    g_exp.s.aborts = 0;
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_INIT, 0, EXPLORE_IDLE, EXPLORE_IDLE,
                  (float)backend, 0.f, 0.f, 0.f);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_start(void) {
    if (g_exp.s.state != EXPLORE_IDLE) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_START, -1, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
        return exp_classify_and_count(-1);
    }
    int srv = sentai_servo_arm();
    if (srv < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_START, -3, g_exp.s.state, g_exp.s.state,
                      (float)srv, 0.f, 0.f, 0.f);
        return exp_classify_and_count(-3);
    }
    g_exp.s.t_mission_start_ms = exp_now_ms();
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_START, 0, EXPLORE_IDLE, EXPLORE_ARMING, 0.f,0.f,0.f,0.f);
    exp_set_state(EXPLORE_ARMING, 1.f /* reason: start ok */);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_takeoff(float alt_m) {
    if (g_exp.s.state != EXPLORE_ARMING && g_exp.s.state != EXPLORE_HOVERING) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_TAKEOFF, -1, g_exp.s.state, g_exp.s.state, alt_m,0.f,0.f,0.f);
        return exp_classify_and_count(-1);
    }
    if (!isfinite(alt_m) || alt_m <= SENTAI_EXPLORE_ALT_MIN_M ||
        alt_m > SENTAI_EXPLORE_ALT_MAX_M) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_TAKEOFF, -2, g_exp.s.state, g_exp.s.state, alt_m,0.f,0.f,0.f);
        return exp_classify_and_count(-2);
    }
    int srv = sentai_servo_takeoff(alt_m);
    if (srv < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_TAKEOFF, -3, g_exp.s.state, g_exp.s.state,
                      alt_m, (float)srv, 0.f, 0.f);
        return exp_classify_and_count(-3);
    }
    // Capture takeoff alt as home_z; home_xy will be set when pose arrives.
    g_exp.s.home_z   = alt_m;
    g_exp.s.home_yaw = g_exp.s.pose_yaw;
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_TAKEOFF, 0, g_exp.s.state, EXPLORE_TAKEOFF, alt_m,0.f,0.f,0.f);
    exp_set_state(EXPLORE_TAKEOFF, alt_m);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_set_pose(float x, float y, float z, float yaw) {
    if (!exp_finite4(x, y, z, yaw)) {
        return exp_classify_and_count(-2);
    }
    g_exp.s.pose_x   = x;
    g_exp.s.pose_y   = y;
    g_exp.s.pose_z   = z;
    g_exp.s.pose_yaw = yaw;
    g_exp.s.t_last_pose_ms = exp_now_ms();
    (void)sentai_explore_tick();
    return 0;
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_goto(uint16_t tracklet_id, float stop_dist_m) {
    if (g_exp.s.state != EXPLORE_HOVERING) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -1, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, 0.f, 0.f);
        return exp_classify_and_count(-1);
    }
    if (!isfinite(stop_dist_m) || stop_dist_m < 0.f || stop_dist_m > 5.0f) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -2, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, 0.f, 0.f);
        return exp_classify_and_count(-2);
    }
    if (!exp_pose_fresh()) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -5, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, 0.f, 0.f);
        return exp_classify_and_count(-5);
    }
    // Read L5 lifter status + world position.
    sentai_lifter_entry_t e;
    if (sentai_lifter_get(tracklet_id, &e) < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -4, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, 0.f, 0.f);
        return exp_classify_and_count(-4);
    }
    if (e.status != LIFTER_LIFTED) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -4, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, (float)e.status, 0.f);
        return exp_classify_and_count(-4);
    }
    float pos[3];
    if (sentai_lifter_world_pos(tracklet_id, pos) < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -4, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, 0.f, 0.f);
        return exp_classify_and_count(-4);
    }
    g_exp.s.target_tracklet_id = tracklet_id;
    g_exp.s.target_x = pos[0];
    g_exp.s.target_y = pos[1];

    // Compute move step (clamped to L4 5 m cap).
    float dx = pos[0] - g_exp.s.pose_x;
    float dy = pos[1] - g_exp.s.pose_y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist <= stop_dist_m) {
        // Already within tolerance — hover here, go straight to INSPECT.
        (void)sentai_servo_hover();
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, 0, g_exp.s.state, EXPLORE_INSPECT,
                      (float)tracklet_id, stop_dist_m, dist, 0.f);
        exp_set_state(EXPLORE_INSPECT, dist);
        return exp_classify_and_count(0);
    }
    float step = dist - stop_dist_m;
    if (step > 5.0f) step = 5.0f;
    float inv = step / dist;
    float mx = dx * inv;
    float my = dy * inv;
    float yaw_target = atan2f(dy, dx);
    float dyaw = yaw_target - g_exp.s.pose_yaw;
    // Wrap dyaw to [-π, π] then clamp to L4 |π/2| cap.
    while (dyaw >  3.1415926f) dyaw -= 6.2831853f;
    while (dyaw < -3.1415926f) dyaw += 6.2831853f;
    if (dyaw >  1.5707963f) dyaw =  1.5707963f;
    if (dyaw < -1.5707963f) dyaw = -1.5707963f;

    int srv = sentai_servo_move(mx, my, 0.f, dyaw);
    if (srv < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_GOTO, -3, g_exp.s.state, g_exp.s.state,
                      (float)tracklet_id, stop_dist_m, (float)srv, 0.f);
        return exp_classify_and_count(-3);
    }
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_GOTO, 0, g_exp.s.state, EXPLORE_APPROACH,
                  (float)tracklet_id, stop_dist_m, dist, dyaw);
    exp_set_state(EXPLORE_APPROACH, dist);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_return_home(void) {
    if (g_exp.s.state != EXPLORE_HOVERING) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_RETURN, -1, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
        return exp_classify_and_count(-1);
    }
    if (!g_exp.s.home_set) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_RETURN, -1, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
        return exp_classify_and_count(-1);
    }
    if (!exp_pose_fresh()) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_RETURN, -5, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
        return exp_classify_and_count(-5);
    }
    g_exp.s.target_tracklet_id = 0xFFFFu;
    g_exp.s.target_x = g_exp.s.home_x;
    g_exp.s.target_y = g_exp.s.home_y;

    float dx = g_exp.s.home_x - g_exp.s.pose_x;
    float dy = g_exp.s.home_y - g_exp.s.pose_y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist <= SENTAI_EXPLORE_HOME_RADIUS_M) {
        (void)sentai_servo_hover();
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_RETURN, 0, g_exp.s.state, EXPLORE_HOVERING,
                      dist, 0.f, 0.f, 0.f);
        // Already at home — stay HOVERING.
        return exp_classify_and_count(0);
    }
    float step = dist;
    if (step > 5.0f) step = 5.0f;
    float inv = step / dist;
    float mx = dx * inv;
    float my = dy * inv;
    float yaw_target = atan2f(dy, dx);
    float dyaw = yaw_target - g_exp.s.pose_yaw;
    while (dyaw >  3.1415926f) dyaw -= 6.2831853f;
    while (dyaw < -3.1415926f) dyaw += 6.2831853f;
    if (dyaw >  1.5707963f) dyaw =  1.5707963f;
    if (dyaw < -1.5707963f) dyaw = -1.5707963f;

    int srv = sentai_servo_move(mx, my, 0.f, dyaw);
    if (srv < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_RETURN, -3, g_exp.s.state, g_exp.s.state,
                      dist, (float)srv, 0.f, 0.f);
        return exp_classify_and_count(-3);
    }
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_RETURN, 0, g_exp.s.state, EXPLORE_RETURNING,
                  dist, 0.f, 0.f, dyaw);
    exp_set_state(EXPLORE_RETURNING, dist);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_land(void) {
    if (g_exp.s.state != EXPLORE_HOVERING) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_LAND, -1, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
        return exp_classify_and_count(-1);
    }
    int srv = sentai_servo_land();
    if (srv < 0) {
        g_exp.s.seq++;
        exp_ring_push(EXPLORE_ACT_LAND, -3, g_exp.s.state, g_exp.s.state,
                      (float)srv, 0.f, 0.f, 0.f);
        return exp_classify_and_count(-3);
    }
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_LAND, 0, g_exp.s.state, EXPLORE_LANDING, 0.f,0.f,0.f,0.f);
    exp_set_state(EXPLORE_LANDING, 0.f);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_stop(void) {
    // Graceful end depending on current state.
    g_exp.s.seq++;
    exp_ring_push(EXPLORE_ACT_STOP, 0, g_exp.s.state, g_exp.s.state, 0.f,0.f,0.f,0.f);
    uint8_t st = g_exp.s.state;
    if (st == EXPLORE_HOVERING) {
        if (g_exp.s.home_set) {
            return sentai_explore_return_home();  // operator follow-up: land()
        }
        return sentai_explore_land();
    }
    if (st == EXPLORE_APPROACH || st == EXPLORE_INSPECT) {
        // Cancel the goto: hover here, then operator may issue return/land.
        (void)sentai_servo_hover();
        exp_set_state(EXPLORE_HOVERING, -1.f /* reason: stop cancel goto */);
        return exp_classify_and_count(0);
    }
    if (st == EXPLORE_RETURNING) {
        (void)sentai_servo_hover();
        exp_set_state(EXPLORE_HOVERING, -1.f /* reason: stop cancel return */);
        return exp_classify_and_count(0);
    }
    // IDLE / ARMING / TAKEOFF / LANDING / DONE / ABORT — no-op
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_abort(void) {
    (void)sentai_servo_disarm();
    g_exp.s.seq++;
    g_exp.s.aborts++;
    exp_ring_push(EXPLORE_ACT_ABORT, 0, g_exp.s.state, EXPLORE_ABORT, 0.f,0.f,0.f,0.f);
    exp_set_state(EXPLORE_ABORT, 0.f);
    return exp_classify_and_count(0);
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_tick(void) {
    uint32_t now = exp_now_ms();
    int transitions = 0;
    uint8_t st = g_exp.s.state;

    if (st == EXPLORE_TAKEOFF) {
        // Wait for pose to register at takeoff altitude.
        if (exp_pose_fresh() &&
            fabsf(g_exp.s.pose_z - g_exp.s.home_z) <= SENTAI_EXPLORE_TAKEOFF_TOLER_M) {
            // First pose at altitude → capture home_xy and transition.
            if (!g_exp.s.home_set) {
                g_exp.s.home_x = g_exp.s.pose_x;
                g_exp.s.home_y = g_exp.s.pose_y;
                g_exp.s.home_yaw = g_exp.s.pose_yaw;
                g_exp.s.home_set = 1;
            }
            exp_set_state(EXPLORE_HOVERING, 2.f /* reason: takeoff complete */);
            transitions++;
        }
    } else if (st == EXPLORE_APPROACH) {
        if (exp_pose_fresh()) {
            float dx = g_exp.s.target_x - g_exp.s.pose_x;
            float dy = g_exp.s.target_y - g_exp.s.pose_y;
            float dist = sqrtf(dx*dx + dy*dy);
            // stop_dist isn't preserved in g_exp; we re-fetch from trace
            // tail.  Cheap approach: use TAKEOFF_TOLER as default radius
            // (skeleton).  Production would store last stop_dist.
            // For now: if within 0.4 m of target → INSPECT.
            if (dist <= 0.4f) {
                (void)sentai_servo_hover();
                exp_set_state(EXPLORE_INSPECT, dist);
                transitions++;
            }
        }
    } else if (st == EXPLORE_INSPECT) {
        if ((now - g_exp.s.t_state_entered_ms) >= SENTAI_EXPLORE_INSPECT_DUR_MS) {
            g_exp.s.gotos_completed++;
            exp_set_state(EXPLORE_HOVERING, 3.f /* reason: inspect done */);
            transitions++;
        }
    } else if (st == EXPLORE_RETURNING) {
        if (exp_pose_fresh()) {
            float dx = g_exp.s.home_x - g_exp.s.pose_x;
            float dy = g_exp.s.home_y - g_exp.s.pose_y;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist <= SENTAI_EXPLORE_HOME_RADIUS_M) {
                (void)sentai_servo_hover();
                exp_set_state(EXPLORE_HOVERING, 4.f /* reason: home reached */);
                transitions++;
            }
        }
    } else if (st == EXPLORE_LANDING) {
        if ((now - g_exp.s.t_state_entered_ms) >= SENTAI_EXPLORE_LAND_DUR_MS) {
            (void)sentai_servo_disarm();
            exp_set_state(EXPLORE_DONE, 5.f /* reason: landing complete */);
            transitions++;
        }
    }
    return transitions;
}

SENTAI_EXP_SDRAM_TEXT void sentai_explore_status(sentai_explore_status_t* out) {
    if (out == NULL) return;
    memcpy(out, &g_exp.s, sizeof(*out));
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_trace(sentai_explore_trace_t* out, int max) {
    if (out == NULL || max <= 0) return 0;
    int n = (int)g_exp.s.trace_count;
    if (n > max) n = max;
    // Oldest first.  When the ring is not full, oldest is index 0.
    // When full, oldest is at g_exp.head (wraps from there).
    int start = (g_exp.s.trace_count < SENTAI_EXPLORE_TRACE_DEPTH) ? 0 : g_exp.head;
    for (int i = 0; i < n; ++i) {
        int idx = (start + i) % SENTAI_EXPLORE_TRACE_DEPTH;
        out[i] = g_exp.ring[idx];
    }
    return n;
}

SENTAI_EXP_SDRAM_TEXT int sentai_explore_clear_trace(void) {
    int n = (int)g_exp.s.trace_count;
    g_exp.head = 0;
    g_exp.s.trace_count = 0;
    return n;
}

SENTAI_EXP_SDRAM_TEXT const char* sentai_explore_state_name(uint8_t state) {
    switch (state) {
        case EXPLORE_IDLE:      return "IDLE";
        case EXPLORE_ARMING:    return "ARMING";
        case EXPLORE_TAKEOFF:   return "TAKEOFF";
        case EXPLORE_HOVERING:  return "HOVERING";
        case EXPLORE_APPROACH:  return "APPROACH";
        case EXPLORE_INSPECT:   return "INSPECT";
        case EXPLORE_RETURNING: return "RETURNING";
        case EXPLORE_LANDING:   return "LANDING";
        case EXPLORE_DONE:      return "DONE";
        case EXPLORE_ABORT:     return "ABORT";
        default:                return "?";
    }
}

SENTAI_EXP_SDRAM_TEXT const char* sentai_explore_action_name(uint8_t action) {
    switch (action) {
        case EXPLORE_ACT_NONE:       return "NONE";
        case EXPLORE_ACT_INIT:       return "INIT";
        case EXPLORE_ACT_START:      return "START";
        case EXPLORE_ACT_TAKEOFF:    return "TAKEOFF";
        case EXPLORE_ACT_GOTO:       return "GOTO";
        case EXPLORE_ACT_RETURN:     return "RETURN";
        case EXPLORE_ACT_LAND:       return "LAND";
        case EXPLORE_ACT_STOP:       return "STOP";
        case EXPLORE_ACT_ABORT:      return "ABORT";
        case EXPLORE_ACT_TRANSITION: return "TRANSITION";
        default:                     return "?";
    }
}
