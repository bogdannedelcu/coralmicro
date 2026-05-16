// sentai_explore.h — ObjectsPlan L6: mission state machine.
// SKELETON.
//
// L6 wraps L4 `sentai_servo` intents and consumes L5
// `sentai_object_lifter` world positions to drive an
// operator-commandable, radio-friendly mission FSM:
//
//   IDLE → ARMING → TAKEOFF → HOVERING ─┬─ APPROACH → INSPECT → HOVERING
//                                       ├─ RETURNING ───────────┘
//                                       └─ LANDING → DONE
//                       (any state) ─ABORT─▶ ABORT (terminal)
//
// Operator API (via REPL, eventually over Crazyflie CRTP radio):
//   start() → takeoff(alt) → goto(obj_id) {N times} → return_home() → land()
//
// Per `ideas/objects_plan.md` §23.5 step 1 (thesis-MVP scope).  See
// experiments/s133_explore_skeleton/README.md for the full design
// rationale.
//
// L6 is **operator-driven**, not autonomous: no spiral/raster search,
// no FileX persistence, no safety FSM (that lives in `sentai.safety`,
// see [[no-safety-logic-in-explore]]).  Single-flight, single-mission.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A "system model first")
// =========================================================================
// Fault model (every public entry returns a small int code):
//   F1 API called in wrong state       → -1, faults_wrong_state++
//   F2 argument OOB (alt/stop_dist/...) → -2, faults_oob++
//   F3 L4 servo refused (propagated)   → -3, faults_servo++
//   F4 L5 lifter refused / not LIFTED  → -4, faults_lifter++
//   F5 pose stale (>2 s) on goto/RTH   → -5, faults_pose_stale++
//
// Faults are LOCAL.  FSM does NOT auto-ABORT on F3/F4 — caller decides
// whether to retry, return_home, or abort.  Per `embeded.md §F`
// (self-healing first, reboot last resort).
//
// Execution model:
//   - All public entries are MP-task context only (REPL).
//   - No ISR access.  No FreeRTOS primitives required.
//   - `tick()` is the polling entry — re-evaluates time + pose-based
//     transitions.  Each `set_pose()` calls tick() internally.
//   - SKELETON is poll-driven; Stage 9 ARM bring-up adds an internal
//     20 Hz tick task.  API does not change.
//   - Trace ring is single-writer / single-reader.
//
// Recovery model:
//   - F1..F5 stay local — FSM state unchanged.
//   - `abort()` is the only escalation: transitions to ABORT, emits
//     `servo.disarm()`.  Terminal state until `init()` is called again.
//   - `clear_trace()` preserves FSM state AND lifetime counters.
//
// Concurrency contract:
//   - L6: single-writer (MP task).  No internal locking.
//   - L4/L5 already follow the same convention; L6 is a thin caller.
//
// Memory:
//   - sentai_explore_fsm_t ≈ 96 B (state + pose + home + counters).
//   - 16-entry trace ring × 32 B ≈ 0.5 KB.
//   - Total static ≈ 0.6 KB.  ARM placement .sdram_bss; SIM .bss.
//   - Zero heap, zero per-call malloc.
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity ----------------------------------------------------------
#define SENTAI_EXPLORE_TRACE_DEPTH       16     // FIFO ring; oldest overwritten

// ---- State enum --------------------------------------------------------
typedef enum {
    EXPLORE_IDLE      = 0,    // pre-start / post-DONE
    EXPLORE_ARMING    = 1,    // start() → L4 servo.arm() done
    EXPLORE_TAKEOFF   = 2,    // takeoff() emitted; waiting for altitude
    EXPLORE_HOVERING  = 3,    // idle in flight; ready for goto/RTH/land
    EXPLORE_APPROACH  = 4,    // moving toward target object
    EXPLORE_INSPECT   = 5,    // at target, dwelling
    EXPLORE_RETURNING = 6,    // moving toward home_xy
    EXPLORE_LANDING   = 7,    // descending; servo.land() emitted
    EXPLORE_DONE      = 8,    // mission complete, disarmed
    EXPLORE_ABORT     = 9,    // terminal fault state (motors disarmed)
    EXPLORE_LOST      = 10,   // pose reference lost; ascending to re-acquire
} sentai_explore_state_t;

// ---- Trace action codes (also stored in ring) --------------------------
typedef enum {
    EXPLORE_ACT_NONE       = 0,
    EXPLORE_ACT_INIT       = 1,
    EXPLORE_ACT_START      = 2,
    EXPLORE_ACT_TAKEOFF    = 3,
    EXPLORE_ACT_GOTO       = 4,
    EXPLORE_ACT_RETURN     = 5,
    EXPLORE_ACT_LAND       = 6,
    EXPLORE_ACT_STOP       = 7,
    EXPLORE_ACT_ABORT      = 8,
    EXPLORE_ACT_TRANSITION = 9,  // state-machine internal transition
    EXPLORE_ACT_LOST       = 10, // entered LOST (e.g., force_lost or auto)
    EXPLORE_ACT_RECOVERED  = 11, // signal_marker_seen → exit LOST
} sentai_explore_action_t;

// ---- Trace ring entry (~32 B) ------------------------------------------
// `param[]` interpretation depends on `action`:
//   INIT       → param[0] = backend id (as float), rest 0
//   START      → all 0
//   TAKEOFF    → param[0] = alt_m
//   GOTO       → param[0] = obj_id, param[1] = stop_dist
//   RETURN     → all 0
//   LAND       → all 0
//   STOP/ABORT → all 0
//   TRANSITION → param[0] = from_state, param[1] = to_state,
//                param[2] = reason_code, param[3] = t_in_state_ms (float)
typedef struct {
    uint32_t seq;                 // monotonic — g_fsm.seq at log time
    uint32_t t_ms;                // ms since boot
    uint8_t  action;              // sentai_explore_action_t
    int8_t   result;              // 0 ok, negative on fault
    uint8_t  state_before;
    uint8_t  state_after;
    float    param[4];
} sentai_explore_trace_t;

// ---- Status snapshot ---------------------------------------------------
typedef struct {
    // Live FSM
    uint8_t  state;                   // sentai_explore_state_t
    uint8_t  backend;                 // mirror of L4 servo backend
    uint16_t trace_count;             // 0..DEPTH

    uint32_t seq;                     // monotonic action attempt counter
    uint32_t t_state_entered_ms;
    uint32_t t_last_pose_ms;
    uint32_t t_mission_start_ms;

    // Live pose (last `set_pose`)
    float    pose_x;
    float    pose_y;
    float    pose_z;
    float    pose_yaw;

    // Home reference (captured at takeoff success)
    float    home_x;
    float    home_y;
    float    home_z;     // landing altitude (==0 unless overridden)
    float    home_yaw;   // datum yaw at takeoff
    uint8_t  home_set;

    // Current goto/return target (informational)
    uint16_t target_tracklet_id;
    float    target_x;
    float    target_y;

    // Lifetime counters
    uint32_t actions_ok;
    uint32_t faults_wrong_state;
    uint32_t faults_oob;
    uint32_t faults_servo;
    uint32_t faults_lifter;
    uint32_t faults_pose_stale;
    uint32_t transitions;
    uint32_t trace_overwrites;

    // Per-mission counters
    uint16_t gotos_completed;
    uint16_t aborts;
} sentai_explore_status_t;

// ---- Tunables (compile-time defaults; not yet runtime-settable) --------
#define SENTAI_EXPLORE_INSPECT_DUR_MS    1500u
#define SENTAI_EXPLORE_LAND_DUR_MS       3000u
#define SENTAI_EXPLORE_HOME_RADIUS_M     0.2f
#define SENTAI_EXPLORE_POSE_STALE_MS     2000u
#define SENTAI_EXPLORE_STOP_DIST_DEFAULT 0.3f
#define SENTAI_EXPLORE_ALT_MIN_M         0.05f
#define SENTAI_EXPLORE_ALT_MAX_M         30.0f
#define SENTAI_EXPLORE_TAKEOFF_TOLER_M   0.15f   // pose.z within this of alt → HOVERING
#define SENTAI_EXPLORE_LOST_ALT_BOOST_M  1.5f    // ascend boost when entering LOST
#define SENTAI_EXPLORE_LOST_TIMEOUT_MS   15000u  // ABORT if no recovery

// ---- C API -------------------------------------------------------------
// All entries: 0 on success, negative on fault per F1..F5.

// Initialise / reset FSM.  Calls sentai_servo_init(backend) internally.
// Side-effects: state = IDLE, counters and trace preserved? — counters
// reset, trace preserved (debuggable post-mortem of prior mission).
// Backend: pass SERVO_BACKEND_SIM/CF2/PX4 from sentai_servo.h.
//   Return: 0 ok, -2 bad backend, -3 servo refused.
int sentai_explore_init(uint8_t backend);

// IDLE → ARMING.  Calls sentai_servo_arm() internally.
//   Pre: state == IDLE.  Returns -1 if not.
int sentai_explore_start(void);

// ARMING/HOVERING → TAKEOFF → HOVERING (after pose reaches alt).
// On entry to TAKEOFF emits servo.takeoff(alt_m).  Records home_xy when
// pose first registers post-takeoff.
//   Pre: state == ARMING (cold takeoff) or HOVERING (re-takeoff after land).
//   alt_m in (ALT_MIN, ALT_MAX].
int sentai_explore_takeoff(float alt_m);

// Update internal pose snapshot.  Calls tick() internally.
//   No state preconditions.  Always 0.
int sentai_explore_set_pose(float x, float y, float z, float yaw);

// HOVERING → APPROACH.  Reads L5 lifter.world_pos(tracklet_id).
// Emits servo.move() toward target (clamped to L4 5 m fault gate).
// stop_dist_m: stop within this radius of the target (≥ 0, ≤ 5 m).
//   Pre: state == HOVERING, lifter has tracklet_id in LIFTED state,
//        pose fresh (≤ POSE_STALE_MS old).
int sentai_explore_goto(uint16_t tracklet_id, float stop_dist_m);

// HOVERING → RETURNING.  Emits servo.move() toward home_xy.
//   Pre: state == HOVERING, home_set == 1, pose fresh.
int sentai_explore_return_home(void);

// HOVERING → LANDING → DONE.  Emits servo.land(); transitions DONE
// after LAND_DUR_MS in LANDING.
//   Pre: state == HOVERING.
int sentai_explore_land(void);

// Graceful end: if HOVERING → return_home → land; if APPROACH/INSPECT →
// hover then return_home + land; if other states → abort.
int sentai_explore_stop(void);

// Emergency: any state → ABORT.  Emits servo.disarm().  Terminal.
int sentai_explore_abort(void);

// Force LOST state — testing entrypoint (later: automatic from timeouts).
// Captures the state we were in (pre_lost_state) for resume.  Emits
// servo.move(0, 0, +lost_alt_boost_m, 0) to ascend.  LOST waits up to
// SENTAI_EXPLORE_LOST_TIMEOUT_MS for a recovery signal, else → ABORT.
//   Pre: state in {HOVERING, APPROACH, INSPECT, RETURNING}.  Returns
//        -1 otherwise.
int sentai_explore_force_lost(void);

// Host-side signal that the drone is seeing a known marker at world (x, y).
// L6 sets pose to (x, y, current z) and transitions LOST → pre_lost_state.
// Used by the host vision loop when ArUco / place match recovers pose.
//   Pre: state == LOST.  Returns -1 otherwise.
int sentai_explore_signal_marker_seen(float wx, float wy);

// Re-evaluate time-based + pose-based transitions.  Called internally by
// set_pose() and by any command; can also be called explicitly from the
// operator loop (poll-driven SKELETON).
//   Returns: number of transitions executed (typically 0 or 1).
int sentai_explore_tick(void);

// Snapshot full FSM + counters.  out must not be NULL.
void sentai_explore_status(sentai_explore_status_t* out);

// Copy trace ring (oldest first) into out[0..max-1].
// Returns # entries written (<= min(max, trace_count)).
int sentai_explore_trace(sentai_explore_trace_t* out, int max);

// Wipe trace ring.  FSM state and lifetime counters preserved.
int sentai_explore_clear_trace(void);

// Translate state enum to short string (for REPL display).
const char* sentai_explore_state_name(uint8_t state);

// Translate action enum to short string.
const char* sentai_explore_action_name(uint8_t action);

// Runtime tunables (override compile-time defaults).  Pass 0 / NaN /
// negative for a slot to leave it unchanged.  Useful when a test
// scenario needs a smaller HOME_RADIUS (tight world) or a shorter
// INSPECT/LAND dwell.  Default values from SENTAI_EXPLORE_*_M[S].
//   Return: 0 ok, -2 if all args invalid.
int sentai_explore_set_tunables(float home_radius_m,
                                int inspect_dur_ms,
                                int land_dur_ms);

// Override LOST-related tunables.  Pass 0/NaN/negative to leave a slot
// unchanged.  Returns 0 ok, -2 if all args invalid.
int sentai_explore_set_lost_tunables(float lost_alt_boost_m,
                                     int lost_timeout_ms);

#ifdef __cplusplus
}
#endif
