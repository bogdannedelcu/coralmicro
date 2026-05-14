// sentai_servo.h — ObjectsPlan L4 (Stage 4): backend-agnostic action layer
// SKELETON.
//
// The L6 mission FSM calls a single namespace (`sentai.servo.*`) for
// intent — takeoff / move / hover / land — instead of branching on
// backend (cf2 CRTP vs PX4 MAVLink) at every state edge.  This layer
// owns the intent vocabulary, an armed/flight state machine, and a
// 16-entry trace ring so the FSM can be verified end-to-end without
// real hardware.
//
// L4 ships the SKELETON ONLY.  Every action records to the trace ring;
// NO bytes are sent on any transport.  Backend wiring (CRTP / MAVLink)
// is Stage 4.A follow-up, NOT in L4 scope (see ideas/objects_plan.md
// Stage 4 + the L4 handoff memory).
//
// Re-implemented for the layered ObjectsPlan (NOT cherry-picked from
// feature/ov5640-camera-support — see [[no-broken-branch-test-reuse]]).
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A "system model first")
// =========================================================================
// Fault model (every public entry point returns a small int code):
//   F1 action before init() ........... return -1, faults_no_backend++
//   F2 action while disarmed (where    return -2, faults_not_armed++
//      armed is required: takeoff/
//      move/hover/land)
//   F3 takeoff(alt) alt out of (0, 30] return -3, faults_oob++
//   F4 move() any |d| > 5 m,           return -3, faults_oob++
//      |dyaw| > π/2, non-finite
//   F5 takeoff() while AIRBORNE        return -3, faults_oob++
//   F6 move/hover/land while GROUND    return -3, faults_oob++
//   F7 init(bad backend id)            return -1 (BAD_BACKEND), state untouched
//
// Execution model:
//   - All entry points are MP-task context only (REPL or /main.py).
//   - No ISR access.  No FreeRTOS primitives required.
//   - Bounded loops only inside list() over a 16-entry ring → ≪ 1 µs.
//   - Backing store is BSS-zeroed at boot; init() is the explicit
//     ENTRY transition, NOT a memory init.
//
// Recovery model:
//   - All errors are local: caller receives a negative return code,
//     trace ring records the failed attempt with `result` set to that
//     code so post-mortem can audit refused intents.  State machine
//     remains consistent on every error path.
//   - clear_trace() preserves the FSM state (backend / armed / flight)
//     and the lifetime counters.  Only the ring entries are wiped.
//   - sentai_servo_reset() (internal-only escalation, exposed only via
//     a fresh init() call) brings the FSM back to BACKEND_NONE.
//
// Concurrency contract:
//   - L4: single-writer, single-reader (MP task).  No internal locking.
//   - Stage 4.A onwards: a 50 Hz C tick task will read intent state to
//     produce velocity setpoints; that task MUST treat the public API
//     as the only writer and itself as a pure reader.  The trace ring
//     stays a single-writer single-reader structure.
//
// Memory:
//   - Static state: ~24 B FSM struct + 16 * 24 B trace entries ≈ 0.4 KB.
//     ARM placement: .sdram_bss (mirror L2/L3 convention).  SIM: .bss.
//   - Zero heap, zero per-call malloc.
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity ----------------------------------------------------------
#define SENTAI_SERVO_TRACE_DEPTH   16    // FIFO ring; oldest entries overwritten

// ---- Backend ids -------------------------------------------------------
typedef enum {
    SERVO_BACKEND_NONE = 0,    // init() not called; all actions return -1
    SERVO_BACKEND_SIM  = 1,    // skeleton: record-only, no transport bytes
    SERVO_BACKEND_CF2  = 2,    // Stage 4.A: bridges to sentai_crazy
    SERVO_BACKEND_PX4  = 3,    // Stage 4.A: bridges to sentai_link
} sentai_servo_backend_t;

// ---- Flight phase ------------------------------------------------------
typedef enum {
    SERVO_FLIGHT_GROUND   = 0,    // post-init or post-land
    SERVO_FLIGHT_AIRBORNE = 1,    // post-takeoff; move/hover/land allowed
} sentai_servo_flight_t;

// ---- Action codes (also stored in trace ring) --------------------------
typedef enum {
    SERVO_ACT_NONE    = 0,
    SERVO_ACT_INIT    = 1,
    SERVO_ACT_ARM     = 2,
    SERVO_ACT_DISARM  = 3,
    SERVO_ACT_TAKEOFF = 4,
    SERVO_ACT_MOVE    = 5,
    SERVO_ACT_HOVER   = 6,
    SERVO_ACT_LAND    = 7,
} sentai_servo_action_t;

// ---- Trace ring entry --------------------------------------------------
// 24 B padded.  param[] interpretation depends on `action`:
//   INIT     → param[0] = backend id (as float), rest 0
//   ARM      → all 0
//   DISARM   → all 0
//   TAKEOFF  → param[0] = alt_m, rest 0
//   MOVE     → param[0..2] = dx,dy,dz (m); param[3] = dyaw (rad)
//   HOVER    → all 0
//   LAND     → all 0
typedef struct {
    uint32_t seq;             // monotonic — matches g_fsm.seq at log time
    uint32_t t_ms;            // ms since boot
    uint8_t  action;          // sentai_servo_action_t
    int8_t   result;          // 0 ok, negative on fault (matches public API)
    uint8_t  backend;         // backend active at log time
    uint8_t  flight;          // flight phase AFTER the action attempt
    float    param[4];        // action-specific (see above)
} sentai_servo_trace_t;

// ---- Counters / status -------------------------------------------------
typedef struct {
    // FSM live state
    uint8_t  backend;             // sentai_servo_backend_t
    uint8_t  armed;               // 0 / 1
    uint8_t  flight;              // sentai_servo_flight_t
    uint8_t  _pad;
    uint32_t seq;                 // monotonic action attempt counter
    uint32_t t_last_action_ms;
    uint8_t  last_action;
    int8_t   last_result;
    uint16_t trace_count;         // number of entries currently in ring (0..DEPTH)

    // Lifetime counters
    uint32_t actions_ok;
    uint32_t faults_no_backend;
    uint32_t faults_not_armed;
    uint32_t faults_oob;
    uint32_t trace_overwrites;    // # of times the ring rolled over
} sentai_servo_status_t;

// ---- C API -------------------------------------------------------------
// Convention: all setters return 0 on success, negative on fault.

// Select backend.  Valid: SERVO_BACKEND_SIM/CF2/PX4 (NONE is not a valid
// init target — pass it via a "reset" path is reserved).  Records an INIT
// trace entry.  Side-effects: forces armed=0, flight=GROUND, seq++.
//   Return: 0 ok, -1 bad backend id.
int sentai_servo_init(uint8_t backend);

// Idempotent.  Returns 0 even if already armed.  Records ARM (result=0).
//   Pre: backend != NONE.  Returns -1 if not initialised.
int sentai_servo_arm(void);

// Idempotent.  Returns 0 even if already disarmed.  Records DISARM.
//   Pre: backend != NONE.  Returns -1 if not initialised.
//   Forces flight=GROUND if currently AIRBORNE (safe abort).
int sentai_servo_disarm(void);

// Records TAKEOFF.  Transitions flight=GROUND → AIRBORNE on success.
//   Pre: backend != NONE, armed=1, flight=GROUND, alt_m in (0, 30].
//   Returns -1/-2/-3 per fault model.
int sentai_servo_takeoff(float alt_m);

// Records MOVE.  No FSM transition (caller may chain).
//   Pre: backend != NONE, armed=1, flight=AIRBORNE,
//        |dx|,|dy|,|dz| ≤ 5 m, |dyaw| ≤ π/2, all finite.
//   Returns -1/-2/-3 per fault model.
int sentai_servo_move(float dx, float dy, float dz, float dyaw);

// Records HOVER.  No FSM transition.
//   Pre: backend != NONE, armed=1, flight=AIRBORNE.
int sentai_servo_hover(void);

// Records LAND.  Transitions flight=AIRBORNE → GROUND on success.
//   Pre: backend != NONE, armed=1, flight=AIRBORNE.
int sentai_servo_land(void);

// Snapshot full FSM + counters.  out_status must not be NULL.
void sentai_servo_status(sentai_servo_status_t* out_status);

// Copy trace ring (oldest first) into out[0..max-1].
// Returns number of entries written (<= min(max, trace_count)).
int sentai_servo_trace(sentai_servo_trace_t* out, int max);

// Wipe ring.  FSM state (backend / armed / flight) AND lifetime
// counters are preserved.  Returns # of entries discarded.
int sentai_servo_clear_trace(void);

#ifdef __cplusplus
}
#endif
