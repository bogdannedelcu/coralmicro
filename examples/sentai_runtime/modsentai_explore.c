// ============== sentai.explore — Autonomous exploration mission =============
// This file is #include'd from modsentai.c (ARM) and sim/modsentai_sim.c (SIM)
// — do NOT compile separately.
//
// Mission state machine for sentai.explore — autonomous two-flight protocol
// per ideas/objects_plan.md §18.  This is the SKELETON only (Stage 3.A):
//
//   - State machine + guard conditions
//   - Lifecycle (start / stop / abort) + tick-driven state advance
//   - Metrics dict per embeded.md §observability + NASA/JPL fault model
//
// What's NOT here yet (next sub-stages):
//   - Intent emission (Stage 3.B — emit to sentai.servo)
//   - Real EXPLORE pattern logic (Stage 3.C — uses sentai.slam)
//   - FileX persistence + restore_from (Stage 1.B + 3.D)
//   - FreeRTOS task that auto-ticks (Stage 3.E — currently manual via tick())
//
// API contract (testable from REPL):
//
//   sentai.explore.start([mission_label="default"])
//        → 0 OK, -1 already running, -2 precond fail
//   sentai.explore.stop()       → 0
//   sentai.explore.abort()      → 0 (immediate ABORT state)
//   sentai.explore.tick()       → str: state after tick
//   sentai.explore.state()      → str: current state name
//   sentai.explore.metrics()    → dict
//
// Fault model F1-F9 deferred to later sub-stages.  For now only F0:
// "tick called without start" → no-op return current state.

#include <string.h>

// ===================== State machine =====================

typedef enum {
    EXP_IDLE              = 0,
    EXP_ARM_AT_MARKER     = 1,
    EXP_TAKEOFF           = 2,
    EXP_ESTABLISH_BASELINE= 3,
    EXP_EXPLORE           = 4,
    EXP_RETURN_HOME       = 5,
    EXP_PRECISION_LAND    = 6,
    EXP_COAST_LAND        = 7,
    EXP_EMERGENCY_HOVER   = 8,
    EXP_ABORT             = 9,
    EXP_DONE              = 10,
    EXP_LOAD_MODEL        = 11,
} exp_state_t;

static const char* k_exp_state_names[] = {
    "IDLE", "ARM_AT_MARKER", "TAKEOFF", "ESTABLISH_BASELINE",
    "EXPLORE", "RETURN_HOME", "PRECISION_LAND", "COAST_LAND",
    "EMERGENCY_HOVER", "ABORT", "DONE", "LOAD_MODEL",
};
#define EXP_STATE_COUNT 12

// Single-mission global state (per Stage 3.A: no FreeRTOS task; manual tick).
static exp_state_t  g_exp_state          = EXP_IDLE;
static char         g_exp_label[32]      = {0};
static uint32_t     g_exp_ticks          = 0;
static uint32_t     g_exp_state_ticks    = 0;
static uint32_t     g_exp_transitions    = 0;
static uint8_t      g_exp_last_from      = EXP_IDLE;
static uint8_t      g_exp_last_to        = EXP_IDLE;
static uint32_t     g_exp_abort_reason   = 0;

// Stage 3.B — sensor inputs pushed in each tick by the operator (or a
// future C-side mission task that polls the actual sources).  Negative
// sentinels mean "unknown" — guards default to NOT-READY, so a state will
// hold until the relevant input is supplied.
//
// NB: battery / IMU / link sensors do NOT live here.  Hardware-side
// safety lives in a separate sentai.safety FSM (out of scope for the
// objects_plan / world-model stack).  This module only tracks
// mission-progress signals.
static float g_exp_alt_m            = -1.0f;   /* drone altitude AGL (m), -1 = unknown */
static int   g_exp_marker_visible   = -1;      /* 0 / 1,                   -1 = unknown */
static float g_exp_dist_home_m      = -1.0f;   /* dist from home (m),      -1 = unknown */
static int   g_exp_cells_visited    = 0;       /* monotonic counter         */
static int   g_exp_arm_ack          = -1;      /* 0 / 1,                   -1 = unknown */

// Mission-side abort codes — exposed in metrics().  Hardware safety
// aborts (battery critical, link loss, IMU fault) come from a
// separate sentai.safety FSM and are NOT defined here.
#define EXP_ABORT_TIMEOUT       1
#define EXP_ABORT_OPERATOR      2

// Stage 3.B — runtime-tunable thresholds.  Defaults safe for cf2 indoor.
static float g_exp_target_alt_m       = 1.0f;    /* TAKEOFF complete when alt ≥ this */
static float g_exp_safe_land_alt_m    = 0.20f;   /* PRECISION → COAST when alt < this */
static float g_exp_done_alt_m         = 0.05f;   /* COAST → DONE                       */
static int   g_exp_explore_cell_budget= 8;       /* EXPLORE → RTH when cells_visited ≥ */
static float g_exp_dist_home_tol_m    = 0.30f;   /* RETURN_HOME → PRECISION_LAND       */
static uint32_t g_exp_explore_timeout_ticks = 600; /* hard timeout for EXPLORE         */

// ===================== Transition helpers =====================

static void exp_goto(exp_state_t next) {
    if (next == g_exp_state) return;
    g_exp_last_from = (uint8_t)g_exp_state;
    g_exp_last_to   = (uint8_t)next;
    g_exp_state = next;
    g_exp_state_ticks = 0;
    g_exp_transitions++;
}

// State step — called per tick.  Returns next state.
// Stage 3.B: tick budgets replaced with guard expressions over the
// mission-progress inputs (set_alt / set_marker / set_dist_home /
// set_cells_visited / set_arm_ack).  Hardware-side safety (battery,
// link, IMU) is NOT considered here — that's the sentai.safety FSM's
// job (out of scope for objects_plan; tracked separately).
// If an input remains at its sentinel ("unknown"), the corresponding
// guard returns false and the state holds — that's the safe default.
static exp_state_t exp_step(void) {
    g_exp_ticks++;
    g_exp_state_ticks++;

    switch (g_exp_state) {
        case EXP_IDLE:
            /* Held by external start() */
            break;
        case EXP_ARM_AT_MARKER:
            /* Ready to take off once arm has been acked AND a marker is
             * visible (operator usually sets marker_visible from the
             * aruco shim when the home tag is centered). */
            if (g_exp_arm_ack == 1 && g_exp_marker_visible == 1) {
                exp_goto(EXP_TAKEOFF);
            }
            break;
        case EXP_TAKEOFF:
            /* Climb until target altitude reached.  Use small hysteresis
             * (-5 cm) to avoid bouncing at the threshold. */
            if (g_exp_alt_m >= g_exp_target_alt_m - 0.05f) {
                exp_goto(EXP_ESTABLISH_BASELINE);
            }
            break;
        case EXP_ESTABLISH_BASELINE:
            /* Brief stabilization while sensors settle.  Real EKF-stable
             * guard belongs here (e.g., slam.cov_trace < threshold) —
             * for now, fixed 3-tick budget. */
            if (g_exp_state_ticks >= 3) exp_goto(EXP_EXPLORE);
            break;
        case EXP_EXPLORE:
            /* Exit when explored cell budget is hit, or the hard tick
             * timeout fires.  Whichever comes first.  (Hardware safety
             * — battery, link — is handled by sentai.safety, not here.) */
            if (g_exp_cells_visited >= g_exp_explore_cell_budget) {
                exp_goto(EXP_RETURN_HOME);
            } else if (g_exp_state_ticks >= g_exp_explore_timeout_ticks) {
                g_exp_abort_reason = EXP_ABORT_TIMEOUT;
                exp_goto(EXP_RETURN_HOME);
            }
            break;
        case EXP_RETURN_HOME:
            /* Close-in: switch to precision land when within tolerance. */
            if (g_exp_dist_home_m >= 0.0f &&
                g_exp_dist_home_m < g_exp_dist_home_tol_m) {
                exp_goto(EXP_PRECISION_LAND);
            }
            break;
        case EXP_PRECISION_LAND:
            /* Below safe-land altitude → release control (coast). */
            if (g_exp_alt_m >= 0.0f && g_exp_alt_m < g_exp_safe_land_alt_m) {
                exp_goto(EXP_COAST_LAND);
            }
            break;
        case EXP_COAST_LAND:
            /* Touchdown when very low. */
            if (g_exp_alt_m >= 0.0f && g_exp_alt_m < g_exp_done_alt_m) {
                exp_goto(EXP_DONE);
            }
            break;
        case EXP_EMERGENCY_HOVER:
        case EXP_ABORT:
        case EXP_DONE:
        case EXP_LOAD_MODEL:
            /* Terminal / hold — no auto-transition. */
            break;
    }
    return g_exp_state;
}

// ===================== MicroPython bindings =====================

static mp_obj_t mod_explore_start(size_t n_args, const mp_obj_t* args) {
    /* Allow restart from terminal states only.  EMERGENCY_HOVER is owned
     * by the (future) sentai.safety FSM — it must clear that hold before
     * a fresh mission can launch. */
    if (g_exp_state != EXP_IDLE  &&
        g_exp_state != EXP_DONE  &&
        g_exp_state != EXP_ABORT) {
        return mp_obj_new_int(-1);   // already running
    }
    const char* lbl = "default";
    if (n_args >= 1) {
        size_t l;
        lbl = mp_obj_str_get_data(args[0], &l);
        (void)l;
    }
    // Reset state
    strncpy(g_exp_label, lbl, sizeof(g_exp_label) - 1);
    g_exp_label[sizeof(g_exp_label) - 1] = 0;
    g_exp_ticks = 0;
    g_exp_state_ticks = 0;
    g_exp_transitions = 0;
    g_exp_abort_reason = 0;
    /* Stage 3.B — sensor inputs reset to unknown so guards hold by default */
    g_exp_alt_m          = -1.0f;
    g_exp_marker_visible = -1;
    g_exp_dist_home_m    = -1.0f;
    g_exp_cells_visited  = 0;
    g_exp_arm_ack        = -1;
    exp_goto(EXP_ARM_AT_MARKER);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_start_obj,
                                            0, 1, mod_explore_start);

static mp_obj_t mod_explore_stop(void) {
    if (g_exp_state != EXP_IDLE) {
        exp_goto(EXP_RETURN_HOME);
    }
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_stop_obj, mod_explore_stop);

static mp_obj_t mod_explore_abort(void) {
    if (g_exp_state != EXP_DONE && g_exp_state != EXP_ABORT) {
        exp_goto(EXP_ABORT);
    }
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_abort_obj, mod_explore_abort);

static mp_obj_t mod_explore_tick(void) {
    exp_state_t s = exp_step();
    return mp_obj_new_str(k_exp_state_names[(int)s],
                          strlen(k_exp_state_names[(int)s]));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_tick_obj, mod_explore_tick);

static mp_obj_t mod_explore_state(void) {
    return mp_obj_new_str(k_exp_state_names[(int)g_exp_state],
                          strlen(k_exp_state_names[(int)g_exp_state]));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_state_obj, mod_explore_state);

static mp_obj_t mod_explore_metrics(void) {
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, mp_obj_new_str("label", 5),
                      mp_obj_new_str(g_exp_label, strlen(g_exp_label)));
    mp_obj_dict_store(d, mp_obj_new_str("state", 5),
                      mp_obj_new_str(k_exp_state_names[(int)g_exp_state],
                                     strlen(k_exp_state_names[(int)g_exp_state])));
    mp_obj_dict_store(d, mp_obj_new_str("ticks", 5),
                      mp_obj_new_int_from_uint(g_exp_ticks));
    mp_obj_dict_store(d, mp_obj_new_str("state_ticks", 11),
                      mp_obj_new_int_from_uint(g_exp_state_ticks));
    mp_obj_dict_store(d, mp_obj_new_str("transitions", 11),
                      mp_obj_new_int_from_uint(g_exp_transitions));
    mp_obj_dict_store(d, mp_obj_new_str("last_from", 9),
                      mp_obj_new_str(k_exp_state_names[(int)g_exp_last_from],
                                     strlen(k_exp_state_names[(int)g_exp_last_from])));
    mp_obj_dict_store(d, mp_obj_new_str("last_to", 7),
                      mp_obj_new_str(k_exp_state_names[(int)g_exp_last_to],
                                     strlen(k_exp_state_names[(int)g_exp_last_to])));
    mp_obj_dict_store(d, mp_obj_new_str("abort_reason", 12),
                      mp_obj_new_int_from_uint(g_exp_abort_reason));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_metrics_obj, mod_explore_metrics);

// ============== Stage 3.B — sensor input + threshold setters ==============

// set_alt(m) -> None
static mp_obj_t mod_explore_set_alt(mp_obj_t v) {
    g_exp_alt_m = mp_obj_get_float(v);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_set_alt_obj, mod_explore_set_alt);

// set_marker(visible_0_or_1) -> None
static mp_obj_t mod_explore_set_marker(mp_obj_t v) {
    g_exp_marker_visible = mp_obj_get_int(v) ? 1 : 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_set_marker_obj, mod_explore_set_marker);

// set_dist_home(m) -> None
static mp_obj_t mod_explore_set_dist_home(mp_obj_t v) {
    g_exp_dist_home_m = mp_obj_get_float(v);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_set_dist_home_obj, mod_explore_set_dist_home);

// set_cells_visited(count) -> None
static mp_obj_t mod_explore_set_cells_visited(mp_obj_t v) {
    g_exp_cells_visited = mp_obj_get_int(v);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_set_cells_visited_obj, mod_explore_set_cells_visited);

// set_arm_ack(0_or_1) -> None
static mp_obj_t mod_explore_set_arm_ack(mp_obj_t v) {
    g_exp_arm_ack = mp_obj_get_int(v) ? 1 : 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_set_arm_ack_obj, mod_explore_set_arm_ack);

// set_thresholds(target_alt [, safe_land_alt [, cell_budget
//                [, dist_home_tol [, done_alt [, explore_timeout]]]]])
// All positional; pass 0 or negative to keep the current value.
// Hardware safety thresholds (battery, link, IMU) belong in sentai.safety.
static mp_obj_t mod_explore_set_thresholds(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) { float v = mp_obj_get_float(args[0]); if (v > 0) g_exp_target_alt_m       = v; }
    if (n_args >= 2) { float v = mp_obj_get_float(args[1]); if (v > 0) g_exp_safe_land_alt_m    = v; }
    if (n_args >= 3) { int   i = mp_obj_get_int  (args[2]); if (i > 0) g_exp_explore_cell_budget= i; }
    if (n_args >= 4) { float v = mp_obj_get_float(args[3]); if (v > 0) g_exp_dist_home_tol_m    = v; }
    if (n_args >= 5) { float v = mp_obj_get_float(args[4]); if (v > 0) g_exp_done_alt_m         = v; }
    if (n_args >= 6) { uint32_t u = (uint32_t)mp_obj_get_int(args[5]); if (u > 0) g_exp_explore_timeout_ticks = u; }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_set_thresholds_obj,
                                            1, 6, mod_explore_set_thresholds);

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_explore_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_explore) },
    { MP_ROM_QSTR(MP_QSTR_start),    MP_ROM_PTR(&mod_explore_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),     MP_ROM_PTR(&mod_explore_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_abort),    MP_ROM_PTR(&mod_explore_abort_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),     MP_ROM_PTR(&mod_explore_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_state),    MP_ROM_PTR(&mod_explore_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_metrics),  MP_ROM_PTR(&mod_explore_metrics_obj) },
    /* Stage 3.B sensor input + threshold setters */
    { MP_ROM_QSTR(MP_QSTR_set_alt),            MP_ROM_PTR(&mod_explore_set_alt_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_marker),         MP_ROM_PTR(&mod_explore_set_marker_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_dist_home),      MP_ROM_PTR(&mod_explore_set_dist_home_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cells_visited),  MP_ROM_PTR(&mod_explore_set_cells_visited_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arm_ack),        MP_ROM_PTR(&mod_explore_set_arm_ack_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_thresholds),     MP_ROM_PTR(&mod_explore_set_thresholds_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_explore_globals, sentai_explore_globals_table);
static const mp_obj_module_t sentai_explore_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_explore_globals,
};
