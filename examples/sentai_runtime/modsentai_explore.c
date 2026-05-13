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
// Skeleton only: each state advances after a fixed tick budget for
// demonstration.  Real guards (track confirmed, EKF stable, etc.)
// arrive in Stage 3.B+.
static exp_state_t exp_step(void) {
    g_exp_ticks++;
    g_exp_state_ticks++;
    switch (g_exp_state) {
        case EXP_IDLE:
            // Stay in IDLE until external start().
            break;
        case EXP_ARM_AT_MARKER:
            // Stage 3.A: synthetic tick budget (will be real precondition check)
            if (g_exp_state_ticks >= 3) exp_goto(EXP_TAKEOFF);
            break;
        case EXP_TAKEOFF:
            if (g_exp_state_ticks >= 5) exp_goto(EXP_ESTABLISH_BASELINE);
            break;
        case EXP_ESTABLISH_BASELINE:
            if (g_exp_state_ticks >= 3) exp_goto(EXP_EXPLORE);
            break;
        case EXP_EXPLORE:
            if (g_exp_state_ticks >= 10) exp_goto(EXP_RETURN_HOME);
            break;
        case EXP_RETURN_HOME:
            if (g_exp_state_ticks >= 5) exp_goto(EXP_PRECISION_LAND);
            break;
        case EXP_PRECISION_LAND:
            if (g_exp_state_ticks >= 5) exp_goto(EXP_DONE);
            break;
        case EXP_COAST_LAND:
        case EXP_EMERGENCY_HOVER:
        case EXP_ABORT:
        case EXP_DONE:
        case EXP_LOAD_MODEL:
            // Terminal / hold states — no auto-transition.
            break;
    }
    return g_exp_state;
}

// ===================== MicroPython bindings =====================

static mp_obj_t mod_explore_start(size_t n_args, const mp_obj_t* args) {
    if (g_exp_state != EXP_IDLE && g_exp_state != EXP_DONE && g_exp_state != EXP_ABORT) {
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

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_explore_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_explore) },
    { MP_ROM_QSTR(MP_QSTR_start),    MP_ROM_PTR(&mod_explore_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),     MP_ROM_PTR(&mod_explore_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_abort),    MP_ROM_PTR(&mod_explore_abort_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),     MP_ROM_PTR(&mod_explore_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_state),    MP_ROM_PTR(&mod_explore_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_metrics),  MP_ROM_PTR(&mod_explore_metrics_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_explore_globals, sentai_explore_globals_table);
static const mp_obj_module_t sentai_explore_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_explore_globals,
};
