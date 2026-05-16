// ============== sentai.explore — mission FSM (ObjectsPlan L6) =========
//
// This file is #include'd from modsentai.c (ARM) AND sim/modsentai_sim.c
// (SIM) — do NOT compile separately.  Single source of truth for the MP
// binding so ARM and SIM expose an identical surface.
//
// Backing store + FSM + trace ring live in sentai_explore.{h,cc}.  L6
// wraps L4 sentai_servo intents and reads L5 sentai_object_lifter world
// positions to drive an operator-commandable mission state machine.
//
// API (per experiments/s133_explore_skeleton/README.md):
//
//   sentai.explore.init(backend="sim")    -> 0|-2|-3
//   sentai.explore.start()                -> 0|-1|-3
//   sentai.explore.takeoff(alt_m)         -> 0|-1|-2|-3
//   sentai.explore.set_pose(x,y,z,yaw)    -> 0|-2
//   sentai.explore.goto(obj_id, stop_dist=0.3) -> 0|-1|-2|-3|-4|-5
//   sentai.explore.return_home()          -> 0|-1|-3|-5
//   sentai.explore.land()                 -> 0|-1|-3
//   sentai.explore.stop()                 -> 0|...
//   sentai.explore.abort()                -> 0
//   sentai.explore.tick()                 -> int (#transitions)
//   sentai.explore.state()                -> str
//   sentai.explore.metrics()              -> dict
//   sentai.explore.trace(max=16)          -> [dict, ...]
//   sentai.explore.clear_trace()          -> int (entries discarded)
//   sentai.explore.{IDLE..ABORT}                  -- state ints
//   sentai.explore.{ACT_NONE..ACT_TRANSITION}     -- action ids

#include "sentai_explore.h"
#include "sentai_servo.h"      // for backend ids (SERVO_BACKEND_*)

#include <string.h>

// ===================== Helpers =========================================

// Convert "sim" / "cf2" / "px4" (or already-int backend id) to the C enum.
// Returns SERVO_BACKEND_NONE for any unrecognised input.
static uint8_t exp_resolve_backend(mp_obj_t arg) {
    if (mp_obj_is_int(arg)) {
        int v = mp_obj_get_int(arg);
        if (v == SERVO_BACKEND_SIM || v == SERVO_BACKEND_CF2 ||
            v == SERVO_BACKEND_PX4) {
            return (uint8_t)v;
        }
        return SERVO_BACKEND_NONE;
    }
    if (mp_obj_is_str(arg)) {
        size_t len = 0;
        const char* s = mp_obj_str_get_data(arg, &len);
        if (len == 3) {
            if (s[0]=='s' && s[1]=='i' && s[2]=='m') return SERVO_BACKEND_SIM;
            if (s[0]=='c' && s[1]=='f' && s[2]=='2') return SERVO_BACKEND_CF2;
            if (s[0]=='p' && s[1]=='x' && s[2]=='4') return SERVO_BACKEND_PX4;
        }
    }
    return SERVO_BACKEND_NONE;
}

static mp_obj_t exp_trace_to_dict(const sentai_explore_trace_t* e) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(10));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),          mp_obj_new_int(e->seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_ms),         mp_obj_new_int(e->t_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_action),       mp_obj_new_int(e->action));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_result),       mp_obj_new_int(e->result));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_state_before), mp_obj_new_int(e->state_before));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_state_after),  mp_obj_new_int(e->state_after));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p0),           mp_obj_new_float(e->param[0]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p1),           mp_obj_new_float(e->param[1]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p2),           mp_obj_new_float(e->param[2]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p3),           mp_obj_new_float(e->param[3]));
    return MP_OBJ_FROM_PTR(d);
}

// ===================== init([backend]) =================================
// Default backend: "sim".

static mp_obj_t mod_explore_init(size_t n_args, const mp_obj_t* args) {
    uint8_t b = SERVO_BACKEND_SIM;
    if (n_args >= 1) {
        b = exp_resolve_backend(args[0]);
        if (b == SERVO_BACKEND_NONE) return mp_obj_new_int(-2);
    }
    return mp_obj_new_int(sentai_explore_init(b));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_init_obj, 0, 1, mod_explore_init);

// ===================== start / takeoff =================================

static mp_obj_t mod_explore_start(void) {
    return mp_obj_new_int(sentai_explore_start());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_start_obj, mod_explore_start);

static mp_obj_t mod_explore_takeoff(mp_obj_t alt_obj) {
    float alt = mp_obj_get_float(alt_obj);
    return mp_obj_new_int(sentai_explore_takeoff(alt));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_explore_takeoff_obj, mod_explore_takeoff);

// ===================== set_pose(x, y, z, yaw) ==========================

static mp_obj_t mod_explore_set_pose(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    float x   = mp_obj_get_float(args[0]);
    float y   = mp_obj_get_float(args[1]);
    float z   = mp_obj_get_float(args[2]);
    float yaw = mp_obj_get_float(args[3]);
    return mp_obj_new_int(sentai_explore_set_pose(x, y, z, yaw));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_set_pose_obj, 4, 4, mod_explore_set_pose);

// ===================== goto(obj_id, stop_dist=0.3) =====================

static mp_obj_t mod_explore_goto(size_t n_args, const mp_obj_t* args) {
    int obj_id  = mp_obj_get_int(args[0]);
    float stop  = (n_args >= 2) ? mp_obj_get_float(args[1])
                                : SENTAI_EXPLORE_STOP_DIST_DEFAULT;
    if (obj_id < 0 || obj_id > 0xFFFF) return mp_obj_new_int(-2);
    return mp_obj_new_int(sentai_explore_goto((uint16_t)obj_id, stop));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_goto_obj, 1, 2, mod_explore_goto);

// ===================== return_home / land / stop / abort ==============

static mp_obj_t mod_explore_return_home(void) {
    return mp_obj_new_int(sentai_explore_return_home());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_return_home_obj, mod_explore_return_home);

static mp_obj_t mod_explore_land(void) {
    return mp_obj_new_int(sentai_explore_land());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_land_obj, mod_explore_land);

static mp_obj_t mod_explore_stop(void) {
    return mp_obj_new_int(sentai_explore_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_stop_obj, mod_explore_stop);

static mp_obj_t mod_explore_abort(void) {
    return mp_obj_new_int(sentai_explore_abort());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_abort_obj, mod_explore_abort);

// ===================== tick() ==========================================

static mp_obj_t mod_explore_tick(void) {
    return mp_obj_new_int(sentai_explore_tick());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_tick_obj, mod_explore_tick);

// ===================== state() — short str =============================

static mp_obj_t mod_explore_state(void) {
    sentai_explore_status_t s;
    sentai_explore_status(&s);
    const char* name = sentai_explore_state_name(s.state);
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_state_obj, mod_explore_state);

// ===================== metrics() — full snapshot =======================

static mp_obj_t mod_explore_metrics(void) {
    sentai_explore_status_t s;
    sentai_explore_status(&s);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(28));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_state),          mp_obj_new_int(s.state));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_backend),        mp_obj_new_int(s.backend));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),            mp_obj_new_int(s.seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_trace_count),    mp_obj_new_int(s.trace_count));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_state_entered_ms),
                                                              mp_obj_new_int(s.t_state_entered_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_last_pose_ms), mp_obj_new_int(s.t_last_pose_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_mission_start_ms),
                                                              mp_obj_new_int(s.t_mission_start_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pose_x),         mp_obj_new_float(s.pose_x));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pose_y),         mp_obj_new_float(s.pose_y));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pose_z),         mp_obj_new_float(s.pose_z));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pose_yaw),       mp_obj_new_float(s.pose_yaw));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_home_x),         mp_obj_new_float(s.home_x));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_home_y),         mp_obj_new_float(s.home_y));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_home_z),         mp_obj_new_float(s.home_z));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_home_yaw),       mp_obj_new_float(s.home_yaw));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_home_set),       mp_obj_new_int(s.home_set));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_target_tracklet_id),
                                                              mp_obj_new_int(s.target_tracklet_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_target_x),       mp_obj_new_float(s.target_x));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_target_y),       mp_obj_new_float(s.target_y));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_actions_ok),     mp_obj_new_int(s.actions_ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_wrong_state),
                                                              mp_obj_new_int(s.faults_wrong_state));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_oob),     mp_obj_new_int(s.faults_oob));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_servo),   mp_obj_new_int(s.faults_servo));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_lifter),  mp_obj_new_int(s.faults_lifter));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_pose_stale),
                                                              mp_obj_new_int(s.faults_pose_stale));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_transitions),    mp_obj_new_int(s.transitions));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_gotos_completed),mp_obj_new_int(s.gotos_completed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_aborts),         mp_obj_new_int(s.aborts));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_trace_overwrites),
                                                              mp_obj_new_int(s.trace_overwrites));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_metrics_obj, mod_explore_metrics);

// ===================== trace(max=16) ===================================
// Snapshot buffer is file-scope static to keep MP-task stack small.
// Single-writer-single-reader contract per sentai_explore.h.

static sentai_explore_trace_t s_exp_trace_snap[SENTAI_EXPLORE_TRACE_DEPTH];

static mp_obj_t mod_explore_trace(size_t n_args, const mp_obj_t* args) {
    int max = SENTAI_EXPLORE_TRACE_DEPTH;
    if (n_args >= 1) {
        int v = mp_obj_get_int(args[0]);
        if (v >= 0 && v < max) max = v;
    }
    int n = sentai_explore_trace(s_exp_trace_snap, max);
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), exp_trace_to_dict(&s_exp_trace_snap[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_trace_obj, 0, 1, mod_explore_trace);

// ===================== clear_trace() ===================================

static mp_obj_t mod_explore_clear_trace(void) {
    return mp_obj_new_int(sentai_explore_clear_trace());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_clear_trace_obj, mod_explore_clear_trace);

// ===================== set_tunables(home_radius_m, inspect_ms, land_ms) ===

static mp_obj_t mod_explore_set_tunables(size_t n_args, const mp_obj_t* args) {
    float hr = (n_args >= 1) ? mp_obj_get_float(args[0]) : -1.f;
    int inspect_ms = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    int land_ms    = (n_args >= 3) ? mp_obj_get_int(args[2]) : -1;
    return mp_obj_new_int(sentai_explore_set_tunables(hr, inspect_ms, land_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_set_tunables_obj, 0, 3, mod_explore_set_tunables);

// ===================== set_lost_tunables(alt_boost_m, timeout_ms) ===

static mp_obj_t mod_explore_set_lost_tunables(size_t n_args, const mp_obj_t* args) {
    float ab = (n_args >= 1) ? mp_obj_get_float(args[0]) : -1.f;
    int to_ms = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    return mp_obj_new_int(sentai_explore_set_lost_tunables(ab, to_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_set_lost_tunables_obj, 0, 2, mod_explore_set_lost_tunables);

// ===================== force_lost() ===

static mp_obj_t mod_explore_force_lost(void) {
    return mp_obj_new_int(sentai_explore_force_lost());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_explore_force_lost_obj, mod_explore_force_lost);

// ===================== signal_marker_seen(wx, wy) ===

static mp_obj_t mod_explore_signal_marker_seen(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    float wx = mp_obj_get_float(args[0]);
    float wy = mp_obj_get_float(args[1]);
    return mp_obj_new_int(sentai_explore_signal_marker_seen(wx, wy));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_explore_signal_marker_seen_obj, 2, 2, mod_explore_signal_marker_seen);

// ===================== Module table ====================================

static const mp_rom_map_elem_t sentai_explore_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_explore) },
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&mod_explore_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),        MP_ROM_PTR(&mod_explore_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),      MP_ROM_PTR(&mod_explore_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pose),     MP_ROM_PTR(&mod_explore_set_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_goto),         MP_ROM_PTR(&mod_explore_goto_obj) },
    { MP_ROM_QSTR(MP_QSTR_return_home),  MP_ROM_PTR(&mod_explore_return_home_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),         MP_ROM_PTR(&mod_explore_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),         MP_ROM_PTR(&mod_explore_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_abort),        MP_ROM_PTR(&mod_explore_abort_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick),         MP_ROM_PTR(&mod_explore_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_state),        MP_ROM_PTR(&mod_explore_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_metrics),      MP_ROM_PTR(&mod_explore_metrics_obj) },
    { MP_ROM_QSTR(MP_QSTR_trace),        MP_ROM_PTR(&mod_explore_trace_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_trace),  MP_ROM_PTR(&mod_explore_clear_trace_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_tunables), MP_ROM_PTR(&mod_explore_set_tunables_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_lost_tunables), MP_ROM_PTR(&mod_explore_set_lost_tunables_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_lost),   MP_ROM_PTR(&mod_explore_force_lost_obj) },
    { MP_ROM_QSTR(MP_QSTR_signal_marker_seen), MP_ROM_PTR(&mod_explore_signal_marker_seen_obj) },

    // State ids (match metrics().state, trace().state_before/after)
    { MP_ROM_QSTR(MP_QSTR_IDLE),         MP_ROM_INT(EXPLORE_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_ARMING),       MP_ROM_INT(EXPLORE_ARMING) },
    { MP_ROM_QSTR(MP_QSTR_TAKEOFF),      MP_ROM_INT(EXPLORE_TAKEOFF) },
    { MP_ROM_QSTR(MP_QSTR_HOVERING),     MP_ROM_INT(EXPLORE_HOVERING) },
    { MP_ROM_QSTR(MP_QSTR_APPROACH),     MP_ROM_INT(EXPLORE_APPROACH) },
    { MP_ROM_QSTR(MP_QSTR_INSPECT),      MP_ROM_INT(EXPLORE_INSPECT) },
    { MP_ROM_QSTR(MP_QSTR_RETURNING),    MP_ROM_INT(EXPLORE_RETURNING) },
    { MP_ROM_QSTR(MP_QSTR_LANDING),      MP_ROM_INT(EXPLORE_LANDING) },
    { MP_ROM_QSTR(MP_QSTR_DONE),         MP_ROM_INT(EXPLORE_DONE) },
    { MP_ROM_QSTR(MP_QSTR_ABORT),        MP_ROM_INT(EXPLORE_ABORT) },
    { MP_ROM_QSTR(MP_QSTR_LOST),         MP_ROM_INT(EXPLORE_LOST) },

    // Action ids (match trace.action)
    { MP_ROM_QSTR(MP_QSTR_ACT_NONE),       MP_ROM_INT(EXPLORE_ACT_NONE) },
    { MP_ROM_QSTR(MP_QSTR_ACT_INIT),       MP_ROM_INT(EXPLORE_ACT_INIT) },
    { MP_ROM_QSTR(MP_QSTR_ACT_START),      MP_ROM_INT(EXPLORE_ACT_START) },
    { MP_ROM_QSTR(MP_QSTR_ACT_TAKEOFF),    MP_ROM_INT(EXPLORE_ACT_TAKEOFF) },
    { MP_ROM_QSTR(MP_QSTR_ACT_GOTO),       MP_ROM_INT(EXPLORE_ACT_GOTO) },
    { MP_ROM_QSTR(MP_QSTR_ACT_RETURN),     MP_ROM_INT(EXPLORE_ACT_RETURN) },
    { MP_ROM_QSTR(MP_QSTR_ACT_LAND),       MP_ROM_INT(EXPLORE_ACT_LAND) },
    { MP_ROM_QSTR(MP_QSTR_ACT_STOP),       MP_ROM_INT(EXPLORE_ACT_STOP) },
    { MP_ROM_QSTR(MP_QSTR_ACT_ABORT),      MP_ROM_INT(EXPLORE_ACT_ABORT) },
    { MP_ROM_QSTR(MP_QSTR_ACT_TRANSITION), MP_ROM_INT(EXPLORE_ACT_TRANSITION) },
    { MP_ROM_QSTR(MP_QSTR_ACT_LOST),       MP_ROM_INT(EXPLORE_ACT_LOST) },
    { MP_ROM_QSTR(MP_QSTR_ACT_RECOVERED),  MP_ROM_INT(EXPLORE_ACT_RECOVERED) },
};
static MP_DEFINE_CONST_DICT(sentai_explore_globals, sentai_explore_globals_table);

static const mp_obj_module_t sentai_explore_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_explore_globals,
};
