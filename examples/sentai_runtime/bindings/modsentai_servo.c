// ============== sentai.servo — backend-agnostic action layer ==========
// ObjectsPlan L4 (Stage 4 skeleton).
//
// This file is #include'd from modsentai.c (ARM) AND sim/modsentai_sim.c
// (SIM) — do NOT compile separately.  Single source of truth for the MP
// binding so ARM and SIM expose an identical surface.
//
// Backing store + FSM + trace ring live in sentai_servo.{h,cc}.  L4 is
// pure scaffolding: every action records into a 16-entry FIFO trace
// ring so the L6 mission FSM can be verified end-to-end without
// transport bytes leaving the board.  Stage 4.A swaps the SIM backend's
// "record-only" semantics for real CRTP / MAVLink writes.
//
// API (per L4 handoff memory + ideas/objects_plan.md Stage 4):
//
//   sentai.servo.init(backend_str)   -> 0|-1   backend ∈ {"sim","cf2","px4"}
//   sentai.servo.arm()               -> 0|-1
//   sentai.servo.disarm()            -> 0|-1
//   sentai.servo.takeoff(alt_m)      -> 0|-1|-2|-3  alt ∈ (0, 30]
//   sentai.servo.move(dx, dy, dz, dyaw=0.0)
//                                    -> 0|-1|-2|-3  |dxyz|≤5, |dyaw|≤π/2
//   sentai.servo.hover()             -> 0|-1|-2|-3
//   sentai.servo.land()              -> 0|-1|-2|-3
//   sentai.servo.status()            -> dict
//   sentai.servo.trace(max=16)       -> [dict, ...]
//   sentai.servo.clear_trace()       -> int (entries discarded)
//   sentai.servo.{NONE,SIM,CF2,PX4}             -- backend ints
//   sentai.servo.{GROUND,AIRBORNE}              -- flight ints
//   sentai.servo.{ACT_NONE..ACT_LAND}           -- action ids

#include "sentai_servo.h"

#include <string.h>

// ===================== Helpers =========================================

// Convert "sim" / "cf2" / "px4" (or already-int backend id) to the C enum.
// Returns SERVO_BACKEND_NONE for any unrecognised input.
static uint8_t srv_resolve_backend(mp_obj_t arg) {
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

static mp_obj_t srv_trace_to_dict(const sentai_servo_trace_t* e) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(10));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),     mp_obj_new_int(e->seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_ms),    mp_obj_new_int(e->t_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_action),  mp_obj_new_int(e->action));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_result),  mp_obj_new_int(e->result));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_backend), mp_obj_new_int(e->backend));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_flight),  mp_obj_new_int(e->flight));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p0),      mp_obj_new_float(e->param[0]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p1),      mp_obj_new_float(e->param[1]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p2),      mp_obj_new_float(e->param[2]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p3),      mp_obj_new_float(e->param[3]));
    return MP_OBJ_FROM_PTR(d);
}

// ===================== init(backend) ===================================

static mp_obj_t mod_servo_init(mp_obj_t backend_obj) {
    uint8_t b = srv_resolve_backend(backend_obj);
    if (b == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_servo_init(b));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_servo_init_obj, mod_servo_init);

// ===================== arm / disarm ====================================

static mp_obj_t mod_servo_arm(void) {
    return mp_obj_new_int(sentai_servo_arm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_arm_obj, mod_servo_arm);

static mp_obj_t mod_servo_disarm(void) {
    return mp_obj_new_int(sentai_servo_disarm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_disarm_obj, mod_servo_disarm);

// ===================== takeoff(alt_m) ==================================

static mp_obj_t mod_servo_takeoff(mp_obj_t alt_obj) {
    float alt = mp_obj_get_float(alt_obj);
    return mp_obj_new_int(sentai_servo_takeoff(alt));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_servo_takeoff_obj, mod_servo_takeoff);

// ===================== move(dx, dy, dz, dyaw=0) ========================

static mp_obj_t mod_servo_move(size_t n_args, const mp_obj_t* args) {
    float dx   =                 mp_obj_get_float(args[0]);
    float dy   =                 mp_obj_get_float(args[1]);
    float dz   =                 mp_obj_get_float(args[2]);
    float dyaw = (n_args >= 4) ? mp_obj_get_float(args[3]) : 0.0f;
    return mp_obj_new_int(sentai_servo_move(dx, dy, dz, dyaw));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_servo_move_obj, 3, 4, mod_servo_move);

// ===================== go_to(x, y, z, yaw=0) — absolute ===============

static mp_obj_t mod_servo_go_to(size_t n_args, const mp_obj_t* args) {
    float x   =                 mp_obj_get_float(args[0]);
    float y   =                 mp_obj_get_float(args[1]);
    float z   =                 mp_obj_get_float(args[2]);
    float yaw = (n_args >= 4) ? mp_obj_get_float(args[3]) : 0.0f;
    return mp_obj_new_int(sentai_servo_go_to(x, y, z, yaw));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_servo_go_to_obj, 3, 4, mod_servo_go_to);

// ===================== hover / land ====================================

static mp_obj_t mod_servo_hover(void) {
    return mp_obj_new_int(sentai_servo_hover());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_hover_obj, mod_servo_hover);

static mp_obj_t mod_servo_land(void) {
    return mp_obj_new_int(sentai_servo_land());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_land_obj, mod_servo_land);

// ===================== status() ========================================

static mp_obj_t mod_servo_status(void) {
    sentai_servo_status_t s;
    sentai_servo_status(&s);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(13));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_backend),           mp_obj_new_int(s.backend));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_armed),             mp_obj_new_int(s.armed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_flight),            mp_obj_new_int(s.flight));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),               mp_obj_new_int(s.seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_last_action_ms),  mp_obj_new_int(s.t_last_action_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_action),       mp_obj_new_int(s.last_action));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_result),       mp_obj_new_int(s.last_result));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_trace_count),       mp_obj_new_int(s.trace_count));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_actions_ok),        mp_obj_new_int(s.actions_ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_no_backend), mp_obj_new_int(s.faults_no_backend));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_not_armed),  mp_obj_new_int(s.faults_not_armed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_faults_oob),        mp_obj_new_int(s.faults_oob));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_trace_overwrites),  mp_obj_new_int(s.trace_overwrites));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_status_obj, mod_servo_status);

// ===================== trace(max=16) ===================================
// Snapshot buffer is file-scope static to keep MP-task stack small.
// Single-writer-single-reader contract per sentai_servo.h.

static sentai_servo_trace_t s_srv_trace_snap[SENTAI_SERVO_TRACE_DEPTH];

static mp_obj_t mod_servo_trace(size_t n_args, const mp_obj_t* args) {
    int max = SENTAI_SERVO_TRACE_DEPTH;
    if (n_args >= 1) {
        int v = mp_obj_get_int(args[0]);
        if (v >= 0 && v < max) max = v;
    }
    int n = sentai_servo_trace(s_srv_trace_snap, max);
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), srv_trace_to_dict(&s_srv_trace_snap[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_servo_trace_obj, 0, 1, mod_servo_trace);

// ===================== clear_trace() ===================================

static mp_obj_t mod_servo_clear_trace(void) {
    return mp_obj_new_int(sentai_servo_clear_trace());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_clear_trace_obj, mod_servo_clear_trace);

// ===================== Stage 4.A: pose() / set_durations() =============

// sentai.servo.set_durations(takeoff_s, move_s, land_s) -> int
static mp_obj_t mod_servo_set_durations(mp_obj_t to_obj, mp_obj_t mo_obj, mp_obj_t la_obj) {
    float to = mp_obj_get_float(to_obj);
    float mo = mp_obj_get_float(mo_obj);
    float la = mp_obj_get_float(la_obj);
    return mp_obj_new_int(sentai_servo_set_durations(to, mo, la));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_servo_set_durations_obj, mod_servo_set_durations);

// sentai.servo.pose() -> (x, y, z, yaw) tuple, or None.
static mp_obj_t mod_servo_pose(void) {
    float x=0, y=0, z=0, yaw=0;
    if (sentai_servo_pose(&x, &y, &z, &yaw) != 0) return mp_const_none;
    mp_obj_t tup[4] = {
        mp_obj_new_float(x), mp_obj_new_float(y),
        mp_obj_new_float(z), mp_obj_new_float(yaw),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_pose_obj, mod_servo_pose);

// sentai.servo.pose_ready() -> 0/1
static mp_obj_t mod_servo_pose_ready(void) {
    return mp_obj_new_int(sentai_servo_pose_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_pose_ready_obj, mod_servo_pose_ready);

// ===================== Module table ====================================

static const mp_rom_map_elem_t sentai_servo_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_servo) },
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&mod_servo_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),          MP_ROM_PTR(&mod_servo_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),       MP_ROM_PTR(&mod_servo_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),      MP_ROM_PTR(&mod_servo_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_move),         MP_ROM_PTR(&mod_servo_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_go_to),        MP_ROM_PTR(&mod_servo_go_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),        MP_ROM_PTR(&mod_servo_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),         MP_ROM_PTR(&mod_servo_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),       MP_ROM_PTR(&mod_servo_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_trace),        MP_ROM_PTR(&mod_servo_trace_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_trace),  MP_ROM_PTR(&mod_servo_clear_trace_obj) },
    // Stage 4.A (#45 + #47): pose + tuning
    { MP_ROM_QSTR(MP_QSTR_pose),         MP_ROM_PTR(&mod_servo_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose_ready),   MP_ROM_PTR(&mod_servo_pose_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_durations),MP_ROM_PTR(&mod_servo_set_durations_obj) },

    // Backend ids
    { MP_ROM_QSTR(MP_QSTR_NONE),         MP_ROM_INT(SERVO_BACKEND_NONE) },
    { MP_ROM_QSTR(MP_QSTR_SIM),          MP_ROM_INT(SERVO_BACKEND_SIM) },
    { MP_ROM_QSTR(MP_QSTR_CF2),          MP_ROM_INT(SERVO_BACKEND_CF2) },
    { MP_ROM_QSTR(MP_QSTR_PX4),          MP_ROM_INT(SERVO_BACKEND_PX4) },

    // Flight phase
    { MP_ROM_QSTR(MP_QSTR_GROUND),       MP_ROM_INT(SERVO_FLIGHT_GROUND) },
    { MP_ROM_QSTR(MP_QSTR_AIRBORNE),     MP_ROM_INT(SERVO_FLIGHT_AIRBORNE) },

    // Action ids (match trace.action / status.last_action)
    { MP_ROM_QSTR(MP_QSTR_ACT_NONE),     MP_ROM_INT(SERVO_ACT_NONE) },
    { MP_ROM_QSTR(MP_QSTR_ACT_INIT),     MP_ROM_INT(SERVO_ACT_INIT) },
    { MP_ROM_QSTR(MP_QSTR_ACT_ARM),      MP_ROM_INT(SERVO_ACT_ARM) },
    { MP_ROM_QSTR(MP_QSTR_ACT_DISARM),   MP_ROM_INT(SERVO_ACT_DISARM) },
    { MP_ROM_QSTR(MP_QSTR_ACT_TAKEOFF),  MP_ROM_INT(SERVO_ACT_TAKEOFF) },
    { MP_ROM_QSTR(MP_QSTR_ACT_MOVE),     MP_ROM_INT(SERVO_ACT_MOVE) },
    { MP_ROM_QSTR(MP_QSTR_ACT_HOVER),    MP_ROM_INT(SERVO_ACT_HOVER) },
    { MP_ROM_QSTR(MP_QSTR_ACT_LAND),     MP_ROM_INT(SERVO_ACT_LAND) },
    { MP_ROM_QSTR(MP_QSTR_ACT_GO_TO),    MP_ROM_INT(SERVO_ACT_GO_TO) },
};
static MP_DEFINE_CONST_DICT(sentai_servo_globals, sentai_servo_globals_table);

static const mp_obj_module_t sentai_servo_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_servo_globals,
};
