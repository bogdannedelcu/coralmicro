// modsentai_safety.c — sentai.safety MP binding (MINIMAL).
// OP-S10-W12-T4.
//
// Per operator 2026-05-18 ("nu vreau sa folosesc multa stiva in MP de
// aceea nu am nevoie de structuri complexe... e suficient sa stiu ca
// e aborted, care e motivul si gata, daca e nevoie o sa vad intr-un
// log care e cauza"): MP API is 7 functions returning only bool / int /
// str.  Detailed diagnostics (snapshot, events, task stats) stay in
// the C API for log inspection by future REPL diag helpers; mission
// code only needs aborted/reason for the abort decision.
//
// NOTE: bindings/*.c are #include-d into modsentai.c / modsentai_sim.c
// (parent pulls in py/runtime.h + string.h).  Do not re-include MP
// headers here.

#include "sentai_safety.h"
#include "sentai_safety_task.h"

// =======================================================================
// Lifecycle: 4 fns
// =======================================================================
static mp_obj_t mod_safety_init_(void) {
    return mp_obj_new_int(sentai_safety_init());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_init_obj, mod_safety_init_);

static mp_obj_t mod_safety_clear_(void) {
    return mp_obj_new_int(sentai_safety_clear());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_clear_obj, mod_safety_clear_);

// enable_aruco(n_min=4, max_loss_s=1.0) — operator HARD-RULE defaults
// per [[flowbaseline2-4markers-abort]].
static mp_obj_t mod_safety_enable_aruco_(size_t n_args, const mp_obj_t* pos,
                                           mp_map_t* kw) {
    sentai_safety_aruco_params_t p = { /*n_min*/ 4, /*max_loss_s*/ 1.0f };
    if (n_args >= 1) p.n_min      = mp_obj_get_int(pos[0]);
    if (n_args >= 2) p.max_loss_s = mp_obj_get_float(pos[1]);
    if (kw) {
        mp_map_elem_t* e;
        e = mp_map_lookup(kw, MP_OBJ_NEW_QSTR(MP_QSTR_n_min), MP_MAP_LOOKUP);
        if (e) p.n_min = mp_obj_get_int(e->value);
        e = mp_map_lookup(kw, MP_OBJ_NEW_QSTR(MP_QSTR_max_loss_s), MP_MAP_LOOKUP);
        if (e) p.max_loss_s = mp_obj_get_float(e->value);
    }
    return mp_obj_new_int(sentai_safety_enable(SENTAI_SAFETY_CHK_ARUCO, &p));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_safety_enable_aruco_obj, 0,
                                    mod_safety_enable_aruco_);

static mp_obj_t mod_safety_disable_aruco_(void) {
    return mp_obj_new_int(sentai_safety_disable(SENTAI_SAFETY_CHK_ARUCO));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_disable_aruco_obj,
                                  mod_safety_disable_aruco_);

// =======================================================================
// Worker task control: 2 fns
// =======================================================================
static mp_obj_t mod_safety_task_start_(void) {
    return mp_obj_new_int(sentai_safety_task_start());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_task_start_obj,
                                  mod_safety_task_start_);

static mp_obj_t mod_safety_task_stop_(void) {
    return mp_obj_new_int(sentai_safety_task_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_task_stop_obj,
                                  mod_safety_task_stop_);

// =======================================================================
// Mission query: 2 fns (the ONLY ones mission code calls in steady state)
// =======================================================================
static mp_obj_t mod_safety_aborted_(void) {
    return mp_obj_new_bool(sentai_safety_is_aborted());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_aborted_obj, mod_safety_aborted_);

static mp_obj_t mod_safety_reason_(void) {
    const char* r = sentai_safety_reason();
    return mp_obj_new_str(r, strlen(r));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_safety_reason_obj, mod_safety_reason_);

// ── TEST-ONLY injector for s171 unit test (bypasses SafetyTask) ───────
// Mission code MUST NOT use this — underscore prefix flags it as test-only.
static mp_obj_t mod_safety_test_push_aruco_(mp_obj_t n_obj, mp_obj_t seq_obj,
                                              mp_obj_t ts_obj) {
    int      n     = mp_obj_get_int(n_obj);
    uint32_t seq   = (uint32_t)mp_obj_get_int(seq_obj);
    uint32_t ts_ms = (uint32_t)mp_obj_get_int(ts_obj);
    return mp_obj_new_int(sentai_safety_on_aruco_result(n, seq, ts_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_safety_test_push_aruco_obj,
                                  mod_safety_test_push_aruco_);

// =======================================================================
// Module table
// =======================================================================
static const mp_rom_map_elem_t sentai_safety_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_safety) },
    { MP_ROM_QSTR(MP_QSTR_init),           MP_ROM_PTR(&mod_safety_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),          MP_ROM_PTR(&mod_safety_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_enable_aruco),   MP_ROM_PTR(&mod_safety_enable_aruco_obj) },
    { MP_ROM_QSTR(MP_QSTR_disable_aruco),  MP_ROM_PTR(&mod_safety_disable_aruco_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_start),     MP_ROM_PTR(&mod_safety_task_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_stop),      MP_ROM_PTR(&mod_safety_task_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_aborted),        MP_ROM_PTR(&mod_safety_aborted_obj) },
    { MP_ROM_QSTR(MP_QSTR_reason),         MP_ROM_PTR(&mod_safety_reason_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_push_aruco),
      MP_ROM_PTR(&mod_safety_test_push_aruco_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_safety_globals, sentai_safety_globals_table);

const mp_obj_module_t sentai_safety_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_safety_globals,
};
