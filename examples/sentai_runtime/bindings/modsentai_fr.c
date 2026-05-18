// modsentai_fr.c — sentai.fr MicroPython binding.  OP-S10-W13-T3.
//
// Minimal per operator 2026-05-18: no dicts, no JSON.  MP returns only
// int / bool / str / small tuple.  Producers from C-side
// (sentai_safety_task) call sentai_fr_push_frame() directly; mission MP
// only opens/closes channels + pushes events/scalars (text).
//
// NOTE: bindings/*.c are #include-d into modsentai.c (ARM) /
// sim/modsentai_sim.c (SIM); both pull py/runtime.h + string.h.
// Don't re-include MP headers here.

#include "sentai_fr.h"

// ── Channel name string → enum dispatch ────────────────────────────────
static sentai_fr_channel_t parse_chan_(mp_obj_t name_obj) {
    if (!mp_obj_is_str(name_obj)) return SENTAI_FR_CH_NONE;
    size_t len;
    const char* s = mp_obj_str_get_data(name_obj, &len);
    if (len == 6 && memcmp(s, "frames", 6) == 0)   return SENTAI_FR_CH_FRAMES;
    if (len == 6 && memcmp(s, "events", 6) == 0)   return SENTAI_FR_CH_EVENTS;
    if (len == 7 && memcmp(s, "scalars", 7) == 0)  return SENTAI_FR_CH_SCALARS;
    if (len == 6 && memcmp(s, "kernel", 6) == 0)   return SENTAI_FR_CH_KERNEL;
    return SENTAI_FR_CH_NONE;
}

// ── Lifecycle ──────────────────────────────────────────────────────────
static mp_obj_t mod_fr_init_(void) {
    return mp_obj_new_int(sentai_fr_init());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_fr_init_obj, mod_fr_init_);

static mp_obj_t mod_fr_open_(mp_obj_t name_obj, mp_obj_t path_obj) {
    sentai_fr_channel_t c = parse_chan_(name_obj);
    if (c == SENTAI_FR_CH_NONE) return mp_obj_new_int(SENTAI_FR_ERR_UNKNOWN);
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_fr_open(c, path));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_fr_open_obj, mod_fr_open_);

static mp_obj_t mod_fr_close_(mp_obj_t name_obj) {
    sentai_fr_channel_t c = parse_chan_(name_obj);
    if (c == SENTAI_FR_CH_NONE) return mp_obj_new_int(SENTAI_FR_ERR_UNKNOWN);
    return mp_obj_new_int(sentai_fr_close(c));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_fr_close_obj, mod_fr_close_);

static mp_obj_t mod_fr_task_start_(void) {
    return mp_obj_new_int(sentai_fr_task_start());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_fr_task_start_obj, mod_fr_task_start_);

static mp_obj_t mod_fr_task_stop_(void) {
    return mp_obj_new_int(sentai_fr_task_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_fr_task_stop_obj, mod_fr_task_stop_);

// ── Push (mission convenience — text-only) ─────────────────────────────
//   sentai.fr.push_event(type, text)
static mp_obj_t mod_fr_push_event_(mp_obj_t type_obj, mp_obj_t text_obj) {
    const char* type = mp_obj_str_get_str(type_obj);
    const char* text = mp_obj_str_get_str(text_obj);
    return mp_obj_new_int(sentai_fr_push_event(type, text));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_fr_push_event_obj, mod_fr_push_event_);

//   sentai.fr.push_scalar(label, value, ts_ms=<auto>)
static mp_obj_t mod_fr_push_scalar_(size_t n_args, const mp_obj_t* args) {
    const char* label = mp_obj_str_get_str(args[0]);
    double value = mp_obj_get_float(args[1]);
    // ts_ms: if caller doesn't pass, we still record (value-only).
    // Mission MP usually doesn't have ms clock handy — pass 0 and
    // let post-mortem correlate by line number / surrounding events.
    uint32_t ts_ms = (n_args >= 3) ? (uint32_t)mp_obj_get_int(args[2]) : 0;
    return mp_obj_new_int(sentai_fr_push_scalar(label, value, ts_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_fr_push_scalar_obj, 2, 3,
                                             mod_fr_push_scalar_);

// ── stats(channel_str) → (pushes_total, pushes_accepted, drops, writes_ok, writes_fail, queue_depth, worst_queue_depth) ──
static mp_obj_t mod_fr_stats_(mp_obj_t name_obj) {
    sentai_fr_channel_t c = parse_chan_(name_obj);
    if (c == SENTAI_FR_CH_NONE) return mp_const_none;
    sentai_fr_channel_stats_t s;
    if (sentai_fr_get_stats(c, &s) != SENTAI_FR_OK) return mp_const_none;
    mp_obj_t items[7] = {
        mp_obj_new_int_from_uint(s.pushes_total),
        mp_obj_new_int_from_uint(s.pushes_accepted),
        mp_obj_new_int_from_uint(s.drops_full),
        mp_obj_new_int_from_uint(s.writes_ok),
        mp_obj_new_int_from_uint(s.writes_fail),
        mp_obj_new_int_from_uint(s.queue_depth),
        mp_obj_new_int_from_uint(s.worst_queue_depth),
    };
    return mp_obj_new_tuple(7, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_fr_stats_obj, mod_fr_stats_);

// ── Module table ───────────────────────────────────────────────────────
static const mp_rom_map_elem_t sentai_fr_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_fr) },
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&mod_fr_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),         MP_ROM_PTR(&mod_fr_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),        MP_ROM_PTR(&mod_fr_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_start),   MP_ROM_PTR(&mod_fr_task_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_stop),    MP_ROM_PTR(&mod_fr_task_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_push_event),   MP_ROM_PTR(&mod_fr_push_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_push_scalar),  MP_ROM_PTR(&mod_fr_push_scalar_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),        MP_ROM_PTR(&mod_fr_stats_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_fr_globals, sentai_fr_globals_table);

const mp_obj_module_t sentai_fr_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_fr_globals,
};
