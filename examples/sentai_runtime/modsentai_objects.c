// ============== sentai.objects — persistent world-frame object map =====
// This file is #include'd from modsentai.c (ARM) AND sim/modsentai_sim.c
// (SIM) — do NOT compile separately.  Single source of truth for the MP
// binding so ARM and SIM expose an identical surface.
//
// Backing store + math live in sentai_objects.{h,cc}.

#include "sentai_objects.h"

#include <math.h>
#include <string.h>

// ===================== Helpers =====================

// Build a python dict from a sentai_object_t snapshot.
static mp_obj_t obj_to_dict(const sentai_object_t* o) {
    mp_obj_t cov_items[6];
    for (int i = 0; i < 6; i++) cov_items[i] = mp_obj_new_float(o->cov_uppertri[i]);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(11));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_id),               mp_obj_new_int(o->id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_status),           mp_obj_new_int(o->status));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_class_id),         mp_obj_new_int(o->class_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_observations),     mp_obj_new_int(o->observations));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_x),                mp_obj_new_float(o->p_W[0]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_y),                mp_obj_new_float(o->p_W[1]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_z),                mp_obj_new_float(o->p_W[2]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cov),              mp_obj_new_tuple(6, cov_items));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_seen_ms),     mp_obj_new_int(o->last_seen_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_updated_ms),  mp_obj_new_int(o->last_updated_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_visited),          mp_obj_new_int(o->visited));
    return MP_OBJ_FROM_PTR(d);
}

// ===================== add(class_id, x, y, z, cov6=None) =====================

static mp_obj_t mod_objects_add(size_t n_args, const mp_obj_t* args) {
    int    class_id = mp_obj_get_int(args[0]);
    float  x        = mp_obj_get_float(args[1]);
    float  y        = mp_obj_get_float(args[2]);
    float  z        = mp_obj_get_float(args[3]);

    float  cov6_buf[6];
    const float* cov6 = NULL;
    if (n_args >= 5 && args[4] != mp_const_none) {
        size_t len;
        mp_obj_t* items;
        mp_obj_get_array(args[4], &len, &items);
        if (len < 6) return mp_obj_new_int(-1);
        for (int i = 0; i < 6; i++) cov6_buf[i] = mp_obj_get_float(items[i]);
        cov6 = cov6_buf;
    }
    if (class_id < 0 || class_id > 255) return mp_obj_new_int(-2);

    return mp_obj_new_int(sentai_objects_add((uint8_t)class_id, x, y, z, cov6));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_objects_add_obj, 4, 5, mod_objects_add);

// ===================== get(id) =====================

static mp_obj_t mod_objects_get(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_const_none;
    sentai_object_t snap;
    if (sentai_objects_get((uint8_t)id, &snap) != 0) return mp_const_none;
    return obj_to_dict(&snap);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_objects_get_obj, mod_objects_get);

// ===================== list() =====================
// Snapshot buffer is file-scope static (≈2 KB) to keep the MP-task stack
// small.  Single-writer-single-reader contract per sentai_objects.h.
static sentai_object_t s_obj_list_snap[SENTAI_OBJECTS_MAX];

static mp_obj_t mod_objects_list(void) {
    int n = sentai_objects_list(s_obj_list_snap, SENTAI_OBJECTS_MAX);
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), obj_to_dict(&s_obj_list_snap[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_objects_list_obj, mod_objects_list);

// ===================== mark_visited(id) =====================

static mp_obj_t mod_objects_mark_visited(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_objects_mark_visited((uint8_t)id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_objects_mark_visited_obj, mod_objects_mark_visited);

// ===================== set_status(id, status) =====================

static mp_obj_t mod_objects_set_status(mp_obj_t id_obj, mp_obj_t status_obj) {
    int id     = mp_obj_get_int(id_obj);
    int status = mp_obj_get_int(status_obj);
    if (id < 0 || id > 255)         return mp_obj_new_int(-1);
    if (status < 0 || status > 255) return mp_obj_new_int(-2);
    return mp_obj_new_int(sentai_objects_set_status((uint8_t)id, (uint8_t)status));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_objects_set_status_obj, mod_objects_set_status);

// ===================== remove(id) =====================

static mp_obj_t mod_objects_remove(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_objects_remove((uint8_t)id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_objects_remove_obj, mod_objects_remove);

// ===================== clear() =====================

static mp_obj_t mod_objects_clear(void) {
    return mp_obj_new_int(sentai_objects_clear());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_objects_clear_obj, mod_objects_clear);

// ===================== count() =====================

static mp_obj_t mod_objects_count(void) {
    return mp_obj_new_int(sentai_objects_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_objects_count_obj, mod_objects_count);

// ===================== stats() =====================

static mp_obj_t mod_objects_stats(void) {
    sentai_obj_stats_t c;
    int used = 0, conf = 0, coast = 0, stale = 0;
    sentai_objects_stats(&c, &used, &conf, &coast, &stale);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(13));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_used),         mp_obj_new_int(used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_hwm),          mp_obj_new_int(c.hwm_used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_capacity),     mp_obj_new_int(SENTAI_OBJECTS_MAX));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_confirmed),    mp_obj_new_int(conf));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_coasting),     mp_obj_new_int(coast));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_stale),        mp_obj_new_int(stale));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_adds),         mp_obj_new_int(c.adds));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_gets),         mp_obj_new_int(c.gets));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_removes),      mp_obj_new_int(c.removes));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_evictions),    mp_obj_new_int(c.evictions));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_oob_rejected), mp_obj_new_int(c.oob_rejected));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bad_ids),      mp_obj_new_int(c.bad_ids));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cov_clamped),  mp_obj_new_int(c.cov_clamped));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_objects_stats_obj, mod_objects_stats);

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_objects_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_objects) },
    { MP_ROM_QSTR(MP_QSTR_add),          MP_ROM_PTR(&mod_objects_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),          MP_ROM_PTR(&mod_objects_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_list),         MP_ROM_PTR(&mod_objects_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_mark_visited), MP_ROM_PTR(&mod_objects_mark_visited_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_status),   MP_ROM_PTR(&mod_objects_set_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),       MP_ROM_PTR(&mod_objects_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),        MP_ROM_PTR(&mod_objects_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_count),        MP_ROM_PTR(&mod_objects_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),        MP_ROM_PTR(&mod_objects_stats_obj) },
    // Status constants (so Python doesn't need magic numbers)
    { MP_ROM_QSTR(MP_QSTR_FREE),         MP_ROM_INT(OBJ_FREE) },
    { MP_ROM_QSTR(MP_QSTR_TENTATIVE),    MP_ROM_INT(OBJ_TENTATIVE) },
    { MP_ROM_QSTR(MP_QSTR_CONFIRMED),    MP_ROM_INT(OBJ_CONFIRMED) },
    { MP_ROM_QSTR(MP_QSTR_COASTING),     MP_ROM_INT(OBJ_COASTING) },
    { MP_ROM_QSTR(MP_QSTR_STALE),        MP_ROM_INT(OBJ_STALE) },
    { MP_ROM_QSTR(MP_QSTR_LOST),         MP_ROM_INT(OBJ_LOST) },
};
static MP_DEFINE_CONST_DICT(sentai_objects_globals, sentai_objects_globals_table);

static const mp_obj_module_t sentai_objects_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_objects_globals,
};
