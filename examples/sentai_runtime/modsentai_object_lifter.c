// ============== sentai.object_lifter — inverse-depth EKF landmark =====
// Per ObjectsPlan L5 (Stage 5). #include'd from modsentai.c (ARM) AND
// sim/modsentai_sim.c (SIM). Math + storage in sentai_object_lifter.{h,cc}.
//
// Minimal surface for driver tests + future detection_task wiring:
//   init_from_bbox, update_bbox, get, world_pos, list, clear, count,
//   mark_lost, stats, set_camera.
//
// Returns integers for init/update so error codes are observable from
// MP without exceptions (driver_t pattern).

#include "sentai_object_lifter.h"

#include <math.h>
#include <string.h>


// ===================== Helpers =====================

static mp_obj_t lif_entry_to_dict(const sentai_lifter_entry_t* e) {
    mp_obj_t anchor[3] = {
        mp_obj_new_float(e->anchor_w[0]),
        mp_obj_new_float(e->anchor_w[1]),
        mp_obj_new_float(e->anchor_w[2]),
    };
    mp_obj_t r_w[3] = {
        mp_obj_new_float(e->r_w[0]),
        mp_obj_new_float(e->r_w[1]),
        mp_obj_new_float(e->r_w[2]),
    };
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(13));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_status),       mp_obj_new_int(e->status));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_class_id),     mp_obj_new_int(e->class_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_tracklet_id),  mp_obj_new_int(e->tracklet_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rho),          mp_obj_new_float(e->rho));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_var_rho),      mp_obj_new_float(e->var_rho));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_anchor_w),     mp_obj_new_tuple(3, anchor));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_r_w),          mp_obj_new_tuple(3, r_w));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_obs),        mp_obj_new_int(e->n_obs));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_rejected),   mp_obj_new_int(e->n_rejected));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_age_ms),       mp_obj_new_int(e->age_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_obs_ms),  mp_obj_new_int(e->last_obs_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_init_ms),      mp_obj_new_int(e->init_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_published_object_id),
                       mp_obj_new_int(e->published_object_id));
    return MP_OBJ_FROM_PTR(d);
}

// Parse a 3-tuple (or list) of floats. Returns 0 on success.
static int lif_parse_vec3(mp_obj_t obj, float* out3) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(obj, &len, &items);
    if (len < 3) return -1;
    for (int i = 0; i < 3; i++) out3[i] = mp_obj_get_float(items[i]);
    return 0;
}

// ===================== init_from_bbox(tid, cls, u, v, w_px, real_m, drone, yaw)

static mp_obj_t mod_lifter_init_from_bbox(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    int      tid      = mp_obj_get_int(args[0]);
    int      cls      = mp_obj_get_int(args[1]);
    float    u_c      = mp_obj_get_float(args[2]);
    float    v_c      = mp_obj_get_float(args[3]);
    float    w_px     = mp_obj_get_float(args[4]);
    float    real_m   = mp_obj_get_float(args[5]);
    float    drone_W[3];
    if (lif_parse_vec3(args[6], drone_W) != 0) return mp_obj_new_int(-1);
    float    yaw_rad  = mp_obj_get_float(args[7]);
    if (tid < 0 || tid > 65535)   return mp_obj_new_int(-1);
    if (cls < 0 || cls > 255)     return mp_obj_new_int(-2);
    int rc = sentai_lifter_init_from_bbox((uint16_t)tid, (uint8_t)cls,
                                          u_c, v_c, w_px, real_m,
                                          drone_W, yaw_rad);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_lifter_init_from_bbox_obj, 8, 8, mod_lifter_init_from_bbox);

// ===================== update_bbox(tid, u, v, drone, yaw, dt_s)

static mp_obj_t mod_lifter_update_bbox(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    int      tid      = mp_obj_get_int(args[0]);
    float    u_c      = mp_obj_get_float(args[1]);
    float    v_c      = mp_obj_get_float(args[2]);
    float    drone_W[3];
    if (lif_parse_vec3(args[3], drone_W) != 0) return mp_obj_new_int(-6);
    float    yaw_rad  = mp_obj_get_float(args[4]);
    float    dt_s     = mp_obj_get_float(args[5]);
    if (tid < 0 || tid > 65535) return mp_obj_new_int(-1);
    int rc = sentai_lifter_update_bbox((uint16_t)tid, u_c, v_c,
                                       drone_W, yaw_rad, dt_s);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_lifter_update_bbox_obj, 6, 6, mod_lifter_update_bbox);

// ===================== get(tid)

static mp_obj_t mod_lifter_get(mp_obj_t tid_obj) {
    int tid = mp_obj_get_int(tid_obj);
    if (tid < 0 || tid > 65535) return mp_const_none;
    sentai_lifter_entry_t snap;
    if (sentai_lifter_get((uint16_t)tid, &snap) != 0) return mp_const_none;
    return lif_entry_to_dict(&snap);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_lifter_get_obj, mod_lifter_get);

// ===================== world_pos(tid)

static mp_obj_t mod_lifter_world_pos(mp_obj_t tid_obj) {
    int tid = mp_obj_get_int(tid_obj);
    if (tid < 0 || tid > 65535) return mp_const_none;
    float p[3] = {0};
    if (sentai_lifter_world_pos((uint16_t)tid, p) != 0) return mp_const_none;
    mp_obj_t items[3] = {
        mp_obj_new_float(p[0]),
        mp_obj_new_float(p[1]),
        mp_obj_new_float(p[2]),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_lifter_world_pos_obj, mod_lifter_world_pos);

// ===================== mark_lost(tid)

static mp_obj_t mod_lifter_mark_lost(mp_obj_t tid_obj) {
    int tid = mp_obj_get_int(tid_obj);
    if (tid < 0 || tid > 65535) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_lifter_mark_lost((uint16_t)tid));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_lifter_mark_lost_obj, mod_lifter_mark_lost);

// ===================== list() =====================

static sentai_lifter_entry_t s_lif_list_snap[SENTAI_LIFTER_MAX];

static mp_obj_t mod_lifter_list(void) {
    int n = sentai_lifter_list(s_lif_list_snap, SENTAI_LIFTER_MAX);
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst),
                            lif_entry_to_dict(&s_lif_list_snap[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_lifter_list_obj, mod_lifter_list);

// ===================== count() =====================

static mp_obj_t mod_lifter_count(void) {
    return mp_obj_new_int(sentai_lifter_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_lifter_count_obj, mod_lifter_count);

// ===================== clear() =====================

static mp_obj_t mod_lifter_clear(void) {
    return mp_obj_new_int(sentai_lifter_clear());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_lifter_clear_obj, mod_lifter_clear);

// ===================== stats() =====================

static mp_obj_t mod_lifter_stats(void) {
    sentai_lifter_stats_t c;
    int used = 0, tracking = 0, lifted = 0, lost = 0;
    sentai_lifter_stats(&c, &used, &tracking, &lifted, &lost);
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(15));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_used),         mp_obj_new_int(used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_hwm),          mp_obj_new_int(c.hwm_used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_capacity),     mp_obj_new_int(SENTAI_LIFTER_MAX));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_tracking),     mp_obj_new_int(tracking));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_lifted),       mp_obj_new_int(c.lifted));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_lost),         mp_obj_new_int(lost));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_inits),        mp_obj_new_int(c.inits));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_updates),      mp_obj_new_int(c.updates));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_rho_clamp),
                       mp_obj_new_int(c.rejects_rho_clamp));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_S_singular),
                       mp_obj_new_int(c.rejects_S_singular));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_var_invalid),
                       mp_obj_new_int(c.rejects_var_invalid));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_behind_camera),
                       mp_obj_new_int(c.rejects_behind_camera));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_unknown_tracklet),
                       mp_obj_new_int(c.rejects_unknown_tracklet));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejects_invalid_input),
                       mp_obj_new_int(c.rejects_invalid_input));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_evictions),    mp_obj_new_int(c.evictions));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_lifter_stats_obj, mod_lifter_stats);

// ===================== set_camera(fx, fy, cx, cy, R9, ofs3)

static mp_obj_t mod_lifter_set_camera(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    float fx = mp_obj_get_float(args[0]);
    float fy = mp_obj_get_float(args[1]);
    float cx = mp_obj_get_float(args[2]);
    float cy = mp_obj_get_float(args[3]);
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(args[4], &len, &items);
    if (len < 9) return mp_obj_new_int(-3);
    float R[9];
    for (int i = 0; i < 9; i++) R[i] = mp_obj_get_float(items[i]);
    float ofs[3];
    if (lif_parse_vec3(args[5], ofs) != 0) return mp_obj_new_int(-5);
    return mp_obj_new_int(
        sentai_lifter_set_camera(fx, fy, cx, cy, R, ofs));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_lifter_set_camera_obj, 6, 6, mod_lifter_set_camera);

// ===================== inject(tid, cls, x, y, z) [TEST-ONLY] =====================

static mp_obj_t mod_lifter_inject(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    int tid = mp_obj_get_int(args[0]);
    int cls = mp_obj_get_int(args[1]);
    float wx = mp_obj_get_float(args[2]);
    float wy = mp_obj_get_float(args[3]);
    float wz = mp_obj_get_float(args[4]);
    if (tid < 0 || tid > 0xFFFF || cls < 0 || cls > 255) {
        return mp_obj_new_int(-2);
    }
    return mp_obj_new_int(
        sentai_lifter_inject((uint16_t)tid, (uint8_t)cls, wx, wy, wz));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_lifter_inject_obj, 5, 5, mod_lifter_inject);

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_object_lifter_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_object_lifter) },
    { MP_ROM_QSTR(MP_QSTR_init_from_bbox),      MP_ROM_PTR(&mod_lifter_init_from_bbox_obj) },
    { MP_ROM_QSTR(MP_QSTR_update_bbox),         MP_ROM_PTR(&mod_lifter_update_bbox_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),                 MP_ROM_PTR(&mod_lifter_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_world_pos),           MP_ROM_PTR(&mod_lifter_world_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_list),                MP_ROM_PTR(&mod_lifter_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_count),               MP_ROM_PTR(&mod_lifter_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_mark_lost),           MP_ROM_PTR(&mod_lifter_mark_lost_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),               MP_ROM_PTR(&mod_lifter_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),               MP_ROM_PTR(&mod_lifter_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_camera),          MP_ROM_PTR(&mod_lifter_set_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_inject),              MP_ROM_PTR(&mod_lifter_inject_obj) },
    // Status constants.
    { MP_ROM_QSTR(MP_QSTR_FREE),                MP_ROM_INT(LIFTER_FREE) },
    { MP_ROM_QSTR(MP_QSTR_TRACKING),            MP_ROM_INT(LIFTER_TRACKING) },
    { MP_ROM_QSTR(MP_QSTR_LIFTED),              MP_ROM_INT(LIFTER_LIFTED) },
    { MP_ROM_QSTR(MP_QSTR_LOST),                MP_ROM_INT(LIFTER_LOST) },
};
static MP_DEFINE_CONST_DICT(sentai_object_lifter_globals,
                             sentai_object_lifter_globals_table);

static const mp_obj_module_t sentai_object_lifter_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_object_lifter_globals,
};
