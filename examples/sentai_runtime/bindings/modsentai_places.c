// ============== sentai.places — H3-indexed place gallery ==============
// This file is #include'd from modsentai.c (ARM) AND sim/modsentai_sim.c
// (SIM) — do NOT compile separately.  Single source of truth for the MP
// binding so ARM and SIM expose an identical surface.
//
// Backing store + L1 match math live in sentai_places.{h,cc}.
// H3 helper bindings (cell_at, cell_to_latlng, neighbors) wrap libh3
// directly so MP code does not need a separate sentai.h3 module.
//
// API (per ideas/objects_plan.md §13.6 + §15.11):
//
//   sentai.places.add(h3_cell, desc_bytes_or_None, x=0, y=0, z=0) -> id|<0
//   sentai.places.get(id)                                          -> dict|None
//   sentai.places.observe(id)                                      -> 0|-1
//   sentai.places.set_status(id, status)                           -> 0|-1|-2
//   sentai.places.remove(id)                                       -> 0|-1
//   sentai.places.clear()                                          -> int
//   sentai.places.count()                                          -> int
//   sentai.places.list()                                           -> [dict, ...]
//   sentai.places.query(desc_bytes, h3_cell=0, k_disk=1, thresh=0) -> dict|None
//   sentai.places.stats()                                          -> dict
//   sentai.places.cell_at(lat, lng, res)                           -> int (H3Index)
//   sentai.places.cell_to_latlng(h3_cell)                          -> (lat, lng)
//   sentai.places.neighbors(h3_cell, k=1)                          -> [int, ...]
//   sentai.places.{FREE, TENTATIVE, CONFIRMED}

#include "sentai_places.h"
#include "sentai_phog.h"
#include "sentai_gist.h"
#include "sentai_hsv.h"
#include "slam_task.h"      // OP-S10-W11-T3 — SlamTask MP lifecycle

#include <math.h>
#include <string.h>

#include "h3api.h"

// Snapshot buffer is file-scope static (≈6 KB) to keep MP-task stack small.
// Single-writer-single-reader contract per sentai_places.h.
static sentai_place_t s_plr_list_snap[SENTAI_PLACES_MAX];

// ===================== Helpers =====================

static mp_obj_t plr_to_dict(const sentai_place_t* p) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(10));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_id),            mp_obj_new_int(p->id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_status),        mp_obj_new_int(p->status));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_visits),        mp_obj_new_int(p->visits));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_h3_cell),       mp_obj_new_int_from_ull(p->h3_cell));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_x),             mp_obj_new_float(p->p_W[0]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_y),             mp_obj_new_float(p->p_W[1]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_z),             mp_obj_new_float(p->p_W[2]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_first_seen_ms), mp_obj_new_int(p->first_seen_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_visit_ms), mp_obj_new_int(p->last_visit_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_desc_set),      mp_obj_new_int(p->desc_set));
    return MP_OBJ_FROM_PTR(d);
}

// Extract DESC_DIM bytes from a Python `bytes`/`bytearray`/buffer-protocol
// object into `out`.  Returns 1 ok, 0 wrong length / not buffer-like.
static int plr_extract_desc(mp_obj_t obj, uint8_t* out) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(obj, &bi, MP_BUFFER_READ)) return 0;
    if (bi.len != SENTAI_PLACES_DESC_DIM) return 0;
    memcpy(out, bi.buf, SENTAI_PLACES_DESC_DIM);
    return 1;
}

// ===================== add(h3_cell, desc, x=0, y=0, z=0) ===============

static mp_obj_t mod_places_add(size_t n_args, const mp_obj_t* args) {
    uint64_t h3_cell = (uint64_t)mp_obj_get_int(args[0]);

    uint8_t  desc_buf[SENTAI_PLACES_DESC_DIM];
    const uint8_t* desc = NULL;
    if (args[1] != mp_const_none) {
        if (!plr_extract_desc(args[1], desc_buf)) return mp_obj_new_int(-2);
        desc = desc_buf;
    }
    float x = (n_args >= 3) ? mp_obj_get_float(args[2]) : 0.0f;
    float y = (n_args >= 4) ? mp_obj_get_float(args[3]) : 0.0f;
    float z = (n_args >= 5) ? mp_obj_get_float(args[4]) : 0.0f;

    return mp_obj_new_int(sentai_places_add(h3_cell, desc, x, y, z));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_add_obj, 2, 5, mod_places_add);

// ===================== get(id) =========================================

// Returns the descriptor bytes for a place, or None if id unknown /
// desc not set.  Complements places.get() (which returns the
// pose+status dict only).
static mp_obj_t mod_places_get_desc(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 1 || id > 255) return mp_const_none;
    sentai_place_t snap;
    if (sentai_places_get((uint8_t)id, &snap) != 0) return mp_const_none;
    if (!snap.desc_set) return mp_const_none;
    return mp_obj_new_bytes(snap.desc, SENTAI_PLACES_DESC_DIM);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_get_desc_obj, mod_places_get_desc);

static mp_obj_t mod_places_get(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_const_none;
    sentai_place_t snap;
    if (sentai_places_get((uint8_t)id, &snap) != 0) return mp_const_none;
    return plr_to_dict(&snap);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_get_obj, mod_places_get);

// ===================== list() ==========================================

static mp_obj_t mod_places_list(void) {
    int n = sentai_places_list(s_plr_list_snap, SENTAI_PLACES_MAX);
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), plr_to_dict(&s_plr_list_snap[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_list_obj, mod_places_list);

// ===================== observe(id) =====================================

static mp_obj_t mod_places_observe(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_places_observe((uint8_t)id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_observe_obj, mod_places_observe);

// ===================== set_status(id, status) ==========================

static mp_obj_t mod_places_set_status(mp_obj_t id_obj, mp_obj_t status_obj) {
    int id     = mp_obj_get_int(id_obj);
    int status = mp_obj_get_int(status_obj);
    if (id < 0 || id > 255)         return mp_obj_new_int(-1);
    if (status < 0 || status > 255) return mp_obj_new_int(-2);
    return mp_obj_new_int(sentai_places_set_status((uint8_t)id, (uint8_t)status));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_places_set_status_obj, mod_places_set_status);

// ===================== remove(id) ======================================

static mp_obj_t mod_places_remove(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    if (id < 0 || id > 255) return mp_obj_new_int(-1);
    return mp_obj_new_int(sentai_places_remove((uint8_t)id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_remove_obj, mod_places_remove);

// ===================== clear() / count() ===============================

static mp_obj_t mod_places_clear(void) {
    return mp_obj_new_int(sentai_places_clear());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_clear_obj, mod_places_clear);

static mp_obj_t mod_places_count(void) {
    return mp_obj_new_int(sentai_places_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_count_obj, mod_places_count);

// ===================== query(desc, cell=0, k_disk=1, thresh=0) =========
// Returns a dict {id, score_pct, l1_dist, hit} or None on hard failure.
// hit is 1 when score_pct >= thresh else 0.  id == 0 also means miss.

static mp_obj_t mod_places_query(size_t n_args, const mp_obj_t* args) {
    uint8_t  desc_buf[SENTAI_PLACES_DESC_DIM];
    if (!plr_extract_desc(args[0], desc_buf)) return mp_const_none;

    uint64_t cell = (n_args >= 2) ? (uint64_t)mp_obj_get_int(args[1]) : 0;
    int      k    = (n_args >= 3) ?           mp_obj_get_int(args[2]) : 1;
    int      thr  = (n_args >= 4) ?           mp_obj_get_int(args[3]) : 0;
    if (k < 0)   k = 0;
    if (thr < 0) thr = 0;
    if (thr > 100) thr = 100;

    sentai_places_match_t r = sentai_places_query(desc_buf, cell, k, thr);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_id),        mp_obj_new_int(r.id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_score_pct), mp_obj_new_int(r.score_pct));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_l1_dist),   mp_obj_new_int(r.l1_dist));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_hit),
                      mp_obj_new_int((r.id != 0 && r.score_pct >= thr) ? 1 : 0));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_query_obj, 1, 4, mod_places_query);

// ===================== stats() =========================================

static mp_obj_t mod_places_stats(void) {
    sentai_places_stats_t c;
    int used = 0, tent = 0, conf = 0;
    sentai_places_stats(&c, &used, &tent, &conf);

    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(13));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_used),                 mp_obj_new_int(used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_hwm),                  mp_obj_new_int(c.hwm_used));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_capacity),             mp_obj_new_int(SENTAI_PLACES_MAX));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_desc_dim),             mp_obj_new_int(SENTAI_PLACES_DESC_DIM));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_tentative),            mp_obj_new_int(tent));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_confirmed),            mp_obj_new_int(conf));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_adds),                 mp_obj_new_int(c.adds));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_removes),              mp_obj_new_int(c.removes));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_evictions),            mp_obj_new_int(c.evictions));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_observations),         mp_obj_new_int(c.observations));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_queries),              mp_obj_new_int(c.queries));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_matches_above_thresh), mp_obj_new_int(c.matches_above_thresh));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_oob_rejected),         mp_obj_new_int(c.oob_rejected));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_stats_obj, mod_places_stats);

// ===================== H3 helpers ======================================
// Thin wrappers around libh3 so MP code can use H3 indexing without a
// separate sentai.h3 module.  Pure-functional, no state, no allocation.

// cell_at(lat_deg, lng_deg, res) -> int H3Index (0 on H3 failure).
static mp_obj_t mod_places_cell_at(mp_obj_t lat_obj, mp_obj_t lng_obj, mp_obj_t res_obj) {
    LatLng ll;
    ll.lat = degsToRads((double)mp_obj_get_float(lat_obj));
    ll.lng = degsToRads((double)mp_obj_get_float(lng_obj));
    int    res = mp_obj_get_int(res_obj);
    if (res < 0 || res > 15) return mp_obj_new_int_from_ull(0);
    H3Index out = 0;
    if (latLngToCell(&ll, res, &out) != E_SUCCESS) return mp_obj_new_int_from_ull(0);
    return mp_obj_new_int_from_ull((unsigned long long)out);
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_places_cell_at_obj, mod_places_cell_at);

// cell_to_latlng(h3_cell) -> (lat_deg, lng_deg) or None.
static mp_obj_t mod_places_cell_to_latlng(mp_obj_t cell_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    if (cell == 0) return mp_const_none;
    LatLng ll = {0, 0};
    if (cellToLatLng(cell, &ll) != E_SUCCESS) return mp_const_none;
    mp_obj_t pair[2] = {
        mp_obj_new_float(radsToDegs(ll.lat)),
        mp_obj_new_float(radsToDegs(ll.lng)),
    };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_cell_to_latlng_obj, mod_places_cell_to_latlng);

// neighbors(h3_cell, k=1) -> list[int].  Cap k at 3 to keep the static
// ring buffer ≤ 37 cells (matches sentai_places.cc internal cap).
static mp_obj_t mod_places_neighbors(size_t n_args, const mp_obj_t* args) {
    H3Index cell = (H3Index)mp_obj_get_int(args[0]);
    int     k    = (n_args >= 2) ? mp_obj_get_int(args[1]) : 1;
    if (k < 0) k = 0;
    if (k > 3) k = 3;

    int64_t max_n = 0;
    if (cell == 0 || maxGridDiskSize(k, &max_n) != E_SUCCESS || max_n <= 0) {
        return mp_obj_new_list(0, NULL);
    }
    if (max_n > 37) max_n = 37;
    H3Index ring[37] = {0};
    if (gridDisk(cell, k, ring) != E_SUCCESS) {
        return mp_obj_new_list(0, NULL);
    }
    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < (int)max_n; i++) {
        if (ring[i] == 0) continue;
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst),
                           mp_obj_new_int_from_ull((unsigned long long)ring[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_neighbors_obj, 1, 2, mod_places_neighbors);

// ===================== Module table ====================================

// ===================== compute_phog(gray_bytes, w, h) -> list[168] ====
// PHOG (Pyramid Histogram of Oriented Gradients) — Track A foundation
// per ideas/objects_plan.md §22.2.

static mp_obj_t mod_places_compute_phog(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    mp_buffer_info_t bi;
    if (!mp_get_buffer(args[0], &bi, MP_BUFFER_READ)) {
        return mp_const_none;
    }
    int w = mp_obj_get_int(args[1]);
    int h = mp_obj_get_int(args[2]);
    if (w <= 0 || h <= 0) return mp_const_none;
    if ((size_t)(w * h) > bi.len) return mp_const_none;

    float out[SENTAI_PHOG_DIM];
    int rc = sentai_phog_compute((const uint8_t*)bi.buf, w, h, out);
    if (rc < 0) return mp_const_none;

    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < SENTAI_PHOG_DIM; ++i) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), mp_obj_new_float(out[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_compute_phog_obj, 3, 3, mod_places_compute_phog);

// ===================== compute_gist(gray_bytes, w, h) -> list[64] ====
// GIST-lite (4 orient × 4×4 cells, decimated 4×) per Oliva & Torralba 2001.

static mp_obj_t mod_places_compute_gist(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    mp_buffer_info_t bi;
    if (!mp_get_buffer(args[0], &bi, MP_BUFFER_READ)) return mp_const_none;
    int w = mp_obj_get_int(args[1]);
    int h = mp_obj_get_int(args[2]);
    if (w <= 0 || h <= 0) return mp_const_none;
    if ((size_t)(w * h) > bi.len) return mp_const_none;

    float out[SENTAI_GIST_DIM];
    int rc = sentai_gist_compute((const uint8_t*)bi.buf, w, h, out);
    if (rc < 0) return mp_const_none;

    mp_obj_list_t* lst = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < SENTAI_GIST_DIM; ++i) {
        mp_obj_list_append(MP_OBJ_FROM_PTR(lst), mp_obj_new_float(out[i]));
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_compute_gist_obj, 3, 3, mod_places_compute_gist);

// ===================== compute_hsv(rgb_bytes, w, h) -> bytes(64) ======
// HS histogram (16 hue × 4 sat bins, V channel discarded) per Smith
// 1978.  Output is SLOT-NATIVE — `SENTAI_PLACES_DESC_DIM == 64 ==
// SENTAI_HSV_DIM`, so the bytes can be passed straight to places.add
// without a quantize() step (unlike PHOG/GIST which return float lists
// that hex_helpers.py:quantize() packs into 64 B).
//
// rgb_bytes layout: 3*w*h bytes, packed [R,G,B, R,G,B, ...].

static mp_obj_t mod_places_compute_hsv(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    mp_buffer_info_t bi;
    if (!mp_get_buffer(args[0], &bi, MP_BUFFER_READ)) return mp_const_none;
    int w = mp_obj_get_int(args[1]);
    int h = mp_obj_get_int(args[2]);
    if (w <= 0 || h <= 0) return mp_const_none;
    if ((size_t)(w * h * 3) > bi.len) return mp_const_none;

    uint8_t out[SENTAI_HSV_DIM];
    int rc = sentai_hsv_compute((const uint8_t*)bi.buf, w, h, out);
    if (rc < 0) return mp_const_none;
    return mp_obj_new_bytes(out, SENTAI_HSV_DIM);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_compute_hsv_obj, 3, 3, mod_places_compute_hsv);

// ===================== SlamTask lifecycle (OP-S10-W11-T3) =============
// Embedded perception loop: PrepTask SLOT_RGB_64 → sentai_hsv_compute →
// sentai_places_query → atomic publish.  No frames/tensors cross the MP
// boundary (per [[no-heavy-data-through-mp]]) — MP only sees small
// scalars (match id, score, l1 dist, monotonic seq, compute µs).

static mp_obj_t mod_places_start_slam(void) {
    return mp_obj_new_int(sentai_slam_start());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_start_slam_obj, mod_places_start_slam);

static mp_obj_t mod_places_stop_slam(void) {
    return mp_obj_new_int(sentai_slam_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_stop_slam_obj, mod_places_stop_slam);

static mp_obj_t mod_places_slam_current(void) {
    sentai_slam_result_t r;
    sentai_slam_get_current(&r);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_match_id),     mp_obj_new_int(r.match_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_score_pct),    mp_obj_new_int(r.score_pct));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_l1_dist),      mp_obj_new_int(r.l1_dist));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),    mp_obj_new_int(r.frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_result_seq),   mp_obj_new_int(r.result_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_t_compute_us), mp_obj_new_int(r.t_compute_us));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_slam_current_obj, mod_places_slam_current);

static mp_obj_t mod_places_slam_stats(void) {
    sentai_slam_stats_t s;
    sentai_slam_get_stats(&s);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_processed), mp_obj_new_int(s.frames_processed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_dropped),   mp_obj_new_int(s.frames_dropped));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_compute_us),  mp_obj_new_int(s.last_compute_us));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_avg_compute_us),   mp_obj_new_int(s.avg_compute_us));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_is_running),       mp_obj_new_int(s.is_running));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_slam_stats_obj, mod_places_slam_stats);

static const mp_rom_map_elem_t sentai_places_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),        MP_ROM_QSTR(MP_QSTR_places) },
    { MP_ROM_QSTR(MP_QSTR_add),             MP_ROM_PTR(&mod_places_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),             MP_ROM_PTR(&mod_places_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_desc),        MP_ROM_PTR(&mod_places_get_desc_obj) },
    { MP_ROM_QSTR(MP_QSTR_list),            MP_ROM_PTR(&mod_places_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_observe),         MP_ROM_PTR(&mod_places_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_status),      MP_ROM_PTR(&mod_places_set_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),          MP_ROM_PTR(&mod_places_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),           MP_ROM_PTR(&mod_places_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_count),           MP_ROM_PTR(&mod_places_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_query),           MP_ROM_PTR(&mod_places_query_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),           MP_ROM_PTR(&mod_places_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_cell_at),         MP_ROM_PTR(&mod_places_cell_at_obj) },
    { MP_ROM_QSTR(MP_QSTR_cell_to_latlng),  MP_ROM_PTR(&mod_places_cell_to_latlng_obj) },
    { MP_ROM_QSTR(MP_QSTR_neighbors),       MP_ROM_PTR(&mod_places_neighbors_obj) },
    // Track A descriptor compute (Bosch 2007 PHOG).
    { MP_ROM_QSTR(MP_QSTR_compute_phog),    MP_ROM_PTR(&mod_places_compute_phog_obj) },
    { MP_ROM_QSTR(MP_QSTR_compute_gist),    MP_ROM_PTR(&mod_places_compute_gist_obj) },
    { MP_ROM_QSTR(MP_QSTR_compute_hsv),     MP_ROM_PTR(&mod_places_compute_hsv_obj) },
    // SlamTask lifecycle (OP-S10-W11-T3).  Embedded perception loop —
    // see slam_task.h for the design contract.
    { MP_ROM_QSTR(MP_QSTR_start_slam),      MP_ROM_PTR(&mod_places_start_slam_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_slam),       MP_ROM_PTR(&mod_places_stop_slam_obj) },
    { MP_ROM_QSTR(MP_QSTR_slam_current),    MP_ROM_PTR(&mod_places_slam_current_obj) },
    { MP_ROM_QSTR(MP_QSTR_slam_stats),      MP_ROM_PTR(&mod_places_slam_stats_obj) },
    // Status constants
    { MP_ROM_QSTR(MP_QSTR_FREE),            MP_ROM_INT(PLR_FREE) },
    { MP_ROM_QSTR(MP_QSTR_TENTATIVE),       MP_ROM_INT(PLR_TENTATIVE) },
    { MP_ROM_QSTR(MP_QSTR_CONFIRMED),       MP_ROM_INT(PLR_CONFIRMED) },
};
static MP_DEFINE_CONST_DICT(sentai_places_globals, sentai_places_globals_table);

static const mp_obj_module_t sentai_places_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_places_globals,
};
