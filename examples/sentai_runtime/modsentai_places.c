// ============== sentai.places — H3-indexed world model (Stage 11.B) ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Hexagonal world-model gallery on top of Uber H3 (third_party/h3).  Each
// observed cell stores: visit counter + small class-id histogram + optional
// embedding slot (wired in Stage 11.D — HSV histogram from sentai_tracker).
//
// Local coordinate convention.  The drone navigates in a local ENU frame
// centered on an origin (typically the ArUco "home" marker).  places.init()
// sets the origin lat/lng + a metric scale (1.0 for PX4 natural, 0.1 for
// cf2 1/10).  Subsequent places.cell(x, y, res) maps local (x,y) meters →
// H3 index using a small-angle planar approximation (sufficient at the
// world scales we care about — kilometers, not continents).
//
// Storage is a fixed-size array of PLACES_MAX cells with linear-probe lookup
// keyed on H3Index.  No malloc, no FreeRTOS dependency — works on M7 and on
// the SIM host.

#include "h3api.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Constants =====================
#define PLACES_MAX           512    // max distinct cells stored
#define PLACES_HIST_BINS     8      // top-K class buckets per cell
#define PLACES_EMB_DIM       128    // HSV 4x4x8 histogram (Stage 11.D wiring)
#define PLACES_INVALID_CELL  ((H3Index)0)

// Earth radius for local-to-geodetic conversion (meters).  Mean radius is
// fine — we only need ~10 m absolute accuracy on the cell boundary, and
// the H3 cell IS the quantization.
#define PLACES_EARTH_R_M     6371000.0

// ===================== Data Structures =====================

typedef struct {
    int   class_id;
    int   count;
} places_class_bucket_t;

typedef struct {
    H3Index   cell;                                   // PLACES_INVALID_CELL = free slot
    int       visits;                                 // total observe() calls hitting this cell
    int       n_classes;                              // populated buckets (≤ PLACES_HIST_BINS)
    places_class_bucket_t  hist[PLACES_HIST_BINS];    // class histogram (sorted by count desc)
    int       has_embedding;                          // 0/1 — Stage 11.D sets this
    uint8_t   embedding[PLACES_EMB_DIM];              // 8-bit normalized histogram (Stage 11.D)
} places_entry_t;

// ===================== Global State =====================

static int             g_places_initialized = 0;
static int             g_places_n_entries   = 0;    // live entries
static double          g_places_origin_lat_rad = 0.0;
static double          g_places_origin_lng_rad = 0.0;
static double          g_places_scale       = 1.0;  // local-meter per drone-meter (1=PX4, 10=cf2)
static int             g_places_default_res = 9;    // default H3 resolution (cell edge ~150 m)
static places_entry_t  g_places[PLACES_MAX];        // dense array (free slot = cell==0)

// ===================== Helpers =====================

// Convert local (x, y) meters to geodetic lat/lng (radians) under the
// equirectangular approximation around the origin.  Sufficient at the
// scales we navigate (≤1 km).
static void places_local_to_geo(double x_m, double y_m,
                                double *lat_rad, double *lng_rad) {
    /* Apply scene-scale (cf2 uses 1/10 scale → multiply by 10 to map to PX4 meters) */
    double X = x_m * g_places_scale;
    double Y = y_m * g_places_scale;
    *lat_rad = g_places_origin_lat_rad + (Y / PLACES_EARTH_R_M);
    *lng_rad = g_places_origin_lng_rad +
               (X / (PLACES_EARTH_R_M * cos(g_places_origin_lat_rad)));
}

// Locate an entry by H3 cell — linear probe; returns index or -1.
static int places_find(H3Index cell) {
    if (cell == PLACES_INVALID_CELL) return -1;
    for (int i = 0; i < PLACES_MAX; i++) {
        if (g_places[i].cell == cell) return i;
    }
    return -1;
}

// Find or allocate an entry for cell; returns index or -1 if table full.
static int places_get_or_create(H3Index cell) {
    int idx = places_find(cell);
    if (idx >= 0) return idx;
    /* Fresh allocation in first free slot */
    for (int i = 0; i < PLACES_MAX; i++) {
        if (g_places[i].cell == PLACES_INVALID_CELL) {
            memset(&g_places[i], 0, sizeof(g_places[i]));
            g_places[i].cell = cell;
            g_places_n_entries++;
            return i;
        }
    }
    return -1;
}

// Bump a class-id count inside an entry, preserving count-desc ordering.
static void places_bump_class(places_entry_t *e, int class_id) {
    for (int b = 0; b < e->n_classes; b++) {
        if (e->hist[b].class_id == class_id) {
            e->hist[b].count++;
            /* Bubble up if needed */
            while (b > 0 && e->hist[b].count > e->hist[b-1].count) {
                places_class_bucket_t tmp = e->hist[b];
                e->hist[b] = e->hist[b-1];
                e->hist[b-1] = tmp;
                b--;
            }
            return;
        }
    }
    /* Not in histogram yet */
    if (e->n_classes < PLACES_HIST_BINS) {
        e->hist[e->n_classes].class_id = class_id;
        e->hist[e->n_classes].count    = 1;
        e->n_classes++;
    }
    /* else: histogram full, drop the new class (could LRU; deferred) */
}

// ===================== MicroPython bindings =====================

// places.init(origin_lat_deg, origin_lng_deg [, scale=1.0 [, default_res=9]]) -> int
//   Sets the local-to-geographic origin and metric scale.  scale=1.0 for
//   PX4 (natural meters); scale=0.1 for cf2 (1/10 sim, see Sim.md §10v).
//   Returns 0 on success, -1 on invalid res.
static mp_obj_t mod_places_init(size_t n_args, const mp_obj_t *args) {
    double lat = mp_obj_get_float(args[0]);
    double lng = mp_obj_get_float(args[1]);
    double scale = (n_args >= 3) ? mp_obj_get_float(args[2]) : 1.0;
    int    res   = (n_args >= 4) ? mp_obj_get_int(args[3])    : 9;
    if (res < 0 || res > 15) return mp_obj_new_int(-1);
    g_places_origin_lat_rad = lat * (M_PI / 180.0);
    g_places_origin_lng_rad = lng * (M_PI / 180.0);
    g_places_scale = scale;
    g_places_default_res = res;
    /* Wipe table on (re-)init */
    memset(g_places, 0, sizeof(g_places));
    g_places_n_entries = 0;
    g_places_initialized = 1;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_init_obj, 2, 4, mod_places_init);

// places.cell(x_m, y_m [, res]) -> int (H3 index as Python int)
static mp_obj_t mod_places_cell(size_t n_args, const mp_obj_t *args) {
    if (!g_places_initialized) return mp_obj_new_int(0);
    double x = mp_obj_get_float(args[0]);
    double y = mp_obj_get_float(args[1]);
    int    res = (n_args >= 3) ? mp_obj_get_int(args[2]) : g_places_default_res;
    if (res < 0 || res > 15) return mp_obj_new_int(0);
    LatLng p;
    places_local_to_geo(x, y, &p.lat, &p.lng);
    H3Index cell = 0;
    H3Error err = latLngToCell(&p, res, &cell);
    if (err) return mp_obj_new_int(0);
    return mp_obj_new_int_from_ull((unsigned long long)cell);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_cell_obj, 2, 3, mod_places_cell);

// places.observe(cell, class_id) -> int  (visit count after bump, or -1 on overflow)
static mp_obj_t mod_places_observe(mp_obj_t cell_obj, mp_obj_t cls_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int class_id = mp_obj_get_int(cls_obj);
    if (cell == PLACES_INVALID_CELL || !isValidCell(cell)) return mp_obj_new_int(-1);
    int idx = places_get_or_create(cell);
    if (idx < 0) return mp_obj_new_int(-1);
    g_places[idx].visits++;
    places_bump_class(&g_places[idx], class_id);
    return mp_obj_new_int(g_places[idx].visits);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_places_observe_obj, mod_places_observe);

// places.classes(cell) -> [(class_id, count), ...]  or empty list if unknown
static mp_obj_t mod_places_classes(mp_obj_t cell_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int idx = places_find(cell);
    if (idx < 0) return mp_obj_new_list(0, NULL);
    mp_obj_t result = mp_obj_new_list(0, NULL);
    for (int b = 0; b < g_places[idx].n_classes; b++) {
        mp_obj_t pair[2] = {
            mp_obj_new_int(g_places[idx].hist[b].class_id),
            mp_obj_new_int(g_places[idx].hist[b].count),
        };
        mp_obj_list_append(result, mp_obj_new_tuple(2, pair));
    }
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_classes_obj, mod_places_classes);

// places.visits(cell) -> int (visit count or 0 if unknown)
static mp_obj_t mod_places_visits(mp_obj_t cell_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int idx = places_find(cell);
    return mp_obj_new_int(idx < 0 ? 0 : g_places[idx].visits);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_visits_obj, mod_places_visits);

// places.neighbors(cell, k) -> [cell, ...]  k-ring neighbors (incl. origin)
static mp_obj_t mod_places_neighbors(mp_obj_t cell_obj, mp_obj_t k_obj) {
    H3Index origin = (H3Index)mp_obj_get_int(cell_obj);
    int k = mp_obj_get_int(k_obj);
    if (k < 0 || k > 5) return mp_obj_new_list(0, NULL);   /* cap to bound stack alloc */
    int64_t n = 0;
    if (maxGridDiskSize(k, &n) || n <= 0 || n > 256) return mp_obj_new_list(0, NULL);
    H3Index out[256] = {0};
    if (gridDisk(origin, k, out)) return mp_obj_new_list(0, NULL);
    mp_obj_t result = mp_obj_new_list(0, NULL);
    for (int i = 0; i < n; i++) {
        if (out[i] != 0) {
            mp_obj_list_append(result, mp_obj_new_int_from_ull((unsigned long long)out[i]));
        }
    }
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_places_neighbors_obj, mod_places_neighbors);

// places.cells() -> [cell, ...]   all known cells
static mp_obj_t mod_places_cells(void) {
    mp_obj_t result = mp_obj_new_list(0, NULL);
    for (int i = 0; i < PLACES_MAX; i++) {
        if (g_places[i].cell != PLACES_INVALID_CELL) {
            mp_obj_list_append(result,
                mp_obj_new_int_from_ull((unsigned long long)g_places[i].cell));
        }
    }
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_cells_obj, mod_places_cells);

// places.clear() -> None  wipes the gallery (origin/scale retained)
static mp_obj_t mod_places_clear(void) {
    memset(g_places, 0, sizeof(g_places));
    g_places_n_entries = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_clear_obj, mod_places_clear);

// places.info() -> dict
static mp_obj_t mod_places_info(void) {
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_initialized),
                      mp_obj_new_int(g_places_initialized));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n),
                      mp_obj_new_int(g_places_n_entries));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_max),
                      mp_obj_new_int(PLACES_MAX));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_default_res),
                      mp_obj_new_int(g_places_default_res));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_scale),
                      mp_obj_new_float(g_places_scale));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_origin_lat),
                      mp_obj_new_float(g_places_origin_lat_rad * (180.0 / M_PI)));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_origin_lng),
                      mp_obj_new_float(g_places_origin_lng_rad * (180.0 / M_PI)));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_places_info_obj, mod_places_info);

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_places_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_places) },
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&mod_places_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_cell),      MP_ROM_PTR(&mod_places_cell_obj) },
    { MP_ROM_QSTR(MP_QSTR_observe),   MP_ROM_PTR(&mod_places_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_classes),   MP_ROM_PTR(&mod_places_classes_obj) },
    { MP_ROM_QSTR(MP_QSTR_visits),    MP_ROM_PTR(&mod_places_visits_obj) },
    { MP_ROM_QSTR(MP_QSTR_neighbors), MP_ROM_PTR(&mod_places_neighbors_obj) },
    { MP_ROM_QSTR(MP_QSTR_cells),     MP_ROM_PTR(&mod_places_cells_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),     MP_ROM_PTR(&mod_places_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_info),      MP_ROM_PTR(&mod_places_info_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_places_globals, sentai_places_globals_table);
static const mp_obj_module_t sentai_places_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_places_globals,
};
