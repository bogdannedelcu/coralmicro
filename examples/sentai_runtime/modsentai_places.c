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
    int       skippable;                              // 0=normal, 1=no features (water/uniform) — SFLVP skips these
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

// Inverse of places_local_to_geo — go from geodetic (radians) back to
// local (x, y) meters, accounting for the scene-scale factor.
static void places_geo_to_local(double lat_rad, double lng_rad,
                                double *x_m, double *y_m) {
    double dLat = lat_rad - g_places_origin_lat_rad;
    double dLng = lng_rad - g_places_origin_lng_rad;
    double Y = dLat * PLACES_EARTH_R_M;
    double X = dLng * PLACES_EARTH_R_M * cos(g_places_origin_lat_rad);
    /* Reverse scene-scale */
    if (g_places_scale > 0.0) {
        *x_m = X / g_places_scale;
        *y_m = Y / g_places_scale;
    } else {
        *x_m = X;
        *y_m = Y;
    }
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

// ===================== Embedding (Stage 11.D) =====================
//
// Convention: 128-byte HSV histogram (4x4 spatial × 8 hue bins) produced by
// sentai_tracker.cc's hist_extract (or an offline equivalent on the host).
// Each 8-bin cell is normalized so it sums to ~255.  Match uses per-cell
// Bhattacharyya distance averaged across all 16 spatial cells, mirroring
// what sentai_tracker uses for re-id.

#define PLACES_BHATTACHARYYA_CELLS 16    /* 4×4 spatial grid */
#define PLACES_BHATTACHARYYA_BINS  8     /* hue bins per cell */

static float places_hist_bhattacharyya(const uint8_t *a, const uint8_t *b) {
    float bc = 0.0f;
    for (int c = 0; c < PLACES_BHATTACHARYYA_CELLS; c++) {
        int base = c * PLACES_BHATTACHARYYA_BINS;
        float sa = 0, sb = 0, cell = 0;
        for (int i = 0; i < PLACES_BHATTACHARYYA_BINS; i++) {
            float ai = (float)a[base + i], bi = (float)b[base + i];
            sa += ai;
            sb += bi;
            cell += sqrtf(ai * bi);
        }
        float norm = sqrtf(sa * sb);
        if (norm > 0.01f) bc += cell / norm;
    }
    bc /= (float)PLACES_BHATTACHARYYA_CELLS;
    return 1.0f - bc;       /* 0 = identical, 1 = different */
}

// places.set_embedding(cell, bytes) -> int  (0=ok, -1=cell invalid/unknown,
//                                            -2=wrong length)
static mp_obj_t mod_places_set_embedding(mp_obj_t cell_obj, mp_obj_t buf_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    if (cell == PLACES_INVALID_CELL || !isValidCell(cell)) return mp_obj_new_int(-1);
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_READ)) return mp_obj_new_int(-2);
    if (bi.len != PLACES_EMB_DIM) return mp_obj_new_int(-2);
    int idx = places_get_or_create(cell);
    if (idx < 0) return mp_obj_new_int(-1);
    memcpy(g_places[idx].embedding, bi.buf, PLACES_EMB_DIM);
    g_places[idx].has_embedding = 1;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_places_set_embedding_obj, mod_places_set_embedding);

// places.embedding(cell) -> bytes(128) | None
static mp_obj_t mod_places_embedding(mp_obj_t cell_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int idx = places_find(cell);
    if (idx < 0 || !g_places[idx].has_embedding) return mp_const_none;
    return mp_obj_new_bytes(g_places[idx].embedding, PLACES_EMB_DIM);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_embedding_obj, mod_places_embedding);

// places.match(bytes [, k=1]) -> [(cell, similarity_pct), ...] best-first
// similarity_pct = (1 - bhattacharyya) × 100, integer 0..100.
// Returns empty list if no embeddings stored or query length != 128.
static mp_obj_t mod_places_match(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(args[0], &bi, MP_BUFFER_READ)) return mp_obj_new_list(0, NULL);
    if (bi.len != PLACES_EMB_DIM) return mp_obj_new_list(0, NULL);
    int k = (n_args >= 2) ? mp_obj_get_int(args[1]) : 1;
    if (k < 1) k = 1;
    if (k > 16) k = 16;

    /* Single-pass top-k via insertion sort into a fixed-size array.
     * Distances are small floats; we negate to keep the top-k smallest
     * (highest similarity).  Result is ascending distance ⇒ best first. */
    typedef struct { float dist; H3Index cell; } cand_t;
    cand_t top[16];
    int n_top = 0;

    for (int i = 0; i < PLACES_MAX; i++) {
        if (g_places[i].cell == PLACES_INVALID_CELL) continue;
        if (!g_places[i].has_embedding) continue;
        float d = places_hist_bhattacharyya((const uint8_t *)bi.buf,
                                            g_places[i].embedding);
        /* Insert into sorted top-k array */
        if (n_top < k) {
            int p = n_top++;
            while (p > 0 && top[p-1].dist > d) {
                top[p] = top[p-1];
                p--;
            }
            top[p].dist = d;
            top[p].cell = g_places[i].cell;
        } else if (d < top[k-1].dist) {
            int p = k - 1;
            while (p > 0 && top[p-1].dist > d) {
                top[p] = top[p-1];
                p--;
            }
            top[p].dist = d;
            top[p].cell = g_places[i].cell;
        }
    }

    mp_obj_t result = mp_obj_new_list(0, NULL);
    for (int i = 0; i < n_top; i++) {
        int sim_pct = (int)((1.0f - top[i].dist) * 100.0f + 0.5f);
        if (sim_pct < 0)   sim_pct = 0;
        if (sim_pct > 100) sim_pct = 100;
        mp_obj_t pair[2] = {
            mp_obj_new_int_from_ull((unsigned long long)top[i].cell),
            mp_obj_new_int(sim_pct),
        };
        mp_obj_list_append(result, mp_obj_new_tuple(2, pair));
    }
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_places_match_obj, 1, 2, mod_places_match);

// places.center(cell) -> (x_m, y_m) — local-meter centroid of an H3 cell.
// Useful for SFLVP traversal: query neighbors via places.neighbors(c,1),
// then fly to each neighbor's center.
static mp_obj_t mod_places_center(mp_obj_t cell_obj) {
    if (!g_places_initialized) return mp_const_none;
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    LatLng p;
    if (cellToLatLng(cell, &p)) return mp_const_none;
    double x, y;
    places_geo_to_local(p.lat, p.lng, &x, &y);
    mp_obj_t pair[2] = { mp_obj_new_float(x), mp_obj_new_float(y) };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_center_obj, mod_places_center);

// places.set_skippable(cell, val) -> int
//   val=1 marks the cell as "no features / skip during SFLVP" (e.g., water,
//   uniform texture).  val=0 clears the flag.  Returns 0 ok, -1 invalid.
static mp_obj_t mod_places_set_skippable(mp_obj_t cell_obj, mp_obj_t val_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int val = mp_obj_get_int(val_obj) ? 1 : 0;
    if (cell == PLACES_INVALID_CELL || !isValidCell(cell)) return mp_obj_new_int(-1);
    int idx = places_get_or_create(cell);
    if (idx < 0) return mp_obj_new_int(-1);
    g_places[idx].skippable = val;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_places_set_skippable_obj, mod_places_set_skippable);

// places.is_skippable(cell) -> int (0=visit, 1=skip; -1 if unknown cell)
static mp_obj_t mod_places_is_skippable(mp_obj_t cell_obj) {
    H3Index cell = (H3Index)mp_obj_get_int(cell_obj);
    int idx = places_find(cell);
    if (idx < 0) return mp_obj_new_int(-1);
    return mp_obj_new_int(g_places[idx].skippable);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_places_is_skippable_obj, mod_places_is_skippable);

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
    { MP_ROM_QSTR(MP_QSTR_center),    MP_ROM_PTR(&mod_places_center_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_skippable), MP_ROM_PTR(&mod_places_set_skippable_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_skippable),  MP_ROM_PTR(&mod_places_is_skippable_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),     MP_ROM_PTR(&mod_places_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_info),      MP_ROM_PTR(&mod_places_info_obj) },
    /* Stage 11.D — HSV histogram embedding */
    { MP_ROM_QSTR(MP_QSTR_set_embedding), MP_ROM_PTR(&mod_places_set_embedding_obj) },
    { MP_ROM_QSTR(MP_QSTR_embedding),     MP_ROM_PTR(&mod_places_embedding_obj) },
    { MP_ROM_QSTR(MP_QSTR_match),         MP_ROM_PTR(&mod_places_match_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_places_globals, sentai_places_globals_table);
static const mp_obj_module_t sentai_places_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_places_globals,
};
