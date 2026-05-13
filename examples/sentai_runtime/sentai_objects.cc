// sentai_objects.cc — ObjectsPlan L2 (Stage 1) implementation.
//
// Pure data + math, no HW.  Single .cc compiles for ARM (sentai_runtime)
// and SIM (sentai_sim).  ARM places state in .sdram_text via the SDRAM
// linker section; SIM lands in plain .data.

#include "sentai_objects.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t obj_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  #define SENTAI_OBJ_SDRAM_TEXT  /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t obj_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  #define SENTAI_OBJ_SDRAM_TEXT  __attribute__((section(".sdram_text")))
#endif

// ---- State (per [[itcm-budget]]: keep big static buffers in SDRAM) -----
// On ARM, parked in .sdram_bss to keep DTCM small.  On SIM these attrs
// are no-ops (the section is unknown but harmless).
#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #define SENTAI_OBJ_SDRAM_BSS  /* nothing */
#else
  #define SENTAI_OBJ_SDRAM_BSS  __attribute__((section(".sdram_bss"), aligned(4)))
#endif

SENTAI_OBJ_SDRAM_BSS static sentai_object_t    g_objects[SENTAI_OBJECTS_MAX];
SENTAI_OBJ_SDRAM_BSS static sentai_obj_stats_t g_stats;
SENTAI_OBJ_SDRAM_BSS static uint16_t           g_next_id_storage;
#define g_next_id  g_next_id_storage          // initialized to 1 in ensure_init below

static inline void obj_ensure_id_init(void) {
    if (g_next_id == 0) g_next_id = 1;
}

// ---- Helpers -----------------------------------------------------------

static inline int obj_finite3(float x, float y, float z) {
    return isfinite(x) && isfinite(y) && isfinite(z);
}

static inline int obj_finite_cov6(const float* c) {
    for (int i = 0; i < 6; i++) {
        if (!isfinite(c[i])) return 0;
    }
    return 1;
}

// Find slot by id (linear, N=32). Returns index or -1.
static int obj_find(uint8_t id) {
    if (id == 0) return -1;
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        if (g_objects[i].status != OBJ_FREE && g_objects[i].id == id) return i;
    }
    return -1;
}

static int obj_first_free(void) {
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        if (g_objects[i].status == OBJ_FREE) return i;
    }
    return -1;
}

// Pick oldest STALE slot (lowest last_seen_ms among OBJ_STALE).  Returns
// index or -1 if none.
static int obj_oldest_stale(void) {
    int best = -1;
    uint32_t best_ms = 0xFFFFFFFFu;
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        if (g_objects[i].status != OBJ_STALE) continue;
        if (g_objects[i].last_seen_ms <= best_ms) {
            best_ms = g_objects[i].last_seen_ms;
            best = i;
        }
    }
    return best;
}

// Allocate next non-zero id.  Skips ids in use.  Linear probe — bounded by
// 256 (the id space).  Worst case: 256 - 32 occupied = 224 finds; still ≪ 1 µs.
static uint8_t obj_alloc_id(void) {
    obj_ensure_id_init();
    for (int tries = 0; tries < 256; tries++) {
        uint8_t cand = (uint8_t)(g_next_id & 0xFF);
        if (cand == 0) cand = 1;
        g_next_id = (uint16_t)(cand + 1);
        if (obj_find(cand) < 0) return cand;
    }
    return 0;  // exhausted — caller must treat as -3 (SERR_OBJ_FULL)
}

// Clamp diagonal entries (xx, yy, zz) of cov6 to >= sigma_min^2.
// Return 1 if any clamping happened, 0 otherwise.
static int obj_clamp_cov_psd(float* c) {
    const float kSigmaMin2 = 1e-6f;  // 1 mm² floor
    int clamped = 0;
    if (c[0] < kSigmaMin2) { c[0] = kSigmaMin2; clamped = 1; }  // xx
    if (c[3] < kSigmaMin2) { c[3] = kSigmaMin2; clamped = 1; }  // yy
    if (c[5] < kSigmaMin2) { c[5] = kSigmaMin2; clamped = 1; }  // zz
    return clamped;
}

static void obj_default_cov(float* c) {
    const float k = 1.0f;  // 1 m² initial uncertainty on diag
    c[0] = k; c[1] = 0;  c[2] = 0;
    c[3] = k; c[4] = 0;  c[5] = k;
}

static void obj_init_slot(int idx, uint8_t id, uint8_t class_id,
                          float x, float y, float z, const float* cov6) {
    sentai_object_t* o = &g_objects[idx];
    memset(o, 0, sizeof(*o));
    o->id              = id;
    o->status          = OBJ_TENTATIVE;
    o->class_id        = class_id;
    o->observations    = 1;
    o->p_W[0]          = x;
    o->p_W[1]          = y;
    o->p_W[2]          = z;
    if (cov6) {
        memcpy(o->cov_uppertri, cov6, 6 * sizeof(float));
    } else {
        obj_default_cov(o->cov_uppertri);
    }
    if (obj_clamp_cov_psd(o->cov_uppertri)) g_stats.cov_clamped++;
    uint32_t now = obj_now_ms();
    o->last_seen_ms    = now;
    o->last_updated_ms = now;
}

// ---- Public API --------------------------------------------------------

int sentai_objects_clear(void) {
    int cleared = 0;
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        if (g_objects[i].status != OBJ_FREE) cleared++;
    }
    memset(g_objects, 0, sizeof(g_objects));
    // counters preserved on clear (per Stage 1 "stats reflects population
    // accurately" — only used/hwm reset; lifetime counters stay).
    return cleared;
}

int sentai_objects_add(uint8_t class_id, float x, float y, float z,
                       const float* cov6) {
    if (!obj_finite3(x, y, z)) { g_stats.oob_rejected++; return -1; }
    if (cov6 && !obj_finite_cov6(cov6)) { g_stats.oob_rejected++; return -1; }
    if (class_id >= SENTAI_OBJECTS_DICT_MAX) {
        g_stats.oob_rejected++;
        return -2;
    }

    int slot = obj_first_free();
    if (slot < 0) {
        slot = obj_oldest_stale();
        if (slot < 0) return -3;
        g_stats.evictions++;
    }

    uint8_t id = obj_alloc_id();
    if (id == 0) return -3;

    obj_init_slot(slot, id, class_id, x, y, z, cov6);
    g_stats.adds++;
    int used = sentai_objects_count();
    if ((uint16_t)used > g_stats.hwm_used) g_stats.hwm_used = (uint16_t)used;
    return id;
}

int sentai_objects_get(uint8_t id, sentai_object_t* out) {
    int idx = obj_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    g_stats.gets++;
    if (out) memcpy(out, &g_objects[idx], sizeof(*out));
    return 0;
}

int sentai_objects_mark_visited(uint8_t id) {
    int idx = obj_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    g_objects[idx].visited = 1;
    g_objects[idx].last_updated_ms = obj_now_ms();
    return 0;
}

int sentai_objects_set_status(uint8_t id, uint8_t status) {
    if (status > OBJ_LOST) return -2;
    int idx = obj_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    g_objects[idx].status          = status;
    g_objects[idx].last_updated_ms = obj_now_ms();
    return 0;
}

int sentai_objects_remove(uint8_t id) {
    int idx = obj_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    memset(&g_objects[idx], 0, sizeof(g_objects[idx]));
    g_stats.removes++;
    return 0;
}

int sentai_objects_count(void) {
    int n = 0;
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        if (g_objects[i].status != OBJ_FREE) n++;
    }
    return n;
}

int sentai_objects_list(sentai_object_t* out, int max) {
    int n = 0;
    for (int i = 0; i < SENTAI_OBJECTS_MAX && n < max; i++) {
        if (g_objects[i].status == OBJ_FREE) continue;
        if (out) memcpy(&out[n], &g_objects[i], sizeof(out[0]));
        n++;
    }
    return n;
}

void sentai_objects_stats(sentai_obj_stats_t* out_counters,
                          int* out_used,
                          int* out_confirmed,
                          int* out_coasting,
                          int* out_stale) {
    if (out_counters) *out_counters = g_stats;
    int used = 0, conf = 0, coast = 0, stale = 0;
    for (int i = 0; i < SENTAI_OBJECTS_MAX; i++) {
        switch (g_objects[i].status) {
            case OBJ_FREE:                   break;
            case OBJ_CONFIRMED: conf++;  used++; break;
            case OBJ_COASTING:  coast++; used++; break;
            case OBJ_STALE:     stale++; used++; break;
            default:            used++; break;  // TENTATIVE / LOST
        }
    }
    if (out_used)      *out_used      = used;
    if (out_confirmed) *out_confirmed = conf;
    if (out_coasting)  *out_coasting  = coast;
    if (out_stale)     *out_stale     = stale;
}
