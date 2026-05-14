// sentai_places.cc — ObjectsPlan L3 (Stage 11.B + 11.D) implementation.
//
// Pure data + math.  H3 calls go through libh3 (sim/CMakeLists.txt and
// examples/sentai_runtime/CMakeLists.txt link libh3_{sim,arm}).  No HW.
// One .cc compiles for ARM (sentai_runtime) and SIM (sentai_sim).

#include "sentai_places.h"

#include <math.h>
#include <string.h>

#include "h3api.h"

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t plr_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  #define SENTAI_PLR_SDRAM_BSS  /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t plr_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  #define SENTAI_PLR_SDRAM_BSS  __attribute__((section(".sdram_bss"), aligned(4)))
#endif

// ---- State (per [[itcm-budget]]: keep big static buffers in SDRAM) -----
SENTAI_PLR_SDRAM_BSS static sentai_place_t        g_places[SENTAI_PLACES_MAX];
SENTAI_PLR_SDRAM_BSS static sentai_places_stats_t g_stats;
SENTAI_PLR_SDRAM_BSS static uint16_t              g_next_id_storage;
#define g_next_id  g_next_id_storage

static inline void plr_ensure_id_init(void) {
    if (g_next_id == 0) g_next_id = 1;
}

// ---- Helpers -----------------------------------------------------------

static inline int plr_finite3(float x, float y, float z) {
    return isfinite(x) && isfinite(y) && isfinite(z);
}

static int plr_find(uint8_t id) {
    if (id == 0) return -1;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        if (g_places[i].status != PLR_FREE && g_places[i].id == id) return i;
    }
    return -1;
}

static int plr_first_free(void) {
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        if (g_places[i].status == PLR_FREE) return i;
    }
    return -1;
}

// Pick LRU non-CONFIRMED slot (CONFIRMED is protected from eviction).
static int plr_oldest_evictable(void) {
    int best = -1;
    uint32_t best_ms = 0xFFFFFFFFu;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        if (g_places[i].status == PLR_FREE) continue;
        if (g_places[i].status == PLR_CONFIRMED) continue;
        if (g_places[i].last_visit_ms <= best_ms) {
            best_ms = g_places[i].last_visit_ms;
            best = i;
        }
    }
    return best;
}

static uint8_t plr_alloc_id(void) {
    plr_ensure_id_init();
    for (int tries = 0; tries < 256; tries++) {
        uint8_t cand = (uint8_t)(g_next_id & 0xFF);
        if (cand == 0) cand = 1;
        g_next_id = (uint16_t)(cand + 1);
        if (plr_find(cand) < 0) return cand;
    }
    return 0;
}

// L1 byte distance between two DESC_DIM byte vectors.
// Bounded loop, no float, no branches in the hot inner step.
// Worst case on M7 @ 800 MHz: ~64 cycles ≈ 80 ns per slot.
static uint16_t plr_l1_dist(const uint8_t* a, const uint8_t* b) {
    uint32_t s = 0;
    for (int i = 0; i < SENTAI_PLACES_DESC_DIM; i++) {
        int d = (int)a[i] - (int)b[i];
        s += (uint32_t)(d < 0 ? -d : d);
    }
    // L1 max is 255 * DESC_DIM = 16320 (fits in uint16_t).
    return (uint16_t)s;
}

// Score: 100 = identical, 0 = max distance.  Integer math only.
static uint8_t plr_score_pct(uint16_t l1_dist) {
    const uint32_t max_dist = 255u * (uint32_t)SENTAI_PLACES_DESC_DIM;
    uint32_t pct = 100u - ((uint32_t)l1_dist * 100u + max_dist / 2u) / max_dist;
    if (pct > 100u) pct = 100u;     // clamp on rounding edge
    return (uint8_t)pct;
}

// Build the H3 candidate ring once per query.  k=0 → just the cell;
// k>=1 → cell + gridDisk(cell, k).  Ring size bounded by 1 + 3k(k+1):
// k=1 → 7, k=2 → 19, k=3 → 37.  We cap at k=3 so the static ring buffer
// stays small (37 cells × 8 B = 296 B stack — well within MP-task stack).
#define PLR_MAX_RING 37
static int plr_build_ring(uint64_t cell, int k_disk, uint64_t* out_ring) {
    if (cell == 0) return 0;
    if (k_disk < 0) k_disk = 0;
    if (k_disk > 3) k_disk = 3;
    if (k_disk == 0) {
        out_ring[0] = cell;
        return 1;
    }
    // gridDisk fills `out` with up to 3k(k+1)+1 cells; unused slots are 0.
    int64_t max_n = 0;
    if (maxGridDiskSize(k_disk, &max_n) != E_SUCCESS || max_n <= 0) {
        out_ring[0] = cell;
        return 1;
    }
    if (max_n > PLR_MAX_RING) max_n = PLR_MAX_RING;
    if (gridDisk(cell, k_disk, out_ring) != E_SUCCESS) {
        out_ring[0] = cell;
        return 1;
    }
    int n = 0;
    for (int i = 0; i < (int)max_n; i++) if (out_ring[i] != 0) n++;
    return n;
}

// Cell membership check on a small ring (linear, < 40 entries).
static int plr_cell_in_ring(uint64_t cell, const uint64_t* ring, int n) {
    for (int i = 0; i < n; i++) if (ring[i] == cell) return 1;
    return 0;
}

static void plr_init_slot(int idx, uint8_t id, uint64_t h3_cell,
                          const uint8_t* desc, float x, float y, float z) {
    sentai_place_t* p = &g_places[idx];
    memset(p, 0, sizeof(*p));
    p->id            = id;
    p->status        = PLR_TENTATIVE;
    p->visits        = 1;
    p->h3_cell       = h3_cell;
    p->p_W[0]        = x;
    p->p_W[1]        = y;
    p->p_W[2]        = z;
    if (desc) {
        memcpy(p->desc, desc, SENTAI_PLACES_DESC_DIM);
        p->desc_set = 1;
    }
    uint32_t now = plr_now_ms();
    p->first_seen_ms = now;
    p->last_visit_ms = now;
}

// ---- Public API --------------------------------------------------------

int sentai_places_clear(void) {
    int cleared = 0;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        if (g_places[i].status != PLR_FREE) cleared++;
    }
    memset(g_places, 0, sizeof(g_places));
    return cleared;
}

int sentai_places_add(uint64_t h3_cell, const uint8_t* desc,
                      float x, float y, float z) {
    if (!plr_finite3(x, y, z)) { g_stats.oob_rejected++; return -1; }
    if (h3_cell == 0 && desc == NULL) {
        // A place with neither spatial index nor descriptor is unusable.
        g_stats.oob_rejected++;
        return -1;
    }
    int slot = plr_first_free();
    if (slot < 0) {
        slot = plr_oldest_evictable();
        if (slot < 0) return -3;
        g_stats.evictions++;
    }
    uint8_t id = plr_alloc_id();
    if (id == 0) return -3;

    plr_init_slot(slot, id, h3_cell, desc, x, y, z);
    g_stats.adds++;
    int used = sentai_places_count();
    if ((uint16_t)used > g_stats.hwm_used) g_stats.hwm_used = (uint16_t)used;
    return id;
}

int sentai_places_get(uint8_t id, sentai_place_t* out) {
    int idx = plr_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    if (out) memcpy(out, &g_places[idx], sizeof(*out));
    return 0;
}

int sentai_places_observe(uint8_t id) {
    int idx = plr_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    sentai_place_t* p = &g_places[idx];
    if (p->visits < 255) p->visits++;
    if (p->status == PLR_TENTATIVE && p->visits >= 2) {
        p->status = PLR_CONFIRMED;
    }
    p->last_visit_ms = plr_now_ms();
    g_stats.observations++;
    return 0;
}

int sentai_places_set_status(uint8_t id, uint8_t status) {
    if (status > PLR_CONFIRMED) return -2;
    int idx = plr_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    g_places[idx].status = status;
    return 0;
}

int sentai_places_remove(uint8_t id) {
    int idx = plr_find(id);
    if (idx < 0) { g_stats.bad_ids++; return -1; }
    memset(&g_places[idx], 0, sizeof(g_places[idx]));
    g_stats.removes++;
    return 0;
}

int sentai_places_count(void) {
    int n = 0;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        if (g_places[i].status != PLR_FREE) n++;
    }
    return n;
}

int sentai_places_list(sentai_place_t* out, int max) {
    int n = 0;
    for (int i = 0; i < SENTAI_PLACES_MAX && n < max; i++) {
        if (g_places[i].status == PLR_FREE) continue;
        if (out) memcpy(&out[n], &g_places[i], sizeof(out[0]));
        n++;
    }
    return n;
}

sentai_places_match_t sentai_places_query(const uint8_t* desc,
                                          uint64_t cell_ring,
                                          int k_disk,
                                          int thresh_pct) {
    sentai_places_match_t r = { 0, 0, 0xFFFFu };
    g_stats.queries++;
    if (desc == NULL) return r;

    uint64_t ring[PLR_MAX_RING];
    int ring_n = plr_build_ring(cell_ring, k_disk, ring);

    int     best_idx  = -1;
    uint16_t best_l1  = 0xFFFFu;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        const sentai_place_t* p = &g_places[i];
        if (p->status == PLR_FREE) continue;
        if (!p->desc_set) continue;
        if (ring_n > 0) {
            // Spatial prefilter: skip slots outside the H3 cell ring.
            if (!plr_cell_in_ring(p->h3_cell, ring, ring_n)) continue;
        }
        uint16_t l1 = plr_l1_dist(desc, p->desc);
        if (l1 < best_l1) {
            best_l1  = l1;
            best_idx = i;
        }
    }
    if (best_idx < 0) return r;
    r.id        = g_places[best_idx].id;
    r.l1_dist   = best_l1;
    r.score_pct = plr_score_pct(best_l1);
    if ((int)r.score_pct >= thresh_pct) g_stats.matches_above_thresh++;
    return r;
}

void sentai_places_stats(sentai_places_stats_t* out_counters,
                         int* out_used,
                         int* out_tentative,
                         int* out_confirmed) {
    if (out_counters) *out_counters = g_stats;
    int used = 0, tent = 0, conf = 0;
    for (int i = 0; i < SENTAI_PLACES_MAX; i++) {
        switch (g_places[i].status) {
            case PLR_FREE:                              break;
            case PLR_TENTATIVE: tent++; used++;         break;
            case PLR_CONFIRMED: conf++; used++;         break;
            default:                  used++;           break;
        }
    }
    if (out_used)      *out_used      = used;
    if (out_tentative) *out_tentative = tent;
    if (out_confirmed) *out_confirmed = conf;
}
