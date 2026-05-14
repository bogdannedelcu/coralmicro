// sentai_places.h — ObjectsPlan L3 (Stage 11.B + 11.D): H3-indexed
// place gallery with per-place HSV-style descriptor.
//
// A "place" is a {H3 cell, descriptor, world pose, visit counters}
// tuple stored in a fixed 64-slot array.  Descriptor is opaque bytes
// to this layer — produced by an upper layer (camera+tracker on ARM,
// REPL/Python on SIM).  Match is L1 byte distance, deterministic and
// bounded: < 4 µs on M7 worst-case for the full 64-slot sweep.
//
// L3 scope: storage + CRUD + descriptor match + H3 neighbour query
// (via libh3) + visit counters + LRU eviction.  Higher layers (L5
// slam, L6 explore) consume the gallery for coarse loop closure +
// SFLVP exploration.
//
// Re-implemented for the layered ObjectsPlan (NOT cherry-picked from
// feature/ov5640-camera-support — see [[no-broken-branch-test-reuse]]).
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A "system model first")
// =========================================================================
// Fault model:
//   F1 add(h3_cell == 0)             reject, return -1, oob_rejected++
//   F2 add(desc_len != DESC_DIM)     reject, return -2, oob_rejected++
//   F3 add when full                 evict OLDEST non-CONFIRMED; if none,
//                                    return -3 (SERR_PLR_FULL)
//   F4 get/remove(unknown id)        return -1, bad_ids++
//   F5 query(no slot in cell ring)   return (0, 0) — caller treats 0 as miss
//   F6 set_status(bad status)        return -2
//
// Execution model (L3):
//   - All entry points are MP-task context only.  No ISR access.
//   - Bounded loops: linear scan over 64 slots → < 4 µs per call on M7.
//   - L1 byte match over DESC_DIM bytes per slot → < 1 µs per slot.
//   - libh3 calls (gridDisk, latLngToCell) are synchronous, single-pass.
//   - BSS-zeroed at boot; no init call required.
//
// Recovery model:
//   - All errors local.  Caller receives a negative return code; map
//     state remains consistent on every error path.
//   - sentai_places_clear() is the only escalation.  Lifetime counters
//     (adds/removes/evictions/queries/matches) preserved across clear.
//
// Concurrency contract:
//   - L3: single-writer, single-reader (MP task).  No internal locking.
//   - L5+: when a future C task ingests descriptors from camera+tracker
//     in a different task, callers MUST serialize via an external mutex
//     (TBD).  The static snapshot buffer used by sentai_places_list /
//     sentai_places_query_topk is part of this contract.
//
// Memory:
//   - Static state: ~64 × (96 + 4) = ~6.4 KB.  Zero heap, zero per-call
//     malloc.  ARM placement: .sdram_bss.  SIM: plain .bss.
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity (per ideas/objects_plan.md §13.6 Stage 11.B) -------------
#define SENTAI_PLACES_MAX        64    // 64 slots × ~96 B ≈ 6 KB SDRAM
#define SENTAI_PLACES_DESC_DIM   64    // 16 hue × 4 sat bins (uint8_t each)

// ---- Status byte enum (compact: places only need 3 states) -------------
typedef enum {
    PLR_FREE      = 0,
    PLR_TENTATIVE = 1,    // 1 visit
    PLR_CONFIRMED = 2,    // ≥ 2 visits — protected from eviction
} sentai_places_status_t;

// ---- Per-slot record (~96 B, padded to 4-byte align) -------------------
typedef struct {
    uint8_t  id;                                // monotonic, 0 = invalid (FREE slots set 0)
    uint8_t  status;                            // sentai_places_status_t
    uint8_t  visits;                            // count of observe() calls (saturates at 255)
    uint8_t  desc_set;                          // 1 if descriptor field is meaningful
    uint64_t h3_cell;                           // H3Index (0 == "no cell")
    float    p_W[3];                            // ENU world-frame metres (origin at takeoff)
    uint32_t first_seen_ms;                     // ms since boot of add()
    uint32_t last_visit_ms;                     // ms since boot of last observe() / add()
    uint8_t  desc[SENTAI_PLACES_DESC_DIM];      // opaque descriptor bytes
} sentai_place_t;

// ---- Counters (returned by sentai_places_stats) ------------------------
typedef struct {
    uint32_t adds;
    uint32_t removes;
    uint32_t evictions;
    uint32_t observations;
    uint32_t queries;
    uint32_t matches_above_thresh;
    uint32_t oob_rejected;
    uint32_t bad_ids;
    uint16_t hwm_used;                          // high-water mark of `used`
} sentai_places_stats_t;

// ---- Match result ------------------------------------------------------
// `score` is a 0..100 integer percent: 100 = identical bytes, 0 = max
// distance.  `id == 0` means "no match" (gallery empty or no candidate
// in the H3 cell ring when `cell != 0`).
typedef struct {
    uint8_t  id;
    uint8_t  score_pct;
    uint16_t l1_dist;                           // raw L1 byte distance (lower = better)
} sentai_places_match_t;

// ---- C API -------------------------------------------------------------

// Reset map to all-FREE.  Returns number of slots that were non-FREE.
// Lifetime counters (adds, removes, evictions, queries, matches) are
// preserved for post-mortem.
int sentai_places_clear(void);

// Insert a new place.  `desc` may be NULL (descriptor slot left zero,
// `desc_set` = 0).  `h3_cell` may be 0 (place stored without spatial
// index — query() will only match on descriptor in this case).
// Return: >=0 assigned id ; -1 NaN/Inf p_W or zero h3_cell when desc
//         also NULL ; -2 (reserved for binding-side desc_len mismatch) ;
//         -3 gallery full and no non-CONFIRMED slot to evict.
int sentai_places_add(uint64_t h3_cell, const uint8_t* desc,
                      float x, float y, float z);

// Lookup by id.  Copies into *out, returns 0 on success, -1 if not found.
int sentai_places_get(uint8_t id, sentai_place_t* out);

// Bump visits + last_visit_ms.  Auto-promotes TENTATIVE → CONFIRMED at
// visits >= 2.  Returns 0 ok, -1 bad id.
int sentai_places_observe(uint8_t id);

// Status transition (caller-driven).  Returns 0 ok, -1 bad id, -2 bad status.
int sentai_places_set_status(uint8_t id, uint8_t status);

// Remove by id.  Returns 0 ok, -1 bad id.
int sentai_places_remove(uint8_t id);

// How many slots are currently non-FREE.
int sentai_places_count(void);

// Snapshot copy of all non-FREE slots (compacted) into out[0..max-1].
// Returns number of slots written.
int sentai_places_list(sentai_place_t* out, int max);

// Query best descriptor match.
//   desc        — DESC_DIM bytes to match against
//   cell_ring   — non-zero: restrict candidates to slots whose h3_cell
//                 equals one of {cell_ring} ∪ gridDisk(cell_ring, k_disk).
//                 Zero: scan whole gallery (no spatial prefilter).
//   k_disk      — H3 ring radius (0..2 typical).  Ignored when cell_ring=0.
//   thresh_pct  — score floor (0..100); matches at or above counted in
//                 stats.matches_above_thresh.
// Returns sentai_places_match_t.  `.id == 0` ⇒ no candidate found.
sentai_places_match_t sentai_places_query(const uint8_t* desc,
                                          uint64_t cell_ring,
                                          int k_disk,
                                          int thresh_pct);

// Read counters + status histogram.  Pointers may be NULL.
void sentai_places_stats(sentai_places_stats_t* out_counters,
                         int* out_used,
                         int* out_tentative,
                         int* out_confirmed);

#ifdef __cplusplus
}
#endif
