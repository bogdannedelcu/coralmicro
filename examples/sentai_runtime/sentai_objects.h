// sentai_objects.h — ObjectsPlan L2 (Stage 1): persistent world-frame object map.
//
// Static array of up to SENTAI_OBJECTS_MAX entries, ENU-metres world frame,
// covariance kept as 3x3 upper-triangular (xx, xy, xz, yy, yz, zz).
//
// L2 scope: storage + CRUD + eviction + NaN reject + counters.
// Higher layers (L4+) link entries back to live tracker/SLAM via tracklet_id.
//
// Reuse notes: pattern mirrors sentai_tracker (static-array, byte-enum
// status, monotonic id allocator) and modsentai_slam (init/clear/save
// lifecycle, dict-returning MP getters). No coupling to either module —
// objects is the persistent map; tracker is per-frame; slam is the EKF.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A "system model first")
// =========================================================================
// Fault model:
//   F1 add(non-finite) ........... reject, return -1, oob_rejected++ (SERR_OBJ_OOB_INPUT)
//   F2 add(class >= DICT_MAX) .... reject, return -2, oob_rejected++ (SERR_OBJ_OOB_INPUT)
//   F3 add when full ............. evict OLDEST OBJ_STALE; if none, return -3 (SERR_OBJ_FULL)
//   F4 get/remove(unknown id) .... return -1, bad_ids++ (SERR_OBJ_BAD_ID)
//   F5 input cov diag < σ_min² ... clamp, cov_clamped++ (SERR_OBJ_COV_CLAMPED)
//
// Execution model (L2):
//   - All entry points are MP-task context only (REPL or /main.py).
//   - No ISR access.  No FreeRTOS primitives required — operations are
//     bounded loops over a 32-slot static array (worst case ≪ 1 µs each).
//   - Backing store is BSS-zeroed at boot; no init call required.
//
// Recovery model:
//   - All errors are local: caller receives a negative return code and
//     can decide policy.  Map state remains consistent on every error path.
//   - sentai_objects_clear() is the only escalation; preserves lifetime
//     counters (adds/removes/evictions) for post-mortem.
//
// Concurrency contract:
//   - L2: single-writer, single-reader (MP task).  No internal locking.
//   - L4+: when sentai_tracker / detection_task ingest into objects from a
//     different task, callers MUST serialize via an external mutex (TBD).
//     The static snapshot buffer used by sentai_objects_list is part of
//     this contract.
//
// Memory:
//   - Static state: SENTAI_OBJECTS_MAX * sizeof(sentai_object_t) ≈ 2 KB
//     (BSS on SIM, .sdram_bss on ARM).
//   - Zero heap, zero per-call malloc.
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity (per ideas/objects.md §4.1) ------------------------------
#define SENTAI_OBJECTS_MAX   32   // 32 * sizeof(sentai_object_t) ≈ 2 KB SDRAM
#define SENTAI_OBJECTS_DICT_MAX 80  // class_id range guard (COCO + headroom)

// ---- Status byte enum (per Stage 1 spec) -------------------------------
typedef enum {
    OBJ_FREE      = 0,
    OBJ_TENTATIVE = 1,
    OBJ_CONFIRMED = 2,
    OBJ_COASTING  = 3,
    OBJ_STALE     = 4,
    OBJ_LOST      = 5,
} sentai_obj_status_t;

// ---- Per-slot record (~64 B) -------------------------------------------
typedef struct {
    uint8_t  id;                 // monotonic, 0 = invalid (FREE slots set 0)
    uint8_t  status;             // sentai_obj_status_t
    uint8_t  class_id;
    uint8_t  observations;
    float    p_W[3];             // ENU world-frame metres
    float    cov_uppertri[6];    // (xx, xy, xz, yy, yz, zz)
    uint32_t last_seen_ms;       // ms since boot of last observation
    uint32_t last_updated_ms;    // ms since boot of last state change
    uint32_t descriptor[2];      // L3 (places) populates HSV-hash; L2 leaves 0
    uint16_t tracklet_id;        // L4 wires to sentai_tracker; L2 leaves 0
    uint8_t  visited;
    uint8_t  _pad;
} sentai_object_t;

// ---- Counters (returned by sentai_objects_stats) -----------------------
typedef struct {
    uint32_t adds;
    uint32_t gets;
    uint32_t removes;
    uint32_t evictions;
    uint32_t oob_rejected;
    uint32_t bad_ids;
    uint32_t cov_clamped;
    uint16_t hwm_used;            // high-water mark of `used`
} sentai_obj_stats_t;

// ---- C API -------------------------------------------------------------

// Reset map to all-FREE.  Returns number of slots that were non-FREE.
int sentai_objects_clear(void);

// Insert a new object.  cov6 may be NULL → identity-σ_min² diag.
// Return: >=0 assigned id ; -1 NaN/Inf input ; -2 class out of range ;
//         -3 map full and no STALE to evict.
int sentai_objects_add(uint8_t class_id, float x, float y, float z,
                       const float* cov6);

// Lookup by id.  Copies into *out, returns 0 on success, -1 if not found.
int sentai_objects_get(uint8_t id, sentai_object_t* out);

// Mark visited.  Returns 0 ok, -1 bad id.
int sentai_objects_mark_visited(uint8_t id);

// Status transition (used by L4+).  Returns 0 ok, -1 bad id, -2 bad status.
int sentai_objects_set_status(uint8_t id, uint8_t status);

// Remove by id.  Returns 0 ok, -1 bad id.
int sentai_objects_remove(uint8_t id);

// How many slots are currently non-FREE.
int sentai_objects_count(void);

// Snapshot copy of all non-FREE slots (compacted) into out[0..max-1].
// Returns number of slots written.
int sentai_objects_list(sentai_object_t* out, int max);

// Read counters + status histogram.  Pointers may not be NULL.
//   confirmed/coasting/stale: counts of slots in those states.
void sentai_objects_stats(sentai_obj_stats_t* out_counters,
                          int* out_used,
                          int* out_confirmed,
                          int* out_coasting,
                          int* out_stale);

#ifdef __cplusplus
}
#endif
