---
name: op-s10-w19-markers-plan-2026-05-20
description: "OP-S10-W19 plan — hard-rename sentai.aruco + sentai.whycon into a unified sentai.markers namespace with backend param.  No shim, no dicts (structs only — NASA embedded discipline).  WhyCon needs PnP first.  Then move ablations from ARM to SIM, focus on quality (closure, detection rate, ID disambiguation), not speed.  Filed 2026-05-20 end-of-day pre-compaction; T1 implementation deferred to next session."
metadata:
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## Goal

Unify the two parallel fiducial-marker namespaces (`sentai.aruco`
and `sentai.whycon`) into a single `sentai.markers` namespace
with a backend parameter at `init()`.  Consumers (SafetyTask,
calib autotune, missions, FlowBaseline gate) consume the unified
API; backend choice is runtime-configurable.

## Operator constraints (2026-05-20 EOD)

- **Hard rename**, no compatibility shim.  `sentai.aruco`
  module disappears.
- **NO dictionaries** crossing the MicroPython heap.  Structs
  only, with caller-allocated `bytearray(sizeof(...))` buffers
  filled by the binding.  Mirrors the existing
  `sentai.flow.gray_snap()` pattern.
- `sentai.calib` namespace stays, but internal calls to
  `sentai_aruco_get_latest()` rewrite to `sentai_markers_get_latest()`.
- `calib.set_context()` extension TBD — decide after T1 lands.

## API sketch

```
sentai.markers.init(backend = "aruco" | "whycon")
sentai.markers.set_intrinsics(fx, fy, cx, cy)
sentai.markers.set_marker_size(meters)      # moved from sentai.aruco
sentai.markers.detect_from_camera() -> n_detected
sentai.markers.get_count()
sentai.markers.get_pose(i, out_buf)         # SentaiMarkersPose struct
sentai.markers.get_stats(out_buf)           # SentaiMarkersStats struct
sentai.markers._detect_cyc()
sentai.markers._use_rolling(0|1)
sentai.markers._test_pgm(path)
sentai.markers._verify_threshold(block)
sentai.markers._thresh_cycles()
sentai.markers._thresh_rolling_verify(block)
sentai.markers._thresh_rolling_cyc()
sentai.markers._thresh_nocache(block)
sentai.markers._backend() -> "aruco" | "whycon"

# Backend-specific extensions:
sentai.markers.aruco.rvec_to_R / R_to_rvec
sentai.markers.aruco._test_synth_and_detect
sentai.markers.whycon._test_synth(n, r)
sentai.markers.whycon._set_concentric
sentai.markers.whycon._stage_cyc
sentai.markers.whycon._get_marker_details(i, out_buf)
```

## Struct ABI (locked)

```c
typedef struct {
    int32_t  id;             /* -1 if backend doesn't assign ID */
    float    pixel_cx, pixel_cy;
    float    tvec_cam[3];    /* NaN if !pose_valid */
    float    rvec_cam[3];    /* NaN if !pose_valid */
    float    reproj_err_px;
    uint8_t  backend;
    uint8_t  pose_valid;
    uint16_t _pad;
} SentaiMarkersPose;   /* sizeof == 48, static_asserted */

typedef struct {
    uint32_t frames_total;
    uint32_t frames_with_detect;
    uint32_t markers_total;
    uint32_t last_detect_us;
    uint8_t  backend;
    uint8_t  _pad[3];
} SentaiMarkersStats;  /* sizeof == 20 */
```

## Prerequisite — WhyCon PnP

WhyCon currently emits only `(cx, cy, axis_a, axis_b, angle)`.
For the unified `pose_valid = 1` contract:

- **W19-T2** — closed-form z from axis_a + physical diameter.
  `z = fx * physical_diameter / (2 * axis_a)`.  No iterative
  PnP needed.  Tilt approximation from `a/b` axis ratio
  acceptable for near-frontal landing scenarios (<5° tilt
  error).
- **W19-T3** — multi-marker constellation pose, 3 asymmetric
  triangle (20/30/36 cm).  Solves yaw + per-marker ID
  ambiguity.

Without these, the rename can still ship — but the WhyCon
backend will return `pose_valid = 0` and `calib.task_start`
will refuse to run on it.

## SIM A/B campaign (post-T1+T2)

Operator explicitly redirects ablations from on-ARM speed
measurements to in-SIM quality measurements:

- detection rate vs altitude (0.5 m → 3 m)
- closure error / ID disambiguation on s127 FlowBaseline
- Flow diamond vs exhaustive closure delta over 30-s walks
- PnP-z accuracy per backend → hover variance

Promotion criteria:

- WhyCon default if `closure_err ≤ ε_aruco` AND detection
  rate ≥ ArUco at all altitudes.
- Flow diamond default if `closure_err ≤ 1.5× ε_exhaustive`
  AND no new "stuck" failure in `_host_flow_validate.py`.

## Execution plan (next session)

1. `sentai_markers.h` — backend enum + struct defs + ABI
   asserts.
2. `sentai_markers_detect()` C dispatcher.
3. `modsentai_markers.c` MP binding — caller-allocated
   bytearray buffers, no dicts.
4. Refactor internal call sites: `sentai_calib*`,
   `sentai_safety_task.cc`.
5. Hard-rename `sentai.aruco` module out of existence.
6. QSTR regen + build + flash + smoke.
7. WhyCon PnP-z (T2).
8. SIM A/B campaign launches.

Estimated: T1 half-day, T2 ~2 hours, T3+SIM campaign 1-2
days running + analysis.

## Open questions

1. File rename: `git mv sentai_aruco.cc sentai_markers.cc`
   to preserve blame, or create new file?  Preference: rename.
2. WhyCon constellation impl source: chronorobotics/whycon
   GitHub or Krajnik JINT 2014 from scratch?
3. `_thresh_nocache` and similar HW diagnostics — keep in
   `sentai.markers` or move to `sentai.diag.threshold.*`?
4. `calib.set_context()` — operator decision deferred.

## Cross-refs

- `[[op-s10-w17-whycon-2026-05-19]]` — WhyCon-lite WP.
- `[[op-s10-w18-flow-search-2026-05-20]]` — Flow Diamond WP.
- `[[ipc-clamp-in-worker-not-host]]` — hard-rule that
  applies to any new IPC sentinel introduced.
- `[[experiments-in-own-folder-log-dead-ends]]` — every SIM
  A/B campaign run lives in its own `sNNN_` folder.
- `[[no-heavy-data-through-mp]]` — load-bearing rationale
  for the struct/bytearray pattern.
- WP doc: `ideas/objects_plan/OP-S10-W19_markers_unified.md`.
