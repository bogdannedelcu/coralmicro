# OP-S10-W19 — `sentai.markers` unified namespace + SIM A/B migration

**Status**: planning filed 2026-05-20 (end-of-day pre-compaction).  T1
implementation deferred to next session.

## 1. Why this WP exists

After OP-S10-W17 + W18 produced two parallel fiducial-marker
detectors (`sentai.aruco` and `sentai.whycon`), operator
2026-05-20 EOD requested:

> "as vrea sa avem cele 2 optiuni ArUco si WhyCon dar sa aliniem
> si namespace-urile, cumva parametrizand ce markeri sa foloseasca.
> [...] Facem hard rename la refactorizare. Fara dictionare te rog,
> suntem pe Embeded... structs only."

Plus:

> "E important acum sa facem curatenie prin cod si sa rerulam
> experimentele pe cod refactorizat in simulator.  [...] facem
> ablation si in simulator acum nu pe viteza de procesare, pe
> calitate algoritmi."

Two intertwined deliverables: refactor + SIM A/B re-validation.

## 2. Design — `sentai.markers` namespace

### 2.1 Hard rename (no shim)

The `sentai.aruco` namespace is removed.  All consumers
(SafetyTask, calib autotune, missions, FlowBaseline gate,
on-board driver scripts, host bench harnesses) must move to
`sentai.markers`.  No compatibility alias is kept — operator
explicitly chose this over a deprecation-shim path.

Migration commits are scoped per consumer to keep blast radius
visible.  CI gate: every consumer touched must compile + pass
its smoke test before the next consumer is touched.

### 2.2 API surface (proposed)

```python
# Init at boot — backend selection.
sentai.markers.init(backend = "aruco" | "whycon")

# Common camera params + physics.
sentai.markers.set_intrinsics(fx, fy, cx, cy)
sentai.markers.set_marker_size(meters)

# Detection.  Returns count; results in a fixed-size struct array,
# read back via get_latest().
n = sentai.markers.detect_from_camera()

# Marker accessors — by-index, struct view, no dict.
n_active = sentai.markers.get_count()
sentai.markers.get_pose(i, out_buf)
# out_buf is a pre-allocated bytearray (sizeof MarkerPose) the
# MicroPython side passes in; binding fills it with the POD bytes.

# Stats.
sentai.markers.get_stats(out_buf)   # fills SentaiMarkersStats struct

# Diagnostics (kept as common, lowercase, prefixed `_`):
sentai.markers._detect_cyc()
sentai.markers._use_rolling(0|1)
sentai.markers._test_pgm(path)
sentai.markers._verify_threshold(block)
sentai.markers._thresh_cycles()
sentai.markers._thresh_rolling_verify(block)
sentai.markers._thresh_rolling_cyc()
sentai.markers._thresh_nocache(block)
sentai.markers._backend()           # returns "aruco" or "whycon"

# Backend-specific extensions:
sentai.markers.aruco.rvec_to_R(rvec_buf, R_buf)
sentai.markers.aruco.R_to_rvec(R_buf, rvec_buf)
sentai.markers.aruco._test_synth_and_detect(marker_id, side_px)

sentai.markers.whycon._test_synth(n_circles, radius)
sentai.markers.whycon._set_concentric(on)
sentai.markers.whycon._stage_cyc(out_buf)   # struct(t_a, t_b, t_w)
sentai.markers.whycon._get_marker_details(i, out_buf)  # struct WhyConMarker
```

### 2.3 Struct definitions (C, exposed as POD blobs to MP)

```c
/* Pose result — common across backends.
 * Fields populated by ArUco today; WhyCon will populate them
 * once its PnP stage lands (currently leaves pose fields as
 * NaN/-1 sentinels). */
typedef struct {
    int32_t  id;              /* marker ID; -1 = not assigned (WhyCon-simple) */
    float    pixel_cx;        /* always populated */
    float    pixel_cy;        /* always populated */
    float    tvec_cam[3];     /* NaN if backend doesn't PnP */
    float    rvec_cam[3];     /* NaN if backend doesn't PnP */
    float    reproj_err_px;   /* NaN if backend doesn't PnP */
    uint8_t  backend;         /* SENTAI_MARKERS_BACKEND_* enum */
    uint8_t  pose_valid;      /* 1 if tvec/rvec are populated */
    uint16_t _pad;
} SentaiMarkersPose;
static_assert(sizeof(SentaiMarkersPose) == 48, "SentaiMarkersPose ABI lock");

typedef struct {
    uint32_t frames_total;
    uint32_t frames_with_detect;
    uint32_t markers_total;
    uint32_t last_detect_us;
    uint8_t  backend;
    uint8_t  _pad[3];
} SentaiMarkersStats;
static_assert(sizeof(SentaiMarkersStats) == 20, "SentaiMarkersStats ABI lock");

typedef struct {
    uint32_t threshold_cyc;     /* Phase A */
    uint32_t flood_fill_cyc;    /* Phase B */
    uint32_t filter_axes_cyc;   /* Phase W (WhyCon) / decode+PnP (ArUco) */
} SentaiMarkersStageCyc;
static_assert(sizeof(SentaiMarkersStageCyc) == 12, "SentaiMarkersStageCyc ABI lock");
```

**No dictionaries cross the MicroPython heap.**  Output is via
caller-provided `bytearray(sizeof(...))` buffers, mirroring the
existing patterns in `sentai.flow.gray_snap()` and friends.

### 2.4 Backend enum

```c
typedef enum {
    SENTAI_MARKERS_BACKEND_NONE   = 0,
    SENTAI_MARKERS_BACKEND_ARUCO  = 1,
    SENTAI_MARKERS_BACKEND_WHYCON = 2,
} sentai_markers_backend_t;
```

MicroPython string→enum conversion at `init()` time.

## 3. Internal refactor — `sentai.calib`

### 3.1 Decision: namespace unchanged

`sentai.calib` keeps its name.  Reason: the calibration math
(Kabsch 3D Procrustes, Åström-Hägglund autotune, hold-test
metrics) is fundamentally marker-agnostic.  It consumes pose
data; it doesn't care which backend produced it.

### 3.2 Internal call-site rewrite

The C-side calls to `sentai_aruco_get_latest()` inside
`sentai_calib*.cc` (autotune, hold-test) are rewritten to call
a new `sentai_markers_get_latest()` that dispatches on the
active backend.  Same call signature, same struct output;
only the underlying source changes.

Touched files (audit pending):
- `examples/sentai_runtime/sentai_calib_autotune.cc`
- `examples/sentai_runtime/sentai_calib_task.cc`
- `examples/sentai_runtime/sentai_calib.h` (extern forward
  declarations)
- `examples/sentai_runtime/sentai_safety_task.cc` (currently
  calls `sentai_aruco_detect` directly — must switch to
  `sentai_markers_detect`)

### 3.3 `set_marker_size()` — moves to `markers`

Currently lives in `sentai.aruco.set_marker_size()`.  Moves to
`sentai.markers.set_marker_size()` (the physical marker
dimension is marker-agnostic by definition).

### 3.4 `set_context()` — **TBD, decide next session**

Marker-constellation layout (4-grid for ArUco, 3-triangle
asymmetric for WhyCon) is mission-specific, not calib-internal.
Operator 2026-05-20: "inca nu sunt hotarat cu calib.set_context,
sa analizam dupa ce incepem noul plan."  Hold the question open
until T1 is partially landed and we can see what the call sites
actually need.

## 4. WhyCon prerequisite — PnP stage

WhyCon's current detect output is `(cx, cy, axis_a, axis_b,
angle)` only — no z, no full pose.  To plug into the calib /
autotune pipelines that expect `tvec_cam[3]` + `rvec_cam[3]`,
WhyCon needs the PnP stage completed.

Two sub-tasks blocking integration:

- **W19-T2 — single-marker z from axis_a + physical diameter.**
  Closed-form: `z = fx * physical_diameter / (2 * axis_a)`.
  No iterative PnP needed; the circle's apparent radius gives
  z directly.  Tilt (roll/pitch) from axis ratio (a/b) — small
  perturbation; for landing-pad scenarios near-frontal, the
  approximation is good to ~5° tilt.

- **W19-T3 — multi-marker constellation pose (3-asymmetric
  triangle).**  3 known world positions + 3 detected pixel
  centers → SolvePnP-equivalent (or simpler: known geometry
  triangulation).  Resolves yaw + the per-marker-ID problem.

Without these, the W19 hard rename can still ship, but the
WhyCon backend will return `pose_valid = 0` on every pose
struct, and calib autotune will refuse to run with backend
selected as WhyCon.

## 5. SIM A/B campaign — quality not speed

After the refactor lands (and W19-T2/T3 close the WhyCon PnP
gap), move ablations to the simulator:

- **A/B detection rate** at varying altitudes (0.5 m → 3.0 m),
  marker visibility angles, lighting (gz scene env), motion
  blur (cf2 maneuver speed).
- **Closure / ID disambiguation** on the s127 FlowBaseline
  gate scene (calibrated indoor rig).
- **Flow diamond vs exhaustive** A/B on the same trajectories
  — closure delta + cumulative drift over 30-s walks.
- **PnP-z accuracy** per backend — closed-loop hover variance.

Verdict criteria:

- WhyCon promoted to default if `closure_err ≤ ε_aruco` on the
  s127 gate AND detection rate ≥ ArUco at all altitudes.
- Flow diamond promoted to default if `closure_err ≤ 1.5×
  ε_exhaustive` (slight degradation acceptable for the 16.5×
  speedup) AND no new "stuck" failure mode appears in
  `_host_flow_validate.py` over the s127 trajectory set.

SIM-side rerun is the post-refactor regression gate before
any real-flight testing.

## 6. Execution plan (next session)

1. **T1**: write `sentai_markers.h` with the backend enum +
   struct definitions + ABI static_asserts.
2. **T1.1**: implement `sentai_markers_detect()` C dispatcher
   (selects aruco vs whycon path based on backend enum).
3. **T1.2**: implement `modsentai_markers.c` MP binding (struct
   buffer pattern, no dicts).
4. **T1.3**: refactor calib + safety_task internal call sites.
5. **T1.4**: rename `sentai.aruco` MP module out of existence
   (operator chose hard rename).
6. **T1.5**: QSTR regen + build + flash + smoke test.
7. **T2**: WhyCon PnP-z from axis (W19-T2).  Updates struct
   `pose_valid = 1` once tvec is populated.
8. **T3**: SIM A/B campaign — kicks off after T1+T2 land.

Estimated effort: T1 ~half-day, T2 ~2 hours, T3 ~1-2 days SIM
running + analysis.

## 7. Hard rules carried into this WP

- `[[ipc-clamp-in-worker-not-host]]` — discovered yesterday;
  applies to any new IPC sentinels added during refactor.
- `[[no-heavy-data-through-mp]]` — pose structs cross MP via
  caller-allocated bytearrays, NOT new dicts.
- `[[no-flow-deck-camera-imu-only]]` — markers backend MUST
  remain camera-only; no IMU dead-reckoning in the markers
  layer.
- NASA/JPL section 4.1: bounded loops, no dynamic allocation,
  static-asserted struct sizes.

## 8. Open questions for next session

1. Do we move `sentai_aruco.cc` content into a new
   `sentai_markers.cc`, or keep the file name and add a
   `sentai_markers_dispatch.cc` shim?  (Vote: rename source file
   too — `git mv sentai_aruco.cc sentai_markers.cc`; preserves
   git blame history.)
2. WhyCon constellation pose (W19-T3) — port the algorithm
   from a reference implementation (chronorobotics/whycon
   GitHub) or implement from the Krajník JINT 2014 paper?
3. Do we keep `_thresh_nocache` and friends, or move them to
   `sentai.diag` (since they're cross-cutting HW
   instrumentation, not marker-specific)?  Tentative vote:
   move to `sentai.diag.threshold.*`.

## 9. Cross-refs

- `[[op-s10-w17-whycon-2026-05-19]]` — WhyCon-lite WP.
- `[[op-s10-w18-flow-search-2026-05-20]]` — Flow diamond WP.
- `[[ipc-clamp-in-worker-not-host]]` — discovered mid-W18,
  applies to any new sentinel-dispatch code in this WP.
- W17-T2 + T2.1 + T4 measurement numbers were INVALID
  (clamp bug) — see W18 §6 for the correction pass.  Real
  M4 numbers landed in commit `ca7cbbc5`.
