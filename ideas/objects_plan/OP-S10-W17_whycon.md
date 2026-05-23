# OP-S10-W17 — WhyCon-lite circular fiducial marker detection

**Status**: T1 + T2 SHIPPED (2026-05-19).  M7 detector + full
optimization stack done; M4 Phase-A ablation done.  Concentric
W3 + PnP + WhyCode + multi-marker pose are open ToDo.

## 1. Why this WP exists

ArUco production after the OP-S10-W14-T18 cv2-parity work measures
**31.8 ms / 4-marker frame** on M7 (rolling variant 24.8 ms).  At
30 Hz SafetyTask the ArUco-production path eats 96 % of the slot,
leaving ~1 ms headroom for everything else.  Per the SOTA review
(Kalaitzakis 2021), the WhyCon family (Krajník/Nitsche 2013) gives
~2 orders of magnitude faster detection than ArUco/AprilTag at the
cost of weaker occlusion tolerance and no per-marker ID without
the WhyCode extension.

Goal: bring up a WhyCon-lite detector on M7, optimize with the
same HW primitives we used for ArUco (OCRAM placement, SIMD
divide-elim, inline moments) and reach a per-frame budget that
gives SafetyTask real headroom.  Then ablation on M4 with the
same Phase-A kernel to keep the M7-vs-M4 chapter consistent.

## 2. Algorithm

WhyCon-lite reuses the first two ArUco pipeline stages verbatim
and adds three circle-specific stages:

| Stage | Source | What it does |
|---|---|---|
| A. Bradley adaptive threshold | reused from ArUco (rolling variant) | binary image |
| B. 8-conn flood-fill labeling | reused from ArUco | connected components + bbox + 0th/1st moments |
| W1. Coarse circular filter | NEW | reject by area + bbox aspect + fill ratio (cheap) |
| W2. 2nd-order moments + axes | NEW | sub-pixel centroid + semi-axes a, b via 2×2 covariance eigvals |
| W3. Concentric validation | TOGGLE STUB | inner-white-disc check — *not implemented yet* |

Math identity:

```
m00       = n_pix            (already in aruco_comp_t)
m10/m01   = cx_sum / cy_sum  (already in aruco_comp_t)
m20/m02/m11 = Σ x², Σ y², Σ xy  (added to aruco_comp_t at W17-T2)
μ20  = m20/m00 - cx²    (central)
μ02  = m02/m00 - cy²
μ11  = m11/m00 - cx·cy
λ1,2 = trace/2 ± √(trace²/4 − det)  (eigenvalues)
a    = 2·√λ_max,  b = 2·√λ_min      (semi-axes)
θ    = ½·atan2(2·μ11, μ20 − μ02)    (major-axis angle)
```

For a perfect disc: a ≈ b, so eccentricity ≈ 0; filter rejects
anything with a/b > 2.0.

## 3. M7 optimization stack

All measured on build #1397, 320×240 synth-disks frame (anti-DCE
via `n_det` listing per iter + `_get_markers()` consumption of
the s_whycon_markers state at the end).  Median over 6 iter.

| Stack | N=1 | N=4 | N=8 | vs baseline |
|---|---:|---:|---:|---:|
| Baseline (init impl, bb0a40d1) | 7.35 | 8.24 | 9.42 | — |
| + OCRAM placement | 7.35 | 7.54 | 8.72 | −8 % |
| + SIMD divide-elim rolling Phase 2 | 5.24 | 6.12 | 7.31 | −26 % |
| + Inline 2nd-order moments in flood-fill | **5.19** | **6.00** | **7.05** | **−27 %** |

### 3.1 OCRAM placement (commit c694cee5)

`ARUCO_MAX_W/H` reduced from 640×480 → 320×240 (PrepTask
SLOT_GRAY_NATIVE produces 320×240, never 640×480; verified no
external user of the bigger constants).  This freed 1.4 MB of
unused SDRAM and let `s_test_gray` (75 KB) + `s_fill_stack`
(16 KB) move into `.ocram_bss` (91 KB total; fits the 103 KB
free in OCRAM after `.tpu_input`).

Saving: −0.7 ms consistent across N.  Modest because D-cache
already absorbed the sequential reads of `s_test_gray`; main
win is in the flood-fill DFS pop/push pattern.

### 3.2 SIMD divide-elim rolling Phase 2 (commit c694cee5)

Same USUB8/SEL + divide-free-compare trick as the production
`aruco_adaptive_threshold`, ported into the rolling-integral
path that operates on `s_rolling_prefix_x` instead of a 2-D
integral image.

- Interior columns (x in [half, W−half−1]): 4-wide SIMD
  compare via `aruco_usub8` / `aruco_sel`.  Math identity:
  `(gray + C + 1) · box_area_int ≤ box_sum`, where
  `box_area_int = block · box_h` is constant per row.
- Borders: scalar with x clamping.

Math-identical to scalar reference: 0 mismatches across blocks
{7, 23, 51, 101} via `_thresh_rolling_verify`.

Saving: −1.4 ms consistent across N.  Operator was skeptical
(remembered the gcc autovec winning on simpler loops), but the
compiler was demonstrably NOT autovectorizing Phase 2 because
the divide + boundary clamp + prefix_x indirection blocked
autovec.

### 3.3 Inline moments (commit 8eccf31b)

`aruco_comp_t` extended with `m20_sum / m02_sum / m11_sum`
(int64).  Accumulated during the flood-fill DFS pop instead
of a Phase W2 bbox rescan.  Each pixel of each blob was
already visited exactly once in the flood-fill — adding the
three MAC operations is amortized for free.

Saving: −0.05 ms at N=1 to −0.26 ms at N=8 (scales with marker
count; the eliminated rescan cost was proportional to total
blob pixel count).

## 4. Comparison vs ArUco production (4 markers)

> **Scope of "WhyCon-lite" measurement.**  Every row below is the
> full end-to-end pipeline executed on the synth 4-marker frame:
> Phase A (rolling adaptive threshold) + Phase B (8-connected flood
> fill with inline 2nd-order moments) + Phase W1 (size/circularity
> filter) + Phase W2 (covariance eigenvalues → axis_a / axis_b /
> angle).  W3 concentric inner-disc validation is a no-op stub
> (see §6.1); PnP / WhyCode / multi-marker pose are NOT included
> (§6.2-6.4) — those would add ~1 ms (W3) plus PnP cost on top of
> the numbers below.  "WhyCon-lite" in this WP and "WhyCon FULL
> pipeline" in the M4 bench code (`m4_aruco_bench.cc`, sentinel
> `0xC200..0xC208`) refer to the SAME code path; naming was kept
> stable here, the M4 sentinel name distinguishes it only from the
> M4 Phase-A-only sibling (`0xC100..0xC108`).

| Algorithm | Latency | vs ArUco prod |
|---|---:|:---:|
| ArUco production (SIMD integral SDRAM) | 31.8 ms | 1.0× |
| ArUco rolling (OCRAM scratch) | 24.8 ms | 1.28× |
| WhyCon-lite baseline | 8.24 ms | 3.86× |
| WhyCon-lite + OCRAM | 7.54 ms | 4.22× |
| WhyCon-lite + OCRAM + SIMD | 6.12 ms | 5.19× |
| **WhyCon-lite + OCRAM + SIMD + inline moments** | **6.00 ms** | **5.30×** |

### 4.1 Apples-to-apples per-stage breakdown (s180, build #1412)

The table above compares the two detectors as a single end-to-end
number, but the two pipelines emit different OUTPUTS — ArUco
production returns per-marker `tvec_cam` / `rvec_cam` (full 6-DOF
pose via IPPE PnP) plus a decoded `marker_id`, while WhyCon-lite
returns only `(cx, cy, axis_a, axis_b, angle)` — 2D ellipse with
no 3D pose and no ID.  T5 (Phase W3 concentric inner-disc) +
W19-T2 (closed-form PnP-z) brought WhyCon up to pose-emitting
parity; T6 added per-stage cycle counters on both detectors;
T8 fixed W3 sample geometry (anchor on `bbox_R`, not eigenvalue
`axis_a`) and the FxUser PGM-staging buffer.  See
`examples/sentai_runtime/experiments/s180_aruco_whycon_apples_to_apples/`
for the bench harness + raw `results.json`.

|                                 |    ArUco prod ¹ | WhyCon production ² |
|---------------------------------|---------------:|--------------------:|
| Bradley adaptive threshold      | **12.03 ms** (block=201) | **3.32 ms** (block=31) |
| 8-conn flood-fill (with inline moments) | 14.02 ms ³ | 2.65 ms |
| geometric gate / W1+W2 (eigenvalue axes) | 0.74 ms | 0.01 ms |
| decode (ID-emit; warp + Otsu + dict) |   1.08 ms | n/a |
| W3 concentric (inner-disc pattern) |  n/a       | 0.03 ms |
| PnP (IPPE for ArUco, closed-form for WhyCon) | 0.05 ms ⁴ | 0.006 ms |
| **TOTAL**                       | **27.91 ms**   | **6.02 ms**         |

¹ ArUco bench on `experiments/s175_pnp_planar_ambiguity/frame_original.pgm`
(Gazebo SIM render, 4 ArUco markers, 320×240, clean) — the same
frame s175 + s176 used to validate IPPE_SQUARE PnP.  4/4 markers
detected and PnP succeeded each iteration.

² WhyCon bench on internal Krajnik-pattern synth (320×240, 4
annular markers).  Post-T8, all 4 survive Phase W3 in production
mode; PnP populates `tvec_cam` + `rvec_cam` in `s_whycon_markers`.

³ ArUco flood-fill is dominated by real-frame texture — the
Gazebo scene has many small high-contrast blobs (ground tiles,
shadows) that all become candidate components.  WhyCon synth
has 4 clean markers + minimal background = much less flood-
fill work.  Future iter (s180-iter3) will render a Krajnik-
markered Gazebo scene to close this asymmetry.

⁴ ArUco IPPE_SQUARE PnP = 12 µs / marker × 4 = 47 µs total.
Closed-form WhyCon PnP-z = 1.5 µs / marker × 4 = 6 µs total.
Both are <0.3 % of their respective budgets.

### 4.2 Headline interpretation

| Comparison | ArUco | WhyCon | Ratio |
|---|--:|--:|--:|
| Full pipeline (pose-emitting, with ID)  | 27.91 ms | 6.02 ms | **4.6×** |
| Without ID stage (decode stripped)      | 26.83 ms | 6.02 ms | **4.5×** |
| Threshold-only (block-size-driven)      | 12.03 ms | 3.32 ms | 3.6× |
| 30 Hz slot utilisation                  | 84 %     | 18 %    | — |

Five take-aways:

1. **The lite-vs-production headline (5.3×) and the strict
   apples-to-apples comparison (4.6×) are consistent.**  Adding
   W3 + PnP to WhyCon costs <40 µs total (sub-percent); the
   small ratio drop from 5.3× to 4.6× comes from comparing
   WhyCon production vs ArUco PRODUCTION (with real-frame
   decode + PnP), not WhyCon-lite vs ArUco-prod-on-synth-no-
   detection (which had decode + PnP at zero cost because
   nothing was detected).

2. **PnP cost is a non-event for both detectors.**  ArUco
   IPPE_SQUARE on this frame = 12 µs / marker; WhyCon closed-
   form = 1.5 µs / marker.  PnP choice does not drive the
   comparison.

3. **WhyCon W3 is essentially free.**  6.5 µs / candidate.
   Pattern validation does NOT change the budget — it's the
   right default to leave on.

4. **The two stages that DO drive the gap**:
   - Bradley threshold block size (ArUco 201 vs WhyCon 31)
     → 8.7 ms difference, intrinsic to marker physical size.
     ArUco markers project as larger pixels and need a larger
     Bradley block to cover them.
   - Flood-fill on cluttered real frames (ArUco 14 ms) vs
     clean synth (WhyCon 2.65 ms) → 11.4 ms difference,
     partly closes when both are benched on equivalent
     scenes; needs a Krajnik-markered Gazebo scene to
     measure (deferred to s180-iter3).

5. **Operational headroom** — at 30 Hz, WhyCon production leaves
   the M7 ~82 % idle slot for Flow, FR, mission FSM, and the
   pose-feedback loop.  ArUco at 84 % slot utilisation is
   marginal even for SafetyTask alone.

At 30 Hz SafetyTask (33 ms slot):

| Detector | Slot used | Headroom |
|---|---:|---:|
| ArUco production | 96 % | 1 ms |
| ArUco rolling | 75 % | 8 ms |
| **WhyCon-lite full opt** | **18 %** | **27 ms** |

Same number at 60 Hz (16.7 ms slot): ArUco prod doesn't fit;
ArUco rolling at 149 %; WhyCon-lite at 36 % — a real
possibility if the operator wants to push SafetyTask faster.

## 5. M4 ablation

Operator asked for an M4 comparison sibling to the W16 ArUco
ablation.  Ported only Phase A (rolling threshold scalar) +
synth-disk generator into `m4_aruco_bench.cc` behind sentinel
range `0xC100..0xC108` (N = block & 0xF disks).  Build #1398,
commit e1b1f598, anti-DCE via XOR-fold into volatile sink.

| N | M4 cycles | M4 @ 400 MHz |
|:-:|---:|---:|
| 1 | 10.36 M | 25.90 ms |
| 3 | 10.34 M | 25.86 ms |
| 4 | 10.34 M | 25.85 ms |
| 6 | 10.36 M | 25.89 ms |
| 8 | 10.33 M | 25.83 ms |

Constant in N (O(W×H) threshold dominates; no per-blob cost
since this M4 path is Phase A only).

M7 Phase-A subset of the full pipeline (estimated as
6.00 − 1.5 ms ≈ 4.5 ms; the Phase-B flood-fill + W2 moments
fold in for ~1.5 ms total on the 4-marker frame):

| Core | Phase-A latency | Speedup |
|---|---:|:---:|
| M4 OCRAM scalar | 25.86 ms | 1.0× |
| M7 OCRAM + SIMD rolling | ~3-4 ms | **~7×** |

Same architectural verdict as OP-S10-W16 ArUco ablation: M7's
800 MHz + dual-issue + D-cache + DSP-SIMD outclass M4F's
400 MHz + uncontested OCRAM + single-issue + no-cache on bulk
pixel workloads.  M4 stays an idle "low-latency interrupt"
core; it is **not** an offload option for the SafetyTask
detection pipeline.

## 6. What WhyCon-lite IS NOT (production gaps)

1. **No concentric inner-disc check (Phase W3)**.  Toggle stub
   exists (`sentai.whycon._set_concentric(1)`) but the actual
   inner-white-disc validation is not implemented.  This is the
   defining difference between "WhyCon-lite" (here) and full
   WhyCon (Krajník original).  Adds ~1 ms est. when wired —
   pre-filter components for outer-disc geometry, then scan a
   smaller inner ROI for the white disc.
2. **No PnP / 3D pose**.  Outputs `(cx, cy, axis_a, axis_b,
   angle)` in pixel coordinates only.  z from `axis_a` and the
   known physical marker diameter is a one-liner; full 6-DoF
   needs multi-marker constellation.
3. **No WhyCode bit decode**.  No per-marker IDs; markers are
   geometrically disambiguated by inter-marker distances in
   the constellation.
4. **No robust ellipse fit** (Fitzgibbon LSQ).  Moments-only
   axes degrade under perspective foreshortening for non-
   frontal viewing.  Sufficient for landing-pad scenario where
   camera is roughly above the pad.
5. **No multi-marker constellation pose**.  Needs a layout
   manager (which marker is at which world coordinate) and a
   PnP solver on the 3+ detected centroids.  FSM not built.
6. **Not wired into SafetyTask**.  Pure perf-instrumentation
   module; ArUco is still the only detector consumed by
   `sentai.safety` + `flow_baseline` + `vpe_forwarder`.

## 7. API surface

MicroPython (`sentai.whycon.*`):

- `._test_synth(n, radius=15) -> int n_detected`
  — generate N filled black disks at deterministic positions
  on the synth grid; run pipeline; return detection count.
- `._test_pgm(path)           -> int n_detected (or <0 err)`
  — load 320×240 P5 PGM (FxUser FAT or LittleFS sys path
  both work, same fallback chain as `sentai.aruco._test_pgm`);
  run pipeline.
- `._detect_cyc()             -> uint cycles last call`
- `._set_concentric(on)       -> previous flag` (W3 stub)
- `._get_markers()            -> list[dict{cx, cy, a, b, ang}]`

C-side (`sentai_aruco.cc`):

- `sentai_whycon_test_synth(n, radius)`
- `sentai_whycon_test_pgm(path)`
- `sentai_whycon_detect_cyc_last(void)`
- `sentai_whycon_set_concentric_check(int on)`
- `sentai_whycon_get_markers(out, cap)`

## 8. Open ToDo (next sessions)

1. **W3 concentric inner-disc validation**.  Cheapest correct
   step toward full WhyCon: scan a smaller ROI inside each
   filtered black blob's bbox, count white pixels, gate on
   white-area / outer-area ratio ≈ 0.36 (Krajník default).
   Adds ~1 ms est., yields false-positive rejection.
2. **PnP from `axis_a` + physical marker diameter**.  Single
   marker → z and tilt; needs `sentai.aruco.set_intrinsics`
   to be reused (or its own `sentai.whycon.set_intrinsics`).
3. **Multi-marker constellation pose** (3-asymmetric-triangle).
   Resolves the per-marker-ID problem by inter-marker
   distance matrix + 6-DoF SolvePnP-equivalent.
4. **WhyCode bit decode**.  Adds ~1-2 ms; only needed if the
   constellation approach proves insufficient (eg. occlusion).
5. **Upload `whycon_real.pgm`** (proper inner-disc pattern,
   generated locally) + bench `_test_pgm` on it to confirm the
   full pipeline detects rings (not just the lite blobs).
6. **Wire into SafetyTask** behind `sentai.safety` mode flag,
   so the same SafetyTask machinery can consume either
   detector.  Threshold + flood-fill are already shared;
   shouldn't be more than ~50 lines of glue.
7. **Robust ellipse fit (Fitzgibbon LSQ)** if perspective
   foreshortening breaks moments-axes at low altitudes.
8. **M4 full WhyCon port** (Phase B + W1 + W2) to complete the
   sibling-of-OP-S10-W16 chart for the thesis.

## 10. T10 — Synthetic perception bench + OpenCV-parity port (2026-05-23)

### Motivation

Through W17-T5 (W3 concentric), W17-T8 (W3 + W1 fix), and the W21-T13
"cv2-tuned threshold" iter-21 grind on real flights (s187/s190/s191),
the WhyCon detector was tested only **inside closed-loop flight**
where detector failure was indistinguishable from EKF / PnP /
RPYT→HL handoff / control coupling.  T13 iter-21d (commit
`6381b7e7`) localized the issue to a contour/RETR_EXTERNAL gap but
could not isolate it because the test rig was a 3D flight.

T10 builds an **air-gapped perception bench** that consumes only
camera frames + manifest metadata, so detector behavior is measurable
independently of flight dynamics.  It also resolves the long-running
semantic mismatch between the Python OpenCV reference baseline and
the embedded C++ implementation by porting both onto the same
documented WhyCon family (Krajník 2013/2014, Nitsche 2015,
`lrse/whycon`).

### Deliverables

| Artefact | Purpose |
|---|---|
| `todo/TD-S10-A1_create_synthetic_dataset_whycon.md` | Stable spec: dataset format, manifest schema, visibility classes, sampling policy |
| `todo/TD-S10-A2_validate_synthetic_whycon.md` | Stable spec: validation methodology, OpenCV ↔ sentai_sim ablation, 16+ report tables, pose-ambiguity diagnostic |
| `todo/TD-S10-B1_*.md` | Implementation log of B1 (dataset generator) with dead-ends kept |
| `todo/TD-S10-B2_*.md` | Implementation log of B2 (validator + sentai_sim port) with iter-by-iter results |
| `sim/scripts/generate_whycon_synthetic_dataset.py` | Gazebo-only renderer; injects asymmetric 7-marker pad into a run-local world copy (upstream `_small.sdf` untouched); writes P5 PGM + manifest.jsonl + config.json |
| `sim/scripts/validate_whycon_synthetic_dataset.py` | Two OpenCV variants (`paper`, `edge_partial`); reproj-gated pose; IPPE + correspondence-permutation ambiguity diagnostic; 16 report tables |
| `sim/scripts/run_sentai_sim_whycon_dataset.py` | Drives `sentai.markers` through `sentai_sim` REPL + `sentai.fs`, no flight simulator |
| `sentai_aruco.cc` rewrite | OpenCV-parity WhyCon: threshold sweep (100/130/150) + 8-CC + concentric dark-dot pairing via bbox/moment, dedupe, W3 retired |
| `sentai_markers.{cc,h}` ABI | New `SentaiMarkersDetection` (cx, cy, axis_a, axis_b, angle_rad, radius_outer, tvec, rvec, reproj_err, pose_valid); `sentai_markers_detect_pgm`, `sentai_markers_get_detection`, `sentai_markers_set_marker_world` |
| `bindings/modsentai_markers.c` | MicroPython exposure: `detect_pgm`, `get_detection`, `get_detection_tuple` |
| `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/` | Canonical 368-frame dataset (only this run kept; smaller smoke runs in `.gitignore`) |
| `dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/` | Canonical ablation result (results.jsonl + summary.json + report_tables/; overlay PNGs in `.gitignore`) |

### Canonical 368-frame ablation result (build #558+)

Source: `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/`.

Frame distribution: `{4:73, 5:56, 6:56, 7:183}` evaluation-visible
markers, `z ∈ {0.30, 0.50, 0.75, 1.00 m}`, yaw 0–315° in 45° bins,
roll/pitch `{(0,0): 200, (4,-3): 168}`.

Detection (fully visible markers only — `partial_crop` excluded):

| backend | recall | FP/frame | complete-frame | centroid p95 |
|---|---|---|---|---|
| opencv `paper` | 1.0000 | 0.005 | 0.995 | 1.51 px |
| sentai_sim    | 0.9982 | 0.052 | 0.938 | 1.51 px |

Geometry parity OpenCV ↔ sentai_sim (matched markers):

| metric | value |
|---|---|
| centroid Δ p95 | 0.36 px |
| axis_a Δ p95 | 0.0004 px |
| axis_b Δ p95 | 0.16 px |
| angle (all) p95 | (high, near-circular ill-conditioned) |
| angle (stable, axis ratio ≥ 1.10) p95 | 2–4° |

Pose vs ground truth:

| backend | pose frames | translation RMSE | translation p95 | yaw MAE | yaw p95 |
|---|---|---|---|---|---|
| opencv     | 364 | 0.128 m\* | 0.041 m | 0.20° | 0.22° |
| sentai_sim | 367 | **0.016 m** | 0.029 m | 0.09° | 0.22° |

\* OpenCV RMSE inflated by ~6 mirrored-branch ~1 m pose outliers
on 4-marker weak-geometry frames at z=1.00 m, with reprojection
RMSE *below* the 0.5 px gate — so a reproj-only gate cannot reject
all of them.  sentai_sim is more robust on this batch because the
production drone-pose path uses prior-guided correspondence
(`get_drone_pose_tuple` does internal permutation + Kabsch +
yaw-anchor mirror picker per `[[feedback-yaw-anchor-mirror-picker]]`),
whereas the OpenCV validation path runs exhaustive correspondence
permutations and is more vulnerable to mirror branches.

### Algorithmic alignment

Both implementations now follow the **connected-component concentric
ring/dot** family:

1. Global inverse threshold sweep (100/130/150).
2. 8-connected component extraction.
3. Outer-component circular gate (area, radius, circularity).
4. Concentric dark-dot pairing via bbox/moment centre (NOT strict
   OpenCV contour hierarchy parent→child→grandchild — that strict
   variant under-detects at high z/edge where the dot is too small
   to survive as a fully-traceable contour).
5. Center-offset + dot/outer-radius gates.
6. Duplicate suppression across threshold passes.

The old W3 concentric check (W17-T5/T8) is retired from the active
result.  The new pairing rule supersedes it.

`edge_partial` is kept as a **labeled variant** for cropped-marker
recovery: it adds `cv2.minEnclosingCircle` over the visible arc.
+11 matched markers on the smoke 30 set, but +12 false positives.
Per A2 §"Non-goals" this is not folded into the parity baseline.

### Anti-cheat status

Clean per `[[feedback-sentai-sim-air-gapped-from-truth]]`:

- Dataset frames are camera-only Gazebo renders (`/dataset_cam/image`).
- Manifest GT consumed only by host-side validator; never piped into
  `sentai_sim`.
- `set_marker_world(...)` provides the *known pad layout* (config,
  same set exists on real HW from calibration) — not the *camera/drone
  pose ground truth*.  Distinction matches the rule.
- Upstream `sentai_whycon_small.sdf` is unmodified; the 7th asymmetric
  marker is injected only into the run-local world copy.

### Remaining work after T10

- **Pose ambiguity on 4-marker weak geometry** is the next real
  blocker, not detector recall.  The validator exposes it as a
  diagnostic; production `get_drone_pose_tuple` must hold the
  prior-guided assignment line.  Fold the assignment recipe from
  TD-S10-B2 §"Runtime Pose Assignment Options" into a regression
  test under this same bench.
- **19 sentai_sim false positives across 368 frames** + 4 missed
  markers at z=1.00 m — inspect via existing
  `sentai_detection_contact_sheet.png` artefacts.
- **ARM build verification** of the new ABI per `[[sim-arm-parity-check]]`.
  SIM build is at #558+ uncommitted; arm-builder should compile the
  same source to confirm no `m_text` / ITCM placement regression.
- **s191 / OP-S10-W21-T12 / T13 rerun** against the committed
  detector.  Open question: does T13 iter-21d's "33/44 match (75%)"
  go to ~98% with the new CC + pairing path?  If yes, T13 closes
  cleanly; the flight blocker becomes pose-side, not detection-side.
- **Promote `edge_partial` to sentai_sim** as a separately named
  variant if cropped-marker recall is needed for flight at z > 0.75 m
  (anchor-forward turns the camera into a heavily cropped FOV).

## 11. Cross-refs

- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — origin of
  the USUB8/SEL + divide-elim trick reused in §3.2.
- `[[op-s10-w16-T3.8-rolling-integral]]` — origin of the
  rolling threshold kernel (lives in `sentai_aruco.cc`).
- `[[op-s10-w16-ablation-findings-2026-05-19]]` — sibling
  M7-vs-M4 ablation on ArUco; same architectural verdict.
- `[[feedback-sentai-sim-air-gapped-from-truth]]` — anti-cheat
  rule that constrains the bench architecture.
- `[[feedback-yaw-anchor-mirror-picker]]` — production-side
  ambiguity tie-breaker reused by `get_drone_pose_tuple`.
- WhyCon SOTA papers:
  - Nitsche, Krajník, Faigl 2013 — original (JINT).
  - Lightbody, Krajník, Hanheide 2017 — WhyCode (SAC best paper).
  - Blaha, Mikula, Vintr, Krajník 2023 — 6-DoF refresh.
- Commits:
  - bb0a40d1 — WhyCon-lite prototype (T1).
  - c694cee5 — OCRAM placement + SIMD (T2).
  - 8eccf31b — inline 2nd-order moments (T2).
  - e1b1f598 — M4 Phase-A ablation (T2).
  - 9d01aa43 / 60e7a62f / 6381b7e7 — W21-T13 threshold gap
    iter-21 (precursor to T10).
