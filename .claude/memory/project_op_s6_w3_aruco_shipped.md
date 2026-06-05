---
name: op-s6-w3-aruco-shipped
description: "OP-S6-W3 sentai.aruco SHIPPED 2026-05-17. Fresh on-board ArUco detector + PnP in C/C++ (the old sentai_aruco_shim from feature/ov5640-camera-support@a5f5a568 was NOT cherry-picked — integration branch reverted, design ideas reused but code written fresh per [[no-broken-branch-test-reuse]]). Pipeline: adaptive threshold (Bradley-Roth 2007 + integral image) → 4-conn flood-fill labeling → quad x±y extreme corner extraction → 4x4 bit decode bilinear+3x3 majority (Garrido-Jurado 2014) → DLT homography → R|t (Hartley-Zisserman 2003). EXP-s159 12/12 SIM PASS. ARM 4.9 KB .sdram_text. FlowBaseline 9.8 cm PASS. Camera frame zerocopy hook in modsentai_sim_camera.c."
metadata:
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

WBS: `OP-S6-W3`, Stage 6 sub-WP 3 (on-board ArUco detector + PnP).
Unblocks `OP-S6-W1-T7` / `EXP-s158` (sentai.calib takeoff phase fed
by real PnP samples).

## Why fresh re-implementation

The branch `feature/ov5640-camera-support` (commit `a5f5a568`) had
`sentai_aruco_shim.h` / `_arm.cc` / `_sim.c` + `sentai_anchor_forward.cc`
+ `sim/scripts/aruco_pose_publisher.py`.  Operator reverted to
`integration/from-180bbb5f` because that branch hit a dead-end; the
ArUco infrastructure went with it.

Per [[no-broken-branch-test-reuse]] + [[objectsplan]] strategy:
design ideas + API shape from `a5f5a568` are OK to reference, but
implementation must be FRESH (the old code may carry the dead-end's
brittleness).  This module follows that rule:
- API differs from the old shim (camera-frame pose multi-marker out
  vs world-frame anchor pose; `sentai.aruco.*` namespace vs subset
  of `sentai.flow.mode("anchor")`).
- Pipeline written from scratch with SOTA references documented in
  the header + experiment README.

Operator suggestion 2026-05-17 — rename "anchor" → "aruco" — adopted:
the new module is `sentai.aruco`, not `sentai.flow.mode("anchor")`.
Legacy "anchor" terminology stays in `flow_phase_corr.cc` for the
phase-correlation reference frame (semantically correct there).

## SOTA references (no wheel reinvention)

| Stage | Reference |
|---|---|
| Adaptive box-mean threshold | Bradley & Roth 2007 |
| Integral image | Viola & Jones 2001 |
| 4-conn flood-fill labeling | Rosenfeld & Pfaltz 1966 |
| Bit sample bilinear + 3x3 majority | Garrido-Jurado et al. 2014 (ArUco) |
| 4-rotation hamming dict | Garrido-Jurado 2014 |
| DLT homography → R\|t | Hartley & Zisserman 2003 §8.1 |
| Rodrigues rvec ↔ R | classical Lie-group exp/log |

## SOTA gaps (deferred upgrades)

- Quad-corner via x±y extremes is a HEURISTIC.  SOTA = Douglas-Peucker
  polygon simplification.  Brittle under oblique views / partial occ.
- PnP is DLT, NOT IPPE (Collins-Bartoli IJCV 2014 — current SOTA for
  planar markers).  DLT works OK at moderate viewing angles but loses
  precision near marker-parallel-to-image.
- Dictionary is an 8-entry placeholder (matches synthetic test
  patterns).  Real OpenCV 4x4_50 = 50 entries.  PNG-to-dict-bits
  helper script is the next deliverable for Gazebo validation.

## Deliverables

| File | Purpose |
|---|---|
| `examples/sentai_runtime/sentai_aruco.h` | API contract + fault model |
| `examples/sentai_runtime/sentai_aruco.cc` | Detector pipeline + PnP + test-synth helper |
| `examples/sentai_runtime/bindings/modsentai_aruco.c` | MP binding (returns scalars only — no image buffers through MP) |
| `sim/modsentai_sim_camera.c` | New `sentai_camera_grab_gray_zerocopy()` C hook used by aruco detector at SIM-side |
| `examples/sentai_runtime/experiments/s159_aruco_smoke/` | EXP-s159 (T7) — 12-gate synthetic smoke |
| linker `.sentai_slow` entry | Routes `sentai_aruco.cc.obj` `.text/.data/.rodata` to SDRAM |

## Validation evidence

- **EXP-s159 SIM smoke**: 12/12 gates PASS.
  - T1 identity recovery, id=0, 96px @ tvec_z=0.253 m, reproj 6.5e-6 px.
  - T2 90/180/270° CW rotations: same id, same tvec.z (rvec varies
    correctly — physical marker rotation IS observable in the pose;
    this is per ArUco spec).
  - T3 ids 1, 2, 3 from placeholder dictionary.
  - T4 size scaling: tvec.z = marker_size * fx / pixel_side (48 px →
    0.51 m, 120 px → 0.20 m).
  - T5 stats counters.
  - T6 cache clear semantics.
  - T7 rvec↔R round-trip err < 3e-8.
- **ARM build**: clean.  `sentai_aruco.cc.obj` text=4956 B
  (.sdram_text per [[itcm-budget]]).
- **.sdram_bss buffers**: ~2.17 MB total (`s_binary`, `s_labels`,
  `s_integral` 1.2 MB int32 integral image, `s_fill_stack` 16 KB,
  `s_components`, plus the test-synth `s_test_gray` 307 KB).
  Total SDRAM usage 15.3 MB / 16 MB available — TIGHT but within
  budget.  Future optimization: gate `s_test_gray` behind a debug
  flag for production builds; consider int16 integral image with
  capped image dims.
- **FlowBaseline post-commit**: `EXP-s127` PASS dist_mean=9.8 cm
  (canonical 7.4 cm — within budget gate of 10 cm).  No regression.

## What's NOT yet validated

- Live Gazebo ArUco frames — needs the real OpenCV 4x4_50 dictionary
  loaded (PNG-to-bits helper TODO).
- Robustness to noise / motion blur / partial occlusion / oblique
  viewing — synthetic noise-free only.
- ARM-side timing budget — Phase 1 builds clean but no on-device
  cycle measurement yet (post-Stage 9 ARM bring-up).
- Numerical stability near marker-parallel-to-image (DLT loses
  precision; IPPE upgrade is the fix).

## Unblocks

- `OP-S6-W1-T7` / `EXP-s158`: mission_s153 takeoff calibration phase
  can now collect real `tvec_cam` samples via
  `sentai.aruco.detect_from_camera()` and feed them into
  `sentai.calib.run_kabsch()`.

## Cross-references

- Algorithm + fault model: `examples/sentai_runtime/sentai_aruco.h` +
  `.cc` (per-stage SOTA citations in comments).
- WBS: `ideas/wbs.md` (OP-S6-W3 added as sub-WP of Stage 6).
- Related: [[op-s6-w1-calib-shipped]], [[no-broken-branch-test-reuse]],
  [[objectsplan]], [[wbs-pmp-2026-05-17]], [[itcm-budget]].
