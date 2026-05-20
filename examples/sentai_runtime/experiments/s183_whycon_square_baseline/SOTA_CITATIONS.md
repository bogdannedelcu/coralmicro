# s183 — SOTA citation map

WBS: **OP-S10-W19-T4 step 4**.  Cross-references every algorithmic choice
in the WhyCon Gazebo eval pipeline to the SOTA paper it derives from.
This is the citation list the thesis chapter on perception will draw
from.

## Pipeline → paper

| Stage | Implementation | Citation |
|---|---|---|
| Adaptive threshold | Bradley rolling integral, SIMD (`aruco_threshold_rolling_simd`) | Bradley & Roth (2007) "Adaptive Thresholding Using the Integral Image", JGT 12(2):13-21 |
| 8-conn flood-fill labeling | `aruco_label_components` with inline 2nd-order moments | Rosenfeld & Pfaltz (1966) "Sequential Operations in Digital Picture Processing", JACM 13(4) |
| WhyCon detection pipeline | Phase A+B+W1+W2 in `sentai_aruco.cc::whycon_*` | Krajník, Nitsche, Faigl, Vaněk, Saska, Přeučil, Duckett, Mejail (2014) "A practical multirobot localization system", JINT 76:539-562 |
| W3 concentric inner-disc validation | `whycon_w3_check_` radial samples at 0.55·R / 0.95·R | Krajník et al. (2014) §3.1 (concentric check); our simplified radial-sample geometry is functionally equivalent to the full inner-blob search |
| 2nd-order moments → semi-axes + orientation | covariance eigenvalues, `2√λ_max`, `½·atan2(2μ₁₁, μ₂₀−μ₀₂)` | Hu (1962) "Visual Pattern Recognition by Moment Invariants", IRE Trans. IT-8; textbook computer vision |
| Single-marker depth `z = fx·d / (2·a)` | `whycon_pnp_inplace_` simplified face-on formula | Special case of Faugeras & Toscani (1986) conic-pose; only exact for face-on circles |
| Single-marker conic-pose (full SOTA) | **not yet implemented** — listed as future work | Faugeras & Toscani (1986) "The calibration problem for stereo", CVPR; Pagani & Stricker (2011) "Structure from motion using full spherical panoramic cameras", ICCV-W; closed-form via 3×3 eigendecomp of K⁻ᵀ·M·K⁻¹ |
| Multi-marker pose (Kabsch SVD) | `verdict_sota.py::kabsch_3d_3d`, joint 3D-3D rigid alignment | Kabsch (1976) "A solution for the best rotation to relate two sets of vectors", Acta Cryst. A32:922-923 ; Arun, Huang, Blostein (1987) "Least-Squares Fitting of Two 3-D Point Sets", IEEE PAMI 9(5):698-700 |
| Assignment search (Procrustes with correspondence) | `kabsch_with_assignment` exhaustive over N! perms; N≤4 trivial | Variant of Iterative Closest Point (Besl & McKay 1992); permutation-Procrustes is the closed-form correspondence solver for small N |
| Reflection-safe rotation | `R = V·diag(1,1,det(V·Uᵀ))·Uᵀ` | Umeyama (1991) "Least-Squares Estimation of Transformation Parameters Between Two Point Patterns", IEEE PAMI 13(4):376-380 — handles the SVD sign ambiguity that Arun et al. left unresolved |
| Closed-form camera intrinsics from FOV | `fx = (W/2) / tan(FOV_h/2)` | Hartley & Zisserman (2003) "Multiple View Geometry in Computer Vision", §6.1 pinhole model |
| Anti-cheat (sim consumes only camera + telemetry) | `gz_to_uds_bridge`, `sentai_camera_grab_gray_zerocopy`, mission inside sentai_sim | Operator-stated 2026-05-17, hard rule `[[sentai-sim-air-gapped-from-truth]]`; methodologically the same as **realistic SITL** practices in PX4 / ArduPilot CI flows |
| Ground truth recorder (host-only post-mortem) | `sim/scripts/gt_recorder.py` | Standard SITL evaluation practice; cf. Furrer et al. (2016) RotorS / Gazebo MAV simulator paper for the same pattern |

## What is original to this work

These are the choices that are NOT directly from a paper — operator-
specific and load-bearing for the thesis chapter:

| Choice | Rationale |
|---|---|
| 8-conn (not 4-conn) flood-fill | More robust at marker boundary thin connections; same SOTA algorithm class (Rosenfeld-Pfaltz) but a stronger neighbour set |
| Inline 2nd-order moments during flood-fill | Avoids a second bbox-rescan pass for axis recovery; net −0.26 ms on M7 at 4 markers vs split passes |
| H-pattern marker layout (4 corners + 2 mid-bar; ICAO Annex 14 helipad) | Symmetric on X, asymmetric on Y (top arm 20 cm, bottom arm 14 cm).  Cited reference: Kim, Yang & Kim 2013 (IROS) "A new approach to drone-based fast localization for landing using only landmark pattern recognition".  Iter-3 5-marker rotated cross abandoned because the markers were collinear on the cardinal axes — uneven lever arms gave X-MAE ≫ Y-MAE under Kabsch fit |
| Permutation-Procrustes for N≤4 | Closed-form correspondence + alignment; replaces the EKF-pose-dependent pixel-projection assoc which fails when cf2 EKF drifts |
| Annulus correction factor `axis_a / R = √(1 + (r₁/R)²)` | Compensates for the discrepancy between eigenvalue-derived semi-axis (which integrates over the annular mass) and the projected outer-ring radius.  Value: 1.166 for the ideal Krajník synth (s181), reduces to ~1.0 for Gazebo-rendered markers due to AA/PBR-induced blob smoothing (s182 iter-3 finding) |
| Cam-intrinsics from SDF FOV at runtime | Eliminates manual `set_intrinsics(...)` calls; reads SDF horizontal_fov + downscale ratio |

## Open SOTA gaps (next iterations)

1. **Conic-section pose recovery** — replace the simplified `z = fx·d/(2·a)`
   with the Faugeras-Toscani / Krajník conic eigendecomp.  Removes the
   annulus-correction factor entirely (the eigendecomp handles arbitrary
   blob shapes by construction).  Implementation in sentai_aruco.cc via
   the existing `jacobi_sym3` 3×3 eigendecomp (reused from sentai_calib
   Kabsch).  ETA: 3-4 hours.

2. **Tilt direction from conic** — gives the proper 2-fold-ambiguous
   normal vector + lets the temporal filter pick the consistent solution.
   Replaces the "perpendicular to projected major axis" heuristic.

3. **VPE forwarder in sentai_sim** — feed the Kabsch-fit drone pose back
   to cf2 EKF via `sentai.crazy.send_extpos` per tick.  Closes the
   Z-stability loop documented in iter-2 (drone drifts open-loop without
   VPE).  This is the standard pattern from the old host-side
   `aruco_to_vision_estimate.py` but moved into firmware per
   `[[missions-run-in-sentai-only]]`.

4. **WhyCode binary ID decoding** — would give per-marker ID without the
   permutation-Procrustes correspondence search.  Citation:
   Lightbody, Krajník, Hanheide (2017) "An efficient visual fiducial
   localisation system", Applied Soft Computing 51:62-71.

## Numbers we have ready to cite

From iter-3 (verdict_sota.py with permutation-Procrustes + corrected
fx=288.3 derived from SDF FOV):

  - X MAE = 5.95 cm,  max = 11.2 cm  (n=41)
  - Y MAE = 0.80 cm,  max =  1.5 cm  (n=41)
  - Z MAE = 101 cm (median 61 cm)    — pending GT-recorder fix +
    cf2-EKF-drift compensation

Iter-1 (per-marker median, deprecated):

  - X MAE = 9.03 cm
  - Y MAE = 17.81 cm
  - Z MAE = 49.39 cm  (different scene, less robust assoc)

The SOTA Kabsch + correct fx + permutation-Procrustes gives ~3×
better Y accuracy and similar X.  Z still drifts due to lacking VPE
feedback (cf2 EKF drift) — that's an open known issue, not an
algorithmic limitation of the Kabsch fit.
