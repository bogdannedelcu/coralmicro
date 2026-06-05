---
name: yaw-anchor-mirror-picker
description: "When Kabsch fits a coplanar pad with rotational/mirror symmetry, disambiguate the 4 valid det=+1 solutions by checking sign(R[0,0]) and sign(R[1,1]) vs cos(cf2_yaw); flip t around the marker centroid on each mismatched axis. Beats history-based temporal coherence."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: aa6526b2-0b0a-48e3-8189-37faf9598c5f
---

# Yaw-anchored mirror picker for symmetric marker pads

When the marker constellation has rotational / mirror symmetry
that maps the set to itself (square 4-corner pad, regular polygon,
etc.) Kabsch can return any of N equally-valid det=+1 rotations
with the same per-point residual.  SVD picks one randomly per
frame -> the estimate flips between mirrors and you get a bimodal
X/Y scatter despite millimetre-level Kabsch residual.

## Rule

After a Kabsch fit on a symmetric pad, disambiguate the solution
using the cf2 estimator's yaw as a physical anchor:

```c
const float cos_yaw_expected = cosf(cf2_yaw);
const int   flip_x = (R[0] * cos_yaw_expected < 0.0f);  // R[0][0]
const int   flip_y = (R[4] * cos_yaw_expected < 0.0f);  // R[1][1]
if (flip_x || flip_y) {
    float mx = 0, my = 0;
    for (int i = 0; i < n; ++i) { mx += marker_W[i].x; my += marker_W[i].y; }
    mx /= n; my /= n;
    if (flip_x) t[0] = 2.0f * mx - t[0];
    if (flip_y) t[1] = 2.0f * my - t[1];
}
```

## Why

A pad symmetric under X-mirror, Y-mirror, AND 180-Z rotation
(e.g. our s183 32x32 cm 6-marker square) gives Kabsch FOUR
equally-valid det=+1 rotations:

| Diag(R)        | Geometric meaning           | Effect on t |
|---------------|------------------------------|--------------|
| (+1, +1, +1)  | identity (correct, yaw=0)    | none         |
| (+1, -1, -1)  | 180 deg around X             | Y-flipped    |
| (-1, +1, -1)  | 180 deg around Y             | X-flipped    |
| (-1, -1, +1)  | 180 deg around Z             | X+Y flipped  |

All have det=+1 (Kabsch's reflection-safe constraint enforces it).
The SVD has no way to choose between them from the point cloud
alone -- they all minimise the Procrustes loss identically.

Physical disambiguation: cf2's onboard EKF yaw is independent of
the marker geometry, so when the drone hovers ~level (|yaw| <
~pi/4) the recovered R must have positive cos(cf2_yaw) on its
top-left 2x2 diagonal.  If a diagonal entry has the wrong sign,
that axis was mirrored -- flip t around the marker centroid on
that axis.

## Why NOT history-based temporal coherence

The intuitive alternative is "track which mirror was picked last
frame, keep picking it".  This fails: when the drone's yaw drifts
close to the mirror axis (yaw approx pi/2 or pi/4 for a 90-deg
symmetric pad) the correct solution itself changes.  Temporal
coherence then locks the estimate into the WRONG mirror for the
new yaw regime, and there's no second observation to escape it.

Yaw-anchor is stateless, costs ~4 fmuls + 4 fcmps per frame, and
never traps in a wrong attractor.

## Empirical result (s183, OP-S10-W19-T6a)

| Axis | iter-4 raw Kabsch | iter-5 yaw-anchored | improvement |
|------|--------------------|----------------------|--------------|
| X MAE | 1.5 cm (bimodal)   | 2 mm                 | 7.5x         |
| Y MAE | 1.4 cm (bimodal)   | 2 mm                 | 7x           |
| Z MAE | 1.1 mm (unaffected) | 1.1 mm              | -            |

## How to apply

1. Use it whenever Kabsch (or any closed-form Procrustes) runs
   on a marker pad with discrete symmetries -- squares, regular
   polygons, mirror-symmetric H/T patterns.

2. The anchor MUST be a yaw source independent of the marker pose
   itself.  cf2 EKF yaw, IMU magnetometer, or a previous-frame
   marker-pose where the picker has converged all work.  Using
   the SAME-frame marker yaw is a tautology and won't help.

3. Asymmetric pads (e.g. three markers in a scalene triangle) do
   not need this -- they have a unique Kabsch fit by construction.

4. For pads with N>4-fold symmetry (regular hexagon: 6-fold) the
   sign-of-diagonal test isn't sufficient; need a multi-hypothesis
   rank by which-hypothesis-best-matches-anchor-yaw.  Out of scope
   for s183's 4-fold pad; defer until we hit it.

## Reference implementations

- Host (Python): `examples/sentai_runtime/experiments/s183_whycon_square_baseline/verdict_sota.py:443-468`
- Runtime (C): to land under W19-T6b on top of [[op-s10-w20-svd-shipped]]'s `sentai_kabsch_align`.
