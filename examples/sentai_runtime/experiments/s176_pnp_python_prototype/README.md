# s176 — Python local PnP prototype (rotation-invariance audit)

## Goal

Validate the IPPE 2-solution disambiguation algorithm in Python on
already-captured frames BEFORE migrating to C in `sentai_aruco.cc`.
Operator 2026-05-18: faster iteration locally; only port to firmware
after the math is proven.

## Approach

`s175` learned the hard way that **rotating the image** with
`cv2.warpAffine` introduces bilinear-interp noise that dominates the
signal we're trying to measure (PnP ambiguity).  So `s176` rotates
the **corner coordinates** algebraically instead — the image (and
therefore the corner-detection accuracy) is held constant; only the
PnP input changes.

### Pipeline

1. Read `frame_original.pgm` (clean Gazebo render from `s175`).
2. Detect ArUco markers with `cv2.aruco.ArucoDetector` → 4 corners
   per marker.  Run this **once**.
3. For each in-plane image rotation θ ∈ {0°, 5°, 10°, ..., 90°}:
   a. Rotate each marker's 4 corner pixel coordinates around the
      image center by θ.
   b. Run PnP in three variants:
      - **REF**: `cv2.solvePnP(SOLVEPNP_IPPE_SQUARE)` — OpenCV's
        reference IPPE 2-solution implementation.
      - **OURS-1**: our homography-decomposition PnP (1 solution),
        ported from `sentai_aruco.cc:aruco_pnp_from_corners` pre-T18.
      - **OURS-2**: same + the `R_180(v) * R1` second-solution pick
        by reprojection error (T18-A draft).
   c. Record per-marker `tvec_cam[2]` (depth from camera to marker).
4. Plot per-marker z vs θ for all three algorithms.

### Pass criterion

For each algorithm, `max |z(θ) - z(0°)|` across θ and across all
markers must be < **5 mm**.  REF is expected to pass (OpenCV's
SOLVEPNP_IPPE_SQUARE is the literature reference).  OURS-1 expected
to fail (no disambiguation).  OURS-2 must pass for us to be allowed
to port back to C.

## Files

- `prototype.py` — the three PnP implementations + sweep + verdict
- `frame_original.pgm` — symlink to s175 source frame
- `run.sh` — invoke prototype and print verdict
- `plot.png` — z vs θ plot (generated)
- `run.log` — captured output
