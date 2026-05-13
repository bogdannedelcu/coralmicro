# s122 — sentai.slam.update_3d() with class-size priors (Stage 1.C)

**Goal.** Verify the new monocular `update_3d()` MP binding uses the
per-class size priors registered in Stage 1.B (`set_class_prior`) to
compute a pseudo-depth from each detection's bounding-box height.

Pinhole pseudo-depth:

  range_m = (class_size_m × focal_px) / bbox_height_px

Where `focal_px = img_w / (2·tan(fov_h/2))`, set by `slam.init(...)`.
If no prior is registered for the class, that detection falls back to
`SLAM_DEFAULT_RANGE` (2.0 m) — same as the existing monocular `update()`.

## Test scenarios

  - Two detections of class 0 (person, prior 1.7 m) at the same centre
    pixel but with bbox heights 120 px vs 30 px → ranges 3.93 m and
    15.71 m (4× ratio).
  - Unknown class (no prior) → still creates a landmark, range = 2.0 m.
  - Degenerate bbox (height < 2 px) → skipped (no landmark added).

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s122_slam_update3d/test_slam_update_3d.py
```

## Verified 2026-05-13

```
  OK: update_3d returned 2 landmarks
  OK: two landmarks in map
  OK: near landmark depth ~3.9 m
  OK: far landmark depth ~15.7 m
  OK: far/near ratio ≥ 3×
  OK: no-prior class still creates lm
  OK: no-prior produces 1 landmark
  OK: no-prior range ≈ default 2.0 m
  OK: degenerate bbox skipped (n=0)
  OK: degenerate bbox produces 0 lms

[test] PASS — slam.update_3d uses class prior for pseudo-depth
```

10/10 PASS.
