# s197 - guarded SOTA camera-orientation calibration

**Task**: TD-S10-A3 / TD-S10-B3
**Status**: organized successor to s195

## Intent

s197 is the cleaned-up A3 implementation path after the s195 smoke tests.  It
uses s194 only as an example of simulator packaging, journaling, and artifact
collection.  The control and calibration logic follows the A3 constraints:

- no `sentai_calib_bringup` state machine as the primary flow;
- no `sentai.markers.get_drone_pose_tuple(yaw)` in the calibration decision
  path;
- no yaw, EKF, or Gazebo ground truth to decide camera/body orientation;
- no lateral position hold before the image response has identified the
  camera/body mapping;
- all marker-lock gates count only fully visible WhyCon markers.

## Method

The mission is built around uncalibrated visual servoing:

1. Detect WhyCon image features in camera space.
2. Gate on fully visible marker count, centroid, radius/scale, and visual-Z
   stability.
3. Use thrust-only takeoff until marker lock.
4. Stabilize Z from visual scale/depth before lateral commands.
5. Apply small complementary RPYT pulses on body axes.
6. Estimate the local image Jacobian from commanded pulses and observed image
   deltas.
7. Score determinant `+1` discrete camera/body rotations against the measured
   response.
8. Validate the selected candidate's optical/body-Z convention from current
   WhyCon pose observations.
9. Validate the selected candidate with smaller motions.
10. Commit and persist only after validation passes repeatedly.

## Current Scope

s197 implements:

- simulator launcher and artifact layout;
- mission setup inside `sentai_sim`;
- WhyCon detector configuration for the `_small` seven-marker layout;
- raw image-feature extraction with full-visibility classification;
- preflight stationary feature sampling;
- thrust-only marker acquisition smoke with neutral RPYT, stopping only after
  7 fully visible markers are observed for 10 consecutive frames;
- post-lock brake/settle phase to reduce upward velocity before Z-hold;
- visual-Z hold smoke using only WhyCon image-space/depth features and thrust,
  targeting a higher calibration band so X/Y probes keep FOV margin;
- first strict complementary one-axis response smoke, measuring centroid
  deltas only and requiring the centroid to return near the local baseline;
- post-mortem GT verdict for altitude/drift sanity checks;
- candidate scoring with explicit command-response/body-axis labels;
- optical-axis validation for the inferred body-Z row before final status;
- final candidate validation with smaller independent pulses;
- summary/journal output.

Persistence remains disabled.  `sentai.calib.commit_R()` and
`sentai.calib.save()` should only be enabled after repeated successful GUI runs
select the same candidate.

## Run

```bash
bash run.sh iter1
```

Artifacts are written under the iteration folder:

```text
examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iterN/
  sentai_repl.log
  mission_s197_journal.txt
  mission_s197_summary.json
  verdict.log
  fr_current/gt.jsonl
```

Later B3 runs should also mirror research artifacts under
`dataset/TD-S10-B3/calib_takeoff_YYYYMMDD_HHMMSS/`.

## Next Steps

- Verify the stationary feature stream in GUI SIM.
- Inspect `verdict.log` and `fr_current/gt.jsonl` after every smoke.  GT is
  post-mortem only and must never enter the mission decision path.
- Tune visual-Z stabilization until it holds the marker band for several
  seconds without losing FOV.
- Tune complementary pulse recorder, still without committing `R`.
- Add candidate scoring and validation.
- Only then enable `sentai.calib.commit_R()` and `sentai.calib.save()`.
