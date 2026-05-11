# s090 results — 2026-05-11 closed-loop hover-over-cat

Result of the run that closed the loop end-to-end: drone takes off
under MotionCommander control, climbs to 2.5m, detects the cat picture
on the ground via on-board SSD MobileNet V2 + SORT, navigates toward
it using tilt-compensated bbox-error → body-velocity, while
sentai.flow phase-correlation results stream into the cf2 SITL EKF as
SENSOR_FLOW_SIM packets at ~10 Hz.

## Numbers

| Metric                                | Value                            |
|---------------------------------------|----------------------------------|
| Run wallclock                          | ~30 s hover phase + 5 s climb/land |
| LOCK events (tid>0, conf≥30%)         | **271**                          |
| Flow packets sent to cf2 EKF          | **284**                          |
| Total STATE rows                      | 395                              |
| With confirmed track (tid>0)          | 315                              |
| With class=16 (cat)                   | majority (95%+ of tid>0)         |
| Peak SSD confidence on cat            | **~75%**                         |
| Drone takeoff height                  | 2.5 m (MotionCommander)          |
| Final drone XY                        | within ~10-25 cm of cat target   |
| Cat target world position             | (+0.40, -0.30) m                 |
| Drone X range during hover            | -0.00 .. +0.90 m (overshoot)     |
| Drone Y range during hover            | -0.75 .. +0.07 m                 |

## Files in this directory

- `run.mp4` — 17.6 s, 5 fps, 640×480, H.264 baseline.  Plays in VLC,
  browsers, mobile.  Has bbox overlay + HUD (drone XYZ, attitude RPY,
  bbox centroid+error, class+conf, gz frame seq).
- `state.tsv` — 17-column TSV, 5 Hz, all STATE rows from hover_logic
  (iter, tid, cls, conf, cx, cy, err_x, err_y, vx, vy, fvx, fvy,
  x1, y1, x2, y2, fseq).  `fseq` is the gz camera frame seq that
  produced this detection — lets you join with PPMs / video frames.
- `flight.tsv` — 7-column TSV, 50 Hz, cf2 EKF state (`ts`, roll,
  pitch, yaw, x, y, z).
- `hover.log` — host-side `[hover]` event log (LOCK events, lifecycle).
- `hover_sim.log` — full sentai_sim stdout (TPU stats, DETS, DIAG,
  raw STATE lines).
- `key_frame_600_cat_locked.png` — frame 600: drone at z=2.5m,
  cat clearly in image, bbox `cat 72.3%` exactly on cat.
- `key_frame_900_centered.png` — mid-flight centering attempt.
- `key_frame_1200_landing.png` — landing descent.

## Files NOT in this dir (regeneratable)

- `frame_NNNNNN.ppm` — 88 raw 640×480 RGB captures.  Each 921 KB,
  total ~80 MB — too big for git.  Re-generate by setting
  `SENTAI_DUMP_FRAMES_DIR=/tmp/foo` + running `/tmp/run_hover.sh`.
- `_vid_seq/frame_NNNNNN.png` — annotated overlay PNGs.  Re-generate
  via `make_video.py /tmp/foo`.

## How to reproduce

```bash
# 1. Stack (one-time per workstation session):
#    - Garden + cf2 running inside crazysim-garden distrobox
#    - TPU helper alive (run_hover.sh auto-restarts it pre-run)
#    - gz_to_uds_bridge ready

# 2. From repo root:
cmake --build build-sim --target sentai_sim
bash /tmp/run_hover.sh                  # full reset + experiment

# 3. After run completes:
python3 examples/sentai_runtime/experiments/s090_hover_over_cat/make_video.py \
        /tmp/sentai_frames_<TIMESTAMP>

# 4. Inspect:
#    - run.mp4 in VLC for visual playback
#    - state.tsv + flight.tsv in pandas for analysis
```

## What this run validated

1. **`sentai.flow` works on x86 SIM** identically to ARM (same
   `flow_phase_corr.cc`, just FFTW3 backend vs CMSIS).
2. **`sentai.flow → cf2 EKF` wire is plumbed end-to-end** via CrazySim's
   `SENSOR_FLOW_SIM` on `CRTP_PORT_SETPOINT_SIM=0x09`.  Patched
   `sensors_sitl.c` accepts per-packet stdDev so noisy phase-corr
   doesn't over-influence the EKF.  No firmware-side `app_sentai_bridge`
   needed — SITL HAL already has the receive plumbing.
3. **SSD MobileNet V2 COCO17 detects the cat picture at ~75%
   confidence** consistently across the hover phase.  Tracker (SORT)
   confirms class=16 (cat) on >95% of tid>0 frames.
4. **Drone responds to commanded body velocity** when:
   - `stabilizer.estimator=2` (Kalman) is set,
   - `kalman.resetEstimation=1` runs AFTER the estimator flip,
   - flow injection is active with reasonable stdDev (~4 px),
   - MotionCommander streams setpoints at 10 Hz on its background thread.
5. **Attitude-compensated bbox** (small-angle virtual horizontal cam
   frame) removes most of the tilt-induced bbox jitter that earlier
   versions of the controller chased as if it were real translation.

See Sim.md §10j for the full recipe + best practices + paper
references.
