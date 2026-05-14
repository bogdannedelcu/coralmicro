# s129 — L4.2Baseline (visual servoing — IBVS / PBVS)

**Purpose**: prove the closed-loop visual centering primitive that L4.1
(EKF-closed-loop) cannot deliver on its own.  Drone starts at origin,
detects an ArUco marker via the downward camera, and navigates **until**
the marker sits within `MARKER_PIXEL_TIGHT_PX` of the image centre —
no world coordinates passed into the inner control loop.

This is the step the operator named when L4.1's id1 visual error stayed
at ~80 px despite EKF residual dropping to 6 cm.  The remaining offset
is mostly camera-CoM offset (4 cm body-back) + small yaw rotation; only
a pixel-/PnP-based loop can correct it.

## Pipeline

```
                    Gazebo /downward_cam/image
                              │
                gz_to_uds_bridge → /tmp/sentai_cam.sock
                              │
                       sentai_sim (camera bridge)
                              │
                  SENTAI_DUMP_FRAMES_DIR (PPM dumps every 15)
                              │
        ┌─────────────────────┴─────────────────────┐
        │  mission_l42.py inner loop                │
        │  (every NAV_SETTLE_S):                    │
        │    1. read latest PPM                     │
        │    2. detect ArUco (estimate_pose=True)   │
        │    3. tvec = marker position, cam frame   │
        │    4. body_delta = R_cam_to_body @ tvec   │
        │    5. mc.move_distance(body_dx,body_dy,0) │
        │    6. servo.move(body_dx,body_dy,0)       │
        │       (intent recording)                  │
        └───────────────────────────────────────────┘
                              │
                  marker pixel error → 0
```

## Control law (PBVS via PnP tvec)

For each detection iteration:

```python
# tvec[0..2] = marker position in CAMERA frame (metres),
#              X right, Y down, Z optical axis (down for downward cam).
# To put camera over marker, camera must move by (-tvec[0], -tvec[1])
# in camera frame.  Convert to body frame:
body_delta = R_CAM_TO_BODY @ (-tvec[0], -tvec[1], 0)
# With R_CAM_TO_BODY from aruco_detector.py:
#   body_dx = -(-tvec[1]) = +tvec[1]
#   body_dy = -(-tvec[0]) = +tvec[0]
# (signs verified empirically in first run; flip if wrong direction.)
```

If `|body_delta| < NAV_TOL_M`: converged.  Otherwise issue
`mc.move_distance(body_dx, body_dy, 0)` + `servo.move(...)` (intent
recording) and wait `NAV_SETTLE_S` before next detection.

Hard cap: `NAV_TIMEOUT_S` total time and `NAV_MAX_ITERS` iterations.

## Pass criteria (verdict.py)

L4-side FSM (same as s128):
- `servo.status()['actions_ok'] ≥ 5 + 1 hover + ≥1 move per iter`
- All fault counters == 0
- `last_action == DISARM`, `last_result == 0`
- Final `armed==0`, `flight==GROUND`

L4.2-specific:
- `converged == True` (loop exited via tolerance, not timeout/max-iters)
- `n_iters ≤ NAV_MAX_ITERS`
- Final visual: `px_dist < MARKER_PIXEL_TIGHT_PX` (30 px ≈ 5 cm physical
  at z=1 m — TIGHTER than the 80 px L4.1 threshold)
- `n_iters_to_converge` is reported but NOT pass-gating; lower is better.

Note: L4.2 deliberately uses NO pre-seeded `sentai.objects` and does
NOT consult cf2 EKF position for the inner loop.  The drone navigates
on camera feedback alone.  This isolates the visual-servoing primitive
from world-frame state.

## How to run

```bash
bash examples/sentai_runtime/experiments/s129_l42baseline_centering/run.sh
```

Exit 0 on PASS, 1 on FAIL.  Always force-respawns cf2 at origin per
`[[experiments-start-from-origin]]`.

## Logs captured

Identical structure to s128 (under `/tmp/s129_l42baseline/`):
- `summary.json`, `journal.txt` (REPL-side journal), `mission.log`,
  `repl.transcript`, `cf2_telemetry.json`, plus
- `ibvs_iter_log.json` — per-iteration `(t_ms, tvec, body_delta,
   pixel_err)` so the convergence curve is plottable post-mortem.

## Related

- s128 L4.1Baseline (`s128_l41baseline_seeded/`) — EKF-closed-loop with
  pre-seeded world coords.  Test that L4.2 IMPROVES upon.
- [[servo-l4-shipped]] — L4 servo skeleton API.
- [[sentai-sim-journal]] — structured logging used here.
- [[experiments-start-from-origin]] — reproducibility rule.
