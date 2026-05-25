# s167 — FlowBaseline2 (calibrated, 4-phase) with GT comparison

**WBS**: OP-S8-W1 (path toward T8 — new FlowBaseline canonical).
**Why**: s166 trial #1 showed cf2 EKF diverges from Gazebo GT by
67–92 cm under a 15 s straight hover at z=1 m post-cheat-removal.
Root cause: position-PID chases an EKF that has no strong vision
anchor → positive-feedback drift + markers leave FOV.

## What changes vs s166

Adds an **explicit calibration phase** at low altitude before the
measurement hover so the EKF is pinned to vision before the controller
is given authority.

| Phase | t (s) | z (m) | What |
|---|---:|---:|---|
| **F1** takeoff | ~3 | 0 → 0.60 | Climb to a wide-FOV altitude (all 4 markers comfortably in view, ~30 cm margin per axis). |
| **F2a** static pin | 2 | 0.60 | VPE @ 20 Hz, full-conf — pin cf2 EKF to PnP. Average PnP-z to seed Z baseline (single re-seed via `extpos.send_extpos(0,0,z_mean)` at phase end). |
| **F2b** axes probe | ~6 | 0.60 | 4 directional steps ±4 cm (+X, −X, +Y, −Y). Per leg: snap PnP+EKF, command setpoint, integrate flow grid-px, snap PnP+EKF again. Least-squares fit 2×2 mapping `EKF_delta_body = BX @ flow_int_grid`. Sign-snap rows to nearest of {(±1,0), (0,±1)} → derived BODY_XFORM. Override the SIM-hardcoded `(0,-1,-1,0)` if the fit disagrees. |
| **F3** measurement | ~12 | 0.60 → 1.00 | Climb 1.5 s to z=1 m, then 10 s hover with VPE @ 5 Hz (gentle, like aruco_hover). This is the actual baseline measurement; gated on GT drift. |
| **F4** land | ~3 | 1.00 → 0 | `mc.land(velocity=0.3)`. |

Total mission: ~26 s (vs aruco_hover's ~20 s).

## Hard rules (enforced by `run.sh`)

1. **Force-respawn at origin** — unconditional `stop.sh` + force-kill, no `is_up` guard. Per [[experiments-start-from-origin]] + OP-S8-W1-T6.
2. **Spawn assert** — `gz model -m crazyflie_0 --pose` query; abort unless `|x|<0.05 AND |y|<0.05 AND z<0.10`.
3. **GT host-side only** — uses the canonical `sim/scripts/gt_recorder.py` writing JSONL. Never injected back into cf2 or sentai_sim.
4. **Gazebo GUI** — `launch_hybrid_cf2.sh` brings up both server + GUI.

## Files

| File | Purpose |
|---|---|
| `mission_flowbaseline2.py`     | The 4-phase mission (cflib client). |
| `run.sh`                       | Orchestrator: respawn → SITL → sentai_sim → bridge → GT recorder → mission → verdict. |
| `verdict.py`                   | Loads `phases.json` + `gt_poses.jsonl`, emits per-phase summary + 3 plots, GT-gates F3. |
| `README.md`                    | This file. |

The canonical GT recorder lives at `sim/scripts/gt_recorder.py` and is
shared by all future GT-aware experiments. Env vars: `GT_RECORDER_OUT`,
`GT_RECORDER_WORLD`, `GT_RECORDER_MODEL`, `GT_RECORDER_DISTROBOX`.

## Outputs (`/tmp/s167_flowbaseline_calibrated/`)

| File | Source | Content |
|---|---|---|
| `gt_poses.jsonl`     | gt_recorder      | Cf2 GT @ ~200 Hz, host time. |
| `phases.json`        | mission          | Per-phase samples + derived BODY_XFORM + ekf_trace. |
| `spawn_pose.json`    | run.sh           | Gz model pose at takeoff (T6 forensics). |
| `summary.json`       | verdict          | Per-phase metrics + gate. |
| `plot_phases.png`    | verdict          | XY trajectory coloured per phase (F1 gray → F2a green → F2b orange → F3 blue → F4 purple) + full GT light red. |
| `plot_xy_F3.png`     | verdict          | F3-only top-down, distinct EKF/GT colours + per-sample residuals. |
| `plot_err_F3.png`    | verdict          | F3 per-axis EKF−GT error vs time. |

The mission ALSO writes a `…/s091_aruco_lowalt/hover_log.json` mirror
of just the F3 portion, in the same shape as s166's, so the s166
verdict script can be re-run on F3 alone if needed.

## Run

```bash
bash examples/sentai_runtime/experiments/s167_flowbaseline_calibrated/run.sh
```

Exit 0 = PASS (F3 GT-based gate), exit 1 = FAIL. Inspect plots either
way — first-time we are calibrating, not just gating.

## Gates (tentative — pre-multi-trial calibration)

```
F3 GT dist_max_m  < 0.30
F3 GT dist_mean_m < 0.20
```

After 3+ trials with stable BODY_XFORM convergence, gates tighten and
the result becomes the new canonical `[[flowbaseline-canonical-2-no-cheat]]`
entry replacing s127's 7.4 cm cheat-masked number.

## Open questions for trial 1

1. Does the F2b probe yield a sign-consistent `BODY_XFORM` across legs (sanity for the derivation), and does it agree with the SIM hardcode `(0,-1,-1,0)`?
2. Does the F2a single `extpos` Z re-seed actually pin the EKF to PnP-z (we should see |EKF-GT| Z stay under 10 cm in F3 instead of >50 cm)?
3. Does F3 still go unstable, indicating that the calibration phase isn't enough and we need ongoing VPE rate-up?

These shape the next iteration (s168 or in-place s167 tweaks).
