# s190 — Calibration bringup via RPYT→HL handoff

**WBS:** `OP-S10-W21-T7`
**Created:** 2026-05-22
**Status:** ⬜ ITER-1 pending
**Supersedes:** `s187` phase-2 (Session 2 grind blocked by no-baro
positioning issue)

## Claim

`sentai.calib.run_bringup()` runs end-to-end in Gazebo cf2 SITL on the
0.5× WhyCon pad **without any GT data injection into cf2** — using:

1. **Classic Commander RPYT** (CRTP port 3 ch 0) for pre-airborne +
   climb.  IMU + onboard PID stabilize attitude; no position estimator
   needed.  Same path as XBox controller flight on a real CF Brushless,
   per `UART_RPYT_AGENT_PROMPT.md` in `~/work/crazyflie/CrazySim/`.
2. **ExtPos** (CRTP port 6 ch 0) with PnP-derived (x, y, z) after
   markers come into view.  Routes through `crtp_localization_service.c`
   identically on SIM and HW — HW-parity guaranteed.
3. **Meta-command `notifySetpointsStop`** (CRTP port 7 ch 1, byte 0)
   to relax commander priority from CRTP=2 back to LOWEST=1, allowing
   HL planner to take over.
4. **HL `go_to`** (NOT `takeoff` — drone is already airborne) for
   smooth navigation to home + bringup start position.

After this handoff completes, the existing
`sentai_calib_run_bringup()` orchestrator runs unchanged.

## Why s187 was blocked and how s190 fixes it

s187 phase-2 hit cf2 SITL chicken-and-egg: `plan_takeoff()` reads
`pos.z` from Kalman, which is garbage without baro + flow_deck.
Session 2 tried injecting GT z via SENSOR_TOF_SIM (iter-85+) —
anti-cheat violation per `[[sentai-sim-air-gapped-from-truth]]`.

s190 sidesteps the chicken-and-egg entirely: RPYT bypasses the
position controller (it goes through stabilizer → rate PID → motor
mixer with `mode.thrust=ABS`, never touches `state.position.z`).
Drone climbs open-loop until markers visible, THEN ExtPos engages
cf2 EKF with vision-PnP-derived position.

## What this iter validates

Single-drone calibration in SIM, on the 0.5× pad.  Per-axis tuning
of `T_BASE_U16`, `T_MAX_U16`, `RAMP_S` to find the empirically-correct
thrust profile for this Gazebo cf2 mass.  Real drones will need
similar tuning (battery age + payload + frame trim) — future work
(W21 entry) is to auto-learn this from a first-flight ramp.

## Pass criteria

For iter-N to be considered a successful bringup:

1. **Anti-cheat clean** — `bash sim/scripts/audit_anti_cheat.sh` PASS
   (no SENSOR_TOF_SIM injection, no GT consumed by sentai_sim)
2. **Markers acquired** — `n_markers ≥ 4` with valid PnP stable for
   ≥ 5 consecutive frames during RPYT climb
3. **Kalman converged** — `|cf2_pose.z − pnp_z| < 5 cm` and
   `|cf2_pose.x − pnp_x| < 10 cm` after 1 s of ExtPos streaming
4. **HL handoff successful** — commander priority returns to 1, HL
   `go_to` setpoints reach motors (drone moves toward target)
5. **Bringup orchestrator PASS** — `phase_reached=DONE_OK`,
   accepted=True, R drift < 1° OR diagnosable cause logged

## Files

- `mission_s190.py` — MicroPython mission, 6-phase implementation
- `run.sh` — host-side orchestrator; uses `sim/scripts/launch_sim.sh`
- `journal.txt` — per-iter log of params + outcome (this is the WBS
  iteration log; treat as append-only)
- `verdict_s190.py` — post-mortem analysis against pass criteria
  (writes summary JSON)
- `fr_current/` — Flight Recorder output (frames, events, scalars)
- `gt.jsonl` — host-side GT recorder output (read by verdict ONLY,
  never injected into sentai_sim — per anti-cheat HR)

## Anti-cheat note

This experiment consumes ONLY:
- Camera frames via `gz_to_uds_bridge` → `/tmp/sentai_cam.sock`
- cf2 telemetry via CRTP LOG (`sentai.crazy.pose_*`)

GT (`/world/.../dynamic_pose/info`) is recorded by host-side
`gt_recorder.py` and read ONLY by `verdict_s190.py` post-mortem.
Never fed back into sentai_sim or cf2.

See `[[sentai-sim-air-gapped-from-truth]]` for the broader rule and
`[[cf2-sitl-cheat-odom-gt]]` for the historical cheat that this
experiment finally removes from the loop.

## How to run

```bash
cd /home/bogdan/work/coralmicro
bash examples/sentai_runtime/experiments/s190_calib_bringup_rpyt_hl_handoff/run.sh
```

The script orchestrates: cleanup → `launch_sim.sh` (Gazebo + cf2 +
camera bridge + gt_recorder) → mission via REPL → verdict.

For interactive iteration tuning, run sentai_sim by hand and call
`mission_s190.run(T_BASE=..., T_MAX=..., RAMP_S=...)` from REPL.
