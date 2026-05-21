# s187 — calib bringup Gazebo end-to-end

**WBS:** `OP-S10-W21-T4` (acceptance gate) — drives the unified bringup
orchestrator `sentai.calib.run_bringup(...)` through the full sequence on
the cf2 SITL stack with the WhyCon square pad in Gazebo.

This is the **digital-twin acceptance test** for the on-board production
bringup method per [[sentai-calib-is-production-bringup]].  Once it
passes, the same MP one-liner will run on the bench in front of a real
drone.

## What runs

```
gz sim worlds/whycon_square_pad.sdf  +  cf2 SITL  +  gz_to_uds_bridge
       │                                  │              │
       └──── camera frames @ /tmp/sentai_cam.sock ───────┘
                              │
                              ▼
                          sentai_sim (SIM build)
                              │
            mission_s187.py: takeoff → run_bringup → land → verdict
```

## Pass criteria (from `OP-S10-W21_calib_unified_bringup.md`)

| Metric                              | Threshold              |
|-------------------------------------|------------------------|
| `R_cam_to_body` drift from SDF      | < 1.0°                 |
| `cam_offset_B` recovery vs SDF      | < 5 mm (∞-norm)        |
| `kp_x`, `kp_y`                      | ∈ [0.30, 0.50]         |
| Hold-validation rms drift           | < 30 mm over 10 s      |
| Hold-validation max drift           | < 80 mm                |
| Persistence round-trip after reboot | calib state bit-identical |

## Anti-cheat compliance

Orchestrator consumes only:
- `sentai_markers_get_latest()` — PnP via WhyCon backend, camera
  frames via bridge ([[sentai-sim-air-gapped-from-truth]]).
- `sentai_crazy_pose()` — cf2 EKF telemetry over CRTP LOG (not GT).

`gt_recorder` runs host-side for verdict comparison only ([[gt-recorder
-tool]]).  No GT injection into sentai state.

## Files

- `README.md` (this) — scope + acceptance gate
- `mission_s187.py` — MP orchestration: takeoff + run_bringup() + poll + land
- `smoke_phase1.py` — host-runnable REPL smoke (no Gazebo) that validates
  the binding surface (run_bringup spawn / phase polling / abort / result
  shape).  Useful for sanity checks during T4 iteration.
- `run.sh` — launcher stub (TBD)
- `verdict_s187.py` — post-mortem comparison vs SDF ground truth (TBD)

## Status

- Phase 1 (binding smoke): PASS — see `smoke_phase1.py` output.
- Phase 2 (Gazebo end-to-end): TBD — depends on `whycon_square_pad.sdf`
  pad geometry matching `mission_s187.py` `MARKER_WORLD`.  Plan to share
  the MARKER_WORLD constant with the s182 fork once that lands.

## Known limitations of T4 v1

The orchestrator computes `cam_offset_B` as the per-sample mean residual
of `(marker_W − R · tvec_cam − drone_W)` assuming the drone is in level
hover.  Per-sample yaw rotation is ignored.  This introduces noise
averaged across the 4-pose sweep — refinement (apply per-sample
`R_world_body(yaw)`) is W21-T5 territory if the < 5 mm gate is missed.

Yaw autotune (`SENTAI_CALIB_AXIS_YAW`) is NOT exercised by T4 — the
existing autotune relay supports X/Y only.  `kp_yaw` is persisted as a
slot in `calib.ini` but populated by future work.
