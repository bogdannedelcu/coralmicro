# s132 — L5 lifter Gazebo integration

**Purpose**: validate end-to-end that `sentai.object_lifter` (shipped in
L5, commit `17cbe3aa`) converges to a correct 3D world position under
**dynamic Gazebo cadence** when driven from real-time bbox observations.

Pas următor după:
- s131 — math validated in pure Python (`lifter_proto.py`)
- L5 — C++ port shipped, 38/38 driver tests PASS in SIM

s132 closes that loop by feeding REAL Gazebo ArUco detections at REAL
Gazebo flight cadence into the C++ lifter via REPL calls.

## Mission profile (simplest path)

1. Force-respawn cf2 at origin (per `[[experiments-start-from-origin]]`)
2. cf2 takeoff → `z=1.5 m`, yaw=0 (single target: marker id 0 at world `(+0.15, +0.10, 0.20)`)
3. REPL: `sentai.object_lifter.clear()` + `set_camera(defaults)`
4. Wait for fresh PPM → detect_in_ppm → marker id 0 → compute:
   - `u_c, v_c` = `marker.corners.mean(axis=0)` (pixel center)
   - `bbox_w_px` = mean of 4 edge lengths (rotation-tolerant)
   - `drone_W` from `cf2.stateEstimate`
   - REPL: `sentai.object_lifter.init_from_bbox(0, 0, u, v, w_px, 0.0625, drone_W, 0.0)`
5. **Lateral pass**: 4 steps × 0.10 m body-Y, settle 0.5 s, capture fresh PPM, update_bbox + log lifter state
6. Final query: `world_pos(0)`, compare to GT `(0.15, 0.10, 0.20)`
7. Land + disarm
8. FlowBaseline gate post (`[[gate-every-layer-no-exceptions]]`)

## Why a lateral pass

Lifter needs **parallax** to converge (monocular triangulation,
see `[[s131-lifter-math-shipped]]`). Pure hover → ρ never updates.
0.4 m lateral baseline at z=1.5 m gives ~26° angle change as seen
from the marker → adequate XY observability. Z is structurally weak
in monocular geometry; pass criteria reflect this.

## Pass criteria

| Check | Threshold | Source |
|---|---|---|
| status == LIFTED at end | required | Civera ε·ρ² linearization gate |
| ‖est_xy − gt_xy‖ | < 20 cm | s131 replay 1.7–7 cm proven; loose for SIM jitter |
| \|est_z − gt_z\| | < 60 cm | s131 Z weak, expected monocular |
| σ_ρ_final | < 0.5 × σ_ρ_init | EKF actually updates |
| n_obs at end | ≥ 4 | ≥4 of 5 updates accepted |
| rejects (any) | ≤ 1 | tolerate 1 outlier on marker edge cases |

Mission FSM sanity (same as s128/s129/s130):
- `last_action == DISARM`, `last_result == 0`
- `armed == 0`, `flight == GROUND`
- Zero servo fault counters

## How to run

```bash
bash examples/sentai_runtime/experiments/s132_lifter_gazebo/run.sh
```

Exit 0 on PASS, 1 on FAIL. Always force-respawns cf2 at origin.

## Logs captured (under `/tmp/s132_lifter_gazebo/`)

- `summary.json` — verdict reads this; includes lifter_trace
- `lifter_trace.json` — per-frame `{u_c, v_c, bbox_w_px, drone_W, status, rho, var_rho, n_obs, world_pos_est, world_pos_err}`
- `journal.txt` — REPL-side sentai.sim.journal_* output
- `mission.log` — host-side step trace
- `repl.transcript` — raw REPL stdout
- `cf2_telemetry.json` — stateEstimate @ 20 ms

## Related

- `[[l5-shipped]]` — L5 commit + API surface
- `[[s131-lifter-math-shipped]]` — Python prototype math validation
- `[[experiments-start-from-origin]]` — cf2 respawn rule
- `[[gate-every-layer-no-exceptions]]` — FlowBaseline gate post
- `[[gazebo-gui-required]]` — Gazebo GUI active during run
- `[[sentai-sim-journal]]` — structured log convention
- `[[flowbaseline-canonical-config]]` — canonical gate config
- objects_plan.md §5 (Stage 5 lifter) + §22 (places track decision)
