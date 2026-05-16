# s147 — Long-distance exploration (MP-only port of s137)

**Date**: 2026-05-16
**Status**: shipped, awaits live Gazebo verification.

## What this migrates

s137 (host-orchestrated) → **MP-only** following the
`[[missions-run-in-sentai-only]]` firm rule.  Same shape, simpler:
- 2 waypoints (matches s137's "2 markers visited")
- multi-leg trajectory ≥ 1 m total
- INSPECT dwell at each waypoint
- return + land with closure < 10 cm vs PHYSICAL_ORIGIN

What we DON'T migrate (deferred to Stage 4):
- L6 explore FSM transitions (APPROACH → INSPECT → RETURNING by state)
- L5 lifter parallax (real marker tracking)

This proof-of-concept demonstrates the **trajectory + verdict** part
of s137 in MP-only.  Stage 4 will add the L6 FSM integration.

## Files

```
s147_explore_long_mp/
├── README.md
├── mission_s147.py    # MP mission, 2 waypoints, INSPECT dwells, closure
└── verdict.py         # 7 hard gates
```

Reuses `crtp_log.py` from `s146_pose_feedback/` (must be staged
alongside in `build-sim/sentai_fs_root/`).

## Run (live)

```
# 1. cf2 SITL + Gazebo up
distrobox enter crazysim-garden -- bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &
until ss -lun | grep -q ":19850"; do sleep 2; done

# 2. stage crtp_log + mission
cp examples/sentai_runtime/experiments/s146_pose_feedback/crtp_log.py build-sim/sentai_fs_root/
cp examples/sentai_runtime/experiments/s147_explore_long_mp/mission_s147.py build-sim/sentai_fs_root/

# 3. run mission via REPL
echo "import mission_s147; r=mission_s147.run(); print('FINAL:', r['status'], 'closure:', r['closure_xy'])" \
    | ./build-sim/sim/sentai_sim

# 4. verdict
python3 examples/sentai_runtime/experiments/s147_explore_long_mp/verdict.py
```

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 8 fired |
| G3 waypoints visited | ≥ 2 |
| G4 total_path_m | ≥ 1.0 m (proves the drone actually moved) |
| G5 closure_xy | < 10 cm |
| G6 no errors |  |
| G7 journal events | all expected |

## Related

- `[[s146-pose-feedback-shipped]]` — base pattern this extends
- `[[missions-run-in-sentai-only]]` — firm rule
- `[[sim-test-must-return-home]]` — closure gate
- Original: `examples/.../s137_explore_long/`
