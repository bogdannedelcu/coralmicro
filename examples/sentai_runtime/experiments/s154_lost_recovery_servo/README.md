# s154 — LOST recovery on sentai.servo paradigm

**Date**: 2026-05-17
**Status**: NEW — servo-paradigm port of s148 (MP-only port of s138)

Migrates s148 from `sentai.crazy.* + crtp_log.py` to `sentai.servo.*` per
[[next-steps-2026-05-17]] A1.  Mid-mission ascend +30 cm as LOST trigger,
dwell, recover, return, land — same gates as s148.

## Files

```
s154_lost_recovery_servo/
├── README.md
├── mission_s154.py
└── verdict.py
```

## Trajectory

```
origin → takeoff(0.75) → approach(0.20, 0.10) → LOST_SIM(+30cm z, dwell 1.5s)
       → RECOVER(back to 0.75m) → tight_return(origin) → land
```

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 8 fired |
| G3 lost_simulated | == true |
| G4 lost ascend Δz | ≥ 15 cm |
| G5 pose_after_recovery captured |  |
| G6 closure_xy | < 10 cm |
| G7 errors empty |  |
| G8 servo_actions_ok | ≥ 7 |
| G9 servo_faults_oob | == 0 |
| G10 journal events all fired |  |

## How to run

```bash
distrobox enter crazysim-garden -- bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &
until ss -lun | grep -q ":19850"; do sleep 2; done

cp examples/sentai_runtime/experiments/s154_lost_recovery_servo/mission_s154.py \
   build-sim/sentai_fs_root/

echo "import mission_s154; r = mission_s154.run(); print('FINAL:', r['status'])" \
    | ./build-sim/sim/sentai_sim

python3 examples/sentai_runtime/experiments/s154_lost_recovery_servo/verdict.py
```
