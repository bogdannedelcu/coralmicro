# s153 — long-distance exploration on sentai.servo paradigm

**Date**: 2026-05-17
**Status**: NEW — servo-paradigm port of s147 (which was MP-only port of s137)

Migrates s147 from `sentai.crazy.* + crtp_log.py` to `sentai.servo.*` per
[[next-steps-2026-05-17]] A1.  Same trajectory + closure gates.

## What this proves

After Stage 4.A landed (commit 58f47bd4) the canonical mission shape uses
`sentai.servo.{init,takeoff,go_to,pose,land}` with pose feedback in C —
no more `crtp_log.py` 270-LoC MP protocol per mission.  s153 demonstrates
the exploration shape under this paradigm.

## Files

```
s153_explore_long_servo/
├── README.md
├── mission_s153.py    # uses sentai.servo only
└── verdict.py         # original 6 gates + 2 servo-specific (actions_ok, faults_oob)
```

No external deps beyond what sentai_sim already provides.

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 7 fired (init/origin/takeoff/waypoints/return_home/land/disarm) |
| G3 waypoints visited | ≥ 2 |
| G4 total_path_m | ≥ 0.85 m (cf2 undershoot budget — see verdict.py) |
| G5 closure_xy | < 10 cm |
| G6 errors empty | |
| G7 servo_actions_ok | ≥ 7 |
| G8 servo_faults_oob | == 0 |
| G9 journal events | all 14 fired |

## How to run

```bash
# 1. SITL stack
distrobox enter crazysim-garden -- bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &
until ss -lun | grep -q ":19850"; do sleep 2; done

# 2. Stage mission
cp examples/sentai_runtime/experiments/s153_explore_long_servo/mission_s153.py \
   build-sim/sentai_fs_root/

# 3. Run
echo "import mission_s153; r = mission_s153.run(); print('FINAL:', r['status'])" \
    | ./build-sim/sim/sentai_sim

# 4. Verdict
python3 examples/sentai_runtime/experiments/s153_explore_long_servo/verdict.py
```
