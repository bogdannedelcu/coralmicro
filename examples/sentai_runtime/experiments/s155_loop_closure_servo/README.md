# s155 — Loop closure on sentai.servo paradigm

**Date**: 2026-05-17
**Status**: NEW — servo-paradigm port of s150 (MP-only port of s143)

Migrates s150 from `sentai.crazy.* + crtp_log.py` to `sentai.servo.*` per
[[next-steps-2026-05-17]] A1.  Same 2-lap loop closure pattern: store 3
descriptors lap-1, query gallery lap-2.

## Files

```
s155_loop_closure_servo/
├── README.md
├── mission_s155.py
└── verdict.py
```

Reuses (no `crtp_log.py` import!):
- `hex_helpers.py` from s142_hex_descriptor_patrol

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 8 fired |
| G3 lap1_stores | ≥ 3 |
| G4 lap2_matches | ≥ 3 |
| G5 matches_ok | ≥ 2/3 |
| G6 closure_xy | < 12 cm (relaxed for 2-lap) |
| G7 errors empty |  |
| G8 servo_actions_ok | ≥ 7 |
| G9 servo_faults_oob | == 0 |
| G10 journal events all fired |  |

## How to run

```bash
distrobox enter crazysim-garden -- bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &
until ss -lun | grep -q ":19850"; do sleep 2; done

cp examples/sentai_runtime/experiments/s142_hex_descriptor_patrol/hex_helpers.py \
   build-sim/sentai_fs_root/
cp examples/sentai_runtime/experiments/s155_loop_closure_servo/mission_s155.py \
   build-sim/sentai_fs_root/

echo "import mission_s155; r = mission_s155.run(); print('FINAL:', r['status'])" \
    | ./build-sim/sim/sentai_sim

python3 examples/sentai_runtime/experiments/s155_loop_closure_servo/verdict.py
```
