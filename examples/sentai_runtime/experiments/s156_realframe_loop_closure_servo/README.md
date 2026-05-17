# s156 — Real-frame loop closure on sentai.servo paradigm

**Date**: 2026-05-17
**Status**: NEW — servo-paradigm port of s151 (MP-only port of s144)

Migrates s151 from `sentai.crazy.* + crtp_log.py` to `sentai.servo.*` per
[[next-steps-2026-05-17]] A1.  Same 2-lap loop closure pattern as s155
but descriptors come from REAL Gazebo frames via `sentai.camera.grab_gray()`.

## Files

```
s156_realframe_loop_closure_servo/
├── README.md
├── mission_s156.py
└── verdict.py
```

Reuses: `hex_helpers.py` from s142 (`real_capture_and_store`, `real_query_at`).

## Prerequisites

Camera bridge must be UP: `gz_to_uds_bridge` running inside crazysim-garden
distrobox, feeding `/downward_cam/image` from Gazebo into
`/tmp/sentai_cam.sock`.  Without it, `sentai.camera.grab_gray()` returns
None and mission reports `FAIL_NO_CAMERA`.

**Status 2026-05-17**: `run.sh` is a stub — the bridge–sentai_sim
handshake needs the s127-style fifo-stdin pipeline (sentai_sim must be
alive *before* the bridge connects, since the bridge bails out on
SIGPIPE when sentai_sim exits).  A short pipe of `import mission_s156`
won't satisfy this.  Proper integration into a follow-up s127-style
runner is TODO.  The mission code itself is a straight servo-paradigm
port of s151 and is correct — the gap is in the test harness, not the
mission.

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 9 fired |
| G3 lap1_real_stores | ≥ 3 |
| G4 lap1 pid valid | all ≥ 0 |
| G5 lap2_real_matches | ≥ 3 |
| G6 matches_ok | ≥ 1/3 (real-frame noise budget) |
| G7 closure_xy | < 12 cm |
| G8 errors empty |  |
| G9 servo_actions_ok | ≥ 7 |
| G10 servo_faults_oob | == 0 |
| G11 journal events all fired |  |

## How to run

```bash
# Camera bridge must be staged (s127 flowbaseline does it as a side-effect)
bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh setup

cp examples/sentai_runtime/experiments/s142_hex_descriptor_patrol/hex_helpers.py \
   build-sim/sentai_fs_root/
cp examples/sentai_runtime/experiments/s156_realframe_loop_closure_servo/mission_s156.py \
   build-sim/sentai_fs_root/

echo "import mission_s156; r = mission_s156.run(); print('FINAL:', r['status'])" \
    | ./build-sim/sim/sentai_sim

python3 examples/sentai_runtime/experiments/s156_realframe_loop_closure_servo/verdict.py
```
