# s152 — HexPatrol on sentai.servo paradigm

**Date**: 2026-05-16
**Status**: NEW (replaces s149 hex_patrol_mp pattern via Stage 4.A wiring)

## What this proves

After Tasks #44+#45+#47 closed the "missions-run-in-sentai-only" chapter, the
**canonical mission shape** is:

```python
sentai.servo.init(sentai.servo.CF2)   # or PX4 — backend-agnostic
sentai.servo.set_durations(2, 6, 2)
sentai.servo.arm()
sentai.servo.takeoff(0.75)
for wp in WAYPOINTS:
    sentai.servo.go_to(*wp)
    sentai.rtos.sleep_ms(int(MOVE_DUR*1000)+500)
    x, y, z, yaw = sentai.servo.pose()    # CRTP LOG in C, no Python protocol
    ... store places ...
sentai.servo.go_to(origin[0], origin[1], origin[2])
sentai.servo.land()
sentai.servo.disarm()
```

vs. the old s149 shape:

```python
sentai.crazy.init()
crtp_log.scan_toc(...)              # ~270 LoC Python protocol
bid = crtp_log.create_pose_block(...)
sentai.crazy.takeoff(...)
for wp in WAYPOINTS:
    sentai.crazy.go_to(*wp, ..., 0, 0, 0)
    crtp_log.poll(); pose = crtp_log.latest_pose()
    ...
sentai.crazy.land(...)
```

### What changed under the hood

| Concern | s149 (old) | s152 (new) |
|---|---|---|
| pose feedback | `crtp_log.py` MP, ~270 LoC | `sentai.crazy.pose_*` C (~430 LoC C, single TU) |
| backend selection | hardcoded `sentai.crazy.*` | runtime `servo.init(backend)` |
| MP heap | 256→512 KB doubled to fit `crtp_log._toc` | back to 256 KB headroom (no protocol dict) |
| portability cf2↔PX4 | rewrite required | flip the backend constant |

## Files

```
s152_hex_patrol_servo/
├── README.md
├── mission_s152.py       # uses sentai.servo only
└── verdict.py            # same 8 hard gates as s149
```

Reuses (no `crtp_log.py` import!):
- `hex_helpers.py` from s142 — synthetic descriptor pipeline

## Hard gates (same as s149)

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 9 fired |
| G3 places_stored | == 3 |
| G4 self_queries | all 3 OK |
| G5 closure_xy | < 10 cm |
| G6 servo_actions_ok | ≥ 12 (1 init + 1 arm + 1 takeoff + 3 wp + 1 home + 1 land + 1 disarm + ≥3 hover) |
| G7 servo_faults_oob | == 0 |
| G8 pose stream live | pose_ready == 1 at the end |

## How to run

```bash
# 1. SITL stack (background, idempotent)
distrobox enter crazysim-garden -- bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &
until ss -lun | grep -q ":19850"; do sleep 2; done

# 2. Stage mission helpers
cp examples/sentai_runtime/experiments/s142_hex_descriptor_patrol/hex_helpers.py \
   build-sim/sentai_fs_root/
cp examples/sentai_runtime/experiments/s152_hex_patrol_servo/mission_s152.py \
   build-sim/sentai_fs_root/

# 3. Run mission (single REPL line)
echo "import mission_s152; r = mission_s152.run(); print('FINAL:', r['status'])" \
    | ./build-sim/sim/sentai_sim

# 4. Verdict
python3 examples/sentai_runtime/experiments/s152_hex_patrol_servo/verdict.py
```

## Why this is the canonical pattern going forward

- Same `.py` will run on Coral Dev Board Micro once Stage 9 lands — only
  `sentai.servo.init(BACKEND_CF2)` flips to the UART CRTP transport.
- PX4 missions trivially derive: change `BACKEND_CF2` → `BACKEND_PX4`,
  rebuild SITL stack with PX4 instead of cf2.
- Drops the ~270 LoC `crtp_log.py` dependency from every mission.
