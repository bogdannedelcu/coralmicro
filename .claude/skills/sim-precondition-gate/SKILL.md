---
name: sim-precondition-gate
description: Verify SIM world / SDF / config preconditions before running a canonical experiment that depends on a specific state (e.g. SDF noise levels, marker positions, cheat plugin disabled, IMU/baro noise restored). Use BEFORE any FlowBaseline gate run, canonical regression test, or any experiment whose published number depends on world state.
---

# /sim-precondition-gate

A SIM experiment's measured number is meaningful ONLY if the world state matches the canonical config under which the number was first established.  Multiple times in this project, an unrelated session zeroed the IMU/baro/gyro noise (to make a different experiment converge), the next day's gate run on FlowBaseline silently got a different number, and nobody noticed for hours.

This skill is the pre-flight check that gates an experiment on its declared preconditions.

## When to run

- Before any `FlowBaseline` canonical regression (s127 family).
- Before any thesis-quality "shipped" measurement where the number will be cited.
- After ANY session that modified vendor SDFs (e.g. CrazySim model.sdf.jinja, the world SDF).
- After ANY session that toggled the cheat plugin (per `OP-S8-W1`).

## The canonical preconditions (as of 2026-05-21)

| Asset | Canonical state | How to verify |
|---|---|---|
| **cf2 IMU noise** | gyro σ = 0.0035 rad/s, accel σ = 0.05 m/s² (vendor defaults restored) | `grep -E 'gyro_bias\|accel_noise\|baro' /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf*` |
| **cf2 baro noise** | σ = 0.01 m | same grep |
| **Cheat plugin disabled** | `gz-sim-odometry-publisher-system` NOT in model.sdf.jinja | `grep -L 'gz-sim-odometry-publisher' <model.sdf.jinja>` (empty = good) |
| **Wind** | NONE (operator-stated [[flowbaseline-canonical-config]] is no-wind) | `grep -c 'WindEffects' <world.sdf>` should be 0 |
| **Marker positions in world SDF** | match `MARKER_WORLD` dict in mission + verdict | per `[[gz-world-edit]]` skill — diff SDF poses vs mission constants |
| **Anti-cheat audit passes** | `audit_anti_cheat.sh` exit 0 | `bash /home/bogdan/work/coralmicro/sim/scripts/audit_anti_cheat.sh` |
| **No `*.pre_*` orphan backups in active path** | If `.pre_no_cheat_20260517` exists, current file may or may not be the patched version — verify | `find <world_path_parent> -name '*.pre_*' -mtime -7` |

## Per-experiment precondition file (convention)

Each experiment that has non-trivial preconditions should ship a `preconditions.yaml` (or `.json`) at the folder root listing what it expects.  Example for `s127_flowbaseline_canonical/`:

```yaml
preconditions:
  cf2_imu_noise: vendor_default          # gyro σ=0.0035, accel σ=0.05
  cf2_baro_noise: 0.01
  cheat_plugin: disabled
  wind: none
  world_sdf: sim/gazebo/sentai_flow_canonical.sdf
  marker_world:
    # must match mission_s127.py and verdict.py MARKER_WORLD dict
    nw: [-0.16, 0.16, 0]
    ne: [0.16, 0.16, 0]
    sw: [-0.16, -0.16, 0]
    se: [0.16, -0.16, 0]
```

If this file exists, the gate script reads it and walks each check.  If missing, fall back to the canonical table above.

## Recipe

```bash
EXP_DIR=/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/sNNN_<name>

# 1. Anti-cheat audit (mandatory)
bash /home/bogdan/work/coralmicro/sim/scripts/audit_anti_cheat.sh
[ $? -ne 0 ] && { echo "ANTI-CHEAT FAIL — fix before gating"; exit 1; }

# 2. IMU / baro noise vendor-default
NOISE=$(grep -E 'gyro_bias_stddev|accel_noise_stddev|baro.*stddev' \
    /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja)
echo "$NOISE"
# Expect: gyro σ=0.0035, accel σ=0.05, baro σ=0.01
# If you see ZEROS: previous W14 autotune session didn't restore — RESTORE before gating

# 3. Cheat plugin absent
grep -l 'gz-sim-odometry-publisher' \
    /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/*.sdf*
# Empty output = good.  Any match = restore before gating.

# 4. Wind plugin absent (if FlowBaseline)
grep -c 'WindEffects' <world.sdf>
# Should be 0.

# 5. Marker positions consistency
# Per /gz-world-edit step 4 — grep MARKER_WORLD in mission + verdict, diff vs SDF.

# 6. .pre_* orphan backups warning
find /home/bogdan/work/crazyflie/CrazySim/CrazySim/.../models/ -name '*.pre_*' -mtime -7
# Recent backups = someone patched something; verify the live file is the intended version.
```

## Recovery — restore vendor SDF noise

If the IMU/baro noise is zeroed (W14 autotune state), restore the canonical:

```bash
cd /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/

# Look for a backup the previous session made
ls *.pre_*
# Common backup pattern: model.sdf.jinja.pre_zero_noise_<date>

# Or restore from git history (the CrazySim repo)
cd /home/bogdan/work/crazyflie/CrazySim
git log --oneline -- crazyflie-firmware/tools/.../model.sdf.jinja | head -5
git checkout <known-good-commit> -- crazyflie-firmware/tools/.../model.sdf.jinja
```

After restoring, RE-RUN the gate check from step 1.

## Reject patterns

- Running a canonical regression without checking preconditions ("it worked yesterday").  World state drifts silently across sessions.
- Skipping the noise check because the gate "passed" — a noise-zeroed gate passes by accident; restore noise THEN gate.
- Editing vendor SDF without leaving a `.pre_<reason>_<date>` backup and a memory entry — future sessions can't tell what's intentional.
- Quoting numbers from a gate run that didn't precede with this check.

## See also

- `[[flowbaseline-canonical-config]]` auto-memory — declares the canonical state
- `[[op-s8-w1-cf2-sim-honest]]` auto-memory — the cheat-plugin crisis context
- `[[gz-world-edit]]` skill — when restoring requires editing
- `[[anti-cheat-auditor]]` agent — automated audit (this skill invokes the same script)
- `[[sim-runner]]` agent — should call this skill before launching gate-class experiments
