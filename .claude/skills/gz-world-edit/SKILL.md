---
name: gz-world-edit
description: Workflow for editing a Gazebo world — marker positions, textures, lights, plugins. Codifies the gotchas from 2026-05-20 (marker dedup across SDF + mission + verdict + GT env, Gazebo texture cache, anti-cheat plugin re-introduction). Use BEFORE any world SDF edit and AFTER any texture replacement.
---

# /gz-world-edit

Editing a Gazebo world is deceptively expensive — yesterday cost hours because each SDF marker-position change required updating THREE other places (mission constants, verdict constants, GT recorder env vars), and replacing a ground texture didn't take effect because Gazebo caches them.

## The 4-place marker-config dedup (load-bearing)

When a marker moves in the world, FOUR files must agree:

| File | What | Why it must match |
|---|---|---|
| `sim/gazebo/<world>.sdf` (or CrazySim copy) | `<pose>` of each marker model | Source of truth for Gazebo physics + render |
| `experiments/sNNN_*/mission_sNNN.py` | `MARKER_WORLD = {...}` constant | Mission's expected positions for control loop |
| `experiments/sNNN_*/verdict.py` | `MARKER_WORLD = {...}` constant | Host-side GT comparator |
| `run.sh` (or shell env) | `GT_RECORDER_MODEL=<name>` for each marker | GT recorder subscribes to specific model topics |

Mismatch → the mission expects marker at (0, 0.16) but Gazebo shows it at (0, -0.16), and the verdict measures a "drift" that's actually a config typo.

**This duplication is technical debt** — flagged in auto-mem `[[op-s10-w19-markers-plan]]` as "Marker config duplication centralization" (open task).  Until that lands, the discipline is: edit one → grep all four → edit each.

## Edit workflow

### Step 1 — Kill gz before editing

```bash
pkill -9 -f 'gz-sim|gz-tools|sentai_sim|gt_recorder|cf2' || true
sleep 1
```

Editing the SDF while Gazebo is running ≠ taking effect.  Always kill first.

### Step 2 — Edit the world SDF

Identify the file you're editing:
```bash
# Project worlds
ls /home/bogdan/work/coralmicro/sim/gazebo/*.sdf

# CrazySim copies (USED at runtime by cf2 SITL)
ls /home/bogdan/work/crazyflie/CrazySim/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/
```

CrazySim has its OWN copy of the world.  Yesterday's lesson: edit BOTH if both are present.

### Step 3 — Texture / image replacement → flush Gazebo cache

If you replaced any image file (ground texture, marker pattern PNG, sky cubemap):

```bash
# Gazebo materials cache
rm -rf ~/.gz/materials/scripts/cache
rm -rf ~/.gz/rendering/

# Or, more conservatively, just remove the cached version of the file you changed:
find ~/.gz -name '<your_texture_basename>*' -delete
```

Without this, Gazebo serves the old cached pixels even though the source file is new.  Symptom yesterday: replaced the rug texture, but PiP still showed old colors → 20 minutes of "is the bridge serving stale frames?" investigation.

### Step 4 — Synchronize MARKER_WORLD in mission + verdict

```bash
# Find every MARKER_WORLD definition
grep -rn 'MARKER_WORLD\s*=' /home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/
# Edit each so the dict literally matches the SDF positions
```

Convention used in s183: marker name keys, `(x, y, z)` tuples in world-frame metres.

### Step 5 — Synchronize GT recorder model names

```bash
grep -n 'GT_RECORDER_MODEL\|GT_RECORDER_OUT' /home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/sNNN_*/run.sh
```

If you renamed a marker model (e.g. `marker_nw_v2`), the GT recorder env var must follow.

### Step 6 — Anti-cheat re-audit

```bash
bash /home/bogdan/work/coralmicro/sim/scripts/audit_anti_cheat.sh
grep -l 'gz-sim-odometry-publisher' <world_sdf_just_edited>
```

Catches the case where an edit-and-paste accidentally re-introduced the cf2 cheat plugin.  See `[[cf2-sitl-cheat-odom-gt]]`.

### Step 7 — PiP visual verify

Launch with GUI per `[[gazebo-gui-required]]`.  Inspect the camera PiP widget:
- Markers visible in expected positions
- Texture is the NEW texture (color / pattern matches what you replaced with)
- No black-screen (light still works)
- No upside-down camera (yaw / pitch unchanged)

Visual verify before running the mission — pixels don't lie, log timestamps do.

## Common failure modes (from 2026-05-20)

| Symptom | Likely cause | Fix |
|---|---|---|
| Mission expects marker, sees nothing | SDF edit missing in CrazySim copy | Sync both SDF files |
| Drift looks systematic in one axis | MARKER_WORLD typo in verdict vs SDF | grep + diff |
| Replaced texture not showing | Gazebo cache | Step 3 cache flush |
| cf2 EKF blows up at takeoff | Cheat plugin accidentally re-added | Step 6 audit |
| GT JSONL is empty | `GT_RECORDER_MODEL` mismatch after rename | Step 5 |
| Markers look right but mission still drifts | Camera mount offset still old in `set_cam_extrinsics(...)` | Audit `[[flow-convention-audit]]` |

## Reject patterns

- Editing the world while gz is running, expecting hot-reload.
- Replacing a texture and skipping cache flush ("it should just work").
- Editing the SDF but forgetting the CrazySim copy (or vice versa).
- Updating MARKER_WORLD in mission but not in verdict (or vice versa) → silent drift.
- Skipping anti-cheat audit "because I only moved one marker".

## See also

- `[[sentai-sim-air-gapped-from-truth]]` — what the anti-cheat audit guards
- `[[crazysim-debug]]` skill — pre-flight after world edits
- `[[anti-cheat-auditor]]` agent — automated audit
- `[[flow-convention-audit]]` skill — when the camera mount offset is involved
- Auto-mem `[[op-s10-w19-markers-plan]]` — pending dedup task
