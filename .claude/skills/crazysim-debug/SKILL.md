---
name: crazysim-debug
description: Pre-flight checklist + recovery recipes for CrazySim + Gazebo Garden 7.9 SIM sessions. Use when a SIM run silently fails (cf2 spawn hangs, gz topics empty, image buffer frozen, cf2 SUP locked) or before starting a new closed-loop experiment.
---

# /crazysim-debug

CrazySim + Garden 7.9 has a set of silent-failure modes that each cost hours to debug.  This skill codifies the pre-flight checklist and the recovery recipes.

## Pre-flight checklist (before launching a SIM experiment)

1. **Line-buffer cf2 stdout** so its log appears in real time, not when the pipe finally flushes:
   ```bash
   stdbuf -oL <cf2_launch_command>
   ```
   Without `stdbuf -oL`, cf2 stdout sits in a 4 KB stdio buffer and you see nothing until ~100 lines later — debugging "did cf2 spawn?" becomes "wait 30 s and hope".

2. **Bump gz verbosity** for an unknown failure:
   ```bash
   gz sim -g -v 4 --gui-config sim/gazebo/sentai_gui.config <world.sdf>
   ```
   `-v 4` shows plugin loading, topic advertise, sensor init.  `-v 3` (default) hides most of these.

3. **Verify gz-transport version match** between bridge and Gazebo:
   ```bash
   ldd $(which gz) | grep gz-transport
   # Inside distrobox: should be gz-transport12 (Garden 7.9)
   ```
   Mismatch → bridge sees zero topics even though `gz topic -l` lists them.

4. **Env isolation**: the canonical split is `cf2 + gz` inside distrobox, `sentai_sim + gz_to_uds_bridge` on host (per Sim.md §4).  Crossing the boundary (e.g. running bridge inside distrobox) breaks UDS file visibility.  Always check `/proc/$(pgrep gz_to_uds_bridge)/root/tmp/` matches host `/tmp/sentai_cam.sock`.

5. **Xvfb fix for any headless rendering**: Garden 7.9 with `--headless-rendering` freezes the image buffer (Gazebo issues #346 / #332 / #2708 / #2851).  Use Xvfb workaround:
   ```bash
   Xvfb :99 -screen 0 1024x768x24 &
   DISPLAY=:99 gz sim -s -r <world.sdf>
   ```
   Or simpler: use `-g` (GUI on) per `[[gazebo-gui-required]]` rule.

## Silent-failure diagnostics

### Symptom: gz topic publishing but image bit-identical across frames

```bash
gz topic -e -t /downward_cam/image --json-output | head -20
# Look at header.stamp — should progress
# Look at data md5 (mentally) — should change frame to frame
md5sum < <(gz topic -e -t /downward_cam/image --json-output | head -5)
```

Frozen pixel buffer = Garden render-thread starvation = needs Xvfb (see step 5).

### Symptom: cf2 spawned but `SUP: Locked, reboot required`

EKF diverged (takeoff with no flow + no extpos → velocity blew up).  Recovery is hard-kill + re-spawn:
```bash
# Kill cf2 inside distrobox
distrobox enter <name> -- pkill -9 cf2

# Re-spawn (or whatever your headless launcher is)
distrobox enter <name> -- bash dbox_launch_cf2_headless.sh
```
Don't try to "unlock" in software — restart the ELF.

### Symptom: anti-cheat plugin still active

The `gz-sim-odometry-publisher-system` plugin was the cf2 cheat (injects GT pose into EKF).  Verify removed from world:
```bash
grep -l 'gz-sim-odometry-publisher' <world_sdf>
grep -l 'gz-sim-odometry-publisher' /home/bogdan/work/crazyflie/CrazySim/.../model.sdf*
```
Empty = clean.  See `[[cf2-sitl-cheat-odom-gt]]` and `OP-S8-W1`.

### Symptom: plugin handshake silently failed

Plugin load errors don't fail Gazebo startup — they print once and the SIM continues with the plugin missing.  Always scan startup output:
```bash
gz sim -g -v 4 <world> 2>&1 | grep -iE 'plugin.*(fail|error|cannot|missing)'
```
Empty = all plugins loaded.

## Gazebo texture / material cache (load-bearing after image replacement)

Gazebo caches loaded textures across `gz sim` restarts.  If you replaced any image file (ground texture, marker PNG, sky cubemap), Gazebo will serve the OLD pixels on the next run.  Yesterday's cost: 20 min debugging "stale frames from the bridge" when the bug was actually a stale texture in `~/.gz/materials/scripts/cache`.

Recipe — flush the cache before relaunching after a texture change:
```bash
# Aggressive: clear ALL Gazebo material + rendering cache
rm -rf ~/.gz/materials/scripts/cache
rm -rf ~/.gz/rendering/

# Conservative: remove only the cached version of the file you changed
find ~/.gz -name '<your_texture_basename>*' -delete
```

Run BEFORE relaunching `gz sim`.  Verify the new texture is loaded via the GUI PiP widget (`[[gazebo-gui-required]]`).

## Cleanup

Orphan gz topics + bridge processes + UDS sockets accumulate across runs.  Always teardown completely:
```bash
pkill -9 -f 'gz-sim|gz-tools|gz_to_uds_bridge|sentai_sim|gt_recorder|cf2'
rm -f /tmp/sentai_cam.sock /tmp/sentai_*.sock
sleep 1
```

## Reject patterns

- Launching without `stdbuf -oL` on cf2 ("debug spirals" — 30 min wasted on "did it crash?").
- `-v 3` when nothing works ("it just doesn't run" — `-v 4` would have shown the plugin error in line 12).
- Trusting `--headless-rendering` ("it works on my machine" — frozen-buffer bug is Garden-version-specific).
- Skipping the anti-cheat audit before each run (`[[sentai-sim-air-gapped-from-truth]]` is not optional).

## See also

- `Sim.md` §10c (operator workflow rules — GUI + PiP + altitude ramp + SUP recovery)
- `Sim.md` §10d (Xvfb workaround in detail)
- `[[sim-launch]]` skill — the launch sequence this checklist guards
- `[[anti-cheat-auditor]]` agent — automated audit
