---
name: sim-log-triage
description: Decision tree for "where do I look first when a SIM experiment fails?". Inventories the 5 log sources (Gazebo, cf2, sentai_sim, mission journal, GT recorder) and maps symptoms to the most informative source. Use when a SIM run fails and root cause is unclear.
---

# /sim-log-triage

A SIM experiment fails.  There are FIVE concurrent log sources, each showing a slice of reality.  Reading all of them in the wrong order = "30 minutes to find that gz was never even running".  This skill is the decision tree.

## The 5 log sources

| # | Source | Where | Best for |
|---|---|---|---|
| 1 | **Gazebo server stdout** | wherever you launched `gz sim` (stdout) | World loaded? Plugins loaded? Topics advertised? |
| 2 | **cf2 SITL stdout** | wherever you launched cf2 (stdout, needs `stdbuf -oL`) | Did cf2 spawn? EKF converging? `SUP: Locked`? |
| 3 | **sentai_sim stdout** | wherever you launched `./build-sim/sentai_sim` | MP REPL output, mission prints, anti-cheat trip, UDS connect failure |
| 4 | **Mission journal** | `experiments/sNNN_*/<iter>/journal.txt` | Per-tick mission state (detections, control, abort triggers) |
| 5 | **GT recorder JSONL** | `experiments/sNNN_*/<iter>/cf2_gt.jsonl` | Ground-truth poses; gaps reveal Gazebo crashes mid-run |

Plus the post-mortem verdict output (`verdict.py` stdout), which references the others.

## Decision tree by symptom

### Symptom A: "Mission didn't even start"

Look in **(1) Gazebo stdout** first:
- Empty stdout / red errors → gz didn't load the world.  Check SDF syntax (`gz sdf -k <world>`).
- `[Err] [Plugin.cc] Failed to load plugin` → plugin handshake failed (see `[[crazysim-debug]]` step 5).
- `world.Load()` succeeded but no topics → render starvation (Garden bug; needs Xvfb workaround per `[[crazysim-debug]]`).

Then **(2) cf2 stdout**:
- No output at all → cf2 binary didn't start; check distrobox / launch script.
- `SUP: Locked, reboot required` early → EKF diverged on init; restart cf2.
- Stuck at "spawning" → cf2 trying to load a model that doesn't exist in the world.

Then **(3) sentai_sim stdout**:
- `bind: Address already in use` on UDS → orphan socket from prior run (`rm /tmp/sentai_*.sock`).
- `anti-cheat: rejecting topic /world/.../dynamic_pose/info` → bridge correctly rejected; world has the cheat plugin.
- Hung at first import → check `sentai_fs_root/` for mission file presence.

### Symptom B: "Mission started but aborted early"

Look in **(4) Mission journal** first:
- Last journal line before abort = the reason.  Common: `n_dets<4 for 30 consecutive frames → ABORT_LAND` per `[[flowbaseline2-4markers-abort]]`.
- No journal file → mission crashed before opening journal; go to (3).

Then **(3) sentai_sim stdout**:
- MP exception traceback → bug in mission script.
- Camera bridge `recv timeout` → bridge died; go to (1).

Then **(2) cf2 stdout** for control failures:
- `crtp.appchannel.tx.dropped` count rising → radio bridge saturated.
- `MOTOR DRIVER FAULT` → emergency motor stop (cf2 fw safety).

### Symptom C: "Mission ran but drift unexpected"

Look in **GT JSONL (5)** + **mission journal (4)** SIDE BY SIDE:
- Time-sync between them (UNIX ts) reveals when control diverged from GT.
- GT JSONL gaps → Gazebo froze (often Garden render-thread starvation).
- Journal "detections OK" + GT shows drift → control loop bias (cam extrinsics / flow sign / MARKER_WORLD mismatch).  See `[[flow-convention-audit]]` and `[[gz-world-edit]]` for the audit recipes.

Then run the **verdict script** to quantify which axis (X/Y/Z) and which iteration is the outlier.

### Symptom D: "It worked once but now doesn't"

Usually environmental, not code.  Check in order:
1. Orphan gz topics → `pkill -9 -f 'gz-sim|gz-tools|sentai_sim'; sleep 1`.
2. Stale UDS → `rm /tmp/sentai_*.sock`.
3. Texture cache (if you replaced an image) → `rm -rf ~/.gz/materials/scripts/cache`.
4. Bridge crashed silently → `ps -ef | grep gz_to_uds_bridge`.
5. Xvfb died → check `ps -ef | grep Xvfb` and `DISPLAY` env var.

### Symptom E: "Verdict numbers nonsensical (NaN, wildly off)"

Look in **(4) journal** for the raw frames the verdict consumed:
- Time-matching bug (e.g. `target = t0_gt + rel_s` instead of absolute wall clock — bit us yesterday).  Pair journal tick `ts_ms` with GT JSONL `time` via UNIX absolute timestamps, not relative.
- MARKER_WORLD typo → check `[[gz-world-edit]]` step 4 audit.

## Quick "first 30 seconds" inventory

When something fails, RUN THIS FIRST before opening any log:

```bash
# Are the expected processes alive?
ps -ef | grep -E 'gz-sim|cf2|sentai_sim|gz_to_uds_bridge|gt_recorder' | grep -v grep

# Are sockets in place?
ls -la /tmp/sentai_*.sock 2>&1

# Did the experiment produce its expected outputs?
ls -la experiments/sNNN_*/<iter>/ 2>&1

# Are gz topics flowing? (only if gz is alive)
timeout 2 gz topic -l 2>&1 | head -10
```

This 5-second inventory often tells you which of the 5 logs to open.

## Hard rules

- **Always `stdbuf -oL` on cf2** — otherwise stdout buffers and you see nothing for ~30 s.
- **Always `gz sim -v 4`** when diagnosing — `-v 3` hides plugin load errors.
- **Time-match in absolute UNIX timestamps** when correlating journal vs GT; don't normalize to zero.
- **One log at a time** — don't open all 5 simultaneously; the decision tree picks ONE per symptom.

## What NOT to do

- Open `verdict.py` plots first and try to reverse-engineer cause from a curve.  Verdicts are post-mortem; the cause is in the logs.
- Re-run the experiment hoping it works this time.  If you don't know WHY it failed, the re-run is a coin flip.
- Skip the 30-second inventory and dive straight into reading 2000-line gz logs.

## See also

- `[[crazysim-debug]]` skill — pre-flight + recovery recipes
- `[[gz-world-edit]]` skill — when symptom traces to a recent world edit
- `[[flow-convention-audit]]` skill — when symptom is drift in a specific axis
- `[[sim-runner]]` agent — orchestrates + filters logs at run time
