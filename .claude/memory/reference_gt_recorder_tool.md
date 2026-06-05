---
name: gt-recorder-tool
description: "Canonical Gazebo GT pose recorder for SIM experiments — sim/scripts/gt_recorder.py.  Subscribes to /world/<W>/dynamic_pose/info via `distrobox enter crazysim-garden -- gz topic -e`, parses protobuf-text dump, writes JSONL with host monotonic + sim time per record.  Env vars GT_RECORDER_{OUT,WORLD,MODEL,DISTROBOX} (CLI flags override).  Anti-cheat: host-side post-mortem ONLY, never injected back into cf2 / sentai_sim.  Operator-promoted 2026-05-18 from per-experiment copy in s165/s166 to canonical path; all future experiments use this one."
metadata: 
  node_type: memory
  type: reference
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## Path

`sim/scripts/gt_recorder.py` — canonical, importable by all future GT-aware
experiments.  Self-contained Python script, no deps beyond stdlib.

## What it does

Spawns `distrobox enter crazysim-garden -- gz topic -e -t /world/<W>/dynamic_pose/info`
and parses the protobuf-text dump for one model's `position { x y z }`
block.  Writes one JSON-lines record per pose update, line-buffered so
data is safe even if the process is killed mid-flight.

Record shape (one line each):
```json
{"t_wall": <host monotonic float>,
 "t_unix": <host wall-clock float>,
 "gz_sec": <int>, "gz_nsec": <int>,
 "x": <float>, "y": <float>, "z": <float>}
```

Typical cadence ~200 Hz (GZ Garden dynamic_pose default).

## Calling convention

CLI flags take precedence over env vars:
```bash
python3 sim/scripts/gt_recorder.py [--world W] [--model M] [--out PATH] [--distrobox NAME]
```

Env (canonical):
- `GT_RECORDER_WORLD`       default `sentai_crazysim`
- `GT_RECORDER_MODEL`       default `crazyflie_0`
- `GT_RECORDER_OUT`         default `/tmp/gt_recorder/gt_poses.jsonl`
- `GT_RECORDER_DISTROBOX`   default `crazysim-garden`

Backwards-compat aliases recognised (lower priority):
`S165_GT_*`, `S166_GT_*`, `S167_GT_*` → `GT_RECORDER_*`.

## Lifecycle gotcha — orphan gz topic

`distrobox enter` → `podman exec` does NOT always forward SIGTERM to the
in-container `gz topic` process.  Killing the Python wrapper alone leaves
an orphan that holds the topic open and prevents respawn.  Orchestrator
MUST follow up with:

```bash
distrobox enter crazysim-garden -- pkill -9 -f \
    "gz topic -e -t /world/<world>/dynamic_pose"
```

Reference implementation in `s167_flowbaseline_calibrated/run.sh::stop_gt_recorder`.

## Anti-cheat positioning

[[sentai-sim-air-gapped-from-truth]] + [[cf2-sitl-cheat-odom-gt]]: GT
JSONL is the *only* anti-cheat-compliant way to obtain Gazebo ground
truth in this project.  Consumers MUST use it host-side, at verdict
time, post-mortem — never read it from a process that writes into cf2
firmware (CRTP) or into sentai_sim (UDS sockets / MP runtime).

Audit cue: if a script reads `/tmp/<exp>/gt_poses.jsonl` AND opens a
cflib `Crazyflie()` link AND calls `cf.extpos.send_extpos()`, that is
the cheat path resurrected.  Block it.

## History

- 2026-05-17  First copy in `s165_square_drift_gt/gt_recorder.py`
  (OP-S10-W11-T5.A v2).  Used `S165_GT_*` env vars.
- 2026-05-18  Cloned into `s166_flowbaseline_gt/gt_recorder.py`
  (FlowBaseline post-cheat trial).  Used `S166_GT_*` env vars.
- 2026-05-18  Operator-promoted to canonical `sim/scripts/gt_recorder.py`.
  Generalised env names + added CLI args + kept legacy aliases.
  All future experiments use the canonical version; historical copies
  in s165/s166 are kept for reproducibility but should not be cloned.

## Likely future improvements (parking lot)

- Optional `--track-models <list>` to record multiple models in one JSONL
  (would change the record schema → add a `name` field).
- Add `--rate <hz>` decimation flag so verdicts don't drown in 200 Hz
  records when only ~10 Hz is needed for trajectory plots.
- Detect missing topic at startup (currently waits silently) and exit
  non-zero with a clear error.
- Optional binary protobuf path via `gz topic -e --json-output` (faster
  parse, less line noise).
