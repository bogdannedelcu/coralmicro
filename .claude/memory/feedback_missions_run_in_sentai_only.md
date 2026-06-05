---
name: missions-run-in-sentai-only
description: "HARD RULE — every flight mission and integration test runs entirely inside SentAI firmware (C++ in sentai_runtime OR MicroPython in sentai_fs_root). NEVER on host Python. Host scripts are pure stack-launcher + observer (start SIM, import mission once, capture journal/summary at end). Load-bearing for thesis claim \"autonomous drone on MCU\"."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-16** in plain terms:
*"Orice experiment îl rulăm fie în cod C++ în sentai fie în MicroPython sentai, niciodată pe host local. Sentai interacționează prin senzorii virtuali cu simulatorul gazebo."*

## The rule

Every mission, integration test, and closed-loop experiment runs
**inside the SentAI firmware** — either:
- **C++** in `examples/sentai_runtime/sentai_*.cc` (with MP binding for
  invocation, e.g. `sentai.explore.start()`), OR
- **MicroPython** as a `.py` file in `build-sim/sentai_fs_root/`
  imported once via the REPL.

**Never** on host Python.  Host scripts exist only as:
1. Stack launcher (start `gz sim`, `cf2 SITL`, `gz_to_uds_bridge`,
   `sentai_sim`).
2. REPL kick-off (single `import mission_sNNN` line).
3. Passive collector (read `summary.json`, `journal.txt`,
   `cf2_telemetry.json` AFTER mission ends).

## Why

Thesis claim is "**drone autonomous on MCU**".  If the mission logic
(state machine, decision making, waypoint sequencing, place memory,
loop closure) runs on x86 Python, the demonstration is "Python script
controls drone" — defeats the claim.

The mission must execute on the platform we're claiming runs it.

## Architectural implications

### Sentai interacts with Gazebo via virtual sensors

```
┌─────────────────────────────────────────────────────────────┐
│                       Gazebo simulator                         │
│  ┌───────────┐    ┌──────────────────┐    ┌──────────────┐  │
│  │ cf2 SITL  │    │ Downward camera   │    │  World/IMU   │  │
│  │ (UDP 19850│    │ (gz topic)        │    │              │  │
│  └─────┬─────┘    └────────┬─────────┘    └──────┬───────┘  │
└────────┼───────────────────┼──────────────────────┼─────────┘
         │                   │                      │
         │ MAVLink/CRTP      │ /tmp/sentai_cam.sock │ (via cf2 EKF)
         │ UDP               │                      │
         ▼                   ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│                     sentai_sim (firmware)                      │
│                                                               │
│  camera_bridge_recv   sentai.flow   sentai_link   sentai.*   │
│        │                   │             │              │     │
│        ▼                   ▼             ▼              ▼     │
│   pipeline / places / lifter / servo / explore / mission.py   │
│                                                               │
│  Mission orchestration HERE (MP file or C++ task)            │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ stdin (one-shot: import mission_sNNN)
                          │ stdout (passive log)
                          ▼
                  Host script (observer only)
```

### What goes where

| Layer | Belongs to |
|---|---|
| Mission state machine | sentai (C++ in `sentai_explore`, MP orchestrator file) |
| Per-frame camera processing | sentai (`sentai.camera`, `sentai_pxp_shim`) |
| Descriptor compute | sentai (`sentai_phog`, `sentai_gist`) |
| L3 place store/query | sentai (`sentai_places`) |
| L5 lifter | sentai (`sentai_object_lifter`) |
| Flow processing | sentai (`sentai.flow`) |
| Telemetry consumption | sentai (set_pose updates inside MP) |
| **cf2 setpoint emission** | **sentai** (`sentai.crazy.*` ARM, `sentai.link.*` SIM) |
| **Decision making, waypoint selection** | **sentai** |
| Gazebo bridge plumbing | host (gz_to_uds_bridge, cflib stack init) |
| SIM process launch | host (`launch_hybrid_cf2.sh`) |
| Mission file import (single line) | host (REPL stdin) |
| Post-mortem journal parse | host (after mission ends) |

### What was wrong before this rule

s136/s137/s138/s142/s143/s144 all had **mission orchestration in
Python host** (`mission_explore.py` 200-400 LoC each).  Host called
`cf.commander.send_position_setpoint(...)` directly, drove the FSM via
sequential REPL commands, etc.  These mission scripts must be migrated
to MP files in `sentai_fs_root/`.

The L6 FSM (`sentai_explore`) is already designed correctly — it lives
in firmware.  The wrapper that orchestrates "store at home, goto t1,
INSPECT, store t1, goto t2, ..." needs to move from host to MP.

## How to apply going forward

### Template for any new mission `sNNN`

1. Mission logic in MP file: `build-sim/sentai_fs_root/mission_sNNN.py`
   - Imports `sentai.*`
   - Defines a top-level `run()` function
   - Uses `sentai.sim.journal_*` for events
   - Returns dict with summary metrics
2. Host script: minimal — start stack, drop file, send REPL line:
   ```python
   repl.exec_int("(__import__('mission_sNNN'), 0)[1]")
   result_json = repl.exec_repr("mission_sNNN.run()")
   ```
3. Host writes `summary.json` from `result_json`.

### What sentai needs (gaps to fill before this rule is fully workable)

- **cf2 setpoint MP API on SIM** — `sentai.crazy.send_position(x, y, z, yaw)` already exists on ARM (CRTP); a SIM equivalent (UDP MAVLink to 19850 or via existing `sentai_link`) is the gap.  See `sentai_link_sim.cc` — it already does MAVLink heartbeat/repl; needs `send_set_position_target_local_ned`.
- **Telemetry stream into sentai** — flow_forwarder thread currently lives in host and pushes via cflib.  Sentai should pull cf2 stateEstimate itself.
- **MP top-level event loop** — current MP missions are blocking; if state-machine needs concurrent telemetry + setpoint, may need a non-blocking pattern.

These gaps will be tracked in objects_plan.md or a dedicated migration
section.

## Related

- `[[sim-test-must-return-home]]` — closure rule (unchanged)
- `[[test-must-be-relevant-to-claim]]` — relevance rule (unchanged)
- `[[sentai-sim-journal]]` — used as the event log medium
- `[[sim-repl-test-recipe]]` — single `import` REPL pattern
- `[[experiments-start-from-origin]]` — applies (cf2 respawn)
- Thesis-MVP §23.1 — demo requires this architecture
