# s145 — Mission migration template (MP-only architecture)

**Date**: 2026-05-16
**Status**: TEMPLATE shipped (smoke PASS).  Live Gazebo run is the
            consumer of this template (s146+ missions).

## Purpose

Canonical pattern for ANY new flight mission, per the firm rule
`[[missions-run-in-sentai-only]]` (Sim.md §10 rule 0): mission
orchestration MUST run inside the SentAI firmware (MP file in
`sentai_fs_root/`), NEVER in host Python.

Demonstrates the minimal viable end-to-end flow using the
`sentai.crazy.*` API shipped in Task #39:

```
init UDP → arm → takeoff → go_to waypoints → land → disarm → stop
        + sentai.sim.journal_* events at every phase
        + summary.json written for host post-mortem
```

## What this is + what this is NOT

| | This template | s136 (real mission) |
|---|---|---|
| Drone control | sentai.crazy.* via CRTP-UDP | cf.commander via cflib (host) |
| Mission FSM | inline phases in MP | sentai.explore L6 FSM |
| Pose feedback | NO (uses cf2 onboard trajectory) | YES (cf2 stateEstimate → set_pose) |
| Camera / ArUco | NO | YES |
| Pure-sentai | YES | NO (orchestration on host) |

This template proves the WIRE-UP — that an MP file can drive cf2 SITL
end-to-end without any host Python in the control loop.  Pose
feedback (needed for closed-loop missions like s136) is the next gap
to fill in Task #41.

## Files

```
s145_mission_template/
├── README.md                  # this file
├── mission_template.py        # canonical MP mission (copy to sentai_fs_root/)
├── t_template_offline.py      # SIM-only smoke (no SITL needed)
├── run.sh                     # host launcher (live cf2 SITL run)
└── verdict.py                 # parses summary.json, gates the run
```

## The pattern

### 1. MP file: `mission_<name>.py`

```python
import sentai

def run():
    sentai.sim.journal_open("mission_<name>_journal.txt")
    summary = {"status": "STARTED", "phases": []}
    try:
        # ... phases, each writing to journal ...
        summary["status"] = "DONE"
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["error"] = str(e)
    finally:
        # always: write summary.json + close journal + stop transport
        sentai.fs.write("mission_<name>_summary.json", _serialize(summary))
        sentai.sim.journal_close()
        sentai.crazy.stop()
    return summary
```

Why a `try/finally` wrapper: a mid-mission exception (e.g. cf2 doesn't
arm) must still write a summary + close journal so the host's verdict
sees the partial state.  Otherwise the host hangs waiting for files.

### 2. Host launcher: `run.sh`

```bash
# 1. bring up SITL stack (Gazebo + cf2 SITL + bridge)
bash sim/scripts/launch_hybrid_cf2.sh sentai_crazysim &

# 2. wait for cf2 UDP 19850 + sentai_cam.sock + gz_to_uds_bridge

# 3. copy mission file into the SIM virtual FS root
cp mission_<name>.py build-sim/sentai_fs_root/

# 4. invoke mission via sentai_sim REPL (single line, single import)
echo "import mission_<name>; mission_<name>.run(); sentai.sys.reset()" \
    | ./build-sim/sim/sentai_sim > stdout.log 2>&1

# 5. read summary written by the mission
cat build-sim/sentai_fs_root/mission_<name>_summary.json

# 6. verdict
python3 verdict.py
```

That's it.  The host's job is **launcher + observer**, not commander.

### 3. Verdict: `verdict.py`

Reads `mission_<name>_summary.json` + `mission_<name>_journal.txt`,
applies HARD GATES per `[[test-must-be-relevant-to-claim]]` +
`[[sim-test-must-return-home]]`:

- `summary["status"] == "DONE"`
- `summary["phases"]` includes every expected milestone
- closure_xy < 10 cm (when telemetry available — pose feedback work)

## Two universal rules apply

1. `[[sim-test-must-return-home]]` — every mission MUST end with the
   drone within 10 cm of its PHYSICAL takeoff origin.  Template does
   this by ending waypoints at (0, 0, takeoff_height).
2. `[[test-must-be-relevant-to-claim]]` — verdict gates the BEHAVIOR,
   not the FSM shape.  Template asserts displacement > 0 + closure
   < 10 cm.

## Reusing this template

Don't blindly copy — read it, understand the shape, then write your
mission against the SAME shape.  The template is intentionally tiny so
you can hold the whole pattern in your head.

Migrations using this template (planned):
- s146: s136 explore-real migrated (needs pose feedback first)
- s147: s137 explore-long migrated
- s148: s138 lost-recovery migrated
- s149: s142 hex-patrol migrated
- s150: s143 loop-closure migrated

## Related

- `[[missions-run-in-sentai-only]]` — the rule this template enforces
- Sim.md §10 rule 0 — same rule in canonical doc form
- `[[sim-repl-test-recipe]]` — single-line import pattern
- `[[sim-test-must-return-home]]` — closure gate
- Task #39 — `sentai.crazy.*` API this template consumes
