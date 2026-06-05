---
name: s145-mission-template-shipped
description: "Canonical MP mission migration template shipped 2026-05-16. mission_template.py drives cf2 SITL end-to-end via sentai.crazy.* (init/arm/takeoff/go_to/land/disarm) with sentai.sim.journal_* event capture + JSON-ish summary. Offline smoke 6/6 gates PASS. The reference shape for all future MP-only missions (s146+) per [[missions-run-in-sentai-only]]."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16** (Task #40).

## The pattern

```
build-sim/sentai_fs_root/mission_<name>.py:
    import sentai
    def run():
        sentai.sim.journal_open("mission_<name>_journal.txt")
        summary = {"status": "STARTED", ...}
        try:
            # phases here, each writing journal events
            summary["status"] = "DONE"
        except Exception as e:
            summary["status"] = "EXCEPTION"
            summary["errors"].append(str(e))
        finally:
            sentai.fs.write("mission_<name>_summary.json", _ser_val(summary))
            sentai.sim.journal_close()
            sentai.crazy.stop()
        return summary
```

Host: `echo "import mission_<name>; mission_<name>.run()" | sentai_sim`,
then read summary.json and run `verdict.py`.

## Files

| File | Role |
|---|---|
| `mission_template.py` | canonical MP file — 6 phases (init/arm/takeoff/waypoints/land/disarm), out-and-back trajectory, JSON-ish serializer |
| `t_template_offline.py` | SIM-only smoke (no Gazebo) — 6/6 PASS, validates structure |
| `verdict.py` | 6 hard gates: terminal status, phase set, waypoint count, crazy_init_rc, journal events, no errors |
| `run.sh` | `--offline` (smoke) + `--live` (Gazebo+SITL) modes |
| `README.md` | pattern doc + migration roadmap (s146-s150) |

## Gap explicitly deferred — pose feedback (G7)

Verdict.py has 6 gates active.  The 7th gate
(`land_xy_err < 10 cm` per `[[sim-test-must-return-home]]`) is
documented but inactive — the template has no pose feedback.  Closing
this gap is the prerequisite for Task #41 (s136 migration):

- Option A: `sentai.crazy.pose()` C wrapper doing CRTP LOG block
  subscription internally (~150-200 LoC in `sentai_crazy_sim.cc`).
- Option B: MP-side CRTP LOG dance via `send_crtp` + `recv_crtp`
  primitives that Task #39 already shipped (~80-100 LoC pure MP).
- Option C: tap cf2 pose via existing Gazebo camera bridge socket
  (already has /tf access?).

Decision deferred to start of Task #41 with operator.

## What this validates

| Claim | Method |
|---|---|
| MP file can drive cf2 SITL via UDP CRTP | offline smoke: all 8 commands send 0 or -3 (kernel-level rc, no orchestration error) |
| `try/finally` cleanup writes artifacts even on failure | structural pattern + smoke runs the full path |
| Single-line `import` REPL launch works | `echo "import ... ; .run()" \| sentai_sim` exits cleanly |
| Host script is observer-only | run.sh has 0 lines of mission logic; everything is `cp + echo \| sentai_sim + verdict` |

## Anti-pattern this template avoids

Old s136 mission_explore.py (host) was 630 LoC of cflib telemetry
observer + REPL driver + ReplDriver(step-by-step orchestration).
Template + verdict total: 240 LoC, with control logic ENTIRELY inside
the MP file.  Migration target is 100× reduction in host complexity
per mission.

## Reproduction

```bash
bash examples/sentai_runtime/experiments/s145_mission_template/run.sh --offline
# Exit 0 + "[verdict] PASS — all gates green" on stdout.
```

## Related

- `[[missions-run-in-sentai-only]]` — the rule this template enforces
- `[[sim-repl-test-recipe]]` — single-line import REPL pattern
- `[[sim-test-must-return-home]]` — closure rule (G7 when pose API lands)
- `[[test-must-be-relevant-to-claim]]` — gates check behavior, not FSM shape
- `[[sentai-sim-journal]]` — journal API used for event capture
- Task #39 — `sentai.crazy.*` API consumed by the template
- Sim.md §10 rule 0 — canonical doc
