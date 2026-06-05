---
name: No complex orchestration over the serial REPL
description: User dislikes host-side scripts that push parameters/control flow over /dev/ttyACM0 — bench drivers must be self-contained on-board files that write CSV to flash, downloaded only at the end
type: feedback
originSessionId: f158eff9-f718-4f1f-82fc-48d9e0e77747
---
User does not want host orchestrators that drive complex test logic over
the serial REPL (push driver, set REPL globals, exec, read stdout, branch
on output, etc.).  Multi-step REPL orchestration is fragile and the user
finds it hard to follow.

**Why:** It's brittle — first-boot prompt races dropped seed-globals on
the iarna p3p4 pipeline run (2026-04-28), MSBlock initially failed with
`NameError: _target_model isn't defined` because seed lines were eaten
during a re-enum prompt-handshake race.  Beyond the bug, the user simply
prefers seeing the experiment as a self-contained artifact on the board
the way agent.md describes: write a driver, push it once, exec, board
writes results, host downloads at the end.

**How to apply:** When designing a new bench:
- Put the entire experiment loop (parameter sweeps, model lists, ratios,
  fps levels, etc.) INSIDE the on-board Python driver under
  `examples/sentai_runtime/diag/_t_*.py`.
- Driver writes incremental CSV to `/diags/sNNN_<name>/results.csv`.
- Host's only roles: (1) push the driver via `_host_upload_repl.py`,
  (2) `exec(sentai.fs.read_str(...))` ONCE, (3) wait for `=== done ===`,
  (4) pull the CSV via `/tmp/pull_csv.py` or HTTP `/api/raw`.
- Per-test isolation (reflash --ram between models) belongs IN the
  driver via `sentai.sys.reset()` checkpoints, NOT in a host orchestrator
  that flashes between iterations.  If reset-between-iterations is
  unavoidable, write a single short host wrapper that ONLY
  reflashes + execs + reads, no per-test parameter passing.
- Do NOT seed globals via `send_line(s, "_target_x = ...")`.  Hardcode
  the parameter list inside the driver.

This preference applies to ALL bench/diag work on this board, not just
the iarna pipeline run that surfaced it.
