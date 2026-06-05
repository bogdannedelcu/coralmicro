---
name: sim-repl-test-recipe
description: "To run a Python test against sentai_sim REPL, write the .py to build-sim/sentai_fs_root/ (no leading underscore) and `import` from REPL. Paste mode + line-by-line defs don't work in the embed-port REPL."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: cfa3374f-a472-4f14-be65-3858146f8b62
---

The minimal MicroPython embed REPL in `sentai_sim` is a line-buffered
friendly REPL.  Multi-line `def` / `if` / `for` blocks fed via stdin
are parsed line-by-line, so the function body trips
`IndentationError`.  Paste mode (Ctrl-E … Ctrl-D) is NOT implemented.
`open(...)` is NOT exposed (no file builtin in this port).  Single-line
`exec("…")` wrappers break on `"` or escaped chars.

**Recipe that works**: drop the test as a real `.py` file under the
SIM virtual FS root (`build-sim/sentai_fs_root/`, override via
`SENTAI_SIM_ROOT` env), then `import` from REPL.  The `import`
mechanism uses `mp_lexer_new_from_file()` which parses the whole file
as one compilation unit, bypassing the line-buffered REPL parser.

**Why** (operator, 2026-05-13): I burned cycles trying paste mode and
escaped one-line `exec()` wrappers before the operator pointed out the
right path.  The recipe is now durable in `Sim.md §10w`.

**How to apply** (verified at L2):
```bash
# Drop test (drop leading _ — Python identifiers can't start with it
# in `import`, and on-board diag uses `_t_*` to mark uploader inclusion).
cp examples/sentai_runtime/diag/_t_NN_foo.py \
   build-sim/sentai_fs_root/tNN_foo.py

# Run via stdin import.
echo 'import tNN_foo' | ./build-sim/sim/sentai_sim
```

The driver test should `print()` PASS/FAIL lines as it goes — output
arrives in stdout from the SIM process.

Reuse this recipe for every SIM test from L2 onwards (s127 FlowBaseline,
diag/_t_*.py drivers, future L3+ smoke tests).

Related: [[no-broken-branch-test-reuse]] (drives test-from-scratch in
the first place), [[objectsplan]] (where the cadence keeps producing
SIM tests).
