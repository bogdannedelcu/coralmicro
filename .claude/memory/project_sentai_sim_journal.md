---
name: sentai-sim-journal
description: "sentai.sim.journal_* (SIM-only): structured append-mode log under SENTAI_SIM_ROOT for complex integration tests. Use for every multi-step test where mid-flight crash localisation matters."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**Added 2026-05-14** in `sim/modsentai_sim.c` (alongside sentai.fs).
SIM-only — ARM build does NOT expose `sentai.sim`.  Full reference:
[Sim.md §10x](../../Sim.md).

## API

```python
sentai.sim.journal_open(path[, truncate=True])  -> 0|-1
sentai.sim.journal_close()                      -> 0
sentai.sim.journal_write(label[, value=None])   -> 0|-1
sentai.sim.journal_status()                     -> dict
```

Line format:
```
<t_ms> <label> <repr(value)|'-'>
```
Header/footer lines start with `#`.  Host parses with `ast.literal_eval`.

## When to use

Any multi-step integration test where "test hung and I can't tell where"
becomes a real cost — i.e., anything more complex than the per-layer
`diag/_t_*.py` driver smokes.

Idiom (host runner driving REPL via subprocess pipes):

```python
def j_int(repl, label, cmd):
    rc = repl.exec_int(cmd)
    repl.exec_int(f"sentai.sim.journal_write('{label}', sentai.servo.status())")
    return rc
```

Canonical user: [[s128-l41baseline-shipped]] (`mission_l41.py`).

## Why not just `sentai.fs.append`

We tried that first; the journal API is the cleaner replacement because:
- One `journal_write(label, value)` is shorter + safer than building a
  repr-formatted dict in MP space.
- Header/footer lines + timestamps come for free.
- Truncate-on-open is the right default for fresh test runs.
- Error counter visible via `journal_status()`.

## Implementation notes (for future maintenance)

- File lives at `${SENTAI_SIM_ROOT}/<name>` (build-sim/sentai_fs_root/).
- Single global FILE* + 1 char[SIM_FS_MAXPATH+1] path.  No locking —
  REPL is single-threaded for command dispatch.
- Each write does `fflush()` so SIGKILL leaves journal parseable up to
  the last complete write.
- Uses `mp_obj_print_helper` + `vstr_add_strn` as the canonical
  MP-repr-to-bytes idiom (NOT `mp_obj_repr` which is not public API).
- Time source: POSIX `clock_gettime(CLOCK_MONOTONIC)` (added `<time.h>`
  include to modsentai_sim.c).

QSTRs registered via `qstrdefs_sim_extra.h` (SIM-side QSTRs aren't
picked up by the ARM QSTR scanner; the keepalive tuple is the standard
workaround per `[[sim-repl-test-recipe]]`).

Related: [[sim-repl-test-recipe]], [[s128-l41baseline-shipped]],
[[servo-l4-shipped]].
