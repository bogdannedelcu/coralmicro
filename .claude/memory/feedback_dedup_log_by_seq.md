---
name: Driver log loops MUST dedupe by frame_seq
description: Otherwise cumulative analysis on the trace double-counts each algorithm result and inflates closure proportionally.
type: feedback
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
**Rule:** when a driver loop polls a per-frame algorithm output
at a rate >camera fps, it MUST dedupe by `frame_seq`.

**Why:** During 2026-05-05 flow rework: driver was logging at
50 Hz target, camera delivered ~25 fps, so each unique algorithm
output appeared 2-3× in the trace.  Cumulative integration on
the trace inflated firmware-cumsum by 2-3× vs offline-cumsum on
the same gray frames.  Looked like a firmware bug; was a
sampling artefact.

**Pattern:**
```python
last_logged_seq = [-1]  # mutable holder so closures can update

def sample_block(...):
    while True:
        d = sentai.flow.read()
        if d["frame_seq"] == last_logged_seq[0]:
            ... yield / sleep small ...
            continue
        last_logged_seq[0] = d["frame_seq"]
        ... log d ...
```

**How to apply:** Any diag driver that loops faster than the
algorithm produces (50 Hz log of a 25 fps algorithm, etc.) needs
this dedup or any cumulative analysis on the trace will be
inflated.  Same for offline replay -- pair gray frames at
unique-seq transitions only.
