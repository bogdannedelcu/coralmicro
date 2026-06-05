---
name: Experiment run cadence — verbose first, then quiet multi-run
description: First shakedown of any new experiment must be a single run with sentai.verbose(1); only after that succeeds do repeat runs with verbose(0)
type: feedback
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
When running a NEW experiment (or an experiment after a firmware change), the first invocation must be a single trial with `sentai.verbose(1)`. Only after that single run completes cleanly should subsequent runs use `sentai.verbose(0)` for quiet multi-trial benchmarking.

**Why:** verbose=1 keeps per-frame firmware prints visible so a hang, watchdog reset, or unexpected state transition is diagnosable immediately. Going straight to verbose=0 multi-trial hides early failures and wastes time on repeats that will all fail the same way.

**How to apply:**
- First call of a new experiment/driver after a firmware flash or new experiment definition: `sentai.verbose(1)`, run once, read the trace end-to-end.
- Only if that trial prints a clean completion ("=== DONE ===", summary stats, expected counts) should the next call wrap multiple trials with `sentai.verbose(0)`.
- If the verbose trial hangs or resets, fix the firmware/experiment before attempting multi-trial runs.
