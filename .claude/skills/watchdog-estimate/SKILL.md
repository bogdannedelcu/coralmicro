---
name: watchdog-estimate
description: Estimate driver wall time + pick deadline strategy so the combined watchdog doesn't kill the board during a long REPL-driven test. Use when writing a new diag driver, especially one that loops over frames / sweeps / repeated TPU invokes.
---

# /watchdog-estimate

The combined watchdog task in `sentai_runtime.cc:CombinedWatchdogTask` treats **>120 s of REPL silence as dead** and stops kicking WDOG1.  WDOG1 then resets the board ~30 s later.  Long-running drivers must plan for this BEFORE starting — silent WDOG hang mid-experiment loses all in-flight data.

## Step 1 — compute expected duration

Sum the wall time of every inner loop and add 25 % margin:

```
T_driver  =  sum( N_frames_i  ×  period_i )  ×  1.25
```

Examples:
- 5 delay points × 30 frames × 50 ms/frame: `5 × 30 × 0.05 × 1.25 ≈ 9.4 s` — well under 120 s.
- 16 delay points × 60 frames × 100 ms/frame: `16 × 60 × 0.10 × 1.25 ≈ 120 s` — touching the edge.

## Step 2 — choose the deadline strategy

| `T_driver` | Action |
|---|---|
| **< 60 s** | Nothing.  REPL task auto-heartbeats every 5 s — covered. |
| **60–120 s** | Call `sentai.diag.repl_kick()` once per outer-loop iteration as defense-in-depth. |
| **> 120 s** | **Split the driver** into ≤ 5 measurement points per file.  Reflash between halves (cross-test contamination per `agent.md` §2.9 anyway). |

## Step 3 — host-side timeout

The raw-drain loop (`agent.md` §5.1 `run_driver()`) must use:
```python
deadline_s = T_driver + 30   # so host doesn't tear down session while board still printing
```

## `repl_kick()` semantics

```python
sentai.diag.repl_kick()
```
- Bumps `g_repl_last_activity` to `now`.
- No-op if called more often than every ~5 s (cheap regardless).
- Same trust model as the auto-heartbeat in `micropython_task.c`, just explicit per-iteration.

## Why not just bump the global threshold?

Bumping the dead-threshold globally would weaken protection against actual hangs — the whole point of the 120 s ceiling is to detect a wedged REPL.  `repl_kick()` per iteration is a heartbeat proving the script IS making forward progress.

## Don't disable the watchdog

The board self-healing contract in `embeded.md` §10 requires WDOG1 to stay armed at all times.  There is intentionally **NO public API to extend the hardware timeout**.  Any new code that tries to is rejected at review.

## Driver template

```python
import sentai
N_OUTER = 16            # delay points
N_INNER = 60            # frames per point
FRAME_MS = 100          # measured: 100 ms / frame

T_DRIVER = N_OUTER * N_INNER * (FRAME_MS / 1000) * 1.25
print(f"# expected T_driver = {T_DRIVER:.1f} s")
assert T_DRIVER < 120, "split this driver"

for outer in range(N_OUTER):
    sentai.diag.repl_kick()      # defense-in-depth heartbeat
    for inner in range(N_INNER):
        # ... measurement ...
        pass

print("=== done ===")
```

## Reject patterns

- Single-monolith driver running 5+ minutes "because reflash between halves is annoying".  The watchdog discipline is non-negotiable.
- Trying to extend the watchdog timeout from MP code.
- Forgetting `print("=== done ===")` at the end — the host-side raw-drain loop never terminates cleanly.
- Skipping the assert — silent overruns become silent board resets.

## See also

- `agent.md` §5.1.1 "Estimate driver duration + extend the watchdog"
- `agent.md` §5.1 "Long-running script idiom" (raw-drain loop)
- `embeded.md` §10 anti-brick (WDOG1 contract)
