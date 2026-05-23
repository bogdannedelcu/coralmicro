---
name: async-dispatch-audit
description: Verify the three MicroPython scheduler-drain hook paths are intact so async callbacks (mp_sched_schedule) actually fire. Use when adding new C-side code that schedules MP callbacks (e.g. event handlers from camera, safety, FR, crazy), or after touching micropython_task.c / mpconfigport.h / mod_sentai sleep/atomic.
---

# /async-dispatch-audit

MicroPython's `mp_sched_schedule(callback, arg)` queues callbacks that fire from a polled trampoline.  Our build has THREE drain points where the trampoline runs.  Remove any one and async callbacks fire silently late or not at all.

## The three drain hooks (ALL must exist)

| Hook | Location | When it drains |
|---|---|---|
| **REPL idle** | `repl_getchar()` in `micropython_task.c` | Every ~10 ms while waiting for input on stdin |
| **Sleep chunked** | `mod_sentai_sleep_ms()` in `bindings/modsentai.c` | Every ≤10 ms slice inside MP `sleep_ms` calls |
| **Atomic section** | `MICROPY_BEGIN/END_ATOMIC_SECTION` in `mpconfigport.h` | At exit of every FreeRTOS critical section MP enters |

## Audit recipe

1. **REPL idle drain present:**
   ```bash
   grep -nE 'mp_handle_pending|mp_sched_keep_alive' \
       /home/bogdan/work/coralmicro/examples/sentai_runtime/micropython_task.c
   ```
   Expect at least one match inside the REPL char-wait loop.

2. **Sleep chunked drain present:**
   ```bash
   grep -nA5 'mod_sentai_sleep_ms' \
       /home/bogdan/work/coralmicro/examples/sentai_runtime/bindings/modsentai.c
   ```
   Expect `mp_handle_pending(true)` inside the slice loop, NOT just `vTaskDelay`.

3. **Atomic-section drain not overridden:**
   ```bash
   grep -nE 'MICROPY_BEGIN_ATOMIC_SECTION|MICROPY_END_ATOMIC_SECTION' \
       /home/bogdan/work/coralmicro/examples/sentai_runtime/mpconfigport.h
   ```
   - **Expected**: either no override, OR override that calls the default + drain.
   - **Reject**: bare `taskENTER_CRITICAL()` override with no MP-pending check on exit.

## Diagnose a "callback never fires" bug

If a `mp_sched_schedule(...)` from C-side code never reaches the Python callback:

```python
# REPL diagnostic
import sentai, micropython
sentai.diag.repl_kick()
# Trigger the event that should schedule the callback
# Then check MP scheduler queue state:
micropython.schedule(lambda x: print("test"), None)
# If even this print never fires, the drain hooks are broken.
```

Then walk the 3 hooks via grep above to find which is missing.

## Why this matters

The async-dispatch path is how C-side subsystems (SafetyTask, FR drain, camera ISR top-half) notify MP-side code.  Examples:
- `sentai_safety` posts an `ABORT` event → MP mission's safety callback fires.
- `sentai_fr` notifies "queue full, frame dropped" → MP test driver records it.
- `sentai_crazy` RX handler queues a CRTP packet → MP mission's RX callback fires.

If a drain hook goes missing, callbacks pile up in MP's `MP_STATE_VM(sched_state)` and either fire much later (when REPL idle drain finally runs) or never (if the test doesn't return to REPL).  Symptom: "the abort happened in the firmware but Python never knew".

## Reject patterns

- Adding `MICROPY_BEGIN_ATOMIC_SECTION` override "to avoid race in mp_print" — it disables the drain.  Use a finer-grained mutex around the specific data instead.
- Replacing `mod_sentai_sleep_ms` with `vTaskDelay(pdMS_TO_TICKS(N))` — vTaskDelay doesn't drain.  Keep the chunked-slice loop.
- "Pure FreeRTOS task notification" between C and MP without going through `mp_sched_schedule` — the MP-side code can't observe it.

## Reference: real bug this catches

Phase 2 of sentai.fr (W13-T5) initially used a direct task notify from FR drain → MP test driver.  Drain hooks were intact but the test was using `time.sleep` (which is C-level, not MP-level).  Adding the `mod_sentai_sleep_ms` path fixed it.  Without this audit checklist, we would have rewritten the C-side instead of fixing the host code.

## See also

- `agent.md` §Pattern C scheduler-drain hooks (~lines 2525-2565)
- `[[crazy-anti-patterns-check]]` skill (D: MICROPY_BEGIN_ATOMIC_SECTION override is in both audits)
