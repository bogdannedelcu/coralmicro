---
name: MicroPython mp_sched_schedule needs explicit drain hooks
description: When delivering async callbacks from a non-MP FreeRTOS task into MicroPython via mp_sched_schedule, the trampoline only fires while MP is executing bytecode with branches. Add mp_handle_pending(true) drain hooks at every "MP idle" point so callbacks fire promptly.
type: feedback
originSessionId: c7a210f2-a5f7-4a53-86b4-b15f5f9be074
---
On the SentAI runtime (embed-port MicroPython on FreeRTOS), `mp_sched_schedule(fn, arg)` queues a Python callback but **only runs it when the VM ticks `mp_handle_pending(true)` from inside a branch opcode of executing bytecode**. The default embed-port `repl_getchar` blocks on stdin, so while at the REPL prompt the scheduler queue accumulates and the trampoline never fires.

**Why:** Discovered 2026-05-06 chasing the Pattern C dispatcher for the Crazyflie radio bridge — frames arrived at the rx_task, dispatch_push enqueued, mp_sched_schedule returned `true` and put state=PENDING, but no trampoline output ever appeared until the user manually ran a Python `for` loop. Cost: ~3 hours of incorrect "wire is broken" / "drone TX is wedged" theorising before instrumenting `MP_STATE_VM(sched_state)` and seeing it reset to IDLE without ever calling our function.

**How to apply:** Whenever you wire a non-MP task to deliver async events into MicroPython via `mp_sched_schedule`:

1. **REPL stdin polling MUST drain the scheduler.** In any `getchar`-style helper that loops on stdin polls + `vTaskDelay`, add `mp_handle_pending(true)` between the read attempt and the delay. See [examples/sentai_runtime/micropython_task.c:repl_getchar](../../../work/coralmicro/examples/sentai_runtime/micropython_task.c) for the canonical version. Mainline ports do this in their `mp_hal_stdin_rx_chr` equivalents — same idea, different name.

2. **Long sleeps MUST chunk + drain.** A naive `vTaskDelay(N)` blocks the VM for the full duration. Wrap MP-visible sleep functions to chunk into ≤10 ms slices and call `mp_handle_pending(true)` after each. See `mod_sentai_sleep_ms` in [modsentai_rtos.c](../../../work/coralmicro/examples/sentai_runtime/modsentai_rtos.c).

3. **Atomic sections MUST be cross-task safe.** The default embed-port `MICROPY_BEGIN_ATOMIC_SECTION` is a no-op. Override it in `mpconfigport.h` to use FreeRTOS critical sections (taskENTER_CRITICAL via extern wrappers in `mp_embed_safe.c` so the QSTR pre-pass survives without the firmware include paths).

4. **MICROPY_ENABLE_SCHEDULER MUST be set to 1.** The minimum-ROM-level config has it off by default.

5. **Don't trust `g_*_pending` dedup flags alone.** A scheduling that succeeds but loses its callback (e.g. exception during dispatch) leaves the flag stuck at 1 forever, blocking all future schedules. Either always re-schedule (the cost is at most one "drain empty" trampoline invocation per push), OR clear the flag at trampoline entry AND in any abort path.

6. **Beware "running bytecode" doesn't mean "scheduler fires often".** `sum(range(N))` / `bytes.join` / any built-in C reduction runs in C with NO branch opcodes between iterations. The scheduler may not tick for the entire computation. If your async path matters during heavy work, prefer a Python `for` loop or sprinkle `time.sleep_ms(0)` calls.

7. **Verify with stack trace, not just `ok=true` returns.** `mp_sched_schedule` returning `true` only means "queued"; actual delivery requires VM cooperation. Add a print at trampoline entry and confirm it fires within a few ms of scheduling — if it doesn't, you're hitting one of the above hooks.

**Diagnostic recipe** when async callbacks aren't firing:
- Print `MP_STATE_VM(sched_state)` and `MP_STATE_VM(sched_len)` right after `mp_sched_schedule` returns. State should be PENDING (1), len should be ≥1.
- Add `printf` at trampoline entry — if you never see it, the VM isn't draining.
- Add `mp_handle_pending(true)` to your stdin getchar OR run a tight Python `for` loop to see if the trampoline fires under bytecode pressure.

**Bonus: FromISR semaphore variants from task context don't work.**
Discovered 2026-05-06 the same day: a CH=2 telemetry rx-handler ran inside the rx FreeRTOS task and used `xSemaphoreGiveFromISR(...)` to wake the query waiter. Drone-side counters confirmed replies were sent, but the board's waiter timed out every time. Fix is `xSemaphoreGive` — `FromISR` variants are undefined behavior outside of true ISR context on the FreeRTOS port we're using.
