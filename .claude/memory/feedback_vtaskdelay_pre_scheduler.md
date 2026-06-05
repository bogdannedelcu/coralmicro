---
name: vTaskDelay forbidden pre-scheduler
description: Calling vTaskDelay() before vTaskStartScheduler bricks boot — pxCurrentTCB null deref. Use bounded_delay_ms helper that gates on xTaskGetSchedulerState().
type: feedback
originSessionId: c7a210f2-a5f7-4a53-86b4-b15f5f9be074
---
NEVER call `vTaskDelay()` (or any blocking FreeRTOS API) pre-scheduler.

**Why:** the M7 FreeRTOS port's `vTaskDelay` manipulates `pxCurrentTCB`,
which is NULL until `vTaskStartScheduler()` runs.  Pre-scheduler →
null deref → HardFault → boot loop.  GPR2-8 fault breadcrumb stays
zero because the fault hits before `sentai_fault_save` is wired.

**How to apply:** any code that's reachable from `CHECK(...)` calls in
`libs/base/main_freertos_m7.cc::real_main` (LfsInit, LfsUserInit,
USB init, etc.) MUST avoid `vTaskDelay`.  If a delay is needed,
use a scheduler-state-aware helper:

```c
inline void bounded_delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    /* Pre-scheduler busy spin — ~130M iters/sec on 800 MHz core. */
    volatile uint32_t i;
    const uint32_t kIters = ms * 100000u;
    for (i = 0; i < kIters; ++i) { __asm__ volatile("nop"); }
}
```

**The bug that taught this:** 2026-05-09 commit `c4577170` introduced
a 3×50 ms bounded-retry loop in `FxUserInit`/`FxUserOpenLxOnly` (called
from `CHECK(LfsUserInit())` pre-scheduler).  Board flashed clean,
boot reached `progress=0x06` (vTaskStartScheduler called), 14 tasks
created, but `app_main` task NEVER ran (`SRC_GPR1 boot_attempts = 0`).
Reset loop, `1fc9:9307` HID-only enum, no `1fc9:c0a1` app mode.

JTAG halt → PC oscillating in boot ROM (`0x223104`) AND in our
`vPortEnterCritical` (FreeRTOS port code).  Diagnosis took ~3 hours:
chip technically reached scheduler but fault triggered immediately
on first task switch attempt because some heap state was corrupted
by the pre-scheduler `vTaskDelay`.

Fix shipped in commit `7a03d9a1` ("fs: bounded_delay_ms — fix
vTaskDelay pre-scheduler boot brick"): both retry sites now call
`bounded_delay_ms(kMountRetryDelayMs)` instead of raw `vTaskDelay`.

**Other pre-scheduler hazards in the same call chain:**
- `xSemaphoreTake(handle, ticks)` with timeout > 0 — also UB
  pre-scheduler.  Use `pdMS_TO_TICKS(0)` (try-take only) or skip.
- `xTaskGetTickCount()` — returns 0 pre-scheduler, harmless but useless
  for timing.
- `printf` — flush spins indefinitely if USB CDC TX ring saturates
  (separate `boot_safe_logf` gate gates this in fx_user_fs.cc).

**Diagnostic recipe** when boot wedges and you suspect pre-scheduler:
1. JLink halt + read `g_boot_persist.progress` (DTC-RAM
   `0x2000ad5c`) — last `sentai_boot_progress_mark()` reached.
   `0x06` = real_main ended, scheduler started.  `0x10` = app_main
   first instruction.  Anything between = hung in scheduler init.
2. Read `xSchedulerRunning` (BSS `0x200101bc`) — 1 = scheduler ran.
3. Read `uxCurrentNumberOfTasks` (BSS `0x20010160`) — count of created
   tasks.  Less than expected = some `xTaskCreate` failed pre-scheduler
   and `CHECK` no-op'd it.
4. Read `SRC_GPR1` at `0x40C04014` — `app_main` increments on entry.
   Zero = `app_main` never ran.

If progress=0x06 + scheduler=1 + tasks=N + GPR1=0, the cause is
something pre-scheduler that corrupted heap or registered an early
HardFault that resets before `sentai_fault_save` could record GPR2-8.
