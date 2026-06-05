---
name: SAFE MODE pre-scheduler brick (build #1225 fix)
description: FxUserInit returning 0 in SAFE MODE bricked board because LfsUserInit→CHECK→vTaskSuspendAll() pre-scheduler froze the scheduler before USB CDC came up
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Build #1225 (2026-05-10) fixes a bug where FileX corruption made the board appear bricked instead of dropping to the documented SAFE MODE (REPL+USB+radio alive for operator recovery).

**Root cause** (`libs/base/fx_user_fs.cc:469`): `FxUserInit()` returned `0` when entering SAFE MODE. `LfsUserInit()` propagates as `return FxUserInit(0) != 0;` → `false`. `CHECK(LfsUserInit())` at `main_freertos_m7.cc:389` then called `EmergencyWrite + vTaskSuspendAll()`. **Pre-scheduler `vTaskSuspendAll` only increments `uxSchedulerSuspended`** (per agent.md §16.1) — silent. Then `vTaskStartScheduler` started the scheduler with `uxSchedulerSuspended >= 1`. Only Timer task got CPU. UsbDeviceTask never ran → USB never enumerated as `1fc9:c0a1` (descriptor read -110 timeout, port power-cycle, "unable to enumerate"). Board appeared bricked even though firmware itself was alive (JTAG showed 14 tasks created, scheduler running but suspended, `xPendedTicks` accumulating).

**Why:** brownout during USB→battery transition (Crazyflie BL deck VBAT ~3.7V vs board's 5V need) leaves a NAND page half-written. ECC fails next boot (`[nand] READ FAIL page=180802`). FxUserInit correctly enters SAFE MODE per the new FSM but the wrong return value bricked it.

**Fix:** `FxUserInit()` now returns `1` in SAFE MODE. `g_mounted=false` is sufficient to gate every `FxUser*` API (each has `if (!g_mounted) return 0/-1;` early-return), so write/read semantics stay safe regardless. The boot completes, USB CDC comes up, REPL is reachable, operator runs `sentai.diag.fx_format(0xDEADBEEF)` to recover.

**How to apply:** When touching boot-path init code that returns 0/false on "soft" failure, ask: does this propagate up to a `CHECK(...)` macro? `CHECK` calls `vTaskSuspendAll` which is silent pre-scheduler — the brick manifests as USB-not-enumerating, NOT as a fault printout. JTAG signature: `xSchedulerRunning=1`, `uxSchedulerSuspended>=1`, `pxCurrentTCB="Tmr Svc"`, `xPendedTicks` rising. Soft-fail paths must return success (boot continues) and gate the actual functionality at the API level via a state flag.

**Validation:** experiments/s086_radio_smoke/test_radio_1plus1.py — `$1+1 → OK 2` via Crazyradio, board on drone battery, USB diagnosed via JTAG before fix, USB enum after fix.

**Open question (Pas 2):** what corrupts NAND at USB→battery transition. Hypothesis: brownout interrupts a FileX commit mid-write. Not addressed in #1225.
