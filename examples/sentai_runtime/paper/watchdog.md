# Hardware Watchdog Implementation

## Overview

SentAI implements a **hardware watchdog** (WDOG1) that ensures automatic system recovery when the board becomes unresponsive. Unlike software watchdogs (FreeRTOS tasks), the hardware watchdog runs on an independent 32 kHz clock and can reset the CPU even when the scheduler is completely blocked.

## Problem Statement

During stress testing, it was discovered that a **software watchdog** (implemented as a FreeRTOS task) is insufficient:

1. If the CPU is blocked (infinite loop, deadlock, or scheduler stall), the watchdog task never runs
2. The board hangs indefinitely and cannot be reflashed without a manual hardware reset button press
3. In field deployments (drones, remote sensors), manual intervention is impossible

## Solution: NXP RT1176 WDOG1

The NXP i.MX RT1176 has a dedicated **Watchdog Timer (WDOG1)** that:

- Runs on an independent 32 kHz RC oscillator
- Continues counting even if the Cortex-M7 CPU is halted
- Generates a hardware reset (not just an interrupt) when it expires
- Cannot be disabled once enabled (until reset)

### Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Clock Source | 32 kHz RC oscillator |
| Timeout Range | 0.5s – 128s |
| Resolution | 0.5 seconds |
| Reset Type | Full chip reset (same as pressing RST button) |
| Location | WDOG1 peripheral at 0x400B_8000 |

## Implementation Details

### Configuration Parameters

```c
constexpr uint32_t kWdogTimeoutSec     = 30;      // Hardware WDOG1 timeout
constexpr uint32_t kKickIntervalMs     = 5000;    // Task wakes every 5s
constexpr uint32_t kWarningThresholdMs = 60000;   // Log warning at 60s idle
constexpr uint32_t kDeadThresholdMs    = 120000;  // Stop kicking at 120s idle
```

Actualizate în build #587.  Pragurile sunt mai generoase decât cele din
primele builds (15s/25s) pentru că o sesiune interactivă Python poate lua
natural pauze lungi între comenzi.  Logica completă de escaladare:

-   `0…60s` idle → HEALTHY, kick la fiecare 5s.
-   `60…120s` idle → WARNING, încă kick, log `E:0501 idle=<sec>`.
-   `>120s` idle → DEAD, oprește kick-ul.  WDOG1 fizic firește după ~30s.
-   Fallback soft-reset la 35s dacă WDOG1 nu e încă prins (ex. debugger):
    ciclu de 7 × `vTaskDelay(5000ms)` apoi `NVIC_SystemReset()`.

### Initialization

```c
wdog_config_t wdog_config;
WDOG_GetDefaultConfig(&wdog_config);
wdog_config.timeoutValue = (kWdogTimeoutSec * 2) - 1;  // 30s = value 59
wdog_config.enableWdog = true;
wdog_config.workMode.enableWait = true;     // Run in WAIT mode
wdog_config.workMode.enableStop = false;    // Don't run in STOP mode
wdog_config.workMode.enableDebug = false;   // Don't pause for debugger!
wdog_config.enableInterrupt = false;        // No interrupt, just reset

// Brief settling delay — USB CDC is already up after main_freertos init.
vTaskDelay(pdMS_TO_TICKS(3000));

// Enable hardware watchdog
WDOG_Init(WDOG1, &wdog_config);
```

### Activity Monitoring

The watchdog monitors two activity sources:

1. **HTTP Server** — `g_http_last_activity` updated on every HTTP request
2. **REPL Input** — `g_repl_last_activity` updated on every keystroke

If **either** interface is active, the system is considered healthy.

### State Machine

```
                     ┌─────────────────────────────────────┐
                     │                                     │
     Activity        │                                     ▼
  ◄──────────────────┤              HEALTHY               │
     (< 60 sec)      │         WDOG_Refresh()             │
                     │       Log status every 5min        │
                     │                                     │
                     └─────────────────────────────────────┘
                                      │
                                      │ No activity for 60s
                                      ▼
                     ┌─────────────────────────────────────┐
                     │                                     │
                     │              WARNING                │
                     │         WDOG_Refresh()             │
                     │   Log E:0501 (SERR_WDG_WARN)        │
                     │                                     │
                     └─────────────────────────────────────┘
                                      │
                                      │ No activity for 120s
                                      ▼
                     ┌─────────────────────────────────────┐
                     │                                     │
                     │            DEAD ZONE                │
                     │         NO WDOG_Refresh()          │
                     │   Log E:05F0 (SERR_WDG_DEAD)        │
                     │                                     │
                     └─────────────────────────────────────┘
                                      │
                                      │ ~30s (WDOG1 expires)
                                      │ or 35s (NVIC fallback)
                                      ▼
                     ┌─────────────────────────────────────┐
                     │                                     │
                     │          HARDWARE RESET             │
                     │         CPU restarts at 0x0        │
                     │                                     │
                     └─────────────────────────────────────┘
```

### Storage-mode bypass

În storage mode (`sentai.usb.drive(1)`) atât REPL-ul cât și
CDC-NCM/HTTP-ul sunt oprite **deliberat** — board-ul expune NAND-ul
ca disc USB host-ului.  În această stare nimic nu mai actualizează
`g_http_last_activity` / `g_repl_last_activity`, deci logica obișnuită
de idle devine patologică: la 120s idle declară DEAD și la ~150s
resetează board-ul în mijlocul sesiunii USB a utilizatorului.

Pentru a evita acest reset parazit, `CombinedWatchdogTask` verifică la
fiecare tick dacă e storage mode și, dacă da, **face doar kick la WDOG1
(păstrând protecția împotriva CPU-lockup) și sare peste logica
dead/warn**:

```c
for (;;) {
    vTaskDelay(pdMS_TO_TICKS(kKickIntervalMs));
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    sentai_fault_set_uptime(now);

    // In storage mode, REPL/HTTP are intentionally off — skip dead logic.
    if (sentai_storage_mode_active()) {
        WDOG_Refresh(WDOG1);
        g_network_healthy = true;
        consecutive_kicks = 0;
        continue;
    }
    /* normal HEALTHY/WARN/DEAD logic ... */
}
```

Fix aplicat în build #587.  Verificat cu o sesiune susținută de 4m1s
în storage mode fără reset (înainte, resetul era fiabil la 150±5 s).
Ieșirea din storage mode se face doar voluntar — `drive(0)`, byte pe
`/dev/ttyACM0`, buton User, buton RESET.  În default mode se revine
automat la logica completă de activity monitoring.

### Task Priority

The watchdog task runs at `configMAX_PRIORITIES - 2` (priority 14 out of 16), ensuring it executes even when other tasks are starving the scheduler.

```c
xTaskCreate(CombinedWatchdogTask, "hw_wdog", 2048, nullptr,
            configMAX_PRIORITIES - 2, nullptr);
```

## Crash Logging

Before the watchdog triggers a reset, crash information is logged to `/log/crash_NNN.log`:

```c
crash_log_write("WATCHDOG_WARNING", "No activity for 20s...");
crash_log_write("WATCHDOG_TIMEOUT", "BOTH interfaces dead for 28s...");
```

Log rotation keeps the last 10 crash logs (`crash_000.log` through `crash_009.log`).

## Why This Works

| Scenario | Software Watchdog | Hardware Watchdog (WDOG1) |
|----------|------------------|---------------------------|
| Task blocks in infinite loop | ❌ Task never runs | ✅ WDOG1 resets CPU |
| Scheduler deadlock | ❌ No context switch | ✅ WDOG1 resets CPU |
| All tasks sleeping | ❌ No task to kick | ✅ WDOG1 resets CPU |
| Normal operation | ✅ Task kicks | ✅ Task kicks WDOG1 |

The key insight: **WDOG1 is hardware, not software**. It counts down regardless of what the CPU is doing.

## Testing Results

Stress test with HTTP server (Build #446):

```
=== HTTP Stress Test with Hardware Watchdog ===
[Test] 100 concurrent requests via 50 threads... Result: 7/100 in 10.1s
[Test] Download browser.html 30x via 20 threads... Result: 14/30 OK
[Test] Sustained load for 60 seconds... Final: 76/76 OK
[Test] Final connectivity check... Board is ALIVE! Status=200
```

The board survived sustained load without hanging. If it had hung, WDOG1 would have reset it within 30 seconds.

## API Functions

### From MicroPython

```python
# Check if watchdog is enabled
sentai.sys.watchdog_enabled()  # Returns True/False

# Get activity timestamps
sentai.sys.http_requests()     # Total HTTP request count
sentai.sys.repl_inputs()       # Total REPL input count
sentai.sys.network_healthy()   # True if recent activity
```

### From C

```c
// Activity markers (called by HTTP server and REPL)
extern void sentai_http_activity(void);   // Mark HTTP request
extern void sentai_repl_activity(void);   // Mark REPL input

// Read statistics
extern uint32_t sentai_get_http_requests(void);
extern uint32_t sentai_get_repl_inputs(void);
extern int sentai_get_network_healthy(void);
```

## Boot Log Indicators

After a watchdog reset, the boot log shows:

```
Reset reason: 0x08010003   ← Includes WDOG reset flag
[HW_WATCHDOG] Started: WDOG1 timeout=30s, kick every 5000ms
[HW_WATCHDOG] Reset if no activity for 25s (HTTP or REPL)
```

## Files

| File | Purpose |
|------|---------|
| `sentai_runtime.cc` | `CombinedWatchdogTask()` implementation, `crash_log_*()` functions |
| `micropython_task.c` | `sentai_repl_activity()` call on REPL input |
| `sentai_httpd.cc` | `sentai_http_activity()` call on HTTP requests |
| `fsl_wdog.h` | NXP SDK driver for WDOG1 |

## Configuration

To adjust watchdog behavior, modify these constants in `sentai_runtime.cc`:

```c
constexpr uint32_t kWdogTimeoutSec     = 30;      // WDOG1 timeout (5-128s)
constexpr uint32_t kKickIntervalMs     = 5000;    // How often task runs
constexpr uint32_t kWarningThresholdMs = 60000;   // Start logging warnings
constexpr uint32_t kDeadThresholdMs    = 120000;  // Stop kicking
```

**Rule of thumb**: `kDeadThresholdMs` trebuie să fie suficient de generos
încât un user interactiv în REPL să nu fie resetat din greșeală între
comenzi (ex. citește un log lung).  `kWdogTimeoutSec` trebuie să fie
`≥ 2 × kKickIntervalMs` ca să acopere un singur kick ratat la jitter de
scheduler.

## Limitations

1. **Debug Mode**: WDOG1 does not pause during JTAG debugging (by design). Set `enableDebug = true` if you need this.
2. **Sleep Modes**: WDOG1 continues in WAIT mode but not in STOP mode.
3. **Minimum Activity**: The system requires at least one HTTP request or REPL keystroke every 120 seconds to avoid reset — *except în storage mode*, unde logica de idle este complet bypassed.
4. **No Disable**: Once WDOG1 is enabled, it cannot be disabled until the next reset.

## Summary

The hardware watchdog provides bulletproof recovery from system hangs:

- **30 second timeout**: Generous enough for normal operation
- **Independent clock**: Works even if CPU is halted
- **Dual-interface monitoring**: HTTP OR REPL activity keeps system alive
- **Crash logging**: Records reason before reset
- **Field-ready**: No manual intervention needed for recovery
