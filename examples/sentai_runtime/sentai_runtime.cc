// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>
#include <unistd.h>
#include "build_version.h"
#include "libs/base/watchdog.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_soc_src.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_wdog.h"

extern "C" {
#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"
}
#undef FAR  // jpeglib defines FAR as empty, conflicts with NXP SDK struct field

#include "libs/base/console_m7.h"
#include "libs/base/filesystem.h"
#include "libs/base/gpio.h"
#include "libs/base/led.h"
#include "libs/base/reset.h"
#include "libs/camera/camera.h"
#include "libs/libjpeg/jpeg.h"
#include "libs/camera/camera_support.h"
#include "fsl_pxp.h"
#if (__CORTEX_M == 7)
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7/fsl_cache.h"
#endif
#include "libs/tensorflow/detection.h"
#include "libs/tensorflow/utils.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
#include "libs/tpu/edgetpu_task.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"

extern "C" {
#include "micropython_task.h"
#include "detection_task.h"
#include "sentai_tracker.h"

// Boot diagnostic getters defined in libs/base/main_freertos_m7.cc.
uint32_t sentai_boot_prev_progress(void);
uint32_t sentai_boot_storage_attempts(void);
uint32_t sentai_boot_sram_magic(void);
uint32_t sentai_boot_sram_check(void);
uint32_t sentai_boot_gpr_snap(int);
void     sentai_boot_progress_mark(uint32_t);
void     sentai_storage_boot_succeeded(void);
int      sentai_storage_mode_active(void);
}

#include "sentai_vision_common.h"
#include "sentai_error.h"
#include "sentai_fault.h"
#include "sentai_health.h"
#include "sentai_lfs_task.h"

// ===================== Camera pipeline optimizations ========================
// Set to 1 to enable, 0 to disable (safe revert).  Build #197+
//
// SENTAI_OPT_FAST_POLL   — reduce GetRawFrame poll from 50ms to 5ms (~22ms less latency)
// SENTAI_DBG_COLOR_ORDER — print first 4 pixels after PXP (once) to verify R/G/B order
// ============================================================================
#define SENTAI_OPT_FAST_POLL    1
#define SENTAI_DBG_COLOR_ORDER  0   // verified — R channel is ch0

// Performs object detection with SSD MobileNet, running on the Edge TPU,
// using a local bitmap file as input.
//
// To build and flash from coralmicro root:
//    bash build.sh
//    python3 scripts/flashtool.py -e detect_image

// [start-sphinx-snippet:detect-image]

// =============================================================================
// Boot Logging System
// =============================================================================
// Captures ALL printf/driver output to /log/boot.log until REPL starts.
// On boot: /log/boot.log → /log/boot_old.log, then fresh boot.log created.
// Uses a RAM buffer that's flushed periodically and at REPL start.

namespace {

// Boot log state
static constexpr size_t kBootLogBufSize = 16 * 1024;  // 16 KB buffer
static char g_boot_log_buf[kBootLogBufSize] __attribute__((section(".sdram_bss")));
static volatile size_t g_boot_log_pos = 0;
static volatile bool g_boot_log_active = false;
static volatile bool g_boot_log_fs_ready = false;
static lfs_file_t g_boot_log_file;
static bool g_boot_log_file_open = false;
// Guards boot_log_flush_to_file() against concurrent flush from multiple tasks.
// Uses GCC atomic test-and-set (lock-free, no RTOS dependency).
static volatile uint32_t g_boot_log_flush_busy = 0;

// Forward declare - flush buffer to file
static void boot_log_flush_to_file();

// Initialize boot logging - rename old log, create new one
static void boot_log_init() {
    g_boot_log_pos = 0;
    g_boot_log_active = true;
    g_boot_log_fs_ready = false;
    g_boot_log_file_open = false;
}

// Called after LFS is ready to set up file
static void boot_log_fs_init() {
    if (!g_boot_log_active) return;
    
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;

    // Create /log directory if it doesn't exist
    lfs_mkdir(lfs, "/log");

    // Check if boot.log exists
    lfs_info info;
    if (lfs_stat(lfs, "/log/boot.log", &info) == LFS_ERR_OK) {
        // Remove old boot_old.log if exists
        lfs_remove(lfs, "/log/boot_old.log");
        // Rename boot.log to boot_old.log
        lfs_rename(lfs, "/log/boot.log", "/log/boot_old.log");
    }

    // Open new boot.log for writing
    if (lfs_file_open(lfs, &g_boot_log_file, "/log/boot.log",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) == LFS_ERR_OK) {
        g_boot_log_file_open = true;
        g_boot_log_fs_ready = true;
        
        // Flush any buffered data
        boot_log_flush_to_file();
    }
}

// Flush RAM buffer to file.
// Re-entrant-safe: if another task/call is already flushing, returns immediately
// (data is still in the RAM buffer and will be picked up on the next flush).
// Must only be called from task context (not ISR) — enforced by callers.
static void boot_log_flush_to_file() {
    if (!g_boot_log_file_open || g_boot_log_pos == 0) return;

    // Acquire flush lock — non-blocking (return if another flush is in progress)
    if (__sync_lock_test_and_set(&g_boot_log_flush_busy, 1u) != 0u) return;

    // Acquire LFS mutex with short timeout. If busy (lfs_task or MP holds it),
    // skip this flush — data stays in the RAM buffer for the next opportunity.
    if (!sentai_lfs_lock()) {
        __sync_lock_release(&g_boot_log_flush_busy);
        return;
    }

    lfs_t* lfs = coralmicro::LfsUser();
    if (lfs) {
        // Snapshot the current byte count inside a critical section so we
        // don't race with boot_log_write() appending new bytes.
        taskENTER_CRITICAL();
        size_t count = g_boot_log_pos;
        taskEXIT_CRITICAL();

        if (count > 0) {
            lfs_file_write(lfs, &g_boot_log_file, g_boot_log_buf, count);
            lfs_file_sync(lfs, &g_boot_log_file);

            // Reset only the bytes we already wrote; bytes appended during the
            // write remain at the front of the buffer for the next flush.
            taskENTER_CRITICAL();
            if (g_boot_log_pos >= count) {
                size_t remaining = g_boot_log_pos - count;
                if (remaining > 0)
                    memmove(g_boot_log_buf, g_boot_log_buf + count, remaining);
                g_boot_log_pos = remaining;
            } else {
                g_boot_log_pos = 0;
            }
            taskEXIT_CRITICAL();
        }
    }

    sentai_lfs_unlock();
    __sync_lock_release(&g_boot_log_flush_busy);
}

// Add data to boot log (from _write override)
// Called from any context (task, ISR) via printf -> _write.
// Uses critical section to protect shared buffer from concurrent access.
static void boot_log_write(const char* data, size_t len) {
    if (!g_boot_log_active) return;

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    // Add to RAM buffer
    size_t space = kBootLogBufSize - g_boot_log_pos;
    size_t to_copy = (len < space) ? len : space;
    if (to_copy > 0) {
        memcpy(g_boot_log_buf + g_boot_log_pos, data, to_copy);
        g_boot_log_pos += to_copy;
    }

    // If buffer is getting full and FS ready, flush.
    // NOTE: flush does LFS I/O which is slow — only in non-ISR context.
    bool need_flush = g_boot_log_fs_ready &&
                      g_boot_log_pos > kBootLogBufSize - 512;

    taskEXIT_CRITICAL_FROM_ISR(saved);

    if (need_flush && xPortIsInsideInterrupt() == pdFALSE) {
        boot_log_flush_to_file();
    }
}

// Stop boot logging (called when REPL starts)
void boot_log_stop() {
    if (!g_boot_log_active) return;
    
    g_boot_log_active = false;

    // Final flush + close (protected by LFS mutex).
    if (g_boot_log_file_open) {
        boot_log_flush_to_file();  // acquires/releases mutex internally
        if (sentai_lfs_lock()) {
            lfs_t* lfs = coralmicro::LfsUser();
            if (lfs) {
                lfs_file_close(lfs, &g_boot_log_file);
            }
            sentai_lfs_unlock();
        }
        g_boot_log_file_open = false;
    }
}

// =============================================================================
// Crash/Hang Logging System + Hardware Watchdog
// =============================================================================
// Saves crash/hang info to /log/crash_NNN.log for post-mortem analysis.
// Keeps multiple crash logs (kCrashLogMaxFiles) with automatic rotation.
// Hardware watchdog auto-resets board if BOTH HTTP AND REPL are dead for 120s.
// This ensures the board can always be recovered remotely.

static constexpr size_t kCrashLogMaxSize = 8 * 1024;   // Max size per crash log file
static constexpr int kCrashLogMaxFiles = 10;           // Keep last N crash logs

// Verbose per-frame console prints (frame-grab, PXP/quant timing, NMS summary,
// cam_switch info).  Default 1 for interactive debugging; disable while the
// background detection pipeline is running so the ~200 lines/sec of per-frame
// output cannot saturate the CDC-ACM TX endpoint and stall printf.
static volatile int g_sentai_frame_verbose = 1;

extern "C" int sentai_verbose_get(void) { return g_sentai_frame_verbose; }
extern "C" void sentai_verbose_set(int v) {
    g_sentai_frame_verbose = v ? 1 : 0;
}

// Activity tracking - HTTP and REPL
static volatile uint32_t g_http_last_activity = 0;    // Last HTTP activity tick
static volatile uint32_t g_http_request_count = 0;    // Total HTTP requests
static volatile uint32_t g_http_hang_count = 0;       // Detected hangs
static volatile bool g_network_healthy = true;        // Network health flag

static volatile uint32_t g_repl_last_activity = 0;    // Last REPL activity tick
static volatile uint32_t g_repl_input_count = 0;      // Total REPL inputs

// Hardware watchdog control
static volatile bool g_hw_watchdog_enabled = false;
static volatile bool g_force_reset_pending = false;

// Handle for the normal-mode watchdog task (saved so it can be monitored).
static TaskHandle_t s_wdog_task_handle = nullptr;

// Anti-brick recovery mode flag (set when boot_attempts >= 3)
static volatile bool s_in_recovery_mode = false;

// Current crash log number (persisted across rotations)
static int g_crash_log_num = -1;  // -1 = not initialized

// Find highest existing crash log number and set g_crash_log_num
static void crash_log_init(lfs_t* lfs) {
    if (g_crash_log_num >= 0) return;  // Already initialized
    
    lfs_dir_t dir;
    if (lfs_dir_open(lfs, &dir, "/log") != LFS_ERR_OK) {
        g_crash_log_num = 0;
        return;
    }
    
    int max_num = -1;
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
        // Look for crash_NNN.log pattern
        if (info.type == LFS_TYPE_REG && 
            strncmp(info.name, "crash_", 6) == 0 &&
            strlen(info.name) == 13) {  // crash_NNN.log = 13 chars
            int num = atoi(info.name + 6);
            if (num > max_num) max_num = num;
        }
    }
    lfs_dir_close(lfs, &dir);
    
    g_crash_log_num = (max_num >= 0) ? max_num : 0;
}

// Get current crash log path, rotate if needed
static void crash_log_get_path(lfs_t* lfs, char* path, size_t path_len) {
    crash_log_init(lfs);
    
    // Check if current file is too big
    snprintf(path, path_len, "/log/crash_%03d.log", g_crash_log_num);
    
    lfs_info info;
    if (lfs_stat(lfs, path, &info) == LFS_ERR_OK) {
        if (info.size > kCrashLogMaxSize - 640) {
            // Rotate to next file
            g_crash_log_num++;
            snprintf(path, path_len, "/log/crash_%03d.log", g_crash_log_num);
            
            // Delete oldest if we have too many
            if (g_crash_log_num >= kCrashLogMaxFiles) {
                char old_path[32];
                snprintf(old_path, sizeof(old_path), "/log/crash_%03d.log", 
                         g_crash_log_num - kCrashLogMaxFiles);
                lfs_remove(lfs, old_path);
            }
        }
    }
}

// sentai_lfs_lock/unlock are defined in sentai_lfs_task.cc (included via header).

// Append a compact FreeRTOS task dump to the current crash log.
// Useful on watchdog warnings to see which task is blocked and on what.
// Format: one line per task: name | state | prio | hwm(bytes)
static void crash_log_task_dump(const char* why) {
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;
    if (!sentai_lfs_lock()) return;

    lfs_mkdir(lfs, "/log");

    constexpr UBaseType_t kMaxTasks = 32;
    TaskStatus_t tasks[kMaxTasks];
    UBaseType_t n = uxTaskGetSystemState(tasks, kMaxTasks, nullptr);

    uint32_t up = xTaskGetTickCount() * portTICK_PERIOD_MS;
    char hdr[160];
    int hl = snprintf(hdr, sizeof(hdr),
        "  TASK_DUMP (%s, t=%lums, n=%u):\r\n",
        why ? why : "?", (unsigned long)up, (unsigned)n);

    char path[32];
    crash_log_get_path(lfs, path, sizeof(path));

    lfs_file_t file;
    if (lfs_file_open(lfs, &file, path,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) == LFS_ERR_OK) {
        if (hl > 0) lfs_file_write(lfs, &file, hdr, hl);
        static const char* state_names[] = {"Run","Rdy","Blk","Sus","Del","Inv"};
        for (UBaseType_t i = 0; i < n; i++) {
            const char* sn = (tasks[i].eCurrentState < 6)
                ? state_names[tasks[i].eCurrentState] : "?";
            char line[128];
            int ll = snprintf(line, sizeof(line),
                "    %-18s %s prio=%lu hwm=%lu\r\n",
                tasks[i].pcTaskName, sn,
                (unsigned long)tasks[i].uxCurrentPriority,
                (unsigned long)(tasks[i].usStackHighWaterMark * sizeof(StackType_t)));
            if (ll > 0) lfs_file_write(lfs, &file, line, ll);
        }
        lfs_file_close(lfs, &file);
    }
    sentai_lfs_unlock();
}

// Write crash entry to /log/crash_NNN.log
static void crash_log_write(const char* event, const char* details) {
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;

    // Serialize LFS access with HTTP server reads to prevent data races.
    // Skip (don't block) if mutex isn't available — crash logs are non-critical.
    if (!sentai_lfs_lock()) return;

    // Create /log directory if it doesn't exist
    lfs_mkdir(lfs, "/log");
    
    // Build timestamp (uptime in ms)
    uint32_t uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t secs = uptime_ms / 1000;
    uint32_t mins = secs / 60;
    uint32_t hours = mins / 60;
    
    uint32_t http_idle = uptime_ms - g_http_last_activity;
    uint32_t repl_idle = uptime_ms - g_repl_last_activity;
    
    // Format entry with both HTTP and REPL status
    char entry[640];
    int len = snprintf(entry, sizeof(entry),
        "[%02lu:%02lu:%02lu.%03lu] %s: %s\r\n"
        "  HTTP: reqs=%lu hangs=%lu idle=%lums\r\n"
        "  REPL: inputs=%lu idle=%lums\r\n"
        "  Network=%d FreeHeap=%lu\r\n\r\n",
        hours, mins % 60, secs % 60, uptime_ms % 1000,
        event, details ? details : "",
        (unsigned long)g_http_request_count,
        (unsigned long)g_http_hang_count,
        (unsigned long)http_idle,
        (unsigned long)g_repl_input_count,
        (unsigned long)repl_idle,
        g_network_healthy ? 1 : 0,
        (unsigned long)xPortGetFreeHeapSize());
    
    // Get current log path (may rotate)
    char path[32];
    crash_log_get_path(lfs, path, sizeof(path));
    
    // Append to crash log
    lfs_file_t file;
    if (lfs_file_open(lfs, &file, path,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) == LFS_ERR_OK) {
        lfs_file_write(lfs, &file, entry, len);
        lfs_file_close(lfs, &file);
    }

    sentai_lfs_unlock();
}

// Called from HTTP handler on each request
extern "C" void sentai_http_activity(void) {
    g_http_last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_http_request_count++;
    g_network_healthy = true;
}

// Called from REPL when user types input
extern "C" void sentai_repl_activity(void) {
    g_repl_last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_repl_input_count++;
}

// Called when HTTP hang is detected
extern "C" void sentai_http_hang_detected(const char* details) {
    g_http_hang_count++;
    g_network_healthy = false;
    crash_log_write("HTTP_HANG", details);
}

// Called on any crash/error condition
extern "C" void sentai_crash_log(const char* event, const char* details) {
    crash_log_write(event, details);
}

// Returns the path of the most recent crash log file, or "" if none.
// Safe to call from any task context.
extern "C" void sentai_get_last_crash_log_path(char* out, size_t out_len) {
    if (!out || out_len == 0) return;
    out[0] = '\0';

    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;

    if (!sentai_lfs_lock()) return;

    lfs_dir_t dir;
    if (lfs_dir_open(lfs, &dir, "/log") != LFS_ERR_OK) {
        sentai_lfs_unlock();
        return;
    }

    int max_num = -1;
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
        if (info.type == LFS_TYPE_REG &&
            strncmp(info.name, "crash_", 6) == 0 &&
            strlen(info.name) == 13) {  // crash_NNN.log = 13 chars
            int num = atoi(info.name + 6);
            if (num > max_num) max_num = num;
        }
    }
    lfs_dir_close(lfs, &dir);
    sentai_lfs_unlock();

    if (max_num >= 0) {
        snprintf(out, out_len, "/log/crash_%03d.log", max_num);
    }
}

// ===================== Anti-Brick C-API (called from modsentai_sys.c) =====================

extern "C" bool sentai_is_recovery_mode(void) {
    return s_in_recovery_mode;
}

extern "C" uint32_t sentai_get_boot_attempts(void) {
    return SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister1);
}

extern "C" void sentai_sys_do_reset(void) {
    crash_log_write("SYS_RESET", "User-requested software reset from REPL");
    vTaskDelay(pdMS_TO_TICKS(50));
    NVIC_SystemReset();
}

// Getter functions for REPL access
extern "C" uint32_t sentai_get_http_requests(void) {
    return g_http_request_count;
}
extern "C" uint32_t sentai_get_http_hangs(void) {
    return g_http_hang_count;
}
extern "C" int sentai_get_network_healthy(void) {
    return g_network_healthy ? 1 : 0;
}
extern "C" uint32_t sentai_get_repl_inputs(void) {
    return g_repl_input_count;
}

// Combined watchdog task - monitors BOTH HTTP and REPL activity
// Uses HARDWARE WATCHDOG (WDOG1) which resets CPU even if scheduler is blocked!
// Strategy:
// - WDOG1 timeout = 30s (hardware, independent of CPU)
// - Task kicks WDOG1 every 5s IF system is healthy
// - If task doesn't run (CPU blocked) -> WDOG1 resets automatically
// - If system hangs (no activity) -> we don't kick -> WDOG1 resets
static void CombinedWatchdogTask(void* param) {
    (void)param;
    
    // === Hardware Watchdog Configuration ===
    // Timeout = 30 seconds, kick every 5 seconds
    // WDOG1 runs on 32kHz clock, independent of CPU
    constexpr uint32_t kWdogTimeoutSec = 30;
    constexpr uint32_t kKickIntervalMs = 5000;    // Kick every 5 seconds
    constexpr uint32_t kWarningThresholdMs = 60000; // 60 seconds = start warning (still kicks)
    constexpr uint32_t kDeadThresholdMs = 120000;   // 120 seconds = stop kicking → WDOG1 fires
    
    // Initialize WDOG1 directly (not using coralmicro API which uses timers)
    wdog_config_t wdog_config;
    WDOG_GetDefaultConfig(&wdog_config);
    wdog_config.timeoutValue = (kWdogTimeoutSec * 2) - 1;  // Register value = (timeout_s * 2) - 1
    wdog_config.enableWdog = true;
    wdog_config.workMode.enableWait = true;
    wdog_config.workMode.enableStop = false;
    wdog_config.workMode.enableDebug = false;  // IMPORTANT: Don't stop in debugger
    wdog_config.enableInterrupt = false;       // No interrupt, just reset
    
    // Brief settling delay — USB CDC is already up after main_freertos init,
    // and most subsystems are fully started well within 3 s.  The old 10 s
    // delay left a window where a runaway task could hang the board without
    // any hardware watchdog active.
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Initialize activity timestamps to now
    uint32_t boot_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_http_last_activity = boot_time;
    g_repl_last_activity = boot_time;
    
    // Enable hardware watchdog
    WDOG_Init(WDOG1, &wdog_config);
    g_hw_watchdog_enabled = true;
    
    // WDG initialized: timeout, kick interval, dead threshold
    SERR_LOG(SERR_WDG_KICK, kWdogTimeoutSec);
    
    uint32_t last_warning_time = 0;
    uint32_t consecutive_kicks = 0;
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kKickIntervalMs));

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sentai_fault_set_uptime(now);  // update crash timestamp hint for fault handler

        // === STORAGE MODE: REPL/HTTP are intentionally off, no activity
        // signal is possible.  Keep the hardware watchdog alive (protects
        // against CPU lockup) but skip the dead/warn logic entirely — the
        // user is working with the board as a USB disk and expects no
        // auto-reset until they exit storage mode themselves.
        if (sentai_storage_mode_active()) {
            WDOG_Refresh(WDOG1);
            g_network_healthy = true;
            consecutive_kicks = 0;
            continue;
        }

        uint32_t http_idle = now - g_http_last_activity;
        uint32_t repl_idle = now - g_repl_last_activity;

        // Find the most recent activity (either HTTP or REPL)
        uint32_t min_idle = (http_idle < repl_idle) ? http_idle : repl_idle;

        // === HEALTHY: At least one interface active recently ===
        if (min_idle < kWarningThresholdMs) {
            // All good - kick the watchdog
            WDOG_Refresh(WDOG1);
            g_network_healthy = true;
            consecutive_kicks++;
            
            // Log periodic status (every 60 kicks = ~5 minutes)
            if (consecutive_kicks % 60 == 0) {
                SERR_LOG(SERR_WDG_KICK, now / 1000);
            }
            continue;
        }
        
        // === WARNING ZONE: 15-25s without activity ===
        if (min_idle >= kWarningThresholdMs && min_idle < kDeadThresholdMs) {
            // Still kick, but log warning (once per warning period)
            WDOG_Refresh(WDOG1);
            
            if (now - last_warning_time > 10000) {  // Log every 10s max
                last_warning_time = now;
                char buf[64];
                snprintf(buf, sizeof(buf), "idle=%lu H=%lu R=%lu",
                    min_idle / 1000, http_idle / 1000, repl_idle / 1000);
                crash_log_write("WDG_WARN", buf);
                SERR_LOG(SERR_WDG_WARN, min_idle / 1000);
                // Snapshot task states so we can see which one is blocked.
                crash_log_task_dump("WDG_WARN");
            }
            consecutive_kicks = 0;
            continue;
        }
        
        // === DEAD ZONE: both HTTP and REPL idle > kDeadThresholdMs ===
        // Log once, then stop kicking. WDOG1 fires 30s later.
        // Belt-and-suspenders: also call NVIC_SystemReset after 35s in case
        // WDOG1 doesn't fire (e.g. debugger attached with enableDebug=false override).
        g_network_healthy = false;
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "idle=%lu H=%lu R=%lu",
                min_idle / 1000, http_idle / 1000, repl_idle / 1000);
            crash_log_write("WDG_DEAD", buf);
            SERR_LOG(SERR_WDG_DEAD, min_idle / 1000);
            // Final task snapshot before WDOG1 fires — useful on next boot.
            crash_log_task_dump("WDG_DEAD");
        }
        // Stop kicking. Give WDOG1 its 30s to fire, then force-reset.
        for (int dead_wait = 0; dead_wait < 7; dead_wait++) {
            vTaskDelay(pdMS_TO_TICKS(5000));  // 7 × 5s = 35s total wait
            // Recovery check: if someone just used HTTP or REPL, cancel the dead state.
            uint32_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((t - g_http_last_activity) < kDeadThresholdMs ||
                (t - g_repl_last_activity) < kDeadThresholdMs) {
                // Interface came back — return to normal kicking
                g_network_healthy = true;
                consecutive_kicks = 0;
                break;
            }
        }
        // If still dead after 35s: WDOG1 should have fired. Force-reset as fallback.
        {
            uint32_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((t - g_http_last_activity) >= kDeadThresholdMs &&
                (t - g_repl_last_activity) >= kDeadThresholdMs) {
                crash_log_write("WDG_RESET", "Force reset after sustained dead state");
                vTaskDelay(pdMS_TO_TICKS(50));  // let crash log flush
                NVIC_SystemReset();
            }
        }
        consecutive_kicks = 0;
    }
}

// ===================== Recovery Mode (Anti-Brick) =====================

// Watchdog task for recovery mode: always kicks WDOG1.
// Recovery mode is minimal and stable — no risk of crashes.
static void RecoveryWatchdogTask(void* param) {
    (void)param;
    wdog_config_t wdog_config;
    WDOG_GetDefaultConfig(&wdog_config);
    wdog_config.timeoutValue  = (30 * 2) - 1; // 30-second timeout
    wdog_config.enableWdog    = true;
    wdog_config.workMode.enableWait  = true;
    wdog_config.workMode.enableStop  = false;
    wdog_config.workMode.enableDebug = false;
    wdog_config.enableInterrupt      = false;
    WDOG_Init(WDOG1, &wdog_config);
    g_hw_watchdog_enabled = true;
    for (;;) {
        WDOG_Refresh(WDOG1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Start the hardware watchdog task at HIGH PRIORITY
// Priority should be just below ISRs (configMAX_PRIORITIES - 2)
// This ensures the task runs even if other tasks are starved
static void start_network_watchdog(void) {
    xTaskCreate(CombinedWatchdogTask, "hw_wdog", 2048, nullptr, 
                configMAX_PRIORITIES - 2, &s_wdog_task_handle);
}

}  // anonymous namespace

// Override _write to capture all printf output
// This replaces the version in libs/base/console_m7.cc
extern "C" int _write(int handle, char* buffer, int size) {
    if ((handle != STDOUT_FILENO) && (handle != STDERR_FILENO)) {
        return -1;
    }

    // Silent kill-switch: when verbose=0 (e.g. while the detection pipeline
    // is running), drop every printf at the earliest point.  Nothing reaches
    // ConsoleM7::Write — this prevents ~200 lines/s of per-frame output from
    // saturating the CDC-ACM bulk-IN endpoint and stalling tx_task (which in
    // turn would block mp_repl's own prints → REPL appears dead).
    // Return size so printf's caller still thinks the write succeeded.
    if (!g_sentai_frame_verbose) return size;

    // Convert bare \n to \r\n for USB/UART terminals.
    char stack_buf[512];
    char* out = buffer;
    int out_len = size;

    bool needs_patch = false;
    for (int i = 0; i < size; ++i) {
        if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
            needs_patch = true;
            break;
        }
    }
    if (needs_patch) {
        int j = 0;
        int cap = (int)sizeof(stack_buf);
        for (int i = 0; i < size && j < cap - 1; ++i) {
            if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
                stack_buf[j++] = '\r';
            }
            if (j < cap) stack_buf[j++] = buffer[i];
        }
        out = stack_buf;
        out_len = j;
    }

    // Write to console
    coralmicro::ConsoleM7::GetSingleton()->Write(out, out_len);

    // Also capture to boot log if active
    boot_log_write(out, out_len);

    return size;
}

// Export for micropython_task.c to call when REPL starts
extern "C" void sentai_boot_log_stop(void) {
    boot_log_stop();
}

// Strong override of the weak default in libs/usb/usb_device_task.cc.
// The string is published as the USB Product descriptor so that
// `lsusb -v -d 1fc9:c0a1` (or just `lsusb`) shows the running build
// number — quick visual confirmation that flashtool succeeded.
extern "C" const char *sentai_build_version_string(void) {
    static char s[64];
    static bool init = false;
    if (!init) {
        snprintf(s, sizeof(s),
                 "autonomous.ro SentAI build #%d", BUILD_VERSION);
        init = true;
    }
    return s;
}

namespace coralmicro {

// External-linkage state — accessed by sentai_slow_bridge.cc (OCRAM)
uint8_t tensor_arena[8 * 1024 * 1024]
    __attribute__((aligned(16)))
    __attribute__((section(".sdram_bss,\"aw\",%nobits @")));
tflite::MicroInterpreter* g_interpreter = nullptr;
volatile bool g_tpu_ready = false;
std::vector<uint8_t>* g_model_data = nullptr;
std::shared_ptr<EdgeTpuContext> g_tpu_context;

namespace {

static TickType_t app_start_tick = 0;

void logf(const char* fmt, ...) {
  TickType_t now = xTaskGetTickCount();
  uint32_t elapsed_ms = (now - app_start_tick) * portTICK_PERIOD_MS;
  uint32_t mins = elapsed_ms / 60000;
  uint32_t secs = (elapsed_ms % 60000) / 1000;
  uint32_t ms   = elapsed_ms % 1000;
  printf("[%02lu:%02lu:%03lu] ", (unsigned long)mins, (unsigned long)secs, (unsigned long)ms);
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
}

void Main() {
  logf("SentAI MicroPython Runtime\r\n");

  // Open EdgeTPU on this task (not app_main) to avoid interfering with USB init
  coralmicro::PerformanceMode tpu_mode = coralmicro::PerformanceMode::kMax;
  g_tpu_context = EdgeTpuManager::GetSingleton()->OpenDevice(tpu_mode);
  if (!g_tpu_context) {
    logf("ERROR: Failed to get EdgeTpu context\r\n");
  } else {
    logf("Edge TPU opened (mode %d)\r\n", static_cast<int>(tpu_mode));
  }

  // Park this task forever - model loading happens from Python
  vTaskSuspend(NULL);
}

}  // namespace
}  // namespace coralmicro

// Enter RECOVERY MODE: minimal USB + REPL only, no application code.
// Called when boot_attempts >= 3 (consecutive crash/hang loop detected).
// USB CDC is already up (initialized by main_freertos before app_main),
// so the board is ALWAYS reflashable without button press.
// NEVER returns — parks in coralmicro::Main().
[[noreturn]] static void enter_recovery_mode(uint32_t attempts) {
    s_in_recovery_mode = true;

    // Minimal subsystem init — only what REPL needs
    sentai_health_init();
    boot_log_init();
    coralmicro::app_start_tick = xTaskGetTickCount();

    // Log to UART and boot log RAM buffer
    SERR_LOG(SERR_SYS_RECOVERY_MODE, attempts);
    coralmicro::logf("\r\n*** RECOVERY MODE *** boot loop after %lu attempts\r\n",
                     (unsigned long)attempts);
    coralmicro::logf("[recovery] last_progress=0x%02lX storage_attempts=%lu sram_magic=%08lX\r\n",
                     (unsigned long)::sentai_boot_prev_progress(),
                     (unsigned long)::sentai_boot_storage_attempts(),
                     (unsigned long)::sentai_boot_sram_magic());
    coralmicro::logf("Reflash: python3 scripts/flashtool.py -e sentai_runtime\r\n");

    // Mount LFS + open /log/boot.log (LFS already mounted by main_freertos)
    boot_log_fs_init();

    // Write recovery event to crash log for post-mortem analysis
    crash_log_write("RECOVERY_MODE", "Boot loop: 3+ consecutive crashes");

    // Start MicroPython REPL — user can inspect files, check status, reflash
    micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);

    // Start recovery watchdog (unconditionally kicks — recovery mode never crashes)
    xTaskCreate(RecoveryWatchdogTask, "rcv_wdog", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);

    // Mark boot complete:
    //   - Clears SRC_GPR boot counter → next boot (after reflash) starts fresh
    //   - Marks REPL as healthy for diag API
    sentai_health_boot_complete();

    // Override system mode to RECOVERY so sentai.diag.sys_mode() returns "RECOVERY"
    sentai_health_set_recovery_mode();

    coralmicro::logf("Recovery: REPL active, boot counter cleared.\r\n");

    // Park here — REPL task handles all user interaction
    coralmicro::Main();
    for (;;) {}  // unreachable — silences [[noreturn]] warning
}

// FreeRTOS hook: Called when stack overflow is detected
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                              char* pcTaskName) {
    (void)xTask;
    // Pack first 4 chars of task name into a uint32 for the breadcrumb
    uint32_t name_hash = 0;
    for (int i = 0; i < 4 && pcTaskName[i]; i++) {
        name_hash = (name_hash << 8) | (uint8_t)pcTaskName[i];
    }
    // Save crash breadcrumb: fault_addr = task name hash, lr = return addr
    sentai_fault_save(SERR_SYS_STACK_OVF,
                      /*pc=*/0,
                      /*lr=*/(uint32_t)__builtin_return_address(0),
                      /*cfsr=*/0,
                      /*fault_addr=*/name_hash,
                      /*uptime_ms=*/g_sentai_uptime_ms,
                      /*r0=*/0);
    printf("\r\n*** STACK OVERFLOW in task '%.16s' — resetting\r\n", pcTaskName);
    SERR_LOG(SERR_SYS_STACK_OVF, name_hash);
    // Reset immediately — watchdog/boot counter will handle repeated failures
    (*(volatile uint32_t*)0xE000ED0CUL) = (0x5FAUL << 16U) | (1UL << 2U);
    for (;;) {}
}

extern "C" void app_main(void* param) {
  (void)param;

  // Diagnostic checkpoint: app_main reached.
  sentai_boot_progress_mark(0x10);

  // === PHASE 8: BOOT LOOP DETECTION — MUST BE FIRST ===
  // SRC_GPR[kSRC_GeneralPurposeRegister1] survives warm reset (WDOG / SW reset)
  // but is cleared on cold boot (power cycle). Provides automatic anti-brick:
  //   - Increment counter on every boot attempt
  //   - If >= 3 consecutive failed boots → automatic RECOVERY MODE (no user action)
  //   - Counter cleared by sentai_health_boot_complete() after healthy boot
  // USB CDC is already up (init'd in main_freertos before vTaskStartScheduler),
  // so NXP USB ID is visible even in recovery mode — board can ALWAYS be reflashed.
  //
  // Also reads and immediately clears any crash breadcrumb left in GPR2-8 by a
  // previous fault handler (HardFault, stack overflow, malloc fail, assert).
  // The record is written to /log/crash.log below, after LFS mounts.

  // Read crash record from previous boot BEFORE clearing GPRs.
  sentai_crash_record_t s_prev_crash;
  bool s_has_prev_crash = sentai_fault_read(&s_prev_crash);
  sentai_fault_clear();  // clear immediately so fresh crashes get their own slot

  {
    uint32_t boot_attempts = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister1);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister1, boot_attempts + 1);
    SERR_LOG(SERR_SYS_BOOT_ATTEMPT, boot_attempts + 1);
    if (boot_attempts >= 3) {
      enter_recovery_mode(boot_attempts);  // [[noreturn]]
    }
  }

  // Initialize health monitoring FIRST
  sentai_health_init();
  
  // Initialize boot logging FIRST (before any printf)
  boot_log_init();
  sentai_boot_progress_mark(0x11);
  
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nSentAI build #%d (%s)\r\n", BUILD_VERSION, BUILD_TIMESTAMP);
  
  // Check and log previous reset reason
  coralmicro::ResetStats stats = coralmicro::ResetGetStats();
  if (stats.reset_reason != 0) {
    coralmicro::logf("Reset reason: 0x%08lX ", (unsigned long)stats.reset_reason);
    if (stats.reset_reason & kSRC_M7CoreWdogResetFlag) {
      coralmicro::logf("[WATCHDOG#%lu] ", (unsigned long)stats.watchdog_resets);
      // Log watchdog reset to crash log
      sentai_crash_log("BOOT_AFTER_WATCHDOG", "Previous reset was watchdog timeout");
    }
    if (stats.reset_reason & kSRC_M7CoreM7LockUpResetFlag) {
      coralmicro::logf("[LOCKUP#%lu] ", (unsigned long)stats.lockup_resets);
      sentai_crash_log("BOOT_AFTER_LOCKUP", "Previous reset was CPU lockup");
    }
    coralmicro::logf("\r\n");
  }
  
  // Initialize LFS and boot log file
  // LFS should be initialized by main_freertos before app_main
  boot_log_fs_init();
  sentai_boot_progress_mark(0x12);
  coralmicro::logf("Boot logging to /log/boot.log\r\n");

  // Log the boot-mode flag so we can diagnose if drive(1) actually wrote
  // the magic and survived the warm reset.  We log ALL persistence signals
  // (DTC-RAM struct + GPR9/10/11/12/15/16) to learn empirically which ones
  // survive NVIC_SystemReset on this silicon.  SRAM is authoritative.
  // (declarations now at file scope inside extern "C" block above)
  coralmicro::logf("[boot-mode] storage=%d attempts=%lu prev_progress=0x%02lX "
                   "sram=%08lX/%08lX GPR9=%08lX GPR10=%08lX GPR11=%08lX "
                   "GPR12=%08lX GPR15=%08lX GPR16=%08lX\r\n",
                   sentai_storage_mode_active(),
                   (unsigned long)sentai_boot_storage_attempts(),
                   (unsigned long)sentai_boot_prev_progress(),
                   (unsigned long)sentai_boot_sram_magic(),
                   (unsigned long)sentai_boot_sram_check(),
                   (unsigned long)sentai_boot_gpr_snap(0),
                   (unsigned long)sentai_boot_gpr_snap(1),
                   (unsigned long)sentai_boot_gpr_snap(2),
                   (unsigned long)sentai_boot_gpr_snap(3),
                   (unsigned long)sentai_boot_gpr_snap(4),
                   (unsigned long)sentai_boot_gpr_snap(5));

  // Write crash breadcrumb from previous boot to /log/crash.log (LFS now ready)
  if (s_has_prev_crash) {
    // Map error code to a short name
    const char* fault_name;
    switch (s_prev_crash.code) {
      case SERR_SYS_HARDFAULT:   fault_name = "HARDFAULT";   break;
      case SERR_SYS_STACK_OVF:   fault_name = "STACK_OVF";   break;
      case SERR_SYS_MALLOC_FAIL: fault_name = "MALLOC_FAIL"; break;
      case SERR_SYS_ASSERT:      fault_name = "ASSERT";       break;
      case SERR_SYS_BUS_FAULT:   fault_name = "BUS_FAULT";   break;
      case SERR_SYS_USAGE_FAULT: fault_name = "USAGE_FAULT"; break;
      case SERR_SYS_MEMMANAGE:   fault_name = "MEMMANAGE";   break;
      default:                   fault_name = "UNKNOWN_FAULT"; break;
    }
    // Decode CFSR into a short readable flag string
    char cfsr_flags[64] = "";
    uint32_t cf = s_prev_crash.cfsr;
    if (cf & 0x00000001UL) strncat(cfsr_flags, "IACCVIOL ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000002UL) strncat(cfsr_flags, "DACCVIOL ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000200UL) strncat(cfsr_flags, "INVSTATE ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000400UL) strncat(cfsr_flags, "INVPC ",     sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00001000UL) strncat(cfsr_flags, "PRECISERR ", sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00002000UL) strncat(cfsr_flags, "STKER ",     sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00004000UL) strncat(cfsr_flags, "LSPER ",     sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00010000UL) strncat(cfsr_flags, "UNALIGNED ", sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00020000UL) strncat(cfsr_flags, "DIVBYZERO ", sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x40000000UL) strncat(cfsr_flags, "VECTTBL ",   sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cfsr_flags[0] == '\0') strncat(cfsr_flags, "-", sizeof(cfsr_flags)-1);

    char details[220];
    snprintf(details, sizeof(details),
             "code=0x%04X PC=0x%08lX LR=0x%08lX "
             "CFSR=0x%08lX[%s] BFAR=0x%08lX up=%lums r0=0x%08lX",
             (unsigned)s_prev_crash.code,
             (unsigned long)s_prev_crash.pc,
             (unsigned long)s_prev_crash.lr,
             (unsigned long)s_prev_crash.cfsr,
             cfsr_flags,
             (unsigned long)s_prev_crash.fault_addr,
             (unsigned long)s_prev_crash.uptime_ms,
             (unsigned long)s_prev_crash.r0);
    sentai_crash_log(fault_name, details);
    coralmicro::logf("Prev crash recovered: %s %s\r\n", fault_name, details);
  }

  // User button task: waits for notification from ISR, then enters
  // storage mode by calling sentai_usb_drive_set(1).  That writes the
  // SRC_GPR15 magic and warm-resets the CPU; on the next boot the
  // composite descriptor exposes only ACM + MSC and /dev/sda comes up.
  // Pressing the physical RESET button (HW POR) clears SRC_GPR15 and
  // returns the board to default REPL+IP mode.
  static TaskHandle_t s_btn_task = nullptr;
  xTaskCreate([](void*) {
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      extern int sentai_usb_drive_set(int on);
      printf("\r\n[btn] User button -> storage mode\r\n");
      sentai_usb_drive_set(1);  // never returns (warm reset)
    }
  }, "btn_usb", configMINIMAL_STACK_SIZE * 4, nullptr,
     tskIDLE_PRIORITY + 1, &s_btn_task);

  // ISR only sends a notification — no flash/LFS work in interrupt context.
  coralmicro::GpioConfigureInterrupt(
      coralmicro::Gpio::kUserButton,
      coralmicro::GpioInterruptMode::kIntModeFalling,
      [&]() {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_btn_task, &woken);
        portYIELD_FROM_ISR(woken);
      },
      /*debounce_interval_us=*/200 * 1000);
  coralmicro::logf("User button -> sentai.usb.drive(1)  [enter STORAGE mode]\r\n");

  // ---- STORAGE mode short-circuit ----
  // In storage mode the firmware is a quiet host for /dev/sda only.
  // No REPL, no LFS task, no HTTP, no detection pipeline — the host has
  // exclusive NAND ownership and our app code must not touch it.
  // Watchdog is still started (so a hang in MSC handler still recovers)
  // and the user button does its usual thing (would re-enter storage,
  // which is harmless: same magic, same reset).
  if (sentai_storage_mode_active()) {
    sentai_boot_progress_mark(0x13);
    coralmicro::logf(
        "** STORAGE MODE active — REPL/IP disabled, /dev/sda is writable.\r\n"
        "** Press the RESET button OR send any input on /dev/ttyACM0\r\n"
        "** to return to default REPL+IP mode.\r\n");
    start_network_watchdog();
    sentai_health_boot_complete();
    // Reaching here means storage-mode boot survived all early init (USB
    // descriptor registration, MSC class init, FreeRTOS scheduler).  Clear
    // the crash-loop counter so the next drive(1) gets a fresh budget.
    sentai_storage_boot_succeeded();

    // In-band exit: poll the USB CDC-ACM RX buffer for ANY input byte and
    // call drive(0) when one arrives.  Lets host scripts return to default
    // mode without physical reset.  Polling cadence is 200 ms — instant
    // enough for a human-typed 'q' but doesn't churn the bus.
    extern int sentai_usb_drive_set(int on);
    char dummy[16];
    for (;;) {
      int n = coralmicro::ConsoleM7::GetSingleton()->Read(dummy, sizeof(dummy));
      if (n > 0) {
        coralmicro::logf("[storage] input received (%d byte%s) — exiting to default REPL+IP\r\n",
                         n, n == 1 ? "" : "s");
        vTaskDelay(pdMS_TO_TICKS(80));  // drain printf to host
        sentai_usb_drive_set(0);  // never returns (warm reset)
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }

  // Launch MicroPython REPL task (interactive Python over serial)
  coralmicro::logf("Starting MicroPython REPL task...\r\n");
  micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);

  // Start combined watchdog (monitors HTTP + REPL, auto-resets if both dead)
  start_network_watchdog();
  coralmicro::logf("Watchdog started (auto-reset if HTTP+REPL dead for 2min)\r\n");

  // Mark boot complete — transitions from BOOTING to NORMAL (if healthy)
  sentai_health_boot_complete();
  coralmicro::logf("Boot complete, system mode: %s\r\n",
      sentai_health_system_mode() == SYS_MODE_NORMAL ? "NORMAL" :
      sentai_health_system_mode() == SYS_MODE_DEGRADED ? "DEGRADED" :
      sentai_health_system_mode() == SYS_MODE_SAFE ? "SAFE" : "?");

  coralmicro::Main();
  // Main() parks itself with vTaskSuspend - never returns
}

// ===================== C bridge for MicroPython =====================
// Called from modsentai.c (C code) - need extern "C" linkage

// Forward declaration (defined after sentai_tpu_invoke).
extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);

// sentai_load_model, sentai_load_image → moved to sentai_slow_bridge.cc (OCRAM)

// Internal invoke — no pipeline guard.  Called by detection_task.cc.
extern "C" int sentai_tpu_invoke_internal(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  TickType_t t0 = xTaskGetTickCount();
  if (coralmicro::g_interpreter->Invoke() != kTfLiteOk) return -2;
  TickType_t t1 = xTaskGetTickCount();
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

// Public invoke — blocks while detection pipeline owns the TPU.
extern "C" int sentai_tpu_invoke(void) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns TPU
  return sentai_tpu_invoke_internal();
}

// Shared uint8→int8 quantization (in-place).
// Fast path zp==-128: XOR 0x80, 8× unrolled with prefetch.
// General path: clamp(pixel + zp, -128, 127).
extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp) {
  if (zp == -128) {
    uint32_t* p32 = reinterpret_cast<uint32_t*>(buf);
    int n32 = count / 4;
    constexpr int kPre = 64;  // prefetch 256 bytes ahead
    for (int i = 0; i < n32; i += 8) {
      if (i + kPre < n32)
        __builtin_prefetch(&p32[i + kPre], 1, 0);
      p32[i]   ^= 0x80808080u;
      p32[i+1] ^= 0x80808080u;
      p32[i+2] ^= 0x80808080u;
      p32[i+3] ^= 0x80808080u;
      p32[i+4] ^= 0x80808080u;
      p32[i+5] ^= 0x80808080u;
      p32[i+6] ^= 0x80808080u;
      p32[i+7] ^= 0x80808080u;
    }
    for (int i = (n32 & ~7) * 4; i < count; i++)
      reinterpret_cast<int8_t*>(buf)[i] =
          static_cast<int8_t>(buf[i] ^ 0x80u);
  } else {
    int8_t* dst = reinterpret_cast<int8_t*>(buf);
    for (int i = 0; i < count; i++) {
      int v = static_cast<int>(buf[i]) + zp;
      if (v < -128) v = -128;
      if (v >  127) v =  127;
      dst[i] = static_cast<int8_t>(v);
    }
  }
}

extern "C" int sentai_tpu_is_ready(void) {
  return coralmicro::g_tpu_ready ? 1 : 0;
}

extern "C" int sentai_tpu_num_outputs(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  return (int)coralmicro::g_interpreter->outputs().size();
}

extern "C" int sentai_tpu_get_output_size(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tpu_get_output_data(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return NULL;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return NULL;
  return coralmicro::g_interpreter->output_tensor(idx)->data.data;
}

extern "C" int sentai_tpu_get_output_num_dims(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return coralmicro::g_interpreter->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tpu_get_output_dim(int idx, int dim) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  auto* t = coralmicro::g_interpreter->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tpu_get_output_type(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return -1;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->type;
}

// Get input tensor quantization: scale (float) and zero_point.
// Returns 0 on success, -1 if not ready.
extern "C" int sentai_tpu_input_quant(float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input) return -1;
  *scale = input->params.scale;
  *zero_point = input->params.zero_point;
  return 0;
}

// Get output tensor quantization: scale (float) and zero_point.
// Returns 0 on success, -1 if not ready/invalid idx.
extern "C" int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return -1;
  auto* t = coralmicro::g_interpreter->output_tensor(idx);
  if (!t) return -1;
  *scale = t->params.scale;
  *zero_point = t->params.zero_point;
  return 0;
}

// Get input tensor type (TfLiteType enum: 9=int8, 3=uint8, 1=float32)
extern "C" int sentai_tpu_input_type(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input) return -1;
  return (int)input->type;
}

// ---------------------------------------------------------------------------
// YOLO NMS post-processing
// Output tensor expected shape [1, C, N] where C = 4 + num_classes, N = candidates
// bbox format: cx, cy, w, h (normalized 0-1 or pixel coords, auto-detected)
// Returns 0 on success.  out_count receives number of detections written.
// Each detection in out_buf: [x1, y1, x2, y2, conf_permil, class_id] (6 × int16)
// Coordinates are in model input pixel space (0 .. input_w/h).
// ---------------------------------------------------------------------------

namespace {
struct YoloCandidate {
  float x1, y1, x2, y2;
  float score;
  int16_t class_id;
};
static constexpr int kMaxNmsCandidates = 512;
static YoloCandidate g_nms_cand[kMaxNmsCandidates]
    __attribute__((section(".sdram_bss")));
static bool g_nms_sup[kMaxNmsCandidates];

// ---------------------------------------------------------------------------
// Draw buffer — stores last to_tensor RGB frame (before int8 quant)
// Max 640×640×3 = 1.2 MB in SDRAM
// Only populated when g_draw_capture_pending is set (by draw() call).
// This avoids a ~2-3ms memcpy on every frame in the critical inference path.
// ---------------------------------------------------------------------------
static constexpr int kMaxDrawPixels = 640 * 640 * 3;
static uint8_t g_draw_rgb[kMaxDrawPixels]
    __attribute__((section(".sdram_bss")));
static int g_draw_w = 0, g_draw_h = 0;
static volatile bool g_draw_capture_pending = false;

// COCO 80 class names
static const char* const kCocoNames[80] = {
  "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
  "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
  "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
  "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
  "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
  "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
  "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
  "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
  "remote","keyboard","cell phone","microwave","oven","toaster","sink",
  "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
  "toothbrush"
};

// 10 distinct box colors (R, G, B)
static const uint8_t kBoxColors[10][3] = {
  {255,  56,  56}, { 56, 255,  56}, { 56,  56, 255},
  {255, 255,  56}, {255,  56, 255}, { 56, 255, 255},
  {255, 128,   0}, {  0, 128, 255}, {255,   0, 128},
  {128, 255,   0}
};

// 5×7 bitmap font — printable ASCII 32..126 (95 glyphs)
// Each glyph = 5 bytes (columns). Each byte: bit0 = top row, bit6 = bottom.
static const uint8_t kFont5x7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
  {0x00,0x00,0x5F,0x00,0x00}, // 33 !
  {0x00,0x07,0x00,0x07,0x00}, // 34 "
  {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
  {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
  {0x23,0x13,0x08,0x64,0x62}, // 37 %
  {0x36,0x49,0x55,0x22,0x50}, // 38 &
  {0x00,0x05,0x03,0x00,0x00}, // 39 '
  {0x00,0x1C,0x22,0x41,0x00}, // 40 (
  {0x00,0x41,0x22,0x1C,0x00}, // 41 )
  {0x08,0x2A,0x1C,0x2A,0x08}, // 42 *
  {0x08,0x08,0x3E,0x08,0x08}, // 43 +
  {0x00,0x50,0x30,0x00,0x00}, // 44 ,
  {0x08,0x08,0x08,0x08,0x08}, // 45 -
  {0x00,0x60,0x60,0x00,0x00}, // 46 .
  {0x20,0x10,0x08,0x04,0x02}, // 47 /
  {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
  {0x00,0x42,0x7F,0x40,0x00}, // 49 1
  {0x42,0x61,0x51,0x49,0x46}, // 50 2
  {0x21,0x41,0x45,0x4B,0x31}, // 51 3
  {0x18,0x14,0x12,0x7F,0x10}, // 52 4
  {0x27,0x45,0x45,0x45,0x39}, // 53 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
  {0x01,0x71,0x09,0x05,0x03}, // 55 7
  {0x36,0x49,0x49,0x49,0x36}, // 56 8
  {0x06,0x49,0x49,0x29,0x1E}, // 57 9
  {0x00,0x36,0x36,0x00,0x00}, // 58 :
  {0x00,0x56,0x36,0x00,0x00}, // 59 ;
  {0x00,0x08,0x14,0x22,0x41}, // 60 <
  {0x14,0x14,0x14,0x14,0x14}, // 61 =
  {0x41,0x22,0x14,0x08,0x00}, // 62 >
  {0x02,0x01,0x51,0x09,0x06}, // 63 ?
  {0x32,0x49,0x79,0x41,0x3E}, // 64 @
  {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
  {0x7F,0x49,0x49,0x49,0x36}, // 66 B
  {0x3E,0x41,0x41,0x41,0x22}, // 67 C
  {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
  {0x7F,0x49,0x49,0x49,0x41}, // 69 E
  {0x7F,0x09,0x09,0x01,0x01}, // 70 F
  {0x3E,0x41,0x41,0x51,0x32}, // 71 G
  {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
  {0x00,0x41,0x7F,0x41,0x00}, // 73 I
  {0x20,0x40,0x41,0x3F,0x01}, // 74 J
  {0x7F,0x08,0x14,0x22,0x41}, // 75 K
  {0x7F,0x40,0x40,0x40,0x40}, // 76 L
  {0x7F,0x02,0x04,0x02,0x7F}, // 77 M
  {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
  {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
  {0x7F,0x09,0x09,0x09,0x06}, // 80 P
  {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
  {0x7F,0x09,0x19,0x29,0x46}, // 82 R
  {0x46,0x49,0x49,0x49,0x31}, // 83 S
  {0x01,0x01,0x7F,0x01,0x01}, // 84 T
  {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
  {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
  {0x7F,0x20,0x18,0x20,0x7F}, // 87 W
  {0x63,0x14,0x08,0x14,0x63}, // 88 X
  {0x03,0x04,0x78,0x04,0x03}, // 89 Y
  {0x61,0x51,0x49,0x45,0x43}, // 90 Z
  {0x00,0x00,0x7F,0x41,0x41}, // 91 [
  {0x02,0x04,0x08,0x10,0x20}, // 92 backslash
  {0x41,0x41,0x7F,0x00,0x00}, // 93 ]
  {0x04,0x02,0x01,0x02,0x04}, // 94 ^
  {0x40,0x40,0x40,0x40,0x40}, // 95 _
  {0x00,0x01,0x02,0x04,0x00}, // 96 `
  {0x20,0x54,0x54,0x54,0x78}, // 97 a
  {0x7F,0x48,0x44,0x44,0x38}, // 98 b
  {0x38,0x44,0x44,0x44,0x20}, // 99 c
  {0x38,0x44,0x44,0x48,0x7F}, //100 d
  {0x38,0x54,0x54,0x54,0x18}, //101 e
  {0x08,0x7E,0x09,0x01,0x02}, //102 f
  {0x08,0x14,0x54,0x54,0x3C}, //103 g
  {0x7F,0x08,0x04,0x04,0x78}, //104 h
  {0x00,0x44,0x7D,0x40,0x00}, //105 i
  {0x20,0x40,0x44,0x3D,0x00}, //106 j
  {0x00,0x7F,0x10,0x28,0x44}, //107 k
  {0x00,0x41,0x7F,0x40,0x00}, //108 l
  {0x7C,0x04,0x18,0x04,0x78}, //109 m
  {0x7C,0x08,0x04,0x04,0x78}, //110 n
  {0x38,0x44,0x44,0x44,0x38}, //111 o
  {0x7C,0x14,0x14,0x14,0x08}, //112 p
  {0x08,0x14,0x14,0x18,0x7C}, //113 q
  {0x7C,0x08,0x04,0x04,0x08}, //114 r
  {0x48,0x54,0x54,0x54,0x20}, //115 s
  {0x04,0x3F,0x44,0x40,0x20}, //116 t
  {0x3C,0x40,0x40,0x20,0x7C}, //117 u
  {0x1C,0x20,0x40,0x20,0x1C}, //118 v
  {0x3C,0x40,0x30,0x40,0x3C}, //119 w
  {0x44,0x28,0x10,0x28,0x44}, //120 x
  {0x0C,0x50,0x50,0x50,0x3C}, //121 y
  {0x44,0x64,0x54,0x4C,0x44}, //122 z
  {0x00,0x08,0x36,0x41,0x00}, //123 {
  {0x00,0x00,0x7F,0x00,0x00}, //124 |
  {0x00,0x41,0x36,0x08,0x00}, //125 }
  {0x10,0x08,0x08,0x10,0x08}, //126 ~
};

// Drawing helpers (operate on RGB888 buffer)
static inline void draw_pixel(uint8_t* buf, int bw, int bh,
                               int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  if (x >= 0 && x < bw && y >= 0 && y < bh) {
    int off = (y * bw + x) * 3;
    buf[off] = r; buf[off+1] = g; buf[off+2] = b;
  }
}

static void draw_rect(uint8_t* buf, int bw, int bh,
                       int x1, int y1, int x2, int y2, int thick,
                       uint8_t r, uint8_t g, uint8_t b) {
  for (int t = 0; t < thick; t++) {
    for (int x = x1 - t; x <= x2 + t; x++) {
      draw_pixel(buf, bw, bh, x, y1 - t, r, g, b);
      draw_pixel(buf, bw, bh, x, y2 + t, r, g, b);
    }
    for (int y = y1 - t + 1; y < y2 + t; y++) {
      draw_pixel(buf, bw, bh, x1 - t, y, r, g, b);
      draw_pixel(buf, bw, bh, x2 + t, y, r, g, b);
    }
  }
}

static void draw_filled_rect(uint8_t* buf, int bw, int bh,
                               int x1, int y1, int x2, int y2,
                               uint8_t r, uint8_t g, uint8_t b) {
  for (int y = (y1 < 0 ? 0 : y1); y <= y2 && y < bh; y++)
    for (int x = (x1 < 0 ? 0 : x1); x <= x2 && x < bw; x++) {
      int off = (y * bw + x) * 3;
      buf[off] = r; buf[off+1] = g; buf[off+2] = b;
    }
}

static void draw_char(uint8_t* buf, int bw, int bh, int cx, int cy, char ch,
                       uint8_t r, uint8_t g, uint8_t b) {
  int idx = (int)ch - 32;
  if (idx < 0 || idx >= 95) idx = '?' - 32;
  const uint8_t* glyph = kFont5x7[idx];
  for (int col = 0; col < 5; col++) {
    uint8_t bits = glyph[col];
    for (int row = 0; row < 7; row++) {
      if (bits & (1 << row))
        draw_pixel(buf, bw, bh, cx + col, cy + row, r, g, b);
    }
  }
}

static void draw_string(uint8_t* buf, int bw, int bh, int sx, int sy,
                          const char* str, uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; str[i]; i++)
    draw_char(buf, bw, bh, sx + i * 6, sy, str[i], r, g, b);
}

}  // namespace

// ---------------------------------------------------------------------------
// YOLO output layout auto-detection
// ---------------------------------------------------------------------------
// Infers (layout, num_classes, num_anchors) from the output tensor's shape
// dims alone.  Two pre-NMS layouts seen in practice:
//
//   YOLO_V5_LIKE    [1, N, 5+C]   row = [cx, cy, w, h, obj, cls_0..cls_C-1]
//                                 Covers classic YOLOv5 and YOLOv5-enhanced
//                                 variants.  Single-class is the C=1
//                                 degenerate: shape [1, N, 6] with the last
//                                 column being the single class confidence.
//
//   YOLO_V8         [1, 4+C, N]   Ultralytics YOLOv8 family (incl. yolo26n).
//                                 Transposed: bbox rows first, class rows
//                                 after.  No separate objectness column.
//                                 COCO yolo26n.edgetpu_1: C=80, [1, 84, 2100].
//
//   UNKNOWN                       shape doesn't match either pattern
//
// Heuristic: the "anchors" dimension is always far larger than 4+C (hundreds
// to thousands), so whichever of dims[1]/dims[2] is larger is N; the smaller
// one is either 5+C (v5-like) or 4+C (v8).  v5-like has anchors in dims[1]
// (anchors-first), v8 has anchors in dims[2] (anchors-last).

enum class YoloLayout : uint8_t {
    kUnknown   = 0,
    kV5Like    = 1,  // [1, N, 5+C]   — YOLOv5 / YOLOv5-enhanced (our 1-class model)
    kV8        = 2,  // [1, 4+C, N]   — YOLOv8 family (yolo26n COCO)
};

struct YoloInfo {
    YoloLayout layout;
    int num_classes;
    int num_anchors;
};

static YoloInfo yolo_infer_info(const TfLiteTensor* output) {
    YoloInfo o = {YoloLayout::kUnknown, 0, 0};
    if (!output || output->dims->size != 3 || output->dims->data[0] != 1) return o;
    int d1 = output->dims->data[1];
    int d2 = output->dims->data[2];
    // The anchor dimension is the LARGER one in every real yolo export —
    // typical anchor counts range from ~500 to ~25 000, while 4+C or 5+C
    // is at most ~100 (COCO's 80 classes).  So:
    //   v5-like [1, N, 5+C]: dims[1] = N (large), dims[2] = 5+C (small)
    //     → d1 > d2
    //   v8      [1, 4+C, N]: dims[1] = 4+C (small), dims[2] = N (large)
    //     → d2 > d1
    if (d1 > d2 && d2 > 4) {
        // v5-like.  rows = 5+C (includes a separate objectness column).
        int C = d2 - 5;
        if (C >= 1) {
            o.layout      = YoloLayout::kV5Like;
            o.num_classes = C;
            o.num_anchors = d1;
        }
    } else if (d2 > d1 && d1 > 4) {
        // v8.  rows = 4+C (no objectness column, class conf directly).
        int C = d1 - 4;
        if (C >= 1) {
            o.layout      = YoloLayout::kV8;
            o.num_classes = C;
            o.num_anchors = d2;
        }
    }
    return o;
}

// Public C bridge — lets MicroPython introspect the loaded model without
// touching tensor internals.  Returns 0 on success with values filled in;
// negative error otherwise.  Callers may pass nullptr for any out param.
extern "C" int sentai_tpu_output_yolo_info(int* layout_out,
                                           int* num_classes_out,
                                           int* num_anchors_out) {
    if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
    const TfLiteTensor* out = coralmicro::g_interpreter->output_tensor(0);
    YoloInfo info = yolo_infer_info(out);
    if (layout_out)      *layout_out      = (int)info.layout;
    if (num_classes_out) *num_classes_out = info.num_classes;
    if (num_anchors_out) *num_anchors_out = info.num_anchors;
    return (info.layout == YoloLayout::kUnknown) ? -2 : 0;
}

extern "C" int sentai_tpu_detect(int conf_permil, int iou_permil,
                                 int max_dets,
                                 int16_t* out_buf, int* out_count) {
  using namespace coralmicro;
  *out_count = 0;
  if (!g_tpu_ready || !g_interpreter) return -1;

  auto* output = g_interpreter->output_tensor(0);
  if (!output || output->dims->size != 3) return -2;

  YoloInfo info = yolo_infer_info(output);
  if (info.layout == YoloLayout::kUnknown) return -3;

  auto* input = g_interpreter->input_tensor(0);
  int in_h = input->dims->data[1];
  int in_w = input->dims->data[2];

  float scale = output->params.scale;
  int zp = output->params.zero_point;

  float conf_thr = conf_permil / 1000.0f;
  float iou_thr  = iou_permil  / 1000.0f;

  int num_cand = 0;
  float max_coord = 0.0f;

  const int N = info.num_anchors;
  const int C = info.num_classes;

  if (info.layout == YoloLayout::kV5Like) {
    // [1, N, 5+C] — rows contiguous in memory.  Row layout:
    //   bytes 0..3   = cx, cy, w, h
    //   byte 4       = objectness
    //   bytes 5..4+C = per-class conf (take argmax)
    // Read as uint8 when the tensor is uint8-quantised, int8 otherwise.
    const int row_bytes = 5 + C;
    const bool is_u8 = (output->type == kTfLiteUInt8);
    const uint8_t* u = reinterpret_cast<const uint8_t*>(output->data.data);
    const int8_t*  s = reinterpret_cast<const int8_t *>(output->data.data);
    for (int j = 0; j < N && num_cand < kMaxNmsCandidates; j++) {
      int base = j * row_bytes;
      auto q = [&](int k) -> float {
          int raw = is_u8 ? (int)u[base + k] : (int)s[base + k];
          return scale * (raw - zp);
      };
      float obj = q(4);
      float best_cls_conf = -1e9f;
      int   best_cls_id   = 0;
      for (int c = 0; c < C; c++) {
          float v = q(5 + c);
          if (v > best_cls_conf) { best_cls_conf = v; best_cls_id = c; }
      }
      float cf = obj * best_cls_conf;
      if (cf < conf_thr) continue;

      float cx = q(0), cy = q(1), bw = q(2), bh = q(3);
      float x1 = cx - bw * 0.5f, y1 = cy - bh * 0.5f;
      float x2 = cx + bw * 0.5f, y2 = cy + bh * 0.5f;
      if (x2 > max_coord) max_coord = x2;
      if (y2 > max_coord) max_coord = y2;
      g_nms_cand[num_cand++] = {x1, y1, x2, y2, cf, (int16_t)best_cls_id};
    }
  } else {
    // kV8: [1, 4+C, N] — column j is one anchor; row c across all j.
    const int8_t* data = reinterpret_cast<const int8_t*>(output->data.data);
    const int CC = 4 + C;  // rows
    for (int j = 0; j < N && num_cand < kMaxNmsCandidates; j++) {
      float best_score = -1e9f;
      int best_cls = 0;
      for (int c = 4; c < CC; c++) {
        float v = scale * ((int)data[c * N + j] - zp);
        if (v > best_score) { best_score = v; best_cls = c - 4; }
      }
      if (best_score < conf_thr) continue;

      float cx = scale * ((int)data[0 * N + j] - zp);
      float cy = scale * ((int)data[1 * N + j] - zp);
      float bw = scale * ((int)data[2 * N + j] - zp);
      float bh = scale * ((int)data[3 * N + j] - zp);
      float x1 = cx - bw * 0.5f, y1 = cy - bh * 0.5f;
      float x2 = cx + bw * 0.5f, y2 = cy + bh * 0.5f;
      if (x2 > max_coord) max_coord = x2;
      if (y2 > max_coord) max_coord = y2;
      g_nms_cand[num_cand++] = {x1, y1, x2, y2, best_score, (int16_t)best_cls};
    }
  }

  if (num_cand == 0) return 0;

  // Auto-detect normalized (0-1) vs pixel-space coords
  // If the largest coordinate < 2.0 then values are normalized → scale to input dims
  float coord_sx = (max_coord < 2.0f) ? (float)in_w : 1.0f;
  float coord_sy = (max_coord < 2.0f) ? (float)in_h : 1.0f;

  // Phase 2: insertion sort by score descending (small N, stack-friendly)
  for (int i = 1; i < num_cand; i++) {
    YoloCandidate key = g_nms_cand[i];
    int j = i - 1;
    while (j >= 0 && g_nms_cand[j].score < key.score) {
      g_nms_cand[j + 1] = g_nms_cand[j]; j--;
    }
    g_nms_cand[j + 1] = key;
  }

  // Phase 3: greedy NMS (class-aware)
  memset(g_nms_sup, 0, sizeof(bool) * num_cand);
  int count = 0;

  for (int i = 0; i < num_cand && count < max_dets; i++) {
    if (g_nms_sup[i]) continue;
    auto& d = g_nms_cand[i];

    // Scale & clamp to model input pixel space
    float sx1 = d.x1 * coord_sx; if (sx1 < 0) sx1 = 0;
    float sy1 = d.y1 * coord_sy; if (sy1 < 0) sy1 = 0;
    float sx2 = d.x2 * coord_sx; if (sx2 > in_w) sx2 = (float)in_w;
    float sy2 = d.y2 * coord_sy; if (sy2 > in_h) sy2 = (float)in_h;

    out_buf[count * 6 + 0] = (int16_t)(sx1 + 0.5f);
    out_buf[count * 6 + 1] = (int16_t)(sy1 + 0.5f);
    out_buf[count * 6 + 2] = (int16_t)(sx2 + 0.5f);
    out_buf[count * 6 + 3] = (int16_t)(sy2 + 0.5f);
    out_buf[count * 6 + 4] = (int16_t)(d.score * 1000.0f + 0.5f);
    out_buf[count * 6 + 5] = d.class_id;
    count++;

    // Suppress overlapping detections of the same class
    float area_i = (sx2 - sx1) * (sy2 - sy1);
    for (int j = i + 1; j < num_cand; j++) {
      if (g_nms_sup[j]) continue;
      if (g_nms_cand[j].class_id != d.class_id) continue;

      float jx1 = g_nms_cand[j].x1 * coord_sx;
      float jy1 = g_nms_cand[j].y1 * coord_sy;
      float jx2 = g_nms_cand[j].x2 * coord_sx;
      float jy2 = g_nms_cand[j].y2 * coord_sy;

      float xx1 = (sx1 > jx1) ? sx1 : jx1;
      float yy1 = (sy1 > jy1) ? sy1 : jy1;
      float xx2 = (sx2 < jx2) ? sx2 : jx2;
      float yy2 = (sy2 < jy2) ? sy2 : jy2;
      float iw  = (xx2 > xx1) ? (xx2 - xx1) : 0;
      float ih  = (yy2 > yy1) ? (yy2 - yy1) : 0;
      float inter = iw * ih;
      float area_j = (jx2 - jx1) * (jy2 - jy1);
      float iou = inter / (area_i + area_j - inter + 1e-6f);
      if (iou > iou_thr) g_nms_sup[j] = true;
    }
  }

  *out_count = count;
  if (g_sentai_frame_verbose) {
    printf("NMS: %d/%d candidates, %d detections (conf>%d%% iou>%d%%)\r\n",
           num_cand, N, count, conf_permil / 10, iou_permil / 10);
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Draw bounding boxes + labels on the last to_tensor() RGB frame, save JPEG.
// dets: flat array of n_dets × 6 int16: [x1,y1,x2,y2,conf_permil,class_id]
// ---------------------------------------------------------------------------
extern "C" int sentai_tpu_draw(const char* path,
                                const int16_t* dets, int n_dets,
                                int quality) {
  // Always request capture for the next cam_to_tensor frame.
  g_draw_capture_pending = true;
  if (g_draw_w == 0 || g_draw_h == 0) return -1;  // no frame saved yet

  int sz = g_draw_w * g_draw_h * 3;

  // Work on a copy so original is preserved for multiple draw() calls
  uint8_t* rgb = (uint8_t*)malloc(sz);
  if (!rgb) return -2;
  memcpy(rgb, g_draw_rgb, sz);

  for (int i = 0; i < n_dets; i++) {
    int x1   = dets[i*6 + 0];
    int y1   = dets[i*6 + 1];
    int x2   = dets[i*6 + 2];
    int y2   = dets[i*6 + 3];
    int conf = dets[i*6 + 4];
    int cls  = dets[i*6 + 5];

    int ci = cls % 10;
    uint8_t cr = kBoxColors[ci][0];
    uint8_t cg = kBoxColors[ci][1];
    uint8_t cb = kBoxColors[ci][2];

    // Draw bounding box (2px thick)
    draw_rect(rgb, g_draw_w, g_draw_h, x1, y1, x2, y2, 2, cr, cg, cb);

    // Build label: "class_name NN%"
    char label[40];
    const char* name = (cls >= 0 && cls < 80) ? kCocoNames[cls] : "?";
    snprintf(label, sizeof(label), "%s %d%%", name, conf / 10);

    int lw = (int)strlen(label) * 6 + 3;
    int lh = 10;
    int ly = (y1 - lh - 1 >= 0) ? y1 - lh - 1 : y1;  // above box, or inside

    // Colored background for label
    draw_filled_rect(rgb, g_draw_w, g_draw_h, x1, ly, x1 + lw, ly + lh,
                     cr, cg, cb);
    // Black text on colored background
    draw_string(rgb, g_draw_w, g_draw_h, x1 + 2, ly + 2, label, 0, 0, 0);
  }

  // JPEG compress and save
  int jpeg_buf_size = sz;
  if (jpeg_buf_size < 64 * 1024) jpeg_buf_size = 64 * 1024;
  uint8_t* jpeg_buf = (uint8_t*)malloc(jpeg_buf_size);
  int rc = -3;
  if (jpeg_buf) {
    unsigned long jpeg_size = coralmicro::JpegCompressRgb(
        rgb, g_draw_w, g_draw_h, quality,
        jpeg_buf, (unsigned long)jpeg_buf_size);
    if (jpeg_size > 0) {
      std::string data((const char*)jpeg_buf, jpeg_size);
      if (coralmicro::LfsUserWriteFile(path, data)) {
        printf("Draw: %dx%d saved %s (%lu bytes, %d dets)\r\n",
               g_draw_w, g_draw_h, path, jpeg_size, n_dets);
        rc = 0;
      }
    }
    free(jpeg_buf);
  }
  free(rgb);
  return rc;
}

// sentai_save_output → moved to sentai_slow_bridge.cc (OCRAM)

// ===================== TFL bridge moved to sentai_tfl_bridge.cc =============
// All TFL extern "C" functions are in sentai_tfl_bridge.cc, which the linker
// script places in OCRAM (.sentai_slow) to keep ITCM (.text) within budget.

// ===================== Detection pipeline wrappers ========================
// Thin C bridges so detection_task.cc can access internal functions.

// Forward declarations (defined later in this file).
static int pxp_scale_xrgb_to_rgb(const uint8_t* src, int src_w, int src_h,
                                  uint8_t* dst, int dst_w, int dst_h);
static int sentai_cam_get_raw_with_recovery(uint8_t** raw_out);

extern "C" int sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                                 uint8_t* dst, int dw, int dh) {
  return pxp_scale_xrgb_to_rgb(src, sw, sh, dst, dw, dh);
}

extern "C" int sentai_get_tensor_info(int* w, int* h, int* ch,
                                       uint8_t** buf, int* type, int* zp) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -2;
  *h  = input->dims->data[1];
  *w  = input->dims->data[2];
  *ch = input->dims->data[3];
  *buf = tflite::GetTensorData<uint8_t>(input);
  *type = static_cast<int>(input->type);
  *zp   = input->params.zero_point;
  return 0;
}

extern "C" int sentai_cam_is_initialized(void);

extern "C" int sentai_cam_grab_latest(uint8_t** raw) {
  return sentai_cam_get_raw_with_recovery(raw);
}

extern "C" void sentai_cam_return_raw(int idx) {
  coralmicro::CameraTask::GetSingleton()->ReturnRawFrame(idx);
}

// ===================== Camera bridge for MicroPython =====================

static volatile bool g_cam_initialized = false;
static int g_cam_width = DEMO_CAMERA_WIDTH;
static int g_cam_height = DEMO_CAMERA_HEIGHT;
// Logical camera id — 0=front, 1=back.  Written by sentai_cam_switch
// (task context) when arming a switch, and by the CSI ISR when the
// auto-alternate scheduler (future) triggers a flip.  Single 32-bit
// aligned write → atomic on Cortex-M7; no lock needed.
volatile int g_cam_current_id = 0;
// g_camera_frame_seq snapshot at MUX switch time.  Now written by the CSI
// ISR in libs/camera/camera_support.c immediately after the atomic GPIO
// flip that selects the new sensor.  Because the ISR fires when a DMA
// buffer has just completed (we are in VBLANK — the MIPI lane is idle
// until the next SOF), the flip lands in a dead window and the next
// DMA buffer is guaranteed to contain pixels from the new sensor only.
// This is the glitch-free path that removes the mid-buffer seam seen
// visually in E17 threshold=1 frames (paper/cam_switch.md).
volatile uint32_t g_cam_switch_seq = 0;
// Set by the CSI ISR when it consumes a pending MUX flip, cleared by
// the first subsequent get-raw call in sentai_cam_get_raw_with_recovery
// after the drain threshold is reached.
volatile bool g_cam_switch_pending = false;
// Armed by sentai_cam_switch (task context, 0 or 1) and consumed by the
// CSI ISR on the next EOF.  -1 means "no switch armed".  Sentinel is the
// ONLY valid idle value; any 0/1 observed is a request to flip.  Atomic
// 32-bit write.  A bounded timeout in sentai_cam_switch falls back to
// the legacy synchronous flip path if the ISR does not consume the arm
// within ~3 frame intervals (CSI stuck, no EOF firing).
volatile int g_cam_pending_mux_id = -1;

// Stateless ratio-alternate scheduler.  When both quotas are > 0, the
// CSI ISR decides which camera should own frame N via a pure modulo of
// the monotonic frame counter (seq % (ratio_a + ratio_b)) < ratio_a →
// cam0, else cam1.  No mutable counter state in ISR context; the policy
// is a function of (seq, ratio_a, ratio_b) only.  Useful for asymmetric
// rates: e.g. (3, 1) gives cam0 at 22.5 fps, cam1 at 7.5 fps at a 30 fps
// sensor, with glitch-free flips (the flip happens in the EOF ISR's
// VBLANK window).  Either field zero disables auto-alternation; manual
// `sentai.camera.select()` always wins because it writes
// g_cam_pending_mux_id and the ISR checks that slot before the
// scheduler arms anything.
volatile uint32_t g_cam_ratio_a = 0;  // frames to stay on cam0
volatile uint32_t g_cam_ratio_b = 0;  // frames to stay on cam1

// Post-switch drain threshold (number of fresh ISR frames required after a
// MUX flip before a frame is considered clean).  Default 2 (one mixed
// post-flip frame + one fully-new frame).  Runtime-settable via
// `sentai.camera.switch_drain(n)` for A/B measurement — see E17
// experiment.  Bounded to [1, 10] at the setter to preserve
// analyzability: threshold=0 would return a pre-flip frame, threshold
// beyond 10 is not a sensible operating point on this 15 FPS pipeline.
static volatile uint32_t g_cam_switch_drain_threshold = 2;

extern "C" uint32_t sentai_cam_switch_drain_get(void) {
  return g_cam_switch_drain_threshold;
}

extern "C" int sentai_cam_switch_drain_set(uint32_t n) {
  if (n < 1 || n > 10) return -1;
  g_cam_switch_drain_threshold = n;
  return 0;
}

// Set the auto-alternate ratio.  Both zero disables auto-alternation.
// Bounds [0, 1000] — max practical quota is well under 1000 frames; we
// reject larger values to avoid surprise wraparound semantics when
// (a+b) is used as a modulus against a 32-bit frame counter.
extern "C" int sentai_cam_ratio_set(uint32_t a, uint32_t b) {
  if (a > 1000u || b > 1000u) return -1;
  g_cam_ratio_a = a;
  g_cam_ratio_b = b;
  return 0;
}

extern "C" void sentai_cam_ratio_get(uint32_t* a, uint32_t* b) {
  if (a) *a = g_cam_ratio_a;
  if (b) *b = g_cam_ratio_b;
}

// ===================== Audio externs for AIfES =====================
// Uses the existing mic implementation in modsentai_hal.cc

extern "C" int sentai_cam_is_initialized(void) {
  return g_cam_initialized ? 1 : 0;
}

// PXP hardware scale+convert: XRGB8888 (native cam) -> RGB888 (scaled output).
// src must be in non-cacheable memory (camera framebuffer).
// dst must be 64-byte aligned for best results.
// Returns 0 on success.
static int pxp_scale_xrgb_to_rgb(const uint8_t* src, int src_w, int src_h,
                                  uint8_t* dst, int dst_w, int dst_h) {
  // Warn once if dst not 32-byte aligned (cache ops may touch adjacent data).
  {
    static bool s_align_warned = false;
    if (!s_align_warned && ((uintptr_t)dst & 31u)) {
      printf("WARN: tensor_buf %p not 32-byte aligned (cache line boundary risk)\r\n", dst);
      s_align_warned = true;
    }
  }

  pxp_ps_buffer_config_t ps_cfg;
  memset(&ps_cfg, 0, sizeof(ps_cfg));
  // kPXP_PsPixelFormatRGB888 = 0x4 = "32-bit pixels without alpha" = XRGB8888
  ps_cfg.pixelFormat = kPXP_PsPixelFormatRGB888;
  ps_cfg.swapByte    = false;
  ps_cfg.bufferAddr  = (uint32_t)src;
  ps_cfg.bufferAddrU = 0;
  ps_cfg.bufferAddrV = 0;
  ps_cfg.pitchBytes  = (src_w + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP;

  pxp_output_buffer_config_t out_cfg;
  memset(&out_cfg, 0, sizeof(out_cfg));
  out_cfg.pixelFormat    = kPXP_OutputPixelFormatRGB888P;
  out_cfg.interlacedMode = kPXP_OutputProgressive;
  out_cfg.buffer0Addr    = (uint32_t)dst;
  out_cfg.buffer1Addr    = 0;
  out_cfg.pitchBytes     = dst_w * 3;
  out_cfg.width          = dst_w;
  out_cfg.height         = dst_h;

  const uint32_t dst_size = dst_w * dst_h * 3;

  // Evict dirty cache lines from previous int8 quantization (XOR) BEFORE PXP
  // DMA writes.  A dirty line evicted after PXP writes would clobber DMA data.
  // CleanInvalidate (writeback then discard) is used instead of plain Invalidate
  // because TFLite arena alignment is 16 bytes, not 32 (the cache line size).
  // If tensor_buf isn't 32-byte aligned, the first/last cache line may contain
  // data from adjacent TFLite tensors — plain Invalidate would silently lose
  // any dirty data in those shared boundary lines.
  // Cost: ~0.1ms more than plain Invalidate (only boundary lines write back).
#if (__CORTEX_M == 7)
  DCACHE_CleanInvalidateByRange((uint32_t)dst, dst_size);
#endif

  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, src_w, src_h, dst_w, dst_h);
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, dst_w - 1, dst_h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);

  PXP_Start(DEMO_PXP);

  // Yield CPU while PXP works instead of busy-waiting
  while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP))) {
    taskYIELD();
  }
  PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);

  // Invalidate cache so CPU sees PXP DMA output.
  // Plain Invalidate (without writeback) is safe here because:
  //   (a) CleanInvalidate above left all tensor lines INVALID — no dirty data.
  //   (b) Between CleanInvalidate and here, NO CPU writes touch the tensor or
  //       its boundary cache lines. (PXP setup writes go to peripheral regs,
  //       taskYIELD runs other tasks but none access the TFLite arena.)
  //   (c) Speculative prefetch may reload CLEAN lines — Invalidate discards
  //       these correctly so the next read fetches fresh PXP data from SDRAM.
  // NOTE: CleanInvalidate here would be WRONG if a boundary line were dirtied
  // during PXP — it would write back stale tensor bytes over PXP output.
#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)dst, dst_size);
#endif

#if SENTAI_DBG_COLOR_ORDER
  // Print first 4 pixels every 100th frame to verify channel order.
  {
    static uint32_t s_color_cnt = 0;
    if ((s_color_cnt++ % 100) == 0) {
      printf("[COLOR_DBG] frame#%lu  first 4 px (ch0,ch1,ch2):", s_color_cnt - 1);
      for (int p = 0; p < 4 && p * 3 + 2 < (int)dst_size; p++) {
        printf("  (%u,%u,%u)", dst[p*3+0], dst[p*3+1], dst[p*3+2]);
      }
      printf("\r\n");
    }
  }
#endif
  return 0;
}

extern "C" int sentai_cam_set_res(int w, int h) {
  if (w <= 0 || h <= 0 || w > DEMO_CAMERA_WIDTH || h > DEMO_CAMERA_HEIGHT) return -1;
  g_cam_width = w;
  g_cam_height = h;
  return 0;
}

extern "C" int sentai_cam_init(int streaming) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (!cam->SetPower(true)) return -1;
  auto mode = streaming ? coralmicro::CameraMode::kStreaming
                        : coralmicro::CameraMode::kTrigger;
  if (!cam->Enable(mode)) return -2;
  g_cam_initialized = true;

  // Cycle through both cameras to ensure CSI/MIPI is fully initialized.
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
  g_cam_current_id = 1;
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
  g_cam_current_id = 0;

  return 0;
}

extern "C" int sentai_cam_stop(void) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->Disable();
  cam->SetPower(false);
  g_cam_initialized = false;
  return 0;
}

// Try to get a raw frame with recovery.
// Drains stale buffered frames first so the caller always gets the LATEST frame.
// NOTE: TryGetRawFrame is NOT truly non-blocking — inside the camera task,
// HandleFrameRequest polls GetFullBuffer up to 40×100ms = 4 seconds.
// So each call either succeeds quickly (~1ms) or blocks up to 4s.
// We try ONCE per attempt, then do recovery if it fails.
static int sentai_cam_get_raw_with_recovery(uint8_t** raw_out) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  const int kMaxRecoveries = 2;
  TickType_t t_start = xTaskGetTickCount();

  // After a camera switch, we need >= 2 fresh ISR frames from the new
  // camera before the image is guaranteed clean (the first frame after MUX
  // flip may be mixed, the second is fully captured by the new sensor).
  // g_cam_switch_seq is written by HandleSwitchCameraRequest in camera.cc
  // atomically with the GpioSet() that flips the mux, so
  // (g_camera_frame_seq - g_cam_switch_seq >= 2) reliably counts only
  // post-flip frames.  Previously the snapshot lived in this file, taken
  // *before* the MUX-flip request was dispatched through the CameraTask
  // queue — a 1–10 ms window during which ISR ticks were counted as
  // post-flip.  That race produced the 65 ms cam1-vs-cam0 asymmetry
  // documented in paper/cam_switch.md before this fix.
  if (g_cam_switch_pending) {
    g_cam_switch_pending = false;
    uint32_t seq_at_switch = g_cam_switch_seq;  // snapshot (usually 0)
    uint32_t seq_now = g_camera_frame_seq;
    uint32_t elapsed = seq_now - seq_at_switch;

    // Snapshot the runtime threshold ONCE per entry: the caller might
    // legitimately change it mid-run (A/B experiments), but each
    // decision in this function must use a single consistent value.
    uint32_t threshold = g_cam_switch_drain_threshold;
    if (threshold < 1) threshold = 1;
    if (threshold > 10) threshold = 10;

    if (elapsed >= threshold) {
      // Fast path: enough ISR frames have already arrived since MUX flip.
      // Just drain stale queued buffers and keep the latest — no blocking.
      printf("  [frame] post-switch FAST: %lu ISR frames elapsed (thr=%lu)\r\n",
             (unsigned long)elapsed, (unsigned long)threshold);
      // fall through to normal drain-and-keep-last below
    } else {
      // Slow path: switch was very recent, not enough frames yet.
      // Drain whatever is queued (stale/mixed), then block for fresh.
      int drained = 0;
      for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT; ++i) {
        uint8_t* tmp = nullptr;
        int idx = cam->TryGetRawFrame(&tmp);
        if (idx < 0 || !tmp) break;
        cam->ReturnRawFrame(idx);
        drained++;
      }
      // Wait until ISR counter shows >= threshold frames from new camera.
      // At 15 fps each frame takes ~67ms, so max wait ≈ threshold × 67ms.
      // Hard ceiling of 300 iterations × 1ms keeps this bounded even if
      // the ISR stops firing (camera driver fault → falls through to
      // recovery path below with g_cam_switch_pending already cleared).
      int wait_iters = 0;
      while ((g_camera_frame_seq - seq_at_switch) < threshold
             && wait_iters < 300) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_iters++;
      }
      // Now grab one fresh frame (blocking)
      uint8_t* frame = nullptr;
      int idx = cam->GetRawFrame(&frame);
      if (idx >= 0 && frame) {
        TickType_t total = xTaskGetTickCount() - t_start;
        printf("  [frame] post-switch SLOW: drained %d, waited %dms, "
               "thr=%lu, seq=%lu, buf#%d (%ldms)\r\n",
               drained, wait_iters, (unsigned long)threshold,
               (unsigned long)g_camera_frame_seq, idx, (long)total);
        *raw_out = frame;
        return idx;
      }
      // Fresh frame failed — fall through to normal recovery path
    }
  }

  for (int recovery = 0; recovery <= kMaxRecoveries; ++recovery) {
    // Drain the FIFO queue non-blockingly and keep only the most recent frame.
    // TryGetRawFrame is now truly non-blocking (single GetFullBuffer probe, no
    // 4-second poll), so the drain loop takes microseconds.  With N DMA
    // buffers the loop tops out at N-1 iterations.  We never wait here — the
    // caller decides whether to bounded-wait for a fresh frame below.
    uint8_t* kept_frame = nullptr;
    int kept_idx = -1;
    int drained = 0;
    for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT - 1; ++i) {
      uint8_t* tmp = nullptr;
      int idx = cam->TryGetRawFrame(&tmp);
      if (idx < 0 || !tmp) break;  // queue empty — stop draining
      if (kept_idx >= 0) cam->ReturnRawFrame(kept_idx);
      kept_idx = idx;
      kept_frame = tmp;
      drained++;
    }

    if (kept_idx >= 0) {
      // Only log when something interesting happened (drained > 1 = we
      // skipped stale frames, or the call took over 20 ms = contention).
      TickType_t total = xTaskGetTickCount() - t_start;
      if (drained > 1 || total > 20) {
        printf("  [frame] drained %d, kept buf#%d (%ldms)\r\n",
               drained, kept_idx, (long)total);
      }
      *raw_out = kept_frame;
      return kept_idx;
    }

    // Queue is empty right now.  The user's intent: "take the next frame I
    // haven't taken yet, never wait for a fresh one".  We fall back to a
    // BOUNDED blocking grab so PrepTask can make forward progress — at 15 FPS
    // the next frame arrives in ≤ 67 ms, well within the 4-second internal
    // poll ceiling.  If even that fails (camera driver stuck), we log and try
    // the toggle-to-recover path below.
    uint8_t* frame = nullptr;
    int idx = cam->GetRawFrame(&frame);
    if (idx >= 0 && frame) {
      TickType_t total = xTaskGetTickCount() - t_start;
      if (total > 20) {
        printf("  [frame] queue empty, waited for buf#%d (%ldms)\r\n",
               idx, (long)total);
      }
      *raw_out = frame;
      return idx;
    }

    // No frames at all — try toggling camera to kick CSI/MIPI
    if (recovery < kMaxRecoveries) {
      int other = (g_cam_current_id == 0) ? 1 : 0;
      printf("[CAM] GetRawFrame failed, toggling %d->%d->%d to recover...\r\n",
             g_cam_current_id, other, g_cam_current_id);
      cam->SwitchCamera(other == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                   : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
      cam->SwitchCamera(g_cam_current_id == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                              : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  printf("[CAM] GetRawFrame failed after all recovery attempts\r\n");
  *raw_out = nullptr;
  return -2;
}

// Capture RGB frame via PXP hardware scaler. Returns 0 on success.
extern "C" int sentai_cam_capture_rgb(uint8_t* buf, int width, int height) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns PXP
  if (!g_cam_initialized) return -1;
  uint8_t* raw = nullptr;
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;

  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  buf, width, height);
  TickType_t t2 = xTaskGetTickCount();
  cam->ReturnRawFrame(idx);
  printf("  [capture_rgb] drain=%ldms pxp=%ldms\r\n",
         (long)(t1 - t0), (long)(t2 - t1));
  return rc;
}

// Persistent RGB buffer in SDRAM — avoids allocating 2.7MB on every call
static uint8_t s_jpeg_rgb_buf[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT * 3]
    __attribute__((section(".sdram_bss")));

// Capture + JPEG compress. Returns JPEG size or negative error.
// Uses PXP hardware for XRGB8888→RGB888 conversion (+ optional scaling),
// then JpegCompressRgb for JPEG encoding.
extern "C" int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                      int width, int height, int quality) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns PXP
  if (!g_cam_initialized) return -1;
  if (width > DEMO_CAMERA_WIDTH || height > DEMO_CAMERA_HEIGHT) return -5;

  uint8_t* raw = nullptr;
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;

  // PXP hardware: XRGB8888 → packed RGB888 (with optional scaling)
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  s_jpeg_rgb_buf, width, height);
  TickType_t t2 = xTaskGetTickCount();
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->ReturnRawFrame(idx);
  if (rc != 0) return rc;

  // JPEG encode the RGB888 buffer
  unsigned long used = coralmicro::JpegCompressRgb(
      s_jpeg_rgb_buf, width, height, quality,
      (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);
  TickType_t t3 = xTaskGetTickCount();

  printf("  [capture_jpeg] drain=%ldms pxp=%ldms jpeg=%ldms total=%ldms (%dx%d)\r\n",
         (long)(t1 - t0), (long)(t2 - t1), (long)(t3 - t2), (long)(t3 - t0),
         width, height);
  return (int)used;
}

// Capture RGB via PXP and feed directly into TPU input tensor.
// If save_path is non-NULL, save a JPEG of the scaled frame before int8 quant.
extern "C" int sentai_cam_to_tensor_ex(const char* save_path, int quality) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns PXP+tensor
  if (!g_cam_initialized) return -1;
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -4;
  int h = input->dims->data[1];
  int w = input->dims->data[2];
  int ch = input->dims->data[3];
  int total_pixels = h * w * ch;
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  TickType_t t_start = xTaskGetTickCount();
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t_frame = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  tensor_buf, w, h);
  cam->ReturnRawFrame(idx);
  TickType_t t_pxp = xTaskGetTickCount();
  if (rc != 0) return rc;

  // Save RGB frame for draw() BEFORE int8 quantization destroys the data.
  // Only copy when draw() has been called (sets g_draw_capture_pending).
  // Skipping this saves ~2-3ms per frame on the critical inference path.
  if (g_draw_capture_pending && total_pixels <= kMaxDrawPixels) {
    memcpy(g_draw_rgb, tensor_buf, total_pixels);
    g_draw_w = w;
    g_draw_h = h;
    g_draw_capture_pending = false;
  }

  // Optionally save JPEG of the scaled RGB frame (before int8 quantization)
  if (save_path && save_path[0]) {
    int jpeg_buf_size = w * h * ch;  // worst-case size
    if (jpeg_buf_size < 64 * 1024) jpeg_buf_size = 64 * 1024;
    uint8_t* jpeg_buf = (uint8_t*)malloc(jpeg_buf_size);
    if (jpeg_buf) {
      unsigned long jpeg_size = coralmicro::JpegCompressRgb(
          tensor_buf, w, h, quality,
          jpeg_buf, (unsigned long)jpeg_buf_size);
      if (jpeg_size > 0) {
        std::string jpeg_data((const char*)jpeg_buf, jpeg_size);
        if (coralmicro::LfsUserWriteFile(save_path, jpeg_data)) {
          printf("Saved %dx%d JPEG to %s (%lu bytes)\r\n", w, h, save_path, jpeg_size);
        } else {
          printf("JPEG save failed: %s\r\n", save_path);
        }
      }
      free(jpeg_buf);
    }
  }

  // If model expects int8 input, apply quantization offset.
  TickType_t t_quant_start = xTaskGetTickCount();
  if (input->type == kTfLiteInt8) {
    sentai_quant_uint8_to_int8(tensor_buf, total_pixels,
                               input->params.zero_point);
  }
  TickType_t t_end = xTaskGetTickCount();
  printf("[to_tensor] frame=%ldms pxp=%ldms quant=%ldms total=%ldms (%dx%d)\r\n",
         (long)(t_frame - t_start), (long)(t_pxp - t_frame),
         (long)(t_end - t_quant_start), (long)(t_end - t_start), w, h);
  return 0;
}

extern "C" int sentai_cam_to_tensor(void) {
  return sentai_cam_to_tensor_ex(NULL, 75);
}

// Switch between front and back cameras. id: 0=front, 1=back.
//
// Glitch-free switching is implemented by arming `g_cam_pending_mux_id`
// and letting the CSI EOF ISR (libs/camera/camera_support.c) perform the
// actual GPIO flip the next time a DMA buffer completes.  That window is
// the MIPI VBLANK interval — both sensors are between frame transmissions
// — so the switch never lands mid-buffer and the following frame is
// guaranteed to be 100% from the new sensor.
//
// Bounded behaviour (embeded.md §B):
//   - Arm is non-blocking from the caller's perspective.
//   - We then poll for ISR consumption up to `kArmTimeoutMs` (3 frame
//     intervals at 30 fps).  During the wait other tasks run freely.
//   - If the arm has not been consumed by then, the CSI is either not
//     firing EOF (driver stuck) or we missed a frame cycle — we fall
//     back to the legacy synchronous path (`cam->SwitchCamera`) which
//     flips the GPIO in task context using the NXP driver mutex.  The
//     synchronous path yields the old mid-buffer seam behaviour but at
//     least the switch completes and the operator sees the degraded
//     path through the `[cam_switch] fallback` log line.
extern "C" int sentai_cam_switch(int id) {
  if (!g_cam_initialized) return -1;
  if (id != 0 && id != 1) return -2;
  if (id == g_cam_current_id && !g_cam_switch_pending) return 0;

  TickType_t ts0 = xTaskGetTickCount();

  // Arm the ISR.  Write the pending-id LAST: if a stale pending flag
  // happens to be observed here, the ISR's own consumer logic clears
  // it (pending_mux_id is written -1 after flip).
  g_cam_pending_mux_id = id;
  sentai_tracker_set_active_camera(id);

  // Bounded wait for ISR consumption.  At 30 fps, one frame = 33 ms.
  // Three frames gives comfortable margin for jitter but caps the
  // worst case well below any user-visible "stuck switch" pathology.
  const TickType_t kArmTimeoutTicks = pdMS_TO_TICKS(150);
  TickType_t deadline = ts0 + kArmTimeoutTicks;
  while (g_cam_pending_mux_id >= 0 && xTaskGetTickCount() < deadline) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (g_cam_pending_mux_id >= 0) {
    // ISR did not fire — CSI likely stuck or frame rate zero.  Disarm
    // the ISR path and flip synchronously via the legacy route.  This
    // path re-introduces the mid-buffer seam, but keeps the system
    // responsive (the alternative is blocking forever).
    g_cam_pending_mux_id = -1;
    auto* cam = coralmicro::CameraTask::GetSingleton();
    if (id == 0) {
      cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
    } else {
      cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
    }
    // Mark the drain path as pending.  The snapshot lives in
    // HandleSwitchCameraRequest (libs/camera/camera.cc) which ran
    // inside cam->SwitchCamera above, so g_cam_switch_seq is already
    // set.
    g_cam_switch_pending = true;
    g_cam_current_id = id;
    TickType_t ts1 = xTaskGetTickCount();
    printf("[cam_switch] fallback sync -> cam%d (%ldms, seq=%lu)\r\n",
           id, (long)(ts1 - ts0), (unsigned long)g_cam_switch_seq);
    return 0;
  }

  // Nominal path: ISR consumed the arm, flipped the GPIO in VBLANK,
  // set g_cam_switch_{seq,pending}.  Update the logical id AFTER the
  // ISR has done its work so any observer that sees
  // g_cam_current_id == id also sees a consistent switch state.
  g_cam_current_id = id;
  TickType_t ts1 = xTaskGetTickCount();
  printf("[cam_switch] -> cam%d (%ldms, seq=%lu, via EOF ISR)\r\n",
         id, (long)(ts1 - ts0), (unsigned long)g_cam_switch_seq);
  return 0;
}

// Rotate camera image. cam_id: 0=front, 1=back. degrees: 0, 90, 180, 270.
extern "C" int sentai_cam_rotate(int cam_id, int degrees) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = cam->SetCameraRotation(cam_id, degrees);
  return ok ? 0 : -2;
}

extern "C" int sentai_cam_get_width(void) {
  return g_cam_width;
}

extern "C" int sentai_cam_get_height(void) {
  return g_cam_height;
}

extern "C" uint32_t sentai_cam_get_frame_seq(void) {
  return g_camera_frame_seq;
}

extern "C" int sentai_cam_get_native_width(void) {
  return coralmicro::CameraTask::kWidth;
}

extern "C" int sentai_cam_get_native_height(void) {
  return coralmicro::CameraTask::kHeight;
}

// ===================== AIfES sensor capture functions =====================

// Capture camera frame for AIfES, resize to w×h, output as RGB or grayscale
// Returns: bytes written, or negative on error
extern "C" int sentai_aifes_capture_camera(uint8_t* out, int w, int h, int grayscale) {
  if (!g_cam_initialized) return -1;
  if (!out || w <= 0 || h <= 0) return -3;
  if (w > DEMO_CAMERA_WIDTH || h > DEMO_CAMERA_HEIGHT) return -5;
  
  // Capture raw frame
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  
  auto* cam = coralmicro::CameraTask::GetSingleton();
  
  if (grayscale) {
    // For grayscale: capture RGB, then convert
    // Use temporary RGB buffer
    std::vector<uint8_t> rgb_buf(w * h * 3);
    int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                    rgb_buf.data(), w, h);
    cam->ReturnRawFrame(idx);
    if (rc != 0) return rc;
    
    // Convert RGB to grayscale: Y = 0.299R + 0.587G + 0.114B
    for (int i = 0; i < w * h; i++) {
      int r = rgb_buf[i * 3];
      int g = rgb_buf[i * 3 + 1];
      int b = rgb_buf[i * 3 + 2];
      out[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
    }
    return w * h;
  } else {
    // RGB: direct PXP output
    int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                    out, w, h);
    cam->ReturnRawFrame(idx);
    if (rc != 0) return rc;
    return w * h * 3;
  }
}
// [end-sphinx-snippet:detect-image]