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
#include <cstdlib>
#include <cmath>
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
// applsci energy sweep: read persisted Edge TPU clock in Main() (see below).
extern "C" size_t FxUserReadFile(const char* path, uint8_t* buf, size_t size);
#include "libs/base/fx_user_fs.h"
#include "libs/base/gpio.h"
#include "libs/base/ipc_m7.h"
#include "libs/base/led.h"
#include "examples/sentai_runtime/flow_shared.h"
#include "libs/base/reset.h"
#include "libs/camera/camera.h"
#include "libs/pmic/pmic.h"
#include "libs/libjpeg/jpeg.h"
#include "libs/camera/camera_support.h"
#include "libs/base/utils.h"   // GetSerialNumber (board UID)
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
#include "third_party/freertos_kernel/include/semphr.h"
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
#include "sentai_fs_task.h"
#include "sentai_pxp_shim.h"
#include "sentai_virtual_camera.h"
#include "sentai_fr.h"

// ===================== Runtime SOURCE geometry of the camera frames =========
// W24-T20 (audit agent/audit_runtime_geometry_2026-11-02.md).  The sensor
// geometry is a runtime property since W24-T11: libs/camera/camera_support.c
// owns g_runtime_cam_w/h (640x480 or 1280x960, camera.set_hw() before init())
// and the capture layout is either XRGB/XYUV 4 B/px (row pitch w*4) or packed
// YUV420 (CSI stores [C,Y1,Y2,0] per pixel pair, row pitch (w/2)*4).  Every
// CPU or PXP reader of a raw frame buffer MUST take width / height / pitch from
// these helpers; the compile-time DEMO_CAMERA_* constants describe only the
// VGA default and produced interleaved half-row images (rc 0) at SXGA.
extern "C" volatile uint32_t g_runtime_cam_w;
extern "C" volatile uint32_t g_runtime_cam_h;
extern volatile int g_sentai_cam_packed420;
static inline int cam_src_w(void) {
  return sentai_virtual_camera_active() ? sentai_virtual_camera_width() : (int)g_runtime_cam_w;
}
static inline int cam_src_h(void) {
  return sentai_virtual_camera_active() ? sentai_virtual_camera_height() : (int)g_runtime_cam_h;
}
// Packed YUV420 frames are not XRGB/XYUV: the PXP scaler paths cannot read them.
static inline bool cam_src_packed(void) {
  return g_sentai_cam_packed420 != 0 && !sentai_virtual_camera_active();
}
static inline size_t cam_src_pitch(void) {
  return cam_src_packed() ? (size_t)(cam_src_w() / 2) * (size_t)DEMO_CAMERA_BUFFER_BPP
                          : (size_t)(cam_src_w() + LINE_PADDING) * (size_t)DEMO_CAMERA_BUFFER_BPP;
}
// W24-T22: per-module register sets applied at the end of camera init (defined
// next to sentai_cam_reg_write, below the init path).
namespace coralmicro { class CameraTask; }
static void cam_apply_module_regs(coralmicro::CameraTask* cam);

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

// Defined in the camera section below (global scope); the watchdog task in
// the anonymous namespace reads it for the CAM_STALL crash-log line.
extern volatile int g_cam_current_id;
extern volatile uint32_t g_cam_ratio_packed;   // defined in the camera section (file scope)
extern volatile uint32_t g_cam_csi_queue_faults, g_cam_csi_queue_fault_kind;  // libs/camera/camera_support.c (SDK patch 0005)
extern volatile uint32_t g_cam_flog_head;                                     // libs/camera/camera_support.c (FB-done log)
extern volatile uint32_t g_cam_ratio_unit;                                    // libs/camera/camera_support.c (1 = per frame, 2 = FB2 pairs)

namespace {

// Boot log state.  Refactored 2026-04-29:
//   1. raw lfs_file_open/write/close → FxUserAppendFile (FileX migration
//      had broken the path because LfsUser() returns nullptr).
//   2. SDRAM .sdram_bss → SDRAM .sdram_boot_log NOLOAD.  The buffer is
//      now preserved across warm-reset / WDOG / hard-fault.  A header
//      carries magic + ~magic + length so the NEXT boot can detect
//      a valid pre-crash trace and flush it to /log/boot_prev.log
//      before starting a fresh trace.  Cleared only by POR.
//
// Layout of g_boot_log_persist (in .sdram_boot_log NOLOAD):
//   [magic:u32][check:u32][len:u32][reserved:u32][bytes:16K-16]
//
// magic == kBootLogMagic AND check == ~magic ⇒ buffer holds a valid
// pre-crash trace of `len` bytes that we should rescue on this boot.
static constexpr uint32_t kBootLogMagic = 0xB00710B0UL;     /* 'BOOT LOG' marker */
static constexpr size_t   kBootLogTotalSize = 16 * 1024;
static constexpr size_t   kBootLogHdrSize   = 16;
static constexpr size_t   kBootLogBufSize   = kBootLogTotalSize - kBootLogHdrSize;
struct BootLogPersist {
    volatile uint32_t magic;
    volatile uint32_t check;
    volatile uint32_t len;
    volatile uint32_t _rsvd;
    char              buf[kBootLogBufSize];
};
__attribute__((section(".sdram_boot_log"))) __attribute__((used))
static volatile BootLogPersist g_boot_log_persist;

// Crash-text slot (2026-10-31, audit base-M1 / §14 review MAJOR 1).  The
// fault handlers write their decoded record here, NOT into the live boot
// log: after boot_log_stop() the live buffer is re-used from offset 0 by
// the next boot's printf before boot_log_fs_init() can rescue it, which
// overwrote the record.  This slot is separate NOLOAD storage, snapshotted
// by boot_log_init() before the first printf of the new boot, and
// written to /log/crash_prev.txt (+ echoed to the console) once the card
// is mounted.
//
// Placement: DTCM (.noinit.$crash_text, linker puts it above the stack at
// the top of m_data), NOT SDRAM.  SDRAM needs refresh: on a board that
// returns to the ROM serial downloader after a reset (BOOT_MODE straps,
// EXP-s248) the seconds until flashtool reloads the image decay the SDRAM
// slot (measured build 1645: record lost).  DTCM holds without refresh
// through the warm reset, the ROM and the elf_loader (whose own DTCM use
// ends at 0x2000A5B0).  Cleared only by POR.
static constexpr uint32_t kCrashTextMagic = 0xC5A5E7E0UL;
static constexpr size_t   kCrashTextSize  = 1008;
struct CrashTextPersist {
    volatile uint32_t magic;
    volatile uint32_t check;      // ~magic
    volatile uint32_t len;        // bytes valid in text[]
    volatile uint32_t truncated;  // 1 = record did not fit
    char              text[kCrashTextSize];
};
__attribute__((section(".noinit.$crash_text"))) __attribute__((used))
static volatile CrashTextPersist g_crash_text_persist;
// Snapshot taken by boot_log_init (before any printf of this boot).
__attribute__((section(".sdram_bss")))
static char     s_prev_crash_text[kCrashTextSize];
static uint32_t s_prev_crash_len = 0;
static uint32_t s_prev_crash_truncated = 0;
// Captured at real_main entry — IF the previous boot crashed mid-log,
// these hold the magic + payload length we observed before zeroing
// the header for the current boot's writes.  flushed to
// /log/boot_prev.log by boot_log_fs_init when the FileX volume mounts.
static uint32_t g_prev_boot_log_magic_at_entry = 0;
static uint32_t g_prev_boot_log_len = 0;
static volatile bool g_boot_log_active = false;
static volatile bool g_boot_log_fs_ready = false;
// Guards boot_log_flush_to_file() against concurrent flush from multiple tasks.
// Uses GCC atomic test-and-set (lock-free, no RTOS dependency).
static volatile uint32_t g_boot_log_flush_busy = 0;

// Forward declare - flush buffer to file
static void boot_log_flush_to_file();

// Initialize boot logging.  Called EARLY in app_main, BEFORE FileX is
// mounted.  Snapshots the pre-existing SDRAM header (which may carry a
// valid trace from a previous boot that crashed) so boot_log_fs_init
// can rescue it later, then resets the live buffer for this boot.
static void boot_log_init() {
    // Snapshot the previous boot's residual header BEFORE we clobber it.
    g_prev_boot_log_magic_at_entry = g_boot_log_persist.magic;
    uint32_t prev_check = g_boot_log_persist.check;
    uint32_t prev_len   = g_boot_log_persist.len;
    if (g_prev_boot_log_magic_at_entry == kBootLogMagic &&
        prev_check == ~kBootLogMagic &&
        prev_len > 0 && prev_len <= kBootLogBufSize) {
        g_prev_boot_log_len = prev_len;
    } else {
        g_prev_boot_log_len = 0;
    }

    // Rescue the previous boot's crash-text record (fault handler output)
    // before anything else can touch SDRAM state.  Cleared right away so
    // a clean boot after the rescue shows nothing stale.
    s_prev_crash_len = 0;
    s_prev_crash_truncated = 0;
    if (g_crash_text_persist.magic == kCrashTextMagic &&
        g_crash_text_persist.check == ~kCrashTextMagic &&
        g_crash_text_persist.len > 0 &&
        g_crash_text_persist.len <= kCrashTextSize) {
        s_prev_crash_len = g_crash_text_persist.len;
        s_prev_crash_truncated = g_crash_text_persist.truncated;
        for (uint32_t i = 0; i < s_prev_crash_len; i++) {
            s_prev_crash_text[i] = g_crash_text_persist.text[i];
        }
    }
    g_crash_text_persist.magic = 0;
    g_crash_text_persist.check = 0;
    g_crash_text_persist.len   = 0;
    g_crash_text_persist.truncated = 0;

    // Reset header for THIS boot's trace.  Magic remains valid only
    // while we're actively logging — boot_log_stop() also keeps it set
    // so a crash AFTER REPL came up still surfaces the boot.log on the
    // next boot.
    g_boot_log_persist.magic = kBootLogMagic;
    g_boot_log_persist.check = ~kBootLogMagic;
    g_boot_log_persist.len   = 0;
    g_boot_log_persist._rsvd = 0;
    g_boot_log_active = true;
    g_boot_log_fs_ready = false;
}

// Called after FileX user volume is mounted to set up the boot.log
// rotation (boot.log -> boot_old.log) and stream subsequent printf bytes
// to FAT via FxUserAppendFile in boot_log_flush_to_file.
//
// Phase 2 of crash-survivable logging (2026-04-29): if the SDRAM
// .sdram_boot_log header indicated a valid pre-crash trace (`g_prev_boot_log_len`
// captured at boot_log_init time), rescue it to /log/boot_prev.log
// here BEFORE starting any normal rotation.  This recovers boot
// traces from firmware that crashed before reaching FxUserInit.
static void boot_log_fs_init() {
    if (!g_boot_log_active) return;
    if (!FxUserIsMounted()) return;

    // Create /log directory if it doesn't exist
    FxUserMakeDirs("/log");

    // Rescue any pre-crash trace from the previous boot's SDRAM
    // buffer.  The header was snapshotted in boot_log_init BEFORE we
    // clobbered it for this boot, so the data is still readable from
    // the (now-overlapping) buffer at offset kBootLogHdrSize.  Flush
    // it to /log/boot_prev.log; on success the trace is durable, on
    // failure we just lose this one (the buffer is gone anyway after
    // we start writing this boot).
    if (g_prev_boot_log_len > 0 &&
        g_prev_boot_log_len <= kBootLogBufSize) {
        FxUserRemove("/log/boot_prev.log");   // best-effort discard older
        // Direct write: header was already overwritten by boot_log_init
        // for the current boot, but the data bytes after offset 16
        // were NOT touched yet (this boot has only printf'd a few
        // hundred bytes by now, all sitting in the live buffer).
        // Use FxUserWriteFile (truncating) so the file is exactly the
        // pre-crash trace size.
        FxUserWriteFile("/log/boot_prev.log",
                        (const uint8_t*)g_boot_log_persist.buf,
                        g_prev_boot_log_len);
    }

    // Crash-text record from the previous boot (HardFault / stack overflow
    // / malloc-failed handler output, see sentai_fault.cc).  Written to
    // its own file and echoed to the console so it is visible on the
    // first REPL screen after a crash.
    if (s_prev_crash_len > 0) {
        FxUserWriteFile("/log/crash_prev.txt",
                        (const uint8_t*)s_prev_crash_text, s_prev_crash_len);
        if (s_prev_crash_truncated) {
            static const char trunc[] = "[record truncated]\r\n";
            FxUserAppendFile("/log/crash_prev.txt", (const uint8_t*)trunc,
                             sizeof(trunc) - 1);
        }
        printf("[boot] previous crash record (/log/crash_prev.txt):\r\n%.*s\r\n",
               (int)s_prev_crash_len, s_prev_crash_text);
    }

    // Rotate boot.log -> boot_old.log if a previous boot left one behind.
    if (FxUserFileExists("/log/boot.log")) {
        // Best-effort: ignore errors on the rename target (might not exist).
        FxUserRemove("/log/boot_old.log");
        FxUserRename("/log/boot.log", "/log/boot_old.log");
    }

    // Truncate any pre-existing /log/boot.log so this boot starts at offset 0.
    // FxUserAppendFile auto-creates the file on first append, so we just
    // remove a stale one if present.
    if (FxUserFileExists("/log/boot.log")) {
        FxUserRemove("/log/boot.log");
    }

    g_boot_log_fs_ready = true;
    boot_log_flush_to_file();   // drain whatever's accumulated pre-FS-up
}

// Flush RAM buffer to file.
// Re-entrant-safe: if another task/call is already flushing, returns immediately
// (data is still in the RAM buffer and will be picked up on the next flush).
// Must only be called from task context (not ISR) — enforced by callers.
static void boot_log_flush_to_file() {
    if (!g_boot_log_fs_ready) return;
    if (g_boot_log_persist.len == 0) return;

    // Acquire flush lock — non-blocking (return if another flush is in progress)
    if (__sync_lock_test_and_set(&g_boot_log_flush_busy, 1u) != 0u) return;

    // Snapshot the current byte count inside a critical section so we
    // don't race with boot_log_write() appending new bytes.
    taskENTER_CRITICAL();
    size_t count = g_boot_log_persist.len;
    taskEXIT_CRITICAL();

    if (count > 0) {
        // FxUserAppendFile is internally serialized via the FileX mutex;
        // no need to hold sentai_fs_lock() too.  Failure (NAND error,
        // OOM) returns 0 — we drop the chunk and keep going.  Cast
        // away `volatile` to feed FxUserAppendFile's plain pointer;
        // the buffer doesn't change during the call because we hold
        // the flush lock and ENTER_CRITICAL'd around the count read.
        FxUserAppendFile("/log/boot.log",
                         (const uint8_t*)(const char*)g_boot_log_persist.buf,
                         count);

        // Reset only the bytes we already wrote; bytes appended during the
        // write remain at the front of the buffer for the next flush.
        taskENTER_CRITICAL();
        if (g_boot_log_persist.len >= count) {
            size_t remaining = g_boot_log_persist.len - count;
            if (remaining > 0)
                memmove((void*)g_boot_log_persist.buf,
                        (const void*)(g_boot_log_persist.buf + count),
                        remaining);
            g_boot_log_persist.len = remaining;
        } else {
            g_boot_log_persist.len = 0;
        }
        taskEXIT_CRITICAL();
    }

    __sync_lock_release(&g_boot_log_flush_busy);
}

// Add data to boot log (from _write override)
// Called from any context (task, ISR) via printf -> _write.
// Uses critical section to protect shared buffer from concurrent access.
static void boot_log_write(const char* data, size_t len) {
    if (!g_boot_log_active) return;

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    // Add to SDRAM persistent buffer (.sdram_boot_log NOLOAD).
    size_t pos = g_boot_log_persist.len;
    size_t space = (pos < kBootLogBufSize) ? (kBootLogBufSize - pos) : 0;
    size_t to_copy = (len < space) ? len : space;
    if (to_copy > 0) {
        memcpy((void*)(g_boot_log_persist.buf + pos), data, to_copy);
        g_boot_log_persist.len = pos + to_copy;
    }

    // If buffer is getting full and FS ready, flush.
    // NOTE: flush does LFS I/O which is slow — only in non-ISR context.
    bool need_flush = g_boot_log_fs_ready &&
                      g_boot_log_persist.len > kBootLogBufSize - 512;

    taskEXIT_CRITICAL_FROM_ISR(saved);

    if (need_flush && xPortIsInsideInterrupt() == pdFALSE) {
        boot_log_flush_to_file();
    }
}

// Stop boot logging (called when REPL starts)
void boot_log_stop() {
    if (!g_boot_log_active) return;

    g_boot_log_active = false;

    // Final flush — append-based, no file handle to close.  Refactored
    // 2026-04-29 to use FxUserAppendFile (was lfs_file_close on
    // LfsUser() which returns nullptr post-FileX migration).  After
    // the final append, force a FAT sync so the boot.log row is
    // durable on NAND — Phase 3.2 dropped the per-call sync from
    // FxUserAppendFile for steady-state perf, so callers at
    // checkpoints (REPL-up, planned reset) MUST explicitly sync.
    if (g_boot_log_fs_ready) {
        boot_log_flush_to_file();
        FxUserSync();
    }

    // Invalidate the SDRAM crash-survivable header now that the boot
    // trace is durable on /log/boot.log.  A subsequent crash that
    // happens AFTER REPL-up no longer needs to be rescued from SDRAM
    // — boot.log already has its bytes — and we don't want a stale
    // (now-empty) buffer to be re-flushed as boot_prev.log on the
    // next boot.
    g_boot_log_persist.magic = 0;
    g_boot_log_persist.check = 0;
    g_boot_log_persist.len   = 0;
}

// =============================================================================
// Crash/Hang Logging System + Hardware Watchdog
// =============================================================================
// Saves crash/hang info to /log/crash_NNN.log for post-mortem analysis.
// Keeps multiple crash logs (kCrashLogMaxFiles) with automatic rotation.
// Hardware watchdog (WDOG1, 30 s) is kicked unconditionally by
// CombinedWatchdogTask every 5 s; it only fires if that task itself stops
// running (scheduler wedge, hard fault without reset, priority inversion).
// Idle is NOT a fault (changed 2026-10-30): a quiet board stays up and only
// logs a task snapshot every 10 min.

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
// Network health flag (audit base-M10, 2026-10-31).  Meaning: the USB-NCM
// TX path dropped nothing during the last watchdog interval.  Set true by
// a successful HTTP request, false by an HTTP hang report or by NCM TX
// drops advancing between two kicks; it used to be forced true on every
// kick and therefore carried no information.
static volatile bool g_network_healthy = true;
extern "C" uint32_t sentai_ncm_tx_drops(void);   // libs/base/main_freertos_m7.cc

static volatile uint32_t g_repl_last_activity = 0;    // Last REPL activity tick
static volatile uint32_t g_repl_input_count = 0;      // Total REPL inputs

// Camera liveness (2026-10-31 audit camera-M5).  Observed by the watchdog
// task every kick: while the camera is initialised and streaming the CSI
// ISR must advance g_camera_sensor_frames.  A stall is REPORTED (SERR +
// dmesg + crash log + counter), never acted on -- recovery stays with the
// grab path (SERR_CAM_GRAB_RETRY) and the operator.  Written by the
// watchdog task only; read from MP via sentai.camera.stats().
static volatile uint32_t g_cam_stall_count    = 0;   // stalls seen since boot
static volatile bool     g_cam_stalled        = false;
static volatile uint32_t g_cam_stall_start_ms = 0;
extern "C" int sentai_cam_is_initialized(void);
// Dual-camera phase monitor (EXP-s251, proposal D): defined in the camera
// section below; the watchdog samples it once per kick while alternating.
extern "C" int sentai_cam_phase_from_log(int32_t* phi_us, uint32_t* T_us);
extern "C" void sentai_cam_phase_monitor_note(int32_t phi_us);

// Hardware watchdog control
static volatile bool g_hw_watchdog_enabled = false;
static volatile bool g_force_reset_pending = false;

// Handle for the normal-mode watchdog task (saved so it can be monitored).
static TaskHandle_t s_wdog_task_handle = nullptr;

// Anti-brick recovery mode flag (set when boot_attempts >= 3)
static volatile bool s_in_recovery_mode = false;

// Current crash log number (persisted across rotations)
static int g_crash_log_num = -1;  // -1 = not initialized

// Helper for FxUserListDir: track max NNN in crash_NNN.log filenames.
struct CrashLogScan {
    int max_num;
};
static int crash_log_scan_cb(const FxDirEntry* e, void* user) {
    CrashLogScan* s = (CrashLogScan*)user;
    // Look for crash_NNN.log (13 chars: crash_NNN.log).  e->is_dir==0 = file.
    if (!e->is_dir && strncmp(e->name, "crash_", 6) == 0 &&
        strlen(e->name) == 13) {
        int num = atoi(e->name + 6);
        if (num > s->max_num) s->max_num = num;
    }
    return 0;  // continue
}

// Find highest existing crash log number and set g_crash_log_num.
// Refactored 2026-04-29 from raw lfs_dir_open/read/close to FxUserListDir.
static void crash_log_init(void) {
    if (g_crash_log_num >= 0) return;  // Already initialized
    if (!FxUserIsMounted()) { g_crash_log_num = 0; return; }

    CrashLogScan s = { -1 };
    int n = FxUserListDir("/log", crash_log_scan_cb, &s);
    if (n < 0) { g_crash_log_num = 0; return; }
    g_crash_log_num = (s.max_num >= 0) ? s.max_num : 0;
}

// Get current crash log path, rotate if needed.
static void crash_log_get_path(char* path, size_t path_len) {
    crash_log_init();

    snprintf(path, path_len, "/log/crash_%03d.log", g_crash_log_num);

    FxStat st;
    if (FxUserStat(path, &st) == 1 && st.exists && !st.is_dir) {
        if (st.size > kCrashLogMaxSize - 640) {
            // Rotate to next file
            g_crash_log_num++;
            snprintf(path, path_len, "/log/crash_%03d.log", g_crash_log_num);

            // Delete oldest if we have too many
            if (g_crash_log_num >= kCrashLogMaxFiles) {
                char old_path[32];
                snprintf(old_path, sizeof(old_path), "/log/crash_%03d.log",
                         g_crash_log_num - kCrashLogMaxFiles);
                FxUserRemove(old_path);
            }
        }
    }
}

// sentai_fs_lock/unlock are defined in sentai_fs_task.cc — kept for callers
// outside crash-log path.  FxUserAppendFile is internally serialized so
// we no longer need the explicit lock here.

// Append a compact FreeRTOS task dump to the current crash log.
// Useful on watchdog warnings to see which task is blocked and on what.
// Format: one line per task: name | state | prio | hwm(bytes)
static void crash_log_task_dump(const char* why) {
    if (!FxUserIsMounted()) return;

    FxUserMakeDirs("/log");

    constexpr UBaseType_t kMaxTasks = 32;
    TaskStatus_t tasks[kMaxTasks];
    UBaseType_t n = uxTaskGetSystemState(tasks, kMaxTasks, nullptr);

    uint32_t up = xTaskGetTickCount() * portTICK_PERIOD_MS;
    char hdr[160];
    int hl = snprintf(hdr, sizeof(hdr),
        "  TASK_DUMP (%s, t=%lums, n=%u):\r\n",
        why ? why : "?", (unsigned long)up, (unsigned)n);

    char path[32];
    crash_log_get_path(path, sizeof(path));

    if (hl > 0) {
        FxUserAppendFile(path, (const uint8_t*)hdr, (size_t)hl);
    }
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
        if (ll > 0) {
            FxUserAppendFile(path, (const uint8_t*)line, (size_t)ll);
        }
    }
}

// Write crash entry to /log/crash_NNN.log
static void crash_log_write(const char* event, const char* details) {
    if (!FxUserIsMounted()) return;

    FxUserMakeDirs("/log");

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

    char path[32];
    crash_log_get_path(path, sizeof(path));

    if (len > 0) {
        FxUserAppendFile(path, (const uint8_t*)entry, (size_t)len);
    }
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
// Safe to call from any task context.  Refactored 2026-04-29 from raw
// lfs_dir_open/read/close to FxUserListDir.
extern "C" void sentai_get_last_crash_log_path(char* out, size_t out_len) {
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!FxUserIsMounted()) return;

    CrashLogScan s = { -1 };
    int n = FxUserListDir("/log", crash_log_scan_cb, &s);
    if (n < 0) return;
    if (s.max_num >= 0) {
        snprintf(out, out_len, "/log/crash_%03d.log", s.max_num);
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
    // FxUserAppendFile does NOT auto-flush (Phase 3.2 perf trade-off);
    // we MUST FxUserSync() before reset or the FAT table never lands on
    // NAND and the crash log row is lost.  Same applies to boot_log
    // entries written this boot.
    if (g_boot_log_fs_ready) boot_log_flush_to_file();
    FxUserSync();
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
// - If this task cannot run (scheduler hung) -> we don't kick -> WDOG1 resets
// - Idle (no REPL/HTTP traffic) is NOT treated as a hang (2026-10-30)
static void CombinedWatchdogTask(void* param) {
    (void)param;
    
    // === Hardware Watchdog Configuration ===
    // Timeout = 30 seconds, kick every 5 seconds
    // WDOG1 runs on 32kHz clock, independent of CPU
    constexpr uint32_t kWdogTimeoutSec = 30;
    constexpr uint32_t kKickIntervalMs = 5000;    // Kick every 5 seconds
    constexpr uint32_t kWarningThresholdMs = 60000; // 60 seconds = start warning (still kicks)
    constexpr uint32_t kDeadThresholdMs = 120000;   // 120 seconds = "long idle" diagnostics tier (still kicks; see below)
    
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

        // --- Network path health (base-M10) ------------------------------
        // NCM TX drops advancing since the previous kick = unhealthy; a
        // quiet interval restores the flag.  Observed only, logged once per
        // transition so a lossy link leaves a trace in dmesg.
        {
            static uint32_t last_drops = 0;
            uint32_t drops = sentai_ncm_tx_drops();
            bool ok = (drops == last_drops);
            if (ok != g_network_healthy) {
                SERR_LOG(ok ? SERR_WDG_KICK : SERR_WDG_WARN, drops);
            }
            g_network_healthy = ok;
            last_drops = drops;
        }

        // --- Dual-camera phase monitor (EXP-s251 D) ----------------------
        // Only meaningful while the ratio scheduler alternates (both quotas
        // > 0) and only from the FB-done log, which advances while frames
        // are consumed.  Report-only: SERR_CAM_PHASE_DRIFT on an excursion.
        if (sentai_cam_is_initialized()) {
            uint32_t rp = g_cam_ratio_packed;
            static uint32_t last_flog_head = 0;
            uint32_t fh = g_cam_flog_head;
            // The log only advances while frames are consumed; do not re-judge
            // the same 64 entries kick after kick.
            if ((rp >> 16) != 0u && (rp & 0xFFFFu) != 0u && fh != last_flog_head) {
                int32_t phi = 0; uint32_t T = 0;
                if (sentai_cam_phase_from_log(&phi, &T) == 0) {
                    sentai_cam_phase_monitor_note(phi);
                }
            }
            last_flog_head = fh;
        }

        // --- CSI queue-fault observer (SDK patch 0005 / audit F15) --------
        {
            static uint32_t last_qf = 0;
            uint32_t qf = g_cam_csi_queue_faults;
            if (qf != last_qf) {
                last_qf = qf;
                SERR_LOG(SERR_CAM_CSI_QUEUE_FAULT, (g_cam_csi_queue_fault_kind << 16) | (qf & 0xFFFFu));
            }
        }

        // --- Camera liveness (camera-M5) ---------------------------------
        // Two consecutive kicks (10 s) without a new sensor frame while the
        // camera is initialised = stall.  kCamStallKicks=2 so one slow kick
        // (SD sync, USB burst) cannot raise a false alarm.  camera.init(0)
        // ("trigger mode") is not a false-positive source: HandleEnableRequest
        // ignores the mode and always starts the CSI receiver (EXP-s245).
        // The SD write below (crash_log_write) is bounded by the FileX lock
        // timeout and fires once per stall edge, inside the 30 s WDOG window.
        {
            constexpr uint32_t kCamStallKicks = 2;
            static uint32_t last_frames = 0;
            static uint32_t idle_kicks  = 0;
            if (sentai_cam_is_initialized()) {
                uint32_t frames = g_camera_sensor_frames;
                if (frames == last_frames) {
                    if (idle_kicks < 0xFFFFu) idle_kicks++;
                } else {
                    if (g_cam_stalled) {
                        g_cam_stalled = false;
                        uint32_t dur = now - g_cam_stall_start_ms;
                        SERR_LOG(SERR_CAM_STALL_END, dur / 1000u);
                        char buf[64];
                        snprintf(buf, sizeof(buf), "frames resumed after %lus (n=%lu)",
                                 (unsigned long)(dur / 1000u), (unsigned long)frames);
                        crash_log_write("CAM_STALL_END", buf);
                    }
                    idle_kicks = 0;
                }
                last_frames = frames;
                if (idle_kicks >= kCamStallKicks && !g_cam_stalled) {
                    g_cam_stalled = true;
                    g_cam_stall_count++;
                    g_cam_stall_start_ms = now - idle_kicks * kKickIntervalMs;
                    SERR_LOG(SERR_CAM_STALL, frames);
                    char buf[64];
                    snprintf(buf, sizeof(buf), "no sensor frame for %lus (n=%lu, cam=%d)",
                             (unsigned long)(idle_kicks * kKickIntervalMs / 1000u),
                             (unsigned long)frames, (int)g_cam_current_id);
                    crash_log_write("CAM_STALL", buf);
                }
            } else {
                // Camera off: nothing to observe; clear any stall state.
                idle_kicks = 0;
                last_frames = g_camera_sensor_frames;
                g_cam_stalled = false;
            }
        }

        // Idle-driven auto-sync: bound the unflushed-data window so a
        // USB→battery brownout doesn't catch a NAND program in-flight.
        // No-op when nothing's pending or when writes are still hot;
        // costs ~200-500 ms only when actually flushing.  See
        // libs/base/fx_user_fs.cc note_write/FxUserMaybeIdleSync (Pas 2,
        // build #1226+).
        FxUserMaybeIdleSync();

        // === STORAGE MODE: REPL/HTTP are intentionally off, no activity
        // signal is possible.  Keep the hardware watchdog alive (protects
        // against CPU lockup) but skip the dead/warn logic entirely — the
        // user is working with the board as a USB disk and expects no
        // auto-reset until they exit storage mode themselves.
        if (sentai_storage_mode_active()) {
            WDOG_Refresh(WDOG1);
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
            consecutive_kicks++;
            
            // Log periodic status (every 60 kicks = ~5 minutes)
            if (consecutive_kicks % 60 == 0) {
                SERR_LOG(SERR_WDG_KICK, now / 1000);
            }
            continue;
        }
        
        // (The former 60-120 s "WARNING ZONE" that wrote a WDG_WARN entry
        // plus a task dump to the card every 10 s was removed 2026-10-31:
        // six SD writes per coffee break, no diagnostic value once idle is
        // not a fault.  kDeadThresholdMs is kept only as the tier boundary.)
        (void)kDeadThresholdMs;
        consecutive_kicks = 0;

        // === LONG IDLE (>= kWarningThresholdMs without REPL/HTTP traffic) ===
        // 2026-10-30 hardening: this used to be the "DEAD ZONE" — stop
        // kicking WDOG1 and force a reset.  That reset a perfectly healthy
        // board that simply had no host talking to it (every idle pause
        // on a RAM-loaded image dropped the board to ROM SDP).  Idle is
        // NOT a fault: a hung scheduler is already caught by the hardware
        // (this task stops running -> WDOG1 fires), and a hung MicroPython
        // script is bounded by the ctrl-c monitor / main.py timeout.  So we
        // keep kicking, keep the diagnostics (one task snapshot per
        // 10 minutes of idle), and never reset on idleness alone.
        WDOG_Refresh(WDOG1);
        if (now - last_warning_time > 600000u) {
            last_warning_time = now;
            char buf[64];
            snprintf(buf, sizeof(buf), "idle=%lu H=%lu R=%lu",
                min_idle / 1000, http_idle / 1000, repl_idle / 1000);
            crash_log_write("WDG_IDLE", buf);
            SERR_LOG(SERR_WDG_WARN, min_idle / 1000);
            crash_log_task_dump("WDG_IDLE");
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
// One slice of console output: converts bare \n to \r\n into a stack
// buffer and forwards it.  `size` <= 256 so the patched copy (<= 512)
// always fits — the old single-buffer version silently TRUNCATED any
// chunk whose patched length exceeded 511 bytes (audit M11: "prints
// > ~1 KB lose data").
static void write_slice(const char* buffer, int size) {
    char stack_buf[512];
    const char* out = buffer;
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
        for (int i = 0; i < size; ++i) {
            if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
                stack_buf[j++] = '\r';
            }
            stack_buf[j++] = buffer[i];
        }
        out = stack_buf;
        out_len = j;
    }

    // FR debug is a bounded, best-effort ring drained by sentai_fr_task.
    // Keep it before the verbose gate so missions can silence USB console
    // spam while still preserving the debug trace in /fr/debug.log if opened.
    sentai_fr_push_debug(out, out_len);

    // Silent kill-switch for the USB console only.  When verbose=0 (e.g.
    // while the detection pipeline is running), nothing reaches
    // ConsoleM7::Write — this prevents the CDC-ACM bulk-IN endpoint from
    // stalling tx_task and the REPL.  FR debug above remains best-effort.
    if (!g_sentai_frame_verbose) return;

    coralmicro::ConsoleM7::GetSingleton()->Write(const_cast<char*>(out), out_len);
    boot_log_write(const_cast<char*>(out), out_len);
}

extern "C" int _write(int handle, char* buffer, int size) {
    if ((handle != STDOUT_FILENO) && (handle != STDERR_FILENO)) {
        return -1;
    }
    // Fault / ISR context (HardFault handlers, stack-overflow and malloc
    // hooks, PendSV): the console path takes RTOS locks and allocates —
    // illegal there and the cause of nested faults that clobbered the
    // breadcrumb (audit M1).  Those paths already saved their breadcrumb
    // with sentai_fault_save(); drop the text.
    if (xPortIsInsideInterrupt()) return size;

    for (int off = 0; off < size; off += 256) {
        int n = size - off;
        if (n > 256) n = 256;
        write_slice(buffer + off, n);
    }
    return size;
}

// Fault-context append into the crash-text slot (2026-10-31 audit
// base-M1).  Called from sentai_fault.cc while the core is in a fault
// handler, PendSV (stack-overflow hook) or with the scheduler suspended
// (malloc-failed hook): no locks, no libc, no RTOS calls -- nothing else
// runs at that point and a reset follows.  Linear slot, 1008 bytes; an
// overflow sets `truncated` instead of wrapping so the head of the record
// (code / PC / LR / CFSR) always survives.
extern "C" void sentai_fault_text(const char* s, size_t len) {
    if (!s || len == 0) return;
    if (g_crash_text_persist.magic != kCrashTextMagic ||
        g_crash_text_persist.check != ~kCrashTextMagic ||
        g_crash_text_persist.len > kCrashTextSize) {
        g_crash_text_persist.magic = kCrashTextMagic;
        g_crash_text_persist.check = ~kCrashTextMagic;
        g_crash_text_persist.len   = 0;
        g_crash_text_persist.truncated = 0;
    }
    size_t pos = g_crash_text_persist.len;
    size_t space = kCrashTextSize - pos;
    if (len > space) { len = space; g_crash_text_persist.truncated = 1; }
    for (size_t i = 0; i < len; i++) g_crash_text_persist.text[pos + i] = s[i];
    g_crash_text_persist.len = pos + len;
}

// Export for micropython_task.c to call when REPL starts
extern "C" void sentai_boot_log_stop(void) {
    boot_log_stop();
}

// Strong override of the weak default in libs/usb/usb_device_task.cc.
// The string is published as the USB Product descriptor so that
// `lsusb -v -d 1fc9:c0a1` (or just `lsusb`) shows the running build
// number — quick visual confirmation that flashtool succeeded.
// Board serial = the 64-bit OCOTP UNIQUE_ID (fuse words 0x900/0x910), as 16
// lower-case hex digits -- the same string the USB iSerial descriptor
// carries (coralmicro::GetSerialNumber).  Stable per chip, so hosts can
// target one board with /dev/serial/by-id even with several plugged in.
extern "C" const char *sentai_board_serial(void) {
    static char s[17];
    static bool init = false;
    if (!init) {
        std::string ser = coralmicro::GetSerialNumber();
        snprintf(s, sizeof(s), "%s", ser.c_str());
        init = true;
    }
    return s;
}

// Short board id = last 4 hex digits of the serial (enough to tell the
// bench boards apart at a glance); shown in sentai.version() and in the
// USB product string so plain `lsusb` lists build AND board.
extern "C" const char *sentai_board_id_short(void) {
    const char *ser = sentai_board_serial();
    size_t n = strlen(ser);
    return (n >= 4) ? ser + n - 4 : ser;
}

extern "C" const char *sentai_build_version_string(void) {
    static char s[96];
    static bool init = false;
    if (!init) {
        snprintf(s, sizeof(s),
                 "autonomous.ro SentAI v1.1 build #%d board %s", BUILD_VERSION,
                 sentai_board_id_short());
        init = true;
    }
    return s;
}

namespace coralmicro {

// External-linkage state — accessed by sentai_slow_bridge.cc (OCRAM)
// Aligned to 32 bytes so the TFLite input tensor (which inherits the arena's
// base alignment) lands on an eDMA 32-byte burst boundary.  TFLite Micro's
// per-tensor allocator then rounds tensor offsets to its own 16-byte stride,
// but as long as the arena itself is 32-aligned the odds of hitting a 32-byte
// boundary for the input tensor are high for typical sizes.  See
// detection_task.cc:sentai_dma_memcpy — 32-byte width path yields ~300 MB/s
// vs 50 MB/s for the 8-byte fallback on SEMC SDRAM.
// 2026-04-24: 1 MB → 2 MB.  yolo26 768×512 reports 1162 KB used —
// the previous 1 MB array overflowed by 138 KB into adjacent
// .sdram_bss (micropython/http/audio/BLE BSS) because
// sentai_slow_bridge.cc was still passing 8 MB as the arena size
// to MicroInterpreter.  2 MB matches the updated kTensorArenaSize
// in the slow bridge and gives ~1.8× headroom for this model.
// Arena stays in SDRAM; OCRAM relocation is a separate follow-up.
uint8_t tensor_arena[2 * 1024 * 1024]
    __attribute__((aligned(32)))
    __attribute__((section(".sdram_bss,\"aw\",%nobits @")));

// Multi-slot extension (Phase 1 retry, 2026-04-28).  Slots 1 and 2
// arenas are HEAP-allocated, NOT placed in .sdram_bss — adding more
// static buffers there shifts the FileX/LevelX state and silently
// breaks NAND I/O (see project_filex_layout_fragility.md).  malloc
// from m_heap (16 MB free) is fine; we lazily allocate on first
// load_slot(N>0) call so unused slots cost zero memory.
constexpr int kNumTpuSlots = 3;
constexpr int kSlotArenaSize = 2 * 1024 * 1024;
uint8_t* g_slot_arena[kNumTpuSlots] = {
    tensor_arena, nullptr, nullptr,   // slot 0 fixed; slots 1/2 lazy-alloc
};
tflite::MicroInterpreter* g_slot_interp[kNumTpuSlots] = {
    nullptr, nullptr, nullptr,
};
std::vector<uint8_t>* g_slot_model_data[kNumTpuSlots] = {
    nullptr, nullptr, nullptr,
};
volatile bool g_slot_ready[kNumTpuSlots] = { false, false, false };

// Phase 2b note (2026-04-28): per-slot output-type dispatch (NMS /
// CLASSIFY / 2HEAD post-processing) was scoped out for now.  Output
// stays RAW per slot — REPL reads bytes via `sentai.tpu.output_slot(N, idx)`,
// host-side does whatever post-processing it wants.  When a real
// consumer needs typed output, revisit per the design sketch in
// experiment.md "Phase 2b output type design".

// Slot 0 keeps its legacy variable names — they're aliases for the
// slot-array entries.  Other TUs (sentai_slow_bridge.cc) use the
// legacy names; we keep them as separate-but-mirrored storage to
// avoid the cross-TU C++ reference linkage corner cases that bit us
// in the first attempt.
tflite::MicroInterpreter* g_interpreter = nullptr;
volatile bool g_tpu_ready = false;
std::vector<uint8_t>* g_model_data = nullptr;
std::shared_ptr<EdgeTpuContext> g_tpu_context;
// Runtime-selectable Edge TPU clock (kLow 63 / kMedium 125 / kHigh 250 / kMax 500 MHz).
// Set via sentai.tpu.set_mode(n); read by Main()'s OpenDevice and by the
// re-open retry in sentai_load_model.  Default kMax (was hardcoded pre-#1590).
PerformanceMode g_tpu_mode = PerformanceMode::kMax;

// Slot 0 ↔ legacy global sync is done INLINE in
// sentai_load_model_slot() (sentai_slow_bridge.cc) — both stores
// happen back-to-back when slot==0.  Earlier `sync_slot0_to_legacy` /
// `sync_legacy_to_slot0` helpers were removed (build #1102) as
// unused dead code.

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

  // applsci energy sweep (WS0): persist the Edge TPU clock across reboots via a
  // one-char file "/tpu_mode" ('0'=kLow '1'=kMedium '2'=kHigh '3'=kMax).  Read
  // here so the device is opened ONCE, cleanly, at the requested clock — no
  // runtime set_mode() reopen (which can wedge the USB/TPU).  Missing/invalid
  // file => keep the compile-time default (kMax).
  {
    uint8_t mb[4] = {0};
    if (::FxUserReadFile("/tpu_mode", mb, sizeof(mb)) > 0) {
      switch (mb[0]) {
        case '0': g_tpu_mode = PerformanceMode::kLow;    break;
        case '1': g_tpu_mode = PerformanceMode::kMedium; break;
        case '2': g_tpu_mode = PerformanceMode::kHigh;   break;
        case '3': g_tpu_mode = PerformanceMode::kMax;    break;
        default: break;
      }
    }
  }

  // Open EdgeTPU on this task (not app_main) to avoid interfering with USB init
  coralmicro::PerformanceMode tpu_mode = g_tpu_mode;
  g_tpu_context = EdgeTpuManager::GetSingleton()->OpenDevice(tpu_mode);
  sentai_health_periph(PERIPH_TPU, g_tpu_context ? 1 : 0);
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
    // This hook runs from vTaskSwitchContext inside PendSV (handler mode):
    // printf / SERR_LOG / dmesg take task-level critical sections there
    // and trip configASSERT, which re-entered the reset path and lost the
    // record (2026-10-31 audit base-M1).  Breadcrumb + text go through the
    // fault-safe path instead; the text surfaces in /log/boot_prev.log.
    sentai_fault_stack_overflow_record(pcTaskName, name_hash,
                                       (uint32_t)__builtin_return_address(0));
    // Reset immediately — watchdog/boot counter will handle repeated failures
    (*(volatile uint32_t*)0xE000ED0CUL) = (0x5FAUL << 16U) | (1UL << 2U);
    for (;;) {}
}

// SD-card user FS entry points used by app_main (libs/base/fx_user_fs.cc,
// libs/base/main_freertos_m7.cc).  Linkage specs must be at file scope.
extern "C" int  FxUserBootMount(uint32_t deadline_ms);
extern "C" int  FxUserOpenBlockOnly(void);
extern "C" int  sentai_storage_log_flush_to_fs(void);
extern "C" void sentai_msc_set_unit_ready(int on);

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

  // Initialize the sentai_prep slot table (refcounts=0, frame_div=1,
  // counters=0).  BSS-zero is defensive (sentai_prep_tick_frame
  // treats frame_div==0 as 1), but an explicit init keeps the
  // contract clean per [[op-s10-w11-prep-pipeline]] and avoids
  // surprise if BSS layout ever changes.
  extern void sentai_prep_init(void);
  sentai_prep_init();

  // Initialize boot logging FIRST (before any printf)
  boot_log_init();
  sentai_boot_progress_mark(0x11);
  
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nSentAI build #%d (%s)\r\n", BUILD_VERSION, BUILD_TIMESTAMP);

  // ArUco benchmark exposed via sentai.diag.aruco_bench() from REPL.

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

  // Dump phase-corr breadcrumbs from previous run if any.  Survives
  // CPU reset / WDOG via .sdram_bss persistence.  Ring is
  // self-clearing on first use, so nominal boot prints "no prior".
  extern void sentai_flow_phase_corr_dump_bc(void);
  sentai_flow_phase_corr_dump_bc();

  // === User filesystem: microSD card (2026-10-30) ===
  // FileX over fx_sd_driver, mounted HERE (post-scheduler) because the
  // USDHC host blocks on FreeRTOS events.  Outcomes:
  //   mounted      -> boot_log_fs_init() below writes /log/* to the card
  //   no/bad card  -> SAFE MODE: FxUserIsMounted()==0, every FxUser* call
  //                   returns a clean error, logs stay in the SDRAM ring,
  //                   USB CDC + REPL + radio are unaffected (anti-brick §M).
  // Storage mode opens only the block layer and then tells MSC the medium
  // is present, so the host mounts the raw card like a USB card reader.
  {
    if (!sentai_storage_mode_active()) {
      /* Bounded: 20 s covers a 64 GB FAT32 free-cluster scan with margin;
       * a corrupt volume that never returns leaves us in SAFE MODE. */
      (void)FxUserBootMount(20000u);
      sentai_health_periph(PERIPH_SD, FxUserIsMounted() ? 1 : 0);
      if (FxUserIsMounted()) {
        /* A previous storage-mode session may have left a debug log in
         * SDRAM; persist it now that the volume is mounted. */
        int storage_log_bytes = sentai_storage_log_flush_to_fs();
        if (storage_log_bytes > 0) {
          coralmicro::logf("[storage] flushed %d bytes of debug log to /log/storage_debug.log\r\n",
                           storage_log_bytes);
        }
      } else {
        coralmicro::logf("[fs] SD card not mounted -- SAFE MODE (logs stay in RAM)\r\n");
      }
    } else {
      if (FxUserOpenBlockOnly()) {
        sentai_msc_set_unit_ready(1);
      } else {
        coralmicro::logf("[storage] SD block layer unavailable -- MSC reports no medium\r\n");
      }
    }
  }

  // Initialize boot log file on the (now mounted) user FS
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
    char cfsr_flags[128] = "";
    uint32_t cf = s_prev_crash.cfsr;
    // Bit positions per ARM DDI0403 (CFSR = MMFSR[7:0] | BFSR[15:8] |
    // UFSR[31:16]).  The previous table was shifted (0x200 was labelled
    // INVSTATE, 0x1000 PRECISERR ...) and mislabelled every bus fault
    // (found on the EXP-s249 crash probe, build 1645).  VECTTBL lives in
    // HFSR, not CFSR, and is not recorded in the breadcrumb.
    if (cf & 0x00000001UL) strncat(cfsr_flags, "IACCVIOL ",   sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000002UL) strncat(cfsr_flags, "DACCVIOL ",   sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000008UL) strncat(cfsr_flags, "MUNSTKERR ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000010UL) strncat(cfsr_flags, "MSTKERR ",    sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000100UL) strncat(cfsr_flags, "IBUSERR ",    sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000200UL) strncat(cfsr_flags, "PRECISERR ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000400UL) strncat(cfsr_flags, "IMPRECISERR ", sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00000800UL) strncat(cfsr_flags, "BUNSTKERR ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00001000UL) strncat(cfsr_flags, "BSTKERR ",    sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00010000UL) strncat(cfsr_flags, "UNDEFINSTR ", sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00020000UL) strncat(cfsr_flags, "INVSTATE ",   sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00040000UL) strncat(cfsr_flags, "INVPC ",      sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x00080000UL) strncat(cfsr_flags, "NOCP ",       sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x01000000UL) strncat(cfsr_flags, "UNALIGNED ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
    if (cf & 0x02000000UL) strncat(cfsr_flags, "DIVBYZERO ",  sizeof(cfsr_flags)-strlen(cfsr_flags)-1);
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
  // In storage mode the host owns the NAND volume through MSC. Keep the
  // CDC REPL alive so the operator can explicitly call sentai.usb.drive(0)
  // after unmounting the host drive, but do not start HTTP, FileX tasks, or
  // the sensing pipeline. Filesystem-facing Python APIs reject access while
  // sentai_usb_drive_get() is true.
  if (sentai_storage_mode_active()) {
    sentai_boot_progress_mark(0x13);
    coralmicro::logf(
        "** STORAGE MODE active — MSC + REPL enabled, IP/pipeline disabled.\r\n"
        "** Unmount/eject the host drive, then run sentai.usb.drive(0)\r\n"
        "** from REPL to return to default mode.\r\n");
    xTaskCreate(RecoveryWatchdogTask, "stg_wdog", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
    sentai_health_boot_complete();
    // Reaching here means storage-mode boot survived all early init (USB
    // descriptor registration, MSC class init, FreeRTOS scheduler).  Clear
    // the crash-loop counter so the next drive(1) gets a fresh budget.
    sentai_storage_boot_succeeded();

    coralmicro::logf("Starting MicroPython REPL task in storage mode...\r\n");
    micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);
    for (;;) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // Launch MicroPython REPL task (interactive Python over serial)
  coralmicro::logf("Starting MicroPython REPL task...\r\n");
  micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);

  // Start combined watchdog (monitors HTTP + REPL, auto-resets if both dead)
  start_network_watchdog();
  coralmicro::logf("Watchdog started (WDOG1 30 s; resets only if the scheduler stalls — idle is not a fault)\r\n");

  // Board bring-up (2026-10-31): probe the IMU once (wake + WHO_AM_I, ~60 ms)
  // so the STAT LED can report it, then start the peripheral indicator:
  // blink code per missing peripheral for 60 s (1 SD, 2 cam0, 3 cam1, 4 IMU,
  // 5 TPU), three short flashes when everything answered.
  { extern int sentai_imu_init(void); sentai_health_periph(PERIPH_IMU, sentai_imu_init() == 0 ? 1 : 0); }
  sentai_health_led_start();

  // Mark boot complete — transitions from BOOTING to NORMAL (if healthy)
  sentai_health_boot_complete();
  coralmicro::logf("Boot complete, system mode: %s\r\n",
      sentai_health_system_mode() == SYS_MODE_NORMAL ? "NORMAL" :
      sentai_health_system_mode() == SYS_MODE_DEGRADED ? "DEGRADED" :
      sentai_health_system_mode() == SYS_MODE_SAFE ? "SAFE" : "?");

  // M4 offload DISABLED 2026-05-05 (build #1130+): SAD moved to M7
  // entirely (see flow_task.cc header).  M4 had unreliable SysTick
  // calibration without BOARD_InitBootClocks (would have reset M7's
  // camera/I2C if called).  M7 has cycles to spare and avoids the
  // cross-core sync + watchdog complications.
  coralmicro::logf("M4 offload: disabled (flow runs on M7).\r\n");

  coralmicro::Main();
  // Main() parks itself with vTaskSuspend - never returns
}

// Runtime opt-in for the M4 core.  Called by MicroPython via
// sentai.flow.m4_enable().  Once started, the M4 cannot be stopped
// without a full system reset.
//
// Liveness is determined via the shared-memory magic the M4 writes
// in flow_task_m4.cc::app_main — the RPMsg-based `M4IsAlive()` path
// is not wired up on our build (we skip IpcM4::Init on purpose to
// avoid re-initialising the M7-owned pins/clocks), so it always
// times out even though the M4 is perfectly healthy.
//
// Return codes:
//    0  : already running, or just started successfully
//   -1  : no M4 image linked into this firmware
//   -3  : M4 did not publish its magic within 2 s of StartM4 — fatal
//         for the flow feature but does NOT touch M7 operation.
// (sentai_flow_m4_enable / sentai_flow_enable now defined in
// flow_task.cc -- M4 stack retired 2026-05-05.)

// ===================== C bridge for MicroPython =====================
// Called from modsentai.c (C code) - need extern "C" linkage

// Forward declaration (defined after sentai_tpu_invoke).
extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);

// sentai_load_model, sentai_load_image → moved to sentai_slow_bridge.cc (OCRAM)

// Internal invoke — no pipeline guard.  Called by detection_task.cc.
// s240: the TPU invoke's USB transfers starve SysTick (the tick ISR is
// preempted by the high-priority USB host task), so xTaskGetTickCount()
// UNDER-COUNTS during Invoke() — it reported ~6 ms for a real 35 ms invoke,
// making every invoke benchmark bogus (167 "fps", physically impossible over
// USB2).  Use the GPT1-backed run-time counter (vPortGetRunTimeCounterValue,
// microseconds, hardware timer, not ISR-dependent — same source as
// sentai.rtos.micros()) and return real milliseconds.
extern "C" uint32_t vPortGetRunTimeCounterValue(void);
extern "C" uint8_t* volatile g_sentai_tpu_input_override[4];
extern "C" int sentai_tpu_invoke_internal(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  // EXP-s269 (2026-11-02): the plain invoke (tpu.load_image / set_input + tpu.invoke)
  // never fed the TPU either -- EdgeTpuExecutable reads the EvalTensor's data
  // pointer, which is not the persistent TfLiteTensor data that load_image fills
  // (same mechanism as the W24-T16 direct-path fix; fingerprint identical for a
  // black, a white and a natural image).  Publish every input tensor's persistent
  // data pointer through g_sentai_tpu_input_override for the duration of the call
  // unless a caller already set one.
  auto* interp = coralmicro::g_interpreter;
  const int n_in = (int)interp->inputs().size();
  uint8_t* prev[4] = {nullptr, nullptr, nullptr, nullptr};
  bool set[4] = {false, false, false, false};
  for (int k = 0; k < n_in && k < 4; ++k) {
    prev[k] = g_sentai_tpu_input_override[k];
    if (!prev[k]) {
      auto* t = interp->input_tensor(k);
      if (t && t->data.uint8) { g_sentai_tpu_input_override[k] = t->data.uint8; set[k] = true; }
    }
  }
  uint32_t t0 = vPortGetRunTimeCounterValue();
  TfLiteStatus rc = interp->Invoke();
  uint32_t t1 = vPortGetRunTimeCounterValue();
  for (int k = 0; k < 4; ++k) if (set[k]) g_sentai_tpu_input_override[k] = prev[k];
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) / 1000u);   // µs -> ms (real)
}

// Direct-input invoke: swap the TFLite input tensor's data pointer to
// `input_buf` for the duration of this Invoke, then restore the original
// arena-backed pointer before returning.  This lets PrepTask write the
// per-frame bytes directly into its own 32-byte aligned ping-pong buffer
// and have InferTask point TFLite at that buffer — eliminating the
// staging→tensor memcpy entirely.
//
// Safety (embeded.md §C/§F):
//   - Both saves and the Invoke happen on the InferTask thread; no
//     concurrent mutator touches input->data.uint8 while we own it.
//   - The restore runs on EVERY exit path (success, Invoke failure, bad
//     input tensor) so a subsequent legacy-path call or external caller
//     of `sentai_get_tensor_info` observes the arena pointer.
//   - `input_buf` is trusted — the caller owns the buffer's lifetime
//     and must keep it valid until this function returns.
// 2026-11-01 (EXP-s262 finding): patching input_tensor(k)->data alone does
// NOT reach the Edge TPU op -- TFLM copies the pointer into that TfLiteTensor
// at allocation and the op reads the EvalTensor (verified on the board: the
// output was scene-independent and equal to an invoke on the untouched arena).
// The executable now honours g_sentai_tpu_input_override[k]; the wrappers
// publish the caller's buffers there for the duration of Invoke().
extern "C" uint8_t* volatile g_sentai_tpu_input_override[4];

extern "C" int sentai_tpu_invoke_with_input(uint8_t* input_buf) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (input_buf == nullptr) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input) return -4;
  uint8_t* const saved = input->data.uint8;
  input->data.uint8 = input_buf;
  g_sentai_tpu_input_override[0] = input_buf;
  uint32_t t0 = vPortGetRunTimeCounterValue();   // µs (SysTick starves during invoke)
  TfLiteStatus rc = coralmicro::g_interpreter->Invoke();
  uint32_t t1 = vPortGetRunTimeCounterValue();
  g_sentai_tpu_input_override[0] = nullptr;
  input->data.uint8 = saved;   // restore on ALL paths (no early return between)
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) / 1000u);   // µs -> ms (real)
}

// Public invoke — blocks while detection pipeline owns the TPU.
extern "C" int sentai_tpu_invoke(void) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns TPU
  return sentai_tpu_invoke_internal();
}

// ---- Multi-slot invoke / accessors (Phase 1 retry, 2026-04-28) ----

extern "C" int sentai_tpu_slot_count(void) { return coralmicro::kNumTpuSlots; }

extern "C" int sentai_tpu_slot_ready(int slot) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  return (coralmicro::g_slot_ready[slot]
          && coralmicro::g_slot_interp[slot]) ? 1 : 0;
}

extern "C" int sentai_tpu_invoke_slot(int slot) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return -3;
  if (slot == 0 && sentai_detection_is_running()) return -10;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return -1;
  auto* interp = coralmicro::g_slot_interp[slot];
  uint32_t t0 = vPortGetRunTimeCounterValue();   // µs (SysTick starves during invoke)
  if (interp->Invoke() != kTfLiteOk) return -2;
  uint32_t t1 = vPortGetRunTimeCounterValue();
  return (int)((t1 - t0) / 1000u);   // µs -> ms (real)
}

// Same pointer-patch trick as sentai_tpu_invoke_with_input for slot N.
// Caller passes a buffer (typically OCRAM .tpu_input shared across
// slots for the future per-cam pipeline dispatcher).
extern "C" int sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return -3;
  if (slot == 0 && sentai_detection_is_running()) return -10;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return -1;
  if (buf == nullptr) return -3;
  auto* interp = coralmicro::g_slot_interp[slot];
  auto* input = interp->input_tensor(0);
  if (!input) return -4;
  uint8_t* const saved = input->data.uint8;
  input->data.uint8 = buf;
  g_sentai_tpu_input_override[0] = buf;
  uint32_t t0 = vPortGetRunTimeCounterValue();   // µs (SysTick starves during invoke)
  TfLiteStatus rc = interp->Invoke();
  uint32_t t1 = vPortGetRunTimeCounterValue();
  g_sentai_tpu_input_override[0] = nullptr;
  input->data.uint8 = saved;
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) / 1000u);   // µs -> ms (real)
}

// s240/YUV420: dual-input direct invoke.  Binds input[0]=UV, input[1]=Y (the
// c2f_thick_yuv420 model order — verified via the tflite: input[0]
// serving_default_uv [1,240,320,2], input[1] serving_default_y [1,480,640,1]),
// invokes, and restores both data pointers.  Same zero-copy pointer-patch as
// sentai_tpu_invoke_with_input, extended to two planes.  Returns real ms.
extern "C" int sentai_tpu_invoke_with_inputs2(uint8_t* uv_buf, uint8_t* y_buf) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* interp = coralmicro::g_interpreter;
  if ((int)interp->inputs().size() < 2) return -5;   // not a dual-input model
  auto* in0 = interp->input_tensor(0);   // UV
  auto* in1 = interp->input_tensor(1);   // Y
  if (!in0 || !in1) return -4;
  uint8_t* const s0 = in0->data.uint8;
  uint8_t* const s1 = in1->data.uint8;
  in0->data.uint8 = uv_buf;
  in1->data.uint8 = y_buf;
  g_sentai_tpu_input_override[0] = uv_buf;   // node input 0 = UV
  g_sentai_tpu_input_override[1] = y_buf;    // node input 1 = Y
  uint32_t t0 = vPortGetRunTimeCounterValue();
  TfLiteStatus rc = interp->Invoke();
  uint32_t t1 = vPortGetRunTimeCounterValue();
  g_sentai_tpu_input_override[0] = nullptr;
  g_sentai_tpu_input_override[1] = nullptr;
  in0->data.uint8 = s0;
  in1->data.uint8 = s1;
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) / 1000u);
}

extern "C" int sentai_tpu_num_outputs_slot(int slot) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return 0;
  return (int)coralmicro::g_slot_interp[slot]->outputs().size();
}

extern "C" int sentai_tpu_get_output_size_slot(int slot, int idx) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return 0;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  return (int)interp->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tpu_get_output_data_slot(int slot, int idx) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return NULL;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return NULL;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return NULL;
  return interp->output_tensor(idx)->data.data;
}

// Phase 2b raw introspection: shape / type / element-count of a slot's
// output tensor.  Lets a REPL caller iterate the bytes meaningfully
// without doing post-processing in MicroPython (which is too slow / heap
// constrained anyway — the structured post-proc will live in C++).
extern "C" int sentai_tpu_get_output_num_dims_slot(int slot, int idx) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return 0;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  return interp->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tpu_get_output_dim_slot(int slot, int idx, int dim) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return 0;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  auto* t = interp->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tpu_get_output_type_slot(int slot, int idx) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return -1;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return -1;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return -1;
  return (int)interp->output_tensor(idx)->type;
}

extern "C" int sentai_tpu_output_quant_slot(int slot, int idx,
                                             float* scale, int32_t* zp) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return -1;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return -1;
  auto* interp = coralmicro::g_slot_interp[slot];
  if (idx < 0 || idx >= (int)interp->outputs().size()) return -1;
  auto* t = interp->output_tensor(idx);
  if (!t) return -1;
  *scale = t->params.scale;
  *zp = t->params.zero_point;
  return 0;
}

// Set the input tensor of slot N from a host-supplied buffer.  Used by
// the validation driver to inject a deterministic input across slots
// and confirm 3 distinct output hashes.
extern "C" int sentai_tpu_set_input_slot(int slot,
                                          const uint8_t* data, int len) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return -3;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return -1;
  auto* input = coralmicro::g_slot_interp[slot]->input_tensor(0);
  if (!input) return -4;
  if (len != (int)input->bytes) return -5;
  memcpy(input->data.uint8, data, len);
  return 0;
}

// FNV-1a 32-bit hash over the concatenated output tensors of slot N.
// Used to verify multi-slot independence — 3 distinct hashes from 3
// slots running on the same input prove 3 different models execute.
extern "C" uint32_t sentai_tpu_output_hash_slot(int slot) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) return 0;
  if (!coralmicro::g_slot_ready[slot] || !coralmicro::g_slot_interp[slot])
    return 0;
  auto* interp = coralmicro::g_slot_interp[slot];
  uint32_t h = 0x811C9DC5u;
  int n = (int)interp->outputs().size();
  for (int oi = 0; oi < n; oi++) {
    auto* t = interp->output_tensor(oi);
    if (!t || !t->data.data) continue;
    const uint8_t* p = (const uint8_t*)t->data.data;
    int len = (int)t->bytes;
    for (int i = 0; i < len; i++) {
      h ^= p[i];
      h *= 0x01000193u;
    }
  }
  return h;
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

// Headless YOLOv5 p3p4 decode (port of experiments/s236_model_speed_sweep_640x480/
// host_decode.py, the reference decoder used for every host-side mAP of these models):
// output k = [1, gh, gw, 5+C] raw logits [tx, ty, tw, th, obj, cls...]; stride =
// in_h / gh (480/30 = 16 for P4, 480/60 = 8 for P3), one anchor per level of
// 2*stride px (32 / 16); x = (sig(tx)*2 - 0.5 + gx)*stride, w = (sig(tw)*2)^2*anchor,
// conf = sig(obj) * sig(best class).  The sigmoid of the dequantised value is a
// 256-entry table per head (the logits are 8-bit), so the decode costs ~7k table
// look-ups per frame, no expf in the loop.  Fills g_nms_cand in input pixels,
// returns the candidate count.
static int yolo_decode_heads(float conf_thr, int in_h, int in_w) {
  using namespace coralmicro;
  (void)in_w;
  int num_cand = 0;
  static float lut[256];
  const int n_out = (int)g_interpreter->outputs().size();
  for (int k = 0; k < n_out && num_cand < kMaxNmsCandidates; ++k) {
    const TfLiteTensor* t = g_interpreter->output_tensor(k);
    if (!t || t->dims->size != 4 || t->dims->data[0] != 1) continue;
    const int gh = t->dims->data[1], gw = t->dims->data[2], ch = t->dims->data[3];
    const int C = ch - 5;
    if (C < 1 || gh <= 0 || gw <= 0) continue;
    const float stride = (float)in_h / (float)gh;
    const float anchor = 2.0f * stride;
    const float scale = t->params.scale;
    const int zp = t->params.zero_point;
    const bool is_u8 = (t->type == kTfLiteUInt8);
    for (int q = 0; q < 256; ++q) {
      const int raw = is_u8 ? q : (int)(int8_t)(uint8_t)q;
      lut[q] = 1.0f / (1.0f + expf(-scale * (float)(raw - zp)));
    }
    const uint8_t* u = reinterpret_cast<const uint8_t*>(t->data.data);
    for (int gy = 0; gy < gh && num_cand < kMaxNmsCandidates; ++gy) {
      for (int gx = 0; gx < gw && num_cand < kMaxNmsCandidates; ++gx) {
        const uint8_t* p = u + ((size_t)gy * gw + gx) * ch;
        const float obj = lut[p[4]];
        float best = -1.0f; int best_c = 0;
        for (int c = 0; c < C; ++c) {
          const float v = lut[p[5 + c]];
          if (v > best) { best = v; best_c = c; }
        }
        const float cf = obj * best;
        if (cf < conf_thr) continue;
        const float x = (lut[p[0]] * 2.0f - 0.5f + (float)gx) * stride;
        const float y = (lut[p[1]] * 2.0f - 0.5f + (float)gy) * stride;
        const float w2 = lut[p[2]] * 2.0f, h2 = lut[p[3]] * 2.0f;
        const float w = w2 * w2 * anchor, h = h2 * h2 * anchor;
        g_nms_cand[num_cand++] = {x - w * 0.5f, y - h * 0.5f, x + w * 0.5f, y + h * 0.5f, cf, (int16_t)best_c};
      }
    }
  }
  return num_cand;
}

extern "C" int sentai_tpu_detect(int conf_permil, int iou_permil,
                                 int max_dets,
                                 int16_t* out_buf, int* out_count) {
  using namespace coralmicro;
  *out_count = 0;
  if (!g_tpu_ready || !g_interpreter) return -1;

  auto* output = g_interpreter->output_tensor(0);
  if (!output) return -2;

  // Model input geometry = the LARGEST input tensor: the dual-input YUV420 models
  // carry UV [1,240,320,2] at index 0 and Y [1,480,640,1] at index 1.
  int in_h = 0, in_w = 0;
  for (size_t k = 0; k < g_interpreter->inputs().size(); ++k) {
    auto* t = g_interpreter->input_tensor(k);
    if (t && t->dims->size == 4) {
      if (t->dims->data[1] > in_h) in_h = t->dims->data[1];
      if (t->dims->data[2] > in_w) in_w = t->dims->data[2];
    }
  }
  if (in_h <= 0 || in_w <= 0) return -2;

  float conf_thr = conf_permil / 1000.0f;
  float iou_thr  = iou_permil  / 1000.0f;

  int num_cand = 0;
  float max_coord = 0.0f;

  YoloInfo info = {YoloLayout::kUnknown, 0, 0};
  float scale = 0.0f;
  int zp = 0;
  int N = 0, C = 0;
  // Headless exports (every iarna / yuv420 model on the card since 2026-04): two raw
  // grid heads [1,gh,gw,5+C] instead of one decoded [1,N,5+C] tensor.  Until build
  // 1767 this function returned -2 for them and the pipeline published 0 detections
  // for every frame (found by EXP-s269, 2026-11-02).
  const bool heads = (output->dims->size == 4);
  if (!heads) {
    if (output->dims->size != 3) return -2;
    info = yolo_infer_info(output);
    if (info.layout == YoloLayout::kUnknown) return -3;
    scale = output->params.scale;
    zp = output->params.zero_point;
    N = info.num_anchors;
    C = info.num_classes;
  }

  if (heads) {
    num_cand = yolo_decode_heads(conf_thr, in_h, in_w);
    max_coord = (float)in_w;   // candidates are already in input pixels
  } else if (info.layout == YoloLayout::kV5Like) {
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

extern "C" int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                                      uint8_t* dst, int dst_w, int dst_h);

extern "C" int sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                                 uint8_t* dst, int dw, int dh) {
  return pxp_scale_xrgb_to_rgb(src, sw, sh, dst, dw, dh);
}

extern "C" int sentai_pxp_transform(const uint8_t* src, int src_w, int src_h,
                                     sentai_pxp_format_t src_format,
                                     uint8_t* dst, int dst_w, int dst_h,
                                     sentai_pxp_format_t dst_format) {
  if (src_format == SENTAI_PXP_FORMAT_XRGB8888 &&
      dst_format == SENTAI_PXP_FORMAT_RGB888) {
    return sentai_pxp_scale(src, src_w, src_h, dst, dst_w, dst_h);
  }
  if (src_format == SENTAI_PXP_FORMAT_XRGB8888 &&
      dst_format == SENTAI_PXP_FORMAT_Y8) {
    return sentai_pxp_xrgb_to_y8(src, src_w, src_h, dst, dst_w, dst_h);
  }
  return -3;
}

extern "C" int sentai_pxp_rgb888_scale(const uint8_t* src, int src_w,
                                        int src_h, uint8_t* dst,
                                        int dst_w, int dst_h) {
  return sentai_pxp_transform(src, src_w, src_h, SENTAI_PXP_FORMAT_RGB888,
                              dst, dst_w, dst_h, SENTAI_PXP_FORMAT_RGB888);
}

extern "C" int sentai_pxp_rgb888_to_y8(const uint8_t* src, int src_w,
                                        int src_h, uint8_t* dst,
                                        int dst_w, int dst_h) {
  return sentai_pxp_transform(src, src_w, src_h, SENTAI_PXP_FORMAT_RGB888,
                              dst, dst_w, dst_h, SENTAI_PXP_FORMAT_Y8);
}

// W24-T11: byte size of input tensor `idx` of the main interpreter (-1 if
// not loaded / out of range).  PrepTask uses it to check that a dual-input
// YUV420 model matches the runtime camera geometry before staging planes.
extern "C" int sentai_get_input_bytes(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || (size_t)idx >= coralmicro::g_interpreter->inputs().size()) return -1;
  auto* t = coralmicro::g_interpreter->input_tensor(idx);
  return t ? (int)t->bytes : -1;
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

// Peek the first `len` bytes of the most-recent camera framebuffer.
// Used by visual-A/B / embedded-data probes to inspect raw XRGB8888
// pixels (and any sensor-injected metadata in the first lines).
// `dst` must be sized >= len (caller-supplied).  Returns bytes copied
// or negative on failure.
extern "C" int sentai_cam_peek_first_row(uint8_t* dst, int len) {
  if (!dst || len <= 0) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  // Buffer is XRGB8888 row layout: pitch =
  // (DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP.
  // We just copy len bytes starting at byte 0 — that's the first
  // pixels of line 0, which is where OV5640 puts embedded data when
  // 0x501F bit 7 is set.
  if (len > 4096) len = 4096;  // sanity cap
  memcpy(dst, raw, (size_t)len);
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
    return len;
  }
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->ReturnRawFrame(idx);
  return len;
}

// EXP-s259 / W24-T14: dump `nbytes` of the most-recent camera framebuffer,
// starting at `offset`, to a file on the user partition -- whatever the CSI
// receiver stored (XYUV, packed YUV420, Y8 ...).  The host reconstructs the
// image.  The slot is held during the SD write (~1 s/MB; the camera has four
// buffers).  Returns bytes written or negative.
extern "C" int sentai_cam_save_raw(const char* path, uint32_t offset, uint32_t nbytes) {
  if (!path || nbytes == 0) return -1;
  const uint32_t cap = (uint32_t)CAM_MAX_WIDTH * (uint32_t)CAM_MAX_HEIGHT * 4u;
  if (offset >= cap) return -3;
  if (offset + nbytes > cap) nbytes = cap - offset;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  int ok = FxUserWriteFile(path, raw + offset, (size_t)nbytes);
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
  } else {
    coralmicro::CameraTask::GetSingleton()->ReturnRawFrame(idx);
  }
  return ok ? (int)nbytes : -4;
}

// Build #935 — single-grab 5-row B-channel sample @ col 40.
// Returns the B byte at col 40 from rows {0, H/4, H/2, 3H/4, H-1}
// of the SAME framebuffer (single dequeue/return).  This is the
// scramble-detection primitive for diag/_t_pattern_31.py PHASE B:
// 5 different rows of ONE buffer, so any inconsistency means the
// MUX flip landed during sensor active rows (true mid-frame mix),
// not "5 different buffers from 5 different cameras".
//
// Also returns the cam_id of the buffer it sampled (via the existing
// per-buffer task tag), so the caller can stitch (frame_idx, cam_tag,
// 5 b40 samples) into one record per dequeue without re-grabbing.
//
// dst5: 5 bytes (caller-allocated) for the B-channel samples.
// Returns: cam_tag (0/1/-1 unknown) on success, negative on error.
extern volatile int g_cam_grabbed_id;
extern "C" int sentai_cam_peek5_b40(uint8_t* dst5) {
  if (!dst5) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  // Runtime geometry (W24-T20): rows spread over the live frame height and the
  // live row pitch (packed YUV420: byte 160 of the row is the C byte of pixel
  // pair 40, still one fixed byte per row, which is all this oracle needs).
  const size_t pitch = cam_src_pitch();
  const int fh = cam_src_h();
  const int kRows[5] = {0, fh / 4, fh / 2, (3 * fh) / 4, fh - 1};
  for (int i = 0; i < 5; ++i) {
    dst5[i] = raw[(size_t)kRows[i] * pitch + (size_t)40 * 4];
  }
  int tag = g_cam_grabbed_id;
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
    return tag;
  }
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->ReturnRawFrame(idx);
  return tag;
}

// EXP-s246 iter4 — same single-dequeue sampling as peek5_b40, but ALSO
// reports every per-buffer tag source for the buffer actually grabbed, so
// content can be scored against each one instead of trusting a comment.
//
// Three arrays exist and they do not agree:
//   g_cam_buf_id_task[]  written at dequeue in CameraTask::HandleFrameRequest
//                        from the SCALAR g_cam_last_completed_id -- i.e. from
//                        whichever camera finished a frame most recently, not
//                        necessarily the one that filled THIS buffer.  This is
//                        what the consumer currently believes.
//   g_cam_buf_id[]       written per-buffer by the CSI ISR at that buffer's
//                        own FB-done, from the pre-flip active_cam.
//   g_cam_buf_id_csi[]   documented "AUTHORITATIVE" but NEVER WRITTEN: build
//                        #944 reduced coralmicro_csi_on_frame_complete() to a
//                        bare counter that discards bufferAddr, so every slot
//                        stays 0xFF.  Reported here to make that visible.
//
// Tags are read BEFORE ReturnRawFrame -- once the slot is returned it can be
// re-armed and re-tagged, which would corrupt the comparison.
//
// out4 = { idx, tag_task, tag_isr, tag_csi }.
extern "C" int sentai_cam_peek5_tags(uint8_t* dst5, int32_t* out4) {
  // Local externs: the file-scope declarations for these live further down
  // (near the camera-switch state block), after this function.
  extern volatile uint8_t g_cam_buf_id[];
  extern volatile uint8_t g_cam_buf_id_csi[];
  if (!dst5 || !out4) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  const size_t pitch = cam_src_pitch();   // runtime geometry (W24-T20), see peek5_b40
  const int fh = cam_src_h();
  const int kRows[5] = {0, fh / 4, fh / 2, (3 * fh) / 4, fh - 1};
  for (int i = 0; i < 5; ++i) {
    dst5[i] = raw[(size_t)kRows[i] * pitch + (size_t)40 * 4];
  }
  out4[0] = idx;
  out4[1] = g_cam_grabbed_id;                 // task array (current behaviour)
  out4[2] = (idx < (int)DEMO_CAMERA_BUFFER_COUNT)
                ? (int32_t)g_cam_buf_id[idx] : -1;
  out4[3] = (idx < (int)DEMO_CAMERA_BUFFER_COUNT)
                ? (int32_t)g_cam_buf_id_csi[idx] : -1;
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
    return 0;
  }
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->ReturnRawFrame(idx);
  return 0;
}

// EXP-s251 — full-column profile of ONE buffer.  dst[r] = B byte of pixel
// (x, r) for r in [0, rows).  With the OV5640 vertical-gradient test
// patterns (0x503D = 0x84 / 0x8C) the profile is a known ramp, so a frame
// made of two sensor frames (vertical roll / seam after a MUX flip) shows as
// a discontinuity, which the 5-row oracle of EXP-s246 could not see.
// out3 = { idx, tag_task, tag_isr }, read before the buffer is returned.
// `byte` selects which byte of the 4-byte pixel is sampled: 0 = B (XYUV: X) in the
// legacy layout, 1 = Y1 in packed YUV420 ([C,Y1,Y2,0] per pixel pair) -- EXP-s266.
extern "C" int sentai_cam_peek_col_byte(int x, int byte, uint8_t* dst, int rows, int32_t* out3);
extern "C" int sentai_cam_peek_col(int x, uint8_t* dst, int rows, int32_t* out3) {
  return sentai_cam_peek_col_byte(x, 0, dst, rows, out3);
}
extern "C" int sentai_cam_peek_col_byte(int x, int byte, uint8_t* dst, int rows, int32_t* out3) {
  extern volatile uint8_t g_cam_buf_id[];
  extern volatile uint32_t g_cam_buf_ts_us[];
  if (byte < 0 || byte > 3) return -1;
  // Runtime geometry (EXP-s266): at SXGA the frame is 1280x960 and the row pitch
  // 1280*4 (640*4 in packed YUV420 mode), not the compile-time VGA constants --
  // with those this function read two columns interleaved over half the frame.
  const int fw = cam_src_w(), fh = cam_src_h();
  if (!dst || !out3 || x < 0 || x >= fw || rows <= 0 || rows > fh) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  const size_t pitch = cam_src_pitch();
  const uint8_t* p = raw + (size_t)x * DEMO_CAMERA_BUFFER_BPP + (size_t)byte;
  for (int r = 0; r < rows; ++r) dst[r] = p[(size_t)r * pitch];
  out3[0] = idx;
  out3[1] = g_cam_grabbed_id;
  out3[2] = (idx < (int)DEMO_CAMERA_BUFFER_COUNT) ? (int32_t)g_cam_buf_id[idx] : -1;
  // out3[3] = FB-done time of this buffer (GPT1 us), out3[4] = now (us):
  // age = now - ts tells a stale queued frame from a fresh one.
  out3[3] = (idx < (int)DEMO_CAMERA_BUFFER_COUNT) ? (int32_t)g_cam_buf_ts_us[idx] : 0;
  out3[4] = (int32_t)GPT1->CNT;
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
    return 0;
  }
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->ReturnRawFrame(idx);
  return 0;
}

// EXP-s251 frame log read-out: copies the last `n` FB-done entries
// (oldest first) under a short critical section.  Returns the count.
extern "C" int sentai_cam_frame_log(int n, uint32_t* ts, uint8_t* idx,
                                    uint8_t* cam, uint16_t* seq) {
  extern volatile uint32_t g_cam_flog_ts[];
  extern volatile uint8_t  g_cam_flog_idx[];
  extern volatile uint8_t  g_cam_flog_cam[];
  extern volatile uint16_t g_cam_flog_seq[];
  extern volatile uint32_t g_cam_flog_head;
  const int kN = 64;
  if (n > kN) n = kN;
  taskENTER_CRITICAL();
  uint32_t head = g_cam_flog_head;
  int avail = (head < (uint32_t)kN) ? (int)head : kN;
  if (n > avail) n = avail;
  for (int k = 0; k < n; ++k) {
    uint32_t i = (head - (uint32_t)n + (uint32_t)k) % (uint32_t)kN;
    ts[k] = g_cam_flog_ts[i]; idx[k] = g_cam_flog_idx[i];
    cam[k] = g_cam_flog_cam[i]; seq[k] = g_cam_flog_seq[i];
  }
  taskEXIT_CRITICAL();
  return n;
}

extern "C" void sentai_cam_return_raw(int idx) {
  if (sentai_virtual_camera_is_frame_idx(idx)) {
    sentai_virtual_camera_return_raw(idx);
    return;
  }
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
extern "C" int sentai_cam_current_id(void) {
  if (sentai_virtual_camera_active()) return sentai_virtual_camera_current_id();
  return g_cam_current_id;
}

/* Source camera that wrote the most recently COMPLETED buffer (NOT the
 * current MUX state).  Set by CSI ISR at FB2-done before the MUX flip.
 * Use this to label captures correctly during alt-mode switching. */
extern volatile int g_cam_last_completed_id;
extern "C" int sentai_cam_last_capture_id(void) { return g_cam_last_completed_id; }

/* Tag of the most recently GRABBED buffer — set by
 * sentai_cam_get_raw_with_recovery from the per-buffer tag array
 * g_cam_buf_id[idx].  This is the SOURCE camera that actually wrote
 * the buffer, regardless of MUX state at grab time. */
extern volatile int g_cam_grabbed_id;
extern volatile uint8_t g_cam_buf_id[];
extern volatile uint8_t g_cam_buf_id_task[];  // build #886 attempt
extern volatile uint8_t g_cam_buf_id_csi[];   // build #893 NXP-driver-hook (authoritative)
extern "C" int sentai_cam_grabbed_id(void) { return g_cam_grabbed_id; }
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

// ---------------------------------------------------------------------
// Camera-switch fault counters (embeded.md §F, §I).
// Written only by sentai_cam_switch / sentai_cam_get_raw_with_recovery.
// Single aligned 32-bit counters — read safely from any context.
// Exposed to MicroPython via `sentai.diag.cam_stats()` so an operator
// can see degraded paths accumulating even when the system appears OK.
// ---------------------------------------------------------------------
static volatile uint32_t g_cam_switch_ok_eof_count    = 0; // ISR consumed arm, glitch-free path
static volatile uint32_t g_cam_switch_fallback_count  = 0; // ISR did not consume within 150 ms
static volatile uint32_t g_cam_drain_timeout_count    = 0; // drain wait_iters hit 300 ms ceiling
static volatile uint32_t g_cam_grab_retry_count       = 0; // GetRawFrame failed, toggling
static volatile uint32_t g_cam_grab_fatal_count       = 0; // GetRawFrame failed after all recoveries

// Camera liveness snapshot (watchdog observer, see CombinedWatchdogTask).
extern "C" void sentai_cam_stall_get(uint32_t* count, uint32_t* stalled) {
  if (count)   *count   = g_cam_stall_count;
  if (stalled) *stalled = g_cam_stalled ? 1u : 0u;
}

extern "C" void sentai_cam_stats_get(uint32_t* ok_eof,
                                     uint32_t* fallback,
                                     uint32_t* drain_timeout,
                                     uint32_t* grab_retry,
                                     uint32_t* grab_fatal) {
  if (ok_eof)        *ok_eof        = g_cam_switch_ok_eof_count;
  if (fallback)      *fallback      = g_cam_switch_fallback_count;
  if (drain_timeout) *drain_timeout = g_cam_drain_timeout_count;
  if (grab_retry)    *grab_retry    = g_cam_grab_retry_count;
  if (grab_fatal)    *grab_fatal    = g_cam_grab_fatal_count;
}

// Stateless ratio-alternate scheduler.  When both quotas are > 0, the
// CSI ISR decides which camera should own frame N via a pure modulo of
// the monotonic frame counter (seq % (ratio_a + ratio_b)) < ratio_a →
// cam0, else cam1.  No mutable counter state in ISR context; the policy
// is a function of (seq, ratio_a, ratio_b) only.
//
// The two quotas are packed into a SINGLE 32-bit volatile so that task-
// context updates are atomic from the ISR's point of view (the ISR
// reads the packed value once, then unpacks).  Previously the two
// fields were separate volatiles and a task-context update could leave
// the ISR seeing (new_a, old_b) for one frame, producing a transient
// wrong flip.  See paper/cam_switch.md §"A2 race fix".
//
// Layout: high 16 bits = a (cam0 quota), low 16 bits = b (cam1 quota).
// Bounded to [0, 1000] per quota by sentai_cam_ratio_set, so 16 bits
// each is ample.
volatile uint32_t g_cam_ratio_packed = 0;

// Post-switch drain threshold (number of fresh ISR frames required after a
// MUX flip before a frame is considered clean).  Default 2 (one mixed
// post-flip frame + one fully-new frame).  Runtime-settable via
// `sentai.camera.switch_drain(n)` for A/B measurement — see E17
// experiment.  Bounded to [1, 10] at the setter to preserve
// analyzability: threshold=0 would return a pre-flip frame, threshold
// beyond 10 is not a sensible operating point on this 15 FPS pipeline.
// Non-static: the CSI ISR (libs/camera/camera_support.c) reads this when
// arming the post-switch countdown.
// 2026-04-22: default dropped 2→1 after switching the ISR counter to
// FB2-gated semantics.  Before the gate each ISR entry ticked the
// counter (~2× sensor rate), so threshold=2 historically meant "wait
// 1 real sensor frame".  Post-gate each tick is one sensor frame, so
// threshold=1 restores the historical per-sensor-frame-wait that
// delivered ~20 FPS/cam under continuous 1:1 alternation.  CSI MUX
// flip happens in VBLANK (see camera_support.c:148-157) so the very
// next DMA buffer is already from the new sensor → one frame wait is
// correct, not overcautious.
volatile uint32_t g_cam_switch_drain_threshold = 1;

extern "C" uint32_t sentai_cam_switch_drain_get(void) {
  return g_cam_switch_drain_threshold;
}

extern "C" int sentai_cam_switch_drain_set(uint32_t n) {
  // 2026-04-25: lower bound relaxed 1→0 for the "no drain wait" A/B.
  // threshold=0 = always fast path (take whatever's in queue).
  if (n > 10) return -1;
  g_cam_switch_drain_threshold = n;
  return 0;
}

// Set the auto-alternate ratio.  Both zero disables auto-alternation.
// Bounds [0, 1000] — max practical quota is well under 1000 frames; we
// reject larger values to avoid surprise wraparound semantics when
// (a+b) is used as a modulus against a 32-bit frame counter, and to
// keep each quota inside 16 bits for the packed atomic update.
//
// Single 32-bit store: atomic on Cortex-M7; the ISR either sees the
// old pair or the new pair, never a mix.
extern "C" int sentai_cam_ratio_set(uint32_t a, uint32_t b) {
  if (a > 1000u || b > 1000u) return -1;
  g_cam_ratio_packed = (a << 16) | (b & 0xFFFFu);
  return 0;
}

extern "C" void sentai_cam_ratio_get(uint32_t* a, uint32_t* b) {
  uint32_t packed = g_cam_ratio_packed;  // single atomic read
  if (a) *a = (packed >> 16) & 0xFFFFu;
  if (b) *b = packed & 0xFFFFu;
}

// ===================== Audio externs for AIfES =====================
// Uses the existing mic implementation in modsentai_hal.cc

extern "C" int sentai_cam_is_initialized(void) {
  if (sentai_virtual_camera_active()) return 1;
  return g_cam_initialized ? 1 : 0;
}

// PXP hardware scale+convert: XRGB8888 (native cam) -> RGB888 (scaled output).
// src must be in non-cacheable memory (camera framebuffer).
// dst must be 64-byte aligned for best results.
// Returns 0 on success.

// W24-T11: PXP completion deadline.  50 ms is sized for a VGA frame (repack
// 7-8 ms, scale ~5 ms); an SXGA XYUV frame is 4x the pixels and the repack
// measured 42 ms at a 15 fps camera on the 16-bit SDRAM, so the deadline is
// scaled with the sensor geometry in sentai_cam_set_hw().
extern "C" volatile uint32_t g_sentai_pxp_deadline_ms = 50;

/* Bounded PXP completion wait (2026-10-30 audit M7).  The Y8 output path
 * is documented to hang the PXP at 8:1 scaling (build 1562), and every
 * caller used to spin on the complete flag with no deadline.  50 ms is
 * >10x the longest legitimate VGA conversion.  On timeout the PXP is reset

 * so the next conversion starts clean; the caller must fail the frame. */
static bool sentai_pxp_wait_complete(void) {
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(g_sentai_pxp_deadline_ms);
  while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP))) {
    // W24-T11: PrepTask and InferTask share priority 2 with time slicing
    // OFF, so a busy-poll here starves InferTask's per-chunk URB submits for
    // the whole PXP pass (42 ms at SXGA -> prep and invoke serialised, 9.5
    // fps instead of ~15).  Yield each poll; costs nothing when nobody else
    // of equal priority is ready.
    taskYIELD();
    if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) {
      PXP_Reset(DEMO_PXP);
      return false;
    }
    taskYIELD();
  }
  PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);
  return true;
}

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
  // kPXP_PsPixelFormatRGB888 = 0x4 = "32-bit pixels without alpha" = XRGB8888.
  // Matches CSI's BPP=4 receiver setup (required for CR18.PARALLEL24_EN).
  // RGB565 attempt reverted 2026-04-25 — see camera_support.h comment.
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
  if (!sentai_pxp_wait_complete()) {
    SERR_LOG(SERR_CAM_PXP_TIMEOUT, __LINE__);
    return -7;
  }

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

// camera.set_resolution(): OUTPUT size of the JPEG / RGB capture helpers only
// (W24-T20).  g_cam_width/height never describe the frame buffers: the source
// geometry is g_runtime_cam_w/h (sentai_cam_get_width/height below).  Before
// this the YUV pipeline and the zero-copy grey grab used g_cam_width as the
// source pitch, so set_resolution(320, 240) silently corrupted them.
extern "C" int sentai_cam_set_res(int w, int h) {
  if (w <= 0 || h <= 0 || w > (int)g_runtime_cam_w || h > (int)g_runtime_cam_h) return -1;
  g_cam_width = w;
  g_cam_height = h;
  return 0;
}

// W24-T11 (2026-10-31): select the SENSOR geometry before the first camera
// init.  Only the two modes with receiver + D-PHY + OV5640 PLL table rows:
// VGA 640x480 (15/30/45/60/90 fps) and SXGA 1280x960 (15/30 fps, 2x2 binned
// full-sensor readout, XYUV 4.9 MB per frame buffer).  Refused once the
// camera is initialised (the receiver cannot be re-initialised; sys.reset()).
// Returns 0, -1 bad geometry, -2 camera already initialised.
extern "C" int sentai_cam_set_hw(int w, int h) {
  if (!((w == 640 && h == 480) || (w == 1280 && h == 960))) return -1;
  if (g_cam_initialized) return -2;
  g_runtime_cam_w = (uint32_t)w;
  g_runtime_cam_h = (uint32_t)h;
  g_cam_width = w;
  g_cam_height = h;
  g_sentai_pxp_deadline_ms = (w * h > 640 * 480) ? 200u : 50u;
  return 0;
}

// ---------------------------------------------------------------------
// PXP HW path: XRGB8888 (camera native) → Y8 luma (single-plane).
// Mirrors pxp_scale_xrgb_to_rgb but outputs kPXP_OutputPixelFormatY8 —
// the PXP computes the BT.601 luma internally so we do NOT write any
// scalar RGB→Y loop ourselves (see [[arm-hw-primitives-first]]).
// Used by sentai.aruco / sentai.calib (OP-S6-W1, OP-S6-W3) to feed the
// detector a Y8 buffer directly from the camera, no MP copy.
// Returns 0 on success.
// ---------------------------------------------------------------------
extern "C"
__attribute__((section(".sdram_text")))
int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                          uint8_t* dst, int dst_w, int dst_h) {
  pxp_ps_buffer_config_t ps_cfg;
  memset(&ps_cfg, 0, sizeof(ps_cfg));
  ps_cfg.pixelFormat = kPXP_PsPixelFormatRGB888;   /* 32-bit, no alpha */
  ps_cfg.swapByte    = false;
  ps_cfg.bufferAddr  = (uint32_t)src;
  ps_cfg.bufferAddrU = 0;
  ps_cfg.bufferAddrV = 0;
  ps_cfg.pitchBytes  = (src_w + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP;

  pxp_output_buffer_config_t out_cfg;
  memset(&out_cfg, 0, sizeof(out_cfg));
  out_cfg.pixelFormat    = kPXP_OutputPixelFormatY8;   /* 8 bpp luma */
  out_cfg.interlacedMode = kPXP_OutputProgressive;
  out_cfg.buffer0Addr    = (uint32_t)dst;
  out_cfg.buffer1Addr    = 0;
  out_cfg.pitchBytes     = dst_w;                       /* 1 byte / px */
  out_cfg.width          = dst_w;
  out_cfg.height         = dst_h;

  const uint32_t dst_size = (uint32_t)(dst_w * dst_h);

#if (__CORTEX_M == 7)
  DCACHE_CleanInvalidateByRange((uint32_t)dst, dst_size);
#endif

  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, src_w, src_h, dst_w, dst_h);
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, dst_w - 1, dst_h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  /* CSC1 is NOT needed here — the Y8 output path computes luma
   * internally using fixed BT.601 coefficients (per RT1176 RM 41.4). */
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);
  PXP_Start(DEMO_PXP);

  if (!sentai_pxp_wait_complete()) {
    SERR_LOG(SERR_CAM_PXP_TIMEOUT, __LINE__);
    return -7;
  }

#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)dst, dst_size);
#endif
  return 0;
}

// ---------------------------------------------------------------------
extern volatile int g_sentai_pxp_block16;   /* defined with sentai_pxp_pack420_drop below */
// sentai_pxp_xyuv_to_yuv420 (s240/YUV420):  ONE PXP pass that splits the
// camera's native XYUV8888 frame into the two planes the dual-input
// YUV420 model wants:
//   y_dst  = Y  plane, src_w x src_h,     1 byte/px  -> model input[1]
//   uv_dst = UV plane, src_w/2 x src_h/2, 2 bytes/px -> model input[0]
// No CSC: the OV5640 already emitted YUV (the model's training color
// space), so we just repack + 2x2-subsample the chroma.  This replaces
// the 900 KB RGB tensor with a 460 KB YUV420 pair (Y 307 KB + UV 153 KB),
// which is HALF the TPU input transfer AND frees ~460 KB of OCRAM.
// Returns 0 on success.  dst dims are implied: Y = src, UV = src/2.
// ---------------------------------------------------------------------
extern "C"
__attribute__((section(".sdram_text")))
int sentai_pxp_xyuv_to_yuv420(const uint8_t* src, int src_w, int src_h,
                              uint8_t* y_dst, uint8_t* uv_dst) {
  pxp_ps_buffer_config_t ps_cfg;
  memset(&ps_cfg, 0, sizeof(ps_cfg));
  ps_cfg.pixelFormat = kPXP_PsPixelFormatYUV1P444;  /* 32-bit XYUV unpacked */
  ps_cfg.swapByte    = false;
  ps_cfg.bufferAddr  = (uint32_t)src;
  ps_cfg.bufferAddrU = 0;
  ps_cfg.bufferAddrV = 0;
  ps_cfg.pitchBytes  = (src_w + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP;

  pxp_output_buffer_config_t out_cfg;
  memset(&out_cfg, 0, sizeof(out_cfg));
  out_cfg.pixelFormat    = kPXP_OutputPixelFormatYUV2P420;  /* Y + interleaved UV */
  out_cfg.interlacedMode = kPXP_OutputProgressive;
  out_cfg.buffer0Addr    = (uint32_t)y_dst;    /* Y  plane */
  out_cfg.buffer1Addr    = (uint32_t)uv_dst;   /* UV plane (2x2 subsampled) */
  out_cfg.pitchBytes     = src_w;              /* Y pitch; UV derives internally */
  out_cfg.width          = src_w;
  out_cfg.height         = src_h;

  const uint32_t y_size  = (uint32_t)(src_w * src_h);
  const uint32_t uv_size = (uint32_t)((src_w / 2) * (src_h / 2) * 2);

#if (__CORTEX_M == 7)
  DCACHE_CleanInvalidateByRange((uint32_t)y_dst, y_size);
  DCACHE_CleanInvalidateByRange((uint32_t)uv_dst, uv_size);
#endif

  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, src_w, src_h, src_w, src_h);  /* Y 1:1 */
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, src_w - 1, src_h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  /* Input is already YUV — no color-space conversion, just subsample. */
  PXP_SetProcessSurfaceYUVFormat(DEMO_PXP, kPXP_PsYUVFormatYUV);
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);
  /* EXP-s260: 16x16 processing blocks (see sentai_pxp_pack420_drop). */
  if (g_sentai_pxp_block16) PXP_SetProcessBlockSize(DEMO_PXP, kPXP_BlockSize16);
  PXP_Start(DEMO_PXP);

  const bool ok = sentai_pxp_wait_complete();
  if (g_sentai_pxp_block16) PXP_SetProcessBlockSize(DEMO_PXP, kPXP_BlockSize8);
  if (!ok) {
    SERR_LOG(SERR_CAM_PXP_TIMEOUT, __LINE__);
    return -7;
  }

#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)y_dst, y_size);
  DCACHE_InvalidateByRange((uint32_t)uv_dst, uv_size);
#endif
  return 0;
}

// EXP-s260 / W24-T16 option B: the packed YUV420 camera buffer (EXP-s259:
// [C,Y1,Y2,0] per pixel pair, C = U on even rows / V on odd rows, i.e. a
// W/2 x H image of 32-bit pixels) is copied by the PXP as RGB888 (32-bit
// unpacked) -> RGB888P (24-bit packed), which simply drops the zero byte:
// dst = [C,Y1,Y2] per pixel pair = 1.5 bytes per camera pixel, the same byte
// count as the model's Y + UV planes.  No scaler, no CSC.  `w2` is the
// camera width / 2.  Returns 0 on success.
/* EXP-s260: PXP 16x16 processing blocks for the two camera repack passes.
 * Without the TPU running they cut the repack 30-40 % under CSI load (SXGA
 * true-30 byte-drop 32.5 -> 19.6 ms, legacy 62 -> 44 ms) with identical
 * output -- but WITH the Edge TPU streaming over USB the longer PXP bursts
 * starve the EHCI reads instead (build 1746: invoke 70 -> 77 ms, SXGA pipeline
 * 13.8 -> 12.5 fps, VGA 44 -> 42.5).  Default OFF (SDK 8x8); the knob
 * sentai.camera.pxp_block16(1) is for camera-only workloads / tests. */
volatile int g_sentai_pxp_block16 = 0;
extern "C" int sentai_pxp_set_block16(int on) { g_sentai_pxp_block16 = on ? 1 : 0; return g_sentai_pxp_block16; }
extern "C"
__attribute__((section(".sdram_text")))
int sentai_pxp_pack420_drop(const uint8_t* src, int w2, int h, uint8_t* dst) {
  pxp_ps_buffer_config_t ps_cfg;
  memset(&ps_cfg, 0, sizeof(ps_cfg));
  ps_cfg.pixelFormat = kPXP_PsPixelFormatRGB888;   /* 32-bit unpacked: B=C, G=Y1, R=Y2, X=0 */
  ps_cfg.swapByte    = false;
  ps_cfg.bufferAddr  = (uint32_t)src;
  ps_cfg.pitchBytes  = (uint32_t)w2 * 4u;

  pxp_output_buffer_config_t out_cfg;
  memset(&out_cfg, 0, sizeof(out_cfg));
  out_cfg.pixelFormat    = kPXP_OutputPixelFormatRGB888P;   /* 24-bit packed */
  out_cfg.interlacedMode = kPXP_OutputProgressive;
  out_cfg.buffer0Addr    = (uint32_t)dst;
  out_cfg.pitchBytes     = (uint32_t)w2 * 3u;
  out_cfg.width          = (uint16_t)w2;
  out_cfg.height         = (uint16_t)h;

  const uint32_t dst_size = (uint32_t)w2 * (uint32_t)h * 3u;
#if (__CORTEX_M == 7)
  DCACHE_CleanInvalidateByRange((uint32_t)dst, dst_size);
#endif
  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, w2, h, w2, h);
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, w2 - 1, h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);
  /* EXP-s260: 16x16 blocks = 16 source rows per block instead of 8 -> fewer
   * SDRAM page switches per byte while the CSI is writing at the same time.
   * Restored to 8x8 afterwards so the other PXP users keep their behaviour. */
  if (g_sentai_pxp_block16) PXP_SetProcessBlockSize(DEMO_PXP, kPXP_BlockSize16);
  PXP_Start(DEMO_PXP);
  const bool ok = sentai_pxp_wait_complete();
  if (g_sentai_pxp_block16) PXP_SetProcessBlockSize(DEMO_PXP, kPXP_BlockSize8);
  if (!ok) {
    SERR_LOG(SERR_CAM_PXP_TIMEOUT, __LINE__);
    return -7;
  }
#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)dst, dst_size);
#endif
  return 0;
}

// EXP-s261 / W24-T15 step 1: PXP repack that CHASES the CSI writer inside the
// frame instead of waiting for the full frame.  Self-contained test (pipeline
// stopped, caller owns the PXP):
//   1. dequeue one full frame (so we are at the start of the next one) and
//      return it;
//   2. the CSI has two armed buffers (DMASA_FB1/FB2); poison the padding byte
//      (byte 3, always written as 0 by the receiver) of the last pixel of a
//      detector row in both, and see which one the CSI clears -> that is the
//      buffer being written right now;
//   3. poison the same byte at the last row of every slice, and for each slice
//      wait until the CSI clears it, then run the PXP on that slice only
//      (packed420 -> byte-drop, legacy -> XYUV->YUV420) into staging slot 0;
//   4. when the frame is done, dequeue it (must be the chased buffer), run the
//      full-frame PXP into staging slot 1 and memcmp the two outputs.
// out[]: ok, n_slices, detect_us (after the dequeue), eof_us (after detect),
//        done_after_eof_us, cmp_equal, same_buffer, then per slice
//        (ready_us after detect, pxp_us).  Returns the number of words written
//        or a negative code.
extern "C" uint8_t* sentai_pipeline_staging_buf(int slot);
extern "C" int sentai_pipeline_staging_bytes(void);
extern "C" int sentai_pxp_xyuv_to_yuv420(const uint8_t* src, int src_w, int src_h,
                                         uint8_t* y_dst, uint8_t* uv_dst);
extern "C" int sentai_pxp_pack420_drop(const uint8_t* src, int w2, int h, uint8_t* dst);
extern "C" int sentai_detection_is_running(void);
extern "C" __attribute__((section(".sdram_text")))
int sentai_pxp_chase_test(int n_slices, uint32_t* out, int out_max) {
  if (!g_cam_initialized) return -1;
  if (sentai_detection_is_running()) return -2;
  if (n_slices < 1 || n_slices > 16 || !out || out_max < 8 + 2 * n_slices) return -3;
  const int w = (int)g_runtime_cam_w, h = (int)g_runtime_cam_h;
  const int packed = g_sentai_cam_packed420;
  const uint32_t pitch = packed ? (uint32_t)(w / 2) * 4u : (uint32_t)w * 4u;
  const int rows = h / n_slices;                   /* even for all our geometries */
  if (rows * n_slices != h || (rows & 1)) return -4;
  uint8_t* dst0 = sentai_pipeline_staging_buf(0);
  uint8_t* dst1 = sentai_pipeline_staging_buf(1);
  const int total = sentai_pipeline_staging_bytes();
  if (!dst0 || !dst1 || total != w * h * 3 / 2) return -5;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  auto now_us = []() { return (uint32_t)vPortGetRunTimeCounterValue(); };
  /* The marker byte is read by the CPU while the CSI DMA writes the same
   * line: invalidate the 32-byte cache line before every read and clean it
   * after every write, so the poll never sees a stale cached value (if the
   * region is non-cacheable these are no-ops). */
  auto marker = [&](uint8_t* buf, int row) -> volatile uint8_t* {
    return (volatile uint8_t*)(buf + (uint32_t)row * pitch + pitch - 1u);
  };
  auto rd = [&](volatile uint8_t* p) -> uint8_t {
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)p & ~31u), 32);
    return *p;
  };
  auto wr = [&](volatile uint8_t* p, uint8_t v) {
    *p = v;
    SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)p & ~31u), 32);
  };

  /* 1. prime the stream (frames only flow while somebody dequeues them), then
   *    sync to a frame boundary */
  uint8_t* raw = nullptr;
  int idx = -1;
  for (int p = 0; p < 4; p++) {
    idx = cam->GetRawFrame(&raw);
    if (idx < 0 || !raw) return -6;
    cam->ReturnRawFrame(idx);
  }
  const uint32_t t0 = now_us();

  /* 2. which buffer is being written now?  The camera task hands the CSI one
   *    buffer at a time, so the driver programs BOTH DMASA registers with the
   *    same address (SDK: "if there is only one active buffer ... both set");
   *    that address is the in-flight buffer and the CSI keeps re-using it
   *    every frame until a new empty buffer is submitted.  Sync on the CSI
   *    frame counter (CR3.FRMCNT increments at frame done) so the slice
   *    markers are poisoned during the vertical blank, before row 0. */
  /*    With two empties available the two registers hold two different
   *    buffers and the hardware alternates them.  So: sync on the frame
   *    counter, let the ISR re-arm (~150 us), poison the slice markers in BOTH
   *    candidates during the vertical blank, and the candidate whose first
   *    slice marker clears is the in-flight buffer. */
  const uint32_t fc0 = (CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT;
  uint32_t t_sync = 0;
  while (now_us() - t0 < 100000u) {
    if (((CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT) != fc0) { t_sync = now_us(); break; }
    taskYIELD();
  }
  if (!t_sync) return -7;
  while (now_us() - t_sync < 150u) { }
  uint8_t* cand[2] = { (uint8_t*)CSI->DMASA_FB1, (uint8_t*)CSI->DMASA_FB2 };
  const int ncand = (cand[0] == cand[1]) ? 1 : 2;
  for (int c = 0; c < ncand; c++) {
    if (FramebufferPtrToIndex(cand[c]) < 0) {
      int n = 0;
      out[n++] = 0u; out[n++] = 1u; out[n++] = (uint32_t)cand[0]; out[n++] = (uint32_t)cand[1];
      return n;
    }
    for (int k = 0; k < n_slices; k++) wr(marker(cand[c], (k + 1) * rows - 1), 0xFF);
  }
  uint8_t* cur = nullptr;
  uint32_t t_det = 0;
  while (now_us() - t_sync < 120000u) {
    for (int c = 0; c < ncand; c++) {
      if (rd(marker(cand[c], rows - 1)) == 0) { cur = cand[c]; break; }
    }
    if (cur) { t_det = now_us(); break; }
    taskYIELD();
  }
  for (int c = 0; c < ncand; c++) {
    if (cand[c] != cur) for (int k = 0; k < n_slices; k++) wr(marker(cand[c], (k + 1) * rows - 1), 0);
  }
  if (!cur) {
    int n = 0;
    out[n++] = 0u; out[n++] = 3u; out[n++] = (uint32_t)cand[0]; out[n++] = (uint32_t)cand[1];
    out[n++] = (CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT; out[n++] = fc0;
    return n;
  }
  /* t_det is the moment slice 0 was complete; the frame started ~one slice
   * earlier.  Report detect latency relative to the frame-counter sync. */
  t_det = t_sync;

  /* 3. chase the slices (slice 0 is already complete) */
  const uint32_t frmcnt_det = (CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT;
  uint32_t ready_us[16], pxp_us[16];
  int rc = 0;
  for (int k = 0; k < n_slices && rc == 0; k++) {
    volatile uint8_t* m = marker(cur, (k + 1) * rows - 1);
    const uint32_t tw = now_us();
    while (rd(m) != 0) {
      if (now_us() - tw > 120000u) { rc = -8; break; }
      taskYIELD();
    }
    if (rc) {
      /* diagnostics: ok=0, tag 2, detect latency, failing slice, CSI frame
       * counter at detect / now, how many slice markers are still poisoned,
       * which framebuffer index we chased, DMASA regs now */
      int n = 0;
      uint32_t still = 0;
      for (int j = 0; j < n_slices; j++) if (rd(marker(cur, (j + 1) * rows - 1)) == 0xFF) still++;
      for (int j = 0; j < n_slices; j++) wr(marker(cur, (j + 1) * rows - 1), 0);
      out[n++] = 0u; out[n++] = 2u; out[n++] = t_det - t0; out[n++] = (uint32_t)k;
      out[n++] = frmcnt_det; out[n++] = (CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT;
      out[n++] = still; out[n++] = (uint32_t)FramebufferPtrToIndex(cur);
      out[n++] = (uint32_t)CSI->DMASA_FB1; out[n++] = (uint32_t)CSI->DMASA_FB2;
      return n;
    }
    ready_us[k] = now_us() - t_det;
    const uint32_t tp = now_us();
    const uint8_t* src = cur + (uint32_t)k * rows * pitch;
    if (packed) {
      rc = sentai_pxp_pack420_drop(src, w / 2, rows, dst0 + (uint32_t)k * rows * (uint32_t)(w / 2) * 3u);
    } else {
      rc = sentai_pxp_xyuv_to_yuv420(src, w, rows, dst0 + (uint32_t)k * rows * (uint32_t)w,
                                     dst0 + (uint32_t)w * h + (uint32_t)(k * rows / 2) * (uint32_t)w);
    }
    pxp_us[k] = now_us() - tp;
  }
  for (int k = 0; k < n_slices; k++) wr(marker(cur, (k + 1) * rows - 1), 0);   /* restore */
  if (rc) return rc < -8 ? rc : -9 + (rc == -8 ? -1 : 0);
  const uint32_t t_eof = t_det + ready_us[n_slices - 1];
  const uint32_t t_done = now_us();

  /* 4. reference: the frame we chased must now come out of the queue */
  uint8_t* raw2 = nullptr;
  int idx2 = cam->GetRawFrame(&raw2);
  if (idx2 < 0 || !raw2) return -11;
  const int same = (raw2 == cur);
  int rc2 = packed ? sentai_pxp_pack420_drop(raw2, w / 2, h, dst1)
                   : sentai_pxp_xyuv_to_yuv420(raw2, w, h, dst1, dst1 + (uint32_t)w * h);
  cam->ReturnRawFrame(idx2);
  if (rc2) return -12;
  const int equal = (memcmp(dst0, dst1, (size_t)total) == 0);

  int n = 0;
  out[n++] = 1u; out[n++] = (uint32_t)n_slices; out[n++] = t_det - t0; out[n++] = t_eof - t_det;
  out[n++] = t_done - t_eof; out[n++] = (uint32_t)equal; out[n++] = (uint32_t)same; out[n++] = (uint32_t)rows;
  for (int k = 0; k < n_slices; k++) { out[n++] = ready_us[k]; out[n++] = pxp_us[k]; }
  return n;
}

// ===========================================================================
// EXP-s263 / W24-T15 step 2: chase producer for PrepTask.  Same mechanics as
// sentai_pxp_chase_test (CSI frame-counter sync, DMASA_FB1/FB2 candidates,
// padding-byte markers with D-cache maintenance) packaged as begin / wait /
// end so the OCRAM-ring producer can start repacking a frame while the CSI
// is still writing it, instead of after GetRawFrame() returns the complete
// frame.  Packed-YUV420 geometry only (byte 3 of every pixel pair is written
// as 0).  One chase at a time (static state, PrepTask only).
// ===========================================================================
namespace {
struct ChaseState { uint8_t* buf; uint32_t pitch; int rows; int nslices; int h; };
ChaseState s_chase = {nullptr, 0, 0, 0, 0};
inline int chase_last_row(const ChaseState& c, int k) { int r = (k + 1) * c.rows; return (r < c.h ? r : c.h) - 1; }
inline volatile uint8_t* chase_marker(uint8_t* buf, uint32_t pitch, int row) {
  return (volatile uint8_t*)(buf + (uint32_t)row * pitch + pitch - 1u);
}
inline uint8_t chase_rd(volatile uint8_t* p) {
  SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)p & ~31u), 32);
  return *p;
}
inline void chase_wr(volatile uint8_t* p, uint8_t v) {
  *p = v;
  SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)p & ~31u), 32);
}
}  // namespace
extern "C" volatile uint32_t g_sentai_chase_begin_fail = 0, g_sentai_chase_slice_fail = 0, g_sentai_chase_ok = 0;

// Returns 0 and the in-flight buffer; <0 on timeout.  `wait_sync` = wait for the
// next CSI frame boundary first (PrepTask calls it right after returning the
// previous frame, i.e. already inside the vertical blank: wait_sync = 0).
extern "C" __attribute__((section(".sdram_text")))
int sentai_cam_chase_begin(int rows_per_slice, int wait_sync, uint8_t** out_buf, uint32_t* t_sync_us) {
  const int w = (int)g_runtime_cam_w, h = (int)g_runtime_cam_h;
  if (!g_sentai_cam_packed420 || rows_per_slice <= 0) return -1;
  const uint32_t pitch = (uint32_t)(w / 2) * 4u;
  const int n = (h + rows_per_slice - 1) / rows_per_slice;   // last slice may be partial (960 = 7.5 x 128)
  auto last_row = [&](int k) { int r = (k + 1) * rows_per_slice; return (r < h ? r : h) - 1; };
  auto now_us = []() { return (uint32_t)vPortGetRunTimeCounterValue(); };
  uint32_t t0 = now_us();
  if (wait_sync) {
    const uint32_t fc0 = (CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT;
    uint32_t ts = 0;
    while (now_us() - t0 < 100000u) {
      if (((CSI->CR3 & CSI_CR3_FRMCNT_MASK) >> CSI_CR3_FRMCNT_SHIFT) != fc0) { ts = now_us(); break; }
      taskYIELD();
    }
    if (!ts) { g_sentai_chase_begin_fail++; return -2; }
    t0 = ts;
    while (now_us() - t0 < 150u) { }
  }
  uint8_t* cand[2] = { (uint8_t*)CSI->DMASA_FB1, (uint8_t*)CSI->DMASA_FB2 };
  const int ncand = (cand[0] == cand[1]) ? 1 : 2;
  for (int c = 0; c < ncand; c++) {
    if (FramebufferPtrToIndex(cand[c]) < 0) { g_sentai_chase_begin_fail++; return -3; }
    for (int k = 0; k < n; k++) chase_wr(chase_marker(cand[c], pitch, last_row(k)), 0xFF);
  }
  uint8_t* cur = nullptr;
  while (now_us() - t0 < 120000u) {
    for (int c = 0; c < ncand; c++) {
      if (chase_rd(chase_marker(cand[c], pitch, rows_per_slice - 1)) == 0) { cur = cand[c]; break; }
    }
    if (cur) break;
    taskYIELD();
  }
  for (int c = 0; c < ncand; c++) {
    if (cand[c] != cur) for (int k = 0; k < n; k++) chase_wr(chase_marker(cand[c], pitch, last_row(k)), 0);
  }
  if (!cur) { g_sentai_chase_begin_fail++; return -4; }
  s_chase.buf = cur; s_chase.pitch = pitch; s_chase.rows = rows_per_slice; s_chase.nslices = n; s_chase.h = h;
  if (out_buf) *out_buf = cur;
  if (t_sync_us) *t_sync_us = t0;
  return 0;
}
extern "C" __attribute__((section(".sdram_text")))
int sentai_cam_chase_wait_slice(int k, uint32_t timeout_us) {
  if (!s_chase.buf || k < 0 || k >= s_chase.nslices) return -1;
  volatile uint8_t* m = chase_marker(s_chase.buf, s_chase.pitch, chase_last_row(s_chase, k));
  const uint32_t tw = (uint32_t)vPortGetRunTimeCounterValue();
  while (chase_rd(m) != 0) {
    if ((uint32_t)vPortGetRunTimeCounterValue() - tw > timeout_us) { g_sentai_chase_slice_fail++; return -2; }
    taskYIELD();
  }
  return 0;
}
extern "C" void sentai_cam_chase_end(void) {
  if (s_chase.buf) {
    for (int k = 0; k < s_chase.nslices; k++) chase_wr(chase_marker(s_chase.buf, s_chase.pitch, chase_last_row(s_chase, k)), 0);
    g_sentai_chase_ok++;
  }
  s_chase.buf = nullptr;
}
extern "C" void sentai_cam_chase_stats(uint32_t* out3) {
  out3[0] = g_sentai_chase_ok; out3[1] = g_sentai_chase_begin_fail; out3[2] = g_sentai_chase_slice_fail;
}

// EXP-s260 option B capture mode: sensor "YUV420 for MIPI only" + CSI receiver
// at half width (2 B/px, see EXP-s259).  Set before camera.init(); the init
// path writes the sensor registers to the ACTIVE camera after the OV5640
// driver init.  PrepTask then uses sentai_pxp_pack420_drop() instead of the
// XYUV -> YUV420 repack.  Everything else still assumes XYUV: diagnostics only.
volatile int g_sentai_cam_packed420 = 0;
extern "C" int sentai_camera_set_csi_mode(uint32_t bus, uint32_t cr18_or, uint32_t cr20_or, uint32_t wdiv);
extern "C" int sentai_cam_set_packed420(int on) {
  if (g_cam_initialized) return -2;
  g_sentai_cam_packed420 = on ? 1 : 0;
  return sentai_camera_set_csi_mode(24u, 0u, 0u, on ? 2u : 1u);
}
extern "C" int sentai_cam_get_packed420(void) { return g_sentai_cam_packed420; }

// s240/YUV420 Phase-3 diagnostic: grab the current (XYUV, yuv_mode=1) camera
// frame, split it into Y + UV planes via the PXP, and dump both raw to FS so
// the host can reconstruct grayscale/chroma images and confirm the extraction
// is clean.  Test buffers live in SDRAM (OCRAM is full); the production
// placement is Phase 4.  Returns 0 ok, <0 on grab/PXP/write failure.
extern "C" int sentai_cam_grab_latest(uint8_t** raw);
extern "C" int sentai_fs_write(const char* path, const uint8_t* data, int size);
extern "C" int sentai_pxp_xyuv_to_yuv420(const uint8_t* src, int src_w, int src_h,
                                         uint8_t* y_dst, uint8_t* uv_dst);
extern "C" int sentai_yuv420_extract_test(const char* y_path, const char* uv_path) {
  static uint8_t s_y[640 * 480]      __attribute__((section(".sdram_bss"), aligned(32)));
  static uint8_t s_uv[320 * 240 * 2] __attribute__((section(".sdram_bss"), aligned(32)));
  // The plane buffers are VGA-sized and the PXP pass needs XYUV rows: refuse
  // any other source geometry instead of dumping interleaved half rows (W24-T20).
  if (cam_src_w() != 640 || cam_src_h() != 480) return -5;
  if (cam_src_packed()) return -7;
  uint8_t* raw = 0;
  int idx = sentai_cam_grab_latest(&raw);
  if (idx < 0 || !raw) return -1;
  int rc = sentai_pxp_xyuv_to_yuv420(raw, 640, 480, s_y, s_uv);
  sentai_cam_return_raw(idx);
  if (rc) return -2;
  if (y_path  && !sentai_fs_write(y_path,  s_y,  (int)sizeof(s_y)))  return -3;
  if (uv_path && !sentai_fs_write(uv_path, s_uv, (int)sizeof(s_uv))) return -4;
  return 0;
}

/* Build #980 — accept fps as the second arg to sentai.camera.init().
 * Backward-compatible: callers passing only `streaming` get the
 * previous behaviour (g_runtime_fps stays at its boot default = 30).
 * BOARD_InitCamera reads g_runtime_fps inside HandleEnableRequest,
 * so as long as we set it BEFORE cam->Enable() the OV5640 driver
 * picks up the new clock-config table entry on first init.
 *
 * Limitation (per memory/project_runtime_fps_switch_blocked.md):
 * a SECOND call with a different fps will wedge — CAMERA_RECEIVER_Init
 * is not re-entrant in this NXP HAL.  For a clean fps switch the
 * caller must reboot (sentai.sys.reset()) and call init() again
 * with the new fps.  This is still a strict win over rebuild +
 * reflash. */
/* Map (hflip, vflip) -> SetCameraRotation degrees code.
 *   (false,false) = no mirror, no flip   -> degrees=0
 *   (true,false)  = mirror only          -> degrees=90
 *   (true,true)   = mirror + flip        -> degrees=180
 *   (false,true)  = flip only            -> degrees=270
 * The OV5640 NXP init driver leaves H-mirror ON by default (so text
 * reads correctly when displayed); callers that want raw sensor
 * orientation must explicitly request hflip=0/vflip=0 here. */
static int sentai_flip_to_degrees(int hflip, int vflip) {
  if (hflip && !vflip) return 90;
  if (hflip && vflip)  return 180;
  if (!hflip && vflip) return 270;
  return 0;
}

/* Full init entry point.  hflip/vflip override the OV5640 mirror/flip
 * registers (0x3820/0x3821 bits 1+2) post-SwitchCamera so both cameras
 * land in the same configured orientation.  -1 (default for both)
 * means "leave whatever the OV5640 init driver programmed", which is
 * H-mirror ON, V-flip OFF.  Pass 0/0 explicitly to clear both. */
// =====================================================================
// Dual-camera genlock (EXP-s251, OP-S10-W23-T9) -- C implementation.
//
// Facts (s251): both OV5640s share one 24 MHz XCLK, so their frame phase is
// fixed for the session; it is set by the order in which the sensors are
// started (one SCCB write apart ~ +411 us with cam0 first).  A MUX flip at
// FB-done of the old sensor is clean only if the new sensor's frame start
// lies inside the old one's vertical blanking (VTS - 2*480 lines ~ 24 lines
// ~ 543 us at VGA45); margins are VBLANK -/+ phi, so the optimum is phi = 0.
// Writing TIMING_VTS + d to one sensor for a single frame delays it by
// exactly d lines.  With phi within +/- 60 us, ratio(1,1) runs at the full
// sensor rate with drain=0 / dirty=0 and 0 mislabelled frames (iter6).
//
// Phase measurement uses the FB-done log written by the CSI frame-complete
// hook: at ratio(1,1) the interval of the first frame after a flip TO cam1
// is T + phi and after a flip TO cam0 it is T - phi (reduced mod T when the
// CSI had to wait a frame).  The hook only runs when the driver queues a
// frame, so the measurement loop DEQUEUES while it measures.
//
// Everything here is task context (REPL / init), bounded (<= 3 corrections,
// <= 4 s per measurement), no heap, no ISR changes.
// =====================================================================
extern "C" int sentai_cam_frame_log(int n, uint32_t* ts, uint8_t* idx,
                                    uint8_t* cam, uint16_t* seq);
extern "C" int sentai_cam_switch(int id);
extern "C" int sentai_cam_ratio_set(uint32_t a, uint32_t b);
extern "C" int sentai_detection_is_running(void);
extern volatile uint32_t g_cam_dirty_consecutive_n;   // camera_support.c
extern volatile uint32_t g_cam_buf_ts_us[];            // camera_support.c

static volatile int32_t  g_cam_phase_us        = 0;   // last measured phi (cam1 - cam0), us
static volatile uint32_t g_cam_phase_valid     = 0;
static volatile uint32_t g_cam_genlock_ok      = 0;
static volatile uint32_t g_cam_genlock_steps   = 0;
static volatile uint32_t g_cam_stale_skips     = 0;   // proposal C: old queued frames dropped
static volatile uint32_t g_cam_phase_drift_cnt = 0;   // proposal D: excursions reported
static volatile uint32_t g_cam_phase_drifting  = 0;
static volatile uint32_t g_cam_genlock_busy    = 0;   // monitor must not judge the genlock's own alternation
// Run the genlock automatically at camera.init()?  Default OFF (2026-10-31):
// the measurement alternates the MUX at the as-found phase (~+400 us, cam0
// margin ~130 us) and 2 of 10 cold boots ended with the CSI input wedged
// (audit F16: FRMCNT frozen, CSI2RX SoT/ECC errors, only a SoC reset
// recovers).  Until the flip-free phase measurement (STROBE pads, W23-T10)
// exists, the operator enables it per mission: sentai.camera.genlock_auto(1).
// 0 = nothing at init; 1 = open-loop PRE-TRIM only (no flips);
// 2 (default) = pre-trim + measured genlock (flips at an already-aligned phase).
static volatile uint32_t g_cam_genlock_auto    = 2;   // operator decision 2026-10-31: best mode by default
// On-the-fly closed loop (operator goal 2026-10-31: "self-calibrate while
// running, no initial calibration").  The watchdog phase monitor derives
// phi from the production flips; two consecutive excursions wake the
// cam_tune task, which applies one blind frame-synchronised VTS trim to the
// leading sensor, rate-limited.  While drifting, the dirty-skip protection
// is re-armed so mixed frames cannot reach a consumer.
static volatile uint32_t g_cam_autotune_enabled = 1;
static volatile uint32_t g_cam_autotune_trims   = 0;
static volatile uint32_t g_cam_pretrim_lines    = 0;   // lines applied by the init pre-trim (0 = none)
static volatile uint32_t g_cam_fullrate         = 0;   // drain=0/dirty=0 regime armed by pretrim/genlock
static volatile int32_t  g_cam_autotune_req_phi = 0;
static volatile uint32_t g_cam_phase_T_us       = 22288;  // last measured sensor period
static volatile uint32_t g_cam_phase_excursions = 0;
static TaskHandle_t      s_cam_autotune_task    = nullptr;
static constexpr uint32_t kAutotuneMinGapMs     = 10000;
// As-found phase after camera.init(): cam0 is started one SCCB write before
// cam1 (EXP-s251 iter7/8/10/11/14: +391..+438 us on every boot).
static constexpr uint32_t kSccbStartGapUs       = 410;
static constexpr int32_t  kGenlockTolUs       = 60;
static constexpr uint32_t kGenlockMaxSteps    = 3;
static constexpr uint32_t kGenlockMaxLines    = 200;  // one correction < 1/4 frame
// Drift thresholds are derived at runtime from the sensor geometry:
// VBLANK_us = (VTS - 960 active rows) * T / VTS, warn at VBLANK/2, clear at
// VBLANK/4 (543 us -> 270/135 at VGA45; 814 -> 407/203 at VGA30; 407 ->
// 203/101 at VGA60).  T = measured period when available, else 1e6/fps.
static volatile uint32_t g_cam_vts_lines = 984;   // last TIMING_VTS read from a sensor
static constexpr uint32_t kActiveRows      = 960; // 2x480 binned rows at VGA
static constexpr uint32_t kDefaultBlankRows = 24;
// Nominal sensor period from the fps key (SXGA key 22 = 22.6 fps real, 2.7 % off:
// fine for a plausibility window).  The measured period is only trusted within
// +-25 % of it (W24-T19 follow-up, 2026-11-02): the old check compared a fresh
// measurement with the PREVIOUS estimate (boot default 22288 us = VGA45) and
// halved anything above 1.5x of it, so the first SXGA30 measurement (33.4 ms,
// 2 us above 1.5 x 22288) folded to 16.7 ms and stayed there, every VTS trim was
// computed for half the real period, genlock reported "not converged" (0x0A0D
// val 40) and the drift monitor fell back to pair-granularity alternation
// (ratio_unit 2 -> tags 1001, EXP-s266 iter7).  VGA30 passed by 2 us of margin.
static uint32_t cam_nominal_period_us(void) {
  return g_runtime_fps ? 1000000u / g_runtime_fps : 22222u;
}
static uint32_t cam_period_us(void) {
  const uint32_t Tn = cam_nominal_period_us();
  uint32_t T = g_cam_phase_T_us;
  if (T < Tn - Tn / 4u || T > Tn + Tn / 4u) T = Tn;
  return T;
}
extern "C" uint32_t sentai_cam_vblank_us(void) {
  uint32_t vts = g_cam_vts_lines ? g_cam_vts_lines : 984u;
  uint32_t blank = (vts > kActiveRows) ? (vts - kActiveRows) : kDefaultBlankRows;
  return (blank * cam_period_us()) / vts;
}

static int32_t cam_median_i32(int32_t* v, int n) {
  // insertion sort, n <= 64
  for (int i = 1; i < n; ++i) {
    int32_t x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; --j; }
    v[j + 1] = x;
  }
  return v[n / 2];
}

// phi from the last 64 FB-done events.  0 = ok, -1 = not enough data,
// -2 = no flips in the window, -3 = inconsistent (cam1/cam0 estimates differ).
extern "C" int sentai_cam_phase_from_log(int32_t* phi_us, uint32_t* T_us) {
  uint32_t ts[64]; uint8_t idx[64]; uint8_t cam[64]; uint16_t seq[64];
  // NOTE: also records T for the autotune task (single 32-bit store).
  int n = sentai_cam_frame_log(64, ts, idx, cam, seq);
  if (n < 16) return -1;
  int32_t steady[64], to1[64], to0[64];
  int ns = 0, n1 = 0, n0 = 0;
  for (int k = 1; k < n; ++k) {
    uint32_t dt = ts[k] - ts[k - 1];           // unsigned wrap-safe
    if (dt > 200000u) continue;                // gap: log not continuous
    if (cam[k] != cam[k - 1]) {
      if (cam[k] == 1) { if (n1 < 64) to1[n1++] = (int32_t)dt; }
      else             { if (n0 < 64) to0[n0++] = (int32_t)dt; }
    } else if (ns < 64) {
      steady[ns++] = (int32_t)dt;
    }
  }
  if (n1 < 2 && n0 < 2) return -2;
  // Period estimate.  Per-frame alternation (ratio_unit 1): every interval is
  // a flip interval, but two consecutive ones sum to (T + phi) + (T - phi) =
  // 2T exactly, so flip pairs give T regardless of phi.  Pair-granularity
  // alternation (ratio_unit 2) has no 0-1-0 triples, so fall back to the
  // same-camera intervals (= T there).  The pairs estimate is tried FIRST:
  // in the per-frame regime a consumer that blocks for > T (JPEG encode,
  // EXP-s256) leaves same-camera entries 2T apart in the log, and judging
  // the regime by their count made T = 2T_real and phi = T_real - 4 us, a
  // spurious PHASE_DRIFT (build 1693 run log).
  int32_t T = 0;
  {
    int32_t pairs[64]; int np = 0;
    for (int k = 2; k < n && np < 64; ++k) {
      uint32_t d1 = ts[k - 1] - ts[k - 2], d2 = ts[k] - ts[k - 1];
      if (d1 > 200000u || d2 > 200000u) continue;
      if (cam[k] == cam[k - 1] || cam[k - 1] == cam[k - 2]) continue;
      pairs[np++] = (int32_t)((d1 + d2) / 2u);
    }
    if (np >= 4)       T = cam_median_i32(pairs, np);
    else if (ns >= 4)  T = cam_median_i32(steady, ns);
    else return -2;
  }
  // Sanity against the NOMINAL period of the current mode (never against the
  // previous estimate, see cam_period_us): a log that mixes regimes can fold to
  // a multiple of it.
  {
    uint32_t Tn = cam_nominal_period_us();
    if (Tn > 1000u) {
      while (T > (int32_t)(Tn + Tn / 2u)) T /= 2;
      if (T < (int32_t)(Tn - Tn / 4u) || T > (int32_t)(Tn + Tn / 4u)) return -4;
    }
  }
  if (T <= 0) return -1;
  int32_t phi = 0, phi0 = 0; bool have1 = false, have0 = false;
  if (n1 >= 2) {
    int32_t e = ((cam_median_i32(to1, n1) - T) % T + T) % T;   // 0..T-1
    phi = (e <= T / 2) ? e : e - T; have1 = true;
  }
  if (n0 >= 2) {
    int32_t e = ((cam_median_i32(to0, n0) - T) % T + T) % T;
    phi0 = -((e <= T / 2) ? e : e - T); have0 = true;
  }
  if (have1 && have0 && (phi - phi0 > 300 || phi0 - phi > 300)) return -3;
  *phi_us = have1 ? phi : phi0;
  *T_us = (uint32_t)T;
  g_cam_phase_T_us = (uint32_t)T;
  return 0;
}

extern "C" void sentai_cam_phase_monitor_note(int32_t phi_us) {
  if (g_cam_genlock_busy) return;
  g_cam_phase_us = phi_us;
  g_cam_phase_valid = 1;
  int32_t a = phi_us < 0 ? -phi_us : phi_us;
  const int32_t warn_us  = (int32_t)(sentai_cam_vblank_us() / 2u);
  const int32_t clear_us = (int32_t)(sentai_cam_vblank_us() / 4u);
  if (a > warn_us) {
    g_cam_phase_excursions++;
    if (!g_cam_phase_drifting) {
      g_cam_phase_drifting = 1;
      g_cam_phase_drift_cnt++;
      // Protect consumers until the phase is back: re-arm the dirty skip.
      if (g_cam_fullrate) { g_cam_ratio_unit = 2; g_cam_dirty_consecutive_n = 1; }
      SERR_LOG(SERR_CAM_PHASE_DRIFT, (uint32_t)phi_us);
    }
    // Two consecutive excursions -> ask cam_tune for one blind trim.
    if (g_cam_phase_excursions >= 2u && g_cam_autotune_enabled && s_cam_autotune_task) {
      g_cam_autotune_req_phi = phi_us;
      xTaskNotifyGive(s_cam_autotune_task);
    }
  } else {
    g_cam_phase_excursions = 0;
    if (a < clear_us && g_cam_phase_drifting) {
      g_cam_phase_drifting = 0;
      if (g_cam_fullrate) { g_cam_dirty_consecutive_n = 0; g_cam_ratio_unit = 1; }
      SERR_LOG(SERR_CAM_PHASE_DRIFT_END, (uint32_t)phi_us);
    }
  }
}

// Dequeue (and immediately return) frames until `nframes` distinct sensor
// frames have passed or `budget_ms` elapsed.  Keeps the CSI queue from
// filling so the FB-done hook keeps logging.
static void cam_consume_frames(int nframes, uint32_t budget_ms) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  TickType_t t0 = xTaskGetTickCount();
  uint32_t last = g_camera_sensor_frames;
  int seen = 0;
  while (seen < nframes &&
         (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(budget_ms)) {
    uint32_t q = g_camera_sensor_frames;
    if (q == last) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    last = q; ++seen;
    uint8_t* tmp = nullptr;
    int idx = cam->TryGetRawFrame(&tmp);
    if (idx >= 0) cam->ReturnRawFrame(idx);
  }
}

static void cam_drain_queue(void) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT; ++i) {
    uint8_t* tmp = nullptr;
    int idx = cam->TryGetRawFrame(&tmp);
    if (idx < 0) break;
    cam->ReturnRawFrame(idx);
  }
}

// Measure phi by alternating 1:1 for ~70 frames while consuming.  Leaves
// ratio(0,0) on cam0.  Returns 0 or a negative code.
// Returns 0, -1 camera off, -10 detection pipeline owns the frames, -30 not
// measurable (fewer than 32 fresh FB-done entries, or no usable flips).
// Restores the caller's ratio and MUX camera; drain/dirty are restored too.
extern "C" int sentai_cam_phase_measure(int32_t* phi_us, uint32_t* T_us) {
  if (!g_cam_initialized) return -1;
  if (sentai_detection_is_running()) return -10;
  uint32_t drain_prev = g_cam_switch_drain_threshold;
  uint32_t dirty_prev = g_cam_dirty_consecutive_n;
  uint32_t ratio_prev = g_cam_ratio_packed;
  uint32_t unit_prev = g_cam_ratio_unit;
  int cam_prev = g_cam_current_id;
  g_cam_ratio_unit = 2;                      // measure on pair granularity (steady intervals)
  sentai_cam_ratio_set(0, 0);
  (void)sentai_cam_switch(0);
  vTaskDelay(pdMS_TO_TICKS(150));
  g_cam_switch_drain_threshold = 0;
  g_cam_dirty_consecutive_n = 0;
  cam_drain_queue();
  uint32_t head0 = g_cam_flog_head;
  sentai_cam_ratio_set(1, 1);
  vTaskDelay(pdMS_TO_TICKS(250));
  // 48 frames = 12 flips: enough for >= 32 fresh log entries (4 steady
  // intervals + >= 2 flips per direction) while keeping the exposure to the
  // F16 flip hazard as short as possible.
  cam_consume_frames(48, 3000);
  int rc = (g_cam_flog_head - head0 >= 32u) ? sentai_cam_phase_from_log(phi_us, T_us) : -1;
  sentai_cam_ratio_set(0, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  cam_drain_queue();
  g_cam_switch_drain_threshold = drain_prev;
  g_cam_dirty_consecutive_n = dirty_prev;
  if (cam_prev == 1) (void)sentai_cam_switch(1);
  g_cam_ratio_unit = unit_prev;
  g_cam_ratio_packed = ratio_prev;
  if (rc == 0) { g_cam_phase_us = *phi_us; g_cam_phase_valid = 1; }
  return rc == 0 ? 0 : -30;
}

static bool cam_vts_read(int cam_id, uint16_t* vts) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  uint8_t h = 0, l = 0;
  if (!cam->ReadFromCam(cam_id, 0x380E, &h) || !cam->ReadFromCam(cam_id, 0x380F, &l)) return false;
  *vts = (uint16_t)((h << 8) | l);
  if (*vts > kActiveRows) g_cam_vts_lines = *vts;   // feeds the fps-independent VBLANK estimate
  return true;
}
static bool cam_vts_write(int cam_id, uint16_t vts) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  return cam->WriteToCam(cam_id, 0x380E, (uint8_t)(vts >> 8)) &&
         cam->WriteToCam(cam_id, 0x380F, (uint8_t)(vts & 0xFF));
}

// Wait for the next FB-done edge of the sensor currently on the MUX.
// Returns false if no frame arrived within budget_ms.
static bool cam_wait_frame_edge(uint32_t budget_ms) {
  TickType_t t0 = xTaskGetTickCount();
  uint32_t q0 = g_camera_sensor_frames;
  while (g_camera_sensor_frames == q0) {
    if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(budget_ms)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return true;
}

// Delay sensor `cam_id` by exactly `lines` lines (one frame of VTS+lines).
// OV5640 timing registers latch at the frame start, so an unsynchronised
// write/restore pair holds the long VTS for one OR two frames depending on
// where in the frame the write landed (iter8: every 19-line tweak moved the
// phase by 2 x 19 lines and the loop oscillated).  Here the sensor is put
// on the MUX so g_camera_sensor_frames follows ITS frame edges; the write
// goes out 5 ms after an edge (latches at the next edge, ~0.75 T later;
// the long frame then spans [0.75 T, 1.75 T + d]) and the restore 1.25 T
// after the write, i.e. mid long frame with +/- 0.5 T of margin.
// Returns 0, -20 no frame edge, -50 SCCB write failed (sensor unchanged),
// -51 the RESTORE failed after 3 attempts: the sensor is left at VTS+lines
// (slower frame) -- reported, the caller must treat the genlock as failed.
// Always leaves the MUX on cam0.
static int cam_vts_tweak_one_frame(int cam_id, uint16_t vts, uint32_t lines, uint32_t T_us) {
  int rc = 0;
  if (sentai_cam_switch(cam_id) < 0) return -50;
  vTaskDelay(pdMS_TO_TICKS(60));
  cam_drain_queue();
  if (!cam_wait_frame_edge(200)) { rc = -20; goto out; }
  vTaskDelay(pdMS_TO_TICKS(T_us / 4000u));                     // 0.25 T after the edge
  {
    TickType_t t_write = xTaskGetTickCount();
    if (!cam_vts_write(cam_id, (uint16_t)(vts + lines))) { rc = -50; goto out; }
    // Restore 1.25 T after the write, anchored to the tick taken before the
    // write so the SCCB/printf cost inside WriteToCam does not eat the margin.
    vTaskDelayUntil(&t_write, pdMS_TO_TICKS((T_us * 5u) / 4000u));
    bool restored = false;
    for (int attempt = 0; attempt < 3 && !restored; ++attempt) {
      restored = cam_vts_write(cam_id, vts);
      if (!restored) vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!restored) {
      uint16_t rb = 0;
      (void)cam_vts_read(cam_id, &rb);
      SERR_LOG(SERR_CAM_GENLOCK_FAIL, 51u);
      printf("[cam] genlock: VTS restore FAILED on cam%d (readback 0x%04X, wanted 0x%04X)\r\n",
             cam_id, (unsigned)rb, (unsigned)vts);
      rc = -51;
      goto out;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(60));
out:
  if (cam_id != 0) (void)sentai_cam_switch(0);
  return rc;
}

// Genlock.  sync_start != 0: first put both sensors in software standby and
// release them back-to-back (phi ~ one SCCB write apart, cam0 first), then
// trim; 0: trim only.  Returns the residual phase in us on success (|phi| <=
// kGenlockTolUs), or a negative code: -1 camera off, -10 detection pipeline
// running (owns the frames), -20 sensors did not restart / no frame edge,
// -30 phase not measurable, -40 did not converge, -50 SCCB write failed
// (sensor unchanged), -51 VTS restore failed (sensor left slower; reported).
// Duration: typically 2-3 s (one measurement + one trim); worst case ~20 s
// (4 measurements of up to 4.5 s + 3 trims).  SERR_CAM_GENLOCK_FAIL val =
// |code|.  Premise: AEC frame-length extension (0x3A00 bit 2, "night
// mode") stays off in the init table, otherwise low light would move the
// phase and trip the drift monitor.
// Wait (1 ms steps) until GPT1 reaches `t_us`; false if it is further than
// `max_us` away or already more than one period in the past.
static bool cam_wait_until_us(uint32_t t_us, uint32_t max_us) {
  for (;;) {
    int32_t d = (int32_t)(t_us - GPT1->CNT);
    if (d <= 0) return (uint32_t)(-d) < max_us;   // not hopelessly late
    if ((uint32_t)d > max_us) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// BLIND one-frame VTS trim of `cam_id`: the sensor is NOT put on the MUX;
// its frame edges are predicted from its last FB-done entry in the frame
// log (ts + k*T, T known to ~1 us).  Same timing as cam_vts_tweak_one_frame
// (write at edge + 0.25 T, restore 1.25 T after the write).  Returns 0, -20
// no recent edge of that sensor in the log, -50/-51 as the sync tweak.
static int cam_vts_trim_blind(int cam_id, uint32_t lines, uint32_t T_us) {
  uint32_t ts[64]; uint8_t idx[64]; uint8_t cam[64]; uint16_t seq[64];
  int n = sentai_cam_frame_log(64, ts, idx, cam, seq);
  int last = -1;
  for (int k = n - 1; k >= 0; --k) if (cam[k] == cam_id) { last = k; break; }
  if (last < 0 || T_us < 5000u) return -20;
  uint32_t now = GPT1->CNT;
  uint32_t age = now - ts[last];
  if (age > 60u * T_us) return -20;                       // > ~1.3 s old: not trusted
  uint32_t k = age / T_us + 1u;                            // next predicted edge after now
  uint32_t edge = ts[last] + k * T_us;
  if ((int32_t)(edge - now) < 2000) edge += T_us;         // leave >= 2 ms to get there
  uint16_t vts = 0;
  if (!cam_vts_read(cam_id, &vts) || vts == 0) return -50;
  if (!cam_wait_until_us(edge + T_us / 4u, 3u * T_us)) return -20;
  uint32_t t_write = GPT1->CNT;
  if (!cam_vts_write(cam_id, (uint16_t)(vts + lines))) return -50;
  (void)cam_wait_until_us(t_write + (T_us * 5u) / 4u, 3u * T_us);
  bool restored = false;
  for (int attempt = 0; attempt < 3 && !restored; ++attempt) {
    restored = cam_vts_write(cam_id, vts);
    if (!restored) vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (!restored) { SERR_LOG(SERR_CAM_GENLOCK_FAIL, 51u); return -51; }
  return 0;
}

// cam_tune task: sleeps until the phase monitor notifies, then applies one
// blind trim to the leading sensor.  Rate-limited; never touches the MUX or
// the frame queue, so it is safe while the detection pipeline runs.
static void CamAutotuneTask(void*) {
  TickType_t last = 0;
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!g_cam_initialized || !g_cam_autotune_enabled) continue;
    TickType_t now = xTaskGetTickCount();
    if (last != 0 && (now - last) < pdMS_TO_TICKS(kAutotuneMinGapMs)) continue;
    int32_t phi = g_cam_autotune_req_phi;
    uint32_t T = g_cam_phase_T_us;
    int32_t a = phi < 0 ? -phi : phi;
    if (a <= kGenlockTolUs || T < 5000u) continue;
    int cam_id = (phi > 0) ? 0 : 1;                        // delaying cam0 lowers phi
    uint16_t vts = 0;
    if (!cam_vts_read(cam_id, &vts) || vts == 0) continue;
    uint32_t line_ns = (T * 1000u) / vts;
    uint32_t lines = ((uint32_t)a * 1000u + line_ns / 2u) / line_ns;
    if (lines == 0) lines = 1;
    if (lines > kGenlockMaxLines) lines = kGenlockMaxLines;
    int rc = cam_vts_trim_blind(cam_id, lines, T);
    last = xTaskGetTickCount();
    g_cam_phase_excursions = 0;
    if (rc == 0) {
      g_cam_autotune_trims++;
      SERR_LOG(SERR_CAM_PHASE_TRIM, ((uint32_t)cam_id << 16) | lines);
    } else {
      SERR_LOG(SERR_CAM_GENLOCK_FAIL, (uint32_t)(-rc));
    }
  }
}

static void cam_autotune_task_start(void) {
  if (s_cam_autotune_task) return;
  (void)xTaskCreate(CamAutotuneTask, "cam_tune", 1024, nullptr,
                    tskIDLE_PRIORITY + 2, &s_cam_autotune_task);
}

// Open-loop pre-trim at camera.init(): remove the deterministic start-order
// offset (cam0 leads by one SCCB write, kSccbStartGapUs) with a single
// frame-synchronised VTS tweak on cam0 -- the sensor already on the MUX, so
// NO MUX flip happens before the sensors are aligned (audit F16).  Returns
// the lines applied (> 0) or a negative tweak code.
extern "C" int sentai_cam_pretrim(void) {
  if (!g_cam_initialized || g_runtime_fps == 0) return -1;
  uint16_t vts0 = 0;
  if (!cam_vts_read(0, &vts0) || vts0 == 0) return -50;
  uint32_t T_us = 1000000u / g_runtime_fps;                // nominal period
  uint32_t line_ns = (T_us * 1000u) / vts0;
  uint32_t lines = (kSccbStartGapUs * 1000u + line_ns / 2u) / line_ns;
  if (lines == 0 || lines > kGenlockMaxLines) return -30;
  g_cam_genlock_busy = 1;
  int rc = cam_vts_tweak_one_frame(0, vts0, lines, T_us);
  g_cam_genlock_busy = 0;
  if (rc != 0) { SERR_LOG(SERR_CAM_GENLOCK_FAIL, (uint32_t)(-rc)); return rc; }
  g_cam_pretrim_lines = lines;
  g_cam_fullrate = 1;
  g_cam_switch_drain_threshold = 0;
  g_cam_dirty_consecutive_n = 0;
  g_cam_ratio_unit = 1;                      // T14: true frame interleave
  SERR_LOG(SERR_CAM_PRETRIM, lines);
  printf("[cam] pre-trim: cam0 delayed %lu lines (%lu us) -> alternation at full rate\r\n",
         (unsigned long)lines, (unsigned long)((lines * line_ns) / 1000u));
  return (int)lines;
}

static int sentai_cam_genlock_run(coralmicro::CameraTask* cam, int sync_start);
extern "C" int sentai_cam_genlock(int sync_start) {
  if (!g_cam_initialized) return -1;
  if (sentai_detection_is_running()) return -10;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  g_cam_genlock_ok = 0;
  g_cam_genlock_steps = 0;
  g_cam_genlock_busy = 1;
  int rc = sentai_cam_genlock_run(cam, sync_start);
  g_cam_genlock_busy = 0;
  return rc;
}

static int sentai_cam_genlock_run(coralmicro::CameraTask* cam, int sync_start) {

  if (sync_start) {
    // F15 hazard: never leave the CSI queue full across the restart.
    sentai_cam_ratio_set(0, 0);
    (void)sentai_cam_switch(0);
    vTaskDelay(pdMS_TO_TICKS(100));
    cam_drain_queue();
    uint32_t q0 = g_camera_sensor_frames;
    bool sb0 = cam->WriteToCam(0, 0x3008, 0x42);
    bool sb1 = sb0 && cam->WriteToCam(1, 0x3008, 0x42);
    bool ok0 = false, ok1 = false;
    if (sb1) {
      vTaskDelay(pdMS_TO_TICKS(60));
      cam_drain_queue();
      // back-to-back release: cam0 then cam1 -> phi ~ +(one write time)
      ok0 = cam->WriteToCam(0, 0x3008, 0x02);
      ok1 = cam->WriteToCam(1, 0x3008, 0x02);
    }
    if (!sb1 || !ok0 || !ok1) {
      // Never leave a sensor in standby: best-effort release of both.
      (void)cam->WriteToCam(0, 0x3008, 0x02);
      (void)cam->WriteToCam(1, 0x3008, 0x02);
      SERR_LOG(SERR_CAM_GENLOCK_FAIL, 50u);
      return -50;
    }
    vTaskDelay(pdMS_TO_TICKS(400));
    if (g_camera_sensor_frames - q0 < 5u) {
      SERR_LOG(SERR_CAM_GENLOCK_FAIL, 20u);
      return -20;
    }
    cam_drain_queue();
  }

  uint16_t vts0 = 0, vts1 = 0;
  if (!cam_vts_read(0, &vts0) || !cam_vts_read(1, &vts1) || vts0 == 0 || vts1 == 0) return -50;

  int32_t phi = 0; uint32_t T = 0;
  for (uint32_t step = 0; step <= kGenlockMaxSteps; ++step) {
    int rc = sentai_cam_phase_measure(&phi, &T);
    if (rc != 0) {
      SERR_LOG(SERR_CAM_GENLOCK_FAIL, (uint32_t)(-rc));
      return rc;
    }
    int32_t a = phi < 0 ? -phi : phi;
    if (a <= kGenlockTolUs) {
      g_cam_genlock_ok = 1;
      g_cam_genlock_steps = step;
      // Proposal B: with the sensors in phase the per-flip skip is no longer
      // needed -- alternation runs at the full sensor rate.
      g_cam_fullrate = 1;
      g_cam_switch_drain_threshold = 0;
      g_cam_dirty_consecutive_n = 0;
      g_cam_ratio_unit = 1;                  // T14: true frame interleave
      SERR_LOG(SERR_CAM_GENLOCK, (step << 16) | ((uint32_t)phi & 0xFFFFu));
      printf("[cam] genlock: phase %ld us after %lu step(s) (T=%lu us)\r\n",
             (long)phi, (unsigned long)step, (unsigned long)T);
      return (int)phi;
    }
    if (step == kGenlockMaxSteps) break;
    // one-frame VTS tweak: delaying cam0 lowers phi, delaying cam1 raises it
    uint32_t line_ns = (T * 1000u) / (uint32_t)vts0;
    if (line_ns == 0) return -30;
    uint32_t lines = ((uint32_t)a * 1000u + line_ns / 2u) / line_ns;
    if (lines == 0) lines = 1;
    if (lines > kGenlockMaxLines) lines = kGenlockMaxLines;
    int cam_id = (phi > 0) ? 0 : 1;
    uint16_t vts = (cam_id == 0) ? vts0 : vts1;
    printf("[cam] genlock step %lu: phase %ld us -> cam%d VTS %u+%lu for one frame\r\n",
           (unsigned long)step, (long)phi, cam_id, (unsigned)vts, (unsigned long)lines);
    int trc = cam_vts_tweak_one_frame(cam_id, vts, lines, T);
    if (trc != 0) {
      if (trc != -51) SERR_LOG(SERR_CAM_GENLOCK_FAIL, (uint32_t)(-trc));  // -51 already logged
      return trc;
    }
    g_cam_genlock_steps = step + 1;
  }
  SERR_LOG(SERR_CAM_GENLOCK_FAIL, 40u);
  printf("[cam] genlock FAILED: residual phase %ld us\r\n", (long)phi);
  return -40;
}

extern "C" int sentai_cam_genlock_auto(int set) {   // set < 0: query; 0/1/2
  if (set >= 0) g_cam_genlock_auto = (set > 2) ? 2u : (uint32_t)set;
  return (int)g_cam_genlock_auto;
}

extern "C" int sentai_cam_autotune(int set) {       // set < 0: query
  if (set >= 0) g_cam_autotune_enabled = set ? 1u : 0u;
  return (int)g_cam_autotune_enabled;
}

extern volatile uint32_t g_cam_flip_delay_us, g_cam_flips_deferred, g_cam_flips_executed;  // camera_support.c
extern "C" int sentai_cam_ratio_unit(int set) {       // set < 0: query; 1 or 2
  if (set == 1 || set == 2) g_cam_ratio_unit = (uint32_t)set;
  return (int)g_cam_ratio_unit;
}
extern "C" int sentai_cam_flip_delay_us(int set) {   // set < 0: query; 0 = legacy immediate flip
  if (set >= 0) g_cam_flip_delay_us = (set > 400) ? 400u : (uint32_t)set;
  return (int)g_cam_flip_delay_us;
}
extern "C" void sentai_cam_flip_stats(uint32_t* deferred, uint32_t* executed) {
  if (deferred) *deferred = g_cam_flips_deferred;
  if (executed) *executed = g_cam_flips_executed;
}

extern "C" void sentai_cam_autotune_stats(uint32_t* pretrim_lines, uint32_t* trims) {
  if (pretrim_lines) *pretrim_lines = g_cam_pretrim_lines;
  if (trims) *trims = g_cam_autotune_trims;
}
extern "C" void sentai_cam_timing_stats(uint32_t* vblank_us, uint32_t* period_us, uint32_t* vts) {
  if (vblank_us) *vblank_us = sentai_cam_vblank_us();
  if (period_us) *period_us = cam_period_us();
  if (vts) *vts = g_cam_vts_lines;
}

// ---- System LittleFS (NAND) raw access for diagnostics -------------------
// Bounded, static-buffer readers over the NAND system partition (the one
// the ROM/elf_loader boots from: /default.elf).  Used to confirm what a
// persistent flashtool run actually left on the NAND when the board cannot
// boot from flash (board rev 2026-10 boot straps, EXP-s248/s253).
// (lfs.h and coralmicro::Lfs() come from libs/base/filesystem.h)
// DTCM (m_data) is full to the byte: cold diagnostic buffers go to SDRAM.
__attribute__((section(".sdram_bss"))) static uint8_t g_sysfs_scan_buf[4096 + 64];
__attribute__((section(".sdram_bss"))) uint8_t g_sysfs_read_buf[512];

extern "C" int sentai_sysfs_size(const char* path) {
  lfs_t* lfs = coralmicro::Lfs();
  if (!lfs) return -1;
  lfs_file_t f;
  if (lfs_file_open(lfs, &f, path, LFS_O_RDONLY) < 0) return -2;
  lfs_soff_t sz = lfs_file_size(lfs, &f);
  lfs_file_close(lfs, &f);
  return (int)sz;
}

extern "C" int sentai_sysfs_read(const char* path, uint32_t off, uint8_t* buf, uint32_t n) {
  lfs_t* lfs = coralmicro::Lfs();
  if (!lfs) return -1;
  lfs_file_t f;
  if (lfs_file_open(lfs, &f, path, LFS_O_RDONLY) < 0) return -2;
  int rc = -3;
  if (lfs_file_seek(lfs, &f, (lfs_soff_t)off, LFS_SEEK_SET) >= 0) {
    lfs_ssize_t r = lfs_file_read(lfs, &f, buf, n);
    rc = (r < 0) ? -3 : (int)r;
  }
  lfs_file_close(lfs, &f);
  return rc;
}

// First offset >= start where `needle` (nlen <= 64) occurs, or -1.  Scans in
// 4 KB chunks with a (nlen-1) overlap; bounded by the file size.
extern "C" int sentai_sysfs_find(const char* path, const uint8_t* needle, uint32_t nlen, uint32_t start) {
  if (nlen == 0 || nlen > 64) return -4;
  lfs_t* lfs = coralmicro::Lfs();
  if (!lfs) return -1;
  lfs_file_t f;
  if (lfs_file_open(lfs, &f, path, LFS_O_RDONLY) < 0) return -2;
  lfs_soff_t size = lfs_file_size(lfs, &f);
  int found = -1;
  uint32_t pos = start;
  while (size > 0 && pos + nlen <= (uint32_t)size) {
    if (lfs_file_seek(lfs, &f, (lfs_soff_t)pos, LFS_SEEK_SET) < 0) break;
    lfs_ssize_t r = lfs_file_read(lfs, &f, g_sysfs_scan_buf, sizeof(g_sysfs_scan_buf));
    if (r < (lfs_ssize_t)nlen) break;
    for (uint32_t i = 0; i + nlen <= (uint32_t)r; ++i) {
      if (g_sysfs_scan_buf[i] == needle[0] && memcmp(g_sysfs_scan_buf + i, needle, nlen) == 0) {
        found = (int)(pos + i);
        break;
      }
    }
    if (found >= 0) break;
    pos += 4096;
  }
  lfs_file_close(lfs, &f);
  return found;
}

extern "C" uint32_t sentai_cam_csi_queue_faults(void) {
  return g_cam_csi_queue_faults;
}

extern "C" void sentai_cam_genlock_stats(int32_t* phase_us, uint32_t* valid,
                                         uint32_t* ok, uint32_t* steps,
                                         uint32_t* stale_skips, uint32_t* drift) {
  if (phase_us) *phase_us = g_cam_phase_us;
  if (valid) *valid = g_cam_phase_valid;
  if (ok) *ok = g_cam_genlock_ok;
  if (steps) *steps = g_cam_genlock_steps;
  if (stale_skips) *stale_skips = g_cam_stale_skips;
  if (drift) *drift = g_cam_phase_drift_cnt;
}

extern "C" __attribute__((section(".sdram_text"))) int sentai_cam_init_full(
    int streaming, int fps, int hflip, int vflip) {
  /* Validate fps against the patched fsl_ov5640.c VGA table. */
  if (fps != 15 && fps != 30 && fps != 45 && fps != 60 && fps != 90 &&
      fps != 22 /* SXGA only: the 22.6 fps PLL-672 row (legacy 4 B/px capture) */) {
    return -10;
  }
  /* SXGA at a true 30 fps with the legacy XYUV8888 capture (4 B/px, 147 MB/s
   * of CSI writes) starves the SDRAM: the full pipeline hung the board (REPL
   * silent, no crash log; EXP-s260).  Only the packed YUV420 capture
   * (camera.packed420(1), 2 B/px) may run that row; legacy SXGA uses 22. */
  if (g_runtime_cam_w == 1280u && fps == 30 && !g_sentai_cam_packed420) return -12;
  if (g_cam_initialized) {
    /* Refuse second init at a different fps — it would wedge.
     * Caller must sentai.sys.reset() first. */
    if ((uint32_t)fps != g_runtime_fps) return -11;
    return 0;  /* already initialized at the same fps */
  }
  /* NASA/JPL fault containment: snapshot the previous runtime fps so
   * we can roll it back on a SetPower / Enable failure.  Without this
   * a failed init would leave g_runtime_fps mutated to the requested
   * value while g_cam_initialized stayed false — the next caller would
   * inherit a half-applied state with no recourse but reboot. */
  const uint32_t prev_fps = g_runtime_fps;
  g_runtime_fps = (uint32_t)fps;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (!cam->SetPower(true)) {
    g_runtime_fps = prev_fps;
    sentai_health_periph(PERIPH_CAM0, (cam->DetectedMask() & 1u) ? 1 : 0);
    sentai_health_periph(PERIPH_CAM1, (cam->DetectedMask() & 2u) ? 1 : 0);
    return -1;
  }
  auto mode = streaming ? coralmicro::CameraMode::kStreaming
                        : coralmicro::CameraMode::kTrigger;
  if (!cam->Enable(mode)) {
    /* Power was raised but Enable failed — bring power back down so
     * the OV5640 isn't left in an indeterminate streaming state, then
     * roll back the runtime fps. */
    (void)cam->SetPower(false);
    g_runtime_fps = prev_fps;
    sentai_health_periph(PERIPH_CAM0, (cam->DetectedMask() & 1u) ? 1 : 0);
    sentai_health_periph(PERIPH_CAM1, (cam->DetectedMask() & 2u) ? 1 : 0);
    return -2;
  }
  g_cam_initialized = true;
  /* Touch each camera in turn so the OV5640 init driver runs against
   * each silicon, then optionally apply orientation override.  Order
   * matters: end on cam 0 so the next pipeline read sees front. */
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
  g_cam_current_id = 1;
  if (hflip >= 0 && vflip >= 0) {
    int deg = sentai_flip_to_degrees(hflip, vflip);
    (void)cam->SetCameraRotation(1, deg);   /* best-effort; logs on failure */
  }
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
  g_cam_current_id = 0;
  if (hflip >= 0 && vflip >= 0) {
    int deg = sentai_flip_to_degrees(hflip, vflip);
    (void)cam->SetCameraRotation(0, deg);
  }
  {
    uint8_t dm = cam->DetectedMask();
    sentai_health_periph(PERIPH_CAM0, (dm & 1u) ? 1 : 0);
    sentai_health_periph(PERIPH_CAM1, (dm & 2u) ? 1 : 0);
  }
  if (g_sentai_cam_packed420) {
    /* EXP-s260 option B: switch the ACTIVE sensor (cam 0) to "YUV420 for
     * MIPI only" (0x4300=0x5F).  The SXGA rows also need the DVP PCLK
     * divider forced to manual /1 (0x460C bit1 + 0x3824), otherwise the
     * bytes arrive scrambled (EXP-s259).  VGA rows already run 0x22/0x02. */
    bool ok = cam->WriteToCam(0, 0x4300, 0x5F);
    if (g_runtime_cam_w == 1280u) {
      ok &= cam->WriteToCam(0, 0x460C, 0x22);
      ok &= cam->WriteToCam(0, 0x3824, 0x01);
    }
    printf("[cam] packed420 capture: sensor format 0x5F%s -> %s\r\n",
           g_runtime_cam_w == 1280u ? " + PCLK div manual/1" : "", ok ? "ok" : "I2C FAIL");
    vTaskDelay(pdMS_TO_TICKS(80));   /* two frames for the format to settle */
  }
  // EXP-s251 proposal A: bring the two sensors in phase so ratio(a,b)
  // alternation is clean at the full sensor rate.  Typically 2-3 s, bounded
  // (worst case ~20 s, see sentai_cam_genlock); failure is logged
  // (SERR_CAM_GENLOCK_FAIL) and leaves the conservative drain/dirty defaults
  // in place.  Measurement only (no standby restart) at init: the restart
  // path is available via sentai.camera.genlock(1).  Streaming mode only.
  if (streaming && cam->DetectedMask() == 0x3) {
    g_cam_phase_T_us = cam_nominal_period_us();   // no stale estimate from another mode
    cam_autotune_task_start();
    if (g_cam_genlock_auto >= 1u) (void)sentai_cam_pretrim();      // no flips
    if (g_cam_genlock_auto >= 2u) (void)sentai_cam_genlock(0);     // measured verify (flips)
  }
  // W24-T22: per-module colour register sets from the user partition.
  if (streaming) cam_apply_module_regs(cam);
  return 0;
}

/* Backward-compat: no orientation override, leave OV5640 defaults. */
extern "C" int sentai_cam_init_fps(int streaming, int fps) {
  return sentai_cam_init_full(streaming, fps, -1, -1);
}

/* Existing entry point — preserved for any caller that doesn't
 * want to pick fps explicitly.  Uses the runtime default (30 unless
 * overridden by a prior init_fps call before stop). */
extern "C" int sentai_cam_init(int streaming) {
  return sentai_cam_init_fps(streaming, (int)g_runtime_fps);
}

extern "C" int sentai_cam_stop(void) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->Disable();
  cam->SetPower(false);
  g_cam_initialized = false;
  return 0;
}

/* Board rev 2026-10 bring-up: the camera LDO rails (PMIC LDO1..4 =
 * VDD_1V8/2V8 for CAM1 and CAM2) are not switched by the camera driver
 * (its PMIC sequence is compiled out, "PMIC is not working well").  On
 * the new board the CAM1 I2C isolation switch (U9) is enabled BY the
 * VDD_1V8_CAM1 rail, so without it camera 1 NAKs on LPI2C1.  REPL knob:
 * sentai.camera.power_rails(1).  2V8 first, then 1V8 (OV5640 sequence). */
extern "C" int sentai_cam_rails_set(int on) {
  auto* pmic = coralmicro::PmicTask::GetSingleton();
  using coralmicro::PmicRail;
  if (on) {
    pmic->SetRailState(PmicRail::kCam1_2V8, true);
    pmic->SetRailState(PmicRail::kCam2_2V8, true);
    vTaskDelay(pdMS_TO_TICKS(20));
    pmic->SetRailState(PmicRail::kCam1_1V8, true);
    pmic->SetRailState(PmicRail::kCam2_1V8, true);
  } else {
    pmic->SetRailState(PmicRail::kCam1_1V8, false);
    pmic->SetRailState(PmicRail::kCam2_1V8, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    pmic->SetRailState(PmicRail::kCam1_2V8, false);
    pmic->SetRailState(PmicRail::kCam2_2V8, false);
  }
  vTaskDelay(pdMS_TO_TICKS(50));  // rail + isolation-switch settle
  return 0;
}

/* Build #958 — runtime fps switch.  Drops camera, updates the
 * runtime fps variable, brings camera back so the OV5640 driver
 * re-runs the clock-config table with the new framePerSec.
 * Valid: 15/30/45/60/90 (per fsl_ov5640.c VGA table).
 *
 * Placed in .sdram_text — this is a one-shot config call invoked
 * from REPL, NOT on the per-frame ISR / pipeline hot path. */
/* Build #980: sentai_cam_set_fps removed — runtime fps switch is
 * blocked by the CSI receiver re-init wedge documented in
 * memory/project_runtime_fps_switch_blocked.md.  Use
 * sentai_cam_init_fps(streaming, fps) at FIRST init instead. */

// Try to get a raw frame with recovery.
// Drains stale buffered frames first so the caller always gets the LATEST frame.
// NOTE: TryGetRawFrame is NOT truly non-blocking — inside the camera task,
// HandleFrameRequest polls GetFullBuffer up to 40×100ms = 4 seconds.
// So each call either succeeds quickly (~1ms) or blocks up to 4s.
// We try ONCE per attempt, then do recovery if it fails.
static int sentai_cam_get_raw_with_recovery(uint8_t** raw_out) {
  if (sentai_virtual_camera_active()) {
    int idx = sentai_virtual_camera_grab_xrgb(raw_out);
    if (idx >= 0) {
      g_cam_grabbed_id = sentai_virtual_camera_grabbed_id();
      return idx;
    }
  }

  auto* cam = coralmicro::CameraTask::GetSingleton();
  const int kMaxRecoveries = 2;

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
    // 2026-04-25: threshold==0 now allowed → always fast path
    if (threshold > 10) threshold = 10;

    if (elapsed >= threshold) {
      // Fast path: enough ISR frames have already arrived since MUX
      // flip.  Drain stale queued buffers and keep the latest — no
      // blocking.  No log: the fast path is the common case and would
      // spam CDC-ACM during any sustained switching loop.
      // Fall through to normal drain-and-keep-last below.
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
      // kDrainWaitMaxIters = 300 iterations × 1 ms = 300 ms ceiling.
      // Rationale (embeded.md §A7 bounded behaviour): even if CSI
      // stops firing ISRs entirely, the task wakes up every ~33 ms
      // (worst case) so 300 ms > 9 frame intervals at 30 fps — enough
      // margin that a real frame arrives, but low enough that a
      // genuine driver fault falls through to the recovery path in
      // sub-second time.  Hitting this ceiling increments
      // g_cam_drain_timeout_count.
      const int kDrainWaitMaxIters = 300;
      int wait_iters = 0;
      while ((g_camera_frame_seq - seq_at_switch) < threshold
             && wait_iters < kDrainWaitMaxIters) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_iters++;
      }
      if (wait_iters >= kDrainWaitMaxIters) {
        g_cam_drain_timeout_count++;
        SERR_LOG(SERR_CAM_DRAIN_TIMEOUT, (uint32_t)wait_iters);
      }
      // Grab one fresh frame (blocking, up to 4 s worst case per
      // cam->GetRawFrame's internal poll; acceptable since we reach
      // here only on the slow path right after a switch).
      uint8_t* frame = nullptr;
      int idx = cam->GetRawFrame(&frame);
      if (idx >= 0 && frame) {
        // Build #953 — slow-path dirty check.  Was missing before:
        // a buffer dequeued via the slow blocking grab could carry
        // a mid-frame-mix from a recent flip, get returned to the
        // caller without skipping.  Now we drop and fall through.
        extern volatile uint8_t g_cam_buf_dirty[];
        extern volatile uint32_t g_cam_buf_dirty_skips;
        if (g_cam_buf_dirty[idx]) {
          g_cam_buf_dirty[idx] = 0u;
          g_cam_buf_dirty_skips++;
          cam->ReturnRawFrame(idx);
          // Fall through to recovery loop below.
        } else {
          g_cam_grabbed_id = (int)g_cam_buf_id_task[idx];
          *raw_out = frame;
          return idx;
        }
      }
      // Fresh frame failed (or was dirty) — fall through to recovery.
    }
  }

  int stale_skips_here = 0;
  for (int recovery = 0; recovery <= kMaxRecoveries; ++recovery) {
    // Drain the FIFO queue non-blockingly and keep only the most recent frame.
    // TryGetRawFrame is now truly non-blocking (single GetFullBuffer probe, no
    // 4-second poll), so the drain loop takes microseconds.  With N DMA
    // buffers the loop tops out at N-1 iterations.  We never wait here — the
    // caller decides whether to bounded-wait for a fresh frame below.
    uint8_t* kept_frame = nullptr;
    int kept_idx = -1;
    int drained = 0;
    static_assert(DEMO_CAMERA_BUFFER_COUNT <= 8, "stale-skip bound assumes a small pool");
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
      // Snapshot the AUTHORITATIVE task-context tag (build #886+).
      // CameraTask::HandleFrameRequest tagged g_cam_buf_id_task[kept_idx]
      // when it dequeued this buffer — guaranteed (idx, source) pair.
      // The legacy ISR-side g_cam_buf_id[] is shuffled by multiple
      // FB-done IRQs per sensor frame (re-arm path generates extras,
      // active_cam stale by one IRQ).
      //
      // Build #911 fix: actually consult g_cam_buf_id_task[] — earlier
      // builds had this comment but still read g_cam_buf_id[], which
      // gave ~90% pattern↔tag match at alt 3:1.  Single replacement
      // here lifts that to ~100% (verified by diag/_t_pattern_31.py
      // PHASE B).
      // Build #942 — skip dirty buffers (mid-frame mix from MUX
      // flip).  Bit is set by CSI ISR at FB-done after the flip;
      // we consume-and-clear via a single store; ReturnRawFrame
      // recycles the buffer and the for(recovery) loop iterates
      // (bounded by kMaxRecoveries+1).  NASA §1: explicit FSM-style
      // skip; §11: single-writer (ISR) / single-clearer (here).
      {
        extern volatile uint8_t g_cam_buf_dirty[];
        extern volatile uint32_t g_cam_buf_dirty_skips;
        if (g_cam_buf_dirty[kept_idx]) {
          g_cam_buf_dirty[kept_idx] = 0u;
          g_cam_buf_dirty_skips++;
          cam->ReturnRawFrame(kept_idx);
          continue;
        }
      }
      // EXP-s251 proposal C: a buffer that sat in the queue while nobody
      // consumed is OLD (measured 0.5-1 s after an idle second: three full
      // buffers wait in the queue while the CSI overwrites the fourth).
      // Drop anything older than two frame periods and let the blocking
      // grab below return a fresh one.  ts == 0 means never queued through
      // the frame-complete hook (e.g. before the first frame): no opinion.
      {
        extern volatile uint32_t g_cam_buf_ts_us[];
        uint32_t ts = (kept_idx < (int)DEMO_CAMERA_BUFFER_COUNT) ? g_cam_buf_ts_us[kept_idx] : 0u;
        if (ts != 0u) {
          uint32_t age_us = GPT1->CNT - ts;
          uint32_t fps = g_runtime_fps ? g_runtime_fps : 30u;
          uint32_t stale_us = 2000000u / fps;
          if (age_us > stale_us && stale_skips_here < DEMO_CAMERA_BUFFER_COUNT) {
            // Bounded by the pool size and does not consume the recovery
            // budget: a stale buffer is not a CSI failure.
            g_cam_stale_skips++;
            stale_skips_here++;
            cam->ReturnRawFrame(kept_idx);
            --recovery;
            continue;
          }
        }
      }
      g_cam_grabbed_id = (int)g_cam_buf_id_task[kept_idx];  // build #911 baseline
      *raw_out = kept_frame;
      return kept_idx;
    }

    // Queue is empty right now.  Bounded blocking grab so PrepTask can
    // make forward progress — at 30 fps the next frame arrives in ≤ 33
    // ms, well within the 4-second internal poll ceiling of
    // cam->GetRawFrame.  If that fails, fall through to the toggle-
    // to-recover path below.
    uint8_t* frame = nullptr;
    int idx = cam->GetRawFrame(&frame);
    if (idx >= 0 && frame) {
      // Build #953 — blocking-path dirty check (was missing).
      extern volatile uint8_t g_cam_buf_dirty[];
      extern volatile uint32_t g_cam_buf_dirty_skips;
      if (g_cam_buf_dirty[idx]) {
        g_cam_buf_dirty[idx] = 0u;
        g_cam_buf_dirty_skips++;
        cam->ReturnRawFrame(idx);
        continue;
      }
      g_cam_grabbed_id = (int)g_cam_buf_id_task[idx];  // build #911 baseline
      *raw_out = frame;
      return idx;
    }

    // No frames at all — try toggling camera to kick CSI/MIPI.
    if (recovery < kMaxRecoveries) {
      g_cam_grab_retry_count++;
      SERR_LOG(SERR_CAM_GRAB_RETRY, (uint32_t)recovery);
      int other = (g_cam_current_id == 0) ? 1 : 0;
      cam->SwitchCamera(other == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                   : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
      cam->SwitchCamera(g_cam_current_id == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                              : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // All recoveries exhausted.  Fatal from the caller's perspective but
  // bounded (we return an error, not hang).
  g_cam_grab_fatal_count++;
  SERR_LOG(SERR_CAM_GRAB_FAIL, (uint32_t)g_cam_current_id);
  *raw_out = nullptr;
  return -2;
}

// Capture RGB frame via PXP hardware scaler. Returns 0 on success.
extern "C" int sentai_cam_capture_rgb(uint8_t* buf, int width, int height) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns PXP
  if (!g_cam_initialized && !sentai_virtual_camera_active()) return -1;
  if (cam_src_packed()) return -7;   // frame is not XRGB (W24-T20)
  const int src_w = cam_src_w(), src_h = cam_src_h();   // runtime geometry
  if (width > src_w || height > src_h) return -5;
  uint8_t* raw = nullptr;
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;

  int rc = pxp_scale_xrgb_to_rgb(raw, src_w, src_h,
                                  buf, width, height);
  TickType_t t2 = xTaskGetTickCount();
  sentai_cam_return_raw(idx);
  printf("  [capture_rgb] drain=%ldms pxp=%ldms\r\n",
         (long)(t1 - t0), (long)(t2 - t1));
  return rc;
}

// Persistent RGB buffer in SDRAM — avoids allocating 2.7MB on every call
static uint8_t s_jpeg_rgb_buf[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT * 3]
    __attribute__((section(".sdram_bss")));

// ---- EXP-s256: YUV 4:2:0 path -------------------------------------------
// The RT1176 PXP has no CSC2 (FSL_FEATURE_PXP_HAS_NO_CSC2), so RGB -> YCbCr
// cannot be done in hardware.  Instead the camera itself runs in native YUV
// (sentai.camera.yuv_mode(1) before init: OV5640 YUV 4:2:2 -> CSI stores
// XYUV8888) and the existing s240 PXP pass repacks that into a Y plane plus
// an interleaved UV plane with 2x2 chroma decimation -- no colour math at
// all.  A short integer pass splits UV into the Cb/Cr planes libjpeg's
// raw_data_in wants.  Same buffer family the Edge TPU YUV420 models use.
static uint8_t s_jpeg_y[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT]
    __attribute__((section(".sdram_bss"), aligned(32)));
static uint8_t s_jpeg_cbcr[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT / 2]
    __attribute__((section(".sdram_bss"), aligned(32)));
static uint8_t s_jpeg_cb[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT / 4]
    __attribute__((section(".sdram_bss"), aligned(32)));
static uint8_t s_jpeg_cr[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT / 4]
    __attribute__((section(".sdram_bss"), aligned(32)));
extern "C" int sentai_pxp_xyuv_to_yuv420(const uint8_t* src, int src_w, int src_h,
                                         uint8_t* y_dst, uint8_t* uv_dst);
extern "C" int sentai_camera_get_yuv_mode(void);

// Split interleaved CbCr (w/2 x h/2 pairs) into two planes: 4 pairs per
// 64-bit step, pure integer, ~1 ms for 640x480 on the M7.
static void cbcr_deinterleave(const uint8_t* cbcr, uint8_t* cb, uint8_t* cr, int pairs) {
  const uint32_t* s = (const uint32_t*)cbcr;
  int i = 0;
  for (; i + 2 <= pairs; i += 2) {
    uint32_t v = *s++;                        // Cb0 Cr0 Cb1 Cr1
    cb[i]     = (uint8_t)(v);
    cr[i]     = (uint8_t)(v >> 8);
    cb[i + 1] = (uint8_t)(v >> 16);
    cr[i + 1] = (uint8_t)(v >> 24);
  }
  for (; i < pairs; ++i) { cb[i] = cbcr[2 * i]; cr[i] = cbcr[2 * i + 1]; }
}

// Capture + JPEG through the YUV path (VGA XYUV frames only: the plane buffers
// above are 640x480 and the PXP split pass reads XYUV rows).
// Returns the JPEG size, or -1 camera off, -2 no frame, -5 source geometry is
// not 640x480, -6 frames are XRGB (use the RGB path), -7 packed YUV420 frames,
// -10 pipeline running.
extern "C" int sentai_cam_capture_jpeg_yuv(uint8_t* jpeg_buf, int jpeg_buf_size, int quality) {
  if (sentai_detection_is_running()) return -10;
  if (!g_cam_initialized) return -1;
  if (!sentai_camera_get_yuv_mode()) return -6;   // frames are XRGB: use the RGB path
  if (g_sentai_cam_packed420) return -7;          // W24-T20: [C,Y1,Y2,0] pairs, not XYUV
  const int w = DEMO_CAMERA_WIDTH, h = DEMO_CAMERA_HEIGHT;   // = sizeof the plane buffers
  if ((int)g_runtime_cam_w != w || (int)g_runtime_cam_h != h) return -5;   // SXGA: planes too small
  uint8_t* raw = nullptr;
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;
  int rc = sentai_pxp_xyuv_to_yuv420(raw, w, h, s_jpeg_y, s_jpeg_cbcr);
  sentai_cam_return_raw(idx);
  if (rc != 0) return rc;
  TickType_t t2 = xTaskGetTickCount();
  cbcr_deinterleave(s_jpeg_cbcr, s_jpeg_cb, s_jpeg_cr, (w / 2) * (h / 2));
  TickType_t t3 = xTaskGetTickCount();
  unsigned long used = coralmicro::JpegCompressYuv420(s_jpeg_y, s_jpeg_cb, s_jpeg_cr, w, h, quality,
                                                      (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);
  TickType_t t4 = xTaskGetTickCount();
  printf("  [capture_jpeg_yuv] drain=%ldms pxp=%ldms split=%ldms jpeg=%ldms total=%ldms\r\n",
         (long)(t1 - t0), (long)(t2 - t1), (long)(t3 - t2), (long)(t4 - t3), (long)(t4 - t0));
  return (int)used;
}

// Capture + JPEG compress. Returns JPEG size or negative error.
// Uses PXP hardware for XRGB8888→RGB888 conversion (+ optional scaling),
// then JpegCompressRgb for JPEG encoding.
extern "C" int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                      int width, int height, int quality) {
  if (sentai_detection_is_running()) return -10;  // pipeline owns PXP
  if (!g_cam_initialized && !sentai_virtual_camera_active()) return -1;
  // Source = the runtime sensor geometry (640x480 or 1280x960, W24-T11), not the
  // compile-time VGA constants: with those every SXGA capture failed with -5
  // (EXP-s266).  The RGB staging buffer holds 640x480x3, so the caller asks for
  // an output no larger than that (the MicroPython binding halves SXGA to VGA).
  int src_w = sentai_virtual_camera_active()
                  ? sentai_virtual_camera_width()
                  : (int)g_runtime_cam_w;
  int src_h = sentai_virtual_camera_active()
                  ? sentai_virtual_camera_height()
                  : (int)g_runtime_cam_h;
  if (width > src_w || height > src_h) return -5;
  if ((size_t)width * (size_t)height * 3u > sizeof(s_jpeg_rgb_buf)) return -6;
  if (g_sentai_cam_packed420 && !sentai_virtual_camera_active()) return -7;  // frame is not XRGB

  uint8_t* raw = nullptr;
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;

  // PXP hardware: XRGB8888 → packed RGB888 (with optional scaling)
  int rc = pxp_scale_xrgb_to_rgb(raw, src_w, src_h,
                                  s_jpeg_rgb_buf, width, height);
  TickType_t t2 = xTaskGetTickCount();
  sentai_cam_return_raw(idx);
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
  if (!g_cam_initialized && !sentai_virtual_camera_active()) return -1;
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -4;
  int h = input->dims->data[1];
  int w = input->dims->data[2];
  int ch = input->dims->data[3];
  int total_pixels = h * w * ch;
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  if (cam_src_packed()) return -7;   // frame is not XRGB (W24-T20)
  const int src_w = cam_src_w(), src_h = cam_src_h();   // runtime geometry
  TickType_t t_start = xTaskGetTickCount();
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t_frame = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;
  int rc = pxp_scale_xrgb_to_rgb(raw, src_w, src_h,
                                  tensor_buf, w, h);
  sentai_cam_return_raw(idx);
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
// and letting the CSI EOF ISR (libs/camera/camera_support.c) perform
// the actual GPIO flip the next time a DMA buffer completes.  That
// window is the MIPI VBLANK interval — both sensors are between frame
// transmissions — so the switch never lands mid-buffer and the
// following frame is guaranteed to be 100 % from the new sensor.
//
// Bounded behaviour (embeded.md §B):
//   - Arm is non-blocking from the caller's perspective.
//   - We then poll for ISR consumption using DELTA-based deadline
//     arithmetic (survives TickType_t wrap at 49.7 d uptime).
//   - Timeout = 3 × frame interval at 30 fps (100 ms) + 50 ms margin
//     for jitter = 150 ms.  Source: embeded.md §A7 (bounded behaviour
//     must be measurable and traceable).
//   - If the arm has not been consumed by then, the CSI is either not
//     firing EOF (driver stuck) or we missed a frame cycle — we fall
//     back to the legacy synchronous path which flips the GPIO in task
//     context.  This path re-introduces the mid-buffer seam; the
//     operator sees both a log line (when verbose is on) and a
//     persistent `g_cam_switch_fallback_count` increment.
//
// Ownership (embeded.md §E):
//   - `g_cam_current_id`:   WRITTEN BY ISR on nominal path, written by
//                           this function only on the fallback path.
//                           Readers must assume either writer.
//   - `g_cam_switch_seq`:   Written by ISR (VBLANK flip) or by
//                           HandleSwitchCameraRequest (sync fallback).
//   - `g_cam_switch_pending`: set by the writer that actually flipped
//                             the GPIO, cleared by the drain path.
//   - `sentai_tracker_set_active_camera`: called AFTER the MUX flip
//                                         has actually happened so
//                                         tracker state follows
//                                         hardware reality.
extern "C" int sentai_cam_switch(int id) {
  if (!g_cam_initialized) return -1;
  if (id != 0 && id != 1) return -2;
  if (id == g_cam_current_id && !g_cam_switch_pending) return 0;

  const TickType_t ts0 = xTaskGetTickCount();

  /* Arm-budget covers worst-case sensor-period × 3 + 50 ms jitter.
   * 250 ms is safe across the entire validated runtime fps range
   * {15, 30, 45, 60, 90} — VGA15 worst case is 200 ms (3 × 66.7),
   * VGA60 fits comfortably (100 ms).  Was 150 ms literal keyed on
   * the unwritten DEMO_CAMERA_FRAME_RATE == 30 assumption — fixed
   * during the build #98x runtime-fps audit.  A constant ceiling
   * is preferred over runtime division to keep this hot REPL path
   * branch-free and ITCM-friendly. */
  const TickType_t kArmTimeoutTicks = pdMS_TO_TICKS(250);

  // Arm the ISR.
  g_cam_pending_mux_id = id;

  // Bounded wait, DELTA-subtraction (A3): `now - ts0` is the same 32-bit
  // unsigned subtraction regardless of any tick-counter wrap, whereas
  // `now < deadline` fails if the deadline straddles 0.  vTaskDelay(1)
  // grants other tasks the rest of the current tick slice.
  while (g_cam_pending_mux_id >= 0 &&
         (xTaskGetTickCount() - ts0) < kArmTimeoutTicks) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (g_cam_pending_mux_id >= 0) {
    // ----- Fallback: ISR did not consume the arm within the budget ---
    // Disarm first so a late ISR does not double-flip, then do the
    // synchronous flip through CameraTask.  Snapshot of
    // g_cam_switch_seq is taken inside HandleSwitchCameraRequest
    // (libs/camera/camera.cc) which runs to completion before
    // cam->SwitchCamera returns (SendRequest blocks on its semaphore).
    g_cam_pending_mux_id = -1;
    g_cam_switch_fallback_count++;  // persistent breadcrumb (embeded.md §F)

    auto* cam = coralmicro::CameraTask::GetSingleton();
    if (id == 0) {
      cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
    } else {
      cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
    }
    g_cam_switch_pending = true;
    g_cam_current_id = id;                          // sync path sole writer
    sentai_tracker_set_active_camera(id);           // A5: after actual flip
    if (g_sentai_frame_verbose) {                   // B4: gate on verbose flag
      SERR_LOG(SERR_CAM_SWITCH_FALLBACK, (uint32_t)id);
    }
    return 0;
  }

  // ----- Nominal path: ISR already consumed the arm.  ---------------
  // g_cam_current_id, g_cam_switch_seq, g_cam_switch_pending were all
  // written by the ISR in VBLANK context — no need to touch them here
  // (A4: removed the redundant task-side write).
  g_cam_switch_ok_eof_count++;
  sentai_tracker_set_active_camera(id);             // A5: after actual flip
  if (g_sentai_frame_verbose) {                     // B4: gate on verbose flag
    SERR_LOG(SERR_CAM_SWITCH_EOF, (uint32_t)id);
  }
  return 0;
}

// Tune the OV5640 auto-exposure target luminance.
// Writes the "stable range" + "fast zone" AEC regs for the chosen
// camera.  Higher `high` → brighter target image.  Typical ranges:
//   NXP default  : high=0x30 low=0x28  (≈ 18% target — dark indoors)
//   OmniVision AN: high=0x78 low=0x68  (≈ 45% target — general)
// Returns 0 on success, negative on error.
extern "C" int sentai_cam_aec_set(int cam_id, int high, int low) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -2;
  if (high < 0x10 || high > 0xF0) return -3;
  if (low  < 0x10 || low  > high) return -4;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = true;
  ok &= cam->WriteToCam(cam_id, 0x3A0F, (uint8_t)high);
  ok &= cam->WriteToCam(cam_id, 0x3A10, (uint8_t)low);
  ok &= cam->WriteToCam(cam_id, 0x3A1B, (uint8_t)high);
  ok &= cam->WriteToCam(cam_id, 0x3A1E, (uint8_t)low);
  // Fast zone widened proportionally so the AEC acts quickly when
  // the scene changes.  Typical AN ratio: fast_high = ~1.1× stable.
  ok &= cam->WriteToCam(cam_id, 0x3A11, (uint8_t)(high + 0x40 > 0xFF
                                                   ? 0xFF : high + 0x40));
  ok &= cam->WriteToCam(cam_id, 0x3A1F, (uint8_t)(low  > 0x20
                                                   ? (low - 0x20) : 0));
  return ok ? 0 : -5;
}

// Tune the OV5640 auto-gain ceiling.  The AEC uses gain to reach
// the target luminance when exposure time alone can't do it (dark
// scenes).  NXP default is 0x007C (15.5× max gain) — too low for
// dimly-lit indoor shots.  OmniVision AN suggests 0x00F8 (31×)
// for low-light scenes; the trade-off is more noise.
//
// Register layout: 0x3A18[1:0] = high 2 bits, 0x3A19[7:0] = low 8 bits
// → 10-bit value in units of 1/16 gain.  Pass `ceiling` as the
// full 10-bit integer (0..1023).
extern "C" int sentai_cam_gain_ceiling_set(int cam_id, int ceiling) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -2;
  if (ceiling < 0x010 || ceiling > 0x3FF) return -3;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = true;
  ok &= cam->WriteToCam(cam_id, 0x3A18, (uint8_t)((ceiling >> 8) & 0x03));
  ok &= cam->WriteToCam(cam_id, 0x3A19, (uint8_t)(ceiling & 0xFF));
  return ok ? 0 : -4;
}

// ===== Synthetic test pattern (used during sentai.pipeline.calibrate)
//
// Activates OV5640 hardware test pattern via reg 0x503D.  The
// sensor injects DETERMINISTIC pixels into the data stream BEFORE
// the ISP pipeline (AWB / CMX / Gamma) which then scales them.
// Absolute byte values are therefore NOT predictable, but the
// SPATIAL STRUCTURE of each pattern is unique and ISP-scaling-
// invariant.  Discriminator is structural, not value-based.
//
// reg 0x503D layout (per OV5640 datasheet table 4-3):
//   bit 7    = enable test pattern (1 = synthetic, 0 = real scene)
//   bit[3:2] = pattern style:
//              00 = standard 8 vertical color bars
//                   (WHITE, YELLOW, CYAN, GREEN, MAGENTA, RED,
//                    BLUE, BLACK; tranzitii brute la fiecare W/8)
//              01 = gradual change at vertical mode 1
//              10 = gradual change at horizontal
//                   (each row = uniform color; gradient down)
//              11 = gradual change at vertical mode 2
//
// mode:
//   0 = OFF      (0x503D = 0x00) real scene, ISP auto
//   1 = BARS     (0x503D = 0x80) vertical bars — row 0 has 7-8
//                 abrupt B-channel transitions across width
//   2 = HBAND    (0x503D = 0x88) horizontal bands — row 0 is
//                 NEAR-UNIFORM (gradient is row-to-row only)
//
// These two are mutually exclusive in spatial signature:
//   BARS:  count_transitions(row0) ≈ 7  (uniform-uniform-... 7 jumps)
//   HBAND: count_transitions(row0) ≈ 0  (no jumps within a row)
//
// Returns 0 ok, negative on error.
extern "C" int sentai_cam_test_pattern(int cam_id, int mode) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -2;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = true;
  uint8_t v = 0x00;
  if      (mode == 0) v = 0x00;  // disable
  else if (mode == 1) v = 0x80;  // bit 7 enable, bit[3:2]=00 → BARS
  else if (mode == 2) v = 0x88;  // bit 7 enable, bit[3:2]=10 → HBAND
  else return -4;
  ok &= cam->WriteToCam(cam_id, 0x503D, v);
  // Make sure manual AEC/AGC isn't sticky from prior experiments
  // — the test pattern is injected BEFORE AEC, but ISP downstream
  // still reads AEC settings; resetting them to auto avoids
  // unrelated brightness drift between runs.
  if (mode == 0) {
    ok &= cam->WriteToCam(cam_id, 0x3503, 0x00);
  }
  return ok ? 0 : -5;
}

// ISP preset bundle — writes a coordinated set of OV5640 ISP
// registers in one call.  Presets are designed to be drop-in
// replacements for NXP's stock init for specific scene types.
//
// Each preset sets:
//   • gamma LUT (0x5480-0x548F)  — tone-curve reshape
//   • SDE ctrl   (0x5580)        — enable brightness/contrast block
//   • SDE bright (0x5585)        — Y offset (0..255 = -128..+127 signed)
//   • SDE contr  (0x5586)        — Y gain   (0x20=unity, higher=boost)
//   • AEC target (0x3A0F..0x3A1F)— auto-exposure luminance target
//   • gain ceil  (0x3A18/0x3A19) — max AGC gain (0x007C=15.5×, 0x1F0=31×)
//
// Presets:
//   "nxp_stock"    : restore NXP defaults (S-curve gamma, tame AEC)
//   "bright_indoor": mild boost (sRGB gamma, +10% contrast, AEC 0x60)
//   "daylight"     : outdoor / well-lit (sRGB gamma, AEC 0x78, std gain)
//   "low_light"    : dim indoor (strong gamma, +30% contrast, max gain)
extern "C" int sentai_cam_isp_preset(int cam_id, const char* name) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -2;
  if (!name) return -3;
  auto* cam = coralmicro::CameraTask::GetSingleton();

  // Gamma LUTs indexed by preset.  Values are Y outputs at OV5640
  // internal x-points (8, 16, 32, 64, 96, 128, 160, 192, 208, 224,
  // 240, 248, 252, 254) plus the slope byte at 0x548F.
  // NXP stock is an S-curve (contrast-leaning); sRGB γ=2.2 is a
  // convex shadow-lift curve; "strong" is γ≈2.8 for very dim scenes.
  static const uint8_t k_gamma_nxp[15] = {
    0x08, 0x14, 0x28, 0x51, 0x65, 0x71, 0x7d, 0x87,
    0x91, 0x9a, 0xaa, 0xb8, 0xcd, 0xdd, 0xea };
  static const uint8_t k_gamma_srgb[15] = {
    0x35, 0x49, 0x63, 0x88, 0xa4, 0xba, 0xce, 0xe0,
    0xe8, 0xf0, 0xf8, 0xfc, 0xfe, 0xfe, 0xfe };
  static const uint8_t k_gamma_strong[15] = {
    0x50, 0x6a, 0x85, 0xa5, 0xbb, 0xcc, 0xda, 0xe6,
    0xec, 0xf2, 0xf8, 0xfc, 0xfe, 0xfe, 0xfe };

  const uint8_t* gamma = k_gamma_nxp;
  uint8_t sde_ctrl  = 0x02;     // NXP default (only base SDE enable)
  uint8_t sde_bright = 0x00;    // no bias
  uint8_t sde_contr  = 0x20;    // unity gain
  uint8_t aec_high  = 0x30, aec_low = 0x28;   // NXP stock
  uint16_t gain_ceiling = 0x007C;              // 15.5×

  if (strcmp(name, "nxp_stock") == 0) {
    // leave defaults
  } else if (strcmp(name, "bright_indoor") == 0) {
    gamma = k_gamma_srgb;
    sde_ctrl  = 0x06;    // enable brightness + contrast blocks
    sde_bright = 0x08;
    sde_contr  = 0x26;   // +20%
    aec_high = 0x60; aec_low = 0x50;
    gain_ceiling = 0x00F8;   // 31×
  } else if (strcmp(name, "daylight") == 0) {
    gamma = k_gamma_srgb;
    sde_ctrl  = 0x06;
    sde_bright = 0x00;
    sde_contr  = 0x24;
    aec_high = 0x78; aec_low = 0x68;
    gain_ceiling = 0x00F8;
  } else if (strcmp(name, "low_light") == 0) {
    gamma = k_gamma_strong;
    sde_ctrl  = 0x06;
    sde_bright = 0x10;
    sde_contr  = 0x2A;   // +30%
    aec_high = 0x60; aec_low = 0x50;   // AEC same as bright_indoor;
                                       // the work is done by gain+gamma
    gain_ceiling = 0x01F0;   // 62.9×
  } else {
    return -4;  // unknown preset
  }

  bool ok = true;

  // Group-write sequence per OV5640 Application Notes: all register
  // writes between "start group" and "end group" are buffered in an
  // internal staging area and applied atomically at the next VSYNC
  // by the "launch" command.  Without this, individual writes take
  // effect on different VSYNC boundaries and the image may show a
  // partial preset for one or more frames (which is what was
  // observed in E38: barely-visible preset differences because AEC
  // was already compensating mid-apply).  Protocol:
  //   0x3212 = 0x03 → enter group 3 (start buffering)
  //   <all register writes>
  //   0x3212 = 0x13 → exit group 3 (stop buffering)
  //   0x3212 = 0xA3 → launch group 3 at next VSYNC
  ok &= cam->WriteToCam(cam_id, 0x3212, 0x03);

  // Gamma curve — 15 bytes from 0x5481 to 0x548F.
  for (int i = 0; i < 15; ++i) {
    ok &= cam->WriteToCam(cam_id, 0x5481 + i, gamma[i]);
  }
  // SDE
  ok &= cam->WriteToCam(cam_id, 0x5580, sde_ctrl);
  ok &= cam->WriteToCam(cam_id, 0x5585, sde_bright);
  ok &= cam->WriteToCam(cam_id, 0x5586, sde_contr);
  // AEC target + fast zone (match the shape sentai_cam_aec_set uses)
  ok &= cam->WriteToCam(cam_id, 0x3A0F, aec_high);
  ok &= cam->WriteToCam(cam_id, 0x3A10, aec_low);
  ok &= cam->WriteToCam(cam_id, 0x3A1B, aec_high);
  ok &= cam->WriteToCam(cam_id, 0x3A1E, aec_low);
  ok &= cam->WriteToCam(cam_id, 0x3A11, (uint8_t)(aec_high + 0x40 > 0xFF
                                                  ? 0xFF : aec_high + 0x40));
  ok &= cam->WriteToCam(cam_id, 0x3A1F, (uint8_t)(aec_low > 0x20
                                                  ? aec_low - 0x20 : 0));
  // Gain ceiling — 10-bit across two regs
  ok &= cam->WriteToCam(cam_id, 0x3A18, (uint8_t)((gain_ceiling >> 8) & 0x03));
  ok &= cam->WriteToCam(cam_id, 0x3A19, (uint8_t)(gain_ceiling & 0xFF));

  // Close the group and launch it atomically.
  ok &= cam->WriteToCam(cam_id, 0x3212, 0x13);
  ok &= cam->WriteToCam(cam_id, 0x3212, 0xA3);
  return ok ? 0 : -5;
}

// Raw SCCB register read/write — exposed for runtime debug so we
// can verify that writes stuck after applying a preset.  Returns
// register value (0..255) on read success, -1 on bus failure.
extern "C" int sentai_cam_reg_read(int cam_id, int reg) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -1;
  if (reg < 0 || reg > 0xFFFF) return -1;
  uint8_t v = 0;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (!cam->ReadFromCam(cam_id, (uint16_t)reg, &v)) return -1;
  return (int)v;
}

extern "C" int sentai_cam_reg_write(int cam_id, int reg, int val) {
  if (!g_cam_initialized) return -1;
  if (cam_id != 0 && cam_id != 1) return -2;
  if (reg < 0 || reg > 0xFFFF) return -3;
  if (val < 0 || val > 0xFF)   return -4;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  return cam->WriteToCam(cam_id, (uint16_t)reg, (uint8_t)val) ? 0 : -5;
}

// ===================== Per-module register sets (W24-T22, production) ========
// The two OV5640 modules need different colour registers (EXP-s265: AWB gains,
// gamma, CMX, SDE, LENC).  The sets live in the project registry
// (camera_modules/modules.json) and are exported by camera_modules/make_regs.py
// to the user partition as /cam/cam0.regs and /cam/cam1.regs: text, one
// "ADDR VAL" hex pair per line, '#' comments.  Sensor registers are volatile
// and camera.init() re-initialises the sensors, so the file of each detected
// connector is applied at the end of every init (and on demand with
// sentai.camera.load_regs()).  Writes go in group-hold batches (0x3212: 0x03
// start, 0x13 end, 0xA3 launch at the next VSYNC) of at most 32 registers,
// one frame apart, so no frame carries a half-applied set.
//
// Refused addresses (a register file must never change the capture mode the
// firmware negotiated): system/PLL/reset 0x3000-0x303F, group control 0x3212,
// timing/window 0x3800-0x383F, OTP 0x3D00-0x3D21, format 0x4300, 0x460C,
// MIPI 0x4800-0x483F.  Returns registers written (> 0), 0 if the file does not
// exist, -1 camera off, -2 bad args, -3 read error, -4 refused address,
// -5 SCCB write failed, -6 parse error.
static bool cam_regs_addr_allowed(unsigned a) {
  if (a <= 0x303F) return false;
  if (a == 0x3212) return false;
  if (a >= 0x3800 && a <= 0x383F) return false;
  if (a >= 0x3D00 && a <= 0x3D21) return false;
  if (a == 0x4300 || a == 0x460C) return false;
  if (a >= 0x4800 && a <= 0x483F) return false;
  return a <= 0xFFFF;
}
extern "C" int sentai_cam_apply_regs_file(int cam_id, const char* path) {
  if (!g_cam_initialized) return -1;
  if ((cam_id != 0 && cam_id != 1) || !path || !path[0]) return -2;
  if (!FxUserFileExists(path)) return 0;
  static uint8_t s_regs_buf[4096] __attribute__((section(".sdram_bss")));   // 109 regs ~ 1 KB
  size_t n = FxUserReadFile(path, s_regs_buf, sizeof(s_regs_buf) - 1);
  if (n == 0 || n >= sizeof(s_regs_buf) - 1) return -3;
  s_regs_buf[n] = 0;
  // Pass 1: parse + validate everything before touching the sensor.
  uint16_t addrs[256]; uint8_t vals[256]; int count = 0;
  const char* p = (const char*)s_regs_buf;
  while (*p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (!*p) break;
    if (*p == '#') { while (*p && *p != '\n') ++p; continue; }
    char* e1 = nullptr; unsigned long a = strtoul(p, &e1, 16);
    if (e1 == p) return -6;
    char* e2 = nullptr; unsigned long v = strtoul(e1, &e2, 16);
    if (e2 == e1 || v > 0xFFul) return -6;
    if (!cam_regs_addr_allowed((unsigned)a)) return -4;
    if (count >= 256) return -6;
    addrs[count] = (uint16_t)a; vals[count] = (uint8_t)v; ++count;
    p = e2;
    while (*p && *p != '\n') ++p;   // ignore trailing comment
  }
  if (count == 0) return 0;
  // Pass 2: direct SCCB writes in file order.  Group-hold batches (0x3212
  // 0x03 / 0x13 / 0xA3, 32 registers per group) were tried first: every
  // register READ BACK correctly, yet the measured colour error of the same
  // 109-register set was dE76 21.8 / 35.7 against 15.2 / 21.9 when the file
  // was written directly (camera_modules/out/measure_prod_*; the OV5640 group
  // write does not take every ISP register of this set).  The set is applied
  // at init, before any consumer, so a frame carrying a half-applied set does
  // not matter.
  auto* cam = coralmicro::CameraTask::GetSingleton();
  for (int k = 0; k < count; ++k) {
    if (!cam->WriteToCam(cam_id, addrs[k], vals[k])) return -5;
  }
  return count;
}
// Called at the end of camera init for every detected connector.
static void cam_apply_module_regs(coralmicro::CameraTask* cam) {
  for (int c = 0; c < 2; ++c) {
    if (!(cam->DetectedMask() & (1u << c))) continue;
    char path[20];
    snprintf(path, sizeof(path), "/cam/cam%d.regs", c);
    int rc = sentai_cam_apply_regs_file(c, path);
    if (rc > 0) {
      SERR_LOG(SERR_CAM_REGS_APPLIED, ((uint32_t)c << 16) | (uint32_t)rc);
      printf("[cam] cam%d: %d registers applied from %s\r\n", c, rc, path);
    } else if (rc < 0) {
      SERR_LOG(SERR_CAM_REGS_FAIL, ((uint32_t)c << 16) | (uint32_t)(-rc));
      printf("[cam] cam%d: register set %s NOT applied (%d)\r\n", c, path, rc);
    }
  }
}

// ───────────────────────────────────────────────────────────────────
// EXP-s245 iter2 (TD-S10-A10) — pin-routing probe.
//
// Question: is Gpio::kCameraTrigger (GPIO8_IO27, pad GPIO_EMC_B2_17)
// physically routed to anything on the SentAI board?  The name is
// inherited from upstream coralmicro (single-camera Dev Board Micro)
// and NO code in this firmware ever drives it — see the audit in
// experiments/s245_camera_trigger_hw_probe/README.md.
//
// Method — weak-pull flip test.  Configure the pad as an input and
// read it under the internal pull-up, then under the internal
// pull-down.  A pad with nothing on it follows whichever pull is
// applied (1 then 0).  A pad tied to a rail, or driven by another
// device, ignores the ~50 kΩ internal pull and reads the same both
// times.
//
// KNOWN LIMITATION (stated up front so the result is not overread):
// a high-impedance INPUT on the far end — which is exactly what an
// OV5640 FREX trigger input would be — also follows the internal
// pull.  So "follows the pulls" means FLOATING-OR-HIGH-Z-LOAD and
// canNOT by itself prove the trace is absent.  Only the TIED/DRIVEN
// outcome is conclusive.  A conclusive positive needs the functional
// test (enable the sensor FREX pad, toggle, observe the sensor).
//
// Methodology control: the same measurement is run on
// Gpio::kEdgeTpuPgood (GPIO8_IO26, pad GPIO_EMC_B2_16) — the
// physically ADJACENT pad in the same bank, known to be routed to the
// EdgeTPU power-good signal.  If the control reads as driven while
// the pin under test follows the pulls, the method is working and the
// difference is real rather than an artefact of the test itself.
//
// SDRAM safety: GPIO_EMC_B2_17's alternate function is SEMC_DM03,
// data mask 3 of a 32-bit SDRAM port.  This board runs the SDRAM at
// kSEMC_PortSize16Bit (board_hardware.c:76), so DM03 is unused and
// re-muxing the pad cannot disturb memory.  We verify that claim
// rather than trusting it: a pattern is written to an SDRAM buffer
// before the probe and checked after.
//
// Restores the boot pad configuration (output, logic 0) on every exit
// path.
//
// out[] layout, 5 entries per pin, pin 0 = kCameraTrigger (under
// test), pin 1 = kEdgeTpuPgood (control):
//   [0] read as plain input, no pull
//   [1] read with internal pull-up
//   [2] read with internal pull-down
//   [3] cycles for the pad to reach logic 1 after being driven low
//       and released to the pull-up (capacitance hint; -1 = never
//       reached within the bound)
//   [4] 1 if the pin followed both pulls (floating-or-high-Z),
//       0 if it ignored them (tied/driven)
// out[10] = SDRAM integrity, 0 = pattern intact.
// Returns the number of entries written, or negative on error.
// ───────────────────────────────────────────────────────────────────
// park_input (EXP-s245 iter3): when non-zero, kCameraTrigger is left
// configured as an INPUT with a pull-down instead of being restored to
// its boot state (output driving 0).
//
// This exists for the strobe-reversal test.  iter3 makes the OV5640
// drive its STROBE pad and re-runs this probe: if GPIO8_IO27 stops
// following the internal pulls, the pad IS routed to the sensor.  But
// an MCU output driving 0 against a sensor output driving 1 is a short
// through both drivers — so the pin MUST be parked as an input for the
// whole window in which any sensor output pad is enabled.
extern "C" int sentai_pin_probe_cam_trigger(int32_t* out, int n,
                                            int park_input) {
  if (!out || n < 11) return -1;
  for (int i = 0; i < n; ++i) out[i] = -1;

  // SDRAM canary.  Region rationale (section-decision): this must live
  // in SDRAM — it is checking SDRAM — but it deliberately does NOT go
  // in .sdram_bss.  Adding a static buffer there shifts the FileX /
  // LevelX state and silently breaks NAND I/O (see the warning above
  // g_slot_arena and project_filex_layout_fragility.md).  malloc from
  // m_heap is the sanctioned alternative and is SDRAM-backed:
  // m_heap ORIGIN = 0x80000000 in MIMXRT1176xxxxx_cm7_ram_mp.ld.
  // Cold path, freed on every exit, so it costs nothing when unused.
  //
  // Scope honesty: this samples 1 KB of a 64 MB SDRAM — a sanity check
  // that the pad re-mux did not disturb memory, not a proof that no
  // byte anywhere moved.
  const int kCanaryWords = 256;
  uint32_t* canary = (uint32_t*)malloc(kCanaryWords * sizeof(uint32_t));
  if (!canary) return -2;
  if (((uintptr_t)canary >> 24) != 0x80u) {   // must be SDRAM to be meaningful
    free(canary);
    return -3;
  }
  for (int i = 0; i < kCanaryWords; ++i) canary[i] = 0xA5A50000u + (uint32_t)i;
  __DSB();

  const coralmicro::Gpio pins[2] = { coralmicro::Gpio::kCameraTrigger,
                                     coralmicro::Gpio::kEdgeTpuPgood };

  for (int p = 0; p < 2; ++p) {
    const coralmicro::Gpio g = pins[p];
    int32_t* o = &out[p * 5];

    // Settle time for a ~50 kΩ internal pull against pad + trace
    // capacitance.  1 tick (1 ms) is ~1000× the RC of even a long
    // board trace, so the reading is the steady state, not a
    // transient.
    coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInput);
    vTaskDelay(pdMS_TO_TICKS(2));
    o[0] = coralmicro::GpioGet(g) ? 1 : 0;

    coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInputPullUp);
    vTaskDelay(pdMS_TO_TICKS(2));
    o[1] = coralmicro::GpioGet(g) ? 1 : 0;

    coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInputPullDown);
    vTaskDelay(pdMS_TO_TICKS(2));
    o[2] = coralmicro::GpioGet(g) ? 1 : 0;

    // Followed both pulls → floating or a high-Z load on the far end.
    o[4] = (o[1] == 1 && o[2] == 0) ? 1 : 0;

    // Capacitance hint: drive the pad low, release it to the internal
    // pull-up, and count DWT cycles until it reads high.  More
    // capacitance (a real trace with a device pin on the end) charges
    // more slowly, so this separates a bare pad from a routed one.
    //
    // DRIVE SAFETY: only run this when the pull test just showed the
    // pad follows the internal pulls.  If anything on the board were
    // actively driving the net, turning our pad into an output would
    // put two push-pull drivers in contention — a direct short
    // through both.  kEdgeTpuPgood is the TPU PMIC's power-good
    // OUTPUT, so the control pin must never be driven; the gate below
    // excludes it for free whenever the PMIC is holding the line.
    o[3] = -1;
    if (o[4] == 1) {
      coralmicro::GpioSetMode(g, coralmicro::GpioMode::kOutput);
      coralmicro::GpioSet(g, false);
      vTaskDelay(pdMS_TO_TICKS(2));
      // Bounded loop (NASA §2): 8e6 cycles ≈ 10 ms at 800 MHz, far
      // beyond any RC on this board.
      const uint32_t kMaxCycles = 8000000u;
      const uint32_t t0 = DWT->CYCCNT;
      coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInputPullUp);
      int32_t measured = -1;
      for (;;) {
        const uint32_t dt = DWT->CYCCNT - t0;
        if (coralmicro::GpioGet(g)) { measured = (int32_t)dt; break; }
        if (dt > kMaxCycles) break;         // never reached 1
      }
      o[3] = measured;
    }

    // Restore.  kEdgeTpuPgood boots as an input and must stay one — it
    // is the TPU PMIC's output and we must never drive it.
    // kCameraTrigger boots as an output driving 0, unless the caller
    // asked for it to be parked as a safe high-impedance input.
    if (g == coralmicro::Gpio::kEdgeTpuPgood) {
      coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInput);
    } else if (park_input) {
      coralmicro::GpioSetMode(g, coralmicro::GpioMode::kInputPullDown);
    } else {
      coralmicro::GpioSetMode(g, coralmicro::GpioMode::kOutput);
      coralmicro::GpioSet(g, false);
    }
  }

  int bad = 0;
  for (int i = 0; i < kCanaryWords; ++i) {
    if (canary[i] != 0xA5A50000u + (uint32_t)i) bad++;
  }
  free(canary);
  out[10] = bad;
  return 11;
}

// Rotate camera image. cam_id: 0=front, 1=back. degrees: 0, 90, 180, 270.
extern "C" int sentai_cam_rotate(int cam_id, int degrees) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = cam->SetCameraRotation(cam_id, degrees);
  return ok ? 0 : -2;
}

// SOURCE geometry of the frame buffers (W24-T20): what every reader of a raw
// frame must use as width / height.  Until build 1760 this returned the
// set_resolution() output size, which several paths used as the source pitch.
extern "C" int sentai_cam_get_width(void)  { return cam_src_w(); }
extern "C" int sentai_cam_get_height(void) { return cam_src_h(); }

// OUTPUT size chosen with camera.set_resolution() (defaults to the sensor
// geometry; camera.jpeg() / save_jpeg() / resolution() use it).
extern "C" int sentai_cam_get_out_width(void) {
  if (sentai_virtual_camera_active()) return sentai_virtual_camera_width();
  return g_cam_width;
}

extern "C" int sentai_cam_get_out_height(void) {
  if (sentai_virtual_camera_active()) return sentai_virtual_camera_height();
  return g_cam_height;
}

extern "C" uint32_t sentai_cam_get_frame_seq(void) {
  if (sentai_virtual_camera_active()) return sentai_virtual_camera_seq();
  return g_camera_frame_seq;
}

extern "C" uint32_t sentai_cam_get_sensor_frames(void) {
  return g_camera_sensor_frames;
}

// camera.native_res(): the sensor geometry selected with set_hw() (640x480 or
// 1280x960), not the compile-time CameraTask::kWidth/kHeight (W24-T20).
extern "C" int sentai_cam_get_native_width(void)  { return cam_src_w(); }
extern "C" int sentai_cam_get_native_height(void) { return cam_src_h(); }

// ===================== AIfES sensor capture functions =====================

// Capture camera frame for AIfES, resize to w×h, output as RGB or grayscale
// Returns: bytes written, or negative on error
extern "C" int sentai_aifes_capture_camera(uint8_t* out, int w, int h, int grayscale) {
  if (!g_cam_initialized && !sentai_virtual_camera_active()) return -1;
  if (!out || w <= 0 || h <= 0) return -3;
  if (cam_src_packed()) return -7;   // frame is not XRGB (W24-T20)
  const int src_w = cam_src_w(), src_h = cam_src_h();   // runtime geometry
  if (w > src_w || h > src_h) return -5;
  
  // Capture raw frame
  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  
  if (grayscale) {
    // For grayscale: capture RGB, then convert
    // Use temporary RGB buffer
    std::vector<uint8_t> rgb_buf(w * h * 3);
    int rc = pxp_scale_xrgb_to_rgb(raw, src_w, src_h,
                                    rgb_buf.data(), w, h);
    sentai_cam_return_raw(idx);
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
    int rc = pxp_scale_xrgb_to_rgb(raw, src_w, src_h,
                                    out, w, h);
    sentai_cam_return_raw(idx);
    if (rc != 0) return rc;
    return w * h * 3;
  }
}
// [end-sphinx-snippet:detect-image]
