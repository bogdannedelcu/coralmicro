/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * FileX/LevelX user-partition filesystem implementation.  See
 * fx_user_fs.h for the public API and the layering diagram.
 *
 * Per embeded.md NASA/JPL discipline:
 *   - All loops are explicitly bounded (path-walks, listing, retries).
 *   - Static allocation only; no heap in steady-state.
 *   - Every fx_/lx_ return is checked.  Failures are surfaced to the
 *     caller through the documented bool/ssize_t contracts AND raise
 *     a structured error log via SERR_LFX_*.
 *   - Single FreeRTOS mutex serialises every public entry point so
 *     callers do not need their own locking.  This mirrors the
 *     existing LittleFS contract via g_lfs_user_mutex.
 */

#include "libs/base/fx_user_fs.h"
#include "libs/base/fx_nand_driver.h"
#include "libs/base/filesystem.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" {
#include "fx_api.h"
#include "lx_api.h"
}

#include "examples/sentai_runtime/sentai_error.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" void sentai_repl_activity(void);

namespace {

/* ===== Static memory =================================================
 *
 * All buffers live in .sdram_bss.  Embeded.md §D: static allocation
 * only; no heap in steady state.  Total SDRAM cost ~48 KB.
 */
constexpr size_t kLxMemorySize  = 32u * 1024u;
constexpr size_t kFxMediaSize   = 16u * 1024u;
constexpr size_t kPathScratch   = 256u;

ULONG g_lx_memory_buffer[kLxMemorySize / sizeof(ULONG)]
    __attribute__((aligned(8), section(".sdram_bss")));
UCHAR g_fx_media_memory[kFxMediaSize]
    __attribute__((aligned(8), section(".sdram_bss")));

LX_NAND_FLASH g_lx_nand   __attribute__((section(".sdram_bss")));
FX_MEDIA      g_fx_media  __attribute__((section(".sdram_bss")));

/* ===== State ========================================================== */
SemaphoreHandle_t g_fx_mutex = nullptr;
volatile bool     g_mounted  = false;
bool              g_systems_initialized = false;
uint32_t          g_format_count = 0;
uint32_t          g_mount_failures = 0;

/* ===== Idle-driven auto-sync (Pas 2 brownout protection) =============
 *
 * Phase 3.2 (build #1074) dropped per-write fx_media_flush for 41-139×
 * small-write speed-up — but writes then live ONLY in the FileX/LevelX
 * SDRAM caches until an explicit FxUserSync().  If the operator pulls
 * USB or the board switches USB→battery (Crazyflie BL deck VBAT ~3.7V
 * vs board's 5V need) before sync, the last write's NAND program may
 * be in-flight when brownout hits, leaving an ECC-corrupted page that
 * fails the next mount.
 *
 * Fix (build #1226+): track unflushed writes + last-write timestamp.
 * The hardware-watchdog task (CombinedWatchdogTask) calls
 * FxUserMaybeIdleSync() once per 5 s tick.  If unflushed > 0 AND no
 * write happened in the last kIdleSyncMs, it forces a sync.
 *
 * Net effect: max window of unflushed data shrinks from "indefinite"
 * to ~7 s (5 s tick + 2 s idle gate) on an idle board, with zero
 * impact on hot-path write throughput (sync only fires when activity
 * has actually stopped).  Critical writes that need stronger
 * guarantees (/main.py uploads, configs) still call sentai.fs.sync()
 * explicitly per agent.md rule #11. */
volatile uint32_t g_unflushed_writes = 0;
volatile uint32_t g_last_write_ms    = 0;
constexpr uint32_t kIdleSyncMs       = 2000u;

static inline void note_write(void) {
    g_unflushed_writes++;
    g_last_write_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ===== Mount retry policy (embeded.md §F: bounded local retry) =====
 *
 * 3 attempts × 50 ms inter-attempt delay = 150 ms worst-case extra
 * boot latency.  Tolerates a single transient NAND ECC blip after a
 * brownout reset without escalating to data-destroying format.
 *
 * If retries are exhausted AND the volume is NOT virgin (LevelX
 * reports a recognisable header), we enter SAFE MODE: g_mounted
 * stays false, all FxUser* APIs return clean errors, but REPL +
 * radio remain alive so the operator can diagnose and explicitly
 * recover via `sentai.diag.fx_format(FX_DESTRUCTIVE_CONFIRM_MAGIC)`.
 *
 * Auto-format runs ONLY when LevelX reports a virgin-NAND signature
 * (LX_SYSTEM_INVALID_FORMAT or LX_NO_PAGES) — first-boot bootstrap.
 * Any other failure code is treated as corruption-of-real-data and
 * preserved for forensic analysis. */
constexpr uint32_t kMountRetries       = 3u;
constexpr uint32_t kMountRetryDelayMs  = 50u;

/* Pre-scheduler-safe delay.  vTaskDelay() requires the scheduler to be
 * running (it manipulates pxCurrentTCB which is NULL pre-scheduler →
 * null deref → HardFault → boot loop).  FxUserInit and FxUserOpenLxOnly
 * are called from CHECK(LfsUserInit()) in main_freertos_m7.cc:389 BEFORE
 * vTaskStartScheduler — so a vTaskDelay(50) inside the bounded retry
 * loop would brick boot the moment any retry was needed.  Discovered
 * 2026-05-09 (see experiment.md "fx_user_fs vTaskDelay pre-scheduler
 * boot brick" session): board ran for 985 ms post-reset then halted at
 * progress=0x06 with 14 tasks created but app_main task never scheduled
 * (xTaskCreate failed silently because heap was clobbered by the fault).
 *
 * Post-scheduler: vTaskDelay (yields CPU, accurate ms timing).
 * Pre-scheduler: cycle-counted busy spin (CPU @ 800 MHz × ms × 1000). */
inline void bounded_delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    /* Busy-spin pre-scheduler.  800 MHz core; one volatile-load per
     * iteration is roughly 4-8 cycles, so 800e6 cycles/sec ÷ ~6 cyc =
     * ~130 M iters/sec.  Cap to be safe. */
    volatile uint32_t i;
    const uint32_t kIters = ms * 100000u; /* generous; busy-spin > vTaskDelay tick anyway */
    for (i = 0; i < kIters; ++i) { __asm__ volatile("nop"); }
}

/* Boot timestamp seed for best-effort mtime.  FAT timestamps require
 * year >= 1980; we substitute (boot tick) seconds and tag the year as
 * a constant so the field is monotonic-ish for the same boot. */
inline uint32_t now_ms() {
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ===== Bridge driver: FX_MEDIA <-> LX_NAND_FLASH ======================
 *
 * Modeled after fx_nand_flash_simulated_driver.c upstream but rewired
 * to OUR LX_NAND_FLASH instance and our hardware.  LX is opened/closed
 * by the caller (FxUserInit / FxUserUnmount) so the DRIVER_INIT and
 * DRIVER_UNINIT cases here are no-ops. */
extern "C" void fx_user_nand_driver(FX_MEDIA* media_ptr) {
    ULONG  logical_sector;
    ULONG  count;
    UCHAR* buffer;
    UINT   status;

    switch (media_ptr->fx_media_driver_request) {
        case FX_DRIVER_READ:
            logical_sector = media_ptr->fx_media_driver_logical_sector;
            count = media_ptr->fx_media_driver_sectors;
            buffer = static_cast<UCHAR*>(media_ptr->fx_media_driver_buffer);
            for (ULONG remaining = count; remaining != 0; --remaining) {
                status = _lx_nand_flash_sector_read(&g_lx_nand,
                                                    logical_sector, buffer);
                if (status != LX_SUCCESS) {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;
                    return;
                }
                ++logical_sector;
                buffer += media_ptr->fx_media_bytes_per_sector;
            }
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;

        case FX_DRIVER_WRITE:
            logical_sector = media_ptr->fx_media_driver_logical_sector;
            count = media_ptr->fx_media_driver_sectors;
            buffer = static_cast<UCHAR*>(media_ptr->fx_media_driver_buffer);
            for (ULONG remaining = count; remaining != 0; --remaining) {
                status = _lx_nand_flash_sector_write(&g_lx_nand,
                                                     logical_sector, buffer);
                if (status != LX_SUCCESS) {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;
                    return;
                }
                ++logical_sector;
                buffer += media_ptr->fx_media_bytes_per_sector;
            }
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;

        case FX_DRIVER_RELEASE_SECTORS:
            logical_sector = media_ptr->fx_media_driver_logical_sector;
            count = media_ptr->fx_media_driver_sectors;
            for (ULONG remaining = count; remaining != 0; --remaining) {
                status = _lx_nand_flash_sector_release(&g_lx_nand,
                                                       logical_sector);
                if (status != LX_SUCCESS) {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;
                    return;
                }
                ++logical_sector;
            }
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;

        case FX_DRIVER_FLUSH:
        case FX_DRIVER_ABORT:
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;

        case FX_DRIVER_INIT:
            media_ptr->fx_media_driver_free_sector_update = FX_TRUE;
            media_ptr->fx_media_driver_status             = FX_SUCCESS;
            break;

        case FX_DRIVER_UNINIT:
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;

        case FX_DRIVER_BOOT_READ:
            status = _lx_nand_flash_sector_read(&g_lx_nand, 0,
                static_cast<UCHAR*>(media_ptr->fx_media_driver_buffer));
            media_ptr->fx_media_driver_status =
                (status == LX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
            break;

        case FX_DRIVER_BOOT_WRITE:
            status = _lx_nand_flash_sector_write(&g_lx_nand, 0,
                static_cast<UCHAR*>(media_ptr->fx_media_driver_buffer));
            media_ptr->fx_media_driver_status =
                (status == LX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
            break;

        default:
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
            break;
    }
}

/* ===== Mutex helpers ================================================ */

/* Bounded mutex acquisition.  Default timeout 2 s — well under the
 * USB MSC and HTTP NCM watchdog ceilings, and short enough that a
 * truly wedged owner is detected quickly.  Embeded.md §B: every
 * timing-sensitive path must have explicit budget awareness.
 *
 * On timeout: SERR_LOG SERR_LFX_LOCK_TIMEOUT(timeout_ms) so post-mortem
 * logs show contention without needing the caller to add tracing. */
inline bool lock(uint32_t timeout_ms = 2000u) {
    if (g_fx_mutex == nullptr) return false;
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(g_fx_mutex, ticks) == pdTRUE) return true;
    SERR_LOG(SERR_LFX_LOCK_TIMEOUT, timeout_ms);
    return false;
}
inline void unlock() {
    if (g_fx_mutex != nullptr) xSemaphoreGive(g_fx_mutex);
}

/* RAII guard for the API-entry mutex.  `held` is exposed so callers
 * can early-out cleanly on lock failure without the silent-zero
 * conflation between "lock failed" and "operation succeeded with
 * empty result" (embeded.md §F: do not silently swallow faults). */
struct LockGuard {
    bool held;
    LockGuard() : held(lock()) {}
    ~LockGuard() { if (held) unlock(); }
};

/* ===== Internal: format the volume (LX wipe + FAT format) ============
 *
 * Caller MUST hold g_fx_mutex.  On entry, both LX and FX may be in any
 * state — we close everything first.  On success the volume is left
 * MOUNTED (fx_media_open done). */
bool format_and_mount() {
    /* Close anything that may already be open.  These are no-ops if the
     * media is not in the matching state; ignore returns. */
    (void)fx_media_close(&g_fx_media);
    (void)_lx_nand_flash_close(&g_lx_nand);
    sentai_repl_activity();

    /* Wipe + write LX metadata. */
    uint32_t t0 = now_ms();
    UINT lx = lx_nand_flash_format(&g_lx_nand, (CHAR*)"sentai_user",
                                   fx_nand_driver_initialize,
                                   g_lx_memory_buffer,
                                   sizeof(g_lx_memory_buffer));
    if (lx != LX_SUCCESS) {
        printf("[fx_user] lx_format failed: %u\r\n", (unsigned)lx);
        SERR_LOG(SERR_LFX_LX_FORMAT, lx);
        return false;
    }
    printf("[fx_user] lx_format ok in %u ms\r\n", (unsigned)(now_ms() - t0));
    sentai_repl_activity();

    /* Re-open LX; format closes. */
    lx = lx_nand_flash_open(&g_lx_nand, (CHAR*)"sentai_user",
                            fx_nand_driver_initialize,
                            g_lx_memory_buffer,
                            sizeof(g_lx_memory_buffer));
    if (lx != LX_SUCCESS) {
        printf("[fx_user] lx_open after format failed: %u\r\n", (unsigned)lx);
        SERR_LOG(SERR_LFX_LX_OPEN, lx);
        return false;
    }

    /* Lay out the FAT. Sector size = data per page (2048, power-of-2 so
     * Linux usb-storage accepts the MSC LUN).  Cluster of 4 sectors
     * (8 KB) cuts FAT updates by 4× for typical write patterns —
     * Phase 3 perf tuning showed each fx_file_write hit ~2.7 s of
     * fixed overhead (FAT + dir entry + flush), independent of size,
     * so larger clusters amortise that cost across more bytes.
     * Tradeoff: minimum file size on disk grows from 2 KB to 8 KB. */
    ULONG total_sectors = (FX_NAND_USER_BLOCK_COUNT - 8u) *
                          FX_NAND_PAGES_PER_BLOCK;
    UINT fx = fx_media_format(&g_fx_media,
                              fx_user_nand_driver,
                              FX_NULL,
                              g_fx_media_memory,
                              sizeof(g_fx_media_memory),
                               (CHAR*)"SENTAI_USER",
                               1,                       /* number of FATs   */
                               256,                     /* root dir entries */
                               0,                       /* hidden sectors   */
                               total_sectors,
                               FX_NAND_BYTES_PER_PAGE,  /* sector size      */
                               1,                       /* sectors/cluster (Phase 3.1 reverted: cluster=4 regressed large-file writes) */
                               1,                       /* heads            */
                               1);                      /* sectors/track    */
    if (fx != FX_SUCCESS) {
        printf("[fx_user] fx_media_format failed: %u\r\n", (unsigned)fx);
        SERR_LOG(SERR_LFX_FX_FORMAT, fx);
        (void)_lx_nand_flash_close(&g_lx_nand);
        return false;
    }
    printf("[fx_user] fx_media_format ok\r\n");
    g_format_count++;
    sentai_repl_activity();

    /* Open the freshly-formatted FAT. */
    fx = fx_media_open(&g_fx_media, (CHAR*)"sentai_user",
                       fx_user_nand_driver, FX_NULL,
                       g_fx_media_memory, sizeof(g_fx_media_memory));
    if (fx != FX_SUCCESS) {
        printf("[fx_user] fx_media_open after format failed: %u\r\n",
               (unsigned)fx);
        SERR_LOG(SERR_LFX_FX_OPEN, fx);
        (void)_lx_nand_flash_close(&g_lx_nand);
        return false;
    }
    g_mounted = true;
    printf("[fx_user] mounted (post-format)\r\n");
    return true;
}

/* Try to mount an EXISTING volume.  Returns LX_SUCCESS only when both
 * LX and FX opened cleanly.  On failure, *fx_out reports the FileX
 * status (FX_NOT_OPEN if the LX layer never opened, otherwise the
 * actual fx_media_open return).  This split lets the caller
 * distinguish three cases:
 *   1. LX != LX_SUCCESS              -> LevelX layer broken (or virgin)
 *   2. LX == LX_SUCCESS, FX != OK    -> FAT layer broken (LX is fine)
 *   3. LX == LX_SUCCESS, FX == OK    -> mounted ok (g_mounted = true)
 *
 * Caller (FxUserInit) uses (1) to decide first-boot bootstrap vs
 * SAFE MODE, and treats (2) as SAFE MODE (FAT corruption alone
 * should NOT be papered over with a format that wipes LX too). */
UINT try_mount_existing(UINT* fx_out) {
    if (fx_out != nullptr) *fx_out = FX_NOT_OPEN;
    UINT lx = lx_nand_flash_open(&g_lx_nand, (CHAR*)"sentai_user",
                                 fx_nand_driver_initialize,
                                 g_lx_memory_buffer,
                                 sizeof(g_lx_memory_buffer));
    if (lx != LX_SUCCESS) {
        return lx;
    }
    UINT fx = fx_media_open(&g_fx_media, (CHAR*)"sentai_user",
                            fx_user_nand_driver, FX_NULL,
                            g_fx_media_memory, sizeof(g_fx_media_memory));
    if (fx_out != nullptr) *fx_out = fx;
    if (fx != FX_SUCCESS) {
        (void)_lx_nand_flash_close(&g_lx_nand);
        return LX_SUCCESS;  /* LX side is clean; caller inspects *fx_out */
    }
    g_mounted = true;
    return LX_SUCCESS;
}

/* True if `lx_status` indicates the LevelX header is missing or the
 * partition is uninitialised — i.e. the only states where auto-format
 * cannot destroy real user data.  Any other non-success code is
 * treated as POTENTIAL corruption of valid data and triggers SAFE
 * MODE, never silent format. */
inline bool is_virgin_nand_signature(UINT lx_status) {
    return lx_status == LX_SYSTEM_INVALID_FORMAT ||
           lx_status == LX_NO_PAGES;
}

/* FileX FAT accepts both '/' and '\\' as separators; we keep the
 * existing LFS-style "/dir/file" paths verbatim in callers.
 *
 * (path_basename helper removed in Phase 3.5 audit — no live caller.) */

}  /* anonymous namespace */

/* ===== Public API ===================================================== */

extern "C" int FxUserInit(int force_format) {
    if (g_fx_mutex == nullptr) {
        g_fx_mutex = xSemaphoreCreateMutex();
        if (g_fx_mutex == nullptr) {
            printf("[fx_user] mutex create failed\r\n");
            return 0;
        }
    }
    LockGuard guard;
    if (!guard.held) return 0;

    if (!g_systems_initialized) {
        fx_system_initialize();
        lx_nand_flash_initialize();
        g_systems_initialized = true;
    }

    /* If currently mounted, close cleanly before re-init. */
    if (g_mounted) {
        (void)fx_media_close(&g_fx_media);
        (void)_lx_nand_flash_close(&g_lx_nand);
        g_mounted = false;
    }

    /* Operator-confirmed format path — only entered with force_format=1
     * (e.g. `sentai.diag.fx_format(FX_DESTRUCTIVE_CONFIRM_MAGIC)`).
     * Distinct from any auto-format below. */
    if (force_format) {
        printf("[fx_user] force_format requested (operator confirmed)\r\n");
        return format_and_mount() ? 1 : 0;
    }

    /* Bounded retry — embeded.md §F: tolerate transient ECC blips
     * before any escalation.  Each attempt either mounts or returns
     * the LX/FX status from the failure for the next decision step. */
    UINT last_lx = LX_ERROR;
    UINT last_fx = FX_NOT_OPEN;
    for (uint32_t attempt = 0; attempt < kMountRetries; ++attempt) {
        last_lx = try_mount_existing(&last_fx);
        if (last_lx == LX_SUCCESS && last_fx == FX_SUCCESS) {
            if (attempt > 0) {
                /* Recovered after retry — log so post-mortem can correlate
                 * with brownout / RX-noise events. */
                SERR_LOG(SERR_LFX_MOUNT_RETRY_OK, attempt);
                printf("[fx_user] mounted (existing volume) after %u retr%s\r\n",
                       (unsigned)attempt, attempt == 1u ? "y" : "ies");
            } else {
                printf("[fx_user] mounted (existing volume)\r\n");
            }
            sentai_repl_activity();
            return 1;
        }
        if (attempt + 1u < kMountRetries) {
            bounded_delay_ms(kMountRetryDelayMs);
        }
    }

    g_mount_failures++;

    /* Virgin-NAND first-boot bootstrap is the ONLY auto-format path.
     * LevelX returns LX_SYSTEM_INVALID_FORMAT or LX_NO_PAGES when the
     * partition has no recognisable LX header — i.e. nothing to
     * destroy.  Any other failure code (LX_ERROR, LX_BAD_BLOCK,
     * LX_NAND_ERROR_NOT_CORRECTED, ...) implies the volume HAD valid
     * data that's now unreadable; we will NOT silently destroy it. */
    if (is_virgin_nand_signature(last_lx)) {
        SERR_LOG(SERR_LFX_FIRST_BOOT_FORMAT, last_lx);
        printf("[fx_user] virgin NAND (lx=%u) -> first-boot format\r\n",
               (unsigned)last_lx);
        return format_and_mount() ? 1 : 0;
    }

    /* SAFE MODE — embeded.md §F escalation: degraded mode preserves
     * data and keeps REPL alive so the operator can decide.  The
     * board STAYS in communication (anti-brick rule §M satisfied):
     * USB CDC is up, REPL is up, radio bridge auto-init still runs.
     * FxUser* APIs return clean errors on every write/read attempt
     * because g_mounted is false. */
    SERR_LOG(SERR_LFX_MOUNT_FAIL_SAFE, last_lx);
    printf("[fx_user] *** MOUNT FAIL (lx=%u, fx=%u) -- SAFE MODE ***\r\n",
           (unsigned)last_lx, (unsigned)last_fx);
    printf("[fx_user]   FS is read+write disabled to protect existing data.\r\n");
    printf("[fx_user]   REPL + radio remain alive for diagnosis.\r\n");
    printf("[fx_user]   To DESTROY user data and re-format:\r\n");
    printf("[fx_user]     sentai.diag.fx_format(0xDEADBEEF)\r\n");
    g_mounted = false;
    /* SAFE MODE = boot SUCCESS with FS in degraded mode.
     * Returning 0 here propagates to LfsUserInit() -> false ->
     * CHECK(LfsUserInit()) fail in main_freertos_m7.cc:389 ->
     * vTaskSuspendAll() pre-scheduler -> scheduler starts with
     * uxSchedulerSuspended >= 1 -> no user task ever runs ->
     * USB CDC never enumerates -> board APPEARS bricked even though
     * the firmware itself is alive.  That directly contradicts the
     * comment above and the operator-facing convention "FileX corrupt
     * => REPL + USB CDC stay alive so we can fx_format from REPL".
     * g_mounted=false is sufficient to gate every FxUser* API call
     * (each early-returns 0/-1 when not mounted), so write/read
     * semantics remain safe regardless of this return value.
     * Diagnosed via JTAG 2026-05-10 — see experiment.md session for
     * the dmesg trace + uxSchedulerSuspended=2 evidence. */
    return 1;
}

extern "C" int FxUserRemount(void) { return FxUserInit(/*force_format=*/0); }

extern "C" void FxUserUnmount(void) {
    LockGuard guard;
    if (!guard.held) return;
    if (g_mounted) {
        (void)fx_media_close(&g_fx_media);
        (void)_lx_nand_flash_close(&g_lx_nand);
        g_mounted = false;
        printf("[fx_user] unmounted\r\n");
    }
}

extern "C" int FxUserIsMounted(void) { return g_mounted ? 1 : 0; }

extern "C" int FxUserStat(const char* path, FxStat* out) {
    if (out == nullptr) return 0;
    out->exists = 0; out->is_dir = 0; out->size = 0; out->mtime_s = 0;
    if (!g_mounted) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    UINT attr = 0; ULONG sz = 0;
    UINT y, mo, d, h, mi, s;
    UINT fx = fx_directory_information_get(&g_fx_media, (CHAR*)path,
                                           &attr, &sz,
                                           &y, &mo, &d, &h, &mi, &s);
    if (fx != FX_SUCCESS) return 0;
    out->exists = 1;
    out->is_dir = (attr & FX_DIRECTORY) ? 1 : 0;
    out->size   = (uint32_t)sz;
    /* Best-effort mtime: epoch-relative seconds-of-day.  FAT lacks
     * sub-second resolution and our RTC is not battery-backed. */
    out->mtime_s = (h * 3600u) + (mi * 60u) + s;
    return 1;
}

extern "C" int FxUserFileExists(const char* path) {
    FxStat st;
    return (FxUserStat(path, &st) && st.exists && !st.is_dir) ? 1 : 0;
}

extern "C" int FxUserDirExists(const char* path) {
    /* Root always exists. */
    if (path == nullptr) return 0;
    if (path[0] == '/' && path[1] == '\0') return g_mounted ? 1 : 0;
    if (path[0] == '\0') return g_mounted ? 1 : 0;
    FxStat st;
    return (FxUserStat(path, &st) && st.exists && st.is_dir) ? 1 : 0;
}

extern "C" ssize_t FxUserSize(const char* path) {
    FxStat st;
    if (!FxUserStat(path, &st) || !st.exists || st.is_dir) return -1;
    return (ssize_t)st.size;
}

extern "C" size_t FxUserReadFile(const char* path, uint8_t* buf, size_t size) {
    if (!g_mounted || buf == nullptr || size == 0) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    FX_FILE f;
    UINT fx = fx_file_open(&g_fx_media, &f, (CHAR*)path, FX_OPEN_FOR_READ);
    if (fx != FX_SUCCESS) return 0;
    ULONG actual = 0;
    fx = fx_file_read(&f, buf, (ULONG)size, &actual);
    (void)fx_file_close(&f);
    if (fx != FX_SUCCESS && fx != FX_END_OF_FILE) {
        SERR_LOG(SERR_LFX_FILE_READ, fx);
        return 0;
    }
    return (size_t)actual;
}

extern "C" int FxUserWriteFile(const char* path, const uint8_t* buf,
                                size_t size) {
    if (!g_mounted || path == nullptr) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    /* Ensure parent dir exists (-p semantics) — many callers expect
     * write to create the leading dir like LFS did via mkdir loops. */
    {
        const char* slash = std::strrchr(path, '/');
        if (slash != nullptr && slash != path) {
            char parent[kPathScratch];
            size_t plen = (size_t)(slash - path);
            if (plen >= kPathScratch) plen = kPathScratch - 1;
            std::memcpy(parent, path, plen);
            parent[plen] = '\0';
            /* Re-entrant call via the mutex: must drop and re-take to
             * avoid recursive-mutex requirement. */
            unlock();
            FxUserMakeDirs(parent);
            (void)lock();
            /* Note: we let any failure here through silently — the
             * subsequent fx_file_create will surface the real issue. */
        }
    }

    /* Truncate-by-create-then-overwrite semantics: try to delete first
     * (ignore not-found), then create + open + write. */
    (void)fx_file_delete(&g_fx_media, (CHAR*)path);
    UINT fx = fx_file_create(&g_fx_media, (CHAR*)path);
    if (fx != FX_SUCCESS && fx != FX_ALREADY_CREATED) {
        SERR_LOG(SERR_LFX_FILE_CREATE, fx);
        return 0;
    }
    FX_FILE f;
    fx = fx_file_open(&g_fx_media, &f, (CHAR*)path, FX_OPEN_FOR_WRITE);
    if (fx != FX_SUCCESS) {
        SERR_LOG(SERR_LFX_FILE_OPEN, fx);
        return 0;
    }
    if (size > 0) {
        fx = fx_file_write(&f, (VOID*)buf, (ULONG)size);
        if (fx != FX_SUCCESS) {
            SERR_LOG(SERR_LFX_FILE_WRITE, fx);
            (void)fx_file_close(&f);
            return 0;
        }
    }
    (void)fx_file_close(&f);
    /* Phase 3.2: NO per-write fx_media_flush.  fx_file_close already
     * flushes the file's own FAT chain and directory entry; an extra
     * media_flush forces a full FAT-table writeback that costs ~1 s
     * on NAND with sector_size=2048 and is redundant for one-shot
     * small writes.  Callers that need durability should call
     * FxUserSync() explicitly. */
    note_write();
    return 1;
}

extern "C" int FxUserAppendFile(const char* path, const uint8_t* buf,
                                 size_t size) {
    if (!g_mounted || path == nullptr) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    UINT fx = fx_file_create(&g_fx_media, (CHAR*)path);
    if (fx != FX_SUCCESS && fx != FX_ALREADY_CREATED) {
        SERR_LOG(SERR_LFX_FILE_CREATE, fx);
        return 0;
    }
    FX_FILE f;
    fx = fx_file_open(&g_fx_media, &f, (CHAR*)path, FX_OPEN_FOR_WRITE);
    if (fx != FX_SUCCESS) {
        SERR_LOG(SERR_LFX_FILE_OPEN, fx);
        return 0;
    }
    /* Seek to end. */
    ULONG cur_size = 0;
    UINT y, mo, d, h, mi, s; UINT attr;
    if (fx_directory_information_get(&g_fx_media, (CHAR*)path, &attr,
                                     &cur_size, &y, &mo, &d, &h, &mi, &s)
            == FX_SUCCESS) {
        (void)fx_file_seek(&f, cur_size);
    }
    int ok = 1;
    if (size > 0) {
        UINT wfx = fx_file_write(&f, (VOID*)buf, (ULONG)size);
        if (wfx != FX_SUCCESS) {
            SERR_LOG(SERR_LFX_FILE_WRITE, wfx);
            ok = 0;
        }
    }
    (void)fx_file_close(&f);
    /* Phase 3.2: no per-call fx_media_flush — see FxUserWriteFile. */
    if (ok) note_write();
    return ok;
}

/* Public sync.  Flushes BOTH cache layers down to NAND so any pending
 * writes are durable across power loss:
 *
 *   N1+N2 (FileX logical-sector cache + FAT/dir cache)
 *         flushed by fx_media_flush.
 *   N3    (LevelX log-page indirection + wear-level mapping table)
 *         flushed only by _lx_nand_flash_close.  Re-open immediately
 *         so consumers see no mount gap.
 *
 * Without the N3 flush, fx_media_flush returns success but log-page
 * mapping stays in g_lx_memory_buffer (32 KB SDRAM).  Power loss
 * then leaves the FAT table on NAND referencing logical sectors
 * whose physical NAND pages were never actually written, so on next
 * boot try_mount_existing returns garbage (or refuses to mount,
 * which used to trigger silent format — now SAFE MODE per §F).
 *
 * Cost: ~200-500 ms per call (NAND write of LX log + metadata).
 * Callers should invoke sparingly — typically before sys.reset()
 * or after a critical write (e.g. /main.py upload).  Per-write
 * sync is NOT needed and was explicitly removed in Phase 3.2 for
 * 41-139× small-write speed-up. */
extern "C" int FxUserSync(void) {
    if (!g_mounted) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    /* Step 1: FileX cache -> LevelX layer. */
    UINT fx = fx_media_flush(&g_fx_media);
    if (fx != FX_SUCCESS) {
        SERR_LOG(SERR_LFX_FX_OPEN, fx);  /* re-using FX_OPEN code: fx err on flush */
        return 0;
    }

    /* Step 2: LevelX cache -> NAND (only path that actually does this). */
    UINT lx = _lx_nand_flash_close(&g_lx_nand);
    if (lx != LX_SUCCESS) {
        /* LX close failed -- volume state is inconsistent (FileX
         * thinks it's mounted but LX is half-closed).  Mark unmounted
         * to force operator-visible error on subsequent FS calls
         * rather than silent corruption. */
        SERR_LOG(SERR_LFX_SYNC_LX_CLOSE, lx);
        g_mounted = false;
        return 0;
    }

    /* Step 3: re-open LX so the volume stays usable.  No FX re-open
     * needed; FileX retains its in-RAM media descriptor and the next
     * fx_* call will issue driver-level sector reads through the new
     * LX session transparently. */
    lx = lx_nand_flash_open(&g_lx_nand, (CHAR*)"sentai_user",
                            fx_nand_driver_initialize,
                            g_lx_memory_buffer,
                            sizeof(g_lx_memory_buffer));
    if (lx != LX_SUCCESS) {
        SERR_LOG(SERR_LFX_SYNC_LX_REOPEN, lx);
        printf("[fx_user] *** SYNC: LX reopen failed (%u) -- UNMOUNTED ***\r\n",
               (unsigned)lx);
        g_mounted = false;
        return 0;
    }
    /* Sync succeeded — every pending write is now durable on NAND. */
    g_unflushed_writes = 0;
    return 1;
}

/* Idle-driven auto-sync helper.  Called from CombinedWatchdogTask once
 * per 5 s tick.  Forces a sync only when:
 *   1. Volume is mounted (SAFE MODE => skip; nothing to flush anyway).
 *   2. There are unflushed writes pending.
 *   3. The last write was > kIdleSyncMs ago — i.e. activity has
 *      stopped, so the sync's ~200-500 ms cost won't compete with a
 *      hot write loop.
 *
 * Returns 1 if a sync ran (and succeeded), 0 if skipped, -1 on sync
 * failure.  Failures are logged via SERR_LOG inside FxUserSync. */
extern "C" int FxUserMaybeIdleSync(void) {
    if (!g_mounted) return 0;
    if (g_unflushed_writes == 0u) return 0;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((now - g_last_write_ms) < kIdleSyncMs) return 0;
    return FxUserSync() ? 1 : -1;
}

extern "C" int FxUserRemove(const char* path) {
    if (!g_mounted || path == nullptr) return -1;
    LockGuard guard;
    if (!guard.held) return -1;
    /* Try as file first; if not a file, try as dir.  This matches
     * lfs_remove which works on both.  No per-call media flush
     * (Phase 3.2): fx_file_delete already commits the directory
     * entry change; full FAT flush costs ~1 s and is unnecessary. */
    UINT fx = fx_file_delete(&g_fx_media, (CHAR*)path);
    if (fx == FX_SUCCESS) {
        note_write();
        return 0;
    }
    fx = fx_directory_delete(&g_fx_media, (CHAR*)path);
    if (fx == FX_SUCCESS) {
        note_write();
        return 0;
    }
    /* Map FX errors to negative LFS-ish return codes for compat. */
    if (fx == FX_NOT_FOUND) return -2;        /* LFS_ERR_NOENT */
    return -5;                                /* LFS_ERR_IO */
}

extern "C" int FxUserMakeDir(const char* path) {
    if (!g_mounted || path == nullptr) return 0;
    LockGuard guard;
    if (!guard.held) return 0;
    UINT fx = fx_directory_create(&g_fx_media, (CHAR*)path);
    if (fx == FX_SUCCESS || fx == FX_ALREADY_CREATED) return 1;
    return 0;
}

extern "C" int FxUserMakeDirs(const char* path) {
    if (!g_mounted || path == nullptr) return 0;
    /* Walk path components and create each.  Path is mutated locally in
     * a stack copy (capped at kPathScratch). */
    char buf[kPathScratch];
    size_t plen = std::strlen(path);
    if (plen == 0) return 1;
    if (plen >= kPathScratch) return 0;
    std::memcpy(buf, path, plen);
    buf[plen] = '\0';

    /* Strip trailing '/'. */
    while (plen > 1u && buf[plen - 1] == '/') {
        buf[--plen] = '\0';
    }
    LockGuard guard;
    if (!guard.held) return 0;

    /* Iterate "/a/b/c" -> create "/a", "/a/b", "/a/b/c".  Treat
     * FX_ALREADY_CREATED as success (idempotent mkdir -p), but
     * surface other errors so the Python `sentai.fs.mkdir` return
     * value tells the truth.  Old behaviour ignored ALL fx codes
     * which silently masked NAND read errors and made callers
     * believe the directory existed when it did not. */
    bool any_created = false;
    for (size_t i = 1; i <= plen; ++i) {
        if (i == plen || buf[i] == '/') {
            char saved = buf[i];
            buf[i] = '\0';
            UINT fx = fx_directory_create(&g_fx_media, buf);
            if (fx == FX_SUCCESS) {
                any_created = true;
            } else if (fx != FX_ALREADY_CREATED) {
                buf[i] = saved;
                return 0;
            }
            buf[i] = saved;
        }
    }
    if (any_created) note_write();
    return 1;
}

extern "C" int FxUserRename(const char* from, const char* to) {
    if (!g_mounted || from == nullptr || to == nullptr) return -1;
    LockGuard guard;
    if (!guard.held) return -1;
    UINT fx = fx_file_rename(&g_fx_media, (CHAR*)from, (CHAR*)to);
    if (fx == FX_SUCCESS) { note_write(); return 0; }
    fx = fx_directory_rename(&g_fx_media, (CHAR*)from, (CHAR*)to);
    if (fx == FX_SUCCESS) { note_write(); return 0; }
    if (fx == FX_NOT_FOUND) return -2;
    return -5;
}

/* RAII guard: save the current FileX default-path, set it to a new
 * value for the duration of a scope, then restore on dtor.  Crucial
 * for FxUserListDir — the iteration walks the "default" dir, but
 * other tasks share the same FX_MEDIA so leaving it pointing at the
 * caller's path would silently affect their subsequent ops.
 * Embeded.md §E: explicit ownership across boundaries. */
struct DefaultPathGuard {
    bool restored;
    CHAR saved[kPathScratch];
    DefaultPathGuard(const CHAR* new_path) : restored(false) {
        UINT g = fx_directory_default_get_copy(&g_fx_media, saved,
                                                sizeof(saved));
        if (g != FX_SUCCESS) saved[0] = '\0';
        UINT s = fx_directory_default_set(&g_fx_media, (CHAR*)new_path);
        if (s != FX_SUCCESS) {
            /* Set failed — saved is still original; mark "no need to
             * restore" so dtor doesn't double-fault. */
            restored = true;
        }
    }
    ~DefaultPathGuard() {
        if (restored) return;
        const CHAR* tgt = (saved[0] != '\0') ? saved : (CHAR*)"/";
        (void)fx_directory_default_set(&g_fx_media, (CHAR*)tgt);
    }
    bool ok() const { return !restored; }
};

extern "C" int FxUserListDir(const char* path, FxDirCallback cb, void* user) {
    if (cb == nullptr) return -1;
    if (!g_mounted) {
        SERR_LOG(SERR_LFX_NOT_MOUNTED, 0x10u /* ListDir */);
        return -1;
    }
    LockGuard guard;
    if (!guard.held) return -1;

    const CHAR* target =
        (path == nullptr || path[0] == '\0') ? (CHAR*)"/" : (CHAR*)path;
    DefaultPathGuard pathg(target);
    if (!pathg.ok()) return -1;

    int count = 0;
    CHAR     entry_name[kPathScratch];
    UINT     attr = 0;
    ULONG    sz = 0;
    UINT     y, mo, d, h, mi, s;
    UINT     fx = fx_directory_first_full_entry_find(&g_fx_media,
                        entry_name, &attr, &sz,
                        &y, &mo, &d, &h, &mi, &s);
    /* Strict bound: we never iterate more than the FAT root cap (256)
     * times in the root, or 4096 in subdirs. */
    constexpr int kMaxEntries = 4096;
    int iter = 0;
    while (fx == FX_SUCCESS && iter < kMaxEntries) {
        ++iter;
        /* Skip "." / ".." (FileX sometimes emits them in subdirs). */
        if (entry_name[0] != '.' ||
            (entry_name[1] != '\0' &&
             (entry_name[1] != '.' || entry_name[2] != '\0'))) {
            FxDirEntry e = {};
            size_t nlen = std::strlen(entry_name);
            if (nlen >= sizeof(e.name)) nlen = sizeof(e.name) - 1;
            std::memcpy(e.name, entry_name, nlen);
            e.name[nlen] = '\0';
            e.is_dir = (attr & FX_DIRECTORY) ? 1 : 0;
            e.size = (uint32_t)sz;
            e.mtime_s = (h * 3600u) + (mi * 60u) + s;
            ++count;
            int r = cb(&e, user);
            if (r != 0) break;
        }
        fx = fx_directory_next_full_entry_find(&g_fx_media,
                        entry_name, &attr, &sz,
                        &y, &mo, &d, &h, &mi, &s);
    }
    /* DefaultPathGuard restores in dtor. */
    return count;
}

extern "C" void FxUserGetStats(FxUserStats* out) {
    if (out == nullptr) return;
    std::memset(out, 0, sizeof(*out));
    out->mounted = g_mounted ? 1u : 0u;
    out->mount_failures = g_mount_failures;
    out->format_count = g_format_count;
    if (!g_mounted) return;
    LockGuard guard;
    if (!guard.held) return;
    out->bytes_per_sector    = g_fx_media.fx_media_bytes_per_sector;
    out->sectors_per_cluster = g_fx_media.fx_media_sectors_per_cluster;
    out->total_clusters      = g_fx_media.fx_media_total_clusters;
    /* Free clusters: walk the FAT.  fx_media_extended_space_available
     * gives total free bytes; convert. */
    ULONG64 free_bytes = 0;
    if (fx_media_extended_space_available(&g_fx_media, &free_bytes)
            == FX_SUCCESS) {
        ULONG cluster_bytes = out->bytes_per_sector *
                              out->sectors_per_cluster;
        if (cluster_bytes > 0) {
            out->free_clusters = (uint32_t)(free_bytes / cluster_bytes);
        }
    }
}

/* ===== Storage-mode MSC entry points ================================ */

namespace {
bool g_lx_only_open = false;  /* true after FxUserOpenLxOnly() succeeds */
}  /* namespace */

/* ===== Storage-mode debug log ========================================
 *
 * Buffer placed in a NOLOAD SDRAM section (.sdram_storage_log) so the
 * startup BSS-zero loop does not touch it across warm reset.  Header
 * uses TWO independent magic words plus a size field so a single
 * stray SDRAM bit pattern cannot falsely validate stale memory
 * (embeded.md §H: persistent state must be validated).
 *
 *   magic1 == 0xC0FFEE51 AND
 *   magic2 == 0xDEB60106 AND
 *   payload_size == sizeof(payload) AND
 *   write_pos <= payload_size
 *
 * On every storage-mode boot we increment `generation` so the flush
 * path can tell "fresh data this run" from "leftover garbage from a
 * crash that didn't reach init".  Set before USB starts, cleared on
 * flush. */
struct StorageLogRing {
    uint32_t magic1;        /* 0xC0FFEE51 */
    uint32_t magic2;        /* 0xDEB60106 */
    uint32_t payload_size;  /* must equal sizeof(payload) at validate */
    uint32_t generation;    /* monotonic across storage-mode boots    */
    uint32_t write_pos;     /* bytes written into payload             */
    uint32_t pad[3];        /* keep payload 32-aligned                */
    char     payload[16u * 1024u - 32u];
};
static_assert(sizeof(StorageLogRing) == 16u * 1024u, "ring size");
constexpr uint32_t kStorageLogMagic1 = 0xC0FFEE51u;
constexpr uint32_t kStorageLogMagic2 = 0xDEB60106u;

/* Allocated in the dedicated SDRAM section to survive warm reset. */
StorageLogRing g_storage_log_ring
    __attribute__((aligned(8), section(".sdram_storage_log")));

/* Non-blocking test-and-set lock (no FreeRTOS dependency, callable from
 * any context including ISR). */
static volatile uint32_t g_storage_log_lock = 0;

/* Helper: STRICT validity check.  All four invariants must hold. */
static inline bool storage_log_ring_valid() {
    return g_storage_log_ring.magic1 == kStorageLogMagic1 &&
           g_storage_log_ring.magic2 == kStorageLogMagic2 &&
           g_storage_log_ring.payload_size ==
               sizeof(g_storage_log_ring.payload) &&
           g_storage_log_ring.write_pos <=
               sizeof(g_storage_log_ring.payload);
}

extern "C" void sentai_storage_log_init(void) {
    /* Read the previous generation BEFORE clobbering, so consumers
     * can see the buffer roll forward across storage sessions. */
    uint32_t prev_gen =
        storage_log_ring_valid() ? g_storage_log_ring.generation : 0u;
    g_storage_log_ring.magic1       = kStorageLogMagic1;
    g_storage_log_ring.magic2       = kStorageLogMagic2;
    g_storage_log_ring.payload_size = sizeof(g_storage_log_ring.payload);
    g_storage_log_ring.generation   = prev_gen + 1u;
    g_storage_log_ring.write_pos    = 0;
    /* Stamp a header so the file is self-describing on flush. */
    int n = std::snprintf(g_storage_log_ring.payload,
                          sizeof(g_storage_log_ring.payload),
                          "[storage-debug] session boot gen=%u uptime=%u ms\r\n",
                          (unsigned)g_storage_log_ring.generation,
                          (unsigned)now_ms());
    if (n > 0) g_storage_log_ring.write_pos = (uint32_t)n;
}

extern "C" void sentai_storage_log(const char* fmt, ...) {
    /* DO NOT lazy-reset here — that would clobber the buffer if a
     * caller raced ahead of sentai_storage_log_init.  Just drop the
     * event if the ring isn't initialised. */
    if (!storage_log_ring_valid()) return;
    if (__sync_lock_test_and_set(&g_storage_log_lock, 1u) != 0u) return;
    uint32_t pos = g_storage_log_ring.write_pos;
    if (pos + 32u < sizeof(g_storage_log_ring.payload)) {
        /* Reserve 2 bytes for trailing \r\n. */
        size_t avail = sizeof(g_storage_log_ring.payload) - 2u - pos;
        /* Prefix every line with uptime ms for easy correlation. */
        int hdr = std::snprintf(g_storage_log_ring.payload + pos, avail,
                                "[%u] ", (unsigned)now_ms());
        if (hdr > 0 && (size_t)hdr < avail) {
            pos += (uint32_t)hdr;
            avail -= (size_t)hdr;
            va_list ap;
            va_start(ap, fmt);
            int n = vsnprintf(g_storage_log_ring.payload + pos, avail,
                              fmt, ap);
            va_end(ap);
            if (n > 0) {
                /* vsnprintf may return n >= avail if truncated; clamp. */
                size_t actual = ((size_t)n < avail) ? (size_t)n
                                                    : avail - 1u;
                pos += (uint32_t)actual;
                /* We reserved 2 bytes — append CRLF unconditionally. */
                g_storage_log_ring.payload[pos++] = '\r';
                g_storage_log_ring.payload[pos++] = '\n';
                g_storage_log_ring.write_pos = pos;
            }
        }
    }
    __sync_lock_release(&g_storage_log_lock);
}

extern "C" int sentai_storage_log_flush_to_fs(void) {
    if (!storage_log_ring_valid()) return 0;
    uint32_t len = g_storage_log_ring.write_pos;
    if (len == 0 || len > sizeof(g_storage_log_ring.payload)) {
        /* Either nothing to flush, or write_pos is corrupt (out of
         * bounds despite passing the magic check — invalidate the
         * whole ring, don't risk writing garbage to FS). */
        g_storage_log_ring.magic1 = 0;
        g_storage_log_ring.magic2 = 0;
        return 0;
    }
    /* Bookend the dump with a trailer line so consecutive sessions
     * are distinguishable in the log file. */
    char trailer[64];
    int t = std::snprintf(trailer, sizeof(trailer),
                          "[storage-debug] flushed %u bytes (uptime=%u ms)\r\n\r\n",
                          (unsigned)len, (unsigned)now_ms());
    /* Write payload then trailer. */
    coralmicro::LfsUserMakeDirs("/log");
    bool ok = ::FxUserAppendFile("/log/storage_debug.log",
                                  reinterpret_cast<const uint8_t*>(g_storage_log_ring.payload),
                                  len) != 0;
    if (t > 0) {
        (void)::FxUserAppendFile("/log/storage_debug.log",
                                  reinterpret_cast<const uint8_t*>(trailer),
                                  (size_t)t);
    }
    /* Invalidate so we don't re-flush on next boot (unless storage
     * mode runs again and re-initialises).  Clear BOTH magic words +
     * write_pos so partial corruption can't fool the validator. */
    g_storage_log_ring.magic1 = 0;
    g_storage_log_ring.magic2 = 0;
    g_storage_log_ring.write_pos = 0;
    return ok ? (int)len : 0;
}

extern "C" int FxUserOpenLxOnly(void) {
    if (g_fx_mutex == nullptr) {
        g_fx_mutex = xSemaphoreCreateMutex();
        if (g_fx_mutex == nullptr) return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;
    if (g_lx_only_open) return 1;
    if (!g_systems_initialized) {
        fx_system_initialize();
        lx_nand_flash_initialize();
        g_systems_initialized = true;
    }

    /* Bounded retry — same policy as FxUserInit (embeded.md §F). */
    UINT lx = LX_ERROR;
    for (uint32_t attempt = 0; attempt < kMountRetries; ++attempt) {
        lx = lx_nand_flash_open(&g_lx_nand, (CHAR*)"sentai_user",
                                fx_nand_driver_initialize,
                                g_lx_memory_buffer,
                                sizeof(g_lx_memory_buffer));
        if (lx == LX_SUCCESS) {
            g_lx_only_open = true;
            if (attempt > 0) {
                SERR_LOG(SERR_LFX_MOUNT_RETRY_OK, attempt);
            }
            printf("[fx_user] LxOnly mounted for storage MSC%s\r\n",
                   attempt > 0 ? " (after retry)" : "");
            return 1;
        }
        if (attempt + 1u < kMountRetries) {
            bounded_delay_ms(kMountRetryDelayMs);
        }
    }

    /* Virgin-NAND first-boot in storage mode is legitimate (board
     * just flashed, host wants to inspect raw partition).  Auto-
     * format ONLY on the unambiguous virgin signature; otherwise
     * refuse and force the operator to recover via default-mode
     * boot + explicit `sentai.diag.fx_format(0xDEADBEEF)`. */
    if (is_virgin_nand_signature(lx)) {
        SERR_LOG(SERR_LFX_FIRST_BOOT_FORMAT, lx);
        printf("[fx_user] LxOnly: virgin NAND (lx=%u) -> first-boot format\r\n",
               (unsigned)lx);
        UINT fmt = lx_nand_flash_format(&g_lx_nand, (CHAR*)"sentai_user",
                                        fx_nand_driver_initialize,
                                        g_lx_memory_buffer,
                                        sizeof(g_lx_memory_buffer));
        if (fmt != LX_SUCCESS) {
            SERR_LOG(SERR_LFX_LX_FORMAT, fmt);
            return 0;
        }
        lx = lx_nand_flash_open(&g_lx_nand, (CHAR*)"sentai_user",
                                fx_nand_driver_initialize,
                                g_lx_memory_buffer,
                                sizeof(g_lx_memory_buffer));
        if (lx != LX_SUCCESS) {
            SERR_LOG(SERR_LFX_LX_OPEN, lx);
            return 0;
        }
        g_lx_only_open = true;
        printf("[fx_user] LxOnly mounted (post first-boot format)\r\n");
        return 1;
    }

    /* SAFE MODE for storage MSC: refuse to mount, refuse to format.
     * Storage MSC simply won't expose /dev/sda; operator boots into
     * default mode where FxUserInit will reach the same SAFE-MODE
     * gate and surface a recoverable error. */
    SERR_LOG(SERR_LFX_LXONLY_FAIL_SAFE, lx);
    printf("[fx_user] *** LxOnly MOUNT FAIL (lx=%u) -- SAFE MODE ***\r\n",
           (unsigned)lx);
    printf("[fx_user]   Storage MSC unavailable; refusing implicit format.\r\n");
    printf("[fx_user]   Reboot to default mode and run:\r\n");
    printf("[fx_user]     sentai.diag.fx_format(0xDEADBEEF)  [DESTROYS DATA]\r\n");
    return 0;
}

extern "C" int FxUserMscLbaSize(void) {
    return (int)FX_NAND_BYTES_PER_PAGE;  /* 2048 — power-of-2 FAT sector */
}

extern "C" int FxUserMscLbaCount(void) {
    /* Match the logical-sector range FileX uses (see format_and_mount). */
    return (int)((FX_NAND_USER_BLOCK_COUNT - 8u) * FX_NAND_PAGES_PER_BLOCK);
}

extern "C" int FxUserMscRead(uint32_t lba, uint8_t* buf) {
    if (buf == nullptr) return 0;
    if (lba >= FX_USER_LBA_COUNT) {
        SERR_LOG(SERR_LFX_LBA_RANGE, lba);
        return 0;
    }
    if (!g_lx_only_open && !g_mounted) {
        SERR_LOG(SERR_LFX_NOT_MOUNTED, 0x01u /* MscRead */);
        return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;  /* SERR_LFX_LOCK_TIMEOUT already logged */
    UINT lx = _lx_nand_flash_sector_read(&g_lx_nand, lba, buf);
    return (lx == LX_SUCCESS) ? 1 : 0;
}

extern "C" int FxUserMscWrite(uint32_t lba, const uint8_t* buf) {
    if (buf == nullptr) return 0;
    if (lba >= FX_USER_LBA_COUNT) {
        SERR_LOG(SERR_LFX_LBA_RANGE, lba);
        return 0;
    }
    if (!g_lx_only_open && !g_mounted) {
        SERR_LOG(SERR_LFX_NOT_MOUNTED, 0x02u /* MscWrite */);
        return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;
    UINT lx = _lx_nand_flash_sector_write(&g_lx_nand, lba, (VOID*)buf);
    return (lx == LX_SUCCESS) ? 1 : 0;
}

extern "C" int FxUserBenchRoot(FxBenchResult* out) {
    if (out == nullptr) return -1;
    std::memset(out, 0, sizeof(*out));
    if (!g_mounted) return -1;
    uint32_t t0 = now_ms();
    int n = FxUserListDir("/", [](const FxDirEntry*, void*) -> int { return 0; },
                          nullptr);
    out->time_list_ms = now_ms() - t0;
    if (n < 0) return -1;
    out->entries = (uint32_t)n;
    out->ok = 1;
    return 0;
}

/* ===== C++ overloads ================================================ */

namespace coralmicro_fx {

bool FxUserReadFile(const char* path, std::vector<uint8_t>* buf) {
    if (buf == nullptr) return false;
    ssize_t sz = ::FxUserSize(path);
    if (sz < 0) return false;
    buf->resize((size_t)sz);
    if (sz == 0) return true;
    size_t n = ::FxUserReadFile(path, buf->data(), buf->size());
    return n == buf->size();
}

bool FxUserReadFile(const char* path, std::string* str) {
    if (str == nullptr) return false;
    ssize_t sz = ::FxUserSize(path);
    if (sz < 0) return false;
    str->resize((size_t)sz);
    if (sz == 0) return true;
    size_t n = ::FxUserReadFile(path,
        reinterpret_cast<uint8_t*>(&(*str)[0]), str->size());
    return n == str->size();
}

bool FxUserWriteFile(const char* path, const std::string& s) {
    return ::FxUserWriteFile(path,
        reinterpret_cast<const uint8_t*>(s.c_str()), s.size()) != 0;
}

}  /* namespace coralmicro_fx */
