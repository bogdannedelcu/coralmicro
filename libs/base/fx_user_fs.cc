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
#include "libs/base/fx_sd_driver.h"
#include "libs/base/filesystem.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" {
#include "fx_api.h"
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
 * only; no heap in steady state.  Total SDRAM cost ~20 KB.
 *
 * 2026-10-30: backend is the microSD card (fx_sd_driver, 512 B blocks,
 * no LevelX).  The FileX sector cache is also the DMA target for the
 * USDHC host, so it is 32-byte aligned (D-cache line) — fx_sd_driver
 * bounces anything less aligned through its own staging buffer.
 */
constexpr size_t kFxMediaSize   = 16u * 1024u;   /* 32 cached sectors */
constexpr size_t kFxFtSize      = 4u * 1024u;    /* >= FX_FAULT_TOLERANT_MINIMAL_BUFFER_SIZE */
constexpr size_t kPathScratch   = 256u;

UCHAR g_fx_media_memory[kFxMediaSize]
    __attribute__((aligned(32), section(".sdram_bss")));
UCHAR g_fx_ft_memory[kFxFtSize]
    __attribute__((aligned(32), section(".sdram_bss")));
/* Scratch for the LBA0 blank-card probe (never handed to FileX). */
UCHAR g_fx_probe_sector[FX_SD_BLOCK_SIZE]
    __attribute__((aligned(32), section(".sdram_bss")));

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

/* ===== Media driver ==================================================
 *
 * fx_sd_driver() (libs/base/fx_sd_driver.cc) is the FX_MEDIA driver:
 * 512-byte card blocks, MBR-partition aware, DMA-alignment safe.  The
 * old FX<->LevelX bridge (NAND user partition) was removed 2026-10-30. */
inline void (*fx_user_media_driver())(FX_MEDIA*) {
    return reinterpret_cast<void (*)(FX_MEDIA*)>(fx_sd_driver);
}

/* FileX fault-tolerant journal.  Must be enabled right after a
 * successful fx_media_open() and before any other media operation.
 * Failure is non-fatal (volume still usable, just without the journal)
 * but logged so brownout forensics know the protection was missing. */
inline void enable_fault_tolerant() {
    UINT fx = fx_fault_tolerant_enable(&g_fx_media, g_fx_ft_memory,
                                       sizeof(g_fx_ft_memory));
    if (fx != FX_SUCCESS) {
        SERR_LOG(SERR_LFX_FT, fx);
        printf("[fx_user] fault_tolerant_enable failed: %u (continuing)\r\n",
               (unsigned)fx);
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

/* ===== Internal: format the card (FAT32 superfloppy) =================
 *
 * Caller MUST hold g_fx_mutex and FxSdInit() must have succeeded.  On
 * success the volume is left MOUNTED (fx_media_open done, journal on).
 *
 * Layout: no MBR (FileX writes the VBR at block 0 — "superfloppy",
 * mounted natively by Linux/Windows/macOS), 512 B sectors, 64 sectors
 * (32 KB) per cluster, one FAT.  FileX picks FAT32 from the cluster
 * count (a 64 GB card → ~1.9 M clusters).  32 KB clusters amortise the
 * per-write FAT/dir-entry overhead for JPEG captures and FR logs; the
 * fault-tolerant journal protects metadata across brownouts. */
bool format_and_mount() {
    (void)fx_media_close(&g_fx_media);   /* no-op if not open */
    sentai_repl_activity();

    ULONG total_sectors = FxSdBlockCount();
    if (total_sectors == 0u) {
        SERR_LOG(SERR_LFX_SD_INIT_FAIL, 0);
        return false;
    }
    /* Superfloppy: the new VBR goes to block 0, whatever MBR offset the
     * failed mount attempts may have learnt from the old content. */
    FxSdResetPartition();
    uint32_t t0 = now_ms();
    UINT fx = fx_media_format(&g_fx_media,
                              fx_user_media_driver(),
                              FX_NULL,
                              g_fx_media_memory,
                              sizeof(g_fx_media_memory),
                              (CHAR*)"SENTAI",
                              1,                 /* number of FATs   */
                              512,               /* root dir entries (FAT12/16 only) */
                              0,                 /* hidden sectors   */
                              total_sectors,
                              FX_SD_BLOCK_SIZE,  /* sector size      */
                              64,                /* sectors/cluster = 32 KB */
                              255,               /* heads            */
                              63);               /* sectors/track    */
    if (fx != FX_SUCCESS) {
        printf("[fx_user] fx_media_format failed: %u\r\n", (unsigned)fx);
        SERR_LOG(SERR_LFX_FX_FORMAT, fx);
        return false;
    }
    printf("[fx_user] fx_media_format ok (%lu sectors) in %u ms\r\n",
           (unsigned long)total_sectors, (unsigned)(now_ms() - t0));
    g_format_count++;
    sentai_repl_activity();

    fx = fx_media_open(&g_fx_media, (CHAR*)"sentai_sd",
                       fx_user_media_driver(), FX_NULL,
                       g_fx_media_memory, sizeof(g_fx_media_memory));
    if (fx != FX_SUCCESS) {
        printf("[fx_user] fx_media_open after format failed: %u\r\n",
               (unsigned)fx);
        SERR_LOG(SERR_LFX_FX_OPEN, fx);
        return false;
    }
    enable_fault_tolerant();
    g_mounted = true;
    printf("[fx_user] mounted (post-format)\r\n");
    return true;
}

/* Boot-sector sanity check BEFORE fx_media_open (embeded.md §H: validate
 * persistent state before trusting it).  FileX itself accepts almost any
 * BPB and then walks FAT chains built from garbage — on a half-formatted
 * card that walk never returned (2026-10-30, build 1613).  Returns 0 when
 * the volume looks like a FAT12/16/32 we can mount, else the id of the
 * failed check (logged as SERR_LFX_BPB_INVALID val). */
extern "C" UINT _fx_partition_offset_calculate(void*, UINT, ULONG*, ULONG*);
inline uint16_t rd16(const UCHAR* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t rd32(const UCHAR* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
int validate_boot_sector() {
    const uint32_t card_blocks = FxSdBlockCount();
    if (!FxSdRead(0, g_fx_probe_sector, 1)) return 1;
    ULONG start = 0, size = 0;
    if (_fx_partition_offset_calculate(g_fx_probe_sector, 0, &start, &size)
            != FX_SUCCESS) return 2;
    if (start != 0u) {
        if (start >= card_blocks) return 3;
        if (!FxSdRead(start, g_fx_probe_sector, 1)) return 1;
    }
    const UCHAR* b = g_fx_probe_sector;
    if (!(b[0] == 0xEBu || b[0] == 0xE9u)) return 4;
    if (!(b[510] == 0x55u && b[511] == 0xAAu)) return 4;
    if (rd16(b + 0x0B) != FX_SD_BLOCK_SIZE) return 5;      /* exFAT has 0 here */
    const uint8_t spc = b[0x0D];
    if (spc == 0u || (spc & (spc - 1u)) != 0u) return 6;    /* power of two */
    if (rd16(b + 0x0E) == 0u) return 7;                      /* reserved sectors */
    const uint8_t fats = b[0x10];
    if (fats == 0u || fats > 2u) return 7;
    uint32_t total = rd16(b + 0x13) ? rd16(b + 0x13) : rd32(b + 0x20);
    if (total == 0u || total > card_blocks - start) return 8;
    uint32_t spf = rd16(b + 0x16) ? rd16(b + 0x16) : rd32(b + 0x24);
    if (spf == 0u || (uint64_t)rd16(b + 0x0E) + (uint64_t)fats * spf >= total) return 9;
    /* hidden_sectors must equal the MBR partition start (or 0 on a
     * superfloppy); the driver applies the MBR offset alone. */
    const uint32_t hidden = rd32(b + 0x1C);
    if (hidden != 0u && hidden != start) return 10;
    return 0;
}

/* Try to mount an EXISTING volume.  Returns the fx_media_open status
 * (FX_MEDIA_INVALID when the boot sector fails validation);
 * FX_SUCCESS => g_mounted = true (journal enabled). */
UINT try_mount_existing() {
    int bad = validate_boot_sector();
    if (bad != 0) {
        SERR_LOG(SERR_LFX_BPB_INVALID, bad);
        return FX_MEDIA_INVALID;
    }
    UINT fx = fx_media_open(&g_fx_media, (CHAR*)"sentai_sd",
                            fx_user_media_driver(), FX_NULL,
                            g_fx_media_memory, sizeof(g_fx_media_memory));
    if (fx != FX_SUCCESS) return fx;
    enable_fault_tolerant();
    g_mounted = true;
    return FX_SUCCESS;
}

/* True when block 0 carries no boot-sector signature (0x55AA) — a card
 * that was never formatted (or was wiped).  That is the ONLY state in
 * which auto-format cannot destroy user data.  A card with a signature
 * that FileX still cannot mount (exFAT, foreign FS, corruption) goes to
 * SAFE MODE; the operator decides with fx_format(0xDEADBEEF). */
inline bool is_blank_card() {
    if (!FxSdRead(0, g_fx_probe_sector, 1)) return false;  /* unreadable != blank */
    return !(g_fx_probe_sector[510] == 0x55u && g_fx_probe_sector[511] == 0xAAu);
}

/* FileX FAT accepts both '/' and '\\' as separators; we keep the
 * existing LFS-style "/dir/file" paths verbatim in callers.
 *
 * (path_basename helper removed in Phase 3.5 audit — no live caller.) */

}  /* anonymous namespace */

/* ===== Public API ===================================================== */

/* ===== Boot-time mount with deadline ==================================
 *
 * fx_media_open() / fx_fault_tolerant_enable() walk on-card structures;
 * on a corrupt volume those walks are not guaranteed to terminate (seen
 * 2026-10-30 on a half-formatted exFAT card: boot never reached the REPL).
 * embeded.md §B: every timing-sensitive path must be bounded, so app_main
 * runs the mount in its own task and waits at most `deadline_ms`.  On
 * timeout the board continues in SAFE MODE (g_mounted stays false, every
 * FxUser* call returns a clean error after a 2 s lock timeout) and the
 * stuck task is left alone until the operator asks for a format, which
 * reaps it (reap_stuck_mount) before touching the card. */
namespace {
TaskHandle_t      g_mount_task      = nullptr;
SemaphoreHandle_t g_mount_done      = nullptr;
volatile bool     g_mount_stuck     = false;

void mount_task_fn(void*) {
    (void)FxUserInit(0);
    if (g_mount_done) xSemaphoreGive(g_mount_done);
    g_mount_task = nullptr;
    vTaskDelete(nullptr);
}

/* Caller must NOT hold g_fx_mutex (the stuck task may own it).
 *
 * No vTaskDelete / vSemaphoreDelete: the stuck task may be inside the SDK
 * host transfer holding host->lock, or inside FileX holding g_fx_mutex
 * with other tasks blocked on it.  Instead the SD layer is switched to
 * "abort": every FxSdRead/Write fails at once, FileX's own error paths
 * unwind, FxUserInit() returns through its normal SAFE-MODE exit (lock
 * released, g_mount_done given) and the task deletes itself.  Bounded
 * wait: one in-flight SD command can still take a hardware data-timeout.
 * If it does not unwind in time, the SD layer stays disabled until
 * reboot — never free objects the stuck task may own. */
constexpr uint32_t kReapWaitMs = 5000u;
bool reap_stuck_mount() {
    if (!g_mount_stuck) return true;
    printf("[fx_user] unwinding stuck boot-mount task (SD abort)\r\n");
    FxSdAbortSet(1);
    bool done = (xSemaphoreTake(g_mount_done, pdMS_TO_TICKS(kReapWaitMs)) == pdTRUE);
    FxSdAbortSet(0);
    if (!done) {
        SERR_LOG(SERR_LFX_MOUNT_REAP_FAIL, kReapWaitMs);
        printf("[fx_user] *** stuck mount did not unwind in %lu ms -- SD disabled until reboot ***\r\n",
               (unsigned long)kReapWaitMs);
        return false;
    }
    (void)fx_media_abort(&g_fx_media);     /* drop any half-open media state */
    g_mounted = false;
    g_mount_stuck = false;
    printf("[fx_user] stuck mount unwound cleanly\r\n");
    return true;
}
}  /* namespace */

extern "C" int FxUserBootMount(uint32_t deadline_ms) {
    if (g_mount_done == nullptr) {
        g_mount_done = xSemaphoreCreateBinary();
        if (g_mount_done == nullptr) return 0;
    }
    if (xTaskCreate(mount_task_fn, "fs_mount", 2048, nullptr,
                    uxTaskPriorityGet(nullptr), &g_mount_task) != pdPASS) {
        printf("[fx_user] fs_mount task create failed -- SAFE MODE\r\n");
        return 0;
    }
    if (xSemaphoreTake(g_mount_done, pdMS_TO_TICKS(deadline_ms)) == pdTRUE) {
        return g_mounted ? 1 : 0;
    }
    g_mount_stuck = true;
    SERR_LOG(SERR_LFX_MOUNT_TIMEOUT, deadline_ms);
    printf("[fx_user] *** MOUNT DEADLINE (%lu ms) EXCEEDED -- SAFE MODE ***\r\n",
           (unsigned long)deadline_ms);
    printf("[fx_user]   Card structures look corrupt.  REPL stays alive;\r\n");
    printf("[fx_user]   sentai.fs.format() reaps the stuck mount and reformats.\r\n");
    return 0;
}

extern "C" int FxUserInit(int force_format) {
    /* A wedged boot-time mount still owns the mutex: only a format
     * request (operator-confirmed) is allowed to unwind it. */
    if (force_format && !reap_stuck_mount()) return 0;
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
        g_systems_initialized = true;
    }

    /* If currently mounted, close cleanly before re-init. */
    if (g_mounted) {
        (void)fx_media_close(&g_fx_media);
        g_mounted = false;
    }

    /* Card bring-up.  Bounded inside the SD stack (command timeouts);
     * no card / dead card => SAFE MODE right here.  Post-scheduler only:
     * the USDHC host blocks on FreeRTOS events (app_main calls us). */
    const uint32_t t_init0 = now_ms();
    const int sd_ok = FxSdInit();
    printf("[fx_user] SD init %s in %u ms\r\n", sd_ok ? "ok" : "FAILED",
           (unsigned)(now_ms() - t_init0));
    if (!sd_ok) {
        g_mount_failures++;
        printf("[fx_user] *** NO SD CARD (or init failed) -- SAFE MODE ***\r\n");
        printf("[fx_user]   Insert a card and reboot, or run from RAM/dmesg only.\r\n");
        g_mounted = false;
        /* Boot path: SAFE MODE is a successful boot (see note at the end).
         * Operator format: there is nothing to format -> report failure. */
        return force_format ? 0 : 1;
    }

    /* Operator-confirmed format path — only entered with force_format=1
     * (e.g. `sentai.diag.fx_format(FX_DESTRUCTIVE_CONFIRM_MAGIC)`).
     * Distinct from the blank-card auto-format below. */
    if (force_format) {
        printf("[fx_user] force_format requested (operator confirmed)\r\n");
        return format_and_mount() ? 1 : 0;
    }

    /* Bounded retry — embeded.md §F: tolerate a transient card hiccup
     * right after power-up before any escalation. */
    UINT last_fx = FX_NOT_OPEN;
    for (uint32_t attempt = 0; attempt < kMountRetries; ++attempt) {
        const uint32_t t_mount0 = now_ms();
        last_fx = try_mount_existing();
        if (last_fx == FX_SUCCESS) {
            printf("[fx_user] fx_media_open + journal in %u ms\r\n",
                   (unsigned)(now_ms() - t_mount0));
            if (attempt > 0) {
                SERR_LOG(SERR_LFX_MOUNT_RETRY_OK, attempt);
                printf("[fx_user] mounted (existing volume) after %u retr%s\r\n",
                       (unsigned)attempt, attempt == 1u ? "y" : "ies");
            } else {
                printf("[fx_user] mounted SD volume (part start %lu, %lu clusters x %lu B)\r\n",
                       (unsigned long)FxSdPartitionStart(),
                       (unsigned long)g_fx_media.fx_media_total_clusters,
                       (unsigned long)(g_fx_media.fx_media_sectors_per_cluster *
                                       g_fx_media.fx_media_bytes_per_sector));
            }
            sentai_repl_activity();
            return 1;
        }
        if (attempt + 1u < kMountRetries) {
            bounded_delay_ms(kMountRetryDelayMs);
        }
    }

    g_mount_failures++;

    /* Blank-card bootstrap is the ONLY auto-format path: no 0x55AA at
     * block 0 means there is nothing to destroy.  Anything else (exFAT
     * from a PC, a foreign FS, a damaged FAT) is preserved for the
     * operator to inspect / decide. */
    if (is_blank_card()) {
        SERR_LOG(SERR_LFX_SD_BLANK_FORMAT, 0);
        printf("[fx_user] blank card (fx=%u) -> first-use FAT32 format\r\n",
               (unsigned)last_fx);
        return format_and_mount() ? 1 : 0;
    }

    /* SAFE MODE — embeded.md §F escalation: degraded mode preserves
     * data and keeps REPL alive so the operator can decide.  The
     * board STAYS in communication (anti-brick rule §M satisfied):
     * USB CDC is up, REPL is up, radio bridge auto-init still runs.
     * FxUser* APIs return clean errors on every write/read attempt
     * because g_mounted is false. */
    SERR_LOG(SERR_LFX_FS_UNKNOWN, last_fx);
    printf("[fx_user] *** MOUNT FAIL (fx=%u) -- SAFE MODE ***\r\n",
           (unsigned)last_fx);
    printf("[fx_user]   Card holds a filesystem FileX cannot mount (exFAT/foreign/damaged).\r\n");
    printf("[fx_user]   FS is read+write disabled to protect existing data.\r\n");
    printf("[fx_user]   REPL + radio remain alive for diagnosis.\r\n");
    printf("[fx_user]   To DESTROY the card contents and format FAT32:\r\n");
    printf("[fx_user]     sentai.fs.format()\r\n");
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

extern "C" size_t FxUserReadFileAt(const char* path, uint32_t offset,
                                    uint8_t* buf, size_t size) {
    if (!g_mounted || path == nullptr || buf == nullptr || size == 0) return 0;
    LockGuard guard;
    if (!guard.held) return 0;

    FX_FILE f;
    UINT fx = fx_file_open(&g_fx_media, &f, (CHAR*)path, FX_OPEN_FOR_READ);
    if (fx != FX_SUCCESS) return 0;
    fx = fx_file_seek(&f, (ULONG)offset);
    if (fx != FX_SUCCESS) {
        (void)fx_file_close(&f);
        return 0;
    }
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
            if (!lock()) {
                /* Re-lock failed (2 s): we must NOT continue into FileX
                 * unlocked next to another user (audit M6 — FAT corruption
                 * class).  The guard must not give a mutex we do not own. */
                guard.held = false;
                return 0;
            }
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

    /* FileX sector cache + FAT + directory entries -> card.  The SD
     * controller commits each WriteBlocks before returning, so after
     * fx_media_flush() every pending write is durable on the card. */
    UINT fx = fx_media_flush(&g_fx_media);
    if (fx != FX_SUCCESS) {
        SERR_LOG(SERR_LFX_FX_OPEN, fx);  /* re-using FX_OPEN code: fx err on flush */
        return 0;
    }
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
    /* Card pulled at runtime (fx_sd_driver flagged SD_LOST after N
     * consecutive command failures): drop to SAFE MODE here, on the 5 s
     * watchdog tick, so cached FAT/dir sectors are never flushed onto a
     * different card.  FxUserRemount() brings a new card back. */
    if (!FxSdIsReady()) {
        LockGuard guard;
        if (guard.held && g_mounted) {
            (void)fx_media_abort(&g_fx_media);
            g_mounted = false;
            g_unflushed_writes = 0;
            printf("[fx_user] *** SD card lost -- FS unmounted (SAFE MODE) ***\r\n");
        }
        return -1;
    }
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
bool g_block_only_open = false;  /* true after FxUserOpenBlockOnly() succeeds */
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

extern "C" int FxUserOpenBlockOnly(void) {
    if (g_fx_mutex == nullptr) {
        g_fx_mutex = xSemaphoreCreateMutex();
        if (g_fx_mutex == nullptr) return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;
    if (g_block_only_open) return 1;
    if (!g_systems_initialized) {
        fx_system_initialize();
        g_systems_initialized = true;
    }

    /* Storage mode exposes the RAW card to the host over MSC, so only
     * the block layer is needed — whatever filesystem the card carries
     * (our FAT32, a PC's exFAT, nothing) is the host's business.  No
     * format is ever implied here. */
    if (!FxSdInit()) {
        SERR_LOG(SERR_LFX_STORAGE_NO_CARD, 0);
        printf("[fx_user] *** storage mode: NO SD CARD -- MSC unavailable ***\r\n");
        return 0;
    }
    g_block_only_open = true;
    printf("[fx_user] SD block layer open for storage MSC (%lu x 512 B)\r\n",
           (unsigned long)FxSdBlockCount());
    return 1;
}

extern "C" int FxUserMscLbaSize(void) {
    return (int)FX_SD_BLOCK_SIZE;
}

extern "C" int FxUserMscLbaCount(void) {
    return (int)FxSdBlockCount();
}

extern "C" int FxUserMscRead(uint32_t lba, uint8_t* buf) {
    if (buf == nullptr) return 0;
    if (lba >= FxSdBlockCount()) {
        SERR_LOG(SERR_LFX_LBA_RANGE, lba);
        return 0;
    }
    if (!g_block_only_open && !g_mounted) {
        SERR_LOG(SERR_LFX_NOT_MOUNTED, 0x01u /* MscRead */);
        return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;  /* SERR_LFX_LOCK_TIMEOUT already logged */
    return FxSdRead(lba, buf, 1);
}

extern "C" int FxUserMscWrite(uint32_t lba, const uint8_t* buf) {
    if (buf == nullptr) return 0;
    if (lba >= FxSdBlockCount()) {
        SERR_LOG(SERR_LFX_LBA_RANGE, lba);
        return 0;
    }
    if (!g_block_only_open && !g_mounted) {
        SERR_LOG(SERR_LFX_NOT_MOUNTED, 0x02u /* MscWrite */);
        return 0;
    }
    LockGuard guard;
    if (!guard.held) return 0;
    return FxSdWrite(lba, buf, 1);
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
