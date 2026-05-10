/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Public API for the FileX/LevelX user-partition filesystem.
 *
 * Phase 2 of the LittleFS -> FileX migration: this header replaces the
 * raw `lfs_t* LfsUser()` accessor with a typed helper API that does not
 * leak LittleFS structures.  The existing `LfsUser*` helpers in
 * libs/base/filesystem.h are reimplemented as thin wrappers over these
 * primitives, so existing high-level callers (sentai.fs.*, modsentai_fs,
 * etc.) continue to work unchanged.
 *
 * Layering:
 *   sentai.fs.* (MicroPython)  ->  modsentai_fs / modsentai_hal
 *   modsentai_*                ->  coralmicro::LfsUser* helpers
 *   coralmicro::LfsUser*       ->  FxUser* (this header)
 *   FxUser*                    ->  fx_*  (FileX) -> lx_* (LevelX)
 *   lx_*                       ->  fx_nand_driver_* -> Nand_Flash_*
 *
 * Concurrency: every FxUser* call serialises through a single FreeRTOS
 * mutex held inside fx_user_fs.cc, mirroring LittleFS's existing
 * g_lfs_user_mutex contract.  Callers do NOT need their own locking.
 */

#ifndef LIBS_BASE_FX_USER_FS_H_
#define LIBS_BASE_FX_USER_FS_H_

/* Magic constant the caller must pass to destructive operations
 * (`sentai.diag.fx_format()` / FxUserInit(force_format=1) on a
 * mounted volume).  Single source of truth — do NOT redefine in
 * downstream code.  Per embeded.md §F: destructive paths require
 * an explicit unique-magic confirm so accidental reuse cannot
 * trigger them. */
#define FX_DESTRUCTIVE_CONFIRM_MAGIC  0xDEADBEEFu

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

#ifdef __cplusplus
#include <string>
#include <vector>
extern "C" {
#endif

/* ===== Mount / lifecycle ============================================== */

/* Bring the FileX-over-LevelX volume up on the user partition (NAND
 * blocks 76..523).  If the volume is unformatted (first boot or after
 * storage-mode wipe), this auto-formats.
 *
 * `force_format` = true wipes the volume unconditionally — used by the
 *                   "factory reset" path.
 *
 * Idempotent on success: a second call mounts cleanly without
 * re-formatting.  Returns 1 on success, 0 on failure (volume left
 * unmounted; FxUser*() helpers all become no-ops returning errors).
 */
int  FxUserInit(int force_format);

/* Re-open the FileX volume after USB MSC host access.  Use this from
 * the storage-mode exit path; equivalent to a forced unmount + remount
 * but does NOT format. */
int  FxUserRemount(void);

/* Tear down the FileX volume (close media, close LX flash) so the host
 * can take exclusive ownership of the NAND in storage mode.  Safe to
 * call on an unmounted volume. */
void FxUserUnmount(void);

/* Returns 1 if the volume is currently mounted and usable. */
int  FxUserIsMounted(void);

/* Force a FAT-table flush so any pending writes hit NAND.  Used as an
 * explicit "save now" — Phase 3.2 dropped the per-call flush from
 * FxUserWriteFile/AppendFile/Remove for ~1 s/op savings, so callers
 * that need durability before a planned reset must call this. */
int  FxUserSync(void);

/* Idle-driven auto-sync.  Called periodically (every 5 s tick) by
 * CombinedWatchdogTask.  Internally checks: writes-pending > 0 AND
 * last-write older than 2 s.  When both true, calls FxUserSync()
 * synchronously (~200-500 ms).  Bounds the unflushed-data window to
 * ~7 s on an idle board so a USB→battery brownout (Crazyflie BL deck
 * VBAT ~3.7V vs board's 5V need) is unlikely to interrupt an
 * in-flight NAND program.  Returns: 1 sync ran ok, 0 skipped (no
 * pending writes, or activity still hot), -1 sync failed (volume
 * marked unmounted, see SERR log).  Build #1226+. */
int  FxUserMaybeIdleSync(void);

/* ===== Stat ========================================================== */

typedef struct {
    int      exists;     /* 1/0 */
    int      is_dir;     /* 1 if directory, 0 if file */
    uint32_t size;       /* file size in bytes (0 for dirs) */
    uint32_t mtime_s;    /* seconds since boot (best-effort, 0 if unknown) */
} FxStat;

/* Stat a path.  out->exists == 0 if not found OR on error. */
int  FxUserStat(const char* path, FxStat* out);

/* ===== Directory listing ============================================= */

typedef struct {
    char     name[64];   /* basename, NUL-terminated */
    int      is_dir;     /* 1 if directory entry */
    uint32_t size;       /* size in bytes (0 for dirs) */
    uint32_t mtime_s;    /* best-effort */
} FxDirEntry;

/* Per-entry callback for FxUserListDir.  Return 0 to continue, non-zero
 * to abort the iteration. */
typedef int (*FxDirCallback)(const FxDirEntry* entry, void* user);

/* Iterate entries of `path` (which must be an existing directory).
 * Skips "." and "..".  Returns the number of entries successfully
 * visited (including those for which the callback returned non-zero
 * and stopped iteration), or -1 on error.
 *
 * The whole iteration is performed under the FS mutex; callbacks must
 * not call other FxUser* helpers (they would deadlock). */
int  FxUserListDir(const char* path, FxDirCallback cb, void* user);

/* ===== File primitives =============================================== */

int  FxUserFileExists(const char* path);
int  FxUserDirExists(const char* path);

/* Returns file size, or -1 if not found / not a file. */
ssize_t FxUserSize(const char* path);

/* Read up to `size` bytes from the file into `buf`.  Returns number
 * of bytes actually read, or 0 on failure. */
size_t FxUserReadFile(const char* path, uint8_t* buf, size_t size);

/* Write the entire `buf` to `path`, creating or truncating as needed.
 * Returns 1 on success, 0 on failure. */
int  FxUserWriteFile(const char* path, const uint8_t* buf, size_t size);

/* Append `buf` to `path`, creating the file if it does not exist.
 * Used by the chunked REPL uploader. */
int  FxUserAppendFile(const char* path, const uint8_t* buf, size_t size);

/* Remove a file or empty directory.  Returns 0 on success, negative
 * on error (matches LittleFS's lfs_remove contract for compatibility). */
int  FxUserRemove(const char* path);

/* Create a single directory.  Returns 1 on success or already-exists,
 * 0 on error. */
int  FxUserMakeDir(const char* path);

/* Create a directory and all missing parent directories.
 * Equivalent to `mkdir -p`. */
int  FxUserMakeDirs(const char* path);

/* Rename a file or directory.  Returns 0 on success, negative on
 * error.  Used by sentai_lfs_task boot-log rotation. */
int  FxUserRename(const char* from, const char* to);

/* ===== Diagnostics =================================================== */

/* NAND BD adapter counters (cumulative since boot).  Pointers may be NULL. */
void fx_nand_driver_get_stats(uint32_t* reads, uint32_t* writes,
                              uint32_t* erases,
                              uint32_t* read_errors, uint32_t* write_errors,
                              uint32_t* erase_errors,
                              uint32_t* bad_blocks);

/* FileX media-level counters. */
typedef struct {
    uint32_t mounted;
    uint32_t free_clusters;
    uint32_t total_clusters;
    uint32_t bytes_per_sector;
    uint32_t sectors_per_cluster;
    uint32_t mount_failures;     /* fx_media_open returned non-success */
    uint32_t format_count;       /* number of times we ran fx_media_format this boot */
} FxUserStats;

void FxUserGetStats(FxUserStats* out);

/* ===== Compile-time MSC geometry constants ==========================
 *
 * Mirrors fx_nand_driver.h so libs/msc_ums/msc_ums.cc can size its
 * static URB buffers without pulling in lx_api.h. */
/* Sector size: matches FX_NAND_BYTES_PER_PAGE = 2048 (full data area;
 * physical NAND OOB carries the LevelX 64-byte spare separately).
 * Power-of-2, so Linux usb-storage accepts the LUN and FAT mounts
 * directly via `mount /dev/sda /mnt`. */
#define FX_USER_LBA_SIZE     2048u
#define FX_USER_LBA_COUNT    28160u  /* (448 - 8) * 64 reserved blocks  */

/* ===== Storage-mode MSC routing =====================================
 *
 * In storage mode the firmware exposes the user partition as a USB MSC
 * LUN.  Phase 2 routes those reads/writes through LevelX so the host
 * sees a real FAT volume at LBA 0 (instead of raw NAND with LX
 * metadata, which is unmountable).  The MSC handler in
 * libs/msc_ums/msc_ums.cc calls these helpers; FxUser* file APIs are
 * NOT used while storage mode is active. */

/* Open ONLY the LevelX flash on the user partition (no FileX media
 * mount).  Used by the storage-mode boot path before MSC starts.
 * Returns 1 on success, 0 on failure.  Idempotent: a second call on an
 * already-open LX instance returns 1 without re-opening. */
int FxUserOpenLxOnly(void);

/* MSC LBA primitives.  Each LBA is FxUserMscLbaSize() bytes, total
 * count is FxUserMscLbaCount().  Reads/writes return 1 on success,
 * 0 on failure (LevelX error logged via SERR_LFX_*). */
int FxUserMscLbaSize(void);
int FxUserMscLbaCount(void);
int FxUserMscRead(uint32_t lba, uint8_t* buf);
int FxUserMscWrite(uint32_t lba, const uint8_t* buf);

/* ===== Storage-mode debug log =======================================
 *
 * Phase 2 quirk: storage mode unmounts FileX (host owns NAND), which
 * means we can't write a normal log file while debugging MSC.  This
 * mechanism captures events into a 16 KB SDRAM ring that survives
 * NVIC_SystemReset, then dumps them to /log/storage_debug.log on the
 * NEXT default-mode boot. */

/* Append a printf-style event to the storage debug ring.  No-op if
 * the buffer is full.  Safe from any task context (uses non-blocking
 * test-and-set lock). */
void sentai_storage_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/* Mark the start of a new log session (called from the storage-mode
 * boot path after FxUserOpenLxOnly succeeds). */
void sentai_storage_log_init(void);

/* If the SDRAM buffer holds unread storage-debug data, append it to
 * /log/storage_debug.log on the user FS, then reset the buffer.
 * Called from the default-mode boot path AFTER FxUserInit succeeds.
 * Returns the number of bytes flushed (0 if nothing to flush). */
int sentai_storage_log_flush_to_fs(void);

/* ===== Bench helpers (non-destructive) =============================== */

typedef struct {
    int      ok;
    uint32_t entries;
    uint32_t time_list_ms;
    uint32_t time_size_ms;
} FxBenchResult;

/* Walk the root directory recording per-entry latency.  Non-destructive,
 * safe to call any time after FxUserInit succeeded. */
int  FxUserBenchRoot(FxBenchResult* out);

#ifdef __cplusplus
}  /* extern "C" */

/* C++ overloads that mirror the LfsUser*ReadFile/WriteFile signatures
 * used by libs/base callers.  Implemented in fx_user_fs.cc. */
namespace coralmicro_fx {
bool FxUserReadFile(const char* path, std::vector<uint8_t>* buf);
bool FxUserReadFile(const char* path, std::string* str);
bool FxUserWriteFile(const char* path, const std::string& str);
}

#endif

#endif  /* LIBS_BASE_FX_USER_FS_H_ */
