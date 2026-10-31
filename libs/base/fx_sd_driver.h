/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * SD-card block layer for the SentAI user filesystem.
 *
 * 2026-10-30: the user FS moved from the NAND FileX/LevelX partition to
 * the microSD card (USDHC1, see libs/sdcard).  LevelX is NOT used — the
 * card's own controller does wear-leveling and bad-block management, so
 * FileX talks to 512-byte card blocks directly through fx_sd_driver().
 *
 * Layering:
 *   FxUser*  (fx_user_fs.cc)  ->  fx_*  (FileX)  ->  fx_sd_driver()
 *   msc_ums  (USB storage)    ->  FxUserMsc*     ->  FxSdRead/FxSdWrite
 *   FxSd*                      ->  coralmicro::SDCard (libs/sdcard)
 *
 * All entry points are post-scheduler only: the USDHC host driver blocks
 * on FreeRTOS events.  Callers serialise through the fx_user_fs.cc mutex;
 * this layer adds no locking of its own.
 */

#ifndef LIBS_BASE_FX_SD_DRIVER_H_
#define LIBS_BASE_FX_SD_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Card block size used by both FileX and MSC. */
#define FX_SD_BLOCK_SIZE  512u

/* Bring the card up (power, host, SD init).  Bounded; idempotent.
 * Returns 1 when the card answers, 0 otherwise (no card / init error —
 * the error is logged via SERR_LFX_SD_INIT_FAIL). */
int FxSdInit(void);

/* 1 while the card is considered present and healthy.  Cleared by
 * FxSdInit() failure and by FX_SD_MAX_CONSECUTIVE_ERRORS failed commands
 * in a row (card pulled at runtime — there is no card-detect switch);
 * FxSdInit() then re-runs the full SD bring-up on the next call. */
int FxSdIsReady(void);
#define FX_SD_MAX_CONSECUTIVE_ERRORS 3u

/* Abort switch for a FileX walk that will not terminate on a corrupt
 * volume: while set, every FxSdRead/FxSdWrite fails immediately, so the
 * walk unwinds through FileX's own error paths and the calling task
 * returns normally (no vTaskDelete of a task that may hold the SDK host
 * mutex).  Used by fx_user_fs reap_stuck_mount(). */
void FxSdAbortSet(int on);

/* Total 512-byte blocks on the card (0 if not ready). */
uint32_t FxSdBlockCount(void);

/* Read / write `count` consecutive 512-byte blocks.  Any buffer alignment
 * is accepted: 32-byte-aligned buffers go straight to DMA, others are
 * bounced through a non-cacheable staging buffer.  Return 1 on success,
 * 0 on failure (logged via SERR_LFX_SD_READ_FAIL / _WRITE_FAIL). */
int FxSdRead(uint32_t lba, uint8_t* buf, uint32_t count);
int FxSdWrite(uint32_t lba, const uint8_t* buf, uint32_t count);

/* FileX media driver entry point (FX_MEDIA.fx_media_driver_entry).
 * Handles an MBR-partitioned card transparently: FX_DRIVER_BOOT_READ
 * resolves partition 0 and every later sector request is offset by its
 * start.  A card without an MBR ("superfloppy", what FxUser format
 * produces) has offset 0. */
void fx_sd_driver(void* fx_media_ptr);

/* Partition start (in blocks) resolved by the last FX_DRIVER_BOOT_READ. */
uint32_t FxSdPartitionStart(void);

/* Forget any partition offset learnt from an MBR.  fx_user_fs calls it
 * before fx_media_format(): the board formats the card as a superfloppy
 * (VBR at block 0), so the new boot sector must land at block 0 even if
 * the previous content was an MBR-partitioned (e.g. exFAT) card. */
void FxSdResetPartition(void);

/* Cumulative counters since boot (pointers may be NULL). */
void fx_sd_driver_get_stats(uint32_t* reads, uint32_t* writes,
                            uint32_t* read_errors, uint32_t* write_errors,
                            uint32_t* bounce_copies);

#ifdef __cplusplus
}
#endif

#endif  /* LIBS_BASE_FX_SD_DRIVER_H_ */
