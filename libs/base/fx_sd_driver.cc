/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * SD-card block layer + FileX media driver.  See fx_sd_driver.h.
 *
 * Per embeded.md NASA/JPL discipline:
 *   - static allocation only (one non-cacheable bounce buffer);
 *   - every SD return checked, surfaced through the 1/0 contract AND a
 *     structured SERR_LFX_SD_* log entry;
 *   - no unbounded loops: transfers are chunked by the bounce size.
 */

#include "libs/base/fx_sd_driver.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" {
#include "fx_api.h"
}

#include "libs/sdcard/sdcard.h"
#include "examples/sentai_runtime/sentai_error.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_common.h"

/* FileX utility (fx_partition_offset_calculate.c): given a boot sector,
 * returns the start/size of `partition` if the sector is an MBR, or
 * start 0 if the sector itself is a volume boot record. */
extern "C" UINT _fx_partition_offset_calculate(void* partition_sector,
                                               UINT partition,
                                               ULONG* partition_start,
                                               ULONG* partition_size);

namespace {

/* USDHC moves data by ADMA2 with D-cache maintenance done by the SDK host
 * (kSDMMCHOST_CacheControlRWBuffer).  That maintenance is only safe on
 * 32-byte-aligned ranges, so unaligned callers (MicroPython heap buffers,
 * FileX's own per-file cache, USB URB buffers) are staged through this
 * non-cacheable buffer, kBounceBlocks blocks at a time. */
constexpr uint32_t kBounceBlocks = 8u;   /* 4 KB */
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t s_bounce[kBounceBlocks * FX_SD_BLOCK_SIZE], 32);

volatile bool     s_ready = false;
volatile bool     s_abort = false;
bool              s_card_was_init = false; /* SDCard::Init() ran once; re-init needs Deinit first */
uint32_t          s_part_start = 0;       /* blocks; 0 = superfloppy */
uint32_t          s_part_size  = 0;       /* blocks; 0 = whole card   */
uint32_t          s_consecutive_err = 0;

uint32_t s_stat_reads = 0, s_stat_writes = 0;
uint32_t s_stat_read_err = 0, s_stat_write_err = 0, s_stat_bounce = 0;

inline bool dma_safe(const void* p) {
  return (reinterpret_cast<uintptr_t>(p) & 31u) == 0u;
}

/* Error accounting: a few consecutive command failures mean the card is
 * gone (no card-detect pin on this board).  Mark the layer not ready so
 * FileX stops hammering the host (each failed command costs a hardware
 * data-timeout) and fx_user_fs can drop to SAFE MODE instead of flushing
 * stale FAT sectors onto whatever card is inserted next. */
inline void note_error(void) {
  if (++s_consecutive_err == FX_SD_MAX_CONSECUTIVE_ERRORS && s_ready) {
    s_ready = false;
    SERR_LOG(SERR_LFX_SD_LOST, s_consecutive_err);
    printf("[fx_sd] %lu consecutive SD errors -> card considered LOST\r\n",
           (unsigned long)s_consecutive_err);
  }
}
inline void note_ok(void) { s_consecutive_err = 0; }

}  // namespace

extern "C" {

int FxSdInit(void) {
  if (s_ready) return 1;
  s_consecutive_err = 0;
  s_abort = false;
  if (s_card_was_init) {
    /* A bare second SD_Init() on the same host deadlocked once (see
     * libs/nxp CMake note on fsl_sd_disk.c); always tear down first. */
    coralmicro::SDCard::Deinit();
    s_card_was_init = false;
  }
  {
    const int r = coralmicro::SDCard::InitBounded(coralmicro::SDCard::kInitDeadlineMs);
    if (r != 1) {
      if (r == -1) SERR_LOG(SERR_LFX_SD_INIT_TIMEOUT, coralmicro::SDCard::kInitDeadlineMs);
      else         SERR_LOG(SERR_LFX_SD_INIT_FAIL, (uint32_t)(-r));
      return 0;
    }
  }
  s_card_was_init = true;
  if (coralmicro::SDCard::GetBlockSize() != FX_SD_BLOCK_SIZE ||
      coralmicro::SDCard::GetBlockCount() == 0u) {
    /* Only 512-byte-block cards are supported (every SDHC/SDXC card). */
    SERR_LOG(SERR_LFX_SD_INIT_FAIL, coralmicro::SDCard::GetBlockSize());
    coralmicro::SDCard::Deinit();
    return 0;
  }
  s_part_start = 0;
  s_part_size = 0;
  s_ready = true;
  printf("[fx_sd] card ready: %lu blocks x %u B (%lu MB)\r\n",
         (unsigned long)coralmicro::SDCard::GetBlockCount(),
         (unsigned)FX_SD_BLOCK_SIZE,
         (unsigned long)(coralmicro::SDCard::GetCapacityBytes() >> 20));
  return 1;
}

int FxSdIsReady(void) { return s_ready ? 1 : 0; }

void FxSdAbortSet(int on) { s_abort = (on != 0); }

uint32_t FxSdBlockCount(void) {
  return s_ready ? coralmicro::SDCard::GetBlockCount() : 0u;
}

uint32_t FxSdPartitionStart(void) { return s_part_start; }

void FxSdResetPartition(void) {
  s_part_start = 0;
  s_part_size = 0;
}

/* Reject out-of-range requests before they reach the card: a corrupt BPB
 * can make FileX ask for sectors past the end.  The SDK host has NO
 * software timeout on a transfer (it waits for the USDHC data-timeout
 * interrupt), so every bad command would otherwise cost a hardware
 * timeout — thousands of them look like a hang on the boot path.  The
 * log is rate-limited: first kErrLogBudget occurrences, then a count. */
constexpr uint32_t kErrLogBudget = 8u;
static uint32_t s_err_logged = 0;
static inline void log_sd_error(uint32_t code, uint32_t val) {
  if (s_err_logged < kErrLogBudget) {
    s_err_logged++;
    SERR_LOG(code, val);
  } else if (s_err_logged == kErrLogBudget) {
    s_err_logged++;
    printf("[fx_sd] further SD errors suppressed from the log\r\n");
  }
}
static inline bool lba_range_ok(uint32_t lba, uint32_t count) {
  const uint32_t total = coralmicro::SDCard::GetBlockCount();
  if (count > total || lba > total - count) {
    log_sd_error(SERR_LFX_LBA_RANGE, lba);
    return false;
  }
  return true;
}

int FxSdRead(uint32_t lba, uint8_t* buf, uint32_t count) {
  if (!s_ready || s_abort || buf == nullptr || count == 0u) return 0;
  if (!lba_range_ok(lba, count)) { s_stat_read_err++; return 0; }
  if (dma_safe(buf)) {
    if (!coralmicro::SDCard::ReadBlocks(lba, buf, count)) {
      s_stat_read_err++;
      log_sd_error(SERR_LFX_SD_READ_FAIL, lba);
      note_error();
      return 0;
    }
    note_ok();
    s_stat_reads += count;
    return 1;
  }
  /* Bounced path, bounded by kBounceBlocks per iteration. */
  while (count != 0u) {
    if (s_abort) return 0;
    uint32_t n = (count > kBounceBlocks) ? kBounceBlocks : count;
    if (!coralmicro::SDCard::ReadBlocks(lba, s_bounce, n)) {
      s_stat_read_err++;
      log_sd_error(SERR_LFX_SD_READ_FAIL, lba);
      note_error();
      return 0;
    }
    note_ok();
    std::memcpy(buf, s_bounce, n * FX_SD_BLOCK_SIZE);
    s_stat_bounce++;
    s_stat_reads += n;
    lba += n;
    buf += n * FX_SD_BLOCK_SIZE;
    count -= n;
  }
  return 1;
}

int FxSdWrite(uint32_t lba, const uint8_t* buf, uint32_t count) {
  if (!s_ready || s_abort || buf == nullptr || count == 0u) return 0;
  if (!lba_range_ok(lba, count)) { s_stat_write_err++; return 0; }
  if (dma_safe(buf)) {
    if (!coralmicro::SDCard::WriteBlocks(lba, buf, count)) {
      s_stat_write_err++;
      log_sd_error(SERR_LFX_SD_WRITE_FAIL, lba);
      note_error();
      return 0;
    }
    note_ok();
    s_stat_writes += count;
    return 1;
  }
  while (count != 0u) {
    if (s_abort) return 0;
    uint32_t n = (count > kBounceBlocks) ? kBounceBlocks : count;
    std::memcpy(s_bounce, buf, n * FX_SD_BLOCK_SIZE);
    if (!coralmicro::SDCard::WriteBlocks(lba, s_bounce, n)) {
      s_stat_write_err++;
      log_sd_error(SERR_LFX_SD_WRITE_FAIL, lba);
      note_error();
      return 0;
    }
    note_ok();
    s_stat_bounce++;
    s_stat_writes += n;
    lba += n;
    buf += n * FX_SD_BLOCK_SIZE;
    count -= n;
  }
  return 1;
}

/* ===== FileX media driver ============================================ */

void fx_sd_driver(void* fx_media_ptr) {
  FX_MEDIA* media_ptr = static_cast<FX_MEDIA*>(fx_media_ptr);
  UCHAR* buffer = static_cast<UCHAR*>(media_ptr->fx_media_driver_buffer);

  switch (media_ptr->fx_media_driver_request) {
    /* ONE offset only.  PC formatters write hidden_sectors == partition
     * start, so adding both (as the FileX RAM-driver template does for a
     * media whose sector 0 IS the VBR) would land every FAT/dir/data
     * sector at 2*start.  validate_boot_sector() check 10 rejects a VBR
     * whose hidden_sectors disagrees with the MBR start. */
    case FX_DRIVER_READ: {
      ULONG lba = media_ptr->fx_media_driver_logical_sector + s_part_start;
      media_ptr->fx_media_driver_status =
          FxSdRead(lba, buffer, media_ptr->fx_media_driver_sectors)
              ? FX_SUCCESS : FX_IO_ERROR;
      break;
    }

    case FX_DRIVER_WRITE: {
      ULONG lba = media_ptr->fx_media_driver_logical_sector + s_part_start;
      media_ptr->fx_media_driver_status =
          FxSdWrite(lba, buffer, media_ptr->fx_media_driver_sectors)
              ? FX_SUCCESS : FX_IO_ERROR;
      break;
    }

    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
    case FX_DRIVER_RELEASE_SECTORS:   /* no TRIM on this path */
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    case FX_DRIVER_INIT:
      /* The card does its own free-sector management; FileX must not
       * issue RELEASE_SECTORS for every freed cluster. */
      media_ptr->fx_media_driver_free_sector_update = FX_FALSE;
      media_ptr->fx_media_driver_write_protect = FX_FALSE;
      media_ptr->fx_media_driver_status = s_ready ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_UNINIT:
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    case FX_DRIVER_BOOT_READ: {
      /* Block 0 may be an MBR (card formatted on a PC) or the VBR itself
       * (FileX superfloppy format).  Resolve partition 0 and re-read the
       * real boot record so fx_media_open() sees a BPB. */
      if (!FxSdRead(0, buffer, 1)) {
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
      }
      ULONG start = 0, size = 0;
      if (_fx_partition_offset_calculate(buffer, 0, &start, &size) != FX_SUCCESS) {
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
      }
      s_part_start = start;
      s_part_size = size;
      if (start != 0u && !FxSdRead(start, buffer, 1)) {
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
      }
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;
    }

    case FX_DRIVER_BOOT_WRITE:
      media_ptr->fx_media_driver_status =
          FxSdWrite(s_part_start, buffer, 1) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    default:
      media_ptr->fx_media_driver_status = FX_IO_ERROR;
      break;
  }
  if (media_ptr->fx_media_driver_status != FX_SUCCESS) {
    /* Cold path: one line per failed request so a format/mount failure
     * names the exact FileX request + sector instead of a bare FX_IO_ERROR. */
    printf("[fx_sd] driver req=%u lba=%lu n=%lu -> status %u\r\n",
           (unsigned)media_ptr->fx_media_driver_request,
           (unsigned long)media_ptr->fx_media_driver_logical_sector,
           (unsigned long)media_ptr->fx_media_driver_sectors,
           (unsigned)media_ptr->fx_media_driver_status);
  }
}

void fx_sd_driver_get_stats(uint32_t* reads, uint32_t* writes,
                            uint32_t* read_errors, uint32_t* write_errors,
                            uint32_t* bounce_copies) {
  if (reads) *reads = s_stat_reads;
  if (writes) *writes = s_stat_writes;
  if (read_errors) *read_errors = s_stat_read_err;
  if (write_errors) *write_errors = s_stat_write_err;
  if (bounce_copies) *bounce_copies = s_stat_bounce;
}

}  // extern "C"
