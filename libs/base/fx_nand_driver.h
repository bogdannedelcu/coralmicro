/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * LevelX NAND driver adapter for the RT1176 NAND on the SentAI/Coral
 * Micro board.  Bridges LevelX's logical-sector model to NXP's
 * Nand_Flash_* page-level API exposed by the rt1176-sdk.
 *
 * Geometry (validated against the existing LittleFS user partition):
 *   total NAND blocks                       : 1024 (~128 MB raw)
 *   pages per block                         : 64
 *   bytes per physical page                 : 2048 (no OOB exposed)
 *   LevelX-managed range (user partition)   : blocks 76..523 (448 blocks)
 *   bytes_per_page reported to LevelX       : 2016
 *   spare_bytes_per_page (emulated)         : 32  (carved from the end
 *                                                  of each 2 KB page)
 *
 * The NXP driver does not expose NAND OOB / spare reads or writes, so we
 * emulate per-page metadata by reserving the last 32 bytes of every
 * physical page.  LevelX always submits data + spare together via the
 * pages_write callback (see the survey done before this commit), which
 * makes a single Page_Program call per page sufficient.
 */

#ifndef LIBS_BASE_FX_NAND_DRIVER_H_
#define LIBS_BASE_FX_NAND_DRIVER_H_

#include "lx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Geometry constants exported for the consumers (FxUser layer + MP
 * binding).  `FX_NAND_USER_BASE_BLOCK` is identical to the LittleFS
 * user-partition base block — the two filesystems are MUTUALLY
 * EXCLUSIVE on the NAND, only one may own the range at a time.
 *
 * Page layout (Phase 2.1, 2026-04-28):
 *   - 2048 bytes data (FAT sector, exposed as power-of-2 LBA to MSC).
 *   - 64 bytes physical OOB used as LevelX per-page spare (page-type +
 *     logical-sector mapping + ECC backstop).
 *   - Total bytes touched per Nand_Flash_Read/Program: 2112.
 *
 * Real NAND parts (Winbond W25N01GVZEIG and similar 2 KB-page parts)
 * carry 64 bytes of physical OOB.  Earlier we emulated spare by
 * carving 32 bytes off the END of the data area, which left only 2016
 * bytes per sector — not a power of 2, and Linux usb-storage refused
 * to mount.  Using the chip's actual OOB lets us keep the full 2048
 * bytes per LX/FAT sector without losing capacity. */
#define FX_NAND_BYTES_PER_PAGE    2048u  /* data area: power-of-2 sector */
#define FX_NAND_SPARE_PER_PAGE    64u    /* physical NAND OOB           */
#define FX_NAND_PAGE_RAW_BYTES    (FX_NAND_BYTES_PER_PAGE + FX_NAND_SPARE_PER_PAGE)
#define FX_NAND_PAGES_PER_BLOCK   64u
#define FX_NAND_USER_BASE_BLOCK   76u
#define FX_NAND_USER_BLOCK_COUNT  448u

/* LevelX driver_initialize callback.  Pass this as the third argument
 * to lx_nand_flash_open_extended(). */
UINT  fx_nand_driver_initialize(LX_NAND_FLASH *nand_flash);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* LIBS_BASE_FX_NAND_DRIVER_H_ */
