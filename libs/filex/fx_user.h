/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 *
 * FileX user configuration for SentAI / coralmicro.
 *
 * Activated via -DFX_INCLUDE_USER_DEFINE_FILE in the libs_filex CMake
 * target.  This file is consumed by FileX's generic port (fx_port.h).
 */

#ifndef FX_USER_H
#define FX_USER_H

/* Drop the ThreadX kernel dependency.  FileX's standalone path:
 *   - replaces FX_PROTECT/FX_UNPROTECT with no-ops (we wrap fx_* calls
 *     in a single FreeRTOS mutex at the call-site layer instead);
 *   - removes the FileX timer (no automatic FAT timestamp updates);
 *   - removes caller-context checks tied to TX_THREAD pointers. */
#ifndef FX_STANDALONE_ENABLE
#define FX_STANDALONE_ENABLE
#endif

/* Long-name support.  Default of 256 is wasteful given our paths peak at
 * something like /diags/sNNN_xxx/i00_camN.jpg (~32 chars).  64 is plenty
 * and saves stack footprint in path-walking calls. */
#ifndef FX_MAX_LONG_NAME_LEN
#define FX_MAX_LONG_NAME_LEN  64
#endif
#ifndef FX_MAX_LAST_NAME_LEN
#define FX_MAX_LAST_NAME_LEN  64
#endif

/* In-memory FAT sector cache.  Power-of-2 only; minimum 2.  8 sectors at
 * 2 KB/sector = 16 KB. */
#ifndef FX_MAX_SECTOR_CACHE
#define FX_MAX_SECTOR_CACHE   8
#endif

/* Enable the fault-tolerant journaling extension.  This is critical: FAT
 * is not atomic by design, and a power-loss during a directory update
 * can corrupt the volume.  With FX_ENABLE_FAULT_TOLERANT, FileX maintains
 * a redo log in a reserved cluster so directory operations replay
 * cleanly on the next mount. */
#ifndef FX_ENABLE_FAULT_TOLERANT
#define FX_ENABLE_FAULT_TOLERANT
#endif

/* Tell the fault-tolerant log where to live (default works on FAT12 but
 * may collide with FAT32 boot sector layout — keep the sample default
 * which targets sector 116 of cluster 0). */
#ifndef FX_FAULT_TOLERANT_BOOT_INDEX
#define FX_FAULT_TOLERANT_BOOT_INDEX  116
#endif

/* The board has no battery-backed RTC for FileX; skip per-write timestamp
 * computation.  FX_NO_TIMER is implied by FX_STANDALONE_ENABLE but we
 * spell it out for clarity. */
#ifndef FX_NO_TIMER
#define FX_NO_TIMER
#endif

#endif /* FX_USER_H */
