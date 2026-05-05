/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 *
 * LevelX user configuration for SentAI / coralmicro on RT1176 NAND.
 *
 * Activated via -DFX_INCLUDE_USER_DEFINE_FILE / -DLX_INCLUDE_USER_DEFINE_FILE
 * in the libs_levelx CMake target.
 */

#ifndef LX_USER_H
#define LX_USER_H

/* Drop the ThreadX kernel dependency.  We bind any required external
 * synchronisation in the BD adapter (libs/base/fx_nand_driver.cc) to a
 * single FreeRTOS mutex held at the FileX layer. */
#ifndef LX_STANDALONE_ENABLE
#define LX_STANDALONE_ENABLE
#endif

/* Per-instance NAND geometry is set at runtime by the driver_initialize
 * callback (see fx_nand_driver.cc).  Only the tunables that LevelX wants
 * to know at compile time are listed here. */

/* How many physical blocks LevelX may use to store its own metadata.
 * Default 4 is fine for our 448-block user partition. */
#ifndef LX_NAND_FLASH_MAX_METADATA_BLOCKS
#define LX_NAND_FLASH_MAX_METADATA_BLOCKS 4
#endif

#endif /* LX_USER_H */
