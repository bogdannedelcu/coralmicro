/*
 * Copyright 2022 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIBS_SDCARD_SDCARD_H_
#define LIBS_SDCARD_SDCARD_H_

#include <cstdint>
#include <cstddef>

namespace coralmicro {

// SD Card information structure
struct SDCardInfo {
  uint32_t block_count;    // Total number of blocks
  uint32_t block_size;     // Block size in bytes (typically 512)
  uint64_t capacity_bytes; // Total capacity in bytes
  bool is_high_capacity;   // True for SDHC/SDXC cards
  uint8_t manufacturer_id; // Card manufacturer ID
  uint16_t oem_id;         // OEM/Application ID
  char product_name[6];    // Product name string (5 chars + null terminator)
  uint32_t bus_clock_hz;   // Current SD bus clock (host-side), e.g. 50000000 in HS
  uint8_t timing_mode;     // sd_timing_mode_t: 0 SDR12/default, 1 SDR25 high speed, 2 SDR50, 3 SDR104, 4 DDR50
  uint8_t bus_width;       // 1 or 4 data lines
};

// microSD card on USDHC1 (board rev 1.1): block-level access plus a FatFs
// volume on logical drive "2:" (Mount/Unmount/Format).
//
// Init() needs the FreeRTOS scheduler running (USDHC host uses OSA events).
//
// Example usage:
// ```
// if (SDCard::Init()) {
//     SDCardInfo info;
//     if (SDCard::GetCardInfo(&info)) {
//         printf("SD Card: %llu MB\n", info.capacity_bytes / (1024 * 1024));
//     }
//
//     uint8_t buffer[512];
//     if (SDCard::ReadBlocks(0, buffer, 1)) {
//         printf("Read block 0 successfully\n");
//     }
//
//     SDCard::Deinit();
// }
// ```
class SDCard {
 public:
  // Initializes the SD card interface.
  //
  // This function initializes the USDHC1 peripheral, configures clocks and
  // pins, detects the card, and performs SD card initialization sequence.
  //
  // @return True if initialization succeeds, false otherwise.
  // Bounded: equivalent to InitBounded(kInitDeadlineMs) == 1.
  static bool Init();

  // Bounded init (docs/board_rev_1_1/init_audit.md F14): SD_Init() has no deadline of its own and never
  // returns on an empty socket / unwired USDHC, so it runs in a dedicated
  // worker task (static stack, no heap) and the caller waits at most
  // `deadline_ms`.  Returns 1 = card initialised, 0 = SD_Init failed,
  // -1 = deadline exceeded (worker still running; a later call returns its
  // result once it finishes), -2 = a previous init is still running.
  static int InitBounded(uint32_t deadline_ms);
  static constexpr uint32_t kInitDeadlineMs = 5000;

  // Deinitializes the SD card interface and powers down the card.
  static void Deinit();

  // Checks if an SD card is inserted and initialized.
  //
  // @return True if a card is present and initialized, false otherwise.
  static bool IsCardInserted();

  // Reads one or more blocks from the SD card.
  //
  // @param start_block The starting block number to read from.
  // @param buffer Pointer to buffer where data will be stored.
  //               Must be at least block_count * 512 bytes and 32-byte aligned.
  // @param block_count Number of blocks to read.
  // @return True if read succeeds, false otherwise.
  static bool ReadBlocks(uint32_t start_block, uint8_t* buffer,
                        uint32_t block_count);

  // Writes one or more blocks to the SD card.
  //
  // @param start_block The starting block number to write to.
  // @param buffer Pointer to buffer containing data to write.
  //               Must be at least block_count * 512 bytes and 32-byte aligned.
  // @param block_count Number of blocks to write.
  // @return True if write succeeds, false otherwise.
  static bool WriteBlocks(uint32_t start_block, const uint8_t* buffer,
                         uint32_t block_count);

  // Gets information about the SD card.
  //
  // @param info Pointer to SDCardInfo structure to fill.
  // @return True if card info is available, false otherwise.
  static bool GetCardInfo(SDCardInfo* info);

  // Gets the total number of blocks on the SD card.
  //
  // @return The number of blocks, or 0 if no card is present.
  static uint32_t GetBlockCount();

  // Gets the block size in bytes (typically 512).
  //
  // @return The block size in bytes, or 0 if no card is present.
  static uint32_t GetBlockSize();

  // Gets the total capacity of the SD card in bytes.
  //
  // @return The capacity in bytes, or 0 if no card is present.
  static uint64_t GetCapacityBytes();

  // Erases blocks on the SD card.
  //
  // @param start_block The starting block number to erase.
  // @param block_count Number of blocks to erase.
  // @return True if erase succeeds, false otherwise.
  static bool EraseBlocks(uint32_t start_block, uint32_t block_count);

  // Progress breadcrumb of the last Init() (1..10 host/card steps, 100 =
  // worker entered); for diagnosing an init that never returns.
  static uint32_t InitStage();

  // FatFs drive prefix of the card ("2:"); use it in f_open() paths.
  static const char* FatFsDrive();

  // Mounts the FAT filesystem on the SD card (FatFs logical drive "2:").
  // The mount point is informational; FatFs paths are "2:/name".
  //
  // Example:
  // ```
  // SDCard::Mount("/sd");
  // FIL f;
  // f_open(&f, "2:/test.txt", FA_WRITE | FA_CREATE_ALWAYS);
  // ...
  // SDCard::Unmount();
  // ```
  //
  // @param mount_point Path that must start with "/" (e.g. "/sd").
  // @return True if mount succeeds, false otherwise.
  static bool Mount(const char* mount_point);

  // Unmounts the FAT filesystem.
  static bool Unmount();

  // Checks if the filesystem is currently mounted.
  static bool IsMounted();

  // Formats the SD card with FAT32.  WARNING: erases all data on the card!
  static bool Format();
};

}  // namespace coralmicro

#endif  // LIBS_SDCARD_SDCARD_H_
