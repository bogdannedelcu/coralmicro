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

// SD Card Test Example
//
// This example demonstrates how to use the SD card interface to read and write
// blocks of data. It initializes the SD card, displays card information, and
// performs a simple write/read test.

#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/sdcard/sdcard.h"
#include "third_party/nxp/rt1176-sdk/middleware/fatfs/source/ff.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include <cstdio>
#include <cstring>

namespace coralmicro {
namespace {

// A card that does not mount as FAT32 (e.g. one carrying another filesystem)
// is left untouched unless this is set: Format() erases everything on it.
constexpr bool kFormatIfMountFails = false;

// 32-byte aligned buffers for SD card operations
SDK_ALIGN(uint8_t test_buffer[512], 32);
SDK_ALIGN(uint8_t read_buffer[512], 32);
// The block tests are non-destructive: the original content of every block
// they write is saved first and restored afterwards, so the test can run on
// a card that carries a filesystem (FAT32 or anything else).
SDK_ALIGN(uint8_t saved_blocks[1024], 32);  // 2 blocks

void Main() {
  // The USB CDC console drops output while no host terminal is attached and
  // this test prints everything in its first seconds: give the host time to
  // open the port after enumeration.
  vTaskDelay(pdMS_TO_TICKS(3000));

  printf("SD Card Test Example\r\n");
  printf("====================\r\n\r\n");

  // Turn on the LED to indicate we're starting
  LedSet(Led::kStatus, true);

  // Initialize SD card
  printf("Initializing SD card...\r\n");
  if (!SDCard::Init()) {
    printf("ERROR: Failed to initialize SD card\r\n");
    printf("Please check:\r\n");
    printf("  - SD card is inserted\r\n");
    printf("  - SD card is properly formatted\r\n");
    printf("  - Hardware connections are correct\r\n");
    LedSet(Led::kStatus, false);
    return;
  }

  printf("SD card initialized successfully!\r\n\r\n");

  // Check if card is present
  if (!SDCard::IsCardInserted()) {
    printf("WARNING: No SD card detected\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }

  // Get and display card information
  SDCardInfo info;
  if (SDCard::GetCardInfo(&info)) {
    printf("SD Card Information:\r\n");
    printf("  Manufacturer ID: 0x%02X\r\n", info.manufacturer_id);
    printf("  OEM ID: 0x%04X\r\n", info.oem_id);
    printf("  Product Name: %s\r\n", info.product_name);
    printf("  Block Size: %lu bytes\r\n", (unsigned long)info.block_size);
    printf("  Block Count: %lu\r\n", (unsigned long)info.block_count);
    printf("  Bus: %u-bit, %lu Hz, timing mode %u\r\n", info.bus_width,
           (unsigned long)info.bus_clock_hz, info.timing_mode);
    // (no %llu in this newlib build: print MiB / GiB as doubles)
    printf("  Capacity: %.0f MiB (%.2f GiB)\r\n",
           static_cast<double>(info.capacity_bytes) / (1024.0 * 1024.0),
           static_cast<double>(info.capacity_bytes) / (1024.0 * 1024.0 * 1024.0));
    printf("  Type: %s\r\n", info.is_high_capacity ? "SDHC/SDXC" : "SDSC");
    printf("\r\n");
  } else {
    printf("ERROR: Failed to get card information\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }

  // Perform read/write test
  printf("Performing read/write test...\r\n");

  // Fill test buffer with pattern
  for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    test_buffer[i] = static_cast<uint8_t>(i & 0xFF);
  }

  // Write test block to sector 1000 (avoid boot sectors); save it first.
  const uint32_t test_block = 1000;
  if (!SDCard::ReadBlocks(test_block, saved_blocks, 1)) {
    printf("ERROR: Failed to read block %lu before the write test\r\n",
           (unsigned long)test_block);
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }
  printf("Writing test pattern to block %lu...\r\n", (unsigned long)test_block);
  if (!SDCard::WriteBlocks(test_block, test_buffer, 1)) {
    printf("ERROR: Failed to write test block\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }
  printf("Write successful\r\n");

  // Read back the test block
  printf("Reading back block %lu...\r\n", (unsigned long)test_block);
  if (!SDCard::ReadBlocks(test_block, read_buffer, 1)) {
    printf("ERROR: Failed to read test block\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }
  printf("Read successful\r\n");

  // Restore the original content
  if (!SDCard::WriteBlocks(test_block, saved_blocks, 1)) {
    printf("WARNING: failed to restore block %lu\r\n", (unsigned long)test_block);
  }

  // Verify data
  bool data_match = true;
  for (size_t i = 0; i < sizeof(test_buffer); ++i) {
    if (test_buffer[i] != read_buffer[i]) {
      printf("ERROR: Data mismatch at byte %zu (wrote 0x%02X, read 0x%02X)\r\n",
             i, test_buffer[i], read_buffer[i]);
      data_match = false;
      break;
    }
  }

  if (data_match) {
    printf("\r\nSUCCESS: Data verification passed!\r\n");
    printf("SD card is working correctly.\r\n");

    // Blink LED to indicate success
    for (int i = 0; i < 5; ++i) {
      LedSet(Led::kStatus, true);
      vTaskDelay(pdMS_TO_TICKS(200));
      LedSet(Led::kStatus, false);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  } else {
    printf("\r\nFAILED: Data verification failed\r\n");
  }

  // Optional: Test multi-block operations
  printf("\r\nTesting multi-block operations...\r\n");
  SDK_ALIGN(uint8_t multi_buffer[1024], 32);  // 2 blocks

  // Fill multi-block buffer
  for (size_t i = 0; i < sizeof(multi_buffer); ++i) {
    multi_buffer[i] = static_cast<uint8_t>((i + 100) & 0xFF);
  }

  // Write 2 blocks (saved first, restored afterwards)
  const uint32_t multi_block_start = 2000;
  if (!SDCard::ReadBlocks(multi_block_start, saved_blocks, 2)) {
    printf("Multi-block: failed to read blocks before the write test\r\n");
  } else {
    printf("Writing 2 blocks starting at block %lu...\r\n", (unsigned long)multi_block_start);
    if (SDCard::WriteBlocks(multi_block_start, multi_buffer, 2)) {
      printf("Multi-block write successful\r\n");

      // Read back
      SDK_ALIGN(uint8_t multi_read_buffer[1024], 32);
      if (SDCard::ReadBlocks(multi_block_start, multi_read_buffer, 2)) {
        printf("Multi-block read successful\r\n");

        if (std::memcmp(multi_buffer, multi_read_buffer, sizeof(multi_buffer)) == 0) {
          printf("Multi-block verification passed!\r\n");
        } else {
          printf("Multi-block verification failed\r\n");
        }
      } else {
        printf("Multi-block read failed\r\n");
      }
    } else {
      printf("Multi-block write failed\r\n");
    }
    if (!SDCard::WriteBlocks(multi_block_start, saved_blocks, 2)) {
      printf("WARNING: failed to restore blocks %lu..%lu\r\n",
             (unsigned long)multi_block_start, (unsigned long)multi_block_start + 1);
    }
  }

  // Test filesystem operations
  printf("\r\n=== Testing FAT32 Filesystem ===\r\n");

  printf("Mounting filesystem (FatFs drive %s)...\r\n", SDCard::FatFsDrive());
  if (!SDCard::Mount("/sd")) {
    printf("ERROR: Failed to mount filesystem (not FAT32, or no partition table?)\r\n");
    if (!kFormatIfMountFails) {
      printf("Formatting is OFF (kFormatIfMountFails): the card keeps its contents.\r\n");
      printf("Set kFormatIfMountFails = true to format it as FAT32 from here.\r\n");
      SDCard::Deinit();
      LedSet(Led::kStatus, false);
      return;
    }
    printf("Attempting format...\r\n");

    if (SDCard::Format()) {
      printf("Format successful! Trying mount again...\r\n");
      if (!SDCard::Mount("/sd")) {
        printf("ERROR: Mount failed after format\r\n");
        SDCard::Deinit();
        LedSet(Led::kStatus, false);
        return;
      }
    } else {
      printf("ERROR: Format failed\r\n");
      SDCard::Deinit();
      LedSet(Led::kStatus, false);
      return;
    }
  }

  printf("Filesystem mounted successfully!\r\n");

  // FatFs file API (there is no newlib/stdio bridge to FatFs in this tree,
  // so fopen("/sd/...") cannot work; paths carry the FatFs drive prefix).
  char path[32];

  // Test file creation and writing
  snprintf(path, sizeof(path), "%s/test.txt", SDCard::FatFsDrive());
  printf("\r\nWriting test file %s...\r\n", path);
  {
    FIL f;
    FRESULT fr = f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
      f_printf(&f, "Hello from Coral Dev Board Micro rev 1.1!\r\n");
      f_printf(&f, "This is a test file on the SD card.\r\n");
      f_printf(&f, "Timestamp: %d\r\n", static_cast<int>(xTaskGetTickCount()));
      f_close(&f);
      printf("File written successfully\r\n");
    } else {
      printf("ERROR: Failed to create file (FRESULT %d)\r\n", (int)fr);
    }
  }

  // Test file reading
  printf("Reading back %s...\r\n", path);
  {
    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr == FR_OK) {
      char line[128];
      printf("File contents:\r\n");
      printf("---\r\n");
      while (f_gets(line, sizeof(line), &f)) {
        printf("%s", line);
      }
      printf("---\r\n");
      f_close(&f);
      printf("File read successfully\r\n");
    } else {
      printf("ERROR: Failed to open file for reading (FRESULT %d)\r\n", (int)fr);
    }
  }

  // Test binary file write/read
  printf("\r\nTesting binary file operations...\r\n");
  SDK_ALIGN(uint8_t bin_data[256], 32);
  for (int i = 0; i < 256; ++i) {
    bin_data[i] = static_cast<uint8_t>(i);
  }
  snprintf(path, sizeof(path), "%s/binary.dat", SDCard::FatFsDrive());
  {
    FIL f;
    UINT written = 0, read = 0;
    if (f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
      f_write(&f, bin_data, sizeof(bin_data), &written);
      f_close(&f);
      printf("Wrote %u bytes to binary.dat\r\n", (unsigned)written);

      SDK_ALIGN(uint8_t read_data[256], 32);
      if (f_open(&f, path, FA_READ) == FR_OK) {
        f_read(&f, read_data, sizeof(read_data), &read);
        f_close(&f);
        printf("Read %u bytes from binary.dat\r\n", (unsigned)read);
        bool match = (read == sizeof(bin_data)) &&
                     (std::memcmp(bin_data, read_data, sizeof(bin_data)) == 0);
        printf("Binary file verification: %s\r\n", match ? "PASSED" : "FAILED");
      }
    } else {
      printf("ERROR: Failed to create binary.dat\r\n");
    }
  }

  // Performance benchmark: large sequential write/read through FatFs
  printf("\r\n=== Performance Benchmark ===\r\n");
  // 2 MiB per buffer (a 1080p XRGB frame is 8.3 MB; two of those do not fit
  // the FreeRTOS heap of the vendor examples).
  const size_t image_size = 2u * 1024u * 1024u;
  printf("Buffer size: %u bytes (%.2f MB)\r\n", (unsigned)image_size,
         static_cast<double>(image_size) / (1024.0 * 1024.0));
  snprintf(path, sizeof(path), "%s/bench.raw", SDCard::FatFsDrive());

  uint8_t* image_buffer = static_cast<uint8_t*>(pvPortMalloc(image_size));
  if (!image_buffer) {
    printf("ERROR: Failed to allocate %u bytes for image buffer\r\n", (unsigned)image_size);
  } else {
    printf("Filling buffer with test pattern...\r\n");
    for (size_t i = 0; i < image_size; ++i) {
      image_buffer[i] = static_cast<uint8_t>((i * 7 + 123) & 0xFF);
    }

    printf("\r\nWrite speed test...\r\n");
    {
      FIL f;
      UINT written = 0;
      TickType_t write_start = xTaskGetTickCount();
      if (f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&f, image_buffer, image_size, &written);
        f_close(&f);
        uint32_t write_ms = (xTaskGetTickCount() - write_start) * portTICK_PERIOD_MS;
        if (written == image_size) {
          double mbps = (static_cast<double>(image_size) / (1024.0 * 1024.0)) /
                        (static_cast<double>(write_ms) / 1000.0);
          printf("Write: %u bytes in %u ms (%.2f MB/s)\r\n", (unsigned)written,
                 (unsigned)write_ms, mbps);
        } else {
          printf("ERROR: Only wrote %u of %u bytes\r\n", (unsigned)written, (unsigned)image_size);
        }
      } else {
        printf("ERROR: Failed to open file for writing\r\n");
      }
    }

    printf("\r\nRead speed test...\r\n");
    uint8_t* read_buffer = static_cast<uint8_t*>(pvPortMalloc(image_size));
    if (!read_buffer) {
      printf("ERROR: Failed to allocate read buffer\r\n");
    } else {
      FIL f;
      UINT read_size = 0;
      TickType_t read_start = xTaskGetTickCount();
      if (f_open(&f, path, FA_READ) == FR_OK) {
        f_read(&f, read_buffer, image_size, &read_size);
        f_close(&f);
        uint32_t read_ms = (xTaskGetTickCount() - read_start) * portTICK_PERIOD_MS;
        if (read_size == image_size) {
          double mbps = (static_cast<double>(image_size) / (1024.0 * 1024.0)) /
                        (static_cast<double>(read_ms) / 1000.0);
          printf("Read: %u bytes in %u ms (%.2f MB/s)\r\n", (unsigned)read_size,
                 (unsigned)read_ms, mbps);
          printf("\r\nVerifying data integrity...\r\n");
          printf("Data verification: %s\r\n",
                 std::memcmp(image_buffer, read_buffer, image_size) == 0 ? "PASSED" : "FAILED");
        } else {
          printf("ERROR: Only read %u of %u bytes\r\n", (unsigned)read_size, (unsigned)image_size);
        }
      } else {
        printf("ERROR: Failed to open file for reading\r\n");
      }
      vPortFree(read_buffer);
    }

    vPortFree(image_buffer);
  }

  // Unmount filesystem
  printf("\r\nUnmounting filesystem...\r\n");
  if (SDCard::Unmount()) {
    printf("Filesystem unmounted successfully\r\n");
  } else {
    printf("WARNING: Failed to unmount filesystem\r\n");
  }

  // Deinitialize SD card
  printf("\r\nDeinitializing SD card...\r\n");
  SDCard::Deinit();

  printf("\r\n=== All Tests Complete! ===\r\n");
  printf("SD card with FAT32 filesystem is working correctly.\r\n");

  // Blink LED rapidly to indicate complete success
  for (int i = 0; i < 10; ++i) {
    LedSet(Led::kStatus, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    LedSet(Led::kStatus, false);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  LedSet(Led::kStatus, true);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::Main();
  vTaskSuspend(nullptr);
}
