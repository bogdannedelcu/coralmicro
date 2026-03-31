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
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include <cstdio>
#include <cstring>

namespace coralmicro {
namespace {

// 32-byte aligned buffer for SD card operations
SDK_ALIGN(uint8_t test_buffer[512], 32);
SDK_ALIGN(uint8_t read_buffer[512], 32);

void Main() {
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
    printf("  Block Size: %u bytes\r\n", info.block_size);
    printf("  Block Count: %u\r\n", info.block_count);
    printf("  Capacity: %llu bytes (%.2f MB)\r\n", info.capacity_bytes,
           static_cast<double>(info.capacity_bytes) / (1024.0 * 1024.0));
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

  // Write test block to sector 1000 (avoid boot sectors)
  const uint32_t test_block = 1000;
  printf("Writing test pattern to block %u...\r\n", test_block);
  if (!SDCard::WriteBlocks(test_block, test_buffer, 1)) {
    printf("ERROR: Failed to write test block\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }
  printf("Write successful\r\n");

  // Read back the test block
  printf("Reading back block %u...\r\n", test_block);
  if (!SDCard::ReadBlocks(test_block, read_buffer, 1)) {
    printf("ERROR: Failed to read test block\r\n");
    SDCard::Deinit();
    LedSet(Led::kStatus, false);
    return;
  }
  printf("Read successful\r\n");

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

  // Write 2 blocks
  const uint32_t multi_block_start = 2000;
  printf("Writing 2 blocks starting at block %u...\r\n", multi_block_start);
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

  // Test filesystem operations
  printf("\r\n=== Testing FAT32 Filesystem ===\r\n");

  printf("Mounting filesystem at /sd...\r\n");
  if (!SDCard::Mount("/sd")) {
    printf("ERROR: Failed to mount filesystem\r\n");
    printf("The SD card may need to be formatted. Attempting format...\r\n");

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

  // Test file creation and writing
  printf("\r\nWriting test file /sd/test.txt...\r\n");
  FILE* f = fopen("/sd/test.txt", "w");
  if (f) {
    fprintf(f, "Hello from Coral Dev Board Micro!\r\n");
    fprintf(f, "This is a test file on the SD card.\r\n");
    fprintf(f, "Timestamp: %d\r\n", static_cast<int>(xTaskGetTickCount()));
    fclose(f);
    printf("File written successfully\r\n");
  } else {
    printf("ERROR: Failed to create file\r\n");
  }

  // Test file reading
  printf("Reading back /sd/test.txt...\r\n");
  f = fopen("/sd/test.txt", "r");
  if (f) {
    char line[128];
    printf("File contents:\r\n");
    printf("---\r\n");
    while (fgets(line, sizeof(line), f)) {
      printf("%s", line);
    }
    printf("---\r\n");
    fclose(f);
    printf("File read successfully\r\n");
  } else {
    printf("ERROR: Failed to open file for reading\r\n");
  }

  // Test binary file write/read
  printf("\r\nTesting binary file operations...\r\n");
  SDK_ALIGN(uint8_t bin_data[256], 32);
  for (int i = 0; i < 256; ++i) {
    bin_data[i] = static_cast<uint8_t>(i);
  }

  f = fopen("/sd/binary.dat", "wb");
  if (f) {
    size_t written = fwrite(bin_data, 1, sizeof(bin_data), f);
    fclose(f);
    printf("Wrote %zu bytes to binary.dat\r\n", written);

    // Read it back
    SDK_ALIGN(uint8_t read_data[256], 32);
    f = fopen("/sd/binary.dat", "rb");
    if (f) {
      size_t read = fread(read_data, 1, sizeof(read_data), f);
      fclose(f);
      printf("Read %zu bytes from binary.dat\r\n", read);

      // Verify
      bool match = (read == sizeof(bin_data)) &&
                   (std::memcmp(bin_data, read_data, sizeof(bin_data)) == 0);
      if (match) {
        printf("Binary file verification: PASSED\r\n");
      } else {
        printf("Binary file verification: FAILED\r\n");
      }
    }
  }

  // Performance benchmark: 1080p ARGB8888 image write/read speed
  printf("\r\n=== Performance Benchmark ===\r\n");
  printf("Testing write/read speed for 1080p ARGB8888 image\r\n");

  // 1080p ARGB8888: 1920 x 1080 x 4 bytes = 8,294,400 bytes (~8.3 MB)
  const size_t image_size = 1920 * 1080 * 4;
  printf("Image size: %zu bytes (%.2f MB)\r\n", image_size,
         static_cast<double>(image_size) / (1024.0 * 1024.0));

  // Allocate buffer (use malloc for large buffer to avoid stack overflow)
  uint8_t* image_buffer = static_cast<uint8_t*>(pvPortMalloc(image_size));
  if (!image_buffer) {
    printf("ERROR: Failed to allocate %zu bytes for image buffer\r\n", image_size);
  } else {
    // Fill buffer with test pattern (simulating image data)
    printf("Filling buffer with test pattern...\r\n");
    for (size_t i = 0; i < image_size; ++i) {
      image_buffer[i] = static_cast<uint8_t>((i * 7 + 123) & 0xFF);
    }

    // Write speed test
    printf("\r\nWrite speed test...\r\n");
    TickType_t write_start = xTaskGetTickCount();

    f = fopen("/sd/image_1080p.raw", "wb");
    if (f) {
      size_t written = fwrite(image_buffer, 1, image_size, f);
      fclose(f);

      TickType_t write_end = xTaskGetTickCount();
      uint32_t write_ms = (write_end - write_start) * portTICK_PERIOD_MS;

      if (written == image_size) {
        double write_speed_mbps = (static_cast<double>(image_size) / (1024.0 * 1024.0)) /
                                   (static_cast<double>(write_ms) / 1000.0);
        printf("Write: %zu bytes in %u ms (%.2f MB/s)\r\n",
               written, write_ms, write_speed_mbps);
      } else {
        printf("ERROR: Only wrote %zu of %zu bytes\r\n", written, image_size);
      }
    } else {
      printf("ERROR: Failed to open file for writing\r\n");
    }

    // Read speed test
    printf("\r\nRead speed test...\r\n");
    uint8_t* read_buffer = static_cast<uint8_t*>(pvPortMalloc(image_size));
    if (!read_buffer) {
      printf("ERROR: Failed to allocate read buffer\r\n");
    } else {
      TickType_t read_start = xTaskGetTickCount();

      f = fopen("/sd/image_1080p.raw", "rb");
      if (f) {
        size_t read_size = fread(read_buffer, 1, image_size, f);
        fclose(f);

        TickType_t read_end = xTaskGetTickCount();
        uint32_t read_ms = (read_end - read_start) * portTICK_PERIOD_MS;

        if (read_size == image_size) {
          double read_speed_mbps = (static_cast<double>(image_size) / (1024.0 * 1024.0)) /
                                    (static_cast<double>(read_ms) / 1000.0);
          printf("Read: %zu bytes in %u ms (%.2f MB/s)\r\n",
                 read_size, read_ms, read_speed_mbps);

          // Verify data integrity
          printf("\r\nVerifying data integrity...\r\n");
          if (std::memcmp(image_buffer, read_buffer, image_size) == 0) {
            printf("Data verification: PASSED\r\n");
          } else {
            printf("Data verification: FAILED\r\n");
          }
        } else {
          printf("ERROR: Only read %zu of %zu bytes\r\n", read_size, image_size);
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
