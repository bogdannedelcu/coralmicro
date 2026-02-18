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

// Memory Test Example
//
// Tests RAM (SDRAM, OCRAM) and verifies flash (FlexSPI NOR) accessibility.
// RAM tests use write/readback patterns. Flash test is read-only.

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

namespace coralmicro {
namespace {

struct TestPattern {
  const char* name;
  uint32_t value;
};

constexpr TestPattern kPatterns[] = {
    {"All zeros", 0x00000000},
    {"All ones", 0xFFFFFFFF},
    {"Checkerboard 0x55", 0x55555555},
    {"Checkerboard 0xAA", 0xAAAAAAAA},
    {"Alternating words", 0xFF00FF00},
};

bool RunPatternTest(volatile uint32_t* base, size_t size_bytes,
                    const TestPattern& pattern) {
  size_t count = size_bytes / sizeof(uint32_t);

  // Write pattern
  for (size_t i = 0; i < count; ++i) {
    base[i] = pattern.value;
  }

  // Verify
  for (size_t i = 0; i < count; ++i) {
    uint32_t read = base[i];
    if (read != pattern.value) {
      printf("  FAIL at offset 0x%08X: wrote 0x%08lX, read 0x%08lX\r\n",
             static_cast<unsigned int>(i * sizeof(uint32_t)),
             static_cast<unsigned long>(pattern.value),
             static_cast<unsigned long>(read));
      return false;
    }
  }
  return true;
}

bool RunAddressTest(volatile uint32_t* base, size_t size_bytes) {
  size_t count = size_bytes / sizeof(uint32_t);

  // Write each location with its own address-derived value
  for (size_t i = 0; i < count; ++i) {
    base[i] = static_cast<uint32_t>(i);
  }

  // Verify
  for (size_t i = 0; i < count; ++i) {
    uint32_t expected = static_cast<uint32_t>(i);
    uint32_t read = base[i];
    if (read != expected) {
      printf("  FAIL at offset 0x%08X: wrote 0x%08lX, read 0x%08lX\r\n",
             static_cast<unsigned int>(i * sizeof(uint32_t)),
             static_cast<unsigned long>(expected),
             static_cast<unsigned long>(read));
      return false;
    }
  }
  return true;
}

bool RunWalkingBitTest(volatile uint32_t* base, size_t size_bytes) {
  size_t count = size_bytes / sizeof(uint32_t);
  size_t test_count = (count < 32) ? count : 32;

  // Walking 1
  for (size_t i = 0; i < test_count; ++i) {
    uint32_t pattern = 1U << i;
    base[i] = pattern;
  }
  for (size_t i = 0; i < test_count; ++i) {
    uint32_t expected = 1U << i;
    uint32_t read = base[i];
    if (read != expected) {
      printf("  FAIL walking-1 at bit %zu: wrote 0x%08lX, read 0x%08lX\r\n",
             i, static_cast<unsigned long>(expected),
             static_cast<unsigned long>(read));
      return false;
    }
  }

  // Walking 0
  for (size_t i = 0; i < test_count; ++i) {
    uint32_t pattern = ~(1U << i);
    base[i] = pattern;
  }
  for (size_t i = 0; i < test_count; ++i) {
    uint32_t expected = ~(1U << i);
    uint32_t read = base[i];
    if (read != expected) {
      printf("  FAIL walking-0 at bit %zu: wrote 0x%08lX, read 0x%08lX\r\n",
             i, static_cast<unsigned long>(expected),
             static_cast<unsigned long>(read));
      return false;
    }
  }
  return true;
}

bool TestRamRegion(const char* name, volatile uint32_t* base,
                   size_t size_bytes) {
  printf("\r\n--- %s Test (%zu bytes at 0x%08lX) ---\r\n", name, size_bytes,
         reinterpret_cast<unsigned long>(base));

  bool all_passed = true;

  // Pattern tests
  for (const auto& pattern : kPatterns) {
    bool ok = RunPatternTest(base, size_bytes, pattern);
    printf("  %-25s %s\r\n", pattern.name, ok ? "PASS" : "FAIL");
    if (!ok) all_passed = false;
  }

  // Address-as-data test
  {
    bool ok = RunAddressTest(base, size_bytes);
    printf("  %-25s %s\r\n", "Address-as-data", ok ? "PASS" : "FAIL");
    if (!ok) all_passed = false;
  }

  // Walking bit test
  {
    bool ok = RunWalkingBitTest(base, size_bytes);
    printf("  %-25s %s\r\n", "Walking bit", ok ? "PASS" : "FAIL");
    if (!ok) all_passed = false;
  }

  printf("  %s: %s\r\n", name, all_passed ? "ALL PASSED" : "FAILED");
  return all_passed;
}

bool TestSdram() {
  printf("\r\n=== SDRAM Test ===\r\n");

  struct BlockSize {
    const char* label;
    size_t bytes;
  };
  constexpr BlockSize kBlockSizes[] = {
      {"1 KB", 1024},
      {"64 KB", 64 * 1024},
      {"1 MB", 1024 * 1024},
  };

  bool all_passed = true;

  for (const auto& block : kBlockSizes) {
    auto* buf = static_cast<uint32_t*>(pvPortMalloc(block.bytes));
    if (!buf) {
      printf("  ERROR: Failed to allocate %s for SDRAM test\r\n", block.label);
      all_passed = false;
      continue;
    }

    char label[64];
    snprintf(label, sizeof(label), "SDRAM %s", block.label);
    bool ok = TestRamRegion(label, reinterpret_cast<volatile uint32_t*>(buf),
                            block.bytes);
    if (!ok) all_passed = false;

    vPortFree(buf);
  }

  return all_passed;
}

bool TestOcram() {
  printf("\r\n=== OCRAM Test ===\r\n");

  // Use a stack-allocated buffer (stack is in OCRAM1/m_data region)
  constexpr size_t kOcramTestSize = 4096;
  uint32_t ocram_buffer[kOcramTestSize / sizeof(uint32_t)];

  return TestRamRegion("OCRAM (stack)",
                       reinterpret_cast<volatile uint32_t*>(ocram_buffer),
                       kOcramTestSize);
}

bool TestFlash() {
  printf("\r\n=== Flash Verification (read-only) ===\r\n");

  bool all_passed = true;

  // FlexSPI NOR flash is memory-mapped at 0x30000000 (XIP region)
  // Flash Configuration Block (FCB) is at offset 0x400
  constexpr uint32_t kFlashBase = 0x30000000;
  constexpr uint32_t kFcbOffset = 0x400;
  const volatile uint32_t* fcb =
      reinterpret_cast<const volatile uint32_t*>(kFlashBase + kFcbOffset);

  // Check FCB tag - should be "FCFB" = 0x42464346
  constexpr uint32_t kFcfbTag = 0x42464346;
  uint32_t tag = fcb[0];
  if (tag == kFcfbTag) {
    printf("  FCB tag:     0x%08lX (FCFB) - PASS\r\n",
           static_cast<unsigned long>(tag));
  } else {
    printf("  FCB tag:     0x%08lX (expected 0x%08lX) - FAIL\r\n",
           static_cast<unsigned long>(tag),
           static_cast<unsigned long>(kFcfbTag));
    all_passed = false;
  }

  // Read FCB version (second word)
  uint32_t version = fcb[1];
  printf("  FCB version: 0x%08lX\r\n", static_cast<unsigned long>(version));

  // Verify code region is populated (IVT at 0x1000, code at 0x2000)
  constexpr uint32_t kCodeOffset = 0x2000;
  const volatile uint32_t* code =
      reinterpret_cast<const volatile uint32_t*>(kFlashBase + kCodeOffset);

  // Read first 16 words and check they're not all 0xFF or 0x00
  bool has_data = false;
  for (int i = 0; i < 16; ++i) {
    uint32_t word = code[i];
    if (word != 0x00000000 && word != 0xFFFFFFFF) {
      has_data = true;
      break;
    }
  }

  if (has_data) {
    printf("  Code region: populated - PASS\r\n");
  } else {
    printf("  Code region: empty/erased - FAIL\r\n");
    all_passed = false;
  }

  // Display first 4 words of code region for reference
  printf("  Code @ 0x%08lX:",
         static_cast<unsigned long>(kFlashBase + kCodeOffset));
  for (int i = 0; i < 4; ++i) {
    printf(" %08lX", static_cast<unsigned long>(code[i]));
  }
  printf("\r\n");

  printf("  Flash: %s\r\n", all_passed ? "ALL PASSED" : "FAILED");
  return all_passed;
}

void Main() {
  printf("\r\nMemory Test Example\r\n");
  printf("===================\r\n");

  LedSet(Led::kStatus, true);

  bool sdram_ok = TestSdram();
  bool ocram_ok = TestOcram();
  bool flash_ok = TestFlash();

  printf("\r\n=== Summary ===\r\n");
  printf("  SDRAM:  %s\r\n", sdram_ok ? "PASS" : "FAIL");
  printf("  OCRAM:  %s\r\n", ocram_ok ? "PASS" : "FAIL");
  printf("  Flash:  %s\r\n", flash_ok ? "PASS" : "FAIL");

  bool all_ok = sdram_ok && ocram_ok && flash_ok;
  printf("\r\n  Overall: %s\r\n", all_ok ? "ALL PASSED" : "FAILED");

  // LED feedback
  if (all_ok) {
    for (int i = 0; i < 5; ++i) {
      LedSet(Led::kStatus, true);
      vTaskDelay(pdMS_TO_TICKS(200));
      LedSet(Led::kStatus, false);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
  LedSet(Led::kStatus, all_ok);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::Main();
  vTaskSuspend(nullptr);
}
