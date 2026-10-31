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

#include "libs/sdcard/sdcard.h"

#include <cstdio>
#include <cstring>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "libs/nxp/rt1176-sdk/clock_config.h"
#include "third_party/modified/nxp/rt1176-sdk/sdmmc_config.h"
#include "third_party/nxp/rt1176-sdk/middleware/sdmmc/sd/fsl_sd.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"

// Board-specific helpers still shared with the SDIO (Wi-Fi) code.
extern "C" {
uint32_t BOARD_USDHC1ClockConfiguration(void);
void BOARD_USDHC_Errata(void);
}

/* Progress breadcrumb of the SD bring-up (1..10 = ConfigureSDHost/SD_Init
 * steps, 100 = worker entered); read with sentai.sys.peek32 or
 * sentai.sdcard.init_stage() when an init does not come back (audit F14/F17). */
extern "C" volatile uint32_t g_sd_init_stage;
volatile uint32_t g_sd_init_stage = 0;

namespace coralmicro {

namespace {

// SD card socket wiring on the current board revision (schematic page 16,
// "Interface", and page 6, RT1176 PART2).  The socket sits on USDHC1
// (GPIO_SD_B1_00..05); it does NOT use USDHC2 and has no card-detect pin.
//   SD_PWREN_B  GPIO_AD_12 (GPIO_MUX3_IO11 -> GPIO3 pin 11): gate of the
//               P-FET U12 that switches VDD_3V3 -> VSD_3V3 (the card supply).
//               Active low: 0 = card powered, 1 = card off.
//   SD1_VSELECT GPIO_AD_16 (GPIO_MUX3_IO15 -> GPIO3 pin 15): select of the
//               NX3L1T3157 analog switch U6 that feeds NVCC_SD1 (the I/O
//               rail of the SD pads).  0 = VDD_3V3, 1 = VDD_1V8.
// The card itself is only ever supplied with 3.3 V, so signalling stays at
// 3.3 V and UHS-I (1.8 V) switching is disabled (ioVoltage == nullptr).
constexpr uint32_t kSdPwrEnPin = 11;
constexpr uint32_t kSdVselectPin = 15;

void SdBoardGpioInit() {
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_12_GPIO_MUX3_IO11, 0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_16_GPIO_MUX3_IO15, 0U);

  // Rail select first (3.3 V I/O), card supply OFF, so the pads are never
  // driven at 1.8 V while a 3.3 V card is powered.
  gpio_pin_config_t cfg = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
  GPIO_PinInit(GPIO3, kSdVselectPin, &cfg);
  cfg.outputLogic = 1;
  GPIO_PinInit(GPIO3, kSdPwrEnPin, &cfg);
}

// SD driver callback: enable == true powers the card.  SD_PWREN_B is
// active low.
void SdBoardPower(bool enable) {
  GPIO_PinWrite(GPIO3, kSdPwrEnPin, enable ? 0 : 1);
}

bool SdBoardCardDetected() {
  // No card-detect switch is wired on this board; assume present and let
  // SD_Init() fail if the socket is empty.
  return true;
}

// DMA descriptor buffer (must be non-cacheable and properly aligned)
AT_NONCACHEABLE_SECTION_ALIGN(uint32_t s_sdmmcHostDmaBuffer[32],
                              SDMMCHOST_DMA_DESCRIPTOR_BUFFER_ALIGN_SIZE);

// Cache-aligned buffer for data transfers if needed
#if defined SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER && \
    SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER
SDK_ALIGN(uint8_t s_sdmmcCacheLineAlignBuffer[64],
          SDMMC_DATA_BUFFER_ALIGN_CACHE);
#endif

// Host controller configuration
sdmmchost_t s_host;

// Card detect structure
sd_detect_card_t s_cd;

// SD card state
sd_card_t s_sdCard;
bool s_cardInitialized = false;

// Configure the SD card host (similar to BOARD_SDIO_Config but for SD cards)
void ConfigureSDHost() {
  s_host.dmaDesBuffer = s_sdmmcHostDmaBuffer;
  s_host.dmaDesBufferWordsNum = 32;

#if ((defined __DCACHE_PRESENT) && __DCACHE_PRESENT) || \
    (defined FSL_FEATURE_HAS_L1CACHE && FSL_FEATURE_HAS_L1CACHE)
  s_host.enableCacheControl = kSDMMCHOST_CacheControlRWBuffer;
#endif

#if defined SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER && \
    SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER
  s_host.cacheAlignBuffer = s_sdmmcCacheLineAlignBuffer;
  s_host.cacheAlignBufferSize = 64;
#endif

    // SD card socket pins (USDHC1).  Pad PULL field: 1 = internal pull-up,
    // 3 = no pull.  Only CMD and DATA0 have external 10k pull-ups to NVCC_SD
    // (R373/R374); DATA1..3 rely on the internal pull-ups.
    #ifndef IOMUXC_SW_PAD_CTL_PAD_APC
    #define IOMUXC_SW_PAD_CTL_PAD_APC_MASK  (0xF0000000U)
    #define IOMUXC_SW_PAD_CTL_PAD_APC_SHIFT (28U)
    #define IOMUXC_SW_PAD_CTL_PAD_APC(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_APC_MASK)
    #endif

    const uint32_t usdhc_cmd_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(1) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                            IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_clk_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(1) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                            IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_data_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(1) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                             IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_data_pullup_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(1) | IOMUXC_SW_PAD_CTL_PAD_PULL(1) |
                                                IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);

    // Rail select + card supply must be settled BEFORE the pads are muxed.
    g_sd_init_stage = 1;
    SdBoardGpioInit();
    g_sd_init_stage = 2;

    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_USDHC1_CMD, true);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_USDHC1_CLK, false);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0, false);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1, false);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2, false);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3, false);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_USDHC1_CMD, usdhc_cmd_pin_settings);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_USDHC1_CLK, usdhc_clk_pin_settings);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_USDHC1_DATA0, usdhc_data_pin_settings);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_USDHC1_DATA1, usdhc_data_pullup_settings);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_04_USDHC1_DATA2, usdhc_data_pullup_settings);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_05_USDHC1_DATA3, usdhc_data_pullup_settings);


  // Configure SD card structure
  std::memset(&s_sdCard, 0, sizeof(s_sdCard));
  s_sdCard.host = &s_host;
  s_sdCard.host->hostController.base = USDHC1;

  g_sd_init_stage = 3;
  s_sdCard.host->hostController.sourceClock_Hz = BOARD_USDHC1ClockConfiguration();
  g_sd_init_stage = 4;

  // Card detect always returns true (assumes card is present)
  s_cd.cdDebounce_ms = 100U;
  s_cd.type = kSD_DetectCardByGpioCD;
  s_cd.cardDetected = SdBoardCardDetected;
  s_cd.callback = nullptr;
  s_cd.userData = nullptr;

  s_sdCard.usrParam.cd = &s_cd;
  s_sdCard.usrParam.pwr = SdBoardPower;
  s_sdCard.usrParam.ioStrength = nullptr;
  s_sdCard.usrParam.ioVoltage = nullptr;  // 3.3 V only, no UHS-I
  s_sdCard.usrParam.maxFreq = 50000000U;  // 50 MHz high-speed, 3.3 V signalling

  // Initialize power control
  // Card supply: off for >= 20 ms (discharge), then on.  C78 on the gate
  // soft-starts VSD_3V3; wait for it to settle before the first command.
  SdBoardPower(false);
  g_sd_init_stage = 5;
  vTaskDelay(pdMS_TO_TICKS(30));
  g_sd_init_stage = 6;
  SdBoardPower(true);
  vTaskDelay(pdMS_TO_TICKS(30));
  g_sd_init_stage = 7;

  // Set interrupt priority
  NVIC_SetPriority(USDHC1_IRQn, 5);

#if __CORTEX_M == 7
  // Apply RT1176 errata workaround
  BOARD_USDHC_Errata();
#endif

}

// ---- bounded init worker (audit F14) --------------------------------------
// Static allocation (embeded.md: no heap in drivers).  The stack lives in
// SDRAM: SD_Init() + the SDMMC host need ~1.5 KB; 2048 words is the same
// size the boot-time mount task uses for the same call chain.
constexpr uint32_t kInitTaskStackWords = 2048;
__attribute__((section(".sdram_bss"))) StackType_t s_initTaskStack[kInitTaskStackWords];
StaticTask_t      s_initTaskTcb;
StaticSemaphore_t s_initDoneStorage;
SemaphoreHandle_t s_initDone   = nullptr;
volatile TaskHandle_t s_initTask = nullptr;
volatile bool     s_initResult = false;

bool InitDirect();

void InitTaskFn(void*) {
  g_sd_init_stage = 100;
  s_initResult = InitDirect();
  s_initTask = nullptr;
  xSemaphoreGive(s_initDone);
  vTaskDelete(nullptr);
}

bool InitDirect() {
  if (s_cardInitialized) {
    printf("[SDCard] Init: Already initialized\r\n");
    return true;
  }

  // Configure host controller
  ConfigureSDHost();
  g_sd_init_stage = 8;
  printf("[SDCard] Init: SD host configured (USDHC1 at %u Hz)\r\n",
         s_sdCard.host->hostController.sourceClock_Hz);
  g_sd_init_stage = 9;

  // Initialize SD card
  status_t status = SD_Init(&s_sdCard);
  g_sd_init_stage = 10;

  if (status != kStatus_Success) {
    printf("[SDCard] Init: FAILED - SD_Init returned error 0x%x\r\n", status);
    return false;
  }

  s_cardInitialized = true;
  printf("[SDCard] Init: SUCCESS - Card initialized\r\n");
  return true;
}

}  // namespace

int SDCard::InitBounded(uint32_t deadline_ms) {
  if (s_cardInitialized) return 1;
  if (s_initTask != nullptr) return -2;          // previous attempt still inside SD_Init()
  if (s_initDone == nullptr) {
    s_initDone = xSemaphoreCreateBinaryStatic(&s_initDoneStorage);
    if (s_initDone == nullptr) return 0;
  }
  (void)xSemaphoreTake(s_initDone, 0);           // drop a stale completion
  s_initResult = false;
  // Static task (no heap): stack in SDRAM, TCB + semaphore storage static.
  // (A/B 2026-10-31: a heap stack behaved identically; the blocked worker
  // seen that day was the card not answering CMD8, audit F17.)
  TaskHandle_t h = xTaskCreateStatic(InitTaskFn, "sd_init", kInitTaskStackWords, nullptr,
                                     uxTaskPriorityGet(nullptr), s_initTaskStack, &s_initTaskTcb);
  if (h == nullptr) return 0;
  s_initTask = h;
  if (xSemaphoreTake(s_initDone, pdMS_TO_TICKS(deadline_ms)) == pdTRUE) {
    return s_initResult ? 1 : 0;
  }
  printf("[SDCard] Init: no answer from SD_Init() within %lu ms (no card / no USDHC wiring?)"
         " -- worker left running, REPL stays alive\r\n", (unsigned long)deadline_ms);
  return -1;
}

bool SDCard::Init() { return InitBounded(kInitDeadlineMs) == 1; }

void SDCard::Deinit() {
  if (!s_cardInitialized) {
    return;
  }

  SD_Deinit(&s_sdCard);
  s_cardInitialized = false;
}

bool SDCard::IsCardInserted() {
  return s_cardInitialized && SD_IsCardPresent(&s_sdCard);
}

bool SDCard::ReadBlocks(uint32_t start_block, uint8_t* buffer,
                       uint32_t block_count) {
  if (!s_cardInitialized) {
    return false;
  }

  status_t status = SD_ReadBlocks(&s_sdCard, buffer, start_block, block_count);
  if (status != kStatus_Success) {
    printf("[SDCard] ReadBlocks(lba=%lu, n=%lu) failed: status=%ld\r\n",
           (unsigned long)start_block, (unsigned long)block_count, (long)status);
  }
  return status == kStatus_Success;
}

bool SDCard::WriteBlocks(uint32_t start_block, const uint8_t* buffer,
                        uint32_t block_count) {
  if (!s_cardInitialized) {
    return false;
  }

  // SD_WriteBlocks() already waits for the card's internal program to finish
  // (SD_Write -> SD_PollingCardStatusBusy before returning), so no extra
  // busy-wait here.  NOTE: SD_PollingCardStatusBusy() signals success with
  // kStatus_SDMMC_CardStatusIdle, NOT kStatus_Success — the inherited code
  // compared against kStatus_Success and reported every good write as a
  // failure (2026-10-30, first FAT32 format attempt).
  status_t status = SD_WriteBlocks(&s_sdCard, buffer, start_block, block_count);
  if (status != kStatus_Success) {
    printf("[SDCard] WriteBlocks(lba=%lu, n=%lu) failed: status=%ld\r\n",
           (unsigned long)start_block, (unsigned long)block_count, (long)status);
    return false;
  }
  return true;
}

bool SDCard::GetCardInfo(SDCardInfo* info) {
  if (!s_cardInitialized || !info) {
    return false;
  }

  info->block_count = s_sdCard.blockCount;
  info->block_size = s_sdCard.blockSize;
  info->capacity_bytes = static_cast<uint64_t>(s_sdCard.blockCount) *
                         static_cast<uint64_t>(s_sdCard.blockSize);
  info->is_high_capacity = (s_sdCard.flags & kSD_SupportHighCapacityFlag) != 0;
  info->manufacturer_id = s_sdCard.cid.manufacturerID;
  info->oem_id = s_sdCard.cid.applicationID;

  // Copy product name (max 5 characters + null terminator)
  std::memcpy(info->product_name, s_sdCard.cid.productName, 5);
  info->product_name[5] = '\0';
  info->bus_clock_hz = s_sdCard.busClock_Hz;
  info->timing_mode = (uint8_t)s_sdCard.currentTiming;
  info->bus_width = (s_sdCard.flags & kSD_Support4BitWidthFlag) ? 4 : 1;

  return true;
}

uint32_t SDCard::GetBlockCount() {
  if (!s_cardInitialized) {
    return 0;
  }
  return s_sdCard.blockCount;
}

uint32_t SDCard::GetBlockSize() {
  if (!s_cardInitialized) {
    return 0;
  }
  return s_sdCard.blockSize;
}

uint64_t SDCard::GetCapacityBytes() {
  if (!s_cardInitialized) {
    return 0;
  }
  return static_cast<uint64_t>(s_sdCard.blockCount) *
         static_cast<uint64_t>(s_sdCard.blockSize);
}

bool SDCard::EraseBlocks(uint32_t start_block, uint32_t block_count) {
  if (!s_cardInitialized) {
    return false;
  }

  status_t status = SD_EraseBlocks(&s_sdCard, start_block, block_count);
  if (status != kStatus_Success) return false;

  // Wait for erase to complete.  SD_PollingCardStatusBusy() reports success
  // as kStatus_SDMMC_CardStatusIdle (see WriteBlocks note above).
  status = SD_PollingCardStatusBusy(&s_sdCard, 30000);  // 30 second timeout for erase
  return status == kStatus_SDMMC_CardStatusIdle || status == kStatus_Success;
}

}  // namespace coralmicro
