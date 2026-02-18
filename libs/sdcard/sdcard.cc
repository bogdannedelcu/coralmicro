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

#include "libs/base/check.h"
#include "libs/nxp/rt1176-sdk/clock_config.h"
#include "third_party/modified/nxp/rt1176-sdk/sdmmc_config.h"
#include "third_party/nxp/rt1176-sdk/middleware/sdmmc/sd/fsl_sd.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/middleware/fatfs/source/ff.h"
#include "third_party/nxp/rt1176-sdk/middleware/fatfs/source/fsl_sd_disk/fsl_sd_disk.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"

// Forward declarations for board-specific functions
extern "C" {
void BOARD_SDCardPowerResetInit(void);
void BOARD_SDCardPowerControl(bool enable);
bool BOARD_SDCardGetDetectStatus(void);
void BOARD_SDCardDetectInit(sd_cd_t cd, void *userData);
uint32_t BOARD_USDHC2ClockConfiguration(void);
void BOARD_USDHC_Errata(void);
}

namespace coralmicro {

namespace {

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

// I/O voltage control
sd_io_voltage_t s_ioVoltage = {
    .type = kSD_IOVoltageCtrlByHost,
    .func = nullptr,
};

// Card detect structure
sd_detect_card_t s_cd;

// SD card state
sd_card_t s_sdCard;
bool s_cardInitialized = false;

// FatFS state
FATFS s_fatfs;
bool s_fsMounted = false;
char s_mountPoint[16] = {0};

}  // namespace

// Reference to g_sd defined in fsl_sd_disk.c
extern "C" {
extern sd_card_t g_sd;

// Provide get_fattime function for FatFS
DWORD get_fattime(void) {
  // Return a default timestamp (2025-01-01 00:00:00)
  // Format: bit31-25=Year(from 1980), bit24-21=Month, bit20-16=Day
  //         bit15-11=Hour, bit10-5=Minute, bit4-0=Second/2
  return ((DWORD)(2025 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
}
}

namespace {

// Configure the SD card host (similar to BOARD_SDIO_Config but for SD cards)
void ConfigureSDHost() {
  printf("[SDCard] ConfigureSDHost: Setting up DMA buffers...\r\n");
  s_host.dmaDesBuffer = s_sdmmcHostDmaBuffer;
  s_host.dmaDesBufferWordsNum = 32;

#if ((defined __DCACHE_PRESENT) && __DCACHE_PRESENT) || \
    (defined FSL_FEATURE_HAS_L1CACHE && FSL_FEATURE_HAS_L1CACHE)
  printf("[SDCard] ConfigureSDHost: Enabling cache control\r\n");
  s_host.enableCacheControl = kSDMMCHOST_CacheControlRWBuffer;
#endif

#if defined SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER && \
    SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER
  s_host.cacheAlignBuffer = s_sdmmcCacheLineAlignBuffer;
  s_host.cacheAlignBufferSize = 64;
#endif

    // SD Card
    #define BOARD_USDHC2_CMD_IOMUXC IOMUXC_GPIO_SD_B2_05_USDHC2_CMD
    #define BOARD_USDHC2_CLK_IOMUXC IOMUXC_GPIO_SD_B2_04_USDHC2_CLK
    #define BOARD_USDHC2_DATA0_IOMUXC IOMUXC_GPIO_SD_B2_03_USDHC2_DATA0
    #define BOARD_USDHC2_DATA1_IOMUXC IOMUXC_GPIO_SD_B2_02_USDHC2_DATA1
    #define BOARD_USDHC2_DATA2_IOMUXC IOMUXC_GPIO_SD_B2_01_USDHC2_DATA2
    #define BOARD_USDHC2_DATA3_IOMUXC IOMUXC_GPIO_SD_B2_00_USDHC2_DATA3

    #ifndef IOMUXC_SW_PAD_CTL_PAD_APC
    #define IOMUXC_SW_PAD_CTL_PAD_APC_MASK  (0xF0000000U)
    #define IOMUXC_SW_PAD_CTL_PAD_APC_SHIFT (28U)
    #define IOMUXC_SW_PAD_CTL_PAD_APC(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_APC_MASK)
    #endif

    const uint32_t usdhc_cmd_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(1) | IOMUXC_SW_PAD_CTL_PAD_PULL(0) |
                                            IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_clk_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(0) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                            IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_data_pin_settings = IOMUXC_SW_PAD_CTL_PAD_PDRV(0) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                            IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);
    const uint32_t usdhc_data_pin_nopull = IOMUXC_SW_PAD_CTL_PAD_PDRV(0) | IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
                                        IOMUXC_SW_PAD_CTL_PAD_ODE(0) | IOMUXC_SW_PAD_CTL_PAD_APC(0);

    IOMUXC_SetPinMux(BOARD_USDHC2_DATA1_IOMUXC, false);
    IOMUXC_SetPinMux(BOARD_USDHC2_DATA2_IOMUXC, false);
    IOMUXC_SetPinMux(BOARD_USDHC2_DATA3_IOMUXC, false);
    IOMUXC_SetPinConfig(BOARD_USDHC2_DATA1_IOMUXC, usdhc_data_pin_settings);
    IOMUXC_SetPinConfig(BOARD_USDHC2_DATA2_IOMUXC, usdhc_data_pin_settings);
    IOMUXC_SetPinConfig(BOARD_USDHC2_DATA3_IOMUXC, usdhc_data_pin_settings);
    IOMUXC_SetPinMux(BOARD_USDHC2_CMD_IOMUXC, true);
    IOMUXC_SetPinMux(BOARD_USDHC2_CLK_IOMUXC, false);
    IOMUXC_SetPinMux(BOARD_USDHC2_DATA0_IOMUXC, false);
    IOMUXC_SetPinConfig(BOARD_USDHC2_CMD_IOMUXC, usdhc_cmd_pin_settings);
    IOMUXC_SetPinConfig(BOARD_USDHC2_CLK_IOMUXC, usdhc_clk_pin_settings);
    IOMUXC_SetPinConfig(BOARD_USDHC2_DATA0_IOMUXC, usdhc_data_pin_settings);

  printf("[SDCard] ConfigureSDHost: DMA and cache setup complete\r\n");

  // Configure SD card structure
  printf("[SDCard] ConfigureSDHost: STEP 1\r\n");
  printf("[SDCard] ConfigureSDHost: STEP 2\r\n");
  std::memset(&s_sdCard, 0, sizeof(s_sdCard));
  printf("[SDCard] ConfigureSDHost: STEP 3 - memset DONE\r\n");
  s_sdCard.host = &s_host;
  printf("[SDCard] ConfigureSDHost: Host pointer assigned, setting USDHC2 base...\r\n");
  s_sdCard.host->hostController.base = USDHC2;
  printf("[SDCard] ConfigureSDHost: USDHC2 base set\r\n");

  printf("[SDCard] ConfigureSDHost: Configuring USDHC2 clock...\r\n");
  s_sdCard.host->hostController.sourceClock_Hz = BOARD_USDHC2ClockConfiguration();
  printf("[SDCard] ConfigureSDHost: Clock configured to %u Hz\r\n",
         s_sdCard.host->hostController.sourceClock_Hz);

  // Card detect always returns true (assumes card is present)
  printf("[SDCard] ConfigureSDHost: Setting up card detect...\r\n");
  s_cd.cdDebounce_ms = 100U;
  s_cd.type = kSD_DetectCardByGpioCD;
  s_cd.cardDetected = BOARD_SDCardGetDetectStatus;
  s_cd.callback = nullptr;
  s_cd.userData = nullptr;

  s_sdCard.usrParam.cd = &s_cd;
  s_sdCard.usrParam.pwr = BOARD_SDCardPowerControl;
  s_sdCard.usrParam.ioStrength = nullptr;
  s_sdCard.usrParam.ioVoltage = &s_ioVoltage;
  s_sdCard.usrParam.maxFreq = 200000000U;  // 200MHz max for SDR104

  // Initialize power control
  printf("[SDCard] ConfigureSDHost: Initializing power control...\r\n");
  BOARD_SDCardPowerResetInit();
  printf("[SDCard] ConfigureSDHost: Initializing card detect GPIO...\r\n");
  BOARD_SDCardDetectInit(nullptr, nullptr);

  // Set interrupt priority
  printf("[SDCard] ConfigureSDHost: Setting USDHC2 interrupt priority...\r\n");
  NVIC_SetPriority(USDHC2_IRQn, 5);

#if __CORTEX_M == 7
  // Apply RT1176 errata workaround
  printf("[SDCard] ConfigureSDHost: Applying RT1176 errata workaround...\r\n");
  BOARD_USDHC_Errata();
#endif

  printf("[SDCard] ConfigureSDHost: Configuration complete\r\n");
}

}  // namespace

bool SDCard::Init() {
  printf("[SDCard] Init: Starting SD card initialization...\r\n");

  if (s_cardInitialized) {
    printf("[SDCard] Init: Already initialized\r\n");
    return true;
  }

  // Configure host controller
  printf("[SDCard] Init: Configuring SD host controller...\r\n");
  ConfigureSDHost();
  printf("[SDCard] Init: SD host configured (USDHC2 at %u Hz)\r\n",
         s_sdCard.host->hostController.sourceClock_Hz);

  // Initialize SD card
  printf("[SDCard] Init: Calling SD_Init()...\r\n");
  status_t status = SD_Init(&s_sdCard);
  printf("[SDCard] Init: SD_Init() returned status = 0x%x\r\n", status);

  if (status != kStatus_Success) {
    printf("[SDCard] Init: FAILED - SD_Init returned error 0x%x\r\n", status);
    return false;
  }

  // Copy to global for FatFS disk layer
  printf("[SDCard] Init: Copying card info to global g_sd...\r\n");
  std::memcpy(&g_sd, &s_sdCard, sizeof(sd_card_t));

  s_cardInitialized = true;
  printf("[SDCard] Init: SUCCESS - Card initialized\r\n");
  return true;
}

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
  return status == kStatus_Success;
}

bool SDCard::WriteBlocks(uint32_t start_block, const uint8_t* buffer,
                        uint32_t block_count) {
  if (!s_cardInitialized) {
    return false;
  }

  status_t status = SD_WriteBlocks(&s_sdCard, buffer, start_block, block_count);

  // Wait for write to complete (SD_WriteBlocks is async)
  if (status == kStatus_Success) {
    status = SD_PollingCardStatusBusy(&s_sdCard, 1000);  // 1 second timeout
  }

  return status == kStatus_Success;
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

  // Wait for erase to complete (SD_EraseBlocks is async)
  if (status == kStatus_Success) {
    status = SD_PollingCardStatusBusy(&s_sdCard, 30000);  // 30 second timeout for erase
  }

  return status == kStatus_Success;
}

bool SDCard::Mount(const char* mount_point) {
  if (!s_cardInitialized) {
    return false;
  }

  if (s_fsMounted) {
    // Already mounted
    return true;
  }

  if (!mount_point || mount_point[0] != '/') {
    // Invalid mount point
    return false;
  }

  // Store mount point (remove leading slash for FatFS)
  const char* drive = mount_point + 1;
  if (std::strlen(drive) >= sizeof(s_mountPoint)) {
    return false;
  }
  std::strcpy(s_mountPoint, drive);

  // Mount filesystem
  FRESULT result = f_mount(&s_fatfs, drive, 1);
  if (result != FR_OK) {
    return false;
  }

  s_fsMounted = true;
  return true;
}

bool SDCard::Unmount() {
  if (!s_fsMounted) {
    return true;
  }

  // Unmount filesystem
  FRESULT result = f_mount(nullptr, s_mountPoint, 0);
  if (result != FR_OK) {
    return false;
  }

  s_fsMounted = false;
  s_mountPoint[0] = '\0';
  return true;
}

bool SDCard::IsMounted() {
  return s_fsMounted;
}

bool SDCard::Format() {
  if (!s_cardInitialized) {
    return false;
  }

  // Unmount if currently mounted
  if (s_fsMounted) {
    Unmount();
  }

  // Format with FAT32
  BYTE work[FF_MAX_SS];  // Work buffer for formatting
  MKFS_PARM opt = {
      FM_FAT32,      // Format type: FAT32
      0,             // Number of FATs (0 = default)
      0,             // Align size (0 = default)
      0,             // Number of root directory entries (0 = default)
      0              // Cluster size (0 = auto)
  };

  FRESULT result = f_mkfs("0:", &opt, work, sizeof(work));
  return result == FR_OK;
}

}  // namespace coralmicro
