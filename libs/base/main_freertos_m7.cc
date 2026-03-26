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

#include "libs/base/main_freertos_m7.h"

#include <functional>

#include "libs/base/check.h"
#include "libs/base/console_m7.h"
#include "libs/base/filesystem.h"
#include "libs/base/gpio.h"
#include "libs/base/ipc_m7.h"
#include "libs/base/network.h"
#include "libs/base/random.h"
#include "libs/base/reset.h"
#include "libs/base/tasks.h"
#include "libs/base/tempsense.h"
#include "libs/base/timer.h"
#include "libs/camera/camera.h"
#include "libs/cdc_eem/cdc_eem.h"
#include "libs/msc_ums/msc_ums.h"
#include "libs/nxp/rt1176-sdk/board_hardware.h"
#include "libs/pmic/pmic.h"
#include "libs/tpu/edgetpu_dfu_task.h"
#include "libs/tpu/edgetpu_task.h"
#include "libs/usb/usb_device_task.h"
#include "libs/usb/usb_host_task.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_sema4.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/apps/httpd.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"

namespace {
lpi2c_rtos_handle_t g_i2c5_handle;
lpi2c_rtos_handle_t g_i2c6_handle;
coralmicro::CdcEem g_cdc_eem;
coralmicro::MscUms g_msc_ums;

// Low-power feature toggles. Set to 1 to enable the corresponding init.
// Dissable network, temp and USB and get 0.1570 Amps on M7
#ifndef ENABLE_NETWORK_STACK
#define ENABLE_NETWORK_STACK 1
#endif
#ifndef ENABLE_USB_EEM
#define ENABLE_USB_EEM 1
#endif
#ifndef ENABLE_TEMP_SENSOR
#define ENABLE_TEMP_SENSOR 1
#endif
#ifndef ENABLE_EDGETPU_DFU
#define ENABLE_EDGETPU_DFU 1
#endif

#ifndef ENABLE_USB_UMS
#define ENABLE_USB_UMS 1
#endif

void InitializeCDCEEM() {
  using namespace std::placeholders;
  g_cdc_eem.Init(
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_interface_value());
  coralmicro::UsbDeviceTask::GetSingleton()->AddDevice(
      g_cdc_eem.config_data(),
      std::bind(&coralmicro::CdcEem::SetClassHandle, &g_cdc_eem, _1),
      std::bind(&coralmicro::CdcEem::HandleEvent, &g_cdc_eem, _1, _2),
      g_cdc_eem.descriptor_data(), g_cdc_eem.descriptor_data_size());
}

void InitializeUMS() {
  using namespace std::placeholders;
  g_msc_ums.Init(
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_interface_value());
  coralmicro::UsbDeviceTask::GetSingleton()->AddDevice(
      g_msc_ums.config_data(),
      std::bind(&coralmicro::MscUms::SetClassHandle, &g_msc_ums, _1),
      std::bind(&coralmicro::MscUms::HandleEvent, &g_msc_ums, _1, _2),
      g_msc_ums.descriptor_data(), g_msc_ums.descriptor_data_size());
}
}  // namespace

extern "C" int sentai_usb_drive_set(int on) {
  if (on) {
    // Unmount user LFS before enabling USB MSC so host has exclusive
    // NAND access and there are no stale LFS cache conflicts.
    lfs_unmount(coralmicro::LfsUser());
  }
  g_msc_ums.SetUnitReady(on != 0);
  if (!on) {
    // Remount user LFS after disabling USB MSC to pick up any changes
    // the host made (new/modified/deleted files).
    // Use LfsUserRemount() — it does NOT auto-format on failure,
    // so user data isn't silently destroyed if mount fails.
    coralmicro::LfsUserRemount();
  }
  return on != 0 ? 1 : 0;
}

extern "C" int sentai_usb_drive_get(void) {
  return g_msc_ums.IsUnitReady() ? 1 : 0;
}

extern "C" lpi2c_rtos_handle_t* I2C5Handle() { return &g_i2c5_handle; }

extern "C" void app_main(void* param);
extern "C" int main(int argc, char** argv) __attribute__((weak));

extern "C" int main(int argc, char** argv) {
  return real_main(argc, argv, true, true);
}

extern "C" int real_main(int argc, char** argv, bool init_console_tx,
                         bool init_console_rx) {
  BOARD_InitHardware(true);
  SEMA4_Init(SEMA4);
  coralmicro::ResetStoreStats();
  coralmicro::TimerInit();
  coralmicro::GpioInit();
  coralmicro::IpcM7::GetSingleton()->Init();
  coralmicro::RandomInit();
  coralmicro::ConsoleM7::GetSingleton()->Init(init_console_tx, init_console_rx);

  CHECK(coralmicro::LfsInit());
  CHECK(coralmicro::LfsUserInit());
  // Make sure this happens before EEM or WICED are initialized.
  #if ENABLE_NETWORK_STACK
    tcpip_init(nullptr, nullptr);
    coralmicro::DnsInit();
  #endif
  #if ENABLE_USB_EEM
    InitializeCDCEEM();
  #endif
  #if ENABLE_USB_UMS
    InitializeUMS();
  #endif
  coralmicro::UsbDeviceTask::GetSingleton()->Init();
  coralmicro::UsbHostTask::GetSingleton()->Init();
  #if ENABLE_EDGETPU_DFU
    coralmicro::EdgeTpuDfuTask::GetSingleton()->Init();
  #endif
  coralmicro::EdgeTpuTask::GetSingleton()->Init();
  #if ENABLE_TEMP_SENSOR
    coralmicro::TempSensorInit();
  #endif

  printf("\r\n\r\n!!!! SentAI  %s %s!!!\r\n", __DATE__, __TIME__);

  // Initialize I2C5 state
  NVIC_SetPriority(LPI2C5_IRQn, 3);
  lpi2c_master_config_t config;
  LPI2C_MasterGetDefaultConfig(&config);
  LPI2C_RTOS_Init(&g_i2c5_handle, reinterpret_cast<LPI2C_Type*>(LPI2C5_BASE),
                  &config, CLOCK_GetFreq(kCLOCK_OscRc48MDiv2));

  // Initialize I2C6 state
#define IOMUXC_GPIO_LPSR_07_LPI2C6_SCL 0x40C0801CU, 0x0U, 0x40C0808CU, 0x0U, 0x40C0805CU
#define IOMUXC_GPIO_LPSR_06_LPI2C6_SDA 0x40C08018U, 0x0U, 0x40C08090U, 0x0U, 0x40C08058U

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_06_LPI2C6_SDA,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_07_LPI2C6_SCL,         /* GPIO_LPSR_07 is configured as LPI2C6_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_07 */
  IOMUXC_SetPinConfig(
        IOMUXC_GPIO_LPSR_06_LPI2C6_SDA, /* GPIO_LPSR_06 PAD functional
                                           properties : */
        0x20U);                         /* Slew Rate Field: Slow Slew Rate
                                           Drive Strength Field: normal driver
                                           Pull / Keep Select Field: Pull Disable
                                           Pull Up / Down Config. Field: Weak pull down
                                           Open Drain LPSR Field: Enabled
                                           Domain write protection: Both cores are allowed
                                           Domain write protection lock: Neither of DWP bits is locked
                                         */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_LPSR_07_LPI2C6_SCL, /* GPIO_LPSR_05 PAD functional
                                           properties : */
        0x20U);                         /* Slew Rate Field: Slow Slew Rate
                                           Drive Strength Field: normal driver
                                           Pull / Keep Select Field: Pull Disable
                                           Pull Up / Down Config. Field: Weak pull down
                                           Open Drain LPSR Field: Enabled
                                           Domain write protection: Both cores are allowed
                                           Domain write protection lock: Neither of DWP bits is locked
                                         */

  NVIC_SetPriority(LPI2C6_IRQn, 3);
  lpi2c_master_config_t config6;
  LPI2C_MasterGetDefaultConfig(&config6);
  LPI2C_RTOS_Init(&g_i2c6_handle, reinterpret_cast<LPI2C_Type*>(LPI2C6_BASE),
                  &config6, CLOCK_GetFreq(kCLOCK_OscRc48MDiv2));

  //coralmicro::PmicTask::GetSingleton()->Init(&g_i2c5_handle);
  coralmicro::CameraTask::GetSingleton()->Init(&g_i2c5_handle, &g_i2c6_handle);

  CHECK(xTaskCreate(app_main, "app_main", configMINIMAL_STACK_SIZE * 30,
                    nullptr, coralmicro::kAppTaskPriority, nullptr) == pdPASS);

  // Allows the AHB clock to run while the core is asleep,
  // so that the TCM is accessible.
  // See section 12.4.4.18 in the IMX1170 TRM for more details.
  IOMUXC_GPR->GPR16 |= IOMUXC_GPR_GPR16_CM7_FORCE_HCLK_EN(1);
  vTaskStartScheduler();
  return 0;
}
