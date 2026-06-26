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
#include "libs/base/fx_user_fs.h"
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
#include "libs/cdc_ncm/cdc_ncm.h"
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
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_soc_src.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/apps/httpd.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"

// Storage-mode persistence — primary mechanism is a 12-byte struct in DTC-RAM
// placed in a NOLOAD .noinit_boot_persist section (linker script).  DTC-RAM
// contents are preserved by NVIC_SystemReset / WDOG reset on RT1176 and only
// cleared by hardware POR (reset button / power cycle), which is exactly the
// semantics we want: drive(1) → warm reset → storage mode; reset button →
// POR → DTC-RAM zeroed → default REPL+IP mode.
//
// We additionally write the same magic to several SRC_GPR registers as
// diagnostic backup so we can verify (in boot.log) which persistence
// mechanisms actually survive on this silicon.  Empirically GPR9 was cleared
// on the previous build, contradicting the reference manual; SRAM is the
// authoritative path.
#define SENTAI_STORAGE_MAGIC  0x57500001UL  // 'SP01' = Storage Profile 1

namespace {

// Struct laid out in the NOLOAD .noinit_boot_persist section.  Validity check
// requires both magic == SENTAI_STORAGE_MAGIC AND check == ~magic so a single
// stuck/uninitialized word can't false-trigger storage mode after POR.
//
// `attempts` counts storage-mode boot tries so a crash loop in storage init
// (e.g. lockup during InitializeUMS) eventually falls back to default mode
// instead of bricking the board across power cycles.  It is incremented
// BEFORE entering storage mode and cleared by sentai_storage_boot_succeeded()
// once we know the boot was stable.
struct __attribute__((packed)) BootPersist {
  uint32_t magic;
  uint32_t check;
  uint32_t attempts;   // incremented on each storage-mode boot try
  uint32_t progress;   // last reached progress checkpoint in storage boot
  uint32_t prev_progress;  // snapshot of progress from previous boot
};
__attribute__((section(".noinit.$boot_persist"))) __attribute__((used))
volatile static BootPersist g_boot_persist;

// After this many consecutive storage-mode boots without a "stable" signal,
// give up and force default REPL+IP so the board is recoverable without
// physical access.  Crash → lockup reset preserves DTC-RAM, so the counter
// survives across attempts.
static constexpr uint32_t kMaxStorageAttempts = 3;

static bool g_sentai_storage_mode = false;
lpi2c_rtos_handle_t g_i2c5_handle;
lpi2c_rtos_handle_t g_i2c6_handle;
coralmicro::CdcEem g_cdc_eem;
coralmicro::MscUms g_msc_ums;

#ifndef ENABLE_USB_NCM
#define ENABLE_USB_NCM 1
#endif

#if ENABLE_USB_NCM
coralmicro::CdcNcm g_cdc_ncm;
static volatile bool g_ncm_initialized = false;
#endif

// Low-power feature toggles. Set to 1 to enable the corresponding init.
// Dissable network, temp and USB and get 0.1570 Amps on M7
#ifndef ENABLE_NETWORK_STACK
#define ENABLE_NETWORK_STACK 0
#endif
#ifndef ENABLE_USB_EEM
#define ENABLE_USB_EEM 0
#endif
#ifndef ENABLE_TEMP_SENSOR
#define ENABLE_TEMP_SENSOR 0
#endif
#ifndef ENABLE_EDGETPU_DFU
#define ENABLE_EDGETPU_DFU 1
#endif

#ifndef ENABLE_USB_UMS
#define ENABLE_USB_UMS 1
#endif

#ifndef ENABLE_RANDOM
#define ENABLE_RANDOM 0
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
  // Mark medium present so /dev/sda appears automatically on first plug.
  // SetUnitReady() no longer auto-arms UNIT_ATTENTION (Fix B), so the
  // first TUR from usb-storage returns GOOD and the SCSI scan completes
  // cleanly without the bulk-IN STALL → BOT-Reset → bus reset cascade
  // we used to hit.  Disk is read-only at boot (write_protected_=true);
  // sentai.usb.drive(1) flips that.
  g_msc_ums.SetUnitReady(true);
}

#if ENABLE_USB_NCM
void InitializeCDCNCM() {
  using namespace std::placeholders;
  auto *usb_task = coralmicro::UsbDeviceTask::GetSingleton();
  uint8_t interrupt_ep = usb_task->next_descriptor_value();
  uint8_t bulk_in_ep = usb_task->next_descriptor_value();
  uint8_t bulk_out_ep = usb_task->next_descriptor_value();
  uint8_t comm_iface = usb_task->next_interface_value();
  uint8_t data_iface = usb_task->next_interface_value();
  g_cdc_ncm.Init(interrupt_ep, bulk_in_ep, bulk_out_ep, comm_iface,
                 data_iface);
  usb_task->AddDevice(
      g_cdc_ncm.config_data(),
      std::bind(&coralmicro::CdcNcm::SetClassHandle, &g_cdc_ncm, _1),
      std::bind(&coralmicro::CdcNcm::HandleEvent, &g_cdc_ncm, _1, _2),
      g_cdc_ncm.descriptor_data(), g_cdc_ncm.descriptor_data_size());
  g_ncm_initialized = true;
}
#endif
}  // namespace

// Toggle storage / default mode by writing the persistence magic to DTC-RAM
// and to several SRC_GPR registers.  Some host/boot paths preserve only the
// GPRs across NVIC_SystemReset, so the boot latch accepts either source.
// The DSB before NVIC_SystemReset guarantees the writes are observable before
// the system reset request hits the SRC.
extern "C" int sentai_usb_drive_set(int on) {
  const uint32_t magic = on ? SENTAI_STORAGE_MAGIC : 0u;
  if (on) {
    printf("[usb] Entering STORAGE mode (MSC + REPL).\r\n"
           "[usb] Unmount host drive, then call sentai.usb.drive(0).\r\n");
  } else {
    printf("[usb] Returning to default REPL+IP mode.\r\n");
  }
  // Drain UART before we yank the world out from under us so the user
  // sees the message on /dev/ttyACM0 before re-enumeration.
  vTaskDelay(pdMS_TO_TICKS(50));

  // Primary: DTC-RAM noinit struct — magic + ~magic must both match.
  // Reset the attempts counter so a fresh storage-mode request gets the
  // full kMaxStorageAttempts budget.
  g_boot_persist.magic         = magic;
  g_boot_persist.check         = ~magic;
  g_boot_persist.attempts      = 0;
  g_boot_persist.progress      = 0;
  g_boot_persist.prev_progress = 0;

  // Diagnostic: write to multiple GPRs we know aren't used elsewhere
  // (1=boot_attempts, 2..8=fault breadcrumb, 13=wdog, 14=lockup are reserved).
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister9,  magic);
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister10, magic);
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister11, magic);
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister12, magic);
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister15, magic);
  SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister16, magic);

  __DSB();   // make sure all peripheral + DTC-RAM writes complete
  __ISB();
  NVIC_SystemReset();  // never returns
  return 0;
}

extern "C" int sentai_usb_drive_get(void) {
  // 1 = currently running in storage mode, 0 = default REPL+IP mode.
  return g_sentai_storage_mode ? 1 : 0;
}

#if ENABLE_USB_NCM
extern "C" int sentai_usb_ip_set(int on) {
  if (!g_ncm_initialized) return -1;
  // NCM is always initialized at boot; this is a no-op toggle for API symmetry.
  // The network stack is active as long as USB is connected.
  (void)on;
  return g_ncm_initialized ? 1 : 0;
}

extern "C" int sentai_usb_ip_get(void) {
  return g_ncm_initialized ? 1 : 0;
}
#else
extern "C" int sentai_usb_ip_set(int on) { (void)on; return -1; }
extern "C" int sentai_usb_ip_get(void) { return 0; }
#endif

extern "C" lpi2c_rtos_handle_t* I2C5Handle() { return &g_i2c5_handle; }

extern "C" void app_main(void* param);
extern "C" int main(int argc, char** argv) __attribute__((weak));

extern "C" int main(int argc, char** argv) {
  return real_main(argc, argv, true, true);
}

// Other modules (sentai_runtime, modsentai) read this to decide whether
// to start REPL / lfs_task / httpd or stay quiet.  Always reflects the
// boot-time decision.
extern "C" int sentai_storage_mode_active(void) {
  return g_sentai_storage_mode ? 1 : 0;
}

// Captured at boot for diagnostic logging once boot_log is up.  We snapshot
// the SRAM struct and several GPRs BEFORE clearing them, so the [boot-mode]
// log line can show which mechanism(s) carried the magic across the reset.
static uint32_t g_sentai_boot_mode_flag = 0;     // legacy/back-compat: GPR15
static uint32_t g_sentai_sram_magic = 0;
static uint32_t g_sentai_sram_check = 0;
static uint32_t g_sentai_gpr_snap[6] = {0,0,0,0,0,0};  // 9,10,11,12,15,16
extern "C" uint32_t sentai_boot_mode_flag_raw(void) {
  return g_sentai_boot_mode_flag;
}
extern "C" uint32_t sentai_boot_sram_magic(void)  { return g_sentai_sram_magic; }
extern "C" uint32_t sentai_boot_sram_check(void)  { return g_sentai_sram_check; }
extern "C" uint32_t sentai_boot_gpr_snap(int i)   {
  return (i >= 0 && i < 6) ? g_sentai_gpr_snap[i] : 0;
}
// Storage mode crash-loop counter from the persistent struct.  Reflects
// the attempt count at the START of THIS boot (after the increment at
// real_main).  Logged in [boot-mode] so we can see lockup loops.
extern "C" uint32_t sentai_boot_storage_attempts(void) {
  return g_boot_persist.attempts;
}
// Last progress checkpoint from the previous boot — diagnoses where a
// storage-mode boot crashed.  See sentai_boot_progress_mark() for codes.
extern "C" uint32_t sentai_boot_prev_progress(void) {
  return g_boot_persist.prev_progress;
}
// Stamp a progress checkpoint into DTC-RAM so the next boot (after a
// possible crash here) can read where we got.  Codes:
//   0x01 BOARD_InitHardware done            0x10 app_main entered
//   0x02 SRAM/GPR snapshot done             0x11 boot_log_init done
//   0x03 LfsInit done                       0x12 boot_log_fs_init done
//   0x04 InitializeUMS done                 0x13 storage short-circuit reached
//   0x05 UsbDeviceTask::Init done           0x14 sentai_storage_boot_succeeded
//   0x06 vTaskStartScheduler about to call
extern "C" void sentai_boot_progress_mark(uint32_t code) {
  g_boot_persist.progress = code;
}
// Called from app_main once the storage-mode boot has reached a safe
// checkpoint (UMS up, scheduler stable, no early lockup).  Resets the
// attempt counter so the next drive(1) request starts fresh.
extern "C" void sentai_storage_boot_succeeded(void) {
  if (g_sentai_storage_mode) {
    g_boot_persist.attempts = 0;
    sentai_boot_progress_mark(0x14);
  }
}

extern "C" int real_main(int argc, char** argv, bool init_console_tx,
                         bool init_console_rx) {
  // Snapshot the previous boot's last progress checkpoint BEFORE we touch
  // anything else (and BEFORE setting any new progress mark on this boot),
  // so prev_progress accurately reflects where the previous boot crashed.
  g_boot_persist.prev_progress = g_boot_persist.progress;
  g_boot_persist.progress = 0;

  BOARD_InitHardware(true);
  SEMA4_Init(SEMA4);
  coralmicro::ResetStoreStats();
  coralmicro::TimerInit();
  coralmicro::GpioInit();
  coralmicro::IpcM7::GetSingleton()->Init();
  #if ENABLE_RANDOM
    coralmicro::RandomInit();
  #endif
  coralmicro::ConsoleM7::GetSingleton()->Init(init_console_tx, init_console_rx);

  sentai_boot_progress_mark(0x01);

  // ---- Boot-mode latch (one-shot, with crash-loop guard) ----
  // Snapshot SRAM and GPRs.  Do NOT clear the SRAM magic here: if storage
  // init crashes (lockup → reset), we want the next boot to either retry
  // (until kMaxStorageAttempts) or fall back to default mode and clear the
  // magic.  GPRs are diagnostic-only and get cleared every boot.
  {
    g_sentai_sram_magic = g_boot_persist.magic;
    g_sentai_sram_check = g_boot_persist.check;

    g_sentai_gpr_snap[0] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister9);
    g_sentai_gpr_snap[1] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister10);
    g_sentai_gpr_snap[2] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister11);
    g_sentai_gpr_snap[3] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister12);
    g_sentai_gpr_snap[4] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister15);
    g_sentai_gpr_snap[5] = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister16);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister9,  0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister10, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister11, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister12, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister15, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister16, 0);

    g_sentai_boot_mode_flag = g_sentai_gpr_snap[4];  // GPR15 for legacy log

    const bool sram_valid =
        (g_sentai_sram_magic == SENTAI_STORAGE_MAGIC) &&
        (g_sentai_sram_check == ~SENTAI_STORAGE_MAGIC);
    bool gpr_valid = false;
    for (uint32_t snap : g_sentai_gpr_snap) {
      if (snap == SENTAI_STORAGE_MAGIC) {
        gpr_valid = true;
        break;
      }
    }
    if (sram_valid || gpr_valid) {
      const uint32_t prev_attempts = sram_valid ? g_boot_persist.attempts : 0u;
      if (prev_attempts >= kMaxStorageAttempts) {
        // Crash-loop in storage mode → wipe magic and force default boot
        // so the board is recoverable without physical reset.
        g_boot_persist.magic    = 0;
        g_boot_persist.check    = 0;
        g_boot_persist.attempts = 0;
        g_sentai_storage_mode = false;
      } else {
        g_boot_persist.magic    = SENTAI_STORAGE_MAGIC;
        g_boot_persist.check    = ~SENTAI_STORAGE_MAGIC;
        g_boot_persist.attempts = prev_attempts + 1;
        g_sentai_storage_mode = true;
      }
    } else {
      // Garbage or already-cleared → default mode; normalise the struct.
      g_boot_persist.magic    = 0;
      g_boot_persist.check    = 0;
      g_boot_persist.attempts = 0;
      g_sentai_storage_mode = false;
    }
  }
  sentai_boot_progress_mark(0x02);

  CHECK(coralmicro::LfsInit());
  sentai_boot_progress_mark(0x03);
  if (!g_sentai_storage_mode) {
    // Default mode: mount the user partition, start the IP stack and
    // expose CDC-NCM.  No MSC interface in the descriptor → usb-storage
    // does not probe and the cdc_ncm cascade we used to see at T+1 s is
    // gone entirely.
    CHECK(coralmicro::LfsUserInit());
    /* If a previous storage-mode session left a debug log in SDRAM,
     * flush it to /log/storage_debug.log now that the FileX volume is
     * remounted. */
    int storage_log_bytes = sentai_storage_log_flush_to_fs();
    if (storage_log_bytes > 0) {
      printf("[storage] flushed %d bytes of debug log to /log/storage_debug.log\r\n",
             storage_log_bytes);
    }
    #if ENABLE_NETWORK_STACK || ENABLE_USB_NCM
      tcpip_init(nullptr, nullptr);
    #endif
    #if ENABLE_NETWORK_STACK
      coralmicro::DnsInit();
    #endif
  }
  // Storage mode: open the LevelX flash so MSC reads/writes are
  // routed through the same wear-leveling layer FileX uses.  This
  // means /dev/sda exposes a real FAT volume that the host can
  // mount directly (`mount /dev/sda /mnt`).  We still skip the FileX
  // media open — only LevelX needs to be live for MSC sector ops.
  if (g_sentai_storage_mode) {
    if (!FxUserOpenLxOnly()) {
      printf("[storage] FxUserOpenLxOnly FAILED — MSC will return I/O errors\r\n");
    }
    /* Reset the SDRAM-backed debug log ring; MSC handler appends events
     * here while we're disconnected from the REPL.  Flushed to
     * /log/storage_debug.log on next default-mode boot. */
    sentai_storage_log_init();
    sentai_storage_log("storage-mode boot ok");
  }
  // Skip lwIP entirely in storage mode (no NCM interface either).
  // /dev/sda comes up clean because the descriptor exposes only
  // ConsoleM7's CDC-ACM (anti-brick anchor) and MSC.
  #if ENABLE_USB_EEM
    InitializeCDCEEM();
  #endif
  // ---- Conditional class registration (mode-dependent) ----
  // Default mode: ACM + NCM (no MSC) → no usb-storage probe, no cascade.
  // Storage mode: ACM + MSC (no NCM) → /dev/sda comes up immediately
  // and cleanly because there is no cdc_ncm interface for usb-storage to
  // race against.  ConsoleM7's CDC-ACM stays in both modes (anti-brick).
  #if ENABLE_USB_NCM
  if (!g_sentai_storage_mode) {
    InitializeCDCNCM();
  }
  #endif
  #if ENABLE_USB_UMS
  if (g_sentai_storage_mode) {
    InitializeUMS();
    g_msc_ums.SetUnitReady(true);
    g_msc_ums.SetWriteProtect(false);  // host owns NAND in this mode
  }
  #endif
  sentai_boot_progress_mark(0x04);
  coralmicro::UsbDeviceTask::GetSingleton()->Init();
  sentai_boot_progress_mark(0x05);
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

  
#define LDO_1V8_INT_EN_GPIO      GPIO9
#define LDO_1V8_INT_EN_PIN       20U
  // Initialize VDD_1V8_INT_EN pin as output
  gpio_pin_config_t ldo_1v8_pin_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0,
      .interruptMode = kGPIO_NoIntmode,
  };

  GPIO_PinInit(LDO_1V8_INT_EN_GPIO, LDO_1V8_INT_EN_PIN, &ldo_1v8_pin_config);
  GPIO_PinWrite(LDO_1V8_INT_EN_GPIO, LDO_1V8_INT_EN_PIN, 0);
  printf("Enable the LDO_1V8_INT_EN\n");

#define THRS_GPIO      GPIO10
#define THRS_PIN       6U
  // Initialize THRS pin as output
  gpio_pin_config_t thrs_pin_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0,
      .interruptMode = kGPIO_NoIntmode,
  };

  GPIO_PinInit(THRS_GPIO, THRS_PIN, &thrs_pin_config);
  GPIO_PinWrite(THRS_GPIO, THRS_PIN, 0);
  printf("Enable the THRS pin\n");

  // Allows the AHB clock to run while the core is asleep,
  // so that the TCM is accessible.
  // See section 12.4.4.18 in the IMX1170 TRM for more details.
  IOMUXC_GPR->GPR16 |= IOMUXC_GPR_GPR16_CM7_FORCE_HCLK_EN(1);

  coralmicro::CameraTask::GetSingleton()->Init(&g_i2c5_handle, &g_i2c6_handle);

  coralmicro::PmicTask::GetSingleton()->Init(&g_i2c5_handle);

  CHECK(xTaskCreate(app_main, "app_main", configMINIMAL_STACK_SIZE * 30,
                    nullptr, coralmicro::kAppTaskPriority, nullptr) == pdPASS);

  sentai_boot_progress_mark(0x06);
  vTaskStartScheduler();
  return 0;
}
