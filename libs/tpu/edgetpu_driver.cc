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

#include "libs/tpu/edgetpu_driver.h"

#include <cassert>
#include <cstring>

#include "libs/base/check.h"
#include "libs/base/timer.h"
#include "libs/tpu/darwinn/driver/config/beagle/beagle_chip_config.h"
#include "libs/tpu/darwinn/driver/config/beagle_csr_helper.h"
#include "libs/tpu/darwinn/driver/config/common_csr_helper.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/task.h"
#ifndef SENTAI_PLATFORM_SIM
#include "fsl_common.h"
#include "third_party/nxp/rt1176-sdk/components/osa/fsl_os_abstraction.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb_spec.h"
#else
#include <sched.h>
#include <time.h>
#endif

#ifdef SENTAI_PLATFORM_SIM
extern "C" volatile uint32_t g_sentai_tpu_sim_outfeed_chunk_length;
#endif

extern "C" volatile int g_sentai_tpu_multi_ep_routing = 0;

namespace coralmicro {
namespace {

#ifdef SENTAI_PLATFORM_SIM
void sentai_sim_host_sleep_1ms() {
  struct timespec ts;
  ts.tv_sec = 0;
  ts.tv_nsec = 1000000L;
  while (nanosleep(&ts, &ts) != 0) {}
}
#endif

// EdgeTPU USB endpoint layout (observed identically on both single_ep and
// multi_ep firmware variants, per sentai_usb_edgetpu_dump_eps):
//   OUT 1, 2, 3 — bulk OUT, 512-byte packets
//   IN  1, 2    — bulk IN , 512-byte packets
//   IN  3       — interrupt, 64-byte packets
// The default driver path sends ALL bulk OUT traffic (params, instructions,
// inputs) on a single endpoint, reads output activations on bulk-IN EP1, and
// reads events on bulk-IN EP2.  When per-tag routing is enabled
// (g_sentai_tpu_multi_ep_routing=1) we spread
// the OUT queues across EP 1..3 and let the TPU's multi_bo_ep=1 CSR route
// each to its matching on-chip FIFO — this is the precondition for later
// concurrent URB submission.
constexpr uint8_t kSingleBulkOutEndpoint = 1;
constexpr uint8_t kEventInEndpoint = 2;
constexpr uint8_t kOutEpInstructions     = 1;
constexpr uint8_t kOutEpInputActivations = 2;
constexpr uint8_t kOutEpParameters       = 3;

// Runtime toggle + one-shot apply latch.  The latch ensures the
// multi_bo_ep CSR is written exactly once per boot the first time routing
// is enabled, without racing with TpuDriver::Initialize (which runs
// before the flag can be set by the user).
static volatile int     s_sentai_multi_bo_ep_applied  = 0;
extern "C" int  sentai_tpu_multi_ep_routing_get(void) { return g_sentai_tpu_multi_ep_routing; }
extern "C" void sentai_tpu_multi_ep_routing_set(int v) { g_sentai_tpu_multi_ep_routing = v ? 1 : 0; }

static uint8_t endpoint_for_tag(DescriptorTag tag) {
    if (g_sentai_tpu_multi_ep_routing == 0) return kSingleBulkOutEndpoint;
    switch (tag) {
        case DescriptorTag::kInstructions:
            return kOutEpInstructions;
        case DescriptorTag::kInputActivations:
            return kOutEpInputActivations;
        case DescriptorTag::kParameters:
            return kOutEpParameters;
        default:
            return kSingleBulkOutEndpoint;
    }
}
constexpr uint8_t kInterruptInEndpoint = 3;
constexpr uint32_t kMaxBulkBufferSize = 32 * 1024;
// sentai: move the USB bulk staging buffer out of default (SDRAM via
// SEMC) and into the on-chip 512 KB m_ocram region.  Rationale: during
// Invoke, the BulkOutTransfer loop memcpys the input tensor into this
// buffer, then the EHCI host DMA reads it out to the USB wire.  With
// SDRAM placement both accesses compete on SEMC with PrepTask's
// concurrent PXP scale (camera SDRAM buffer → tensor SDRAM buffer).
// Moving this 32 KB buffer to OCRAM removes the USB-side traffic from
// SEMC entirely, freeing bandwidth for PXP and cutting SEMC contention
// during overlapped Invoke.  32 KB easily fits in OCRAM alongside the
// existing MicroPython / audio / aifes sections (~300 KB headroom).
uint8_t BulkTransferBuffer[kMaxBulkBufferSize]
#ifdef SENTAI_PLATFORM_SIM
    __attribute__((aligned(32)));
#else
    __attribute__((section(".ocram_bss,\"aw\",%nobits @")))
    __attribute__((aligned(32)));
#endif

struct UsbTransferMetadata {
  SemaphoreHandle_t sema;
  usb_status_t status;
  size_t bytes_transferred;
  DescriptorTag tag;
  uint32_t bytes_requested;
  uint32_t submit_start_cyc;
  uint32_t submit_return_cyc;
  volatile uint32_t callback_cyc;
  uint32_t submit_start_us;
  uint32_t submit_return_us;
  volatile uint32_t callback_us;
};

enum TpuUrbStatIndex {
  kUrbStatInstructions = 0,
  kUrbStatInput = 1,
  kUrbStatParameters = 2,
  kUrbStatOutput = 3,
  kUrbStatEvent = 4,
  kUrbStatUnknown = 5,
  kUrbStatCount = 6,
};

enum TpuUrbStatField {
  kUrbFieldCalls = 0,
  kUrbFieldCallbacks = 1,
  kUrbFieldBytesRequested = 2,
  kUrbFieldBytesDone = 3,
  kUrbFieldSubmitCycles = 4,
  kUrbFieldCallbackCycles = 5,
  kUrbFieldWaitCycles = 6,
  kUrbFieldErrors = 7,
  kUrbFieldTimeouts = 8,
  kUrbFieldSubmitFailures = 9,
  kUrbFieldSubmitUs = 10,
  kUrbFieldCallbackUs = 11,
  kUrbFieldWaitUs = 12,
  kUrbFieldCount = 13,
};

static uint64_t s_tpu_urb_stats[kUrbStatCount][kUrbFieldCount];

static inline uint32_t TpuUrbCycleNow() {
#ifdef SENTAI_PLATFORM_SIM
  return (uint32_t)xTaskGetTickCount();
#else
  return DWT->CYCCNT;
#endif
}

static inline uint32_t TpuUrbMicrosNow() {
#ifdef SENTAI_PLATFORM_SIM
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000u);
#else
  return (uint32_t)coralmicro::TimerMicros();
#endif
}

static inline uint32_t TpuUrbDelta(uint32_t end, uint32_t start) {
  return end - start;
}

static int TpuUrbStatIndexForTag(DescriptorTag tag) {
  switch (tag) {
    case DescriptorTag::kInstructions:
      return kUrbStatInstructions;
    case DescriptorTag::kInputActivations:
      return kUrbStatInput;
    case DescriptorTag::kParameters:
      return kUrbStatParameters;
    case DescriptorTag::kOutputActivations:
      return kUrbStatOutput;
    case DescriptorTag::kInterrupt0:
    case DescriptorTag::kInterrupt1:
    case DescriptorTag::kInterrupt2:
    case DescriptorTag::kInterrupt3:
      return kUrbStatEvent;
    default:
      return kUrbStatUnknown;
  }
}

static void TpuUrbStatAdd(int idx, int field, uint64_t value) {
  if (idx < 0 || idx >= kUrbStatCount || field < 0 || field >= kUrbFieldCount) {
    return;
  }
  taskENTER_CRITICAL();
  s_tpu_urb_stats[idx][field] += value;
  taskEXIT_CRITICAL();
}

static void TpuUrbRecordSubmitFailure(DescriptorTag tag, uint32_t bytes,
                                      uint32_t submit_cycles,
                                      uint32_t submit_us) {
  const int idx = TpuUrbStatIndexForTag(tag);
  TpuUrbStatAdd(idx, kUrbFieldCalls, 1);
  TpuUrbStatAdd(idx, kUrbFieldBytesRequested, bytes);
  TpuUrbStatAdd(idx, kUrbFieldSubmitCycles, submit_cycles);
  TpuUrbStatAdd(idx, kUrbFieldSubmitUs, submit_us);
  TpuUrbStatAdd(idx, kUrbFieldSubmitFailures, 1);
}

static void TpuUrbRecordDone(const UsbTransferMetadata& meta,
                             uint32_t wait_end_cyc, uint32_t wait_end_us,
                             bool timeout) {
  const int idx = TpuUrbStatIndexForTag(meta.tag);
  TpuUrbStatAdd(idx, kUrbFieldCalls, 1);
  TpuUrbStatAdd(idx, kUrbFieldBytesRequested, meta.bytes_requested);
  TpuUrbStatAdd(idx, kUrbFieldSubmitCycles,
                TpuUrbDelta(meta.submit_return_cyc, meta.submit_start_cyc));
  TpuUrbStatAdd(idx, kUrbFieldWaitCycles,
                TpuUrbDelta(wait_end_cyc, meta.submit_start_cyc));
  TpuUrbStatAdd(idx, kUrbFieldSubmitUs,
                TpuUrbDelta(meta.submit_return_us, meta.submit_start_us));
  TpuUrbStatAdd(idx, kUrbFieldWaitUs,
                TpuUrbDelta(wait_end_us, meta.submit_start_us));
  if (timeout) {
    TpuUrbStatAdd(idx, kUrbFieldTimeouts, 1);
    return;
  }
  TpuUrbStatAdd(idx, kUrbFieldCallbacks, 1);
  TpuUrbStatAdd(idx, kUrbFieldBytesDone, meta.bytes_transferred);
  TpuUrbStatAdd(idx, kUrbFieldCallbackCycles,
                TpuUrbDelta(meta.callback_cyc, meta.submit_start_cyc));
  TpuUrbStatAdd(idx, kUrbFieldCallbackUs,
                TpuUrbDelta(meta.callback_us, meta.submit_start_us));
  if (meta.status != kStatus_USB_Success) {
    TpuUrbStatAdd(idx, kUrbFieldErrors, 1);
  }
}

extern "C" void sentai_tpu_urb_stats_reset(void) {
  taskENTER_CRITICAL();
  for (int i = 0; i < kUrbStatCount; ++i) {
    for (int j = 0; j < kUrbFieldCount; ++j) {
      s_tpu_urb_stats[i][j] = 0;
    }
  }
  taskEXIT_CRITICAL();
}

extern "C" uint32_t sentai_tpu_urb_cycle_hz(void) {
#ifdef SENTAI_PLATFORM_SIM
  return configTICK_RATE_HZ;
#else
  return SystemCoreClock;
#endif
}

extern "C" uint32_t sentai_tpu_urb_stats(uint64_t* out, uint32_t max_words) {
  const uint32_t need = kUrbStatCount * kUrbFieldCount;
  if (!out || max_words < need) {
    return need;
  }
  uint32_t k = 0;
  taskENTER_CRITICAL();
  for (int i = 0; i < kUrbStatCount; ++i) {
    for (int j = 0; j < kUrbFieldCount; ++j) {
      out[k++] = s_tpu_urb_stats[i][j];
    }
  }
  taskEXIT_CRITICAL();
  return need;
}

// One-time-init persistent semaphore for the bulk-transfer completion
// handshake.  Old code used xSemaphoreCreateBinary + vSemaphoreDelete
// on every chunk (~50 µs each on M7) — for a 24-chunk 786 KB input
// that's ~2.4 ms of sema overhead per invoke.  A persistent sema
// saves that round-trip without changing any other behaviour.
//
// Create-first then publish-via-release-store so any racing caller
// either sees the NULL sentinel and spins, or sees the fully-built
// handle.  A CAS-before-create would flash `inited=1` to a racing
// caller while the handle was still NULL — that path ended with
// xSemaphoreGive(NULL) from the USB host callback, hanging the TPU
// driver (observed during tpu.load on build #751–753).
static SemaphoreHandle_t s_bulk_sema  = NULL;

static void InitBulkSema() {
    if (__atomic_load_n(&s_bulk_sema, __ATOMIC_ACQUIRE) != NULL) return;
    // Create first, then publish.  vTaskSuspendAll() protects against
    // two tasks racing into the factory at the same time — FreeRTOS
    // object creation is not interrupt-safe, and concurrent creation
    // would leak a handle.
    vTaskSuspendAll();
    if (s_bulk_sema == NULL) {
        SemaphoreHandle_t local = xSemaphoreCreateBinary();
        // Release-store so callers on other cores/tasks see the fully
        // initialized sema via an ACQUIRE load.
        __atomic_store_n(&s_bulk_sema, local, __ATOMIC_RELEASE);
    }
    xTaskResumeAll();
}

#ifdef SENTAI_PLATFORM_SIM
static void SimSleepUs(long us) {
  struct timespec ts;
  ts.tv_sec = us / 1000000L;
  ts.tv_nsec = (us % 1000000L) * 1000L;
  nanosleep(&ts, nullptr);
}

static void SimInitTrace(const char* msg) {
  printf("[TPU init] %s\r\n", msg);
  fflush(stdout);
}
#else
static void SimInitTrace(const char*) {}
#endif
}  // namespace

namespace registers = platforms::darwinn::driver::config::registers;

bool TpuDriver::Initialize(usb_host_edgetpu_instance_t *usb_instance,
                           PerformanceMode mode) {
  if (usb_instance == nullptr) {
    return false;
  }
  usb_instance_ = usb_instance;

  // Check chip id and test write
  SimInitTrace("read omc0_00");
  uint32_t omc0_00_reg;
  if (!Read32(chip_config_.GetApexCsrOffsets().omc0_00, &omc0_00_reg)) {
    printf("[TPU init] failed: read omc0_00\r\n");
    return false;
  }

  registers::Omc000 omc0_00(omc0_00_reg);
  SimInitTrace("check chip id");
  if (0x89A != omc0_00.chip_id()) {
    printf("[TPU init] failed: chip_id=0x%x\r\n",
           (unsigned)omc0_00.chip_id());
    return false;
  }

  omc0_00.set_test_reg0(0xAA);
  SimInitTrace("write test reg");
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_00, omc0_00.raw()));

  omc0_00_reg = 0;
  SimInitTrace("read test reg");
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_00, &omc0_00_reg));
  omc0_00.set_raw(omc0_00_reg);
  CHECK(0xAA == omc0_00.test_reg0());

  // Disable inactive mode
  SimInitTrace("disable inactive mode");
  uint32_t scu_ctrl_0_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, &scu_ctrl_0_reg));
  registers::ScuCtrl0 scu_ctrl_0(scu_ctrl_0_reg);
  scu_ctrl_0.set_rg_pcie_inact_phy_mode(0);
  scu_ctrl_0.set_rg_usb_inact_phy_mode(0);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, scu_ctrl_0.raw()));
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, &scu_ctrl_0_reg));

  // Disable clock gating
  SimInitTrace("disable clock gating");
  uint32_t scu_ctrl_2_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));
  registers::ScuCtrl2 scu_ctrl_2(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(0x2);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, scu_ctrl_2.raw()));
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));

  // Bounded iteration count for all TPU register polls below.  At
  // 800 MHz CPU, even if a single register read costs 500 cycles,
  // 10 000 iterations = 6 ms worst case — comfortably below our
  // 30 s WDOG.  If any of these poll loops hits the cap, the TPU
  // is in an unexpected state and Initialize() must fail loudly
  // instead of hanging.
  constexpr int kMaxPollIter = 10000;

  // Go into reset, if we're not there
  SimInitTrace("enter reset");
  uint32_t scu_ctrl_3_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
  registers::ScuCtrl3 scu_ctrl_3(scu_ctrl_3_reg);
  if (scu_ctrl_3.rg_force_sleep() != 0x3) {
    scu_ctrl_3.set_rg_force_sleep(0x3);
    CHECK(
        Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, scu_ctrl_3.raw()));
    int iter = 0;
    do {
      CHECK(
          Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
      scu_ctrl_3.set_raw(scu_ctrl_3_reg);
      if (++iter >= kMaxPollIter) {
        printf("[TPU] poll timeout: rg_force_sleep→sleep (cur_pwr_state=%u)\r\n",
               (unsigned)scu_ctrl_3.cur_pwr_state());
        return false;
      }
    } while (scu_ctrl_3.cur_pwr_state() != 0x2);
    CHECK(Write32(chip_config_.GetCbBridgeCsrOffsets().gcbb_credit0, 0xF));
    CHECK(Write32(chip_config_.GetCbBridgeCsrOffsets().gcbb_credit0, 0x0));
  }

  // Set performance mode and exit reset.
  SimInitTrace("exit reset");
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
  scu_ctrl_3.set_raw(scu_ctrl_3_reg);
  scu_ctrl_3.set_rg_force_sleep(0x2);
  switch (mode) {
    case PerformanceMode::kMax:
      scu_ctrl_3.set_gcb_clock_rate(registers::ScuCtrl3::GcbClock::k500MHZ);
      scu_ctrl_3.set_axi_clock_rate(registers::ScuCtrl3::AxiClock::k250MHZ);
      scu_ctrl_3.set_usb_8051_clock_rate(
          registers::ScuCtrl3::Usb8051Clock::k500MHZ);
      break;
    case PerformanceMode::kHigh:
      scu_ctrl_3.set_gcb_clock_rate(registers::ScuCtrl3::GcbClock::k250MHZ);
      scu_ctrl_3.set_axi_clock_rate(registers::ScuCtrl3::AxiClock::k125MHZ);
      scu_ctrl_3.set_usb_8051_clock_rate(
          registers::ScuCtrl3::Usb8051Clock::k500MHZ);
      break;
    case PerformanceMode::kMedium:
      scu_ctrl_3.set_gcb_clock_rate(registers::ScuCtrl3::GcbClock::k125MHZ);
      scu_ctrl_3.set_axi_clock_rate(registers::ScuCtrl3::AxiClock::k125MHZ);
      scu_ctrl_3.set_usb_8051_clock_rate(
          registers::ScuCtrl3::Usb8051Clock::k500MHZ);
      break;
    case PerformanceMode::kLow:
      scu_ctrl_3.set_gcb_clock_rate(registers::ScuCtrl3::GcbClock::k63MHZ);
      scu_ctrl_3.set_axi_clock_rate(registers::ScuCtrl3::AxiClock::k125MHZ);
      scu_ctrl_3.set_usb_8051_clock_rate(
          registers::ScuCtrl3::Usb8051Clock::k250MHZ);
      break;
  }
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, scu_ctrl_3.raw()));

  {
    int iter = 0;
    do {
      CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
      scu_ctrl_3.set_raw(scu_ctrl_3_reg);
      if (++iter >= kMaxPollIter) {
        printf("[TPU] poll timeout: exit reset (cur_pwr_state=%u)\r\n",
               (unsigned)scu_ctrl_3.cur_pwr_state());
        return false;
      }
    } while (scu_ctrl_3.cur_pwr_state() != 0x0);
  }

  // Check a known register to verify reset exit.
  SimInitTrace("check scalar run control");
  uint64_t scalar_core_run_control;
  {
    int iter = 0;
    do {
      CHECK(Read64(chip_config_.GetScalarCoreCsrOffsets().scalarCoreRunControl,
                   &scalar_core_run_control));
      if (++iter >= kMaxPollIter) {
        printf("[TPU] poll timeout: scalarCoreRunControl (=0x%llx)\r\n",
               (unsigned long long)scalar_core_run_control);
        return false;
      }
    } while (scalar_core_run_control != 0);
  }

  SimInitTrace("write idle register");
  registers::IdleRegister idle_reg;
  idle_reg.set_enable();
  idle_reg.set_counter(1);
  CHECK(Write64(chip_config_.GetMiscCsrOffsets().idleRegister, idle_reg.raw()));

  SimInitTrace("write tile config");
  registers::TileConfig<7> tile_config;
  tile_config.set_broadcast();
  CHECK(Write64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                tile_config.raw()));

  uint64_t tile_config_reg;
  {
    int iter = 0;
    do {
      CHECK(Read64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                   &tile_config_reg));
      if (++iter >= kMaxPollIter) {
        printf("[TPU] poll timeout: tileconfig0 write-back\r\n");
        return false;
      }
    } while (tile_config.raw() != tile_config_reg);
  }

  SimInitTrace("write deep sleep");
  registers::DeepSleep deep_sleep_reg;
  deep_sleep_reg.set_to_sleep_delay(2);
  deep_sleep_reg.set_to_wake_delay(30);
  CHECK(Write64(chip_config_.GetTileCsrOffsets().deepSleep,
                deep_sleep_reg.raw()));

  // Enable clock gating
  SimInitTrace("enable clock gating");
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));
  scu_ctrl_2.set_raw(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(1);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, scu_ctrl_2.raw()));

  SimInitTrace("configure usb csr");
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().descr_ep, 0xF0));
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().multi_bo_ep, 0));
  // NB: 0x20 (256 B) is required on NXP RT1176 EHCI — empirically tested
  // 0x80 (1 KB) which broke bulk-in reads entirely (0 frames through
  // pipeline).  libedgetpu (driver/usb/usb_driver.cc:349-374) documents
  // this as a b/73181174 "short packet" workaround.  The POSIX/libusb bridge
  // can opt into libedgetpu's forced-largest chunk path for host-side sweeps,
  // while ARM keeps the conservative shipped value.
#ifdef SENTAI_PLATFORM_SIM
  uint32_t outfeed_chunk_length = ::g_sentai_tpu_sim_outfeed_chunk_length;
  if (outfeed_chunk_length != 0x20 && outfeed_chunk_length != 0x80) {
    outfeed_chunk_length = 0x20;
  }
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().outfeed_chunk_length,
                outfeed_chunk_length));
#else
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().outfeed_chunk_length, 0x20));
#endif

  uint32_t omc0_d0_reg, omc0_d8_reg, omc0_dc_reg;

  // Enables tempsense clock.
  SimInitTrace("enable tempsense clock");
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_d0, &omc0_d0_reg));
  registers::Omc0D0 omc0_d0(omc0_d0_reg);
  omc0_d0.set_clk_en(0x1);
  omc0_d0.set_adr(0xC);
  omc0_d0.set_tref(0);
  omc0_d0.set_tslope(0);
  omc0_d0.set_t_setting(0);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_d0, omc0_d0.raw()));

  // Enables tempsense input ports.
  SimInitTrace("enable tempsense input ports");
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_d8, &omc0_d8_reg));
  registers::Omc0D8 omc0_d8(omc0_d8_reg);
  omc0_d8.set_enbg(0x1);
  omc0_d8.set_envr(0x1);
  omc0_d8.set_enad(0x1);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_d8, omc0_d8.raw()));

  // Wait 100 us before enabling tempsense flow.
#ifdef SENTAI_PLATFORM_SIM
  SimSleepUs(100);
#else
  SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CpuClk));
#endif

  // Enables tempsense flow.
  SimInitTrace("enable tempsense flow");
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_dc, &omc0_dc_reg));
  registers::Omc0DC omc0_dc(omc0_dc_reg);
  omc0_dc.set_enthmc(0x1);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_dc, omc0_dc.raw()));

  SimInitTrace("move to run");
  CHECK(DoRunControl(platforms::darwinn::driver::RunControl::kMoveToRun));

  SimInitTrace("done");
  return true;
}

bool TpuDriver::CSRTransfer(uint64_t reg, void *data, bool read,
                            RegisterSize reg_size) {
  bool ret = false;
  usb_status_t control_status;
  usb_setup_struct_t setup_packet;
  setup_packet.bmRequestType =
      USB_REQUEST_TYPE_TYPE_VENDOR | USB_REQUEST_TYPE_RECIPIENT_DEVICE;
  setup_packet.bmRequestType |=
      read ? USB_REQUEST_TYPE_DIR_IN : USB_REQUEST_TYPE_DIR_OUT;
  switch (reg_size) {
    case RegisterSize::kRegSize32:
      setup_packet.bRequest = 1;
      setup_packet.wLength = 4;
      break;
    case RegisterSize::kRegSize64:
      setup_packet.bRequest = 0;
      setup_packet.wLength = 8;
      break;
  }

  setup_packet.wValue = 0xFFFF & reg;
  setup_packet.wIndex = 0xFFFF & (reg >> 16);

#ifdef SENTAI_PLATFORM_SIM
  control_status = USB_HostEdgeTpuControl(
      usb_instance_, &setup_packet, (uint8_t *)data, nullptr, nullptr);
  return control_status == kStatus_USB_Success;
#else
  SemaphoreHandle_t sema = xSemaphoreCreateBinary();

  control_status = USB_HostEdgeTpuControl(
      usb_instance_, &setup_packet, (uint8_t *)data,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        SemaphoreHandle_t sema = (SemaphoreHandle_t)param;
        xSemaphoreGive(sema);
      },
      sema);
  // Counters rather than printf to avoid the USB-CDC feedback loop
  // documented above (printf → CDC → USB device task → more
  // contention → more timeouts).  Callers that need visibility poll
  // g_sentai_tpu_csr_* counters from async_stats.
  if (control_status != kStatus_USB_Success) {
    goto exit;
  }
  if (xSemaphoreTake(sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
    ret = false;
    goto exit;
  }

  ret = true;
exit:
  vSemaphoreDelete(sema);
  return ret;
#endif
}

bool TpuDriver::SendData(DescriptorTag tag, const uint8_t *data,
                         uint32_t length) const {
  // sentai: when per-queue routing is enabled, the multi_bo_ep CSR must
  // be 1 (the default initialisation at TpuDriver::Initialize sets it to
  // 0).  Apply once, lazily, so enabling the flag from Python after boot
  // takes effect on the next SendData without requiring a fresh
  // OpenDevice.  Failure to write the CSR just skips the latch and we
  // stay on the single-endpoint path — safe degradation.
  if (g_sentai_tpu_multi_ep_routing && !s_sentai_multi_bo_ep_applied) {
    s_sentai_multi_bo_ep_applied = 1;  // latch before attempt to avoid retry storms
    if (!const_cast<TpuDriver*>(this)->Write64(
            chip_config_.GetUsbCsrOffsets().multi_bo_ep, 1)) {
      printf("[multi_ep] failed to write multi_bo_ep=1, staying on single EP\r\n");
      g_sentai_tpu_multi_ep_routing = 0;
    } else {
      printf("[multi_ep] routing enabled: multi_bo_ep=1\r\n");
    }
  }

  const uint8_t out_ep = endpoint_for_tag(tag);
  // In multi-endpoint mode the TPU routes by endpoint number alone — the
  // 8-byte [length|tag] header must be OMITTED (libedgetpu/driver/usb/
  // usb_driver.cc lines 696-732 vs 780-838: header is only ever sent on
  // kSingleBulkOutEndpoint in single-EP mode).  Sending a header on EP 2/3
  // in multi-EP mode poisons the DMA descriptor parse and hangs the TPU
  // (confirmed empirically before this fix was discovered).
  if (g_sentai_tpu_multi_ep_routing == 0) {
    if (!WriteHeader(tag, length, out_ep)) {
      // No printf: counters (async_stats) already track this.  Under
      // pipeline load, these failures happen per-frame, and
      // printf → USB CDC-ACM → USB device task competing with USB
      // host task → more timeouts → more printfs.  The feedback
      // loop was measured to amplify the underlying timeout rate.
      return false;
    }
  }

  if (!BulkOutTransfer(out_ep, data, length, tag)) {
    return false;
  }
  return true;
}

// Raw call counters — each Send* increments these once per call,
// regardless of any cache/skip path beneath.  Visible from REPL via
// sentai.diag.tpu_call_stats().  Used to verify how many Send* calls
// the TFLite/Apex stack actually issues per Invoke().
extern "C" volatile uint32_t g_sentai_tpu_send_params_calls   = 0;
extern "C" volatile uint32_t g_sentai_tpu_send_params_bytes   = 0;
extern "C" volatile uint32_t g_sentai_tpu_send_ins_calls      = 0;
extern "C" volatile uint32_t g_sentai_tpu_send_ins_bytes      = 0;
extern "C" volatile uint32_t g_sentai_tpu_send_inputs_calls   = 0;
extern "C" volatile uint32_t g_sentai_tpu_send_inputs_bytes   = 0;
// Forward decl — actual definition appears with the fine-grained sync
// block ~200 lines below; we need it here for sentai_tpu_call_stats.
extern "C" volatile uint32_t g_sentai_tpu_input_done_count;

extern "C" void sentai_tpu_call_stats(uint32_t* p_calls, uint32_t* p_bytes,
                                       uint32_t* i_calls, uint32_t* i_bytes,
                                       uint32_t* in_calls, uint32_t* in_bytes,
                                       uint32_t* in_done) {
    if (p_calls)  *p_calls  = g_sentai_tpu_send_params_calls;
    if (p_bytes)  *p_bytes  = g_sentai_tpu_send_params_bytes;
    if (i_calls)  *i_calls  = g_sentai_tpu_send_ins_calls;
    if (i_bytes)  *i_bytes  = g_sentai_tpu_send_ins_bytes;
    if (in_calls) *in_calls = g_sentai_tpu_send_inputs_calls;
    if (in_bytes) *in_bytes = g_sentai_tpu_send_inputs_bytes;
    if (in_done)  *in_done  = g_sentai_tpu_input_done_count;
}
extern "C" void sentai_tpu_call_reset(void) {
    g_sentai_tpu_send_params_calls = 0;
    g_sentai_tpu_send_params_bytes = 0;
    g_sentai_tpu_send_ins_calls    = 0;
    g_sentai_tpu_send_ins_bytes    = 0;
    g_sentai_tpu_send_inputs_calls = 0;
    g_sentai_tpu_send_inputs_bytes = 0;
    g_sentai_tpu_input_done_count  = 0;
}

bool TpuDriver::SendParameters(const uint8_t *data, uint32_t length) const {
  g_sentai_tpu_send_params_calls++;
  g_sentai_tpu_send_params_bytes += length;
  return SendData(DescriptorTag::kParameters, data, length);
}

// sentai: zero-copy bulk-out toggle.  Default ON — directly submits
// the caller's source buffer to USB.  When OFF, falls back to a
// staging memcpy → DTCM buffer → USB path (the pre-V9 safe path).
//
// Zero-copy is 100% safe for params/instructions (static flatbuffer
// in SDRAM, never concurrently written).  For INPUTS in pipeline
// mode the ping-pong input buffer has concurrent PXP writer +
// M7-quant writer + USB DMA reader.  Cache coherency between M7
// dirty lines and PXP's DMA-into-SDRAM is handled by
// DCACHE_CleanByRange inside USB_HostSend, but some subtle race
// between PXP IOC / cache clean / USB submit still causes TPU
// invoke failures (`E:0420:2`) under load.  Staging into DTCM
// sidesteps this by copying via M7 (cache-coherent) to an
// uncached buffer before USB touches it.
//
// `g_sentai_tpu_zero_copy_input` controls INPUT-phase only.  Params
// and instructions always use zero-copy (they're bit-identical
// static data, never concurrent-write hazard).
extern "C" volatile int g_sentai_tpu_zero_copy_input = 1;
extern "C" int  sentai_tpu_zero_copy_input_get(void) { return g_sentai_tpu_zero_copy_input; }
extern "C" void sentai_tpu_zero_copy_input_set(int v) { g_sentai_tpu_zero_copy_input = v ? 1 : 0; }

// Staging buffer in DTCM for the safe fallback path.  32 KB is
// enough for ONE chunk at the current kChunk; the outer loop
// breaks a bigger transfer into chunks.  Aligned 32 B for USB
// DMA + cache-line boundary.
static uint8_t s_bulk_staging[32 * 1024] __attribute__((aligned(32)));

// sentai: runtime-tunable bulk-OUT chunk size for fast A/B sweeps without
// reflashing.  ARM keeps the 36 KB value found by the 2026-04-22 hardware
// sweep.  SIM/POSIX uses a larger libusb-friendly default, validated on the
// COCO cat smoke at 19+ FPS after parameter warmup.
// Range is clamped to [4 KB, 160 KB] on ARM.  SIM/POSIX can sweep up to the
// libedgetpu default 1 MB bulk-out transfer size because it uses libusb
// zero-copy buffers rather than the ARM EHCI/OCRAM staging constraints.
#ifdef SENTAI_PLATFORM_SIM
extern "C" volatile uint32_t g_sentai_tpu_chunk_size =
    160 * 1024;
#else
extern "C" volatile uint32_t g_sentai_tpu_chunk_size = 36 * 1024;
#endif
extern "C" uint32_t sentai_tpu_chunk_size_get(void) { return g_sentai_tpu_chunk_size; }
extern "C" void     sentai_tpu_chunk_size_set(uint32_t n) {
    if (n < 4096) n = 4096;
#ifdef SENTAI_PLATFORM_SIM
    if (n > 1024 * 1024) n = 1024 * 1024;
#else
    if (n > 160 * 1024) n = 160 * 1024;
#endif
    g_sentai_tpu_chunk_size = n;
}

// sentai: async-input toggle.  When enabled, SendInputs uses a
// pipelined path that keeps 2 URBs in flight on the input pipe —
// while URB[i] is on the wire, the EHCI schedule already has
// URB[i+1] queued behind it, so the second transfer starts the
// instant the first's IOC fires.  Requires per-transfer callback
// support in usb_host_edgetpu.c (`USB_HostEdgeTpuBulkOutSendAsync`).
extern "C" volatile int g_sentai_tpu_async_input_enabled = 0;
extern "C" int  sentai_tpu_async_input_get(void) { return g_sentai_tpu_async_input_enabled; }
extern "C" void sentai_tpu_async_input_set(int v) { g_sentai_tpu_async_input_enabled = v ? 1 : 0; }

// Separate persistent semaphore for the async pipelined path — keeps
// it isolated from the legacy synchronous bulk slot sema.  One sema
// per outstanding slot so we can correlate completions.  Two slots =
// two semas; ping-pong.
struct AsyncSlot {
  SemaphoreHandle_t sema;
  volatile ssize_t  result;
};
static AsyncSlot s_async_slots[2];
static volatile int s_async_slots_inited = 0;
static void InitAsyncSlots(void) {
    if (__atomic_load_n(&s_async_slots_inited, __ATOMIC_ACQUIRE)) return;
    vTaskSuspendAll();
    if (!s_async_slots_inited) {
        for (int i = 0; i < 2; ++i) {
            s_async_slots[i].sema = xSemaphoreCreateBinary();
            s_async_slots[i].result = 0;
        }
        __atomic_store_n(&s_async_slots_inited, 1, __ATOMIC_RELEASE);
    }
    xTaskResumeAll();
}

static void AsyncSlotCallback(void *param, uint8_t *, uint32_t data_length,
                              usb_status_t status) {
    AsyncSlot *slot = static_cast<AsyncSlot *>(param);
    slot->result = (status == kStatus_USB_Success)
                       ? static_cast<ssize_t>(data_length)
                       : -static_cast<ssize_t>(status);
    xSemaphoreGive(slot->sema);
}

// Pipelined bulk OUT specifically for the input-activations path.
// At steady state, one URB is on the wire while the next is
// memcpy'd nothing (zero-copy — we submit from the source pointer
// directly).  The second URB is submitted BEFORE waiting on the
// first, so EHCI queues it at QH-tail and starts it the instant
// the first's IOC fires — no CPU-side RTT between chunks.
static bool BulkOutTransferPipelined(usb_host_edgetpu_instance_t *usb,
                                     uint8_t endpoint,
                                     const uint8_t *data,
                                     uint32_t data_length) {
  if (data_length == 0) return true;
  InitAsyncSlots();
  const uint8_t *src = data;
  uint32_t remain = data_length;
  uint32_t kChunk = g_sentai_tpu_chunk_size;

  // Drain stale sema signals.
  for (int i = 0; i < 2; ++i) {
      (void)xSemaphoreTake(s_async_slots[i].sema, 0);
      s_async_slots[i].result = 0;
  }

  // Prime slot 0.
  uint32_t n0 = std::min<uint32_t>(kChunk, remain);
  usb_status_t st0 = USB_HostEdgeTpuBulkOutSendAsync(
      usb, endpoint, const_cast<uint8_t *>(src), n0,
      AsyncSlotCallback, &s_async_slots[0]);
  if (st0 != kStatus_USB_Success) {
      printf("BulkOutPipelined prime submit failed (%d)\r\n", st0);
      return false;
  }
  src    += n0;
  remain -= n0;
  int cur = 0;

  while (remain > 0) {
      int nxt = cur ^ 1;
      uint32_t nn = std::min<uint32_t>(kChunk, remain);

      // Submit the NEXT chunk BEFORE waiting on the current one.
      // EHCI appends it to the QH tail; it starts the moment the
      // current QTD's IOC fires.
      usb_status_t st = USB_HostEdgeTpuBulkOutSendAsync(
          usb, endpoint, const_cast<uint8_t *>(src), nn,
          AsyncSlotCallback, &s_async_slots[nxt]);
      if (st != kStatus_USB_Success) {
          printf("BulkOutPipelined submit failed (%d)\r\n", st);
          // Still drain the in-flight slot to keep pool healthy.
          (void)xSemaphoreTake(s_async_slots[cur].sema,
                                pdMS_TO_TICKS(2000));
          return false;
      }

      // Now wait for the current slot — it may already be done.
      if (xSemaphoreTake(s_async_slots[cur].sema,
                          pdMS_TO_TICKS(2000)) == pdFALSE) {
          printf("BulkOutPipelined slot[%d] sema timeout\r\n", cur);
          return false;
      }
      if (s_async_slots[cur].result <= 0) {
          printf("BulkOutPipelined slot[%d] bad result %ld\r\n",
                 cur, (long)s_async_slots[cur].result);
          return false;
      }
      s_async_slots[cur].result = 0;

      src    += nn;
      remain -= nn;
      cur     = nxt;
  }

  // Drain final in-flight slot.
  if (xSemaphoreTake(s_async_slots[cur].sema,
                      pdMS_TO_TICKS(2000)) == pdFALSE) {
      printf("BulkOutPipelined final slot[%d] sema timeout\r\n", cur);
      return false;
  }
  return s_async_slots[cur].result > 0;
}

// Staged BulkOut specifically for input.  Forces the staging path
// even when zero-copy is the default everywhere else.  Used when
// the caller's source buffer races with a concurrent writer (PXP
// ISR + M7 quantization in pipeline mode) — the extra memcpy
// through DTCM provides a clean cache-coherent snapshot.
static bool BulkOutTransferStaged(usb_host_edgetpu_instance_t *usb,
                                  uint8_t endpoint,
                                  const uint8_t *data,
                                  uint32_t data_length) {
  if (data_length == 0) return true;
  const uint8_t *src = data;
  uint32_t remain = data_length;
  uint32_t kChunk = g_sentai_tpu_chunk_size;
  if (kChunk > sizeof(s_bulk_staging)) kChunk = sizeof(s_bulk_staging);

  while (remain > 0) {
    uint32_t nn = std::min<uint32_t>(kChunk, remain);
    memcpy(s_bulk_staging, src, nn);  // M7 → DTCM, cache-coherent
    UsbTransferMetadata meta;
#ifdef SENTAI_PLATFORM_SIM
    meta.sema = nullptr;
#else
    InitBulkSema();
    meta.sema = s_bulk_sema;
    (void)xSemaphoreTake(s_bulk_sema, 0);
#endif
    meta.status = kStatus_USB_Error;
    meta.bytes_transferred = 0;
    meta.tag = DescriptorTag::kInputActivations;
    meta.bytes_requested = nn;
    meta.submit_start_cyc = TpuUrbCycleNow();
    meta.submit_start_us = TpuUrbMicrosNow();
    meta.submit_return_cyc = meta.submit_start_cyc;
    meta.submit_return_us = meta.submit_start_us;
    meta.callback_cyc = 0;
    meta.callback_us = 0;
    usb_status_t st = USB_HostEdgeTpuBulkOutSend(
        usb, endpoint, s_bulk_staging, nn,
        [](void *param, uint8_t *, uint32_t len, usb_status_t s) {
            UsbTransferMetadata *m = static_cast<UsbTransferMetadata *>(param);
            if (!m) return;
            m->bytes_transferred = len;
            m->status = s;
            m->callback_cyc = TpuUrbCycleNow();
            m->callback_us = TpuUrbMicrosNow();
#ifndef SENTAI_PLATFORM_SIM
            if (m->sema) xSemaphoreGive(m->sema);
#endif
        },
        &meta);
    meta.submit_return_cyc = TpuUrbCycleNow();
    meta.submit_return_us = TpuUrbMicrosNow();
    if (st != kStatus_USB_Success) {
        TpuUrbRecordSubmitFailure(DescriptorTag::kInputActivations, nn,
                                  TpuUrbDelta(meta.submit_return_cyc,
                                              meta.submit_start_cyc),
                                  TpuUrbDelta(meta.submit_return_us,
                                              meta.submit_start_us));
        printf("BulkOutStaged submit failed (%d)\r\n", st);
        return false;
    }
#ifdef SENTAI_PLATFORM_SIM
    TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
    if (meta.status != kStatus_USB_Success || meta.bytes_transferred != nn) {
        printf("BulkOutStaged bad sync result\r\n");
        return false;
    }
#else
    if (xSemaphoreTake(meta.sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
        TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), true);
        printf("BulkOutStaged timeout\r\n");
        return false;
    }
    TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
    if (meta.status != kStatus_USB_Success) {
        printf("BulkOutStaged bad result\r\n");
        return false;
    }
#endif
    src    += nn;
    remain -= nn;
  }
  return true;
}

// Fine-grained pipeline sync (2026-04-25): SendInputs signals this
// semaphore at the very end of a successful input bulk-OUT phase.
// InferTask in detection_task.cc sets the pointer to s_sem_bufs_free
// before invoke and clears it after, so PrepTask can start writing the
// NEXT frame's .tpu_input the instant USB is done reading the current
// frame -- instead of having to wait for the full invoke (compute +
// output) to finish.  Eliminates the PrepTask-writes-while-InferTask-
// reads race on .tpu_input OCRAM without reducing parallelism.
// Null-safe: when no pipeline owns it, SendInputs gives nothing.
extern "C" volatile SemaphoreHandle_t g_sentai_tpu_input_done_sema = nullptr;
extern "C" volatile uint32_t          g_sentai_tpu_input_done_count = 0;

bool TpuDriver::SendInputs(const uint8_t *data, uint32_t length) const {
  g_sentai_tpu_send_inputs_calls++;
  g_sentai_tpu_send_inputs_bytes += length;
  // `.tpu_input` lives in OCRAM (V22 baseline), so SendInputs reads via
  // AXBS — not SEMC — and zero-copy is the production path.  When
  // zero-copy is disabled (diagnostic A/B), route input through the
  // staged DTCM path so we keep cache coherency with concurrent writers.
  // Parameters and instructions always zero-copy: static flatbuffer
  // data, no concurrent writer.
  bool ok;
  if (!g_sentai_tpu_zero_copy_input) {
      if (g_sentai_tpu_multi_ep_routing == 0) {
          uint8_t header[8];
          PrepareHeaderInto(DescriptorTag::kInputActivations, length, header);
          if (!BulkOutTransfer(kSingleBulkOutEndpoint, header, 8,
                               DescriptorTag::kInputActivations)) {
            return false;
          }
          ok = BulkOutTransferStaged(usb_instance_,
                                     kSingleBulkOutEndpoint,
                                     data, length);
      } else {
          ok = BulkOutTransferStaged(usb_instance_,
                                     kOutEpInputActivations,
                                     data, length);
      }
  } else if (g_sentai_tpu_async_input_enabled) {
      // Multi-EP header is OMITTED in multi-EP mode; in single-EP
      // mode the legacy SendData path writes a header then uses
      // BulkOutTransfer.  Here we mirror the single-EP path but
      // bypass BulkOutTransfer to use the pipelined async variant.
      if (g_sentai_tpu_multi_ep_routing == 0) {
          uint8_t header[8];
          PrepareHeaderInto(DescriptorTag::kInputActivations, length, header);
          if (!BulkOutTransfer(kSingleBulkOutEndpoint, header, 8,
                               DescriptorTag::kInputActivations)) {
            return false;
          }
          ok = BulkOutTransferPipelined(
              usb_instance_, kSingleBulkOutEndpoint, data, length);
      } else {
          // Multi-EP path — dedicated input endpoint, no header.
          ok = BulkOutTransferPipelined(
              usb_instance_, kOutEpInputActivations, data, length);
      }
  } else {
      ok = SendData(DescriptorTag::kInputActivations, data, length);
  }

  // Fine-grained pipeline sync: signal that USB no longer needs the
  // .tpu_input OCRAM buffer so PrepTask can start writing the NEXT
  // frame's bytes while InferTask continues with compute + output.
  // ONE-SHOT semantic: we consume the pointer (atomically swap to null)
  // and give exactly once per arm.  TFLite's invoke() calls SendInputs
  // twice per invoke for models with parameter_caching_exe (the caching
  // path + the inference path each feed input); without the one-shot
  // consume we'd give sem_bufs_free twice, letting PrepTask over-run
  // and burn SDRAM bandwidth on throwaway frames (measured: 2:1 prep
  // to infer ratio).  InferTask rearms before each invoke.  Bare REPL
  // tpu.invoke() never arms the sema, so this is a no-op there.
  // Failure path leaves the pointer intact so InferTask's post-invoke
  // cleanup can decide whether to give manually.
  if (ok) {
      SemaphoreHandle_t s = reinterpret_cast<SemaphoreHandle_t>(
          __atomic_exchange_n(
              reinterpret_cast<void* volatile*>(&g_sentai_tpu_input_done_sema),
              static_cast<void*>(nullptr),
              __ATOMIC_ACQ_REL));
      if (s) {
          xSemaphoreGive(s);
          g_sentai_tpu_input_done_count++;
      }
  }
  return ok;
}

// External hook for detection_task.cc (or any consumer) to register /
// clear the per-pipeline sema signaled at the end of SendInputs.  Safe
// to call from task context; atomic release-store so the next
// SendInputs either sees the full new value or null.
extern "C" void sentai_tpu_set_input_done_sema(SemaphoreHandle_t sema) {
    __atomic_store_n(&g_sentai_tpu_input_done_sema, sema, __ATOMIC_RELEASE);
}

bool TpuDriver::SendInstructions(const uint8_t *data, uint32_t length) const {
  g_sentai_tpu_send_ins_calls++;
  g_sentai_tpu_send_ins_bytes += length;
  return SendData(DescriptorTag::kInstructions, data, length);
}

bool TpuDriver::GetOutputs(uint8_t *data, uint32_t length) const {
  return BulkInTransfer(data, length, DescriptorTag::kOutputActivations);
}

// Wide-model activation spill: device->host (OUTFEED) leg.  Same bulk-IN
// path as GetOutputs -- the DescriptorTag is host-side bookkeeping only (no
// tag byte is sent on IN transfers).  Draining this transfer is what keeps
// the bulk-IN stream in sync; skipping it (the original port's behaviour)
// makes the next GetOutputs read spill bytes instead of the real output and
// wedges the endpoint.
bool TpuDriver::GetScratch(uint8_t *data, uint32_t length) const {
  return BulkInTransfer(data, length, DescriptorTag::kOutputActivations);
}

// Wide-model activation spill: host->device (INFEED) leg.  The spilled bytes
// drained earlier are handed back to the TPU's activation infeed DMA, so we
// route them on the input-activation tag (single-EP: the 8-byte header's tag
// nibble selects the on-chip destination FIFO; there is no dedicated scratch
// tag in the USB protocol).
bool TpuDriver::SendScratch(const uint8_t *data, uint32_t length) const {
  return SendData(DescriptorTag::kInputActivations, data, length);
}

bool TpuDriver::Read32(uint64_t reg, uint32_t *val) {
  return CSRTransfer(reg, val, true, RegisterSize::kRegSize32);
}

bool TpuDriver::Read64(uint64_t reg, uint64_t *val) {
  return CSRTransfer(reg, val, true, RegisterSize::kRegSize64);
}

bool TpuDriver::Write32(uint64_t reg, uint32_t val) {
  return CSRTransfer(reg, &val, false, RegisterSize::kRegSize32);
}

bool TpuDriver::Write64(uint64_t reg, uint64_t val) {
  return CSRTransfer(reg, &val, false, RegisterSize::kRegSize64);
}

extern "C" volatile uint32_t g_sentai_tpu_lambda_entered = 0;
extern "C" volatile uint32_t g_sentai_tpu_lambda_gave = 0;
extern "C" volatile uint32_t g_sentai_tpu_lambda_null_sema = 0;
extern "C" volatile uint32_t g_sentai_tpu_take_failed = 0;
extern "C" volatile uint32_t g_sentai_tpu_take_succeeded = 0;

// Fault-tolerance knob: per-URB wait ceiling (ms) before we call
// USB_HostEdgeTpuCancelInFlight and declare the URB lost.  Default
// 50 ms — 5× the observed p99 for our current workload.  Shorter
// recovery than the legacy 2000 ms cap means a stuck URB costs us
// one frame, not 40.  Caller (InferTask) treats -1 return as
// "drop this frame" and advances.  Runtime-tunable via
// sentai.diag.tpu_urb_timeout_ms(n).
// 200 ms — BulkIn issues its receive BEFORE TPU finishes compute.
// The sema fires when the output activations arrive, which happens
// after ~16 ms TPU compute + wire time.  Under pipeline load the
// compute path can stretch past 50 ms for a brief window; 200 ms
// is a fault-tolerant ceiling (~10× nominal) below which we
// don't declare the URB lost.  50 ms was too aggressive for Bulk
// IN; too permissive for Bulk OUT is a non-issue since OUT URBs
// complete fast even under load.
extern "C" volatile uint32_t g_sentai_tpu_urb_timeout_ms = 200;
extern "C" uint32_t sentai_tpu_urb_timeout_ms_get(void) { return g_sentai_tpu_urb_timeout_ms; }
extern "C" void     sentai_tpu_urb_timeout_ms_set(uint32_t n) {
    if (n < 5)    n = 5;
    if (n > 5000) n = 5000;
    g_sentai_tpu_urb_timeout_ms = n;
}

// Cancelled-frames counter — exposed so operators can track how
// often the fault-tolerance path fires.
extern "C" volatile uint32_t g_sentai_tpu_urb_cancelled = 0;
extern "C" volatile uint32_t g_sentai_tpu_urb_cancel_no_cb = 0;
extern "C" volatile uint8_t g_sentai_tpu_trace;
extern "C" volatile int g_sentai_tpu_sim_break_on_short_bulkin = 0;
extern "C" volatile uint32_t g_sentai_tpu_sim_bulkin_chunk_size = 256;
extern "C" volatile uint32_t g_sentai_tpu_sim_outfeed_chunk_length = 0x20;
extern "C" volatile int g_sentai_tpu_sim_sleep_after_bulkin = 0;
extern "C" volatile uint32_t g_sentai_tpu_sim_bulkin_queue_depth = 0;

#ifdef SENTAI_PLATFORM_SIM
constexpr int kSimBulkInMaxQueueDepth = 32;
constexpr uint32_t kSimBulkInMaxChunkSize = 1024;

struct SimBulkInSlot {
  StaticSemaphore_t sema_mem;
  SemaphoreHandle_t sema;
  volatile usb_status_t status;
  volatile uint32_t requested;
  volatile uint32_t actual;
};

static SimBulkInSlot s_sim_bulkin_slots[kSimBulkInMaxQueueDepth];
static uint8_t s_sim_bulkin_buffers[kSimBulkInMaxQueueDepth]
                                   [kSimBulkInMaxChunkSize]
    __attribute__((aligned(32)));
static volatile int s_sim_bulkin_slots_ready = 0;

static void InitSimBulkInSlots(void) {
  if (s_sim_bulkin_slots_ready) return;
  for (int i = 0; i < kSimBulkInMaxQueueDepth; ++i) {
    s_sim_bulkin_slots[i].sema =
        xSemaphoreCreateBinaryStatic(&s_sim_bulkin_slots[i].sema_mem);
    s_sim_bulkin_slots[i].status = kStatus_USB_Error;
    s_sim_bulkin_slots[i].requested = 0;
    s_sim_bulkin_slots[i].actual = 0;
  }
  s_sim_bulkin_slots_ready = 1;
}

static void SimBulkInCallback(void *param, uint8_t *, uint32_t data_length,
                              usb_status_t status) {
  SimBulkInSlot *slot = static_cast<SimBulkInSlot *>(param);
  if (!slot) return;
  slot->actual = data_length;
  slot->status = status;
  if (slot->sema) xSemaphoreGive(slot->sema);
}

static bool SimBulkInTransferQueued(usb_host_edgetpu_instance_t *usb,
                                    uint8_t endpoint,
                                    uint8_t *dst,
                                    uint32_t data_length,
                                    uint32_t chunk_size,
                                    uint32_t queue_depth) {
  if (!usb || !dst || data_length == 0) return data_length == 0;
  if (chunk_size == 0 || chunk_size > kSimBulkInMaxChunkSize) return false;
  if (queue_depth > kSimBulkInMaxQueueDepth) {
    queue_depth = kSimBulkInMaxQueueDepth;
  }
  if (queue_depth < 2) return false;
  InitSimBulkInSlots();

  uint32_t completed = 0;
  uint32_t inflight_requested = 0;
  uint32_t active = 0;
  uint32_t submit_idx = 0;
  uint32_t consume_idx = 0;

  auto submit_one = [&]() -> bool {
    if (completed + inflight_requested >= data_length) return true;
    SimBulkInSlot *slot = &s_sim_bulkin_slots[submit_idx];
    if (!slot->sema) return false;
    while (xSemaphoreTake(slot->sema, 0) == pdTRUE) {}
    const uint32_t remaining = data_length - completed - inflight_requested;
    const uint32_t request = std::min<uint32_t>(chunk_size, remaining);
    slot->requested = request;
    slot->actual = 0;
    slot->status = kStatus_USB_Error;
    usb_status_t st = USB_HostEdgeTpuBulkInRecvAsync(
        usb, endpoint, s_sim_bulkin_buffers[submit_idx], request,
        SimBulkInCallback, slot);
    if (st != kStatus_USB_Success) return false;
    inflight_requested += request;
    ++active;
    submit_idx = (submit_idx + 1) % queue_depth;
    return true;
  };

  while (active < queue_depth && completed + inflight_requested < data_length) {
    if (!submit_one()) return false;
  }

  while (completed < data_length) {
    if (active == 0) {
      if (!submit_one()) return false;
      continue;
    }
    SimBulkInSlot *slot = &s_sim_bulkin_slots[consume_idx];
    if (xSemaphoreTake(slot->sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
      return false;
    }
    --active;
    if (inflight_requested >= slot->requested) {
      inflight_requested -= slot->requested;
    } else {
      inflight_requested = 0;
    }
    const usb_status_t status = slot->status;
    const uint32_t actual = slot->actual;
    if (actual == 0) return false;
    if (status != kStatus_USB_Success &&
        !(status == kStatus_USB_TransferFailed &&
          actual > 0 && actual < slot->requested)) {
      return false;
    }
    const uint32_t copy_n =
        std::min<uint32_t>(actual, data_length - completed);
    memcpy(dst + completed, s_sim_bulkin_buffers[consume_idx], copy_n);
    completed += copy_n;
    consume_idx = (consume_idx + 1) % queue_depth;
    while (active < queue_depth &&
           completed + inflight_requested < data_length) {
      if (!submit_one()) return false;
    }
  }
  return true;
}
#endif

__attribute__((noinline, cold, section(".sdram_text")))
static void trace_bulkout(char where, uint32_t v1, uint32_t v2) {
  printf("[bulkout] %c v1=%lu v2=%lu\r\n", where,
         (unsigned long)v1, (unsigned long)v2);
}

ssize_t TpuDriver::BulkOutTransferInternal(uint8_t endpoint,
                                           const uint8_t *data,
                                           uint32_t data_length,
                                           DescriptorTag tag) const {
#ifdef SENTAI_PLATFORM_SIM
  UsbTransferMetadata meta;
  meta.sema = nullptr;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = tag;
  meta.bytes_requested = data_length;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;
  usb_status_t bulk_status = USB_HostEdgeTpuBulkOutSend(
      usb_instance_, endpoint, (uint8_t *)data, data_length,
      [](void *param, uint8_t *, uint32_t data_length,
         usb_status_t status) {
        g_sentai_tpu_lambda_entered++;
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        if (!meta) return;
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
        g_sentai_tpu_lambda_gave++;
      },
      &meta);
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();
  if (bulk_status != kStatus_USB_Success) {
    TpuUrbRecordSubmitFailure(tag, data_length,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    return -(ssize_t)bulk_status;
  }
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
  if (meta.status == kStatus_USB_Success) {
    g_sentai_tpu_take_succeeded++;
    return meta.bytes_transferred;
  }
  return -meta.status;
#else
  InitBulkSema();
  UsbTransferMetadata meta;
  meta.sema   = s_bulk_sema;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = tag;
  meta.bytes_requested = data_length;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;
  (void)xSemaphoreTake(s_bulk_sema, 0);

  usb_status_t bulk_status = USB_HostEdgeTpuBulkOutSend(
      usb_instance_, endpoint, (uint8_t *)data, data_length,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        g_sentai_tpu_lambda_entered++;
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        if (!meta || !meta->sema) { g_sentai_tpu_lambda_null_sema++; return; }
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
        BaseType_t r = xSemaphoreGive(meta->sema);
        if (r == pdTRUE) g_sentai_tpu_lambda_gave++;
      },
      &meta);
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();

  if (bulk_status != kStatus_USB_Success) {
    // No printf — feedback loop through USB CDC-ACM.  Counters
    // (bo_send in async_stats) track submit failures.
    TpuUrbRecordSubmitFailure(tag, data_length,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    return -(ssize_t)bulk_status;
  }

  if (xSemaphoreTake(meta.sema, pdMS_TO_TICKS(g_sentai_tpu_urb_timeout_ms))
        == pdFALSE) {
    g_sentai_tpu_take_failed++;
    TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), true);
    // NO CANCEL.  User theory: cancelling a partial bulk-OUT leaves
    // the TPU device in an undefined state (it expected a complete
    // block).  Confirmed empirically: after pipeline runs + cancels,
    // even standalone `tpu.invoke()` fails identically until full
    // reflash — so the TPU silicon was being corrupted, not just
    // the host-side pipe.
    //
    // New strategy: **pretend we don't care** — just skip the frame.
    // The URB is still in the EHCI async schedule; its callback
    // will fire eventually on stack-allocated `meta` (safe: we
    // return but the stack slot for `meta` is reused by the next
    // caller, and the lambda's `meta->bytes_transferred = len;
    // meta->status = status; xSemaphoreGive(meta->sema)` writes
    // to valid addresses even if `meta` now represents a different
    // in-flight transfer — the give is idempotent on a binary sema,
    // the bytes/status fields are overwritten by the new caller
    // anyway).  Worst case: next call's Take returns early via a
    // stale give — but THAT take's own in-flight URB will still
    // complete legit later.  Net effect: occasional duplicate
    // wake, no TPU corruption, no cascade.
    return -1;
  } else {
    g_sentai_tpu_take_succeeded++;
  }
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);

  if (meta.status == kStatus_USB_Success) {
    return meta.bytes_transferred;
  } else {
    return -meta.status;
  }
#endif
}

// Zero-copy bulk OUT.  Old code memcpy'd each 32 KB chunk from the
// caller's source buffer (SDRAM tensor_arena / flatbuffer vector)
// into a DTCM staging buffer, then EHCI DMA'd from DTCM.  That
// double-hop added ~0.1-0.2 ms per 32 KB chunk of pure CPU memcpy
// on top of the USB wire time, and capped chunks at 32 KB.
//
// USB_HostSend (usb_host_hci.c:385) already does a
// DCACHE_CleanByRange on the transfer buffer before submission when
// USB_HOST_CONFIG_BUFFER_PROPERTY_CACHEABLE=1 (see usb_host_config.h:99),
// so cache coherency is handled for arbitrary source regions —
// we can submit directly from the flatbuffer in SDRAM (cacheable).
//
// Larger chunks reduce per-URB host overhead on the POSIX/libusb bridge.  ARM
// remains tuned separately via g_sentai_tpu_chunk_size's non-SIM default.
bool TpuDriver::BulkOutTransfer(uint8_t endpoint,
                                const uint8_t *data,
                                uint32_t data_length,
                                DescriptorTag tag) const {
  const uint8_t *current_chunk = data;
  uint32_t bytes_left = data_length;
  // Snapshot once at loop entry so mid-transfer reconfiguration cannot split a
  // URB unevenly.
  uint32_t kChunk = g_sentai_tpu_chunk_size;

  while (bytes_left > 0) {
    uint32_t chunk_size = std::min<uint32_t>(kChunk, bytes_left);
    if (g_sentai_tpu_trace) {
      trace_bulkout('B', chunk_size, bytes_left);
    }
    ssize_t bytes_sent = BulkOutTransferInternal(
        endpoint, current_chunk, chunk_size, tag);
    if (bytes_sent > 0) {
      if (g_sentai_tpu_trace) {
        trace_bulkout('b', (uint32_t)bytes_sent, bytes_left - (uint32_t)bytes_sent);
      }
      current_chunk += bytes_sent;
      bytes_left    -= bytes_sent;
#ifdef SENTAI_PLATFORM_SIM
      /*
       * POSIX/libusb can complete consecutive bulk OUT URBs faster than the
       * FreeRTOS POSIX scheduler and Coral USB app stay balanced during the
       * first multi-megabyte parameter upload.  The real ARM path is paced by
       * the USB host stack/interrupt task; in SIM we explicitly yield one tick
       * between chunks so libusb event handling and lower-priority runtime
       * tasks keep making progress.  POSIX TPU throughput is diagnostic, not a
       * flight-performance target.
       */
      sentai_sim_host_sleep_1ms();
#endif
    } else {
      return false;  // printf removed: CDC-ACM feedback loop
    }
  }
  return true;
}

extern "C" volatile uint8_t g_sentai_tpu_trace;
__attribute__((noinline, cold, section(".sdram_text")))
static void trace_bulkin(char where, uint32_t v1, uint32_t v2) {
  printf("[bulkin] %c v1=%lu v2=%lu\r\n", where,
         (unsigned long)v1, (unsigned long)v2);
}

ssize_t TpuDriver::BulkInTransferInternal(uint8_t endpoint, uint8_t *data,
                                          uint32_t data_length,
                                          DescriptorTag tag) const {
#ifdef SENTAI_PLATFORM_SIM
  UsbTransferMetadata meta;
  meta.sema = nullptr;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = tag;
  meta.bytes_requested = data_length;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;
  if (g_sentai_tpu_trace) trace_bulkin('S', endpoint, data_length);
  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, endpoint, data, data_length,
      [](void *param, uint8_t *, uint32_t data_length,
         usb_status_t status) {
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        if (!meta) return;
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
      },
      &meta);
  if (bulk_status != kStatus_USB_Success) {
    meta.submit_return_cyc = TpuUrbCycleNow();
    meta.submit_return_us = TpuUrbMicrosNow();
    TpuUrbRecordSubmitFailure(tag, data_length,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    if (g_sentai_tpu_trace) trace_bulkin('e', (uint32_t)bulk_status, 0);
    return -(ssize_t)bulk_status;
  }
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
  if (g_sentai_tpu_trace)
    trace_bulkin('D', meta.bytes_transferred, (uint32_t)meta.status);
  if (meta.status == kStatus_USB_Success) return meta.bytes_transferred;
  if (meta.status == kStatus_USB_TransferFailed &&
      meta.bytes_transferred > 0 &&
      meta.bytes_transferred < data_length) {
    return meta.bytes_transferred;
  }
  return -meta.status;
#else
  InitBulkSema();
  UsbTransferMetadata meta;
  meta.sema   = s_bulk_sema;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = tag;
  meta.bytes_requested = data_length;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;
  (void)xSemaphoreTake(s_bulk_sema, 0);

  if (g_sentai_tpu_trace) trace_bulkin('S', endpoint, data_length);

  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, endpoint, data, data_length,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
        xSemaphoreGive(meta->sema);
      },
      &meta);
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();

  if (bulk_status != kStatus_USB_Success) {
    TpuUrbRecordSubmitFailure(tag, data_length,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    if (g_sentai_tpu_trace) trace_bulkin('e', (uint32_t)bulk_status, 0);
    printf("USB_HostEdgeTpuBulkInRecv failed\r\n");
    return -(ssize_t)bulk_status;
  }

  if (xSemaphoreTake(meta.sema, pdMS_TO_TICKS(g_sentai_tpu_urb_timeout_ms))
        == pdFALSE) {
    g_sentai_tpu_take_failed++;
    TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), true);
    if (g_sentai_tpu_trace)
      trace_bulkin('T', g_sentai_tpu_urb_timeout_ms, data_length);
    // Skip without cancel — see BulkOutTransferInternal for
    // rationale.  Cancel was corrupting TPU state.
    return -1;
  }
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);

  if (g_sentai_tpu_trace)
    trace_bulkin('D', meta.bytes_transferred, (uint32_t)meta.status);
  if (meta.status == kStatus_USB_Success) {
    return meta.bytes_transferred;
  }
  /* USB Bulk-IN short transfer: NXP's EHCI host stack reports a
   * legitimate short-packet termination (device done, < requested
   * bytes) as kStatus_USB_TransferFailed (= 11) with bytes_transferred
   * set to whatever the device sent.  yolo_1 outputs are exact 64-byte
   * multiples so this case never fires there; iarna's 3rd output
   * (declared 7200 B, dma_hint requests 9600 B padded) terminates at
   * 9472 B + status=Failed → the call loop bailed out as error and
   * the whole invoke was rejected.  Treat short-but-nonzero as
   * success and return the actual bytes; the outer BulkInTransfer
   * loop interprets a short return as end-of-data and exits cleanly. */
  if (meta.status == kStatus_USB_TransferFailed &&
      meta.bytes_transferred > 0 &&
      meta.bytes_transferred < data_length) {
    if (g_sentai_tpu_trace)
      trace_bulkin('s', meta.bytes_transferred, data_length);
    return meta.bytes_transferred;
  }
  return -meta.status;
#endif
}

// Zero-copy bulk IN.  Old code received into DTCM staging, memcpy'd
// out to caller.  New: receive directly into caller's buffer —
// USB_HostRecv (usb_host_hci.c:501) does DCACHE_CleanInvalidateByRange
// on the transfer buffer around submission so the caller sees fresh
// data without our memcpy.
bool TpuDriver::BulkInTransfer(uint8_t *data, uint32_t data_length,
                               DescriptorTag tag) const {
  uint8_t *current_chunk = data;
  uint32_t bytes_left = data_length;
  // Snapshot once at loop entry so mid-transfer reconfiguration cannot split a
  // URB unevenly.  SIM overrides bulk-IN to 256B by default, matching the
  // libedgetpu USB2 policy.
  uint32_t kChunk = g_sentai_tpu_chunk_size;  // see BulkOutTransfer
#ifdef SENTAI_PLATFORM_SIM
  if (g_sentai_tpu_sim_bulkin_chunk_size > 0) {
    kChunk = g_sentai_tpu_sim_bulkin_chunk_size;
  }
  if (g_sentai_tpu_sim_bulkin_queue_depth > 1 &&
      kChunk <= kSimBulkInMaxChunkSize) {
    return SimBulkInTransferQueued(
        usb_instance_, kSingleBulkOutEndpoint, data, data_length, kChunk,
        g_sentai_tpu_sim_bulkin_queue_depth);
  }
#endif
  while (bytes_left > 0) {
    uint32_t chunk_size = std::min<uint32_t>(kChunk, bytes_left);
    ssize_t bytes_received = BulkInTransferInternal(
        kSingleBulkOutEndpoint, current_chunk, chunk_size, tag);
    if (bytes_received > 0) {
      current_chunk += bytes_received;
      bytes_left    -= (uint32_t)bytes_received;
#ifdef SENTAI_PLATFORM_SIM
      if (g_sentai_tpu_sim_sleep_after_bulkin) {
        sentai_sim_host_sleep_1ms();
      }
      if ((uint32_t)bytes_received < chunk_size &&
          g_sentai_tpu_sim_break_on_short_bulkin == 0) {
        continue;
      }
#endif
      /* Short transfer = device terminated stream.  Don't issue
       * another URB; remaining buffer stays as caller initialised
       * (zero from arena init or prior content).  Output activations
       * smaller than the dma_hint padded size are valid; OutputLayer
       * Relayout extracts only the real tensor bytes.  POSIX/libusb keeps
       * the historical continue-by-default behavior because it can surface
       * short successful transfers before the whole TPU output stream has
       * been drained. */
      if ((uint32_t)bytes_received < chunk_size) break;
    } else {
      return false;  // printf removed: CDC-ACM feedback loop
    }
  }
  return true;
}

// PrepareHeader writes the 8-byte [length|tag|zeros] descriptor
// into `out` — NO HEAP ALLOCATION.  Legacy std::vector variant
// retained for out-of-tree callers that still expect it; new code
// calls PrepareHeaderInto().  The hot path (WriteHeader +
// SendInputs staged/async branches) uses only the stack buffer.
void TpuDriver::PrepareHeaderInto(DescriptorTag tag, uint32_t length,
                                  uint8_t out[8]) {
  // [0..3] = length (little-endian), [4] = tag (4-bit), [5..7] = 0
  memset(out, 0, 8);
  memcpy(out, &length, sizeof(length));
  out[sizeof(length)] = static_cast<uint8_t>(tag) & 0xF;
}

std::vector<uint8_t> TpuDriver::PrepareHeader(DescriptorTag tag,
                                              uint32_t length) const {
  std::vector<uint8_t> header_packet(8);
  PrepareHeaderInto(tag, length, header_packet.data());
  return header_packet;
}

bool TpuDriver::WriteHeader(DescriptorTag tag, uint32_t length,
                            uint8_t endpoint) const {
  uint8_t header[8];
  PrepareHeaderInto(tag, length, header);
  return BulkOutTransfer(endpoint, header, 8, tag);
}

bool TpuDriver::ReadEvent() const {
  // No-heap hot path: 16-byte event buffer + sema are static/singleton.
  // Called once per Invoke; eliminating the per-invoke OSA_MemoryAllocate
  // (SDRAM heap) + xSemaphoreCreateBinary churn removes one source of
  // SDRAM bus contention observed during pipeline load.
  constexpr size_t kEventSizeBytes = 16;
  static uint8_t s_event_buf[kEventSizeBytes]
      __attribute__((aligned(32), section(".sdram_bss")));
#ifdef SENTAI_PLATFORM_SIM
  UsbTransferMetadata meta;
  meta.sema = nullptr;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = DescriptorTag::kInterrupt0;
  meta.bytes_requested = kEventSizeBytes;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;
  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, kEventInEndpoint, s_event_buf, kEventSizeBytes,
      [](void *param, uint8_t *, uint32_t data_length,
         usb_status_t status) {
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        if (!meta) return;
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
      },
      &meta);
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();
  if (bulk_status != kStatus_USB_Success) {
    TpuUrbRecordSubmitFailure(DescriptorTag::kInterrupt0, kEventSizeBytes,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    return false;
  }
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
  return bulk_status == kStatus_USB_Success;
#else
  static StaticSemaphore_t s_event_sema_mem;
  static SemaphoreHandle_t s_event_sema = nullptr;
  if (!s_event_sema) {
    s_event_sema = xSemaphoreCreateBinaryStatic(&s_event_sema_mem);
  }
  // Drain any stale give from a prior timed-out call.
  (void)xSemaphoreTake(s_event_sema, 0);
  UsbTransferMetadata meta;
  meta.sema = s_event_sema;
  meta.status = kStatus_USB_Error;
  meta.bytes_transferred = 0;
  meta.tag = DescriptorTag::kInterrupt0;
  meta.bytes_requested = kEventSizeBytes;
  meta.submit_start_cyc = TpuUrbCycleNow();
  meta.submit_start_us = TpuUrbMicrosNow();
  meta.submit_return_cyc = meta.submit_start_cyc;
  meta.submit_return_us = meta.submit_start_us;
  meta.callback_cyc = 0;
  meta.callback_us = 0;

  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, kEventInEndpoint, s_event_buf, kEventSizeBytes,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        (void)data;
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        if (!meta) return;
        meta->bytes_transferred = data_length;
        meta->status = status;
        meta->callback_cyc = TpuUrbCycleNow();
        meta->callback_us = TpuUrbMicrosNow();
        xSemaphoreGive(meta->sema);
      },
      &meta);
  meta.submit_return_cyc = TpuUrbCycleNow();
  meta.submit_return_us = TpuUrbMicrosNow();
  if (bulk_status != kStatus_USB_Success) {
    TpuUrbRecordSubmitFailure(DescriptorTag::kInterrupt0, kEventSizeBytes,
                              TpuUrbDelta(meta.submit_return_cyc,
                                          meta.submit_start_cyc),
                              TpuUrbDelta(meta.submit_return_us,
                                          meta.submit_start_us));
    return false;
  }
  if (xSemaphoreTake(s_event_sema, pdMS_TO_TICKS(2000)) != pdTRUE) {
    TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), true);
    return false;
  }
  TpuUrbRecordDone(meta, TpuUrbCycleNow(), TpuUrbMicrosNow(), false);
  return meta.status == kStatus_USB_Success;
#endif
}

bool TpuDriver::DoRunControl(platforms::darwinn::driver::RunControl run_state) {
  const uint64_t run_state_value = static_cast<uint64_t>(run_state);
  CHECK(Write64(chip_config_.GetScalarCoreCsrOffsets().scalarCoreRunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetScalarCoreCsrOffsets().avDataPopRunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetScalarCoreCsrOffsets().parameterPopRunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetScalarCoreCsrOffsets().infeedRunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetScalarCoreCsrOffsets().outfeedRunControl,
                run_state_value));

  registers::TileConfig<7> helper;
  helper.set_broadcast();
  CHECK(Write64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                helper.raw()));

  // Wait until tileconfig0 is set correctly. Subsequent writes are going to
  // tiles, but hardware does not guarantee correct ordering with previous
  // write.  Bounded: same kMaxPollIter rationale as Initialize().
  uint64_t tileconfig0_reg;
  {
    constexpr int kMaxPollIter = 10000;
    int iter = 0;
    do {
      CHECK(Read64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                   &tileconfig0_reg));
      if (++iter >= kMaxPollIter) {
        printf("[TPU] DoRunControl poll timeout: tileconfig0 write-back\r\n");
        return false;
      }
    } while (tileconfig0_reg != helper.raw());
  }

  if (chip_config_.GetTileCsrOffsets().opRunControl !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_0 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_0,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_1 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_1,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_2 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_2,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_3 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_3,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_4 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_4,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_5 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_5,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_6 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_6,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().opRunControl_7 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().opRunControl_7,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_0 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_0,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_1 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_1,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_2 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_2,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_3 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_3,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_4 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_4,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_5 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_5,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_6 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_6,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().narrowToWideRunControl_7 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToWideRunControl_7,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_0 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_0,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_1 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_1,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_2 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_2,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_3 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_3,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_4 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_4,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_5 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_5,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_6 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_6,
                  run_state_value));
  }
  if (chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_7 !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().wideToNarrowRunControl_7,
                  run_state_value));
  }

  CHECK(Write64(chip_config_.GetTileCsrOffsets().meshBus0RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().meshBus1RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().meshBus2RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().meshBus3RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().ringBusConsumer0RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().ringBusConsumer1RunControl,
                run_state_value));
  CHECK(Write64(chip_config_.GetTileCsrOffsets().ringBusProducerRunControl,
                run_state_value));
  if (chip_config_.GetTileCsrOffsets().narrowToNarrowRunControl !=
      static_cast<uint64_t>(-1)) {
    CHECK(Write64(chip_config_.GetTileCsrOffsets().narrowToNarrowRunControl,
                  run_state_value));
  }

  return true;
}

float TpuDriver::GetTemperature() {
  uint32_t omc0_dc_reg;
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_dc, &omc0_dc_reg));
  registers::Omc0DC omc0_dc(omc0_dc_reg);
  float temperature = (662 - omc0_dc.data()) * 250 + 550;
  // temerature is currently in mC, divide by 1000 for C.
  return temperature / 1000;
}

// sentai: JTAG-readable snapshot of TPU error CSRs at the instant of an invoke
// fault. Layout (uint32 words):
//   [0] magic 0xFA17C500 | (code & 0xff)   (written LAST = "valid" marker)
//   [1] hib_error_status        [2] hib_first_error_status
//   [3] hib_first_error_tstamp  [4] top_level_int_status
//   [5] sc_host_int_count       [6] dma_paused
//   [7] scalarCoreRunControl    [8] currentPc
//   [9] read-ok bitmask (bit i set => CSR i read returned ok)
//   [10] invoke_fail_code
extern "C" volatile uint32_t g_sentai_tpu_fault_csr[16] = {0};

void TpuDriver::LatchErrorCsrs(uint16_t code) const {
  const auto& hib = chip_config_.GetHibUserCsrOffsets();
  const auto& dsc = chip_config_.GetDebugScalarCoreCsrOffsets();
  uint32_t okmask = 0;
  int idx = 1;
  auto rd = [&](uint64_t off) {
    uint64_t v = 0;
    bool ok = const_cast<TpuDriver*>(this)->Read64(off, &v);
    if (ok) okmask |= (1u << idx);
    g_sentai_tpu_fault_csr[idx] = (uint32_t)(v & 0xffffffffu);
    idx++;
  };
  rd(hib.hib_error_status);          // [1]
  rd(hib.hib_first_error_status);    // [2]
  rd(hib.hib_first_error_timestamp); // [3]
  rd(hib.top_level_int_status);      // [4]
  rd(hib.sc_host_int_count);         // [5]
  rd(hib.dma_paused);                // [6]
  rd(dsc.scalarCoreRunControl);      // [7]
  rd(dsc.currentPc);                 // [8]
  g_sentai_tpu_fault_csr[9]  = okmask;
  g_sentai_tpu_fault_csr[10] = code;
  // write the valid marker LAST so a JTAG reader can tell the snapshot is complete
  g_sentai_tpu_fault_csr[0] = 0xFA17C500u | (code & 0xffu);
}

void TpuDriver::DumpErrorCsrs() const {
  const auto& hib = chip_config_.GetHibUserCsrOffsets();
  const auto& dsc = chip_config_.GetDebugScalarCoreCsrOffsets();
  auto rd = [this](const char* name, uint64_t off) {
    uint64_t v = 0;
    bool ok = const_cast<TpuDriver*>(this)->Read64(off, &v);
    printf("  %-26s off=0x%08lx ok=%d val=0x%08lx%08lx\r\n", name,
           (unsigned long)off, ok ? 1 : 0,
           (unsigned long)(v >> 32), (unsigned long)(v & 0xffffffffu));
  };
  printf("[tpu_csr] HIB + scalar-core status:\r\n");
  rd("hib_error_status",        hib.hib_error_status);
  rd("hib_first_error_status",  hib.hib_first_error_status);
  rd("hib_first_error_tstamp",  hib.hib_first_error_timestamp);
  rd("top_level_int_status",    hib.top_level_int_status);
  rd("sc_host_int_count",       hib.sc_host_int_count);
  rd("dma_paused",              hib.dma_paused);
  rd("scalarCoreRunControl",    dsc.scalarCoreRunControl);
  rd("currentPc",               dsc.currentPc);
}

}  // namespace coralmicro
