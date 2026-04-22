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

#include "libs/base/check.h"
#include "libs/tpu/darwinn/driver/config/beagle/beagle_chip_config.h"
#include "libs/tpu/darwinn/driver/config/beagle_csr_helper.h"
#include "libs/tpu/darwinn/driver/config/common_csr_helper.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/nxp/rt1176-sdk/components/osa/fsl_os_abstraction.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb_spec.h"

namespace coralmicro {
namespace {
// EdgeTPU USB endpoint layout (observed identically on both single_ep and
// multi_ep firmware variants, per sentai_usb_edgetpu_dump_eps):
//   OUT 1, 2, 3 — bulk OUT, 512-byte packets
//   IN  1, 2    — bulk IN , 512-byte packets
//   IN  3       — interrupt, 64-byte packets
// The default driver path sends ALL bulk OUT traffic (params, instructions,
// inputs) on a single endpoint and reads outputs + events on EP 2.  When
// per-tag routing is enabled (g_sentai_tpu_multi_ep_routing=1) we spread
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
extern "C" volatile int g_sentai_tpu_multi_ep_routing = 0;
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
    __attribute__((section(".ocram_bss,\"aw\",%nobits @")))
    __attribute__((aligned(32)));

struct UsbTransferMetadata {
  SemaphoreHandle_t sema;
  usb_status_t status;
  size_t bytes_transferred;
};

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
}  // namespace

namespace registers = platforms::darwinn::driver::config::registers;

bool TpuDriver::Initialize(usb_host_edgetpu_instance_t *usb_instance,
                           PerformanceMode mode) {
  if (usb_instance == nullptr) {
    return false;
  }
  usb_instance_ = usb_instance;

  // Check chip id and test write
  uint32_t omc0_00_reg;
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_00, &omc0_00_reg));

  registers::Omc000 omc0_00(omc0_00_reg);
  CHECK(0x89A == omc0_00.chip_id());

  omc0_00.set_test_reg0(0xAA);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_00, omc0_00.raw()));

  omc0_00_reg = 0;
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_00, &omc0_00_reg));
  omc0_00.set_raw(omc0_00_reg);
  CHECK(0xAA == omc0_00.test_reg0());

  // Disable inactive mode
  uint32_t scu_ctrl_0_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, &scu_ctrl_0_reg));
  registers::ScuCtrl0 scu_ctrl_0(scu_ctrl_0_reg);
  scu_ctrl_0.set_rg_pcie_inact_phy_mode(0);
  scu_ctrl_0.set_rg_usb_inact_phy_mode(0);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, scu_ctrl_0.raw()));
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_0, &scu_ctrl_0_reg));

  // Disable clock gating
  uint32_t scu_ctrl_2_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));
  registers::ScuCtrl2 scu_ctrl_2(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(0x2);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, scu_ctrl_2.raw()));
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));

  // Go into reset, if we're not there
  uint32_t scu_ctrl_3_reg;
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
  registers::ScuCtrl3 scu_ctrl_3(scu_ctrl_3_reg);
  if (scu_ctrl_3.rg_force_sleep() != 0x3) {
    scu_ctrl_3.set_rg_force_sleep(0x3);
    CHECK(
        Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, scu_ctrl_3.raw()));
    do {
      CHECK(
          Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
      scu_ctrl_3.set_raw(scu_ctrl_3_reg);
    } while (scu_ctrl_3.cur_pwr_state() != 0x2);
    CHECK(Write32(chip_config_.GetCbBridgeCsrOffsets().gcbb_credit0, 0xF));
    CHECK(Write32(chip_config_.GetCbBridgeCsrOffsets().gcbb_credit0, 0x0));
  }

  // Set performance mode and exit reset.
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

  do {
    CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_3, &scu_ctrl_3_reg));
    scu_ctrl_3.set_raw(scu_ctrl_3_reg);
  } while (scu_ctrl_3.cur_pwr_state() != 0x0);

  // Check a known register to verify reset exit.
  uint64_t scalar_core_run_control;
  do {
    CHECK(Read64(chip_config_.GetScalarCoreCsrOffsets().scalarCoreRunControl,
                 &scalar_core_run_control));
  } while (scalar_core_run_control != 0);

  registers::IdleRegister idle_reg;
  idle_reg.set_enable();
  idle_reg.set_counter(1);
  CHECK(Write64(chip_config_.GetMiscCsrOffsets().idleRegister, idle_reg.raw()));

  registers::TileConfig<7> tile_config;
  tile_config.set_broadcast();
  CHECK(Write64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                tile_config.raw()));

  uint64_t tile_config_reg;
  do {
    CHECK(Read64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                 &tile_config_reg));
  } while (tile_config.raw() != tile_config_reg);

  registers::DeepSleep deep_sleep_reg;
  deep_sleep_reg.set_to_sleep_delay(2);
  deep_sleep_reg.set_to_wake_delay(30);
  CHECK(Write64(chip_config_.GetTileCsrOffsets().deepSleep,
                deep_sleep_reg.raw()));

  // Enable clock gating
  CHECK(Read32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, &scu_ctrl_2_reg));
  scu_ctrl_2.set_raw(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(1);
  CHECK(Write32(chip_config_.GetScuCsrOffsets().scu_ctrl_2, scu_ctrl_2.raw()));

  CHECK(Write64(chip_config_.GetUsbCsrOffsets().descr_ep, 0xF0));
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().multi_bo_ep, 0));
  // NB: 0x20 (256 B) is required on NXP RT1176 EHCI — empirically tested
  // 0x80 (1 KB) which broke bulk-in reads entirely (0 frames through
  // pipeline).  libedgetpu (driver/usb/usb_driver.cc:349-374) documents
  // this as a b/73181174 "short packet" workaround; our host controller
  // evidently needs it.  Keeping 0x20 matches the shipped behaviour.
  CHECK(Write64(chip_config_.GetUsbCsrOffsets().outfeed_chunk_length, 0x20));

  uint32_t omc0_d0_reg, omc0_d8_reg, omc0_dc_reg;

  // Enables tempsense clock.
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_d0, &omc0_d0_reg));
  registers::Omc0D0 omc0_d0(omc0_d0_reg);
  omc0_d0.set_clk_en(0x1);
  omc0_d0.set_adr(0xC);
  omc0_d0.set_tref(0);
  omc0_d0.set_tslope(0);
  omc0_d0.set_t_setting(0);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_d0, omc0_d0.raw()));

  // Enables tempsense input ports.
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_d8, &omc0_d8_reg));
  registers::Omc0D8 omc0_d8(omc0_d8_reg);
  omc0_d8.set_enbg(0x1);
  omc0_d8.set_envr(0x1);
  omc0_d8.set_enad(0x1);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_d8, omc0_d8.raw()));

  // Wait 100 us before enabling tempsense flow.
  SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CpuClk));

  // Enables tempsense flow.
  CHECK(Read32(chip_config_.GetApexCsrOffsets().omc0_dc, &omc0_dc_reg));
  registers::Omc0DC omc0_dc(omc0_dc_reg);
  omc0_dc.set_enthmc(0x1);
  CHECK(Write32(chip_config_.GetApexCsrOffsets().omc0_dc, omc0_dc.raw()));

  CHECK(DoRunControl(platforms::darwinn::driver::RunControl::kMoveToRun));

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

  SemaphoreHandle_t sema = xSemaphoreCreateBinary();

  control_status = USB_HostEdgeTpuControl(
      usb_instance_, &setup_packet, (uint8_t *)data,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        SemaphoreHandle_t sema = (SemaphoreHandle_t)param;
        xSemaphoreGive(sema);
      },
      sema);
  if (control_status != kStatus_USB_Success) {
    printf("USB_HostEdgeTpuControl failed\r\n");
    goto exit;
  }
  if (xSemaphoreTake(sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
    ret = false;
    printf("%s didn't get semaphore\r\n", __func__);
    goto exit;
  }

  ret = true;
exit:
  vSemaphoreDelete(sema);
  return ret;
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
      printf("WriteHeader failed\r\n");
      return false;
    }
  }

  if (!BulkOutTransfer(out_ep, data, length)) {
    printf("BulkOutTransfer failed\r\n");
    return false;
  }
  return true;
}

bool TpuDriver::SendParameters(const uint8_t *data, uint32_t length) const {
  return SendData(DescriptorTag::kParameters, data, length);
}

// sentai: runtime-tunable bulk chunk size for fast A/B sweeps
// without reflashing.  Default 64 KB (empirical V11 sweet spot).
// Range enforced at the use site: clamped to [4 KB, 160 KB].  160 KB
// is the hard ceiling from USB_HOST_CONFIG_EHCI_MAX_QTD=16 × 16 KB
// data-per-QTD (see NXP fsl_usb_host_ehci.c:2052 and our
// usb_host_config.h comment).
// 33 KB default — empirical sweet spot found 2026-04-22 via a sweep
// of 4..160 KB on this YOLO 512 workload.  Below ~36 KB the invoke
// completes in ~14 ms; at 38 KB and above it jumps to ~37 ms (a
// ~2.5× cliff).  Hypothesis: the EdgeTPU's bulk-OUT receive FIFO
// is ~32-36 KB; URBs that fit inside let the TPU stream them into
// its inference pipe without back-pressure, while larger URBs
// stall the USB while the FIFO drains.  33 KB (33792 B) was the
// specific minimum in the sweep — see agent/experiment.md for the
// full table.
extern "C" volatile uint32_t g_sentai_tpu_chunk_size = 33 * 1024;
extern "C" uint32_t sentai_tpu_chunk_size_get(void) { return g_sentai_tpu_chunk_size; }
extern "C" void     sentai_tpu_chunk_size_set(uint32_t n) {
    if (n < 4096) n = 4096;
    if (n > 160 * 1024) n = 160 * 1024;
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

bool TpuDriver::SendInputs(const uint8_t *data, uint32_t length) const {
  if (g_sentai_tpu_async_input_enabled) {
      // Multi-EP header is OMITTED in multi-EP mode; in single-EP
      // mode the legacy SendData path writes a header then uses
      // BulkOutTransfer.  Here we mirror the single-EP path but
      // bypass BulkOutTransfer to use the pipelined async variant.
      if (g_sentai_tpu_multi_ep_routing == 0) {
          std::vector<uint8_t> header =
              PrepareHeader(DescriptorTag::kInputActivations, length);
          if (!BulkOutTransfer(kSingleBulkOutEndpoint, header.data(),
                                header.size())) {
              return false;
          }
          return BulkOutTransferPipelined(
              usb_instance_, kSingleBulkOutEndpoint, data, length);
      }
      // Multi-EP path — dedicated input endpoint, no header.
      return BulkOutTransferPipelined(
          usb_instance_, kOutEpInputActivations, data, length);
  }
  return SendData(DescriptorTag::kInputActivations, data, length);
}

bool TpuDriver::SendInstructions(const uint8_t *data, uint32_t length) const {
  return SendData(DescriptorTag::kInstructions, data, length);
}

bool TpuDriver::GetOutputs(uint8_t *data, uint32_t length) const {
  return BulkInTransfer(data, length);
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

ssize_t TpuDriver::BulkOutTransferInternal(uint8_t endpoint,
                                           const uint8_t *data,
                                           uint32_t data_length) const {
  InitBulkSema();
  UsbTransferMetadata meta;
  meta.sema   = s_bulk_sema;
  meta.status = kStatus_USB_Error;
  (void)xSemaphoreTake(s_bulk_sema, 0);

  usb_status_t bulk_status = USB_HostEdgeTpuBulkOutSend(
      usb_instance_, endpoint, (uint8_t *)data, data_length,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        meta->bytes_transferred = data_length;
        meta->status = status;
        xSemaphoreGive(meta->sema);
      },
      &meta);

  if (bulk_status != kStatus_USB_Success) {
    printf("USB_HostEdgeTpuBulkOutSend failed\r\n");
    return -(ssize_t)bulk_status;
  }

  if (xSemaphoreTake(meta.sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
    printf("%s didn't get semaphore\r\n", __func__);
  };

  if (meta.status == kStatus_USB_Success) {
    return meta.bytes_transferred;
  } else {
    return -meta.status;
  }
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
// Chunk cap is now the legacy uint16_t length arg = 64 KB - 1.
// (A later pass can widen USB_HostEdgeTpuBulkOutSend to uint32_t to
// push chunks even higher.)  At 40 MB/s effective wire rate and
// ~100 µs per-URB submit overhead, going from 32 KB → 64 KB chunks
// halves the number of URBs for the ~786 KB input transfer: 24 →
// 12 submissions, shaving ~12 × 100 µs = 1.2 ms extra.
bool TpuDriver::BulkOutTransfer(uint8_t endpoint,
                                const uint8_t *data,
                                uint32_t data_length) const {
  const uint8_t *current_chunk = data;
  uint32_t bytes_left = data_length;
  // Runtime-tunable chunk size via g_sentai_tpu_chunk_size (MP:
  // sentai.diag.tpu_chunk_size(n)).  Default 64 KB.  Snapshot once
  // at loop entry so mid-transfer reconfiguration can't split a
  // URB unevenly.
  uint32_t kChunk = g_sentai_tpu_chunk_size;

  while (bytes_left > 0) {
    uint32_t chunk_size = std::min<uint32_t>(kChunk, bytes_left);
    ssize_t bytes_sent = BulkOutTransferInternal(
        endpoint, current_chunk, chunk_size);
    if (bytes_sent > 0) {
      current_chunk += bytes_sent;
      bytes_left    -= bytes_sent;
    } else {
      printf("Bad BulkOutTransferInternal\r\n");
      return false;
    }
  }
  return true;
}

ssize_t TpuDriver::BulkInTransferInternal(uint8_t endpoint, uint8_t *data,
                                          uint32_t data_length) const {
  InitBulkSema();
  UsbTransferMetadata meta;
  meta.sema   = s_bulk_sema;
  meta.status = kStatus_USB_Error;
  (void)xSemaphoreTake(s_bulk_sema, 0);

  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, endpoint, data, data_length,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        UsbTransferMetadata *meta = static_cast<UsbTransferMetadata *>(param);
        meta->bytes_transferred = data_length;
        meta->status = status;
        xSemaphoreGive(meta->sema);
      },
      &meta);

  if (bulk_status != kStatus_USB_Success) {
    printf("USB_HostEdgeTpuBulkInRecv failed\r\n");
    return -(ssize_t)bulk_status;
  }

  if (xSemaphoreTake(meta.sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
    printf("%s didn't get semaphore\r\n", __func__);
  };

  if (meta.status == kStatus_USB_Success) {
    return meta.bytes_transferred;
  } else {
    return -meta.status;
  }
}

// Zero-copy bulk IN.  Old code received into DTCM staging, memcpy'd
// out to caller.  New: receive directly into caller's buffer —
// USB_HostRecv (usb_host_hci.c:501) does DCACHE_CleanInvalidateByRange
// on the transfer buffer around submission so the caller sees fresh
// data without our memcpy.
bool TpuDriver::BulkInTransfer(uint8_t *data, uint32_t data_length) const {
  uint8_t *current_chunk = data;
  uint32_t bytes_left = data_length;
  // Runtime-tunable chunk size via g_sentai_tpu_chunk_size (MP:
  // sentai.diag.tpu_chunk_size(n)).  Default 64 KB.  Snapshot once
  // at loop entry so mid-transfer reconfiguration can't split a
  // URB unevenly.
  uint32_t kChunk = g_sentai_tpu_chunk_size;  // see BulkOutTransfer
  while (bytes_left > 0) {
    uint32_t chunk_size = std::min<uint32_t>(kChunk, bytes_left);
    ssize_t bytes_received = BulkInTransferInternal(
        kSingleBulkOutEndpoint, current_chunk, chunk_size);
    if (bytes_received > 0) {
      current_chunk += bytes_received;
      bytes_left    -= (uint32_t)bytes_received;
    } else {
      printf("Bad BulkInTransferInternal\r\n");
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> TpuDriver::PrepareHeader(DescriptorTag tag,
                                              uint32_t length) const {
  constexpr size_t kPacketHeaderRawDataSizeInBytes = 8;
  constexpr size_t kLengthSizeInBytes = sizeof(length);
  std::vector<uint8_t> header_packet(kPacketHeaderRawDataSizeInBytes);
  std::fill(header_packet.begin(), header_packet.end(), 0);
  memcpy(header_packet.data(), &length, kLengthSizeInBytes);

  *(header_packet.data() + sizeof(kLengthSizeInBytes)) =
      (static_cast<uint8_t>(tag) & 0xF);

  return header_packet;
}

bool TpuDriver::WriteHeader(DescriptorTag tag, uint32_t length,
                            uint8_t endpoint) const {
  std::vector<uint8_t> header_packet = PrepareHeader(tag, length);
  return BulkOutTransfer(endpoint, header_packet.data(), header_packet.size());
}

bool TpuDriver::ReadEvent() const {
  bool ret = false;
  constexpr size_t kEventSizeBytes = 16;
  uint8_t *buf = (uint8_t *)OSA_MemoryAllocate(kEventSizeBytes);
  SemaphoreHandle_t sema = xSemaphoreCreateBinary();
  usb_status_t bulk_status = USB_HostEdgeTpuBulkInRecv(
      usb_instance_, kEventInEndpoint, buf, kEventSizeBytes,
      [](void *param, uint8_t *data, uint32_t data_length,
         usb_status_t status) {
        uint32_t len;
        uint64_t address;
        uint8_t tag;
        memcpy(&address, data, sizeof(address));
        memcpy(&len, data + sizeof(address), sizeof(len));
        tag = *(data + sizeof(address) + sizeof(len)) & 0xF;
        // For now, we don't do anything with these events we've read back.
        (void)tag;
        SemaphoreHandle_t sema = (SemaphoreHandle_t)param;
        xSemaphoreGive(sema);
      },
      sema);
  if (bulk_status != kStatus_USB_Success) {
    printf("ReadEvent failed\r\n");
    goto exit;
  }
  if (xSemaphoreTake(sema, pdMS_TO_TICKS(2000)) == pdFALSE) {
    goto exit;
  };
  ret = true;
exit:
  vSemaphoreDelete(sema);
  OSA_MemoryFree(buf);
  return ret;
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
  // write.
  uint64_t tileconfig0_reg;
  do {
    CHECK(Read64(chip_config_.GetTileConfigCsrOffsets().tileconfig0,
                 &tileconfig0_reg));
  } while (tileconfig0_reg != helper.raw());

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

}  // namespace coralmicro
