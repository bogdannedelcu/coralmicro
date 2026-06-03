// Minimal TpuDriver implementation for B8.8 emulator probes.
//
// Early gates only validate EdgeTpuManager::OpenDevice() sequencing.  The
// MMIO-send bridge gate intentionally moves one step deeper: it keeps the
// firmware-side injection point at the real TpuDriver methods used by
// EdgeTpuExecutable (SendParameters/SendInputs/SendInstructions/GetOutputs/
// ReadEvent), while Renode supplies the transport endpoint.

#include "libs/tpu/edgetpu_driver.h"

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

namespace coralmicro {

#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
namespace {

#if SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
constexpr uintptr_t kBridgeBase = 0x40901800u;
#else
constexpr uintptr_t kBridgeBase = 0x40900800u;
#endif
constexpr uint32_t kStatusIdle = 0;
constexpr uint32_t kStatusPending = 1;
constexpr uint32_t kStatusDone = 2;
constexpr uint32_t kStatusError = 3;

constexpr uint32_t kCmdSendParameters = 1;
constexpr uint32_t kCmdSendInputs = 2;
constexpr uint32_t kCmdSendInstructions = 3;
constexpr uint32_t kCmdGetOutputs = 4;
constexpr uint32_t kCmdReadEvent = 5;

enum RegOffset : uint32_t {
  kRegStatus = 0x00,
  kRegCommand = 0x04,
  kRegDataPtr = 0x08,
  kRegDataLen = 0x0C,
  kRegOutPtr = 0x10,
  kRegOutLen = 0x14,
  kRegResult = 0x18,
  kRegSeq = 0x1C,
};

volatile uint32_t& Reg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kBridgeBase + offset);
}

bool BridgeRequest(uint32_t command, const uint8_t* data, uint32_t data_len,
                   uint8_t* out, uint32_t out_len) {
  const uint32_t seq = Reg(kRegSeq) + 1;
  Reg(kRegDataPtr) = reinterpret_cast<uintptr_t>(data);
  Reg(kRegDataLen) = data_len;
  Reg(kRegOutPtr) = reinterpret_cast<uintptr_t>(out);
  Reg(kRegOutLen) = out_len;
  Reg(kRegCommand) = command;
  Reg(kRegResult) = 0xFFFFFFFFu;
  Reg(kRegSeq) = seq;
  Reg(kRegStatus) = kStatusPending;

  for (int i = 0; i < 1000; ++i) {
    const uint32_t status = Reg(kRegStatus);
    if (status == kStatusDone && Reg(kRegSeq) == seq) {
      return Reg(kRegResult) == 0;
    }
    if (status == kStatusError) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_tpu_bridge_params_calls = 0;
volatile uint32_t g_sentai_emu_tpu_bridge_input_calls = 0;
volatile uint32_t g_sentai_emu_tpu_bridge_ins_calls = 0;
volatile uint32_t g_sentai_emu_tpu_bridge_output_calls = 0;
volatile uint32_t g_sentai_emu_tpu_bridge_event_calls = 0;
volatile uint32_t g_sentai_emu_tpu_bridge_last_result = 0xFFFFFFFFu;
}
#endif

bool TpuDriver::Initialize(usb_host_edgetpu_instance_t* usb_instance,
                           PerformanceMode mode) {
  (void)usb_instance;
  (void)mode;
#if SENTAI_EMU_TPU_DRIVER_INIT_OK || SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  return true;
#else
  return false;
#endif
}

bool TpuDriver::SendParameters(const uint8_t* data, uint32_t length) const {
#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  ++g_sentai_emu_tpu_bridge_params_calls;
  const bool ok = BridgeRequest(kCmdSendParameters, data, length, nullptr, 0);
  g_sentai_emu_tpu_bridge_last_result = ok ? 0 : 1;
  return ok;
#else
  (void)data;
  (void)length;
  return false;
#endif
}

bool TpuDriver::SendInputs(const uint8_t* data, uint32_t length) const {
#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  ++g_sentai_emu_tpu_bridge_input_calls;
  const bool ok = BridgeRequest(kCmdSendInputs, data, length, nullptr, 0);
  g_sentai_emu_tpu_bridge_last_result = ok ? 0 : 2;
  return ok;
#else
  (void)data;
  (void)length;
  return false;
#endif
}

bool TpuDriver::SendInstructions(const uint8_t* data, uint32_t length) const {
#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  ++g_sentai_emu_tpu_bridge_ins_calls;
  const bool ok = BridgeRequest(kCmdSendInstructions, data, length, nullptr, 0);
  g_sentai_emu_tpu_bridge_last_result = ok ? 0 : 3;
  return ok;
#else
  (void)data;
  (void)length;
  return false;
#endif
}

bool TpuDriver::GetOutputs(uint8_t* data, uint32_t length) const {
#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  ++g_sentai_emu_tpu_bridge_output_calls;
  const bool ok = BridgeRequest(kCmdGetOutputs, nullptr, 0, data, length);
  g_sentai_emu_tpu_bridge_last_result = ok ? 0 : 4;
  return ok;
#else
  (void)data;
  (void)length;
  return false;
#endif
}

bool TpuDriver::ReadEvent() const {
#if SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE || \
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE
  ++g_sentai_emu_tpu_bridge_event_calls;
  const bool ok = BridgeRequest(kCmdReadEvent, nullptr, 0, nullptr, 0);
  g_sentai_emu_tpu_bridge_last_result = ok ? 0 : 5;
  return ok;
#else
  return false;
#endif
}

float TpuDriver::GetTemperature() { return 0.0f; }

}  // namespace coralmicro
