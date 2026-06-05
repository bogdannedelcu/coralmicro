// Emulator-only serial backend for sentai.crazy.
//
// The production ARM path sends sentai.crazy traffic through
// sentai_uart_serial_* backed by ConsoleM7's UART serial mode.  In Renode we
// keep the high-level sentai.crazy implementation, but terminate the serial
// calls at a small MMIO bridge so LPUART6 can remain the debug/REPL console.

#include <stdint.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

namespace {

constexpr uintptr_t kBridgeBase = 0x40902400u;

constexpr uint32_t kStatusIdle = 0;
constexpr uint32_t kStatusPending = 1;
constexpr uint32_t kStatusDone = 2;
constexpr uint32_t kStatusError = 3;

constexpr uint32_t kCmdOpen = 1;
constexpr uint32_t kCmdClose = 2;
constexpr uint32_t kCmdWrite = 3;
constexpr uint32_t kCmdRead = 4;
constexpr uint32_t kCmdAvailable = 5;
constexpr uint32_t kCmdSetBaudrate = 6;

enum RegOffset : uint32_t {
  kRegStatus = 0x00,
  kRegCommand = 0x04,
  kRegDataPtr = 0x08,
  kRegDataLen = 0x0C,
  kRegOutPtr = 0x10,
  kRegOutLen = 0x14,
  kRegResult = 0x18,
  kRegSeq = 0x1C,
  kRegValue = 0x20,
};

volatile uint32_t& Reg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kBridgeBase + offset);
}

int BridgeRequest(uint32_t command, const uint8_t* data, uint32_t data_len,
                  uint8_t* out, uint32_t out_len, uint32_t value) {
  const uint32_t seq = Reg(kRegSeq) + 1;
  Reg(kRegDataPtr) = reinterpret_cast<uintptr_t>(data);
  Reg(kRegDataLen) = data_len;
  Reg(kRegOutPtr) = reinterpret_cast<uintptr_t>(out);
  Reg(kRegOutLen) = out_len;
  Reg(kRegValue) = value;
  Reg(kRegCommand) = command;
  Reg(kRegResult) = 0xFFFFFFFFu;
  Reg(kRegSeq) = seq;
  Reg(kRegStatus) = kStatusPending;

  for (int i = 0; i < 1000; ++i) {
    const uint32_t status = Reg(kRegStatus);
    if (status == kStatusDone && Reg(kRegSeq) == seq) {
      return static_cast<int>(Reg(kRegResult));
    }
    if (status == kStatusError) {
      return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return -1;
}

}  // namespace

extern "C" {

volatile uint32_t g_sentai_emu_crazy_serial_open_calls = 0;
volatile uint32_t g_sentai_emu_crazy_serial_write_calls = 0;
volatile uint32_t g_sentai_emu_crazy_serial_read_calls = 0;
volatile uint32_t g_sentai_emu_crazy_serial_bytes_tx = 0;
volatile uint32_t g_sentai_emu_crazy_serial_bytes_rx = 0;
volatile uint32_t g_sentai_emu_crazy_serial_last_result = 0xFFFFFFFFu;

// Emulator reports "USB REPL" semantics so sentai.crazy is allowed to claim
// the serial transport while LPUART6 remains the Renode debug console.
int sentai_console_get_target(void) {
  return 0;
}

int sentai_console_set_target(int target) {
  (void)target;
  return 0;
}

int sentai_uart_serial_open(void) {
  ++g_sentai_emu_crazy_serial_open_calls;
  const int r = BridgeRequest(kCmdOpen, nullptr, 0, nullptr, 0, 0);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
  return r >= 0 ? 1 : 0;
}

void sentai_uart_serial_close(void) {
  const int r = BridgeRequest(kCmdClose, nullptr, 0, nullptr, 0, 0);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
}

int sentai_uart_serial_is_open(void) {
  return 1;
}

int sentai_uart_serial_write(const uint8_t* buf, int size) {
  if (!buf || size < 0) return -1;
  ++g_sentai_emu_crazy_serial_write_calls;
  const int r = BridgeRequest(kCmdWrite, buf, static_cast<uint32_t>(size),
                              nullptr, 0, 0);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
  if (r >= 0) {
    g_sentai_emu_crazy_serial_bytes_tx += static_cast<uint32_t>(r);
  }
  return r;
}

int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
  if (!buf || max_size < 0) return -1;
  ++g_sentai_emu_crazy_serial_read_calls;

  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout_ticks =
      (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

  while (true) {
    const int r = BridgeRequest(kCmdRead, nullptr, 0, buf,
                                static_cast<uint32_t>(max_size), 0);
    g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
    if (r > 0) {
      g_sentai_emu_crazy_serial_bytes_rx += static_cast<uint32_t>(r);
      return r;
    }
    if (r < 0) {
      return -1;
    }
    if (timeout_ms == 0) {
      return 0;
    }
    if (timeout_ms >= 0 &&
        (xTaskGetTickCount() - start) >= timeout_ticks) {
      return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

int sentai_uart_serial_available(void) {
  const int r = BridgeRequest(kCmdAvailable, nullptr, 0, nullptr, 0, 0);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
  return r < 0 ? 0 : r;
}

void sentai_uart_set_baudrate(uint32_t baudrate) {
  const int r = BridgeRequest(kCmdSetBaudrate, nullptr, 0, nullptr, 0, baudrate);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
}

void sentai_uart_restore_baudrate(void) {
  const int r = BridgeRequest(kCmdSetBaudrate, nullptr, 0, nullptr, 0, 115200u);
  g_sentai_emu_crazy_serial_last_result = static_cast<uint32_t>(r);
}

}  // extern "C"
