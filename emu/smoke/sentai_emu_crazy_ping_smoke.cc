// B9/s219 guest-side sentai.crazy bridge smoke.
//
// This target links the shared sentai_crazy.cc implementation and replaces
// only the serial transport below sentai_uart_serial_* with the emulator MMIO
// bridge.  It is intentionally a small FreeRTOS smoke before the full
// MicroPython `sentai.crazy` namespace target.

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "examples/sentai_runtime/sentai_crazy.h"
#include "examples/sentai_runtime/sentai_health.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_crazy_init_rc;
extern "C" volatile uint32_t g_sentai_emu_crazy_ping_ms;

extern "C" volatile uint32_t g_sentai_emu_crazy_serial_open_calls;
extern "C" volatile uint32_t g_sentai_emu_crazy_serial_write_calls;
extern "C" volatile uint32_t g_sentai_emu_crazy_serial_read_calls;
extern "C" volatile uint32_t g_sentai_emu_crazy_serial_bytes_tx;
extern "C" volatile uint32_t g_sentai_emu_crazy_serial_bytes_rx;
extern "C" volatile uint32_t g_sentai_emu_crazy_serial_last_result;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootTaskRunning = 0x0300;
constexpr uint32_t kBootInitOk = 0x0A00;
constexpr uint32_t kBootPingOk = 0x0B00;
constexpr uint32_t kBootPingTimeout = 0x0B03;
constexpr uint32_t kBootInitFail = 0xE901;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uintptr_t kLpuart6Base = 0x40090000u;
constexpr uint32_t kLpuartStatOffset = 0x14u;
constexpr uint32_t kLpuartCtrlOffset = 0x18u;
constexpr uint32_t kLpuartDataOffset = 0x1Cu;
constexpr uint32_t kLpuartStatTdre = 1u << 23;
constexpr uint32_t kLpuartCtrlRe = 1u << 18;
constexpr uint32_t kLpuartCtrlTe = 1u << 19;

StaticTask_t g_task_tcb;
StackType_t g_task_stack[configMINIMAL_STACK_SIZE * 32]
    __attribute__((aligned(8)));

volatile uint32_t& UartReg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kLpuart6Base + offset);
}

void UartInit() {
  UartReg(kLpuartCtrlOffset) = kLpuartCtrlRe | kLpuartCtrlTe;
}

void UartPutChar(char ch) {
  for (int i = 0; i < 1000; ++i) {
    if (UartReg(kLpuartStatOffset) & kLpuartStatTdre) break;
  }
  UartReg(kLpuartDataOffset) = static_cast<uint8_t>(ch);
}

void UartWrite(const char* s) {
  while (*s) UartPutChar(*s++);
}

void UartWriteDecSigned(int32_t value) {
  if (value < 0) {
    UartPutChar('-');
    value = -value;
  }
  char tmp[11];
  int n = 0;
  uint32_t v = static_cast<uint32_t>(value);
  do {
    tmp[n++] = static_cast<char>('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u && n < static_cast<int>(sizeof(tmp)));
  while (n > 0) UartPutChar(tmp[--n]);
}

void UartWriteDec(uint32_t value) {
  UartWriteDecSigned(static_cast<int32_t>(value));
}

void UartMetric(const char* name, int32_t value) {
  UartWrite(name);
  UartWrite("=");
  UartWriteDecSigned(value);
  UartWrite("\r\n");
}

void UartMetricU(const char* name, uint32_t value) {
  UartWrite(name);
  UartWrite("=");
  UartWriteDec(value);
  UartWrite("\r\n");
}

void SmokeTask(void*) {
  g_sentai_emu_boot_state = kBootTaskRunning;
  UartWrite("SentAI EMU Crazy bridge smoke\r\n");

  sentai_crazy_set_debug(0);
  const int init_rc = sentai_crazy_init(576000);
  g_sentai_emu_crazy_init_rc = static_cast<uint32_t>(init_rc);
  UartMetric("CRAZY_INIT_RC", init_rc);

  if (init_rc != 0) {
    g_sentai_emu_boot_state = kBootInitFail;
    while (true) {
      ++g_sentai_emu_heartbeat;
      g_sentai_emu_last_tick = xTaskGetTickCount();
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }

  g_sentai_emu_boot_state = kBootInitOk;
  vTaskDelay(pdMS_TO_TICKS(250));

  const int ping_ms = sentai_crazy_ping(1500);
  g_sentai_emu_crazy_ping_ms = static_cast<uint32_t>(ping_ms);
  UartMetric("CRAZY_PING_MS", ping_ms);
  UartMetricU("CRAZY_RUNNING", sentai_crazy_is_running() ? 1u : 0u);
  UartMetricU("SERIAL_OPEN_CALLS", g_sentai_emu_crazy_serial_open_calls);
  UartMetricU("SERIAL_WRITE_CALLS", g_sentai_emu_crazy_serial_write_calls);
  UartMetricU("SERIAL_READ_CALLS", g_sentai_emu_crazy_serial_read_calls);
  UartMetricU("SERIAL_BYTES_TX", g_sentai_emu_crazy_serial_bytes_tx);
  UartMetricU("SERIAL_BYTES_RX", g_sentai_emu_crazy_serial_bytes_rx);
  UartMetricU("SERIAL_LAST_RESULT", g_sentai_emu_crazy_serial_last_result);

  g_sentai_emu_boot_state = (ping_ms >= 0) ? kBootPingOk : kBootPingTimeout;

  sentai_crazy_stop();
  UartWrite("CRAZY_STOPPED\r\n");

  while (true) {
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_crazy_init_rc = 0xFFFFFFFFu;
volatile uint32_t g_sentai_emu_crazy_ping_ms = 0xFFFFFFFFu;
}

extern "C" void sentai_led_set(int on) {
  (void)on;
}

extern "C" void sentai_health_init(void) {}
extern "C" void sentai_health_success(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_fail(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_timeout(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_set_unavailable(SubsystemId_t subsys) {
  (void)subsys;
}
extern "C" void sentai_health_set_recovering(SubsystemId_t subsys) {
  (void)subsys;
}

extern "C" const HealthRecord_t* sentai_health_get(SubsystemId_t subsys) {
  (void)subsys;
  return nullptr;
}

extern "C" SystemMode_t sentai_health_system_mode(void) {
  return SYS_MODE_NORMAL;
}

extern "C" const char* sentai_health_state_name(HealthState_t state) {
  (void)state;
  return "emu";
}

extern "C" const char* sentai_health_subsys_name(SubsystemId_t subsys) {
  (void)subsys;
  return "emu";
}

extern "C" int sentai_health_summary(char* buf, int buf_size) {
  if (buf && buf_size > 0) buf[0] = '\0';
  return 0;
}

extern "C" void sentai_health_boot_complete(void) {}
extern "C" void sentai_health_set_recovery_mode(void) {}
extern "C" int sentai_health_is_safe_mode(void) { return 0; }

extern "C" void sentai_logf(const char* tag, const char* fmt, ...) {
  (void)tag;
  (void)fmt;
}

extern "C" int sentai_crazy_handler_is_set(void) {
  return 0;
}

extern "C" void sentai_crazy_request_drain(void) {}

int main() {
  UartInit();
  g_sentai_emu_boot_state = kBootEnteredMain;

  TaskHandle_t task = xTaskCreateStatic(
      SmokeTask, "crazy_smoke", sizeof(g_task_stack) / sizeof(g_task_stack[0]),
      nullptr, tskIDLE_PRIORITY + 2, g_task_stack, &g_task_tcb);
  if (!task) {
    g_sentai_emu_boot_state = kBootCreateTaskFailed;
    while (true) {}
  }

  g_sentai_emu_boot_state = kBootTaskCreated;
  vTaskStartScheduler();
  g_sentai_emu_boot_state = kBootSchedulerReturned;
  while (true) {}
}
