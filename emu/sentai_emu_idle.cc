// B8.1 ARM emulator smoke: boot CM7 startup and FreeRTOS, then idle.
//
// This target deliberately initializes no board peripherals.  Renode verifies
// progress by reading the globals below instead of relying on UART.

#include <stdint.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootHeartbeatRunning = 0x0300;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uint32_t kHeartbeatPeriodMs = 10;
constexpr uint32_t kHeartbeatStackWords = configMINIMAL_STACK_SIZE * 2;

#if SENTAI_EMU_UART_SMOKE
constexpr uintptr_t kLpuart6Base = 0x40090000u;
constexpr uint32_t kLpuartStatOffset = 0x14u;
constexpr uint32_t kLpuartCtrlOffset = 0x18u;
constexpr uint32_t kLpuartDataOffset = 0x1Cu;
constexpr uint32_t kLpuartStatTdre = 1u << 23;
constexpr uint32_t kLpuartCtrlRe = 1u << 18;
constexpr uint32_t kLpuartCtrlTe = 1u << 19;

volatile uint32_t& UartReg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kLpuart6Base + offset);
}

void UartInit() {
  UartReg(kLpuartCtrlOffset) = kLpuartCtrlRe | kLpuartCtrlTe;
}

void UartPutChar(char ch) {
  for (int i = 0; i < 1000; ++i) {
    if (UartReg(kLpuartStatOffset) & kLpuartStatTdre) {
      break;
    }
  }
  UartReg(kLpuartDataOffset) = static_cast<uint8_t>(ch);
}

void UartWrite(const char* s) {
  while (*s) {
    UartPutChar(*s++);
  }
}
#endif  // SENTAI_EMU_UART_SMOKE

StaticTask_t g_heartbeat_tcb;
StackType_t g_heartbeat_stack[kHeartbeatStackWords]
    __attribute__((aligned(8)));

void HeartbeatTask(void*) {
  g_sentai_emu_boot_state = kBootHeartbeatRunning;
#if SENTAI_EMU_UART_SMOKE
  UartWrite("SentAI EMU UART task online\r\n");
#endif
  while (true) {
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#if SENTAI_EMU_UART_SMOKE
    if ((g_sentai_emu_heartbeat % 10u) == 0u) {
      UartWrite("SentAI EMU UART heartbeat\r\n");
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(kHeartbeatPeriodMs));
  }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
}

extern "C" int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  g_sentai_emu_boot_state = kBootEnteredMain;
#if SENTAI_EMU_UART_SMOKE
  UartInit();
  UartWrite("SentAI EMU UART boot\r\n");
#endif
  TaskHandle_t task = xTaskCreateStatic(HeartbeatTask, "emu_heartbeat",
                                        kHeartbeatStackWords, nullptr,
                                        tskIDLE_PRIORITY + 1,
                                        g_heartbeat_stack,
                                        &g_heartbeat_tcb);
  if (!task) {
    g_sentai_emu_boot_state = kBootCreateTaskFailed;
    while (true) {
    }
  }

  g_sentai_emu_boot_state = kBootTaskCreated;
  vTaskStartScheduler();

  g_sentai_emu_boot_state = kBootSchedulerReturned;
  while (true) {
  }
}
