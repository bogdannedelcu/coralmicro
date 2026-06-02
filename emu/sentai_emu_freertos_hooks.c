// Minimal FreeRTOS hooks for the B8.1 ARM emulator smoke target.
//
// No console or board peripherals are used here; failures are exposed through
// g_sentai_emu_boot_state for Renode to inspect.

#include <stdint.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern volatile uint32_t g_sentai_emu_boot_state;
extern volatile uint32_t g_sentai_emu_heartbeat;
extern uint32_t SystemCoreClock;

enum {
  kBootMallocFailed = 0xF001,
  kBootStackOverflow = 0xF002,
  kBootAssertFailed = 0xF003,
};

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
  static StaticTask_t idle_task;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE]
      __attribute__((aligned(8)));
  *ppxIdleTaskTCBBuffer = &idle_task;
  *ppxIdleTaskStackBuffer = idle_task_stack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) {
  static StaticTask_t timer_task;
  static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH]
      __attribute__((aligned(8)));
  *ppxTimerTaskTCBBuffer = &timer_task;
  *ppxTimerTaskStackBuffer = timer_task_stack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationMallocFailedHook(void) {
  g_sentai_emu_boot_state = kBootMallocFailed;
  while (1) {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  g_sentai_emu_boot_state = kBootStackOverflow;
  while (1) {
  }
}

void sentai_assert_fail(unsigned int lr) {
  (void)lr;
  g_sentai_emu_boot_state = kBootAssertFailed;
}

uint32_t vPortGetRunTimeCounterValue(void) {
  return g_sentai_emu_heartbeat;
}

void vPortSetupTimerInterrupt(void) {
  enum {
    kSysTickCtrlEnable = 1u << 0,
    kSysTickCtrlTickInt = 1u << 1,
    kSysTickCtrlClkSource = 1u << 2,
  };
  volatile uint32_t* const systick_ctrl = (volatile uint32_t*)0xE000E010u;
  volatile uint32_t* const systick_load = (volatile uint32_t*)0xE000E014u;
  volatile uint32_t* const systick_current = (volatile uint32_t*)0xE000E018u;
  *systick_ctrl = 0;
  *systick_current = 0;
  *systick_load = (SystemCoreClock / configTICK_RATE_HZ) - 1u;
  *systick_ctrl = kSysTickCtrlEnable | kSysTickCtrlTickInt |
                  kSysTickCtrlClkSource;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime) {
  (void)xExpectedIdleTime;
}

void vGeneratePrimaryToSecondaryInterrupt(void* pxStreamBuffer) {
  (void)pxStreamBuffer;
}
