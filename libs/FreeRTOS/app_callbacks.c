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

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  static StaticTask_t idle_task;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
  *ppxIdleTaskTCBBuffer = &idle_task;
  *ppxIdleTaskStackBuffer = idle_task_stack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  static StaticTask_t timer_task;
  static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];
  *ppxTimerTaskTCBBuffer = &timer_task;
  *ppxTimerTaskStackBuffer = timer_task_stack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Weak fallback — overridden by the strong definition in sentai_fault.cc
// which saves a crash breadcrumb to SRC GPR registers before resetting.
__attribute__((weak)) void vApplicationMallocFailedHook(void) {
  DbgConsole_Printf("malloc failed, spin...\r\n");
  while (1) {
  }
}

// Weak no-op — configASSERT calls sentai_assert_fail() before resetting.
// sentai_fault.cc provides the strong version that saves the GPR breadcrumb.
// This fallback lets non-sentai firmware builds link without sentai_fault.cc.
__attribute__((weak)) void sentai_assert_fail(unsigned int lr) {
  (void)lr;
}
