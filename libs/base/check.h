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

#ifndef LIBS_BASE_CHECK_H_
#define LIBS_BASE_CHECK_H_

#include <cstdint>
#include <cstdio>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#ifdef SENTAI_PLATFORM_SIM
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#define CHECK(a)                                                   \
  do {                                                             \
    if (!(a)) {                                                    \
      std::printf("%s:%d %s was not true.\n", __FILE__, __LINE__, #a); \
      std::abort();                                                \
    }                                                              \
  } while (0)
#else
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/fsl_device_registers.h"

#if (__CORTEX_M == 7)
#include "libs/base/console_m7.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
/* W24-T6: a post-scheduler CHECK failure now leaves the same breadcrumb as
 * configASSERT (SERR_SYS_ASSERT, r0 = address of the #a text, BFAR field =
 * address of the __FILE__ text) before the reset, so the next boot's crash
 * record names it instead of showing a bare software reset.  Both literals
 * already exist for the EmergencyWrite below, so the compiler merges them:
 * no extra rodata per CHECK site.  Weak fallbacks: libs/FreeRTOS/app_callbacks.c. */
extern "C" void sentai_assert_fail(uint32_t lr);   /* matches sentai_fault.h */
extern "C" volatile const char* g_sentai_assert_expr;
extern "C" volatile const char* g_sentai_assert_file;
/* One cold out-of-line helper (libs/base/console_m7.cc) instead of the
 * report + scheduler check + reset sequence inlined at every CHECK site:
 * keeps ITCM flat (the inlined form cost 5 KB over ~300 sites). */
extern "C" void sentai_check_fail(const char* file, int line, const char* expr);
/* 2026-10-30 hardening (embeded.md §10 anti-brick, audit C2/M11):
 *  - BEFORE the scheduler runs, a failed CHECK must NOT call
 *    vTaskSuspendAll(): it leaves uxSchedulerSuspended != 0, the first
 *    task to block then trips configASSERT -> reset loop before USB ever
 *    enumerates (a true brick).  We report and continue; every
 *    pre-scheduler precondition has a downstream guard.
 *  - AFTER the scheduler runs, a failed CHECK used to freeze every task
 *    silently until WDOG1 fired 30 s later.  Report, then reset at once
 *    (same path as configASSERT) so the breadcrumb is the last message. */
#define CHECK(a)                                                            \
  do {                                                                      \
    if (!(a)) sentai_check_fail(__FILE__, __LINE__, #a);                    \
  } while (0)
#elif (__CORTEX_M == 4)
#include "libs/base/console_m4.h"
#define CHECK(a)                                                        \
  do {                                                                  \
    if (!(a)) {                                                         \
      coralmicro::ConsoleM4EmergencyWrite("%s:%d %s was not true.\r\n", \
                                          __FILE__, __LINE__, #a);      \
      vTaskSuspendAll();                                                \
    }                                                                   \
  } while (0)
#else
#error "Unsupported platform"
#endif
#endif

#endif  // LIBS_BASE_CHECK_H_
