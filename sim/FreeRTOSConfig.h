/*
 * FreeRTOSConfig.h for the SentAI x86/Linux SIM build.
 *
 * Counterpart to third_party/modified/FreeRTOS/FreeRTOSConfig.h (ARM build).
 * Stripped of all Cortex-M-specific definitions (NVIC, SVC/PendSV/SysTick
 * handler aliases, configPRIO_BITS, configCPU_CLOCK_HZ tied to
 * SystemCoreClock, tickless idle, IPC dual-core hooks).
 *
 * The POSIX port (third_party/freertos_kernel/portable/ThirdParty/GCC/Posix)
 * provides its own scheduler implementation using pthread + signals.  This
 * config selects the kernel features SentAI uses; values intentionally match
 * the ARM config where they're meaningful, so behavior stays comparable.
 *
 * Update both configs in lockstep when adding new FreeRTOS features.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/* Scheduler */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1   /* POSIX port enjoys time slicing */
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TICKLESS_IDLE                 0   /* No CPU-sleep on Linux */

/* Tick rate.  POSIX port translates this to a SIGALRM / itimer interval.
 * 1000 Hz matches ARM build for behavior parity. */
#define configTICK_RATE_HZ                      ((TickType_t) 1000)
#define configCPU_CLOCK_HZ                      (1000000000UL)  /* dummy, irrelevant on Linux */

/* Task model */
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                ((unsigned short) 1024) /* Linux pthread stacks need more headroom than ARM */
#define configMAX_TASK_NAME_LEN                 20
#define configUSE_16_BIT_TICKS                  0
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   2

/* Synchronization primitives */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               8

/* Memory */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configFRTOS_MEMORY_SCHEME               4   /* heap_4.c */
#define configTOTAL_HEAP_SIZE                   ((size_t) (4 * 1024 * 1024)) /* 4 MB heap, generous on Linux */

/* Hooks */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          0   /* POSIX port doesn't enforce — pthread stacks are huge */
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/* Newlib reentrancy */
#define configUSE_NEWLIB_REENTRANT              0   /* not needed on glibc */
/* The vendored POSIX port (V10.4.1, Cambridge Consultants) uses pre-V8
 * legacy type names (portTickType, pdTASK_CODE) that only resolve when
 * backward compatibility is enabled.  Keep this ON for SIM until/unless
 * we patch the port. */
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* Stats / debug */
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configRECORD_STACK_HIGH_ADDRESS         1
#define configGENERATE_RUN_TIME_STATS           0   /* skip in SIM for simplicity */

/* Co-routines (deprecated, off) */
#define configUSE_CO_ROUTINES                   0

/* Asserts.  Plain printf + abort on Linux — much friendlier than SCB->AIRCR
 * SYSRESETREQ.  Lets us run under valgrind / gdb / asan and get a normal
 * stack trace at the failure site. */
#define configASSERT(x)                                                     \
    do {                                                                    \
        if (!(x)) {                                                         \
            fprintf(stderr, "[SIM configASSERT] %s:%d  %s was not true\n", \
                    __FILE__, __LINE__, #x);                                \
            abort();                                                        \
        }                                                                   \
    } while (0)

/* Optional API knobs (must match ARM build for source compatibility) */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1

#endif /* FREERTOS_CONFIG_H */
