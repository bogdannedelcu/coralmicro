/*
 * sim/test_freertos.c — Phase 1 step 4 smoke test.
 *
 * Validates the FreeRTOS POSIX/Linux port:
 *   - Scheduler boots
 *   - Two tasks at different priorities printf in interleaved fashion
 *   - vTaskDelay actually delays (not busy-spin)
 *   - Auto-exit after 5 seconds so CI doesn't hang
 *
 * Pass criteria:
 *   - Stdout shows alternating "[A] tick=N" and "[B] tick=N" lines
 *   - Tick counter advances at ~1 kHz (matches configTICK_RATE_HZ)
 *   - Process exits with status 0 after ~5 s
 *
 * Run:
 *   cmake -B build-sim -S . -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake -DSENTAI_SIM=ON
 *   cmake --build build-sim --target sentai_sim_kernel_only
 *   ./build-sim/sim/sentai_sim_kernel_only
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#define TASK_A_PRIORITY  (tskIDLE_PRIORITY + 2)
#define TASK_B_PRIORITY  (tskIDLE_PRIORITY + 1)
#define STACK_WORDS      (configMINIMAL_STACK_SIZE * 4)
#define RUN_DURATION_MS  5000u

static volatile uint32_t g_should_exit = 0u;

static void task_a(void* arg) {
    (void) arg;
    for (uint32_t i = 0u; !g_should_exit; ++i) {
        const uint32_t tick = (uint32_t) xTaskGetTickCount();
        printf("[A] iter=%u tick=%u\n", (unsigned) i, (unsigned) tick);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void task_b(void* arg) {
    (void) arg;
    for (uint32_t i = 0u; !g_should_exit; ++i) {
        const uint32_t tick = (uint32_t) xTaskGetTickCount();
        printf("[B] iter=%u tick=%u\n", (unsigned) i, (unsigned) tick);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
    vTaskDelete(NULL);
}

static void watchdog_task(void* arg) {
    (void) arg;
    /* Run-duration cap so CI never hangs.  After RUN_DURATION_MS, set the
     * exit flag and give worker tasks one tick to notice, then end the
     * scheduler by exiting the process cleanly. */
    vTaskDelay(pdMS_TO_TICKS(RUN_DURATION_MS));
    g_should_exit = 1u;
    vTaskDelay(pdMS_TO_TICKS(50));
    printf("[watchdog] %u ms elapsed, exiting cleanly\n",
           (unsigned) RUN_DURATION_MS);
    fflush(stdout);
    /* On the POSIX port the cleanest way out is to call exit() directly —
     * vTaskEndScheduler is not portable to all hosts. */
    exit(0);
}

int main(void) {
    printf("[sim] sentai_sim_kernel_only — FreeRTOS POSIX port smoke test\n");
    printf("[sim] tick rate = %u Hz, max prios = %u, heap = %u bytes\n",
           (unsigned) configTICK_RATE_HZ,
           (unsigned) configMAX_PRIORITIES,
           (unsigned) configTOTAL_HEAP_SIZE);
    fflush(stdout);

    BaseType_t ok;
    ok = xTaskCreate(task_a, "task_a", STACK_WORDS, NULL,
                     TASK_A_PRIORITY, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(task_b, "task_b", STACK_WORDS, NULL,
                     TASK_B_PRIORITY, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(watchdog_task, "watchdog", STACK_WORDS, NULL,
                     tskIDLE_PRIORITY + 3, NULL);
    configASSERT(ok == pdPASS);

    printf("[sim] starting scheduler...\n");
    fflush(stdout);
    vTaskStartScheduler();

    /* Should not return.  If we get here, scheduler init failed. */
    printf("[sim] FATAL: vTaskStartScheduler returned\n");
    return 1;
}

/* Required hooks for FreeRTOS (referenced when configUSE_MALLOC_FAILED_HOOK=1
 * and similar).  The POSIX port's idle task and timer task are auto-created. */
void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "[sim] FATAL: malloc failed in FreeRTOS heap\n");
    abort();
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void) task;
    fprintf(stderr, "[sim] FATAL: stack overflow in task '%s'\n",
            name ? name : "?");
    abort();
}
#endif

/* configSUPPORT_STATIC_ALLOCATION=1 requires the application to provide
 * storage for the idle and timer tasks.  Static buffers in .bss are fine
 * here (no MCU memory regions to worry about). */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
    static StaticTask_t s_idle_tcb;
    static StackType_t  s_idle_stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &s_idle_tcb;
    *ppxIdleTaskStackBuffer = s_idle_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) {
    static StaticTask_t s_timer_tcb;
    static StackType_t  s_timer_stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &s_timer_tcb;
    *ppxTimerTaskStackBuffer = s_timer_stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
