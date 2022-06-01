#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include <cstdio>

extern "C" void app_main(void *param) {
    (void)param;
    printf("CdcAcm\r\n");
    vTaskSuspend(nullptr);
}