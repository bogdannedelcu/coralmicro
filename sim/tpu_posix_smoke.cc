// Minimal host-side EdgeTPU transport smoke.
//
// This intentionally stops below model load/invoke: it verifies that the
// existing SentAI TpuDriver can initialize a Coral USB stick through the
// POSIX/libusb USB_HostEdgeTpu backend.

#include <cstdio>

#include "libs/tpu/edgetpu_driver.h"
#include "libs/tpu/usb_host_edgetpu.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" {
volatile uint8_t g_sentai_tpu_trace = 0;
}

static void tpu_smoke_task(void *) {
  usb_host_edgetpu_instance_t *inst = nullptr;
  usb_status_t st = USB_HostEdgeTpuOpenPosix(&inst);
  if (st != kStatus_USB_Success || !inst) {
    std::printf("FAIL open_posix status=%u\n", (unsigned)st);
    std::fflush(stdout);
    _Exit(2);
  }

  coralmicro::TpuDriver driver;
  if (!driver.Initialize(inst, coralmicro::PerformanceMode::kHigh)) {
    std::printf("FAIL driver_initialize\n");
    USB_HostEdgeTpuClosePosix(inst);
    std::fflush(stdout);
    _Exit(3);
  }

  float temp = driver.GetTemperature();
  std::printf("OK tpu_posix_smoke temp_c=%.2f\n", temp);
  USB_HostEdgeTpuClosePosix(inst);
  std::fflush(stdout);
  _Exit(0);
}

int main() {
  if (xTaskCreate(tpu_smoke_task, "tpu_smoke", 8192, nullptr,
                  tskIDLE_PRIORITY + 1, nullptr) != pdPASS) {
    std::printf("FAIL xTaskCreate\n");
    return 1;
  }
  vTaskStartScheduler();
  std::printf("FAIL scheduler_returned\n");
  return 1;
}
