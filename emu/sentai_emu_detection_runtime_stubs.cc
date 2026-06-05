// Emulator-only board/TPU stubs needed to link detection_task.cc when S233
// starts PrepTask without InferTask.  The real image pipeline code remains in
// examples/sentai_runtime/detection_task.cc; these calls are unreachable on
// the prep-only path and return safe "not ready" results if invoked.

#include <stdint.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"

extern "C" int sentai_tpu_is_ready(void) {
  return 0;
}

extern "C" int sentai_tpu_invoke_internal(void) {
  return -1;
}

extern "C" int sentai_tpu_invoke_with_input(uint8_t* input_buf) {
  (void)input_buf;
  return -1;
}

extern "C" int sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf) {
  (void)slot;
  (void)buf;
  return -1;
}

extern "C" void sentai_tpu_set_input_done_sema(SemaphoreHandle_t sema) {
  (void)sema;
}

extern "C" int sentai_tpu_detect(int conf_permil, int iou_permil,
                                  int max_dets, int16_t* out_buf,
                                  int* out_count) {
  (void)conf_permil;
  (void)iou_permil;
  (void)max_dets;
  (void)out_buf;
  if (out_count) *out_count = 0;
  return -1;
}

extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp) {
  (void)buf;
  (void)count;
  (void)zp;
}

extern "C" int sentai_imu_read_accel(float* x_mg, float* y_mg,
                                      float* z_mg, float* temp_c) {
  if (x_mg) *x_mg = 0.0f;
  if (y_mg) *y_mg = 0.0f;
  if (z_mg) *z_mg = 1000.0f;
  if (temp_c) *temp_c = 25.0f;
  return 0;
}

extern "C" int sentai_fs_cache_write(const uint8_t* data, int size) {
  (void)data;
  (void)size;
  return -3;
}
