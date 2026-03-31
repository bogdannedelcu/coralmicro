// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>
#include "build_version.h"

extern "C" {
#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"
}
#undef FAR  // jpeglib defines FAR as empty, conflicts with NXP SDK struct field

#include "libs/base/filesystem.h"
#include "libs/base/gpio.h"
#include "libs/base/led.h"
#include "libs/camera/camera.h"
#include "libs/libjpeg/jpeg.h"
#include "libs/camera/camera_support.h"
#include "fsl_pxp.h"
#if (__CORTEX_M == 7)
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7/fsl_cache.h"
#endif
#include "libs/tensorflow/detection.h"
#include "libs/tensorflow/utils.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
#include "libs/tpu/edgetpu_task.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"

extern "C" {
#include "micropython_task.h"
}

// Performs object detection with SSD MobileNet, running on the Edge TPU,
// using a local bitmap file as input.
//
// To build and flash from coralmicro root:
//    bash build.sh
//    python3 scripts/flashtool.py -e detect_image

// [start-sphinx-snippet:detect-image]
namespace coralmicro {
namespace {

constexpr int kTensorArenaSize = 8 * 1024 * 1024;

STATIC_TENSOR_ARENA_IN_SDRAM(tensor_arena, kTensorArenaSize);

static TickType_t app_start_tick = 0;

void logf(const char* fmt, ...) {
  TickType_t now = xTaskGetTickCount();
  uint32_t elapsed_ms = (now - app_start_tick) * portTICK_PERIOD_MS;
  uint32_t mins = elapsed_ms / 60000;
  uint32_t secs = (elapsed_ms % 60000) / 1000;
  uint32_t ms   = elapsed_ms % 1000;
  printf("[%02lu:%02lu:%03lu] ", (unsigned long)mins, (unsigned long)secs, (unsigned long)ms);
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
}

// Global state - interpreter, model data, TPU context
static tflite::MicroInterpreter* g_interpreter = nullptr;
static volatile bool g_tpu_ready = false;
static std::vector<uint8_t>* g_model_data = nullptr;
static std::shared_ptr<coralmicro::EdgeTpuContext> g_tpu_context;

void Main() {
  logf("SentAI MicroPython Runtime\r\n");

  // Open EdgeTPU once at boot
  coralmicro::PerformanceMode tpu_mode = coralmicro::PerformanceMode::kMax;
  g_tpu_context = EdgeTpuManager::GetSingleton()->OpenDevice(tpu_mode);
  if (!g_tpu_context) {
    logf("ERROR: Failed to get EdgeTpu context\r\n");
    return;
  }
  logf("Edge TPU device opened successfully in mode %d\r\n", static_cast<int>(tpu_mode));
  logf("Use sentai.tpu.load('/models/xxx.tflite') from Python REPL\r\n");

  // Park this task forever - model loading happens from Python
  vTaskSuspend(NULL);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nSentAI build #%d (%s)\r\n", BUILD_VERSION, BUILD_TIMESTAMP);
  // vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for console to be ready

  // User button task: waits for notification from ISR, then safely
  // calls sentai_usb_drive_set(0) from task context (not ISR).
  static TaskHandle_t s_btn_task = nullptr;
  xTaskCreate([](void*) {
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      extern int sentai_usb_drive_get(void);
      extern int sentai_usb_drive_set(int on);
      extern int sentai_console_set_target(int target);
      if (sentai_usb_drive_get()) {
        sentai_usb_drive_set(0);
        // Switch REPL back to USB (was moved to UART when drive was enabled)
        sentai_console_set_target(0);  // 0 = USB
        printf("\r\n*****\r\nBack from host\r\n*****\r\n>>> ");
      }
    }
  }, "btn_usb", configMINIMAL_STACK_SIZE * 4, nullptr,
     tskIDLE_PRIORITY + 1, &s_btn_task);

  // ISR only sends a notification — no flash/LFS work in interrupt context.
  coralmicro::GpioConfigureInterrupt(
      coralmicro::Gpio::kUserButton,
      coralmicro::GpioInterruptMode::kIntModeFalling,
      [&]() {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_btn_task, &woken);
        portYIELD_FROM_ISR(woken);
      },
      /*debounce_interval_us=*/200 * 1000);
  coralmicro::logf("User button -> usb.drive(0)\r\n");

  // Launch MicroPython REPL task (interactive Python over serial)
  coralmicro::logf("Starting MicroPython REPL task...\r\n");
  micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);

  coralmicro::Main();
  // Main() parks itself with vTaskSuspend - never returns
}

// ===================== C bridge for MicroPython =====================
// Called from modsentai.c (C code) - need extern "C" linkage

// Load a TFLite model from flash and create interpreter.
// Returns 0 on success, negative on error.
extern "C" int sentai_load_model(const char* path) {
  // If there's an existing interpreter, tear it down
  if (coralmicro::g_interpreter) {
    coralmicro::g_tpu_ready = false;
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
  }
  // Free previous model data
  if (coralmicro::g_model_data) {
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
  }
  if (!coralmicro::g_tpu_context) {
    printf("ERROR: EdgeTPU not initialized\r\n");
    return -1;
  }

  // Load model from user LFS
  coralmicro::g_model_data = new std::vector<uint8_t>();
  if (!coralmicro::LfsUserReadFile(path, coralmicro::g_model_data)) {
    printf("ERROR: Failed to load %s\r\n", path);
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
    return -2;
  }
  printf("Model loaded: %lu bytes\r\n",
         (unsigned long)coralmicro::g_model_data->size());

  // Create resolver with EdgeTPU custom op
  static tflite::MicroErrorReporter error_reporter;
  static tflite::MicroMutableOpResolver<1> resolver;
  static bool resolver_init = false;
  if (!resolver_init) {
    resolver.AddCustom(coralmicro::kCustomOp, coralmicro::RegisterCustomOp());
    resolver_init = true;
  }

  // Create interpreter (heap-allocated so it persists)
  coralmicro::g_interpreter = new tflite::MicroInterpreter(
      tflite::GetModel(coralmicro::g_model_data->data()), resolver,
      coralmicro::tensor_arena, coralmicro::kTensorArenaSize, &error_reporter);

  if (coralmicro::g_interpreter->AllocateTensors() != kTfLiteOk) {
    printf("ERROR: AllocateTensors() failed\r\n");
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
    return -3;
  }

  if (coralmicro::g_interpreter->inputs().size() != 1) {
    printf("ERROR: Model must have exactly one input tensor\r\n");
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
    return -4;
  }

  auto* input = coralmicro::g_interpreter->input_tensor(0);
  printf("Input: %ld bytes, dims=[%ld",
         (long)input->bytes, (long)input->dims->data[0]);
  for (int i = 1; i < input->dims->size; i++)
    printf(",%ld", (long)input->dims->data[i]);
  printf("], type=%d\r\n", (int)input->type);

  printf("Arena used: %lu / %d KB\r\n",
         (unsigned long)(coralmicro::g_interpreter->arena_used_bytes() / 1024),
         coralmicro::kTensorArenaSize / 1024);

  coralmicro::g_tpu_ready = true;
  return 0;
}

// Load an image file from flash into the input tensor.
// Supports raw RGB and JPEG (auto-detected by file header).
// If image is larger than tensor, it is cropped from top-left.
// Returns 0 on success, negative on error.
extern "C" int sentai_load_image(const char* path) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;

  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -2;

  int tensor_h = input->dims->data[1];
  int tensor_w = input->dims->data[2];
  int tensor_c = input->dims->data[3];
  int tensor_bytes = tensor_h * tensor_w * tensor_c;
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  // Read file from user LFS
  std::vector<uint8_t> file_data;
  if (!coralmicro::LfsUserReadFile(path, &file_data)) {
    printf("ERROR: Failed to read %s\r\n", path);
    return -3;
  }

  // Detect JPEG by magic bytes (FF D8 FF)
  bool is_jpeg = (file_data.size() >= 3 &&
                  file_data[0] == 0xFF &&
                  file_data[1] == 0xD8 &&
                  file_data[2] == 0xFF);

  if (is_jpeg) {
    // JPEG decompress to RGB using libjpeg
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, file_data.data(), file_data.size());

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
      jpeg_destroy_decompress(&cinfo);
      printf("ERROR: Invalid JPEG header\r\n");
      return -4;
    }

    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    int img_w = cinfo.output_width;
    int img_h = cinfo.output_height;
    int img_c = cinfo.output_components;  // 3 for RGB
    int row_stride = img_w * img_c;

    printf("JPEG: %dx%d ch=%d -> tensor %dx%dx%d\r\n",
           img_w, img_h, img_c, tensor_w, tensor_h, tensor_c);

    // Read scanlines directly into tensor (crop if image > tensor)
    int copy_w = (img_w < tensor_w) ? img_w : tensor_w;
    int copy_c = (img_c < tensor_c) ? img_c : tensor_c;
    int copy_bytes = copy_w * copy_c;

    // Clear tensor first (in case image is smaller)
    memset(tensor_buf, 0, tensor_bytes);

    // Temp buffer for one scanline if we need to crop width
    std::vector<uint8_t> scanline_buf(row_stride);
    JSAMPROW row_ptr = scanline_buf.data();

    int row = 0;
    while (cinfo.output_scanline < cinfo.output_height) {
      jpeg_read_scanlines(&cinfo, &row_ptr, 1);
      if (row < tensor_h) {
        memcpy(tensor_buf + row * tensor_w * tensor_c, row_ptr, copy_bytes);
        row++;
      }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
  } else {
    // Raw RGB - copy directly into tensor, crop/pad as needed
    int raw_size = (int)file_data.size();
    printf("Raw image: %d bytes -> tensor %d bytes\r\n", raw_size, tensor_bytes);

    if (raw_size >= tensor_bytes) {
      memcpy(tensor_buf, file_data.data(), tensor_bytes);
    } else {
      memset(tensor_buf, 0, tensor_bytes);
      memcpy(tensor_buf, file_data.data(), raw_size);
    }
  }

  return 0;
}

extern "C" int sentai_tpu_invoke(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  TickType_t t0 = xTaskGetTickCount();
  if (coralmicro::g_interpreter->Invoke() != kTfLiteOk) return -2;
  TickType_t t1 = xTaskGetTickCount();
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

extern "C" int sentai_tpu_is_ready(void) {
  return coralmicro::g_tpu_ready ? 1 : 0;
}

extern "C" int sentai_tpu_num_outputs(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  return (int)coralmicro::g_interpreter->outputs().size();
}

extern "C" int sentai_tpu_get_output_size(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tpu_get_output_data(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return NULL;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return NULL;
  return coralmicro::g_interpreter->output_tensor(idx)->data.data;
}

extern "C" int sentai_tpu_get_output_num_dims(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return coralmicro::g_interpreter->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tpu_get_output_dim(int idx, int dim) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  auto* t = coralmicro::g_interpreter->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tpu_get_output_type(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return -1;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->type;
}

// Save all output tensors to a CSV file on the filesystem.
// Format: one section per tensor, header line with dims, then rows of values.
// Returns 0 on success, negative on error.
extern "C" int sentai_save_output(const char* path) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;

  int num_out = (int)coralmicro::g_interpreter->outputs().size();
  if (num_out == 0) return -2;

  std::string csv;
  char tmp[32];

  for (int oi = 0; oi < num_out; oi++) {
    auto* t = coralmicro::g_interpreter->output_tensor(oi);
    if (!t) continue;

    // Header: output_index,type,dim0,dim1,...
    snprintf(tmp, sizeof(tmp), "# output %d, type=%d, dims=", oi, (int)t->type);
    csv += tmp;
    for (int d = 0; d < t->dims->size; d++) {
      if (d > 0) csv += 'x';
      snprintf(tmp, sizeof(tmp), "%d", t->dims->data[d]);
      csv += tmp;
    }
    csv += '\n';

    // Compute total elements
    int total = 1;
    for (int d = 0; d < t->dims->size; d++)
      total *= t->dims->data[d];

    // Number of columns = last dimension (or total if 1D)
    int cols = (t->dims->size >= 2) ? t->dims->data[t->dims->size - 1] : total;
    int rows = total / cols;

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        int flat = r * cols + c;
        if (c > 0) csv += ',';

        switch (t->type) {
          case kTfLiteFloat32: {
            float v = ((const float*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%.6f", v);
            csv += tmp;
            break;
          }
          case kTfLiteInt8: {
            int8_t v = ((const int8_t*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%d", (int)v);
            csv += tmp;
            break;
          }
          case kTfLiteUInt8: {
            uint8_t v = ((const uint8_t*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%u", (unsigned)v);
            csv += tmp;
            break;
          }
          case kTfLiteInt32: {
            int32_t v = ((const int32_t*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%ld", (long)v);
            csv += tmp;
            break;
          }
          case kTfLiteInt16: {
            int16_t v = ((const int16_t*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%d", (int)v);
            csv += tmp;
            break;
          }
          default: {
            uint8_t v = ((const uint8_t*)t->data.data)[flat];
            snprintf(tmp, sizeof(tmp), "%u", (unsigned)v);
            csv += tmp;
            break;
          }
        }
      }
      csv += '\n';
    }
  }

  if (!coralmicro::LfsUserWriteFile(path, csv)) {
    printf("ERROR: Failed to write %s\r\n", path);
    return -3;
  }
  printf("Saved %d outputs to %s (%lu bytes)\r\n", num_out, path,
         (unsigned long)csv.size());
  return 0;
}

// ===================== Camera bridge for MicroPython =====================

static volatile bool g_cam_initialized = false;
static int g_cam_width = DEMO_CAMERA_WIDTH;
static int g_cam_height = DEMO_CAMERA_HEIGHT;
static int g_cam_current_id = 0;  // 0=front, 1=back

// PXP hardware scale+convert: XRGB8888 (native cam) -> RGB888 (scaled output).
// src must be in non-cacheable memory (camera framebuffer).
// dst must be 64-byte aligned for best results.
// Returns 0 on success.
static int pxp_scale_xrgb_to_rgb(const uint8_t* src, int src_w, int src_h,
                                  uint8_t* dst, int dst_w, int dst_h) {
  pxp_ps_buffer_config_t ps_cfg;
  memset(&ps_cfg, 0, sizeof(ps_cfg));
  // kPXP_PsPixelFormatRGB888 = 0x4 = "32-bit pixels without alpha" = XRGB8888
  ps_cfg.pixelFormat = kPXP_PsPixelFormatRGB888;
  ps_cfg.swapByte    = false;
  ps_cfg.bufferAddr  = (uint32_t)src;
  ps_cfg.bufferAddrU = 0;
  ps_cfg.bufferAddrV = 0;
  ps_cfg.pitchBytes  = (src_w + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP;

  pxp_output_buffer_config_t out_cfg;
  memset(&out_cfg, 0, sizeof(out_cfg));
  out_cfg.pixelFormat    = kPXP_OutputPixelFormatRGB888P;
  out_cfg.interlacedMode = kPXP_OutputProgressive;
  out_cfg.buffer0Addr    = (uint32_t)dst;
  out_cfg.buffer1Addr    = 0;
  out_cfg.pitchBytes     = dst_w * 3;
  out_cfg.width          = dst_w;
  out_cfg.height         = dst_h;

  const uint32_t dst_size = dst_w * dst_h * 3;

  // Clean dst cache so PXP DMA doesn't collide with dirty cache lines
#if (__CORTEX_M == 7)
  DCACHE_CleanByRange((uint32_t)dst, dst_size);
#endif

  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, src_w, src_h, dst_w, dst_h);
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, dst_w - 1, dst_h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);

  PXP_Start(DEMO_PXP);

  // Yield CPU while PXP works instead of busy-waiting
  while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP))) {
    taskYIELD();
  }
  PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);

  // Invalidate cache so CPU sees PXP DMA output
#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)dst, dst_size);
#endif
  return 0;
}

extern "C" int sentai_cam_set_res(int w, int h) {
  if (w <= 0 || h <= 0 || w > DEMO_CAMERA_WIDTH || h > DEMO_CAMERA_HEIGHT) return -1;
  g_cam_width = w;
  g_cam_height = h;
  return 0;
}

extern "C" int sentai_cam_init(int streaming) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (!cam->SetPower(true)) return -1;
  auto mode = streaming ? coralmicro::CameraMode::kStreaming
                        : coralmicro::CameraMode::kTrigger;
  if (!cam->Enable(mode)) return -2;
  g_cam_initialized = true;

  // Cycle through both cameras to ensure CSI/MIPI is fully initialized.
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
  g_cam_current_id = 1;
  cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
  g_cam_current_id = 0;

  return 0;
}

extern "C" int sentai_cam_stop(void) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->Disable();
  cam->SetPower(false);
  g_cam_initialized = false;
  return 0;
}

// Try to get a raw frame with timeout and camera-toggle recovery.
// Returns framebuffer index (>=0) on success, writes raw pointer to *raw_out.
// On failure returns -2.
static int sentai_cam_get_raw_with_recovery(uint8_t** raw_out) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  const int kTimeoutMs = 1000;    // 1 second timeout per attempt
  const int kPollMs    = 50;      // poll interval
  const int kMaxRecoveries = 2;   // how many toggle-recovery attempts

  for (int recovery = 0; recovery <= kMaxRecoveries; ++recovery) {
    uint32_t t0 = xTaskGetTickCount();
    int idx = -1;
    while ((int)(xTaskGetTickCount() - t0) < kTimeoutMs) {
      *raw_out = nullptr;
      idx = cam->TryGetRawFrame(raw_out);
      if (idx >= 0 && *raw_out) {
        return idx;
      }
      vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }

    // Timed out — try toggling camera to kick CSI/MIPI
    if (recovery < kMaxRecoveries) {
      int other = (g_cam_current_id == 0) ? 1 : 0;
      printf("[CAM] GetRawFrame timeout, toggling %d->%d->%d to recover...\r\n",
             g_cam_current_id, other, g_cam_current_id);
      cam->SwitchCamera(other == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                   : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
      cam->SwitchCamera(g_cam_current_id == 0 ? coralmicro::SwitchCameraId::kCameraFront
                                              : coralmicro::SwitchCameraId::kCameraBack);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  printf("[CAM] GetRawFrame failed after all recovery attempts\r\n");
  *raw_out = nullptr;
  return -2;
}

// Capture RGB frame via PXP hardware scaler. Returns 0 on success.
extern "C" int sentai_cam_capture_rgb(uint8_t* buf, int width, int height) {
  if (!g_cam_initialized) return -1;
  uint8_t* raw = nullptr;
  uint32_t t0 = xTaskGetTickCount();
  printf("[DBG] @%lu capture_rgb: calling GetRawFrame...\r\n", (unsigned long)t0);
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  uint32_t t1 = xTaskGetTickCount();
  printf("[DBG] @%lu capture_rgb: GetRawFrame returned idx=%d raw=%p (+%lums)\r\n", (unsigned long)t1, idx, raw, (unsigned long)(t1-t0));
  if (idx < 0 || !raw) return -2;

  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  buf, width, height);
  uint32_t t2 = xTaskGetTickCount();
  printf("[DBG] @%lu capture_rgb: PXP done rc=%d (+%lums), ReturnRawFrame(%d)\r\n", (unsigned long)t2, rc, (unsigned long)(t2-t1), idx);
  cam->ReturnRawFrame(idx);
  printf("[DBG] @%lu capture_rgb: done (total %lums)\r\n", (unsigned long)xTaskGetTickCount(), (unsigned long)(xTaskGetTickCount()-t0));
  return rc;
}

// Capture + JPEG compress. Returns JPEG size or negative error.
extern "C" int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                      int width, int height, int quality) {
  if (!g_cam_initialized) return -1;
  int rgb_size = width * height * 3;
  std::vector<uint8_t> rgb(rgb_size);
  int rc = sentai_cam_capture_rgb(rgb.data(), width, height);
  if (rc != 0) return rc;
  unsigned long used = coralmicro::JpegCompressRgb(
      rgb.data(), width, height, quality,
      (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);
  return (int)used;
}

// Capture RGB via PXP and feed directly into TPU input tensor.
extern "C" int sentai_cam_to_tensor(void) {
  if (!g_cam_initialized) return -1;
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -4;
  int h = input->dims->data[1];
  int w = input->dims->data[2];
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  tensor_buf, w, h);
  cam->ReturnRawFrame(idx);
  return rc;
}

// Switch between front and back cameras. id: 0=front, 1=back.
extern "C" int sentai_cam_switch(int id) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  uint32_t ts0 = xTaskGetTickCount();
  printf("[DBG] @%lu cam_switch: id=%d, calling SwitchCamera...\r\n", (unsigned long)ts0, id);
  if (id == 0) {
    cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
    g_cam_current_id = 0;
  } else if (id == 1) {
    cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
    g_cam_current_id = 1;
  } else {
    return -2;
  }
  uint32_t ts1 = xTaskGetTickCount();
  printf("[DBG] @%lu cam_switch: done (+%lums)\r\n", (unsigned long)ts1, (unsigned long)(ts1-ts0));
  return 0;
}

// Rotate camera image. cam_id: 0=front, 1=back. degrees: 0, 90, 180, 270.
extern "C" int sentai_cam_rotate(int cam_id, int degrees) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  bool ok = cam->SetCameraRotation(cam_id, degrees);
  return ok ? 0 : -2;
}

extern "C" int sentai_cam_get_width(void) {
  return g_cam_width;
}

extern "C" int sentai_cam_get_height(void) {
  return g_cam_height;
}

extern "C" int sentai_cam_get_native_width(void) {
  return coralmicro::CameraTask::kWidth;
}

extern "C" int sentai_cam_get_native_height(void) {
  return coralmicro::CameraTask::kHeight;
}
// [end-sphinx-snippet:detect-image]