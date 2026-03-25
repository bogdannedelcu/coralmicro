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

#include "libs/base/filesystem.h"
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

//constexpr char kModelPath[] =
//    "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite";

constexpr char kModelPath[] =
    "/models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite";

constexpr char kImagePath[] =
    "/examples/detect_objects_file/test_512x512.rgb";

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

// Global pointer so MicroPython bridge can call Invoke()
static tflite::MicroInterpreter* g_interpreter = nullptr;
static volatile bool g_tpu_ready = false;

void Main() {
  logf("Detect Image Example!\r\n");

  std::vector<uint8_t> model;
  if (!LfsReadFile(kModelPath, &model)) {
    logf("ERROR: Failed to load %s\r\n", kModelPath);
    return;
  }

  logf("Model loaded of size: %lu bytes\r\n", (unsigned long)model.size());

  coralmicro::PerformanceMode tpu_mode = coralmicro::PerformanceMode::kMax;
  auto tpu_context =
      EdgeTpuManager::GetSingleton()->OpenDevice(tpu_mode);
  if (!tpu_context) {
    logf("ERROR: Failed to get EdgeTpu context\r\n");
    return;
  }

  logf("Edge TPU device opened successfully in mode %d\r\n", static_cast<int>(tpu_mode));
  
  tflite::MicroErrorReporter error_reporter;
  /*
  tflite::MicroMutableOpResolver<3> resolver;
  resolver.AddDequantize();
  resolver.AddDetectionPostprocess();
  resolver.AddCustom(kCustomOp, RegisterCustomOp());

  tflite::MicroInterpreter interpreter(
      tflite::GetModel(model.data()), resolver, tensor_arena,
      kTensorArenaSize, &error_reporter);
*/
  tflite::MicroMutableOpResolver<11> resolver;
  // resolver.AddDequantize();
  // resolver.AddTranspose();
  // resolver.AddReshape();
  // resolver.AddConcatenation();
  // resolver.AddSoftmax();
  // resolver.AddConv2D();
  // resolver.AddMul();
  // resolver.AddStridedSlice();
  // resolver.AddAdd();
  // resolver.AddSub();
  resolver.AddCustom(kCustomOp, RegisterCustomOp());
  tflite::MicroInterpreter interpreter(
      tflite::GetModel(model.data()), resolver, tensor_arena,
      kTensorArenaSize, &error_reporter);

  if (interpreter.AllocateTensors() != kTfLiteOk) {
    logf("ERROR: AllocateTensors() failed\r\n");
    return;
  }

  if (interpreter.inputs().size() != 1) {
    logf("ERROR: Model must have only one input tensor\r\n");
    return;
  }

  logf("Tensor arena allocated successfully\r\n");

  logf("Tensor arena used: %lu bytes (%lu KB) out of %d bytes (%d KB)\r\n",
         (unsigned long)interpreter.arena_used_bytes(),
         (unsigned long)(interpreter.arena_used_bytes() / 1024),
         kTensorArenaSize,
         kTensorArenaSize / 1024);

  auto* input_tensor = interpreter.input_tensor(0);
  logf("Input tensor: %ld bytes, dims=[%ld,%ld,%ld,%ld], type=%d\r\n",
         (long)input_tensor->bytes,
         (long)input_tensor->dims->data[0],
         (long)input_tensor->dims->data[1],
         (long)input_tensor->dims->data[2],
         (long)input_tensor->dims->data[3],
         (int)input_tensor->type);
  logf("Outputs: %lu\r\n", (unsigned long)interpreter.outputs().size());
  logf("Output tensor 0: %ld bytes, dims=[%ld,%ld], type=%d\r\n",
         (long)interpreter.output_tensor(0)->bytes,
         (long)interpreter.output_tensor(0)->dims->data[0],
         (long)interpreter.output_tensor(0)->dims->data[1],
         (int)interpreter.output_tensor(0)->type);

  // Load image into input tensor
  std::vector<uint8_t> image_buffer(input_tensor->bytes);
  if (!LfsReadFile(kImagePath, image_buffer.data(), image_buffer.size())) {
    logf("ERROR: Failed to load %s\r\n", kImagePath);
    return;
  }
  logf("Image loaded: %lu bytes\r\n", (unsigned long)image_buffer.size());

  std::memcpy(tflite::GetTensorData<uint8_t>(input_tensor),
              image_buffer.data(), image_buffer.size());

  // Warmup invoke
  logf("Warmup invoke...\r\n");
  if (interpreter.Invoke() != kTfLiteOk) {
    logf("ERROR: Warmup Invoke() failed\r\n");
    return;
  }
  logf("Warmup done.\r\n");

  // Expose interpreter for Python REPL bridge
  g_interpreter = &interpreter;
  g_tpu_ready = true;
  logf("TPU ready - use coral.invoke() from Python REPL\r\n");

  // Park this task forever - keeps interpreter alive on stack
  vTaskSuspend(NULL);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nStarting new\r\n");
  // vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for console to be ready

  // Launch MicroPython REPL task (interactive Python over serial)
  coralmicro::logf("Starting MicroPython REPL task...\r\n");
  micropython_start_repl_task(16384, tskIDLE_PRIORITY + 1);

  coralmicro::Main();
  // Main() parks itself with vTaskSuspend - never returns
}

// ===================== C bridge for MicroPython =====================
// Called from modcoral.c (C code) - need extern "C" linkage

extern "C" int coral_tpu_invoke(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  TickType_t t0 = xTaskGetTickCount();
  if (coralmicro::g_interpreter->Invoke() != kTfLiteOk) return -2;
  TickType_t t1 = xTaskGetTickCount();
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

extern "C" int coral_tpu_is_ready(void) {
  return coralmicro::g_tpu_ready ? 1 : 0;
}

extern "C" int coral_tpu_num_outputs(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  return (int)coralmicro::g_interpreter->outputs().size();
}

extern "C" int coral_tpu_get_output_size(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->bytes;
}

extern "C" const void* coral_tpu_get_output_data(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return NULL;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return NULL;
  return coralmicro::g_interpreter->output_tensor(idx)->data.data;
}

extern "C" int coral_tpu_get_output_num_dims(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return 0;
  return coralmicro::g_interpreter->output_tensor(idx)->dims->size;
}

extern "C" int coral_tpu_get_output_dim(int idx, int dim) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return 0;
  auto* t = coralmicro::g_interpreter->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int coral_tpu_get_output_type(int idx) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return -1;
  return (int)coralmicro::g_interpreter->output_tensor(idx)->type;
}

// ===================== Camera bridge for MicroPython =====================

static volatile bool g_cam_initialized = false;
static int g_cam_width = DEMO_CAMERA_WIDTH;
static int g_cam_height = DEMO_CAMERA_HEIGHT;

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

  // Clean+invalidate dst cache region before PXP DMA writes
#if (__CORTEX_M == 7)
  DCACHE_CleanInvalidateByRange((uint32_t)dst, dst_w * dst_h * 3);
#endif

  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &ps_cfg);
  PXP_SetProcessSurfaceScaler(DEMO_PXP, src_w, src_h, dst_w, dst_h);
  PXP_SetProcessSurfacePosition(DEMO_PXP, 0, 0, dst_w - 1, dst_h - 1);
  PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  PXP_EnableCsc1(DEMO_PXP, false);
  PXP_SetOutputBufferConfig(DEMO_PXP, &out_cfg);

  PXP_Start(DEMO_PXP);
  while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP)));
  PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);

  // Invalidate cache so CPU sees PXP DMA output
#if (__CORTEX_M == 7)
  DCACHE_InvalidateByRange((uint32_t)dst, dst_w * dst_h * 3);
#endif
  return 0;
}

extern "C" int coral_cam_set_res(int w, int h) {
  if (w <= 0 || h <= 0 || w > DEMO_CAMERA_WIDTH || h > DEMO_CAMERA_HEIGHT) return -1;
  g_cam_width = w;
  g_cam_height = h;
  return 0;
}

extern "C" int coral_cam_init(int streaming) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (!cam->SetPower(true)) return -1;
  auto mode = streaming ? coralmicro::CameraMode::kStreaming
                        : coralmicro::CameraMode::kTrigger;
  if (!cam->Enable(mode)) return -2;
  g_cam_initialized = true;
  return 0;
}

extern "C" int coral_cam_stop(void) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  cam->Disable();
  cam->SetPower(false);
  g_cam_initialized = false;
  return 0;
}

// Capture RGB frame via PXP hardware scaler. Returns 0 on success.
extern "C" int coral_cam_capture_rgb(uint8_t* buf, int width, int height) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  uint8_t* raw = nullptr;
  int idx = cam->GetRawFrame(&raw);
  if (idx < 0 || !raw) return -2;

  int rc;
  if (width == DEMO_CAMERA_WIDTH && height == DEMO_CAMERA_HEIGHT) {
    // No scaling needed - just color convert XRGB->RGB (PXP 1:1)
    rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                buf, width, height);
  } else {
    // PXP hardware downscale + color convert
    rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                buf, width, height);
  }
  cam->ReturnRawFrame(idx);
  return rc;
}

// Capture + JPEG compress. Returns JPEG size or negative error.
extern "C" int coral_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                      int width, int height, int quality) {
  if (!g_cam_initialized) return -1;
  int rgb_size = width * height * 3;
  std::vector<uint8_t> rgb(rgb_size);
  int rc = coral_cam_capture_rgb(rgb.data(), width, height);
  if (rc != 0) return rc;
  unsigned long used = coralmicro::JpegCompressRgb(
      rgb.data(), width, height, quality,
      (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);
  return (int)used;
}

// Capture RGB via PXP and feed directly into TPU input tensor.
extern "C" int coral_cam_to_tensor(void) {
  if (!g_cam_initialized) return -1;
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -4;
  int h = input->dims->data[1];
  int w = input->dims->data[2];
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  auto* cam = coralmicro::CameraTask::GetSingleton();
  uint8_t* raw = nullptr;
  int idx = cam->GetRawFrame(&raw);
  if (idx < 0 || !raw) return -2;
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  tensor_buf, w, h);
  cam->ReturnRawFrame(idx);
  return rc;
}

// Switch between front and back cameras. id: 0=front, 1=back.
extern "C" int coral_cam_switch(int id) {
  if (!g_cam_initialized) return -1;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  if (id == 0) {
    cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraFront);
  } else if (id == 1) {
    cam->SwitchCamera(coralmicro::SwitchCameraId::kCameraBack);
  } else {
    return -2;
  }
  return 0;
}

extern "C" int coral_cam_get_width(void) {
  return g_cam_width;
}

extern "C" int coral_cam_get_height(void) {
  return g_cam_height;
}

extern "C" int coral_cam_get_native_width(void) {
  return coralmicro::CameraTask::kWidth;
}

extern "C" int coral_cam_get_native_height(void) {
  return coralmicro::CameraTask::kHeight;
}
// [end-sphinx-snippet:detect-image]