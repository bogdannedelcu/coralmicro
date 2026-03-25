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
constexpr int kNumRuns = 20;

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

struct BenchmarkRow {
  uint32_t memcpy_ms;
  uint32_t invoke_ms;
  uint32_t detection_ms;
  uint32_t total_ms;
  uint32_t num_detections;
};

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

  std::vector<uint8_t> image_buffer(input_tensor->bytes);
  if (!LfsReadFile(kImagePath, image_buffer.data(), image_buffer.size())) {
    logf("ERROR: Failed to load %s\r\n", kImagePath);
    return;
  }

  logf("Image loaded of size: %lu bytes\r\n", (unsigned long)image_buffer.size());

  BenchmarkRow rows[kNumRuns] = {};

  uint32_t sum_memcpy_ms = 0;
  uint32_t sum_invoke_ms = 0;
  uint32_t sum_detection_ms = 0;
  uint32_t sum_total_ms = 0;

  std::memcpy(tflite::GetTensorData<uint8_t>(input_tensor),
                image_buffer.data(), image_buffer.size());


  for (int i = 0; i < kNumRuns; ++i) {
    TickType_t t0 = xTaskGetTickCount();


    TickType_t t1 = xTaskGetTickCount();

    if (interpreter.Invoke() != kTfLiteOk) {
      logf("ERROR: Invoke() failed at run %d\r\n", i + 1);
      return;
    }

    TickType_t t2 = xTaskGetTickCount();

    //auto results = tensorflow::GetDetectionResults(&interpreter, 0.6, 3);

    TickType_t t3 = xTaskGetTickCount();

    rows[i].memcpy_ms = (t1 - t0) * portTICK_PERIOD_MS;
    rows[i].invoke_ms = (t2 - t1) * portTICK_PERIOD_MS;
    rows[i].detection_ms = (t3 - t2) * portTICK_PERIOD_MS;
    rows[i].total_ms = (t3 - t0) * portTICK_PERIOD_MS;
    //rows[i].num_detections = static_cast<uint32_t>(results.size());

    if(i>0) {
      sum_memcpy_ms += rows[i].memcpy_ms;
      sum_invoke_ms += rows[i].invoke_ms;
      sum_detection_ms += rows[i].detection_ms;
      sum_total_ms += rows[i].total_ms;
    }

    // Warmup delay după prima rulare
    if (i == 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      //vTaskDelay(pdMS_TO_TICKS(50));
    }
  }

 char report[4096];
size_t off = 0;

off += snprintf(report + off, sizeof(report) - off,
                "\r\n==== BENCHMARK RESULTS ====\r\n");
off += snprintf(report + off, sizeof(report) - off,
                "run,memcpy_ms,inference_ms,detection_ms,total_ms,num_detections\r\n");

for (int i = 0; i < kNumRuns; ++i) {
  off += snprintf(report + off, sizeof(report) - off,
                  "%d,%lu,%lu,%lu,%lu,%lu\r\n",
                  i + 1,
                  static_cast<unsigned long>(rows[i].memcpy_ms),
                  static_cast<unsigned long>(rows[i].invoke_ms),
                  static_cast<unsigned long>(rows[i].detection_ms),
                  static_cast<unsigned long>(rows[i].total_ms),
                  static_cast<unsigned long>(rows[i].num_detections));
}

off += snprintf(report + off, sizeof(report) - off,
                "\r\n==== AVERAGES ====\r\n");
off += snprintf(report + off, sizeof(report) - off,
                "avg_memcpy_ms=%lu\r\n",
                static_cast<unsigned long>(sum_memcpy_ms / (kNumRuns - 1)));
off += snprintf(report + off, sizeof(report) - off,
                "avg_inference_ms=%lu\r\n",
                static_cast<unsigned long>(sum_invoke_ms / (kNumRuns - 1)));
off += snprintf(report + off, sizeof(report) - off,
                "avg_detection_ms=%lu\r\n",
                static_cast<unsigned long>(sum_detection_ms / (kNumRuns - 1)));
off += snprintf(report + off, sizeof(report) - off,
                "avg_total_ms=%lu\r\n",
                static_cast<unsigned long>(sum_total_ms / (kNumRuns - 1)));

logf("%s", report);
fflush(stdout);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nStarting new\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for console to be ready
  coralmicro::Main();

  // --- Deep sleep: EdgeTPU power off + CM7 tickless idle for 5s ---
  coralmicro::logf("Powering off EdgeTPU...\r\n");
  // Release tpu_context (goes out of scope in Main), but force power off here:
  coralmicro::EdgeTpuTask::GetSingleton()->SetPower(false);
  coralmicro::logf("EdgeTPU powered off. Entering CM7 low-power sleep for 5s...\r\n");
  fflush(stdout);

  // FreeRTOS tickless idle: vTaskDelay with configUSE_TICKLESS_IDLE=2
  // will call vPortSuppressTicksAndSleep -> __WFI, putting CM7 to sleep.
  // GPT2 wakeup interrupt fires after 5 seconds.
  vTaskDelay(pdMS_TO_TICKS(5000));

  coralmicro::logf("Woke up from sleep!\r\n");
  fflush(stdout);

  vTaskSuspend(nullptr);
}
// [end-sphinx-snippet:detect-image]