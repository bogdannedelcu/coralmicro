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

#include <cstring>
#include <vector>

#include "libs/base/filesystem.h"
#include "libs/base/led.h"
#include "libs/tensorflow/detection.h"
#include "libs/tensorflow/utils.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
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
    "/models/yolo_1_class_512_1_upsample_1_c3_512_inloc_de_1024_la_P5_32.tflite";

constexpr char kImagePath[] =
    "/examples/detect_objects_file/test_512x512.rgb";

constexpr int kTensorArenaSize = 8 * 1024 * 1024;
constexpr int kNumRuns = 20;

STATIC_TENSOR_ARENA_IN_SDRAM(tensor_arena, kTensorArenaSize);

struct BenchmarkRow {
  uint32_t memcpy_ms;
  uint32_t invoke_ms;
  uint32_t detection_ms;
  uint32_t total_ms;
  uint32_t num_detections;
};

void Main() {
  printf("Detect Image Example!\r\n");

  std::vector<uint8_t> model;
  if (!LfsReadFile(kModelPath, &model)) {
    printf("ERROR: Failed to load %s\r\n", kModelPath);
    return;
  }

  auto tpu_context =
      EdgeTpuManager::GetSingleton()->OpenDevice(PerformanceMode::kHigh);
  if (!tpu_context) {
    printf("ERROR: Failed to get EdgeTpu context\r\n");
    return;
  }

  
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
  resolver.AddDequantize();
  resolver.AddTranspose();
  resolver.AddReshape();
  resolver.AddConcatenation();
  resolver.AddSoftmax();
  resolver.AddConv2D();
  resolver.AddMul();
  resolver.AddStridedSlice();
  resolver.AddAdd();
  resolver.AddSub();
  resolver.AddCustom(kCustomOp, RegisterCustomOp());
  tflite::MicroInterpreter interpreter(
      tflite::GetModel(model.data()), resolver, tensor_arena,
      kTensorArenaSize, &error_reporter);

  if (interpreter.AllocateTensors() != kTfLiteOk) {
    printf("ERROR: AllocateTensors() failed\r\n");
    return;
  }

  if (interpreter.inputs().size() != 1) {
    printf("ERROR: Model must have only one input tensor\r\n");
    return;
  }

  printf("Tensor arena used: %lu bytes (%lu KB) out of %d bytes (%d KB)\r\n",
         (unsigned long)interpreter.arena_used_bytes(),
         (unsigned long)(interpreter.arena_used_bytes() / 1024),
         kTensorArenaSize,
         kTensorArenaSize / 1024);

  auto* input_tensor = interpreter.input_tensor(0);
  printf("Input tensor: %ld bytes, dims=[%ld,%ld,%ld,%ld], type=%d\r\n",
         (long)input_tensor->bytes,
         (long)input_tensor->dims->data[0],
         (long)input_tensor->dims->data[1],
         (long)input_tensor->dims->data[2],
         (long)input_tensor->dims->data[3],
         (int)input_tensor->type);
  printf("Outputs: %lu\r\n", (unsigned long)interpreter.outputs().size());

  std::vector<uint8_t> image_buffer(input_tensor->bytes);
  if (!LfsReadFile(kImagePath, image_buffer.data(), image_buffer.size())) {
    printf("ERROR: Failed to load %s\r\n", kImagePath);
    return;
  }

  BenchmarkRow rows[kNumRuns] = {};

  uint32_t sum_memcpy_ms = 0;
  uint32_t sum_invoke_ms = 0;
  uint32_t sum_detection_ms = 0;
  uint32_t sum_total_ms = 0;

  for (int i = 0; i < kNumRuns; ++i) {
    TickType_t t0 = xTaskGetTickCount();

    std::memcpy(tflite::GetTensorData<uint8_t>(input_tensor),
                image_buffer.data(), image_buffer.size());

    TickType_t t1 = xTaskGetTickCount();

    if (interpreter.Invoke() != kTfLiteOk) {
      printf("ERROR: Invoke() failed at run %d\r\n", i + 1);
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

printf("%s", report);
fflush(stdout);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  printf("\r\nStarting new\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for console to be ready
  coralmicro::Main();
  vTaskSuspend(nullptr);
}
// [end-sphinx-snippet:detect-image]