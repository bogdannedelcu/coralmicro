// TFL (CPU-only TFLite Micro) bridge — separate file for OCRAM placement.
// All code in this file is placed in .sentai_slow (OCRAM) by the linker script,
// freeing precious ITCM (.text) space for latency-critical paths.

#include <cstdio>
#include <cstring>
#include <new>     // std::nothrow for safe `new` on newlib_nano (no exceptions)
#include <vector>

#include "sentai_error.h"
#include "libs/base/filesystem.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"

// TFL state — owned by this file, accessible via extern "C" bridge functions.
namespace coralmicro {
tflite::MicroInterpreter* g_tfl_interpreter = nullptr;
volatile bool g_tfl_ready = false;
std::vector<uint8_t>* g_tfl_model_data = nullptr;
uint8_t* g_tfl_arena_raw = nullptr;
uint8_t* g_tfl_arena = nullptr;
int g_tfl_arena_size = 0;
}  // namespace coralmicro

// Internal — free TFL resources.
static void tfl_cleanup() {
  if (coralmicro::g_tfl_interpreter) {
    delete coralmicro::g_tfl_interpreter;
    coralmicro::g_tfl_interpreter = nullptr;
  }
  if (coralmicro::g_tfl_model_data) {
    delete coralmicro::g_tfl_model_data;
    coralmicro::g_tfl_model_data = nullptr;
  }
  if (coralmicro::g_tfl_arena_raw) {
    free(coralmicro::g_tfl_arena_raw);
    coralmicro::g_tfl_arena_raw = nullptr;
    coralmicro::g_tfl_arena = nullptr;
    coralmicro::g_tfl_arena_size = 0;
  }
  coralmicro::g_tfl_ready = false;
}

// Load a TFLite model for CPU-only inference.
// arena_kb: tensor arena size in KB (default 1024 = 1 MB).
// Returns 0 on success, negative on error.
static int tfl_load_impl(const char* path, int arena_kb) {
  tfl_cleanup();

  if (arena_kb <= 0) arena_kb = 1024;
  int arena_bytes = arena_kb * 1024;

  // Allocate arena with 16-byte alignment
  coralmicro::g_tfl_arena_raw = (uint8_t*)malloc(arena_bytes + 16);
  if (!coralmicro::g_tfl_arena_raw) {
    SERR_LOG(SERR_TPU_ARENA_ALLOC, arena_kb);
    return -5;
  }
  coralmicro::g_tfl_arena = (uint8_t*)(
      ((uintptr_t)coralmicro::g_tfl_arena_raw + 15) & ~(uintptr_t)15);
  coralmicro::g_tfl_arena_size = arena_bytes;

  // Load model from user LFS — null-safe `new` (newlib_nano without exceptions).
  coralmicro::g_tfl_model_data = new(std::nothrow) std::vector<uint8_t>();
  if (!coralmicro::g_tfl_model_data) {
    SERR_LOG(SERR_TPU_SLOT_VEC, 0u);
    tfl_cleanup();
    return -6;
  }
  if (!coralmicro::LfsUserReadFile(path, coralmicro::g_tfl_model_data)) {
    SERR_LOG(SERR_TPU_MODEL_LOAD, 0);
    tfl_cleanup();
    return -2;
  }
  printf("TFL model loaded: %lu bytes\r\n",
         (unsigned long)coralmicro::g_tfl_model_data->size());

  // MicroMutableOpResolver with CPU-only ops (no EdgeTPU custom op).
  // Covers FC-based models (hello_world, keyword detection, classifiers).
  // For CNN models, use sentai.tpu (EdgeTPU) instead — much faster.
  static tflite::MicroMutableOpResolver<14> tfl_resolver;
  static tflite::MicroErrorReporter tfl_error_reporter;
  static bool tfl_resolver_init = false;
  if (!tfl_resolver_init) {
    tfl_resolver.AddFullyConnected();
    tfl_resolver.AddSoftmax();
    tfl_resolver.AddLogistic();
    tfl_resolver.AddRelu();
    tfl_resolver.AddRelu6();
    tfl_resolver.AddAdd();
    tfl_resolver.AddMul();
    tfl_resolver.AddMean();
    tfl_resolver.AddReshape();
    tfl_resolver.AddConcatenation();
    tfl_resolver.AddTranspose();
    tfl_resolver.AddQuantize();
    tfl_resolver.AddDequantize();
    tfl_resolver.AddL2Normalization();
    tfl_resolver_init = true;
  }

  coralmicro::g_tfl_interpreter = new(std::nothrow) tflite::MicroInterpreter(
      tflite::GetModel(coralmicro::g_tfl_model_data->data()), tfl_resolver,
      coralmicro::g_tfl_arena, coralmicro::g_tfl_arena_size,
      &tfl_error_reporter);
  if (!coralmicro::g_tfl_interpreter) {
    SERR_LOG(SERR_TPU_SLOT_INTERP, 0u);
    tfl_cleanup();
    return -7;
  }

  if (coralmicro::g_tfl_interpreter->AllocateTensors() != kTfLiteOk) {
    SERR_LOG(SERR_TPU_ALLOC_TENSORS, arena_kb);
    tfl_cleanup();
    return -3;
  }

  if (coralmicro::g_tfl_interpreter->inputs().size() != 1) {
    SERR_LOG(SERR_TPU_INPUT_COUNT, coralmicro::g_tfl_interpreter->inputs().size());
    tfl_cleanup();
    return -4;
  }

  auto* input = coralmicro::g_tfl_interpreter->input_tensor(0);
  printf("TFL Input: type=%d [", (int)input->type);
  for (int i = 0; i < input->dims->size; i++)
    printf("%s%ld", i ? "," : "", (long)input->dims->data[i]);
  printf("] %ld B  scale=%.6f zp=%d\r\n", (long)input->bytes,
         (double)input->params.scale, (int)input->params.zero_point);

  int num_out = (int)coralmicro::g_tfl_interpreter->outputs().size();
  for (int oi = 0; oi < num_out; oi++) {
    auto* t = coralmicro::g_tfl_interpreter->output_tensor(oi);
    printf("TFL Out[%d]: type=%d [", oi, (int)t->type);
    for (int i = 0; i < t->dims->size; i++)
      printf("%s%ld", i ? "," : "", (long)t->dims->data[i]);
    printf("] %ld B  scale=%.6f zp=%d\r\n", (long)t->bytes,
           (double)t->params.scale, (int)t->params.zero_point);
  }

  printf("TFL Arena used: %lu / %d KB\r\n",
         (unsigned long)(coralmicro::g_tfl_interpreter->arena_used_bytes() / 1024),
         arena_kb);

  coralmicro::g_tfl_ready = true;
  return 0;
}

// ---- extern "C" bridge functions called from MicroPython (modsentai_tfl.c) ----

extern "C" int sentai_tfl_load(const char* path, int arena_kb) {
  return tfl_load_impl(path, arena_kb);
}

extern "C" void sentai_tfl_unload(void) {
  tfl_cleanup();
  printf("TFL model unloaded\r\n");
}

extern "C" int sentai_tfl_invoke(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  TickType_t t0 = xTaskGetTickCount();
  if (coralmicro::g_tfl_interpreter->Invoke() != kTfLiteOk) return -2;
  TickType_t t1 = xTaskGetTickCount();
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

extern "C" int sentai_tfl_is_ready(void) {
  return coralmicro::g_tfl_ready ? 1 : 0;
}

extern "C" int sentai_tfl_num_outputs(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  return (int)coralmicro::g_tfl_interpreter->outputs().size();
}

extern "C" int sentai_tfl_get_output_size(int idx) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_tfl_interpreter->outputs().size()) return 0;
  return (int)coralmicro::g_tfl_interpreter->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tfl_get_output_data(int idx) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return NULL;
  if (idx < 0 || idx >= (int)coralmicro::g_tfl_interpreter->outputs().size()) return NULL;
  return coralmicro::g_tfl_interpreter->output_tensor(idx)->data.data;
}

extern "C" int sentai_tfl_get_output_num_dims(int idx) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  if (idx < 0 || idx >= (int)coralmicro::g_tfl_interpreter->outputs().size()) return 0;
  return coralmicro::g_tfl_interpreter->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tfl_get_output_dim(int idx, int dim) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  auto* t = coralmicro::g_tfl_interpreter->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tfl_get_output_type(int idx) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_tfl_interpreter->outputs().size()) return -1;
  return (int)coralmicro::g_tfl_interpreter->output_tensor(idx)->type;
}

extern "C" int sentai_tfl_input_quant(float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  auto* input = coralmicro::g_tfl_interpreter->input_tensor(0);
  if (!input) return -1;
  *scale = input->params.scale;
  *zero_point = input->params.zero_point;
  return 0;
}

extern "C" int sentai_tfl_output_quant(int idx, float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_tfl_interpreter->outputs().size()) return -1;
  auto* t = coralmicro::g_tfl_interpreter->output_tensor(idx);
  if (!t) return -1;
  *scale = t->params.scale;
  *zero_point = t->params.zero_point;
  return 0;
}

extern "C" int sentai_tfl_input_type(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  auto* input = coralmicro::g_tfl_interpreter->input_tensor(0);
  if (!input) return -1;
  return (int)input->type;
}

extern "C" int sentai_tfl_input_size(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  return (int)coralmicro::g_tfl_interpreter->input_tensor(0)->bytes;
}

extern "C" int sentai_tfl_input_num_dims(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  return coralmicro::g_tfl_interpreter->input_tensor(0)->dims->size;
}

extern "C" int sentai_tfl_input_dim(int dim) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return 0;
  auto* input = coralmicro::g_tfl_interpreter->input_tensor(0);
  if (!input || dim < 0 || dim >= input->dims->size) return 0;
  return input->dims->data[dim];
}

extern "C" int sentai_tfl_set_input(const uint8_t* data, int size) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return -1;
  auto* input = coralmicro::g_tfl_interpreter->input_tensor(0);
  if (!input) return -1;
  if (size != (int)input->bytes) return -2;
  memcpy(input->data.data, data, size);
  return 0;
}

extern "C" void* sentai_tfl_get_input_data(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) return NULL;
  return coralmicro::g_tfl_interpreter->input_tensor(0)->data.data;
}

extern "C" int sentai_tfl_load_image(const char* /* path */) {
  return -99;  // not implemented — use set_input() with fs.read()
}

extern "C" int sentai_tfl_save_output(const char* /* path */) {
  return -99;  // not implemented — use output_floats() or output()
}

extern "C" int sentai_tfl_info(void) {
  if (!coralmicro::g_tfl_ready || !coralmicro::g_tfl_interpreter) {
    printf("TFL: no model loaded\r\n");
    return -1;
  }
  printf("TFL Arena: %lu / %d KB\r\n",
         (unsigned long)(coralmicro::g_tfl_interpreter->arena_used_bytes() / 1024),
         coralmicro::g_tfl_arena_size / 1024);
  printf("TFL Inputs: %d  Outputs: %d\r\n",
         (int)coralmicro::g_tfl_interpreter->inputs().size(),
         (int)coralmicro::g_tfl_interpreter->outputs().size());
  return 0;
}
