// sentai_slow_bridge.cc — Large, rarely-called functions extracted from
// sentai_runtime.cc so the linker can place them in OCRAM (.sentai_slow)
// instead of the 252KB ITCM (.text) region.
//
// Functions here: sentai_load_model, sentai_load_image, sentai_save_output
//
// NOTE: GCC 9 ARM ignores __attribute__((section())) on extern "C" functions.
// The ONLY way to move code to OCRAM is via a separate .cc file with linker
// script glob rules.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"
}
#undef FAR

#include "libs/base/filesystem.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "sentai_vision_common.h"

extern "C" {
#include "detection_task.h"
}

// ---- External state defined in sentai_runtime.cc ----
namespace coralmicro {
extern uint8_t tensor_arena[];
extern tflite::MicroInterpreter* g_interpreter;
extern volatile bool g_tpu_ready;
extern std::vector<uint8_t>* g_model_data;
extern std::shared_ptr<EdgeTpuContext> g_tpu_context;
}  // namespace coralmicro

static constexpr int kTensorArenaSize = 8 * 1024 * 1024;

extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);

// ---- sentai_load_model ----

// Forward decl: invalidate the descriptor cache in edgetpu_executable.cc
// whenever a new model is loaded — a new package has a new (this, token)
// identity and must upload its parameters + instructions afresh.
extern "C" void sentai_tpu_desc_cache_invalidate(void);

extern "C" int sentai_load_model(const char* path) {
  if (sentai_detection_is_running()) {
    printf("ERROR: stop detection pipeline before loading a new model\r\n");
    return -10;
  }
  if (coralmicro::g_interpreter) {
    coralmicro::g_tpu_ready = false;
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
  }
  sentai_tpu_desc_cache_invalidate();  // always invalidate on (re)load
  if (coralmicro::g_model_data) {
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
  }
  if (!coralmicro::g_tpu_context) {
    // If the context was lost (EdgeTPU USB re-enumerated, brownout,
    // etc.), Main()'s one-shot OpenDevice() won't run again because
    // that task is parked.  Retry from here: try OpenDevice() every
    // second for up to 15 s so a transient USB blip doesn't brick
    // model loading.  The coralmicro::EdgeTpuManager singleton
    // internally debounces repeat opens — calling it on an already-
    // live device is a no-op returning the existing context.
    printf("Waiting for EdgeTPU init");
    for (int i = 0; i < 15 && !coralmicro::g_tpu_context; i++) {
      coralmicro::g_tpu_context =
          coralmicro::EdgeTpuManager::GetSingleton()->OpenDevice(
              coralmicro::PerformanceMode::kMax);
      if (coralmicro::g_tpu_context) break;
      printf(".");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\r\n");
    if (!coralmicro::g_tpu_context) {
      printf("ERROR: EdgeTPU not initialized after 15s\r\n");
      printf("  Check: is EdgeTPU connected? Try power-cycling the board.\r\n");
      return -1;
    }
    printf("EdgeTPU ready!\r\n");
  }

  coralmicro::g_model_data = new std::vector<uint8_t>();
  if (!coralmicro::LfsUserReadFile(path, coralmicro::g_model_data)) {
    printf("ERROR: Failed to load %s\r\n", path);
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
    return -2;
  }
  printf("Model loaded: %lu bytes\r\n",
         (unsigned long)coralmicro::g_model_data->size());

  static tflite::MicroErrorReporter error_reporter;
  static tflite::MicroMutableOpResolver<7> resolver;
  static bool resolver_init = false;
  if (!resolver_init) {
    resolver.AddCustom(coralmicro::kCustomOp, coralmicro::RegisterCustomOp());
    resolver.AddTranspose();
    resolver.AddReshape();
    resolver.AddConcatenation();
    resolver.AddLogistic();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver_init = true;
  }

  coralmicro::g_interpreter = new tflite::MicroInterpreter(
      tflite::GetModel(coralmicro::g_model_data->data()), resolver,
      coralmicro::tensor_arena, kTensorArenaSize, &error_reporter);

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

  auto type_name = [](TfLiteType t) -> const char* {
    switch (t) {
      case kTfLiteFloat32: return "float32";
      case kTfLiteInt32:   return "int32";
      case kTfLiteUInt8:   return "uint8";
      case kTfLiteInt8:    return "int8";
      case kTfLiteInt16:   return "int16";
      case kTfLiteFloat16: return "float16";
      default:             return "unknown";
    }
  };

  auto* input = coralmicro::g_interpreter->input_tensor(0);
  printf("Input:  %s[", type_name(input->type));
  for (int i = 0; i < input->dims->size; i++)
    printf("%s%ld", i ? "," : "", (long)input->dims->data[i]);
  printf("] (%ld bytes)\r\n", (long)input->bytes);
  printf("  quant: scale=%.8f  zero_point=%d\r\n",
         (double)input->params.scale, (int)input->params.zero_point);

  int num_out = (int)coralmicro::g_interpreter->outputs().size();
  printf("Outputs: %d\r\n", num_out);
  for (int oi = 0; oi < num_out; oi++) {
    auto* t = coralmicro::g_interpreter->output_tensor(oi);
    printf("  [%d] %s[", oi, type_name(t->type));
    for (int i = 0; i < t->dims->size; i++)
      printf("%s%ld", i ? "," : "", (long)t->dims->data[i]);
    printf("] (%ld bytes)\r\n", (long)t->bytes);
    printf("      quant: scale=%.8f  zero_point=%d\r\n",
           (double)t->params.scale, (int)t->params.zero_point);
  }

  printf("Arena used: %lu / %d KB\r\n",
         (unsigned long)(coralmicro::g_interpreter->arena_used_bytes() / 1024),
         kTensorArenaSize / 1024);

  coralmicro::g_tpu_ready = true;

  vision_set_model_name(path);
  printf("Model name: %s\r\n", vision_get_model_name());

  return 0;
}

// ---- sentai_load_image ----

extern "C" int sentai_load_image(const char* path) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;

  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -2;

  int tensor_h = input->dims->data[1];
  int tensor_w = input->dims->data[2];
  int tensor_c = input->dims->data[3];
  int tensor_bytes = tensor_h * tensor_w * tensor_c;
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  std::vector<uint8_t> file_data;
  if (!coralmicro::LfsUserReadFile(path, &file_data)) {
    printf("ERROR: Failed to read %s\r\n", path);
    return -3;
  }

  bool is_jpeg = (file_data.size() >= 3 &&
                  file_data[0] == 0xFF &&
                  file_data[1] == 0xD8 &&
                  file_data[2] == 0xFF);

  if (is_jpeg) {
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
    int img_c = cinfo.output_components;
    int row_stride = img_w * img_c;

    printf("JPEG: %dx%d ch=%d -> tensor %dx%dx%d\r\n",
           img_w, img_h, img_c, tensor_w, tensor_h, tensor_c);

    int copy_w = (img_w < tensor_w) ? img_w : tensor_w;
    int copy_c = (img_c < tensor_c) ? img_c : tensor_c;
    int copy_bytes = copy_w * copy_c;

    memset(tensor_buf, 0, tensor_bytes);

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
    int raw_size = (int)file_data.size();
    printf("Raw image: %d bytes -> tensor %d bytes\r\n", raw_size, tensor_bytes);

    if (raw_size >= tensor_bytes) {
      memcpy(tensor_buf, file_data.data(), tensor_bytes);
    } else {
      memset(tensor_buf, 0, tensor_bytes);
      memcpy(tensor_buf, file_data.data(), raw_size);
    }
  }

  if (input->type == kTfLiteInt8) {
    sentai_quant_uint8_to_int8(tensor_buf, tensor_bytes,
                               input->params.zero_point);
  }

  return 0;
}

// ---- sentai_save_output ----

extern "C" int sentai_save_output(const char* path) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;

  int num_out = (int)coralmicro::g_interpreter->outputs().size();
  if (num_out == 0) return -2;

  std::string csv;
  char tmp[32];

  for (int oi = 0; oi < num_out; oi++) {
    auto* t = coralmicro::g_interpreter->output_tensor(oi);
    if (!t) continue;

    snprintf(tmp, sizeof(tmp), "# output %d, type=%d, dims=", oi, (int)t->type);
    csv += tmp;
    for (int d = 0; d < t->dims->size; d++) {
      if (d > 0) csv += 'x';
      snprintf(tmp, sizeof(tmp), "%d", t->dims->data[d]);
      csv += tmp;
    }
    csv += '\n';

    int total = 1;
    for (int d = 0; d < t->dims->size; d++)
      total *= t->dims->data[d];

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
