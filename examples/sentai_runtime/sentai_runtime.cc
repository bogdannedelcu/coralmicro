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
#include <unistd.h>
#include "build_version.h"

extern "C" {
#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"
}
#undef FAR  // jpeglib defines FAR as empty, conflicts with NXP SDK struct field

#include "libs/base/console_m7.h"
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

// ===================== Camera pipeline optimizations ========================
// Set to 1 to enable, 0 to disable (safe revert).  Build #197+
//
// SENTAI_OPT_FAST_POLL   — reduce GetRawFrame poll from 50ms to 5ms (~22ms less latency)
// SENTAI_OPT_LAZY_DRAW   — skip memcpy to g_draw_rgb on every to_tensor;
//                          copy only when draw() is actually called
// SENTAI_OPT_SKIP_CLEAN  — skip DCACHE_CleanByRange before PXP (dst is always fresh)
// SENTAI_DBG_COLOR_ORDER — print first 4 pixels after PXP (once) to verify R/G/B order
// ============================================================================
#define SENTAI_OPT_FAST_POLL    1
#define SENTAI_OPT_LAZY_DRAW    1
#define SENTAI_OPT_SKIP_CLEAN   1
#define SENTAI_DBG_COLOR_ORDER  0   // verified — R channel is ch0

// Performs object detection with SSD MobileNet, running on the Edge TPU,
// using a local bitmap file as input.
//
// To build and flash from coralmicro root:
//    bash build.sh
//    python3 scripts/flashtool.py -e detect_image

// [start-sphinx-snippet:detect-image]

// =============================================================================
// Boot Logging System
// =============================================================================
// Captures ALL printf/driver output to /log/boot.log until REPL starts.
// On boot: /log/boot.log → /log/boot_old.log, then fresh boot.log created.
// Uses a RAM buffer that's flushed periodically and at REPL start.

namespace {

// Boot log state
static constexpr size_t kBootLogBufSize = 16 * 1024;  // 16 KB buffer
static char g_boot_log_buf[kBootLogBufSize] __attribute__((section(".sdram_bss")));
static volatile size_t g_boot_log_pos = 0;
static volatile bool g_boot_log_active = false;
static volatile bool g_boot_log_fs_ready = false;
static lfs_file_t g_boot_log_file;
static bool g_boot_log_file_open = false;

// Forward declare - flush buffer to file
static void boot_log_flush_to_file();

// Initialize boot logging - rename old log, create new one
static void boot_log_init() {
    g_boot_log_pos = 0;
    g_boot_log_active = true;
    g_boot_log_fs_ready = false;
    g_boot_log_file_open = false;
}

// Called after LFS is ready to set up file
static void boot_log_fs_init() {
    if (!g_boot_log_active) return;
    
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;

    // Create /log directory if it doesn't exist
    lfs_mkdir(lfs, "/log");

    // Check if boot.log exists
    lfs_info info;
    if (lfs_stat(lfs, "/log/boot.log", &info) == LFS_ERR_OK) {
        // Remove old boot_old.log if exists
        lfs_remove(lfs, "/log/boot_old.log");
        // Rename boot.log to boot_old.log
        lfs_rename(lfs, "/log/boot.log", "/log/boot_old.log");
    }

    // Open new boot.log for writing
    if (lfs_file_open(lfs, &g_boot_log_file, "/log/boot.log",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) == LFS_ERR_OK) {
        g_boot_log_file_open = true;
        g_boot_log_fs_ready = true;
        
        // Flush any buffered data
        boot_log_flush_to_file();
    }
}

// Flush RAM buffer to file
static void boot_log_flush_to_file() {
    if (!g_boot_log_file_open || g_boot_log_pos == 0) return;
    
    lfs_t* lfs = coralmicro::LfsUser();
    if (!lfs) return;

    lfs_file_write(lfs, &g_boot_log_file, g_boot_log_buf, g_boot_log_pos);
    lfs_file_sync(lfs, &g_boot_log_file);
    g_boot_log_pos = 0;
}

// Add data to boot log (from _write override)
static void boot_log_write(const char* data, size_t len) {
    if (!g_boot_log_active) return;

    // Add to RAM buffer
    size_t space = kBootLogBufSize - g_boot_log_pos;
    size_t to_copy = (len < space) ? len : space;
    if (to_copy > 0) {
        memcpy(g_boot_log_buf + g_boot_log_pos, data, to_copy);
        g_boot_log_pos += to_copy;
    }

    // If buffer is getting full and FS ready, flush
    if (g_boot_log_fs_ready && g_boot_log_pos > kBootLogBufSize - 512) {
        boot_log_flush_to_file();
    }
}

// Stop boot logging (called when REPL starts)
void boot_log_stop() {
    if (!g_boot_log_active) return;
    
    g_boot_log_active = false;

    // Final flush
    if (g_boot_log_file_open) {
        boot_log_flush_to_file();
        lfs_t* lfs = coralmicro::LfsUser();
        if (lfs) {
            lfs_file_close(lfs, &g_boot_log_file);
        }
        g_boot_log_file_open = false;
    }
}

}  // anonymous namespace

// Override _write to capture all printf output
// This replaces the version in libs/base/console_m7.cc
extern "C" int _write(int handle, char* buffer, int size) {
    if ((handle != STDOUT_FILENO) && (handle != STDERR_FILENO)) {
        return -1;
    }

    // Convert bare \n to \r\n for USB/UART terminals.
    char stack_buf[512];
    char* out = buffer;
    int out_len = size;

    bool needs_patch = false;
    for (int i = 0; i < size; ++i) {
        if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
            needs_patch = true;
            break;
        }
    }
    if (needs_patch) {
        int j = 0;
        int cap = (int)sizeof(stack_buf);
        for (int i = 0; i < size && j < cap - 1; ++i) {
            if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
                stack_buf[j++] = '\r';
            }
            if (j < cap) stack_buf[j++] = buffer[i];
        }
        out = stack_buf;
        out_len = j;
    }

    // Write to console
    coralmicro::ConsoleM7::GetSingleton()->Write(out, out_len);

    // Also capture to boot log if active
    boot_log_write(out, out_len);

    return size;
}

// Export for micropython_task.c to call when REPL starts
extern "C" void sentai_boot_log_stop(void) {
    boot_log_stop();
}

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

  // Open EdgeTPU on this task (not app_main) to avoid interfering with USB init
  coralmicro::PerformanceMode tpu_mode = coralmicro::PerformanceMode::kMax;
  g_tpu_context = EdgeTpuManager::GetSingleton()->OpenDevice(tpu_mode);
  if (!g_tpu_context) {
    logf("ERROR: Failed to get EdgeTpu context\r\n");
  } else {
    logf("Edge TPU opened (mode %d)\r\n", static_cast<int>(tpu_mode));
  }

  // Park this task forever - model loading happens from Python
  vTaskSuspend(NULL);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  
  // Initialize boot logging FIRST (before any printf)
  boot_log_init();
  
  coralmicro::app_start_tick = xTaskGetTickCount();
  coralmicro::logf("\r\nSentAI build #%d (%s)\r\n", BUILD_VERSION, BUILD_TIMESTAMP);
  
  // Initialize LFS and boot log file
  // LFS should be initialized by main_freertos before app_main
  boot_log_fs_init();
  coralmicro::logf("Boot logging to /log/boot.log\r\n");

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
  // EdgeTPU init runs on Main() task — wait up to 15s for it
  if (!coralmicro::g_tpu_context) {
    printf("Waiting for EdgeTPU init");
    for (int i = 0; i < 150 && !coralmicro::g_tpu_context; i++) {
      vTaskDelay(pdMS_TO_TICKS(100));
      if (i % 10 == 9) printf(".");  // dot every second
    }
    printf("\r\n");
    if (!coralmicro::g_tpu_context) {
      printf("ERROR: EdgeTPU not initialized after 15s\r\n");
      printf("  Check: is EdgeTPU connected? Try power-cycling the board.\r\n");
      return -1;
    }
    printf("EdgeTPU ready!\r\n");
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

  // Create resolver with EdgeTPU custom op + CPU ops for YOLO post-processing
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

  // Helper to print type name
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

  // Print input tensor info
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  printf("Input:  %s[", type_name(input->type));
  for (int i = 0; i < input->dims->size; i++)
    printf("%s%ld", i ? "," : "", (long)input->dims->data[i]);
  printf("] (%ld bytes)\r\n", (long)input->bytes);
  printf("  quant: scale=%.8f  zero_point=%d\r\n",
         (double)input->params.scale, (int)input->params.zero_point);

  // Print output tensor info
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

  // If model expects int8 input, apply quantization offset.
  // Image pixels are uint8 [0..255]. For int8 models:
  //   q = clamp(pixel + zero_point, -128, 127)
  // Common case: zero_point=-128 → q = pixel - 128
  if (input->type == kTfLiteInt8) {
    int zp = input->params.zero_point;
    int8_t* dst = reinterpret_cast<int8_t*>(tensor_buf);
    if (zp == -128) {
      uint32_t* p32 = reinterpret_cast<uint32_t*>(dst);
      int n32 = tensor_bytes / 4;
      for (int i = 0; i < n32; i++) p32[i] ^= 0x80808080u;
      for (int i = n32 * 4; i < tensor_bytes; i++)
        dst[i] = (int8_t)((uint8_t)dst[i] ^ 0x80u);
    } else {
      for (int i = 0; i < tensor_bytes; i++) {
        int v = (int)tensor_buf[i] + zp;
        if (v < -128) v = -128;
        if (v > 127) v = 127;
        dst[i] = (int8_t)v;
      }
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

// Get input tensor quantization: scale (float) and zero_point.
// Returns 0 on success, -1 if not ready.
extern "C" int sentai_tpu_input_quant(float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input) return -1;
  *scale = input->params.scale;
  *zero_point = input->params.zero_point;
  return 0;
}

// Get output tensor quantization: scale (float) and zero_point.
// Returns 0 on success, -1 if not ready/invalid idx.
extern "C" int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  if (idx < 0 || idx >= (int)coralmicro::g_interpreter->outputs().size()) return -1;
  auto* t = coralmicro::g_interpreter->output_tensor(idx);
  if (!t) return -1;
  *scale = t->params.scale;
  *zero_point = t->params.zero_point;
  return 0;
}

// Get input tensor type (TfLiteType enum: 9=int8, 3=uint8, 1=float32)
extern "C" int sentai_tpu_input_type(void) {
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -1;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input) return -1;
  return (int)input->type;
}

// ---------------------------------------------------------------------------
// YOLO NMS post-processing
// Output tensor expected shape [1, C, N] where C = 4 + num_classes, N = candidates
// bbox format: cx, cy, w, h (normalized 0-1 or pixel coords, auto-detected)
// Returns 0 on success.  out_count receives number of detections written.
// Each detection in out_buf: [x1, y1, x2, y2, conf_permil, class_id] (6 × int16)
// Coordinates are in model input pixel space (0 .. input_w/h).
// ---------------------------------------------------------------------------

namespace {
struct YoloCandidate {
  float x1, y1, x2, y2;
  float score;
  int16_t class_id;
};
static constexpr int kMaxNmsCandidates = 512;
static YoloCandidate g_nms_cand[kMaxNmsCandidates]
    __attribute__((section(".sdram_bss")));
static bool g_nms_sup[kMaxNmsCandidates];

// ---------------------------------------------------------------------------
// Draw buffer — stores last to_tensor RGB frame (before int8 quant)
// Max 640×640×3 = 1.2 MB in SDRAM
// ---------------------------------------------------------------------------
static constexpr int kMaxDrawPixels = 640 * 640 * 3;
static uint8_t g_draw_rgb[kMaxDrawPixels]
    __attribute__((section(".sdram_bss")));
static int g_draw_w = 0, g_draw_h = 0;
#if SENTAI_OPT_LAZY_DRAW
static bool g_draw_stale = true;  // true = g_draw_rgb needs refresh from tensor
static uint8_t* g_draw_tensor_src = nullptr;  // pointer to last tensor_buf (valid until next to_tensor)
static int g_draw_total = 0;  // total_pixels of last frame
#endif

// COCO 80 class names
static const char* const kCocoNames[80] = {
  "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
  "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
  "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
  "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
  "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
  "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
  "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
  "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
  "remote","keyboard","cell phone","microwave","oven","toaster","sink",
  "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
  "toothbrush"
};

// 10 distinct box colors (R, G, B)
static const uint8_t kBoxColors[10][3] = {
  {255,  56,  56}, { 56, 255,  56}, { 56,  56, 255},
  {255, 255,  56}, {255,  56, 255}, { 56, 255, 255},
  {255, 128,   0}, {  0, 128, 255}, {255,   0, 128},
  {128, 255,   0}
};

// 5×7 bitmap font — printable ASCII 32..126 (95 glyphs)
// Each glyph = 5 bytes (columns). Each byte: bit0 = top row, bit6 = bottom.
static const uint8_t kFont5x7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
  {0x00,0x00,0x5F,0x00,0x00}, // 33 !
  {0x00,0x07,0x00,0x07,0x00}, // 34 "
  {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
  {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
  {0x23,0x13,0x08,0x64,0x62}, // 37 %
  {0x36,0x49,0x55,0x22,0x50}, // 38 &
  {0x00,0x05,0x03,0x00,0x00}, // 39 '
  {0x00,0x1C,0x22,0x41,0x00}, // 40 (
  {0x00,0x41,0x22,0x1C,0x00}, // 41 )
  {0x08,0x2A,0x1C,0x2A,0x08}, // 42 *
  {0x08,0x08,0x3E,0x08,0x08}, // 43 +
  {0x00,0x50,0x30,0x00,0x00}, // 44 ,
  {0x08,0x08,0x08,0x08,0x08}, // 45 -
  {0x00,0x60,0x60,0x00,0x00}, // 46 .
  {0x20,0x10,0x08,0x04,0x02}, // 47 /
  {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
  {0x00,0x42,0x7F,0x40,0x00}, // 49 1
  {0x42,0x61,0x51,0x49,0x46}, // 50 2
  {0x21,0x41,0x45,0x4B,0x31}, // 51 3
  {0x18,0x14,0x12,0x7F,0x10}, // 52 4
  {0x27,0x45,0x45,0x45,0x39}, // 53 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
  {0x01,0x71,0x09,0x05,0x03}, // 55 7
  {0x36,0x49,0x49,0x49,0x36}, // 56 8
  {0x06,0x49,0x49,0x29,0x1E}, // 57 9
  {0x00,0x36,0x36,0x00,0x00}, // 58 :
  {0x00,0x56,0x36,0x00,0x00}, // 59 ;
  {0x00,0x08,0x14,0x22,0x41}, // 60 <
  {0x14,0x14,0x14,0x14,0x14}, // 61 =
  {0x41,0x22,0x14,0x08,0x00}, // 62 >
  {0x02,0x01,0x51,0x09,0x06}, // 63 ?
  {0x32,0x49,0x79,0x41,0x3E}, // 64 @
  {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
  {0x7F,0x49,0x49,0x49,0x36}, // 66 B
  {0x3E,0x41,0x41,0x41,0x22}, // 67 C
  {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
  {0x7F,0x49,0x49,0x49,0x41}, // 69 E
  {0x7F,0x09,0x09,0x01,0x01}, // 70 F
  {0x3E,0x41,0x41,0x51,0x32}, // 71 G
  {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
  {0x00,0x41,0x7F,0x41,0x00}, // 73 I
  {0x20,0x40,0x41,0x3F,0x01}, // 74 J
  {0x7F,0x08,0x14,0x22,0x41}, // 75 K
  {0x7F,0x40,0x40,0x40,0x40}, // 76 L
  {0x7F,0x02,0x04,0x02,0x7F}, // 77 M
  {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
  {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
  {0x7F,0x09,0x09,0x09,0x06}, // 80 P
  {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
  {0x7F,0x09,0x19,0x29,0x46}, // 82 R
  {0x46,0x49,0x49,0x49,0x31}, // 83 S
  {0x01,0x01,0x7F,0x01,0x01}, // 84 T
  {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
  {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
  {0x7F,0x20,0x18,0x20,0x7F}, // 87 W
  {0x63,0x14,0x08,0x14,0x63}, // 88 X
  {0x03,0x04,0x78,0x04,0x03}, // 89 Y
  {0x61,0x51,0x49,0x45,0x43}, // 90 Z
  {0x00,0x00,0x7F,0x41,0x41}, // 91 [
  {0x02,0x04,0x08,0x10,0x20}, // 92 backslash
  {0x41,0x41,0x7F,0x00,0x00}, // 93 ]
  {0x04,0x02,0x01,0x02,0x04}, // 94 ^
  {0x40,0x40,0x40,0x40,0x40}, // 95 _
  {0x00,0x01,0x02,0x04,0x00}, // 96 `
  {0x20,0x54,0x54,0x54,0x78}, // 97 a
  {0x7F,0x48,0x44,0x44,0x38}, // 98 b
  {0x38,0x44,0x44,0x44,0x20}, // 99 c
  {0x38,0x44,0x44,0x48,0x7F}, //100 d
  {0x38,0x54,0x54,0x54,0x18}, //101 e
  {0x08,0x7E,0x09,0x01,0x02}, //102 f
  {0x08,0x14,0x54,0x54,0x3C}, //103 g
  {0x7F,0x08,0x04,0x04,0x78}, //104 h
  {0x00,0x44,0x7D,0x40,0x00}, //105 i
  {0x20,0x40,0x44,0x3D,0x00}, //106 j
  {0x00,0x7F,0x10,0x28,0x44}, //107 k
  {0x00,0x41,0x7F,0x40,0x00}, //108 l
  {0x7C,0x04,0x18,0x04,0x78}, //109 m
  {0x7C,0x08,0x04,0x04,0x78}, //110 n
  {0x38,0x44,0x44,0x44,0x38}, //111 o
  {0x7C,0x14,0x14,0x14,0x08}, //112 p
  {0x08,0x14,0x14,0x18,0x7C}, //113 q
  {0x7C,0x08,0x04,0x04,0x08}, //114 r
  {0x48,0x54,0x54,0x54,0x20}, //115 s
  {0x04,0x3F,0x44,0x40,0x20}, //116 t
  {0x3C,0x40,0x40,0x20,0x7C}, //117 u
  {0x1C,0x20,0x40,0x20,0x1C}, //118 v
  {0x3C,0x40,0x30,0x40,0x3C}, //119 w
  {0x44,0x28,0x10,0x28,0x44}, //120 x
  {0x0C,0x50,0x50,0x50,0x3C}, //121 y
  {0x44,0x64,0x54,0x4C,0x44}, //122 z
  {0x00,0x08,0x36,0x41,0x00}, //123 {
  {0x00,0x00,0x7F,0x00,0x00}, //124 |
  {0x00,0x41,0x36,0x08,0x00}, //125 }
  {0x10,0x08,0x08,0x10,0x08}, //126 ~
};

// Drawing helpers (operate on RGB888 buffer)
static inline void draw_pixel(uint8_t* buf, int bw, int bh,
                               int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  if (x >= 0 && x < bw && y >= 0 && y < bh) {
    int off = (y * bw + x) * 3;
    buf[off] = r; buf[off+1] = g; buf[off+2] = b;
  }
}

static void draw_rect(uint8_t* buf, int bw, int bh,
                       int x1, int y1, int x2, int y2, int thick,
                       uint8_t r, uint8_t g, uint8_t b) {
  for (int t = 0; t < thick; t++) {
    for (int x = x1 - t; x <= x2 + t; x++) {
      draw_pixel(buf, bw, bh, x, y1 - t, r, g, b);
      draw_pixel(buf, bw, bh, x, y2 + t, r, g, b);
    }
    for (int y = y1 - t + 1; y < y2 + t; y++) {
      draw_pixel(buf, bw, bh, x1 - t, y, r, g, b);
      draw_pixel(buf, bw, bh, x2 + t, y, r, g, b);
    }
  }
}

static void draw_filled_rect(uint8_t* buf, int bw, int bh,
                               int x1, int y1, int x2, int y2,
                               uint8_t r, uint8_t g, uint8_t b) {
  for (int y = (y1 < 0 ? 0 : y1); y <= y2 && y < bh; y++)
    for (int x = (x1 < 0 ? 0 : x1); x <= x2 && x < bw; x++) {
      int off = (y * bw + x) * 3;
      buf[off] = r; buf[off+1] = g; buf[off+2] = b;
    }
}

static void draw_char(uint8_t* buf, int bw, int bh, int cx, int cy, char ch,
                       uint8_t r, uint8_t g, uint8_t b) {
  int idx = (int)ch - 32;
  if (idx < 0 || idx >= 95) idx = '?' - 32;
  const uint8_t* glyph = kFont5x7[idx];
  for (int col = 0; col < 5; col++) {
    uint8_t bits = glyph[col];
    for (int row = 0; row < 7; row++) {
      if (bits & (1 << row))
        draw_pixel(buf, bw, bh, cx + col, cy + row, r, g, b);
    }
  }
}

static void draw_string(uint8_t* buf, int bw, int bh, int sx, int sy,
                          const char* str, uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; str[i]; i++)
    draw_char(buf, bw, bh, sx + i * 6, sy, str[i], r, g, b);
}

}  // namespace

extern "C" int sentai_tpu_detect(int conf_permil, int iou_permil,
                                 int max_dets,
                                 int16_t* out_buf, int* out_count) {
  using namespace coralmicro;
  *out_count = 0;
  if (!g_tpu_ready || !g_interpreter) return -1;

  auto* output = g_interpreter->output_tensor(0);
  if (!output || output->dims->size != 3) return -2;

  int C = output->dims->data[1];   // e.g. 84 = 4 bbox + 80 classes
  int N = output->dims->data[2];   // e.g. 2100 candidate anchors
  int num_classes = C - 4;
  if (num_classes <= 0) return -3;

  auto* input = g_interpreter->input_tensor(0);
  int in_h = input->dims->data[1];
  int in_w = input->dims->data[2];

  float scale = output->params.scale;
  int zp = output->params.zero_point;
  const int8_t* data = reinterpret_cast<const int8_t*>(output->data.data);

  float conf_thr = conf_permil / 1000.0f;
  float iou_thr  = iou_permil  / 1000.0f;

  // Phase 1: confidence filter — keep only candidates with max class score > threshold
  int num_cand = 0;
  float max_coord = 0.0f;

  for (int j = 0; j < N && num_cand < kMaxNmsCandidates; j++) {
    // Find best class score for candidate j
    float best_score = -1e9f;
    int best_cls = 0;
    for (int c = 4; c < C; c++) {
      float s = scale * ((int)data[c * N + j] - zp);
      if (s > best_score) { best_score = s; best_cls = c - 4; }
    }
    if (best_score < conf_thr) continue;

    // Dequantize bbox: cx, cy, w, h
    float cx = scale * ((int)data[0 * N + j] - zp);
    float cy = scale * ((int)data[1 * N + j] - zp);
    float bw = scale * ((int)data[2 * N + j] - zp);
    float bh = scale * ((int)data[3 * N + j] - zp);

    float x1 = cx - bw * 0.5f;
    float y1 = cy - bh * 0.5f;
    float x2 = cx + bw * 0.5f;
    float y2 = cy + bh * 0.5f;

    if (x2 > max_coord) max_coord = x2;
    if (y2 > max_coord) max_coord = y2;

    g_nms_cand[num_cand++] = {x1, y1, x2, y2, best_score, (int16_t)best_cls};
  }

  if (num_cand == 0) return 0;

  // Auto-detect normalized (0-1) vs pixel-space coords
  // If the largest coordinate < 2.0 then values are normalized → scale to input dims
  float coord_sx = (max_coord < 2.0f) ? (float)in_w : 1.0f;
  float coord_sy = (max_coord < 2.0f) ? (float)in_h : 1.0f;

  // Phase 2: insertion sort by score descending (small N, stack-friendly)
  for (int i = 1; i < num_cand; i++) {
    YoloCandidate key = g_nms_cand[i];
    int j = i - 1;
    while (j >= 0 && g_nms_cand[j].score < key.score) {
      g_nms_cand[j + 1] = g_nms_cand[j]; j--;
    }
    g_nms_cand[j + 1] = key;
  }

  // Phase 3: greedy NMS (class-aware)
  memset(g_nms_sup, 0, sizeof(bool) * num_cand);
  int count = 0;

  for (int i = 0; i < num_cand && count < max_dets; i++) {
    if (g_nms_sup[i]) continue;
    auto& d = g_nms_cand[i];

    // Scale & clamp to model input pixel space
    float sx1 = d.x1 * coord_sx; if (sx1 < 0) sx1 = 0;
    float sy1 = d.y1 * coord_sy; if (sy1 < 0) sy1 = 0;
    float sx2 = d.x2 * coord_sx; if (sx2 > in_w) sx2 = (float)in_w;
    float sy2 = d.y2 * coord_sy; if (sy2 > in_h) sy2 = (float)in_h;

    out_buf[count * 6 + 0] = (int16_t)(sx1 + 0.5f);
    out_buf[count * 6 + 1] = (int16_t)(sy1 + 0.5f);
    out_buf[count * 6 + 2] = (int16_t)(sx2 + 0.5f);
    out_buf[count * 6 + 3] = (int16_t)(sy2 + 0.5f);
    out_buf[count * 6 + 4] = (int16_t)(d.score * 1000.0f + 0.5f);
    out_buf[count * 6 + 5] = d.class_id;
    count++;

    // Suppress overlapping detections of the same class
    float area_i = (sx2 - sx1) * (sy2 - sy1);
    for (int j = i + 1; j < num_cand; j++) {
      if (g_nms_sup[j]) continue;
      if (g_nms_cand[j].class_id != d.class_id) continue;

      float jx1 = g_nms_cand[j].x1 * coord_sx;
      float jy1 = g_nms_cand[j].y1 * coord_sy;
      float jx2 = g_nms_cand[j].x2 * coord_sx;
      float jy2 = g_nms_cand[j].y2 * coord_sy;

      float xx1 = (sx1 > jx1) ? sx1 : jx1;
      float yy1 = (sy1 > jy1) ? sy1 : jy1;
      float xx2 = (sx2 < jx2) ? sx2 : jx2;
      float yy2 = (sy2 < jy2) ? sy2 : jy2;
      float iw  = (xx2 > xx1) ? (xx2 - xx1) : 0;
      float ih  = (yy2 > yy1) ? (yy2 - yy1) : 0;
      float inter = iw * ih;
      float area_j = (jx2 - jx1) * (jy2 - jy1);
      float iou = inter / (area_i + area_j - inter + 1e-6f);
      if (iou > iou_thr) g_nms_sup[j] = true;
    }
  }

  *out_count = count;
  printf("NMS: %d/%d candidates, %d detections (conf>%d%% iou>%d%%)\r\n",
         num_cand, N, count, conf_permil / 10, iou_permil / 10);
  return 0;
}

// ---------------------------------------------------------------------------
// Draw bounding boxes + labels on the last to_tensor() RGB frame, save JPEG.
// dets: flat array of n_dets × 6 int16: [x1,y1,x2,y2,conf_permil,class_id]
// ---------------------------------------------------------------------------
extern "C" int sentai_tpu_draw(const char* path,
                                const int16_t* dets, int n_dets,
                                int quality) {
  if (g_draw_w == 0 || g_draw_h == 0) return -1;  // no frame saved

#if SENTAI_OPT_LAZY_DRAW
  // Lazy flush: copy tensor data → g_draw_rgb only when draw() is called
  if (g_draw_stale && g_draw_tensor_src && g_draw_total > 0) {
    memcpy(g_draw_rgb, g_draw_tensor_src, g_draw_total);
    g_draw_stale = false;
  }
#endif

  int sz = g_draw_w * g_draw_h * 3;

  // Work on a copy so original is preserved for multiple draw() calls
  uint8_t* rgb = (uint8_t*)malloc(sz);
  if (!rgb) return -2;
  memcpy(rgb, g_draw_rgb, sz);

  for (int i = 0; i < n_dets; i++) {
    int x1   = dets[i*6 + 0];
    int y1   = dets[i*6 + 1];
    int x2   = dets[i*6 + 2];
    int y2   = dets[i*6 + 3];
    int conf = dets[i*6 + 4];
    int cls  = dets[i*6 + 5];

    int ci = cls % 10;
    uint8_t cr = kBoxColors[ci][0];
    uint8_t cg = kBoxColors[ci][1];
    uint8_t cb = kBoxColors[ci][2];

    // Draw bounding box (2px thick)
    draw_rect(rgb, g_draw_w, g_draw_h, x1, y1, x2, y2, 2, cr, cg, cb);

    // Build label: "class_name NN%"
    char label[40];
    const char* name = (cls >= 0 && cls < 80) ? kCocoNames[cls] : "?";
    snprintf(label, sizeof(label), "%s %d%%", name, conf / 10);

    int lw = (int)strlen(label) * 6 + 3;
    int lh = 10;
    int ly = (y1 - lh - 1 >= 0) ? y1 - lh - 1 : y1;  // above box, or inside

    // Colored background for label
    draw_filled_rect(rgb, g_draw_w, g_draw_h, x1, ly, x1 + lw, ly + lh,
                     cr, cg, cb);
    // Black text on colored background
    draw_string(rgb, g_draw_w, g_draw_h, x1 + 2, ly + 2, label, 0, 0, 0);
  }

  // JPEG compress and save
  int jpeg_buf_size = sz;
  if (jpeg_buf_size < 64 * 1024) jpeg_buf_size = 64 * 1024;
  uint8_t* jpeg_buf = (uint8_t*)malloc(jpeg_buf_size);
  int rc = -3;
  if (jpeg_buf) {
    unsigned long jpeg_size = coralmicro::JpegCompressRgb(
        rgb, g_draw_w, g_draw_h, quality,
        jpeg_buf, (unsigned long)jpeg_buf_size);
    if (jpeg_size > 0) {
      std::string data((const char*)jpeg_buf, jpeg_size);
      if (coralmicro::LfsUserWriteFile(path, data)) {
        printf("Draw: %dx%d saved %s (%lu bytes, %d dets)\r\n",
               g_draw_w, g_draw_h, path, jpeg_size, n_dets);
        rc = 0;
      }
    }
    free(jpeg_buf);
  }
  free(rgb);
  return rc;
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
#if !SENTAI_OPT_SKIP_CLEAN
  DCACHE_CleanByRange((uint32_t)dst, dst_size);
#endif
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

#if SENTAI_DBG_COLOR_ORDER
  // Print first 4 pixels every 100th frame to verify channel order.
  {
    static uint32_t s_color_cnt = 0;
    if ((s_color_cnt++ % 100) == 0) {
      printf("[COLOR_DBG] frame#%lu  first 4 px (ch0,ch1,ch2):", s_color_cnt - 1);
      for (int p = 0; p < 4 && p * 3 + 2 < (int)dst_size; p++) {
        printf("  (%u,%u,%u)", dst[p*3+0], dst[p*3+1], dst[p*3+2]);
      }
      printf("\r\n");
    }
  }
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

// Try to get a raw frame with recovery.
// Drains stale buffered frames first so the caller always gets the LATEST frame.
// NOTE: TryGetRawFrame is NOT truly non-blocking — inside the camera task,
// HandleFrameRequest polls GetFullBuffer up to 40×100ms = 4 seconds.
// So each call either succeeds quickly (~1ms) or blocks up to 4s.
// We try ONCE per attempt, then do recovery if it fails.
static int sentai_cam_get_raw_with_recovery(uint8_t** raw_out) {
  auto* cam = coralmicro::CameraTask::GetSingleton();
  const int kMaxRecoveries = 2;
  TickType_t t_start = xTaskGetTickCount();

  for (int recovery = 0; recovery <= kMaxRecoveries; ++recovery) {
    // Strategy: drain the FIFO queue, keep only the LAST (most recent) buffer.
    // With N DMA buffers, at most N-1 can be queued (1 is being written by DMA).
    // The last one out of the queue is the most recently completed frame.
    // We never wait for a NEW frame — just grab what's already available.
    // Only fall back to blocking GetRawFrame if queue was completely empty.
    {
      uint8_t* kept_frame = nullptr;
      int kept_idx = -1;
      int drained = 0;
      for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT - 1; ++i) {
        uint8_t* tmp = nullptr;
        int idx = cam->TryGetRawFrame(&tmp);
        if (idx < 0 || !tmp) break;  // queue empty
        // Return the previous frame, keep this (newer) one
        if (kept_idx >= 0) {
          cam->ReturnRawFrame(kept_idx);
        }
        kept_idx = idx;
        kept_frame = tmp;
        drained++;
      }

      if (kept_idx >= 0) {
        // Got the most recent completed frame — no waiting needed
        TickType_t total = xTaskGetTickCount() - t_start;
        printf("  [frame] drained %d, kept buf#%d (%ldms)\r\n",
               drained, kept_idx, (long)total);
        *raw_out = kept_frame;
        return kept_idx;
      }

      // Queue was empty — camera may be slow or just started.
      // Fall back to blocking wait for the next frame.
      uint8_t* frame = nullptr;
      int idx = cam->GetRawFrame(&frame);
      if (idx >= 0 && frame) {
        TickType_t total = xTaskGetTickCount() - t_start;
        printf("  [frame] queue empty, waited for buf#%d (%ldms)\r\n",
               idx, (long)total);
        *raw_out = frame;
        return idx;
      }
    }

    // No frames at all — try toggling camera to kick CSI/MIPI
    if (recovery < kMaxRecoveries) {
      int other = (g_cam_current_id == 0) ? 1 : 0;
      printf("[CAM] GetRawFrame failed, toggling %d->%d->%d to recover...\r\n",
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
  TickType_t t0 = xTaskGetTickCount();
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  TickType_t t1 = xTaskGetTickCount();
  if (idx < 0 || !raw) return -2;

  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  buf, width, height);
  TickType_t t2 = xTaskGetTickCount();
  cam->ReturnRawFrame(idx);
  printf("  [capture_rgb] drain=%ldms pxp=%ldms\r\n",
         (long)(t1 - t0), (long)(t2 - t1));
  return rc;
}

// Persistent RGB buffer in SDRAM — avoids allocating 2.7MB on every call
static uint8_t s_jpeg_rgb_buf[DEMO_CAMERA_WIDTH * DEMO_CAMERA_HEIGHT * 3]
    __attribute__((section(".sdram_bss")));

// ---- libjpeg buffer-destination manager (same logic as jpeg.cc) -----------
struct sentai_buf_dest_mgr {
  struct jpeg_destination_mgr pub;
  unsigned long capacity;
  unsigned long* out_size;
};

static void sentai_init_dest(j_compress_ptr) {}
static boolean sentai_empty_buf(j_compress_ptr) { return FALSE; }
static void sentai_term_dest(j_compress_ptr cinfo) {
  auto* d = reinterpret_cast<sentai_buf_dest_mgr*>(cinfo->dest);
  *d->out_size = d->capacity - d->pub.free_in_buffer;
}

static void sentai_jpeg_buf_dest(j_compress_ptr cinfo, unsigned char* buf,
                                 unsigned long size, unsigned long* out_size) {
  if (!cinfo->dest)
    cinfo->dest = (struct jpeg_destination_mgr*)(*cinfo->mem->alloc_small)(
        (j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(sentai_buf_dest_mgr));
  auto* d = reinterpret_cast<sentai_buf_dest_mgr*>(cinfo->dest);
  d->pub.init_destination    = sentai_init_dest;
  d->pub.empty_output_buffer = sentai_empty_buf;
  d->pub.term_destination    = sentai_term_dest;
  d->pub.next_output_byte    = buf;
  d->pub.free_in_buffer      = size;
  d->capacity = size;
  d->out_size = out_size;
}

// Direct JPEG compression from raw XRGB8888 camera buffer — bypasses PXP.
// Camera stores pixels as [B, G, R, X] in memory (little-endian XRGB8888).
// Converts one row at a time to [R, G, B] for libjpeg.
// Uses JDCT_IFAST for ~30-40% faster DCT on Cortex-M7.
// Only works at full camera resolution (no scaling).
static unsigned long jpeg_compress_xrgb_direct(
    const uint8_t* xrgb, int width, int height, int pitch_bytes,
    int quality, uint8_t* jpeg_buf, unsigned long jpeg_buf_size) {

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  unsigned long out_size = 0;
  sentai_jpeg_buf_dest(&cinfo, jpeg_buf, jpeg_buf_size, &out_size);

  cinfo.image_width      = width;
  cinfo.image_height     = height;
  cinfo.input_components = 3;
  cinfo.in_color_space   = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  cinfo.dct_method = JDCT_IFAST;  // ~30-40% faster than JDCT_ISLOW
  jpeg_start_compress(&cinfo, TRUE);

  // Row conversion buffer — 1280*3 = 3840 bytes, fine on stack
  uint8_t row_rgb[DEMO_CAMERA_WIDTH * 3];

  while (cinfo.next_scanline < cinfo.image_height) {
    const uint8_t* src_row = xrgb + cinfo.next_scanline * pitch_bytes;
    // Optimized RGBX→RGB using 32-bit reads from non-cacheable OCRAM.
    // One uint32 load per pixel instead of 3 separate byte loads.
    // Memory layout: [R, G, B, X] per pixel.
    // LE uint32 = R | (G<<8) | (B<<16) | (X<<24)
    const uint32_t* src32 = (const uint32_t*)src_row;
    int x = 0;
    // Unrolled 4x for pipeline efficiency
    for (; x + 3 < width; x += 4) {
      uint32_t p0 = src32[x + 0];
      uint32_t p1 = src32[x + 1];
      uint32_t p2 = src32[x + 2];
      uint32_t p3 = src32[x + 3];
      uint8_t* d = &row_rgb[x * 3];
      d[0]  = (uint8_t)p0;          d[1]  = (uint8_t)(p0 >> 8); d[2]  = (uint8_t)(p0 >> 16);
      d[3]  = (uint8_t)p1;          d[4]  = (uint8_t)(p1 >> 8); d[5]  = (uint8_t)(p1 >> 16);
      d[6]  = (uint8_t)p2;          d[7]  = (uint8_t)(p2 >> 8); d[8]  = (uint8_t)(p2 >> 16);
      d[9]  = (uint8_t)p3;          d[10] = (uint8_t)(p3 >> 8); d[11] = (uint8_t)(p3 >> 16);
    }
    // Tail: handle remaining pixels (width not multiple of 4)
    for (; x < width; ++x) {
      row_rgb[x * 3 + 0] = src_row[x * 4 + 0];  // R
      row_rgb[x * 3 + 1] = src_row[x * 4 + 1];  // G
      row_rgb[x * 3 + 2] = src_row[x * 4 + 2];  // B
    }
    JSAMPROW rp = row_rgb;
    jpeg_write_scanlines(&cinfo, &rp, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  return out_size;
}

// Capture + JPEG compress. Returns JPEG size or negative error.
// At full resolution, bypasses PXP entirely (direct XRGB→JPEG row-by-row).
// At scaled resolution, uses PXP hardware scaler + JpegCompressRgb.
extern "C" int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                      int width, int height, int quality) {
  if (!g_cam_initialized) return -1;
  if (width > DEMO_CAMERA_WIDTH || height > DEMO_CAMERA_HEIGHT) return -5;

  // Full resolution — bypass PXP, encode directly from camera buffer
  if (width == DEMO_CAMERA_WIDTH && height == DEMO_CAMERA_HEIGHT) {
    uint8_t* raw = nullptr;
    TickType_t t0 = xTaskGetTickCount();
    int idx = sentai_cam_get_raw_with_recovery(&raw);
    TickType_t t1 = xTaskGetTickCount();
    if (idx < 0 || !raw) return -2;

    int pitch = (DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP;
    unsigned long used = jpeg_compress_xrgb_direct(
        raw, width, height, pitch, quality,
        (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);

    TickType_t t2 = xTaskGetTickCount();
    auto* cam = coralmicro::CameraTask::GetSingleton();
    cam->ReturnRawFrame(idx);

    printf("  [capture_jpeg] drain=%ldms xrgb_jpeg=%ldms total=%ldms (direct)\r\n",
           (long)(t1 - t0), (long)(t2 - t1), (long)(t2 - t0));
    return (int)used;
  }

  // Scaled resolution — need PXP for hardware scaling
  TickType_t t0 = xTaskGetTickCount();
  int rc = sentai_cam_capture_rgb(s_jpeg_rgb_buf, width, height);
  TickType_t t1 = xTaskGetTickCount();
  if (rc != 0) return rc;
  unsigned long used = coralmicro::JpegCompressRgb(
      s_jpeg_rgb_buf, width, height, quality,
      (unsigned char*)jpeg_buf, (unsigned long)jpeg_buf_size);
  TickType_t t2 = xTaskGetTickCount();
  printf("  [capture_jpeg] rgb=%ldms jpeg_encode=%ldms total=%ldms (pxp+scale)\r\n",
         (long)(t1 - t0), (long)(t2 - t1), (long)(t2 - t0));
  return (int)used;
}

// Capture RGB via PXP and feed directly into TPU input tensor.
// If save_path is non-NULL, save a JPEG of the scaled frame before int8 quant.
extern "C" int sentai_cam_to_tensor_ex(const char* save_path, int quality) {
  if (!g_cam_initialized) return -1;
  if (!coralmicro::g_tpu_ready || !coralmicro::g_interpreter) return -3;
  auto* input = coralmicro::g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -4;
  int h = input->dims->data[1];
  int w = input->dims->data[2];
  int ch = input->dims->data[3];
  int total_pixels = h * w * ch;
  uint8_t* tensor_buf = tflite::GetTensorData<uint8_t>(input);

  uint8_t* raw = nullptr;
  int idx = sentai_cam_get_raw_with_recovery(&raw);
  if (idx < 0 || !raw) return -2;
  auto* cam = coralmicro::CameraTask::GetSingleton();
  int rc = pxp_scale_xrgb_to_rgb(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  tensor_buf, w, h);
  cam->ReturnRawFrame(idx);
  if (rc != 0) return rc;

  // Save RGB frame for draw() before int8 quantization
#if SENTAI_OPT_LAZY_DRAW
  // Defer memcpy — only copy if draw() is actually called
  if (total_pixels <= kMaxDrawPixels) {
    g_draw_tensor_src = tensor_buf;
    g_draw_total = total_pixels;
    g_draw_w = w;
    g_draw_h = h;
    g_draw_stale = true;
  }
#else
  if (total_pixels <= kMaxDrawPixels) {
    memcpy(g_draw_rgb, tensor_buf, total_pixels);
    g_draw_w = w;
    g_draw_h = h;
  }
#endif

  // Optionally save JPEG of the scaled RGB frame (before int8 quantization)
  if (save_path && save_path[0]) {
    int jpeg_buf_size = w * h * ch;  // worst-case size
    if (jpeg_buf_size < 64 * 1024) jpeg_buf_size = 64 * 1024;
    uint8_t* jpeg_buf = (uint8_t*)malloc(jpeg_buf_size);
    if (jpeg_buf) {
      unsigned long jpeg_size = coralmicro::JpegCompressRgb(
          tensor_buf, w, h, quality,
          jpeg_buf, (unsigned long)jpeg_buf_size);
      if (jpeg_size > 0) {
        std::string jpeg_data((const char*)jpeg_buf, jpeg_size);
        if (coralmicro::LfsUserWriteFile(save_path, jpeg_data)) {
          printf("Saved %dx%d JPEG to %s (%lu bytes)\r\n", w, h, save_path, jpeg_size);
        } else {
          printf("JPEG save failed: %s\r\n", save_path);
        }
      }
      free(jpeg_buf);
    }
  }

  // If model expects int8 input, apply quantization offset.
  if (input->type == kTfLiteInt8) {
    int8_t* dst = reinterpret_cast<int8_t*>(tensor_buf);
    int zp = input->params.zero_point;
    if (zp == -128) {
      uint32_t* p32 = reinterpret_cast<uint32_t*>(dst);
      int n32 = total_pixels / 4;
      for (int i = 0; i < n32; i++) p32[i] ^= 0x80808080u;
      for (int i = n32 * 4; i < total_pixels; i++)
        dst[i] = (int8_t)((uint8_t)dst[i] ^ 0x80u);
    } else {
      for (int i = 0; i < total_pixels; i++) {
        int v = (int)tensor_buf[i] + zp;
        if (v < -128) v = -128;
        if (v > 127) v = 127;
        dst[i] = (int8_t)v;
      }
    }
  }
  return 0;
}

extern "C" int sentai_cam_to_tensor(void) {
  return sentai_cam_to_tensor_ex(NULL, 75);
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