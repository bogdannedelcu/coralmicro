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
#include <new>     // std::nothrow for safe `new` on newlib_nano (no exceptions)
#include <string>
#include <vector>

extern "C" {
#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"
}
#undef FAR

#include "libs/base/filesystem.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
#include "libs/tpu/edgetpu_task.h"   // s240: EdgeTpuTask::SetPower for wedge recovery
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "sentai_error.h"
#include "sentai_vision_common.h"

extern "C" {
#include "detection_task.h"
}

// ---- External state defined in sentai_runtime.cc ----
namespace coralmicro {
constexpr int kNumTpuSlots = 3;
constexpr int kSlotArenaSize = 2 * 1024 * 1024;
extern uint8_t tensor_arena[];
extern tflite::MicroInterpreter* g_interpreter;
extern volatile bool g_tpu_ready;
extern std::vector<uint8_t>* g_model_data;
extern uint8_t* g_slot_arena[kNumTpuSlots];
extern tflite::MicroInterpreter* g_slot_interp[kNumTpuSlots];
extern std::vector<uint8_t>* g_slot_model_data[kNumTpuSlots];
extern volatile bool g_slot_ready[kNumTpuSlots];
extern std::shared_ptr<EdgeTpuContext> g_tpu_context;
extern PerformanceMode g_tpu_mode;  // runtime-selectable Edge TPU clock (see sentai_runtime.cc)
}  // namespace coralmicro

static constexpr int kTensorArenaSize = 8 * 1024 * 1024;

extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);

// ---- sentai_load_model ----

// Forward decl: invalidate the descriptor cache in edgetpu_executable.cc
// whenever a new model is loaded — a new package has a new (this, token)
// identity and must upload its parameters + instructions afresh.
extern "C" void sentai_tpu_desc_cache_invalidate(void);

// s240: remember the last-loaded model path so the TPU wedge-recovery
// (sentai_tpu_recover) can re-upload the same model after a power-cycle.
static char g_current_model_path[160] = {0};

// s240/YUV420: set to 1 by the loader when the model has >=2 inputs (dual-input
// YUV420 = UV + Y).  Read by detection_task.cc's PrepTask/InferTask.
extern "C" { volatile int g_sentai_yuv_dual = 0; }

extern "C" int sentai_load_model(const char* path) {
  if (sentai_detection_is_running()) {
    printf("ERROR: stop detection pipeline before loading a new model\r\n");
    return -10;
  }
  if (path && path != g_current_model_path) {
    strncpy(g_current_model_path, path, sizeof(g_current_model_path) - 1);
    g_current_model_path[sizeof(g_current_model_path) - 1] = '\0';
  }
  if (coralmicro::g_interpreter) {
    coralmicro::g_tpu_ready = false;
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
  }
  // W24-T6: the custom-op packages are keyed by the address of the custom-op
  // buffer inside g_model_data; drop THIS model's packages before that buffer
  // is freed so a new model at the same address cannot be served the old
  // executable (and so the per-load EdgeTpuPackage leak stops).  Slot 1..2
  // models keep their packages (per-range, see edgetpu_manager.h).
  sentai_tpu_desc_cache_invalidate();  // always invalidate on (re)load
  if (coralmicro::g_model_data) {
    coralmicro::EdgeTpuManager::GetSingleton()->UnregisterPackagesInRange(
        coralmicro::g_model_data->data(), coralmicro::g_model_data->size());
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
              coralmicro::g_tpu_mode);
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

  // Null-safe `new` (newlib_nano builds without exceptions).
  coralmicro::g_model_data = new(std::nothrow) std::vector<uint8_t>();
  if (!coralmicro::g_model_data) {
    SERR_LOG(SERR_TPU_SLOT_VEC, 0u);
    return -6;
  }
  if (!coralmicro::LfsUserReadFile(path, coralmicro::g_model_data)) {
    SERR_LOG(SERR_TPU_MODEL_LOAD, 0u);
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

  coralmicro::g_interpreter = new(std::nothrow) tflite::MicroInterpreter(
      tflite::GetModel(coralmicro::g_model_data->data()), resolver,
      coralmicro::tensor_arena, kTensorArenaSize, &error_reporter);
  if (!coralmicro::g_interpreter) {
    SERR_LOG(SERR_TPU_SLOT_INTERP, 0u);
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
    return -7;
  }

  if (coralmicro::g_interpreter->AllocateTensors() != kTfLiteOk) {
    SERR_LOG(SERR_TPU_ALLOC_TENSORS, 0u);
    printf("ERROR: AllocateTensors() failed\r\n");
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
    return -3;
  }

  // s240/YUV420: accept multi-input models (e.g. dual-input YUV420 = UV + Y).
  // Single-input RGB models still work unchanged.  Reject only 0 inputs.
  const int n_in = (int)coralmicro::g_interpreter->inputs().size();
  if (n_in < 1) {
    printf("ERROR: Model has no input tensor\r\n");
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

  printf("Inputs: %d\r\n", n_in);
  for (int ii = 0; ii < n_in; ii++) {
    auto* input = coralmicro::g_interpreter->input_tensor(ii);
    printf("  in[%d] %s[", ii, type_name(input->type));
    for (int i = 0; i < input->dims->size; i++)
      printf("%s%ld", i ? "," : "", (long)input->dims->data[i]);
    printf("] (%ld bytes)  scale=%.8f zp=%d\r\n", (long)input->bytes,
           (double)input->params.scale, (int)input->params.zero_point);
  }
  // s240/YUV420: a 2-input model is the dual-input YUV420 net (UV + Y).  Flag it
  // so PrepTask produces Y+UV via PXP (no RGB scale/quant) and InferTask binds
  // both tensors.  Requires the camera in yuv_mode(1) so `raw` is XYUV8888.
  g_sentai_yuv_dual = (n_in >= 2) ? 1 : 0;

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

// ---- sentai_tpu_set_mode: change Edge TPU clock at runtime (energy sweep) ----
//
// applsci energy paper, WS0 transfer-bound test.  Arg: 0=kLow(63) 1=kMedium(125)
// 2=kHigh(250) 3=kMax(500) MHz.  Tears down the interpreter + releases the TPU
// context so the device actually closes (EdgeTpuManager debounces repeat opens on
// a live device — only a full release lets a re-OpenDevice apply a new clock),
// re-opens at the requested clock, then reloads the current model on it.
extern "C" int sentai_tpu_set_mode(int m) {
  if (sentai_detection_is_running()) {
    printf("ERROR: stop detection pipeline before changing TPU clock\r\n");
    return -10;
  }
  coralmicro::PerformanceMode mode;
  switch (m) {
    case 0:  mode = coralmicro::PerformanceMode::kLow;    break;
    case 1:  mode = coralmicro::PerformanceMode::kMedium; break;
    case 2:  mode = coralmicro::PerformanceMode::kHigh;   break;
    default: mode = coralmicro::PerformanceMode::kMax;    break;
  }
  if (coralmicro::g_interpreter) {
    coralmicro::g_tpu_ready = false;
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
  }
  sentai_tpu_desc_cache_invalidate();
  if (coralmicro::g_model_data) {
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
  }
  coralmicro::g_tpu_context.reset();  // drop the last strong ref -> device closes
  coralmicro::g_tpu_mode = mode;
  coralmicro::g_tpu_context =
      coralmicro::EdgeTpuManager::GetSingleton()->OpenDevice(mode);
  if (!coralmicro::g_tpu_context) {
    printf("ERROR: re-open EdgeTPU at mode %d failed\r\n", m);
    return -1;
  }
  printf("Edge TPU clock -> mode %d\r\n", (int)mode);
  if (g_current_model_path[0]) return sentai_load_model(g_current_model_path);
  return 0;
}

// ---- sentai_tpu_recover: PMIC power-cycle + re-enumerate + reload ----
//
// s240 wedge recovery.  When the EdgeTPU wedges under sustained load (SEMC
// contention -> URB timeout cascade; see tpu-usb-wedge-mechanism memory), the
// ONLY safe reset is a full hardware power-cycle — cancelling an in-flight USB
// transfer corrupts the TPU silicon (needs reflash).  Here we:
//   1. tear down the stale interpreter + context (drops the power ref),
//   2. force the EdgeTPU power rail off then on via the PMIC/GPIO
//      (EdgeTpuTask::SetPower -> kEdgeTpuPmic + kEdgeTpuReset, waits PGOOD),
//      which triggers automatic USB re-enumeration (kAttach -> kConnected),
//   3. reload the last model (sentai_load_model re-runs OpenDevice + rebuilds
//      the interpreter + re-uploads the executable to the fresh TPU).
// Caller MUST have stopped the detection pipeline first (sentai_load_model
// refuses while it runs).  Returns 0 on success, sentai_load_model's rc<0 else,
// -20 if no model was ever loaded.
extern "C" int sentai_tpu_recover(void) {
  if (g_current_model_path[0] == '\0') {
    printf("TPU recover: no model was loaded; nothing to restore\r\n");
    return -20;
  }
  printf("TPU recover: power-cycling EdgeTPU + reloading %s\r\n",
         g_current_model_path);

  // 1. Block invokes and tear down the wedged context/interpreter.
  coralmicro::g_tpu_ready = false;
  if (coralmicro::g_interpreter) {
    delete coralmicro::g_interpreter;
    coralmicro::g_interpreter = nullptr;
  }
  sentai_tpu_desc_cache_invalidate();
  if (coralmicro::g_model_data) {
    delete coralmicro::g_model_data;
    coralmicro::g_model_data = nullptr;
  }
  // Releasing the context drops its power ref (dtor -> SetPower(false)).
  coralmicro::g_tpu_context.reset();

  // 2. Force a clean rail power-cycle.  The extra SetPower(false) drives
  //    enabled_count_ to 0 (clamped) so the GPIO actually de-asserts even if
  //    a stale ref lingered; SetPower(true) re-powers + waits PGOOD, and the
  //    USB host re-enumerates the freshly-booted TPU asynchronously.
  auto* tpu = coralmicro::EdgeTpuTask::GetSingleton();
  tpu->SetPower(false);
  vTaskDelay(pdMS_TO_TICKS(50));
  tpu->SetPower(true);
  vTaskDelay(pdMS_TO_TICKS(250));   // let USB re-enumerate to kConnected

  // 3. Re-open + rebuild interpreter + re-upload model (OpenDevice retry loop
  //    inside sentai_load_model covers any residual enumeration latency).
  int rc = sentai_load_model(g_current_model_path);
  if (rc == 0) {
    printf("TPU recover: OK — invokes restored without reflash\r\n");
  } else {
    printf("TPU recover: FAILED (rc=%d) — TPU may need a board power-cycle\r\n", rc);
  }
  return rc;
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

// ============================================================================
// Multi-slot extension (Phase 1, 2026-04-28).
// Slot 0 is the legacy single-slot path — sentai_load_model(path) above.
// Slots 1..N-1 use heap-allocated arenas to avoid disturbing the
// .sdram_bss layout that FileX/LevelX state depends on.
// ============================================================================

extern "C" int sentai_load_model_slot(int slot, const char* path) {
  if (slot < 0 || slot >= coralmicro::kNumTpuSlots) {
    SERR_LOG(SERR_TPU_SLOT_OOB, (uint32_t)slot);
    return -11;
  }

  // Pipeline guard for ALL slots (Phase 2a hardening, 2026-04-29).
  // Once `pipeline.set_slot_for_cam(cam, slot)` binds a slot to a
  // camera, InferTask invokes that slot every frame.  A concurrent
  // load_slot would race against the InferTask hot path: between
  // `delete g_slot_interp[slot]` and the InferTask's
  // `g_slot_interp[slot]->Invoke()` is a use-after-free window even
  // with the slot_ready guard, because InferTask snapshots the
  // pointer locally before deref.  Force the caller to stop the
  // pipeline before any load.
  if (sentai_detection_is_running()) {
    printf("ERROR: stop detection pipeline before loading slot %d\r\n", slot);
    return -10;
  }

  // Slot 0 routes through the legacy load function so its code path
  // is unchanged byte-for-byte from the single-slot build.  Then
  // mirror state into g_slot_interp[0] / g_slot_model_data[0] /
  // g_slot_ready[0] so the unified slot accessors work.
  if (slot == 0) {
    int rc = sentai_load_model(path);
    if (rc == 0) {
      coralmicro::g_slot_interp[0]     = coralmicro::g_interpreter;
      coralmicro::g_slot_model_data[0] = coralmicro::g_model_data;
      coralmicro::g_slot_ready[0]      = coralmicro::g_tpu_ready;
    }
    return rc;
  }

  // Slot N (N >= 1): heap-allocate the arena on first use, then run a
  // fresh load that uses g_slot_* state instead of the legacy globals.
  // Tear-down order matters: clear ready FIRST, then null the
  // pointer with a memory barrier, then delete.  Any reader between
  // the steps sees either ready=false (rejects via slot_ready check)
  // or interp=nullptr (rejects via second leg of the && check) — but
  // never a dangling/freed pointer.
  if (coralmicro::g_slot_interp[slot]) {
    coralmicro::g_slot_ready[slot] = false;
    auto* old_interp = coralmicro::g_slot_interp[slot];
    coralmicro::g_slot_interp[slot] = nullptr;
    __sync_synchronize();   // ensure both stores visible before delete
    delete old_interp;
  }
  if (coralmicro::g_slot_model_data[slot]) {
    auto* old_data = coralmicro::g_slot_model_data[slot];
    coralmicro::g_slot_model_data[slot] = nullptr;
    __sync_synchronize();
    // W24-T6: same per-model package teardown as slot 0 (see sentai_load_model).
    coralmicro::EdgeTpuManager::GetSingleton()->UnregisterPackagesInRange(
        old_data->data(), old_data->size());
    sentai_tpu_desc_cache_invalidate();
    delete old_data;
  }
  if (!coralmicro::g_slot_arena[slot]) {
    // Lazy alloc, 32-byte aligned via malloc + manual round-up
    // (newlib_nano lacks aligned_alloc/posix_memalign).  The 32-byte
    // alignment matches the legacy static buffer so the input tensor
    // lands on an eDMA burst boundary.  We don't track the raw
    // pointer because slot arenas are kept for the lifetime of the
    // boot — never freed.
    uint8_t* raw = (uint8_t*)malloc(coralmicro::kSlotArenaSize + 32);
    if (!raw) {
      SERR_LOG(SERR_TPU_SLOT_ALLOC, (uint32_t)slot);
      printf("ERROR: slot %d arena malloc failed\r\n", slot);
      return -5;
    }
    uint8_t* aligned = (uint8_t*)(((uintptr_t)raw + 31) & ~(uintptr_t)31);
    coralmicro::g_slot_arena[slot] = aligned;
    printf("Slot %d arena heap-allocated at %p (raw=%p, %d KB)\r\n",
           slot, aligned, raw, coralmicro::kSlotArenaSize / 1024);
  }

  if (!coralmicro::g_tpu_context) {
    printf("ERROR: EdgeTPU not ready (load slot 0 first to open the device)\r\n");
    return -1;
  }

  // newlib_nano builds without exceptions: `new` returns nullptr on
  // OOM rather than throwing.  Check before any deref to avoid the
  // null-deref class of bug.
  coralmicro::g_slot_model_data[slot] = new(std::nothrow) std::vector<uint8_t>();
  if (!coralmicro::g_slot_model_data[slot]) {
    SERR_LOG(SERR_TPU_SLOT_VEC, (uint32_t)slot);
    return -6;
  }
  if (!coralmicro::LfsUserReadFile(path,
                                    coralmicro::g_slot_model_data[slot])) {
    SERR_LOG(SERR_TPU_MODEL_LOAD, (uint32_t)slot);
    printf("ERROR: slot %d failed to load %s\r\n", slot, path);
    delete coralmicro::g_slot_model_data[slot];
    coralmicro::g_slot_model_data[slot] = nullptr;
    return -2;
  }
  printf("Slot %d model loaded: %lu bytes\r\n", slot,
         (unsigned long)coralmicro::g_slot_model_data[slot]->size());

  // Re-use the same MicroErrorReporter + resolver as the legacy path
  // (file-scope statics).  Both are stateless after init, so sharing
  // is safe across slots.  Resolver init is guarded by a static bool.
  static tflite::MicroErrorReporter slot_error_reporter;
  static tflite::MicroMutableOpResolver<7> slot_resolver;
  static bool slot_resolver_init = false;
  if (!slot_resolver_init) {
    slot_resolver.AddCustom(coralmicro::kCustomOp,
                            coralmicro::RegisterCustomOp());
    slot_resolver.AddTranspose();
    slot_resolver.AddReshape();
    slot_resolver.AddConcatenation();
    slot_resolver.AddLogistic();
    slot_resolver.AddQuantize();
    slot_resolver.AddDequantize();
    slot_resolver_init = true;
  }

  coralmicro::g_slot_interp[slot] = new(std::nothrow) tflite::MicroInterpreter(
      tflite::GetModel(coralmicro::g_slot_model_data[slot]->data()),
      slot_resolver, coralmicro::g_slot_arena[slot],
      coralmicro::kSlotArenaSize, &slot_error_reporter);
  if (!coralmicro::g_slot_interp[slot]) {
    SERR_LOG(SERR_TPU_SLOT_INTERP, (uint32_t)slot);
    delete coralmicro::g_slot_model_data[slot];
    coralmicro::g_slot_model_data[slot] = nullptr;
    return -7;
  }

  if (coralmicro::g_slot_interp[slot]->AllocateTensors() != kTfLiteOk) {
    SERR_LOG(SERR_TPU_ALLOC_TENSORS, (uint32_t)slot);
    printf("ERROR: slot %d AllocateTensors() failed\r\n", slot);
    delete coralmicro::g_slot_interp[slot];
    coralmicro::g_slot_interp[slot] = nullptr;
    return -3;
  }

  if (coralmicro::g_slot_interp[slot]->inputs().size() != 1) {
    printf("ERROR: slot %d model must have exactly one input tensor\r\n", slot);
    delete coralmicro::g_slot_interp[slot];
    coralmicro::g_slot_interp[slot] = nullptr;
    return -4;
  }

  printf("Slot %d arena used: %lu / %d KB\r\n", slot,
         (unsigned long)(coralmicro::g_slot_interp[slot]->arena_used_bytes()
                         / 1024),
         coralmicro::kSlotArenaSize / 1024);

  coralmicro::g_slot_ready[slot] = true;
  return 0;
}
