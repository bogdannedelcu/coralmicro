// tpu_posix_invoke_smoke.cc -- run one EdgeTPU custom-op package through the
// SentAI ARM-like EdgeTpuManager/TpuDriver path using the POSIX/libusb bridge.
//
// This intentionally does not use PyCoral and does not use the host TFLite
// interpreter.  The TFLite flatbuffer is parsed only to find the
// edgetpu-custom-op custom_options blob and tensor metadata.  Execution then
// goes through coralmicro::EdgeTpuManager -> EdgeTpuExecutable -> TpuDriver
// -> USB_HostEdgeTpu* exactly like the ARM path, with SIM/libusb as transport.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/usb_host_edgetpu.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

void MicroPrintf(const char* format, ...) {
  std::printf("[tflm] ");
  va_list ap;
  va_start(ap, format);
  std::vprintf(format, ap);
  va_end(ap);
  std::printf("\n");
}

namespace tflite {

class ErrorReporter;
TfLiteRegistration* Register_DETECTION_POSTPROCESS();

TfLiteStatus ConvertTensorType(TensorType tensor_type, TfLiteType* type,
                               ErrorReporter*) {
  if (!type) return kTfLiteError;
  switch (tensor_type) {
    case TensorType_FLOAT32:
      *type = kTfLiteFloat32;
      return kTfLiteOk;
    case TensorType_INT32:
      *type = kTfLiteInt32;
      return kTfLiteOk;
    case TensorType_UINT8:
      *type = kTfLiteUInt8;
      return kTfLiteOk;
    case TensorType_INT8:
      *type = kTfLiteInt8;
      return kTfLiteOk;
    default:
      *type = kTfLiteNoType;
      return kTfLiteError;
  }
}

}  // namespace tflite

extern "C" volatile int g_sentai_tpu_desc_cache_enabled;
extern "C" volatile uint32_t g_sentai_tpu_desc_cache_sent_params;
extern "C" volatile uint32_t g_sentai_tpu_desc_cache_sent_ins;
extern "C" volatile uint32_t g_sentai_tpu_desc_cache_skip_params;
extern "C" volatile uint32_t g_sentai_tpu_desc_cache_skip_ins;
extern "C" volatile uint32_t g_sentai_tpu_cyc_params;
extern "C" volatile uint32_t g_sentai_tpu_cyc_ins;
extern "C" volatile uint32_t g_sentai_tpu_cyc_input;
extern "C" volatile uint32_t g_sentai_tpu_cyc_output;
extern "C" volatile uint32_t g_sentai_tpu_cyc_event;
extern "C" volatile uint32_t g_sentai_tpu_n_params;
extern "C" volatile uint32_t g_sentai_tpu_n_ins;
extern "C" volatile uint32_t g_sentai_tpu_n_input;
extern "C" volatile uint32_t g_sentai_tpu_n_output;
extern "C" volatile uint32_t g_sentai_tpu_n_event;
extern "C" volatile uint32_t g_sentai_tpu_by_params;
extern "C" volatile uint32_t g_sentai_tpu_by_ins;
extern "C" volatile uint32_t g_sentai_tpu_by_input;
extern "C" volatile uint32_t g_sentai_tpu_by_output;
extern "C" volatile uint32_t g_sentai_tpu_send_params_calls;
extern "C" volatile uint32_t g_sentai_tpu_send_params_bytes;
extern "C" volatile uint32_t g_sentai_tpu_send_ins_calls;
extern "C" volatile uint32_t g_sentai_tpu_send_ins_bytes;
extern "C" volatile uint32_t g_sentai_tpu_send_inputs_calls;
extern "C" volatile uint32_t g_sentai_tpu_send_inputs_bytes;
extern "C" volatile int g_sentai_tpu_multi_ep_routing;
extern "C" volatile int g_sentai_tpu_zero_copy_input;
extern "C" volatile uint32_t g_sentai_tpu_chunk_size;
extern "C" volatile int g_sentai_tpu_async_input_enabled;
extern "C" volatile int g_sentai_tpu_sim_break_on_short_bulkin;
extern "C" volatile uint32_t g_sentai_tpu_sim_bulkin_chunk_size;
extern "C" volatile int g_sentai_tpu_sim_sleep_after_bulkin;
extern "C" volatile uint32_t g_sentai_tpu_sim_bulkin_queue_depth;
extern "C" volatile int g_sentai_tpu_posix_fast_sync_wait;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_out_calls;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_out_req;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_out_done;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_out_us;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_out_short;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_in_calls;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_in_req;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_in_done;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_in_us;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_in_short;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_event_calls;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_event_req;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_event_done;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_event_us;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_event_short;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_intr_calls;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_intr_req;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_intr_done;
extern "C" volatile uint64_t g_sentai_tpu_posix_usb_intr_us;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_intr_short;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_timeouts;
extern "C" volatile uint32_t g_sentai_tpu_posix_usb_failed;
extern "C" void sentai_tpu_call_reset(void);
extern "C" void sentai_tpu_perf_reset(void);
extern "C" void sentai_tpu_posix_usb_stats_reset(void);

namespace {

constexpr const char* kDefaultModel =
    "examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/"
    "iter01_virtual_camera_tpu_cat/fs_root/models/"
    "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite";
constexpr const char* kDefaultBmp =
    "examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/"
    "iter01_virtual_camera_tpu_cat/fs_root/images/cat_640x480.bmp";
constexpr const char* kEdgeTpuCustomOp = "edgetpu-custom-op";

struct SmokeArgs {
  const char* model_path = kDefaultModel;
  const char* bmp_path = kDefaultBmp;
  const char* server_cmd_path = nullptr;
  bool inspect_only = false;
  bool server_mode = false;
  int runs = 1;
  int warmup = 0;
  coralmicro::PerformanceMode perf_mode = coralmicro::PerformanceMode::kLow;
  const char* perf_mode_name = "low";
  int desc_cache = 0;
  int async_input = 0;
  int multi_ep = 0;
  int zero_copy_input = 1;
  int break_short_bulkin = 0;
  int fast_sync_wait = 0;
  uint32_t chunk_size = 0;
  uint32_t bulkin_chunk_size = 256;
  uint32_t bulkin_queue_depth = 0;
  int sleep_after_bulkin = 0;
};

struct CustomOpView {
  const tflite::Operator* op = nullptr;
  const tflite::SubGraph* subgraph = nullptr;
  const tflite::Model* model = nullptr;
  const uint8_t* custom_options = nullptr;
  size_t custom_options_len = 0;
};

struct CenterSizeEncoding {
  float y;
  float x;
  float h;
  float w;
};

struct SmokeDetectionOpData {
  int max_detections;
  int max_classes_per_detection;
  int detections_per_class;
  float non_max_suppression_score_threshold;
  float intersection_over_union_threshold;
  int num_classes;
  bool use_regular_non_max_suppression;
  CenterSizeEncoding scale_values;
  int active_candidate_idx;
  int decoded_boxes_idx;
  int scores_idx;
  int score_buffer_idx;
  int keep_scores_idx;
  int scores_after_regular_non_max_suppression_idx;
  int sorted_values_idx;
  int keep_indices_idx;
  int sorted_indices_idx;
  int buffer_idx;
  int selected_idx;
  TfLiteQuantizationParams input_box_encodings;
  TfLiteQuantizationParams input_class_predictions;
  TfLiteQuantizationParams input_anchors;
};

struct SmokeRuntime {
  std::vector<TfLiteEvalTensor>* tensors = nullptr;
  std::vector<std::vector<uint8_t>> scratch;
  std::vector<void*> persistent;
};

void ResetBenchCounters() {
  sentai_tpu_call_reset();
  sentai_tpu_perf_reset();
  sentai_tpu_posix_usb_stats_reset();
  g_sentai_tpu_desc_cache_sent_params = 0;
  g_sentai_tpu_desc_cache_sent_ins = 0;
  g_sentai_tpu_desc_cache_skip_params = 0;
  g_sentai_tpu_desc_cache_skip_ins = 0;
}

void PrintBenchCounters() {
  std::printf(
      "TPU_STAGE_STATS "
      "params_calls=%u params_bytes=%u params_ticks=%u "
      "ins_calls=%u ins_bytes=%u ins_ticks=%u "
      "input_calls=%u input_bytes=%u input_ticks=%u "
      "output_calls=%u output_bytes=%u output_ticks=%u "
      "event_calls=%u event_ticks=%u\n",
      (unsigned)g_sentai_tpu_n_params, (unsigned)g_sentai_tpu_by_params,
      (unsigned)g_sentai_tpu_cyc_params, (unsigned)g_sentai_tpu_n_ins,
      (unsigned)g_sentai_tpu_by_ins, (unsigned)g_sentai_tpu_cyc_ins,
      (unsigned)g_sentai_tpu_n_input, (unsigned)g_sentai_tpu_by_input,
      (unsigned)g_sentai_tpu_cyc_input, (unsigned)g_sentai_tpu_n_output,
      (unsigned)g_sentai_tpu_by_output, (unsigned)g_sentai_tpu_cyc_output,
      (unsigned)g_sentai_tpu_n_event, (unsigned)g_sentai_tpu_cyc_event);
  std::printf(
      "TPU_CALL_STATS "
      "send_params_calls=%u send_params_bytes=%u "
      "send_ins_calls=%u send_ins_bytes=%u "
      "send_inputs_calls=%u send_inputs_bytes=%u\n",
      (unsigned)g_sentai_tpu_send_params_calls,
      (unsigned)g_sentai_tpu_send_params_bytes,
      (unsigned)g_sentai_tpu_send_ins_calls,
      (unsigned)g_sentai_tpu_send_ins_bytes,
      (unsigned)g_sentai_tpu_send_inputs_calls,
      (unsigned)g_sentai_tpu_send_inputs_bytes);
  std::printf(
      "TPU_USB_STATS "
      "out_calls=%u out_req=%llu out_done=%llu out_us=%llu out_short=%u "
      "in_calls=%u in_req=%llu in_done=%llu in_us=%llu in_short=%u "
      "event_calls=%u event_req=%llu event_done=%llu event_us=%llu event_short=%u "
      "intr_calls=%u intr_req=%llu intr_done=%llu intr_us=%llu intr_short=%u "
      "timeouts=%u failed=%u\n",
      (unsigned)g_sentai_tpu_posix_usb_out_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_out_req,
      (unsigned long long)g_sentai_tpu_posix_usb_out_done,
      (unsigned long long)g_sentai_tpu_posix_usb_out_us,
      (unsigned)g_sentai_tpu_posix_usb_out_short,
      (unsigned)g_sentai_tpu_posix_usb_in_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_in_req,
      (unsigned long long)g_sentai_tpu_posix_usb_in_done,
      (unsigned long long)g_sentai_tpu_posix_usb_in_us,
      (unsigned)g_sentai_tpu_posix_usb_in_short,
      (unsigned)g_sentai_tpu_posix_usb_event_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_event_req,
      (unsigned long long)g_sentai_tpu_posix_usb_event_done,
      (unsigned long long)g_sentai_tpu_posix_usb_event_us,
      (unsigned)g_sentai_tpu_posix_usb_event_short,
      (unsigned)g_sentai_tpu_posix_usb_intr_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_req,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_done,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_us,
      (unsigned)g_sentai_tpu_posix_usb_intr_short,
      (unsigned)g_sentai_tpu_posix_usb_timeouts,
      (unsigned)g_sentai_tpu_posix_usb_failed);
  std::printf(
      "TPU_DESC_CACHE_STATS "
      "sent_params=%u sent_ins=%u skip_params=%u skip_ins=%u\n",
      (unsigned)g_sentai_tpu_desc_cache_sent_params,
      (unsigned)g_sentai_tpu_desc_cache_sent_ins,
      (unsigned)g_sentai_tpu_desc_cache_skip_params,
      (unsigned)g_sentai_tpu_desc_cache_skip_ins);
}

std::vector<uint8_t> ReadFile(const char* path) {
  std::FILE* f = std::fopen(path, "rb");
  if (!f) return {};
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (n <= 0) {
    std::fclose(f);
    return {};
  }
  std::vector<uint8_t> out(static_cast<size_t>(n));
  size_t got = std::fread(out.data(), 1, out.size(), f);
  std::fclose(f);
  if (got != out.size()) return {};
  return out;
}

uint16_t Le16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t Le32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int32_t SLe32(const uint8_t* p) {
  return static_cast<int32_t>(Le32(p));
}

bool LoadBmpResizeRgb(const char* path, int dst_w, int dst_h,
                      std::vector<uint8_t>* out) {
  std::vector<uint8_t> bmp = ReadFile(path);
  if (bmp.size() < 54 || bmp[0] != 'B' || bmp[1] != 'M') return false;
  const uint32_t pix_off = Le32(&bmp[10]);
  const int32_t src_w = SLe32(&bmp[18]);
  const int32_t src_h_raw = SLe32(&bmp[22]);
  const uint16_t planes = Le16(&bmp[26]);
  const uint16_t bpp = Le16(&bmp[28]);
  const uint32_t compression = Le32(&bmp[30]);
  if (planes != 1 || bpp != 24 || compression != 0 ||
      src_w <= 0 || src_h_raw == 0 || pix_off >= bmp.size()) {
    return false;
  }
  const int src_h = src_h_raw < 0 ? -src_h_raw : src_h_raw;
  const bool top_down = src_h_raw < 0;
  const int stride = ((src_w * 3 + 3) / 4) * 4;
  if (pix_off + static_cast<uint32_t>(stride * src_h) > bmp.size()) {
    return false;
  }

  out->assign(static_cast<size_t>(dst_w * dst_h * 3), 0);
  for (int y = 0; y < dst_h; ++y) {
    int sy = (y * src_h) / dst_h;
    int file_y = top_down ? sy : (src_h - 1 - sy);
    const uint8_t* row = bmp.data() + pix_off + file_y * stride;
    for (int x = 0; x < dst_w; ++x) {
      int sx = (x * src_w) / dst_w;
      const uint8_t* bgr = row + sx * 3;
      uint8_t* rgb = out->data() + (y * dst_w + x) * 3;
      rgb[0] = bgr[2];
      rgb[1] = bgr[1];
      rgb[2] = bgr[0];
    }
  }
  return true;
}

int TypeSize(tflite::TensorType t) {
  switch (t) {
    case tflite::TensorType_FLOAT32:
    case tflite::TensorType_INT32:
      return 4;
    case tflite::TensorType_UINT8:
    case tflite::TensorType_INT8:
      return 1;
    default:
      return 1;
  }
}

TfLiteType ToTfLiteType(tflite::TensorType t) {
  switch (t) {
    case tflite::TensorType_FLOAT32:
      return kTfLiteFloat32;
    case tflite::TensorType_INT32:
      return kTfLiteInt32;
    case tflite::TensorType_UINT8:
      return kTfLiteUInt8;
    case tflite::TensorType_INT8:
      return kTfLiteInt8;
    default:
      return kTfLiteNoType;
  }
}

int FlatSize(const flatbuffers::Vector<int32_t>* shape) {
  if (!shape || shape->size() == 0) return 1;
  int n = 1;
  for (uint32_t i = 0; i < shape->size(); ++i) {
    int d = shape->Get(i);
    if (d <= 0) d = 1;
    n *= d;
  }
  return n;
}

TfLiteIntArray* MakeTfLiteIntArray(const flatbuffers::Vector<int32_t>* shape) {
  const int rank = shape ? static_cast<int>(shape->size()) : 0;
  size_t bytes = sizeof(TfLiteIntArray) + sizeof(int) * (rank > 0 ? rank : 1);
  auto* arr = reinterpret_cast<TfLiteIntArray*>(std::calloc(1, bytes));
  arr->size = rank;
  for (int i = 0; i < rank; ++i) arr->data[i] = shape->Get(i);
  return arr;
}

TfLiteIntArray* MakeIndexArray(const flatbuffers::Vector<int32_t>* ids) {
  const int n = ids ? static_cast<int>(ids->size()) : 0;
  size_t bytes = sizeof(TfLiteIntArray) + sizeof(int) * (n > 0 ? n : 1);
  auto* arr = reinterpret_cast<TfLiteIntArray*>(std::calloc(1, bytes));
  arr->size = n;
  for (int i = 0; i < n; ++i) arr->data[i] = ids->Get(i);
  return arr;
}

CustomOpView FindEdgeTpuCustomOp(const uint8_t* model_data, size_t model_len) {
  CustomOpView view;
  flatbuffers::Verifier verifier(model_data, model_len);
  if (!verifier.VerifyBuffer<tflite::Model>()) return view;
  const tflite::Model* model = tflite::GetModel(model_data);
  if (!model || !model->subgraphs() || model->subgraphs()->size() == 0) {
    return view;
  }
  const tflite::SubGraph* sg = model->subgraphs()->Get(0);
  if (!sg || !sg->operators()) return view;
  for (const tflite::Operator* op : *sg->operators()) {
    const tflite::OperatorCode* code =
        model->operator_codes()->Get(op->opcode_index());
    const auto* custom = code ? code->custom_code() : nullptr;
    if (!custom || std::strcmp(custom->c_str(), kEdgeTpuCustomOp) != 0) {
      continue;
    }
    const auto* opts = op->custom_options();
    if (!opts || opts->size() == 0) continue;
    view.op = op;
    view.subgraph = sg;
    view.model = model;
    view.custom_options = opts->data();
    view.custom_options_len = opts->size();
    return view;
  }
  return view;
}

CustomOpView FindCustomOp(const tflite::Model* model,
                          const tflite::SubGraph* sg,
                          const char* custom_name) {
  CustomOpView view;
  if (!model || !sg || !sg->operators()) return view;
  for (const tflite::Operator* op : *sg->operators()) {
    const tflite::OperatorCode* code =
        model->operator_codes()->Get(op->opcode_index());
    const auto* custom = code ? code->custom_code() : nullptr;
    if (!custom || std::strcmp(custom->c_str(), custom_name) != 0) {
      continue;
    }
    view.op = op;
    view.subgraph = sg;
    view.model = model;
    if (op->custom_options()) {
      view.custom_options = op->custom_options()->data();
      view.custom_options_len = op->custom_options()->size();
    }
    return view;
  }
  return view;
}

TfLiteEvalTensor* SmokeGetEvalTensor(const TfLiteContext* context,
                                     int tensor_idx) {
  if (!context || !context->impl_) return nullptr;
  auto* runtime = reinterpret_cast<SmokeRuntime*>(context->impl_);
  auto* tensors = runtime->tensors;
  if (!tensors || tensor_idx < 0 ||
      tensor_idx >= static_cast<int>(tensors->size())) {
    return nullptr;
  }
  return tensors->data() + tensor_idx;
}

void* SmokeAllocatePersistentBuffer(TfLiteContext* context, size_t bytes) {
  auto* runtime = reinterpret_cast<SmokeRuntime*>(context->impl_);
  void* ptr = std::calloc(1, bytes ? bytes : 1);
  if (ptr) runtime->persistent.push_back(ptr);
  return ptr;
}

TfLiteStatus SmokeRequestScratchBuffer(TfLiteContext* context, size_t bytes,
                                       int* buffer_idx) {
  if (!buffer_idx) return kTfLiteError;
  auto* runtime = reinterpret_cast<SmokeRuntime*>(context->impl_);
  *buffer_idx = static_cast<int>(runtime->scratch.size());
  runtime->scratch.emplace_back(bytes ? bytes : 1, 0);
  return kTfLiteOk;
}

void* SmokeGetScratchBuffer(TfLiteContext* context, int buffer_idx) {
  auto* runtime = reinterpret_cast<SmokeRuntime*>(context->impl_);
  if (buffer_idx < 0 ||
      buffer_idx >= static_cast<int>(runtime->scratch.size())) {
    return nullptr;
  }
  return runtime->scratch[buffer_idx].data();
}

const char* TensorTypeName(tflite::TensorType t) {
  switch (t) {
    case tflite::TensorType_FLOAT32:
      return "FLOAT32";
    case tflite::TensorType_INT32:
      return "INT32";
    case tflite::TensorType_UINT8:
      return "UINT8";
    case tflite::TensorType_INT8:
      return "INT8";
    default:
      return "OTHER";
  }
}

void PrintTensor(const tflite::SubGraph* sg, int idx) {
  if (idx < 0) {
    std::printf("      tensor[%d] optional\n", idx);
    return;
  }
  const auto* tensors = sg->tensors();
  if (!tensors || idx >= static_cast<int>(tensors->size())) {
    std::printf("      tensor[%d] out-of-range\n", idx);
    return;
  }
  const tflite::Tensor* t = tensors->Get(idx);
  std::printf("      tensor[%d] name=%s type=%s shape=[", idx,
              t->name() ? t->name()->c_str() : "",
              TensorTypeName(t->type()));
  const auto* shape = t->shape();
  for (uint32_t i = 0; shape && i < shape->size(); ++i) {
    if (i) std::printf(",");
    std::printf("%d", shape->Get(i));
  }
  std::printf("] buffer=%u", t->buffer());
  const auto* q = t->quantization();
  if (q && q->scale() && q->scale()->size() > 0) {
    std::printf(" scale=%g", q->scale()->Get(0));
  }
  if (q && q->zero_point() && q->zero_point()->size() > 0) {
    std::printf(" zp=%lld", static_cast<long long>(q->zero_point()->Get(0)));
  }
  std::printf("\n");
}

void InspectModel(const uint8_t* model_data, size_t model_len) {
  flatbuffers::Verifier verifier(model_data, model_len);
  if (!verifier.VerifyBuffer<tflite::Model>()) {
    std::printf("FAIL inspect verifier\n");
    return;
  }
  const tflite::Model* model = tflite::GetModel(model_data);
  const tflite::SubGraph* sg = model->subgraphs()->Get(0);
  std::printf("MODEL bytes=%zu operators=%u tensors=%u\n", model_len,
              sg->operators()->size(), sg->tensors()->size());
  for (uint32_t op_i = 0; op_i < sg->operators()->size(); ++op_i) {
    const tflite::Operator* op = sg->operators()->Get(op_i);
    const tflite::OperatorCode* code =
        model->operator_codes()->Get(op->opcode_index());
    const auto* custom = code ? code->custom_code() : nullptr;
    std::printf("OP[%u] %s custom_len=%u\n", op_i,
                custom ? custom->c_str() : "builtin",
                op->custom_options() ? op->custom_options()->size() : 0);
    std::printf("  inputs:\n");
    for (uint32_t i = 0; op->inputs() && i < op->inputs()->size(); ++i) {
      PrintTensor(sg, op->inputs()->Get(i));
    }
    std::printf("  outputs:\n");
    for (uint32_t i = 0; op->outputs() && i < op->outputs()->size(); ++i) {
      PrintTensor(sg, op->outputs()->Get(i));
    }
  }
}

void ReportError(TfLiteContext*, const char* msg, ...) {
  std::printf("TFLITE_ERROR %s\n", msg ? msg : "(null)");
}

TfLiteQuantizationParams TensorQuantParams(const tflite::Tensor* tensor) {
  TfLiteQuantizationParams q = {};
  const auto* quant = tensor ? tensor->quantization() : nullptr;
  if (quant && quant->scale() && quant->scale()->size() > 0) {
    q.scale = quant->scale()->Get(0);
  }
  if (quant && quant->zero_point() && quant->zero_point()->size() > 0) {
    q.zero_point = static_cast<int32_t>(quant->zero_point()->Get(0));
  }
  return q;
}

void SetTensorData(TfLiteEvalTensor* tensor, void* data) {
  tensor->data.data = data;
}

void InitEvalTensorFromModel(const tflite::Tensor* src,
                             TfLiteEvalTensor* dst,
                             std::vector<uint8_t>* buffer,
                             const tflite::Model* model,
                             bool allocate_rw) {
  dst->dims = MakeTfLiteIntArray(src->shape());
  dst->type = ToTfLiteType(src->type());
  const int bytes = FlatSize(src->shape()) * TypeSize(src->type());
  const auto* buffers = model ? model->buffers() : nullptr;
  const uint32_t buffer_id = src->buffer();
  const flatbuffers::Vector<uint8_t>* const_data = nullptr;
  if (buffers && buffer_id < buffers->size() && buffers->Get(buffer_id)) {
    const_data = buffers->Get(buffer_id)->data();
  }
  if (const_data && const_data->size() >= static_cast<uint32_t>(bytes)) {
    SetTensorData(dst, const_cast<uint8_t*>(const_data->data()));
  } else if (allocate_rw) {
    buffer->assign(static_cast<size_t>(bytes), 0);
    SetTensorData(dst, buffer->data());
  }
}

void DequantizeTensor(const TfLiteEvalTensor& input,
                      const tflite::Tensor* input_meta,
                      TfLiteEvalTensor* output) {
  const TfLiteQuantizationParams q = TensorQuantParams(input_meta);
  const int n = input.dims ? FlatSize(input_meta->shape()) : 0;
  float* out = output->data.f;
  if (!out) return;
  if (input.type == kTfLiteInt8) {
    const int8_t* in = input.data.int8;
    for (int i = 0; i < n; ++i) {
      out[i] = (static_cast<int>(in[i]) - q.zero_point) * q.scale;
    }
  } else if (input.type == kTfLiteUInt8) {
    const uint8_t* in = input.data.uint8;
    for (int i = 0; i < n; ++i) {
      out[i] = (static_cast<int>(in[i]) - q.zero_point) * q.scale;
    }
  }
}

int RequestDppScratch(TfLiteContext* ctx, SmokeDetectionOpData* d,
                      int num_boxes, int num_classes, int max_detections) {
  SmokeRequestScratchBuffer(ctx, num_boxes, &d->active_candidate_idx);
  SmokeRequestScratchBuffer(ctx, num_boxes * 4 * sizeof(float),
                            &d->decoded_boxes_idx);
  SmokeRequestScratchBuffer(ctx, num_boxes * num_classes * sizeof(float),
                            &d->scores_idx);
  SmokeRequestScratchBuffer(ctx, num_boxes * sizeof(float),
                            &d->score_buffer_idx);
  SmokeRequestScratchBuffer(ctx, num_boxes * sizeof(float),
                            &d->keep_scores_idx);
  SmokeRequestScratchBuffer(ctx, max_detections * num_boxes * sizeof(float),
                            &d->scores_after_regular_non_max_suppression_idx);
  SmokeRequestScratchBuffer(ctx, max_detections * num_boxes * sizeof(float),
                            &d->sorted_values_idx);
  SmokeRequestScratchBuffer(ctx, num_boxes * sizeof(int),
                            &d->keep_indices_idx);
  SmokeRequestScratchBuffer(ctx, max_detections * num_boxes * sizeof(int),
                            &d->sorted_indices_idx);
  const int buffer_size = num_classes > max_detections
                              ? num_classes
                              : max_detections;
  SmokeRequestScratchBuffer(ctx, buffer_size * num_boxes * sizeof(int),
                            &d->buffer_idx);
  const int selected_size = num_boxes < max_detections
                                ? num_boxes
                                : max_detections;
  SmokeRequestScratchBuffer(ctx, selected_size * num_boxes * sizeof(int),
                            &d->selected_idx);
  return 0;
}

uint32_t Checksum(const std::vector<uint8_t>& bytes) {
  uint32_t s = 0;
  for (uint8_t b : bytes) s = (s * 131u) + b;
  return s;
}

void SmokeTask(void* arg) {
  const auto* args = reinterpret_cast<const SmokeArgs*>(arg);
  std::vector<uint8_t> model = ReadFile(args->model_path);
  if (model.empty()) {
    std::printf("FAIL read_model path=%s\n", args->model_path);
    std::fflush(stdout);
    _Exit(2);
  }
  if (args->inspect_only) {
    InspectModel(model.data(), model.size());
    std::fflush(stdout);
    _Exit(0);
  }

  CustomOpView custom = FindEdgeTpuCustomOp(model.data(), model.size());
  if (!custom.op) {
    std::printf("FAIL find_edgetpu_custom_op\n");
    std::fflush(stdout);
    _Exit(3);
  }

  usb_host_edgetpu_instance_t* inst = nullptr;
  usb_status_t st = USB_HostEdgeTpuOpenPosix(&inst);
  if (st != kStatus_USB_Success || !inst) {
    std::printf("FAIL open_posix status=%u\n", (unsigned)st);
    std::fflush(stdout);
    _Exit(4);
  }

  auto* manager = coralmicro::EdgeTpuManager::GetSingleton();
  manager->NotifyConnected(inst);
  auto ctx = manager->OpenDevice(args->perf_mode);
  if (!ctx) {
    std::printf("FAIL opendevice\n");
    USB_HostEdgeTpuClosePosix(inst);
    std::fflush(stdout);
    _Exit(5);
  }

  coralmicro::EdgeTpuPackage* package = manager->RegisterPackage(
      reinterpret_cast<const char*>(custom.custom_options),
      custom.custom_options_len);
  if (!package) {
    std::printf("FAIL register_package custom_options=%zu\n",
                custom.custom_options_len);
    USB_HostEdgeTpuClosePosix(inst);
    std::fflush(stdout);
    _Exit(6);
  }

  CustomOpView post = FindCustomOp(custom.model, custom.subgraph,
                                   "TFLite_Detection_PostProcess");

  std::vector<TfLiteIntArray*> dims_to_free;

  const auto* sg_tensors = custom.subgraph->tensors();
  std::vector<TfLiteEvalTensor> tensors(sg_tensors->size());
  std::vector<std::vector<uint8_t>> buffers(sg_tensors->size());
  for (uint32_t i = 0; i < sg_tensors->size(); ++i) {
    InitEvalTensorFromModel(sg_tensors->Get(i), &tensors[i], &buffers[i],
                            custom.model, true);
    if (tensors[i].dims) dims_to_free.push_back(tensors[i].dims);
  }

  const int input_id = custom.op->inputs()->Get(0);
  const tflite::Tensor* input_tensor = sg_tensors->Get(input_id);
  int input_elements = FlatSize(input_tensor->shape());
  int input_bytes = input_elements * TypeSize(input_tensor->type());
  buffers[input_id].assign(static_cast<size_t>(input_bytes), 127);
  auto reload_input = [&]() -> bool {
    if (input_tensor->type() != tflite::TensorType_UINT8 ||
        !input_tensor->shape() || input_tensor->shape()->size() < 4) {
      tensors[input_id].data.uint8 = buffers[input_id].data();
      return true;
    }
    const int h = input_tensor->shape()->Get(1);
    const int w = input_tensor->shape()->Get(2);
    const int c = input_tensor->shape()->Get(3);
    if (c != 3) {
      tensors[input_id].data.uint8 = buffers[input_id].data();
      return true;
    }
    std::vector<uint8_t> rgb;
    if (!LoadBmpResizeRgb(args->bmp_path, w, h, &rgb) ||
        rgb.size() != buffers[input_id].size()) {
      return false;
    }
    buffers[input_id] = std::move(rgb);
    tensors[input_id].data.uint8 = buffers[input_id].data();
    return true;
  };
  (void)reload_input();
  tensors[input_id].data.uint8 = buffers[input_id].data();

  SmokeRuntime runtime;
  runtime.tensors = &tensors;
  TfLiteContext tflite_ctx = {};
  tflite_ctx.impl_ = &runtime;
  tflite_ctx.tensors_size = tensors.size();
  tflite_ctx.AllocatePersistentBuffer = SmokeAllocatePersistentBuffer;
  tflite_ctx.RequestScratchBufferInArena = SmokeRequestScratchBuffer;
  tflite_ctx.GetScratchBuffer = SmokeGetScratchBuffer;
  tflite_ctx.GetEvalTensor = SmokeGetEvalTensor;
  tflite_ctx.ReportError = ReportError;

  TfLiteNode node = {};
  node.inputs = MakeIndexArray(custom.op->inputs());
  node.outputs = MakeIndexArray(custom.op->outputs());
  node.user_data = package;

  int last_detection_count = 0;
  uint32_t last_output_sum = 0;
  int last_output_bytes_total = 0;
  uint32_t invoke_ms_sum = 0;
  uint32_t invoke_ms_min = 0xFFFFFFFFu;
  uint32_t invoke_ms_max = 0;

  auto run_one = [&](bool print_detections, uint32_t* invoke_ms_out)
      -> TfLiteStatus {
    TickType_t t0 = xTaskGetTickCount();
    TfLiteStatus invoke_st = manager->Invoke(package, &tflite_ctx, &node);
    TickType_t t1 = xTaskGetTickCount();
    const uint32_t invoke_ms =
        static_cast<uint32_t>((t1 - t0) * portTICK_PERIOD_MS);
    if (invoke_ms_out) *invoke_ms_out = invoke_ms;

    runtime.scratch.clear();
    for (void* p : runtime.persistent) std::free(p);
    runtime.persistent.clear();

    TfLiteStatus post_st = kTfLiteOk;
    if (invoke_st == kTfLiteOk && post.op) {
      for (const tflite::Operator* op : *custom.subgraph->operators()) {
        const tflite::OperatorCode* code =
            custom.model->operator_codes()->Get(op->opcode_index());
        if (code && code->custom_code()) continue;
        if (!op->inputs() || !op->outputs() ||
            op->inputs()->size() != 1 || op->outputs()->size() != 1) {
          continue;
        }
        const int in_id = op->inputs()->Get(0);
        const int out_id = op->outputs()->Get(0);
        if (tensors[out_id].type == kTfLiteFloat32 &&
            (tensors[in_id].type == kTfLiteInt8 ||
             tensors[in_id].type == kTfLiteUInt8)) {
          DequantizeTensor(tensors[in_id], sg_tensors->Get(in_id),
                           &tensors[out_id]);
        }
      }

      TfLiteRegistration* post_reg = tflite::Register_DETECTION_POSTPROCESS();
      TfLiteNode post_node = {};
      post_node.inputs = MakeIndexArray(post.op->inputs());
      post_node.outputs = MakeIndexArray(post.op->outputs());
      post_node.custom_initial_data = post.custom_options;
      post_node.custom_initial_data_size =
          static_cast<int>(post.custom_options_len);
      post_node.user_data = post_reg->init(
          &tflite_ctx, reinterpret_cast<const char*>(post.custom_options),
          post.custom_options_len);
      auto* d = reinterpret_cast<SmokeDetectionOpData*>(post_node.user_data);
      if (!d) {
        post_st = kTfLiteError;
      } else {
        const int box_id = post.op->inputs()->Get(0);
        const int score_id = post.op->inputs()->Get(1);
        const int anchor_id = post.op->inputs()->Get(2);
        d->input_box_encodings = TensorQuantParams(sg_tensors->Get(box_id));
        d->input_class_predictions =
            TensorQuantParams(sg_tensors->Get(score_id));
        d->input_anchors = TensorQuantParams(sg_tensors->Get(anchor_id));
        const int num_boxes = tensors[box_id].dims->data[1];
        RequestDppScratch(&tflite_ctx, d, num_boxes, d->num_classes,
                          d->max_detections);
        post_st = post_reg->invoke(&tflite_ctx, &post_node);
      }

      if (post_node.inputs) std::free(post_node.inputs);
      if (post_node.outputs) std::free(post_node.outputs);
    }

    uint32_t output_sum = 0;
    int output_bytes_total = 0;
    for (int i = 0; i < custom.op->outputs()->size(); ++i) {
      const int output_id = custom.op->outputs()->Get(i);
      output_sum ^= Checksum(buffers[output_id]) + 0x9e3779b9u +
                    (output_sum << 6) + (output_sum >> 2);
      output_bytes_total += static_cast<int>(buffers[output_id].size());
    }
    last_output_sum = output_sum;
    last_output_bytes_total = output_bytes_total;

    if (invoke_st == kTfLiteOk && post_st == kTfLiteOk && post.op) {
      const int boxes_id = post.op->outputs()->Get(0);
      const int classes_id = post.op->outputs()->Get(1);
      const int scores_id = post.op->outputs()->Get(2);
      const int num_id = post.op->outputs()->Get(3);
      const int n = tensors[num_id].data.f
          ? static_cast<int>(tensors[num_id].data.f[0])
          : 0;
      last_detection_count = n;
      if (print_detections) {
        std::printf("DETECTIONS n=%d\n", n);
        const int limit = n < 20 ? n : 20;
        for (int i = 0; i < limit; ++i) {
          const float* b = tensors[boxes_id].data.f + i * 4;
          std::printf(
              "DET[%d] class=%.0f score=%.3f box=[%.3f %.3f %.3f %.3f]\n",
              i, tensors[classes_id].data.f[i],
              tensors[scores_id].data.f[i],
              b[0], b[1], b[2], b[3]);
        }
      }
    }
    return (invoke_st == kTfLiteOk && post_st == kTfLiteOk)
               ? kTfLiteOk
               : kTfLiteError;
  };

  auto run_benchmark = [&](int requested_runs, int requested_warmup,
                           bool print_summary) -> TfLiteStatus {
    TfLiteStatus last_status = kTfLiteOk;
    const int warmup = requested_warmup < 0 ? 0 : requested_warmup;
    const int runs = requested_runs <= 0 ? 1 : requested_runs;
    invoke_ms_sum = 0;
    invoke_ms_min = 0xFFFFFFFFu;
    invoke_ms_max = 0;
    ResetBenchCounters();
    for (int i = 0; i < warmup; ++i) {
      uint32_t invoke_ms = 0;
      last_status = run_one(/*print_detections=*/false, &invoke_ms);
      if (last_status != kTfLiteOk) break;
    }
    ResetBenchCounters();

    auto bench_t0 = std::chrono::steady_clock::now();
    int completed = 0;
    for (int i = 0; last_status == kTfLiteOk && i < runs; ++i) {
      uint32_t invoke_ms = 0;
      last_status = run_one(/*print_detections=*/i == runs - 1, &invoke_ms);
      if (last_status != kTfLiteOk) break;
      ++completed;
      invoke_ms_sum += invoke_ms;
      if (invoke_ms < invoke_ms_min) invoke_ms_min = invoke_ms;
      if (invoke_ms > invoke_ms_max) invoke_ms_max = invoke_ms;
    }
    auto bench_t1 = std::chrono::steady_clock::now();
    const auto measured_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(bench_t1 -
                                                              bench_t0)
            .count();
    const uint32_t fps_x100 =
        (completed > 0 && measured_ms > 0)
            ? static_cast<uint32_t>(
                  (static_cast<uint64_t>(completed) * 100000u) /
                  static_cast<uint64_t>(measured_ms))
            : 0;

    if (print_summary) {
      std::printf(
          "%s invoke_ms=%u custom_options=%zu input_bytes=%d output_bytes=%d "
          "output_checksum=%08x outputs=%d post=%d\n",
          last_status == kTfLiteOk ? "OK tpu_posix_invoke" : "FAIL invoke",
          completed == 1 ? invoke_ms_sum
                         : (completed ? invoke_ms_sum / completed : 0),
          custom.custom_options_len, input_bytes, last_output_bytes_total,
          last_output_sum, custom.op->outputs()->size(),
          last_status == kTfLiteOk ? 0 : 1);
      if (runs > 1 || warmup > 0) {
        std::printf(
            "FPS_BENCH runs=%d warmup=%d completed=%d measured_ms=%lld "
            "fps_x100=%u invoke_ms_sum=%u invoke_ms_min=%u invoke_ms_max=%u "
            "detections=%d\n",
            runs, warmup, completed, static_cast<long long>(measured_ms),
            fps_x100, invoke_ms_sum,
            invoke_ms_min == 0xFFFFFFFFu ? 0 : invoke_ms_min, invoke_ms_max,
            last_detection_count);
      }
      PrintBenchCounters();
    }
    return last_status;
  };

  if (args->server_mode) {
    std::printf("SERVER_READY\n");
    std::fflush(stdout);
    char line[128];
    uint32_t last_cmd_seq = 0;
    auto read_next_command = [&]() -> bool {
      if (!args->server_cmd_path) {
        return std::fgets(line, sizeof(line), stdin) != nullptr;
      }
      while (true) {
        std::vector<uint8_t> bytes = ReadFile(args->server_cmd_path);
        if (!bytes.empty()) {
          bytes.push_back(0);
          uint32_t seq = 0;
          int pos = 0;
          if (std::sscanf(reinterpret_cast<const char*>(bytes.data()),
                          "%u %n", &seq, &pos) == 1 &&
              seq != last_cmd_seq && pos > 0 &&
              pos < static_cast<int>(bytes.size())) {
            last_cmd_seq = seq;
            std::snprintf(line, sizeof(line), "%s",
                          reinterpret_cast<const char*>(bytes.data()) + pos);
            return true;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };

    while (read_next_command()) {
      if (std::strncmp(line, "stop", 4) == 0) {
        std::printf("SERVER_STOPPED\n");
        std::fflush(stdout);
        break;
      }
      if (std::strncmp(line, "image", 5) == 0) {
        const bool ok = reload_input();
        std::printf(ok ? "SERVER_IMAGE_OK\n" : "SERVER_IMAGE_FAIL\n");
        std::fflush(stdout);
        continue;
      }
      if (std::strncmp(line, "invoke", 6) == 0) {
        std::printf("SERVER_INVOKE_BEGIN\n");
        std::fflush(stdout);
        TfLiteStatus st = run_benchmark(1, 0, true);
        std::printf("SERVER_DONE cmd=invoke rc=%d invoke_ms=%u detections=%d\n",
                    st == kTfLiteOk ? 0 : 1, invoke_ms_sum,
                    last_detection_count);
        std::fflush(stdout);
        continue;
      }
      if (std::strncmp(line, "fps", 3) == 0) {
        int requested_runs = args->runs <= 0 ? 1 : args->runs;
        int requested_warmup = args->warmup < 0 ? 0 : args->warmup;
        (void)std::sscanf(line + 3, "%d %d", &requested_runs,
                          &requested_warmup);
        std::printf("SERVER_FPS_BEGIN runs=%d warmup=%d\n",
                    requested_runs, requested_warmup);
        std::fflush(stdout);
        TfLiteStatus st =
            run_benchmark(requested_runs, requested_warmup, true);
        std::printf("SERVER_DONE cmd=fps rc=%d invoke_ms_sum=%u detections=%d\n",
                    st == kTfLiteOk ? 0 : 1, invoke_ms_sum,
                    last_detection_count);
        std::fflush(stdout);
        continue;
      }
      std::printf("SERVER_ERROR unknown_command\n");
      std::fflush(stdout);
    }

    std::free(node.inputs);
    std::free(node.outputs);
    for (auto* d : dims_to_free) std::free(d);
    for (void* p : runtime.persistent) std::free(p);
    USB_HostEdgeTpuClosePosix(inst);
    _Exit(0);
  }

  const int warmup = args->warmup < 0 ? 0 : args->warmup;
  const int runs = args->runs <= 0 ? 1 : args->runs;
  TfLiteStatus last_status = run_benchmark(runs, warmup, true);
  std::fflush(stdout);

  std::free(node.inputs);
  std::free(node.outputs);
  for (auto* d : dims_to_free) std::free(d);
  for (void* p : runtime.persistent) std::free(p);
  USB_HostEdgeTpuClosePosix(inst);
  _Exit(last_status == kTfLiteOk ? 0 : 7);
}

}  // namespace

int main(int argc, char** argv) {
  static SmokeArgs args;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--inspect") == 0) {
      args.inspect_only = true;
    } else if (std::strcmp(argv[i], "--server") == 0) {
      args.server_mode = true;
    } else if (std::strcmp(argv[i], "--server-cmd") == 0 && i + 1 < argc) {
      args.server_cmd_path = argv[++i];
    } else if (std::strcmp(argv[i], "--runs") == 0 && i + 1 < argc) {
      args.runs = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--warmup") == 0 && i + 1 < argc) {
      args.warmup = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--perf") == 0 && i + 1 < argc) {
      const char* mode = argv[++i];
      args.perf_mode_name = mode;
      if (std::strcmp(mode, "low") == 0) {
        args.perf_mode = coralmicro::PerformanceMode::kLow;
      } else if (std::strcmp(mode, "medium") == 0) {
        args.perf_mode = coralmicro::PerformanceMode::kMedium;
      } else if (std::strcmp(mode, "high") == 0) {
        args.perf_mode = coralmicro::PerformanceMode::kHigh;
      } else if (std::strcmp(mode, "max") == 0) {
        args.perf_mode = coralmicro::PerformanceMode::kMax;
      } else {
        std::printf("FAIL bad_perf_mode=%s\n", mode);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--desc-cache") == 0) {
      args.desc_cache = 1;
    } else if (std::strcmp(argv[i], "--no-desc-cache") == 0) {
      args.desc_cache = 0;
    } else if (std::strcmp(argv[i], "--async-input") == 0) {
      args.async_input = 1;
    } else if (std::strcmp(argv[i], "--no-async-input") == 0) {
      args.async_input = 0;
    } else if (std::strcmp(argv[i], "--multi-ep") == 0) {
      args.multi_ep = 1;
    } else if (std::strcmp(argv[i], "--no-multi-ep") == 0) {
      args.multi_ep = 0;
    } else if (std::strcmp(argv[i], "--zero-copy-input") == 0) {
      args.zero_copy_input = 1;
    } else if (std::strcmp(argv[i], "--no-zero-copy-input") == 0) {
      args.zero_copy_input = 0;
    } else if (std::strcmp(argv[i], "--break-short-bulkin") == 0) {
      args.break_short_bulkin = 1;
    } else if (std::strcmp(argv[i], "--no-break-short-bulkin") == 0) {
      args.break_short_bulkin = 0;
    } else if (std::strcmp(argv[i], "--fast-sync-wait") == 0) {
      args.fast_sync_wait = 1;
    } else if (std::strcmp(argv[i], "--no-fast-sync-wait") == 0) {
      args.fast_sync_wait = 0;
    } else if (std::strcmp(argv[i], "--chunk-size") == 0 && i + 1 < argc) {
      args.chunk_size = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--chunk-kb") == 0 && i + 1 < argc) {
      args.chunk_size =
          static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0) * 1024u);
    } else if (std::strcmp(argv[i], "--bulkin-chunk-size") == 0 &&
               i + 1 < argc) {
      args.bulkin_chunk_size =
          static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--bulkin-chunk-kb") == 0 &&
               i + 1 < argc) {
      args.bulkin_chunk_size =
          static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0) * 1024u);
    } else if (std::strcmp(argv[i], "--bulkin-queue-depth") == 0 &&
               i + 1 < argc) {
      args.bulkin_queue_depth =
          static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--sleep-after-bulkin") == 0) {
      args.sleep_after_bulkin = 1;
    } else if (std::strcmp(argv[i], "--no-sleep-after-bulkin") == 0) {
      args.sleep_after_bulkin = 0;
    } else if (args.model_path == kDefaultModel) {
      args.model_path = argv[i];
    } else {
      args.bmp_path = argv[i];
    }
  }
  g_sentai_tpu_desc_cache_enabled = args.desc_cache ? 1 : 0;
  g_sentai_tpu_async_input_enabled = args.async_input ? 1 : 0;
  g_sentai_tpu_multi_ep_routing = args.multi_ep ? 1 : 0;
  g_sentai_tpu_zero_copy_input = args.zero_copy_input ? 1 : 0;
  g_sentai_tpu_sim_break_on_short_bulkin = args.break_short_bulkin ? 1 : 0;
  g_sentai_tpu_sim_sleep_after_bulkin = args.sleep_after_bulkin ? 1 : 0;
  g_sentai_tpu_posix_fast_sync_wait = args.fast_sync_wait ? 1 : 0;
  if (args.bulkin_chunk_size < 64u) args.bulkin_chunk_size = 64u;
  if (args.bulkin_chunk_size > 64u * 1024u) {
    args.bulkin_chunk_size = 64u * 1024u;
  }
  g_sentai_tpu_sim_bulkin_chunk_size = args.bulkin_chunk_size;
  if (args.bulkin_queue_depth > 32u) args.bulkin_queue_depth = 32u;
  g_sentai_tpu_sim_bulkin_queue_depth = args.bulkin_queue_depth;
  if (args.chunk_size != 0) {
    if (args.chunk_size < 4096u) args.chunk_size = 4096u;
    if (args.chunk_size > 160u * 1024u) args.chunk_size = 160u * 1024u;
    g_sentai_tpu_chunk_size = args.chunk_size;
  }
  std::printf(
      "TPU_BENCH_CONFIG perf=%s desc_cache=%d chunk_size=%u "
      "bulkin_chunk_size=%u bulkin_queue_depth=%u sleep_after_bulkin=%d "
      "async_input=%d zero_copy_input=%d multi_ep=%d "
      "break_short_bulkin=%d fast_sync_wait=%d runs=%d warmup=%d\n",
      args.perf_mode_name, args.desc_cache, (unsigned)g_sentai_tpu_chunk_size,
      (unsigned)g_sentai_tpu_sim_bulkin_chunk_size,
      (unsigned)g_sentai_tpu_sim_bulkin_queue_depth,
      args.sleep_after_bulkin,
      args.async_input, args.zero_copy_input, args.multi_ep,
      args.break_short_bulkin, args.fast_sync_wait, args.runs, args.warmup);
  if (xTaskCreate(SmokeTask, "tpu_invoke", 16384, &args,
                  tskIDLE_PRIORITY + 1, nullptr) != pdPASS) {
    std::printf("FAIL xTaskCreate\n");
    return 1;
  }
  vTaskStartScheduler();
  std::printf("FAIL scheduler_returned\n");
  return 1;
}
