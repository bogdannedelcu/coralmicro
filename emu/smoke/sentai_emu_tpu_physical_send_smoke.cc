// B8.11 guest-side physical Coral smoke.
//
// This target keeps the EdgeTPU ownership in the emulated ARM firmware:
//   FxUser/FileX reads model + image
//   EdgeTpuManager parses/registers the model package
//   EdgeTpuExecutable emits SendParameters/SendInputs/SendInstructions/
//       GetOutputs/ReadEvent
//   sentai_emu_tpu_driver_stubs.cc forwards those calls through Renode MMIO
//       to the host POSIX/libusb TpuDriver bridge.
//
// There is intentionally no PyCoral and no high-level host invoke here.

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

#include "libs/base/fx_user_fs.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/usb_host_edgetpu.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_tpu_model_bytes;
extern "C" volatile uint32_t g_sentai_emu_tpu_image_bytes;
extern "C" volatile uint32_t g_sentai_emu_tpu_input_bytes;
extern "C" volatile uint32_t g_sentai_emu_tpu_output_bytes;
extern "C" volatile uint32_t g_sentai_emu_tpu_completed;
extern "C" volatile uint32_t g_sentai_emu_tpu_fail_code;
extern "C" volatile uint32_t g_sentai_emu_tpu_first_ms;
extern "C" volatile uint32_t g_sentai_emu_tpu_steady_ms;
extern "C" volatile uint32_t g_sentai_emu_tpu_output_sum;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_params_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_input_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_ins_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_output_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_event_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_last_result;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootTaskRunning = 0x0300;
constexpr uint32_t kBootPass = 0x0B00;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uint32_t kFailFxInit = 0xE801;
constexpr uint32_t kFailReadModel = 0xE802;
constexpr uint32_t kFailReadImage = 0xE803;
constexpr uint32_t kFailFindCustomOp = 0xE804;
constexpr uint32_t kFailOpenDevice = 0xE805;
constexpr uint32_t kFailRegisterPackage = 0xE806;
constexpr uint32_t kFailBuildTensors = 0xE807;
constexpr uint32_t kFailInvoke = 0xE808;

constexpr const char* kModelPath =
    "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite";
constexpr const char* kImagePath = "/images/cat_640x480.bmp";
constexpr const char* kEdgeTpuCustomOp = "edgetpu-custom-op";
#ifndef SENTAI_EMU_TPU_PHYSICAL_SEND_RUNS
#define SENTAI_EMU_TPU_PHYSICAL_SEND_RUNS 2
#endif
constexpr int kInvokeRuns = SENTAI_EMU_TPU_PHYSICAL_SEND_RUNS;

constexpr uintptr_t kLpuart6Base = 0x40090000u;
constexpr uint32_t kLpuartStatOffset = 0x14u;
constexpr uint32_t kLpuartCtrlOffset = 0x18u;
constexpr uint32_t kLpuartDataOffset = 0x1Cu;
constexpr uint32_t kLpuartStatTdre = 1u << 23;
constexpr uint32_t kLpuartCtrlRe = 1u << 18;
constexpr uint32_t kLpuartCtrlTe = 1u << 19;

StaticTask_t g_tpu_tcb;
StackType_t g_tpu_stack[configMINIMAL_STACK_SIZE * 96]
    __attribute__((aligned(8)));

struct CustomOpView {
  const tflite::Operator* op = nullptr;
  const tflite::SubGraph* subgraph = nullptr;
  const tflite::Model* model = nullptr;
  const uint8_t* custom_options = nullptr;
  size_t custom_options_len = 0;
};

struct Runtime {
  std::vector<TfLiteEvalTensor>* tensors = nullptr;
};

volatile uint32_t& UartReg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kLpuart6Base + offset);
}

void UartInit() {
  UartReg(kLpuartCtrlOffset) = kLpuartCtrlRe | kLpuartCtrlTe;
}

void UartPutChar(char ch) {
  for (int i = 0; i < 1000; ++i) {
    if (UartReg(kLpuartStatOffset) & kLpuartStatTdre) break;
  }
  UartReg(kLpuartDataOffset) = static_cast<uint8_t>(ch);
}

void UartWrite(const char* s) {
  while (*s) UartPutChar(*s++);
}

void UartWriteHex(uint32_t value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  UartWrite("0x");
  for (int shift = 28; shift >= 0; shift -= 4) {
    UartPutChar(kHex[(value >> shift) & 0xFu]);
  }
}

void UartWriteDec(uint32_t value) {
  char tmp[11];
  int n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0u && n < static_cast<int>(sizeof(tmp)));
  while (n > 0) UartPutChar(tmp[--n]);
}

void UartMetric(const char* name, uint32_t value) {
  UartWrite(name);
  UartWrite("=");
  UartWriteDec(value);
  UartWrite("\r\n");
}

void Fail(uint32_t code) {
  g_sentai_emu_tpu_fail_code = code;
  g_sentai_emu_boot_state = code;
  UartWrite("TPU_PHYSICAL_SEND FAIL ");
  UartWriteHex(code);
  UartWrite("\r\n");
}

std::vector<uint8_t> ReadFsFile(const char* path) {
  const ssize_t size = FxUserSize(path);
  if (size <= 0) return {};
  std::vector<uint8_t> out(static_cast<size_t>(size));
  const size_t got = FxUserReadFile(path, out.data(), out.size());
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

bool LoadBmpResizeRgb(const std::vector<uint8_t>& bmp, int dst_w, int dst_h,
                      std::vector<uint8_t>* out) {
  if (bmp.size() < 54 || bmp[0] != 'B' || bmp[1] != 'M') return false;
  const uint32_t pix_off = Le32(&bmp[10]);
  const int32_t src_w = SLe32(&bmp[18]);
  const int32_t src_h_raw = SLe32(&bmp[22]);
  const uint16_t planes = Le16(&bmp[26]);
  const uint16_t bpp = Le16(&bmp[28]);
  const uint32_t compression = Le32(&bmp[30]);
  if (planes != 1 || bpp != 24 || compression != 0 || src_w <= 0 ||
      src_h_raw == 0 || pix_off >= bmp.size()) {
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
    const int sy = (y * src_h) / dst_h;
    const int file_y = top_down ? sy : (src_h - 1 - sy);
    const uint8_t* row = bmp.data() + pix_off + file_y * stride;
    for (int x = 0; x < dst_w; ++x) {
      const int sx = (x * src_w) / dst_w;
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
  const size_t bytes =
      sizeof(TfLiteIntArray) + sizeof(int) * (rank > 0 ? rank : 1);
  auto* arr = reinterpret_cast<TfLiteIntArray*>(calloc(1, bytes));
  if (!arr) return nullptr;
  arr->size = rank;
  for (int i = 0; i < rank; ++i) arr->data[i] = shape->Get(i);
  return arr;
}

TfLiteIntArray* MakeIndexArray(const flatbuffers::Vector<int32_t>* ids) {
  const int n = ids ? static_cast<int>(ids->size()) : 0;
  const size_t bytes =
      sizeof(TfLiteIntArray) + sizeof(int) * (n > 0 ? n : 1);
  auto* arr = reinterpret_cast<TfLiteIntArray*>(calloc(1, bytes));
  if (!arr) return nullptr;
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
  if (!sg || !sg->operators() || !model->operator_codes()) return view;
  for (const tflite::Operator* op : *sg->operators()) {
    const tflite::OperatorCode* code =
        model->operator_codes()->Get(op->opcode_index());
    const auto* custom = code ? code->custom_code() : nullptr;
    if (!custom || strcmp(custom->c_str(), kEdgeTpuCustomOp) != 0) {
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

TfLiteEvalTensor* GetEvalTensor(const TfLiteContext* context, int tensor_idx) {
  if (!context || !context->impl_) return nullptr;
  auto* runtime = reinterpret_cast<Runtime*>(context->impl_);
  if (!runtime->tensors || tensor_idx < 0 ||
      tensor_idx >= static_cast<int>(runtime->tensors->size())) {
    return nullptr;
  }
  return runtime->tensors->data() + tensor_idx;
}

void ReportError(TfLiteContext*, const char* msg, ...) {
  (void)msg;
}

uint32_t Checksum(const uint8_t* data, size_t size) {
  uint32_t sum = 0;
  for (size_t i = 0; i < size; ++i) sum = (sum * 131u) + data[i];
  return sum;
}

void TpuTask(void*) {
  g_sentai_emu_boot_state = kBootTaskRunning;
  UartWrite("TPU_PHYSICAL_SEND BEGIN\r\n");
  UartMetric("RUNS", static_cast<uint32_t>(kInvokeRuns));

  if (!FxUserInit(0)) {
    Fail(kFailFxInit);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  std::vector<uint8_t> model = ReadFsFile(kModelPath);
  if (model.empty()) {
    Fail(kFailReadModel);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  g_sentai_emu_tpu_model_bytes = static_cast<uint32_t>(model.size());
  UartMetric("MODEL_BYTES", g_sentai_emu_tpu_model_bytes);

  std::vector<uint8_t> bmp = ReadFsFile(kImagePath);
  if (bmp.empty()) {
    Fail(kFailReadImage);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  g_sentai_emu_tpu_image_bytes = static_cast<uint32_t>(bmp.size());
  UartMetric("IMAGE_BYTES", g_sentai_emu_tpu_image_bytes);

  CustomOpView custom = FindEdgeTpuCustomOp(model.data(), model.size());
  if (!custom.op) {
    Fail(kFailFindCustomOp);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  static usb_host_edgetpu_instance_t dummy_usb = {};
  auto* manager = coralmicro::EdgeTpuManager::GetSingleton();
  manager->NotifyConnected(&dummy_usb);
  auto context = manager->OpenDevice(coralmicro::PerformanceMode::kLow);
  if (!context) {
    Fail(kFailOpenDevice);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  coralmicro::EdgeTpuPackage* package = manager->RegisterPackage(
      reinterpret_cast<const char*>(custom.custom_options),
      custom.custom_options_len);
  if (!package) {
    Fail(kFailRegisterPackage);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  const auto* sg_tensors = custom.subgraph->tensors();
  std::vector<TfLiteEvalTensor> tensors(sg_tensors->size());
  std::vector<std::vector<uint8_t>> buffers(sg_tensors->size());
  std::vector<TfLiteIntArray*> dims_to_free;

  for (uint32_t i = 0; i < sg_tensors->size(); ++i) {
    const tflite::Tensor* t = sg_tensors->Get(i);
    tensors[i].dims = MakeTfLiteIntArray(t->shape());
    tensors[i].type = ToTfLiteType(t->type());
    if (!tensors[i].dims) {
      Fail(kFailBuildTensors);
      while (true) vTaskDelay(pdMS_TO_TICKS(100));
    }
    dims_to_free.push_back(tensors[i].dims);
  }

  const int input_id = custom.op->inputs()->Get(0);
  const tflite::Tensor* input_tensor = sg_tensors->Get(input_id);
  if (!input_tensor->shape() || input_tensor->shape()->size() < 4 ||
      input_tensor->type() != tflite::TensorType_UINT8) {
    Fail(kFailBuildTensors);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  const int input_h = input_tensor->shape()->Get(1);
  const int input_w = input_tensor->shape()->Get(2);
  const int input_c = input_tensor->shape()->Get(3);
  if (input_c != 3 || !LoadBmpResizeRgb(bmp, input_w, input_h,
                                        &buffers[input_id])) {
    Fail(kFailReadImage);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  tensors[input_id].data.uint8 = buffers[input_id].data();
  g_sentai_emu_tpu_input_bytes =
      static_cast<uint32_t>(buffers[input_id].size());
  UartMetric("INPUT_BYTES", g_sentai_emu_tpu_input_bytes);

  uint32_t output_bytes_total = 0;
  for (uint32_t i = 0; i < custom.op->outputs()->size(); ++i) {
    const int output_id = custom.op->outputs()->Get(i);
    const tflite::Tensor* out_tensor = sg_tensors->Get(output_id);
    const int bytes = FlatSize(out_tensor->shape()) * TypeSize(out_tensor->type());
    buffers[output_id].assign(static_cast<size_t>(bytes), 0);
    tensors[output_id].data.uint8 = buffers[output_id].data();
    output_bytes_total += static_cast<uint32_t>(bytes);
  }
  g_sentai_emu_tpu_output_bytes = output_bytes_total;
  UartMetric("OUTPUT_BYTES", g_sentai_emu_tpu_output_bytes);

  Runtime runtime;
  runtime.tensors = &tensors;
  TfLiteContext tflite_ctx = {};
  tflite_ctx.impl_ = &runtime;
  tflite_ctx.tensors_size = tensors.size();
  tflite_ctx.GetEvalTensor = GetEvalTensor;
  tflite_ctx.ReportError = ReportError;

  TfLiteNode node = {};
  node.inputs = MakeIndexArray(custom.op->inputs());
  node.outputs = MakeIndexArray(custom.op->outputs());
  node.user_data = package;
  if (!node.inputs || !node.outputs) {
    Fail(kFailBuildTensors);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  uint32_t steady_ms_sum = 0;
  uint32_t output_sum = 0;
  for (int run = 0; run < kInvokeRuns; ++run) {
    const TickType_t t0 = xTaskGetTickCount();
    TfLiteStatus st = manager->Invoke(package, &tflite_ctx, &node);
    const TickType_t t1 = xTaskGetTickCount();
    const uint32_t elapsed_ms =
        static_cast<uint32_t>((t1 - t0) * portTICK_PERIOD_MS);
    if (st != kTfLiteOk) {
      Fail(kFailInvoke);
      while (true) vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (run == 0) {
      g_sentai_emu_tpu_first_ms = elapsed_ms;
    } else {
      steady_ms_sum += elapsed_ms;
    }
    ++g_sentai_emu_tpu_completed;
    UartWrite("INVOKE ");
    UartWriteDec(static_cast<uint32_t>(run + 1));
    UartWrite(" ms=");
    UartWriteDec(elapsed_ms);
    UartWrite("\r\n");
  }
  g_sentai_emu_tpu_steady_ms =
      kInvokeRuns > 1 ? steady_ms_sum / static_cast<uint32_t>(kInvokeRuns - 1) : 0;

  for (uint32_t i = 0; i < custom.op->outputs()->size(); ++i) {
    const int output_id = custom.op->outputs()->Get(i);
    output_sum ^= Checksum(buffers[output_id].data(), buffers[output_id].size()) +
                  0x9e3779b9u + (output_sum << 6) + (output_sum >> 2);
  }
  g_sentai_emu_tpu_output_sum = output_sum;

  UartMetric("FIRST_INVOKE_MS", g_sentai_emu_tpu_first_ms);
  UartMetric("STEADY_INVOKE_MS", g_sentai_emu_tpu_steady_ms);
  UartMetric("COMPLETED", g_sentai_emu_tpu_completed);
  UartMetric("OUTPUT_CHECKSUM", g_sentai_emu_tpu_output_sum);
  UartMetric("BRIDGE_PARAMS_CALLS", g_sentai_emu_tpu_bridge_params_calls);
  UartMetric("BRIDGE_INPUT_CALLS", g_sentai_emu_tpu_bridge_input_calls);
  UartMetric("BRIDGE_INS_CALLS", g_sentai_emu_tpu_bridge_ins_calls);
  UartMetric("BRIDGE_OUTPUT_CALLS", g_sentai_emu_tpu_bridge_output_calls);
  UartMetric("BRIDGE_EVENT_CALLS", g_sentai_emu_tpu_bridge_event_calls);
  UartMetric("BRIDGE_LAST_RESULT", g_sentai_emu_tpu_bridge_last_result);
  UartWrite("TPU_PHYSICAL_SEND PASS\r\n");

  for (auto* d : dims_to_free) free(d);
  free(node.inputs);
  free(node.outputs);

  g_sentai_emu_boot_state = kBootPass;
  while (true) {
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_tpu_model_bytes = 0;
volatile uint32_t g_sentai_emu_tpu_image_bytes = 0;
volatile uint32_t g_sentai_emu_tpu_input_bytes = 0;
volatile uint32_t g_sentai_emu_tpu_output_bytes = 0;
volatile uint32_t g_sentai_emu_tpu_completed = 0;
volatile uint32_t g_sentai_emu_tpu_fail_code = 0;
volatile uint32_t g_sentai_emu_tpu_first_ms = 0;
volatile uint32_t g_sentai_emu_tpu_steady_ms = 0;
volatile uint32_t g_sentai_emu_tpu_output_sum = 0;
}

extern "C" int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  g_sentai_emu_boot_state = kBootEnteredMain;
  UartInit();
  UartWrite("SentAI EMU TPU physical Send* smoke boot\r\n");

  TaskHandle_t task =
      xTaskCreateStatic(TpuTask, "tpu_phy_send",
                        configMINIMAL_STACK_SIZE * 96, nullptr,
                        tskIDLE_PRIORITY + 2, g_tpu_stack, &g_tpu_tcb);
  if (!task) {
    g_sentai_emu_boot_state = kBootCreateTaskFailed;
    while (true) {}
  }

  g_sentai_emu_boot_state = kBootTaskCreated;
  vTaskStartScheduler();

  g_sentai_emu_boot_state = kBootSchedulerReturned;
  while (true) {}
}
