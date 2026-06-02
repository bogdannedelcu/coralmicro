// sentai_tpu_posix_backend.cc -- SIM TPU backend using direct POSIX/libusb.
//
// This is the simulator platform backend for the shared sentai.tpu ABI.  It
// intentionally does not call PyCoral: the SIM process talks to the Coral USB
// EdgeTPU through the same CoralMicro EdgeTpuManager/TFLite-Micro path as ARM,
// with only the USB transport injected by usb_host_edgetpu_posix.c.

#include "examples/sentai_runtime/sentai_tpu_shim.h"
#include "examples/sentai_runtime/sentai_virtual_camera.h"
#include "examples/sentai_runtime/sentai_prep.h"
#include "examples/sentai_runtime/sentai_log.h"

#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "libs/tpu/edgetpu_manager.h"
#include "libs/tpu/edgetpu_op.h"
#include "libs/tpu/usb_host_edgetpu.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "third_party/tflite-micro/tensorflow/lite/schema/schema_generated.h"

extern "C" int sentai_fs_size(const char* path);
extern "C" int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
extern "C" int sentai_fs_write(const char* path, const uint8_t* buf, int size);
extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);
extern "C" int sim_fs_resolve(const char* bpath, char* out, size_t outsz);
extern "C" int sentai_camera_backend_publish_prep_once(void);
extern "C" volatile uint8_t g_sentai_tpu_trace;
extern "C" volatile SemaphoreHandle_t g_sentai_tpu_input_done_sema;

namespace {

constexpr int kNumTpuSlots = 3;
constexpr int kTensorArenaSize = 8 * 1024 * 1024;
constexpr int kSlotArenaSize = 2 * 1024 * 1024;
constexpr int kMaxModelBytes = 16 * 1024 * 1024;
constexpr int kMaxImageBytes = 640 * 480 * 3;
constexpr int kMaxDrawBmpBytes = 54 + 1024 * 1024;
constexpr int kMaxHostOutputs = 8;
constexpr int kMaxHostOutputBytes = 2 * 1024 * 1024;
constexpr int kMaxLine = 512;

struct HostTensor {
  int type = 0;
  int bytes = 0;
  int ndims = 0;
  int dims[4] = {0, 0, 0, 0};
  float scale = 0.0f;
  int32_t zp = 0;
  alignas(32) uint8_t data[kMaxHostOutputBytes];
};

usb_host_edgetpu_instance_t* g_usb_instance = nullptr;
bool g_usb_notified = false;
std::shared_ptr<coralmicro::EdgeTpuContext> g_tpu_context;
tflite::MicroInterpreter* g_interpreter = nullptr;
volatile bool g_tpu_ready = false;
uint8_t* g_tensor_arena = nullptr;

uint8_t* g_slot_arena[kNumTpuSlots] = {nullptr, nullptr, nullptr};
tflite::MicroInterpreter* g_slot_interp[kNumTpuSlots] = {nullptr, nullptr, nullptr};
alignas(32) uint8_t g_slot_model_storage[kNumTpuSlots][kMaxModelBytes];
size_t g_slot_model_size[kNumTpuSlots] = {0, 0, 0};
volatile bool g_slot_ready[kNumTpuSlots] = {false, false, false};

uint8_t g_last_input_tensor[kMaxImageBytes];
uint8_t g_load_image_tensor[kMaxImageBytes];
uint8_t g_draw_bmp[kMaxDrawBmpBytes];

int g_host_fd = -1;
pid_t g_host_pid = -1;
char g_host_sock_path[128];
bool g_host_ready[kNumTpuSlots] = {false, false, false};
HostTensor g_host_input[kNumTpuSlots];
HostTensor g_host_output[kNumTpuSlots][kMaxHostOutputs];
int g_host_num_outputs[kNumTpuSlots] = {0, 0, 0};
alignas(32) uint8_t g_host_input_storage[kNumTpuSlots][kMaxImageBytes];

bool ensure_tpu_context();

bool using_host_pycoral() {
  const char* v = getenv("SENTAI_TPU_BACKEND");
  return v && (!strcmp(v, "pycoral") || !strcmp(v, "host") ||
               !strcmp(v, "host_pycoral"));
}

const HostTensor* host_out(int slot, int idx) {
  if (slot < 0 || slot >= kNumTpuSlots) return nullptr;
  if (!g_host_ready[slot] || idx < 0 || idx >= g_host_num_outputs[slot]) {
    return nullptr;
  }
  return &g_host_output[slot][idx];
}

bool write_all(int fd, const void* data, size_t n) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  while (n > 0) {
    ssize_t w = write(fd, p, n);
    if (w < 0 && errno == EINTR) continue;
    if (w <= 0) return false;
    p += w;
    n -= (size_t)w;
  }
  return true;
}

bool read_exact_fd(int fd, void* data, size_t n) {
  uint8_t* p = static_cast<uint8_t*>(data);
  while (n > 0) {
    ssize_t r = read(fd, p, n);
    if (r < 0 && errno == EINTR) continue;
    if (r <= 0) return false;
    p += r;
    n -= (size_t)r;
  }
  return true;
}

bool read_line_fd(int fd, char* out, size_t outsz) {
  if (!out || outsz == 0) return false;
  size_t n = 0;
  while (n + 1 < outsz) {
    char c = 0;
    ssize_t r = read(fd, &c, 1);
    if (r < 0 && errno == EINTR) continue;
    if (r <= 0) return false;
    if (c == '\n') break;
    out[n++] = c;
  }
  out[n] = '\0';
  return true;
}

bool parse_out_line(const char* line, HostTensor* t) {
  if (!line || !t) return false;
  int idx = 0;
  int type = 0;
  int bytes = 0;
  int ndims = 0;
  int dims[4] = {0, 0, 0, 0};
  float scale = 0.0f;
  int zp = 0;
  int n = sscanf(line, "OUT %d %d %d %d %d %d %d %d %f %d",
                 &idx, &type, &bytes, &ndims,
                 &dims[0], &dims[1], &dims[2], &dims[3], &scale, &zp);
  if (n != 10 || bytes < 0 || bytes > kMaxHostOutputBytes ||
      ndims < 0 || ndims > 4) {
    return false;
  }
  t->type = type;
  t->bytes = bytes;
  t->ndims = ndims;
  for (int i = 0; i < 4; ++i) t->dims[i] = dims[i];
  t->scale = scale;
  t->zp = zp;
  return true;
}

void host_signal_input_done(void) {
  auto sem = __atomic_exchange_n(&g_sentai_tpu_input_done_sema, nullptr,
                                 __ATOMIC_ACQ_REL);
  if (sem) xSemaphoreGive(sem);
}

bool ensure_host_pycoral() {
  if (g_host_fd >= 0) return true;
  snprintf(g_host_sock_path, sizeof(g_host_sock_path),
           "/tmp/sentai_tpu_pycoral_%ld.sock", (long)getpid());
  unlink(g_host_sock_path);

  const char* py = getenv("SENTAI_TPU_PYTHON");
  if (!py || !py[0]) py = "venv-coral/bin/python";
  const char* helper = getenv("SENTAI_TPU_PYCORAL_HELPER");
  if (!helper || !helper[0]) helper = "sim/scripts/sentai_tpu_pycoral_server.py";

  pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    execl(py, py, helper, g_host_sock_path, (char*)nullptr);
    _exit(127);
  }
  g_host_pid = pid;

  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return false;
  sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, g_host_sock_path, sizeof(addr.sun_path) - 1);
  int last_errno = 0;
  for (int i = 0; i < 200; ++i) {
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
      g_host_fd = fd;
      sentai_logf("tpu-pycoral", "connected helper pid=%ld sock=%s",
                  (long)g_host_pid, g_host_sock_path);
      return true;
    }
    last_errno = errno;
    usleep(50 * 1000);
  }
  sentai_logf("tpu-pycoral", "connect failed errno=%d path=%s",
              last_errno, g_host_sock_path);
  close(fd);
  return false;
}

int host_load_model_slot(int slot, const char* path) {
  if (slot < 0 || slot >= kNumTpuSlots || !path) return -3;
  if (!ensure_host_pycoral()) return -1;
  char host_path[1024];
  if (sim_fs_resolve(path, host_path, sizeof(host_path)) != 0) return -2;
  char cmd[1200];
  snprintf(cmd, sizeof(cmd), "LOAD %d %s\n", slot, host_path);
  if (!write_all(g_host_fd, cmd, strlen(cmd))) return -4;
  char line[kMaxLine];
  if (!read_line_fd(g_host_fd, line, sizeof(line))) return -5;
  sentai_logf("tpu-pycoral", "LOAD reply: %s", line);
  HostTensor& in = g_host_input[slot];
  int rslot = -1;
  int nouts = 0;
  int n = sscanf(line, "OK LOAD %d %d %d %d %d %d %d %d %f %d %d",
                 &rslot, &in.type, &in.bytes, &in.ndims,
                 &in.dims[0], &in.dims[1], &in.dims[2], &in.dims[3],
                 &in.scale, &in.zp, &nouts);
  if (n != 11 || rslot != slot || in.bytes <= 0 ||
      in.bytes > kMaxImageBytes || in.ndims < 1 || in.ndims > 4 ||
      nouts < 0 || nouts > kMaxHostOutputs) {
    sentai_logf("tpu-pycoral", "bad LOAD reply: %s", line);
    return -6;
  }
  g_host_num_outputs[slot] = nouts;
  for (int i = 0; i < nouts; ++i) {
    if (!read_line_fd(g_host_fd, line, sizeof(line)) ||
        !parse_out_line(line, &g_host_output[slot][i])) {
      sentai_logf("tpu-pycoral", "bad OUT reply[%d]: %s", i, line);
      return -7;
    }
    sentai_logf("tpu-pycoral", "OUT[%d]: bytes=%d shape=%dx%dx%dx%d",
                i, g_host_output[slot][i].bytes,
                g_host_output[slot][i].dims[0],
                g_host_output[slot][i].dims[1],
                g_host_output[slot][i].dims[2],
                g_host_output[slot][i].dims[3]);
  }
  memset(g_host_input_storage[slot], 0, (size_t)in.bytes);
  g_host_ready[slot] = true;
  if (slot == 0) g_tpu_ready = true;
  sentai_logf("tpu-pycoral", "loaded slot=%d input=%dx%dx%d outputs=%d",
              slot, in.dims[2], in.dims[1], in.dims[3], nouts);
  return 0;
}

int host_invoke_slot(int slot, uint8_t* input_buf) {
  if (slot < 0 || slot >= kNumTpuSlots || !g_host_ready[slot]) return -1;
  HostTensor& in = g_host_input[slot];
  uint8_t* src = input_buf ? input_buf : g_host_input_storage[slot];
  if (!src || in.bytes <= 0 || in.bytes > kMaxImageBytes) return -2;
  if (slot == 0) memcpy(g_last_input_tensor, src, (size_t)in.bytes);
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "INVOKE %d %d\n", slot, in.bytes);
  if (!write_all(g_host_fd, cmd, strlen(cmd)) ||
      !write_all(g_host_fd, src, (size_t)in.bytes)) {
    return -3;
  }
  host_signal_input_done();
  char line[kMaxLine];
  if (!read_line_fd(g_host_fd, line, sizeof(line))) return -4;
  int ms = 0;
  int nouts = 0;
  int total = 0;
  int n = sscanf(line, "OK INVOKE %d %d %d", &ms, &nouts, &total);
  if (n != 3 || nouts < 0 || nouts > kMaxHostOutputs || total < 0) {
    sentai_logf("tpu-pycoral", "bad INVOKE reply: %s", line);
    return -5;
  }
  g_host_num_outputs[slot] = nouts;
  for (int i = 0; i < nouts; ++i) {
    if (!read_line_fd(g_host_fd, line, sizeof(line)) ||
        !parse_out_line(line, &g_host_output[slot][i])) {
      return -6;
    }
  }
  for (int i = 0; i < nouts; ++i) {
    HostTensor& out = g_host_output[slot][i];
    if (out.bytes < 0 || out.bytes > kMaxHostOutputBytes) return -7;
    if (!read_exact_fd(g_host_fd, out.data, (size_t)out.bytes)) return -8;
  }
  return ms;
}

void cleanup_slot(int slot) {
  if (slot < 0 || slot >= kNumTpuSlots) return;
  g_slot_ready[slot] = false;
  if (g_slot_interp[slot] && g_slot_interp[slot] != g_interpreter) {
    delete g_slot_interp[slot];
  }
  g_slot_interp[slot] = nullptr;
  g_slot_model_size[slot] = 0;
}

void cleanup_default_model() {
  g_tpu_ready = false;
  cleanup_slot(0);
  if (g_interpreter) {
    delete g_interpreter;
    g_interpreter = nullptr;
  }
  g_slot_model_size[0] = 0;
}

bool read_model_from_fs(const char* path, uint8_t* out,
                        size_t capacity, size_t* out_size) {
  if (!path || !out || !out_size) return false;
  *out_size = 0;
  char host_path[1024];
  if (sim_fs_resolve(path, host_path, sizeof(host_path)) != 0) return false;
  int fd = open(host_path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  struct stat st;
  if (fstat(fd, &st) != 0 || st.st_size <= 0 ||
      st.st_size > (off_t)kMaxModelBytes) {
    close(fd);
    return false;
  }
  int size = (int)st.st_size;
  sentai_logf("tpu-posix", "model size=%d", size);
  if (size <= 0 || size > kMaxModelBytes) {
    close(fd);
    return false;
  }
  if ((size_t)size > capacity) {
    close(fd);
    return false;
  }
  sentai_logf("tpu-posix", "reading model bytes into static storage");
  int got = 0;
  while (got < size) {
    ssize_t n = read(fd, out + got, (size_t)(size - got));
    if (n <= 0) {
      close(fd);
      return false;
    }
    got += (int)n;
  }
  close(fd);
  sentai_logf("tpu-posix", "model read got=%d", got);
  if (got != size) return false;
  *out_size = (size_t)got;
  return true;
}

tflite::MicroMutableOpResolver<8>& tpu_resolver() {
  static tflite::MicroMutableOpResolver<8> resolver;
  static bool init = false;
  if (!init) {
    resolver.AddCustom(coralmicro::kCustomOp, coralmicro::RegisterCustomOp());
    resolver.AddTranspose();
    resolver.AddReshape();
    resolver.AddConcatenation();
    resolver.AddLogistic();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddDetectionPostprocess();
    init = true;
  }
  return resolver;
}

const char* type_name(TfLiteType t) {
  switch (t) {
    case kTfLiteFloat32: return "float32";
    case kTfLiteInt32: return "int32";
    case kTfLiteUInt8: return "uint8";
    case kTfLiteInt8: return "int8";
    case kTfLiteInt16: return "int16";
    default: return "unknown";
  }
}

void print_tensor(const char* label, int idx, const TfLiteTensor* t) {
  if (!t || !t->dims) return;
  char dims[96];
  int off = 0;
  for (int i = 0; i < t->dims->size; ++i) {
    int n = snprintf(dims + off, sizeof(dims) - (size_t)off,
                     "%s%d", i ? "," : "", t->dims->data[i]);
    if (n < 0) break;
    off += n;
    if (off >= (int)sizeof(dims)) {
      off = (int)sizeof(dims) - 1;
      break;
    }
  }
  dims[off] = '\0';
  sentai_logf("tpu-posix", "%s[%d]: %s[%s] %ld B scale=%.8f zp=%d",
              label, idx, type_name(t->type), dims, (long)t->bytes,
              (double)t->params.scale, (int)t->params.zero_point);
}

int load_model_into_slot_impl(int slot, const char* path) {
  if (slot < 0 || slot >= kNumTpuSlots || !path) return -3;
  sentai_logf("tpu-posix", "load slot=%d path=%s", slot, path);
  if (!ensure_tpu_context()) return -1;

  if (slot == 0) cleanup_default_model();
  else cleanup_slot(slot);

  sentai_logf("tpu-posix", "reading model from SentAI FS");
  uint8_t* model_data = g_slot_model_storage[slot];
  size_t model_size = 0;
  if (!read_model_from_fs(path, model_data, kMaxModelBytes, &model_size)) {
    sentai_logf("tpu-posix", "failed to read model from SIM FS: %s", path);
    return -2;
  }
  g_slot_model_size[slot] = model_size;
  sentai_logf("tpu-posix", "model loaded from FS: %s (%lu bytes)",
              path, (unsigned long)model_size);

  uint8_t* arena = nullptr;
  int arena_size = (slot == 0) ? kTensorArenaSize : kSlotArenaSize;
  if (slot == 0) {
    if (!g_tensor_arena) {
      void* p = nullptr;
      if (posix_memalign(&p, 32, kTensorArenaSize) != 0 || !p) {
        return -5;
      }
      g_tensor_arena = static_cast<uint8_t*>(p);
    }
    arena = g_tensor_arena;
  } else {
    if (!g_slot_arena[slot]) {
      void* p = nullptr;
      if (posix_memalign(&p, 32, kSlotArenaSize) != 0 || !p) {
        return -5;
      }
      g_slot_arena[slot] = static_cast<uint8_t*>(p);
    }
    arena = g_slot_arena[slot];
  }

  static tflite::MicroErrorReporter error_reporter;
  auto* interp = new(std::nothrow) tflite::MicroInterpreter(
      tflite::GetModel(model_data), tpu_resolver(), arena, arena_size,
      &error_reporter);
  if (!interp) {
    return -7;
  }
  if (interp->AllocateTensors() != kTfLiteOk) {
    sentai_logf("tpu-posix", "AllocateTensors failed for slot %d", slot);
    delete interp;
    return -4;
  }
  if (interp->inputs().size() != 1) {
    sentai_logf("tpu-posix", "model must have exactly one input tensor");
    delete interp;
    return -8;
  }

  print_tensor("TPU Input", slot, interp->input_tensor(0));
  int out_count = (int)interp->outputs().size();
  for (int i = 0; i < out_count; ++i) {
    print_tensor("TPU Out", i, interp->output_tensor(i));
  }
  sentai_logf("tpu-posix", "arena used: %lu / %d KB",
              (unsigned long)(interp->arena_used_bytes() / 1024),
              arena_size / 1024);

  if (slot == 0) {
    sentai_logf("tpu-posix", "publish slot0 begin");
    g_interpreter = interp;
    g_tpu_ready = true;
    g_slot_interp[0] = g_interpreter;
    g_slot_ready[0] = true;
    sentai_logf("tpu-posix", "publish slot0 done");
  } else {
    sentai_logf("tpu-posix", "publish slot%d begin", slot);
    g_slot_interp[slot] = interp;
    g_slot_ready[slot] = true;
    sentai_logf("tpu-posix", "publish slot%d done", slot);
  }
  return 0;
}

tflite::MicroInterpreter* interp_for_slot(int slot) {
  if (slot < 0 || slot >= kNumTpuSlots) return nullptr;
  if (!g_slot_ready[slot]) return nullptr;
  return g_slot_interp[slot];
}

int invoke_slot_with_input_ptr_impl(int slot, uint8_t* input_buf) {
  auto* interp = interp_for_slot(slot);
  if (!interp || !input_buf) return -1;
  auto* input = interp->input_tensor(0);
  if (!input) return -4;
  uint8_t* saved = input->data.uint8;
  input->data.uint8 = input_buf;
  TickType_t t0 = xTaskGetTickCount();
  uint8_t prev_trace = g_sentai_tpu_trace;
  if (getenv("SENTAI_TPU_TRACE_INVOKE")) g_sentai_tpu_trace = 1;
  TfLiteStatus rc = interp->Invoke();
  g_sentai_tpu_trace = prev_trace;
  TickType_t t1 = xTaskGetTickCount();
  input->data.uint8 = saved;
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

int invoke_slot_impl(int slot) {
  auto* interp = interp_for_slot(slot);
  if (!interp) return -1;
  TickType_t t0 = xTaskGetTickCount();
  uint8_t prev_trace = g_sentai_tpu_trace;
  if (getenv("SENTAI_TPU_TRACE_INVOKE")) g_sentai_tpu_trace = 1;
  TfLiteStatus rc = interp->Invoke();
  g_sentai_tpu_trace = prev_trace;
  TickType_t t1 = xTaskGetTickCount();
  if (rc != kTfLiteOk) return -2;
  return (int)((t1 - t0) * portTICK_PERIOD_MS);
}

bool ensure_usb_instance() {
  if (g_usb_instance) {
    if (!g_usb_notified) {
      coralmicro::EdgeTpuManager::GetSingleton()->NotifyConnected(g_usb_instance);
      g_usb_notified = true;
    }
    return true;
  }
  sentai_logf("tpu-posix", "opening Coral USB via POSIX/libusb backend");
  usb_host_edgetpu_instance_t* inst = nullptr;
  usb_status_t status = USB_HostEdgeTpuOpenPosix(&inst);
  if (status != kStatus_USB_Success || !inst) {
    sentai_logf("tpu-posix", "USB_HostEdgeTpuOpenPosix failed status=%u",
                (unsigned)status);
    return false;
  }
  g_usb_instance = inst;
  coralmicro::EdgeTpuManager::GetSingleton()->NotifyConnected(g_usb_instance);
  g_usb_notified = true;
  return true;
}

bool ensure_tpu_context() {
  if (g_tpu_context) return true;
  if (!ensure_usb_instance()) return false;
  sentai_logf("tpu-posix", "opening EdgeTpuManager context");
  g_tpu_context = coralmicro::EdgeTpuManager::GetSingleton()->OpenDevice(
      coralmicro::PerformanceMode::kHigh);
  if (!g_tpu_context) {
    sentai_logf("tpu-posix", "EdgeTpuManager::OpenDevice failed");
    return false;
  }
  sentai_logf("tpu-posix", "EdgeTPU ready via direct libusb backend");
  return true;
}

void put_le16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xffu);
  p[1] = (uint8_t)(v >> 8);
}

void put_le32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xffu);
  p[1] = (uint8_t)((v >> 8) & 0xffu);
  p[2] = (uint8_t)((v >> 16) & 0xffu);
  p[3] = (uint8_t)((v >> 24) & 0xffu);
}

void draw_rect_rgb(uint8_t* img, int w, int h,
                   int x1, int y1, int x2, int y2) {
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;
  if (x2 >= w) x2 = w - 1;
  if (y2 >= h) y2 = h - 1;
  if (x2 <= x1 || y2 <= y1) return;
  for (int t = 0; t < 3; ++t) {
    int xx1 = x1 + t, xx2 = x2 - t, yy1 = y1 + t, yy2 = y2 - t;
    for (int x = xx1; x <= xx2; ++x) {
      uint8_t* a = img + (yy1 * w + x) * 3;
      uint8_t* b = img + (yy2 * w + x) * 3;
      a[0] = 255; a[1] = 32; a[2] = 32;
      b[0] = 255; b[1] = 32; b[2] = 32;
    }
    for (int y = yy1; y <= yy2; ++y) {
      uint8_t* a = img + (y * w + xx1) * 3;
      uint8_t* b = img + (y * w + xx2) * 3;
      a[0] = 255; a[1] = 32; a[2] = 32;
      b[0] = 255; b[1] = 32; b[2] = 32;
    }
  }
}

}  // namespace

extern "C" int sentai_tpu_load_model(const char* path) {
  if (using_host_pycoral()) return host_load_model_slot(0, path);
  return load_model_into_slot_impl(0, path);
}

extern "C" int sentai_load_model(const char* path) {
  return sentai_tpu_load_model(path);
}

extern "C" int sentai_tpu_load_model_slot(int slot, const char* path) {
  if (using_host_pycoral()) return host_load_model_slot(slot, path);
  return load_model_into_slot_impl(slot, path);
}

extern "C" int sentai_load_model_slot(int slot, const char* path) {
  return sentai_tpu_load_model_slot(slot, path);
}

extern "C" int sentai_tpu_invoke_internal(void) {
  if (using_host_pycoral()) return host_invoke_slot(0, nullptr);
  return invoke_slot_impl(0);
}

extern "C" int sentai_tpu_invoke(void) {
  if (using_host_pycoral()) return host_invoke_slot(0, nullptr);
  return invoke_slot_impl(0);
}

extern "C" int sentai_tpu_invoke_with_input(uint8_t* input_buf) {
  if (using_host_pycoral()) return host_invoke_slot(0, input_buf);
  if (!input_buf || !g_interpreter) return -1;
  auto* input = g_interpreter->input_tensor(0);
  if (!input) return -4;
  if ((int)input->bytes <= kMaxImageBytes) {
    memcpy(g_last_input_tensor, input_buf, input->bytes);
  }
  int rc = invoke_slot_with_input_ptr_impl(0, input_buf);
  return rc;
}

extern "C" int sentai_tpu_is_ready(void) {
  if (using_host_pycoral()) return g_host_ready[0] ? 1 : 0;
  return (g_tpu_ready && g_interpreter) ? 1 : 0;
}

extern "C" int sentai_tpu_num_outputs(void) {
  if (using_host_pycoral()) return g_host_ready[0] ? g_host_num_outputs[0] : 0;
  return g_interpreter ? (int)g_interpreter->outputs().size() : 0;
}

extern "C" int sentai_tpu_get_output_size(int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    return t ? t->bytes : 0;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return 0;
  }
  return (int)g_interpreter->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tpu_get_output_data(int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    return t ? t->data : nullptr;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return nullptr;
  }
  return g_interpreter->output_tensor(idx)->data.data;
}

extern "C" int sentai_tpu_get_output_num_dims(int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    return t ? t->ndims : 0;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return 0;
  }
  return g_interpreter->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tpu_get_output_dim(int idx, int dim) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    return (t && dim >= 0 && dim < t->ndims && dim < 4) ? t->dims[dim] : 0;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return 0;
  }
  auto* t = g_interpreter->output_tensor(idx);
  if (!t || !t->dims || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tpu_get_output_type(int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    return t ? t->type : -1;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return -1;
  }
  return (int)g_interpreter->output_tensor(idx)->type;
}

extern "C" int sentai_tpu_input_quant(float* scale, int32_t* zero_point) {
  if (using_host_pycoral()) {
    if (!g_host_ready[0]) return -1;
    if (scale) *scale = g_host_input[0].scale;
    if (zero_point) *zero_point = g_host_input[0].zp;
    return 0;
  }
  if (!g_interpreter) return -1;
  auto* input = g_interpreter->input_tensor(0);
  if (!input) return -1;
  if (scale) *scale = input->params.scale;
  if (zero_point) *zero_point = input->params.zero_point;
  return 0;
}

extern "C" int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(0, idx);
    if (!t) return -1;
    if (scale) *scale = t->scale;
    if (zero_point) *zero_point = t->zp;
    return 0;
  }
  if (!g_interpreter || idx < 0 || idx >= (int)g_interpreter->outputs().size()) {
    return -1;
  }
  auto* t = g_interpreter->output_tensor(idx);
  if (!t) return -1;
  if (scale) *scale = t->params.scale;
  if (zero_point) *zero_point = t->params.zero_point;
  return 0;
}

extern "C" int sentai_tpu_input_type(void) {
  if (using_host_pycoral()) return g_host_ready[0] ? g_host_input[0].type : -1;
  if (!g_interpreter) return -1;
  auto* input = g_interpreter->input_tensor(0);
  return input ? (int)input->type : -1;
}

extern "C" int sentai_tpu_slot_count(void) {
  return kNumTpuSlots;
}

extern "C" int sentai_tpu_slot_ready(int slot) {
  if (using_host_pycoral()) {
    return (slot >= 0 && slot < kNumTpuSlots && g_host_ready[slot]) ? 1 : 0;
  }
  return interp_for_slot(slot) ? 1 : 0;
}

extern "C" int sentai_tpu_invoke_slot(int slot) {
  if (using_host_pycoral()) return host_invoke_slot(slot, nullptr);
  return invoke_slot_impl(slot);
}

extern "C" int sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf) {
  if (using_host_pycoral()) return host_invoke_slot(slot, buf);
  return invoke_slot_with_input_ptr_impl(slot, buf);
}

extern "C" int sentai_tpu_num_outputs_slot(int slot) {
  if (using_host_pycoral()) {
    return (slot >= 0 && slot < kNumTpuSlots && g_host_ready[slot])
               ? g_host_num_outputs[slot]
               : 0;
  }
  auto* interp = interp_for_slot(slot);
  return interp ? (int)interp->outputs().size() : 0;
}

extern "C" int sentai_tpu_get_output_size_slot(int slot, int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    return t ? t->bytes : 0;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  return (int)interp->output_tensor(idx)->bytes;
}

extern "C" const void* sentai_tpu_get_output_data_slot(int slot, int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    return t ? t->data : nullptr;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return nullptr;
  return interp->output_tensor(idx)->data.data;
}

extern "C" int sentai_tpu_get_output_num_dims_slot(int slot, int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    return t ? t->ndims : 0;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  return interp->output_tensor(idx)->dims->size;
}

extern "C" int sentai_tpu_get_output_dim_slot(int slot, int idx, int dim) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    return (t && dim >= 0 && dim < t->ndims && dim < 4) ? t->dims[dim] : 0;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return 0;
  auto* t = interp->output_tensor(idx);
  if (!t || dim < 0 || dim >= t->dims->size) return 0;
  return t->dims->data[dim];
}

extern "C" int sentai_tpu_get_output_type_slot(int slot, int idx) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    return t ? t->type : -1;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return -1;
  return (int)interp->output_tensor(idx)->type;
}

extern "C" int sentai_tpu_output_quant_slot(int slot, int idx,
                                             float* scale, int32_t* zero_point) {
  if (using_host_pycoral()) {
    const HostTensor* t = host_out(slot, idx);
    if (!t) return -1;
    if (scale) *scale = t->scale;
    if (zero_point) *zero_point = t->zp;
    return 0;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || idx < 0 || idx >= (int)interp->outputs().size()) return -1;
  auto* t = interp->output_tensor(idx);
  if (!t) return -1;
  if (scale) *scale = t->params.scale;
  if (zero_point) *zero_point = t->params.zero_point;
  return 0;
}

extern "C" int sentai_tpu_set_input_slot(int slot, const uint8_t* data, int bytes) {
  if (using_host_pycoral()) {
    if (slot < 0 || slot >= kNumTpuSlots || !g_host_ready[slot] ||
        !data || bytes <= 0 || bytes != g_host_input[slot].bytes ||
        bytes > kMaxImageBytes) {
      return -1;
    }
    memcpy(g_host_input_storage[slot], data, (size_t)bytes);
    if (slot == 0) memcpy(g_last_input_tensor, data, (size_t)bytes);
    return 0;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp || !data || bytes <= 0) return -1;
  auto* input = interp->input_tensor(0);
  if (!input) return -4;
  if (bytes != (int)input->bytes) return -5;
  memcpy(input->data.uint8, data, (size_t)bytes);
  if (slot == 0 && bytes <= kMaxImageBytes) {
    memcpy(g_last_input_tensor, data, (size_t)bytes);
  }
  return 0;
}

extern "C" uint32_t sentai_tpu_output_hash_slot(int slot) {
  if (using_host_pycoral()) {
    if (slot < 0 || slot >= kNumTpuSlots || !g_host_ready[slot]) return 0;
    uint32_t h = 0x811C9DC5u;
    for (int oi = 0; oi < g_host_num_outputs[slot]; ++oi) {
      const HostTensor& t = g_host_output[slot][oi];
      for (int i = 0; i < t.bytes; ++i) {
        h ^= t.data[i];
        h *= 0x01000193u;
      }
    }
    return h;
  }
  auto* interp = interp_for_slot(slot);
  if (!interp) return 0;
  uint32_t h = 0x811C9DC5u;
  for (int oi = 0; oi < (int)interp->outputs().size(); ++oi) {
    auto* t = interp->output_tensor(oi);
    if (!t || !t->data.data) continue;
    const uint8_t* p = (const uint8_t*)t->data.data;
    for (int i = 0; i < (int)t->bytes; ++i) {
      h ^= p[i];
      h *= 0x01000193u;
    }
  }
  return h;
}

extern "C" int sentai_get_tensor_info(int* w, int* h, int* ch,
                                      uint8_t** buf, int* type, int* zp) {
  if (using_host_pycoral()) {
    if (!g_host_ready[0] || g_host_input[0].ndims < 4) return -1;
    if (h) *h = g_host_input[0].dims[1];
    if (w) *w = g_host_input[0].dims[2];
    if (ch) *ch = g_host_input[0].dims[3];
    if (buf) *buf = g_host_input_storage[0];
    if (type) *type = g_host_input[0].type;
    if (zp) *zp = g_host_input[0].zp;
    return 0;
  }
  if (!g_interpreter) return -1;
  auto* input = g_interpreter->input_tensor(0);
  if (!input || !input->dims || input->dims->size < 4) return -2;
  if (h) *h = input->dims->data[1];
  if (w) *w = input->dims->data[2];
  if (ch) *ch = input->dims->data[3];
  if (buf) *buf = input->data.uint8;
  if (type) *type = (int)input->type;
  if (zp) *zp = input->params.zero_point;
  return 0;
}

extern "C" int sentai_load_image(const char* path) {
  if (using_host_pycoral()) {
    if (!path || !g_host_ready[0] || g_host_input[0].ndims < 4) return -1;
    int iw = g_host_input[0].dims[2];
    int ih = g_host_input[0].dims[1];
    int ic = g_host_input[0].dims[3];
    if (iw <= 0 || ih <= 0 || ic != 3 || iw * ih * ic > kMaxImageBytes) return -3;
    (void)sentai_prep_slot_enable(SENTAI_PREP_SLOT_TPU_RGB);
    int rc = sentai_virtual_camera_select(path);
    if (rc != 0) {
      (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
      return -10 + rc;
    }
    (void)sentai_camera_backend_publish_prep_once();
    const uint8_t* prep = nullptr;
    int sw = 0;
    int sh = 0;
    uint32_t seq = 0;
    rc = sentai_prep_slot_get(SENTAI_PREP_SLOT_TPU_RGB, &prep, &sw, &sh, &seq);
    if (rc != 0 || !prep) {
      (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
      return -20;
    }
    if (sw != iw || sh != ih) {
      (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
      return -21;
    }
    int bytes = iw * ih * ic;
    memcpy(g_load_image_tensor, prep, (size_t)bytes);
    if (g_host_input[0].type == kTfLiteInt8) {
      sentai_quant_uint8_to_int8(g_load_image_tensor, bytes,
                                 g_host_input[0].zp);
    }
    rc = sentai_tpu_set_input_slot(0, g_load_image_tensor, bytes);
    (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
    (void)seq;
    return rc;
  }
  if (!path || !g_interpreter) return -1;
  sentai_logf("tpu-posix", "load_image begin path=%s", path);
  auto* input = g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -2;
  int iw = input->dims->data[2];
  int ih = input->dims->data[1];
  int ic = input->dims->data[3];
  if (iw <= 0 || ih <= 0 || ic != 3 || iw * ih * ic > kMaxImageBytes) return -3;
  (void)sentai_prep_slot_enable(SENTAI_PREP_SLOT_TPU_RGB);
  int rc = sentai_virtual_camera_select(path);
  if (rc != 0) {
    (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
    return -10 + rc;
  }
  (void)sentai_camera_backend_publish_prep_once();
  sentai_logf("tpu-posix", "load_image selected");
  const uint8_t* prep = nullptr;
  int sw = 0;
  int sh = 0;
  uint32_t seq = 0;
  rc = sentai_prep_slot_get(SENTAI_PREP_SLOT_TPU_RGB, &prep, &sw, &sh, &seq);
  if (rc != 0 || !prep) {
    (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
    return -20;
  }
  if (sw != iw || sh != ih) {
    (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
    return -21;
  }
  const int bytes = iw * ih * ic;
  memcpy(g_load_image_tensor, prep, (size_t)bytes);
  sentai_logf("tpu-posix", "load_image prep slot %dx%d seq=%lu",
              sw, sh, (unsigned long)seq);
  if (input->type == kTfLiteInt8) {
    sentai_quant_uint8_to_int8(g_load_image_tensor, bytes,
                               input->params.zero_point);
  }
  rc = sentai_tpu_set_input_slot(0, g_load_image_tensor, bytes);
  (void)sentai_prep_slot_disable(SENTAI_PREP_SLOT_TPU_RGB);
  sentai_logf("tpu-posix", "load_image set_input rc=%d", rc);
  return rc;
}

extern "C" int sentai_save_output(const char* path) {
  if (using_host_pycoral()) {
    if (!path || !g_host_ready[0]) return -1;
    std::string csv;
    char tmp[96];
    for (int oi = 0; oi < g_host_num_outputs[0]; ++oi) {
      const HostTensor& t = g_host_output[0][oi];
      snprintf(tmp, sizeof(tmp), "# output %d, type=%d, dims=", oi, t.type);
      csv += tmp;
      for (int d = 0; d < t.ndims && d < 4; ++d) {
        if (d) csv += 'x';
        snprintf(tmp, sizeof(tmp), "%d", t.dims[d]);
        csv += tmp;
      }
      csv += '\n';
      for (int i = 0; i < t.bytes; ++i) {
        if (i) csv += ',';
        snprintf(tmp, sizeof(tmp), "%u", (unsigned)t.data[i]);
        csv += tmp;
      }
      csv += '\n';
    }
    return sentai_fs_write(path, (const uint8_t*)csv.data(), (int)csv.size()) ==
                   (int)csv.size()
               ? 0
               : -2;
  }
  if (!path || !g_interpreter) return -1;
  std::string csv;
  char tmp[64];
  int n = (int)g_interpreter->outputs().size();
  for (int oi = 0; oi < n; ++oi) {
    auto* t = g_interpreter->output_tensor(oi);
    if (!t) continue;
    snprintf(tmp, sizeof(tmp), "# output %d, type=%d, dims=", oi, (int)t->type);
    csv += tmp;
    int total = 1;
    for (int d = 0; d < t->dims->size; ++d) {
      if (d) csv += 'x';
      snprintf(tmp, sizeof(tmp), "%d", t->dims->data[d]);
      csv += tmp;
      total *= t->dims->data[d];
    }
    csv += '\n';
    const uint8_t* bytes = (const uint8_t*)t->data.data;
    int byte_count = (int)t->bytes;
    for (int i = 0; i < byte_count; ++i) {
      if (i) csv += ',';
      snprintf(tmp, sizeof(tmp), "%u", (unsigned)bytes[i]);
      csv += tmp;
    }
    csv += '\n';
    (void)total;
  }
  return sentai_fs_write(path, (const uint8_t*)csv.data(), (int)csv.size()) ==
                 (int)csv.size()
             ? 0
             : -2;
}

extern "C" int sentai_cam_to_tensor(void) {
  return sentai_load_image("/images/cat_640x480.bmp");
}

extern "C" int sentai_cam_to_tensor_ex(const char* save_path, int quality) {
  (void)save_path;
  (void)quality;
  return sentai_cam_to_tensor();
}

extern "C" void sentai_usb_edgetpu_dump_eps(void) {
  if (!g_usb_instance) {
    sentai_logf("tpu-posix", "EdgeTPU not opened yet");
    return;
  }
  sentai_logf("tpu-posix",
              "iface=%u bulk_out=%02x,%02x,%02x,%02x bulk_in=%02x,%02x,%02x intr=%02x",
              (unsigned)g_usb_instance->interface_number,
              (unsigned)g_usb_instance->bulk_out_ep[0],
              (unsigned)g_usb_instance->bulk_out_ep[1],
              (unsigned)g_usb_instance->bulk_out_ep[2],
              (unsigned)g_usb_instance->bulk_out_ep[3],
              (unsigned)g_usb_instance->bulk_in_ep[0],
              (unsigned)g_usb_instance->bulk_in_ep[1],
              (unsigned)g_usb_instance->bulk_in_ep[2],
              (unsigned)g_usb_instance->interrupt_in_ep);
}

extern "C" void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp) {
  if (!buf || count <= 0) return;
  int8_t* dst = reinterpret_cast<int8_t*>(buf);
  for (int i = 0; i < count; ++i) {
    int v = (int)buf[i] + zp;
    if (v < -128) v = -128;
    if (v > 127) v = 127;
    dst[i] = (int8_t)v;
  }
}

extern "C" int sentai_tpu_detect(int conf_permil, int iou_permil,
                                 int max_dets, int16_t* out_buf,
                                 int* out_count) {
  (void)iou_permil;
  if (out_count) *out_count = 0;
  if (using_host_pycoral()) {
    if (!out_buf || !out_count || !g_host_ready[0] ||
        g_host_num_outputs[0] < 4) {
      return -1;
    }
    const float* scores = nullptr;
    const float* boxes = nullptr;
    const float* num = nullptr;
    const float* classes = nullptr;
    int vector_candidates[2] = {-1, -1};
    int vector_count = 0;
    for (int oi = 0; oi < g_host_num_outputs[0]; ++oi) {
      const HostTensor& t = g_host_output[0][oi];
      if (t.type != kTfLiteFloat32) continue;
      if (t.ndims == 2 && t.dims[0] == 1) {
        if (vector_count < 2) vector_candidates[vector_count++] = oi;
      } else if (t.ndims == 3 && t.dims[0] == 1 && t.dims[2] == 4) {
        boxes = reinterpret_cast<const float*>(t.data);
      } else if (t.ndims == 1 && t.dims[0] == 1) {
        num = reinterpret_cast<const float*>(t.data);
      }
    }
    if (vector_count == 2) {
      const HostTensor& a = g_host_output[0][vector_candidates[0]];
      const HostTensor& b = g_host_output[0][vector_candidates[1]];
      const float* av = reinterpret_cast<const float*>(a.data);
      const float* bv = reinterpret_cast<const float*>(b.data);
      int n = a.dims[1];
      if (b.dims[1] < n) n = b.dims[1];
      float amax = 0.0f, bmax = 0.0f;
      for (int i = 0; i < n && i < 20; ++i) {
        if (av[i] > amax) amax = av[i];
        if (bv[i] > bmax) bmax = bv[i];
      }
      if (amax > 2.0f && bmax <= 2.0f) {
        classes = av;
        scores = bv;
      } else if (bmax > 2.0f && amax <= 2.0f) {
        scores = av;
        classes = bv;
      } else {
        scores = av;
        classes = bv;
      }
    }
    if (!scores || !boxes || !num || !classes) return -2;
    int input_w = g_host_input[0].dims[2];
    int input_h = g_host_input[0].dims[1];
    int n_avail = (int)num[0];
    if (n_avail < 0) n_avail = 0;
    if (n_avail > 100) n_avail = 100;
    if (max_dets < 1) max_dets = 1;
    if (max_dets > 50) max_dets = 50;
    int kept = 0;
    for (int i = 0; i < n_avail && kept < max_dets; ++i) {
      int permil = (int)(scores[i] * 1000.0f + 0.5f);
      if (permil < conf_permil) continue;
      int16_t* d = out_buf + kept * 6;
      d[0] = (int16_t)(boxes[i * 4 + 1] * (float)input_w);
      d[1] = (int16_t)(boxes[i * 4 + 0] * (float)input_h);
      d[2] = (int16_t)(boxes[i * 4 + 3] * (float)input_w);
      d[3] = (int16_t)(boxes[i * 4 + 2] * (float)input_h);
      d[4] = (int16_t)permil;
      d[5] = (int16_t)classes[i];
      ++kept;
    }
    *out_count = kept;
    return 0;
  }
  if (!out_buf || !out_count || !g_interpreter ||
      g_interpreter->outputs().size() < 4) {
    return -1;
  }
  const float* scores = nullptr;
  const float* boxes = nullptr;
  const float* num = nullptr;
  const float* classes = nullptr;
  int vector_candidates[2] = {-1, -1};
  int vector_count = 0;
  for (int oi = 0; oi < (int)g_interpreter->outputs().size(); ++oi) {
    auto* t = g_interpreter->output_tensor(oi);
    if (!t || t->type != kTfLiteFloat32 || !t->dims) continue;
    if (t->dims->size == 2 && t->dims->data[0] == 1) {
      if (vector_count < 2) vector_candidates[vector_count++] = oi;
    } else if (t->dims->size == 3 && t->dims->data[0] == 1 &&
               t->dims->data[2] == 4) {
      boxes = (const float*)t->data.data;
    } else if (t->dims->size == 1 && t->dims->data[0] == 1) {
      num = (const float*)t->data.data;
    }
  }
  if (vector_count == 2) {
    auto* a = g_interpreter->output_tensor(vector_candidates[0]);
    auto* b = g_interpreter->output_tensor(vector_candidates[1]);
    const float* av = (const float*)a->data.data;
    const float* bv = (const float*)b->data.data;
    int n = a->dims->data[1];
    if (b->dims->data[1] < n) n = b->dims->data[1];
    float amax = 0.0f, bmax = 0.0f;
    for (int i = 0; i < n && i < 20; ++i) {
      if (av[i] > amax) amax = av[i];
      if (bv[i] > bmax) bmax = bv[i];
    }
    if (amax > 2.0f && bmax <= 2.0f) {
      classes = av;
      scores = bv;
    } else if (bmax > 2.0f && amax <= 2.0f) {
      scores = av;
      classes = bv;
    } else {
      scores = av;
      classes = bv;
    }
  }
  if (!scores || !boxes || !num || !classes) return -2;
  auto* input = g_interpreter->input_tensor(0);
  int input_w = input->dims->data[2];
  int input_h = input->dims->data[1];
  int n_avail = (int)num[0];
  if (n_avail < 0) n_avail = 0;
  if (n_avail > 100) n_avail = 100;
  if (max_dets < 1) max_dets = 1;
  if (max_dets > 50) max_dets = 50;
  int kept = 0;
  for (int i = 0; i < n_avail && kept < max_dets; ++i) {
    int permil = (int)(scores[i] * 1000.0f + 0.5f);
    if (permil < conf_permil) continue;
    int16_t* d = out_buf + kept * 6;
    d[0] = (int16_t)(boxes[i * 4 + 1] * (float)input_w);
    d[1] = (int16_t)(boxes[i * 4 + 0] * (float)input_h);
    d[2] = (int16_t)(boxes[i * 4 + 3] * (float)input_w);
    d[3] = (int16_t)(boxes[i * 4 + 2] * (float)input_h);
    d[4] = (int16_t)permil;
    d[5] = (int16_t)classes[i];
    ++kept;
  }
  *out_count = kept;
  return 0;
}

extern "C" int sentai_tpu_output_yolo_info(int* layout_out,
                                           int* num_classes_out,
                                           int* num_anchors_out) {
  if (layout_out) *layout_out = 0;
  if (num_classes_out) *num_classes_out = 0;
  if (num_anchors_out) *num_anchors_out = 0;
  return -2;
}

extern "C" int sentai_tpu_draw(const char* path,
                               const int16_t* dets, int n_dets,
                               int quality) {
  (void)quality;
  if (using_host_pycoral()) {
    if (!path || !dets || !g_host_ready[0] || g_host_input[0].ndims < 4) return -1;
    int w = g_host_input[0].dims[2];
    int h = g_host_input[0].dims[1];
    int ch = g_host_input[0].dims[3];
    int img_bytes = w * h * ch;
    if (w <= 0 || h <= 0 || ch != 3 ||
        img_bytes > kMaxImageBytes || 54 + img_bytes > kMaxDrawBmpBytes) {
      return -3;
    }
    memset(g_draw_bmp, 0, 54);
    g_draw_bmp[0] = 'B'; g_draw_bmp[1] = 'M';
    put_le32(g_draw_bmp + 2, (uint32_t)(54 + img_bytes));
    put_le32(g_draw_bmp + 10, 54);
    put_le32(g_draw_bmp + 14, 40);
    put_le32(g_draw_bmp + 18, (uint32_t)w);
    put_le32(g_draw_bmp + 22, (uint32_t)h);
    put_le16(g_draw_bmp + 26, 1);
    put_le16(g_draw_bmp + 28, 24);
    put_le32(g_draw_bmp + 34, (uint32_t)img_bytes);
    uint8_t* pix = g_draw_bmp + 54;
    memcpy(pix, g_last_input_tensor, (size_t)img_bytes);
    for (int i = 0; i < n_dets; ++i) {
      draw_rect_rgb(pix, w, h, dets[i * 6 + 0], dets[i * 6 + 1],
                    dets[i * 6 + 2], dets[i * 6 + 3]);
    }
    for (int y = 0; y < h / 2; ++y) {
      uint8_t* a = pix + y * w * 3;
      uint8_t* b = pix + (h - 1 - y) * w * 3;
      for (int x = 0; x < w * 3; ++x) {
        uint8_t tmp = a[x]; a[x] = b[x]; b[x] = tmp;
      }
    }
    for (int y = 0; y < h; ++y) {
      uint8_t* row = pix + y * w * 3;
      for (int x = 0; x < w; ++x) {
        uint8_t* p = row + x * 3;
        uint8_t r = p[0]; p[0] = p[2]; p[2] = r;
      }
    }
    int ok = sentai_fs_write(path, g_draw_bmp, 54 + img_bytes);
    return ok == 1 ? 0 : -4;
  }
  if (!path || !dets || !g_interpreter) return -1;
  auto* input = g_interpreter->input_tensor(0);
  if (!input || input->dims->size < 4) return -2;
  int w = input->dims->data[2], h = input->dims->data[1], ch = input->dims->data[3];
  int img_bytes = w * h * ch;
  if (w <= 0 || h <= 0 || ch != 3 ||
      img_bytes > kMaxImageBytes || 54 + img_bytes > kMaxDrawBmpBytes) {
    return -3;
  }

  memset(g_draw_bmp, 0, 54);
  g_draw_bmp[0] = 'B'; g_draw_bmp[1] = 'M';
  put_le32(g_draw_bmp + 2, (uint32_t)(54 + img_bytes));
  put_le32(g_draw_bmp + 10, 54);
  put_le32(g_draw_bmp + 14, 40);
  put_le32(g_draw_bmp + 18, (uint32_t)w);
  put_le32(g_draw_bmp + 22, (uint32_t)h);
  put_le16(g_draw_bmp + 26, 1);
  put_le16(g_draw_bmp + 28, 24);
  put_le32(g_draw_bmp + 34, (uint32_t)img_bytes);
  uint8_t* pix = g_draw_bmp + 54;
  memcpy(pix, g_last_input_tensor, (size_t)img_bytes);
  for (int i = 0; i < n_dets; ++i) {
    draw_rect_rgb(pix, w, h, dets[i * 6 + 0], dets[i * 6 + 1],
                  dets[i * 6 + 2], dets[i * 6 + 3]);
  }
  for (int y = 0; y < h / 2; ++y) {
    uint8_t* a = pix + y * w * 3;
    uint8_t* b = pix + (h - 1 - y) * w * 3;
    for (int x = 0; x < w * 3; ++x) {
      uint8_t tmp = a[x]; a[x] = b[x]; b[x] = tmp;
    }
  }
  for (int y = 0; y < h; ++y) {
    uint8_t* row = pix + y * w * 3;
    for (int x = 0; x < w; ++x) {
      uint8_t* p = row + x * 3;
      uint8_t r = p[0]; p[0] = p[2]; p[2] = r;
    }
  }
  int ok = sentai_fs_write(path, g_draw_bmp, 54 + img_bytes);
  return ok == 1 ? 0 : -4;
}
