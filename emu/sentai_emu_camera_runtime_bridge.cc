// sentai_emu_camera_runtime_bridge.cc -- guest-side camera glue for Renode.
//
// This file is not a host program.  It supplies the small HAL surface that
// production sentai.camera expects, while frame ownership stays in the shared
// sentai_runtime virtual-camera + camera_frame_backend code.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "examples/sentai_runtime/sentai_virtual_camera.h"
#include "libs/base/fx_user_fs.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" int sentai_camera_backend_publish_prep_once(void);
extern "C" int sentai_cam_grab_latest(uint8_t** raw);
extern "C" void sentai_cam_return_raw(int idx);
extern "C" int sentai_cam_get_width(void);
extern "C" int sentai_cam_get_height(void);
extern "C" int sentai_cam_grabbed_id(void);

extern "C" {
volatile uint32_t g_runtime_fps = 30;
volatile uint8_t g_cam_buf_id[4] = {0, 0, 0, 0};
volatile uint8_t g_cam_buf_id_task[4] = {0, 0, 0, 0};
volatile uint8_t g_cam_buf_id_csi[4] = {0, 0, 0, 0};
volatile uint32_t g_cam_buf_tag_writes = 0;
volatile uint32_t g_cam_buf_tag_idx_miss = 0;
volatile uint32_t g_cam_buf_tag_skip_both = 0;
volatile uint32_t g_cam_buf_task_writes = 0;
volatile uint32_t g_cam_buf_task_unknown = 0;
volatile uint32_t g_cam_csi_hook_fires = 0;
volatile uint32_t g_cam_dirty_consecutive_n = 1;
}

namespace {

constexpr int kDefaultW = 640;
constexpr int kDefaultH = 480;

int g_cam_w = kDefaultW;
int g_cam_h = kDefaultH;
int g_cam_running = 0;
uint32_t g_switch_drain = 2;

struct ListCtx {
  char (*names)[96];
  int max_entries;
  int count;
};

int ListCb(const FxDirEntry* entry, void* user) {
  ListCtx* ctx = static_cast<ListCtx*>(user);
  if (!ctx || ctx->count >= ctx->max_entries) return 1;
  strncpy(ctx->names[ctx->count], entry->name, 95);
  ctx->names[ctx->count][95] = '\0';
  ++ctx->count;
  return 0;
}

int CopyLatestRow(uint8_t* dst, int len, int row) {
  if (!dst || len <= 0) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_grab_latest(&raw);
  if (idx < 0 || !raw) return idx;
  int got = 0;
  const int stride = sentai_cam_get_width() * 4;
  const int h = sentai_cam_get_height();
  if (stride > 0 && row >= 0 && row < h) {
    if (len > stride) len = stride;
    memcpy(dst, raw + row * stride, static_cast<size_t>(len));
    got = len;
  } else {
    got = -1;
  }
  sentai_cam_return_raw(idx);
  return got;
}

}  // namespace

extern "C" uint32_t sentai_ticks_ms(void) {
  return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

extern "C" int sentai_fs_size(const char* path) {
  const ssize_t size = FxUserSize(path);
  return size < 0 ? -1 : static_cast<int>(size);
}

extern "C" int sentai_fs_read(const char* path, uint8_t* buf, int max_size) {
  if (!path || !buf || max_size < 0) return -1;
  return static_cast<int>(FxUserReadFile(path, buf, static_cast<size_t>(max_size)));
}

extern "C" int sentai_fs_write(const char* path, const uint8_t* buf, int size) {
  if (!path || size < 0) return 0;
  return FxUserWriteFile(path, buf, static_cast<size_t>(size));
}

extern "C" int sentai_fs_listdir(const char* path,
                                  char names[][96],
                                  int max_entries) {
  if (!path || !names || max_entries <= 0) return -1;
  ListCtx ctx{names, max_entries, 0};
  const int n = FxUserListDir(path, ListCb, &ctx);
  return n < 0 ? -1 : ctx.count;
}

extern "C" int sentai_cam_init_full(int streaming, int fps,
                                     int hflip, int vflip) {
  (void)streaming;
  (void)hflip;
  (void)vflip;
  if (fps > 0) g_runtime_fps = static_cast<uint32_t>(fps);
  g_cam_running = 1;
  return 0;
}

extern "C" int sentai_cam_init_fps(int streaming, int fps) {
  return sentai_cam_init_full(streaming, fps, 0, 1);
}

extern "C" int sentai_cam_stop(void) {
  g_cam_running = 0;
  sentai_virtual_camera_disable();
  return 0;
}

extern "C" int sentai_cam_capture_jpeg(uint8_t* jpeg_buf,
                                        int jpeg_buf_size,
                                        int width, int height,
                                        int quality) {
  (void)jpeg_buf;
  (void)jpeg_buf_size;
  (void)width;
  (void)height;
  (void)quality;
  return -3;
}

extern "C" int sentai_cam_to_tensor_ex(const char* save_path, int quality) {
  (void)save_path;
  (void)quality;
  return -3;
}

extern "C" int sentai_cam_get_width(void) {
  if (sentai_virtual_camera_active()) {
    const int w = sentai_virtual_camera_width();
    return w > 0 ? w : g_cam_w;
  }
  return g_cam_w;
}

extern "C" int sentai_cam_get_height(void) {
  if (sentai_virtual_camera_active()) {
    const int h = sentai_virtual_camera_height();
    return h > 0 ? h : g_cam_h;
  }
  return g_cam_h;
}

extern "C" int sentai_cam_set_res(int w, int h) {
  if (w <= 0 || h <= 0 || w > kDefaultW || h > kDefaultH) return -1;
  g_cam_w = w;
  g_cam_h = h;
  return 0;
}

extern "C" int sentai_cam_get_native_width(void) { return kDefaultW; }
extern "C" int sentai_cam_get_native_height(void) { return kDefaultH; }

extern "C" int sentai_cam_switch(int id) {
  if (id < 0 || id > 1) return -2;
  (void)id;
  return 0;
}

extern "C" void sentai_cam_stats_get(uint32_t* ok_eof,
                                      uint32_t* fallback,
                                      uint32_t* drain_timeout,
                                      uint32_t* grab_retry,
                                      uint32_t* grab_fatal) {
  if (ok_eof) *ok_eof = 0;
  if (fallback) *fallback = 0;
  if (drain_timeout) *drain_timeout = 0;
  if (grab_retry) *grab_retry = 0;
  if (grab_fatal) *grab_fatal = 0;
}

extern "C" int sentai_cam_peek5_b40(uint8_t* dst5) {
  if (!dst5) return -1;
  uint8_t* raw = nullptr;
  int idx = sentai_cam_grab_latest(&raw);
  if (idx < 0 || !raw) return idx;
  const int w = sentai_cam_get_width();
  const int h = sentai_cam_get_height();
  for (int i = 0; i < 5; ++i) {
    const int y = (h > 1) ? (i * (h - 1)) / 4 : 0;
    const int x = (w > 40) ? 40 : 0;
    dst5[i] = raw[(y * w + x) * 4 + 0];
  }
  sentai_cam_return_raw(idx);
  return sentai_cam_grabbed_id();
}

extern "C" int sentai_cam_peek_first_row(uint8_t* dst, int len) {
  return CopyLatestRow(dst, len, 0);
}

extern "C" int sentai_cam_rotate(int cam_id, int degrees) {
  (void)cam_id;
  return (degrees == 0 || degrees == 180) ? 0 : -2;
}

extern "C" int sentai_cam_aec_set(int cam_id, int high, int low) {
  (void)cam_id;
  (void)high;
  (void)low;
  return 0;
}

extern "C" int sentai_cam_gain_ceiling_set(int cam_id, int ceiling) {
  (void)cam_id;
  (void)ceiling;
  return 0;
}

extern "C" int sentai_cam_isp_preset(int cam_id, const char* name) {
  (void)cam_id;
  (void)name;
  return 0;
}

extern "C" int sentai_cam_reg_read(int cam_id, int reg) {
  (void)cam_id;
  (void)reg;
  return -1;
}

extern "C" int sentai_cam_reg_write(int cam_id, int reg, int val) {
  (void)cam_id;
  (void)reg;
  (void)val;
  return -3;
}

extern "C" uint32_t sentai_cam_switch_drain_get(void) {
  return g_switch_drain;
}

extern "C" int sentai_cam_switch_drain_set(uint32_t n) {
  if (n < 1 || n > 10) return -1;
  g_switch_drain = n;
  return 0;
}

extern "C" int sentai_get_tensor_info(int* w, int* h, int* ch,
                                       uint8_t** buf, int* type, int* zp) {
  (void)w;
  (void)h;
  (void)ch;
  (void)buf;
  (void)type;
  (void)zp;
  return -1;
}

extern "C" int sentai_prep_publish_slot_rgb_64(const uint8_t* raw_xrgb,
                                                int src_w, int src_h) {
  (void)raw_xrgb;
  (void)src_w;
  (void)src_h;
  return 0;
}

extern "C" __attribute__((weak)) void sentai_flow_poll_once(void) {}

extern "C" int sentai_fr_push_frame(const uint8_t* gray, int w, int h,
                                     int markers, uint32_t frame_seq,
                                     uint32_t ts_ms) {
  (void)gray;
  (void)w;
  (void)h;
  (void)markers;
  (void)frame_seq;
  (void)ts_ms;
  return -3;
}
