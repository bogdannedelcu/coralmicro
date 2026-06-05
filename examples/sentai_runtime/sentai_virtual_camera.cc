#include "examples/sentai_runtime/sentai_virtual_camera.h"

#include "examples/sentai_runtime/sentai_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" int sentai_fs_size(const char* path);
extern "C" int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
extern "C" int sentai_fs_listdir(
    const char* path,
    void (*callback)(const char* name, int type, int size, void* ud),
    void* user_data);

extern "C" void sentai_virtual_camera_after_select(void) __attribute__((weak));

namespace {

#ifndef SENTAI_RUNTIME_CAMERA_W
#define SENTAI_RUNTIME_CAMERA_W 640
#endif
#ifndef SENTAI_RUNTIME_CAMERA_H
#define SENTAI_RUNTIME_CAMERA_H 480
#endif
constexpr int kMaxW = SENTAI_RUNTIME_CAMERA_W;
constexpr int kMaxH = SENTAI_RUNTIME_CAMERA_H;
constexpr int kMaxPath = 128;
constexpr int kMaxPlaybackFrames = 64;
constexpr int kPlaybackStackWords = configMINIMAL_STACK_SIZE * 12;

static uint8_t s_loaded_xrgb[kMaxW * kMaxH * 4]
#if !defined(SENTAI_PLATFORM_SIM)
    __attribute__((section(".sdram_bss")))
#endif
    ;

static uint8_t s_frame_xrgb[SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS][kMaxW * kMaxH * 4]
#if !defined(SENTAI_PLATFORM_SIM)
    __attribute__((section(".sdram_bss")))
#endif
    ;

static uint8_t s_bmp_file[54 + kMaxW * kMaxH * 4]
#if !defined(SENTAI_PLATFORM_SIM)
    __attribute__((section(".sdram_bss")))
#endif
    ;

typedef struct {
  uint32_t seq;
  int cam_id;
  uint8_t full;
  uint8_t held;
} FrameSlot;

static int s_active = 0;
static int s_w = 0;
static int s_h = 0;
static uint32_t s_seq = 0;
static char s_path[kMaxPath] = {0};
static FrameSlot s_slots[SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS];
static int s_next_slot = 0;
static int s_last_grabbed_cam_id = SENTAI_VIRTUAL_CAMERA_ID;
static int s_current_cam_id = SENTAI_VIRTUAL_CAMERA_ID;

static StaticTask_t s_play_tcb;
static StackType_t s_play_stack[kPlaybackStackWords]
    __attribute__((aligned(8)
#if !defined(SENTAI_PLATFORM_SIM)
                   , section(".sdram_bss")
#endif
                   ));
static TaskHandle_t s_play_task = nullptr;
static volatile int s_play_running = 0;
static volatile int s_play_stop_requested = 0;
static volatile int s_play_index = 0;
static volatile int s_play_last_rc = 0;
static int s_play_count = 0;
static int s_play_limit = 0;
static int s_play_fps = 10;
static int s_play_memory_mode = 0;
static char s_play_dir[kMaxPath] = {0};
static char s_play_paths[kMaxPlaybackFrames][kMaxPath];

static uint16_t rd16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t rd32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t rds32(const uint8_t* p) {
  return (int32_t)rd32(p);
}

static int load_bmp(const char* path, const uint8_t* file, int len) {
  if (!path || !file || len < 54) return -1;
  if (file[0] != 'B' || file[1] != 'M') return -2;

  const uint32_t pixel_off = rd32(file + 10);
  const uint32_t dib_size = rd32(file + 14);
  if (dib_size < 40 || pixel_off >= (uint32_t)len) return -3;

  const int32_t w_signed = rds32(file + 18);
  const int32_t h_signed = rds32(file + 22);
  const uint16_t planes = rd16(file + 26);
  const uint16_t bpp = rd16(file + 28);
  const uint32_t compression = rd32(file + 30);
  if (planes != 1 || compression != 0) return -4;
  if (bpp != 24 && bpp != 32) return -5;
  if (w_signed <= 0 || h_signed == 0) return -6;

  const int w = (int)w_signed;
  const int h = (h_signed < 0) ? (int)(-h_signed) : (int)h_signed;
  if (w <= 0 || h <= 0 || w > kMaxW || h > kMaxH) return -7;

  const int bytes_pp = bpp / 8;
  const int row_bytes = ((w * bytes_pp + 3) / 4) * 4;
  const uint32_t need = pixel_off + (uint32_t)(row_bytes * h);
  if (need > (uint32_t)len) return -8;

  const int top_down = (h_signed < 0);
  if (w != kMaxW || h != kMaxH) {
    memset(s_loaded_xrgb, 0, sizeof(s_loaded_xrgb));
  }
  for (int y = 0; y < h; ++y) {
    const int src_y = top_down ? y : (h - 1 - y);
    const uint8_t* src = file + pixel_off + (uint32_t)(src_y * row_bytes);
    uint8_t* dst = s_loaded_xrgb + (size_t)y * (size_t)kMaxW * 4u;
    for (int x = 0; x < w; ++x) {
      const uint8_t b = src[x * bytes_pp + 0];
      const uint8_t g = src[x * bytes_pp + 1];
      const uint8_t r = src[x * bytes_pp + 2];
      dst[x * 4 + 0] = b;
      dst[x * 4 + 1] = g;
      dst[x * 4 + 2] = r;
      dst[x * 4 + 3] = 0;
    }
  }

  // Prepare a VGA-sized virtual camera frame.  A few legacy ARM consumers
  // still assume DEMO_CAMERA_WIDTH/HEIGHT when scaling camera raw, so the
  // virtual framebuffer keeps that contract and places the BMP at top-left.
  s_w = kMaxW;
  s_h = kMaxH;
  s_active = 1;
  snprintf(s_path, sizeof(s_path), "%s", path);
  return 0;
}

static int slot_from_frame_idx(int idx) {
  const int slot = idx - SENTAI_VIRTUAL_CAMERA_FRAME_IDX;
  if (slot < 0 || slot >= SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS) return -1;
  return slot;
}

static int choose_write_slot(void) {
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS; ++i) {
      const int slot = (s_next_slot + i) % SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS;
      if (s_slots[slot].held) continue;
      if (pass == 0 && s_slots[slot].full) continue;
      s_next_slot = (slot + 1) % SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS;
      return slot;
    }
  }
  return -1;
}

static int publish_xrgb(uint32_t seq, int cam_id, const uint8_t* xrgb) {
  if (!xrgb) return -1;
  const int slot = choose_write_slot();
  if (slot < 0) return -2;

  memcpy(s_frame_xrgb[slot], xrgb, (size_t)kMaxW * (size_t)kMaxH * 4u);
  taskENTER_CRITICAL();
  const uint32_t out_seq = seq ? seq : (s_seq + 1u);
  s_seq = out_seq;
  s_current_cam_id = cam_id;
  s_slots[slot].seq = out_seq;
  s_slots[slot].cam_id = cam_id;
  s_slots[slot].full = 1;
  s_slots[slot].held = 0;
  s_active = 1;
  taskEXIT_CRITICAL();

  if (sentai_virtual_camera_after_select) {
    sentai_virtual_camera_after_select();
  }
  return 0;
}

static int has_bmp_suffix(const char* name) {
  if (!name) return 0;
  const size_t n = strlen(name);
  if (n < 5) return 0;
  const char* s = name + n - 4;
  return (s[0] == '.') &&
         (s[1] == 'b' || s[1] == 'B') &&
         (s[2] == 'm' || s[2] == 'M') &&
         (s[3] == 'p' || s[3] == 'P');
}

static int build_child_path(char* dst, size_t dst_size,
                            const char* dir, const char* name) {
  if (!dst || dst_size == 0 || !dir || !name) return -1;
  const size_t dn = strlen(dir);
  const char sep = (dn > 0 && dir[dn - 1] == '/') ? '\0' : '/';
  int n = 0;
  if (sep) {
    n = snprintf(dst, dst_size, "%s/%s", dir, name);
  } else {
    n = snprintf(dst, dst_size, "%s%s", dir, name);
  }
  return (n > 0 && (size_t)n < dst_size) ? 0 : -1;
}

static void sort_play_paths(void) {
  for (int i = 1; i < s_play_count; ++i) {
    char tmp[kMaxPath];
    snprintf(tmp, sizeof(tmp), "%s", s_play_paths[i]);
    int j = i - 1;
    while (j >= 0 && strcmp(s_play_paths[j], tmp) > 0) {
      snprintf(s_play_paths[j + 1], sizeof(s_play_paths[j + 1]), "%s",
               s_play_paths[j]);
      --j;
    }
    snprintf(s_play_paths[j + 1], sizeof(s_play_paths[j + 1]), "%s", tmp);
  }
}

struct ListCtx {
  const char* dir;
};

static void list_play_frame_cb(const char* name, int type, int size, void* ud) {
  (void)size;
  ListCtx* ctx = static_cast<ListCtx*>(ud);
  if (!ctx || !ctx->dir || type != 1 || !has_bmp_suffix(name)) return;
  if (s_play_count >= kMaxPlaybackFrames) return;
  if (build_child_path(s_play_paths[s_play_count],
                       sizeof(s_play_paths[s_play_count]),
                       ctx->dir, name) == 0) {
    ++s_play_count;
  }
}

static int load_play_list(const char* dir) {
  if (!dir || !dir[0]) return -1;
  s_play_count = 0;
  memset(s_play_paths, 0, sizeof(s_play_paths));
  ListCtx ctx = {dir};
  const int rc = sentai_fs_listdir(dir, list_play_frame_cb, &ctx);
  if (rc < 0) return -2;
  if (s_play_count <= 0) return -3;
  sort_play_paths();
  return s_play_count;
}

static int select_from_fs(const char* path, int publish) {
  if (!path || !path[0]) return -1;
  const int size = sentai_fs_size(path);
  if (size <= 0) return -2;
  if ((size_t)size > sizeof(s_bmp_file)) return -3;
  const int n = sentai_fs_read(path, s_bmp_file, size);
  if (n != size) {
    return -4;
  }
  const int rc = load_bmp(path, s_bmp_file, size);
  if (rc == 0 && publish) {
    return publish_xrgb(0, SENTAI_VIRTUAL_CAMERA_ID, s_loaded_xrgb);
  }
  return rc;
}

static int publish_current_from_memory(void) {
  if (!s_active || s_w <= 0 || s_h <= 0) return -1;
  return publish_xrgb(0, SENTAI_VIRTUAL_CAMERA_ID, s_loaded_xrgb);
}

static void virtual_camera_task_fn(void*) {
  s_play_running = 1;
  int published = 0;
  const int period_ms = (s_play_fps <= 0) ? 100 :
                        ((1000 + s_play_fps - 1) / s_play_fps);
  const TickType_t period_ticks = pdMS_TO_TICKS((uint32_t)period_ms);
  sentai_logf("virt_cam", "START mode=%s dir=%s frames=%d fps=%d limit=%d",
              s_play_memory_mode ? "memory" : "fs",
              s_play_dir, s_play_count, s_play_fps, s_play_limit);

  while (!s_play_stop_requested &&
         (s_play_limit <= 0 || published < s_play_limit)) {
    const int idx = s_play_index;
    s_play_last_rc = s_play_memory_mode
        ? publish_current_from_memory()
        : select_from_fs(s_play_paths[idx], 1);
    if (published < 8 || s_play_last_rc != 0) {
      sentai_logf("virt_cam", "publish idx=%d rc=%d path=%s",
                  idx, s_play_last_rc,
                  s_play_memory_mode ? s_path : s_play_paths[idx]);
    }
    if (s_play_last_rc == 0) {
      ++published;
      s_play_index = s_play_memory_mode ? 0 : ((idx + 1) % s_play_count);
    }
    vTaskDelay(period_ticks ? period_ticks : 1);
  }

  sentai_logf("virt_cam", "STOP published=%d last_rc=%d",
              published, s_play_last_rc);
  s_play_running = 0;
  s_play_task = nullptr;
  vTaskDelete(nullptr);
}

}  // namespace

extern "C" int sentai_virtual_camera_select(const char* path) {
  (void)sentai_virtual_camera_play_stop();
  return select_from_fs(path, 0);
}

extern "C" int sentai_virtual_camera_play(const char* dir, int fps, int count) {
  (void)sentai_virtual_camera_play_stop();
  if (!dir || !dir[0]) return -1;
  if (fps < 1) fps = 1;
  if (fps > 60) fps = 60;
  if (count < 0) count = 0;
  const int n = load_play_list(dir);
  if (n < 0) return n;
  snprintf(s_play_dir, sizeof(s_play_dir), "%s", dir);
  s_play_fps = fps;
  s_play_limit = count;
  s_play_index = 0;
  s_play_memory_mode = 0;
  s_play_last_rc = 0;
  s_play_stop_requested = 0;
  s_play_running = 0;
  s_play_task = xTaskCreateStatic(virtual_camera_task_fn, "virt_cam",
                                  kPlaybackStackWords, nullptr,
                                  tskIDLE_PRIORITY + 2,
                                  s_play_stack, &s_play_tcb);
  if (!s_play_task) {
    s_play_running = 0;
    return -4;
  }
  return n;
}

extern "C" int sentai_virtual_camera_replay(int fps, int count) {
  (void)sentai_virtual_camera_play_stop();
  if (!s_active || s_w <= 0 || s_h <= 0) return -1;
  if (fps < 1) fps = 1;
  if (fps > 60) fps = 60;
  if (count < 0) count = 0;
  snprintf(s_play_dir, sizeof(s_play_dir), "<memory>");
  s_play_count = 1;
  s_play_fps = fps;
  s_play_limit = count;
  s_play_index = 0;
  s_play_memory_mode = 1;
  s_play_last_rc = 0;
  s_play_stop_requested = 0;
  s_play_running = 0;
  s_play_task = xTaskCreateStatic(virtual_camera_task_fn, "virt_cam",
                                  kPlaybackStackWords, nullptr,
                                  tskIDLE_PRIORITY + 2,
                                  s_play_stack, &s_play_tcb);
  if (!s_play_task) {
    s_play_running = 0;
    return -4;
  }
  return 1;
}

extern "C" int sentai_virtual_camera_publish_loaded(void) {
  return publish_current_from_memory();
}

extern "C" int sentai_virtual_camera_play_stop(void) {
  TaskHandle_t h = s_play_task;
  if (!h) {
    s_play_running = 0;
    return 0;
  }
  s_play_stop_requested = 1;
  for (int i = 0; i < 1200 && s_play_task != nullptr; ++i) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return (s_play_task == nullptr) ? 0 : -1;
}

extern "C" int sentai_virtual_camera_playing(void) {
  return s_play_running ? 1 : 0;
}

extern "C" void sentai_virtual_camera_disable(void) {
  (void)sentai_virtual_camera_play_stop();
  taskENTER_CRITICAL();
  s_active = 0;
  s_w = 0;
  s_h = 0;
  s_path[0] = '\0';
  s_seq++;
  memset(s_slots, 0, sizeof(s_slots));
  taskEXIT_CRITICAL();
}

extern "C" int sentai_virtual_camera_active(void) {
  return s_active;
}

extern "C" int sentai_virtual_camera_width(void) {
  return s_w;
}

extern "C" int sentai_virtual_camera_height(void) {
  return s_h;
}

extern "C" uint32_t sentai_virtual_camera_seq(void) {
  return s_seq;
}

extern "C" const char* sentai_virtual_camera_path(void) {
  return s_path;
}

extern "C" int sentai_virtual_camera_grab_xrgb(uint8_t** out_raw) {
  if (!out_raw || !s_active || s_w <= 0 || s_h <= 0) return -1;
  int best = -1;
  uint32_t best_seq = 0;
  taskENTER_CRITICAL();
  for (int i = 0; i < SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS; ++i) {
    if (!s_slots[i].full || s_slots[i].held) continue;
    if (best < 0 || s_slots[i].seq > best_seq) {
      best = i;
      best_seq = s_slots[i].seq;
    }
  }
  if (best >= 0) {
    for (int i = 0; i < SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS; ++i) {
      if (i != best && !s_slots[i].held) {
        s_slots[i].full = 0;
      }
    }
    s_slots[best].held = 1;
    s_last_grabbed_cam_id = s_slots[best].cam_id;
  }
  taskEXIT_CRITICAL();
  if (best < 0) return -2;
  *out_raw = s_frame_xrgb[best];
  return SENTAI_VIRTUAL_CAMERA_FRAME_IDX + best;
}

extern "C" void sentai_virtual_camera_return_raw(int idx) {
  const int slot = slot_from_frame_idx(idx);
  if (slot < 0) return;
  taskENTER_CRITICAL();
  s_slots[slot].held = 0;
  s_slots[slot].full = 0;
  taskEXIT_CRITICAL();
}

extern "C" int sentai_virtual_camera_is_frame_idx(int idx) {
  return slot_from_frame_idx(idx) >= 0 ? 1 : 0;
}

extern "C" int sentai_virtual_camera_grabbed_id(void) {
  return s_last_grabbed_cam_id;
}

extern "C" int sentai_virtual_camera_current_id(void) {
  return s_current_cam_id;
}

extern "C" int sentai_virtual_camera_publish_xrgb(uint32_t seq, int cam_id,
                                                  const uint8_t* xrgb) {
  if (s_w <= 0 || s_h <= 0) {
    s_w = kMaxW;
    s_h = kMaxH;
  }
  return publish_xrgb(seq, cam_id, xrgb);
}

extern "C" size_t sentai_virtual_camera_get_rgb(uint8_t* dst, size_t max_bytes,
                                                 int* out_w, int* out_h,
                                                 uint32_t* out_seq) {
  if (s_w < 0 || s_h < 0 || s_w > kMaxW || s_h > kMaxH) {
    sentai_logf("virt_cam", "bad dims w=%d h=%d active=%d seq=%lu",
                s_w, s_h, s_active, (unsigned long)s_seq);
    if (out_w) *out_w = 0;
    if (out_h) *out_h = 0;
    if (out_seq) *out_seq = s_seq;
    return 0;
  }
  if (out_w) *out_w = s_w;
  if (out_h) *out_h = s_h;
  if (out_seq) *out_seq = s_seq;
  if (!s_active || !dst || s_w <= 0 || s_h <= 0) return 0;
  const uint8_t* src_base = s_loaded_xrgb;
  uint32_t best_seq = 0;
  for (int i = 0; i < SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS; ++i) {
    if (s_slots[i].full && s_slots[i].seq >= best_seq) {
      best_seq = s_slots[i].seq;
      src_base = s_frame_xrgb[i];
    }
  }
  if (out_seq && best_seq) *out_seq = best_seq;
  const size_t need = (size_t)s_w * (size_t)s_h * 3u;
  if (max_bytes < need) return 0;
  for (int y = 0; y < s_h; ++y) {
    const uint8_t* src = src_base + (size_t)y * (size_t)kMaxW * 4u;
    uint8_t* out = dst + (size_t)y * (size_t)s_w * 3u;
    for (int x = 0; x < s_w; ++x) {
      out[x * 3 + 0] = src[x * 4 + 2];
      out[x * 3 + 1] = src[x * 4 + 1];
      out[x * 3 + 2] = src[x * 4 + 0];
    }
  }
  return need;
}
