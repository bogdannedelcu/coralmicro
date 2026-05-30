#include "examples/sentai_runtime/sentai_virtual_camera.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" int sentai_fs_size(const char* path);
extern "C" int sentai_fs_read(const char* path, uint8_t* buf, int max_size);

namespace {

constexpr int kMaxW = 640;
constexpr int kMaxH = 480;
constexpr int kMaxPath = 128;

static uint8_t s_xrgb[kMaxW * kMaxH * 4]
#if !defined(SENTAI_PLATFORM_SIM)
    __attribute__((section(".sdram_bss")))
#endif
    ;

static int s_active = 0;
static int s_w = 0;
static int s_h = 0;
static uint32_t s_seq = 0;
static char s_path[kMaxPath] = {0};

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
  memset(s_xrgb, 0, sizeof(s_xrgb));
  for (int y = 0; y < h; ++y) {
    const int src_y = top_down ? y : (h - 1 - y);
    const uint8_t* src = file + pixel_off + (uint32_t)(src_y * row_bytes);
    uint8_t* dst = s_xrgb + (size_t)y * (size_t)kMaxW * 4u;
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

  // Publish a VGA-sized virtual camera frame.  A few legacy ARM consumers
  // still assume DEMO_CAMERA_WIDTH/HEIGHT when scaling camera raw, so the
  // virtual framebuffer keeps that contract and places the BMP at top-left.
  s_w = kMaxW;
  s_h = kMaxH;
  s_active = 1;
  s_seq++;
  snprintf(s_path, sizeof(s_path), "%s", path);
  return 0;
}

}  // namespace

extern "C" int sentai_virtual_camera_select(const char* path) {
  if (!path || !path[0]) return -1;
  const int size = sentai_fs_size(path);
  if (size <= 0) return -2;
  uint8_t* buf = (uint8_t*)malloc((size_t)size);
  if (!buf) return -3;
  const int n = sentai_fs_read(path, buf, size);
  if (n != size) {
    free(buf);
    return -4;
  }
  const int rc = load_bmp(path, buf, size);
  free(buf);
  return rc;
}

extern "C" void sentai_virtual_camera_disable(void) {
  s_active = 0;
  s_w = 0;
  s_h = 0;
  s_path[0] = '\0';
  s_seq++;
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
  *out_raw = s_xrgb;
  return SENTAI_VIRTUAL_CAMERA_FRAME_IDX;
}

extern "C" size_t sentai_virtual_camera_get_rgb(uint8_t* dst, size_t max_bytes,
                                                 int* out_w, int* out_h,
                                                 uint32_t* out_seq) {
  if (out_w) *out_w = s_w;
  if (out_h) *out_h = s_h;
  if (out_seq) *out_seq = s_seq;
  if (!s_active || !dst || s_w <= 0 || s_h <= 0) return 0;
  const size_t need = (size_t)s_w * (size_t)s_h * 3u;
  if (max_bytes < need) return 0;
  for (int y = 0; y < s_h; ++y) {
    const uint8_t* src = s_xrgb + (size_t)y * (size_t)kMaxW * 4u;
    uint8_t* out = dst + (size_t)y * (size_t)s_w * 3u;
    for (int x = 0; x < s_w; ++x) {
      out[x * 3 + 0] = src[x * 4 + 2];
      out[x * 3 + 1] = src[x * 4 + 1];
      out[x * 3 + 2] = src[x * 4 + 0];
    }
  }
  return need;
}
