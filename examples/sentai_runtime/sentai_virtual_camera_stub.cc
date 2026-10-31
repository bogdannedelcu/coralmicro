// W24-T11 (2026-10-31): board build without the virtual camera.
//
// sentai_virtual_camera.cc (BMP playback into the pipeline, a SIM / bench
// tool) carries 7.4 MB of XYUV frame storage in the non-cacheable camera
// region on the ARM build (1 loaded frame + 4 slots + 1 BMP file buffer at
// 640x480x4).  The physical board never uses it (operator, 2026-10-31), and
// the SXGA camera mode needs that SDRAM for its own frame buffers, so the ARM
// target links this stub instead: every entry point reports "inactive" and
// the callers in sentai_runtime.cc / detection_task.cc take their real-camera
// branch.  The SIM build keeps the full implementation (sim/CMakeLists.txt).
// Re-enable on the board with -DSENTAI_VIRTUAL_CAMERA=ON.

#include "examples/sentai_runtime/sentai_virtual_camera.h"

extern "C" {

int sentai_virtual_camera_select(const char*) { return -1; }
int sentai_virtual_camera_play(const char*, int, int) { return -1; }
int sentai_virtual_camera_replay(int, int) { return -1; }
int sentai_virtual_camera_publish_loaded(void) { return -1; }
int sentai_virtual_camera_play_stop(void) { return 0; }
int sentai_virtual_camera_playing(void) { return 0; }
void sentai_virtual_camera_disable(void) {}
int sentai_virtual_camera_active(void) { return 0; }
int sentai_virtual_camera_width(void) { return 0; }
int sentai_virtual_camera_height(void) { return 0; }
uint32_t sentai_virtual_camera_seq(void) { return 0; }
const char* sentai_virtual_camera_path(void) { return ""; }

int sentai_virtual_camera_grab_xrgb(uint8_t** out_raw) {
  if (out_raw) *out_raw = nullptr;
  return -1;
}
void sentai_virtual_camera_return_raw(int) {}
int sentai_virtual_camera_is_frame_idx(int) { return 0; }
int sentai_virtual_camera_grabbed_id(void) { return -1; }
int sentai_virtual_camera_current_id(void) { return -1; }
int sentai_virtual_camera_publish_xrgb(uint32_t, int, const uint8_t*) { return -1; }
int sentai_virtual_camera_reserve_xrgb(uint8_t** out_raw, size_t* out_bytes) {
  if (out_raw) *out_raw = nullptr;
  if (out_bytes) *out_bytes = 0;
  return -1;
}
int sentai_virtual_camera_commit_xrgb(int, uint32_t, int) { return -1; }
void sentai_virtual_camera_abort_xrgb(int) {}
size_t sentai_virtual_camera_get_rgb(uint8_t*, size_t, int* out_w, int* out_h,
                                     uint32_t* out_seq) {
  if (out_w) *out_w = 0;
  if (out_h) *out_h = 0;
  if (out_seq) *out_seq = 0;
  return 0;
}

}  // extern "C"
