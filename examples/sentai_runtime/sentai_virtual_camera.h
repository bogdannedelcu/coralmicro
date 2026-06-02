#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_VIRTUAL_CAMERA_ID (-1)
#define SENTAI_VIRTUAL_CAMERA_FRAME_IDX 32767
#define SENTAI_VIRTUAL_CAMERA_FRAME_SLOTS 4

int sentai_virtual_camera_select(const char* path);
int sentai_virtual_camera_play(const char* dir, int fps, int count);
int sentai_virtual_camera_replay(int fps, int count);
int sentai_virtual_camera_publish_loaded(void);
int sentai_virtual_camera_play_stop(void);
int sentai_virtual_camera_playing(void);
void sentai_virtual_camera_disable(void);
int sentai_virtual_camera_active(void);
int sentai_virtual_camera_width(void);
int sentai_virtual_camera_height(void);
uint32_t sentai_virtual_camera_seq(void);
const char* sentai_virtual_camera_path(void);

int sentai_virtual_camera_grab_xrgb(uint8_t** out_raw);
void sentai_virtual_camera_return_raw(int idx);
int sentai_virtual_camera_is_frame_idx(int idx);
int sentai_virtual_camera_grabbed_id(void);
int sentai_virtual_camera_current_id(void);
int sentai_virtual_camera_publish_xrgb(uint32_t seq, int cam_id,
                                       const uint8_t* xrgb);
size_t sentai_virtual_camera_get_rgb(uint8_t* dst, size_t max_bytes,
                                     int* out_w, int* out_h,
                                     uint32_t* out_seq);

#ifdef __cplusplus
}
#endif
