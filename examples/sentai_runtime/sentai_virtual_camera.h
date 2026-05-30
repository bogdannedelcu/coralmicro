#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_VIRTUAL_CAMERA_ID (-1)
#define SENTAI_VIRTUAL_CAMERA_FRAME_IDX 32767

int sentai_virtual_camera_select(const char* path);
void sentai_virtual_camera_disable(void);
int sentai_virtual_camera_active(void);
int sentai_virtual_camera_width(void);
int sentai_virtual_camera_height(void);
uint32_t sentai_virtual_camera_seq(void);
const char* sentai_virtual_camera_path(void);

int sentai_virtual_camera_grab_xrgb(uint8_t** out_raw);
size_t sentai_virtual_camera_get_rgb(uint8_t* dst, size_t max_bytes,
                                     int* out_w, int* out_h,
                                     uint32_t* out_seq);

#ifdef __cplusplus
}
#endif
