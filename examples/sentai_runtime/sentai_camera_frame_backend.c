// sentai_camera_frame_backend.c -- shared camera frame publication helpers.
//
// This is runtime code, not a Gazebo simulator bridge.  It owns the common
// "latest RGB/XRGB frame -> PrepTask slots" boundary used by deterministic
// virtual-camera frames and by simulator camera providers.  Platform-specific
// producers inject RGB888 frames through sentai_camera_backend_publish_rgb888().

#include "examples/sentai_runtime/sentai_prep.h"
#include "examples/sentai_runtime/sentai_virtual_camera.h"
#include "examples/sentai_runtime/flow_shared.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SENTAI_CAMERA_FRAME_W 640
#define SENTAI_CAMERA_FRAME_H 480
#define SENTAI_CAMERA_FLOW_W  80
#define SENTAI_CAMERA_FLOW_H  60

extern int sentai_get_tensor_info(int* w, int* h, int* ch,
                                  uint8_t** buf, int* type, int* zp);
extern int sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                            uint8_t* dst, int dw, int dh);
extern int sentai_prep_publish_slot_rgb_64(const uint8_t* raw_xrgb,
                                           int src_w, int src_h);
extern void sentai_flow_poll_once(void);
extern int sentai_virtual_camera_publish_xrgb(uint32_t seq, int cam_id,
                                              const uint8_t* xrgb);
extern void sentai_virtual_camera_return_raw(int idx);
extern int sentai_virtual_camera_is_frame_idx(int idx);
extern int sentai_virtual_camera_grabbed_id(void);
extern int sentai_virtual_camera_current_id(void);

typedef struct {
    volatile uint32_t seq;
    volatile int32_t  dx_q1000;
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t  dz_q1000;
    volatile uint32_t dz_conf;
} sim_flow_snapshot_t;

static sim_flow_snapshot_t s_flow_snapshot;

static uint8_t s_rgb_full_pub[SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H * 3];
static volatile uint32_t s_rgb_full_seq;
static uint32_t s_last_grab_seq;

static uint8_t s_xrgb_buf[SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H * 4];
static uint8_t s_prep_rgb_full[SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H * 3];
static uint8_t s_rgb_small[SENTAI_CAMERA_FLOW_W * SENTAI_CAMERA_FLOW_H * 3];
static uint8_t s_gray80x60[SENTAI_CAMERA_FLOW_W * SENTAI_CAMERA_FLOW_H];
static uint32_t s_prep_fire_mask;
static volatile int s_current_id = 0;
static volatile int s_last_capture_id = -1;
static volatile int s_grabbed_id = -1;
static StaticSemaphore_t s_frame_sem_buf;
static SemaphoreHandle_t s_frame_sem;

static SemaphoreHandle_t frame_sem(void) {
    if (!s_frame_sem) {
        s_frame_sem = xSemaphoreCreateBinaryStatic(&s_frame_sem_buf);
    }
    return s_frame_sem;
}

static void rgb888_to_xrgb8888(const uint8_t* rgb, uint8_t* xrgb,
                               int n_pixels) {
    for (int i = 0; i < n_pixels; ++i) {
        uint8_t r = rgb[i * 3 + 0];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        xrgb[i * 4 + 0] = b;
        xrgb[i * 4 + 1] = g;
        xrgb[i * 4 + 2] = r;
        xrgb[i * 4 + 3] = 0xFF;
    }
}

static void rgb888_to_y(const uint8_t* rgb, uint8_t* y, int n_pixels) {
    for (int i = 0; i < n_pixels; ++i) {
        uint8_t r = rgb[i * 3 + 0];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        y[i] = (uint8_t)((77u * r + 150u * g + 29u * b) >> 8);
    }
}

static void publish_prep_slots_from_xrgb(const uint8_t* xrgb) {
    const uint32_t fire_mask = sentai_prep_tick_frame();
    s_prep_fire_mask = fire_mask;
    if (fire_mask & (1u << SENTAI_PREP_SLOT_RGB_64)) {
        (void)sentai_prep_publish_slot_rgb_64(xrgb, SENTAI_CAMERA_FRAME_W,
                                              SENTAI_CAMERA_FRAME_H);
    }
    if (fire_mask & (1u << SENTAI_PREP_SLOT_TPU_RGB)) {
        int tw = 0, th = 0, tch = 0, ttype = 0, tzp = 0;
        uint8_t* tensor = NULL;
        if (sentai_get_tensor_info(&tw, &th, &tch, &tensor,
                                   &ttype, &tzp) == 0 &&
                tch == 3 && tw > 0 && th > 0 &&
                tw <= SENTAI_CAMERA_FRAME_W &&
                th <= SENTAI_CAMERA_FRAME_H) {
            int sw = 0, sh = 0;
            uint8_t* sbuf = sentai_prep_slot_begin_write(
                SENTAI_PREP_SLOT_TPU_RGB, &sw, &sh);
            if (sbuf && tw <= sw && th <= sh &&
                    sentai_pxp_scale(xrgb, SENTAI_CAMERA_FRAME_W,
                                     SENTAI_CAMERA_FRAME_H,
                                     sbuf, tw, th) == 0) {
                sentai_prep_slot_commit_dims(SENTAI_PREP_SLOT_TPU_RGB,
                                             tw, th);
            }
        }
    }
}

static void publish_prep_flow_gray80x60(const uint8_t* gray) {
    if (!(s_prep_fire_mask & (1u << SENTAI_PREP_SLOT_FLOW_GRAY_80x60))) {
        return;
    }
    int sw = 0, sh = 0;
    uint8_t* sbuf = sentai_prep_slot_begin_write(
        SENTAI_PREP_SLOT_FLOW_GRAY_80x60, &sw, &sh);
    if (!sbuf || sw != SENTAI_CAMERA_FLOW_W || sh != SENTAI_CAMERA_FLOW_H) {
        return;
    }
    memcpy(sbuf, gray, SENTAI_CAMERA_FLOW_W * SENTAI_CAMERA_FLOW_H);
    sentai_prep_slot_commit(SENTAI_PREP_SLOT_FLOW_GRAY_80x60);
}

static void publish_xrgb_frame(uint32_t seq, const uint8_t* xrgb) {
    s_rgb_full_seq = seq ? seq : (s_rgb_full_seq + 1);
    publish_prep_slots_from_xrgb(xrgb);
    if (s_prep_fire_mask & (1u << SENTAI_PREP_SLOT_FLOW_GRAY_80x60)) {
        if (sentai_pxp_scale(xrgb, SENTAI_CAMERA_FRAME_W,
                             SENTAI_CAMERA_FRAME_H,
                             s_rgb_small, SENTAI_CAMERA_FLOW_W,
                             SENTAI_CAMERA_FLOW_H) == 0) {
            rgb888_to_y(s_rgb_small, s_gray80x60,
                        SENTAI_CAMERA_FLOW_W * SENTAI_CAMERA_FLOW_H);
            publish_prep_flow_gray80x60(s_gray80x60);
        }
    }
    SemaphoreHandle_t sem = frame_sem();
    if (sem) {
        xSemaphoreGive(sem);
    }
}

extern int sentai_camera_wait_frame(int timeout_ms) {
    SemaphoreHandle_t sem = frame_sem();
    if (!sem) return -1;
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY :
                       pdMS_TO_TICKS((uint32_t)timeout_ms);
    return (xSemaphoreTake(sem, ticks) == pdTRUE) ? 0 : -2;
}

extern int sentai_camera_backend_publish_rgb888(uint32_t seq,
                                                const uint8_t* rgb,
                                                size_t bytes) {
    const size_t need = SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H * 3u;
    if (!rgb || bytes != need) return -1;

    memcpy(s_rgb_full_pub, rgb, need);
    __sync_synchronize();
    s_rgb_full_seq = seq ? seq : (s_rgb_full_seq + 1);
    s_last_capture_id = s_current_id;

    rgb888_to_xrgb8888(rgb, s_xrgb_buf,
                       SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H);
    return sentai_virtual_camera_publish_xrgb(s_rgb_full_seq, s_current_id,
                                              s_xrgb_buf);
}

extern size_t sim_camera_latest_rgb(uint8_t* dst, size_t max_bytes,
                                    int* out_w, int* out_h,
                                    uint32_t* out_seq) {
    size_t vgot = sentai_virtual_camera_get_rgb(dst, max_bytes,
                                                out_w, out_h, out_seq);
    if (vgot > 0) return vgot;

    if (out_w) *out_w = SENTAI_CAMERA_FRAME_W;
    if (out_h) *out_h = SENTAI_CAMERA_FRAME_H;
    const size_t need = SENTAI_CAMERA_FRAME_W * SENTAI_CAMERA_FRAME_H * 3u;
    if (!dst || max_bytes < need || s_rgb_full_seq == 0) {
        if (out_seq) *out_seq = 0;
        return 0;
    }
    uint32_t s0 = s_rgb_full_seq;
    memcpy(dst, s_rgb_full_pub, need);
    uint32_t s1 = s_rgb_full_seq;
    if (s1 != s0) {
        memcpy(dst, s_rgb_full_pub, need);
        s0 = s_rgb_full_seq;
    }
    if (out_seq) *out_seq = s0;
    return need;
}

extern int sentai_camera_backend_publish_prep_once(void) {
    if (sentai_virtual_camera_active()) {
        uint8_t* raw = NULL;
        int idx = sentai_virtual_camera_grab_xrgb(&raw);
        if (sentai_virtual_camera_is_frame_idx(idx) && raw != NULL) {
            publish_xrgb_frame(sentai_virtual_camera_seq(), raw);
            sentai_virtual_camera_return_raw(idx);
            return 0;
        }
    }
    int w = 0, h = 0;
    uint32_t seq = 0;
    size_t got = sim_camera_latest_rgb(s_prep_rgb_full,
                                       sizeof(s_prep_rgb_full),
                                       &w, &h, &seq);
    if (got == 0 || w != SENTAI_CAMERA_FRAME_W ||
            h != SENTAI_CAMERA_FRAME_H) {
        return -1;
    }
    return sentai_camera_backend_publish_rgb888(
        seq ? seq : sentai_virtual_camera_seq(), s_prep_rgb_full, got);
}

extern int sentai_camera_backend_publish_flow_once(void) {
    int rc = sentai_camera_backend_publish_prep_once();
    if (rc != 0) return rc;
    sentai_flow_poll_once();
    return 0;
}

extern void sentai_virtual_camera_after_select(void) {
    // A provider has committed a frame into the virtual/common frame queue.
    // Wake consumers exactly like a camera frame-ready event; PrepTask owns
    // resize/slot publication after it grabs the frame.
    SemaphoreHandle_t sem = frame_sem();
    if (sem) {
        xSemaphoreGive(sem);
    }
}

extern int sentai_cam_is_initialized(void) {
    return 1;
}

extern int sentai_cam_grab_latest(uint8_t** raw) {
    if (!raw) return -1;
    int idx = sentai_virtual_camera_grab_xrgb(raw);
    if (idx < 0 || !*raw) return -2;
    s_rgb_full_seq = sentai_virtual_camera_seq();
    s_last_grab_seq = s_rgb_full_seq;
    s_grabbed_id = sentai_virtual_camera_grabbed_id();
    return idx;
}

extern void sentai_cam_return_raw(int idx) {
    sentai_virtual_camera_return_raw(idx);
}

extern uint32_t sentai_cam_get_frame_seq(void) {
    return s_rgb_full_seq;
}

extern uint32_t sentai_cam_get_sensor_frames(void) {
    return s_rgb_full_seq;
}

extern int sentai_cam_current_id(void) {
    return sentai_virtual_camera_active() ? sentai_virtual_camera_current_id() :
           (int)s_current_id;
}

extern int sentai_cam_last_capture_id(void) {
    return (int)s_last_capture_id;
}

extern int sentai_cam_grabbed_id(void) {
    return (int)s_grabbed_id;
}

extern int sentai_cam_test_pattern(int cam_id, int mode) {
    (void)cam_id;
    (void)mode;
    return -1;
}

static uint32_t s_cam_ratio_a;
static uint32_t s_cam_ratio_b;
volatile uint32_t g_cam_ratio_packed;

extern int sentai_cam_ratio_set(uint32_t a, uint32_t b) {
    s_cam_ratio_a = a;
    s_cam_ratio_b = b;
    g_cam_ratio_packed = (a << 16) | (b & 0xffffu);
    return 0;
}

extern void sentai_cam_ratio_get(uint32_t* a, uint32_t* b) {
    if (a) *a = s_cam_ratio_a;
    if (b) *b = s_cam_ratio_b;
}

extern const sim_flow_snapshot_t* sim_camera_flow_snapshot(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic == FLOW_SHARED_MAGIC) {
        s_flow_snapshot.seq = sh->last_frame_seq;
        s_flow_snapshot.dx_q1000 = sh->last_dx;
        s_flow_snapshot.dy_q1000 = sh->last_dy;
        s_flow_snapshot.conf = sh->last_confidence;
        s_flow_snapshot.latency_us = sh->last_compute_us;
        s_flow_snapshot.dz_q1000 = 0;
        s_flow_snapshot.dz_conf = 0;
    }
    return &s_flow_snapshot;
}
