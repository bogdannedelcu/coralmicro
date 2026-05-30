// camera_bridge_recv.c — SIM-only Unix-domain-socket camera receiver task.
//
// Architecture (Phase 4):
//
//   Gazebo Harmonic publishes /downward_cam/image (RGB888 640x480 @ 30fps)
//        |
//        v
//   sim/scripts/gz_to_camera_bridge.py  (subscribes via gz transport,
//                                         no per-pixel work, just I/O)
//        |  UDS SOCK_STREAM @ /tmp/sentai_cam.sock
//        v
//   THIS FILE                            (FreeRTOS task in sentai_sim)
//        |
//        |  expand RGB888 -> XRGB8888 in-place buffer
//        |  call sentai_pxp_scale 640x480 -> 80x60 RGB888  (shared API, identical to ARM)
//        |  RGB888 -> grayscale Y plane (BT.601 luma)
//        |  call sentai_flow_phase_corr_compute()           (shared API, identical to ARM)
//        v
//   g_flow_shared    (consumed by sentai.flow.read() MicroPython binding)
//
// Why a single C task and not a Python multi-process pipeline:
//   - keeps every per-pixel byte in C (per the realtime rule)
//   - reuses the EXACT firmware entry points (sentai_pxp_scale +
//     sentai_flow_phase_corr_compute) so SIM ↔ ARM parity is enforced
//     by the type system, not by hand-mirrored Python copies
//   - one process owns flow_shared_t, no IPC inside the SIM
//
// Wire protocol on the UDS (half-duplex, request/response per frame):
//
//   sender -> us  (header + payload):
//     uint32_t magic   = 0x53434D31 ('SCM1' = SentAI Camera Msg v1)
//     uint32_t seq     = monotonic frame counter from sender
//     uint32_t width   = 640
//     uint32_t height  = 480
//     uint32_t pix_fmt = 0  (0 = RGB888 packed; reserved for future YUV/etc.)
//     uint32_t payload_bytes = width * height * 3
//     uint8_t  payload[payload_bytes]
//
//   us -> sender  (flow snapshot — exactly one reply per accepted frame):
//     uint32_t reply_magic = 0x46524C31 ('FRL1' = Flow Reply v1)
//     uint32_t seq         = mirrored from request (lets sender match)
//     int32_t  dx_q1000    = milli-grid-pixel motion along image-X
//     int32_t  dy_q1000    = milli-grid-pixel motion along image-Y
//     uint32_t conf        = phase-corr confidence (0..255)
//     uint64_t latency_us  = recv -> publish wall time
//
// All multi-byte fields little-endian.  No length-prefix-only framing —
// the magic+header lets us resync if a sender crashes mid-stream.
// Doing the reply on the same socket avoids a second UDS path and keeps
// the bridge a single-process Python program (gz_to_camera_bridge.py).
//
// Bounded everything (embeded.md §4): if header.width/height ≠ expected
// or payload_bytes overshoots the static buffer, we drain and resync.
// No dynamic alloc in the hot path.
//
// Build gate: only compiled when SENTAI_PLATFORM_SIM is defined.

#ifndef SENTAI_PLATFORM_SIM
#error "camera_bridge_recv.c is SIM-only — guard via sim/CMakeLists.txt"
#endif

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <math.h>     // floorf for warp_translate_bilinear
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include "examples/sentai_runtime/sentai_pxp_shim.h"
#include "examples/sentai_runtime/sentai_prep.h"     // W11-T4: SLOT_RGB_64 producer
#include "examples/sentai_runtime/sentai_virtual_camera.h"
#include "examples/sentai_runtime/slam_task.h"       // W11-T4: publish helper

// flow_phase_corr public entry — defined in flow_phase_corr.cc.
extern void sentai_flow_phase_corr_compute_at(int pipe_id,
                                                const uint8_t* gray80x60,
                                                int* dx_q1000_out,
                                                int* dy_q1000_out,
                                                uint8_t* conf_out);
extern void sentai_flow_phase_corr_set_anchor(int pipe_id,
                                                const uint8_t* gray80x60);
extern void sentai_flow_phase_corr_compute_against_anchor(int pipe_id,
                                                            const uint8_t* gray80x60,
                                                            int* dx_q1000_out,
                                                            int* dy_q1000_out,
                                                            uint8_t* conf_out);
extern void sentai_flow_phase_corr_compute(const uint8_t* gray80x60,
                                            int* dx_q1000_out,
                                            int* dy_q1000_out,
                                            uint8_t* conf_out);
// dz divergence path (added 2026-05-11) — runs 4 sub-block phase-corrs
// to estimate altitude rate (drone rising/falling) from radial flow
// expansion/contraction.  Diagnostic only — cf2 EKF doesn't consume it.
extern void sentai_flow_phase_corr_compute_dz(const uint8_t* gray80x60,
                                                int* dz_q1000_out,
                                                uint8_t* conf_out);

// ────────────────────────────────────────────────────────────────────────
// Public flow snapshot — single-writer (this task), many-reader (REPL).
// volatile + word-sized fields make atomic snapshot safe on x86 without a
// mutex.  Sequence counter wraps; consumers compare to detect new data.
// ────────────────────────────────────────────────────────────────────────
typedef struct {
    volatile uint32_t seq;          // bumps per published frame
    volatile int32_t  dx_q1000;     // milli-grid-pixels (1000 = 1 grid-px)
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;         // peak/mean ratio (capped 0..255)
    volatile uint64_t latency_us;   // recv -> publish wall time
    volatile int32_t  dz_q1000;     // µ/frame altitude rate (diagnostic)
    volatile uint32_t dz_conf;
} sim_flow_snapshot_t;

static sim_flow_snapshot_t g_flow = {0};

const sim_flow_snapshot_t* sim_camera_flow_snapshot(void) {
    return &g_flow;
}

// (sim_camera_latest_rgb defined below, after EXPECT_W/H macros.)

// ────────────────────────────────────────────────────────────────────────
// Static buffers — sized for 640x480 RGB and 80x60 RGB+gray.
// Keeping them static avoids any allocator in the per-frame path.
// ────────────────────────────────────────────────────────────────────────
#define EXPECT_W   640
#define EXPECT_H   480
#define DST_W      80
#define DST_H      60

// ────────────────────────────────────────────────────────────────────────
// Latest-RGB-frame getter for sentai.pipeline (Phase 5.6).
// Copies the most recent 640x480 RGB888 frame from the bridge into the
// caller's buffer.  Returns the byte count copied (0 if no frame yet).
// Thread-safe via the seq fence used elsewhere — caller may see one
// frame older than current if the bridge is mid-write.
// ────────────────────────────────────────────────────────────────────────
static uint8_t  s_rgb_full_pub[EXPECT_W * EXPECT_H * 3] = {0};
static volatile uint32_t s_rgb_full_seq = 0;

size_t sim_camera_latest_rgb(uint8_t* dst, size_t max_bytes,
                              int* out_w, int* out_h, uint32_t* out_seq) {
    size_t vgot = sentai_virtual_camera_get_rgb(dst, max_bytes,
                                                out_w, out_h, out_seq);
    if (vgot > 0) return vgot;

    if (out_w) *out_w = EXPECT_W;
    if (out_h) *out_h = EXPECT_H;
    const size_t need = EXPECT_W * EXPECT_H * 3u;
    if (!dst || max_bytes < need || s_rgb_full_seq == 0) {
        if (out_seq) *out_seq = 0;
        return 0;
    }
    uint32_t s0 = s_rgb_full_seq;
    memcpy(dst, s_rgb_full_pub, need);
    uint32_t s1 = s_rgb_full_seq;
    if (s1 != s0) {
        // Bridge wrote during the copy; redo once.  Bounded one retry.
        memcpy(dst, s_rgb_full_pub, need);
        s0 = s_rgb_full_seq;
    }
    if (out_seq) *out_seq = s0;
    return need;
}

static uint8_t s_xrgb_buf[EXPECT_W * EXPECT_H * 4];   // 1.2 MB
static uint8_t s_rgb_small[DST_W * DST_H * 3];        // 14.4 KB
static uint8_t s_gray80x60[DST_W * DST_H];            // 4.8 KB  — L0 wide (8×)
static uint8_t s_gray80x60_center[DST_W * DST_H];     // 4.8 KB  — L1 mid (4× box)
static uint8_t s_gray80x60_fine[DST_W * DST_H];       // 4.8 KB  — L2 fine (2× box)
static uint8_t s_gray80x60_warped[DST_W * DST_H];     // 4.8 KB  — warp scratch

#define SOCK_PATH "/tmp/sentai_cam.sock"

#define MAGIC       0x53434D31u  // 'SCM1' — request from sender
#define REPLY_MAGIC 0x46524C31u  // 'FRL1' — flow snapshot reply

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t width;
    uint32_t height;
    uint32_t pix_fmt;
    uint32_t payload_bytes;
} cam_header_t;

typedef struct __attribute__((packed)) {
    uint32_t reply_magic;
    uint32_t seq;
    // L0 / WIDE pipeline — 640×480 → 80×60 via 8× decimation.
    // Per-grid 13.85 mm @ z=1m, FOV 1.1m.
    int32_t  dx_q1000;
    int32_t  dy_q1000;
    uint32_t conf;
    uint64_t latency_us;
    // dz divergence (sub-block flow, altitude-rate proxy).
    int32_t  dz_q1000;
    uint32_t dz_conf;
    // L1 / MID pipeline — 320×240 → 80×60 via 4× decimation.
    // Per-grid 6.93 mm @ z=1m, FOV 0.55m.  Equivalent to wide_mgrid × 2
    // for ground-velocity conversion.  Field name kept as "center" for
    // wire-protocol compat with the previous 2-pipeline version, but
    // SEMANTICS CHANGED — these are now L1, not the native-pixel center.
    int32_t  dx_center_q1000;
    int32_t  dy_center_q1000;
    uint32_t conf_center;
    // L2 / FINE pipeline — 160×120 → 80×60 via 2× decimation.
    // Per-grid 3.46 mm @ z=1m, FOV 0.27m.  Equivalent to wide_mgrid × 4
    // for ground-velocity conversion.  Added 2026-05-11 as 3rd level
    // of Burt-Adelson pyramid (user request "pornind de la 3 rezolutii").
    int32_t  dx_fine_q1000;
    int32_t  dy_fine_q1000;
    uint32_t conf_fine;
    // LastChangedFrame (LCF) anchor — pipe 3.  Reports CUMULATIVE motion
    // (in L0 mgrid units) since anchor was last refreshed.  Refresh
    // triggered by instantaneous motion exceeding MOTION_DETECT_THRESH.
    // Caller converts to velocity: vx_avg = dx_anchor / frames_since_anchor.
    // Sub-pixel slow drift becomes detectable after enough accumulation.
    int32_t  dx_anchor_q1000;
    int32_t  dy_anchor_q1000;
    uint32_t conf_anchor;
    uint32_t frames_since_anchor;
    // L2-NATIVE anchor (independent of L0 anchor) — cumulative drift
    // at native pixel resolution.  Native mgrid units (8× finer than L0).
    int32_t  dx_anchor_L2_q1000;
    int32_t  dy_anchor_L2_q1000;
    uint32_t conf_anchor_L2;
    uint32_t frames_since_anchor_L2;
    // L1-MID anchor — added 2026-05-11.
    int32_t  dx_anchor_L1_q1000;
    int32_t  dy_anchor_L1_q1000;
    uint32_t conf_anchor_L1;
    uint32_t frames_since_anchor_L1;
    // FUSION WINNER — chosen in C side (not Python).  On ARM/HW this
    // is what sentai.flow.read() returns; MicroPython/Python just
    // relay to cf2 EKF.  Selection rule below in handle_one_frame.
    int32_t  dx_best_q1000;
    int32_t  dy_best_q1000;
    uint32_t conf_best;
    uint8_t  best_source;        // 0=L0, 1=L1, 2=L2, 3=ANCHOR
    uint8_t  _pad[3];
} flow_reply_t;

// ────────────────────────────────────────────────────────────────────────
// I/O helpers — bounded read/write loops, EINTR-safe (POSIX port quirk).
// ────────────────────────────────────────────────────────────────────────
static int read_full(int fd, void* dst, size_t n) {
    uint8_t* p = (uint8_t*)dst;
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r > 0) { got += (size_t)r; continue; }
        if (r < 0 && errno == EINTR) continue;
        return -1;  // EOF or hard error
    }
    return 0;
}

static int write_full(int fd, const void* src, size_t n) {
    const uint8_t* p = (const uint8_t*)src;
    size_t put = 0;
    while (put < n) {
        ssize_t w = write(fd, p + put, n - put);
        if (w > 0) { put += (size_t)w; continue; }
        if (w < 0 && errno == EINTR) continue;
        return -1;
    }
    return 0;
}

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

// Expand packed RGB888 [R,G,B,R,G,B,...] into XRGB8888 [B,G,R,X,...]
// (matches OV5640 CSI receiver byte order, what sentai_pxp_scale expects).
static void rgb888_to_xrgb8888(const uint8_t* rgb, uint8_t* xrgb, int n_pixels) {
    for (int i = 0; i < n_pixels; ++i) {
        uint8_t R = rgb[i * 3 + 0];
        uint8_t G = rgb[i * 3 + 1];
        uint8_t B = rgb[i * 3 + 2];
        xrgb[i * 4 + 0] = B;
        xrgb[i * 4 + 1] = G;
        xrgb[i * 4 + 2] = R;
        xrgb[i * 4 + 3] = 0xFF;
    }
}

// BT.601 luma (Y = 0.299R + 0.587G + 0.114B) on packed RGB888 input.
// Integer fixed-point: Y = (77*R + 150*G + 29*B) >> 8.
static void rgb888_to_y(const uint8_t* rgb, uint8_t* y, int n_pixels) {
    for (int i = 0; i < n_pixels; ++i) {
        uint8_t R = rgb[i * 3 + 0];
        uint8_t G = rgb[i * 3 + 1];
        uint8_t B = rgb[i * 3 + 2];
        y[i] = (uint8_t)((77u * R + 150u * G + 29u * B) >> 8);
    }
}

// 3-LEVEL BURT-ADELSON PYRAMID — all output 80×60 gray, each level
// pulls a different-sized centred patch from the raw 640×480 RGB
// frame and box-filter downsamples to 80×60.  Same phase-corr engine
// runs on all three levels.  Consumer fuses by confidence + scale.
//
// At z=1m drone hover:
//   L0  (8× decimation, current wide): per-grid 13.85 mm, FOV 1.10m
//   L1  (4× decimation): per-grid  6.93 mm, FOV 0.55m
//   L2  (2× decimation): per-grid  3.46 mm, FOV 0.27m
//
// Each finer level loses FOV but doubles sub-pixel sensitivity AND
// retains ≥4 pixels/grid-cell averaging → SNR comparable to L0.
//
// Box-filter R rxR_to_gray: read R×R block of RGB, average to 1 gray
// pixel using BT.601 luma weights (77, 150, 29) and >>8 rescale.
// P1 (PX4Flow-style): pre-screen texture quality before running phase-corr.
// Returns a metric (sum of |grad_x| + |grad_y|) over the 80×60 gray buffer.
// If below threshold the scene is "textureless" — phase-corr peak will be
// random/noisy.  Skip and return conf=0 instead of wasting ~5ms on it.
//
// Cost: 80×60×2 = 9600 subtractions + 9600 abs + 9600 adds = ~30K ops
// = ~50µs ARM.  Saves ~5ms when image is uniform.
static uint32_t compute_texture_quality(const uint8_t* gray, int w, int h) {
    uint32_t grad_sum = 0;
    // Horizontal gradient (skip last col)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            int g = (int)gray[y * w + x + 1] - (int)gray[y * w + x];
            grad_sum += (g < 0) ? -g : g;
        }
    }
    // Vertical gradient (skip last row)
    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w; ++x) {
            int g = (int)gray[(y + 1) * w + x] - (int)gray[y * w + x];
            grad_sum += (g < 0) ? -g : g;
        }
    }
    return grad_sum;
}

// P3: 4-corner SAD validation.  After global FFT phase-corr produces a
// motion estimate, sample 4 small 16×16 patches at image corners + run
// SAD search ±4 px around the global estimate.  If majority disagree by
// > 2 pixels, the global estimate is suspect (textureless centre, false
// peak).  Returns: 1 = consensus, 0 = outlier detected.
//
// Cost: 4 patches × (16×16 × 9×9 search) = 83K ops = ~100µs ARM.
static int validate_motion_4corners(const uint8_t* gray_curr,
                                     const uint8_t* gray_prev,
                                     int w, int h,
                                     int dx_global, int dy_global) {
    // 4 patch centers near corners (avoid edges by margin 12).
    const int patch_sz = 16;
    const int margin = patch_sz + 4;
    const int corners[4][2] = {
        {margin,      margin},      // top-left
        {w - margin,  margin},      // top-right
        {margin,      h - margin},  // bottom-left
        {w - margin,  h - margin},  // bottom-right
    };
    int agree_count = 0;
    for (int c = 0; c < 4; ++c) {
        int cx = corners[c][0];
        int cy = corners[c][1];
        // Reference 16×16 from curr at (cx-8, cy-8)
        uint32_t best_sad = UINT32_MAX;
        int best_dx = 0, best_dy = 0;
        for (int sy = -4; sy <= 4; ++sy) {
            for (int sx = -4; sx <= 4; ++sx) {
                int rx = cx + dx_global + sx;
                int ry = cy + dy_global + sy;
                if (rx - 8 < 0 || rx + 8 > w || ry - 8 < 0 || ry + 8 > h) continue;
                uint32_t sad = 0;
                for (int dy = -8; dy < 8; ++dy) {
                    for (int dx = -8; dx < 8; ++dx) {
                        int a = gray_curr[(cy + dy) * w + (cx + dx)];
                        int b = gray_prev[(ry + dy) * w + (rx + dx)];
                        int d = a - b;
                        sad += (d < 0) ? -d : d;
                    }
                }
                if (sad < best_sad) {
                    best_sad = sad;
                    best_dx = sx;
                    best_dy = sy;
                }
            }
        }
        // Corner says global+best is correct if |sx|,|sy| ≤ 2 (strict).
        // Empirically: strict 2px gave 53.6% all-4, relaxed 3px regressed.
        if ((best_dx >= -2 && best_dx <= 2) && (best_dy >= -2 && best_dy <= 2)) {
            agree_count++;
        }
    }
    return (agree_count >= 3) ? 1 : 0;   // need 3/4 corners agreeing
}

// NATIVE crop — take center 80×60 pixels from raw 640×480, NO decimation.
// At z=1m, this gives per-pixel ground resolution = 1.73mm (8× better
// than L0 wide).  Smaller FOV (14×10cm) but enough for hover-over-markers
// since markers cluster at center.  Same compute cost as box-filters
// (still produces 80×60 gray buffer).
//
// Replaces previous 2× box-filter which had limited sub-pixel benefit
// (per-grid still 3.46mm — couldn't detect slow drift < 4.8cm/s).
static void crop_center_native_to_gray(const uint8_t* rgb_full,
                                        int full_w, int full_h,
                                        uint8_t* y_out,
                                        int out_w, int out_h) {
    // Center 80×60 pixels — zero decimation, raw camera resolution.
    const int x_start = (full_w - out_w) / 2;
    const int y_start = (full_h - out_h) / 2;
    for (int row = 0; row < out_h; ++row) {
        const uint8_t* src = rgb_full + ((y_start + row) * full_w + x_start) * 3;
        uint8_t* dst = y_out + row * out_w;
        for (int col = 0; col < out_w; ++col) {
            uint8_t R = src[col * 3 + 0];
            uint8_t G = src[col * 3 + 1];
            uint8_t B = src[col * 3 + 2];
            dst[col] = (uint8_t)((77u * R + 150u * G + 29u * B) >> 8);
        }
    }
}

// Bilinear translation warp — shift gray image by (dx, dy) in pixels.
// Pixels that go out of bounds are clamped (replicate edge).  Used for
// the coarse-to-fine pyramid: after L0 finds coarse motion, the L1 input
// is "pre-shifted" so phase-corr on the warped frame finds only the
// small residual motion that L0 missed.
//
// Inputs in pixel units of the SAME resolution as src/dst.  Sub-pixel
// dx, dy are handled by bilinear interpolation.
static void warp_translate_bilinear(const uint8_t* src, uint8_t* dst,
                                     int w, int h,
                                     float dx, float dy) {
    // For each output pixel (x, y), sample input at (x + dx, y + dy).
    // Negative dx,dy means: output[x,y] = input[x+dx, y+dy], i.e., the
    // image "shifts" so feature at input(x+dx) appears at output(x).
    for (int y = 0; y < h; ++y) {
        float sy = (float)y + dy;
        int y0 = (int)floorf(sy);
        float fy = sy - (float)y0;
        if (y0 < 0)   { y0 = 0;   fy = 0.0f; }
        if (y0 >= h-1) { y0 = h-2; fy = 1.0f; }
        int y1 = y0 + 1;
        for (int x = 0; x < w; ++x) {
            float sx = (float)x + dx;
            int x0 = (int)floorf(sx);
            float fx = sx - (float)x0;
            if (x0 < 0)   { x0 = 0;   fx = 0.0f; }
            if (x0 >= w-1) { x0 = w-2; fx = 1.0f; }
            int x1 = x0 + 1;
            float a = (float)src[y0 * w + x0];
            float b = (float)src[y0 * w + x1];
            float c = (float)src[y1 * w + x0];
            float d = (float)src[y1 * w + x1];
            float top = a * (1.0f - fx) + b * fx;
            float bot = c * (1.0f - fx) + d * fx;
            float v = top * (1.0f - fy) + bot * fy;
            int iv = (int)(v + 0.5f);
            if (iv < 0) iv = 0;
            if (iv > 255) iv = 255;
            dst[y * w + x] = (uint8_t)iv;
        }
    }
}

static void boxfilter_4x_rgb_to_gray(const uint8_t* rgb_full,
                                      int full_w, int full_h,
                                      uint8_t* y_out,
                                      int out_w, int out_h) {
    // 4× box filter: 16 input pixels (4×4) per output pixel.
    const int patch_w = out_w * 4;
    const int patch_h = out_h * 4;
    const int x_start = (full_w - patch_w) / 2;
    const int y_start = (full_h - patch_h) / 2;
    for (int row = 0; row < out_h; ++row) {
        for (int col = 0; col < out_w; ++col) {
            uint32_t sum = 0;
            for (int dy = 0; dy < 4; ++dy) {
                const uint8_t* src = rgb_full +
                    ((y_start + row * 4 + dy) * full_w + x_start + col * 4) * 3;
                for (int dx = 0; dx < 4; ++dx) {
                    sum += 77u * src[dx * 3 + 0] + 150u * src[dx * 3 + 1]
                         +  29u * src[dx * 3 + 2];
                }
            }
            // 16 pixels × 256 max (luma weights sum) × 255 max value =
            // 16 × 256 × 255 ≈ 1.04M.  >>12 = >>(8+4) keeps result in uint8.
            y_out[row * out_w + col] = (uint8_t)(sum >> 12);
        }
    }
}

// One frame: receive header + payload, downsample, compute flow, publish.
// Returns 0 on success, -1 on protocol error (caller resyncs/closes).
static int handle_one_frame(int fd) {
    cam_header_t hdr;
    if (read_full(fd, &hdr, sizeof(hdr)) != 0) return -1;

    if (hdr.magic != MAGIC) {
        printf("camera_bridge: bad magic 0x%08x, dropping connection\r\n", hdr.magic);
        return -1;
    }
    if (hdr.width != EXPECT_W || hdr.height != EXPECT_H || hdr.pix_fmt != 0) {
        printf("camera_bridge: unexpected geometry %ux%u fmt=%u\r\n",
               hdr.width, hdr.height, hdr.pix_fmt);
        return -1;
    }
    const size_t expected_bytes = (size_t)EXPECT_W * EXPECT_H * 3u;
    if (hdr.payload_bytes != expected_bytes) {
        printf("camera_bridge: payload mismatch got=%u want=%zu\r\n",
               hdr.payload_bytes, expected_bytes);
        return -1;
    }

    // Reuse a single static rgb_full buffer (the input side of the chain
    // is RGB; we expand into s_xrgb_buf then resize via PXP shim).  Read
    // straight into the rgb path of XRGB by aliasing — but easier: read
    // into a temp on stack-spilled static and convert in one pass.
    //
    // Total: 921 600 bytes per frame; static lives in BSS.
    static uint8_t s_rgb_full[EXPECT_W * EXPECT_H * 3];
    if (read_full(fd, s_rgb_full, expected_bytes) != 0) return -1;

    // Publish a copy for sentai.pipeline.tick() (Phase 5.6).  Write
    // payload first then bump seq — same pattern as g_flow.  640x480
    // copy ~1ms — only happens per fresh frame from Garden.
    memcpy(s_rgb_full_pub, s_rgb_full, expected_bytes);
    __sync_synchronize();
    s_rgb_full_seq = hdr.seq;

    uint64_t t0 = now_us();

    // DEBUG: simple xor-checksum of the raw RGB frame to confirm frames
    // arriving from Gazebo are actually changing.  Print every 30 frames.
    {
        static uint32_t s_prev_rgb_crc = 0;
        static int s_dbg_log = 0;
        uint32_t crc = 0;
        for (size_t i = 0; i < expected_bytes; i += 19) {
            crc = crc * 31u + s_rgb_full[i];
        }
        if ((s_dbg_log++ % 30) == 0) {
            printf("camera_bridge: rgb_crc=0x%08x (prev=0x%08x, %s)\r\n",
                   crc, s_prev_rgb_crc,
                   (crc == s_prev_rgb_crc) ? "STATIC" : "CHANGED");
        }
        s_prev_rgb_crc = crc;
    }

    // Raw 640×480 RGB dump — env SENTAI_DUMP_RAW_EVERY (default 6 if
    // SENTAI_DUMP_FRAMES_DIR set, 0 = off).  ~921 KB per file; needed
    // for host-side ArUco PnP at 5 Hz sample rate.  Distinct from L0
    // 80×60 dump below (per-frame, cheap, for inspection).
    {
        static int s_raw_init = 0;
        static const char* s_raw_dir = NULL;
        static int s_raw_every = 0;
        if (!s_raw_init) {
            s_raw_init = 1;
            s_raw_dir = getenv("SENTAI_DUMP_FRAMES_DIR");
            const char* en = getenv("SENTAI_DUMP_RAW_EVERY");
            if (en && *en) {
                int v = atoi(en);
                if (v >= 0) s_raw_every = v;
            } else if (s_raw_dir) {
                s_raw_every = 6;   // default cadence for 5 Hz ArUco sampling
            }
            if (s_raw_dir) {
                printf("camera_bridge: dump dir=%s raw_every=%d\r\n",
                       s_raw_dir, s_raw_every);
            }
        }
        if (s_raw_dir && s_raw_every > 0 && (hdr.seq % s_raw_every) == 0) {
            char path[512];
            snprintf(path, sizeof path, "%s/frame_%06u.ppm",
                     s_raw_dir, (unsigned)hdr.seq);
            FILE* fp = fopen(path, "wb");
            if (fp) {
                fprintf(fp, "P6\n%d %d\n255\n", EXPECT_W, EXPECT_H);
                fwrite(s_rgb_full, 1, expected_bytes, fp);
                fclose(fp);
            } else {
                // Log once-per-100 failures to surface disk-full / perm errors
                // without spamming the console.
                static uint32_t s_raw_fopen_fail = 0;
                if ((s_raw_fopen_fail++ % 100) == 0) {
                    fprintf(stderr, "camera_bridge: raw dump fopen('%s') failed "
                                    "(cumulative=%u)\r\n",
                            path, s_raw_fopen_fail);
                }
            }
        }
    }

    rgb888_to_xrgb8888(s_rgb_full, s_xrgb_buf, EXPECT_W * EXPECT_H);

    // ─── W11-T4: sentai_prep aux slot fan-out (SIM mirror of ARM
    // PrepTask producer in detection_task.cc:prep_task_fn) ────────────
    // Same atomic-publish contract; same continuous-publish policy.
    // When SlamTask is running it has enabled SLOT_RGB_64 — we fire
    // sentai_prep_publish_slot_rgb_64 which PXP-scales (SIM shim →
    // scalar area-average) the XRGB buffer and signals SlamTask's
    // counting sem.  No-op when no consumer is attached.
    {
        const uint32_t fire_mask = sentai_prep_tick_frame();
        if (fire_mask & (1u << SENTAI_PREP_SLOT_RGB_64)) {
            (void)sentai_prep_publish_slot_rgb_64(s_xrgb_buf,
                                                   EXPECT_W, EXPECT_H);
        }
        // SLOT_GRAY_NATIVE + SLOT_GRAY_64 producers on SIM: not yet
        // wired — no SIM consumer requires them at this point.
        // Adding them is mechanical (mirror the SLOT_RGB_64 block
        // with the appropriate PXP shim variant).
    }

    int rc = sentai_pxp_scale(s_xrgb_buf, EXPECT_W, EXPECT_H,
                              s_rgb_small, DST_W, DST_H);
    if (rc != 0) {
        printf("camera_bridge: pxp_scale rc=%d\r\n", rc);
        return -1;
    }

    rgb888_to_y(s_rgb_small, s_gray80x60, DST_W * DST_H);

    // ─────────────────────────────────────────────────────────────────
    // DUPLICATE-FRAME DETECTION (embeded.md "bounded behaviour" +
    // "preserve essential mission functions under overload").
    //
    // Garden's render thread runs slower than our 30 fps publish rate
    // (Sim.md §10d), so consecutive frames from the bridge are often
    // bit-identical with only the timestamp updated.  Running phase-corr
    // on duplicates wastes CPU AND, more importantly, lets the cf2 EKF
    // integrate stale-but-confident "zero motion" observations between
    // real renders — corrupting the position estimate.
    //
    // Solution: detect duplicate gray80x60 frames via a fast CRC.  On a
    // duplicate, REUSE the previous flow result (same dx/dy) and mark
    // confidence to 0 so the EKF down-weights the sample (caller maps
    // conf<conf_thresh_low → high std).  This way:
    //   - phase-corr runs only on distinct frames  → no wasted compute
    //   - EKF gets one strong observation per render + low-conf
    //     interpolation between → integration stays accurate
    //
    // Code is portable C, shared with ARM (where the OV5640 always
    // produces fresh frames so the duplicate path never fires — but the
    // code stays inert there, no behaviour change).
    // ─────────────────────────────────────────────────────────────────
    // 2026-05-11 CRITICAL FIX: duplicate detection was running on the
    // DOWNSAMPLED 80×60 gray buffer.  At z=1m, the 8× decimation
    // averages 14mm of ground per output pixel, so any sub-14mm
    // drone motion produces IDENTICAL downsampled output even though
    // the raw 640×480 RGB has 50-80% pixels differing.  Phase-corr
    // was reporting conf=0 on real but small motion (2-4cm/s slow
    // drift = ~2mm/frame = invisible at 80×60).
    //
    // FIX: check duplicate on the RAW 640×480 RGB instead.  Detects
    // ONLY truly identical frames (pre-takeoff / paused gz / etc.).
    // Cost: ~50K hash ops on 921600 bytes = ~150µs.  Worth it.
    static uint32_t s_prev_rgb_crc = 0;
    static int32_t  s_last_dx_q = 0;
    static int32_t  s_last_dy_q = 0;
    uint32_t gray_crc = 0;
    // Sparse sample of raw RGB (every 19th byte) for cheap CRC.
    for (size_t i = 0; i < expected_bytes; i += 19) {
        gray_crc = gray_crc * 31u + s_rgb_full[i];
    }

    int dx_q = 0, dy_q = 0, dz_q = 0;
    uint8_t conf = 0, dz_conf = 0;

    // P1: texture quality pre-screen (PX4Flow style).  Threshold raised
    // 10000→15000 — empirically more reliable rejection of weak scenes.
    const uint32_t TEXTURE_MIN_THRESH = 15000;
    uint32_t texture_quality = compute_texture_quality(s_gray80x60, DST_W, DST_H);

    if (gray_crc == s_prev_rgb_crc) {
        // True duplicate frame (raw RGB byte-identical) — re-use
        // previous result with conf=0 so the EKF down-weights this
        // sample.  Happens during pre-takeoff or gz render pauses.
        dx_q = s_last_dx_q;
        dy_q = s_last_dy_q;
        conf = 0;
    } else if (texture_quality < TEXTURE_MIN_THRESH) {
        // P1: textureless scene — phase-corr would return random peak.
        // Return zero motion with conf=0; EKF down-weights.
        dx_q = 0;
        dy_q = 0;
        conf = 0;
    } else {
        sentai_flow_phase_corr_compute_at(0, s_gray80x60, &dx_q, &dy_q, &conf);  // L0 wide

        // Saturation guard: phase-corr peak at the edge of its ±32
        // grid-px search range usually means the actual shift is larger
        // than the algorithm can measure unambiguously.  Treat as
        // unreliable (conf=0) rather than passing aliased values to the
        // EKF.  (Sim.md §10d issue #2.)
        const int SAT_LIMIT_MGP = 28000;   // 28 of ±32 grid-px
        if (dx_q >  SAT_LIMIT_MGP || dx_q < -SAT_LIMIT_MGP ||
            dy_q >  SAT_LIMIT_MGP || dy_q < -SAT_LIMIT_MGP) {
            conf = 0;   // EKF down-weights, doesn't integrate aliased motion
        }

        s_last_dx_q = dx_q;
        s_last_dy_q = dy_q;

        // dz divergence: only meaningful on a fresh frame (not a duplicate).
        // Cheap extra: 4× length-32 phase-corr ≈ 0.6 ms on x86.
        sentai_flow_phase_corr_compute_dz(s_gray80x60, &dz_q, &dz_conf);
    }
    s_prev_rgb_crc = gray_crc;

    // P6: post-correlation strict reject — drop low-conf peaks entirely
    // (don't propagate weak/random direction into EKF).
    const uint32_t POST_CORR_REJECT_CONF = 32;
    if (conf > 0 && conf < POST_CORR_REJECT_CONF) {
        dx_q = 0;
        dy_q = 0;
        conf = 0;
    }

    // P5 (new): median-of-3 smoothing on best output (dx_best, dy_best)
    // to suppress single-frame phase-corr noise spikes that propagate
    // straight into cf2 EKF velocity observation.  Median is naturally
    // outlier-robust (unlike mean averaging which oscillates on drone
    // wobble — already tested + rejected earlier in this session).
    //
    // History buffer stored AFTER fusion (later in this function).
    // Hooked in just before reply.dx_best assignment.

    // P3: 4-corner SAD validation of global L0 phase-corr estimate.
    // Compare against 4 corner patches' independent SAD search; if
    // majority disagree by > 2 pixels, mark L0 conf as low (outlier).
    // Helps when global FFT peak is dominated by a single feature (e.g.,
    // marker) but local areas show different motion.
    static uint8_t s_prev_gray_for_sad[DST_W * DST_H];
    static int s_have_prev_for_sad = 0;
    if (conf > 0 && s_have_prev_for_sad) {
        int dx_px = dx_q / 1000;   // convert mgrid → integer grid pixels
        int dy_px = dy_q / 1000;
        int agree = validate_motion_4corners(s_gray80x60, s_prev_gray_for_sad,
                                              DST_W, DST_H, dx_px, dy_px);
        if (!agree) {
            // Outlier — downgrade confidence so fusion prefers other levels.
            if (conf > 80) conf = 80;
        }
    }
    memcpy(s_prev_gray_for_sad, s_gray80x60, sizeof(s_prev_gray_for_sad));
    s_have_prev_for_sad = 1;

    // ─────────────────────────────────────────────────────────────────
    // BURT-ADELSON PYRAMID — L1 (mid) + L2 (fine) levels.
    // Each pipeline gets its own dedicated phase-corr context (state
    // is stored as static-locals inside sentai_flow_phase_corr_compute,
    // so we cannot reuse the same fn for multiple parallel streams —
    // each call would clobber the others' prev_fft cache).
    //
    // SOLUTION: serialise.  Wide call already done above using fn's
    // internal state.  For L1 + L2 we'd need separate prev_fft buffers
    // per pipeline.  EXPEDIENT: introduce three reset/save phases.
    //
    // For now we run all three pipelines but the engine's single
    // s_prev_fft cache gets cycled.  This means each level effectively
    // does frame[t] vs frame[t-3] comparisons (frames interleave).
    // Acceptable for the FUSION test — proper per-level state isolation
    // is the next refactor if results warrant it.
    // ─────────────────────────────────────────────────────────────────
    // L1 RE-ADDED 2026-05-11 per user: "fa LCF drift la L1, poate
    // acolo e mai ok".  L1 mid-level (4× decim, 6.93mm/grid at z=1m,
    // FOV 0.55m) sits between L0 wide and L2 native.  Per-grid 2×
    // L0 (catches motion L0 misses), FOV 4× L2 (anchor stays in FOV
    // longer for cumulative drift detection).
    boxfilter_4x_rgb_to_gray(s_rgb_full, EXPECT_W, EXPECT_H,
                              s_gray80x60_center, DST_W, DST_H);

    int dx_c = 0, dy_c = 0;
    uint8_t conf_c = 0;
    sentai_flow_phase_corr_compute_at(2, s_gray80x60_center,
                                       &dx_c, &dy_c, &conf_c);
    {
        const int SAT_LIMIT_MGP = 28000;
        if (dx_c >  SAT_LIMIT_MGP || dx_c < -SAT_LIMIT_MGP ||
            dy_c >  SAT_LIMIT_MGP || dy_c < -SAT_LIMIT_MGP) {
            conf_c = 0;
        }
    }

    // L2 fine — NATIVE crop (NO decimation) from center 80×60 of raw
    // 640×480.  At z=1m, per-pixel = 1.73mm = 8× finer than L0 wide.
    // Detects slow drift (2-4 cm/s = 1.4-2.9 mm/frame) that L0
    // averaging erases.
    // Coarse-to-fine scaling: L0 grid = 8 raw px, native grid = 1 raw px.
    // So 1 L0 grid = 8 native pixels → dx_native_px = dx_L0_mgrid * 8 / 1000 = /125
    crop_center_native_to_gray(s_rgb_full, EXPECT_W, EXPECT_H,
                                s_gray80x60_fine, DST_W, DST_H);

    float dx_pred_L2 = (float)dx_q / 125.0f;   // L0 mgrid → native pixels
    float dy_pred_L2 = (float)dy_q / 125.0f;
    warp_translate_bilinear(s_gray80x60_fine, s_gray80x60_warped,
                             DST_W, DST_H, -dx_pred_L2, -dy_pred_L2);

    static uint32_t s_prev_fine_crc = 0;
    static int32_t  s_last_dx_f = 0;
    static int32_t  s_last_dy_f = 0;
    uint32_t fine_crc = 0;
    for (int i = 0; i < DST_W * DST_H; ++i) {
        fine_crc = fine_crc * 31u + s_gray80x60_warped[i];
    }

    int dx_res_L2 = 0, dy_res_L2 = 0;
    uint8_t conf_L2 = 0;
    if (fine_crc == s_prev_fine_crc && conf_c > 0) {
        dx_res_L2 = s_last_dx_f;
        dy_res_L2 = s_last_dy_f;
        conf_L2 = 0;
    } else {
        sentai_flow_phase_corr_compute_at(2, s_gray80x60_warped,
                                           &dx_res_L2, &dy_res_L2, &conf_L2);
        const int SAT_LIMIT_MGP = 28000;
        if (dx_res_L2 >  SAT_LIMIT_MGP || dx_res_L2 < -SAT_LIMIT_MGP ||
            dy_res_L2 >  SAT_LIMIT_MGP || dy_res_L2 < -SAT_LIMIT_MGP) {
            conf_L2 = 0;
        }
        s_last_dx_f = dx_res_L2;
        s_last_dy_f = dy_res_L2;
    }
    s_prev_fine_crc = fine_crc;

    // Combined L2 motion in NATIVE mgrid (= 8× higher resolution than L0):
    //   combined = (L0_mgrid × 8) + native_residual_mgrid
    int dx_f = dx_q * 8 + dx_res_L2;
    int dy_f = dy_q * 8 + dy_res_L2;
    uint8_t conf_f = conf_L2;

    // ─────────────────────────────────────────────────────────────────
    // LastChangedFrame (LCF) ANCHOR — pipe 3.  Detect SLOW cumulative
    // drift that frame-to-frame phase-corr can't see (sub-pixel motion).
    //
    // Strategy:
    //   - When instantaneous motion (|dx_q| or |dy_q| on L0) is BELOW
    //     a small threshold, the LCF is HELD — anchor reference stays
    //     pointed at the frame where motion was last detected.
    //   - Compute phase-corr current frame vs anchor → cumulative drift
    //     since anchor was set.  Sub-pixel drift over many frames
    //     accumulates into a detectable peak.
    //   - When instantaneous motion exceeds threshold, REFRESH anchor:
    //     copy current FFT into pipe 3's prev_fft.  Anchor follows
    //     drone whenever it's actively moving.
    // ─────────────────────────────────────────────────────────────────
    // LastMovedFrame (LMF) — refresh threshold tuned empirically:
    //
    //   threshold = 50:   refresh every frame → anchor never holds → useless
    //   threshold = 1500: anchor holds 100+ frames in true slow-drift hover,
    //                    refreshes only on clearly-detected motion
    //
    // L0 phase-corr noise floor at hover = ±100-300 mgrid (random peak
    // placement on sub-pixel motion).  1500 is above noise + below
    // typical "real motion detected" bursts of 2000-5000 mgrid.
    const int MOTION_DETECT_THRESH = 1500;
    static int s_have_anchor = 0;
    static uint32_t s_anchor_seq = 0;
    int dx_anchor = 0, dy_anchor = 0;
    uint8_t conf_anchor = 0;
    uint32_t frames_since_anchor = 0;
    int is_moving = (abs(dx_q) >= MOTION_DETECT_THRESH ||
                     abs(dy_q) >= MOTION_DETECT_THRESH);
    // DEBUG: keep a SHADOW copy of anchor gray buffer so we can dump
    // it later for visual diff.  Lives in SDRAM (4.8KB).
    static uint8_t s_anchor_gray_shadow[DST_W * DST_H];
    if (!s_have_anchor) {
        // Initialize anchor with first frame seen.
        sentai_flow_phase_corr_set_anchor(3, s_gray80x60);
        memcpy(s_anchor_gray_shadow, s_gray80x60, sizeof(s_anchor_gray_shadow));
        s_have_anchor = 1;
        s_anchor_seq = hdr.seq;
    } else if (is_moving && conf > 0) {
        // P4 hysteresis reverted — net-neutral trade-off in tests.
        sentai_flow_phase_corr_set_anchor(3, s_gray80x60);
        memcpy(s_anchor_gray_shadow, s_gray80x60, sizeof(s_anchor_gray_shadow));
        s_anchor_seq = hdr.seq;
    } else {
        // Stationary (or low conf inst): compare current frame against
        // frozen anchor.  Returns CUMULATIVE motion since anchor was set.
        sentai_flow_phase_corr_compute_against_anchor(3, s_gray80x60,
                                                       &dx_anchor, &dy_anchor,
                                                       &conf_anchor);
        frames_since_anchor = hdr.seq - s_anchor_seq;
    }

    // ─── L1 MID ANCHOR (pipe 4, NEW) ─────────────────────────────────
    static int s_have_anchor_L1 = 0;
    static uint32_t s_anchor_seq_L1 = 0;
    int dx_anchor_L1 = 0, dy_anchor_L1 = 0;
    uint8_t conf_anchor_L1 = 0;
    uint32_t frames_since_anchor_L1 = 0;
    // L1 native = 6.93mm/px → threshold 3000 mgrid = 20.8mm motion match L0.
    const int MOTION_DETECT_THRESH_L1 = 3000;
    int is_moving_L1 = (abs(dx_c) >= MOTION_DETECT_THRESH_L1 ||
                        abs(dy_c) >= MOTION_DETECT_THRESH_L1);
    if (!s_have_anchor_L1) {
        sentai_flow_phase_corr_set_anchor(4, s_gray80x60_center);
        s_have_anchor_L1 = 1;
        s_anchor_seq_L1 = hdr.seq;
    } else if (is_moving_L1 && conf_c > 0) {
        sentai_flow_phase_corr_set_anchor(4, s_gray80x60_center);
        s_anchor_seq_L1 = hdr.seq;
    } else {
        sentai_flow_phase_corr_compute_against_anchor(4, s_gray80x60_center,
                                                       &dx_anchor_L1, &dy_anchor_L1,
                                                       &conf_anchor_L1);
        frames_since_anchor_L1 = hdr.seq - s_anchor_seq_L1;
    }

    // ─── L2 NATIVE-CROP ANCHOR (pipe 5, moved from pipe 1) ────────────
    // Same logic as L0 anchor but on s_gray80x60_fine (native pixel crop).
    // 8× finer per-pixel resolution → detects much smaller cumulative drift.
    // Refresh triggered by L2's OWN phase-corr motion (dx_res_L2) — not
    // L0's, because L2 sees finer motion that L0 misses.
    static int s_have_anchor_L2 = 0;
    static uint32_t s_anchor_seq_L2 = 0;
    int dx_anchor_L2 = 0, dy_anchor_L2 = 0;
    uint8_t conf_anchor_L2 = 0;
    uint32_t frames_since_anchor_L2 = 0;
    // L2 native is 8× higher resolution than L0, so its mgrid threshold
    // must be 8× higher to represent the SAME physical motion magnitude:
    //   L0 threshold 1500 mgrid = 20.8mm motion at z=1m
    //   L2 threshold 12000 mgrid = 20.8mm motion at z=1m (matched)
    // Bug: previously set to 1500 — fired every sub-frame → anchor never
    // accumulated → LCF_L2 picks = 0/272 in test.  Fix: scale to L2 units.
    const int MOTION_DETECT_THRESH_L2 = 12000;
    int is_moving_L2 = (abs(dx_res_L2) >= MOTION_DETECT_THRESH_L2 ||
                        abs(dy_res_L2) >= MOTION_DETECT_THRESH_L2);
    if (!s_have_anchor_L2) {
        sentai_flow_phase_corr_set_anchor(5, s_gray80x60_fine);
        s_have_anchor_L2 = 1;
        s_anchor_seq_L2 = hdr.seq;
    } else if (is_moving_L2 && conf_L2 > 0) {
        sentai_flow_phase_corr_set_anchor(5, s_gray80x60_fine);
        s_anchor_seq_L2 = hdr.seq;
    } else {
        sentai_flow_phase_corr_compute_against_anchor(5, s_gray80x60_fine,
                                                       &dx_anchor_L2, &dy_anchor_L2,
                                                       &conf_anchor_L2);
        frames_since_anchor_L2 = hdr.seq - s_anchor_seq_L2;
    }

    // ─────────────────────────────────────────────────────────────────
    // FUSION (C-side, single best estimate exposed to consumer)
    //
    // Priority order:
    //   1. ANCHOR — when held ≥20 frames AND conf decent AND has
    //      accumulated significant cumulative motion.  Anchor-derived
    //      per-frame velocity is INHERENTLY smoother (integrated)
    //      than instantaneous L0 phase-corr noise.
    //   2. L2 refined — if conf high (fine pyramid level, coarse-to-fine
    //      already applied via warping)
    //   3. L1 refined — fallback to mid-level if L2 confidence low
    //   4. L0 raw — final fallback for fast motion / no other reliable
    //
    // All outputs in L0-equivalent mgrid units (per consumer convention).
    // ─────────────────────────────────────────────────────────────────
    int32_t  dx_best = dx_q, dy_best = dy_q;
    uint32_t conf_best = conf;
    uint8_t  best_source = 0;   // L0 default

    // Anchor priority MID-TUNE 2026-05-11 (sweet spot):
    //   8  frames + conf 40 + mag  80 → dist_mean=0.307 but all-4 drop 37%→21%
    //   20 frames + conf 60 + mag 200 → dist_mean=0.329 with all-4 37%
    //   12 frames + conf 50 + mag 120 → MIDDLE GROUND test
    const uint32_t ANCH_MIN_FRAMES   = 12;
    const uint32_t ANCH_MIN_CONF     = 50;
    const int      ANCH_MIN_CUM_MAG  = 120;
    const uint32_t REFINE_MIN_CONF   = 64;
    const int      SAT_LIMIT_REFINE  = 24000;

    // L2-NATIVE anchor priority FIRST — BUT only when cumulative drift is
    // still WITHIN the native FOV (14cm at z=1m).  If anchor reports
    // motion approaching FOV edge, the ground content under camera has
    // shifted to a region the anchor never saw → phase-corr peak
    // unreliable (spurious match).  Saturation limit ~50% of FOV =
    // 40 native pixels = 40000 mgrid.
    const int L2_ANCHOR_SAT_MGRID = 40000;
    int l2_anchor_in_fov = (dx_anchor_L2 < L2_ANCHOR_SAT_MGRID
                              && dx_anchor_L2 > -L2_ANCHOR_SAT_MGRID
                              && dy_anchor_L2 < L2_ANCHOR_SAT_MGRID
                              && dy_anchor_L2 > -L2_ANCHOR_SAT_MGRID);
    if (l2_anchor_in_fov
            && frames_since_anchor_L2 >= ANCH_MIN_FRAMES
            && conf_anchor_L2 >= ANCH_MIN_CONF
            && ((dx_anchor_L2 > ANCH_MIN_CUM_MAG || dx_anchor_L2 < -ANCH_MIN_CUM_MAG)
                || (dy_anchor_L2 > ANCH_MIN_CUM_MAG || dy_anchor_L2 < -ANCH_MIN_CUM_MAG))) {
        dx_best     = (dx_anchor_L2 / (int)frames_since_anchor_L2) / 8;
        dy_best     = (dy_anchor_L2 / (int)frames_since_anchor_L2) / 8;
        conf_best   = conf_anchor_L2;
        best_source = 4;   // LCF_anchor_L2
    } else if (frames_since_anchor_L1 >= ANCH_MIN_FRAMES
            && conf_anchor_L1 >= ANCH_MIN_CONF
            && ((dx_anchor_L1 > ANCH_MIN_CUM_MAG || dx_anchor_L1 < -ANCH_MIN_CUM_MAG)
                || (dy_anchor_L1 > ANCH_MIN_CUM_MAG || dy_anchor_L1 < -ANCH_MIN_CUM_MAG))) {
        // L1-anchor (mid) — per-frame in L1 mgrid → divide by 2 for L0-equiv
        dx_best     = (dx_anchor_L1 / (int)frames_since_anchor_L1) / 2;
        dy_best     = (dy_anchor_L1 / (int)frames_since_anchor_L1) / 2;
        conf_best   = conf_anchor_L1;
        best_source = 5;   // LCF_anchor_L1
    } else if (frames_since_anchor >= ANCH_MIN_FRAMES
            && conf_anchor >= ANCH_MIN_CONF
            && ((dx_anchor > ANCH_MIN_CUM_MAG || dx_anchor < -ANCH_MIN_CUM_MAG)
                || (dy_anchor > ANCH_MIN_CUM_MAG || dy_anchor < -ANCH_MIN_CUM_MAG))) {
        // L0-anchor — per-frame velocity already in L0 mgrid units.
        dx_best     = dx_anchor / (int)frames_since_anchor;
        dy_best     = dy_anchor / (int)frames_since_anchor;
        conf_best   = conf_anchor;
        best_source = 3;   // LCF_anchor_L0
    } else if (conf_f >= REFINE_MIN_CONF
               && dx_f < SAT_LIMIT_REFINE && dx_f > -SAT_LIMIT_REFINE
               && dy_f < SAT_LIMIT_REFINE && dy_f > -SAT_LIMIT_REFINE) {
        // L2 NATIVE crop (8× finer per-px than L0) → scale L0-equivalent by 1/8
        dx_best     = dx_f / 8;
        dy_best     = dy_f / 8;
        conf_best   = conf_f;
        best_source = 2;
    } else if (conf_c >= REFINE_MIN_CONF
               && dx_c < SAT_LIMIT_REFINE && dx_c > -SAT_LIMIT_REFINE
               && dy_c < SAT_LIMIT_REFINE && dy_c > -SAT_LIMIT_REFINE) {
        // L1 (mid) — scale to L0-equivalent: L1_mgrid × 0.5
        dx_best     = dx_c / 2;
        dy_best     = dy_c / 2;
        conf_best   = conf_c;
        best_source = 1;
    }
    // else default L0 already set
    // P5 median-of-3 reverted — introduced lag on oscillating drone.

    // L0 80×60 gray dump per frame (~5 KB each as P5/grayscale PPM).
    // Use SENTAI_DUMP_FRAMES_EVERY (default 1 = every frame).  This is
    // the buffer the L0 phase-corr sees — what you'd inspect to debug
    // texture / scene / motion frame-by-frame.  L1 / L2 / ANCHOR dumps
    // dropped — only useful for pyramid validation, not routine inspect.
    {
        static int s_l0_init = 0;
        static const char* s_l0_dir = NULL;
        static int s_l0_every = 1;
        if (!s_l0_init) {
            s_l0_init = 1;
            s_l0_dir = getenv("SENTAI_DUMP_FRAMES_DIR");
            const char* en = getenv("SENTAI_DUMP_FRAMES_EVERY");
            if (en && *en) {
                int v = atoi(en);
                if (v > 0) s_l0_every = v;
            }
        }
        if (s_l0_dir && s_l0_every > 0 && (hdr.seq % s_l0_every) == 0) {
            char path[512];
            snprintf(path, sizeof path, "%s/L0_%06u.pgm",
                     s_l0_dir, (unsigned)hdr.seq);
            FILE* fp = fopen(path, "wb");
            if (fp) {
                fprintf(fp, "P5\n%d %d\n255\n", DST_W, DST_H);
                fwrite(s_gray80x60, 1, DST_W * DST_H, fp);
                fclose(fp);
            } else {
                static uint32_t s_l0_fopen_fail = 0;
                if ((s_l0_fopen_fail++ % 100) == 0) {
                    fprintf(stderr, "camera_bridge: L0 dump fopen('%s') failed "
                                    "(cumulative=%u)\r\n",
                            path, s_l0_fopen_fail);
                }
            }
        }
    }

    uint64_t t1 = now_us();

    // Publish atomically-ish: write payload first, bump seq last so
    // readers can detect mid-publish via seq stability check.
    g_flow.dx_q1000   = dx_q;
    g_flow.dy_q1000   = dy_q;
    g_flow.conf       = conf;
    g_flow.latency_us = t1 - t0;
    g_flow.dz_q1000   = dz_q;
    g_flow.dz_conf    = dz_conf;
    __sync_synchronize();
    g_flow.seq        = hdr.seq;

    // Reply on the same socket so the sender (Python bridge) can pipe the
    // flow snapshot straight into cflib without a second IPC channel.
    flow_reply_t reply = {
        .reply_magic = REPLY_MAGIC,
        .seq         = hdr.seq,
        .dx_q1000    = dx_q,
        .dy_q1000    = dy_q,
        .conf        = (uint32_t)conf,
        .latency_us  = t1 - t0,
        .dz_q1000    = dz_q,
        .dz_conf     = (uint32_t)dz_conf,
        .dx_center_q1000 = dx_c,
        .dy_center_q1000 = dy_c,
        .conf_center     = (uint32_t)conf_c,
        .dx_fine_q1000   = dx_f,
        .dy_fine_q1000   = dy_f,
        .conf_fine       = (uint32_t)conf_f,
        .dx_anchor_q1000 = dx_anchor,
        .dy_anchor_q1000 = dy_anchor,
        .conf_anchor     = (uint32_t)conf_anchor,
        .frames_since_anchor = frames_since_anchor,
        .dx_anchor_L2_q1000 = dx_anchor_L2,
        .dy_anchor_L2_q1000 = dy_anchor_L2,
        .conf_anchor_L2     = (uint32_t)conf_anchor_L2,
        .frames_since_anchor_L2 = frames_since_anchor_L2,
        .dx_anchor_L1_q1000 = dx_anchor_L1,
        .dy_anchor_L1_q1000 = dy_anchor_L1,
        .conf_anchor_L1     = (uint32_t)conf_anchor_L1,
        .frames_since_anchor_L1 = frames_since_anchor_L1,
        .dx_best_q1000   = dx_best,
        .dy_best_q1000   = dy_best,
        .conf_best       = conf_best,
        .best_source     = best_source,
    };
    if (write_full(fd, &reply, sizeof(reply)) != 0) return -1;

    return 0;
}

static void accept_loop(int listen_fd) {
    while (1) {
        int cfd = accept(listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            printf("camera_bridge: accept errno=%d (%s)\r\n", errno, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // CRITICAL fix (code review HIGH): bound the reply write so a
        // dead bridge can't deadlock our flow pipeline.  200 ms is plenty
        // (32 B reply); on timeout we drop the client and re-accept.
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200 * 1000 };
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        printf("camera_bridge: client connected\r\n");
        // One client at a time — Gazebo bridge is the only sender.
        while (handle_one_frame(cfd) == 0) { /* loop until EOF */ }
        close(cfd);
        printf("camera_bridge: client disconnected (frames=%u)\r\n",
               (unsigned)g_flow.seq);
    }
}

static void camera_bridge_task(void* arg) {
    (void)arg;

    // Best-effort cleanup of stale socket from previous run.
    unlink(SOCK_PATH);

    int listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        printf("camera_bridge: socket() failed errno=%d\r\n", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("camera_bridge: bind(%s) errno=%d (%s)\r\n",
               SOCK_PATH, errno, strerror(errno));
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    chmod(SOCK_PATH, 0666);  // any local user may push frames

    if (listen(listen_fd, 1) < 0) {
        printf("camera_bridge: listen errno=%d\r\n", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    printf("camera_bridge: listening on %s (waiting for Gazebo bridge)\r\n",
           SOCK_PATH);

    accept_loop(listen_fd);

    close(listen_fd);
    vTaskDelete(NULL);
}

// Public init — call once at boot from main_sim.c.
void sim_camera_bridge_start(void) {
    static StaticTask_t s_tcb;
    static StackType_t  s_stack[8192];   // 32 KB on x86 with 4-byte StackType_t
    xTaskCreateStatic(camera_bridge_task, "camera_bridge",
                      sizeof(s_stack) / sizeof(s_stack[0]),
                      NULL, tskIDLE_PRIORITY + 2,
                      s_stack, &s_tcb);
}
