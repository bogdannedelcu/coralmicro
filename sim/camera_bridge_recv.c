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

// flow_phase_corr public entry — defined in flow_phase_corr.cc.
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

// ────────────────────────────────────────────────────────────────────────
// Static buffers — sized for 640x480 RGB and 80x60 RGB+gray.
// Keeping them static avoids any allocator in the per-frame path.
// ────────────────────────────────────────────────────────────────────────
#define EXPECT_W   640
#define EXPECT_H   480
#define DST_W      80
#define DST_H      60

static uint8_t s_xrgb_buf[EXPECT_W * EXPECT_H * 4];   // 1.2 MB
static uint8_t s_rgb_small[DST_W * DST_H * 3];        // 14.4 KB
static uint8_t s_gray80x60[DST_W * DST_H];            // 4.8 KB

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
    int32_t  dx_q1000;
    int32_t  dy_q1000;
    uint32_t conf;
    uint64_t latency_us;
    // dz from sub-block divergence (added 2026-05-11):
    //   units: micro per frame (1000 = +0.1% altitude/frame)
    //   sign:  positive = drone rising  (image features expand)
    //          negative = drone falling (image features converge)
    // Diagnostic only; cf2 EKF doesn't accept dz from flow.
    int32_t  dz_q1000;
    uint32_t dz_conf;
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

    rgb888_to_xrgb8888(s_rgb_full, s_xrgb_buf, EXPECT_W * EXPECT_H);

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
    static uint32_t s_prev_gray_crc = 0;
    static int32_t  s_last_dx_q = 0;
    static int32_t  s_last_dy_q = 0;
    uint32_t gray_crc = 0;
    for (int i = 0; i < DST_W * DST_H; ++i) {
        gray_crc = gray_crc * 31u + s_gray80x60[i];
    }

    int dx_q = 0, dy_q = 0, dz_q = 0;
    uint8_t conf = 0, dz_conf = 0;

    if (gray_crc == s_prev_gray_crc) {
        // Duplicate frame — re-use previous result with conf=0 so the
        // EKF treats it as a low-confidence interpolation between renders.
        dx_q = s_last_dx_q;
        dy_q = s_last_dy_q;
        conf = 0;
    } else {
        sentai_flow_phase_corr_compute(s_gray80x60, &dx_q, &dy_q, &conf);

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
    s_prev_gray_crc = gray_crc;

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
