// camera_bridge_recv.c -- Gazebo RGB camera socket bridge for sentai_sim.
//
// This file is intentionally only a simulator bridge.  It accepts RGB888
// frames from sim/scripts/gz_to_camera_bridge.py and injects them into the
// shared SentAI camera frame backend.  Virtual camera, PrepTask slot
// publication, and FlowTask processing live in examples/sentai_runtime.

#ifndef SENTAI_PLATFORM_SIM
#error "camera_bridge_recv.c is SIM-only -- guard via sim/CMakeLists.txt"
#endif

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "examples/sentai_runtime/sentai_log.h"

#define EXPECT_W 640
#define EXPECT_H 480
#define SOCK_PATH "/tmp/sentai_cam.sock"

#define MAGIC       0x53434D31u  // 'SCM1'
#define REPLY_MAGIC 0x46524C31u  // 'FRL1'

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
    int32_t  dz_q1000;
    uint32_t dz_conf;
    int32_t  dx_center_q1000;
    int32_t  dy_center_q1000;
    uint32_t conf_center;
    int32_t  dx_fine_q1000;
    int32_t  dy_fine_q1000;
    uint32_t conf_fine;
    int32_t  dx_anchor_q1000;
    int32_t  dy_anchor_q1000;
    uint32_t conf_anchor;
    uint32_t frames_since_anchor;
    int32_t  dx_anchor_L2_q1000;
    int32_t  dy_anchor_L2_q1000;
    uint32_t conf_anchor_L2;
    uint32_t frames_since_anchor_L2;
    int32_t  dx_anchor_L1_q1000;
    int32_t  dy_anchor_L1_q1000;
    uint32_t conf_anchor_L1;
    uint32_t frames_since_anchor_L1;
    int32_t  dx_best_q1000;
    int32_t  dy_best_q1000;
    uint32_t conf_best;
    uint8_t  best_source;
    uint8_t  _pad[3];
} flow_reply_t;

typedef struct {
    volatile uint32_t seq;
    volatile int32_t  dx_q1000;
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t  dz_q1000;
    volatile uint32_t dz_conf;
} sim_flow_snapshot_t;

extern int sentai_camera_backend_publish_rgb888(uint32_t seq,
                                                const uint8_t* rgb,
                                                size_t bytes);
extern void sentai_flow_poll_once(void);
extern const sim_flow_snapshot_t* sim_camera_flow_snapshot(void);

static int read_full(int fd, void* dst, size_t n) {
    uint8_t* p = (uint8_t*)dst;
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r > 0) {
            got += (size_t)r;
            continue;
        }
        if (r < 0 && errno == EINTR) continue;
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        return -1;
    }
    return 0;
}

static int write_full(int fd, const void* src, size_t n) {
    const uint8_t* p = (const uint8_t*)src;
    size_t put = 0;
    while (put < n) {
        ssize_t w = write(fd, p + put, n - put);
        if (w > 0) {
            put += (size_t)w;
            continue;
        }
        if (w < 0 && errno == EINTR) continue;
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        return -1;
    }
    return 0;
}

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static int handle_one_frame(int fd) {
    cam_header_t hdr;
    if (read_full(fd, &hdr, sizeof(hdr)) != 0) return -1;

    if (hdr.magic != MAGIC) {
        sentai_logf("camera_bridge", "bad magic 0x%08x, dropping connection",
                    hdr.magic);
        return -1;
    }
    if (hdr.width != EXPECT_W || hdr.height != EXPECT_H || hdr.pix_fmt != 0) {
        sentai_logf("camera_bridge", "unexpected geometry %ux%u fmt=%u",
                    hdr.width, hdr.height, hdr.pix_fmt);
        return -1;
    }
    const size_t expected_bytes = (size_t)EXPECT_W * EXPECT_H * 3u;
    if (hdr.payload_bytes != expected_bytes) {
        sentai_logf("camera_bridge", "payload mismatch got=%u want=%zu",
                    hdr.payload_bytes, expected_bytes);
        return -1;
    }

    static uint8_t s_rgb_full[EXPECT_W * EXPECT_H * 3];
    if (read_full(fd, s_rgb_full, expected_bytes) != 0) return -1;

    const uint64_t t0 = now_us();
    if (sentai_camera_backend_publish_rgb888(hdr.seq, s_rgb_full,
                                             expected_bytes) != 0) {
        return -1;
    }
    sentai_flow_poll_once();
    const uint64_t t1 = now_us();

    const sim_flow_snapshot_t* s = sim_camera_flow_snapshot();
    int32_t dx = s ? s->dx_q1000 : 0;
    int32_t dy = s ? s->dy_q1000 : 0;
    uint32_t conf = s ? s->conf : 0;
    int32_t dz = s ? s->dz_q1000 : 0;
    uint32_t dz_conf = s ? s->dz_conf : 0;
    uint64_t latency = s && s->latency_us ? s->latency_us : (t1 - t0);

    flow_reply_t reply;
    memset(&reply, 0, sizeof(reply));
    reply.reply_magic = REPLY_MAGIC;
    reply.seq = hdr.seq;
    reply.dx_q1000 = dx;
    reply.dy_q1000 = dy;
    reply.conf = conf;
    reply.latency_us = latency;
    reply.dz_q1000 = dz;
    reply.dz_conf = dz_conf;
    reply.dx_best_q1000 = dx;
    reply.dy_best_q1000 = dy;
    reply.conf_best = conf;
    reply.best_source = 0;
    return write_full(fd, &reply, sizeof(reply));
}

static void accept_loop(int listen_fd) {
    while (1) {
        int cfd = accept(listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            sentai_logf("camera_bridge", "accept errno=%d (%s)", errno,
                        strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200 * 1000 };
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int flags = fcntl(cfd, F_GETFL, 0);
        if (flags >= 0) (void)fcntl(cfd, F_SETFL, flags | O_NONBLOCK);
        sentai_logf("camera_bridge", "client connected");
        while (handle_one_frame(cfd) == 0) {}
        close(cfd);
        const sim_flow_snapshot_t* s = sim_camera_flow_snapshot();
        sentai_logf("camera_bridge", "client disconnected frames=%u",
                    s ? (unsigned)s->seq : 0u);
    }
}

static void camera_bridge_task(void* arg) {
    (void)arg;
    unlink(SOCK_PATH);

    int listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        sentai_logf("camera_bridge", "socket failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        sentai_logf("camera_bridge", "bind(%s) errno=%d (%s)",
                    SOCK_PATH, errno, strerror(errno));
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    chmod(SOCK_PATH, 0666);

    if (listen(listen_fd, 1) < 0) {
        sentai_logf("camera_bridge", "listen errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    int flags = fcntl(listen_fd, F_GETFL, 0);
    if (flags >= 0) (void)fcntl(listen_fd, F_SETFL, flags | O_NONBLOCK);

    sentai_logf("camera_bridge", "listening on %s waiting for Gazebo bridge",
                SOCK_PATH);
    accept_loop(listen_fd);
}

void sim_camera_bridge_start(void) {
    static StaticTask_t s_tcb;
    static StackType_t s_stack[4096];
    xTaskCreateStatic(camera_bridge_task, "camera_bridge",
                      sizeof(s_stack) / sizeof(s_stack[0]),
                      NULL, tskIDLE_PRIORITY + 2,
                      s_stack, &s_tcb);
}
