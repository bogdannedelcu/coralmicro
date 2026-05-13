// sentai_aruco_shim_sim.c — SIM implementation of the ArUco anchor shim.
//
// Architecture:
//
//   Gazebo /downward_cam/image
//        |
//        v
//   sim/scripts/aruco_pose_publisher.py  (cv2.aruco + solvePnP +
//                                         estimate_drone_world_pose)
//        |  UDS SOCK_DGRAM @ /tmp/sentai_aruco_pose.sock
//        |  (one packet per detection, fixed 36-byte struct)
//        v
//   THIS FILE                             (FreeRTOS task in sentai_sim)
//        |  cached as sentai_aruco_pose_t
//        v
//   sentai_aruco_detect() / sentai_aruco_get_latest()
//
// Why SOCK_DGRAM not STREAM:
//   - publisher sends one fixed-size frame; loss-tolerant (latest
//     value wins)
//   - reader can recvfrom non-blocking → never stalls the flow path
//   - no framing / no length prefix needed
//
// Wire format (matches the Python publisher's struct.pack):
//   uint32_t magic        = 0x41524332 ('ARC2')
//   uint32_t frame_seq    monotonic from publisher
//   uint8_t  detected     0/1
//   uint8_t  num_markers  0..N
//   uint16_t _pad
//   float    x_m, y_m, z_m, yaw_rad
//   uint32_t detect_us
//   uint32_t src_ts_ms
//   total = 36 bytes, little-endian (host = x86)
//
// All multi-byte fields little-endian; struct is naturally aligned.

#include "examples/sentai_runtime/sentai_aruco_shim.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#define ARUCO_UDS_PATH      "/tmp/sentai_aruco_pose.sock"
#define ARUCO_RECV_UDS_PATH "/tmp/sentai_aruco_pose_recv.sock"  // our bind
#define ARUCO_WIRE_MAGIC    0x41524332u
#define ARUCO_WIRE_SIZE     36u

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t frame_seq;
    uint8_t  detected;
    uint8_t  num_markers;
    uint16_t pad;
    float    x_m;
    float    y_m;
    float    z_m;
    float    yaw_rad;
    uint32_t detect_us;
    uint32_t src_ts_ms;
} aruco_wire_t;
#pragma pack(pop)
_Static_assert(sizeof(aruco_wire_t) == ARUCO_WIRE_SIZE,
               "aruco_wire_t size != 36 bytes — wire format mismatch with publisher");

static int                  s_fd          = -1;
static pthread_mutex_t      s_mtx         = PTHREAD_MUTEX_INITIALIZER;
static sentai_aruco_pose_t  s_latest      = {0};
static int                  s_inited      = 0;
static uint64_t             s_pkt_count   = 0;
static uint64_t             s_pkt_dropped = 0;

// Drain everything currently queued on the socket so we keep the
// most-recent pose snapshot, never a stale one.  Bounded loop.
static void drain_latest(void) {
    aruco_wire_t w;
    int got_any = 0;
    while (1) {
        ssize_t n = recv(s_fd, &w, sizeof(w), MSG_DONTWAIT);
        if (n < 0) {
            // EAGAIN / EWOULDBLOCK = nothing more to read; break cleanly.
            break;
        }
        if (n == 0) break;
        if (n != (ssize_t)sizeof(w)) continue;
        if (w.magic != ARUCO_WIRE_MAGIC) {
            // Stray packet on the socket; skip.  Bounded by recv loop.
            continue;
        }
        pthread_mutex_lock(&s_mtx);
        if (got_any) s_pkt_dropped++;
        s_latest.detected    = w.detected;
        s_latest.num_markers = w.num_markers;
        s_latest.x_m         = w.x_m;
        s_latest.y_m         = w.y_m;
        s_latest.z_m         = w.z_m;
        s_latest.yaw_rad     = w.yaw_rad;
        s_latest.frame_seq   = w.frame_seq;
        s_latest.detect_us   = w.detect_us;
        s_latest.src_ts_ms   = w.src_ts_ms;
        pthread_mutex_unlock(&s_mtx);
        s_pkt_count++;
        got_any = 1;
    }
}

int sentai_aruco_init(void) {
    if (s_inited) return 0;

    s_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (s_fd < 0) {
        printf("aruco_shim: socket() failed errno=%d (%s)\r\n",
               errno, strerror(errno));
        return -1;
    }
    // Bind to our own path so the publisher knows where to sendto().
    // SOCK_DGRAM unix sockets need a named endpoint at both sides.
    unlink(ARUCO_RECV_UDS_PATH);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, ARUCO_RECV_UDS_PATH, sizeof(addr.sun_path) - 1);
    if (bind(s_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("aruco_shim: bind(%s) failed errno=%d (%s)\r\n",
               ARUCO_RECV_UDS_PATH, errno, strerror(errno));
        close(s_fd);
        s_fd = -1;
        return -2;
    }
    // World-writable so the publisher can sendto() us if it runs
    // under a different uid (rare, but cheap insurance).
    chmod(ARUCO_RECV_UDS_PATH, 0666);

    memset(&s_latest, 0, sizeof(s_latest));
    s_inited = 1;
    printf("aruco_shim: SIM ready, bound to %s, awaiting %s\r\n",
           ARUCO_RECV_UDS_PATH, ARUCO_UDS_PATH);
    return 0;
}

int sentai_aruco_detect(const uint8_t* gray, int w, int h,
                        sentai_aruco_pose_t* out) {
    (void)gray; (void)w; (void)h;
    if (!s_inited) {
        int rc = sentai_aruco_init();
        if (rc != 0) return rc;
    }
    drain_latest();
    return sentai_aruco_get_latest(out);
}

int sentai_aruco_get_latest(sentai_aruco_pose_t* out) {
    if (!out) return -1;
    if (!s_inited) {
        memset(out, 0, sizeof(*out));
        return 0;
    }
    // Drain on read too — `sentai.flow.anchor_pose()` should reflect
    // the latest publisher datagram even without an explicit detect().
    drain_latest();
    pthread_mutex_lock(&s_mtx);
    memcpy(out, &s_latest, sizeof(*out));
    pthread_mutex_unlock(&s_mtx);
    return 0;
}

void sentai_aruco_shutdown(void) {
    if (s_fd >= 0) close(s_fd);
    s_fd = -1;
    s_inited = 0;
    unlink(ARUCO_RECV_UDS_PATH);
}
