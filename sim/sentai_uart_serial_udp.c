/*
 * sentai_uart_serial_udp.c — SIM-only UDP backend for sentai.link.
 *
 * Provides the same C ABI as the ARM `sentai_uart_serial_*` family
 * (defined in libs/base/, used by sentai_link.cc) so that the
 * MicroPython binding `sentai.link.*` works UNCHANGED in the SIM
 * build.  ARM keeps LPUART6; SIM gets a UDP datagram pipe to whatever
 * MAVLink peer is configured via env vars.
 *
 * Env vars (read at sentai_uart_serial_open()):
 *   SENTAI_LINK_UDP_HOST        (default "127.0.0.1") — peer address
 *   SENTAI_LINK_UDP_REMOTE_PORT (default 14580)        — peer port
 *   SENTAI_LINK_UDP_LOCAL_PORT  (default 14540)        — our bind port
 *
 * Defaults match PX4 v1.14 offboard MAVLink endpoint, instance 0:
 *   PX4 BINDS 14580 (waits for offboard cmds from companion)
 *   PX4 SENDS  14540 (telemetry / heartbeat → companion)
 *   sentai BINDS 14540, SENDS 14580.  See PX4 ROMFS/.../px4-rc.mavlink.
 *
 * Port allocation (Sim.md §10m): 14540/14580 reserved for sentai↔PX4
 * companion link; 18570 reserved for GCS (host pymavlink/MAVSDK);
 * 19850 reserved for CrazySim cflib.  NO conflicts when all 3 share
 * the same host.
 *
 * Concurrency: sentai_link.cc has ONE reader task and ONE writer task,
 * each calling the *write / *read functions sequentially.  No locking
 * needed for the socket fd because we don't share it with anything else
 * in SIM.  The fd is opened once on _open() and closed on _close().
 *
 * Failure semantics:
 *   - open: returns 1 (success) only after the socket binds + connects.
 *     Returns 0 on any error; cumulative error count printed once per 100.
 *   - write: returns bytes actually sent (0 on dead socket).
 *   - read: returns bytes read, or 0 on timeout (does NOT block forever).
 *   - timeout_ms is honored via select(); upper bound enforced (5 s).
 *
 * NASA/JPL discipline (per agent/embeded.md): bounded I/O, checked
 * returns, no dynamic allocation after init, no recursion.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define UDP_BUF_SZ      2048   /* MAVLink v2 max frame = 280 B; 2 KB headroom */
#define MAX_TIMEOUT_MS  5000   /* upper bound on any read timeout */

static int          s_sock = -1;
static struct sockaddr_in s_peer;
static int          s_have_peer = 0;
static uint32_t     s_open_fail_count = 0;
static uint32_t     s_io_fail_count = 0;


static int env_int(const char* name, int def) {
    const char* v = getenv(name);
    if (!v || !*v) return def;
    int n = atoi(v);
    return (n > 0 && n < 65536) ? n : def;
}


int sentai_uart_serial_open(void) {
    if (s_sock >= 0) {
        return 1;   /* already open */
    }
    const char* host = getenv("SENTAI_LINK_UDP_HOST");
    if (!host || !*host) host = "127.0.0.1";
    int remote_port = env_int("SENTAI_LINK_UDP_REMOTE_PORT", 14580);
    int local_port  = env_int("SENTAI_LINK_UDP_LOCAL_PORT",  14540);

    int sk = socket(AF_INET, SOCK_DGRAM, 0);
    if (sk < 0) {
        if ((s_open_fail_count++ % 100) == 0) {
            fprintf(stderr, "sentai_uart_udp: socket() failed errno=%d (cum=%u)\r\n",
                    errno, s_open_fail_count);
        }
        return 0;
    }

    /* Bind to local port so PX4 knows where to send replies */
    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons((uint16_t)local_port);
    if (bind(sk, (struct sockaddr*)&local, sizeof local) < 0) {
        if ((s_open_fail_count++ % 100) == 0) {
            fprintf(stderr, "sentai_uart_udp: bind(:%d) failed errno=%d (cum=%u)\r\n",
                    local_port, errno, s_open_fail_count);
        }
        close(sk);
        return 0;
    }

    /* Stash peer address — sendto() target.  PX4's offboard MAVLink
     * remote port is 14540 by default (PX4 init.d-posix/px4-rc.mavlink). */
    memset(&s_peer, 0, sizeof s_peer);
    s_peer.sin_family = AF_INET;
    s_peer.sin_port = htons((uint16_t)remote_port);
    if (inet_pton(AF_INET, host, &s_peer.sin_addr) != 1) {
        if ((s_open_fail_count++ % 100) == 0) {
            fprintf(stderr, "sentai_uart_udp: inet_pton('%s') failed (cum=%u)\r\n",
                    host, s_open_fail_count);
        }
        close(sk);
        return 0;
    }

    s_sock = sk;
    s_have_peer = 1;
    fprintf(stderr, "sentai_uart_udp: open OK — bind :%d → peer %s:%d\r\n",
            local_port, host, remote_port);
    return 1;
}


void sentai_uart_serial_close(void) {
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
        s_have_peer = 0;
        fprintf(stderr, "sentai_uart_udp: closed\r\n");
    }
}


int sentai_uart_serial_is_open(void) {
    return s_sock >= 0 ? 1 : 0;
}


int sentai_uart_serial_write(const uint8_t* buf, int size) {
    if (s_sock < 0 || !s_have_peer || size <= 0) return 0;
    if (size > UDP_BUF_SZ) size = UDP_BUF_SZ;   /* bound */
    ssize_t n = sendto(s_sock, buf, (size_t)size, 0,
                        (struct sockaddr*)&s_peer, sizeof s_peer);
    if (n < 0) {
        if ((s_io_fail_count++ % 100) == 0) {
            fprintf(stderr, "sentai_uart_udp: sendto errno=%d (cum=%u)\r\n",
                    errno, s_io_fail_count);
        }
        return 0;
    }
    return (int)n;
}


int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    if (s_sock < 0 || max_size <= 0) return 0;
    if (max_size > UDP_BUF_SZ) max_size = UDP_BUF_SZ;
    /* Bound timeout — refuse forever-block.  0 → poll-only, no wait. */
    if (timeout_ms < 0) timeout_ms = 0;
    if (timeout_ms > MAX_TIMEOUT_MS) timeout_ms = MAX_TIMEOUT_MS;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s_sock, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int sel = select(s_sock + 1, &rfds, NULL, NULL, &tv);
    if (sel <= 0) return 0;   /* timeout or error → no data this call */

    /* Capture sender so the FIRST datagram updates s_peer.  PX4 binds
     * an ephemeral src port; we want subsequent sends to go to the real
     * source, not the configured remote port (which PX4 may not use). */
    struct sockaddr_in src;
    socklen_t srclen = sizeof src;
    ssize_t n = recvfrom(s_sock, buf, (size_t)max_size, 0,
                          (struct sockaddr*)&src, &srclen);
    if (n < 0) {
        if ((s_io_fail_count++ % 100) == 0) {
            fprintf(stderr, "sentai_uart_udp: recvfrom errno=%d (cum=%u)\r\n",
                    errno, s_io_fail_count);
        }
        return 0;
    }
    /* Lock onto first responder's actual address — PX4 GZBridge/mavlink
     * uses ephemeral ports that don't match the configured remote. */
    if (srclen == sizeof src && src.sin_addr.s_addr != 0) {
        s_peer.sin_addr = src.sin_addr;
        s_peer.sin_port = src.sin_port;
    }
    return (int)n;
}


int sentai_uart_serial_available(void) {
    if (s_sock < 0) return 0;
    /* Poll without blocking */
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s_sock, &rfds);
    struct timeval zero = {0, 0};
    int sel = select(s_sock + 1, &rfds, NULL, NULL, &zero);
    return (sel > 0) ? 1 : 0;
}


void sentai_uart_set_baudrate(uint32_t baudrate) {
    /* No-op on UDP — kept for ABI compatibility with ARM build. */
    (void)baudrate;
}


void sentai_uart_restore_baudrate(void) {
    /* No-op. */
}
