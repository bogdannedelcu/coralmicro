/*
 * sentai_crazy_sim.cc — SIM-only Crazyflie bridge over CRTP-UDP.
 *
 * Implements the same C ABI as examples/sentai_runtime/sentai_crazy.h so
 * MicroPython missions written for the ARM target can run unchanged in
 * SIM.  Transport is POSIX UDP to cf2 SITL (default 127.0.0.1:19850 —
 * Bitcraze cflib UdpDriver convention).
 *
 * Wire format (cflib/crtp/udpdriver.py + crtpstack.py):
 *   datagram = [header_byte] + [0..30 payload bytes]
 *   header   = (port & 0x0F) << 4 | 3 << 2 | (channel & 0x03)
 *
 * Service codes mirrored from cflib:
 *   port 0x08 SETPOINT_HL ch 0 — high-level commander (takeoff/land/go_to/stop)
 *   port 0x09 SUPERVISOR  ch 1 — arming command (CMD_ARM_SYSTEM=1)
 *   port 0x07 COMMANDER_GENERIC ch 0 — hover setpoint (type=5)
 *
 * NASA/JPL discipline (embeded.md §2):
 *   - All loops bounded (RX task uses select() with timeout, drains FIFO)
 *   - All allocations static (single rx ring, no malloc per packet)
 *   - All returns checked (send_crtp_raw validates len + returns)
 *   - Single-writer ring is lock-free SPSC with std::atomic indices
 *   - Bounded shutdown: stop() waits ≤500 ms for RX task to exit
 *
 * Scope cut for v1 (this commit):
 *   - send/receive raw CRTP, HL commander, arm/disarm, hover
 *   - NO CPX framing (UDP carries CRTP directly, no UART CTS)
 *   - NO log subscription yet (MP can compose via send_crtp + recv_crtp)
 *   - NO param TOC scan, NO test_fly, NO attitude/fly_stop (CPX-only features)
 *     -> these return -ENOTSUP-like sentinels so mission code that
 *        accidentally hits them fails loudly instead of hanging.
 */
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

/* ===================== Configuration ===================== */
#define CF2_SITL_HOST          "127.0.0.1"
#define CF2_SITL_PORT          19850
#define CRTP_MAX_PAYLOAD       30
#define CRTP_PKT_MAX           31           /* 1 header + 30 payload */
#define RX_RING_SZ             32           /* power-of-2 friendly bound */

/* CRTP port numbers (mirror cflib/crtp/crtpstack.py CRTPPort) */
#define CRTP_PORT_CONSOLE      0x00
#define CRTP_PORT_PARAM        0x02
#define CRTP_PORT_COMMANDER    0x03
#define CRTP_PORT_LOGGING      0x05
#define CRTP_PORT_GENERIC      0x07
#define CRTP_PORT_SETPOINT_HL  0x08
#define CRTP_PORT_SUPERVISOR   0x09
#define CRTP_PORT_PLATFORM     0x0D
#define CRTP_PORT_LINKCTRL     0x0F

/* HL Commander sub-commands (cflib/crazyflie/high_level_commander.py) */
#define HL_CMD_SET_GROUP_MASK  0
#define HL_CMD_STOP            3
#define HL_CMD_GO_TO           4
#define HL_CMD_TAKEOFF_2       7
#define HL_CMD_LAND_2          8

/* Supervisor sub-commands (cflib/crazyflie/supervisor.py) */
#define SUPERVISOR_CMD_ARM     1     /* CMD_ARM_SYSTEM */
#define SUPERVISOR_CH_COMMAND  1

/* Generic Commander hover packet type (firmware
 * src/modules/src/crtp_commander_generic.c — typeHover=5) */
#define GENERIC_HOVER_TYPE     5

/* ===================== Module state ===================== */
struct rx_pkt_t {
    uint8_t  port;
    uint8_t  ch;
    uint8_t  len;
    uint8_t  data[CRTP_MAX_PAYLOAD];
};

static int                       s_sock = -1;
static std::atomic<bool>         s_open{false};
static std::atomic<bool>         s_rx_stop{false};
static TaskHandle_t              s_rx_task = nullptr;
static int                       s_debug = 0;

static rx_pkt_t                  s_rx_ring[RX_RING_SZ];
static std::atomic<uint32_t>     s_rx_w{0};
static std::atomic<uint32_t>     s_rx_r{0};
static std::atomic<uint32_t>     s_rx_dropped{0};
static std::atomic<uint32_t>     s_tx_count{0};
static std::atomic<uint32_t>     s_rx_count{0};

/* ===================== Helpers ===================== */
static inline uint8_t crtp_header(uint8_t port, uint8_t channel) {
    /* Bit layout: [port:4][reserved:2=11][channel:2] */
    return (uint8_t)(((port & 0x0F) << 4) | 0x0C | (channel & 0x03));
}

static inline void pack_f32(uint8_t* p, float v) {
    /* x86 is LE, cf2 firmware on Cortex-M is LE.  Direct memcpy. */
    memcpy(p, &v, 4);
}

static int send_crtp_raw(uint8_t port, uint8_t channel,
                          const uint8_t* data, int len) {
    if (!s_open.load() || s_sock < 0) return -1;
    if (len < 0 || len > CRTP_MAX_PAYLOAD) return -2;
    uint8_t buf[CRTP_PKT_MAX];
    buf[0] = crtp_header(port, channel);
    if (len > 0 && data) memcpy(buf + 1, data, (size_t)len);
    ssize_t w = send(s_sock, buf, (size_t)(1 + len), 0);
    if (w != (ssize_t)(1 + len)) return -3;
    s_tx_count.fetch_add(1, std::memory_order_relaxed);
    if (s_debug >= 2) {
        fprintf(stderr, "[crazy.tx] port=0x%02X ch=%u len=%d hdr=0x%02X\r\n",
                port, channel, len, buf[0]);
    }
    return 0;
}

/* ===================== RX task ===================== */
static void crazy_rx_task(void* arg) {
    (void)arg;
    uint8_t buf[1024];
    fprintf(stderr, "sentai.crazy: rx task started (sock=%d, %s:%d)\r\n",
            s_sock, CF2_SITL_HOST, CF2_SITL_PORT);
    while (!s_rx_stop.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s_sock, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100 * 1000;   /* 100 ms — bounded poll */
        int sel = select(s_sock + 1, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;
        ssize_t r = recv(s_sock, buf, sizeof(buf), 0);
        if (r < 1) continue;
        s_rx_count.fetch_add(1, std::memory_order_relaxed);
        uint8_t hdr = buf[0];
        uint8_t port = (uint8_t)((hdr >> 4) & 0x0F);
        uint8_t ch   = (uint8_t)(hdr & 0x03);
        int payload_len = (int)r - 1;
        if (payload_len < 0) payload_len = 0;
        if (payload_len > CRTP_MAX_PAYLOAD) payload_len = CRTP_MAX_PAYLOAD;
        /* SPSC ring push (drop on full — better than blocking the network
         * thread on a slow MP consumer). */
        uint32_t w = s_rx_w.load(std::memory_order_relaxed);
        uint32_t nw = (w + 1) % RX_RING_SZ;
        if (nw == s_rx_r.load(std::memory_order_acquire)) {
            s_rx_dropped.fetch_add(1, std::memory_order_relaxed);
            if (s_debug >= 1) {
                fprintf(stderr, "[crazy.rx] FIFO full, dropping pkt port=0x%02X\r\n",
                        port);
            }
            continue;
        }
        s_rx_ring[w].port = port;
        s_rx_ring[w].ch   = ch;
        s_rx_ring[w].len  = (uint8_t)payload_len;
        if (payload_len > 0) {
            memcpy(s_rx_ring[w].data, buf + 1, (size_t)payload_len);
        }
        s_rx_w.store(nw, std::memory_order_release);
        if (s_debug >= 2) {
            fprintf(stderr, "[crazy.rx] port=0x%02X ch=%u len=%d\r\n",
                    port, ch, payload_len);
        }
    }
    s_rx_task = nullptr;
    fprintf(stderr, "sentai.crazy: rx task exiting\r\n");
    vTaskDelete(nullptr);
}

/* ===================== Public C ABI ===================== */
extern "C" int sentai_crazy_init(uint32_t baudrate) {
    (void)baudrate;   /* UDP has no baud — kept for ABI compat */
    if (s_open.load()) return 0;       /* idempotent */

    s_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_sock < 0) {
        fprintf(stderr, "sentai.crazy: socket() failed: %s\r\n", strerror(errno));
        return -1;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(CF2_SITL_PORT);
    if (inet_pton(AF_INET, CF2_SITL_HOST, &addr.sin_addr) != 1) {
        close(s_sock); s_sock = -1; return -2;
    }
    if (connect(s_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "sentai.crazy: connect() failed: %s\r\n", strerror(errno));
        close(s_sock); s_sock = -1; return -3;
    }
    /* Probe packet (cflib scan convention): single 0xFF wakes up the
     * cf2 SITL UDP server.  Failure here is non-fatal — server may
     * already be active. */
    uint8_t probe = 0xFF;
    (void)send(s_sock, &probe, 1, 0);

    /* Spawn RX task. */
    s_rx_w.store(0); s_rx_r.store(0);
    s_rx_dropped.store(0); s_tx_count.store(0); s_rx_count.store(0);
    s_rx_stop.store(false);
    BaseType_t ok = xTaskCreate(crazy_rx_task, "crazy_rx",
                                  configMINIMAL_STACK_SIZE * 4,
                                  nullptr, tskIDLE_PRIORITY + 2,
                                  &s_rx_task);
    if (ok != pdPASS) {
        close(s_sock); s_sock = -1; return -4;
    }
    s_open.store(true);
    fprintf(stderr, "sentai.crazy: init OK (UDP %s:%d)\r\n",
            CF2_SITL_HOST, CF2_SITL_PORT);
    return 0;
}

extern "C" int sentai_crazy_stop(void) {
    if (!s_open.load()) return 0;
    s_rx_stop.store(true);
    /* Bounded join (≤500 ms) — RX task wakes from select() within 100 ms. */
    for (int i = 0; i < 50 && s_rx_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_sock >= 0) { close(s_sock); s_sock = -1; }
    s_open.store(false);
    return 0;
}

extern "C" int sentai_crazy_is_running(void) {
    return s_open.load() ? 1 : 0;
}

extern "C" void sentai_crazy_set_debug(int level) {
    if (level < 0) level = 0;
    if (level > 2) level = 2;
    s_debug = level;
}

/* ---------- Arm / disarm: Supervisor port 0x09 channel 1 ----------
 * Wire (cflib supervisor.py line 277-278):
 *   pk.set_header(SUPERVISOR, SUPERVISOR_CH_COMMAND)
 *   pk.data = (CMD_ARM_SYSTEM=1, do_arm)
 * payload = <BB> (cmd, bool) — 2 bytes
 */
extern "C" int sentai_crazy_arm(void) {
    uint8_t p[2] = { SUPERVISOR_CMD_ARM, 1 };
    return send_crtp_raw(CRTP_PORT_SUPERVISOR, SUPERVISOR_CH_COMMAND, p, 2);
}

extern "C" int sentai_crazy_disarm(void) {
    uint8_t p[2] = { SUPERVISOR_CMD_ARM, 0 };
    return send_crtp_raw(CRTP_PORT_SUPERVISOR, SUPERVISOR_CH_COMMAND, p, 2);
}

/* ---------- HL Commander: takeoff (CMD_TAKEOFF_2=7) ----------
 * struct.pack('<BBff?f', cmd, group, height, yaw, use_current_yaw, duration)
 * 1+1+4+4+1+4 = 15 bytes
 */
extern "C" int sentai_crazy_takeoff(float height, float duration,
                                     float yaw, int use_current_yaw,
                                     uint8_t group_mask) {
    uint8_t p[15];
    p[0]  = HL_CMD_TAKEOFF_2;
    p[1]  = group_mask;
    pack_f32(p + 2,  height);
    pack_f32(p + 6,  use_current_yaw ? 0.0f : yaw);
    p[10] = use_current_yaw ? 1 : 0;
    pack_f32(p + 11, duration);
    return send_crtp_raw(CRTP_PORT_SETPOINT_HL, 0, p, 15);
}

extern "C" int sentai_crazy_land(float height, float duration,
                                  float yaw, int use_current_yaw,
                                  uint8_t group_mask) {
    uint8_t p[15];
    p[0]  = HL_CMD_LAND_2;
    p[1]  = group_mask;
    pack_f32(p + 2,  height);
    pack_f32(p + 6,  use_current_yaw ? 0.0f : yaw);
    p[10] = use_current_yaw ? 1 : 0;
    pack_f32(p + 11, duration);
    return send_crtp_raw(CRTP_PORT_SETPOINT_HL, 0, p, 15);
}

/* ---------- HL Commander: stop (CMD_STOP=3) ----------
 * struct.pack('<BB', cmd, group)  — 2 bytes, motors off.
 */
extern "C" int sentai_crazy_stop_motors(uint8_t group_mask) {
    uint8_t p[2] = { HL_CMD_STOP, group_mask };
    return send_crtp_raw(CRTP_PORT_SETPOINT_HL, 0, p, 2);
}

/* ---------- HL Commander: go_to (legacy CMD_GO_TO=4) ----------
 * struct.pack('<BBBfffff', cmd, group, relative, x, y, z, yaw, duration)
 * 1+1+1+4*5 = 23 bytes.
 * Note: legacy GO_TO has no `linear` flag — that's COMMAND_GO_TO_2 in
 *       protocol v8+.  We use the legacy form for max compatibility with
 *       older SITL builds.
 */
extern "C" int sentai_crazy_hl_stop(uint8_t group_mask) {
    /* HL_CMD_STOP — puts the HL Commander into IDLE state.  Generic
     * Setpoints (hover/attitude) then become the authoritative
     * setpoint source.  Caller responsibility: keep sending Generic
     * Setpoints at 10+ Hz or motors cut after ~1 s watchdog. */
    uint8_t p[2];
    p[0] = HL_CMD_STOP;
    p[1] = group_mask;
    return send_crtp_raw(CRTP_PORT_SETPOINT_HL, 0, p, 2);
}

extern "C" int sentai_crazy_go_to(float x, float y, float z, float yaw,
                                   float duration, int relative, int linear,
                                   uint8_t group_mask) {
    (void)linear;   /* legacy GO_TO does not carry this flag */
    uint8_t p[23];
    p[0] = HL_CMD_GO_TO;
    p[1] = group_mask;
    p[2] = relative ? 1 : 0;
    pack_f32(p + 3,  x);
    pack_f32(p + 7,  y);
    pack_f32(p + 11, z);
    pack_f32(p + 15, yaw);
    pack_f32(p + 19, duration);
    return send_crtp_raw(CRTP_PORT_SETPOINT_HL, 0, p, 23);
}

/* ---------- Generic Commander: hover (typeHover=5) ----------
 * struct.pack('<Bffff', type, vx, vy, yaw_rate, z) — 17 bytes.
 * Must be sent CONTINUOUSLY at ~10-20 Hz, else cf2 watchdog cuts motors.
 */
extern "C" int sentai_crazy_hover(float vx, float vy, float yaw_rate,
                                   float z_distance) {
    uint8_t p[17];
    p[0] = GENERIC_HOVER_TYPE;
    pack_f32(p + 1,  vx);
    pack_f32(p + 5,  vy);
    pack_f32(p + 9,  yaw_rate);
    pack_f32(p + 13, z_distance);
    return send_crtp_raw(CRTP_PORT_GENERIC, 0, p, 17);
}

/* ---------- Raw CRTP send ---------- */
extern "C" int sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                                       const uint8_t* data, int len) {
    return send_crtp_raw(port, channel, data, len);
}

/* ---------- RX FIFO pop (used by MP binding to drain incoming) ----------
 * Returns 1 if a packet was popped, 0 if empty.
 * On success fills *port, *ch, *out_len; copies up to max_len into data[].
 */
extern "C" int sentai_crazy_recv_pop(uint8_t* port, uint8_t* ch,
                                      uint8_t* data, int max_len, int* out_len) {
    if (!port || !ch || !out_len) return 0;
    *out_len = 0;
    uint32_t r = s_rx_r.load(std::memory_order_relaxed);
    if (r == s_rx_w.load(std::memory_order_acquire)) return 0;
    rx_pkt_t* p = &s_rx_ring[r];
    *port = p->port;
    *ch   = p->ch;
    int n = (int)p->len;
    if (n > max_len) n = max_len;
    if (n > 0 && data) memcpy(data, p->data, (size_t)n);
    *out_len = n;
    s_rx_r.store((r + 1) % RX_RING_SZ, std::memory_order_release);
    return 1;
}

/* ---------- Diagnostics ----------
 * out[0]=tx_count, out[1]=rx_count, out[2]=rx_dropped, out[3]=is_running.
 */
extern "C" void sentai_crazy_get_stats(uint32_t out[4]) {
    if (!out) return;
    out[0] = s_tx_count.load();
    out[1] = s_rx_count.load();
    out[2] = s_rx_dropped.load();
    out[3] = s_open.load() ? 1u : 0u;
}

/* ===================== ABI stubs (unused-in-SIM) =====================
 * These are CPX/UART-only features on the ARM side.  In SIM the UDP
 * transport gives us bidirectional CRTP for free — no CTS, no echo
 * ping needed.  Return a clearly-negative sentinel so callers that
 * accidentally hit these in SIM fail loudly. */
extern "C" int sentai_crazy_ping(int timeout_ms) {
    (void)timeout_ms;
    return -99;   /* unsupported in SIM */
}

extern "C" int sentai_crazy_test_fly(uint16_t power, int duration_ms) {
    (void)power; (void)duration_ms;
    return -99;
}

extern "C" int sentai_crazy_fly(float height_m, int hold_ms,
                                 int takeoff_ms, int land_ms) {
    (void)height_m; (void)hold_ms; (void)takeoff_ms; (void)land_ms;
    return -99;
}

extern "C" int sentai_crazy_attitude(float roll, float pitch,
                                      float yawrate, uint16_t thrust) {
    (void)roll; (void)pitch; (void)yawrate; (void)thrust;
    return -99;
}

extern "C" int sentai_crazy_fly_stop(void) { return -99; }

extern "C" float sentai_crazy_get_altitude(void) { return -999.0f; }

/* No CPX telemetry channel in SIM — log subscription will be done from
 * MicroPython via send_crtp(LOGGING_PORT, ...) when needed. */
extern "C" int sentai_crazy_query_telemetry(uint8_t cmd, float* out,
                                             int timeout_ms) {
    (void)cmd; (void)timeout_ms; (void)out;
    return -99;
}
extern "C" int sentai_crazy_set_telem(uint8_t cmd, float value,
                                       float* out_readback, int timeout_ms) {
    (void)cmd; (void)value; (void)out_readback; (void)timeout_ms;
    return -99;
}
