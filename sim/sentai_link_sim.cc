/*
 * sentai_link_sim.cc — SIM-only slim MAVLink bridge (replaces the ARM
 * sentai_link.cc which depends on tracker/mesh/health/nanopb).
 *
 * Implements the SAME public C ABI as sentai_link.cc so that
 * modsentai_link.c (the MicroPython binding) and any `sentai.link.*`
 * call work unchanged in SIM.
 *
 * Transport: sentai_uart_serial_udp.c (UDP datagram pipe to PX4 SITL
 * MAVLink endpoint, default 127.0.0.1:14540).
 *
 * Phase 6 scope:
 *   - init / stop / debug-level
 *   - send_heartbeat (MAV_TYPE_ONBOARD_CONTROLLER by default)
 *   - send_statustext (severity + text)
 *   - background reader task: parses incoming heartbeats, counts them
 *     in g_link_stats, exposed via sentai.link.stats() for the ping test
 *
 * Future:
 *   - send_tunnel(payload_type, data, len) for REPL-over-MAVLink
 *   - rx callback hook so REPL bytes coming back can be fed into the
 *     MicroPython stdin queue
 *
 * NASA/JPL discipline:
 *   - All loops bounded (reader has bounded recv + sleep, no busy-wait)
 *   - All allocations static (no malloc per packet)
 *   - All returns checked
 *   - Reader task has a stop flag + bounded join
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <atomic>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" {
/* UART backend (sim/sentai_uart_serial_udp.c) — same ABI as ARM */
extern int  sentai_uart_serial_open(void);
extern void sentai_uart_serial_close(void);
extern int  sentai_uart_serial_is_open(void);
extern int  sentai_uart_serial_write(const uint8_t* buf, int size);
extern int  sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern void sentai_uart_set_baudrate(uint32_t baudrate);
extern void sentai_uart_restore_baudrate(void);
}

/* Pull in MAVLink common dialect (header-only, same as ARM build).
 * NOTE: MAVLINK_USE_CONVENIENCE_FUNCTIONS is checked via #ifdef, so it
 * must NOT be defined here.  We use mavlink_*_pack + mavlink_msg_to_send_buffer
 * which are the non-convenience APIs (no comm_send_ch dependency). */
#define MAVLINK_NO_CONVERSION_HELPERS
#define MAVLINK_NO_SIGNATURE_CHECK
#include "third_party/mavlink/common/mavlink.h"


/* ---- REPL-over-MAVLink TUNNEL bridge (Sim.md §10m.b) ---------------------
 *
 * Host (pymavlink) → sentai_sim REPL:
 *   sender wraps a REPL command in a MAVLink TUNNEL message with
 *   payload_type = SENTAI_REPL_PAYLOAD (0xC0DE, in the >32767 "local
 *   experimental" block).  Reader task below detects it and pushes
 *   bytes into the FIFO.  main_sim.c::sim_read_line() drains the FIFO
 *   alongside actual stdin, so MicroPython sees radio bytes as if they
 *   were typed locally.
 *
 * Bounded: 4 KB ring (~32 command lines), drop-on-overflow with a
 * dropped-byte counter (no infinite buffering).  Single producer
 * (reader task), single consumer (main REPL task) → lock-free SPSC
 * with std::atomic indices.
 */
#define SENTAI_REPL_PAYLOAD     0xC0DE
#define REPL_FIFO_SZ            4096

static char         s_repl_fifo[REPL_FIFO_SZ];
static std::atomic<uint32_t> s_repl_w{0};
static std::atomic<uint32_t> s_repl_r{0};
static std::atomic<uint32_t> s_repl_dropped{0};


static void repl_fifo_push(const uint8_t* src, size_t n) {
    /* Bounded loop: at most `n` iterations, each O(1). */
    for (size_t i = 0; i < n; ++i) {
        uint32_t w = s_repl_w.load(std::memory_order_relaxed);
        uint32_t nw = (w + 1) % REPL_FIFO_SZ;
        if (nw == s_repl_r.load(std::memory_order_acquire)) {
            /* Full — drop and count.  Better than blocking on a fifo
             * the consumer might not be draining (consumer = main MP
             * task which can be busy in an exec).  */
            s_repl_dropped.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        s_repl_fifo[w] = (char)src[i];
        s_repl_w.store(nw, std::memory_order_release);
    }
}


extern "C" int sentai_link_repl_rx_pop(char* out, int max_n) {
    if (!out || max_n <= 0) return 0;
    int popped = 0;
    while (popped < max_n) {
        uint32_t r = s_repl_r.load(std::memory_order_relaxed);
        if (r == s_repl_w.load(std::memory_order_acquire)) break;
        out[popped++] = s_repl_fifo[r];
        s_repl_r.store((r + 1) % REPL_FIFO_SZ, std::memory_order_release);
    }
    return popped;
}


extern "C" int sentai_link_send_tunnel(uint16_t payload_type,
                                         const uint8_t* data, int len,
                                         uint8_t target_sys,
                                         uint8_t target_comp) {
    if (!data || len <= 0) return 0;
    if (len > 128) len = 128;
    /* Static payload buffer to avoid memcpy from caller into stack. */
    uint8_t payload[128];
    memset(payload, 0, sizeof payload);
    memcpy(payload, data, (size_t)len);
    mavlink_message_t msg;
    extern uint8_t s_sysid_get(void);   /* fwd decl below */
    extern uint8_t s_compid_get(void);
    mavlink_msg_tunnel_pack(s_sysid_get(), s_compid_get(), &msg,
        target_sys, target_comp, payload_type, (uint8_t)len, payload);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int wlen = mavlink_msg_to_send_buffer(wire, &msg);
    return sentai_uart_serial_write(wire, wlen);
}


/* ---- Module state ---- */
struct LinkStats {
    std::atomic<uint32_t> tx_heartbeat{0};
    std::atomic<uint32_t> tx_statustext{0};
    std::atomic<uint32_t> tx_flow{0};
    std::atomic<uint32_t> rx_total{0};
    std::atomic<uint32_t> rx_heartbeat{0};
    std::atomic<uint32_t> rx_other{0};
    std::atomic<uint32_t> rx_parse_err{0};
    std::atomic<uint32_t> last_peer_sysid{0};
    std::atomic<uint32_t> last_peer_compid{0};
};
static LinkStats          s_stats;
static uint8_t            s_sysid  = 1;
static uint8_t            s_compid = 191;   /* MAV_COMP_ID_ONBOARD_COMPUTER */
static int                s_debug_level = 0;
static TaskHandle_t       s_reader_task = nullptr;
static std::atomic<bool>  s_reader_stop{false};
static std::atomic<bool>  s_open{false};

/* ---- Flow forwarder state (C-side, Python is on/off only) ---- */
static TaskHandle_t       s_fwd_task = nullptr;
static std::atomic<bool>  s_fwd_run{false};
static float              s_fwd_distance_m = 1.0f;

/* Accessors used by extern "C" send_tunnel above. */
extern "C" uint8_t s_sysid_get(void)  { return s_sysid; }
extern "C" uint8_t s_compid_get(void) { return s_compid; }


/* ---- Reader task: parses MAVLink frames coming back from PX4 ---- */
static void link_reader_task(void* arg) {
    (void)arg;
    uint8_t rxbuf[512];
    fprintf(stderr, "sentai.link: reader task started (s_sock=open=%d)\r\n",
            sentai_uart_serial_is_open());
    while (!s_reader_stop.load()) {
        int n = sentai_uart_serial_read(rxbuf, sizeof rxbuf, 100);   /* 100ms timeout */
        if (n <= 0) {
            /* No data → loop back (timeout already bounded inside the read). */
            continue;
        }
        s_stats.rx_total.fetch_add((uint32_t)n);
        /* Feed bytes through MAVLink parser, channel 0. */
        mavlink_message_t msg;
        mavlink_status_t  st;
        for (int i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rxbuf[i], &msg, &st)) {
                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    s_stats.rx_heartbeat.fetch_add(1);
                    s_stats.last_peer_sysid.store(msg.sysid);
                    s_stats.last_peer_compid.store(msg.compid);
                    if (s_debug_level >= 1) {
                        fprintf(stderr, "[link.rx] HEARTBEAT sys=%u comp=%u "
                                        "(tot=%u)\r\n",
                                msg.sysid, msg.compid,
                                s_stats.rx_heartbeat.load());
                    }
                } else if (msg.msgid == MAVLINK_MSG_ID_TUNNEL) {
                    mavlink_tunnel_t t;
                    mavlink_msg_tunnel_decode(&msg, &t);
                    if (t.payload_type == SENTAI_REPL_PAYLOAD) {
                        /* Target check: 0 = broadcast, our sysid = direct. */
                        if (t.target_system == 0 || t.target_system == s_sysid) {
                            uint8_t len = t.payload_length;
                            if (len > 128) len = 128;
                            /* Crazyflie-radio convention: only lines that
                             * start with '$' are executed (matches
                             * `$exec` pattern from
                             * feedback_radio_no_file_transfer.md memory).
                             * Strip the '$' before pushing so the REPL
                             * sees clean Python.  Lines without '$' are
                             * dropped silently — stray bytes won't run. */
                            const uint8_t* p = t.payload;
                            int n = (int)len;
                            if (n > 0 && p[0] == '$') {
                                /* Accept three forms (Crazyflie-radio
                                 * `$exec` convention):
                                 *   `$cmd`        → strip '$'
                                 *   `$ cmd`       → strip '$ '
                                 *   `$exec cmd`   → strip '$exec '
                                 */
                                p++; n--;
                                if (n >= 5 && memcmp(p, "exec ", 5) == 0) {
                                    p += 5; n -= 5;
                                } else if (n > 0 && p[0] == ' ') {
                                    p++; n--;
                                }
                                repl_fifo_push(p, (size_t)n);
                                if (s_debug_level >= 1) {
                                    fprintf(stderr, "[link.rx] REPL TUNNEL "
                                                    "%d bytes (after $-strip) "
                                                    "from sys=%u\r\n",
                                            n, msg.sysid);
                                }
                            } else {
                                if (s_debug_level >= 1) {
                                    fprintf(stderr, "[link.rx] TUNNEL %u "
                                                    "bytes DROPPED (no $ "
                                                    "marker)\r\n", len);
                                }
                            }
                        }
                    } else {
                        s_stats.rx_other.fetch_add(1);
                    }
                } else {
                    s_stats.rx_other.fetch_add(1);
                    if (s_debug_level >= 2) {
                        fprintf(stderr, "[link.rx] msgid=%u sys=%u comp=%u\r\n",
                                msg.msgid, msg.sysid, msg.compid);
                    }
                }
            }
            if (st.parse_error) {
                s_stats.rx_parse_err.fetch_add(1);
            }
        }
    }
    s_reader_task = nullptr;
    vTaskDelete(nullptr);
}


/* ---- Public C ABI (matches sentai_link.cc symbols) ---- */
extern "C" int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid) {
    if (s_open.load()) return 1;       /* already up */
    sentai_uart_set_baudrate(baudrate);     /* no-op in SIM, but ABI compat */
    if (!sentai_uart_serial_open()) {
        return 0;
    }
    s_sysid = sysid;
    s_compid = compid;
    /* Spawn the reader task — bounded stack, same prio as main MP task */
    s_reader_stop.store(false);
    BaseType_t ok = xTaskCreate(link_reader_task, "link_rx",
                                  configMINIMAL_STACK_SIZE * 4,
                                  nullptr, tskIDLE_PRIORITY + 2,
                                  &s_reader_task);
    fprintf(stderr, "sentai.link: xTaskCreate -> %s (handle=%p)\r\n",
            (ok == pdPASS) ? "OK" : "FAIL",
            (void*)s_reader_task);
    if (ok != pdPASS) {
        sentai_uart_serial_close();
        return 0;
    }
    s_open.store(true);
    fprintf(stderr, "sentai.link: init sys=%u comp=%u (UDP backend)\r\n",
            s_sysid, s_compid);
    return 1;
}


extern "C" int sentai_link_stop(void) {
    if (!s_open.load()) return 1;
    s_reader_stop.store(true);
    /* Bounded join: wait up to 500 ms for the task to exit on its own. */
    for (int i = 0; i < 50 && s_reader_task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    sentai_uart_serial_close();
    sentai_uart_restore_baudrate();
    s_open.store(false);
    return 1;
}


extern "C" void sentai_link_set_debug(int level) {
    if (level < 0) level = 0;
    if (level > 2) level = 2;
    s_debug_level = level;
}


extern "C" int sentai_link_send_heartbeat(uint8_t type) {
    if (!s_open.load()) return 0;
    mavlink_message_t msg;
    /* MAVLink defaults: state=ACTIVE (4), base_mode=0, custom_mode=0 */
    mavlink_msg_heartbeat_pack(s_sysid, s_compid, &msg,
        type,                             /* MAV_TYPE_*  */
        MAV_AUTOPILOT_INVALID,            /* non-autopilot — companion comp */
        MAV_MODE_FLAG_SAFETY_ARMED,       /* base_mode */
        0,                                /* custom_mode */
        MAV_STATE_ACTIVE);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    int w = sentai_uart_serial_write(buf, len);
    if (w > 0) s_stats.tx_heartbeat.fetch_add(1);
    if (s_debug_level >= 1) {
        fprintf(stderr, "[link.tx] HEARTBEAT type=%u len=%d (tot=%u)\r\n",
                type, w, s_stats.tx_heartbeat.load());
    }
    return w;
}


/* ---- High-level MAV_CMD wrappers (Phase 6b) ----
 *
 * Convenience C entry points for MicroPython bindings (modsentai_sim.c).
 * Each sends MAV_CMD_* via mavlink COMMAND_LONG to PX4 (target sysid=1).
 * Return 1 on send-success, 0 on link not open / send fail.
 *
 * Caller is responsible for waiting for COMMAND_ACK if needed (PX4 emits
 * ACKs in a separate MAVLink message we don't currently parse; for the
 * MVP we just send-and-pray — sufficient for arm/takeoff/land in
 * SITL).
 */
static int link_send_command_long(uint16_t command,
                                    float p1, float p2, float p3, float p4,
                                    float p5, float p6, float p7,
                                    uint8_t target_sys = 1,
                                    uint8_t target_comp = 1) {
    if (!s_open.load()) return 0;
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(s_sysid, s_compid, &msg,
        target_sys, target_comp,
        command,
        0,                  /* confirmation */
        p1, p2, p3, p4, p5, p6, p7);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int wlen = mavlink_msg_to_send_buffer(wire, &msg);
    int w = sentai_uart_serial_write(wire, wlen);
    if (s_debug_level >= 1) {
        fprintf(stderr, "[link.tx] CMD %u (p1=%.2f) → %d bytes\r\n",
                command, (double)p1, w);
    }
    return (w > 0) ? 1 : 0;
}


extern "C" int sentai_link_cmd_arm(int do_arm) {
    /* MAV_CMD_COMPONENT_ARM_DISARM (400): p1 = 1.0 arm / 0.0 disarm.
     * p2 = 21196 forces arm even if pre-arm checks fail (SITL convenience).
     */
    return link_send_command_long(400, do_arm ? 1.0f : 0.0f, 21196.0f,
                                    0,0,0,0,0);
}


extern "C" int sentai_link_cmd_takeoff(float altitude_m) {
    /* MAV_CMD_NAV_TAKEOFF (22): p7 = altitude (m).
     *
     * IMPORTANT (PX4 issue #21601): lat/lon=0 is "arbitrary coords"
     * and triggers "Disarmed by auto preflight disarming" or wild
     * flight behaviour.  Pass NaN for lat/lon/yaw → PX4 interprets
     * as "use current/home position".
     */
    return link_send_command_long(22,
        /* p1 min_pitch */ 0,
        /* p2 unused   */ 0,
        /* p3 unused   */ 0,
        /* p4 yaw      */ NAN,
        /* p5 lat      */ NAN,
        /* p6 lon      */ NAN,
        /* p7 alt      */ altitude_m);
}


extern "C" int sentai_link_cmd_land(void) {
    /* MAV_CMD_NAV_LAND (21): land at current XY, descend to ground. */
    return link_send_command_long(21, 0,0,0,0, 0,0,0);
}


extern "C" int sentai_link_send_flow(float dx_rad, float dy_rad,
                                       uint32_t dt_us, uint8_t quality,
                                       float distance_m) {
    /* MAVLINK_MSG_ID_OPTICAL_FLOW_RAD (106).  PX4 v1.14 EKF2 fuses
     * this when EKF2_AID_MASK has OPT_FLOW bit set AND the message
     * carries valid integration_time + distance.
     *
     * Fields (matches PX4 handle_message_optical_flow_rad):
     *   integrated_x / _y      → cumulative angular flow (radians)
     *   integration_time_us    → time window over which integrated
     *   quality                → 0..255, threshold by EKF2_OF_QMIN
     *   integrated_xgyro/ygyro/zgyro → IMU de-rotation (NaN to defer)
     *   distance               → rangefinder reading (m), <0 = unknown
     */
    if (!s_open.load()) return 0;
    mavlink_message_t msg;
    float nan_v = NAN;
    mavlink_msg_optical_flow_rad_pack(s_sysid, s_compid, &msg,
        (uint64_t)dt_us /* time_usec since boot (approx, PX4 ignores) */,
        0 /* sensor_id */,
        dt_us,
        dx_rad, dy_rad,
        nan_v, nan_v, nan_v /* integrated_xgyro/ygyro/zgyro — let PX4 derotate */,
        20 /* temperature, centidegC, unused */,
        quality,
        (uint32_t)dt_us /* time_delta_distance_us */,
        distance_m);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int wlen = mavlink_msg_to_send_buffer(wire, &msg);
    int w = sentai_uart_serial_write(wire, wlen);
    if (s_debug_level >= 2) {
        fprintf(stderr, "[link.tx] FLOW dx=%.4f dy=%.4f dt=%uus q=%u d=%.2f → %d\r\n",
                (double)dx_rad, (double)dy_rad, dt_us, quality, (double)distance_m, w);
    }
    return (w > 0) ? 1 : 0;
}


/* ---- C-side flow forwarder ----
 *
 * Reads g_flow snapshot published by sim/camera_bridge_recv.c, converts
 * mgrid → radians (1 L0 grid-px = 12.6 mrad @ HFOV 58°/640 × 8× decim,
 * 1 mgrid = 1/1000 grid-px → 12.6e-6 rad), and sends OPTICAL_FLOW_RAD
 * directly via the C send_flow path.
 *
 * Python role is on/off only — once started, the task runs at
 * tskIDLE_PRIORITY+2 (same as link reader) and never crosses MP VM.
 * Parity with ARM HW where Crazy radio bridge auto-inits in firmware
 * before /main.py (see memory: project_crazyflie_radio_bridge.md).
 */
extern "C" {
typedef struct {
    volatile uint32_t seq;
    volatile int32_t  dx_q1000;
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t  dz_q1000;
    volatile uint32_t dz_conf;
} sim_flow_snapshot_t;
const sim_flow_snapshot_t* sim_camera_flow_snapshot(void);
}

/* 1 L0 grid-pixel = 12.6 mrad (HFOV 58° / 640 px × 8× decimation).
 * Snapshot is in milli-grid (1000 = 1 grid-px), so:
 *   rad = (dx_q1000 / 1000) * 0.0126 = dx_q1000 * 12.6e-6
 */
static constexpr float MGRID_TO_RAD = 12.6e-6f;

static void link_flow_forward_task(void* arg) {
    (void)arg;
    const sim_flow_snapshot_t* fs = sim_camera_flow_snapshot();
    uint32_t last_seq = 0;
    TickType_t last_tick = xTaskGetTickCount();
    fprintf(stderr, "sentai.link: flow forwarder task started (dist=%.2fm)\r\n",
            (double)s_fwd_distance_m);
    while (s_fwd_run.load()) {
        /* Snapshot read with seq fence to detect tearing. */
        uint32_t s0, s1;
        int32_t  dx_q, dy_q;
        uint32_t conf;
        int retry = 2;
        do {
            s0 = fs->seq;
            dx_q = fs->dx_q1000;
            dy_q = fs->dy_q1000;
            conf = fs->conf;
            s1 = fs->seq;
        } while (s0 != s1 && --retry > 0);

        if (s0 != 0 && s0 != last_seq) {
            TickType_t now = xTaskGetTickCount();
            uint32_t dt_ms = (uint32_t)(now - last_tick);
            uint32_t dt_us = (dt_ms == 0) ? 1000U : (dt_ms * 1000U);
            last_tick = now;
            last_seq = s0;

            float dx_rad = (float)dx_q * MGRID_TO_RAD;
            float dy_rad = (float)dy_q * MGRID_TO_RAD;
            uint8_t q = (conf > 255) ? 255 : (uint8_t)conf;

            if (sentai_link_send_flow(dx_rad, dy_rad, dt_us, q,
                                       s_fwd_distance_m)) {
                s_stats.tx_flow.fetch_add(1);
            }
            if (s_debug_level >= 2) {
                fprintf(stderr, "[link.fwd] seq=%u dx=%.4f dy=%.4f dt=%uus "
                                "q=%u d=%.2f\r\n",
                        s0, (double)dx_rad, (double)dy_rad, dt_us, q,
                        (double)s_fwd_distance_m);
            }
        }
        /* Bounded poll — 50 Hz cap matches typical camera bridge cadence. */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_fwd_task = nullptr;
    fprintf(stderr, "sentai.link: flow forwarder task exiting\r\n");
    vTaskDelete(nullptr);
}


extern "C" int sentai_link_flow_forward(int enable) {
    if (enable) {
        if (s_fwd_run.load()) return 1;     /* already running */
        if (!s_open.load()) return 0;       /* link must be up first */
        s_fwd_run.store(true);
        BaseType_t ok = xTaskCreate(link_flow_forward_task, "link_fwd",
                                      configMINIMAL_STACK_SIZE * 4,
                                      nullptr, tskIDLE_PRIORITY + 2,
                                      &s_fwd_task);
        if (ok != pdPASS) {
            s_fwd_run.store(false);
            return 0;
        }
        return 1;
    } else {
        if (!s_fwd_run.load()) return 1;    /* already stopped */
        s_fwd_run.store(false);
        /* Bounded join: wait up to 500 ms for task to exit. */
        for (int i = 0; i < 50 && s_fwd_task != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return 1;
    }
}


extern "C" int sentai_link_flow_set_distance(float dist_m) {
    if (dist_m < 0.05f) dist_m = 0.05f;
    if (dist_m > 100.0f) dist_m = 100.0f;
    s_fwd_distance_m = dist_m;
    return 1;
}


extern "C" int sentai_link_cmd_set_mode(uint8_t main_mode, uint8_t sub_mode) {
    /* MAV_CMD_DO_SET_MODE (176).  PX4 custom_mode packing:
     *   p1 = base_mode (1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED).
     *   p2 = main_mode (PX4_CUSTOM_MAIN_MODE_*: 1=MANUAL, 2=ALTCTL,
     *        3=POSCTL, 4=AUTO, 5=ACRO, 6=OFFBOARD, 7=STABILIZED, 8=RATTITUDE)
     *   p3 = sub_mode (for AUTO: 2=TAKEOFF, 3=LOITER, 5=LAND, etc.)
     */
    return link_send_command_long(176, 1.0f /* CUSTOM enabled */,
                                    (float)main_mode, (float)sub_mode,
                                    0,0,0,0);
}


extern "C" int sentai_link_send_statustext(uint8_t severity, const char* text) {
    if (!s_open.load() || !text) return 0;
    /* STATUSTEXT is 50 chars max; bounded copy. */
    char buf50[51] = {0};
    size_t n = strnlen(text, 50);
    memcpy(buf50, text, n);
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(s_sysid, s_compid, &msg,
        severity, buf50, 0 /* id */, 0 /* chunk_seq */);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(wire, &msg);
    int w = sentai_uart_serial_write(wire, len);
    if (w > 0) s_stats.tx_statustext.fetch_add(1);
    return w;
}


/* ---- Stats accessor for Python diag ---- */
extern "C" void sentai_link_get_stats(uint32_t out[9]) {
    if (!out) return;
    out[0] = s_stats.tx_heartbeat.load();
    out[1] = s_stats.tx_statustext.load();
    out[2] = s_stats.rx_total.load();
    out[3] = s_stats.rx_heartbeat.load();
    out[4] = s_stats.rx_other.load();
    out[5] = s_stats.rx_parse_err.load();
    out[6] = s_stats.last_peer_sysid.load();
    out[7] = s_stats.last_peer_compid.load();
    out[8] = s_stats.tx_flow.load();
}


/* ---- Stubs for unused-in-SIM bits referenced by modsentai_link.c ---- */
extern "C" int sentai_console_get_target(void) {
    /* SIM has stdin/stdout REPL only — no USB/UART switching.  Return 0
     * (= "REPL on USB") so the precondition check in modsentai_link.c
     * passes. */
    return 0;
}

extern "C" int sentai_mesh_is_running(void) {
    /* No mesh in SIM. */
    return 0;
}

extern "C" uint32_t sentai_mesh_my_node_num(void) {
    return 0;
}


/* ---- Detection/Update senders — SIM stub.  Real impl lives on ARM in
 *      sentai_link.cc; SIM just no-ops with success so modsentai_link.c
 *      doesn't choke on link errors.  REPL test doesn't need detections. */
extern "C" int sentai_link_send_vision(
    uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
    uint8_t, uint8_t, uint8_t, uint8_t,
    uint32_t, uint32_t,
    int32_t, int32_t, int16_t, uint8_t) {
    return 0;
}

extern "C" int sentai_link_send_vision_update(
    uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
    uint8_t, uint8_t, uint8_t, uint8_t,
    uint32_t, uint32_t,
    int32_t, int32_t, uint8_t) {
    return 0;
}
