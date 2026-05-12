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


/* ---- Module state ---- */
struct LinkStats {
    std::atomic<uint32_t> tx_heartbeat{0};
    std::atomic<uint32_t> tx_statustext{0};
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
extern "C" void sentai_link_get_stats(uint32_t out[8]) {
    if (!out) return;
    out[0] = s_stats.tx_heartbeat.load();
    out[1] = s_stats.tx_statustext.load();
    out[2] = s_stats.rx_total.load();
    out[3] = s_stats.rx_heartbeat.load();
    out[4] = s_stats.rx_other.load();
    out[5] = s_stats.rx_parse_err.load();
    out[6] = s_stats.last_peer_sysid.load();
    out[7] = s_stats.last_peer_compid.load();
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
