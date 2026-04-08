// MAVLink v2 bridge — uses official c_library_v2 (header-only)
//
// RX: FreeRTOS task feeds UART bytes into mavlink_parse_char(), queues
//     decoded messages as link_rx_msg_t (wrapping mavlink_message_t).
// TX: pack with mavlink_msg_*_pack(), serialize with mavlink_msg_to_send_buffer(),
//     write to UART under mutex.
// VisionMessage TX: nanopb-encode → base64 → chunked STATUSTEXT (id + chunk_seq).

#include "sentai_link.h"
#include "sentai_tracker.h"
#include "sentai_vision_common.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/queue.h"
#include "third_party/freertos_kernel/include/semphr.h"

#include <pb_encode.h>
#include "generated/visionmesh.pb.h"

// Existing UART bridge functions (from modsentai_hal.cc)
extern "C" {
extern int  sentai_uart_serial_open(void);
extern void sentai_uart_serial_close(void);
extern int  sentai_uart_serial_is_open(void);
extern int  sentai_uart_serial_write(const uint8_t* buf, int size);
extern int  sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern int  sentai_uart_serial_available(void);
extern void sentai_uart_set_baudrate(uint32_t baudrate);
extern void sentai_uart_restore_baudrate(void);
extern uint32_t sentai_mesh_my_node_num(void);
}

// ===================== Module state =====================
static volatile int      g_link_running = 0;
static volatile int      g_link_debug = 0;  // 0=off, 1=summary, 2=hex
static TaskHandle_t      g_link_rx_task = nullptr;
static QueueHandle_t     g_link_rx_queue = nullptr;
static SemaphoreHandle_t g_link_tx_mutex = nullptr;
static uint8_t           g_link_sysid = 1;
static uint8_t           g_link_compid = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
static uint16_t          g_link_vision_msg_id = 1; // auto-increment for STATUSTEXT chunking

// ===================== Base64 encoder =====================
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t* src, int src_len, char* dst, int dst_max) {
    int j = 0;
    for (int i = 0; i < src_len && j + 4 <= dst_max; i += 3) {
        uint32_t a = src[i];
        uint32_t b = (i + 1 < src_len) ? src[i + 1] : 0;
        uint32_t c = (i + 2 < src_len) ? src[i + 2] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        dst[j++] = b64_table[(triple >> 18) & 0x3F];
        dst[j++] = b64_table[(triple >> 12) & 0x3F];
        dst[j++] = (i + 1 < src_len) ? b64_table[(triple >> 6) & 0x3F] : '=';
        dst[j++] = (i + 2 < src_len) ? b64_table[triple & 0x3F] : '=';
    }
    if (j < dst_max) dst[j] = '\0';
    return j;
}

// ===================== TX helper: serialize + UART write =====================
static int link_send_msg(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (len == 0) return -1;

    if (g_link_debug >= 1) {
        printf("[link] TX msgid=%lu sysid=%u compid=%u seq=%u len=%u wire=%u\r\n",
               (unsigned long)msg->msgid, msg->sysid, msg->compid,
               msg->seq, msg->len, len);
    }
    if (g_link_debug >= 2) {
        printf("[link] TX hex:");
        for (int i = 0; i < (int)len && i < 32; i++)
            printf(" %02X", buf[i]);
        if (len > 32) printf(" ...");
        printf("\r\n");
    }

    xSemaphoreTake(g_link_tx_mutex, portMAX_DELAY);
    int n = sentai_uart_serial_write(buf, (int)len);
    xSemaphoreGive(g_link_tx_mutex);

    if (g_link_debug >= 1 && n != (int)len) {
        printf("[link] TX FAIL: wrote %d/%u\r\n", n, len);
    }

    return (n == (int)len) ? 0 : -2;
}

// ===================== RX task =====================
static void link_rx_task(void* param) {
    (void)param;
    mavlink_message_t rx_msg;
    mavlink_status_t  rx_status;

    printf("[link] RX task started\r\n");

    while (g_link_running) {
        uint8_t buf[64];
        int n = sentai_uart_serial_read(buf, sizeof(buf), 50);
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx_msg, &rx_status)) {
                // Got a complete, CRC-valid message
                if (g_link_debug >= 1) {
                    printf("[link] RX msgid=%lu sysid=%u compid=%u seq=%u len=%u\r\n",
                           (unsigned long)rx_msg.msgid, rx_msg.sysid,
                           rx_msg.compid, rx_msg.seq, rx_msg.len);
                }
                link_rx_msg_t item;
                memcpy(&item.msg, &rx_msg, sizeof(mavlink_message_t));
                xQueueSend(g_link_rx_queue, &item, 0);
            }
        }
    }

    printf("[link] RX task stopped\r\n");
    vTaskDelete(nullptr);
}

// ===================== Public API =====================

extern "C" int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid) {
    if (g_link_running) return 0;  // already running

    g_link_sysid = sysid;
    g_link_compid = compid;

    // Set baudrate if non-default
    if (baudrate != 115200 && baudrate > 0) {
        sentai_uart_set_baudrate(baudrate);
    }

    // Open UART
    if (!sentai_uart_serial_open()) {
        printf("[link] UART open failed\r\n");
        return -1;
    }

    // Create queue and mutex
    g_link_rx_queue = xQueueCreate(LINK_RX_QUEUE_DEPTH, sizeof(link_rx_msg_t));
    g_link_tx_mutex = xSemaphoreCreateMutex();

    if (!g_link_rx_queue || !g_link_tx_mutex) {
        printf("[link] queue/mutex create failed\r\n");
        sentai_uart_serial_close();
        return -2;
    }

    g_link_running = 1;
    g_link_vision_msg_id = (uint16_t)(xTaskGetTickCount() & 0xFFFF);

    // Create RX task (priority just above idle, 6 KB stack for MAVLink parser)
    BaseType_t rc = xTaskCreate(link_rx_task, "link_rx", 6144 / sizeof(StackType_t),
                                nullptr, tskIDLE_PRIORITY + 2, &g_link_rx_task);
    if (rc != pdPASS) {
        printf("[link] task create failed\r\n");
        g_link_running = 0;
        sentai_uart_serial_close();
        return -3;
    }

    printf("[link] initialized at %lu baud, sysid=%u compid=%u\r\n",
           (unsigned long)baudrate, sysid, compid);
    return 0;
}

extern "C" int sentai_link_stop(void) {
    if (!g_link_running) return 0;

    g_link_running = 0;
    vTaskDelay(pdMS_TO_TICKS(300));

    if (g_link_rx_queue) { vQueueDelete(g_link_rx_queue); g_link_rx_queue = nullptr; }
    if (g_link_tx_mutex) { vSemaphoreDelete(g_link_tx_mutex); g_link_tx_mutex = nullptr; }

    sentai_uart_restore_baudrate();
    sentai_uart_serial_close();

    printf("[link] stopped\r\n");
    return 0;
}

extern "C" int sentai_link_is_running(void) {
    return g_link_running;
}

extern "C" int sentai_link_available(void) {
    if (!g_link_rx_queue) return 0;
    return (int)uxQueueMessagesWaiting(g_link_rx_queue);
}

extern "C" int sentai_link_receive(link_rx_msg_t* msg) {
    if (!g_link_rx_queue || !msg) return 0;
    return (xQueueReceive(g_link_rx_queue, msg, 0) == pdTRUE) ? 1 : 0;
}

extern "C" int sentai_link_receive_wait(link_rx_msg_t* msg, int timeout_ms) {
    if (!g_link_rx_queue || !msg) return 0;
    TickType_t ticks;
    if (timeout_ms < 0)       ticks = portMAX_DELAY;
    else if (timeout_ms == 0) ticks = 0;
    else                      ticks = pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive(g_link_rx_queue, msg, ticks) == pdTRUE) ? 1 : 0;
}

// ===================== TX: Heartbeat =====================
extern "C" int sentai_link_send_heartbeat(uint8_t type) {
    if (!g_link_running) return -1;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        g_link_sysid, g_link_compid, &msg,
        type,                               // MAV_TYPE
        MAV_AUTOPILOT_INVALID,              // not a flight controller
        0,                                  // base_mode
        0,                                  // custom_mode
        MAV_STATE_ACTIVE                    // system_status
    );
    return link_send_msg(&msg);
}

// ===================== TX: StatusText (single) =====================
extern "C" int sentai_link_send_statustext(uint8_t severity, const char* text) {
    if (!g_link_running) return -1;

    mavlink_message_t msg;
    // The library expects char text[50], we pass the string directly.
    // mavlink_msg_statustext_pack will copy up to 50 chars.
    mavlink_msg_statustext_pack(
        g_link_sysid, g_link_compid, &msg,
        severity,
        text,
        0,      // id=0 means single message (not chunked)
        0       // chunk_seq=0
    );
    return link_send_msg(&msg);
}

// ===================== TX: Chunked STATUSTEXT with base64 payload =====================
// Sends base64 data split into 50-char STATUSTEXT chunks with id + chunk_seq.
static int link_send_base64_chunks(const uint8_t* data, int data_len, uint8_t severity) {
    // Base64 encode
    int b64_max = ((data_len + 2) / 3) * 4 + 1;
    char* b64 = (char*)pvPortMalloc(b64_max);
    if (!b64) return -3;

    int b64_len = base64_encode(data, data_len, b64, b64_max);

    uint16_t msg_id = g_link_vision_msg_id++;
    int chunk_seq = 0;
    int offset = 0;

    while (offset < b64_len) {
        int remain = b64_len - offset;
        int chunk_len = (remain > 50) ? 50 : remain;

        // Build a 50-char text buffer (null-padded)
        char text[50];
        memset(text, 0, sizeof(text));
        memcpy(text, b64 + offset, chunk_len);

        mavlink_message_t msg;
        mavlink_msg_statustext_pack(
            g_link_sysid, g_link_compid, &msg,
            severity,
            text,
            msg_id,
            (uint8_t)chunk_seq
        );

        int rc = link_send_msg(&msg);
        if (rc != 0) {
            vPortFree(b64);
            return rc;
        }

        offset += chunk_len;
        chunk_seq++;

        // Small delay between chunks to avoid overwhelming the receiver
        if (offset < b64_len) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    vPortFree(b64);
    return 0;
}

// Per-transport pose/config tracking state (shared logic in sentai_vision_common.h)
static VisionTxState g_link_tx_state = VISION_TX_STATE_INIT;

// Encode VisionMessage and send as base64 STATUSTEXT chunks.
static int link_encode_and_send(const visionmesh_VisionMessage* vision, uint8_t severity) {
    uint8_t pb_buf[visionmesh_VisionMessage_size];
    pb_ostream_t stream = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
    if (!pb_encode(&stream, visionmesh_VisionMessage_fields, vision)) {
        printf("[link] pb_encode VisionMessage failed: %s\r\n", PB_GET_ERROR(&stream));
        return -2;
    }
    return link_send_base64_chunks(pb_buf, (int)stream.bytes_written, severity);
}

// ===================== TX: VisionMessage (NewDetection) as base64 chunks =====================
extern "C" int sentai_link_send_vision(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    int32_t gx_cm, int32_t gy_cm, int16_t width_cm,
    uint8_t severity)
{
    if (!g_link_running) return -1;

    visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
    vision_fill_header(&vision, sensor_id, track_id, alarm_type, timestamp_utc, seq);
    vision_attach_metadata(&vision, &g_link_tx_state);
    vision_fill_new_detection(&vision, x, y, w, h, conf, class_id,
                              gx_cm, gy_cm, width_cm);

    return link_encode_and_send(&vision, severity);
}

// ===================== TX: VisionMessage (UpdateDetection) as base64 chunks =====================
extern "C" int sentai_link_send_vision_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    int32_t gx_cm, int32_t gy_cm,
    uint8_t severity)
{
    if (!g_link_running) return -1;

    visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
    vision_fill_header(&vision, sensor_id, track_id, alarm_type, timestamp_utc, seq);
    vision_attach_metadata(&vision, &g_link_tx_state);
    vision_fill_update(&vision, x, y, w, h, conf, age, gx_cm, gy_cm);

    return link_encode_and_send(&vision, severity);
}

// ===================== TX: VisionMessage (DeleteDetection) as base64 chunks =====================
extern "C" int sentai_link_send_vision_delete(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint32_t reason, uint32_t age, uint32_t total_hits,
    int32_t last_gx_cm, int32_t last_gy_cm,
    uint8_t severity)
{
    if (!g_link_running) return -1;

    visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
    vision_fill_header(&vision, sensor_id, track_id, alarm_type, timestamp_utc, seq);
    vision_attach_metadata(&vision, &g_link_tx_state);
    vision_fill_delete(&vision, reason, age, total_hits, last_gx_cm, last_gy_cm);

    return link_encode_and_send(&vision, severity);
}

// ===================== TX: COMMAND_LONG =====================
extern "C" int sentai_link_send_command_long(
    uint8_t target_sys, uint8_t target_comp,
    uint16_t command, uint8_t confirmation,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7)
{
    if (!g_link_running) return -1;

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        g_link_sysid, g_link_compid, &msg,
        target_sys, target_comp,
        command, confirmation,
        param1, param2, param3, param4, param5, param6, param7
    );
    return link_send_msg(&msg);
}

// ===================== Debug level =====================
extern "C" void sentai_link_set_debug(int level) {
    g_link_debug = level;
    printf("[link] debug=%d\r\n", level);
}

// ===================== Accessors for opaque link_rx_msg_t =====================
// Used by modsentai.c (C code) which cannot include mavlink.h directly.

extern "C" uint32_t sentai_link_rx_msgid(const link_rx_msg_t* m) {
    return m->msg.msgid;
}

extern "C" uint8_t sentai_link_rx_sysid(const link_rx_msg_t* m) {
    return m->msg.sysid;
}

extern "C" uint8_t sentai_link_rx_compid(const link_rx_msg_t* m) {
    return m->msg.compid;
}

extern "C" uint8_t sentai_link_rx_seq(const link_rx_msg_t* m) {
    return m->msg.seq;
}

extern "C" uint8_t sentai_link_rx_len(const link_rx_msg_t* m) {
    return m->msg.len;
}

// ===================== Decode: LOCAL_POSITION_NED (msgid 32) =====================
// Returns native float values: position in metres, velocity in m/s.
extern "C" void sentai_link_rx_local_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    float* x, float* y, float* z,
    float* vx, float* vy, float* vz)
{
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(&m->msg, &pos);
    *time_boot_ms = pos.time_boot_ms;
    *x  = pos.x;
    *y  = pos.y;
    *z  = pos.z;
    *vx = pos.vx;
    *vy = pos.vy;
    *vz = pos.vz;
}

// ===================== Decode: GLOBAL_POSITION_INT (msgid 33) =====================
// Already integer in MAVLink: lat/lon in degE7, alt in mm, vx/vy/vz in cm/s, hdg in cdeg.
#include <standard/mavlink_msg_global_position_int.h>

extern "C" void sentai_link_rx_global_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    int32_t* lat, int32_t* lon, int32_t* alt, int32_t* relative_alt,
    int16_t* vx, int16_t* vy, int16_t* vz, uint16_t* hdg)
{
    mavlink_global_position_int_t gpos;
    mavlink_msg_global_position_int_decode(&m->msg, &gpos);
    *time_boot_ms  = gpos.time_boot_ms;
    *lat           = gpos.lat;
    *lon           = gpos.lon;
    *alt           = gpos.alt;
    *relative_alt  = gpos.relative_alt;
    *vx            = gpos.vx;
    *vy            = gpos.vy;
    *vz            = gpos.vz;
    *hdg           = gpos.hdg;
}
