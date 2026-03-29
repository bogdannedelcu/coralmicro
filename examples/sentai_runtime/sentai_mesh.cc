// Meshtastic mesh radio bridge — nanopb implementation
//
// Uses nanopb-generated C code from official Meshtastic .proto files
// and the VisionMessage custom proto for SentAI detection payloads.

#include "sentai_mesh.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/queue.h"
#include "third_party/freertos_kernel/include/semphr.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "generated/meshtastic/mesh.pb.h"
#include "generated/meshtastic/portnums.pb.h"
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
}

// ===================== Meshtastic framing constants =====================
static const uint8_t MESH_START1 = 0x94;
static const uint8_t MESH_START2 = 0xC3;
static const int     MESH_MAX_PAYLOAD = 512;

// ===================== Module state =====================
static volatile int  g_mesh_running = 0;
static TaskHandle_t  g_mesh_rx_task = nullptr;
static QueueHandle_t g_mesh_text_queue = nullptr;
static QueueHandle_t g_mesh_vision_queue = nullptr;
static SemaphoreHandle_t g_mesh_tx_mutex = nullptr;
static uint32_t      g_mesh_my_node_num = 0;
static uint32_t      g_mesh_packet_id = 1;

#define APP_VERSION 1

// ===================== Frame send (with mutex) =====================

// Send a framed Meshtastic serial packet: [0x94, 0xC3, len_msb, len_lsb, payload...]
static int mesh_send_frame(const uint8_t* payload, int len) {
    if (len <= 0 || len > MESH_MAX_PAYLOAD) return -1;

    uint8_t header[4];
    header[0] = MESH_START1;
    header[1] = MESH_START2;
    header[2] = (uint8_t)((len >> 8) & 0xFF);
    header[3] = (uint8_t)(len & 0xFF);

    xSemaphoreTake(g_mesh_tx_mutex, portMAX_DELAY);
    int n1 = sentai_uart_serial_write(header, 4);
    int n2 = sentai_uart_serial_write(payload, len);
    xSemaphoreGive(g_mesh_tx_mutex);

    return (n1 == 4 && n2 == len) ? 0 : -2;
}

// ===================== ToRadio encode + send helpers =====================

// Encode a ToRadio containing a MeshPacket and send it framed.
// The MeshPacket.decoded (Data) must already be filled in.
static int mesh_send_toradio_packet(meshtastic_Data* data,
                                    uint32_t dest, uint8_t channel, bool want_ack) {
    meshtastic_ToRadio toRadio = meshtastic_ToRadio_init_default;
    toRadio.which_payload_variant = meshtastic_ToRadio_packet_tag;

    meshtastic_MeshPacket* pkt = &toRadio.packet;
    pkt->to = dest;
    pkt->channel = channel;
    pkt->want_ack = want_ack;
    pkt->id = g_mesh_packet_id++;
    pkt->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    memcpy(&pkt->decoded, data, sizeof(meshtastic_Data));

    // Encode with nanopb
    uint8_t buf[meshtastic_MeshPacket_size + 16];  // ToRadio envelope overhead
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, meshtastic_ToRadio_fields, &toRadio)) {
        printf("[mesh] pb_encode ToRadio failed: %s\r\n", PB_GET_ERROR(&stream));
        return -3;
    }

    return mesh_send_frame(buf, (int)stream.bytes_written);
}

// ===================== RX task: read frames, decode FromRadio =====================

// Read exactly `count` bytes from UART with timeout per byte.
// Returns number of bytes actually read.
static int uart_read_exact(uint8_t* buf, int count, int timeout_ms) {
    int total = 0;
    while (total < count) {
        int n = sentai_uart_serial_read(buf + total, count - total, timeout_ms);
        if (n <= 0) break;
        total += n;
    }
    return total;
}

// Process a decoded MeshPacket from a FromRadio message.
static void mesh_handle_packet(const meshtastic_MeshPacket* pkt) {
    // Only handle decoded (not encrypted) packets
    if (pkt->which_payload_variant != meshtastic_MeshPacket_decoded_tag) return;

    const meshtastic_Data* data = &pkt->decoded;

    if (data->portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
        // Text message → queue to text queue
        if (data->payload.size > 0 && data->payload.size <= MESH_MAX_TEXT_LEN &&
            g_mesh_text_queue != nullptr) {
            mesh_rx_msg_t rx;
            rx.from = pkt->from;
            rx.to = pkt->to;
            rx.id = pkt->id;
            rx.rx_rssi = pkt->rx_rssi;
            rx.rx_snr = pkt->rx_snr;
            rx.channel = pkt->channel;
            rx.hop_limit = pkt->hop_limit;
            rx.text_len = (uint16_t)data->payload.size;
            memcpy(rx.text, data->payload.bytes, data->payload.size);
            rx.text[data->payload.size] = '\0';
            xQueueSend(g_mesh_text_queue, &rx, 0);
        }
    } else if (data->portnum == meshtastic_PortNum_PRIVATE_APP) {
        // Try to decode as VisionMessage
        visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
        pb_istream_t istream = pb_istream_from_buffer(
            data->payload.bytes, data->payload.size);
        if (pb_decode(&istream, visionmesh_VisionMessage_fields, &vision)) {
            if (g_mesh_vision_queue != nullptr) {
                mesh_rx_vision_t rx;
                rx.from = pkt->from;
                rx.to = pkt->to;
                rx.id = pkt->id;
                rx.rx_rssi = pkt->rx_rssi;
                rx.rx_snr = pkt->rx_snr;
                rx.channel = pkt->channel;
                rx.vision = vision;
                xQueueSend(g_mesh_vision_queue, &rx, 0);
            }
        }
    }
}

static void mesh_rx_task(void* param) {
    (void)param;
    uint8_t frame_buf[MESH_MAX_PAYLOAD];

    printf("[mesh] RX task started\r\n");

    while (g_mesh_running) {
        // Sync: find frame header 0x94 0xC3
        uint8_t b;
        int n = sentai_uart_serial_read(&b, 1, 100);
        if (n <= 0) continue;
        if (b != MESH_START1) continue;

        n = sentai_uart_serial_read(&b, 1, 50);
        if (n <= 0) continue;
        if (b != MESH_START2) continue;

        // Read 2-byte length
        uint8_t len_buf[2];
        if (uart_read_exact(len_buf, 2, 100) != 2) continue;
        int frame_len = ((int)len_buf[0] << 8) | len_buf[1];

        if (frame_len <= 0 || frame_len > MESH_MAX_PAYLOAD) {
            printf("[mesh] bad frame len %d\r\n", frame_len);
            continue;
        }

        // Read payload
        if (uart_read_exact(frame_buf, frame_len, 200) != frame_len) {
            printf("[mesh] short read, expected %d\r\n", frame_len);
            continue;
        }

        // Decode FromRadio
        meshtastic_FromRadio fromRadio = meshtastic_FromRadio_init_default;
        pb_istream_t stream = pb_istream_from_buffer(frame_buf, frame_len);
        if (!pb_decode(&stream, meshtastic_FromRadio_fields, &fromRadio)) {
            printf("[mesh] pb_decode FromRadio failed: %s\r\n", PB_GET_ERROR(&stream));
            continue;
        }

        switch (fromRadio.which_payload_variant) {
            case meshtastic_FromRadio_packet_tag:
                mesh_handle_packet(&fromRadio.packet);
                break;

            case meshtastic_FromRadio_my_info_tag:
                g_mesh_my_node_num = fromRadio.my_info.my_node_num;
                printf("[mesh] my_node_num = 0x%08lx\r\n",
                       (unsigned long)g_mesh_my_node_num);
                break;

            case meshtastic_FromRadio_config_complete_id_tag:
                printf("[mesh] config complete (id=%lu)\r\n",
                       (unsigned long)fromRadio.config_complete_id);
                break;

            case meshtastic_FromRadio_rebooted_tag:
                printf("[mesh] radio rebooted\r\n");
                break;

            default:
                // Ignore other FromRadio variants (config, channel, nodeInfo, etc.)
                break;
        }
    }

    printf("[mesh] RX task stopped\r\n");
    vTaskDelete(nullptr);
}

// ===================== Public API =====================

extern "C" int sentai_mesh_init(uint32_t baudrate) {
    if (g_mesh_running) return 0;  // already running

    // Set baudrate if non-default
    if (baudrate != 115200 && baudrate > 0) {
        sentai_uart_set_baudrate(baudrate);
    }

    // Open UART
    if (!sentai_uart_serial_open()) {
        printf("[mesh] UART open failed\r\n");
        return -1;
    }

    // Create queues and mutex
    g_mesh_text_queue = xQueueCreate(MESH_RX_QUEUE_DEPTH, sizeof(mesh_rx_msg_t));
    g_mesh_vision_queue = xQueueCreate(MESH_RX_QUEUE_DEPTH, sizeof(mesh_rx_vision_t));
    g_mesh_tx_mutex = xSemaphoreCreateMutex();

    if (!g_mesh_text_queue || !g_mesh_vision_queue || !g_mesh_tx_mutex) {
        printf("[mesh] queue/mutex create failed\r\n");
        sentai_uart_serial_close();
        return -2;
    }

    g_mesh_running = 1;
    g_mesh_packet_id = (uint32_t)(xTaskGetTickCount() & 0xFFFF) | 0x10000;

    // Create RX task (priority just above idle, 4 KB stack)
    BaseType_t rc = xTaskCreate(mesh_rx_task, "mesh_rx", 4096 / sizeof(StackType_t),
                                nullptr, tskIDLE_PRIORITY + 2, &g_mesh_rx_task);
    if (rc != pdPASS) {
        printf("[mesh] task create failed\r\n");
        g_mesh_running = 0;
        sentai_uart_serial_close();
        return -3;
    }

    printf("[mesh] initialized at %lu baud\r\n", (unsigned long)baudrate);
    return 0;
}

extern "C" int sentai_mesh_stop(void) {
    if (!g_mesh_running) return 0;

    g_mesh_running = 0;
    // Wait for RX task to exit
    vTaskDelay(pdMS_TO_TICKS(300));

    if (g_mesh_text_queue) { vQueueDelete(g_mesh_text_queue); g_mesh_text_queue = nullptr; }
    if (g_mesh_vision_queue) { vQueueDelete(g_mesh_vision_queue); g_mesh_vision_queue = nullptr; }
    if (g_mesh_tx_mutex) { vSemaphoreDelete(g_mesh_tx_mutex); g_mesh_tx_mutex = nullptr; }

    sentai_uart_restore_baudrate();
    sentai_uart_serial_close();

    printf("[mesh] stopped\r\n");
    return 0;
}

extern "C" int sentai_mesh_send_text(const char* text, uint32_t dest,
                                     uint8_t channel, int want_ack) {
    if (!g_mesh_running) return -1;

    size_t text_len = strlen(text);
    if (text_len > MESH_MAX_TEXT_LEN) text_len = MESH_MAX_TEXT_LEN;

    meshtastic_Data data = meshtastic_Data_init_default;
    data.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    data.payload.size = (pb_size_t)text_len;
    memcpy(data.payload.bytes, text, text_len);

    return mesh_send_toradio_packet(&data, dest, channel, want_ack != 0);
}

extern "C" int sentai_mesh_send_detection(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    const uint8_t* embedding, uint32_t embed_len, uint32_t embed_crc8,
    uint32_t dest, uint8_t channel, int want_ack)
{
    if (!g_mesh_running) return -1;

    // Build VisionMessage with NewDetection
    visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
    vision.app_version = APP_VERSION;
    vision.sensor_id = sensor_id;
    vision.track_id = track_id;
    vision.alarm_type = alarm_type;
    vision.timestamp_utc = timestamp_utc;
    vision.seq = seq;
    vision.which_body = visionmesh_VisionMessage_new_detection_tag;

    visionmesh_NewDetection* det = &vision.body.new_detection;
    det->xywh_packed = ((uint32_t)x) | ((uint32_t)y << 8) |
                       ((uint32_t)w << 16) | ((uint32_t)h << 24);
    det->conf = conf;
    det->class_id = class_id;
    det->embed_crc8 = embed_crc8;
    if (embedding && embed_len > 0) {
        if (embed_len > 64) embed_len = 64;
        det->embedding.size = embed_len;
        memcpy(det->embedding.bytes, embedding, embed_len);
    } else {
        det->embedding.size = 0;
    }

    // Encode VisionMessage into Data.payload
    uint8_t vision_buf[visionmesh_VisionMessage_size];
    pb_ostream_t vstream = pb_ostream_from_buffer(vision_buf, sizeof(vision_buf));
    if (!pb_encode(&vstream, visionmesh_VisionMessage_fields, &vision)) {
        printf("[mesh] encode VisionMessage failed: %s\r\n", PB_GET_ERROR(&vstream));
        return -2;
    }

    meshtastic_Data data = meshtastic_Data_init_default;
    data.portnum = meshtastic_PortNum_PRIVATE_APP;
    data.payload.size = (pb_size_t)vstream.bytes_written;
    memcpy(data.payload.bytes, vision_buf, vstream.bytes_written);

    return mesh_send_toradio_packet(&data, dest, channel, want_ack != 0);
}

extern "C" int sentai_mesh_send_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    uint32_t dest, uint8_t channel, int want_ack)
{
    if (!g_mesh_running) return -1;

    visionmesh_VisionMessage vision = visionmesh_VisionMessage_init_zero;
    vision.app_version = APP_VERSION;
    vision.sensor_id = sensor_id;
    vision.track_id = track_id;
    vision.alarm_type = alarm_type;
    vision.timestamp_utc = timestamp_utc;
    vision.seq = seq;
    vision.which_body = visionmesh_VisionMessage_update_detection_tag;

    visionmesh_UpdateDetection* upd = &vision.body.update_detection;
    upd->xywh_packed = ((uint32_t)x) | ((uint32_t)y << 8) |
                       ((uint32_t)w << 16) | ((uint32_t)h << 24);
    upd->conf = conf;
    upd->age = age;

    uint8_t vision_buf[visionmesh_VisionMessage_size];
    pb_ostream_t vstream = pb_ostream_from_buffer(vision_buf, sizeof(vision_buf));
    if (!pb_encode(&vstream, visionmesh_VisionMessage_fields, &vision)) {
        printf("[mesh] encode VisionMessage failed: %s\r\n", PB_GET_ERROR(&vstream));
        return -2;
    }

    meshtastic_Data data = meshtastic_Data_init_default;
    data.portnum = meshtastic_PortNum_PRIVATE_APP;
    data.payload.size = (pb_size_t)vstream.bytes_written;
    memcpy(data.payload.bytes, vision_buf, vstream.bytes_written);

    return mesh_send_toradio_packet(&data, dest, channel, want_ack != 0);
}

extern "C" int sentai_mesh_text_available(void) {
    if (!g_mesh_text_queue) return 0;
    return (int)uxQueueMessagesWaiting(g_mesh_text_queue);
}

extern "C" int sentai_mesh_vision_available(void) {
    if (!g_mesh_vision_queue) return 0;
    return (int)uxQueueMessagesWaiting(g_mesh_vision_queue);
}

extern "C" int sentai_mesh_receive_text(mesh_rx_msg_t* msg) {
    if (!g_mesh_text_queue || !msg) return 0;
    return (xQueueReceive(g_mesh_text_queue, msg, 0) == pdTRUE) ? 1 : 0;
}

extern "C" int sentai_mesh_receive_text_wait(mesh_rx_msg_t* msg, int timeout_ms) {
    if (!g_mesh_text_queue || !msg) return 0;
    TickType_t ticks;
    if (timeout_ms < 0)      ticks = portMAX_DELAY;
    else if (timeout_ms == 0) ticks = 0;
    else                      ticks = pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive(g_mesh_text_queue, msg, ticks) == pdTRUE) ? 1 : 0;
}

extern "C" int sentai_mesh_receive_vision(mesh_rx_vision_t* msg) {
    if (!g_mesh_vision_queue || !msg) return 0;
    return (xQueueReceive(g_mesh_vision_queue, msg, 0) == pdTRUE) ? 1 : 0;
}

extern "C" int sentai_mesh_receive_vision_wait(mesh_rx_vision_t* msg, int timeout_ms) {
    if (!g_mesh_vision_queue || !msg) return 0;
    TickType_t ticks;
    if (timeout_ms < 0)      ticks = portMAX_DELAY;
    else if (timeout_ms == 0) ticks = 0;
    else                      ticks = pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive(g_mesh_vision_queue, msg, ticks) == pdTRUE) ? 1 : 0;
}

extern "C" int sentai_mesh_request_config(uint32_t config_id) {
    if (!g_mesh_running) return -1;

    meshtastic_ToRadio toRadio = meshtastic_ToRadio_init_default;
    toRadio.which_payload_variant = meshtastic_ToRadio_want_config_id_tag;
    toRadio.want_config_id = config_id;

    uint8_t buf[32];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, meshtastic_ToRadio_fields, &toRadio)) {
        return -2;
    }
    return mesh_send_frame(buf, (int)stream.bytes_written);
}

extern "C" int sentai_mesh_is_running(void) {
    return g_mesh_running;
}

extern "C" uint32_t sentai_mesh_my_node_num(void) {
    return g_mesh_my_node_num;
}
