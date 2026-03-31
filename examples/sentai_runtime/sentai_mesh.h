// Meshtastic mesh radio bridge for SentAI
// Communicates with a Meshtastic device (e.g. WisMesh Tap) over UART
// using the Meshtastic serial protobuf protocol.
//
// Protocol: 4-byte header (0x94, 0xC3, LEN_MSB, LEN_LSB) + protobuf payload
// Max payload: 512 bytes. Uses ToRadio/FromRadio protobuf messages.
//
// Architecture:
//   - Dedicated FreeRTOS RX task reads UART, parses frames, decodes protobuf
//   - Received messages (text + VisionMessage) are queued in FreeRTOS queues
//   - Send is synchronous: encode protobuf + frame + UART write (mutex-protected)
//   - Uses existing sentai_uart_serial_* bridge (LPUART6 via ConsoleM7)
//
// Protobuf: uses nanopb-generated code from official Meshtastic .proto files

#ifndef SENTAI_MESH_H_
#define SENTAI_MESH_H_

#include <stdint.h>
#include "generated/visionmesh.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum text message length (Meshtastic DATA_PAYLOAD_LEN)
#define MESH_MAX_TEXT_LEN     233

// Maximum number of received messages buffered
#define MESH_RX_QUEUE_DEPTH   8

// Received text message (stored in RX queue)
typedef struct {
    uint32_t from;                    // sender node number
    uint32_t to;                      // destination node number
    uint32_t id;                      // packet id
    int32_t  rx_rssi;                 // RSSI of received packet
    float    rx_snr;                  // SNR of received packet
    uint8_t  channel;                 // channel index
    uint8_t  hop_limit;              // remaining hops
    uint16_t text_len;               // length of text payload
    char     text[MESH_MAX_TEXT_LEN + 1]; // null-terminated text
} mesh_rx_msg_t;

// Received VisionMessage (stored in RX queue)
typedef struct {
    uint32_t from;                    // sender node number
    uint32_t to;                      // destination node number
    uint32_t id;                      // packet id
    int32_t  rx_rssi;
    float    rx_snr;
    uint8_t  channel;
    visionmesh_VisionMessage vision;  // decoded VisionMessage
} mesh_rx_vision_t;

// Initialize mesh bridge: opens UART at given baudrate, starts RX task.
// baudrate: typically 115200 for WisMesh.
// Returns 0 on success, negative on error.
int sentai_mesh_init(uint32_t baudrate);

// Stop mesh bridge: stops RX task, closes UART.
// Returns 0 on success.
int sentai_mesh_stop(void);

// Send a text message to the mesh.
// text: UTF-8 string to send
// dest: destination node number (0xFFFFFFFF = broadcast)
// channel: channel index (0 = primary)
// want_ack: 1 = request acknowledgment
// Returns 0 on success, negative on error.
int sentai_mesh_send_text(const char* text, uint32_t dest, uint8_t channel, int want_ack);

// Send a VisionMessage (NewDetection) to the mesh.
// Uses PRIVATE_APP portnum (256).
// Returns 0 on success, negative on error.
int sentai_mesh_send_detection(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    const uint8_t* embedding, uint32_t embed_len, uint32_t embed_crc8,
    uint32_t dest, uint8_t channel, int want_ack);

// Send a VisionMessage (UpdateDetection) to the mesh.
// Uses PRIVATE_APP portnum (256).
// Returns 0 on success, negative on error.
int sentai_mesh_send_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    uint32_t dest, uint8_t channel, int want_ack);

// Check how many received text messages are waiting.
int sentai_mesh_text_available(void);

// Check how many received VisionMessages are waiting.
int sentai_mesh_vision_available(void);

// Receive the next text message (non-blocking).
// Returns 1 if a message was available, 0 if queue empty.
int sentai_mesh_receive_text(mesh_rx_msg_t* msg);

// Receive the next text message with timeout.
// timeout_ms: 0=non-blocking, -1=forever, >0=wait up to N ms.
// Returns 1 if a message was received, 0 on timeout.
int sentai_mesh_receive_text_wait(mesh_rx_msg_t* msg, int timeout_ms);

// Receive the next VisionMessage (non-blocking).
// Returns 1 if a message was available, 0 if queue empty.
int sentai_mesh_receive_vision(mesh_rx_vision_t* msg);

// Receive the next VisionMessage with timeout.
int sentai_mesh_receive_vision_wait(mesh_rx_vision_t* msg, int timeout_ms);

// Send want_config_id to request NodeDB/config from the radio.
int sentai_mesh_request_config(uint32_t config_id);

// Check if mesh bridge is running.
int sentai_mesh_is_running(void);

// Get the node number of the connected radio (populated after config request).
uint32_t sentai_mesh_my_node_num(void);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_MESH_H_
