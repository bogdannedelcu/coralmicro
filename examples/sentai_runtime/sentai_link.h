// MAVLink v2 bridge for SentAI
// Communicates with a MAVLink-capable autopilot / GCS over UART.
//
// Uses the official MAVLink C library (c_library_v2) from third_party/mavlink/.
// All message types and CRC_EXTRA values come from the library headers.
//
// TX: VisionMessage encoded as base64 in chunked STATUSTEXT messages.
// TX: HEARTBEAT, COMMAND_LONG, raw STATUSTEXT.
// RX: All common MAVLink messages decoded via the official library.
//
// Architecture:
//   - Dedicated FreeRTOS RX task reads UART, feeds mavlink_parse_char()
//   - Decoded messages are queued in a FreeRTOS queue as link_rx_msg_t
//   - TX is synchronous: pack with mavlink_msg_*_pack(), serialize with
//     mavlink_msg_to_send_buffer(), send over UART (mutex-protected)
//   - Uses existing sentai_uart_serial_* bridge (LPUART6 via ConsoleM7)
//   - UART cannot be shared with sentai.mesh — only one at a time

#ifndef SENTAI_LINK_H_
#define SENTAI_LINK_H_

#include <stdint.h>

// Include official MAVLink common dialect (defines all msg types, enums, CRCs)
#include <common/mavlink.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== Queue config =====================
#define LINK_RX_QUEUE_DEPTH     16

// ===================== Received message wrapper =====================
// Wraps a mavlink_message_t with convenience fields for MicroPython.
// The full decoded payload is accessible via mavlink_msg_*_decode().
typedef struct {
    mavlink_message_t msg;    // official MAVLink message (contains msgid, sysid, compid, seq, payload)
} link_rx_msg_t;

// ===================== Public API =====================

// Initialize MAVLink bridge: opens UART, starts RX task.
// baudrate: typically 57600 or 115200 for MAVLink telemetry.
// sysid/compid: our MAVLink system/component ID (default 1/191 = onboard computer).
// Returns 0 on success, negative on error.
int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid);

// Stop MAVLink bridge: stops RX task, closes UART.
int sentai_link_stop(void);

// Check if MAVLink bridge is running.
int sentai_link_is_running(void);

// Set debug level: 0=off, 1=TX/RX summary, 2=+hex dump
void sentai_link_set_debug(int level);

// Check how many received MAVLink messages are queued.
int sentai_link_available(void);

// Receive the next MAVLink message (non-blocking). Returns 1 if got one.
int sentai_link_receive(link_rx_msg_t* msg);

// Receive with timeout (ms). 0=poll, -1=forever.
int sentai_link_receive_wait(link_rx_msg_t* msg, int timeout_ms);

// Send a HEARTBEAT message.
// type: MAV_TYPE (e.g. 6=GCS, 18=onboard_controller)
int sentai_link_send_heartbeat(uint8_t type);

// Send a STATUSTEXT message (single, max 50 chars).
// severity: MAV_SEVERITY (0-7)
int sentai_link_send_statustext(uint8_t severity, const char* text);

// Send a VisionMessage as base64-encoded STATUSTEXT chunks.
// The VisionMessage is nanopb-encoded, base64-encoded, then split into
// multiple STATUSTEXT messages using id + chunk_seq for reassembly.
// severity: MAV_SEVERITY for the STATUSTEXT wrapper.
int sentai_link_send_vision(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    const uint8_t* embedding, uint32_t embed_len, uint32_t embed_crc8,
    uint8_t severity);

// Send a VisionMessage (UpdateDetection) as base64 STATUSTEXT chunks.
int sentai_link_send_vision_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    uint8_t severity);

// Send raw COMMAND_LONG message.
// param1..7 are native float values (sent directly on wire).
int sentai_link_send_command_long(
    uint8_t target_sys, uint8_t target_comp,
    uint16_t command, uint8_t confirmation,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_LINK_H_
