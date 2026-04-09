// CrazyFlie autopilot bridge — CRTP over CPX over UART
//
// TX: Build CRTP command → wrap in CPX routing → frame with CRC → UART write.
// RX: FreeRTOS task reads UART, parses CPX frames, handles CTS flow control.
//
// CPX UART frame:  [0xFF][LEN(1B)][CPX_HDR(2B) + payload...][CRC]
// CTS (ready):     [0xFF][0x00]   — 2 bytes, no CRC
// LEN:             payload size (CPX_HDR + DATA), without CRC
// CRC:             XOR of ALL bytes including 0xFF and LEN
//
// CPX_HDR byte 0: [destination:4][source:4]
// CPX_HDR byte 1: [function:8]
//
// CRTP-in-CPX:     routing dst=STM32(0x01), src=HOST(0x03), fn=CRTP(0x02)
//                  then [CRTP_header(1B)][CRTP_payload...]

#include "sentai_crazy.h"

#include <cstdio>
#include <cstring>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"

// Existing UART bridge functions (from modsentai_hal.cc)
extern "C" {
extern int  sentai_uart_serial_open(void);
extern void sentai_uart_serial_close(void);
extern int  sentai_uart_serial_is_open(void);
extern int  sentai_uart_serial_write(const uint8_t* buf, int size);
extern int  sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern void sentai_uart_set_baudrate(uint32_t baudrate);
extern void sentai_uart_restore_baudrate(void);
}

// ===================== CPX Protocol Constants =====================
#define CPX_START_BYTE   0xFF
#define CPX_MTU          100    // max payload bytes per CPX frame (hdr + data)

// CPX target IDs (3 bits)
#define CPX_T_STM32      1      // CrazyFlie main MCU (runs CRTP)
#define CPX_T_ESP32      2      // WiFi co-processor
#define CPX_T_HOST       3      // external MCU (us! = WIFI_HOST in CF FW)

// CPX function IDs (6 bits — firmware enum starts at 1!)
#define CPX_F_SYSTEM     1
#define CPX_F_CONSOLE    2
#define CPX_F_CRTP       3

// CPX SYSTEM sub-commands
#define CPX_SYS_SET_CLIENT   0x20   // data: 0x01=connected, 0x00=disconnected
#define CPX_SYS_SET_BRIDGE   0x21   // data: 0x01=enable, 0x00=disable

// ===================== CRTP Protocol Constants =====================
#define CRTP_PORT_PARAM          0x02   // Parameter system
#define CRTP_PORT_COMMANDER      0x03   // Commander (roll/pitch/yaw/thrust)
#define CRTP_PORT_SETPOINT_HL    0x08   // High-Level Commander
#define CRTP_PORT_SETPOINT_GEN   0x07   // Generic Setpoint
#define CRTP_PORT_PLATFORM       0x0D   // Platform service (arm, version, etc.)
#define CRTP_MAX_PAYLOAD         30

// CRTP param channel IDs
#define PARAM_TOC_CH             0
#define PARAM_WRITE_CH           2

// CRTP param TOC commands (V2)
#define PARAM_TOC_GET_ITEM_V2    2
#define PARAM_TOC_GET_INFO_V2    3

// Platform service sub-commands (port 0x0D, channel 0)
#define PLATFORM_CMD_ARM_SYSTEM  1      // data[1]: 0=disarm, 1=arm

// High-Level Commander command IDs
#define HL_CMD_STOP        3
#define HL_CMD_TAKEOFF_2   7
#define HL_CMD_LAND_2      8
#define HL_CMD_GO_TO_2     12

// Generic Setpoint type IDs
#define SETPOINT_TYPE_HOVER  5

// ===================== Module State =====================
static volatile int      g_crazy_running = 0;
static volatile int      g_crazy_debug   = 0;   // 0=off, 1=summary, 2=hex
static TaskHandle_t      g_crazy_rx_task = nullptr;
static SemaphoreHandle_t g_crazy_tx_mutex = nullptr;  // serializes UART writes
static SemaphoreHandle_t g_crazy_cts = nullptr;        // CTS flow control
static SemaphoreHandle_t g_crazy_ping_sem = nullptr;   // echo response signal

// CRTP param response slot (single response at a time)
static uint8_t  g_param_resp_buf[CRTP_MAX_PAYLOAD];
static volatile int g_param_resp_len = 0;
static SemaphoreHandle_t g_param_resp_sem = nullptr;

// motorPowerSet param IDs (discovered from CF param TOC)
static int16_t g_param_motor_m1     = -1;
static int16_t g_param_motor_m2     = -1;
static int16_t g_param_motor_m3     = -1;
static int16_t g_param_motor_m4     = -1;
static int16_t g_param_motor_enable = -1;

// ===================== CPX Routing Header =====================
// Pack 2-byte CPX routing header (packed bitfields, ARM LE):
//   byte 0: [reserved:1][lastPacket:1][source:3][destination:3]
//   byte 1: [version:2][function:6]
static inline void cpx_pack_route(uint8_t* hdr,
                                  uint8_t dst, uint8_t src, uint8_t fn) {
    hdr[0] = (1 << 6) | ((src & 0x07) << 3) | (dst & 0x07);  // lastPacket=1
    hdr[1] = fn & 0x3F;  // version=0
}

// ===================== CPX CRC =====================
// XOR over ALL frame bytes including 0xFF start and LEN.
static inline uint8_t cpx_crc(const uint8_t* frame, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= frame[i];
    return crc;
}

// ===================== CPX UART TX: CTS =====================
// Send a CTS (Clear-To-Send): [0xFF][0x00] — 2 bytes, no CRC.
static void cpx_send_cts(void) {
    uint8_t cts[2] = {CPX_START_BYTE, 0x00};
    xSemaphoreTake(g_crazy_tx_mutex, portMAX_DELAY);
    sentai_uart_serial_write(cts, 2);
    xSemaphoreGive(g_crazy_tx_mutex);
}

// ===================== CPX UART TX: Data Frame =====================
// Blocks until CTS received, then sends the CPX frame.
// cpx_data: [routing(2B) + payload(N)]
// cpx_len:  total data length (2..MTU)
// Frame on wire: [0xFF][LEN(1B)][cpx_data...][CRC]
// CRC = XOR of all bytes including 0xFF and LEN.
static int cpx_send_frame(const uint8_t* cpx_data, int cpx_len) {
    if (cpx_len < 2 || cpx_len > CPX_MTU) return -1;

    // Wait for CTS from CrazyFlie (it's ready to receive)
    if (xSemaphoreTake(g_crazy_cts, pdMS_TO_TICKS(1000)) != pdTRUE) {
        if (g_crazy_debug >= 1) printf("[crazy] TX: CTS timeout\r\n");
        return -3;
    }

    // Build UART frame: [0xFF][LEN][data...][CRC]
    uint8_t frame[CPX_MTU + 3];     // start(1) + len(1) + data(MTU) + crc(1)
    int idx = 0;
    frame[idx++] = CPX_START_BYTE;
    frame[idx++] = (uint8_t)cpx_len;
    memcpy(&frame[idx], cpx_data, cpx_len);
    idx += cpx_len;
    // CRC = XOR of all preceding bytes (start + len + data)
    frame[idx] = cpx_crc(frame, idx);
    idx++;

    if (g_crazy_debug >= 2) {
        printf("[crazy] TX %d bytes:", idx);
        for (int i = 0; i < idx && i < 24; i++) printf(" %02X", frame[i]);
        if (idx > 24) printf(" ...");
        printf("\r\n");
    }

    xSemaphoreTake(g_crazy_tx_mutex, portMAX_DELAY);
    int n = sentai_uart_serial_write(frame, idx);
    xSemaphoreGive(g_crazy_tx_mutex);

    if (n != idx) {
        if (g_crazy_debug >= 1)
            printf("[crazy] TX FAIL: wrote %d/%d\r\n", n, idx);
        return -2;
    }

    return 0;
}

// ===================== CRTP-over-CPX Send =====================
// Wraps a CRTP packet in CPX routing and sends it.
static int crtp_send(uint8_t port, uint8_t channel,
                     const uint8_t* data, int len) {
    if (len > CRTP_MAX_PAYLOAD) return -1;

    uint8_t buf[CPX_MTU];
    int idx = 0;

    // CPX routing: dst=STM32, src=HOST, fn=CRTP
    cpx_pack_route(&buf[idx], CPX_T_STM32, CPX_T_HOST, CPX_F_CRTP);
    idx += 2;

    // CRTP header: [port:4][link:2][channel:2]  (channel in bits 0-1)
    buf[idx++] = ((port & 0x0F) << 4) | (channel & 0x03);

    // CRTP payload
    if (len > 0 && data) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }

    if (g_crazy_debug >= 1) {
        printf("[crazy] CRTP TX port=%u ch=%u len=%d\r\n", port, channel, len);
    }

    return cpx_send_frame(buf, idx);
}

// ===================== CPX RX: State Machine =====================
enum CpxRxState {
    CPX_RX_WAIT_START = 0,
    CPX_RX_LEN,
    CPX_RX_DATA,
    CPX_RX_CRC,
};

// Process a complete, CRC-valid received CPX frame.
static void cpx_process_rx(const uint8_t* data, uint16_t len) {
    if (len < 2) return;

    uint8_t dst = data[0] & 0x07;
    uint8_t src = (data[0] >> 3) & 0x07;
    uint8_t fn  = data[1] & 0x3F;

    if (g_crazy_debug >= 2) {
        printf("[crazy] RX CPX dst=%u src=%u fn=%u len=%u\r\n",
               dst, src, fn, len);
    }

    // Print console output from CrazyFlie (always if debug >= 1)
    if (fn == CPX_F_CONSOLE && len > 2 && g_crazy_debug >= 1) {
        int txt_len = len - 2;
        printf("[crazy] CF: %.*s", txt_len, (const char*)&data[2]);
    }

    // CRTP response
    if (fn == CPX_F_CRTP && len > 2) {
        uint8_t hdr = data[2];
        uint8_t rport = (hdr >> 4) & 0x0F;
        uint8_t rch   = hdr & 0x03;

        // Echo response on LINK port (0x0F), channel 0 → unblock ping()
        if (rport == 0x0F && rch == 0 && g_crazy_ping_sem) {
            xSemaphoreGive(g_crazy_ping_sem);
        }

        // Param response → copy to response slot
        if (rport == CRTP_PORT_PARAM && g_param_resp_sem) {
            int plen = (int)len - 3;
            if (g_crazy_debug >= 1) {
                printf("[crazy] RX PARAM ch=%u plen=%d", rch, plen);
                for (int k = 0; k < plen && k < 12; k++)
                    printf(" %02X", data[3 + k]);
                printf("\r\n");
            }
            if (plen > 0 && plen <= CRTP_MAX_PAYLOAD) {
                memcpy(g_param_resp_buf, &data[3], plen);
                g_param_resp_len = plen;
            }
            xSemaphoreGive(g_param_resp_sem);
        }

        if (g_crazy_debug >= 2) {
            printf("[crazy] RX CRTP port=%u ch=%u payload=%u bytes\r\n",
                   rport, rch, len - 3);
        }
    }
}

// ===================== CPX RX Task =====================
static void crazy_rx_task(void* param) {
    (void)param;
    enum CpxRxState state = CPX_RX_WAIT_START;
    uint8_t frame_buf[CPX_MTU];
    uint8_t frame_len = 0;     // LEN byte (1 byte)
    uint16_t frame_idx = 0;

    printf("[crazy] RX task started\r\n");

    while (g_crazy_running) {
        uint8_t buf[32];
        int n = sentai_uart_serial_read(buf, sizeof(buf), 50);
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];

            switch (state) {
            case CPX_RX_WAIT_START:
                if (b == CPX_START_BYTE) state = CPX_RX_LEN;
                break;

            case CPX_RX_LEN:
                frame_len = b;
                if (frame_len == 0) {
                    // CTS from CrazyFlie — it's ready to receive
                    xSemaphoreGive(g_crazy_cts);
                    if (g_crazy_debug >= 2) printf("[crazy] RX CTS\r\n");
                    state = CPX_RX_WAIT_START;
                } else if (frame_len > CPX_MTU) {
                    // Invalid length — resync
                    if (g_crazy_debug >= 1)
                        printf("[crazy] RX bad len %u, resync\r\n", frame_len);
                    state = CPX_RX_WAIT_START;
                } else {
                    frame_idx = 0;
                    state = CPX_RX_DATA;
                }
                break;

            case CPX_RX_DATA:
                if (frame_idx < CPX_MTU) {
                    frame_buf[frame_idx] = b;
                }
                frame_idx++;
                if (frame_idx >= frame_len) state = CPX_RX_CRC;
                break;

            case CPX_RX_CRC: {
                // CRC = XOR of all bytes: 0xFF, LEN, data[0..len-1]
                uint8_t expected = CPX_START_BYTE ^ frame_len;
                for (uint16_t j = 0; j < frame_len; j++) expected ^= frame_buf[j];
                if (b != expected) {
                    if (g_crazy_debug >= 1)
                        printf("[crazy] RX CRC err: got %02X exp %02X\r\n",
                               b, expected);
                } else {
                    cpx_process_rx(frame_buf, frame_len);
                }
                // Tell CrazyFlie we're ready for the next frame
                cpx_send_cts();
                state = CPX_RX_WAIT_START;
                break;
            }

            default:
                state = CPX_RX_WAIT_START;
                break;
            }
        }
    }

    printf("[crazy] RX task stopped\r\n");
    vTaskDelete(nullptr);
}

// ===================== CPX System Command =====================
// Send a CPX SYSTEM command to STM32 (e.g. enable bridge, set client).
// Returns 0 on success, negative on error.
static int cpx_send_system(uint8_t subcmd, uint8_t value) {
    uint8_t buf[4];
    cpx_pack_route(&buf[0], CPX_T_STM32, CPX_T_HOST, CPX_F_SYSTEM);
    buf[2] = subcmd;
    buf[3] = value;
    return cpx_send_frame(buf, 4);
}

// ===================== Public API: Init / Stop =====================

extern "C" int sentai_crazy_init(uint32_t baudrate) {
    if (g_crazy_running) return 0;  // already running

    // Set baudrate (576000 is typical for CF deck UART)
    if (baudrate != 115200 && baudrate > 0) {
        sentai_uart_set_baudrate(baudrate);
    }

    // Open UART
    if (!sentai_uart_serial_open()) {
        printf("[crazy] UART open failed\r\n");
        return -1;
    }

    // Create TX mutex and CTS semaphore
    g_crazy_tx_mutex = xSemaphoreCreateMutex();
    g_crazy_cts      = xSemaphoreCreateBinary();
    g_crazy_ping_sem = xSemaphoreCreateBinary();
    g_param_resp_sem = xSemaphoreCreateBinary();

    if (!g_crazy_tx_mutex || !g_crazy_cts || !g_crazy_ping_sem || !g_param_resp_sem) {
        printf("[crazy] semaphore create failed\r\n");
        sentai_uart_serial_close();
        return -2;
    }

    g_crazy_running = 1;

    // Start RX task (parses CPX frames, handles CTS flow control)
    BaseType_t rc = xTaskCreate(crazy_rx_task, "crazy_rx",
                                4096 / sizeof(StackType_t),
                                nullptr, tskIDLE_PRIORITY + 2,
                                &g_crazy_rx_task);
    if (rc != pdPASS) {
        printf("[crazy] task create failed\r\n");
        g_crazy_running = 0;
        sentai_uart_serial_close();
        return -3;
    }

    // --- Sync / reconnect sequence ---
    // Handles both fresh boot (CF sends CTS) and reconnect (CF TX blocked).
    // 1. Flush stale RX data
    {
        uint8_t junk[64];
        while (sentai_uart_serial_read(junk, sizeof(junk), 10) > 0) {}
    }

    // 2. Send CTS to unblock firmware TX (in case it's waiting)
    {
        uint8_t cts[2] = {CPX_START_BYTE, 0x00};
        sentai_uart_serial_write(cts, 2);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Flush any stale response
    {
        uint8_t junk[64];
        while (sentai_uart_serial_read(junk, sizeof(junk), 10) > 0) {}
    }

    // 4. Send CTS again — clean state
    {
        uint8_t cts[2] = {CPX_START_BYTE, 0x00};
        sentai_uart_serial_write(cts, 2);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 5. Pre-give CTS semaphore so bridge enable TX can proceed
    xSemaphoreGive(g_crazy_cts);

    // 6. Enable CRTP bridge on CrazyFlie
    int r1 = cpx_send_system(CPX_SYS_SET_BRIDGE, 0x01);
    if (g_crazy_debug >= 1 || r1 != 0) {
        printf("[crazy] enable bridge: %s (%d)\r\n", r1 == 0 ? "ok" : "FAIL", r1);
    }

    // Give CTS again for next TX (CF should have sent CTS after receiving)
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreGive(g_crazy_cts);

    // 7. Set client connected
    int r2 = cpx_send_system(CPX_SYS_SET_CLIENT, 0x01);
    if (g_crazy_debug >= 1 || r2 != 0) {
        printf("[crazy] set client: %s (%d)\r\n", r2 == 0 ? "ok" : "FAIL", r2);
    }

    printf("[crazy] initialized at %lu baud (CPX/CRTP)\r\n",
           (unsigned long)baudrate);
    return 0;
}

extern "C" int sentai_crazy_stop(void) {
    if (!g_crazy_running) return 0;

    // Clean disconnect: disable bridge + client disconnected
    // Give CTS in case TX is blocked
    if (g_crazy_cts) xSemaphoreGive(g_crazy_cts);
    cpx_send_system(CPX_SYS_SET_BRIDGE, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (g_crazy_cts) xSemaphoreGive(g_crazy_cts);
    cpx_send_system(CPX_SYS_SET_CLIENT, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    g_crazy_running = 0;
    vTaskDelay(pdMS_TO_TICKS(200));  // let RX task exit

    if (g_crazy_tx_mutex) { vSemaphoreDelete(g_crazy_tx_mutex); g_crazy_tx_mutex = nullptr; }
    if (g_crazy_cts)      { vSemaphoreDelete(g_crazy_cts);      g_crazy_cts = nullptr; }
    if (g_crazy_ping_sem) { vSemaphoreDelete(g_crazy_ping_sem); g_crazy_ping_sem = nullptr; }
    if (g_param_resp_sem) { vSemaphoreDelete(g_param_resp_sem); g_param_resp_sem = nullptr; }

    // Reset cached param IDs (will re-discover on next init)
    g_param_motor_m1 = g_param_motor_m2 = g_param_motor_m3 = g_param_motor_m4 = -1;
    g_param_motor_enable = -1;

    sentai_uart_restore_baudrate();
    sentai_uart_serial_close();

    printf("[crazy] stopped\r\n");
    return 0;
}

extern "C" int sentai_crazy_is_running(void) {
    return g_crazy_running;
}

extern "C" void sentai_crazy_set_debug(int level) {
    g_crazy_debug = level;
    printf("[crazy] debug=%d\r\n", level);
}

// ===================== Platform: Arm / Disarm =====================
// CrazyFlie firmware 2023+ requires explicit arming before motor output.
// Uses CRTP Platform service (port 0x0D, ch 0, sub-cmd armSystem).
// Without arming, the supervisor stays in 'idle' and ignores all thrust.

extern "C" int sentai_crazy_arm(void) {
    if (!g_crazy_running) return -1;

    uint8_t data[2] = {PLATFORM_CMD_ARM_SYSTEM, 0x01};
    int rc = crtp_send(CRTP_PORT_PLATFORM, 0, data, 2);

    if (g_crazy_debug >= 1)
        printf("[crazy] ARM %s\r\n", rc == 0 ? "sent" : "FAILED");
    return rc;
}

extern "C" int sentai_crazy_disarm(void) {
    if (!g_crazy_running) return -1;

    uint8_t data[2] = {PLATFORM_CMD_ARM_SYSTEM, 0x00};
    int rc = crtp_send(CRTP_PORT_PLATFORM, 0, data, 2);

    if (g_crazy_debug >= 1)
        printf("[crazy] DISARM %s\r\n", rc == 0 ? "sent" : "FAILED");
    return rc;
}

// ===================== HL Commander: Takeoff =====================
// CRTP port 0x08, channel 0
// Payload: [cmd=7][groupMask][height:f32][yaw:f32][useCurrentYaw:u8][duration:f32]
extern "C" int sentai_crazy_takeoff(float height, float duration,
                                     float yaw, int use_current_yaw,
                                     uint8_t group_mask) {
    if (!g_crazy_running) return -1;

    uint8_t data[15];
    int idx = 0;
    data[idx++] = HL_CMD_TAKEOFF_2;
    data[idx++] = group_mask;
    memcpy(&data[idx], &height, 4);    idx += 4;
    memcpy(&data[idx], &yaw, 4);       idx += 4;
    data[idx++] = use_current_yaw ? 1 : 0;
    memcpy(&data[idx], &duration, 4);  idx += 4;

    if (g_crazy_debug >= 1)
        printf("[crazy] TAKEOFF h=%.2f dur=%.1f yaw=%.2f curYaw=%d\r\n",
               (double)height, (double)duration, (double)yaw, use_current_yaw);

    return crtp_send(CRTP_PORT_SETPOINT_HL, 0, data, idx);
}

// ===================== HL Commander: Land =====================
// Same struct as takeoff, cmd=8.
extern "C" int sentai_crazy_land(float height, float duration,
                                  float yaw, int use_current_yaw,
                                  uint8_t group_mask) {
    if (!g_crazy_running) return -1;

    uint8_t data[15];
    int idx = 0;
    data[idx++] = HL_CMD_LAND_2;
    data[idx++] = group_mask;
    memcpy(&data[idx], &height, 4);    idx += 4;
    memcpy(&data[idx], &yaw, 4);       idx += 4;
    data[idx++] = use_current_yaw ? 1 : 0;
    memcpy(&data[idx], &duration, 4);  idx += 4;

    if (g_crazy_debug >= 1)
        printf("[crazy] LAND h=%.2f dur=%.1f\r\n",
               (double)height, (double)duration);

    return crtp_send(CRTP_PORT_SETPOINT_HL, 0, data, idx);
}

// ===================== HL Commander: Stop =====================
// Payload: [cmd=3][groupMask]
extern "C" int sentai_crazy_stop_motors(uint8_t group_mask) {
    if (!g_crazy_running) return -1;

    uint8_t data[2];
    data[0] = HL_CMD_STOP;
    data[1] = group_mask;

    if (g_crazy_debug >= 1) printf("[crazy] STOP motors\r\n");

    return crtp_send(CRTP_PORT_SETPOINT_HL, 0, data, 2);
}

// ===================== HL Commander: GoTo =====================
// Payload: [cmd=12][groupMask][relative][linear][x:f32][y:f32][z:f32][yaw:f32][duration:f32]
extern "C" int sentai_crazy_go_to(float x, float y, float z, float yaw,
                                   float duration,
                                   int relative, int linear,
                                   uint8_t group_mask) {
    if (!g_crazy_running) return -1;

    uint8_t data[24];
    int idx = 0;
    data[idx++] = HL_CMD_GO_TO_2;
    data[idx++] = group_mask;
    data[idx++] = relative ? 1 : 0;
    data[idx++] = linear   ? 1 : 0;
    memcpy(&data[idx], &x, 4);         idx += 4;
    memcpy(&data[idx], &y, 4);         idx += 4;
    memcpy(&data[idx], &z, 4);         idx += 4;
    memcpy(&data[idx], &yaw, 4);       idx += 4;
    memcpy(&data[idx], &duration, 4);  idx += 4;

    if (g_crazy_debug >= 1)
        printf("[crazy] GOTO (%.2f,%.2f,%.2f) yaw=%.2f dur=%.1f rel=%d lin=%d\r\n",
               (double)x, (double)y, (double)z, (double)yaw,
               (double)duration, relative, linear);

    return crtp_send(CRTP_PORT_SETPOINT_HL, 0, data, idx);
}

// ===================== Generic Setpoint: Hover =====================
// CRTP port 0x07, channel 0
// Payload: [type=5][vx:f32][vy:f32][yawrate:f32][zDistance:f32]
//
// IMPORTANT: Must be sent continuously at ~10-20 Hz.
// If sending stops, the CrazyFlie safety watchdog cuts motors after ~1s.
extern "C" int sentai_crazy_hover(float vx, float vy,
                                   float yaw_rate, float z_distance) {
    if (!g_crazy_running) return -1;

    uint8_t data[17];
    int idx = 0;
    data[idx++] = SETPOINT_TYPE_HOVER;
    memcpy(&data[idx], &vx, 4);          idx += 4;
    memcpy(&data[idx], &vy, 4);          idx += 4;
    memcpy(&data[idx], &yaw_rate, 4);    idx += 4;
    memcpy(&data[idx], &z_distance, 4);  idx += 4;

    return crtp_send(CRTP_PORT_SETPOINT_GEN, 0, data, idx);
}

// ===================== Raw CRTP =====================
extern "C" int sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                                       const uint8_t* data, int len) {
    if (!g_crazy_running) return -1;
    return crtp_send(port, channel, data, len);
}

// ===================== Ping (CRTP Echo) =====================
// Send CRTP echo on LINK port (0x0F), channel 0.
// CrazyFlie echoes the payload back — we measure round-trip time.
extern "C" int sentai_crazy_ping(int timeout_ms) {
    if (!g_crazy_running) return -1;
    if (!g_crazy_ping_sem) return -1;

    // Drain any stale semaphore
    xSemaphoreTake(g_crazy_ping_sem, 0);

    // Echo payload — arbitrary test pattern
    uint8_t echo[4] = {0xDE, 0xAD, 0xBE, 0xEF};

    TickType_t t0 = xTaskGetTickCount();

    int rc = crtp_send(0x0F, 0, echo, 4);
    if (rc != 0) {
        if (g_crazy_debug >= 1) printf("[crazy] ping: send failed (%d)\r\n", rc);
        return -2;
    }

    // Wait for echo response (RX task gives g_crazy_ping_sem)
    if (xSemaphoreTake(g_crazy_ping_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        TickType_t elapsed = xTaskGetTickCount() - t0;
        int ms = (int)(elapsed * portTICK_PERIOD_MS);
        if (ms == 0) ms = 1;  // at least 1 ms if we got a response
        if (g_crazy_debug >= 1) printf("[crazy] ping: %d ms\r\n", ms);
        return ms;
    }

    if (g_crazy_debug >= 1) printf("[crazy] ping: timeout\r\n");
    return -3;
}

// ===================== CRTP Param Helpers =====================
// Send a CRTP packet and wait for response on the same port.
// Copies response payload (after CRTP header) into resp buffer.
// Returns 0 on success, -1 send fail, -2 timeout.
static int crtp_param_send_and_wait(uint8_t ch,
                                    const uint8_t* data, int len,
                                    uint8_t* resp, int* resp_len,
                                    int timeout_ms) {
    // Drain stale semaphore
    xSemaphoreTake(g_param_resp_sem, 0);
    g_param_resp_len = 0;

    int rc = crtp_send(CRTP_PORT_PARAM, ch, data, len);
    if (rc != 0) return -1;

    if (xSemaphoreTake(g_param_resp_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (resp && resp_len) {
            memcpy(resp, g_param_resp_buf, g_param_resp_len);
            *resp_len = g_param_resp_len;
        }
        return 0;
    }
    return -2;
}

// Write a uint8 param by ID.  Returns 0=ok, negative=error.
static int param_write_u8(uint16_t id, uint8_t value) {
    uint8_t data[3] = {(uint8_t)(id & 0xFF), (uint8_t)(id >> 8), value};
    return crtp_param_send_and_wait(PARAM_WRITE_CH, data, 3, nullptr, nullptr, 500);
}

// Write a uint16 param by ID.  Returns 0=ok, negative=error.
static int param_write_u16(uint16_t id, uint16_t value) {
    uint8_t data[4];
    data[0] = id & 0xFF;
    data[1] = id >> 8;
    data[2] = value & 0xFF;
    data[3] = value >> 8;
    return crtp_param_send_and_wait(PARAM_WRITE_CH, data, 4, nullptr, nullptr, 500);
}

// Scan the CF param TOC to discover motorPowerSet.{m1,m2,m3,m4,enable} IDs.
// Stores IDs in g_param_motor_* statics.  Returns 0 if all 5 found.
static int param_discover_motors(void) {
    // Step 1: Get TOC info (param count)
    uint8_t cmd = PARAM_TOC_GET_INFO_V2;
    uint8_t resp[32];
    int rlen = 0;

    int src = crtp_param_send_and_wait(PARAM_TOC_CH, &cmd, 1, resp, &rlen, 1000);
    if (src != 0) {
        printf("[crazy] param TOC info failed (send_and_wait=%d)\r\n", src);
        return -1;
    }
    if (g_crazy_debug >= 1) {
        printf("[crazy] TOC info resp: rlen=%d", rlen);
        for (int k = 0; k < rlen && k < 12; k++) printf(" %02X", resp[k]);
        printf("\r\n");
    }
    if (rlen < 3 || resp[0] != PARAM_TOC_GET_INFO_V2) {
        printf("[crazy] TOC info bad: rlen=%d resp[0]=%02X (want %02X)\r\n",
               rlen, rlen > 0 ? resp[0] : 0xFF, PARAM_TOC_GET_INFO_V2);
        return -2;
    }

    uint16_t count = resp[1] | (resp[2] << 8);
    if (g_crazy_debug >= 1)
        printf("[crazy] param TOC: %u params, scanning...\r\n", count);

    // Step 2: Scan TOC items for motorPowerSet group
    int found = 0;
    for (uint16_t id = 0; id < count && found < 5; id++) {
        uint8_t req[3] = {PARAM_TOC_GET_ITEM_V2,
                          (uint8_t)(id & 0xFF), (uint8_t)(id >> 8)};
        rlen = 0;

        if (crtp_param_send_and_wait(PARAM_TOC_CH, req, 3, resp, &rlen, 500) != 0)
            continue;

        // Response: [cmd, id_lo, id_hi, type, group\0name\0]
        if (rlen < 6) continue;

        const char* group = (const char*)&resp[4];
        int group_len = (int)strlen(group);
        if (4 + group_len + 1 >= rlen) continue;
        const char* name = group + group_len + 1;

        if (strcmp(group, "motorPowerSet") == 0) {
            if      (strcmp(name, "m1") == 0)     { g_param_motor_m1 = (int16_t)id; found++; }
            else if (strcmp(name, "m2") == 0)     { g_param_motor_m2 = (int16_t)id; found++; }
            else if (strcmp(name, "m3") == 0)     { g_param_motor_m3 = (int16_t)id; found++; }
            else if (strcmp(name, "m4") == 0)     { g_param_motor_m4 = (int16_t)id; found++; }
            else if (strcmp(name, "enable") == 0) { g_param_motor_enable = (int16_t)id; found++; }

            if (g_crazy_debug >= 1)
                printf("[crazy] param %s.%s → id=%u\r\n", group, name, id);
        }
    }

    if (found == 5) {
        if (g_crazy_debug >= 1) printf("[crazy] all motorPowerSet params found\r\n");
        return 0;
    }

    printf("[crazy] motorPowerSet: found %d/5 params\r\n", found);
    return -3;
}

// ===================== Motor Test (motorPowerSet params) =====================
// Sets motorPowerSet.m1-m4 to power, enables motorPowerSet.enable=1,
// waits duration_ms, then disables and zeros.  Exactly like cflib motor test.
// power: 0-65535 (10% = 6553, 100% = 65535)
// Returns 0 on success, negative on error.
extern "C" int sentai_crazy_test_fly(uint16_t power, int duration_ms) {
    if (!g_crazy_running) return -1;

    // Discover motor param IDs if not already done
    if (g_param_motor_enable < 0) {
        if (g_crazy_debug >= 1) printf("[crazy] test_fly: discovering motor params...\r\n");
        int rc = param_discover_motors();
        if (rc != 0) {
            printf("[crazy] test_fly: motor param discovery failed (%d)\r\n", rc);
            return -2;
        }
    }

    if (g_crazy_debug >= 1)
        printf("[crazy] TEST_FLY power=%u dur=%d ms\r\n", power, duration_ms);

    // Set motor power
    param_write_u16((uint16_t)g_param_motor_m1, power);
    param_write_u16((uint16_t)g_param_motor_m2, power);
    param_write_u16((uint16_t)g_param_motor_m3, power);
    param_write_u16((uint16_t)g_param_motor_m4, power);
    param_write_u8((uint16_t)g_param_motor_enable, 1);

    if (g_crazy_debug >= 1) printf("[crazy] motors ON\r\n");

    // Wait
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop
    param_write_u8((uint16_t)g_param_motor_enable, 0);
    param_write_u16((uint16_t)g_param_motor_m1, 0);
    param_write_u16((uint16_t)g_param_motor_m2, 0);
    param_write_u16((uint16_t)g_param_motor_m3, 0);
    param_write_u16((uint16_t)g_param_motor_m4, 0);

    if (g_crazy_debug >= 1) printf("[crazy] motors OFF\r\n");
    return 0;
}
