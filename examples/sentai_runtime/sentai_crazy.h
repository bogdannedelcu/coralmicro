// CrazyFlie autopilot bridge for SentAI
// CRTP commands tunneled through CPX over UART.
//
// Protocol stack:
//   Application (takeoff/land/hover/goto) → CRTP → CPX → UART
//
// Architecture:
//   - Dedicated FreeRTOS RX task parses CPX frames, handles flow control
//   - TX is synchronous: build CRTP packet, wrap in CPX, send over UART
//   - Uses existing sentai_uart_serial_* bridge (LPUART6 via ConsoleM7)
//   - UART cannot be shared with sentai.mesh or sentai.link — only one at a time
//
// CrazyFlie CRTP ports used:
//   0x08 — High-Level Commander (takeoff, land, stop, goto)
//   0x07 — Generic Setpoint (hover velocity setpoint)
//
// CPX framing (UART):
//   Frame:  [0xFF] [LEN(1B)] [CPX_HDR(2B) + DATA...] [CRC]
//   CTS:    [0xFF] [0x00]    (2 bytes — Clear-To-Send, no CRC)
//   LEN:    payload size (CPX_HDR + DATA), without CRC
//   CRC:    XOR of ALL bytes including 0xFF and LEN
//   MTU:    100 bytes max payload per frame
//   CPX_HDR byte 0: [destination:4][source:4]
//   CPX_HDR byte 1: [function:8]
//
// Targets: STM32=0x01, ESP32=0x02, HOST=0x03
// Functions: SYSTEM=0x00, CONSOLE=0x01, CRTP=0x02
//
// Init sequence: CTS sync → enable CRTP bridge → client connected

#ifndef SENTAI_CRAZY_H_
#define SENTAI_CRAZY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== Initialization =====================

// Initialize CrazyFlie CPX bridge: opens UART, starts RX task.
// baudrate: typically 576000 for CrazyFlie deck UART (UART2).
// Returns 0 on success, negative on error.
int sentai_crazy_init(uint32_t baudrate);

// Stop bridge: stops RX task, closes UART.
int sentai_crazy_stop(void);

// Check if bridge is running.
int sentai_crazy_is_running(void);

// Set debug level: 0=off, 1=TX/RX summary, 2=+hex dump
void sentai_crazy_set_debug(int level);

// ===================== Platform: Arm / Disarm =====================
// CrazyFlie 2023+ firmware requires arming before motor output.
// Without arm(), the supervisor stays in 'idle' and ignores thrust.
// Uses CRTP Platform service (port 0x0D, ch 0, armSystem command).
int sentai_crazy_arm(void);
int sentai_crazy_disarm(void);

// ===================== High-Level Commander (one-shot) =====================
// These are trajectory commands executed autonomously by the CrazyFlie.
// Send once → CF executes the full manoeuvre.

// Takeoff to absolute height (metres) over duration (seconds).
// yaw: heading in radians (ignored if use_current_yaw != 0).
// group_mask: 0 = all, bitmask for specific groups.
int sentai_crazy_takeoff(float height, float duration,
                         float yaw, int use_current_yaw, uint8_t group_mask);

// Land to height (metres, typically 0.0) over duration (seconds).
int sentai_crazy_land(float height, float duration,
                      float yaw, int use_current_yaw, uint8_t group_mask);

// Emergency stop — all motors off immediately.
int sentai_crazy_stop_motors(uint8_t group_mask);

// Go to position (x, y, z metres) with yaw (radians) over duration (seconds).
// relative: 1 = relative to current position, 0 = absolute.
// linear:   1 = straight line, 0 = smooth polynomial trajectory.
int sentai_crazy_go_to(float x, float y, float z, float yaw, float duration,
                       int relative, int linear, uint8_t group_mask);

// ===================== Generic Setpoint: Hover =====================
// Velocity-based hover. Must be sent CONTINUOUSLY at ~10-20 Hz.
// If you stop sending, the CrazyFlie safety watchdog cuts motors after ~1s.
//
// vx, vy:     velocity in m/s (body frame)
// yaw_rate:   rotation in deg/s
// z_distance: absolute height in metres
int sentai_crazy_hover(float vx, float vy, float yaw_rate, float z_distance);

// ===================== Raw CRTP =====================
// Send an arbitrary CRTP packet through the CPX tunnel.
// port: 0-15, channel: 0-3, data: up to 30 bytes payload.
int sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                           const uint8_t* data, int len);

// ===================== Status / Ping =====================
// Ping CrazyFlie via CRTP echo (LINK port 0x0F, channel 0).
// Sends 4 bytes, waits for echo response.
// Returns round-trip time in ms on success, or negative on error:
//   -1: bridge not running
//   -2: send failed (CTS timeout or UART error)
//   -3: no response (timeout — drone not responding)
int sentai_crazy_ping(int timeout_ms);

// ===================== Commander: Test Fly =====================
// Set motorPowerSet params (m1-m4 + enable) via CRTP param system.
// Exactly like cflib motor test — no arming or position estimator needed.
// power: 0-65535 (10% = 6553)
// duration_ms: how long to run (function blocks)
// First call scans param TOC (takes ~1-2s), subsequent calls are instant.
// Returns 0 on success, -1=not running, -2=param discovery failed.
int sentai_crazy_test_fly(uint16_t power, int duration_ms);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_CRAZY_H_
