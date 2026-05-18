// CrazyFlie autopilot bridge for SentAI
// CRTP commands tunneled through CPX over UART.
//
// Protocol stack:
//   Application (fly/attitude/takeoff/land/hover) → CRTP → CPX → UART
//
// Architecture:
//   - FreeRTOS RX task: parses CPX frames, handles CTS flow control
//   - FreeRTOS CMD task: sends Commander setpoints at 20 Hz (for attitude())
//   - TX is synchronous: build CRTP packet, wrap in CPX, send over UART
//   - Uses existing sentai_uart_serial_* bridge (LPUART6 via ConsoleM7)
//   - UART cannot be shared with sentai.mesh or sentai.link — only one at a time
//
// CrazyFlie CRTP ports used:
//   0x03 — Commander (roll/pitch/yawrate/thrust at 50Hz, IMU-stabilized)
//   0x08 — High-Level Commander (takeoff, land, stop, goto — uses baro+Kalman)
//   0x07 — Generic Setpoint (hover velocity setpoint — needs flow deck)
//   0x09 — Supervisor (arm/disarm)
//   0x02 — Parameter system (TOC scan, motorPowerSet)
//
// CPX framing (UART):
//   Frame:  [0xFF] [LEN(1B)] [CPX_HDR(2B) + DATA...] [CRC]
//   CTS:    [0xFF] [0x00]    (2 bytes — Clear-To-Send, no CRC)
//   CPX_HDR byte 0: [reserved:1][lastPacket:1][source:3][destination:3]
//   CPX_HDR byte 1: [version:2][function:6]
//   Targets (3-bit): STM32=1, ESP32=2, HOST=3
//   Functions (6-bit): SYSTEM=1, CONSOLE=2, CRTP=3
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

// HL Commander STOP — sets the HL Commander state to IDLE so Generic
// Setpoint commands (hover/attitude) take effect WITHOUT being
// overridden by HL position-hold setpoints.  Does NOT kill motors;
// the caller must already be sending Generic Setpoints at 10+ Hz or
// the cf2 Commander watchdog will trigger motors-off after ~1 s.
// Use case (OP-S10-W14): release HL after sentai_crazy_takeoff() so
// the autotuner's hover() velocity commands actually drive the drone.
int sentai_crazy_hl_stop(uint8_t group_mask);

// Send PnP-derived absolute position (world frame, m) to the cf2
// EKF as an external measurement (CRTP LOCALIZATION port, ExtPos
// channel).  cf2 fuses with its baro + IMU to correct internal
// estimate drift.  Recommended rate: ~5 Hz.  Anti-cheat compliant
// because the caller passes PERCEPTION-derived (x,y,z), not GT.
int sentai_crazy_send_extpos(float x, float y, float z);

// OP-S10-W14-T13 — full POSE (position + quaternion).  cf2's EKF
// fuses BOTH position AND orientation, so this also corrects yaw
// drift (ExtPos alone leaves yaw to drift via gyro integration).
// Pass identity (0,0,0,1) to lock yaw to world +X, or PnP-derived
// quaternion for true orientation correction.
int sentai_crazy_send_extpose(float x, float y, float z,
                                float qx, float qy, float qz, float qw);

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

// ===================== Attitude-Stabilized Flight =====================
// Uses CRTP Commander (port 3, ch 0) with a FreeRTOS task at 50 Hz.
// CF onboard PID stabilizes roll/pitch using IMU (gyro + accel).
// Works without flow deck — no altitude hold, but drone stays level.

// Blocking HL Commander flight: arm → takeoff → hold → land → disarm.
// Uses Kalman estimator + barometer for altitude hold.
// HL Commander generates setpoints internally (no 20Hz loop needed).
// height_m: altitude in metres (0.5 = 50cm above takeoff point)
// hold_ms: time to hold at altitude (ms)
// takeoff_ms: takeoff duration (ms)
// land_ms: landing duration (ms)
// Returns 0=ok, -1=not running, -2=busy, -3=arm fail,
//         -4=takeoff fail, -5=aborted by fly_stop().
int sentai_crazy_fly(float height_m, int hold_ms,
                     int takeoff_ms, int land_ms);

// Non-blocking manual setpoint. CMD task sends it at 50 Hz.
// Auto-arms on first call. Use fly_stop() to land and disarm.
// roll/pitch: degrees, yawrate: deg/s, thrust: 0-65535 raw.
// Returns 0=ok, -1=not running, -2=auto fly in progress.
int sentai_crazy_attitude(float roll, float pitch,
                          float yawrate, uint16_t thrust);

// Stop flying and disarm. Non-blocking, safe from any state.
// Returns 0=ok, -1=not running.
int sentai_crazy_fly_stop(void);

// Get altitude from CF Kalman estimator (stateEstimate.z).
// On first call: discovers log var + starts streaming (~2-5s).
// Subsequent calls return the latest cached value (10 Hz updates).
// Returns altitude in metres, or -999.0 on error.
float sentai_crazy_get_altitude(void);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_CRAZY_H_
