# OP-S10-W26 — `sentai.link` review and PX4 connection readiness

**WBS**: OP-S10-W26-T2 (review) → T12…T17 (preparation tasks, appended 2026-11-03)
**Date**: 2026-11-03
**Scope**: what the MAVLink bridge can do today on the ARM board, what the pilot follow-me work package (chapter 23) needs from it, the defects found, and the ordered list of things to prepare before the first PX4 connection. STROBE wiring is dropped (operator: no free wires), so time alignment is purely `TIMESYNC` + GPT1 frame stamps.

## 1. What exists

### 1.1 Transport (ARM)

| Item | State | Source |
|---|---|---|
| Physical port | **LPUART6**, the single UART on the board (the pads the Crazyflie deck uses). Shared, one consumer at a time: REPL-on-UART, raw `sentai.uart`, `sentai.crazy`, `sentai.mesh`, `sentai.link`. | `agent/agent.md` §17–18, `modsentai_hal.cc:246-281` |
| Baud | `link.init(baud=57600)`; `LPUART_SetBaudRate(LPUART6, baud, 24 MHz)`; restored to 115200 on stop. | `modsentai_hal.cc:276`, `sentai_link.cc:169` |
| RX path | ConsoleM7 IRQ ring **2048 B** → `link_rx_task` (priority idle+2, 6 KB stack) reads 64 B per call with a 50 ms timeout → `mavlink_parse_char()` → 16-deep FreeRTOS queue of whole `mavlink_message_t` (~290 B each). | `console_m7.h:124`, `sentai_link.cc:129-165` |
| TX path | Synchronous pack + `mavlink_msg_to_send_buffer()` + UART write under a mutex (100 ms timeout, 3 retries). | `sentai_link.cc:84-126` |
| Exclusivity checks at init | REPL must be on USB; mesh must be stopped. **Does not check `sentai.crazy`** (same wire). | `modsentai_link.c:9-16` |
| Health / errors | `SUBSYS_LINK` health counters; `SERR_LINK_TX_TIMEOUT`, `SERR_LINK_TX_FAIL`, `SERR_LINK_RX_DROP`, `SERR_LINK_RX_PARSE` defined. Only the TX codes are ever logged. | `sentai_error.h:105-108` |
| MAVLink library | `third_party/mavlink` = c_library_v2 at 2026-03-26, `common` dialect; has `TIMESYNC`, `SYSTEM_TIME`, `ATTITUDE_QUATERNION`, `LOCAL_POSITION_NED`, `SET_POSITION_TARGET_LOCAL_NED`, `FOLLOW_TARGET`, `MESSAGE_INTERVAL`, `COMMAND_ACK`. Nothing to upgrade. | `.gitmodules`, submodule 358d35fd |

### 1.2 Messages handled on ARM

| Direction | Message | Notes |
|---|---|---|
| TX | HEARTBEAT (manual, from MicroPython) | no auto task on ARM; the SIM bridge has one |
| TX | STATUSTEXT (single and chunked base64 nanopb "visionmesh" alarm protocol) | project-specific, not a PX4 concern |
| TX | COMMAND_LONG (explicit target sys/comp) | no COMMAND_ACK tracking |
| TX | OBSTACLE_DISTANCE (raw, from tracker, from points) | `time_usec = tick × 1000`, not on the PX4 clock; PX4 collision prevention ignores it in Offboard anyway |
| RX | LOCAL_POSITION_NED (32), GLOBAL_POSITION_INT (33) decoded into a dict | **only consumer is `sentai.link.receive()` in MicroPython** |
| RX | everything else | queued raw; header fields only |

`sentai.link.stats / arm / takeoff / land / send_flow / flow` are **weak symbols**: on ARM they return `-99` (or zeros for `stats`) because only `sim/sentai_mavlink_udp_bridge.cc` implements them. The same is true of `sentai_link_pose()` and `sentai_link_cmd_move()` that the PX4 backend of `sentai.servo` needs, so **`sentai.servo` on PX4 is non-functional on hardware today** (`sentai_servo.cc:195-230`).

### 1.3 The SIM bridge (port source)

`sim/sentai_mavlink_udp_bridge.cc` (x86 UDP to PX4 SITL, s095–s110) already has: a reader task with a seqlock pose cache (LOCAL_POSITION_NED + ATTITUDE → ENU + yaw), a 1 Hz heartbeat task, arm / takeoff / land / set-mode helpers, `cmd_move` as `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_OFFSET_NED` (position bits only), the C optical-flow forwarder (`OPTICAL_FLOW_RAD`, 50 Hz), stats. Validated in s107 (Offboard position hold on PX4 SITL, drift < 2 cm in 28 s). It is a separate implementation from the ARM file, not the same code with a different transport.

### 1.4 Documentation state

`paper/appendices/13_sentai_link.md` describes the ARM API but omits the weak-symbol functions and the UART exclusivity; `SENTAI_API.md` has **no `sentai.link` section**; `agent/agent.md` §17–18 is the authoritative note on the shared UART.

## 2. What W26 needs from the link

| # | Requirement | Why |
|---|---|---|
| R1 | Attitude (quaternion) + angular rates + local position at ≥ 50 Hz, each stamped with the PX4 `time_boot_ms`/µs | chapter 23 §3: attitude at the frame's exposure time, rate gating |
| R2 | Clock sync: `TIMESYNC` exchange, offset PX4 clock ↔ GPT1 µs (`g_cam_buf_ts_us` already stamps every camera buffer) | replaces STROBE |
| R3 | Time-indexed pose ring in C (128 entries at 100 Hz ≈ 1.3 s) with `pose_at(t_us)` interpolation | estimator input (memo 23b §8.4) |
| R4 | GLOBAL_POSITION_INT + HOME/GPS_GLOBAL_ORIGIN decode, plus `DISTANCE_SENSOR` / `dist_bottom` if a rangefinder exists | lat/lon ↔ local frame for FOLLOW_TARGET; height above ground for the ground-plane projection |
| R5 | TX `FOLLOW_TARGET` at 10–20 Hz from C | T0 baseline (PX4 Follow Me on our estimate) and the first flying demo |
| R6 | TX `SET_POSITION_TARGET_LOCAL_NED` with velocity + acceleration feed-forward + yaw/yaw-rate at 20–50 Hz **from a C task** (Offboard keep-alive ≥ 2 Hz), `DO_SET_MODE` Offboard, arm | controller (chapter 23 §5) |
| R7 | Auto heartbeat task (1 Hz) and peer tracking: PX4 sysid/compid, mode, armed, last-seen time | Offboard entry and link-loss detection |
| R8 | ~~RX of the pilot coordinate~~ — dropped: vision only (operator 2026-11-03, chapter 23 §0). `MAV_1_FORWARD` stays useful only for GCS diagnostics. | — |
| R9 | `COMMAND_ACK` handling, `MESSAGE_INTERVAL` requests at init | deterministic stream rates, no silent command failures |
| R10 | Baud ≥ 460 800 (see §3.1) | bandwidth |
| R11 | `NAMED_VALUE_FLOAT` / `DEBUG_VECT` TX for estimator scalars | ulog post-mortem with FC-aligned time |
| R12 | Link-loss → `sentai.safety` event → stop setpoints (PX4 `COM_OBL_RC_ACT` then acts) | safe state |
| R13 | UART ownership resolved at boot (Crazyflie auto-init must not own LPUART6 on a PX4 vehicle) | the link cannot open otherwise |

## 3. Findings (ranked)

### 3.1 Bandwidth: 57 600 baud cannot carry the pose stream

Wire bytes per message (payload + 12 B MAVLink v2 framing): ATTITUDE_QUATERNION 44, ATTITUDE 40, LOCAL_POSITION_NED 40, GLOBAL_POSITION_INT 40, HEARTBEAT 21, HIGHRES_IMU 74. At 10 bits per byte:

| Stream mix | Bytes/s | Bits/s | 57 600 | 230 400 | 921 600 |
|---|---|---|---|---|---|
| attitude 50 Hz + local pos 50 Hz + global 10 Hz + hb | 4.6 k | 46 k | 80 % | 20 % | 5 % |
| attitude 100 Hz + local pos 50 Hz + IMU 100 Hz | 13.8 k | 138 k | over | 60 % | 15 % |
| our TX: setpoints 50 Hz (65 B) + FOLLOW_TARGET 20 Hz (105 B) + hb | 5.4 k | 54 k | 94 % | 23 % | 6 % |

Set PX4 `SER_TEL2_BAUD = 921600` (the companion default) and `link.init(921600)`. LPUART6 runs from a 24 MHz root; the SDK divider search handles 921 600 (verify the actual error with `LPUART_SetBaudRate` return code — it is currently ignored).

### 3.2 No C-side consumer: every message goes through MicroPython

At 100 messages/s the 16-deep queue overflows unless a Python loop drains it faster than 10 ms per message. This is the "no per-frame work in MicroPython" anti-pattern applied to telemetry. Needed: a C dispatcher in `link_rx_task` that handles HEARTBEAT, ATTITUDE(_QUATERNION), LOCAL_POSITION_NED, GLOBAL_POSITION_INT, TIMESYNC, SYSTEM_TIME, COMMAND_ACK in place (pose ring, peer state, clock offset) and queues only unhandled or subscribed message IDs for `receive()`.

### 3.3 RX ring vs task priority

The ConsoleM7 ring is 2048 B: at 921 600 baud it fills in 22 ms. `link_rx_task` runs at idle+2 and will be starved by the camera, PXP and TPU tasks for longer than that under load (W24 soak logs show > 30 ms gaps). Either enlarge the ring (8–16 KB) or raise the task priority above the pipeline consumers, and log `SERR_LINK_RX_PARSE` from `mavlink_status.packet_rx_drop_count` so overruns are visible.

### 3.4 Offboard prerequisites absent on ARM

No heartbeat task, no setpoint task, no `cmd_move`, no `pose`, no mode/arm helpers. PX4 refuses Offboard without a setpoint stream and drops it if the stream stops; the SIM bridge code is the port source (§1.3).

### 3.5 Time base

`OBSTACLE_DISTANCE.time_usec = xTaskGetTickCount() × 1000`. Every outgoing timestamp must use GPT1 µs plus the `TIMESYNC` offset; every incoming pose must keep its PX4 stamp.

### 3.6 Init does not check `sentai.crazy`

`link.init()` refuses REPL-on-UART and mesh, but not a running Crazyflie bridge, which uses the same LPUART6 at 576 000 baud. Add the check, and confirm on a PX4 vehicle that the boot-time Crazyflie auto-init (memory: radio bridge auto-inits pre-main) is disabled by configuration, otherwise the link can never open cleanly.

### 3.7 `stats()` lies on ARM

The weak `sentai_link_get_stats` makes `sentai.link.stats()` return nine zeros on hardware although `g_link_tx_ok / tx_fail / tx_timeout / rx_ok / rx_dropped` exist. Export the real counters, add `rx_parse_err`, `last_peer_sysid/compid`, `last_heartbeat_age_ms`, `time_offset_us`.

### 3.8 Error logging gaps

RX queue overflow bumps a counter and the health subsystem but never `SERR_LOG(SERR_LINK_RX_DROP)`; parse errors are never counted. Append-only rule: keep the codes, add the log calls.

### 3.9 Minor

- `receive()` allocates a dict per message (fine for diagnostics only).
- `visionmesh` STATUSTEXT chunking is dead weight for PX4; leave it, do not extend it.
- The ARM and SIM link implementations have diverged (two files, no shared dispatcher). SIM link is also currently unlinkable (W24-T21). Sharing the dispatcher and pose ring with only the byte transport differing is the right shape, but it is a refactor, not a prerequisite for the first hardware connection.

## 4. Preparation tasks before the first PX4 connection (ordered)

| Task | Content | Done when |
|---|---|---|
| **T12 Bench and wiring** | PX4 TELEM2 ↔ LPUART6 on the deck pads (TX/RX/GND, 3.3 V logic); PX4 params `MAV_1_CONFIG=TELEM2`, `MAV_1_MODE=Onboard`, `MAV_1_RATE`, `MAV_1_FORWARD=1` (so phone/GCS traffic is visible), `SER_TEL2_BAUD=921600`; PX4 version recorded (Follow Me needs ≥ 1.14); Crazyflie auto-init disabled by config on this vehicle. New experiment `s270_px4_link_ping_arm`: heartbeats both ways, RX drop count 0 over 10 min with the camera pipeline running. | `stats()` shows peer sysid, 0 drops, 0 parse errors at 921 600 |
| **T13 Link core on ARM** | C dispatcher (§3.2); pose ring + `pose_at(t_us)`; peer state; auto heartbeat task; `MESSAGE_INTERVAL` requests at init (attitude 100 Hz, local pos 50 Hz, global 10 Hz); real `stats()`; `SERR_LINK_RX_*` logging; `crazy` exclusivity check; RX ring/priority fix (§3.3); `link.init()` returns the baud error. MicroPython sees only scalars (`sentai.link.pose()`, `sentai.link.peer()`). | s270 passes at full stream rates; `pose()` age < 20 ms under pipeline load |
| **T14 Clock sync** | `TIMESYNC` client (board `tc1=0, ts1=GPT1 µs`; reply gives offset; median of 8, re-sync every 1 s); `sentai.link.time_offset_us()`; camera frame stamp → PX4 time via `g_cam_buf_ts_us`; experiment `s271_px4_timesync_latency`: offset jitter and round-trip, pose latency vs frame stamp. | offset jitter < 1 ms; documented one-way latency |
| **T15 Offboard TX in C** | Port `cmd_move`, `pose`, arm, mode (`DO_SET_MODE` Offboard) from the SIM bridge; `setpoint_task` at 50 Hz sending `SET_POSITION_TARGET_LOCAL_NED` from a shared struct (velocity + acceleration + yaw/yaw-rate type-mask variants); `FOLLOW_TARGET` TX helper for T0; `NAMED_VALUE_FLOAT` helper for ulog. `sentai.servo` PX4 backend becomes functional on hardware. | PX4 SITL over the real UART (or a USB-serial PX4 bench) holds Offboard for 60 s driven from the board |
| **T16 Safety glue** | Heartbeat-age watchdog → `sentai.safety` LINK_LOST event → setpoint task stops within 200 ms; PX4 `COM_OBL_RC_ACT`, `COM_OF_LOSS_T`, geofence set and tested by pulling the UART. | PX4 enters the configured failsafe within its timeout; event logged |
| **T17 Docs + parity** | `SENTAI_API.md` link section; appendix 13 update; mark SIM-only functions; plan the shared dispatcher refactor with W24-T21 (SIM link). | docs match the binding |

Nothing in T12–T16 touches the camera, TPU or flow hot paths; the only shared resource is LPUART6, which this work package takes away from the Crazyflie bridge on the PX4 vehicle.
