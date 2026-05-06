// ============== sentai.crazy — CrazyFlie autopilot bridge ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Python API:
//   sentai.crazy.init(baudrate=576000)        -> int (0=ok)
//   sentai.crazy.stop()                       -> int
//   sentai.crazy.debug(level)                 -> None
//   sentai.crazy.takeoff(h=0.5, dur=2.0, ...)  -> int
//   sentai.crazy.land(h=0.0, dur=2.0, ...)     -> int
//   sentai.crazy.stop_motors(group=0)          -> int
//   sentai.crazy.hover(vx=0, vy=0, yr=0, z=0.5) -> int  (call at ~10-20 Hz!)
//   sentai.crazy.go_to(x, y, z, ...)           -> int
//   sentai.crazy.send_crtp(port, ch, data)     -> int  (raw CRTP packet)

// sentai.crazy.init(baudrate=576000) -> int
static mp_obj_t mod_sentai_crazy_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 576000;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    if (sentai_mesh_is_running()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("mesh is running: call sentai.mesh.stop() first"));
    }
    if (sentai_link_is_running()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("link is running: call sentai.link.stop() first"));
    }
    return mp_obj_new_int(sentai_crazy_init(baudrate));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_init_obj, 0, 1, mod_sentai_crazy_init);

// sentai.crazy.stop() -> int
static mp_obj_t mod_sentai_crazy_stop(void) {
    return mp_obj_new_int(sentai_crazy_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_stop_obj, mod_sentai_crazy_stop);

// sentai.crazy.debug(level) -> None
// 0=off, 1=TX/RX summary, 2=+hex dump
static mp_obj_t mod_sentai_crazy_debug(mp_obj_t level_obj) {
    sentai_crazy_set_debug(mp_obj_get_int(level_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_crazy_debug_obj, mod_sentai_crazy_debug);

// sentai.crazy.arm() -> int
// Arm the CrazyFlie (required before motors will spin).
// CF 2023+ firmware requires explicit arming. Returns 0=ok.
static mp_obj_t mod_sentai_crazy_arm(void) {
    return mp_obj_new_int(sentai_crazy_arm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_arm_obj, mod_sentai_crazy_arm);

// sentai.crazy.disarm() -> int
// Disarm the CrazyFlie (motors won't spin until re-armed).
static mp_obj_t mod_sentai_crazy_disarm(void) {
    return mp_obj_new_int(sentai_crazy_disarm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_disarm_obj, mod_sentai_crazy_disarm);

// sentai.crazy.takeoff(height=0.5, duration=2.0, yaw=0, use_current_yaw=1, group=0) -> int
static mp_obj_t mod_sentai_crazy_takeoff(size_t n_args, const mp_obj_t *args) {
    float height   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.5f;
    float duration = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw      = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int use_yaw    = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t group  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_takeoff(height, duration, yaw, use_yaw, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_takeoff_obj, 0, 5, mod_sentai_crazy_takeoff);

// sentai.crazy.land(height=0.0, duration=2.0, yaw=0, use_current_yaw=1, group=0) -> int
static mp_obj_t mod_sentai_crazy_land(size_t n_args, const mp_obj_t *args) {
    float height   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float duration = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw      = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int use_yaw    = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t group  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_land(height, duration, yaw, use_yaw, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_land_obj, 0, 5, mod_sentai_crazy_land);

// sentai.crazy.stop_motors(group=0) -> int
// Emergency: kills all motors immediately.
static mp_obj_t mod_sentai_crazy_stop_motors(size_t n_args, const mp_obj_t *args) {
    uint8_t group = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_stop_motors(group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_stop_motors_obj, 0, 1, mod_sentai_crazy_stop_motors);

// sentai.crazy.hover(vx=0, vy=0, yaw_rate=0, z=0.5) -> int
// MUST be called continuously at ~10-20 Hz to maintain hover!
// vx, vy: m/s (body frame), yaw_rate: deg/s, z: metres (absolute)
static mp_obj_t mod_sentai_crazy_hover(size_t n_args, const mp_obj_t *args) {
    float vx       = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float vy       = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0f;
    float yaw_rate = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    float z        = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.5f;
    return mp_obj_new_int(sentai_crazy_hover(vx, vy, yaw_rate, z));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_hover_obj, 0, 4, mod_sentai_crazy_hover);

// sentai.crazy.go_to(x, y, z, yaw=0, duration=3.0, relative=1, linear=0, group=0) -> int
// x, y, z: metres;  yaw: radians;  duration: seconds
// relative=1: relative to current position;  linear=1: straight line
static mp_obj_t mod_sentai_crazy_go_to(size_t n_args, const mp_obj_t *args) {
    float x        = mp_obj_get_float(args[0]);
    float y        = mp_obj_get_float(args[1]);
    float z        = mp_obj_get_float(args[2]);
    float yaw      = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.0f;
    float duration = (n_args > 4) ? mp_obj_get_float(args[4]) : 3.0f;
    int relative   = (n_args > 5) ? mp_obj_get_int(args[5])   : 1;
    int linear     = (n_args > 6) ? mp_obj_get_int(args[6])   : 0;
    uint8_t group  = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 0;
    return mp_obj_new_int(sentai_crazy_go_to(x, y, z, yaw, duration,
                                             relative, linear, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_go_to_obj, 3, 8, mod_sentai_crazy_go_to);

// sentai.crazy.send_crtp(port, channel, data) -> int
// Raw CRTP packet. data: bytes object (max 30 bytes).
static mp_obj_t mod_sentai_crazy_send_crtp(mp_obj_t port_obj,
                                           mp_obj_t ch_obj,
                                           mp_obj_t data_obj) {
    uint8_t port = (uint8_t)mp_obj_get_int(port_obj);
    uint8_t ch   = (uint8_t)mp_obj_get_int(ch_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_crazy_send_crtp(
        port, ch, (const uint8_t*)bufinfo.buf, bufinfo.len));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_sentai_crazy_send_crtp_obj, mod_sentai_crazy_send_crtp);

// sentai.crazy.ping(timeout_ms=1000) -> int
// Send CRTP echo to CrazyFlie and wait for response.
// Returns round-trip time in ms (>0) on success, or negative on error:
//   -1: bridge not running
//   -2: send failed
//   -3: timeout (drone not responding)
static mp_obj_t mod_sentai_crazy_ping(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 1000;
    return mp_obj_new_int(sentai_crazy_ping(timeout_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_ping_obj, 0, 1, mod_sentai_crazy_ping);

// sentai.crazy.test_fly(power=6553, duration_ms=2000) -> int
// Motor test via motorPowerSet params (same as cflib).
// power: 0-65535 (10% = 6553), duration_ms: blocking time.
// First call scans param TOC (~1-2s), then instant.
static mp_obj_t mod_sentai_crazy_test_fly(size_t n_args, const mp_obj_t *args) {
    uint16_t power      = (n_args > 0) ? (uint16_t)mp_obj_get_int(args[0]) : 6553;
    int      duration_ms = (n_args > 1) ? mp_obj_get_int(args[1])           : 2000;
    return mp_obj_new_int(sentai_crazy_test_fly(power, duration_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_test_fly_obj, 0, 2, mod_sentai_crazy_test_fly);

// sentai.crazy.fly(height=0.5, hold_ms=2000, takeoff_ms=3000, land_ms=3000) -> int
// Blocking HL Commander flight: arm → takeoff → hold → land → disarm.
// Uses Kalman estimator + barometer for altitude hold.
// height: altitude in metres (0.5 = 50cm above takeoff point).
static mp_obj_t mod_sentai_crazy_fly(size_t n_args, const mp_obj_t *args) {
    float height_m   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.5f;
    int   hold_ms    = (n_args > 1) ? mp_obj_get_int(args[1])   : 2000;
    int   takeoff_ms = (n_args > 2) ? mp_obj_get_int(args[2])   : 3000;
    int   land_ms    = (n_args > 3) ? mp_obj_get_int(args[3])   : 3000;
    return mp_obj_new_int(sentai_crazy_fly(height_m, hold_ms, takeoff_ms, land_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_fly_obj, 0, 4, mod_sentai_crazy_fly);

// sentai.crazy.attitude(roll=0, pitch=0, yaw_rate=0, thrust=0) -> int
// Non-blocking: set Commander setpoint. CMD task sends at 50 Hz.
// Auto-arms on first call. Use fly_stop() to land.
static mp_obj_t mod_sentai_crazy_attitude(size_t n_args, const mp_obj_t *args) {
    float    roll    = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float    pitch   = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0f;
    float    yawrate = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    uint16_t thrust  = (n_args > 3) ? (uint16_t)mp_obj_get_int(args[3]) : 0;
    return mp_obj_new_int(sentai_crazy_attitude(roll, pitch, yawrate, thrust));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_attitude_obj, 0, 4, mod_sentai_crazy_attitude);

// sentai.crazy.fly_stop() -> int
// Stop flying and disarm. Non-blocking, safe from any state.
static mp_obj_t mod_sentai_crazy_fly_stop(void) {
    return mp_obj_new_int(sentai_crazy_fly_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_fly_stop_obj, mod_sentai_crazy_fly_stop);

// =====================================================================
// CH_TELEM (channel 2) — drone telemetry queries via the deck-driver
// path. Each call sends a 1-byte cmd to the drone, drone reads the
// matching log var and replies with `[cmd][float32]`. Round-trip is
// typically a few ms (UART latency dominated).
//
// Returns the float (NaN on unknown cmd, default on transport error).
//
// On error returns the default-on-fail value (-999.0 by convention,
// matching the legacy `sentai.crazy.altitude()` API). Callers wanting
// to distinguish should use `sentai.crazy.telem(cmd, timeout)` which
// raises OSError on transport failure.
// =====================================================================
extern int sentai_crazy_query_telemetry(uint8_t cmd, float* out, int timeout_ms);

#define TELEM_BARO_ASL     0x01
#define TELEM_STATE_Z      0x02
#define TELEM_BATTERY_V    0x03
#define TELEM_BATTERY_PCT  0x04
#define TELEM_TEMP_C       0x05
#define TELEM_PRESSURE     0x06

static float telem_or_default(uint8_t cmd, int timeout_ms, float fallback) {
    float v = fallback;
    if (sentai_crazy_query_telemetry(cmd, &v, timeout_ms) != 0) return fallback;
    return v;
}

// sentai.crazy.baro(timeout_ms=200) -> float
//   Barometric altitude above sea level (m). Raw baro reading, NOT
//   fused with IMU. Use altitude() for the EKF-fused estimate.
static mp_obj_t mod_sentai_crazy_baro(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BARO_ASL, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_baro_obj, 0, 1, mod_sentai_crazy_baro);

// sentai.crazy.altitude(timeout_ms=200) -> float
//   Fused altitude estimate (m) from CF Kalman estimator. Combines
//   barometer + IMU + flow if a flow deck is attached. Replaces the
//   former CPX-based altitude() which is now defunct (CPX disabled).
static mp_obj_t mod_sentai_crazy_altitude(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_STATE_Z, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_altitude_obj, 0, 1, mod_sentai_crazy_altitude);

// sentai.crazy.battery(timeout_ms=200) -> float
//   Battery voltage (V).
static mp_obj_t mod_sentai_crazy_battery(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BATTERY_V, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_battery_obj, 0, 1, mod_sentai_crazy_battery);

// sentai.crazy.battery_pct(timeout_ms=200) -> float
//   Battery level (%).
static mp_obj_t mod_sentai_crazy_battery_pct(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BATTERY_PCT, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_battery_pct_obj, 0, 1, mod_sentai_crazy_battery_pct);

// sentai.crazy.temp(timeout_ms=200) -> float
//   Barometer temperature (°C).
static mp_obj_t mod_sentai_crazy_temp(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_TEMP_C, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_temp_obj, 0, 1, mod_sentai_crazy_temp);

// sentai.crazy.pressure(timeout_ms=200) -> float
//   Atmospheric pressure (mbar).
static mp_obj_t mod_sentai_crazy_pressure(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_PRESSURE, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_pressure_obj, 0, 1, mod_sentai_crazy_pressure);

// sentai.crazy.telem(cmd:int, timeout_ms=200) -> float
//   Generic telemetry query — drops directly through to channel 2.
//   Useful for forward-compat: drone-side firmware can add cmd codes
//   without needing a new MP binding. Raises OSError on transport
//   failure so callers can distinguish "drone offline" from "value =
//   -999.0".
static mp_obj_t mod_sentai_crazy_telem(size_t n_args, const mp_obj_t* args) {
    uint8_t cmd = (uint8_t)mp_obj_get_int(args[0]);
    int t = (n_args > 1) ? mp_obj_get_int(args[1]) : 200;
    float v = 0.0f;
    int rc = sentai_crazy_query_telemetry(cmd, &v, t);
    if (rc != 0) {
        mp_raise_OSError(rc);
    }
    return mp_obj_new_float(v);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_telem_obj, 1, 2, mod_sentai_crazy_telem);

// =====================================================================
// sentai.crazy.on_message(callback|None) — async dispatch
// =====================================================================
//
// Register a single Python callable to receive every inbound 0xAA frame
// from the drone-side bridge except those handled by the built-in
// `$`-prefix REPL exec path. Setting None drops further frames silently.
//
// Callback signature:    fn(channel: int, data: bytes) -> None
//
// Invocation runs in the MicroPython VM context (via mp_sched_schedule)
// — fully heap-safe, can call link_send to reply, can raise exceptions
// (caught + reported back over the radio as `ERR ...`).
//
// Inbound CH_REPL fragments are reassembled in C (MF byte stripped); the
// callback receives the *full* message. Other channels deliver each
// frame's raw body untouched.
//
// Wire-side semantics live in sentai_crazy.cc. This file owns the MP
// interface only.
//
// (runtime.h / obj.h / lexer.h / parse.h / compile.h are already pulled
//  in by modsentai.c — modsentai_crazy.c is #include'd from there.)

extern int sentai_crazy_dispatch_pop(uint8_t* kind, uint8_t* channel,
                                     uint8_t* buf, int max, int* out_len);
extern int sentai_crazy_link_send(int channel, const uint8_t* data, int len);

// GC-rooted handler slot. NULL = no handler (drop frames).
MP_REGISTER_ROOT_POINTER(mp_obj_t crazy_msg_handler);

#define CRAZY_KIND_EXEC   1u
#define CRAZY_KIND_USER   2u
#define CRAZY_DRAIN_BUF   256

/* Note: an earlier "g_crazy_drain_pending" dedup flag was removed
 * after we found that a single lost trampoline call could leave it
 * stuck at 1 forever (no consumer to clear it), silently blocking
 * all further schedules. The cost of always-schedule is at most a
 * few "drain empty" trampoline invocations per burst — each exits
 * immediately on FIFO empty. Worth it for the safety. */

int sentai_crazy_handler_is_set(void) {
    /* Read of an aligned pointer is atomic on 32-bit; the rx task only
     * needs a binary "set or not" answer to decide whether to enqueue
     * USER-kind frames at all. Race against on_message(None) at worst
     * delivers one extra frame to the trampoline, which checks again. */
    return MP_STATE_VM(crazy_msg_handler) != MP_OBJ_NULL ? 1 : 0;
}

/* ---------- $-prefix built-in REPL exec ---------- */

/* One screenful is the bound on every reply path. Keeps stack vstrs tiny
 * and prevents runaway repr() output from monopolizing the radio link. */
#define CRAZY_REPLY_BUF   200

/* Silent-truncation counter for reply payloads. Visible via
 * sentai.crazy.diag() (TODO) — callers can detect when their REPL
 * output is being clipped instead of suspecting a network bug. */
static volatile uint32_t g_crazy_reply_truncated = 0;

static void crazy_send_reply(const uint8_t* data, int len) {
    if (len < 0) len = 0;
    if (len > CRAZY_REPLY_BUF) {
        g_crazy_reply_truncated++;
        len = CRAZY_REPLY_BUF;
    }
    sentai_crazy_link_send(0, data, len);
}

static void crazy_send_reply_str(const char* s) {
    crazy_send_reply((const uint8_t*)s, (int)strlen(s));
}

static void crazy_run_exec(const uint8_t* src, int src_len) {
    if (src_len <= 0) {
        crazy_send_reply_str("ERR empty");
        return;
    }

    /* mp_lexer_new_from_str_len with free_len=0 doesn't take ownership
     * of the bytes — it reads exactly `src_len` chars. The dispatch
     * trampoline keeps the source buffer alive on its stack until this
     * function returns, so no copy / heap alloc is needed here.
     *
     * We lex twice (EVAL first, FILE on SyntaxError) because mp_parse
     * consumes its lexer; pre-tokenizing once would require a custom
     * reader. Two lex passes are fine — the source is at most 200 B. */
    qstr src_name = qstr_from_str("<radio>");
    mp_obj_t val = mp_const_none;
    bool used_eval = true;
    bool ok = false;

    nlr_buf_t outer;
    if (nlr_push(&outer) == 0) {
        nlr_buf_t inner;
        mp_parse_tree_t pt;
        if (nlr_push(&inner) == 0) {
            mp_lexer_t* lex = mp_lexer_new_from_str_len(src_name,
                (const char*)src, (size_t)src_len, 0);
            pt = mp_parse(lex, MP_PARSE_EVAL_INPUT);
            nlr_pop();
        } else {
            used_eval = false;
            mp_lexer_t* lex2 = mp_lexer_new_from_str_len(src_name,
                (const char*)src, (size_t)src_len, 0);
            pt = mp_parse(lex2, MP_PARSE_FILE_INPUT);
        }
        mp_obj_t mod = mp_compile(&pt, src_name, false);
        val = mp_call_function_0(mod);
        ok = true;
        nlr_pop();
    }

    /* Stack-only vstr — no GC heap on the reply path. Wrap the print in
     * an inner NLR: vstr_ensure_extra on a fixed-buf vstr raises
     * RuntimeError when full, and we want to ship whatever fit instead
     * of silently propagating the overflow up the dispatch trampoline. */
    VSTR_FIXED(v, CRAZY_REPLY_BUF);
    mp_print_t pr = { &v, (mp_print_strn_t)vstr_add_strn };

    if (ok) {
        if (used_eval) {
            vstr_add_str(&v, "OK ");
            nlr_buf_t pn;
            if (nlr_push(&pn) == 0) {
                mp_obj_print_helper(&pr, val, PRINT_REPR);
                nlr_pop();
            }
            /* On overflow we just send the truncated prefix as-is. */
        } else {
            vstr_add_str(&v, "OK");
        }
    } else {
        vstr_add_str(&v, "ERR ");
        nlr_buf_t pn;
        if (nlr_push(&pn) == 0) {
            mp_obj_print_helper(&pr, MP_OBJ_FROM_PTR(outer.ret_val), PRINT_EXC);
            nlr_pop();
        }
    }
    /* Always ship the reply, even when truncated or empty. */
    crazy_send_reply((const uint8_t*)vstr_str(&v), (int)vstr_len(&v));
}

/* ---------- user on_message(channel, data) ---------- */

static void crazy_run_user(uint8_t channel, const uint8_t* data, int len) {
    mp_obj_t handler = MP_STATE_VM(crazy_msg_handler);
    if (handler == MP_OBJ_NULL) return;

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t args[2] = {
            MP_OBJ_NEW_SMALL_INT((mp_int_t)channel),
            mp_obj_new_bytes(data, (size_t)len),
        };
        mp_call_function_n_kw(handler, 2, 0, args);
        nlr_pop();
    } else {
        /* User handler raised — send a short error back, no traceback.
         * Inner NLR catches vstr fixed-buf overflow (RuntimeError) so we
         * always ship whatever fit. */
        VSTR_FIXED(v, CRAZY_REPLY_BUF);
        mp_print_t pr = { &v, (mp_print_strn_t)vstr_add_strn };
        vstr_add_str(&v, "ERR ");
        nlr_buf_t pn;
        if (nlr_push(&pn) == 0) {
            mp_obj_print_helper(&pr, MP_OBJ_FROM_PTR(nlr.ret_val), PRINT_EXC);
            nlr_pop();
        }
        crazy_send_reply((const uint8_t*)vstr_str(&v), (int)vstr_len(&v));
    }
}

/* ---------- drain trampoline (one mp_sched slot, drains entire FIFO) ---------- */

static mp_obj_t crazy_dispatch_drain(mp_obj_t arg) {
    (void)arg;
    for (;;) {
        uint8_t kind = 0, channel = 0;
        uint8_t buf[CRAZY_DRAIN_BUF];
        int len = 0;
        if (sentai_crazy_dispatch_pop(&kind, &channel, buf, sizeof(buf), &len) == 0)
            break;
        if (kind == CRAZY_KIND_EXEC) {
            crazy_run_exec(buf, len);
        } else if (kind == CRAZY_KIND_USER) {
            crazy_run_user(channel, buf, len);
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(crazy_dispatch_drain_obj, crazy_dispatch_drain);

void sentai_crazy_request_drain(void) {
    /* Always attempt to schedule (no dedup) — see comment near the
     * removed g_crazy_drain_pending. If the MP scheduler queue is
     * full, mp_sched_schedule returns false and the next push retries
     * naturally. Discarding the return value is safe because:
     *   - the FIFO already has the message stashed
     *   - any subsequent push will re-attempt the schedule
     *   - if the trampoline is already in-flight it will drain us anyway */
    (void)mp_sched_schedule(MP_OBJ_FROM_PTR(&crazy_dispatch_drain_obj),
                            mp_const_none);
}

// sentai.crazy.on_message(callback) -> None
//   callback(channel: int, data: bytes) is invoked for every inbound
//   frame except `$`-prefix exec messages on CH_REPL.
//   Pass None to detach (frames are dropped silently).
static mp_obj_t mod_sentai_crazy_on_message(mp_obj_t cb_obj) {
    if (cb_obj == mp_const_none) {
        MP_STATE_VM(crazy_msg_handler) = MP_OBJ_NULL;
    } else {
        if (!mp_obj_is_callable(cb_obj)) {
            mp_raise_TypeError(MP_ERROR_TEXT("on_message: callable or None required"));
        }
        MP_STATE_VM(crazy_msg_handler) = cb_obj;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_crazy_on_message_obj, mod_sentai_crazy_on_message);

// sentai.crazy.link_send(channel, data) -> int
// Send a payload to the drone-side "sentai" deck driver over UART2,
// using channel multiplexing on the 0xAA wire format:
//   channel=0  REPL / text reply    (drone forwards over radio CRTP port 0x0E)
//   channel=1  optical-flow data    (drone injects into EKF via estimatorEnqueueFlow)
//   channel=2  reserved
//   channel=3  reserved
// Returns 0=ok, -1=crazy not initialized, -2=invalid args, -3=UART tx fail.
extern int sentai_crazy_link_send(int channel, const uint8_t* data, int len);
static mp_obj_t mod_sentai_crazy_link_send(mp_obj_t channel_obj, mp_obj_t data_obj) {
    int channel = mp_obj_get_int(channel_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(data_obj, &bi, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_crazy_link_send(
        channel, (const uint8_t*)bi.buf, (int)bi.len));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_crazy_link_send_obj, mod_sentai_crazy_link_send);

// ---- module table ----
static const mp_rom_map_elem_t sentai_crazy_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_crazy) },
    { MP_ROM_QSTR(MP_QSTR_init),          MP_ROM_PTR(&mod_sentai_crazy_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mod_sentai_crazy_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),         MP_ROM_PTR(&mod_sentai_crazy_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),            MP_ROM_PTR(&mod_sentai_crazy_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),         MP_ROM_PTR(&mod_sentai_crazy_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),       MP_ROM_PTR(&mod_sentai_crazy_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),          MP_ROM_PTR(&mod_sentai_crazy_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_motors),   MP_ROM_PTR(&mod_sentai_crazy_stop_motors_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),         MP_ROM_PTR(&mod_sentai_crazy_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_go_to),         MP_ROM_PTR(&mod_sentai_crazy_go_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_crtp),     MP_ROM_PTR(&mod_sentai_crazy_send_crtp_obj) },
    { MP_ROM_QSTR(MP_QSTR_ping),          MP_ROM_PTR(&mod_sentai_crazy_ping_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_fly),      MP_ROM_PTR(&mod_sentai_crazy_test_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly),           MP_ROM_PTR(&mod_sentai_crazy_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_attitude),      MP_ROM_PTR(&mod_sentai_crazy_attitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly_stop),      MP_ROM_PTR(&mod_sentai_crazy_fly_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_altitude),       MP_ROM_PTR(&mod_sentai_crazy_altitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_baro),           MP_ROM_PTR(&mod_sentai_crazy_baro_obj) },
    { MP_ROM_QSTR(MP_QSTR_battery),        MP_ROM_PTR(&mod_sentai_crazy_battery_obj) },
    { MP_ROM_QSTR(MP_QSTR_battery_pct),    MP_ROM_PTR(&mod_sentai_crazy_battery_pct_obj) },
    { MP_ROM_QSTR(MP_QSTR_temp),           MP_ROM_PTR(&mod_sentai_crazy_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_pressure),       MP_ROM_PTR(&mod_sentai_crazy_pressure_obj) },
    { MP_ROM_QSTR(MP_QSTR_telem),          MP_ROM_PTR(&mod_sentai_crazy_telem_obj) },
    { MP_ROM_QSTR(MP_QSTR_on_message),     MP_ROM_PTR(&mod_sentai_crazy_on_message_obj) },
    { MP_ROM_QSTR(MP_QSTR_link_send),      MP_ROM_PTR(&mod_sentai_crazy_link_send_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_crazy_globals, sentai_crazy_globals_table);
static const mp_obj_module_t sentai_crazy_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_crazy_globals,
};
