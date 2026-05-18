/* modsentai_crazy_sim.c — SIM-only MicroPython bindings for sentai.crazy.
 *
 * Companion to sim/sentai_crazy_sim.cc (CRTP-over-UDP transport).
 * Intentionally *not* a thin wrapper around examples/sentai_runtime/
 * modsentai_crazy.c — that file embeds ARM-only preconditions (UART
 * sharing with mesh/link) and CPX-only features (telem channel,
 * test_fly, fly_stop, on_message dispatch trampoline) that don't apply
 * when the transport is a dedicated UDP socket.  Keeping a SIM-flavour
 * binding here avoids ifdef explosion and keeps the surface small +
 * thesis-relevant: arm/disarm, takeoff/land/go_to/stop, hover,
 * send_crtp/recv_crtp, stats.
 *
 * This file is #include'd from sim/modsentai_sim.c (same pattern as
 * modsentai_objects.c / modsentai_places.c reuse from ARM tree).
 *
 * Python API:
 *   sentai.crazy.init([baudrate=0])    -> int  (0=ok, <0=err)
 *   sentai.crazy.stop()                -> int
 *   sentai.crazy.is_running()          -> int  (1=up, 0=down)
 *   sentai.crazy.debug(level)          -> None (0=off, 1=tx/rx, 2=hex)
 *   sentai.crazy.arm()                 -> int
 *   sentai.crazy.disarm()              -> int
 *   sentai.crazy.takeoff(h=0.5, dur=2.0, yaw=0, use_current_yaw=1, group=0)
 *   sentai.crazy.land(h=0.0, dur=2.0, ...)
 *   sentai.crazy.stop_motors(group=0)
 *   sentai.crazy.go_to(x, y, z, yaw=0, dur=3.0, relative=1, linear=0, group=0)
 *   sentai.crazy.hover(vx=0, vy=0, yaw_rate=0, z=0.5)
 *   sentai.crazy.send_crtp(port:int, channel:int, data:bytes)  -> int
 *   sentai.crazy.recv_crtp()           -> (port, ch, data:bytes) | None
 *   sentai.crazy.stats()               -> (tx, rx, dropped, running)
 */

/* Forward decls — implementations live in sim/sentai_crazy_sim.cc */
extern int   sentai_crazy_init(uint32_t baudrate);
extern int   sentai_crazy_stop(void);
extern int   sentai_crazy_is_running(void);
extern void  sentai_crazy_set_debug(int level);
extern int   sentai_crazy_arm(void);
extern int   sentai_crazy_disarm(void);
extern int   sentai_crazy_takeoff(float height, float duration,
                                   float yaw, int use_current_yaw,
                                   uint8_t group_mask);
extern int   sentai_crazy_land(float height, float duration,
                                float yaw, int use_current_yaw,
                                uint8_t group_mask);
extern int   sentai_crazy_stop_motors(uint8_t group_mask);
extern int   sentai_crazy_go_to(float x, float y, float z, float yaw,
                                 float duration, int relative, int linear,
                                 uint8_t group_mask);
extern int   sentai_crazy_hover(float vx, float vy, float yaw_rate, float z);
extern int   sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                                     const uint8_t* data, int len);
extern int   sentai_crazy_recv_pop(uint8_t* port, uint8_t* ch,
                                    uint8_t* data, int max_len, int* out_len);
extern void  sentai_crazy_get_stats(uint32_t out[4]);

/* ===================== Bindings ===================== */

static mp_obj_t sim_crazy_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baud = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_init(baud));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_init_obj, 0, 1, sim_crazy_init);

static mp_obj_t sim_crazy_stop(void) {
    return mp_obj_new_int(sentai_crazy_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_stop_obj, sim_crazy_stop);

static mp_obj_t sim_crazy_is_running(void) {
    return mp_obj_new_int(sentai_crazy_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_is_running_obj, sim_crazy_is_running);

static mp_obj_t sim_crazy_debug(mp_obj_t level_obj) {
    sentai_crazy_set_debug(mp_obj_get_int(level_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sim_crazy_debug_obj, sim_crazy_debug);

static mp_obj_t sim_crazy_arm(void) {
    return mp_obj_new_int(sentai_crazy_arm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_arm_obj, sim_crazy_arm);

static mp_obj_t sim_crazy_disarm(void) {
    return mp_obj_new_int(sentai_crazy_disarm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_disarm_obj, sim_crazy_disarm);

static mp_obj_t sim_crazy_takeoff(size_t n_args, const mp_obj_t *args) {
    float h    = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.5f;
    float dur  = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw  = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int   uc   = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t g  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_takeoff(h, dur, yaw, uc, g));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_takeoff_obj, 0, 5, sim_crazy_takeoff);

static mp_obj_t sim_crazy_land(size_t n_args, const mp_obj_t *args) {
    float h    = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float dur  = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw  = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int   uc   = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t g  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_land(h, dur, yaw, uc, g));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_land_obj, 0, 5, sim_crazy_land);

static mp_obj_t sim_crazy_stop_motors(size_t n_args, const mp_obj_t *args) {
    uint8_t g = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_stop_motors(g));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_stop_motors_obj, 0, 1, sim_crazy_stop_motors);

extern int sentai_crazy_hl_stop(uint8_t group_mask);
static mp_obj_t sim_crazy_hl_stop(size_t n_args, const mp_obj_t *args) {
    uint8_t g = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_hl_stop(g));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_hl_stop_obj, 0, 1, sim_crazy_hl_stop);

static mp_obj_t sim_crazy_go_to(size_t n_args, const mp_obj_t *args) {
    float x   = mp_obj_get_float(args[0]);
    float y   = mp_obj_get_float(args[1]);
    float z   = mp_obj_get_float(args[2]);
    float yaw = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.0f;
    float dur = (n_args > 4) ? mp_obj_get_float(args[4]) : 3.0f;
    int   rel = (n_args > 5) ? mp_obj_get_int(args[5])   : 1;
    int   lin = (n_args > 6) ? mp_obj_get_int(args[6])   : 0;
    uint8_t g = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 0;
    return mp_obj_new_int(sentai_crazy_go_to(x, y, z, yaw, dur, rel, lin, g));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_go_to_obj, 3, 8, sim_crazy_go_to);

static mp_obj_t sim_crazy_hover(size_t n_args, const mp_obj_t *args) {
    float vx = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float vy = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0f;
    float yr = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    float z  = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.5f;
    return mp_obj_new_int(sentai_crazy_hover(vx, vy, yr, z));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_hover_obj, 0, 4, sim_crazy_hover);

static mp_obj_t sim_crazy_send_crtp(mp_obj_t port_obj, mp_obj_t ch_obj, mp_obj_t data_obj) {
    uint8_t port = (uint8_t)mp_obj_get_int(port_obj);
    uint8_t ch   = (uint8_t)mp_obj_get_int(ch_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(data_obj, &bi, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_crazy_send_crtp(
        port, ch, (const uint8_t*)bi.buf, (int)bi.len));
}
static MP_DEFINE_CONST_FUN_OBJ_3(sim_crazy_send_crtp_obj, sim_crazy_send_crtp);

/* recv_crtp() -> (port, ch, data:bytes) | None
 * Non-blocking — pops one packet from the RX FIFO if available. */
static mp_obj_t sim_crazy_recv_crtp(void) {
    uint8_t port = 0, ch = 0;
    uint8_t buf[30];
    int     out_len = 0;
    if (!sentai_crazy_recv_pop(&port, &ch, buf, sizeof(buf), &out_len)) {
        return mp_const_none;
    }
    mp_obj_t tup[3] = {
        mp_obj_new_int(port),
        mp_obj_new_int(ch),
        mp_obj_new_bytes(buf, (size_t)out_len),
    };
    return mp_obj_new_tuple(3, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_recv_crtp_obj, sim_crazy_recv_crtp);

/* stats() -> (tx_count, rx_count, dropped, running) */
static mp_obj_t sim_crazy_stats(void) {
    uint32_t s[4];
    sentai_crazy_get_stats(s);
    mp_obj_t tup[4] = {
        mp_obj_new_int_from_uint(s[0]),
        mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]),
        mp_obj_new_int_from_uint(s[3]),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_stats_obj, sim_crazy_stats);

/* ========== sentai.crazy.pose_* — Task #44 (CRTP LOG in C) ============
 * Shared impl with ARM lives in examples/sentai_runtime/sentai_crazy_log.cc,
 * added to sentai_sim CMakeLists below.
 */
extern int  sentai_crazy_pose_subscribe(int period_ms);
extern int  sentai_crazy_pose(float* x, float* y, float* z, float* yaw);
extern int  sentai_crazy_pose_stop(void);
extern int  sentai_crazy_pose_ready(void);
extern void sentai_crazy_log_stats(uint32_t out[4]);

static mp_obj_t sim_crazy_pose_subscribe(size_t n_args, const mp_obj_t *args) {
    int period_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 100;
    return mp_obj_new_int(sentai_crazy_pose_subscribe(period_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_crazy_pose_subscribe_obj, 0, 1, sim_crazy_pose_subscribe);

static mp_obj_t sim_crazy_pose(void) {
    float x=0, y=0, z=0, yaw=0;
    if (sentai_crazy_pose(&x, &y, &z, &yaw) != 0) return mp_const_none;
    mp_obj_t tup[4] = {
        mp_obj_new_float(x), mp_obj_new_float(y),
        mp_obj_new_float(z), mp_obj_new_float(yaw),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_pose_obj, sim_crazy_pose);

static mp_obj_t sim_crazy_pose_stop(void) {
    return mp_obj_new_int(sentai_crazy_pose_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_pose_stop_obj, sim_crazy_pose_stop);

static mp_obj_t sim_crazy_pose_ready(void) {
    return mp_obj_new_int(sentai_crazy_pose_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_pose_ready_obj, sim_crazy_pose_ready);

static mp_obj_t sim_crazy_log_stats(void) {
    uint32_t s[4];
    sentai_crazy_log_stats(s);
    mp_obj_t tup[4] = {
        mp_obj_new_int_from_uint(s[0]), mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]), mp_obj_new_int_from_uint(s[3]),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_crazy_log_stats_obj, sim_crazy_log_stats);

/* ===================== Module table ===================== */
static const mp_rom_map_elem_t sentai_crazy_sim_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_crazy) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&sim_crazy_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&sim_crazy_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running),  MP_ROM_PTR(&sim_crazy_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),       MP_ROM_PTR(&sim_crazy_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),         MP_ROM_PTR(&sim_crazy_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),      MP_ROM_PTR(&sim_crazy_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),     MP_ROM_PTR(&sim_crazy_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),        MP_ROM_PTR(&sim_crazy_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_motors), MP_ROM_PTR(&sim_crazy_stop_motors_obj) },
    { MP_ROM_QSTR(MP_QSTR_hl_stop),     MP_ROM_PTR(&sim_crazy_hl_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_go_to),       MP_ROM_PTR(&sim_crazy_go_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),       MP_ROM_PTR(&sim_crazy_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_crtp),   MP_ROM_PTR(&sim_crazy_send_crtp_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv_crtp),   MP_ROM_PTR(&sim_crazy_recv_crtp_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),       MP_ROM_PTR(&sim_crazy_stats_obj) },
    /* Task #44 — pose_* (CRTP LOG in C) */
    { MP_ROM_QSTR(MP_QSTR_pose_subscribe), MP_ROM_PTR(&sim_crazy_pose_subscribe_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose),           MP_ROM_PTR(&sim_crazy_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose_stop),      MP_ROM_PTR(&sim_crazy_pose_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose_ready),     MP_ROM_PTR(&sim_crazy_pose_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_stats),      MP_ROM_PTR(&sim_crazy_log_stats_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_crazy_sim_globals, sentai_crazy_sim_globals_table);
static const mp_obj_module_t sentai_crazy_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_crazy_sim_globals,
};
