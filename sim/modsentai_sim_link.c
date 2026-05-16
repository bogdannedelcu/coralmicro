/* modsentai_sim_link.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== top-level sentai module ===== */
/* ===== sentai.link — SIM MAVLink bridge (Phase 6) ============================
 * Slim mirror of examples/sentai_runtime/modsentai_link.c.  Same Python
 * surface as ARM (init/stop/debug/heartbeat/send/stats); transport is
 * UDP via sentai_uart_serial_udp.c instead of LPUART.
 */
extern int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid);
extern int sentai_link_stop(void);
extern void sentai_link_set_debug(int level);
extern int sentai_link_send_heartbeat(uint8_t type);
extern int sentai_link_send_statustext(uint8_t severity, const char* text);
extern void sentai_link_get_stats(uint32_t out[8]);
extern int sentai_link_cmd_arm(int do_arm);
extern int sentai_link_cmd_takeoff(float altitude_m);
extern int sentai_link_cmd_land(void);
extern int sentai_link_cmd_set_mode(uint8_t main_mode, uint8_t sub_mode);
extern int sentai_link_flow_forward(int enable);
extern int sentai_link_flow_set_distance(float dist_m);
extern int sentai_link_send_flow(float dx_rad, float dy_rad,
                                  uint32_t dt_us, uint8_t quality,
                                  float distance_m);

static mp_obj_t sim_link_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 57600;
    uint8_t sysid  = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 1;
    uint8_t compid = (n_args > 2) ? (uint8_t)mp_obj_get_int(args[2]) : 191;
    return mp_obj_new_int(sentai_link_init(baudrate, sysid, compid));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_init_obj, 0, 3, sim_link_init);

static mp_obj_t sim_link_stop(void) { return mp_obj_new_int(sentai_link_stop()); }
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_stop_obj, sim_link_stop);

static mp_obj_t sim_link_debug(mp_obj_t lvl) {
    sentai_link_set_debug(mp_obj_get_int(lvl));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sim_link_debug_obj, sim_link_debug);

static mp_obj_t sim_link_heartbeat(size_t n_args, const mp_obj_t *args) {
    uint8_t type = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 18;
    return mp_obj_new_int(sentai_link_send_heartbeat(type));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_heartbeat_obj, 0, 1, sim_link_heartbeat);

static mp_obj_t sim_link_send(size_t n_args, const mp_obj_t *args) {
    const char* text = mp_obj_str_get_str(args[0]);
    uint8_t severity = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 6;
    return mp_obj_new_int(sentai_link_send_statustext(severity, text));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_send_obj, 1, 2, sim_link_send);

static mp_obj_t sim_link_stats(void) {
    uint32_t s[9] = {0};
    sentai_link_get_stats(s);
    mp_obj_t items[9] = {
        mp_obj_new_int_from_uint(s[0]), mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]), mp_obj_new_int_from_uint(s[3]),
        mp_obj_new_int_from_uint(s[4]), mp_obj_new_int_from_uint(s[5]),
        mp_obj_new_int_from_uint(s[6]), mp_obj_new_int_from_uint(s[7]),
        mp_obj_new_int_from_uint(s[8]),
    };
    return mp_obj_new_tuple(9, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_stats_obj, sim_link_stats);

/* sentai.link.flow(enable[, distance_m])
 *   - enable=1: start C-side forwarder task (reads g_flow snapshot,
 *     converts mgrid→rad, sends OPTICAL_FLOW_RAD; Python NOT in loop).
 *   - enable=0: stop the task.
 *   - optional distance_m sets the flow message distance field (default
 *     1.0 m).  Reuses existing `flow` QSTR (no regen needed).
 * Returns 1 on success, 0 on failure (e.g., link not init).
 */
static mp_obj_t sim_link_flow(size_t n_args, const mp_obj_t *args) {
    int en = mp_obj_get_int(args[0]);
    if (n_args > 1) {
        sentai_link_flow_set_distance(mp_obj_get_float(args[1]));
    }
    return mp_obj_new_int(sentai_link_flow_forward(en));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_flow_obj, 1, 2, sim_link_flow);

static mp_obj_t sim_link_arm(size_t n_args, const mp_obj_t *args) {
    int do_arm = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(sentai_link_cmd_arm(do_arm));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_arm_obj, 0, 1, sim_link_arm);

static mp_obj_t sim_link_takeoff(size_t n_args, const mp_obj_t *args) {
    float alt = (n_args > 0) ? mp_obj_get_float(args[0]) : 1.0f;
    return mp_obj_new_int(sentai_link_cmd_takeoff(alt));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_takeoff_obj, 0, 1, sim_link_takeoff);

static mp_obj_t sim_link_land(void) {
    return mp_obj_new_int(sentai_link_cmd_land());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_land_obj, sim_link_land);

/* sentai.link.send_flow(dx_rad, dy_rad, dt_us, quality=200, distance_m=1.0)
 * Build + send MAVLINK_MSG_ID_OPTICAL_FLOW_RAD to PX4.  Used by hover
 * scripts to feed sentai.flow into PX4 EKF2 (set EKF2_AID_MASK to
 * include flow). */
static mp_obj_t sim_link_send_flow(size_t n_args, const mp_obj_t *args) {
    float dx = mp_obj_get_float(args[0]);
    float dy = mp_obj_get_float(args[1]);
    uint32_t dt = (uint32_t)mp_obj_get_int(args[2]);
    uint8_t q = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 200;
    float dist = (n_args > 4) ? mp_obj_get_float(args[4]) : 1.0f;
    return mp_obj_new_int(sentai_link_send_flow(dx, dy, dt, q, dist));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_send_flow_obj, 3, 5, sim_link_send_flow);

/* set_mode: dropped from binding for MVP (QSTR `set_mode` not in pool).
 * MP_REGISTER_MODULE regen needed to expose it.  In the meantime
 * sentai.link.send_command_long(176, ...) (also not exposed yet) or
 * direct python pymavlink-from-host is the workaround. */

static const mp_rom_map_elem_t sentai_link_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_link) },
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&sim_link_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&sim_link_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),     MP_ROM_PTR(&sim_link_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_heartbeat), MP_ROM_PTR(&sim_link_heartbeat_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),      MP_ROM_PTR(&sim_link_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),     MP_ROM_PTR(&sim_link_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),       MP_ROM_PTR(&sim_link_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),   MP_ROM_PTR(&sim_link_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),      MP_ROM_PTR(&sim_link_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_flow), MP_ROM_PTR(&sim_link_send_flow_obj) },
    { MP_ROM_QSTR(MP_QSTR_flow),      MP_ROM_PTR(&sim_link_flow_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_link_globals, sentai_link_globals_table);
static const mp_obj_module_t sentai_link_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_link_globals,
};
