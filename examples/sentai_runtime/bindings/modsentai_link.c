// ============== sentai.link — MAVLink telemetry bridge ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.link.init(baudrate=57600, sysid=1, compid=191) -> int
static mp_obj_t mod_sentai_link_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 57600;
    uint8_t sysid = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 1;
    uint8_t compid = (n_args > 2) ? (uint8_t)mp_obj_get_int(args[2]) : 191;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    if (sentai_mesh_is_running()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("mesh is running: call sentai.mesh.stop() first"));
    }
    return mp_obj_new_int(sentai_link_init(baudrate, sysid, compid));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_init_obj, 0, 3, mod_sentai_link_init);

// sentai.link.stop() -> int
static mp_obj_t mod_sentai_link_stop(void) {
    return mp_obj_new_int(sentai_link_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_link_stop_obj, mod_sentai_link_stop);

// sentai.link.debug(level) -> None
// 0=off, 1=TX/RX summary on console, 2=+hex dump
extern void sentai_link_set_debug(int level);
static mp_obj_t mod_sentai_link_debug(mp_obj_t level_obj) {
    sentai_link_set_debug(mp_obj_get_int(level_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_link_debug_obj, mod_sentai_link_debug);

// sentai.link.heartbeat(type=18) -> int
// type 18 = MAV_TYPE_ONBOARD_CONTROLLER
static mp_obj_t mod_sentai_link_heartbeat(size_t n_args, const mp_obj_t *args) {
    uint8_t type = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 18;
    return mp_obj_new_int(sentai_link_send_heartbeat(type));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_heartbeat_obj, 0, 1, mod_sentai_link_heartbeat);

// sentai.link.send(text, severity=6) -> int
static mp_obj_t mod_sentai_link_send(size_t n_args, const mp_obj_t *args) {
    const char* text = mp_obj_str_get_str(args[0]);
    uint8_t severity = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 6;
    return mp_obj_new_int(sentai_link_send_statustext(severity, text));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_obj, 1, 2, mod_sentai_link_send);

// sentai.link.send_detection(...) -> int
// Args: sensor_id, track_id, alarm_type, timestamp, seq,
//       x, y, w, h, conf, class_id,
//       gx_cm=0, gy_cm=0, width_cm=0,
//       severity=6
static mp_obj_t mod_sentai_link_send_detection(size_t n_args, const mp_obj_t *args) {
    uint32_t sensor_id = (uint32_t)mp_obj_get_int(args[0]);
    uint32_t track_id  = (uint32_t)mp_obj_get_int(args[1]);
    uint32_t alarm_type = (uint32_t)mp_obj_get_int(args[2]);
    uint32_t timestamp = (uint32_t)mp_obj_get_int(args[3]);
    uint32_t seq       = (uint32_t)mp_obj_get_int(args[4]);
    uint8_t x = (uint8_t)mp_obj_get_int(args[5]);
    uint8_t y = (uint8_t)mp_obj_get_int(args[6]);
    uint8_t w = (uint8_t)mp_obj_get_int(args[7]);
    uint8_t h = (uint8_t)mp_obj_get_int(args[8]);
    uint32_t conf = (uint32_t)mp_obj_get_int(args[9]);
    uint32_t class_id = (uint32_t)mp_obj_get_int(args[10]);
    int32_t gx_cm = 0;
    int32_t gy_cm = 0;
    int16_t width_cm = 0;
    uint8_t severity = 6;
    if (n_args > 11) gx_cm = (int32_t)mp_obj_get_int(args[11]);
    if (n_args > 12) gy_cm = (int32_t)mp_obj_get_int(args[12]);
    if (n_args > 13) width_cm = (int16_t)mp_obj_get_int(args[13]);
    if (n_args > 14) severity = (uint8_t)mp_obj_get_int(args[14]);
    return mp_obj_new_int(sentai_link_send_vision(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, class_id,
        gx_cm, gy_cm, width_cm, severity));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_detection_obj, 11, 15, mod_sentai_link_send_detection);

// sentai.link.send_update(...) -> int
// Args: sensor_id, track_id, alarm_type, timestamp, seq,
//       x, y, w, h, conf, age,
//       gx_cm=0, gy_cm=0,
//       severity=6
static mp_obj_t mod_sentai_link_send_update(size_t n_args, const mp_obj_t *args) {
    uint32_t sensor_id = (uint32_t)mp_obj_get_int(args[0]);
    uint32_t track_id  = (uint32_t)mp_obj_get_int(args[1]);
    uint32_t alarm_type = (uint32_t)mp_obj_get_int(args[2]);
    uint32_t timestamp = (uint32_t)mp_obj_get_int(args[3]);
    uint32_t seq       = (uint32_t)mp_obj_get_int(args[4]);
    uint8_t x = (uint8_t)mp_obj_get_int(args[5]);
    uint8_t y = (uint8_t)mp_obj_get_int(args[6]);
    uint8_t w = (uint8_t)mp_obj_get_int(args[7]);
    uint8_t h = (uint8_t)mp_obj_get_int(args[8]);
    uint32_t conf = (uint32_t)mp_obj_get_int(args[9]);
    uint32_t age  = (uint32_t)mp_obj_get_int(args[10]);
    int32_t gx_cm = (n_args > 11) ? (int32_t)mp_obj_get_int(args[11]) : 0;
    int32_t gy_cm = (n_args > 12) ? (int32_t)mp_obj_get_int(args[12]) : 0;
    uint8_t severity = (n_args > 13) ? (uint8_t)mp_obj_get_int(args[13]) : 6;
    return mp_obj_new_int(sentai_link_send_vision_update(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, age,
        gx_cm, gy_cm, severity));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_update_obj, 11, 14, mod_sentai_link_send_update);

// sentai.link.send_delete(...) -> int
// Args: sensor_id, track_id, alarm_type, timestamp, seq,
//       reason, age, total_hits,
//       last_gx_cm=0, last_gy_cm=0,
//       severity=6
static mp_obj_t mod_sentai_link_send_delete(size_t n_args, const mp_obj_t *args) {
    uint32_t sensor_id = (uint32_t)mp_obj_get_int(args[0]);
    uint32_t track_id  = (uint32_t)mp_obj_get_int(args[1]);
    uint32_t alarm_type = (uint32_t)mp_obj_get_int(args[2]);
    uint32_t timestamp = (uint32_t)mp_obj_get_int(args[3]);
    uint32_t seq       = (uint32_t)mp_obj_get_int(args[4]);
    uint32_t reason    = (uint32_t)mp_obj_get_int(args[5]);
    uint32_t age       = (uint32_t)mp_obj_get_int(args[6]);
    uint32_t total_hits = (uint32_t)mp_obj_get_int(args[7]);
    int32_t last_gx = (n_args > 8) ? (int32_t)mp_obj_get_int(args[8]) : 0;
    int32_t last_gy = (n_args > 9) ? (int32_t)mp_obj_get_int(args[9]) : 0;
    uint8_t severity = (n_args > 10) ? (uint8_t)mp_obj_get_int(args[10]) : 6;
    return mp_obj_new_int(sentai_link_send_vision_delete(
        sensor_id, track_id, alarm_type, timestamp, seq,
        reason, age, total_hits,
        last_gx, last_gy, severity));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_delete_obj, 8, 11, mod_sentai_link_send_delete);

// sentai.link.command(target_sys, target_comp, cmd, conf, p1..p7) -> int
// params are native floats
static mp_obj_t mod_sentai_link_command(size_t n_args, const mp_obj_t *args) {
    uint8_t tsys = (uint8_t)mp_obj_get_int(args[0]);
    uint8_t tcomp = (uint8_t)mp_obj_get_int(args[1]);
    uint16_t cmd = (uint16_t)mp_obj_get_int(args[2]);
    uint8_t conf = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 0;
    float p1 = (n_args > 4) ? mp_obj_get_float(args[4]) : 0.0f;
    float p2 = (n_args > 5) ? mp_obj_get_float(args[5]) : 0.0f;
    float p3 = (n_args > 6) ? mp_obj_get_float(args[6]) : 0.0f;
    float p4 = (n_args > 7) ? mp_obj_get_float(args[7]) : 0.0f;
    float p5 = (n_args > 8) ? mp_obj_get_float(args[8]) : 0.0f;
    float p6 = (n_args > 9) ? mp_obj_get_float(args[9]) : 0.0f;
    float p7 = (n_args > 10) ? mp_obj_get_float(args[10]) : 0.0f;
    return mp_obj_new_int(sentai_link_send_command_long(
        tsys, tcomp, cmd, conf, p1, p2, p3, p4, p5, p6, p7));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_command_obj, 3, 11, mod_sentai_link_command);

// sentai.link.obstacle_distance(distances72,
//                               increment_deg=5, min_cm=20, max_cm=800,
//                               increment_f_deg=0.0, angle_offset_deg=0.0,
//                               sensor_type=0, frame=12) -> int
static mp_obj_t mod_sentai_link_obstacle_distance(size_t n_args, const mp_obj_t *args) {
    mp_uint_t n = 0;
    mp_obj_t *items = NULL;
    mp_obj_get_array(args[0], &n, &items);
    if (n != 72) {
        mp_raise_ValueError(MP_ERROR_TEXT("distances must have 72 elements"));
    }

    uint16_t distances[72];
    for (mp_uint_t i = 0; i < 72; ++i) {
        int v = mp_obj_get_int(items[i]);
        if (v < 0) v = 0;
        if (v > 65535) v = 65535;
        distances[i] = (uint16_t)v;
    }

    uint8_t increment_deg = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 5;
    uint16_t min_cm = (n_args > 2) ? (uint16_t)mp_obj_get_int(args[2]) : 20;
    uint16_t max_cm = (n_args > 3) ? (uint16_t)mp_obj_get_int(args[3]) : 800;
    float increment_f_deg = (n_args > 4) ? mp_obj_get_float(args[4]) : 0.0f;
    float angle_offset_deg = (n_args > 5) ? mp_obj_get_float(args[5]) : 0.0f;
    uint8_t sensor_type = (n_args > 6) ? (uint8_t)mp_obj_get_int(args[6]) : 0;
    uint8_t frame = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 12;

    return mp_obj_new_int(sentai_link_send_obstacle_distance(
        distances,
        increment_deg,
        min_cm,
        max_cm,
        increment_f_deg,
        angle_offset_deg,
        sensor_type,
        frame));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_obstacle_distance_obj, 1, 8,
                                           mod_sentai_link_obstacle_distance);

// sentai.link.obstacles_from_tracker(max_cm=800, min_cm=20,
//                                    h_fov_deg=360.0, increment_deg=5,
//                                    include_lost=0, angle_offset_deg=0.0,
//                                    sensor_type=0, frame=12) -> int
static mp_obj_t mod_sentai_link_obstacles_from_tracker(size_t n_args, const mp_obj_t *args) {
    uint16_t max_cm = (n_args > 0) ? (uint16_t)mp_obj_get_int(args[0]) : 800;
    uint16_t min_cm = (n_args > 1) ? (uint16_t)mp_obj_get_int(args[1]) : 20;
    float h_fov_deg = (n_args > 2) ? mp_obj_get_float(args[2]) : 360.0f;
    uint8_t increment_deg = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 5;
    uint8_t include_lost = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    float angle_offset_deg = (n_args > 5) ? mp_obj_get_float(args[5]) : 0.0f;
    uint8_t sensor_type = (n_args > 6) ? (uint8_t)mp_obj_get_int(args[6]) : 0;
    uint8_t frame = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 12;

    return mp_obj_new_int(sentai_link_send_obstacles_from_tracker(
        max_cm,
        min_cm,
        h_fov_deg,
        increment_deg,
        include_lost,
        angle_offset_deg,
        sensor_type,
        frame));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_obstacles_from_tracker_obj, 0, 8,
                                           mod_sentai_link_obstacles_from_tracker);

// sentai.link.obstacles_from_points(points,
//                                   radii=None,
//                                   max_cm=800, min_cm=20,
//                                   increment_deg=5, angle_offset_deg=0.0,
//                                   sensor_type=0, frame=12) -> int
// points can be [(x,y), ...] or flat [x0,y0,x1,y1,...]
static mp_obj_t mod_sentai_link_obstacles_from_points(size_t n_args, const mp_obj_t *args) {
    mp_uint_t p_len = 0;
    mp_obj_t *p_items = NULL;
    mp_obj_get_array(args[0], &p_len, &p_items);

    int count = 0;
    int nested_points = 0;
    if (p_len > 0 && (mp_obj_is_type(p_items[0], &mp_type_tuple) ||
                      mp_obj_is_type(p_items[0], &mp_type_list))) {
        nested_points = 1;
        count = (int)p_len;
    } else {
        if ((p_len % 2) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("flat points length must be even"));
        }
        count = (int)(p_len / 2);
    }

    int32_t *points_xy = NULL;
    uint16_t *radii = NULL;
    if (count > 0) {
        points_xy = m_new(int32_t, count * 2);
        if (nested_points) {
            for (int i = 0; i < count; ++i) {
                mp_uint_t tlen = 0;
                mp_obj_t *titems = NULL;
                mp_obj_get_array(p_items[i], &tlen, &titems);
                if (tlen < 2) {
                    m_del(int32_t, points_xy, count * 2);
                    mp_raise_ValueError(MP_ERROR_TEXT("each point must have at least 2 elements"));
                }
                points_xy[i * 2 + 0] = (int32_t)mp_obj_get_int(titems[0]);
                points_xy[i * 2 + 1] = (int32_t)mp_obj_get_int(titems[1]);
            }
        } else {
            for (int i = 0; i < count; ++i) {
                points_xy[i * 2 + 0] = (int32_t)mp_obj_get_int(p_items[i * 2 + 0]);
                points_xy[i * 2 + 1] = (int32_t)mp_obj_get_int(p_items[i * 2 + 1]);
            }
        }
    }

    if (n_args > 1 && args[1] != mp_const_none) {
        mp_uint_t r_len = 0;
        mp_obj_t *r_items = NULL;
        mp_obj_get_array(args[1], &r_len, &r_items);
        if ((int)r_len != count) {
            if (points_xy) m_del(int32_t, points_xy, count * 2);
            mp_raise_ValueError(MP_ERROR_TEXT("radii length must match points count"));
        }
        if (count > 0) {
            radii = m_new(uint16_t, count);
            for (int i = 0; i < count; ++i) {
                int v = mp_obj_get_int(r_items[i]);
                if (v < 0) v = 0;
                if (v > 65535) v = 65535;
                radii[i] = (uint16_t)v;
            }
        }
    }

    uint16_t max_cm = (n_args > 2) ? (uint16_t)mp_obj_get_int(args[2]) : 800;
    uint16_t min_cm = (n_args > 3) ? (uint16_t)mp_obj_get_int(args[3]) : 20;
    uint8_t increment_deg = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 5;
    float angle_offset_deg = (n_args > 5) ? mp_obj_get_float(args[5]) : 0.0f;
    uint8_t sensor_type = (n_args > 6) ? (uint8_t)mp_obj_get_int(args[6]) : 0;
    uint8_t frame = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 12;

    int32_t empty_points[2] = {0, 0};
    int rc = sentai_link_send_obstacles_from_points(
        (count > 0) ? points_xy : empty_points,
        radii,
        count,
        max_cm,
        min_cm,
        increment_deg,
        angle_offset_deg,
        sensor_type,
        frame);

    if (radii) m_del(uint16_t, radii, count);
    if (points_xy) m_del(int32_t, points_xy, count * 2);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_obstacles_from_points_obj, 1, 8,
                                           mod_sentai_link_obstacles_from_points);

// sentai.link.available() -> int
static mp_obj_t mod_sentai_link_available(void) {
    return mp_obj_new_int(sentai_link_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_link_available_obj, mod_sentai_link_available);

// sentai.link.receive(timeout_ms=0) -> dict or None
// Returns dict: {msgid, sysid, compid, seq, len, ...}
// For msgid 32 (LOCAL_POSITION_NED): + time, x, y, z, vx, vy, vz (mm / mm/s)
// For msgid 33 (GLOBAL_POSITION_INT): + time, lat, lon, alt, rel_alt, vx, vy, vz, hdg
static mp_obj_t mod_sentai_link_receive(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 0;
    link_rx_msg_t rx;
    int got;
    if (timeout_ms == 0)
        got = sentai_link_receive(&rx);
    else
        got = sentai_link_receive_wait(&rx, timeout_ms);
    if (!got) return mp_const_none;
    uint32_t mid = sentai_link_rx_msgid(&rx);
    mp_obj_dict_t *d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_msgid), mp_obj_new_int_from_uint(mid));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sysid), mp_obj_new_int(sentai_link_rx_sysid(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_compid), mp_obj_new_int(sentai_link_rx_compid(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_seq), mp_obj_new_int(sentai_link_rx_seq(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_len), mp_obj_new_int(sentai_link_rx_len(&rx)));
    if (mid == 32) {
        // LOCAL_POSITION_NED — native floats (metres, m/s)
        uint32_t t; float x, y, z, vx, vy, vz;
        sentai_link_rx_local_pos(&rx, &t, &x, &y, &z, &vx, &vy, &vz);
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_time), mp_obj_new_int_from_uint(t));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_float(x));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_float(y));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_z), mp_obj_new_float(z));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vx), mp_obj_new_float(vx));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vy), mp_obj_new_float(vy));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vz), mp_obj_new_float(vz));
    } else if (mid == 33) {
        // GLOBAL_POSITION_INT — already integer (degE7, mm, cm/s, cdeg)
        uint32_t t; int32_t lat, lon, alt, rel_alt;
        int16_t vx, vy, vz; uint16_t hdg;
        sentai_link_rx_global_pos(&rx, &t, &lat, &lon, &alt, &rel_alt,
                                  &vx, &vy, &vz, &hdg);
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_time), mp_obj_new_int_from_uint(t));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_lat), mp_obj_new_int(lat));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_lon), mp_obj_new_int(lon));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_alt), mp_obj_new_int(alt));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rel_alt), mp_obj_new_int(rel_alt));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vx), mp_obj_new_int(vx));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vy), mp_obj_new_int(vy));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_vz), mp_obj_new_int(vz));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_hdg), mp_obj_new_int(hdg));
    }
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_receive_obj, 0, 1, mod_sentai_link_receive);

// Optional backend helpers.  SIM implements these for PX4 SITL; ARM may
// leave some of them absent until the transport owns equivalent behavior.
extern void sentai_link_get_stats(uint32_t out[9]) __attribute__((weak));
extern int sentai_link_cmd_arm(int do_arm) __attribute__((weak));
extern int sentai_link_cmd_takeoff(float altitude_m) __attribute__((weak));
extern int sentai_link_cmd_land(void) __attribute__((weak));
extern int sentai_link_send_flow(float dx_rad, float dy_rad, uint32_t dt_us,
                                 uint8_t quality, float distance_m)
    __attribute__((weak));
extern int sentai_link_flow_forward(int enable) __attribute__((weak));
extern int sentai_link_flow_set_distance(float dist_m) __attribute__((weak));

// sentai.link.stats() -> tuple
// (tx_heartbeat, tx_statustext, rx_total, rx_heartbeat, rx_other,
//  rx_parse_err, last_peer_sysid, last_peer_compid, tx_flow)
static mp_obj_t mod_sentai_link_stats(void) {
    uint32_t s[9] = {0};
    if (sentai_link_get_stats) {
        sentai_link_get_stats(s);
    }
    mp_obj_t items[9] = {
        mp_obj_new_int_from_uint(s[0]), mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]), mp_obj_new_int_from_uint(s[3]),
        mp_obj_new_int_from_uint(s[4]), mp_obj_new_int_from_uint(s[5]),
        mp_obj_new_int_from_uint(s[6]), mp_obj_new_int_from_uint(s[7]),
        mp_obj_new_int_from_uint(s[8]),
    };
    return mp_obj_new_tuple(9, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_link_stats_obj,
                                  mod_sentai_link_stats);

static mp_obj_t mod_sentai_link_arm(size_t n_args, const mp_obj_t *args) {
    int do_arm = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(sentai_link_cmd_arm ?
        sentai_link_cmd_arm(do_arm) : -99);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_arm_obj, 0, 1,
                                            mod_sentai_link_arm);

static mp_obj_t mod_sentai_link_takeoff(size_t n_args,
                                         const mp_obj_t *args) {
    float alt = (n_args > 0) ? mp_obj_get_float(args[0]) : 1.0f;
    return mp_obj_new_int(sentai_link_cmd_takeoff ?
        sentai_link_cmd_takeoff(alt) : -99);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_takeoff_obj, 0, 1,
                                            mod_sentai_link_takeoff);

static mp_obj_t mod_sentai_link_land(void) {
    return mp_obj_new_int(sentai_link_cmd_land ?
        sentai_link_cmd_land() : -99);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_link_land_obj,
                                  mod_sentai_link_land);

static mp_obj_t mod_sentai_link_send_flow(size_t n_args,
                                           const mp_obj_t *args) {
    float dx = mp_obj_get_float(args[0]);
    float dy = mp_obj_get_float(args[1]);
    uint32_t dt = (uint32_t)mp_obj_get_int(args[2]);
    uint8_t q = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 200;
    float dist = (n_args > 4) ? mp_obj_get_float(args[4]) : 1.0f;
    return mp_obj_new_int(sentai_link_send_flow ?
        sentai_link_send_flow(dx, dy, dt, q, dist) : -99);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_flow_obj, 3, 5,
                                            mod_sentai_link_send_flow);

// sentai.link.flow(enable[, distance_m]) — enable/disable backend flow
// forwarder when the backend supports it.
static mp_obj_t mod_sentai_link_flow(size_t n_args, const mp_obj_t *args) {
    int en = mp_obj_get_int(args[0]);
    if (n_args > 1 && sentai_link_flow_set_distance) {
        sentai_link_flow_set_distance(mp_obj_get_float(args[1]));
    }
    return mp_obj_new_int(sentai_link_flow_forward ?
        sentai_link_flow_forward(en) : -99);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_flow_obj, 1, 2,
                                            mod_sentai_link_flow);

// ---- module table ----
static const mp_rom_map_elem_t sentai_link_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_link) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&mod_sentai_link_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),              MP_ROM_PTR(&mod_sentai_link_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),             MP_ROM_PTR(&mod_sentai_link_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_heartbeat),         MP_ROM_PTR(&mod_sentai_link_heartbeat_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),              MP_ROM_PTR(&mod_sentai_link_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_detection),    MP_ROM_PTR(&mod_sentai_link_send_detection_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_update),       MP_ROM_PTR(&mod_sentai_link_send_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_delete),       MP_ROM_PTR(&mod_sentai_link_send_delete_obj) },
    { MP_ROM_QSTR(MP_QSTR_command),           MP_ROM_PTR(&mod_sentai_link_command_obj) },
    { MP_ROM_QSTR(MP_QSTR_obstacle_distance), MP_ROM_PTR(&mod_sentai_link_obstacle_distance_obj) },
    { MP_ROM_QSTR(MP_QSTR_obstacles_from_tracker), MP_ROM_PTR(&mod_sentai_link_obstacles_from_tracker_obj) },
    { MP_ROM_QSTR(MP_QSTR_obstacles_from_points), MP_ROM_PTR(&mod_sentai_link_obstacles_from_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_link_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),           MP_ROM_PTR(&mod_sentai_link_receive_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),             MP_ROM_PTR(&mod_sentai_link_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),               MP_ROM_PTR(&mod_sentai_link_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),           MP_ROM_PTR(&mod_sentai_link_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),              MP_ROM_PTR(&mod_sentai_link_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_flow),         MP_ROM_PTR(&mod_sentai_link_send_flow_obj) },
    { MP_ROM_QSTR(MP_QSTR_flow),              MP_ROM_PTR(&mod_sentai_link_flow_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_link_globals, sentai_link_globals_table);
static const mp_obj_module_t sentai_link_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_link_globals,
};
