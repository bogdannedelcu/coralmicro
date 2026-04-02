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
    const uint8_t* emb = NULL;
    uint32_t emb_len = 0;
    uint32_t emb_crc = 0;
    uint8_t severity = 6;
    if (n_args > 11 && args[11] != mp_const_none) {
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[11], &bufinfo, MP_BUFFER_READ);
        emb = (const uint8_t*)bufinfo.buf;
        emb_len = bufinfo.len;
    }
    if (n_args > 12) emb_crc = (uint32_t)mp_obj_get_int(args[12]);
    if (n_args > 13) severity = (uint8_t)mp_obj_get_int(args[13]);
    return mp_obj_new_int(sentai_link_send_vision(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, class_id,
        emb, emb_len, emb_crc, severity));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_detection_obj, 11, 14, mod_sentai_link_send_detection);

// sentai.link.send_update(...) -> int
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
    uint8_t severity = (n_args > 11) ? (uint8_t)mp_obj_get_int(args[11]) : 6;
    return mp_obj_new_int(sentai_link_send_vision_update(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, age, severity));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_send_update_obj, 11, 12, mod_sentai_link_send_update);

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
    { MP_ROM_QSTR(MP_QSTR_command),           MP_ROM_PTR(&mod_sentai_link_command_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_link_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),           MP_ROM_PTR(&mod_sentai_link_receive_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_link_globals, sentai_link_globals_table);
static const mp_obj_module_t sentai_link_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_link_globals,
};
