// ============== sentai.mesh — Meshtastic mesh radio ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.mesh.init(baudrate=38400) -> int
static mp_obj_t mod_sentai_mesh_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 38400;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    return mp_obj_new_int(sentai_mesh_init(baudrate));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_init_obj, 0, 1, mod_sentai_mesh_init);

// sentai.mesh.stop() -> int
static mp_obj_t mod_sentai_mesh_stop(void) {
    return mp_obj_new_int(sentai_mesh_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mesh_stop_obj, mod_sentai_mesh_stop);

// sentai.mesh.send(text, dest=0xFFFFFFFF, channel=0, ack=1) -> int
static mp_obj_t mod_sentai_mesh_send(size_t n_args, const mp_obj_t *args) {
    const char* text = mp_obj_str_get_str(args[0]);
    uint32_t dest = (n_args > 1) ? (uint32_t)mp_obj_get_int(args[1]) : 0xFFFFFFFF;
    uint8_t channel = (n_args > 2) ? (uint8_t)mp_obj_get_int(args[2]) : 0;
    int want_ack = (n_args > 3) ? mp_obj_get_int(args[3]) : 1;
    return mp_obj_new_int(sentai_mesh_send_text(text, dest, channel, want_ack));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_send_obj, 1, 4, mod_sentai_mesh_send);

// sentai.mesh.send_detection(sensor_id, track_id, alarm_type, timestamp, seq,
//                            x, y, w, h, conf, class_id,
//                            embedding=None, embed_crc8=0,
//                            dest=0xFFFFFFFF, channel=0, ack=1) -> int
static mp_obj_t mod_sentai_mesh_send_detection(size_t n_args, const mp_obj_t *args) {
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
    // Optional: embedding (bytes), embed_crc8, dest, channel, ack
    const uint8_t* emb = NULL;
    uint32_t emb_len = 0;
    uint32_t emb_crc = 0;
    uint32_t dest = 0xFFFFFFFF;
    uint8_t channel = 0;
    int want_ack = 1;
    if (n_args > 11 && args[11] != mp_const_none) {
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[11], &bufinfo, MP_BUFFER_READ);
        emb = (const uint8_t*)bufinfo.buf;
        emb_len = bufinfo.len;
    }
    if (n_args > 12) emb_crc = (uint32_t)mp_obj_get_int(args[12]);
    if (n_args > 13) dest = (uint32_t)mp_obj_get_int(args[13]);
    if (n_args > 14) channel = (uint8_t)mp_obj_get_int(args[14]);
    if (n_args > 15) want_ack = mp_obj_get_int(args[15]);
    return mp_obj_new_int(sentai_mesh_send_detection(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, class_id,
        emb, emb_len, emb_crc,
        dest, channel, want_ack));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_send_detection_obj, 11, 16, mod_sentai_mesh_send_detection);

// sentai.mesh.send_update(sensor_id, track_id, alarm_type, timestamp, seq,
//                         x, y, w, h, conf, age,
//                         dest=0xFFFFFFFF, channel=0, ack=1) -> int
static mp_obj_t mod_sentai_mesh_send_update(size_t n_args, const mp_obj_t *args) {
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
    uint32_t dest = (n_args > 11) ? (uint32_t)mp_obj_get_int(args[11]) : 0xFFFFFFFF;
    uint8_t channel = (n_args > 12) ? (uint8_t)mp_obj_get_int(args[12]) : 0;
    int want_ack = (n_args > 13) ? mp_obj_get_int(args[13]) : 1;
    return mp_obj_new_int(sentai_mesh_send_update(
        sensor_id, track_id, alarm_type, timestamp, seq,
        x, y, w, h, conf, age,
        dest, channel, want_ack));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_send_update_obj, 11, 14, mod_sentai_mesh_send_update);

// sentai.mesh.receive(timeout_ms=0) -> dict or None
// Returns dict with: from, to, text, id, rssi, snr, channel, hop_limit
static mp_obj_t mod_sentai_mesh_receive(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 0;
    mesh_rx_msg_t msg;
    int got;
    if (timeout_ms == 0)
        got = sentai_mesh_receive_text(&msg);
    else
        got = sentai_mesh_receive_text_wait(&msg, timeout_ms);
    if (!got) return mp_const_none;
    mp_obj_dict_t *d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_from), mp_obj_new_int_from_uint(msg.from));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_to), mp_obj_new_int_from_uint(msg.to));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_text),
                      mp_obj_new_str(msg.text, msg.text_len));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_id), mp_obj_new_int_from_uint(msg.id));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rssi), mp_obj_new_int(msg.rx_rssi));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_snr),
                      mp_obj_new_int((int)(msg.rx_snr * 100)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_channel), mp_obj_new_int(msg.channel));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_hop_limit), mp_obj_new_int(msg.hop_limit));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_receive_obj, 0, 1, mod_sentai_mesh_receive);

// sentai.mesh.receive_vision(timeout_ms=0) -> dict or None
// Returns dict with: from, to, id, rssi, snr, channel,
//   sensor_id, track_id, alarm_type, timestamp, seq,
//   type ('new' or 'update'), and detection-specific fields
static mp_obj_t mod_sentai_mesh_receive_vision(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 0;
    mesh_rx_vision_t msg;
    int got;
    if (timeout_ms == 0)
        got = sentai_mesh_receive_vision(&msg);
    else
        got = sentai_mesh_receive_vision_wait(&msg, timeout_ms);
    if (!got) return mp_const_none;
    mp_obj_dict_t *d = mp_obj_new_dict(16);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_from), mp_obj_new_int_from_uint(msg.from));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_to), mp_obj_new_int_from_uint(msg.to));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_id), mp_obj_new_int_from_uint(msg.id));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rssi), mp_obj_new_int(msg.rx_rssi));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_snr),
                      mp_obj_new_int((int)(msg.rx_snr * 100)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_channel), mp_obj_new_int(msg.channel));
    // VisionMessage common fields
    const visionmesh_VisionMessage* v = &msg.vision;
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_id), mp_obj_new_int_from_uint(v->sensor_id));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_track_id), mp_obj_new_int_from_uint(v->track_id));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_alarm_type), mp_obj_new_int_from_uint(v->alarm_type));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_timestamp), mp_obj_new_int_from_uint(v->timestamp_utc));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_seq), mp_obj_new_int_from_uint(v->seq));
    if (v->which_body == visionmesh_VisionMessage_new_detection_tag) {
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_type),
                          mp_obj_new_str("new", 3));
        const visionmesh_NewDetection* nd = &v->body.new_detection;
        uint32_t xywh = nd->xywh_packed;
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_int(xywh & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_int((xywh >> 8) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_w), mp_obj_new_int((xywh >> 16) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_h), mp_obj_new_int((xywh >> 24) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_conf), mp_obj_new_int(nd->conf));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_class_id), mp_obj_new_int(nd->class_id));
        if (nd->embedding.size > 0) {
            mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_embedding),
                              mp_obj_new_bytes(nd->embedding.bytes, nd->embedding.size));
            mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_embed_crc8), mp_obj_new_int(nd->embed_crc8));
        }
    } else if (v->which_body == visionmesh_VisionMessage_update_detection_tag) {
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_type),
                          mp_obj_new_str("update", 6));
        const visionmesh_UpdateDetection* ud = &v->body.update_detection;
        uint32_t xywh = ud->xywh_packed;
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_int(xywh & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_int((xywh >> 8) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_w), mp_obj_new_int((xywh >> 16) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_h), mp_obj_new_int((xywh >> 24) & 0xFF));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_conf), mp_obj_new_int(ud->conf));
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_age), mp_obj_new_int(ud->age));
    }
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_receive_vision_obj, 0, 1, mod_sentai_mesh_receive_vision);

// sentai.mesh.available() -> int  (text messages)
static mp_obj_t mod_sentai_mesh_available(void) {
    return mp_obj_new_int(sentai_mesh_text_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mesh_available_obj, mod_sentai_mesh_available);

// sentai.mesh.vision_available() -> int
static mp_obj_t mod_sentai_mesh_vision_available(void) {
    return mp_obj_new_int(sentai_mesh_vision_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mesh_vision_available_obj, mod_sentai_mesh_vision_available);

// sentai.mesh.node() -> int
static mp_obj_t mod_sentai_mesh_node(void) {
    return mp_obj_new_int_from_uint(sentai_mesh_my_node_num());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mesh_node_obj, mod_sentai_mesh_node);

// sentai.mesh.config(nonce=0) -> int
static mp_obj_t mod_sentai_mesh_config(size_t n_args, const mp_obj_t *args) {
    uint32_t nonce = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_mesh_request_config(nonce));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mesh_config_obj, 0, 1, mod_sentai_mesh_config);

// ---- module table ----
static const mp_rom_map_elem_t sentai_mesh_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_mesh) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&mod_sentai_mesh_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),              MP_ROM_PTR(&mod_sentai_mesh_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),              MP_ROM_PTR(&mod_sentai_mesh_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_detection),    MP_ROM_PTR(&mod_sentai_mesh_send_detection_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_update),       MP_ROM_PTR(&mod_sentai_mesh_send_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),           MP_ROM_PTR(&mod_sentai_mesh_receive_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive_vision),    MP_ROM_PTR(&mod_sentai_mesh_receive_vision_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_mesh_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_vision_available),  MP_ROM_PTR(&mod_sentai_mesh_vision_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_node),              MP_ROM_PTR(&mod_sentai_mesh_node_obj) },
    { MP_ROM_QSTR(MP_QSTR_config),            MP_ROM_PTR(&mod_sentai_mesh_config_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_mesh_globals, sentai_mesh_globals_table);
static const mp_obj_module_t sentai_mesh_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_mesh_globals,
};
