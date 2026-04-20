// ============== sentai.camera — Camera ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.camera.init(streaming=1) -> int (0=ok, <0=error)
// streaming=1: continuous mode, streaming=0: trigger mode
static mp_obj_t mod_sentai_cam_init(size_t n_args, const mp_obj_t *args) {
    int streaming = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(sentai_cam_init(streaming));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_init_obj, 0, 1, mod_sentai_cam_init);

// sentai.camera.stop() -> int
static mp_obj_t mod_sentai_cam_stop(void) {
    return mp_obj_new_int(sentai_cam_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_stop_obj, mod_sentai_cam_stop);

// sentai.camera.jpeg(quality=75) -> bytes (JPEG data)
// Captures a frame at camera native resolution and returns JPEG
static mp_obj_t mod_sentai_cam_jpeg(size_t n_args, const mp_obj_t *args) {
    int quality = (n_args > 0) ? mp_obj_get_int(args[0]) : 75;
    int w = sentai_cam_get_width();
    int h = sentai_cam_get_height();
    // Max JPEG buffer - typically much smaller than RGB
    int max_jpeg = w * h;  // generous upper bound
    uint8_t* buf = (uint8_t*)malloc(max_jpeg);
    if (!buf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("jpeg buf alloc"));
    }
    int jpeg_size = sentai_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    if (jpeg_size <= 0) {
        free(buf);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }
    mp_obj_t result = mp_obj_new_bytes(buf, jpeg_size);
    free(buf);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_jpeg_obj, 0, 1, mod_sentai_cam_jpeg);

// sentai.camera.to_tensor([path[, quality]]) -> int
// Captures camera frame into TPU input tensor.  If path is given, also saves
// the scaled RGB frame as JPEG before int8 quantization.
static mp_obj_t mod_sentai_cam_to_tensor(size_t n_args, const mp_obj_t *args) {
    const char* path = NULL;
    int quality = 75;
    if (n_args >= 1 && args[0] != mp_const_none) {
        _fs_check_usb();
        path = mp_obj_str_get_str(args[0]);
    }
    if (n_args >= 2) {
        quality = mp_obj_get_int(args[1]);
    }
    return mp_obj_new_int(sentai_cam_to_tensor_ex(path, quality));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_to_tensor_obj, 0, 2, mod_sentai_cam_to_tensor);

// sentai.camera.save_jpeg(path, quality=75) -> int (bytes written)
// Capture + JPEG + save to LFS in one call
static mp_obj_t mod_sentai_cam_save_jpeg(size_t n_args, const mp_obj_t *args) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(args[0]);
    int quality = (n_args > 1) ? mp_obj_get_int(args[1]) : 75;
    int w = sentai_cam_get_width();
    int h = sentai_cam_get_height();
    int max_jpeg = w * h;
    uint8_t* buf = (uint8_t*)malloc(max_jpeg);
    if (!buf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("jpeg buf alloc"));
    }

    uint32_t t0 = sentai_ticks_ms();
    int jpeg_size = sentai_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    uint32_t t1 = sentai_ticks_ms();

    if (jpeg_size <= 0) {
        free(buf);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }

    int ok = sentai_fs_write(path, buf, jpeg_size);
    uint32_t t2 = sentai_ticks_ms();

    free(buf);
    if (!ok) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("write fail"));
    }

    printf("[save_jpeg] %s: capture=%lums save=%lums total=%lums (%d bytes, q=%d)\r\n",
           path, (unsigned long)(t1 - t0), (unsigned long)(t2 - t1),
           (unsigned long)(t2 - t0), jpeg_size, quality);

    return mp_obj_new_int(jpeg_size);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_save_jpeg_obj, 1, 2, mod_sentai_cam_save_jpeg);

// sentai.camera.resolution() -> tuple (width, height) - current capture resolution
static mp_obj_t mod_sentai_cam_resolution(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_width());
    items[1] = mp_obj_new_int(sentai_cam_get_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_resolution_obj, mod_sentai_cam_resolution);

// sentai.camera.set_resolution(w, h) -> int (0=ok, -1=invalid)
// Set capture output resolution. Max = native sensor res.
static mp_obj_t mod_sentai_cam_set_resolution(mp_obj_t w_obj, mp_obj_t h_obj) {
    int w = mp_obj_get_int(w_obj);
    int h = mp_obj_get_int(h_obj);
    return mp_obj_new_int(sentai_cam_set_res(w, h));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_set_resolution_obj, mod_sentai_cam_set_resolution);

// sentai.camera.native_res() -> tuple (width, height) - sensor native resolution
static mp_obj_t mod_sentai_cam_native_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_native_width());
    items[1] = mp_obj_new_int(sentai_cam_get_native_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_native_res_obj, mod_sentai_cam_native_res);

// sentai.camera.select(id) -> int (0=ok). id: 0=front, 1=back
static mp_obj_t mod_sentai_cam_select(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    return mp_obj_new_int(sentai_cam_switch(id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_select_obj, mod_sentai_cam_select);

// sentai.camera.frame_count() -> int
// Monotonic ISR-level frame counter. Never resets.
// Use delta (frame_count_after - frame_count_before) to count DMA frames.
static mp_obj_t mod_sentai_cam_frame_count(void) {
    return mp_obj_new_int(sentai_cam_get_frame_seq());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_frame_count_obj, mod_sentai_cam_frame_count);

// sentai.camera.rotate(cam_id, degrees) -> int (0=ok)
// cam_id: 0=front, 1=back.  degrees: 0, 90, 180, 270.
static mp_obj_t mod_sentai_cam_rotate(mp_obj_t cam_obj, mp_obj_t deg_obj) {
    int cam_id = mp_obj_get_int(cam_obj);
    int degrees = mp_obj_get_int(deg_obj);
    return mp_obj_new_int(sentai_cam_rotate(cam_id, degrees));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_rotate_obj, mod_sentai_cam_rotate);

// sentai.camera.switch_drain([n]) -> int (previous value)
//
// Read/write the post-switch drain threshold: number of fresh ISR frames
// required after a MUX flip before get_raw_with_recovery returns a frame.
// Default 2 (one possibly-mixed frame + one fully-new frame).  Accepts
// n in [1,10]; raises ValueError otherwise.  A/B runtime toggle used by
// experiment E17 to inspect post-switch image artifacts at threshold=1.
extern uint32_t sentai_cam_switch_drain_get(void);
extern int      sentai_cam_switch_drain_set(uint32_t n);
static mp_obj_t mod_sentai_cam_switch_drain(size_t n_args, const mp_obj_t *args) {
    int prev = (int) sentai_cam_switch_drain_get();
    if (n_args >= 1) {
        int v = mp_obj_get_int(args[0]);
        if (sentai_cam_switch_drain_set((uint32_t) v) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("switch_drain must be 1..10"));
        }
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_switch_drain_obj,
                                            0, 1, mod_sentai_cam_switch_drain);

// ---- module table ----
static const mp_rom_map_elem_t sentai_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_cam_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mod_sentai_cam_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_jpeg),       MP_ROM_PTR(&mod_sentai_cam_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_tensor),  MP_ROM_PTR(&mod_sentai_cam_to_tensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_jpeg),  MP_ROM_PTR(&mod_sentai_cam_save_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_resolution),     MP_ROM_PTR(&mod_sentai_cam_resolution_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_resolution), MP_ROM_PTR(&mod_sentai_cam_set_resolution_obj) },
    { MP_ROM_QSTR(MP_QSTR_native_res), MP_ROM_PTR(&mod_sentai_cam_native_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_select),     MP_ROM_PTR(&mod_sentai_cam_select_obj) },
    { MP_ROM_QSTR(MP_QSTR_frame_count),  MP_ROM_PTR(&mod_sentai_cam_frame_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotate),     MP_ROM_PTR(&mod_sentai_cam_rotate_obj) },
    { MP_ROM_QSTR(MP_QSTR_switch_drain), MP_ROM_PTR(&mod_sentai_cam_switch_drain_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_camera_globals, sentai_camera_globals_table);
static const mp_obj_module_t sentai_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_camera_globals,
};
