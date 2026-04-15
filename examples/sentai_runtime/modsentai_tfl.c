// ============== sentai.tfl — TFLite Micro CPU-only inference ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
// Runs TFLite models on the Cortex-M7 CPU without EdgeTPU.
// API mirrors sentai.tpu where applicable, but adds set_input() and unload().

// sentai.tfl.load(path [, arena_kb]) -> int
// Load a .tflite model for CPU inference.
// arena_kb: tensor arena size in KB (default 1024 = 1 MB).
// Returns 0 on success, negative on error.
static mp_obj_t mod_tfl_load(size_t n_args, const mp_obj_t *args) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(args[0]);
    int arena_kb = (n_args >= 2) ? mp_obj_get_int(args[1]) : 1024;
    int rc = sentai_tfl_load(path, arena_kb);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_tfl_load_obj, 1, 2, mod_tfl_load);

// sentai.tfl.unload() -> None
// Free the interpreter and arena memory.
static mp_obj_t mod_tfl_unload(void) {
    sentai_tfl_unload();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_unload_obj, mod_tfl_unload);

// sentai.tfl.invoke() -> int (inference time in ms, -1 if not ready, -2 if failed)
static mp_obj_t mod_tfl_invoke(void) {
    int result = sentai_tfl_invoke();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_invoke_obj, mod_tfl_invoke);

// sentai.tfl.ready() -> bool
static mp_obj_t mod_tfl_ready(void) {
    return mp_obj_new_bool(sentai_tfl_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_ready_obj, mod_tfl_ready);

// sentai.tfl.num_outputs() -> int
static mp_obj_t mod_tfl_num_outputs(void) {
    return mp_obj_new_int(sentai_tfl_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_num_outputs_obj, mod_tfl_num_outputs);

// sentai.tfl.output_size(idx) -> int (bytes)
static mp_obj_t mod_tfl_output_size(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tfl_get_output_size(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_size_obj, mod_tfl_output_size);

// sentai.tfl.output(idx) -> bytes (raw tensor data)
static mp_obj_t mod_tfl_output(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int size = sentai_tfl_get_output_size(idx);
    const void* data = sentai_tfl_get_output_data(idx);
    if (!data || size <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    return mp_obj_new_bytes((const byte*)data, size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_obj, mod_tfl_output);

// sentai.tfl.output_dims(idx) -> tuple (dim0, dim1, ...)
static mp_obj_t mod_tfl_output_dims(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int ndims = sentai_tfl_get_output_num_dims(idx);
    mp_obj_t items[8];
    if (ndims > 8) ndims = 8;
    for (int i = 0; i < ndims; i++) {
        items[i] = mp_obj_new_int(sentai_tfl_get_output_dim(idx, i));
    }
    return mp_obj_new_tuple(ndims, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_dims_obj, mod_tfl_output_dims);

// sentai.tfl.output_type(idx) -> int (TfLiteType enum)
static mp_obj_t mod_tfl_output_type(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tfl_get_output_type(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_type_obj, mod_tfl_output_type);

// sentai.tfl.row(output_idx, row) -> tuple of ints
// Same semantics as sentai.tpu.row().
static mp_obj_t mod_tfl_row(mp_obj_t oidx_obj, mp_obj_t row_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int row = mp_obj_get_int(row_obj);
    int ndims = sentai_tfl_get_output_num_dims(oidx);
    int type = sentai_tfl_get_output_type(oidx);
    const uint8_t* data = (const uint8_t*)sentai_tfl_get_output_data(oidx);
    if (!data || ndims < 2) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    int cols = sentai_tfl_get_output_dim(oidx, ndims - 1);
    int total_bytes = sentai_tfl_get_output_size(oidx);

    int elem_size = 1;
    if (type == 1) elem_size = 4; // float32
    if (type == 2) elem_size = 4; // int32

    int byte_offset = row * cols * elem_size;
    if (byte_offset < 0 || byte_offset + cols * elem_size > total_bytes) {
        mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("row OOB"));
    }

    mp_obj_t items[64];
    if (cols > 64) cols = 64;

    if (type == 1) {
        // float32
        const float* p = (const float*)(data + byte_offset);
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_float(p[i]);
    } else if (type == 9) {
        const int8_t* p = (const int8_t*)(data + byte_offset);
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    } else if (type == 2) {
        const int32_t* p = (const int32_t*)(data + byte_offset);
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    } else {
        const uint8_t* p = data + byte_offset;
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    }
    return mp_obj_new_tuple(cols, items);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_tfl_row_obj, mod_tfl_row);

// sentai.tfl.val(output_idx, flat_index) -> int or float
static mp_obj_t mod_tfl_val(mp_obj_t oidx_obj, mp_obj_t fi_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int fi = mp_obj_get_int(fi_obj);
    int type = sentai_tfl_get_output_type(oidx);
    int total_bytes = sentai_tfl_get_output_size(oidx);
    const uint8_t* data = (const uint8_t*)sentai_tfl_get_output_data(oidx);
    if (!data) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));

    if (type == 1) {
        // float32
        if (fi < 0 || fi * 4 + 4 > total_bytes) mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("OOB"));
        return mp_obj_new_float(((const float*)data)[fi]);
    } else if (type == 9) {
        if (fi < 0 || fi >= total_bytes) mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("OOB"));
        return mp_obj_new_int(((const int8_t*)data)[fi]);
    } else if (type == 2) {
        if (fi < 0 || fi * 4 + 4 > total_bytes) mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("OOB"));
        return mp_obj_new_int(((const int32_t*)data)[fi]);
    } else {
        if (fi < 0 || fi >= total_bytes) mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("OOB"));
        return mp_obj_new_int(data[fi]);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_tfl_val_obj, mod_tfl_val);

// sentai.tfl.input_quant() -> (scale, zero_point)
static mp_obj_t mod_tfl_input_quant(void) {
    float scale = 0;
    int32_t zp = 0;
    int rc = sentai_tfl_input_quant(&scale, &zp);
    if (rc != 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("model not loaded"));
    }
    mp_obj_t items[2] = {
        mp_obj_new_float(scale),
        mp_obj_new_int(zp),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_input_quant_obj, mod_tfl_input_quant);

// sentai.tfl.output_quant(idx) -> (scale, zero_point)
static mp_obj_t mod_tfl_output_quant(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    float scale = 0;
    int32_t zp = 0;
    int rc = sentai_tfl_output_quant(idx, &scale, &zp);
    if (rc != 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("invalid output idx"));
    }
    mp_obj_t items[2] = {
        mp_obj_new_float(scale),
        mp_obj_new_int(zp),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_quant_obj, mod_tfl_output_quant);

// sentai.tfl.output_floats(idx) -> list of floats (dequantized)
static mp_obj_t mod_tfl_output_floats(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int size = sentai_tfl_get_output_size(idx);
    int type = sentai_tfl_get_output_type(idx);
    const void* data = sentai_tfl_get_output_data(idx);

    if (!data || size <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }

    float scale = 1.0f;
    int32_t zero_point = 0;
    int n_elements = 0;

    if (type == 1) {
        n_elements = size / 4;
    } else if (type == 9 || type == 3) {
        sentai_tfl_output_quant(idx, &scale, &zero_point);
        n_elements = size;
    } else {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("unsupported type"));
    }

    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(n_elements, NULL));

    if (type == 1) {
        const float* fdata = (const float*)data;
        for (int i = 0; i < n_elements; i++) {
            list->items[i] = mp_obj_new_float(fdata[i]);
        }
    } else if (type == 9) {
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            float val = scale * ((float)idata[i] - (float)zero_point);
            list->items[i] = mp_obj_new_float(val);
        }
    } else {
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            float val = scale * ((float)udata[i] - (float)zero_point);
            list->items[i] = mp_obj_new_float(val);
        }
    }

    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_output_floats_obj, mod_tfl_output_floats);

// sentai.tfl.input_type() -> int (TfLiteType: 9=int8, 3=uint8, 1=float32)
static mp_obj_t mod_tfl_input_type(void) {
    return mp_obj_new_int(sentai_tfl_input_type());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_input_type_obj, mod_tfl_input_type);

// sentai.tfl.input_size() -> int (bytes)
static mp_obj_t mod_tfl_input_size(void) {
    return mp_obj_new_int(sentai_tfl_input_size());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_input_size_obj, mod_tfl_input_size);

// sentai.tfl.input_dims() -> tuple (dim0, dim1, ...)
static mp_obj_t mod_tfl_input_dims(void) {
    int ndims = sentai_tfl_input_num_dims();
    mp_obj_t items[8];
    if (ndims > 8) ndims = 8;
    for (int i = 0; i < ndims; i++) {
        items[i] = mp_obj_new_int(sentai_tfl_input_dim(i));
    }
    return mp_obj_new_tuple(ndims, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_input_dims_obj, mod_tfl_input_dims);

// sentai.tfl.set_input(data) -> int
// Write raw bytes into the input tensor.
// data: bytes object, must match input tensor size exactly.
// For float32 inputs, pack floats with struct.pack('<Nf', ...).
static mp_obj_t mod_tfl_set_input(mp_obj_t data_obj) {
    mp_buffer_info_t buf_info;
    mp_get_buffer_raise(data_obj, &buf_info, MP_BUFFER_READ);
    int rc = sentai_tfl_set_input((const uint8_t*)buf_info.buf, buf_info.len);
    if (rc == -1) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("model not loaded"));
    }
    if (rc == -2) {
        mp_raise_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("size mismatch: got %d, need %d"),
            (int)buf_info.len, sentai_tfl_input_size());
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_set_input_obj, mod_tfl_set_input);

// sentai.tfl.load_image(path) -> int
// Load an image file (JPEG or raw RGB) into the input tensor.
static mp_obj_t mod_tfl_load_image(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_tfl_load_image(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_load_image_obj, mod_tfl_load_image);

// sentai.tfl.save_output(path) -> int
// Save all output tensors to CSV file.
static mp_obj_t mod_tfl_save_output(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_tfl_save_output(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_tfl_save_output_obj, mod_tfl_save_output);

// sentai.tfl.info() -> None
// Print model details (arena usage, input/output tensor shapes).
static mp_obj_t mod_tfl_info(void) {
    sentai_tfl_info();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_tfl_info_obj, mod_tfl_info);

// ---- module table ----
static const mp_rom_map_elem_t sentai_tfl_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_tfl) },
    // Lifecycle
    { MP_ROM_QSTR(MP_QSTR_load),          MP_ROM_PTR(&mod_tfl_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_unload),        MP_ROM_PTR(&mod_tfl_unload_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke),        MP_ROM_PTR(&mod_tfl_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_ready),         MP_ROM_PTR(&mod_tfl_ready_obj) },
    // Input
    { MP_ROM_QSTR(MP_QSTR_set_input),     MP_ROM_PTR(&mod_tfl_set_input_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_image),    MP_ROM_PTR(&mod_tfl_load_image_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_size),    MP_ROM_PTR(&mod_tfl_input_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_dims),    MP_ROM_PTR(&mod_tfl_input_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_type),    MP_ROM_PTR(&mod_tfl_input_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_quant),   MP_ROM_PTR(&mod_tfl_input_quant_obj) },
    // Output
    { MP_ROM_QSTR(MP_QSTR_num_outputs),   MP_ROM_PTR(&mod_tfl_num_outputs_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size),   MP_ROM_PTR(&mod_tfl_output_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_output),        MP_ROM_PTR(&mod_tfl_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims),   MP_ROM_PTR(&mod_tfl_output_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type),   MP_ROM_PTR(&mod_tfl_output_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_quant),  MP_ROM_PTR(&mod_tfl_output_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_floats), MP_ROM_PTR(&mod_tfl_output_floats_obj) },
    { MP_ROM_QSTR(MP_QSTR_row),           MP_ROM_PTR(&mod_tfl_row_obj) },
    { MP_ROM_QSTR(MP_QSTR_val),           MP_ROM_PTR(&mod_tfl_val_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save_output),   MP_ROM_PTR(&mod_tfl_save_output_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_info),          MP_ROM_PTR(&mod_tfl_info_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tfl_globals, sentai_tfl_globals_table);
static const mp_obj_module_t sentai_tfl_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_tfl_globals,
};
