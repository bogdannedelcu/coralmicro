// ============== sentai.tpu — EdgeTPU inference ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.tpu.invoke() -> int (inference time in ms, -1 if not ready, -2 if invoke failed)
static mp_obj_t mod_sentai_invoke(void) {
    int result = sentai_tpu_invoke();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_invoke_obj, mod_sentai_invoke);

// sentai.tpu.ready() -> bool
static mp_obj_t mod_sentai_is_ready(void) {
    return mp_obj_new_bool(sentai_tpu_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_is_ready_obj, mod_sentai_is_ready);

// sentai.tpu.num_outputs() -> int
static mp_obj_t mod_sentai_num_outputs(void) {
    return mp_obj_new_int(sentai_tpu_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_num_outputs_obj, mod_sentai_num_outputs);

// sentai.tpu.output_size(idx) -> int (bytes)
static mp_obj_t mod_sentai_output_size(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tpu_get_output_size(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_size_obj, mod_sentai_output_size);

// sentai.tpu.output(idx) -> bytes (raw tensor data, integers)
static mp_obj_t mod_sentai_get_output(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int size = sentai_tpu_get_output_size(idx);
    const void* data = sentai_tpu_get_output_data(idx);
    if (!data || size <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    return mp_obj_new_bytes((const byte*)data, size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_get_output_obj, mod_sentai_get_output);

// sentai.tpu.output_dims(idx) -> tuple (dim0, dim1, ...)
static mp_obj_t mod_sentai_output_dims(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int ndims = sentai_tpu_get_output_num_dims(idx);
    mp_obj_t items[8];
    if (ndims > 8) ndims = 8;
    for (int i = 0; i < ndims; i++) {
        items[i] = mp_obj_new_int(sentai_tpu_get_output_dim(idx, i));
    }
    return mp_obj_new_tuple(ndims, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_dims_obj, mod_sentai_output_dims);

// sentai.tpu.output_type(idx) -> int (TfLiteType enum)
static mp_obj_t mod_sentai_output_type(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tpu_get_output_type(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_type_obj, mod_sentai_output_type);

// sentai.tpu.row(output_idx, row) -> tuple of ints
// For tensor shape [1, 1344, 5]: row(0, 42) returns (v0, v1, v2, v3, v4)
// Handles int8 sign extension automatically.
static mp_obj_t mod_sentai_get_row(mp_obj_t oidx_obj, mp_obj_t row_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int row = mp_obj_get_int(row_obj);
    int ndims = sentai_tpu_get_output_num_dims(oidx);
    int type = sentai_tpu_get_output_type(oidx);
    const uint8_t* data = (const uint8_t*)sentai_tpu_get_output_data(oidx);
    if (!data || ndims < 2) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    // Last dimension = number of columns per row
    int cols = sentai_tpu_get_output_dim(oidx, ndims - 1);
    int total_bytes = sentai_tpu_get_output_size(oidx);

    // Element size based on type
    int elem_size = 1;
    if (type == 2) elem_size = 4; // int32

    int byte_offset = row * cols * elem_size;
    if (byte_offset < 0 || byte_offset + cols * elem_size > total_bytes) {
        mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("row OOB"));
    }

    mp_obj_t items[64];
    if (cols > 64) cols = 64;

    if (type == 9) {
        // int8 - sign extend
        const int8_t* p = (const int8_t*)(data + byte_offset);
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    } else if (type == 2) {
        // int32
        const int32_t* p = (const int32_t*)(data + byte_offset);
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    } else {
        // uint8 (type 3) or other
        const uint8_t* p = data + byte_offset;
        for (int i = 0; i < cols; i++) items[i] = mp_obj_new_int(p[i]);
    }
    return mp_obj_new_tuple(cols, items);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_get_row_obj, mod_sentai_get_row);

// sentai.tpu.value(output_idx, flat_index) -> int
// Access a single value from the flat tensor array with proper type handling.
static mp_obj_t mod_sentai_get_value(mp_obj_t oidx_obj, mp_obj_t fi_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int fi = mp_obj_get_int(fi_obj);
    int type = sentai_tpu_get_output_type(oidx);
    int total_bytes = sentai_tpu_get_output_size(oidx);
    const uint8_t* data = (const uint8_t*)sentai_tpu_get_output_data(oidx);
    if (!data) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));

    if (type == 9) {
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
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_get_value_obj, mod_sentai_get_value);

// sentai.tpu.input_quant() -> (scale, zero_point)
// Returns quantization params for the input tensor.
// scale is a float, zero_point is int.
// real_value = scale * (q - zero_point)
static mp_obj_t mod_sentai_input_quant(void) {
    float scale = 0;
    int32_t zp = 0;
    int rc = sentai_tpu_input_quant(&scale, &zp);
    if (rc != 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("model not loaded"));
    }
    mp_obj_t items[2] = {
        mp_obj_new_float(scale),
        mp_obj_new_int(zp),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_input_quant_obj, mod_sentai_input_quant);

// sentai.tpu.output_quant(idx) -> (scale, zero_point)
// Returns quantization params for output tensor idx.
static mp_obj_t mod_sentai_output_quant(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    float scale = 0;
    int32_t zp = 0;
    int rc = sentai_tpu_output_quant(idx, &scale, &zp);
    if (rc != 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("invalid output idx"));
    }
    mp_obj_t items[2] = {
        mp_obj_new_float(scale),
        mp_obj_new_int(zp),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_quant_obj, mod_sentai_output_quant);

// sentai.tpu.output_floats(idx) -> list of floats (dequantized)
// Returns output tensor idx as a list of float values.
// Automatically dequantizes int8/uint8 using scale and zero_point.
// For float32 tensors, returns values directly.
// Useful for feeding TPU features to AIfES for transfer learning.
static mp_obj_t mod_sentai_output_floats(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int size = sentai_tpu_get_output_size(idx);
    int type = sentai_tpu_get_output_type(idx);
    const void* data = sentai_tpu_get_output_data(idx);
    
    if (!data || size <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    
    float scale = 1.0f;
    int32_t zero_point = 0;
    int n_elements = 0;
    
    if (type == 1) {
        // float32 - no dequantization needed
        n_elements = size / 4;
    } else if (type == 9 || type == 3) {
        // int8 or uint8 - get quantization params
        sentai_tpu_output_quant(idx, &scale, &zero_point);
        n_elements = size;
    } else {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("unsupported type"));
    }
    
    // Allocate list
    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(n_elements, NULL));
    
    if (type == 1) {
        // float32
        const float* fdata = (const float*)data;
        for (int i = 0; i < n_elements; i++) {
            list->items[i] = mp_obj_new_float(fdata[i]);
        }
    } else if (type == 9) {
        // int8 - dequantize: real = scale * (q - zero_point)
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            float val = scale * ((float)idata[i] - (float)zero_point);
            list->items[i] = mp_obj_new_float(val);
        }
    } else {
        // uint8 - dequantize
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            float val = scale * ((float)udata[i] - (float)zero_point);
            list->items[i] = mp_obj_new_float(val);
        }
    }
    
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_floats_obj, mod_sentai_output_floats);

// sentai.tpu.input_type() -> int (TfLiteType: 9=int8, 3=uint8, 1=float32)
static mp_obj_t mod_sentai_input_type(void) {
    return mp_obj_new_int(sentai_tpu_input_type());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_input_type_obj, mod_sentai_input_type);

// sentai.tpu.load(path) -> int  - Load a TFLite model from flash
static mp_obj_t mod_sentai_load_model(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_load_model(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_load_model_obj, mod_sentai_load_model);

// sentai.tpu.load_image(path) -> int  - Load an image into input tensor
static mp_obj_t mod_sentai_load_image(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_load_image(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_load_image_obj, mod_sentai_load_image);

// sentai.tpu.save_output(path) -> int  - Save all output tensors to CSV file
static mp_obj_t mod_sentai_save_output(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_save_output(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_save_output_obj, mod_sentai_save_output);

// detect / draw / yolo_info MicroPython wrappers were retired in build
// #1100 (REPL exposes raw output bytes only; structured post-processing
// stays in C++).  The C-side functions `sentai_tpu_detect` /
// `sentai_tpu_draw` / `sentai_tpu_output_yolo_info` are still used by
// detection_task.cc internally — keeping them.  Removing the MP wrappers
// reclaims ~4 KB of ITCM/text and ~80 lines of dead Python-side glue.

// ============== sentai.tpu multi-slot API (Phase 1, 2026-04-28) ==============
extern int sentai_load_model_slot(int slot, const char* path);
extern int sentai_tpu_invoke_slot(int slot);
extern int sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf);
extern int sentai_tpu_slot_ready(int slot);
extern int sentai_tpu_slot_count(void);
extern int sentai_tpu_num_outputs_slot(int slot);
extern int sentai_tpu_get_output_size_slot(int slot, int idx);
extern const void* sentai_tpu_get_output_data_slot(int slot, int idx);
extern int sentai_tpu_set_input_slot(int slot, const uint8_t* data, int len);
extern uint32_t sentai_tpu_output_hash_slot(int slot);
extern int sentai_tpu_get_output_num_dims_slot(int slot, int idx);
extern int sentai_tpu_get_output_dim_slot(int slot, int idx, int dim);
extern int sentai_tpu_get_output_type_slot(int slot, int idx);
extern int sentai_tpu_output_quant_slot(int slot, int idx, float* scale, int32_t* zp);

static mp_obj_t mod_sentai_load_model_slot(mp_obj_t slot_obj, mp_obj_t path_obj) {
    _fs_check_usb();
    int slot = mp_obj_get_int(slot_obj);
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_load_model_slot(slot, path));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_load_model_slot_obj, mod_sentai_load_model_slot);

static mp_obj_t mod_sentai_invoke_slot(mp_obj_t slot_obj) {
    return mp_obj_new_int(sentai_tpu_invoke_slot(mp_obj_get_int(slot_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_invoke_slot_obj, mod_sentai_invoke_slot);

static mp_obj_t mod_sentai_slot_ready(mp_obj_t slot_obj) {
    return mp_obj_new_bool(sentai_tpu_slot_ready(mp_obj_get_int(slot_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_slot_ready_obj, mod_sentai_slot_ready);

static mp_obj_t mod_sentai_slot_count(void) {
    return mp_obj_new_int(sentai_tpu_slot_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_slot_count_obj, mod_sentai_slot_count);

static mp_obj_t mod_sentai_set_input_slot(mp_obj_t slot_obj, mp_obj_t bytes_obj) {
    int slot = mp_obj_get_int(slot_obj);
    mp_buffer_info_t buf;
    mp_get_buffer_raise(bytes_obj, &buf, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_tpu_set_input_slot(slot, (const uint8_t*)buf.buf, (int)buf.len));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_set_input_slot_obj, mod_sentai_set_input_slot);

static mp_obj_t mod_sentai_output_hash(mp_obj_t slot_obj) {
    uint32_t h = sentai_tpu_output_hash_slot(mp_obj_get_int(slot_obj));
    return mp_obj_new_int_from_uint(h);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_hash_obj, mod_sentai_output_hash);

static mp_obj_t mod_sentai_output_slot(mp_obj_t slot_obj, mp_obj_t idx_obj) {
    int slot = mp_obj_get_int(slot_obj);
    int idx  = mp_obj_get_int(idx_obj);
    int size = sentai_tpu_get_output_size_slot(slot, idx);
    if (size <= 0) return mp_const_none;
    const void* data = sentai_tpu_get_output_data_slot(slot, idx);
    if (!data) return mp_const_none;
    return mp_obj_new_bytes((const byte*)data, size);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_output_slot_obj, mod_sentai_output_slot);

// Phase 2b raw introspection: per-slot output shape/type/quant.
// Lets a REPL caller iterate raw bytes without doing post-processing
// in MicroPython.  Real post-processing (NMS / classify / etc.)
// will live in C++ when needed.
static mp_obj_t mod_sentai_output_size_slot(mp_obj_t slot_obj, mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_size_slot(
        mp_obj_get_int(slot_obj), mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_output_size_slot_obj, mod_sentai_output_size_slot);

static mp_obj_t mod_sentai_output_dims_slot(mp_obj_t slot_obj, mp_obj_t idx_obj) {
    int slot = mp_obj_get_int(slot_obj);
    int idx  = mp_obj_get_int(idx_obj);
    int n = sentai_tpu_get_output_num_dims_slot(slot, idx);
    if (n <= 0) return mp_obj_new_tuple(0, NULL);
    if (n > 8) n = 8;
    mp_obj_t items[8];
    for (int i = 0; i < n; i++) {
        items[i] = mp_obj_new_int(sentai_tpu_get_output_dim_slot(slot, idx, i));
    }
    return mp_obj_new_tuple(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_output_dims_slot_obj, mod_sentai_output_dims_slot);

static mp_obj_t mod_sentai_output_type_slot(mp_obj_t slot_obj, mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_type_slot(
        mp_obj_get_int(slot_obj), mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_output_type_slot_obj, mod_sentai_output_type_slot);

static mp_obj_t mod_sentai_output_quant_slot(mp_obj_t slot_obj, mp_obj_t idx_obj) {
    float scale = 0;
    int32_t zp = 0;
    int rc = sentai_tpu_output_quant_slot(
        mp_obj_get_int(slot_obj), mp_obj_get_int(idx_obj), &scale, &zp);
    if (rc != 0) return mp_const_none;
    mp_obj_t items[2] = { mp_obj_new_float(scale), mp_obj_new_int(zp) };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_output_quant_slot_obj, mod_sentai_output_quant_slot);

static mp_obj_t mod_sentai_num_outputs_slot(mp_obj_t slot_obj) {
    return mp_obj_new_int(sentai_tpu_num_outputs_slot(mp_obj_get_int(slot_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_num_outputs_slot_obj, mod_sentai_num_outputs_slot);

// sentai.tpu.dump_eps() — print the USB endpoint descriptor table that
// was observed during EdgeTPU enumeration.  Safe from Python because the
// underlying printer is just printf in task context.  Useful when testing
// multi_ep firmware: lets us see how many bulk endpoints the TPU
// advertised and what their numbers/directions are.
extern void sentai_usb_edgetpu_dump_eps(void);
static mp_obj_t mod_sentai_tpu_dump_eps(void) {
    sentai_usb_edgetpu_dump_eps();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_tpu_dump_eps_obj, mod_sentai_tpu_dump_eps);

// ---- module table ----
static const mp_rom_map_elem_t sentai_tpu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_tpu) },
    { MP_ROM_QSTR(MP_QSTR_load),        MP_ROM_PTR(&mod_sentai_load_model_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_image),  MP_ROM_PTR(&mod_sentai_load_image_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke),      MP_ROM_PTR(&mod_sentai_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_ready),       MP_ROM_PTR(&mod_sentai_is_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_num_outputs), MP_ROM_PTR(&mod_sentai_num_outputs_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size), MP_ROM_PTR(&mod_sentai_output_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_output),      MP_ROM_PTR(&mod_sentai_get_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims), MP_ROM_PTR(&mod_sentai_output_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type), MP_ROM_PTR(&mod_sentai_output_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_row),         MP_ROM_PTR(&mod_sentai_get_row_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),        MP_ROM_PTR(&mod_sentai_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_output), MP_ROM_PTR(&mod_sentai_save_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_quant), MP_ROM_PTR(&mod_sentai_input_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_quant),MP_ROM_PTR(&mod_sentai_output_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_floats),MP_ROM_PTR(&mod_sentai_output_floats_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_type),  MP_ROM_PTR(&mod_sentai_input_type_obj) },
    // detect / yolo_info / draw retired from REPL surface 2026-04-28.
    // Post-processing (NMS, classify, etc.) will move to typed C++
    // helpers when needed — REPL only exposes raw output bytes + shape.
    { MP_ROM_QSTR(MP_QSTR_dump_eps),    MP_ROM_PTR(&mod_sentai_tpu_dump_eps_obj) },
    // Multi-slot extension (Phase 1).
    { MP_ROM_QSTR(MP_QSTR_load_slot),    MP_ROM_PTR(&mod_sentai_load_model_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke_slot),  MP_ROM_PTR(&mod_sentai_invoke_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_slot_ready),   MP_ROM_PTR(&mod_sentai_slot_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_slot_count),   MP_ROM_PTR(&mod_sentai_slot_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_input_slot), MP_ROM_PTR(&mod_sentai_set_input_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_hash),  MP_ROM_PTR(&mod_sentai_output_hash_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_slot),  MP_ROM_PTR(&mod_sentai_output_slot_obj) },
    // Phase 2b raw introspection (no post-proc in MP).
    { MP_ROM_QSTR(MP_QSTR_num_outputs_slot), MP_ROM_PTR(&mod_sentai_num_outputs_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size_slot), MP_ROM_PTR(&mod_sentai_output_size_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims_slot), MP_ROM_PTR(&mod_sentai_output_dims_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type_slot), MP_ROM_PTR(&mod_sentai_output_type_slot_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_quant_slot),MP_ROM_PTR(&mod_sentai_output_quant_slot_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tpu_globals, sentai_tpu_globals_table);
static const mp_obj_module_t sentai_tpu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_tpu_globals,
};
