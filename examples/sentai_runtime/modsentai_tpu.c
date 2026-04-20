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

// sentai.tpu.detect([conf[, iou[, max]]]) -> list of (x1,y1,x2,y2,conf,class_id)
// YOLO NMS post-processing.  conf/iou are float 0.0-1.0 (e.g. 0.25 = 25%).
// Coordinates are in model input pixel space.  conf is float 0.0-1.0.
static mp_obj_t mod_sentai_detect(size_t n_args, const mp_obj_t *args) {
    int conf = (n_args >= 1) ? (int)(mp_obj_get_float(args[0]) * 1000.0f) : 250;
    int iou  = (n_args >= 2) ? (int)(mp_obj_get_float(args[1]) * 1000.0f) : 450;
    int maxd = (n_args >= 3) ? mp_obj_get_int(args[2]) : 50;
    if (maxd > 200) maxd = 200;
    if (maxd < 1)   maxd = 1;

    int16_t* buf = m_new(int16_t, maxd * 6);
    int count = 0;
    int rc = sentai_tpu_detect(conf, iou, maxd, buf, &count);

    if (rc != 0) {
        m_del(int16_t, buf, maxd * 6);
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("detect failed (%d)"), rc);
    }

    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(count, NULL));
    for (int i = 0; i < count; i++) {
        mp_obj_t items[6] = {
            mp_obj_new_int(buf[i * 6 + 0]),  // x1
            mp_obj_new_int(buf[i * 6 + 1]),  // y1
            mp_obj_new_int(buf[i * 6 + 2]),  // x2
            mp_obj_new_int(buf[i * 6 + 3]),  // y2
            mp_obj_new_float(buf[i * 6 + 4] / 1000.0f),  // conf (0.0-1.0)
            mp_obj_new_int(buf[i * 6 + 5]),  // class_id
        };
        list->items[i] = mp_obj_new_tuple(6, items);
    }

    m_del(int16_t, buf, maxd * 6);
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_detect_obj, 0, 3, mod_sentai_detect);

// sentai.tpu.draw(path, [dets], [quality]) -> int
// Draw bounding boxes + class labels on the last to_tensor() frame and save JPEG.
// If dets is omitted, runs detect() automatically with default thresholds.
// dets: list of (x1,y1,x2,y2,conf,class_id) tuples from detect().
static mp_obj_t mod_sentai_draw(size_t n_args, const mp_obj_t *args) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(args[0]);

    mp_obj_t list_obj;
    int quality = 75;

    if (n_args >= 2 && mp_obj_is_type(args[1], &mp_type_list)) {
        // draw(path, dets, [quality])
        list_obj = args[1];
        if (n_args >= 3) quality = mp_obj_get_int(args[2]);
    } else {
        // draw(path) or draw(path, quality) — auto-detect
        if (n_args >= 2) quality = mp_obj_get_int(args[1]);
        int16_t* abuf = m_new(int16_t, 50 * 6);
        int acount = 0;
        int arc = sentai_tpu_detect(250, 450, 50, abuf, &acount);
        if (arc != 0) { m_del(int16_t, abuf, 50 * 6); mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("detect failed")); }
        int rc = sentai_tpu_draw(path, abuf, acount, quality);
        m_del(int16_t, abuf, 50 * 6);
        if (rc != 0) mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("draw failed (%d)"), rc);
        return mp_obj_new_int(acount);
    }

    // Parse list of detection tuples
    size_t n_dets;
    mp_obj_t *items;
    mp_obj_list_get(list_obj, &n_dets, &items);

    // Pack into flat int16 array (conf: float 0-1 → permil int16)
    int16_t* buf = m_new(int16_t, n_dets * 6);
    for (size_t i = 0; i < n_dets; i++) {
        size_t tlen;
        mp_obj_t *titems;
        mp_obj_tuple_get(items[i], &tlen, &titems);
        if (tlen != 6) {
            m_del(int16_t, buf, n_dets * 6);
            mp_raise_ValueError(MP_ERROR_TEXT("each det must be (x1,y1,x2,y2,conf,cls)"));
        }
        buf[i * 6 + 0] = (int16_t)mp_obj_get_int(titems[0]);  // x1
        buf[i * 6 + 1] = (int16_t)mp_obj_get_int(titems[1]);  // y1
        buf[i * 6 + 2] = (int16_t)mp_obj_get_int(titems[2]);  // x2
        buf[i * 6 + 3] = (int16_t)mp_obj_get_int(titems[3]);  // y2
        // conf is float 0.0-1.0 → convert to permil int16 for C++
        buf[i * 6 + 4] = (int16_t)(mp_obj_get_float(titems[4]) * 1000.0f + 0.5f);
        buf[i * 6 + 5] = (int16_t)mp_obj_get_int(titems[5]);  // class_id
    }

    int rc = sentai_tpu_draw(path, buf, (int)n_dets, quality);
    m_del(int16_t, buf, n_dets * 6);

    if (rc != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("draw failed (%d)"), rc);
    }
    return mp_obj_new_int((int)n_dets);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_draw_obj, 1, 3, mod_sentai_draw);

// sentai.tpu.yolo_info() -> (layout_str, num_classes, num_anchors) or None
//
// Auto-detects the loaded model's YOLO output layout and class count from the
// output tensor shape alone — no metadata required, so it works on edgetpu-
// compiled .tflite files (the compiler strips most metadata buffers).
//
// Supported layouts:
//   "v5_like"  [1, N, 5+C]   — rows = cx,cy,w,h,obj,cls_0..cls_{C-1}
//                              (YOLOv5-enhanced 1-class is the C=1 degenerate:
//                               [1, N, 6] with last col = class_conf)
//   "v8"       [1, 4+C, N]   — transposed: bbox rows first, then class rows
//   "unknown"                — shape doesn't match any known pattern
//
// Returns None if no model is loaded.
extern int sentai_tpu_output_yolo_info(int* layout, int* num_classes,
                                       int* num_anchors);
static mp_obj_t mod_sentai_yolo_info(void) {
    int lay = 0, nc = 0, na = 0;
    int rc = sentai_tpu_output_yolo_info(&lay, &nc, &na);
    if (rc == -1) return mp_const_none;  // no model loaded
    const char* name;
    switch (lay) {
        case 1:  name = "v5_like"; break;
        case 2:  name = "v8";      break;
        default: name = "unknown"; break;
    }
    mp_obj_t items[3] = {
        mp_obj_new_str(name, strlen(name)),
        mp_obj_new_int(nc),
        mp_obj_new_int(na),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_yolo_info_obj, mod_sentai_yolo_info);

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
    { MP_ROM_QSTR(MP_QSTR_detect),      MP_ROM_PTR(&mod_sentai_detect_obj) },
    { MP_ROM_QSTR(MP_QSTR_yolo_info),   MP_ROM_PTR(&mod_sentai_yolo_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw),        MP_ROM_PTR(&mod_sentai_draw_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tpu_globals, sentai_tpu_globals_table);
static const mp_obj_module_t sentai_tpu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_tpu_globals,
};
