// Custom MicroPython C module: 'coral'
// Provides LED control, sleep, and EdgeTPU inference bridge

#include "py/runtime.h"
#include "py/obj.h"
#include <string.h>

// Implemented in C++ (modcoral_hal.cc), exposed as extern "C"
extern void coral_led_set(int on);
extern void coral_sleep_ms(uint32_t ms);
extern uint32_t coral_ticks_ms(void);

// TPU bridge - implemented in detect_objects_file.cc
extern int coral_tpu_invoke(void);
extern int coral_tpu_is_ready(void);
extern int coral_tpu_num_outputs(void);
extern int coral_tpu_get_output_size(int idx);
extern const void* coral_tpu_get_output_data(int idx);
extern int coral_tpu_get_output_num_dims(int idx);
extern int coral_tpu_get_output_dim(int idx, int dim);
extern int coral_tpu_get_output_type(int idx);

// Filesystem bridge - implemented in modcoral_hal.cc
extern int coral_fs_read(const char* path, uint8_t* buf, int max_size);
extern int coral_fs_size(const char* path);
extern int coral_fs_file_exists(const char* path);
extern int coral_fs_dir_exists(const char* path);
extern int coral_fs_write(const char* path, const uint8_t* buf, int size);
extern int coral_fs_remove(const char* path);
extern int coral_fs_makedirs(const char* path);
extern int coral_fs_listdir(const char* path,
                     void (*callback)(const char* name, int type, int size, void* ud),
                     void* user_data);

// Camera bridge - implemented in detect_objects_file.cc
extern int coral_cam_init(int streaming);
extern int coral_cam_stop(void);
extern int coral_cam_capture_rgb(uint8_t* buf, int width, int height);
extern int coral_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                  int width, int height, int quality);
extern int coral_cam_to_tensor(void);
extern int coral_cam_get_width(void);
extern int coral_cam_get_height(void);
extern int coral_cam_set_res(int w, int h);
extern int coral_cam_get_native_width(void);
extern int coral_cam_get_native_height(void);
extern int coral_cam_switch(int id);

// USB drive bridge - implemented in main_freertos_m7.cc
extern int coral_usb_drive_set(int on);

// coral.led_on()
static mp_obj_t mod_coral_led_on(void) {
    coral_led_set(1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_led_on_obj, mod_coral_led_on);

// coral.led_off()
static mp_obj_t mod_coral_led_off(void) {
    coral_led_set(0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_led_off_obj, mod_coral_led_off);

// coral.sleep_ms(ms)
static mp_obj_t mod_coral_sleep_ms(mp_obj_t ms_obj) {
    coral_sleep_ms(mp_obj_get_int(ms_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_sleep_ms_obj, mod_coral_sleep_ms);

// coral.ticks_ms()
static mp_obj_t mod_coral_ticks_ms(void) {
    return mp_obj_new_int(coral_ticks_ms());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_ticks_ms_obj, mod_coral_ticks_ms);

// coral.invoke() -> int (inference time in ms, -1 if not ready, -2 if invoke failed)
static mp_obj_t mod_coral_invoke(void) {
    int result = coral_tpu_invoke();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_invoke_obj, mod_coral_invoke);

// coral.is_ready() -> bool
static mp_obj_t mod_coral_is_ready(void) {
    return mp_obj_new_bool(coral_tpu_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_is_ready_obj, mod_coral_is_ready);

// coral.num_outputs() -> int
static mp_obj_t mod_coral_num_outputs(void) {
    return mp_obj_new_int(coral_tpu_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_num_outputs_obj, mod_coral_num_outputs);

// coral.output_size(idx) -> int (bytes)
static mp_obj_t mod_coral_output_size(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(coral_tpu_get_output_size(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_output_size_obj, mod_coral_output_size);

// coral.get_output(idx) -> bytes (raw tensor data, integers)
static mp_obj_t mod_coral_get_output(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int size = coral_tpu_get_output_size(idx);
    const void* data = coral_tpu_get_output_data(idx);
    if (!data || size <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    return mp_obj_new_bytes((const byte*)data, size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_get_output_obj, mod_coral_get_output);

// coral.output_dims(idx) -> tuple (dim0, dim1, ...) 
static mp_obj_t mod_coral_output_dims(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int ndims = coral_tpu_get_output_num_dims(idx);
    mp_obj_t items[8];
    if (ndims > 8) ndims = 8;
    for (int i = 0; i < ndims; i++) {
        items[i] = mp_obj_new_int(coral_tpu_get_output_dim(idx, i));
    }
    return mp_obj_new_tuple(ndims, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_output_dims_obj, mod_coral_output_dims);

// coral.output_type(idx) -> int (TfLiteType enum)
static mp_obj_t mod_coral_output_type(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(coral_tpu_get_output_type(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_output_type_obj, mod_coral_output_type);

// coral.get_row(output_idx, row) -> tuple of ints
// For tensor shape [1, 1344, 5]: get_row(0, 42) returns (v0, v1, v2, v3, v4)
// Handles int8 sign extension automatically.
static mp_obj_t mod_coral_get_row(mp_obj_t oidx_obj, mp_obj_t row_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int row = mp_obj_get_int(row_obj);
    int ndims = coral_tpu_get_output_num_dims(oidx);
    int type = coral_tpu_get_output_type(oidx);
    const uint8_t* data = (const uint8_t*)coral_tpu_get_output_data(oidx);
    if (!data || ndims < 2) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    }
    // Last dimension = number of columns per row
    int cols = coral_tpu_get_output_dim(oidx, ndims - 1);
    int total_bytes = coral_tpu_get_output_size(oidx);

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
static MP_DEFINE_CONST_FUN_OBJ_2(mod_coral_get_row_obj, mod_coral_get_row);

// coral.get_val(output_idx, flat_index) -> int
// Access a single value from the flat tensor array with proper type handling.
static mp_obj_t mod_coral_get_val(mp_obj_t oidx_obj, mp_obj_t fi_obj) {
    int oidx = mp_obj_get_int(oidx_obj);
    int fi = mp_obj_get_int(fi_obj);
    int type = coral_tpu_get_output_type(oidx);
    int total_bytes = coral_tpu_get_output_size(oidx);
    const uint8_t* data = (const uint8_t*)coral_tpu_get_output_data(oidx);
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
static MP_DEFINE_CONST_FUN_OBJ_2(mod_coral_get_val_obj, mod_coral_get_val);

// ===================== Filesystem functions =====================

// coral.fs_read(path) -> bytes
// Read entire file, returns bytes object
static mp_obj_t mod_coral_fs_read(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int size = coral_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    // Allocate temporary buffer
    uint8_t* buf = m_new(uint8_t, size);
    int n = coral_fs_read(path, buf, size);
    mp_obj_t result = mp_obj_new_bytes(buf, n > 0 ? n : 0);
    m_del(uint8_t, buf, size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_read_obj, mod_coral_fs_read);

// coral.fs_read_str(path) -> str
// Read file as string
static mp_obj_t mod_coral_fs_read_str(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int size = coral_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, size);
    int n = coral_fs_read(path, buf, size);
    mp_obj_t result = mp_obj_new_str((const char*)buf, n > 0 ? n : 0);
    m_del(uint8_t, buf, size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_read_str_obj, mod_coral_fs_read_str);

// coral.fs_read_base64(path) -> str
// Read file and return base64-encoded string (printable, copy-pasteable)
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static mp_obj_t mod_coral_fs_read_base64(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int size = coral_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, size);
    int n = coral_fs_read(path, buf, size);
    if (n <= 0) {
        m_del(uint8_t, buf, size);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    // Calculate base64 output size: 4 * ceil(n/3)
    int b64_len = ((n + 2) / 3) * 4;
    char* b64 = m_new(char, b64_len + 1);
    int j = 0;
    for (int i = 0; i < n; i += 3) {
        uint32_t a = buf[i];
        uint32_t b = (i + 1 < n) ? buf[i + 1] : 0;
        uint32_t c = (i + 2 < n) ? buf[i + 2] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        b64[j++] = b64_table[(triple >> 18) & 0x3F];
        b64[j++] = b64_table[(triple >> 12) & 0x3F];
        b64[j++] = (i + 1 < n) ? b64_table[(triple >> 6) & 0x3F] : '=';
        b64[j++] = (i + 2 < n) ? b64_table[triple & 0x3F] : '=';
    }
    b64[j] = '\0';
    m_del(uint8_t, buf, size);
    // Print directly to console in 76-char lines for easy copy-paste
    for (int i = 0; i < j; i += 76) {
        int chunk = (j - i > 76) ? 76 : (j - i);
        coral_console_write(b64 + i, chunk);
        coral_console_write("\r\n", 2);
    }
    mp_obj_t result = mp_obj_new_str(b64, j);
    m_del(char, b64, b64_len + 1);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_read_base64_obj, mod_coral_fs_read_base64);

// coral.fs_write(path, data) -> bool
// Write bytes or str to file
static mp_obj_t mod_coral_fs_write(mp_obj_t path_obj, mp_obj_t data_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int ok = coral_fs_write(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_coral_fs_write_obj, mod_coral_fs_write);

// coral.fs_size(path) -> int (-1 if not found)
static mp_obj_t mod_coral_fs_size(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(coral_fs_size(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_size_obj, mod_coral_fs_size);

// coral.fs_exists(path) -> bool (checks both file and dir)
static mp_obj_t mod_coral_fs_exists(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_bool(coral_fs_file_exists(path) || coral_fs_dir_exists(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_exists_obj, mod_coral_fs_exists);

// coral.fs_remove(path) -> bool
static mp_obj_t mod_coral_fs_remove(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_bool(coral_fs_remove(path) == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_remove_obj, mod_coral_fs_remove);

// coral.fs_mkdir(path) -> bool (mkdir -p)
static mp_obj_t mod_coral_fs_mkdir(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_bool(coral_fs_makedirs(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_fs_mkdir_obj, mod_coral_fs_mkdir);

// Callback context for listdir
typedef struct {
    mp_obj_list_t* list;
} listdir_ctx_t;

static void listdir_cb(const char* name, int type, int size, void* ud) {
    listdir_ctx_t* ctx = (listdir_ctx_t*)ud;
    // Create tuple (name, type, size) - type: 1=file, 2=dir
    mp_obj_t items[3];
    items[0] = mp_obj_new_str(name, strlen(name));
    items[1] = mp_obj_new_int(type);
    items[2] = mp_obj_new_int(size);
    mp_obj_list_append(MP_OBJ_FROM_PTR(ctx->list), mp_obj_new_tuple(3, items));
}

// coral.ls(path) -> list of (name, type, size) tuples
// type: 1=file, 2=dir
static mp_obj_t mod_coral_ls(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    listdir_ctx_t ctx = { .list = result };
    int n = coral_fs_listdir(path, listdir_cb, &ctx);
    if (n < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dir not found"));
    }
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_ls_obj, mod_coral_ls);

// ===================== Camera functions =====================

// coral.cam_init(streaming=1) -> int (0=ok, <0=error)
// streaming=1: continuous mode, streaming=0: trigger mode
static mp_obj_t mod_coral_cam_init(size_t n_args, const mp_obj_t *args) {
    int streaming = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(coral_cam_init(streaming));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_coral_cam_init_obj, 0, 1, mod_coral_cam_init);

// coral.cam_stop() -> int
static mp_obj_t mod_coral_cam_stop(void) {
    return mp_obj_new_int(coral_cam_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_cam_stop_obj, mod_coral_cam_stop);

// coral.cam_jpeg(quality=75) -> bytes (JPEG data)
// Captures a frame at camera native resolution and returns JPEG
static mp_obj_t mod_coral_cam_jpeg(size_t n_args, const mp_obj_t *args) {
    int quality = (n_args > 0) ? mp_obj_get_int(args[0]) : 75;
    int w = coral_cam_get_width();
    int h = coral_cam_get_height();
    // Max JPEG buffer - typically much smaller than RGB
    int max_jpeg = w * h;  // generous upper bound
    uint8_t* buf = m_new(uint8_t, max_jpeg);
    int jpeg_size = coral_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    if (jpeg_size <= 0) {
        m_del(uint8_t, buf, max_jpeg);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }
    mp_obj_t result = mp_obj_new_bytes(buf, jpeg_size);
    m_del(uint8_t, buf, max_jpeg);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_coral_cam_jpeg_obj, 0, 1, mod_coral_cam_jpeg);

// coral.cam_to_tensor() -> int (0=ok, captures camera frame directly into TPU input tensor)
static mp_obj_t mod_coral_cam_to_tensor(void) {
    return mp_obj_new_int(coral_cam_to_tensor());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_cam_to_tensor_obj, mod_coral_cam_to_tensor);

// coral.cam_save_jpeg(path, quality=75) -> int (bytes written)
// Capture + JPEG + save to LFS in one call
static mp_obj_t mod_coral_cam_save_jpeg(size_t n_args, const mp_obj_t *args) {
    const char* path = mp_obj_str_get_str(args[0]);
    int quality = (n_args > 1) ? mp_obj_get_int(args[1]) : 75;
    int w = coral_cam_get_width();
    int h = coral_cam_get_height();
    int max_jpeg = w * h;
    uint8_t* buf = m_new(uint8_t, max_jpeg);
    int jpeg_size = coral_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    if (jpeg_size <= 0) {
        m_del(uint8_t, buf, max_jpeg);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }
    int ok = coral_fs_write(path, buf, jpeg_size);
    m_del(uint8_t, buf, max_jpeg);
    if (!ok) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("write fail"));
    }
    return mp_obj_new_int(jpeg_size);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_coral_cam_save_jpeg_obj, 1, 2, mod_coral_cam_save_jpeg);

// coral.cam_res() -> tuple (width, height) - current capture resolution
static mp_obj_t mod_coral_cam_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(coral_cam_get_width());
    items[1] = mp_obj_new_int(coral_cam_get_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_cam_res_obj, mod_coral_cam_res);

// coral.cam_set_res(w, h) -> int (0=ok, -1=invalid)
// Set capture output resolution. Max = native sensor res.
static mp_obj_t mod_coral_cam_set_res(mp_obj_t w_obj, mp_obj_t h_obj) {
    int w = mp_obj_get_int(w_obj);
    int h = mp_obj_get_int(h_obj);
    return mp_obj_new_int(coral_cam_set_res(w, h));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_coral_cam_set_res_obj, mod_coral_cam_set_res);

// coral.cam_native_res() -> tuple (width, height) - sensor native resolution
static mp_obj_t mod_coral_cam_native_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(coral_cam_get_native_width());
    items[1] = mp_obj_new_int(coral_cam_get_native_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_coral_cam_native_res_obj, mod_coral_cam_native_res);

// coral.cam_switch(id) -> int (0=ok). id: 0=front, 1=back
static mp_obj_t mod_coral_cam_switch(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    return mp_obj_new_int(coral_cam_switch(id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_cam_switch_obj, mod_coral_cam_switch);

// coral.usb_drive(on) -> int (1=enabled, 0=disabled)
// Enable/disable USB mass storage. 1=drive visible to host, 0=ejected.
static mp_obj_t mod_coral_usb_drive(mp_obj_t on_obj) {
    int on = mp_obj_get_int(on_obj);
    return mp_obj_new_int(coral_usb_drive_set(on));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_coral_usb_drive_obj, mod_coral_usb_drive);

// Module globals table
static const mp_rom_map_elem_t coral_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_coral) },
    { MP_ROM_QSTR(MP_QSTR_led_on),        MP_ROM_PTR(&mod_coral_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),       MP_ROM_PTR(&mod_coral_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms),      MP_ROM_PTR(&mod_coral_sleep_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_ms),      MP_ROM_PTR(&mod_coral_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke),        MP_ROM_PTR(&mod_coral_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_ready),      MP_ROM_PTR(&mod_coral_is_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_num_outputs),   MP_ROM_PTR(&mod_coral_num_outputs_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size),   MP_ROM_PTR(&mod_coral_output_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_output),    MP_ROM_PTR(&mod_coral_get_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims),   MP_ROM_PTR(&mod_coral_output_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type),   MP_ROM_PTR(&mod_coral_output_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_row),       MP_ROM_PTR(&mod_coral_get_row_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_val),       MP_ROM_PTR(&mod_coral_get_val_obj) },
    // Filesystem
    { MP_ROM_QSTR(MP_QSTR_fs_read),       MP_ROM_PTR(&mod_coral_fs_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_read_str),   MP_ROM_PTR(&mod_coral_fs_read_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_write),      MP_ROM_PTR(&mod_coral_fs_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_size),       MP_ROM_PTR(&mod_coral_fs_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_exists),     MP_ROM_PTR(&mod_coral_fs_exists_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_remove),     MP_ROM_PTR(&mod_coral_fs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_mkdir),      MP_ROM_PTR(&mod_coral_fs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_ls),            MP_ROM_PTR(&mod_coral_ls_obj) },
    // Camera
    { MP_ROM_QSTR(MP_QSTR_cam_init),       MP_ROM_PTR(&mod_coral_cam_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_stop),       MP_ROM_PTR(&mod_coral_cam_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_jpeg),       MP_ROM_PTR(&mod_coral_cam_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_to_tensor),  MP_ROM_PTR(&mod_coral_cam_to_tensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_save_jpeg),  MP_ROM_PTR(&mod_coral_cam_save_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_res),        MP_ROM_PTR(&mod_coral_cam_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_set_res),    MP_ROM_PTR(&mod_coral_cam_set_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_native_res), MP_ROM_PTR(&mod_coral_cam_native_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_cam_switch),     MP_ROM_PTR(&mod_coral_cam_switch_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_read_base64), MP_ROM_PTR(&mod_coral_fs_read_base64_obj) },
    // USB
    { MP_ROM_QSTR(MP_QSTR_usb_drive),      MP_ROM_PTR(&mod_coral_usb_drive_obj) },
};
static MP_DEFINE_CONST_DICT(coral_module_globals, coral_module_globals_table);

const mp_obj_module_t mp_module_coral = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&coral_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_coral, mp_module_coral);
