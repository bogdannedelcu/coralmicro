// Custom MicroPython C module: 'sentai'
// Provides LED control, sleep, and EdgeTPU inference bridge

#include "py/runtime.h"
#include "py/obj.h"
#include "py/gc.h"
#include "py/lexer.h"
#include "py/parse.h"
#include "py/compile.h"
#include <string.h>
#include "sentai_mesh.h"
#include <stdlib.h>

// FreeRTOS - for sentai.tasks(), coral.heap() (guarded for QSTR generation pass)
#ifndef NO_QSTR
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/portable.h"
#endif

// Implemented in C++ (modsentai_hal.cc), exposed as extern "C"
extern void sentai_led_set(int on);
extern void sentai_sleep_ms(uint32_t ms);
extern uint32_t sentai_ticks_ms(void);

// TPU bridge - implemented in sentai_runtime.cc
extern int sentai_tpu_invoke(void);
extern int sentai_tpu_is_ready(void);
extern int sentai_tpu_num_outputs(void);
extern int sentai_tpu_get_output_size(int idx);
extern const void* sentai_tpu_get_output_data(int idx);
extern int sentai_tpu_get_output_num_dims(int idx);
extern int sentai_tpu_get_output_dim(int idx, int dim);
extern int sentai_tpu_get_output_type(int idx);

// Filesystem bridge - implemented in modsentai_hal.cc
extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
extern int sentai_fs_size(const char* path);
extern int sentai_fs_file_exists(const char* path);
extern int sentai_fs_dir_exists(const char* path);
extern int sentai_fs_write(const char* path, const uint8_t* buf, int size);
extern int sentai_fs_remove(const char* path);
extern int sentai_fs_makedirs(const char* path);
extern int sentai_fs_listdir(const char* path,
                     void (*callback)(const char* name, int type, int size, void* ud),
                     void* user_data);

// Camera bridge - implemented in sentai_runtime.cc
extern int sentai_cam_init(int streaming);
extern int sentai_cam_stop(void);
extern int sentai_cam_capture_rgb(uint8_t* buf, int width, int height);
extern int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                  int width, int height, int quality);
extern int sentai_cam_to_tensor(void);
extern int sentai_cam_get_width(void);
extern int sentai_cam_get_height(void);
extern int sentai_cam_set_res(int w, int h);
extern int sentai_cam_get_native_width(void);
extern int sentai_cam_get_native_height(void);
extern int sentai_cam_switch(int id);
extern int sentai_cam_rotate(int cam_id, int degrees);

// Model/image loading - implemented in sentai_runtime.cc
extern int sentai_load_model(const char* path);
extern int sentai_load_image(const char* path);
extern int sentai_save_output(const char* path);

// USB drive bridge - implemented in main_freertos_m7.cc
extern int sentai_usb_drive_set(int on);
extern int sentai_usb_drive_get(void);

// USB serial bridge - implemented in modsentai_hal.cc
extern int sentai_usb_serial_open(void);
extern void sentai_usb_serial_close(void);
extern int sentai_usb_serial_is_open(void);
extern int sentai_usb_serial_write(const uint8_t* buf, int size);
extern int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern int sentai_usb_serial_available(void);

// Console REPL target - implemented in modsentai_hal.cc
extern int sentai_console_set_target(int target);
extern int sentai_console_get_target(void);

// UART serial bridge - implemented in modsentai_hal.cc
extern int sentai_uart_serial_open(void);
extern void sentai_uart_serial_close(void);
extern int sentai_uart_serial_is_open(void);
extern int sentai_uart_serial_write(const uint8_t* buf, int size);
extern int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern int sentai_uart_serial_available(void);
extern void sentai_uart_set_baudrate(uint32_t baudrate);
extern void sentai_uart_restore_baudrate(void);

// Mesh bridge (sentai_mesh.cc)
extern int sentai_mesh_init(uint32_t baudrate);
extern int sentai_mesh_stop(void);
extern int sentai_mesh_send_text(const char* text, uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_send_detection(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    const uint8_t* embedding, uint32_t embed_len, uint32_t embed_crc8,
    uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_send_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_text_available(void);
extern int sentai_mesh_vision_available(void);
extern int sentai_mesh_receive_text(mesh_rx_msg_t* msg);
extern int sentai_mesh_receive_text_wait(mesh_rx_msg_t* msg, int timeout_ms);
extern int sentai_mesh_receive_vision(mesh_rx_vision_t* msg);
extern int sentai_mesh_receive_vision_wait(mesh_rx_vision_t* msg, int timeout_ms);
extern int sentai_mesh_request_config(uint32_t config_id);
extern int sentai_mesh_is_running(void);
extern uint32_t sentai_mesh_my_node_num(void);

// MAVLink link bridge (sentai_link.cc)
// link_rx_msg_t wraps mavlink_message_t; we use accessor functions to read fields.
typedef struct { uint8_t _opaque[296]; } link_rx_msg_t;  // sizeof(mavlink_message_t) padded
extern int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid);
extern int sentai_link_stop(void);
extern int sentai_link_is_running(void);
extern int sentai_link_available(void);
extern int sentai_link_receive(link_rx_msg_t* msg);
extern int sentai_link_receive_wait(link_rx_msg_t* msg, int timeout_ms);
extern int sentai_link_send_heartbeat(uint8_t type);
extern int sentai_link_send_statustext(uint8_t severity, const char* text);
extern int sentai_link_send_vision(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    const uint8_t* embedding, uint32_t embed_len, uint32_t embed_crc8,
    uint8_t severity);
extern int sentai_link_send_vision_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    uint8_t severity);
extern int sentai_link_send_command_long(
    uint8_t target_sys, uint8_t target_comp,
    uint16_t command, uint8_t confirmation,
    int32_t param1, int32_t param2, int32_t param3, int32_t param4,
    int32_t param5, int32_t param6, int32_t param7);
// Accessor functions (implemented in sentai_link.cc, safe across compilers)
extern uint32_t sentai_link_rx_msgid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_sysid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_compid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_seq(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_len(const link_rx_msg_t* m);

// Help file reading from system flash partition
extern int sentai_help_read(char* buf, int max_size);

// Check USB drive state; raise OSError if active.
// Filesystem is unmounted while USB MSC is active — Python must not
// access flash until the user calls sentai.usb.drive(0).
static void _fs_check_usb(void) {
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("flash busy: call sentai.usb.drive(0) first"));
    }
}

// sentai.led_on()
static mp_obj_t mod_sentai_led_on(void) {
    sentai_led_set(1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_led_on_obj, mod_sentai_led_on);

// sentai.led_off()
static mp_obj_t mod_sentai_led_off(void) {
    sentai_led_set(0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_led_off_obj, mod_sentai_led_off);

// sentai.sleep_ms(ms)
static mp_obj_t mod_sentai_sleep_ms(mp_obj_t ms_obj) {
    sentai_sleep_ms(mp_obj_get_int(ms_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_sleep_ms_obj, mod_sentai_sleep_ms);

// sentai.ticks_ms()
static mp_obj_t mod_sentai_ticks_ms(void) {
    return mp_obj_new_int(sentai_ticks_ms());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_ticks_ms_obj, mod_sentai_ticks_ms);

// sentai.invoke() -> int (inference time in ms, -1 if not ready, -2 if invoke failed)
static mp_obj_t mod_sentai_invoke(void) {
    int result = sentai_tpu_invoke();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_invoke_obj, mod_sentai_invoke);

// sentai.is_ready() -> bool
static mp_obj_t mod_sentai_is_ready(void) {
    return mp_obj_new_bool(sentai_tpu_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_is_ready_obj, mod_sentai_is_ready);

// sentai.num_outputs() -> int
static mp_obj_t mod_sentai_num_outputs(void) {
    return mp_obj_new_int(sentai_tpu_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_num_outputs_obj, mod_sentai_num_outputs);

// sentai.output_size(idx) -> int (bytes)
static mp_obj_t mod_sentai_output_size(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tpu_get_output_size(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_size_obj, mod_sentai_output_size);

// sentai.get_output(idx) -> bytes (raw tensor data, integers)
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

// sentai.output_dims(idx) -> tuple (dim0, dim1, ...) 
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

// sentai.output_type(idx) -> int (TfLiteType enum)
static mp_obj_t mod_sentai_output_type(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    return mp_obj_new_int(sentai_tpu_get_output_type(idx));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_output_type_obj, mod_sentai_output_type);

// sentai.get_row(output_idx, row) -> tuple of ints
// For tensor shape [1, 1344, 5]: get_row(0, 42) returns (v0, v1, v2, v3, v4)
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

// sentai.get_val(output_idx, flat_index) -> int
// Access a single value from the flat tensor array with proper type handling.
static mp_obj_t mod_sentai_get_val(mp_obj_t oidx_obj, mp_obj_t fi_obj) {
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
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_get_val_obj, mod_sentai_get_val);

// ===================== Filesystem functions =====================

// sentai.fs_read(path) -> bytes
// Read entire file, returns bytes object
static mp_obj_t mod_sentai_fs_read(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, size);
    int n = sentai_fs_read(path, buf, size);
    mp_obj_t result = mp_obj_new_bytes(buf, n > 0 ? n : 0);
    m_del(uint8_t, buf, size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_obj, mod_sentai_fs_read);

// sentai.fs_read_str(path) -> str
// Read file as string
static mp_obj_t mod_sentai_fs_read_str(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, size);
    int n = sentai_fs_read(path, buf, size);
    mp_obj_t result = mp_obj_new_str((const char*)buf, n > 0 ? n : 0);
    m_del(uint8_t, buf, size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_str_obj, mod_sentai_fs_read_str);

// sentai.fs_read_base64(path) -> str
// Read file and return base64-encoded string (printable, copy-pasteable)
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static mp_obj_t mod_sentai_fs_read_base64(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, size);
    int n = sentai_fs_read(path, buf, size);
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
        sentai_console_write(b64 + i, chunk);
        sentai_console_write("\r\n", 2);
    }
    mp_obj_t result = mp_obj_new_str(b64, j);
    m_del(char, b64, b64_len + 1);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_base64_obj, mod_sentai_fs_read_base64);

// sentai.fs_write(path, data) -> bool
// Write bytes or str to file
static mp_obj_t mod_sentai_fs_write(mp_obj_t path_obj, mp_obj_t data_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int ok = sentai_fs_write(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_fs_write_obj, mod_sentai_fs_write);

// sentai.fs_size(path) -> int (-1 if not found)
static mp_obj_t mod_sentai_fs_size(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int result = sentai_fs_size(path);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_size_obj, mod_sentai_fs_size);

// sentai.fs_exists(path) -> bool (checks both file and dir)
static mp_obj_t mod_sentai_fs_exists(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int exists = sentai_fs_file_exists(path) || sentai_fs_dir_exists(path);
    return mp_obj_new_bool(exists);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_exists_obj, mod_sentai_fs_exists);

// sentai.fs_format() -> bool  (force reformat user partition)
// Special: if USB drive is active, disable it first (format needs exclusive access).
extern int sentai_fs_format(void);
static mp_obj_t mod_sentai_fs_format(void) {
    if (sentai_usb_drive_get()) sentai_usb_drive_set(0);
    int rc = sentai_fs_format();
    return mp_obj_new_bool(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_fs_format_obj, mod_sentai_fs_format);

// sentai.fs_remove(path) -> bool
static mp_obj_t mod_sentai_fs_remove(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_fs_remove(path);
    return mp_obj_new_bool(rc == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_remove_obj, mod_sentai_fs_remove);

// sentai.fs_mkdir(path) -> bool (mkdir -p)
static mp_obj_t mod_sentai_fs_mkdir(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_fs_makedirs(path);
    return mp_obj_new_bool(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_mkdir_obj, mod_sentai_fs_mkdir);

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

// sentai.ls(path) -> list of (name, type, size) tuples
// type: 1=file, 2=dir
static mp_obj_t mod_sentai_ls(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    listdir_ctx_t ctx = { .list = result };
    int n = sentai_fs_listdir(path, listdir_cb, &ctx);
    if (n < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dir not found"));
    }
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_ls_obj, mod_sentai_ls);

// ===================== Camera functions =====================

// sentai.cam_init(streaming=1) -> int (0=ok, <0=error)
// streaming=1: continuous mode, streaming=0: trigger mode
static mp_obj_t mod_sentai_cam_init(size_t n_args, const mp_obj_t *args) {
    int streaming = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(sentai_cam_init(streaming));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_init_obj, 0, 1, mod_sentai_cam_init);

// sentai.cam_stop() -> int
static mp_obj_t mod_sentai_cam_stop(void) {
    return mp_obj_new_int(sentai_cam_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_stop_obj, mod_sentai_cam_stop);

// sentai.cam_jpeg(quality=75) -> bytes (JPEG data)
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

// sentai.cam_to_tensor() -> int (0=ok, captures camera frame directly into TPU input tensor)
static mp_obj_t mod_sentai_cam_to_tensor(void) {
    return mp_obj_new_int(sentai_cam_to_tensor());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_to_tensor_obj, mod_sentai_cam_to_tensor);

// sentai.cam_save_jpeg(path, quality=75) -> int (bytes written)
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
    int jpeg_size = sentai_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    if (jpeg_size <= 0) {
        free(buf);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }
    int ok = sentai_fs_write(path, buf, jpeg_size);
    free(buf);
    if (!ok) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("write fail"));
    }
    return mp_obj_new_int(jpeg_size);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_save_jpeg_obj, 1, 2, mod_sentai_cam_save_jpeg);

// sentai.cam_res() -> tuple (width, height) - current capture resolution
static mp_obj_t mod_sentai_cam_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_width());
    items[1] = mp_obj_new_int(sentai_cam_get_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_res_obj, mod_sentai_cam_res);

// sentai.cam_set_res(w, h) -> int (0=ok, -1=invalid)
// Set capture output resolution. Max = native sensor res.
static mp_obj_t mod_sentai_cam_set_res(mp_obj_t w_obj, mp_obj_t h_obj) {
    int w = mp_obj_get_int(w_obj);
    int h = mp_obj_get_int(h_obj);
    return mp_obj_new_int(sentai_cam_set_res(w, h));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_set_res_obj, mod_sentai_cam_set_res);

// sentai.cam_native_res() -> tuple (width, height) - sensor native resolution
static mp_obj_t mod_sentai_cam_native_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_native_width());
    items[1] = mp_obj_new_int(sentai_cam_get_native_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_native_res_obj, mod_sentai_cam_native_res);

// sentai.cam_switch(id) -> int (0=ok). id: 0=front, 1=back
static mp_obj_t mod_sentai_cam_switch(mp_obj_t id_obj) {
    int id = mp_obj_get_int(id_obj);
    return mp_obj_new_int(sentai_cam_switch(id));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_switch_obj, mod_sentai_cam_switch);

// sentai.camera.rotate(cam_id, degrees) -> int (0=ok)
// cam_id: 0=front, 1=back.  degrees: 0, 90, 180, 270.
static mp_obj_t mod_sentai_cam_rotate(mp_obj_t cam_obj, mp_obj_t deg_obj) {
    int cam_id = mp_obj_get_int(cam_obj);
    int degrees = mp_obj_get_int(deg_obj);
    return mp_obj_new_int(sentai_cam_rotate(cam_id, degrees));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_rotate_obj, mod_sentai_cam_rotate);

// sentai.usb_drive(on) -> int (1=enabled, 0=disabled)
// Enable/disable USB mass storage. 1=drive visible to host, 0=ejected.
// Auto-switches REPL to UART when mounting drive.
static mp_obj_t mod_sentai_usb_drive(mp_obj_t on_obj) {
    int on = mp_obj_get_int(on_obj);
    if (on) {
        // Print mount command hint before switching console away from USB
        printf("\r\n*****\r\n"
               "sudo littlefs-fuse "
               "  --block_size=131072 "
               "  --read_size=2048 "
               "  --prog_size=2048 "
               "  --block_count=448 "
               "  --cache_size=2048 "
               "  --lookahead_size=2048 "
               "  -o allow_other "
               "  /dev/sda /mnt/coral\r\n"
               "*****\r\n"
               "Switch to Linux\r\n");
        // Let the TX task flush the message to USB before switching away
        vTaskDelay(pdMS_TO_TICKS(100));
        // Auto-switch REPL to UART when mounting USB drive
        sentai_console_set_target(1);  // 1 = UART
    }
    return mp_obj_new_int(sentai_usb_drive_set(on));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_usb_drive_obj, mod_sentai_usb_drive);

// ===================== USB Serial functions =====================

// sentai.usb.serial_open() -> bool
// Opens USB CDC ACM port for Python serial I/O.
// Requires REPL on UART. Fails if USB drive is active.
static mp_obj_t mod_sentai_usb_serial_open(void) {
    if (sentai_console_get_target() != 1) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on UART: call sentai.console('uart')"));
    }
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("USB drive active: call sentai.usb.drive(0) first"));
    }
    return mp_obj_new_bool(sentai_usb_serial_open());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_open_obj, mod_sentai_usb_serial_open);

// sentai.usb.serial_close()
// Returns USB CDC ACM to normal console mode.
static mp_obj_t mod_sentai_usb_serial_close(void) {
    sentai_usb_serial_close();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_close_obj, mod_sentai_usb_serial_close);

// sentai.usb.serial_write(data) -> int (bytes written, -1 on error)
// data: str or bytes
static mp_obj_t mod_sentai_usb_serial_write(mp_obj_t data_obj) {
    if (!sentai_usb_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("serial not open"));
    }
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int n = sentai_usb_serial_write((const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(n);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_usb_serial_write_obj, mod_sentai_usb_serial_write);

// sentai.usb.serial_read(max_bytes=256, timeout_ms=1000) -> bytes
// Returns up to max_bytes of data received from USB host.
// timeout_ms: -1=block forever, 0=non-blocking, >0=wait up to N ms
static mp_obj_t mod_sentai_usb_serial_read(size_t n_args, const mp_obj_t *args) {
    if (!sentai_usb_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("serial not open"));
    }
    int max_bytes = (n_args > 0) ? mp_obj_get_int(args[0]) : 256;
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (max_bytes <= 0 || max_bytes > 2048) max_bytes = 256;
    uint8_t* buf = m_new(uint8_t, max_bytes);
    int n = sentai_usb_serial_read(buf, max_bytes, timeout_ms);
    if (n <= 0) {
        m_del(uint8_t, buf, max_bytes);
        return mp_obj_new_bytes((const byte*)"", 0);
    }
    mp_obj_t result = mp_obj_new_bytes(buf, n);
    m_del(uint8_t, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_usb_serial_read_obj, 0, 2, mod_sentai_usb_serial_read);

// sentai.usb.serial_available() -> int (bytes waiting in RX buffer)
static mp_obj_t mod_sentai_usb_serial_available(void) {
    return mp_obj_new_int(sentai_usb_serial_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_available_obj, mod_sentai_usb_serial_available);

// sentai.load_model(path) - Load a TFLite model from flash
static mp_obj_t mod_sentai_load_model(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_load_model(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_load_model_obj, mod_sentai_load_model);

// sentai.load_image(path) - Load an image into input tensor
static mp_obj_t mod_sentai_load_image(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_load_image(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_load_image_obj, mod_sentai_load_image);

// sentai.save_output(path) - Save all output tensors to CSV file
static mp_obj_t mod_sentai_save_output(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = sentai_save_output(path);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_save_output_obj, mod_sentai_save_output);

// ===================== Script execution =====================

// sentai.run(path) - Read and execute a .py file from flash
// Unlike import, this always re-executes (no caching).
// Variables defined in the script are available in the REPL afterwards.
static mp_obj_t mod_sentai_run(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);

    // Read file from LFS
    _fs_check_usb();
    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    byte* buf = m_new(byte, size + 1);
    int n = sentai_fs_read(path, (uint8_t*)buf, size);

    if (n <= 0) {
        m_del(byte, buf, size + 1);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    buf[n] = '\0';

    // Compile and execute in current context (is_repl=true so globals are shared)
    mp_lexer_t *lex = mp_lexer_new_from_str_len(
        qstr_from_str(path), (const char*)buf, n, size + 1);
    qstr source_name = lex->source_name;
    mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
    mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
    mp_call_function_0(module_fun);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_run_obj, mod_sentai_run);

// ===================== FreeRTOS task listing =====================

// sentai.tasks() -> list of (name, state, priority, stack_hwm) tuples
// Lists all FreeRTOS tasks currently in the system.
// state: "running", "ready", "blocked", "suspended", "deleted"
// stack_hwm: minimum free stack (words) since task creation (high water mark)
static mp_obj_t mod_sentai_tasks(void) {
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    uint32_t total_runtime;
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, &total_runtime);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));

    for (UBaseType_t i = 0; i < n; i++) {
        mp_obj_t items[4];
        items[0] = mp_obj_new_str(task_buf[i].pcTaskName,
                                  strlen(task_buf[i].pcTaskName));
        const char* state_str;
        switch (task_buf[i].eCurrentState) {
            case eRunning:   state_str = "running"; break;
            case eReady:     state_str = "ready"; break;
            case eBlocked:   state_str = "blocked"; break;
            case eSuspended: state_str = "suspended"; break;
            case eDeleted:   state_str = "deleted"; break;
            default:         state_str = "?"; break;
        }
        items[1] = mp_obj_new_str(state_str, strlen(state_str));
        items[2] = mp_obj_new_int(task_buf[i].uxCurrentPriority);
        items[3] = mp_obj_new_int(task_buf[i].usStackHighWaterMark);
        mp_obj_list_append(MP_OBJ_FROM_PTR(result), mp_obj_new_tuple(4, items));
    }
    #undef MAX_TASKS
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_tasks_obj, mod_sentai_tasks);

// ===================== System info: heap =====================

// sentai.heap() -> dict with FreeRTOS heap stats + MicroPython GC stats
static mp_obj_t mod_sentai_heap(void) {
    // MicroPython GC info
    gc_info_t gc;
    gc_info(&gc);

    mp_obj_dict_t *d = MP_OBJ_TO_PTR(mp_obj_new_dict(7));

    // FreeRTOS/newlib heap (xPortGetFreeHeapSize wraps mallinfo + sbrk remainder)
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("rtos_free", 9),
        mp_obj_new_int(xPortGetFreeHeapSize()));

    // MicroPython GC heap
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_total", 8),
        mp_obj_new_int(gc.total));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_used", 7),
        mp_obj_new_int(gc.used));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_free", 7),
        mp_obj_new_int(gc.free));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_max_free", 11),
        mp_obj_new_int(gc.max_free));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_max_block", 12),
        mp_obj_new_int(gc.max_block));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_num_1block", 13),
        mp_obj_new_int(gc.num_1block));

    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_heap_obj, mod_sentai_heap);

// ===================== System info: CPU usage =====================

// sentai.cpu() -> list of (name, cpu_percent) tuples, sorted by CPU% descending
// Uses FreeRTOS runtime stats (configGENERATE_RUN_TIME_STATS=1)
static mp_obj_t mod_sentai_cpu(void) {
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    uint32_t total_runtime;
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, &total_runtime);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));

    if (total_runtime == 0) total_runtime = 1;  // avoid div by zero

    for (UBaseType_t i = 0; i < n; i++) {
        mp_obj_t items[2];
        items[0] = mp_obj_new_str(task_buf[i].pcTaskName,
                                  strlen(task_buf[i].pcTaskName));
        uint32_t pct = (task_buf[i].ulRunTimeCounter * 100) / total_runtime;
        items[1] = mp_obj_new_int(pct);
        mp_obj_list_append(MP_OBJ_FROM_PTR(result), mp_obj_new_tuple(2, items));
    }
    #undef MAX_TASKS
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cpu_obj, mod_sentai_cpu);

// ===================== System info: uptime =====================

// sentai.uptime() -> int (seconds since boot)
static mp_obj_t mod_sentai_uptime(void) {
    return mp_obj_new_int(xTaskGetTickCount() / configTICK_RATE_HZ);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uptime_obj, mod_sentai_uptime);

// ===================== Help system (flash-based) =====================

// Search for "[section_name]" in buf and return pointers to content
static int help_find_section(const char* buf, int len, const char* section,
                             const char** out_start, const char** out_end) {
    char marker[32];
    int mlen = snprintf(marker, sizeof(marker), "[%s]", section);
    const char* buf_end = buf + len;

    for (const char* p = buf; p < buf_end - mlen; p++) {
        if ((p == buf || *(p-1) == '\n') && memcmp(p, marker, mlen) == 0) {
            const char* start = p + mlen;
            while (start < buf_end && *start != '\n') start++;
            if (start < buf_end) start++;
            const char* end = start;
            while (end < buf_end) {
                if (*end == '[' && (end == start || *(end-1) == '\n')) break;
                end++;
            }
            *out_start = start;
            *out_end = end;
            return 1;
        }
    }
    return 0;
}

// Print text line-by-line with \r\n, using mp_print_str for each line.
// This avoids the cooked-output path that fragments USB CDC packets.
static void help_print(const char *text, int len) {
    const char *p = text;
    const char *end = text + len;
    char line[120];
    while (p < end) {
        const char *nl = p;
        while (nl < end && *nl != '\n') nl++;
        int llen = nl - p;
        if (llen > (int)sizeof(line) - 3) llen = (int)sizeof(line) - 3;
        memcpy(line, p, llen);
        line[llen] = '\r';
        line[llen+1] = '\n';
        line[llen+2] = '\0';
        mp_print_str(MP_PYTHON_PRINTER, line);
        p = (nl < end) ? nl + 1 : end;
    }
}

// sentai.help([topic]) — read and print help from flash
// topic: "io", "rtos", "tpu", "fs", "camera", "usb", "uart", "console", "mesh", "serial", "all"
static mp_obj_t mod_sentai_help(size_t n_args, const mp_obj_t *args) {
    const char* topic = (n_args > 0) ? mp_obj_str_get_str(args[0]) : NULL;

    #define HELP_BUF_SIZE 12288
    char* hbuf = (char*)malloc(HELP_BUF_SIZE);
    if (!hbuf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("help buf alloc"));
    }
    int n = sentai_help_read(hbuf, HELP_BUF_SIZE);

    if (n <= 0) {
        free(hbuf);
        mp_print_str(MP_PYTHON_PRINTER, "Help file not found on flash.\r\n");
        return mp_const_none;
    }

    if (topic == NULL) {
        const char *start, *end;
        if (help_find_section(hbuf, n, "overview", &start, &end)) {
            help_print(start, end - start);
        }
    } else if (strcmp(topic, "all") == 0) {
        help_print(hbuf, n);
    } else {
        const char *start, *end;
        if (help_find_section(hbuf, n, topic, &start, &end)) {
            help_print(start, end - start);
        } else {
            mp_print_str(MP_PYTHON_PRINTER,
                "Unknown topic. Available: io, rtos, tpu, fs, camera, usb, uart, console, mesh, link, serial, all\r\n");
        }
    }

    free(hbuf);
    #undef HELP_BUF_SIZE
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_help_obj, 0, 1, mod_sentai_help);

// ===================== Console REPL target =====================

// sentai.console([target]) -> str
// Get or set the REPL console target ("usb" or "uart").
static mp_obj_t mod_sentai_console(size_t n_args, const mp_obj_t *args) {
    if (n_args > 0) {
        const char* target = mp_obj_str_get_str(args[0]);
        if (strcmp(target, "usb") == 0) {
            sentai_console_set_target(0);
        } else if (strcmp(target, "uart") == 0) {
            sentai_console_set_target(1);
        } else {
            mp_raise_ValueError(MP_ERROR_TEXT("use 'usb' or 'uart'"));
        }
    }
    int t = sentai_console_get_target();
    return mp_obj_new_str(t == 0 ? "usb" : "uart", t == 0 ? 3 : 4);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_console_obj, 0, 1, mod_sentai_console);

// ===================== UART Serial functions =====================

// sentai.uart.open(baudrate=38400) -> bool
static mp_obj_t mod_sentai_uart_serial_open(size_t n_args, const mp_obj_t *args) {
    int baudrate = (n_args > 0) ? mp_obj_get_int(args[0]) : 38400;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    if (baudrate != 115200 && baudrate > 0) {
        sentai_uart_set_baudrate((uint32_t)baudrate);
    }
    return mp_obj_new_bool(sentai_uart_serial_open());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_uart_serial_open_obj, 0, 1, mod_sentai_uart_serial_open);

// sentai.uart.serial_close()
static mp_obj_t mod_sentai_uart_serial_close(void) {
    sentai_uart_restore_baudrate();
    sentai_uart_serial_close();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uart_serial_close_obj, mod_sentai_uart_serial_close);

// sentai.uart.serial_write(data) -> int
static mp_obj_t mod_sentai_uart_serial_write(mp_obj_t data_obj) {
    if (!sentai_uart_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART serial not open"));
    }
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int n = sentai_uart_serial_write((const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(n);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_uart_serial_write_obj, mod_sentai_uart_serial_write);

// sentai.uart.serial_read(max_bytes=256, timeout_ms=1000) -> bytes
static mp_obj_t mod_sentai_uart_serial_read(size_t n_args, const mp_obj_t *args) {
    if (!sentai_uart_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART serial not open"));
    }
    int max_bytes = (n_args > 0) ? mp_obj_get_int(args[0]) : 256;
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (max_bytes <= 0 || max_bytes > 2048) max_bytes = 256;
    uint8_t* buf = m_new(uint8_t, max_bytes);
    int n = sentai_uart_serial_read(buf, max_bytes, timeout_ms);
    if (n <= 0) {
        m_del(uint8_t, buf, max_bytes);
        return mp_obj_new_bytes((const byte*)"", 0);
    }
    mp_obj_t result = mp_obj_new_bytes(buf, n);
    m_del(uint8_t, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_uart_serial_read_obj, 0, 2, mod_sentai_uart_serial_read);

// sentai.uart.serial_available() -> int
static mp_obj_t mod_sentai_uart_serial_available(void) {
    return mp_obj_new_int(sentai_uart_serial_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uart_serial_available_obj, mod_sentai_uart_serial_available);

// ===================== Mesh functions =====================

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

// ===================== MAVLink link functions =====================

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
// params are integer x1000 (divided to float on wire)
static mp_obj_t mod_sentai_link_command(size_t n_args, const mp_obj_t *args) {
    uint8_t tsys = (uint8_t)mp_obj_get_int(args[0]);
    uint8_t tcomp = (uint8_t)mp_obj_get_int(args[1]);
    uint16_t cmd = (uint16_t)mp_obj_get_int(args[2]);
    uint8_t conf = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 0;
    int32_t p1 = (n_args > 4) ? (int32_t)mp_obj_get_int(args[4]) : 0;
    int32_t p2 = (n_args > 5) ? (int32_t)mp_obj_get_int(args[5]) : 0;
    int32_t p3 = (n_args > 6) ? (int32_t)mp_obj_get_int(args[6]) : 0;
    int32_t p4 = (n_args > 7) ? (int32_t)mp_obj_get_int(args[7]) : 0;
    int32_t p5 = (n_args > 8) ? (int32_t)mp_obj_get_int(args[8]) : 0;
    int32_t p6 = (n_args > 9) ? (int32_t)mp_obj_get_int(args[9]) : 0;
    int32_t p7 = (n_args > 10) ? (int32_t)mp_obj_get_int(args[10]) : 0;
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
// Returns dict: {msgid, sysid, compid, seq, len}
static mp_obj_t mod_sentai_link_receive(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 0;
    link_rx_msg_t rx;
    int got;
    if (timeout_ms == 0)
        got = sentai_link_receive(&rx);
    else
        got = sentai_link_receive_wait(&rx, timeout_ms);
    if (!got) return mp_const_none;
    mp_obj_dict_t *d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_msgid), mp_obj_new_int_from_uint(sentai_link_rx_msgid(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sysid), mp_obj_new_int(sentai_link_rx_sysid(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_compid), mp_obj_new_int(sentai_link_rx_compid(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_seq), mp_obj_new_int(sentai_link_rx_seq(&rx)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_len), mp_obj_new_int(sentai_link_rx_len(&rx)));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_link_receive_obj, 0, 1, mod_sentai_link_receive);

// =====================================================================
// Sub-module definitions: sentai.io, sentai.rtos, sentai.tpu,
//                         sentai.fs, sentai.camera, sentai.usb,
//                         sentai.uart, sentai.mesh, sentai.link
// =====================================================================

// ============== sentai.io — LED / GPIO ==============
static const mp_rom_map_elem_t sentai_io_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io) },
    { MP_ROM_QSTR(MP_QSTR_led_on),   MP_ROM_PTR(&mod_sentai_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),  MP_ROM_PTR(&mod_sentai_led_off_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_io_globals, sentai_io_globals_table);
static const mp_obj_module_t sentai_io_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_io_globals,
};

// ============== sentai.rtos — FreeRTOS system ==============
static const mp_rom_map_elem_t sentai_rtos_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_rtos) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms),  MP_ROM_PTR(&mod_sentai_sleep_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_ms),  MP_ROM_PTR(&mod_sentai_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_tasks),     MP_ROM_PTR(&mod_sentai_tasks_obj) },
    { MP_ROM_QSTR(MP_QSTR_heap),      MP_ROM_PTR(&mod_sentai_heap_obj) },
    { MP_ROM_QSTR(MP_QSTR_cpu),       MP_ROM_PTR(&mod_sentai_cpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_uptime),    MP_ROM_PTR(&mod_sentai_uptime_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_rtos_globals, sentai_rtos_globals_table);
static const mp_obj_module_t sentai_rtos_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_rtos_globals,
};

// ============== sentai.tpu — EdgeTPU inference ==============
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
    { MP_ROM_QSTR(MP_QSTR_val),         MP_ROM_PTR(&mod_sentai_get_val_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_output), MP_ROM_PTR(&mod_sentai_save_output_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tpu_globals, sentai_tpu_globals_table);
static const mp_obj_module_t sentai_tpu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_tpu_globals,
};

// ============== sentai.fs — Filesystem (LittleFS) ==============
static const mp_rom_map_elem_t sentai_fs_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_fs) },
    { MP_ROM_QSTR(MP_QSTR_read),        MP_ROM_PTR(&mod_sentai_fs_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_str),    MP_ROM_PTR(&mod_sentai_fs_read_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_base64), MP_ROM_PTR(&mod_sentai_fs_read_base64_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),       MP_ROM_PTR(&mod_sentai_fs_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_size),        MP_ROM_PTR(&mod_sentai_fs_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_exists),      MP_ROM_PTR(&mod_sentai_fs_exists_obj) },
    { MP_ROM_QSTR(MP_QSTR_format),      MP_ROM_PTR(&mod_sentai_fs_format_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),      MP_ROM_PTR(&mod_sentai_fs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),       MP_ROM_PTR(&mod_sentai_fs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_ls),          MP_ROM_PTR(&mod_sentai_ls_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_fs_globals, sentai_fs_globals_table);
static const mp_obj_module_t sentai_fs_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_fs_globals,
};

// ============== sentai.camera — Camera ==============
static const mp_rom_map_elem_t sentai_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_cam_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mod_sentai_cam_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_jpeg),       MP_ROM_PTR(&mod_sentai_cam_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_tensor),  MP_ROM_PTR(&mod_sentai_cam_to_tensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_jpeg),  MP_ROM_PTR(&mod_sentai_cam_save_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_res),        MP_ROM_PTR(&mod_sentai_cam_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_res),    MP_ROM_PTR(&mod_sentai_cam_set_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_native_res), MP_ROM_PTR(&mod_sentai_cam_native_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_switch),     MP_ROM_PTR(&mod_sentai_cam_switch_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotate),     MP_ROM_PTR(&mod_sentai_cam_rotate_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_camera_globals, sentai_camera_globals_table);
static const mp_obj_module_t sentai_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_camera_globals,
};

// ============== sentai.usb — USB mass storage + serial ==============
static const mp_rom_map_elem_t sentai_usb_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_usb) },
    { MP_ROM_QSTR(MP_QSTR_drive),             MP_ROM_PTR(&mod_sentai_usb_drive_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),              MP_ROM_PTR(&mod_sentai_usb_serial_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),             MP_ROM_PTR(&mod_sentai_usb_serial_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),             MP_ROM_PTR(&mod_sentai_usb_serial_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),              MP_ROM_PTR(&mod_sentai_usb_serial_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_usb_serial_available_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_usb_globals, sentai_usb_globals_table);
static const mp_obj_module_t sentai_usb_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_usb_globals,
};

// ============== sentai.uart — UART serial I/O ==============
static const mp_rom_map_elem_t sentai_uart_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_uart) },
    { MP_ROM_QSTR(MP_QSTR_open),              MP_ROM_PTR(&mod_sentai_uart_serial_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),             MP_ROM_PTR(&mod_sentai_uart_serial_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),             MP_ROM_PTR(&mod_sentai_uart_serial_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),              MP_ROM_PTR(&mod_sentai_uart_serial_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_uart_serial_available_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_uart_globals, sentai_uart_globals_table);
static const mp_obj_module_t sentai_uart_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_uart_globals,
};

// ============== sentai.mesh — Meshtastic mesh radio ==============
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

// ============== sentai.link — MAVLink telemetry bridge ==============
static const mp_rom_map_elem_t sentai_link_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_link) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&mod_sentai_link_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),              MP_ROM_PTR(&mod_sentai_link_stop_obj) },
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

// =====================================================================
// Top-level module: import sentai
// =====================================================================
// Usage:
//   import sentai
//   sentai.run("/test.py")
//   sentai.io.led_on()
//   sentai.fs.read("/file.txt")
//   sentai.tpu.load("/model.tflite")
//   sentai.camera.init()
//   sentai.rtos.uptime()
//   sentai.usb.drive(1)

static const mp_rom_map_elem_t sentai_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai) },
    // Help, console control & script execution
    { MP_ROM_QSTR(MP_QSTR_help),      MP_ROM_PTR(&mod_sentai_help_obj) },
    { MP_ROM_QSTR(MP_QSTR_console),   MP_ROM_PTR(&mod_sentai_console_obj) },
    { MP_ROM_QSTR(MP_QSTR_run),       MP_ROM_PTR(&mod_sentai_run_obj) },
    // Sub-modules
    { MP_ROM_QSTR(MP_QSTR_io),        MP_ROM_PTR(&sentai_io_module) },
    { MP_ROM_QSTR(MP_QSTR_rtos),      MP_ROM_PTR(&sentai_rtos_module) },
    { MP_ROM_QSTR(MP_QSTR_tpu),       MP_ROM_PTR(&sentai_tpu_module) },
    { MP_ROM_QSTR(MP_QSTR_fs),        MP_ROM_PTR(&sentai_fs_module) },
    { MP_ROM_QSTR(MP_QSTR_camera),    MP_ROM_PTR(&sentai_camera_module) },
    { MP_ROM_QSTR(MP_QSTR_usb),       MP_ROM_PTR(&sentai_usb_module) },
    { MP_ROM_QSTR(MP_QSTR_uart),      MP_ROM_PTR(&sentai_uart_module) },
    { MP_ROM_QSTR(MP_QSTR_mesh),      MP_ROM_PTR(&sentai_mesh_module) },
    { MP_ROM_QSTR(MP_QSTR_link),      MP_ROM_PTR(&sentai_link_module) },
};
static MP_DEFINE_CONST_DICT(sentai_module_globals, sentai_module_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_sentai, mp_module_sentai);
