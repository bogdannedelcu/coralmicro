// Custom MicroPython C module: 'sentai'
// Main skeleton — namespace implementations are in modsentai_*.c files.
//
// Split into per-namespace files (all #include'd below):
//   modsentai_io.c       — sentai.io      (LED / GPIO)
//   modsentai_rtos.c     — sentai.rtos    (FreeRTOS tasks, heap, CPU, sleep_ms)
//   modsentai_tpu.c      — sentai.tpu     (EdgeTPU inference, detect, draw)
//   modsentai_fs.c       — sentai.fs      (LittleFS filesystem)
//   modsentai_camera.c   — sentai.camera  (Camera capture, to_tensor)
//   modsentai_usb.c      — sentai.usb     (USB mass storage + serial)
//   modsentai_uart.c     — sentai.uart    (UART serial)
//   modsentai_mesh.c     — sentai.mesh    (Meshtastic mesh radio)
//   modsentai_link.c     — sentai.link    (MAVLink telemetry bridge)
//   modsentai_imu.c      — sentai.imu     (LIS2DU12 accelerometer)
//   modsentai_mic.c      — sentai.mic     (Microphone / MP3)
//   modsentai_sleep_ns.c — sentai.sleep   (Light sleep / idle)

#include "build_version.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/gc.h"
#include "py/lexer.h"
#include "py/parse.h"
#include "py/compile.h"
#include <string.h>
#include "sentai_mesh.h"
#include <stdlib.h>

// FreeRTOS - for sentai.tasks(), sentai.heap() (guarded for QSTR generation pass)
#ifndef NO_QSTR
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/portable.h"
#endif

// =====================================================================
// Extern declarations — bridges to C++ runtime / HAL
// =====================================================================

// LED / sleep / ticks — modsentai_hal.cc
extern void sentai_led_set(int on);
extern void sentai_sleep_ms(uint32_t ms);
extern uint32_t sentai_ticks_ms(void);

// TPU bridge — sentai_runtime.cc
extern int sentai_tpu_invoke(void);
extern int sentai_tpu_is_ready(void);
extern int sentai_tpu_num_outputs(void);
extern int sentai_tpu_get_output_size(int idx);
extern const void* sentai_tpu_get_output_data(int idx);
extern int sentai_tpu_get_output_num_dims(int idx);
extern int sentai_tpu_get_output_dim(int idx, int dim);
extern int sentai_tpu_get_output_type(int idx);
extern int sentai_tpu_input_quant(float* scale, int32_t* zero_point);
extern int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);
extern int sentai_tpu_input_type(void);
extern int sentai_tpu_detect(int conf_permil, int iou_permil,
                             int max_dets, int16_t* out_buf, int* out_count);
extern int sentai_tpu_draw(const char* path,
                           const int16_t* dets, int n_dets, int quality);

// Filesystem bridge — modsentai_hal.cc
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

// Camera bridge — sentai_runtime.cc
extern int sentai_cam_init(int streaming);
extern int sentai_cam_stop(void);
extern int sentai_cam_capture_rgb(uint8_t* buf, int width, int height);
extern int sentai_cam_capture_jpeg(uint8_t* jpeg_buf, int jpeg_buf_size,
                                  int width, int height, int quality);
extern int sentai_cam_to_tensor(void);
extern int sentai_cam_to_tensor_ex(const char* save_path, int quality);
extern int sentai_cam_get_width(void);
extern int sentai_cam_get_height(void);
extern int sentai_cam_set_res(int w, int h);
extern int sentai_cam_get_native_width(void);
extern int sentai_cam_get_native_height(void);
extern int sentai_cam_switch(int id);
extern int sentai_cam_rotate(int cam_id, int degrees);

// Model/image loading — sentai_runtime.cc
extern int sentai_load_model(const char* path);
extern int sentai_load_image(const char* path);
extern int sentai_save_output(const char* path);

// USB drive bridge — main_freertos_m7.cc
extern int sentai_usb_drive_set(int on);
extern int sentai_usb_drive_get(void);

// USB serial bridge — modsentai_hal.cc
extern int sentai_usb_serial_open(void);
extern void sentai_usb_serial_close(void);
extern int sentai_usb_serial_is_open(void);
extern int sentai_usb_serial_write(const uint8_t* buf, int size);
extern int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern int sentai_usb_serial_available(void);

// Console — modsentai_hal.cc
extern int sentai_console_set_target(int target);
extern int sentai_console_get_target(void);
extern void sentai_console_write(const char* buf, int size);

// UART serial bridge — modsentai_hal.cc
extern int sentai_uart_serial_open(void);
extern void sentai_uart_serial_close(void);
extern int sentai_uart_serial_is_open(void);
extern int sentai_uart_serial_write(const uint8_t* buf, int size);
extern int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
extern int sentai_uart_serial_available(void);
extern void sentai_uart_set_baudrate(uint32_t baudrate);
extern void sentai_uart_restore_baudrate(void);

// IMU (LIS2DU12 accelerometer) — modsentai_hal.cc
extern int sentai_imu_init(void);
extern int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c);

// Microphone (PDM → MP3 ring buffer) — modsentai_hal.cc
extern int sentai_mic_start(int max_seconds);
extern int sentai_mic_stop(void);
extern int sentai_mic_busy(void);
extern int sentai_mic_samples(void);
extern int sentai_mic_save_l3(char* out_name, int name_size);
extern int sentai_mic_level(void);

// Mesh bridge — sentai_mesh.cc
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

// MAVLink link bridge — sentai_link.cc
typedef struct { uint8_t _opaque[296]; } link_rx_msg_t;
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
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7);
extern uint32_t sentai_link_rx_msgid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_sysid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_compid(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_seq(const link_rx_msg_t* m);
extern uint8_t  sentai_link_rx_len(const link_rx_msg_t* m);
extern void sentai_link_rx_local_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    float* x, float* y, float* z,
    float* vx, float* vy, float* vz);
extern void sentai_link_rx_global_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    int32_t* lat, int32_t* lon, int32_t* alt, int32_t* relative_alt,
    int16_t* vx, int16_t* vy, int16_t* vz, uint16_t* hdg);
extern void sentai_link_set_debug(int level);

// Help file reading from system flash partition
extern int sentai_help_read(char* buf, int max_size);

// Debug — audio subsystem
extern int g_audio_debug;

// =====================================================================
// Shared helpers
// =====================================================================

// Check USB drive state; raise OSError if active.
static void _fs_check_usb(void) {
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("flash busy: call sentai.usb.drive(0) first"));
    }
}

// =====================================================================
// Include per-namespace implementations
// =====================================================================

#include "modsentai_io.c"
#include "modsentai_rtos.c"
#include "modsentai_tpu.c"
#include "modsentai_fs.c"
#include "modsentai_camera.c"
#include "modsentai_usb.c"
#include "modsentai_uart.c"
#include "modsentai_mesh.c"
#include "modsentai_link.c"
#include "modsentai_imu.c"
#include "modsentai_mic.c"
#include "modsentai_sleep_ns.c"

// =====================================================================
// Top-level module functions (sentai.help, sentai.console, etc.)
// =====================================================================

// ===================== Help system (flash-based) =====================

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

// sentai.help([topic])
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
                "Unknown topic. Available: io, rtos, tpu, fs, camera, imu, mic, usb, uart, console, mesh, link, serial, all\r\n");
        }
    }

    free(hbuf);
    #undef HELP_BUF_SIZE
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_help_obj, 0, 1, mod_sentai_help);

// ===================== Console REPL target =====================

// sentai.console([target]) -> str
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

// ===================== Global debug =====================

// sentai.debug(level) -> None
static mp_obj_t mod_sentai_debug(mp_obj_t level_obj) {
    int level = mp_obj_get_int(level_obj);
    g_audio_debug = level;
    sentai_link_set_debug(level);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_debug_obj, mod_sentai_debug);

// ===================== Script execution =====================

// sentai.run(path) - Read and execute a .py file from flash
static mp_obj_t mod_sentai_run(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);

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

    mp_lexer_t *lex = mp_lexer_new_from_str_len(
        qstr_from_str(path), (const char*)buf, n, size + 1);
    qstr source_name = lex->source_name;
    mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
    mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
    mp_call_function_0(module_fun);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_run_obj, mod_sentai_run);

// =====================================================================
// Top-level module: import sentai
// =====================================================================

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)
#define SENTAI_VERSION_STR "SentAI v1.0 build " STRINGIFY(BUILD_VERSION) " (" BUILD_TIMESTAMP ")"

// sentai.version() -> str
static mp_obj_t mod_sentai_version(void) {
    return mp_obj_new_str(SENTAI_VERSION_STR, strlen(SENTAI_VERSION_STR));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_version_obj, mod_sentai_version);

static const mp_rom_map_elem_t sentai_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai) },
    { MP_ROM_QSTR(MP_QSTR_version),  MP_ROM_PTR(&mod_sentai_version_obj) },
    // Help, console control & script execution
    { MP_ROM_QSTR(MP_QSTR_help),      MP_ROM_PTR(&mod_sentai_help_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),     MP_ROM_PTR(&mod_sentai_debug_obj) },
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
    { MP_ROM_QSTR(MP_QSTR_imu),       MP_ROM_PTR(&sentai_imu_module) },
    { MP_ROM_QSTR(MP_QSTR_mic),       MP_ROM_PTR(&sentai_mic_module) },
    { MP_ROM_QSTR(MP_QSTR_sleep),     MP_ROM_PTR(&sentai_sleep_module) },
};
static MP_DEFINE_CONST_DICT(sentai_module_globals, sentai_module_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_sentai, mp_module_sentai);
