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
//   modsentai_crazy.c    — sentai.crazy   (CrazyFlie autopilot bridge)
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
#include <math.h>
#include "sentai_mesh.h"
#include <stdlib.h>
#include "detection_task.h"
#include "sentai_tracker.h"

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
extern uint32_t sentai_cam_get_frame_seq(void);

// Model/image loading — sentai_runtime.cc
extern int sentai_load_model(const char* path);
extern int sentai_load_image(const char* path);
extern int sentai_save_output(const char* path);

// TFL (CPU-only TFLite Micro) bridge — sentai_runtime.cc
extern int sentai_tfl_load(const char* path, int arena_kb);
extern void sentai_tfl_unload(void);
extern int sentai_tfl_invoke(void);
extern int sentai_tfl_is_ready(void);
extern int sentai_tfl_num_outputs(void);
extern int sentai_tfl_get_output_size(int idx);
extern const void* sentai_tfl_get_output_data(int idx);
extern int sentai_tfl_get_output_num_dims(int idx);
extern int sentai_tfl_get_output_dim(int idx, int dim);
extern int sentai_tfl_get_output_type(int idx);
extern int sentai_tfl_input_quant(float* scale, int32_t* zero_point);
extern int sentai_tfl_output_quant(int idx, float* scale, int32_t* zero_point);
extern int sentai_tfl_input_type(void);
extern int sentai_tfl_input_size(void);
extern int sentai_tfl_input_num_dims(void);
extern int sentai_tfl_input_dim(int dim);
extern int sentai_tfl_set_input(const uint8_t* data, int size);
extern void* sentai_tfl_get_input_data(void);
extern int sentai_tfl_load_image(const char* path);
extern int sentai_tfl_save_output(const char* path);
extern int sentai_tfl_info(void);

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
    int32_t gx_cm, int32_t gy_cm, int16_t width_cm,
    uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_send_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    int32_t gx_cm, int32_t gy_cm,
    uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_send_delete(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint32_t reason, uint32_t age, uint32_t total_hits,
    int32_t last_gx_cm, int32_t last_gy_cm,
    uint32_t dest, uint8_t channel, int want_ack);
extern int sentai_mesh_text_available(void);
extern int sentai_mesh_receive_text(mesh_rx_msg_t* msg);
extern int sentai_mesh_receive_text_wait(mesh_rx_msg_t* msg, int timeout_ms);
extern int sentai_mesh_request_config(uint32_t config_id);
extern int sentai_mesh_is_running(void);
extern uint32_t sentai_mesh_my_node_num(void);
extern void sentai_mesh_set_pose(int32_t pitch_deg, int32_t roll_deg,
                                 uint32_t altitude_cm, uint32_t heading_deg);

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
    int32_t gx_cm, int32_t gy_cm, int16_t width_cm,
    uint8_t severity);
extern int sentai_link_send_vision_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    int32_t gx_cm, int32_t gy_cm,
    uint8_t severity);
extern int sentai_link_send_vision_delete(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint32_t reason, uint32_t age, uint32_t total_hits,
    int32_t last_gx_cm, int32_t last_gy_cm,
    uint8_t severity);
extern int sentai_link_send_command_long(
    uint8_t target_sys, uint8_t target_comp,
    uint16_t command, uint8_t confirmation,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7);
extern int sentai_link_send_obstacle_distance(
    const uint16_t* distances_cm,
    uint8_t increment_deg,
    uint16_t min_distance_cm,
    uint16_t max_distance_cm,
    float increment_f_deg,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
extern int sentai_link_send_obstacles_from_tracker(
    uint16_t max_distance_cm,
    uint16_t min_distance_cm,
    float horizontal_fov_deg,
    uint8_t increment_deg,
    uint8_t include_lost,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
extern int sentai_link_send_obstacles_from_points(
    const int32_t* points_xy_cm,
    const uint16_t* radii_cm,
    int count,
    uint16_t max_distance_cm,
    uint16_t min_distance_cm,
    uint8_t increment_deg,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
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

// CrazyFlie autopilot bridge — sentai_crazy.cc
extern int sentai_crazy_init(uint32_t baudrate);
extern int sentai_crazy_stop(void);
extern int sentai_crazy_is_running(void);
extern void sentai_crazy_set_debug(int level);
extern int sentai_crazy_arm(void);
extern int sentai_crazy_disarm(void);
extern int sentai_crazy_takeoff(float height, float duration,
                                float yaw, int use_current_yaw, uint8_t group_mask);
extern int sentai_crazy_land(float height, float duration,
                             float yaw, int use_current_yaw, uint8_t group_mask);
extern int sentai_crazy_stop_motors(uint8_t group_mask);
extern int sentai_crazy_go_to(float x, float y, float z, float yaw, float duration,
                              int relative, int linear, uint8_t group_mask);
extern int sentai_crazy_hover(float vx, float vy, float yaw_rate, float z_distance);
extern int sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                                  const uint8_t* data, int len);
extern int sentai_crazy_ping(int timeout_ms);
extern int sentai_crazy_test_fly(uint16_t power, int duration_ms);
extern int sentai_crazy_fly(float height_m, int hold_ms,
                            int takeoff_ms, int land_ms);
extern int sentai_crazy_attitude(float roll, float pitch,
                                 float yawrate, uint16_t thrust);
extern int sentai_crazy_fly_stop(void);
extern float sentai_crazy_get_altitude(void);

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
#include "modsentai_crazy.c"
#include "modsentai_imu.c"
#include "modsentai_mic.c"
#include "modsentai_sleep_ns.c"
#include "modsentai_pipeline.c"
#include "modsentai_aifes.c"
#include "modsentai_kmeans.c"
#include "modsentai_pca.c"
#include "modsentai_anomaly.c"
#include "modsentai_dtw.c"
#include "modsentai_hmm.c"
#include "modsentai_rl.c"
#include "modsentai_slam.c"
#include "modsentai_tfl.c"

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

    #define HELP_BUF_SIZE 73728
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
                "Unknown topic. Available: io, rtos, tpu, fs, camera, imu, mic, usb, uart, console, mesh, link, crazy, pipeline, serial, all\r\n");
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
    { MP_ROM_QSTR(MP_QSTR_crazy),     MP_ROM_PTR(&sentai_crazy_module) },
    { MP_ROM_QSTR(MP_QSTR_imu),       MP_ROM_PTR(&sentai_imu_module) },
    { MP_ROM_QSTR(MP_QSTR_mic),       MP_ROM_PTR(&sentai_mic_module) },
    { MP_ROM_QSTR(MP_QSTR_sleep),     MP_ROM_PTR(&sentai_sleep_module) },
    { MP_ROM_QSTR(MP_QSTR_pipeline),  MP_ROM_PTR(&sentai_pipeline_module) },
    { MP_ROM_QSTR(MP_QSTR_aifes),     MP_ROM_PTR(&sentai_aifes_module) },
    { MP_ROM_QSTR(MP_QSTR_kmeans),    MP_ROM_PTR(&sentai_kmeans_module) },
    { MP_ROM_QSTR(MP_QSTR_pca),       MP_ROM_PTR(&sentai_pca_module) },
    { MP_ROM_QSTR(MP_QSTR_anomaly),   MP_ROM_PTR(&sentai_anomaly_module) },
    { MP_ROM_QSTR(MP_QSTR_dtw),       MP_ROM_PTR(&sentai_dtw_module) },
    { MP_ROM_QSTR(MP_QSTR_hmm),       MP_ROM_PTR(&sentai_hmm_module) },
    { MP_ROM_QSTR(MP_QSTR_rl),        MP_ROM_PTR(&sentai_rl_module) },
    { MP_ROM_QSTR(MP_QSTR_slam),      MP_ROM_PTR(&sentai_slam_module) },
    { MP_ROM_QSTR(MP_QSTR_tfl),       MP_ROM_PTR(&sentai_tfl_module) },
};
static MP_DEFINE_CONST_DICT(sentai_module_globals, sentai_module_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_sentai, mp_module_sentai);
