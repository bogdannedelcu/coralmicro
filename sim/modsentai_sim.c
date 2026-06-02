/*
 * sim/modsentai_sim.c — Phase 1.5 sentai module bindings for SIM.
 *
 * Provides hardware-independent bindings so the REPL feels like the real
 * board.  All bindings reuse QSTR table entries already present in the
 * firmware build (examples/sentai_runtime/micropython_embed/genhdr/
 * qstrdefs.generated.h).  No QSTR regen needed.
 *
 * What we expose now:
 *   sentai.version()       -> "SentAI SIM v1.0 (Phase 1.5) ..."
 *   sentai.verbose([on])   -> bool, gates [SIM] log output (set/get)
 *   sentai.io.led_on()     -> printf "[LED] ON"  (no real LED in SIM)
 *   sentai.io.led_off()    -> printf "[LED] OFF"
 *   sentai.rtos.sleep_ms(ms) -> vTaskDelay (real FreeRTOS, EINTR-safe)
 *   sentai.sys.reset()     -> exit(0) — clean SIM exit
 *
 * Same API contract as ARM firmware; SIM-only code is limited to platform
 * backends/bridges injected under the shared sentai_runtime bindings.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#include "py/runtime.h"
#include "py/objstr.h"
#include "py/objmodule.h"
#include "py/objtuple.h"
#include "py/lexer.h"
#include "py/parse.h"
#include "py/compile.h"
#include "py/gc.h"

#include "FreeRTOS.h"
#include "task.h"

#include "build_version.h"
#include "sentai_mesh.h"
#include "sentai_crazy.h"
#include "sentai_tpu_shim.h"
#include "detection_task.h"
#include "sentai_tracker.h"

/* SIM platform backend ABIs used by shared bindings. */
int sentai_console_get_target(void);
int sentai_console_set_target(int target);
void sentai_console_write(const char* buf, int size);
void sentai_uart_set_baudrate(uint32_t baudrate);
void sentai_uart_restore_baudrate(void);
int sentai_uart_serial_open(void);
void sentai_uart_serial_close(void);
int sentai_uart_serial_is_open(void);
int sentai_uart_serial_write(const uint8_t* buf, int size);
int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms);
int sentai_uart_serial_available(void);
int sentai_usb_drive_set(int on);
int sentai_usb_drive_get(void);
int sentai_usb_serial_open(void);
void sentai_usb_serial_close(void);
int sentai_usb_serial_is_open(void);
int sentai_usb_serial_write(const uint8_t* buf, int size);
int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms);
int sentai_usb_serial_available(void);
int sentai_usb_ip_set(int on);
int sentai_usb_ip_get(void);
void sentai_httpd_start(void);
void sentai_led_set(int on);
void sentai_sleep_ms(uint32_t ms);
uint32_t sentai_ticks_ms(void);
void sentai_repl_activity(void);
bool sentai_is_recovery_mode(void);
unsigned int sentai_get_boot_attempts(void);
void sentai_sys_do_reset(void);
int sentai_fs_lock(void);
void sentai_fs_unlock(void);
int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
int sentai_fs_size(const char* path);
int sentai_fs_file_exists(const char* path);
int sentai_fs_dir_exists(const char* path);
int sentai_fs_write(const char* path, const uint8_t* buf, int size);
int sentai_fs_append(const char* path, const uint8_t* buf, int size);
int sentai_fs_remove(const char* path);
int sentai_fs_makedirs(const char* path);
int sentai_fs_listdir(const char* path,
                      void (*callback)(const char* name, int type, int size,
                                       void* ud),
                      void* user_data);
int sentai_fs_sync(void);
int sentai_fs_format(void);
int sentai_imu_init(void);
int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c);
int sentai_imu_tap_start(void);
int sentai_imu_tap_stop(void);
int sentai_imu_tap_poll(int timeout_ms, uint32_t* ev_out);
int sentai_mic_start(int max_seconds);
int sentai_mic_stop(void);
int sentai_mic_busy(void);
int sentai_mic_samples(void);
int sentai_mic_is_initialized(void);
int sentai_mic_save_l3(char* out_name, int name_size);
int sentai_mic_level(void);
int sentai_sleep_idle(int threshold_db, int timeout_ms, int enable_tap);
int sentai_tfl_load(const char* path, int arena_kb);
void sentai_tfl_unload(void);
int sentai_tfl_invoke(void);
int sentai_tfl_is_ready(void);
int sentai_tfl_num_outputs(void);
int sentai_tfl_get_output_size(int idx);
const void* sentai_tfl_get_output_data(int idx);
int sentai_tfl_get_output_num_dims(int idx);
int sentai_tfl_get_output_dim(int idx, int dim);
int sentai_tfl_get_output_type(int idx);
int sentai_tfl_input_quant(float* scale, int32_t* zero_point);
int sentai_tfl_output_quant(int idx, float* scale, int32_t* zero_point);
int sentai_tfl_input_type(void);
int sentai_tfl_input_size(void);
int sentai_tfl_input_num_dims(void);
int sentai_tfl_input_dim(int dim);
int sentai_tfl_set_input(const uint8_t* data, int size);
void* sentai_tfl_get_input_data(void);
int sentai_tfl_load_image(const char* path);
int sentai_tfl_save_output(const char* path);
int sentai_tfl_info(void);

typedef struct { uint8_t _opaque[296]; } link_rx_msg_t;
int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid);
int sentai_link_stop(void);
int sentai_link_is_running(void);
int sentai_link_available(void);
int sentai_link_receive(link_rx_msg_t* msg);
int sentai_link_receive_wait(link_rx_msg_t* msg, int timeout_ms);
int sentai_link_send_heartbeat(uint8_t type);
int sentai_link_send_statustext(uint8_t severity, const char* text);
int sentai_link_send_vision(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    int32_t gx_cm, int32_t gy_cm, int16_t width_cm,
    uint8_t severity);
int sentai_link_send_vision_update(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    int32_t gx_cm, int32_t gy_cm,
    uint8_t severity);
int sentai_link_send_vision_delete(
    uint32_t sensor_id, uint32_t track_id, uint32_t alarm_type,
    uint32_t timestamp_utc, uint32_t seq,
    uint32_t reason, uint32_t age, uint32_t total_hits,
    int32_t last_gx_cm, int32_t last_gy_cm,
    uint8_t severity);
int sentai_link_send_command_long(
    uint8_t target_sys, uint8_t target_comp,
    uint16_t command, uint8_t confirmation,
    float param1, float param2, float param3, float param4,
    float param5, float param6, float param7);
int sentai_link_send_obstacle_distance(
    const uint16_t* distances_cm,
    uint8_t increment_deg,
    uint16_t min_distance_cm,
    uint16_t max_distance_cm,
    float increment_f_deg,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
int sentai_link_send_obstacles_from_tracker(
    uint16_t max_distance_cm,
    uint16_t min_distance_cm,
    float horizontal_fov_deg,
    uint8_t increment_deg,
    uint8_t include_lost,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
int sentai_link_send_obstacles_from_points(
    const int32_t* points_xy_cm,
    const uint16_t* radii_cm,
    int count,
    uint16_t max_distance_cm,
    uint16_t min_distance_cm,
    uint8_t increment_deg,
    float angle_offset_deg,
    uint8_t sensor_type,
    uint8_t frame);
uint32_t sentai_link_rx_msgid(const link_rx_msg_t* m);
uint8_t sentai_link_rx_sysid(const link_rx_msg_t* m);
uint8_t sentai_link_rx_compid(const link_rx_msg_t* m);
uint8_t sentai_link_rx_seq(const link_rx_msg_t* m);
uint8_t sentai_link_rx_len(const link_rx_msg_t* m);
void sentai_link_rx_local_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    float* x, float* y, float* z,
    float* vx, float* vy, float* vz);
void sentai_link_rx_global_pos(
    const link_rx_msg_t* m,
    uint32_t* time_boot_ms,
    int32_t* lat, int32_t* lon,
    int32_t* alt, int32_t* relative_alt,
    int16_t* vx, int16_t* vy, int16_t* vz,
    uint16_t* hdg);
void sentai_link_set_debug(int level);

static void _fs_check_usb(void) {
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("flash busy: call sentai.usb.drive(0) first"));
    }
}

/* ---- sentai.version() ---- */
static mp_obj_t sentai_version(void) {
    /* Mirror the firmware format: "SentAI v1.0 build NNN (timestamp)".
     * SIM has its own build counter (sim/build_version.h), separate from
     * ARM's, so the operator can tell them apart at a glance. */
    static char vers[128];
    int n = snprintf(vers, sizeof(vers),
                     "SentAI SIM v1.0 build %d (%s) - FreeRTOS POSIX + MicroPython embed",
                     BUILD_VERSION, BUILD_TIMESTAMP);
    if (n < 0) n = 0;
    return mp_obj_new_str(vers, (size_t) n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_version_obj, sentai_version);

/* ---- sentai.verbose([on]) ---- */
static int s_verbose = 1;
static mp_obj_t sentai_verbose(size_t n_args, const mp_obj_t *args) {
    int prev = s_verbose;
    if (n_args >= 1) {
        s_verbose = mp_obj_is_true(args[0]) ? 1 : 0;
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_verbose_obj, 0, 1,
                                            sentai_verbose);


/* ============================================================
 * SIM SENTAI MODULE BINDINGS — fragment includes (refactor T0)
 *
 * Each subsystem binding is #include'd here so the whole thing stays a
 * single MicroPython module translation unit.  Platform-specific SIM
 * behavior lives in backend/bridge files, not in duplicate namespace
 * fragments.
 *
 * To add a new SIM subsystem:
 *   1. prefer the shared `examples/sentai_runtime/bindings/modsentai_<name>.c`,
 *   2. add only a `sentai_<name>_sim_backend.c` or `*_bridge.c` if the
 *      platform implementation differs,
 *   3. add a `{ MP_ROM_QSTR(MP_QSTR_<name>), MP_ROM_PTR(&sentai_<name>_module) }`
 *      entry to `sentai_globals_table` at the bottom,
 *   4. if any SIM-only QSTR is used, append it to
 *      `examples/sentai_runtime/qstrdefs_sim_extra.h` + regen QSTRs
 *      (see CLAUDE.md §QSTR regen).
 * ============================================================ */

#include "../examples/sentai_runtime/bindings/modsentai_io.c"
#include "../examples/sentai_runtime/bindings/modsentai_rtos.c"
#include "../examples/sentai_runtime/bindings/modsentai_sys.c"
#include "../examples/sentai_runtime/bindings/modsentai_fs.c"
#include "../examples/sentai_runtime/bindings/modsentai_camera_sim.c"
#include "../examples/sentai_runtime/bindings/modsentai_flow.c"
#include "bindings/modsentai_tpu.c"
#include "../examples/sentai_runtime/bindings/modsentai_pipeline.c"
#include "../examples/sentai_runtime/bindings/modsentai_link.c"
#include "../examples/sentai_runtime/bindings/modsentai_uart.c"
#include "../examples/sentai_runtime/bindings/modsentai_usb.c"
#include "../examples/sentai_runtime/bindings/modsentai_mesh.c"
#include "../examples/sentai_runtime/bindings/modsentai_imu.c"
#include "../examples/sentai_runtime/bindings/modsentai_mic.c"
#include "../examples/sentai_runtime/bindings/modsentai_sleep_ns.c"
#include "../examples/sentai_runtime/bindings/modsentai_aifes.c"

// Shared pure-compute helper namespaces.  These are not flight controllers;
// exposing them in SIM keeps the sentai.* surface aligned with ARM.
#include "../examples/sentai_runtime/bindings/modsentai_kmeans.c"
#include "../examples/sentai_runtime/bindings/modsentai_pca.c"
#include "../examples/sentai_runtime/bindings/modsentai_anomaly.c"
#include "../examples/sentai_runtime/bindings/modsentai_dtw.c"
#include "../examples/sentai_runtime/bindings/modsentai_hmm.c"
#include "../examples/sentai_runtime/bindings/modsentai_rl.c"

// ObjectsPlan L2 — sentai.objects.  Pure data-layer binding shared with
// ARM via #include of the canonical source under examples/sentai_runtime/.
// Keeps ARM and SIM exposing an identical surface; backing store + math
// live in sentai_objects.{h,cc} (also in this build via sim/CMakeLists.txt).
#include "../examples/sentai_runtime/bindings/modsentai_objects.c"
// ObjectsPlan L3 — sentai.places.  Same #include pattern as L2; the
// libh3_sim target is linked by sim/CMakeLists.txt so the H3 calls
// (cell_at, neighbors, gridDisk inside query) resolve at link time.
#include "../examples/sentai_runtime/bindings/modsentai_places.c"
// ObjectsPlan L4 — sentai.servo (action layer skeleton).  Pure FSM +
// trace ring shared with ARM via the same #include pattern.
#include "../examples/sentai_runtime/bindings/modsentai_servo.c"
// ObjectsPlan L5 — sentai.object_lifter (inverse-depth EKF landmark
// lifter). Pure float math; shared with ARM via the same #include.
#include "../examples/sentai_runtime/bindings/modsentai_object_lifter.c"
// ObjectsPlan OP-S6-W1 — sentai.calib (camera-to-body Kabsch).  Shared
// with ARM via the same #include; the .cc file branches on
// SENTAI_HAVE_FXUSER to pick FileX (ARM) vs host stdio (SIM).
#include "../examples/sentai_runtime/bindings/modsentai_calib.c"
// Object-level EKF-SLAM binding.  Pure MP binding with internal map state;
// SIM already links slam_task.cc for the place/SLAM worker path.
#include "../examples/sentai_runtime/bindings/modsentai_slam.c"
// OP-S10-W19-T1 hard rename: legacy sentai.aruco + sentai.whycon
// MP bindings de-registered.  C backends (sentai_aruco.cc + WhyCon
// helpers in the same TU) remain as backend implementations called
// by sentai_markers_*.  Only the unified dispatcher binding is
// included.
#include "../examples/sentai_runtime/bindings/modsentai_markers.c"
// ObjectsPlan OP-S10-W12 — sentai.safety (firmware-side mission safety).
// State machine + SafetyTask worker; consumes sentai.aruco results.
// See Safety.md for architecture.
#include "../examples/sentai_runtime/bindings/modsentai_safety.c"
// ObjectsPlan OP-S10-W13 — sentai.fr (Flight Recorder subsystem).
// Multi-channel recorder; producers push items, drain task writes to disk.
#include "../examples/sentai_runtime/bindings/modsentai_fr.c"
// ObjectsPlan L6 — sentai.explore (mission FSM).  Pure FSM that wraps
// L4 servo + L5 lifter; shared with ARM via the same #include.
#include "../examples/sentai_runtime/bindings/modsentai_explore.c"
#include "../examples/sentai_runtime/bindings/modsentai_tfl.c"
#include "../examples/sentai_runtime/bindings/modsentai_top.c"

// Task #39 — sentai.crazy over CRTP-UDP to cf2 SITL (SIM-only).  ARM
// uses examples/sentai_runtime/modsentai_crazy.c (CPX-over-UART); this
// is the SIM-flavor binding with the same Python surface (arm /
// takeoff / land / go_to / hover / send_crtp / recv_crtp / stats).
#include "../examples/sentai_runtime/bindings/modsentai_crazy.c"


static const mp_rom_map_elem_t sentai_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai) },
    { MP_ROM_QSTR(MP_QSTR_version),  MP_ROM_PTR(&sentai_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_verbose),  MP_ROM_PTR(&sentai_verbose_obj) },
    { MP_ROM_QSTR(MP_QSTR_help),     MP_ROM_PTR(&mod_sentai_help_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),    MP_ROM_PTR(&mod_sentai_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_console),  MP_ROM_PTR(&mod_sentai_console_obj) },
    { MP_ROM_QSTR(MP_QSTR_run),      MP_ROM_PTR(&mod_sentai_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_io),       MP_ROM_PTR(&sentai_io_module) },
    { MP_ROM_QSTR(MP_QSTR_rtos),     MP_ROM_PTR(&sentai_rtos_module) },
    { MP_ROM_QSTR(MP_QSTR_sys),      MP_ROM_PTR(&sentai_sys_module) },
    { MP_ROM_QSTR(MP_QSTR_fs),       MP_ROM_PTR(&sentai_fs_module) },
    { MP_ROM_QSTR(MP_QSTR_camera),   MP_ROM_PTR(&sentai_camera_module) },
    { MP_ROM_QSTR(MP_QSTR_flow),     MP_ROM_PTR(&sentai_flow_module) },
    { MP_ROM_QSTR(MP_QSTR_tpu),      MP_ROM_PTR(&sentai_tpu_module) },
    { MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&sentai_pipeline_module) },
    { MP_ROM_QSTR(MP_QSTR_link),     MP_ROM_PTR(&sentai_link_module) },
    { MP_ROM_QSTR(MP_QSTR_uart),     MP_ROM_PTR(&sentai_uart_module) },
    { MP_ROM_QSTR(MP_QSTR_usb),      MP_ROM_PTR(&sentai_usb_module) },
    { MP_ROM_QSTR(MP_QSTR_mesh),     MP_ROM_PTR(&sentai_mesh_module) },
    { MP_ROM_QSTR(MP_QSTR_imu),      MP_ROM_PTR(&sentai_imu_module) },
    { MP_ROM_QSTR(MP_QSTR_mic),      MP_ROM_PTR(&sentai_mic_module) },
    { MP_ROM_QSTR(MP_QSTR_sleep),    MP_ROM_PTR(&sentai_sleep_module) },
    { MP_ROM_QSTR(MP_QSTR_aifes),    MP_ROM_PTR(&sentai_aifes_module) },
    { MP_ROM_QSTR(MP_QSTR_kmeans),   MP_ROM_PTR(&sentai_kmeans_module) },
    { MP_ROM_QSTR(MP_QSTR_pca),      MP_ROM_PTR(&sentai_pca_module) },
    { MP_ROM_QSTR(MP_QSTR_anomaly),  MP_ROM_PTR(&sentai_anomaly_module) },
    { MP_ROM_QSTR(MP_QSTR_dtw),      MP_ROM_PTR(&sentai_dtw_module) },
    { MP_ROM_QSTR(MP_QSTR_hmm),      MP_ROM_PTR(&sentai_hmm_module) },
    { MP_ROM_QSTR(MP_QSTR_rl),       MP_ROM_PTR(&sentai_rl_module) },
    { MP_ROM_QSTR(MP_QSTR_objects),  MP_ROM_PTR(&sentai_objects_module) },
    { MP_ROM_QSTR(MP_QSTR_places),   MP_ROM_PTR(&sentai_places_module) },
    { MP_ROM_QSTR(MP_QSTR_servo),    MP_ROM_PTR(&sentai_servo_module) },
    { MP_ROM_QSTR(MP_QSTR_object_lifter), MP_ROM_PTR(&sentai_object_lifter_module) },
    { MP_ROM_QSTR(MP_QSTR_calib),    MP_ROM_PTR(&sentai_calib_module) },
    { MP_ROM_QSTR(MP_QSTR_slam),     MP_ROM_PTR(&sentai_slam_module) },
    // OP-S10-W19-T1 hard rename: sentai.markers is the canonical
    // fiducial-marker namespace (mirrors modsentai.c on ARM).
    { MP_ROM_QSTR(MP_QSTR_markers),  MP_ROM_PTR(&sentai_markers_module) },
    { MP_ROM_QSTR(MP_QSTR_safety),   MP_ROM_PTR(&sentai_safety_module) },
    { MP_ROM_QSTR(MP_QSTR_fr),       MP_ROM_PTR(&sentai_fr_module) },
    { MP_ROM_QSTR(MP_QSTR_explore),  MP_ROM_PTR(&sentai_explore_module) },
    { MP_ROM_QSTR(MP_QSTR_tfl),      MP_ROM_PTR(&sentai_tfl_module) },
    { MP_ROM_QSTR(MP_QSTR_crazy),    MP_ROM_PTR(&sentai_crazy_module) },
};
static MP_DEFINE_CONST_DICT(sentai_globals, sentai_globals_table);

/* This symbol is referenced from moduledefs.h (generated for the firmware
 * build).  Replaces the empty stub previously in main_sim.c. */
const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_globals,
};
