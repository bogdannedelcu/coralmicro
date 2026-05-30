/*
 * sentai_platform_sim_backend.c
 *
 * Minimal SIM platform backend for shared SentAI binding ABIs.  The Python
 * bindings live in examples/sentai_runtime/bindings and are shared with ARM;
 * this file only provides host-side behavior where physical board devices are
 * absent or platform access differs.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sentai_mesh.h"

extern const char sentai_help_builtin_text[];

void sentai_sleep_ms(uint32_t ms);

static int s_console_target = 0;
int g_audio_debug = 0;

int sentai_console_set_target(int target) {
    if (target != 0 && target != 1) return -1;
    s_console_target = target;
    return 0;
}

int sentai_console_get_target(void) {
    return s_console_target;
}

int sentai_help_read(char* buf, int max_size) {
    if (!buf || max_size <= 0) return -1;
    int len = (int)strlen(sentai_help_builtin_text);
    int to_copy = len < max_size - 1 ? len : max_size - 1;
    memcpy(buf, sentai_help_builtin_text, (size_t)to_copy);
    buf[to_copy] = '\0';
    return to_copy;
}

int sentai_imu_init(void) {
    return 0;
}

int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c) {
    if (x_mg) *x_mg = 0.0f;
    if (y_mg) *y_mg = 0.0f;
    if (z_mg) *z_mg = 1000.0f;
    if (temp_c) *temp_c = 25.0f;
    return 0;
}

int sentai_imu_tap_start(void) {
    return 0;
}

int sentai_imu_tap_stop(void) {
    return 0;
}

int sentai_imu_tap_poll(int timeout_ms, uint32_t* ev_out) {
    (void)timeout_ms;
    if (ev_out) *ev_out = 0;
    return 0;
}

int sentai_mic_start(int max_seconds) {
    (void)max_seconds;
    return 0;
}

int sentai_mic_stop(void) {
    return 0;
}

int sentai_mic_busy(void) {
    return 0;
}

int sentai_mic_samples(void) {
    return 0;
}

int sentai_mic_is_initialized(void) {
    return 1;
}

int sentai_mic_save_l3(char* out_name, int name_size) {
    if (out_name && name_size > 0) out_name[0] = '\0';
    return -1;
}

int sentai_mic_level(void) {
    return 0;
}

int sentai_sleep_idle(int threshold_db, int timeout_ms, int enable_tap) {
    (void)threshold_db;
    (void)enable_tap;
    if (timeout_ms > 0) sentai_sleep_ms((uint32_t)timeout_ms);
    return 0;
}

int sentai_usb_drive_set(int on) {
    (void)on;
    return -1;
}

int sentai_usb_drive_get(void) {
    return 0;
}

int sentai_usb_serial_open(void) {
    return 0;
}

void sentai_usb_serial_close(void) {
}

int sentai_usb_serial_is_open(void) {
    return 0;
}

int sentai_usb_serial_write(const uint8_t* buf, int size) {
    (void)buf;
    (void)size;
    return -1;
}

int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    (void)buf;
    (void)max_size;
    (void)timeout_ms;
    return 0;
}

int sentai_usb_serial_available(void) {
    return 0;
}

int sentai_usb_ip_set(int on) {
    (void)on;
    return -1;
}

int sentai_usb_ip_get(void) {
    return -1;
}

void sentai_httpd_start(void) {
}

int sentai_mesh_init(uint32_t baudrate) {
    (void)baudrate;
    return -1;
}

int sentai_mesh_stop(void) {
    return 0;
}

int sentai_mesh_send_text(const char* text, uint32_t dest, uint8_t channel,
                          int want_ack) {
    (void)text;
    (void)dest;
    (void)channel;
    (void)want_ack;
    return -1;
}

int sentai_mesh_send_detection(uint32_t sensor_id, uint32_t track_id,
                               uint32_t alarm_type, uint32_t timestamp,
                               uint32_t seq, uint8_t x, uint8_t y,
                               uint8_t w, uint8_t h, uint32_t conf,
                               uint32_t class_id, int32_t gx_cm,
                               int32_t gy_cm, int16_t width_cm,
                               uint32_t dest, uint8_t channel, int want_ack) {
    (void)sensor_id; (void)track_id; (void)alarm_type; (void)timestamp;
    (void)seq; (void)x; (void)y; (void)w; (void)h; (void)conf;
    (void)class_id; (void)gx_cm; (void)gy_cm; (void)width_cm;
    (void)dest; (void)channel; (void)want_ack;
    return -1;
}

int sentai_mesh_send_update(uint32_t sensor_id, uint32_t track_id,
                            uint32_t alarm_type, uint32_t timestamp,
                            uint32_t seq, uint8_t x, uint8_t y,
                            uint8_t w, uint8_t h, uint32_t conf,
                            uint32_t age, int32_t gx_cm, int32_t gy_cm,
                            uint32_t dest, uint8_t channel, int want_ack) {
    (void)sensor_id; (void)track_id; (void)alarm_type; (void)timestamp;
    (void)seq; (void)x; (void)y; (void)w; (void)h; (void)conf;
    (void)age; (void)gx_cm; (void)gy_cm; (void)dest; (void)channel;
    (void)want_ack;
    return -1;
}

int sentai_mesh_send_delete(uint32_t sensor_id, uint32_t track_id,
                            uint32_t alarm_type, uint32_t timestamp,
                            uint32_t seq, uint32_t reason, uint32_t age,
                            uint32_t total_hits, int32_t last_gx_cm,
                            int32_t last_gy_cm, uint32_t dest,
                            uint8_t channel, int want_ack) {
    (void)sensor_id; (void)track_id; (void)alarm_type; (void)timestamp;
    (void)seq; (void)reason; (void)age; (void)total_hits;
    (void)last_gx_cm; (void)last_gy_cm; (void)dest; (void)channel;
    (void)want_ack;
    return -1;
}

int sentai_mesh_text_available(void) {
    return 0;
}

int sentai_mesh_receive_text(mesh_rx_msg_t* msg) {
    (void)msg;
    return 0;
}

int sentai_mesh_receive_text_wait(mesh_rx_msg_t* msg, int timeout_ms) {
    (void)msg;
    if (timeout_ms > 0) sentai_sleep_ms((uint32_t)timeout_ms);
    return 0;
}

int sentai_mesh_request_config(uint32_t config_id) {
    (void)config_id;
    return -1;
}

void sentai_mesh_set_pose(int32_t pitch_deg, int32_t roll_deg,
                          uint32_t altitude_cm, uint32_t heading_deg) {
    (void)pitch_deg;
    (void)roll_deg;
    (void)altitude_cm;
    (void)heading_deg;
}

int aifes_load_model(const char* yaml_path) {
    (void)yaml_path;
    return -1;
}

int aifes_load_weights(const char* weights_path) {
    (void)weights_path;
    return -1;
}

int aifes_save_weights(const char* weights_path) {
    (void)weights_path;
    return -1;
}

float aifes_train(const float* x_data, const float* y_data,
                  int n_samples, int input_size, int output_size,
                  int epochs, int batch_size, float learning_rate,
                  int optimizer, int loss_fn, float val_split,
                  const char* log_path, float sigreg_lambda) {
    (void)x_data; (void)y_data; (void)n_samples; (void)input_size;
    (void)output_size; (void)epochs; (void)batch_size; (void)learning_rate;
    (void)optimizer; (void)loss_fn; (void)val_split; (void)log_path;
    (void)sigreg_lambda;
    return -1.0f;
}

int aifes_predict(const float* input, float* output) {
    (void)input;
    (void)output;
    return -1;
}

int aifes_is_loaded(void) {
    return 0;
}

const char* aifes_get_model_name(void) {
    return "sim_stub";
}

int aifes_get_layer_count(void) {
    return 0;
}

int aifes_get_input_size(void) {
    return 0;
}

int aifes_get_output_size(void) {
    return 0;
}

int aifes_get_total_params(void) {
    return 0;
}

const char* aifes_get_log_path(void) {
    return "";
}

int aifes_get_best_epoch(void) {
    return -1;
}

float aifes_get_best_val_loss(void) {
    return 0.0f;
}

void aifes_unload(void) {
}

int aifes_set_input(const float* data, int size) {
    (void)data;
    (void)size;
    return -1;
}

int aifes_from_tpu(int idx) {
    (void)idx;
    return -1;
}

int aifes_from_camera(int width, int height, int grayscale) {
    (void)width;
    (void)height;
    (void)grayscale;
    return -1;
}

int aifes_from_mic(int samples) {
    (void)samples;
    return -1;
}

int aifes_invoke(void) {
    return -1;
}

const float* aifes_get_output_data(void) {
    return NULL;
}

int aifes_get_output_count(void) {
    return 0;
}

int sentai_tfl_load(const char* path, int arena_kb) {
    (void)path;
    (void)arena_kb;
    return -99;
}

void sentai_tfl_unload(void) {
}

int sentai_tfl_invoke(void) {
    return -1;
}

int sentai_tfl_is_ready(void) {
    return 0;
}

int sentai_tfl_num_outputs(void) {
    return 0;
}

int sentai_tfl_get_output_size(int idx) {
    (void)idx;
    return 0;
}

const void* sentai_tfl_get_output_data(int idx) {
    (void)idx;
    return NULL;
}

int sentai_tfl_get_output_num_dims(int idx) {
    (void)idx;
    return 0;
}

int sentai_tfl_get_output_dim(int idx, int dim) {
    (void)idx;
    (void)dim;
    return 0;
}

int sentai_tfl_get_output_type(int idx) {
    (void)idx;
    return -1;
}

int sentai_tfl_input_quant(float* scale, int32_t* zero_point) {
    if (scale) *scale = 0.0f;
    if (zero_point) *zero_point = 0;
    return -1;
}

int sentai_tfl_output_quant(int idx, float* scale, int32_t* zero_point) {
    (void)idx;
    if (scale) *scale = 0.0f;
    if (zero_point) *zero_point = 0;
    return -1;
}

int sentai_tfl_input_type(void) {
    return -1;
}

int sentai_tfl_input_size(void) {
    return 0;
}

int sentai_tfl_input_num_dims(void) {
    return 0;
}

int sentai_tfl_input_dim(int dim) {
    (void)dim;
    return 0;
}

int sentai_tfl_set_input(const uint8_t* data, int size) {
    (void)data;
    (void)size;
    return -1;
}

void* sentai_tfl_get_input_data(void) {
    return NULL;
}

int sentai_tfl_load_image(const char* path) {
    (void)path;
    return -99;
}

int sentai_tfl_save_output(const char* path) {
    (void)path;
    return -99;
}

int sentai_tfl_info(void) {
    return -1;
}
