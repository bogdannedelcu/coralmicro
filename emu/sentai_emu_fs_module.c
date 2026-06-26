// B8 ARM-emulator `sentai.fs` module.
//
// This is intentionally a thin MicroPython binding over the production
// FxUser* API.  It does not introduce a RAM filesystem and does not bypass
// FileX/LevelX; the only emulator substitution lives below fx_nand_driver_*.

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>

#include "examples/sentai_runtime/build_version.h"
#include "examples/sentai_runtime/sentai_dmesg.h"
#include "examples/sentai_runtime/sentai_fr.h"
#include "libs/base/fx_user_fs.h"
#include "py/obj.h"
#include "py/compile.h"
#include "py/gc.h"
#include "py/lexer.h"
#include "py/mphal.h"
#include "py/mpprint.h"
#include "py/parse.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#if !MICROPY_ENABLE_SCHEDULER
extern bool mp_sched_schedule(mp_obj_t function, mp_obj_t arg);
#endif

#ifndef SENTAI_EMU_WEAK
#define SENTAI_EMU_WEAK __attribute__((weak))
#endif

#ifndef SENTAI_EMU_HW_STUB_MODULES
#define SENTAI_EMU_HW_STUB_MODULES 0
#endif

#ifndef SENTAI_EMU_SAFETY_BINDING
#define SENTAI_EMU_SAFETY_BINDING 0
#endif

#ifndef SENTAI_EMU_SERVO_BINDING
#define SENTAI_EMU_SERVO_BINDING 0
#endif

#ifndef SENTAI_EMU_CALIB_BINDING
#define SENTAI_EMU_CALIB_BINDING 0
#endif

#ifndef SENTAI_EMU_UART_BINDING
#define SENTAI_EMU_UART_BINDING 0
#endif

#ifndef SENTAI_EMU_OBJECT_LIFTER_BINDING
#define SENTAI_EMU_OBJECT_LIFTER_BINDING 0
#endif

#ifndef SENTAI_EMU_USB_BINDING
#define SENTAI_EMU_USB_BINDING 0
#endif

#if SENTAI_EMU_CRAZY_BINDING
extern int sentai_console_get_target(void);
extern int sentai_mesh_is_running(void);
extern int sentai_link_is_running(void);
extern int sentai_crazy_init(uint32_t baudrate);
extern int sentai_crazy_stop(void);
extern int sentai_crazy_is_running(void);
extern void sentai_crazy_set_debug(int level);
extern int sentai_crazy_arm(void);
extern int sentai_crazy_disarm(void);
extern int sentai_crazy_takeoff(float height, float duration,
                                float yaw, int use_current_yaw,
                                uint8_t group_mask);
extern int sentai_crazy_land(float height, float duration,
                             float yaw, int use_current_yaw,
                             uint8_t group_mask);
extern int sentai_crazy_stop_motors(uint8_t group_mask);
extern int sentai_crazy_hl_stop(uint8_t group_mask);
extern int sentai_crazy_go_to(float x, float y, float z, float yaw,
                              float duration, int relative, int linear,
                              uint8_t group_mask);
extern int sentai_crazy_hover(float vx, float vy, float yaw_rate,
                              float z_distance);
extern int sentai_crazy_send_crtp(uint8_t port, uint8_t channel,
                                  const uint8_t* data, int len);
extern int sentai_crazy_ping(int timeout_ms);
extern int sentai_crazy_test_fly(uint16_t power, int duration_ms);
extern int sentai_crazy_fly(float height_m, int hold_ms,
                            int takeoff_ms, int land_ms);
extern int sentai_crazy_attitude(float roll, float pitch, float yawrate,
                                 uint16_t thrust);
extern int sentai_crazy_attitude_release_no_disarm(void);
extern int sentai_crazy_fly_stop(void);
extern float sentai_crazy_get_altitude(void);
#include "examples/sentai_runtime/bindings/modsentai_crazy.c"
#endif

static int g_sentai_emu_verbose = 1;
static int g_sentai_emu_console_target = 1;  // 0 = usb, 1 = uart
static int g_sentai_emu_led_state = 0;
static int g_sentai_emu_reset_requested = 0;
int g_audio_debug = 0;
volatile uint32_t g_sentai_uptime_ms = 0;

int sentai_verbose_get(void) { return g_sentai_emu_verbose; }

void sentai_verbose_set(int v) {
    g_sentai_emu_verbose = v ? 1 : 0;
}

SENTAI_EMU_WEAK int sentai_uart_serial_open(void) { return 0; }
SENTAI_EMU_WEAK void sentai_uart_serial_close(void) {}
SENTAI_EMU_WEAK int sentai_uart_serial_is_open(void) { return 0; }
SENTAI_EMU_WEAK int sentai_uart_serial_write(const uint8_t* buf, int size) {
    (void)buf;
    (void)size;
    return -1;
}
SENTAI_EMU_WEAK int sentai_uart_serial_read(uint8_t* buf, int max_size,
                                            int timeout_ms) {
    (void)buf;
    (void)max_size;
    (void)timeout_ms;
    return 0;
}
SENTAI_EMU_WEAK int sentai_uart_serial_available(void) { return 0; }
SENTAI_EMU_WEAK void sentai_uart_set_baudrate(uint32_t baudrate) {
    (void)baudrate;
}
SENTAI_EMU_WEAK void sentai_uart_restore_baudrate(void) {}

SENTAI_EMU_WEAK int sentai_console_set_target(int target) {
    if (target != 0 && target != 1) return -1;
    g_sentai_emu_console_target = target;
    return 0;
}

SENTAI_EMU_WEAK int sentai_console_get_target(void) {
    return g_sentai_emu_console_target;
}

void sentai_console_write(const char* buf, int size) {
    if (!buf || size <= 0) return;
    (void)mp_hal_stdout_tx_strn(buf, (size_t)size);
}

SENTAI_EMU_WEAK void sentai_link_set_debug(int level) {
    (void)level;
}

SENTAI_EMU_WEAK void sentai_led_set(int on) {
    g_sentai_emu_led_state = on ? 1 : 0;
}

int sentai_imu_init(void) { return -1; }

int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg,
                          float* temp_c) {
    (void)x_mg;
    (void)y_mg;
    (void)z_mg;
    (void)temp_c;
    return -1;
}

int sentai_imu_tap_start(void) { return -1; }
int sentai_imu_tap_stop(void) { return -1; }

int sentai_imu_tap_poll(int timeout_ms, uint32_t* ev_out) {
    (void)timeout_ms;
    if (ev_out) *ev_out = 0;
    return -1;
}

int sentai_mic_start(int max_seconds) {
    (void)max_seconds;
    return -1;
}

int sentai_mic_stop(void) { return -1; }
int sentai_mic_busy(void) { return -1; }
int sentai_mic_samples(void) { return -1; }
int sentai_mic_level(void) { return -1; }

int sentai_mic_save_l3(char* out_name, int name_size) {
    if (out_name && name_size > 0) out_name[0] = '\0';
    return -1;
}

void sentai_sleep_ms(uint32_t ms) {
    while (ms > 0) {
        uint32_t chunk = (ms > 100u) ? 100u : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        ms -= chunk;
    }
    if (ms == 0) {
        taskYIELD();
    }
}

uint32_t sentai_ticks_ms(void) {
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    g_sentai_uptime_ms = now;
    return now;
}

SENTAI_EMU_WEAK void sentai_repl_activity(void) {}

bool sentai_is_recovery_mode(void) { return false; }

unsigned int sentai_get_boot_attempts(void) { return 0; }

void sentai_sys_do_reset(void) {
    g_sentai_emu_reset_requested = 1;
    (void)FxUserSync();
}

uint32_t sentai_get_http_requests(void) { return 0; }
uint32_t sentai_get_http_hangs(void) { return 0; }
int sentai_get_network_healthy(void) { return 1; }

const char* sentai_dmesg_basename(const char* path) {
    if (!path) return "?";
    const char* last = path;
    for (const char* p = path; *p; ++p) {
        if (*p == '/' || *p == '\\') last = p + 1;
    }
    return last;
}

void sentai_dmesg_v(dmesg_level_t level, const char* fmt, va_list ap) {
    (void)level;
    (void)fmt;
    (void)ap;
}

void sentai_dmesg(dmesg_level_t level, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    sentai_dmesg_v(level, fmt, ap);
    va_end(ap);
}

size_t sentai_dmesg_read(char* out, size_t out_size) {
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    return 0;
}

void sentai_dmesg_clear(void) {}
size_t sentai_dmesg_used(void) { return 0; }
uint32_t sentai_dmesg_dropped(void) { return 0; }

int sentai_usb_drive_get(void) { return 0; }
int sentai_usb_drive_set(int on) {
    return on ? -3 : 0;
}
int sentai_usb_serial_open(void) { return 0; }
void sentai_usb_serial_close(void) {}
int sentai_usb_serial_is_open(void) { return 0; }
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
int sentai_usb_serial_available(void) { return 0; }
int sentai_usb_ip_set(int on) {
    (void)on;
    return -1;
}
int sentai_usb_ip_get(void) { return -1; }
void sentai_httpd_start(void) {}

int sentai_fs_lock(void) { return 1; }
void sentai_fs_unlock(void) {}

int sentai_fs_size(const char* path) {
    return (int)FxUserSize(path);
}

int sentai_fs_read(const char* path, uint8_t* buf, int max_size) {
    if (!buf || max_size < 0) return 0;
    return (int)FxUserReadFile(path, buf, (size_t)max_size);
}

int sentai_fs_write(const char* path, const uint8_t* buf, int size) {
    if (size < 0) return 0;
    return FxUserWriteFile(path, buf, (size_t)size);
}

int sentai_fs_append(const char* path, const uint8_t* buf, int size) {
    if (size < 0) return 0;
    return FxUserAppendFile(path, buf, (size_t)size);
}

int sentai_fs_file_exists(const char* path) { return FxUserFileExists(path); }
int sentai_fs_dir_exists(const char* path) { return FxUserDirExists(path); }
int sentai_fs_remove(const char* path) { return FxUserRemove(path); }
int sentai_fs_makedirs(const char* path) { return FxUserMakeDirs(path); }
int sentai_fs_sync(void) { return FxUserSync(); }
int sentai_fs_format(void) { return FxUserInit(1); }

typedef struct {
    void (*callback)(const char* name, int type, int size, void* ud);
    void* user_data;
} sentai_emu_fs_list_ctx_t;

static int sentai_emu_fs_list_cb(const FxDirEntry* entry, void* user) {
    sentai_emu_fs_list_ctx_t* ctx = (sentai_emu_fs_list_ctx_t*)user;
    ctx->callback(entry->name, entry->is_dir ? 2 : 1, (int)entry->size,
                  ctx->user_data);
    return 0;
}

int sentai_fs_listdir(const char* path,
                      void (*callback)(const char* name, int type, int size,
                                       void* ud),
                      void* user_data) {
    if (!callback) return -1;
    sentai_emu_fs_list_ctx_t ctx = {
        .callback = callback,
        .user_data = user_data,
    };
    return FxUserListDir(path, sentai_emu_fs_list_cb, &ctx);
}

static void _fs_check_usb(void) {
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("flash busy: call sentai.usb.drive(0) first"));
    }
}

static const char kSentaiEmuSharedHelp[] =
    "[overview]\n"
    "SentAI ARM emulator module. Core namespaces use shared sentai_runtime "
    "bindings; emulator-specific behavior lives in FxUser/Renode backends.\n"
    "[fs]\n"
    "sentai.fs: FileX-backed filesystem: read, read_str, read_base64, write, "
    "append, size, exists, format, remove, mkdir, sync, ls.\n"
    "[rtos]\n"
    "sentai.rtos: shared FreeRTOS binding: sleep_ms, ticks_ms, uptime, "
    "repl_kick, task/heap/cpu diagnostics, dmesg.\n"
    "[io]\n"
    "sentai.io: led_on, led_off. Emulator backend updates in-memory LED state.\n"
    "[sys]\n"
    "sentai.sys: reset, recovery_mode, boot_attempts. Emulator reset records "
    "a request and flushes FileX.\n"
    "[fr]\n"
    "sentai.fr: shared binding with FxUser-backed emulator recorder for "
    "events and scalars.\n";

int sentai_help_read(char* buf, int max_size) {
    if (!buf || max_size <= 0) return -1;
    int len = (int)strlen(kSentaiEmuSharedHelp);
    int n = (len < max_size - 1) ? len : max_size - 1;
    memcpy(buf, kSentaiEmuSharedHelp, (size_t)n);
    buf[n] = '\0';
    return n;
}

typedef struct {
    sentai_fr_channel_stats_t stats;
} sentai_emu_fr_channel_t;

static sentai_emu_fr_channel_t g_sentai_emu_fr[SENTAI_FR_CH__COUNT];

static void sentai_emu_fr_parent_dir(const char* path) {
    char parent[128];
    size_t len = strlen(path);
    if (len >= sizeof(parent)) return;
    memcpy(parent, path, len + 1);
    char* slash = strrchr(parent, '/');
    if (!slash || slash == parent) return;
    *slash = '\0';
    (void)FxUserMakeDirs(parent);
}

static void sentai_emu_fr_copy_str(char* dst, size_t cap, const char* src) {
    if (cap == 0) return;
    if (!src) src = "";
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static void sentai_emu_fr_sanitize(char* s) {
    for (; s && *s; ++s) {
        if (*s == ',' || *s == '\n' || *s == '\r') *s = ' ';
    }
}

int sentai_fr_init(void) {
    memset(g_sentai_emu_fr, 0, sizeof(g_sentai_emu_fr));
    return SENTAI_FR_OK;
}

int sentai_fr_open(sentai_fr_channel_t ch, const char* path) {
    if (ch <= SENTAI_FR_CH_NONE || ch >= SENTAI_FR_CH__COUNT)
        return SENTAI_FR_ERR_UNKNOWN;
    if (!path || !*path) return SENTAI_FR_ERR_PARAMS;

    sentai_emu_fr_channel_t* channel = &g_sentai_emu_fr[ch];
    channel->stats.enabled = 1;
    sentai_emu_fr_copy_str(channel->stats.path, sizeof(channel->stats.path),
                           path);

    if (ch == SENTAI_FR_CH_EVENTS || ch == SENTAI_FR_CH_SCALARS ||
        ch == SENTAI_FR_CH_DEBUG || ch == SENTAI_FR_CH_KERNEL) {
        sentai_emu_fr_parent_dir(path);
        const char* header = "";
        if (ch == SENTAI_FR_CH_EVENTS) {
            header = "# sentai.fr events  ts_ms,type,text\n";
        } else if (ch == SENTAI_FR_CH_SCALARS) {
            header = "# sentai.fr scalars  ts_ms,label,value\n";
        }
        if (!FxUserWriteFile(path, (const uint8_t*)header, strlen(header))) {
            channel->stats.writes_fail++;
            return SENTAI_FR_ERR_IO;
        }
    } else if (ch == SENTAI_FR_CH_FRAMES) {
        if (!FxUserMakeDirs(path)) return SENTAI_FR_ERR_IO;
    }

    return SENTAI_FR_OK;
}

int sentai_fr_close(sentai_fr_channel_t ch) {
    if (ch <= SENTAI_FR_CH_NONE || ch >= SENTAI_FR_CH__COUNT)
        return SENTAI_FR_ERR_UNKNOWN;
    g_sentai_emu_fr[ch].stats.enabled = 0;
    return SENTAI_FR_OK;
}

int sentai_fr_task_start(void) { return SENTAI_FR_OK; }

int sentai_fr_task_stop(void) {
    return FxUserSync() ? SENTAI_FR_OK : SENTAI_FR_ERR_IO;
}

static int sentai_emu_fr_append(sentai_fr_channel_t ch, const char* line) {
    if (ch <= SENTAI_FR_CH_NONE || ch >= SENTAI_FR_CH__COUNT)
        return SENTAI_FR_ERR_UNKNOWN;
    sentai_emu_fr_channel_t* channel = &g_sentai_emu_fr[ch];
    if (!channel->stats.enabled) return SENTAI_FR_OK;
    channel->stats.pushes_total++;
    if (!FxUserAppendFile(channel->stats.path, (const uint8_t*)line,
                          strlen(line))) {
        channel->stats.writes_fail++;
        return SENTAI_FR_ERR_IO;
    }
    channel->stats.pushes_accepted++;
    channel->stats.writes_ok++;
    return SENTAI_FR_OK;
}

int sentai_fr_push_frame(const uint8_t* gray, int w, int h,
                         int n_dets, uint32_t seq, uint32_t ts_ms) {
    (void)gray;
    (void)w;
    (void)h;
    (void)n_dets;
    (void)seq;
    (void)ts_ms;
    if (SENTAI_FR_CH_FRAMES < SENTAI_FR_CH__COUNT)
        g_sentai_emu_fr[SENTAI_FR_CH_FRAMES].stats.pushes_total++;
    return SENTAI_FR_OK;
}

int sentai_fr_push_event(const char* type, const char* text) {
    char safe_type[SENTAI_FR_EVENT_TYPE_LEN];
    char safe_text[SENTAI_FR_EVENT_TEXT_LEN];
    sentai_emu_fr_copy_str(safe_type, sizeof(safe_type), type);
    sentai_emu_fr_copy_str(safe_text, sizeof(safe_text), text);
    sentai_emu_fr_sanitize(safe_type);
    sentai_emu_fr_sanitize(safe_text);
    char line[256];
    snprintf(line, sizeof(line), "%lu,%s,%s\n",
             (unsigned long)sentai_ticks_ms(), safe_type, safe_text);
    return sentai_emu_fr_append(SENTAI_FR_CH_EVENTS, line);
}

int sentai_fr_push_scalar(const char* label, double value, uint32_t ts_ms) {
    char safe_label[SENTAI_FR_SCALAR_LABEL_LEN];
    sentai_emu_fr_copy_str(safe_label, sizeof(safe_label), label);
    sentai_emu_fr_sanitize(safe_label);
    char line[160];
    snprintf(line, sizeof(line), "%lu,%s,%.9g\n",
             (unsigned long)(ts_ms ? ts_ms : sentai_ticks_ms()), safe_label,
             value);
    return sentai_emu_fr_append(SENTAI_FR_CH_SCALARS, line);
}

int sentai_fr_push_debug(const char* data, int len) {
    if (!data || len <= 0) return SENTAI_FR_ERR_PARAMS;
    char line[SENTAI_FR_DEBUG_TEXT_LEN + 1];
    int n = len;
    if (n > SENTAI_FR_DEBUG_TEXT_LEN) n = SENTAI_FR_DEBUG_TEXT_LEN;
    memcpy(line, data, (size_t)n);
    line[n] = '\0';
    return sentai_emu_fr_append(SENTAI_FR_CH_DEBUG, line);
}

uint32_t sentai_fr_drain_round(void) { return 0; }

int sentai_fr_get_stats(sentai_fr_channel_t ch,
                        sentai_fr_channel_stats_t* out) {
    if (!out) return SENTAI_FR_ERR_PARAMS;
    if (ch <= SENTAI_FR_CH_NONE || ch >= SENTAI_FR_CH__COUNT)
        return SENTAI_FR_ERR_UNKNOWN;
    *out = g_sentai_emu_fr[ch].stats;
    return SENTAI_FR_OK;
}

#define SENTAI_VERSION_PREFIX "SentAI EMU B9"
#include "examples/sentai_runtime/bindings/modsentai_version.c"
#include "examples/sentai_runtime/bindings/modsentai_top.c"
#include "examples/sentai_runtime/bindings/modsentai_io.c"
#include "examples/sentai_runtime/bindings/modsentai_rtos.c"
#include "examples/sentai_runtime/bindings/modsentai_fs.c"
#include "examples/sentai_runtime/bindings/modsentai_fr.c"
#include "examples/sentai_runtime/bindings/modsentai_sys.c"

#if SENTAI_EMU_CAMERA_BINDING
#include "examples/sentai_runtime/bindings/modsentai_camera.c"
#endif

#if SENTAI_EMU_MARKERS_BINDING
#include "examples/sentai_runtime/bindings/modsentai_markers.c"
#endif

#if SENTAI_EMU_FLOW_BINDING
#include "examples/sentai_runtime/bindings/modsentai_flow.c"
#endif

#if SENTAI_EMU_TPU_HOST_BRIDGE
#include "examples/sentai_runtime/sentai_tpu_shim.h"
extern int sentai_tpu_load_image_mem(const char* path, int stream_to_host);
extern uint32_t sentai_tpu_image_mem_size(void);
extern int sentai_tpu_bridge_stats(uint32_t* out, int max_words);
extern int sentai_tpu_start(void);
extern int sentai_tpu_stop(void);
extern int sentai_tpu_fps_invoke(int runs, uint32_t out[6]);
extern int sentai_tpu_fps(int runs, uint32_t out[6]);
#include "examples/sentai_runtime/bindings/modsentai_tpu.c"
#endif

#if SENTAI_EMU_TPU_HOST_BRIDGE || SENTAI_EMU_PIPELINE_PREP_BINDING
extern int sentai_tpu_is_ready(void);
#include "examples/sentai_runtime/detection_task.h"
#include "examples/sentai_runtime/sentai_tracker.h"
#include "examples/sentai_runtime/bindings/modsentai_pipeline.c"
#endif

#if SENTAI_EMU_HW_STUB_MODULES
#include "examples/sentai_runtime/bindings/modsentai_imu.c"
#include "examples/sentai_runtime/bindings/modsentai_mic.c"
#endif

#if SENTAI_EMU_SAFETY_BINDING
#include "examples/sentai_runtime/bindings/modsentai_safety.c"
#endif

#if SENTAI_EMU_SERVO_BINDING
#include "examples/sentai_runtime/bindings/modsentai_servo.c"
#endif

#if SENTAI_EMU_CALIB_BINDING
#include "examples/sentai_runtime/bindings/modsentai_calib.c"
#endif

#if SENTAI_EMU_UART_BINDING
#include "examples/sentai_runtime/bindings/modsentai_uart.c"
#endif

#if SENTAI_EMU_OBJECT_LIFTER_BINDING
#include "examples/sentai_runtime/bindings/modsentai_object_lifter.c"
#endif

#if SENTAI_EMU_USB_BINDING
#include "examples/sentai_runtime/bindings/modsentai_usb.c"
#endif

#if 0  // Legacy EMU-local core namespace bindings; kept only for reference.
static void EmuHelpPrintLines(const char* text) {
    const char* p = text;
    while (*p) {
        const char* nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        mp_printf(MP_PYTHON_PRINTER, "%.*s\r\n", (int)n, p);
        if (!nl) break;
        p = nl + 1;
    }
}

static const char kSentaiEmuHelpOverview[] =
    "SentAI ARM emulator module (B8/B9)\n"
    "Compiled namespaces: sentai.fs, sentai.rtos, sentai.fr, sentai.io, "
    "sentai.sys"
#if SENTAI_EMU_HW_STUB_MODULES
    ", sentai.usb, sentai.uart, sentai.imu, sentai.mic, "
    "sentai.servo, sentai.calib, sentai.object_lifter"
#if SENTAI_EMU_SAFETY_BINDING
    ", sentai.safety"
#endif
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE
    ", sentai.tpu, sentai.pipeline"
#elif SENTAI_EMU_PIPELINE_PREP_BINDING
    ", sentai.pipeline"
#endif
#if SENTAI_EMU_CRAZY_BINDING
    ", sentai.crazy"
#endif
#if SENTAI_EMU_CAMERA_BINDING
    ", sentai.camera"
#endif
#if SENTAI_EMU_MARKERS_BINDING
    ", sentai.markers"
#endif
#if SENTAI_EMU_FLOW_BINDING
    ", sentai.flow"
#endif
    "\n"
    "This is an emulator bring-up subset, not the full production "
    "examples/sentai_runtime/modsentai.c module yet.\n"
    "Useful commands:\n"
    "  sentai.help('fs')\n"
    "  sentai.help('io')\n"
    "  sentai.help('sys')\n"
#if SENTAI_EMU_HW_STUB_MODULES
    "  sentai.help('hardware')\n"
#endif
    "  sentai.fs.ls('/')\n"
    "  sentai.fs.size('/path')\n"
    "  sentai.rtos.ticks_ms()\n";

static const char kSentaiEmuHelpFs[] =
    "sentai.fs: FileX-backed emulator filesystem\n"
    "  ls(path) -> list of (name, type, size)\n"
    "  size(path) -> bytes or -1\n"
    "  exists(path) -> bool\n"
    "  read(path) -> bytes\n"
    "  read_str(path) -> str\n"
    "  write(path, bytes) -> bool\n"
    "  append(path, bytes) -> bool\n"
    "  mkdir(path) -> bool\n"
    "  sync() -> bool\n";

static const char kSentaiEmuHelpRtos[] =
    "sentai.rtos: minimal emulator timing helpers\n"
    "  ticks_ms() -> FreeRTOS tick count in ms\n"
    "  sleep_ms(ms) -> vTaskDelay wrapper\n"
    "  uptime() -> seconds since boot\n"
    "  repl_kick() -> emulator-safe no-op\n";

static const char kSentaiEmuHelpIo[] =
    "sentai.io: emulator-safe GPIO/LED stubs\n"
    "  led_on(), led_off() update an in-memory LED state only\n";

static const char kSentaiEmuHelpSys[] =
    "sentai.sys: emulator-safe system status stubs\n"
    "  recovery_mode() -> False\n"
    "  boot_attempts() -> 0\n"
    "  reset() records a reset request and returns without rebooting Renode\n";

static const char kSentaiEmuHelpFr[] =
    "sentai.fr: FileX-backed flight-recorder subset\n"
    "  init(), open(channel), close(channel)\n"
    "  task_start(name), task_stop(name)\n"
    "  push_event(label[, ts_ms])\n"
    "  push_scalar(label, value[, ts_ms])\n"
    "  stats(channel)\n";

#if SENTAI_EMU_HW_STUB_MODULES
static const char kSentaiEmuHelpHardware[] =
    "sentai hardware namespaces in emulator:\n"
    "  usb, uart: present as safe closed transports; no host serial/MSC yet\n"
    "  imu, mic: shared board-only bindings; unavailable in emulator\n"
    "  servo, object_lifter, calib: safe no-motion/no-flight stubs\n"
#if SENTAI_EMU_SAFETY_BINDING
    "  safety: shared sentai_runtime binding and state machine\n"
#endif
    ;
#endif

#if SENTAI_EMU_TPU_HOST_BRIDGE
static const char kSentaiEmuHelpTpu[] =
    "sentai.tpu / sentai.pipeline: B8 emulator TPU bridge surface\n"
    "Guest-side calls are bridged to the host-side physical Coral path used "
    "by the emulator smoke and timing tests.\n";
#endif

#if SENTAI_EMU_PIPELINE_PREP_BINDING
static const char kSentaiEmuHelpPipelinePrep[] =
    "sentai.pipeline: PrepTask-only emulator surface\n"
    "  prep_start(), prep_stop(), prep_fps(), prep_stats(), prep_reset()\n";
#endif

#if SENTAI_EMU_CRAZY_BINDING
static const char kSentaiEmuHelpCrazy[] =
    "sentai.crazy: shared Crazyflie CPX/CRTP binding\n"
    "  init([baud=576000]) -> int\n"
    "  ping([timeout_ms=1000]) -> int\n"
    "  canfly(), is_flying(), is_tumbled()\n"
    "  battery(), altitude(), attitude_get(), velocity()\n"
    "In the ARM emulator the UART transport is backed by the "
    "Renode CPX/UDP bridge to cf2.\n";
#endif

#if SENTAI_EMU_CAMERA_BINDING
static const char kSentaiEmuHelpCamera[] =
    "sentai.camera: shared runtime virtual-camera surface in emulator\n"
    "  select(cam_id, path) loads a staged BMP from FileX\n"
    "  prep_once() publishes the loaded frame into the common frame backend\n"
    "  frame_count(), resolution(), current_id(), grabbed_id() expose state\n";
#endif

#if SENTAI_EMU_MARKERS_BINDING
static const char kSentaiEmuHelpMarkers[] =
    "sentai.markers: unified marker namespace in emulator\n"
    "  init('whycon') selects the preferred circle-marker backend\n"
    "  detect_pgm(path) checks a staged P5 image directly\n"
    "  detect_from_camera() consumes the latest sentai.camera frame\n";
#endif

#if SENTAI_EMU_FLOW_BINDING
static const char kSentaiEmuHelpFlow[] =
    "sentai.flow: shared FlowTask binding in emulator\n"
    "  start(-1) consumes PrepTask FLOW_GRAY_80x60 slot\n"
    "  read(), read_tuple(), pub_stats(), perf()\n";
#endif

static mp_obj_t emu_sentai_help(size_t n_args, const mp_obj_t* args) {
    const char* topic = (n_args > 0) ? mp_obj_str_get_str(args[0]) : NULL;
    const char* text = kSentaiEmuHelpOverview;
    if (topic) {
        if (strcmp(topic, "fs") == 0) {
            text = kSentaiEmuHelpFs;
        } else if (strcmp(topic, "rtos") == 0) {
            text = kSentaiEmuHelpRtos;
        } else if (strcmp(topic, "io") == 0) {
            text = kSentaiEmuHelpIo;
        } else if (strcmp(topic, "sys") == 0) {
            text = kSentaiEmuHelpSys;
        } else if (strcmp(topic, "fr") == 0) {
            text = kSentaiEmuHelpFr;
#if SENTAI_EMU_HW_STUB_MODULES
        } else if (strcmp(topic, "hardware") == 0 ||
                   strcmp(topic, "usb") == 0 ||
                   strcmp(topic, "uart") == 0 ||
                   strcmp(topic, "imu") == 0 ||
                   strcmp(topic, "mic") == 0 ||
                   strcmp(topic, "sleep") == 0 ||
                   strcmp(topic, "servo") == 0 ||
                   strcmp(topic, "calib") == 0 ||
                   strcmp(topic, "object_lifter") == 0 ||
                   strcmp(topic, "safety") == 0) {
            text = kSentaiEmuHelpHardware;
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE
        } else if (strcmp(topic, "tpu") == 0 ||
                   strcmp(topic, "pipeline") == 0) {
            text = kSentaiEmuHelpTpu;
#elif SENTAI_EMU_PIPELINE_PREP_BINDING
        } else if (strcmp(topic, "pipeline") == 0) {
            text = kSentaiEmuHelpPipelinePrep;
#endif
#if SENTAI_EMU_CRAZY_BINDING
        } else if (strcmp(topic, "crazy") == 0) {
            text = kSentaiEmuHelpCrazy;
#endif
#if SENTAI_EMU_CAMERA_BINDING
        } else if (strcmp(topic, "camera") == 0) {
            text = kSentaiEmuHelpCamera;
#endif
#if SENTAI_EMU_MARKERS_BINDING
        } else if (strcmp(topic, "markers") == 0) {
            text = kSentaiEmuHelpMarkers;
#endif
#if SENTAI_EMU_FLOW_BINDING
        } else if (strcmp(topic, "flow") == 0) {
            text = kSentaiEmuHelpFlow;
#endif
        } else if (strcmp(topic, "all") != 0) {
            mp_print_str(MP_PYTHON_PRINTER,
                         "Unknown emulator help topic. Available: fs, rtos, fr, io, sys"
#if SENTAI_EMU_HW_STUB_MODULES
                         ", hardware"
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE
                         ", tpu, pipeline"
#elif SENTAI_EMU_PIPELINE_PREP_BINDING
                         ", pipeline"
#endif
#if SENTAI_EMU_CRAZY_BINDING
                         ", crazy"
#endif
#if SENTAI_EMU_CAMERA_BINDING
                         ", camera"
#endif
#if SENTAI_EMU_MARKERS_BINDING
                         ", markers"
#endif
#if SENTAI_EMU_FLOW_BINDING
                         ", flow"
#endif
                         ", all\r\n");
            return mp_const_none;
        }
    }
    EmuHelpPrintLines(text);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_sentai_help_obj, 0, 1,
                                           emu_sentai_help);

static mp_obj_t emu_sentai_version(void) {
    char version[96];
    int n = snprintf(version, sizeof(version),
                     "SentAI EMU B9 build %d (%s)",
                     BUILD_VERSION, BUILD_TIMESTAMP);
    if (n < 0) {
        return mp_obj_new_str("SentAI EMU B9 build unknown", 27);
    }
    if ((size_t)n >= sizeof(version)) {
        n = (int)sizeof(version) - 1;
    }
    return mp_obj_new_str(version, (size_t)n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_sentai_version_obj, emu_sentai_version);

static int g_emu_verbose = 1;
static int g_emu_debug = 0;
static int g_emu_console_target = 1;  // 0 = usb, 1 = uart

static mp_obj_t emu_sentai_verbose(size_t n_args, const mp_obj_t* args) {
    int prev = g_emu_verbose;
    if (n_args > 0) {
        g_emu_verbose = mp_obj_is_true(args[0]) ? 1 : 0;
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_sentai_verbose_obj, 0, 1,
                                           emu_sentai_verbose);

static mp_obj_t emu_sentai_debug(mp_obj_t level_obj) {
    g_emu_debug = mp_obj_get_int(level_obj);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_sentai_debug_obj, emu_sentai_debug);

static mp_obj_t emu_sentai_console(size_t n_args, const mp_obj_t* args) {
    if (n_args > 0) {
        const char* target = mp_obj_str_get_str(args[0]);
        if (strcmp(target, "usb") == 0) {
            g_emu_console_target = 0;
        } else if (strcmp(target, "uart") == 0) {
            g_emu_console_target = 1;
        } else {
            mp_raise_ValueError(MP_ERROR_TEXT("use 'usb' or 'uart'"));
        }
    }
    return mp_obj_new_str(g_emu_console_target == 0 ? "usb" : "uart",
                          g_emu_console_target == 0 ? 3 : 4);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_sentai_console_obj, 0, 1,
                                           emu_sentai_console);

static mp_obj_t emu_sentai_run(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    ssize_t size = FxUserSize(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    uint8_t* buf = m_new(uint8_t, (size_t)size + 1u);
    size_t n = FxUserReadFile(path, buf, (size_t)size);
    if (n == 0 && size > 0) {
        m_del(uint8_t, buf, (size_t)size + 1u);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    buf[n] = '\0';
    mp_lexer_t* lex = mp_lexer_new_from_str_len(qstr_from_str(path),
                                                (const char*)buf, n, n + 1u);
    qstr source_name = lex->source_name;
    mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
    mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
    mp_call_function_0(module_fun);
    m_del(uint8_t, buf, (size_t)size + 1u);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_sentai_run_obj, emu_sentai_run);

static mp_obj_t emu_fs_read(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    ssize_t size = FxUserSize(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    if (size == 0) {
        return mp_obj_new_bytes((const uint8_t*)"", 0);
    }
    uint8_t* buf = m_new(uint8_t, (size_t)size);
    size_t n = FxUserReadFile(path, buf, (size_t)size);
    mp_obj_t result = mp_obj_new_bytes(buf, n);
    m_del(uint8_t, buf, (size_t)size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_read_obj, emu_fs_read);

static mp_obj_t emu_fs_read_str(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    ssize_t size = FxUserSize(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    if (size == 0) {
        return mp_obj_new_str("", 0);
    }
    uint8_t* buf = m_new(uint8_t, (size_t)size);
    size_t n = FxUserReadFile(path, buf, (size_t)size);
    mp_obj_t result = mp_obj_new_str((const char*)buf, n);
    m_del(uint8_t, buf, (size_t)size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_read_str_obj, emu_fs_read_str);

static const char kB64Table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static mp_obj_t emu_fs_read_base64(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    ssize_t size = FxUserSize(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    if (size == 0) {
        return mp_obj_new_str("", 0);
    }

    uint8_t* buf = m_new(uint8_t, (size_t)size);
    size_t n = FxUserReadFile(path, buf, (size_t)size);
    if (n == 0) {
        m_del(uint8_t, buf, (size_t)size);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }

    size_t b64_len = ((n + 2u) / 3u) * 4u;
    char* b64 = m_new(char, b64_len + 1u);
    size_t j = 0;
    for (size_t i = 0; i < n; i += 3u) {
        uint32_t a = buf[i];
        uint32_t b = (i + 1u < n) ? buf[i + 1u] : 0u;
        uint32_t c = (i + 2u < n) ? buf[i + 2u] : 0u;
        uint32_t triple = (a << 16) | (b << 8) | c;
        b64[j++] = kB64Table[(triple >> 18) & 0x3Fu];
        b64[j++] = kB64Table[(triple >> 12) & 0x3Fu];
        b64[j++] = (i + 1u < n) ? kB64Table[(triple >> 6) & 0x3Fu] : '=';
        b64[j++] = (i + 2u < n) ? kB64Table[triple & 0x3Fu] : '=';
    }
    b64[j] = '\0';

    mp_obj_t result = mp_obj_new_str(b64, j);
    m_del(char, b64, b64_len + 1u);
    m_del(uint8_t, buf, (size_t)size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_read_base64_obj,
                                 emu_fs_read_base64);

static mp_obj_t emu_fs_write(mp_obj_t path_obj, mp_obj_t data_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int ok = FxUserWriteFile(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(emu_fs_write_obj, emu_fs_write);

static mp_obj_t emu_fs_append(mp_obj_t path_obj, mp_obj_t data_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int ok = FxUserAppendFile(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(emu_fs_append_obj, emu_fs_append);

static mp_obj_t emu_fs_size(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(FxUserSize(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_size_obj, emu_fs_size);

static mp_obj_t emu_fs_exists(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int exists = FxUserFileExists(path) || FxUserDirExists(path);
    return mp_obj_new_bool(exists);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_exists_obj, emu_fs_exists);

static mp_obj_t emu_fs_format(void) {
    return mp_obj_new_bool(FxUserInit(1));
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_fs_format_obj, emu_fs_format);

static mp_obj_t emu_fs_remove(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_bool(FxUserRemove(path) == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_remove_obj, emu_fs_remove);

static mp_obj_t emu_fs_mkdir(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_bool(FxUserMakeDirs(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_mkdir_obj, emu_fs_mkdir);

static mp_obj_t emu_fs_sync(void) {
    return mp_obj_new_bool(FxUserSync());
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_fs_sync_obj, emu_fs_sync);

typedef struct {
    mp_obj_list_t* list;
} emu_fs_list_ctx_t;

static int emu_fs_list_cb(const FxDirEntry* entry, void* user) {
    emu_fs_list_ctx_t* ctx = (emu_fs_list_ctx_t*)user;
    mp_obj_t items[3];
    items[0] = mp_obj_new_str(entry->name, strlen(entry->name));
    items[1] = mp_obj_new_int(entry->is_dir ? 2 : 1);
    items[2] = mp_obj_new_int(entry->size);
    mp_obj_list_append(MP_OBJ_FROM_PTR(ctx->list), mp_obj_new_tuple(3, items));
    return 0;
}

static mp_obj_t emu_fs_ls(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    emu_fs_list_ctx_t ctx = {.list = result};
    int n = FxUserListDir(path, emu_fs_list_cb, &ctx);
    if (n < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dir not found"));
    }
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fs_ls_obj, emu_fs_ls);

static const mp_rom_map_elem_t emu_fs_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fs)},
    {MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&emu_fs_read_obj)},
    {MP_ROM_QSTR(MP_QSTR_read_str), MP_ROM_PTR(&emu_fs_read_str_obj)},
    {MP_ROM_QSTR(MP_QSTR_read_base64), MP_ROM_PTR(&emu_fs_read_base64_obj)},
    {MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&emu_fs_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_append), MP_ROM_PTR(&emu_fs_append_obj)},
    {MP_ROM_QSTR(MP_QSTR_size), MP_ROM_PTR(&emu_fs_size_obj)},
    {MP_ROM_QSTR(MP_QSTR_exists), MP_ROM_PTR(&emu_fs_exists_obj)},
    {MP_ROM_QSTR(MP_QSTR_format), MP_ROM_PTR(&emu_fs_format_obj)},
    {MP_ROM_QSTR(MP_QSTR_remove), MP_ROM_PTR(&emu_fs_remove_obj)},
    {MP_ROM_QSTR(MP_QSTR_mkdir), MP_ROM_PTR(&emu_fs_mkdir_obj)},
    {MP_ROM_QSTR(MP_QSTR_sync), MP_ROM_PTR(&emu_fs_sync_obj)},
    {MP_ROM_QSTR(MP_QSTR_ls), MP_ROM_PTR(&emu_fs_ls_obj)},
};
static MP_DEFINE_CONST_DICT(emu_fs_globals, emu_fs_globals_table);

static const mp_obj_module_t emu_fs_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_fs_globals,
};

static mp_obj_t emu_rtos_ticks_ms(void) {
    return mp_obj_new_int_from_uint((uint32_t)xTaskGetTickCount());
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_rtos_ticks_ms_obj, emu_rtos_ticks_ms);

static mp_obj_t emu_rtos_sleep_ms(mp_obj_t ms_obj) {
    uint32_t ms = (uint32_t)mp_obj_get_int(ms_obj);
    if (ms == 0) {
        taskYIELD();
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_rtos_sleep_ms_obj, emu_rtos_sleep_ms);

static mp_obj_t emu_rtos_uptime(void) {
    return mp_obj_new_int((uint32_t)xTaskGetTickCount() / configTICK_RATE_HZ);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_rtos_uptime_obj, emu_rtos_uptime);

static mp_obj_t emu_rtos_repl_kick(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_rtos_repl_kick_obj, emu_rtos_repl_kick);

static const mp_rom_map_elem_t emu_rtos_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rtos)},
    {MP_ROM_QSTR(MP_QSTR_ticks_ms), MP_ROM_PTR(&emu_rtos_ticks_ms_obj)},
    {MP_ROM_QSTR(MP_QSTR_sleep_ms), MP_ROM_PTR(&emu_rtos_sleep_ms_obj)},
    {MP_ROM_QSTR(MP_QSTR_uptime), MP_ROM_PTR(&emu_rtos_uptime_obj)},
    {MP_ROM_QSTR(MP_QSTR_repl_kick), MP_ROM_PTR(&emu_rtos_repl_kick_obj)},
};
static MP_DEFINE_CONST_DICT(emu_rtos_globals, emu_rtos_globals_table);

static const mp_obj_module_t emu_rtos_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_rtos_globals,
};

static int g_emu_led_state = 0;
static int g_emu_reset_requested = 0;

static mp_obj_t emu_io_led_on(void) {
    g_emu_led_state = 1;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_io_led_on_obj, emu_io_led_on);

static mp_obj_t emu_io_led_off(void) {
    g_emu_led_state = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_io_led_off_obj, emu_io_led_off);

static const mp_rom_map_elem_t emu_io_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io)},
    {MP_ROM_QSTR(MP_QSTR_led_on), MP_ROM_PTR(&emu_io_led_on_obj)},
    {MP_ROM_QSTR(MP_QSTR_led_off), MP_ROM_PTR(&emu_io_led_off_obj)},
};
static MP_DEFINE_CONST_DICT(emu_io_globals, emu_io_globals_table);

static const mp_obj_module_t emu_io_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_io_globals,
};

static mp_obj_t emu_sys_reset(void) {
    g_emu_reset_requested = 1;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_sys_reset_obj, emu_sys_reset);

static mp_obj_t emu_sys_recovery_mode(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_sys_recovery_mode_obj,
                                 emu_sys_recovery_mode);

static mp_obj_t emu_sys_boot_attempts(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_sys_boot_attempts_obj,
                                 emu_sys_boot_attempts);

static const mp_rom_map_elem_t emu_sys_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sys)},
    {MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&emu_sys_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_recovery_mode),
     MP_ROM_PTR(&emu_sys_recovery_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_boot_attempts),
     MP_ROM_PTR(&emu_sys_boot_attempts_obj)},
};
static MP_DEFINE_CONST_DICT(emu_sys_globals, emu_sys_globals_table);

static const mp_obj_module_t emu_sys_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_sys_globals,
};
#endif  // Legacy EMU-local core namespace bindings.

#if 0  // Legacy EMU-local sentai.servo binding; shared binding is used instead.
enum {
    kServoBackendNone = 0,
    kServoBackendSim = 1,
    kServoBackendCf2 = 2,
    kServoBackendPx4 = 3,
    kServoFlightGround = 0,
    kServoFlightAirborne = 1,
    kServoActNone = 0,
    kServoActInit = 1,
    kServoActArm = 2,
    kServoActDisarm = 3,
    kServoActTakeoff = 4,
    kServoActMove = 5,
    kServoActHover = 6,
    kServoActLand = 7,
    kServoActGoTo = 8,
};

static int g_emu_servo_backend = kServoBackendNone;
static int g_emu_servo_seq = 0;
static int g_emu_servo_last_action = kServoActNone;
static int g_emu_servo_last_result = 0;
static int g_emu_servo_faults_no_backend = 0;
static int g_emu_servo_faults_not_armed = 0;
static int g_emu_servo_faults_oob = 0;

static int EmuServoResolveBackend(mp_obj_t arg) {
    if (mp_obj_is_int(arg)) {
        int v = mp_obj_get_int(arg);
        if (v >= kServoBackendSim && v <= kServoBackendPx4) return v;
        return kServoBackendNone;
    }
    size_t len = 0;
    const char* s = mp_obj_str_get_data(arg, &len);
    if (len == 3 && memcmp(s, "sim", 3) == 0) return kServoBackendSim;
    if (len == 3 && memcmp(s, "cf2", 3) == 0) return kServoBackendCf2;
    if (len == 3 && memcmp(s, "px4", 3) == 0) return kServoBackendPx4;
    return kServoBackendNone;
}

static int EmuServoRecord(int action, int result) {
    g_emu_servo_seq++;
    g_emu_servo_last_action = action;
    g_emu_servo_last_result = result;
    return result;
}

static mp_obj_t emu_servo_init(mp_obj_t backend_obj) {
    int backend = EmuServoResolveBackend(backend_obj);
    g_emu_servo_backend = backend;
    return mp_obj_new_int(EmuServoRecord(kServoActInit,
                                         backend == kServoBackendNone ? -1 : 0));
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_servo_init_obj, emu_servo_init);

static mp_obj_t emu_servo_disarm(void) {
    return mp_obj_new_int(EmuServoRecord(kServoActDisarm, 0));
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_disarm_obj, emu_servo_disarm);

static mp_obj_t emu_servo_reject_action(int action) {
    if (g_emu_servo_backend == kServoBackendNone) g_emu_servo_faults_no_backend++;
    g_emu_servo_faults_not_armed++;
    return mp_obj_new_int(EmuServoRecord(action, -3));
}

static mp_obj_t emu_servo_arm(void) {
    return emu_servo_reject_action(kServoActArm);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_arm_obj, emu_servo_arm);

static mp_obj_t emu_servo_takeoff(mp_obj_t alt_obj) {
    (void)alt_obj;
    return emu_servo_reject_action(kServoActTakeoff);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_servo_takeoff_obj, emu_servo_takeoff);

static mp_obj_t emu_servo_move(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return emu_servo_reject_action(kServoActMove);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_servo_move_obj, 3, 4,
                                           emu_servo_move);

static mp_obj_t emu_servo_go_to(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return emu_servo_reject_action(kServoActGoTo);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_servo_go_to_obj, 3, 4,
                                           emu_servo_go_to);

static mp_obj_t emu_servo_hover(void) {
    return emu_servo_reject_action(kServoActHover);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_hover_obj, emu_servo_hover);

static mp_obj_t emu_servo_land(void) {
    return emu_servo_reject_action(kServoActLand);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_land_obj, emu_servo_land);

static mp_obj_t emu_servo_status(void) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(13));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_backend),
                      mp_obj_new_int(g_emu_servo_backend));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_armed), mp_obj_new_int(0));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_flight),
                      mp_obj_new_int(kServoFlightGround));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_seq),
                      mp_obj_new_int(g_emu_servo_seq));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_last_action),
                      mp_obj_new_int(g_emu_servo_last_action));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_last_result),
                      mp_obj_new_int(g_emu_servo_last_result));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_trace_count),
                      mp_obj_new_int(g_emu_servo_seq > 0 ? 1 : 0));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_actions_ok), mp_obj_new_int(0));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_faults_no_backend),
                      mp_obj_new_int(g_emu_servo_faults_no_backend));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_faults_not_armed),
                      mp_obj_new_int(g_emu_servo_faults_not_armed));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_faults_oob),
                      mp_obj_new_int(g_emu_servo_faults_oob));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_trace_overwrites),
                      mp_obj_new_int(0));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_status_obj, emu_servo_status);

static mp_obj_t emu_servo_trace(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_list(0, NULL);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_servo_trace_obj, 0, 1,
                                           emu_servo_trace);

static mp_obj_t emu_servo_clear_trace(void) {
    g_emu_servo_seq = 0;
    g_emu_servo_last_action = kServoActNone;
    g_emu_servo_last_result = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_clear_trace_obj,
                                 emu_servo_clear_trace);

static mp_obj_t emu_servo_pose(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_pose_obj, emu_servo_pose);

static mp_obj_t emu_servo_pose_ready(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_servo_pose_ready_obj,
                                 emu_servo_pose_ready);

static mp_obj_t emu_servo_set_durations(mp_obj_t a, mp_obj_t b, mp_obj_t c) {
    (void)a;
    (void)b;
    (void)c;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_3(emu_servo_set_durations_obj,
                                 emu_servo_set_durations);

static const mp_rom_map_elem_t emu_servo_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_servo)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&emu_servo_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_arm), MP_ROM_PTR(&emu_servo_arm_obj)},
    {MP_ROM_QSTR(MP_QSTR_disarm), MP_ROM_PTR(&emu_servo_disarm_obj)},
    {MP_ROM_QSTR(MP_QSTR_takeoff), MP_ROM_PTR(&emu_servo_takeoff_obj)},
    {MP_ROM_QSTR(MP_QSTR_move), MP_ROM_PTR(&emu_servo_move_obj)},
    {MP_ROM_QSTR(MP_QSTR_go_to), MP_ROM_PTR(&emu_servo_go_to_obj)},
    {MP_ROM_QSTR(MP_QSTR_hover), MP_ROM_PTR(&emu_servo_hover_obj)},
    {MP_ROM_QSTR(MP_QSTR_land), MP_ROM_PTR(&emu_servo_land_obj)},
    {MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&emu_servo_status_obj)},
    {MP_ROM_QSTR(MP_QSTR_trace), MP_ROM_PTR(&emu_servo_trace_obj)},
    {MP_ROM_QSTR(MP_QSTR_clear_trace), MP_ROM_PTR(&emu_servo_clear_trace_obj)},
    {MP_ROM_QSTR(MP_QSTR_pose), MP_ROM_PTR(&emu_servo_pose_obj)},
    {MP_ROM_QSTR(MP_QSTR_pose_ready), MP_ROM_PTR(&emu_servo_pose_ready_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_durations),
     MP_ROM_PTR(&emu_servo_set_durations_obj)},
    {MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_INT(kServoBackendNone)},
    {MP_ROM_QSTR(MP_QSTR_SIM), MP_ROM_INT(kServoBackendSim)},
    {MP_ROM_QSTR(MP_QSTR_CF2), MP_ROM_INT(kServoBackendCf2)},
    {MP_ROM_QSTR(MP_QSTR_PX4), MP_ROM_INT(kServoBackendPx4)},
    {MP_ROM_QSTR(MP_QSTR_GROUND), MP_ROM_INT(kServoFlightGround)},
    {MP_ROM_QSTR(MP_QSTR_AIRBORNE), MP_ROM_INT(kServoFlightAirborne)},
    {MP_ROM_QSTR(MP_QSTR_ACT_NONE), MP_ROM_INT(kServoActNone)},
    {MP_ROM_QSTR(MP_QSTR_ACT_INIT), MP_ROM_INT(kServoActInit)},
    {MP_ROM_QSTR(MP_QSTR_ACT_ARM), MP_ROM_INT(kServoActArm)},
    {MP_ROM_QSTR(MP_QSTR_ACT_DISARM), MP_ROM_INT(kServoActDisarm)},
    {MP_ROM_QSTR(MP_QSTR_ACT_TAKEOFF), MP_ROM_INT(kServoActTakeoff)},
    {MP_ROM_QSTR(MP_QSTR_ACT_MOVE), MP_ROM_INT(kServoActMove)},
    {MP_ROM_QSTR(MP_QSTR_ACT_HOVER), MP_ROM_INT(kServoActHover)},
    {MP_ROM_QSTR(MP_QSTR_ACT_LAND), MP_ROM_INT(kServoActLand)},
    {MP_ROM_QSTR(MP_QSTR_ACT_GO_TO), MP_ROM_INT(kServoActGoTo)},
};
static MP_DEFINE_CONST_DICT(emu_servo_globals, emu_servo_globals_table);

static const mp_obj_module_t emu_servo_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_servo_globals,
};
#endif

#if 0  // Legacy EMU-local sentai.fr binding; shared binding is used above.
enum {
    kFrEvents = 0,
    kFrScalars = 1,
    kFrChannelCount = 2,
};

typedef struct {
    char path[128];
    uint32_t pushes_total;
    uint32_t pushes_accepted;
    uint32_t writes_ok;
    uint32_t writes_fail;
    bool open;
} emu_fr_channel_t;

static emu_fr_channel_t g_fr_channels[kFrChannelCount];

static int FrParseChannel(mp_obj_t name_obj) {
    size_t len = 0;
    const char* name = mp_obj_str_get_data(name_obj, &len);
    if (len == 6 && memcmp(name, "events", 6) == 0) return kFrEvents;
    if (len == 7 && memcmp(name, "scalars", 7) == 0) return kFrScalars;
    return -1;
}

static void FrMakeParentDir(const char* path) {
    char parent[128];
    size_t len = strlen(path);
    if (len >= sizeof(parent)) return;
    memcpy(parent, path, len + 1);
    char* slash = strrchr(parent, '/');
    if (!slash || slash == parent) return;
    *slash = '\0';
    (void)FxUserMakeDirs(parent);
}

static int FrAppendLine(int channel, const char* line) {
    if (channel < 0 || channel >= kFrChannelCount) return -2;
    emu_fr_channel_t* ch = &g_fr_channels[channel];
    if (!ch->open) return -3;
    ch->pushes_total++;
    const bool ok = FxUserAppendFile(ch->path, (const uint8_t*)line,
                                     strlen(line));
    if (ok) {
        ch->pushes_accepted++;
        ch->writes_ok++;
        return 0;
    }
    ch->writes_fail++;
    return -4;
}

static void FrValueToString(mp_obj_t value_obj, char* out, size_t out_size) {
    if (mp_obj_is_int(value_obj)) {
        snprintf(out, out_size, "%ld", (long)mp_obj_get_int(value_obj));
        return;
    }
    double value = mp_obj_get_float(value_obj);
    long whole = (long)value;
    double frac_d = value - (double)whole;
    if (frac_d < 0) frac_d = -frac_d;
    long frac = (long)(frac_d * 1000.0 + 0.5);
    if (frac >= 1000) {
        frac -= 1000;
        whole += (value < 0) ? -1 : 1;
    }
    snprintf(out, out_size, "%ld.%03ld", whole, frac);
}

static mp_obj_t emu_fr_init(void) {
    memset(g_fr_channels, 0, sizeof(g_fr_channels));
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_fr_init_obj, emu_fr_init);

static mp_obj_t emu_fr_open(mp_obj_t name_obj, mp_obj_t path_obj) {
    int channel = FrParseChannel(name_obj);
    if (channel < 0) return mp_obj_new_int(-2);
    size_t path_len = 0;
    const char* path = mp_obj_str_get_data(path_obj, &path_len);
    if (path_len == 0 || path_len >= sizeof(g_fr_channels[channel].path)) {
        return mp_obj_new_int(-3);
    }
    memcpy(g_fr_channels[channel].path, path, path_len);
    g_fr_channels[channel].path[path_len] = '\0';
    FrMakeParentDir(g_fr_channels[channel].path);
    const char* header = (channel == kFrEvents)
                             ? "# sentai.fr events ts_ms,type,text\n"
                             : "# sentai.fr scalars ts_ms,label,value\n";
    if (!FxUserWriteFile(g_fr_channels[channel].path,
                         (const uint8_t*)header, strlen(header))) {
        return mp_obj_new_int(-4);
    }
    g_fr_channels[channel].open = true;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_2(emu_fr_open_obj, emu_fr_open);

static mp_obj_t emu_fr_close(mp_obj_t name_obj) {
    int channel = FrParseChannel(name_obj);
    if (channel < 0) return mp_obj_new_int(-2);
    g_fr_channels[channel].open = false;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fr_close_obj, emu_fr_close);

static mp_obj_t emu_fr_task_start(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_fr_task_start_obj, emu_fr_task_start);

static mp_obj_t emu_fr_task_stop(void) {
    (void)FxUserSync();
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_fr_task_stop_obj, emu_fr_task_stop);

static mp_obj_t emu_fr_push_event(mp_obj_t type_obj, mp_obj_t text_obj) {
    const char* type = mp_obj_str_get_str(type_obj);
    const char* text = mp_obj_str_get_str(text_obj);
    char line[256];
    snprintf(line, sizeof(line), "%lu,%s,%s\n",
             (unsigned long)xTaskGetTickCount(), type, text);
    return mp_obj_new_int(FrAppendLine(kFrEvents, line));
}
static MP_DEFINE_CONST_FUN_OBJ_2(emu_fr_push_event_obj, emu_fr_push_event);

static mp_obj_t emu_fr_push_scalar(size_t n_args, const mp_obj_t* args) {
    const char* label = mp_obj_str_get_str(args[0]);
    uint32_t ts_ms =
        (n_args >= 3) ? (uint32_t)mp_obj_get_int(args[2])
                      : (uint32_t)xTaskGetTickCount();
    char value[48];
    FrValueToString(args[1], value, sizeof(value));
    char line[192];
    snprintf(line, sizeof(line), "%lu,%s,%s\n",
             (unsigned long)ts_ms, label, value);
    return mp_obj_new_int(FrAppendLine(kFrScalars, line));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_fr_push_scalar_obj, 2, 3,
                                           emu_fr_push_scalar);

static mp_obj_t emu_fr_stats(mp_obj_t name_obj) {
    int channel = FrParseChannel(name_obj);
    if (channel < 0) return mp_const_none;
    emu_fr_channel_t* ch = &g_fr_channels[channel];
    mp_obj_t items[7] = {
        mp_obj_new_int_from_uint(ch->pushes_total),
        mp_obj_new_int_from_uint(ch->pushes_accepted),
        mp_obj_new_int(0),
        mp_obj_new_int_from_uint(ch->writes_ok),
        mp_obj_new_int_from_uint(ch->writes_fail),
        mp_obj_new_int(0),
        mp_obj_new_int(0),
    };
    return mp_obj_new_tuple(7, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_fr_stats_obj, emu_fr_stats);

static const mp_rom_map_elem_t emu_fr_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fr)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&emu_fr_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&emu_fr_open_obj)},
    {MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&emu_fr_close_obj)},
    {MP_ROM_QSTR(MP_QSTR_task_start), MP_ROM_PTR(&emu_fr_task_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_task_stop), MP_ROM_PTR(&emu_fr_task_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_push_event), MP_ROM_PTR(&emu_fr_push_event_obj)},
    {MP_ROM_QSTR(MP_QSTR_push_scalar), MP_ROM_PTR(&emu_fr_push_scalar_obj)},
    {MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&emu_fr_stats_obj)},
};
static MP_DEFINE_CONST_DICT(emu_fr_globals, emu_fr_globals_table);

static const mp_obj_module_t emu_fr_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_fr_globals,
};
#endif  // Legacy EMU-local sentai.fr binding.

static const mp_rom_map_elem_t emu_sentai_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai)},
    {MP_ROM_QSTR(MP_QSTR_version), MP_ROM_PTR(&mod_sentai_version_obj)},
    {MP_ROM_QSTR(MP_QSTR_verbose), MP_ROM_PTR(&mod_sentai_verbose_obj)},
    {MP_ROM_QSTR(MP_QSTR_help), MP_ROM_PTR(&mod_sentai_help_obj)},
    {MP_ROM_QSTR(MP_QSTR_debug), MP_ROM_PTR(&mod_sentai_debug_obj)},
    {MP_ROM_QSTR(MP_QSTR_console), MP_ROM_PTR(&mod_sentai_console_obj)},
    {MP_ROM_QSTR(MP_QSTR_run), MP_ROM_PTR(&mod_sentai_run_obj)},
    {MP_ROM_QSTR(MP_QSTR_io), MP_ROM_PTR(&sentai_io_module)},
    {MP_ROM_QSTR(MP_QSTR_fs), MP_ROM_PTR(&sentai_fs_module)},
    {MP_ROM_QSTR(MP_QSTR_rtos), MP_ROM_PTR(&sentai_rtos_module)},
    {MP_ROM_QSTR(MP_QSTR_fr), MP_ROM_PTR(&sentai_fr_module)},
    {MP_ROM_QSTR(MP_QSTR_sys), MP_ROM_PTR(&sentai_sys_module)},
#if SENTAI_EMU_HW_STUB_MODULES
    {MP_ROM_QSTR(MP_QSTR_imu), MP_ROM_PTR(&sentai_imu_module)},
    {MP_ROM_QSTR(MP_QSTR_mic), MP_ROM_PTR(&sentai_mic_module)},
#endif
#if SENTAI_EMU_USB_BINDING
    {MP_ROM_QSTR(MP_QSTR_usb), MP_ROM_PTR(&sentai_usb_module)},
#endif
#if SENTAI_EMU_UART_BINDING
    {MP_ROM_QSTR(MP_QSTR_uart), MP_ROM_PTR(&sentai_uart_module)},
#endif
#if SENTAI_EMU_OBJECT_LIFTER_BINDING
    {MP_ROM_QSTR(MP_QSTR_object_lifter),
     MP_ROM_PTR(&sentai_object_lifter_module)},
#endif
#if SENTAI_EMU_SAFETY_BINDING
    {MP_ROM_QSTR(MP_QSTR_safety), MP_ROM_PTR(&sentai_safety_module)},
#endif
#if SENTAI_EMU_SERVO_BINDING
    {MP_ROM_QSTR(MP_QSTR_servo), MP_ROM_PTR(&sentai_servo_module)},
#endif
#if SENTAI_EMU_CALIB_BINDING
    {MP_ROM_QSTR(MP_QSTR_calib), MP_ROM_PTR(&sentai_calib_module)},
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE
    {MP_ROM_QSTR(MP_QSTR_tpu), MP_ROM_PTR(&sentai_tpu_module)},
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE || SENTAI_EMU_PIPELINE_PREP_BINDING
    {MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&sentai_pipeline_module)},
#endif
#if SENTAI_EMU_CRAZY_BINDING
    {MP_ROM_QSTR(MP_QSTR_crazy), MP_ROM_PTR(&sentai_crazy_module)},
#endif
#if SENTAI_EMU_CAMERA_BINDING
    {MP_ROM_QSTR(MP_QSTR_camera), MP_ROM_PTR(&sentai_camera_module)},
#endif
#if SENTAI_EMU_MARKERS_BINDING
    {MP_ROM_QSTR(MP_QSTR_markers), MP_ROM_PTR(&sentai_markers_module)},
#endif
#if SENTAI_EMU_FLOW_BINDING
    {MP_ROM_QSTR(MP_QSTR_flow), MP_ROM_PTR(&sentai_flow_module)},
#endif
};
static MP_DEFINE_CONST_DICT(emu_sentai_globals, emu_sentai_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_sentai_globals,
};
