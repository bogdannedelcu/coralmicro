// B8 ARM-emulator `sentai.fs` module.
//
// This is intentionally a thin MicroPython binding over the production
// FxUser* API.  It does not introduce a RAM filesystem and does not bypass
// FileX/LevelX; the only emulator substitution lives below fx_nand_driver_*.

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "libs/base/fx_user_fs.h"
#include "py/obj.h"
#include "py/compile.h"
#include "py/lexer.h"
#include "py/mpprint.h"
#include "py/parse.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#if SENTAI_EMU_TPU_HOST_BRIDGE || SENTAI_EMU_PIPELINE_PREP_BINDING
extern const mp_obj_module_t emu_pipeline_module;
#endif

#if SENTAI_EMU_TPU_HOST_BRIDGE
extern const mp_obj_module_t emu_tpu_module;
#endif

#ifndef SENTAI_EMU_HW_STUB_MODULES
#define SENTAI_EMU_HW_STUB_MODULES 0
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

#if SENTAI_EMU_CAMERA_BINDING || SENTAI_EMU_MARKERS_BINDING
static void _fs_check_usb(void) {}
#endif

#if SENTAI_EMU_CAMERA_BINDING
#include "examples/sentai_runtime/bindings/modsentai_camera.c"
#endif

#if SENTAI_EMU_MARKERS_BINDING
#include "examples/sentai_runtime/bindings/modsentai_markers.c"
#endif

#if SENTAI_EMU_FLOW_BINDING
#include "examples/sentai_runtime/bindings/modsentai_flow.c"
#endif

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
    "sentai.sleep, sentai.servo, sentai.calib, sentai.object_lifter, "
    "sentai.safety"
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
    "  imu: deterministic level sample, no tap hardware\n"
    "  mic: inactive no-audio stub\n"
    "  sleep: timeout-only idle shim\n"
    "  servo, object_lifter, calib, safety: safe no-motion/no-flight stubs\n";
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
    return mp_obj_new_str("SentAI EMU B8", 13);
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

#if SENTAI_EMU_HW_STUB_MODULES
static int g_emu_usb_drive = 0;

static mp_obj_t emu_usb_drive(mp_obj_t on_obj) {
    int on = mp_obj_get_int(on_obj);
    if (on) return mp_obj_new_int(-3);
    g_emu_usb_drive = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_usb_drive_obj, emu_usb_drive);

static mp_obj_t emu_usb_ip(mp_obj_t on_obj) {
    (void)on_obj;
    return mp_obj_new_int(-1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_usb_ip_obj, emu_usb_ip);

static mp_obj_t emu_usb_open(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_usb_open_obj, emu_usb_open);

static mp_obj_t emu_usb_close(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_usb_close_obj, emu_usb_close);

static mp_obj_t emu_usb_write(mp_obj_t data_obj) {
    (void)data_obj;
    return mp_obj_new_int(-1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_usb_write_obj, emu_usb_write);

static mp_obj_t emu_usb_read(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_bytes((const uint8_t*)"", 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_usb_read_obj, 0, 2,
                                           emu_usb_read);

static mp_obj_t emu_usb_available(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_usb_available_obj, emu_usb_available);

static const mp_rom_map_elem_t emu_usb_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_usb)},
    {MP_ROM_QSTR(MP_QSTR_drive), MP_ROM_PTR(&emu_usb_drive_obj)},
    {MP_ROM_QSTR(MP_QSTR_ip), MP_ROM_PTR(&emu_usb_ip_obj)},
    {MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&emu_usb_open_obj)},
    {MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&emu_usb_close_obj)},
    {MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&emu_usb_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&emu_usb_read_obj)},
    {MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&emu_usb_available_obj)},
};
static MP_DEFINE_CONST_DICT(emu_usb_globals, emu_usb_globals_table);

static const mp_obj_module_t emu_usb_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_usb_globals,
};

static mp_obj_t emu_uart_open(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_uart_open_obj, 0, 1,
                                           emu_uart_open);

static mp_obj_t emu_uart_close(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_uart_close_obj, emu_uart_close);

static mp_obj_t emu_uart_write(mp_obj_t data_obj) {
    (void)data_obj;
    return mp_obj_new_int(-1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_uart_write_obj, emu_uart_write);

static mp_obj_t emu_uart_read(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_bytes((const uint8_t*)"", 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_uart_read_obj, 0, 2,
                                           emu_uart_read);

static mp_obj_t emu_uart_available(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_uart_available_obj, emu_uart_available);

static const mp_rom_map_elem_t emu_uart_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_uart)},
    {MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&emu_uart_open_obj)},
    {MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&emu_uart_close_obj)},
    {MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&emu_uart_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&emu_uart_read_obj)},
    {MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&emu_uart_available_obj)},
};
static MP_DEFINE_CONST_DICT(emu_uart_globals, emu_uart_globals_table);

static const mp_obj_module_t emu_uart_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_uart_globals,
};

static mp_obj_t emu_imu_init(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_init_obj, emu_imu_init);

static mp_obj_t emu_imu_read(void) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_float(0.0f));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_float(0.0f));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_z), mp_obj_new_float(1000.0f));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(25.0f));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_read_obj, emu_imu_read);

static mp_obj_t emu_imu_degrees(void) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(3));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_pitch), mp_obj_new_float(0.0f));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_roll), mp_obj_new_float(0.0f));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(25.0f));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_degrees_obj, emu_imu_degrees);

static mp_obj_t emu_imu_radians(void) {
    return emu_imu_degrees();
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_radians_obj, emu_imu_radians);

static mp_obj_t emu_imu_tap_start(void) {
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_tap_start_obj, emu_imu_tap_start);

static mp_obj_t emu_imu_tap_stop(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_imu_tap_stop_obj, emu_imu_tap_stop);

static mp_obj_t emu_imu_poll_event(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_imu_poll_event_obj, 0, 1,
                                           emu_imu_poll_event);

static const mp_rom_map_elem_t emu_imu_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_imu)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&emu_imu_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&emu_imu_read_obj)},
    {MP_ROM_QSTR(MP_QSTR_degrees), MP_ROM_PTR(&emu_imu_degrees_obj)},
    {MP_ROM_QSTR(MP_QSTR_radians), MP_ROM_PTR(&emu_imu_radians_obj)},
    {MP_ROM_QSTR(MP_QSTR_tap_start), MP_ROM_PTR(&emu_imu_tap_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_tap_stop), MP_ROM_PTR(&emu_imu_tap_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_poll_event), MP_ROM_PTR(&emu_imu_poll_event_obj)},
};
static MP_DEFINE_CONST_DICT(emu_imu_globals, emu_imu_globals_table);

static const mp_obj_module_t emu_imu_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_imu_globals,
};

static mp_obj_t emu_mic_start(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_mic_start_obj, 0, 1,
                                           emu_mic_start);

static mp_obj_t emu_mic_stop(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_mic_stop_obj, emu_mic_stop);

static mp_obj_t emu_mic_recording(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_mic_recording_obj, emu_mic_recording);

static mp_obj_t emu_mic_samples(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_mic_samples_obj, emu_mic_samples);

static mp_obj_t emu_mic_save_mp3(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_mic_save_mp3_obj, emu_mic_save_mp3);

static mp_obj_t emu_mic_level(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_mic_level_obj, emu_mic_level);

static const mp_rom_map_elem_t emu_mic_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_mic)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&emu_mic_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&emu_mic_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_recording), MP_ROM_PTR(&emu_mic_recording_obj)},
    {MP_ROM_QSTR(MP_QSTR_samples), MP_ROM_PTR(&emu_mic_samples_obj)},
    {MP_ROM_QSTR(MP_QSTR_save_mp3), MP_ROM_PTR(&emu_mic_save_mp3_obj)},
    {MP_ROM_QSTR(MP_QSTR_level), MP_ROM_PTR(&emu_mic_level_obj)},
};
static MP_DEFINE_CONST_DICT(emu_mic_globals, emu_mic_globals_table);

static const mp_obj_module_t emu_mic_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_mic_globals,
};

static mp_obj_t emu_sleep_idle(size_t n_args, const mp_obj_t* args) {
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    if (timeout_ms > 0) {
        if (timeout_ms > 100) timeout_ms = 100;
        vTaskDelay(pdMS_TO_TICKS((uint32_t)timeout_ms));
    } else {
        taskYIELD();
    }
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_sleep_idle_obj, 0, 3,
                                           emu_sleep_idle);

static const mp_rom_map_elem_t emu_sleep_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sleep)},
    {MP_ROM_QSTR(MP_QSTR_idle), MP_ROM_PTR(&emu_sleep_idle_obj)},
};
static MP_DEFINE_CONST_DICT(emu_sleep_globals, emu_sleep_globals_table);

static const mp_obj_module_t emu_sleep_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_sleep_globals,
};

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

static const float kIdentityR[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};
static const float kZero3[3] = {0.0f, 0.0f, 0.0f};

static mp_obj_t EmuTupleFromFloats(const float* values, size_t n) {
    mp_obj_t* items = m_new(mp_obj_t, n);
    for (size_t i = 0; i < n; ++i) {
        items[i] = mp_obj_new_float(values[i]);
    }
    mp_obj_t result = mp_obj_new_tuple(n, items);
    m_del(mp_obj_t, items, n);
    return result;
}

static mp_obj_t emu_calib_init(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_init_obj, emu_calib_init);

static mp_obj_t emu_calib_clear(void) {
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_clear_obj, emu_calib_clear);

static mp_obj_t emu_calib_run_kabsch(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    mp_obj_dict_t* q = MP_OBJ_TO_PTR(mp_obj_new_dict(3));
    mp_obj_dict_store(q, MP_OBJ_NEW_QSTR(MP_QSTR_n_samples), mp_obj_new_int(0));
    mp_obj_dict_store(q, MP_OBJ_NEW_QSTR(MP_QSTR_accepted), mp_const_false);
    mp_obj_dict_store(q, MP_OBJ_NEW_QSTR(MP_QSTR_reject_code),
                      mp_obj_new_int(-3));
    mp_obj_t out[2] = {
        EmuTupleFromFloats(kIdentityR, 9),
        MP_OBJ_FROM_PTR(q),
    };
    return mp_obj_new_tuple(2, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_calib_run_kabsch_obj, 1, 2,
                                           emu_calib_run_kabsch);

static mp_obj_t emu_calib_commit_R(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_calib_commit_R_obj, 1, 2,
                                           emu_calib_commit_R);

static mp_obj_t emu_calib_save(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_save_obj, emu_calib_save);

static mp_obj_t emu_calib_load(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_load_obj, emu_calib_load);

static mp_obj_t emu_calib_get_R(void) {
    return EmuTupleFromFloats(kIdentityR, 9);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_get_R_obj, emu_calib_get_R);

static mp_obj_t emu_calib_get_off(void) {
    return EmuTupleFromFloats(kZero3, 3);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_get_off_obj, emu_calib_get_off);

static mp_obj_t emu_calib_is_calibrated(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_calib_is_calibrated_obj,
                                 emu_calib_is_calibrated);

static const mp_rom_map_elem_t emu_calib_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_calib)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&emu_calib_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&emu_calib_clear_obj)},
    {MP_ROM_QSTR(MP_QSTR_run_kabsch), MP_ROM_PTR(&emu_calib_run_kabsch_obj)},
    {MP_ROM_QSTR(MP_QSTR_commit_R), MP_ROM_PTR(&emu_calib_commit_R_obj)},
    {MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&emu_calib_save_obj)},
    {MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&emu_calib_load_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_R_cam_to_body),
     MP_ROM_PTR(&emu_calib_get_R_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_cam_offset_B),
     MP_ROM_PTR(&emu_calib_get_off_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_calibrated),
     MP_ROM_PTR(&emu_calib_is_calibrated_obj)},
};
static MP_DEFINE_CONST_DICT(emu_calib_globals, emu_calib_globals_table);

static const mp_obj_module_t emu_calib_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_calib_globals,
};

static mp_obj_t emu_lifter_init_from_bbox(size_t n_args,
                                          const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_lifter_init_from_bbox_obj, 8, 8,
                                           emu_lifter_init_from_bbox);

static mp_obj_t emu_lifter_update_bbox(size_t n_args,
                                       const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_lifter_update_bbox_obj, 6, 6,
                                           emu_lifter_update_bbox);

static mp_obj_t emu_lifter_get(mp_obj_t tid_obj) {
    (void)tid_obj;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_lifter_get_obj, emu_lifter_get);

static mp_obj_t emu_lifter_world_pos(mp_obj_t tid_obj) {
    (void)tid_obj;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_lifter_world_pos_obj,
                                 emu_lifter_world_pos);

static mp_obj_t emu_lifter_list(void) {
    return mp_obj_new_list(0, NULL);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_lifter_list_obj, emu_lifter_list);

static mp_obj_t emu_lifter_count(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_lifter_count_obj, emu_lifter_count);

static mp_obj_t emu_lifter_mark_lost(mp_obj_t tid_obj) {
    (void)tid_obj;
    return mp_obj_new_int(-1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_lifter_mark_lost_obj,
                                 emu_lifter_mark_lost);

static mp_obj_t emu_lifter_clear(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_lifter_clear_obj, emu_lifter_clear);

static mp_obj_t emu_lifter_stats(void) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(3));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_used), mp_obj_new_int(0));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_hwm), mp_obj_new_int(0));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_capacity), mp_obj_new_int(0));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_lifter_stats_obj, emu_lifter_stats);

static mp_obj_t emu_lifter_set_camera(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_lifter_set_camera_obj, 6, 6,
                                           emu_lifter_set_camera);

static mp_obj_t emu_lifter_inject(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_lifter_inject_obj, 5, 5,
                                           emu_lifter_inject);

static const mp_rom_map_elem_t emu_object_lifter_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_object_lifter)},
    {MP_ROM_QSTR(MP_QSTR_init_from_bbox),
     MP_ROM_PTR(&emu_lifter_init_from_bbox_obj)},
    {MP_ROM_QSTR(MP_QSTR_update_bbox),
     MP_ROM_PTR(&emu_lifter_update_bbox_obj)},
    {MP_ROM_QSTR(MP_QSTR_get), MP_ROM_PTR(&emu_lifter_get_obj)},
    {MP_ROM_QSTR(MP_QSTR_world_pos), MP_ROM_PTR(&emu_lifter_world_pos_obj)},
    {MP_ROM_QSTR(MP_QSTR_list), MP_ROM_PTR(&emu_lifter_list_obj)},
    {MP_ROM_QSTR(MP_QSTR_count), MP_ROM_PTR(&emu_lifter_count_obj)},
    {MP_ROM_QSTR(MP_QSTR_mark_lost), MP_ROM_PTR(&emu_lifter_mark_lost_obj)},
    {MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&emu_lifter_clear_obj)},
    {MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&emu_lifter_stats_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_camera), MP_ROM_PTR(&emu_lifter_set_camera_obj)},
    {MP_ROM_QSTR(MP_QSTR_inject), MP_ROM_PTR(&emu_lifter_inject_obj)},
    {MP_ROM_QSTR(MP_QSTR_FREE), MP_ROM_INT(0)},
    {MP_ROM_QSTR(MP_QSTR_TRACKING), MP_ROM_INT(1)},
    {MP_ROM_QSTR(MP_QSTR_LIFTED), MP_ROM_INT(2)},
    {MP_ROM_QSTR(MP_QSTR_LOST), MP_ROM_INT(3)},
};
static MP_DEFINE_CONST_DICT(emu_object_lifter_globals,
                            emu_object_lifter_globals_table);

static const mp_obj_module_t emu_object_lifter_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_object_lifter_globals,
};

static int g_emu_safety_aborted = 0;
static const char* g_emu_safety_reason = "";

static mp_obj_t emu_safety_init(void) {
    g_emu_safety_aborted = 0;
    g_emu_safety_reason = "";
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_init_obj, emu_safety_init);

static mp_obj_t emu_safety_clear(void) {
    g_emu_safety_aborted = 0;
    g_emu_safety_reason = "";
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_clear_obj, emu_safety_clear);

static mp_obj_t emu_safety_enable_markers(size_t n_args, const mp_obj_t* pos,
                                          mp_map_t* kw) {
    (void)n_args;
    (void)pos;
    (void)kw;
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(emu_safety_enable_markers_obj, 0,
                                  emu_safety_enable_markers);

static mp_obj_t emu_safety_disable_markers(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_disable_markers_obj,
                                 emu_safety_disable_markers);

static mp_obj_t emu_safety_task_start(void) {
    return mp_obj_new_int(-3);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_task_start_obj,
                                 emu_safety_task_start);

static mp_obj_t emu_safety_task_stop(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_task_stop_obj,
                                 emu_safety_task_stop);

static mp_obj_t emu_safety_aborted(void) {
    return mp_obj_new_bool(g_emu_safety_aborted);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_aborted_obj, emu_safety_aborted);

static mp_obj_t emu_safety_reason(void) {
    return mp_obj_new_str(g_emu_safety_reason, strlen(g_emu_safety_reason));
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_safety_reason_obj, emu_safety_reason);

static mp_obj_t emu_safety_test_push_aruco(mp_obj_t n_obj, mp_obj_t seq_obj,
                                           mp_obj_t ts_obj) {
    (void)seq_obj;
    (void)ts_obj;
    int n = mp_obj_get_int(n_obj);
    if (n < 4) {
        g_emu_safety_aborted = 1;
        g_emu_safety_reason = "markers";
    }
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_3(emu_safety_test_push_aruco_obj,
                                 emu_safety_test_push_aruco);

static const mp_rom_map_elem_t emu_safety_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_safety)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&emu_safety_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&emu_safety_clear_obj)},
    {MP_ROM_QSTR(MP_QSTR_enable_aruco),
     MP_ROM_PTR(&emu_safety_enable_markers_obj)},
    {MP_ROM_QSTR(MP_QSTR_disable_aruco),
     MP_ROM_PTR(&emu_safety_disable_markers_obj)},
    {MP_ROM_QSTR(MP_QSTR_enable_markers),
     MP_ROM_PTR(&emu_safety_enable_markers_obj)},
    {MP_ROM_QSTR(MP_QSTR_disable_markers),
     MP_ROM_PTR(&emu_safety_disable_markers_obj)},
    {MP_ROM_QSTR(MP_QSTR_task_start), MP_ROM_PTR(&emu_safety_task_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_task_stop), MP_ROM_PTR(&emu_safety_task_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_aborted), MP_ROM_PTR(&emu_safety_aborted_obj)},
    {MP_ROM_QSTR(MP_QSTR_reason), MP_ROM_PTR(&emu_safety_reason_obj)},
    {MP_ROM_QSTR(MP_QSTR__test_push_aruco),
     MP_ROM_PTR(&emu_safety_test_push_aruco_obj)},
};
static MP_DEFINE_CONST_DICT(emu_safety_globals, emu_safety_globals_table);

static const mp_obj_module_t emu_safety_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_safety_globals,
};
#endif  // SENTAI_EMU_HW_STUB_MODULES

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

static const mp_rom_map_elem_t emu_sentai_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai)},
    {MP_ROM_QSTR(MP_QSTR_version), MP_ROM_PTR(&emu_sentai_version_obj)},
    {MP_ROM_QSTR(MP_QSTR_verbose), MP_ROM_PTR(&emu_sentai_verbose_obj)},
    {MP_ROM_QSTR(MP_QSTR_help), MP_ROM_PTR(&emu_sentai_help_obj)},
    {MP_ROM_QSTR(MP_QSTR_debug), MP_ROM_PTR(&emu_sentai_debug_obj)},
    {MP_ROM_QSTR(MP_QSTR_console), MP_ROM_PTR(&emu_sentai_console_obj)},
    {MP_ROM_QSTR(MP_QSTR_run), MP_ROM_PTR(&emu_sentai_run_obj)},
    {MP_ROM_QSTR(MP_QSTR_io), MP_ROM_PTR(&emu_io_module)},
    {MP_ROM_QSTR(MP_QSTR_fs), MP_ROM_PTR(&emu_fs_module)},
    {MP_ROM_QSTR(MP_QSTR_rtos), MP_ROM_PTR(&emu_rtos_module)},
    {MP_ROM_QSTR(MP_QSTR_fr), MP_ROM_PTR(&emu_fr_module)},
    {MP_ROM_QSTR(MP_QSTR_sys), MP_ROM_PTR(&emu_sys_module)},
#if SENTAI_EMU_HW_STUB_MODULES
    {MP_ROM_QSTR(MP_QSTR_usb), MP_ROM_PTR(&emu_usb_module)},
    {MP_ROM_QSTR(MP_QSTR_uart), MP_ROM_PTR(&emu_uart_module)},
    {MP_ROM_QSTR(MP_QSTR_imu), MP_ROM_PTR(&emu_imu_module)},
    {MP_ROM_QSTR(MP_QSTR_mic), MP_ROM_PTR(&emu_mic_module)},
    {MP_ROM_QSTR(MP_QSTR_sleep), MP_ROM_PTR(&emu_sleep_module)},
    {MP_ROM_QSTR(MP_QSTR_servo), MP_ROM_PTR(&emu_servo_module)},
    {MP_ROM_QSTR(MP_QSTR_calib), MP_ROM_PTR(&emu_calib_module)},
    {MP_ROM_QSTR(MP_QSTR_object_lifter),
     MP_ROM_PTR(&emu_object_lifter_module)},
    {MP_ROM_QSTR(MP_QSTR_safety), MP_ROM_PTR(&emu_safety_module)},
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE
    {MP_ROM_QSTR(MP_QSTR_tpu), MP_ROM_PTR(&emu_tpu_module)},
#endif
#if SENTAI_EMU_TPU_HOST_BRIDGE || SENTAI_EMU_PIPELINE_PREP_BINDING
    {MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&emu_pipeline_module)},
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
