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
#include "py/runtime.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#if SENTAI_EMU_TPU_HOST_BRIDGE
extern const mp_obj_module_t emu_tpu_module;
extern const mp_obj_module_t emu_pipeline_module;
#endif

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

static const mp_rom_map_elem_t emu_rtos_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rtos)},
    {MP_ROM_QSTR(MP_QSTR_ticks_ms), MP_ROM_PTR(&emu_rtos_ticks_ms_obj)},
    {MP_ROM_QSTR(MP_QSTR_sleep_ms), MP_ROM_PTR(&emu_rtos_sleep_ms_obj)},
};
static MP_DEFINE_CONST_DICT(emu_rtos_globals, emu_rtos_globals_table);

static const mp_obj_module_t emu_rtos_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_rtos_globals,
};

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
    {MP_ROM_QSTR(MP_QSTR_fs), MP_ROM_PTR(&emu_fs_module)},
    {MP_ROM_QSTR(MP_QSTR_rtos), MP_ROM_PTR(&emu_rtos_module)},
    {MP_ROM_QSTR(MP_QSTR_fr), MP_ROM_PTR(&emu_fr_module)},
#if SENTAI_EMU_TPU_HOST_BRIDGE
    {MP_ROM_QSTR(MP_QSTR_tpu), MP_ROM_PTR(&emu_tpu_module)},
    {MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&emu_pipeline_module)},
#endif
};
static MP_DEFINE_CONST_DICT(emu_sentai_globals, emu_sentai_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_sentai_globals,
};
