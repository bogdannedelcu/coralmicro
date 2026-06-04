// B8.10 emulator guest-side sentai.tpu/sentai.pipeline bridge.
//
// This is an emulator platform backend, not a replacement for the ARM TPU
// driver.  The guest reads model/image bytes from FxUser/FileX and streams
// them to a Renode MMIO bridge.  The actual host-side code lives under
// emu/host or emu/renode; this file is compiled into the ARM-emulated guest.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "libs/base/fx_user_fs.h"
#include "py/obj.h"
#include "py/runtime.h"

#define kBridgeBase 0x40901400u
#define kCmdBeginFile 1u
#define kCmdAppendChunk 2u
#define kCmdInvoke 3u
#define kCmdGetDetection 4u
#define kCmdBenchmark 5u
#define kCmdSessionStart 6u
#define kCmdSessionStop 7u
#define kCmdSessionInvoke 8u
#define kCmdSessionBenchmark 9u
#define kCmdSessionReloadImage 10u
#define kStatusOk 1u
#define kFileModel 1u
#define kFileImage 2u
#define kChunkBytes (32u * 1024u)
#define kImageMemMaxBytes (1024u * 1024u)
#define kMaxDetections 20

uint8_t g_stream_buf[kChunkBytes] __attribute__((aligned(8)));
uint8_t* g_image_mem = NULL;
uint32_t g_image_mem_size = 0;
int g_model_loaded = 0;
int g_image_loaded = 0;
int g_tpu_session_started = 0;
int g_last_invoke_ms = -1;
int g_last_detection_count = 0;

extern uint32_t xTaskGetTickCount(void);

static inline volatile uint32_t* RegPtr(uint32_t offset) {
    return (volatile uint32_t*)(kBridgeBase + offset);
}

static inline uint32_t RegRead(uint32_t offset) {
    return *RegPtr(offset);
}

static inline void RegWrite(uint32_t offset, uint32_t value) {
    *RegPtr(offset) = value;
}

static bool BridgeStatusOk(void) { return RegRead(0x14) == kStatusOk; }

static bool BeginFile(uint32_t file_kind, uint32_t size) {
    RegWrite(0x04, file_kind);
    RegWrite(0x08, size);
    RegWrite(0x00, kCmdBeginFile);
    return BridgeStatusOk();
}

static bool AppendChunk(uint32_t file_kind, const uint8_t* data,
                        uint32_t size) {
    RegWrite(0x04, file_kind);
    RegWrite(0x0C, (uint32_t)data);
    RegWrite(0x10, size);
    RegWrite(0x00, kCmdAppendChunk);
    return BridgeStatusOk();
}

static int StreamFileToHost(const char* path, uint32_t file_kind) {
    const ssize_t size = FxUserSize(path);
    if (size < 0) return -2;
    if (!BeginFile(file_kind, (uint32_t)size)) return -3;

    uint32_t offset = 0;
    while (offset < (uint32_t)size) {
        uint32_t remaining = (uint32_t)size - offset;
        uint32_t want = remaining > kChunkBytes ? (uint32_t)kChunkBytes
                                                : remaining;
        size_t got = FxUserReadFileAt(path, offset, g_stream_buf, want);
        if (got == 0 || got > want) return -4;
        if (!AppendChunk(file_kind, g_stream_buf, (uint32_t)got)) return -5;
        offset += (uint32_t)got;
    }
    return 0;
}

static int LoadImageFileToMemory(const char* path) {
    const ssize_t size = FxUserSize(path);
    if (size < 0) return -2;
    if ((uint32_t)size > kImageMemMaxBytes) return -6;
    if (!g_image_mem) {
        g_image_mem = (uint8_t*)malloc(kImageMemMaxBytes);
        if (!g_image_mem) return -7;
    }

    uint32_t offset = 0;
    while (offset < (uint32_t)size) {
        uint32_t remaining = (uint32_t)size - offset;
        uint32_t want = remaining > kChunkBytes ? (uint32_t)kChunkBytes
                                                : remaining;
        size_t got = FxUserReadFileAt(path, offset, g_image_mem + offset, want);
        if (got == 0 || got > want) {
            g_image_mem_size = 0;
            return -4;
        }
        offset += (uint32_t)got;
    }

    g_image_mem_size = (uint32_t)size;
    return 0;
}

static int StreamImageMemoryToHost(uint32_t file_kind) {
    if (g_image_mem_size == 0) return -2;
    if (!BeginFile(file_kind, g_image_mem_size)) return -3;

    uint32_t offset = 0;
    while (offset < g_image_mem_size) {
        uint32_t remaining = g_image_mem_size - offset;
        uint32_t want = remaining > kChunkBytes ? (uint32_t)kChunkBytes
                                                : remaining;
        if (!AppendChunk(file_kind, g_image_mem + offset, want)) return -5;
        offset += want;
    }
    return 0;
}

static int ReloadSessionImageIfStarted(void) {
    if (!g_tpu_session_started) return 0;
    RegWrite(0x00, kCmdSessionReloadImage);
    return BridgeStatusOk() ? 0 : -8;
}

static mp_obj_t emu_tpu_load(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = StreamFileToHost(path, kFileModel);
    g_model_loaded = (rc == 0) ? 1 : 0;
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_tpu_load_obj, emu_tpu_load);

static mp_obj_t emu_tpu_load_image(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int rc = StreamFileToHost(path, kFileImage);
    if (rc == 0) rc = ReloadSessionImageIfStarted();
    g_image_loaded = (rc == 0) ? 1 : 0;
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(emu_tpu_load_image_obj, emu_tpu_load_image);

static mp_obj_t emu_tpu_load_image_mem(size_t n_args, const mp_obj_t* args) {
    size_t path_len = 0;
    const char* path = mp_obj_str_get_data(args[0], &path_len);
    const bool stream_to_host = (n_args < 2) ? true : mp_obj_is_true(args[1]);
    int rc = 0;

    if (path_len > 0) {
        rc = LoadImageFileToMemory(path);
        if (rc != 0) {
            g_image_loaded = 0;
            return mp_obj_new_int(rc);
        }
        if (!stream_to_host) {
            g_image_loaded = 0;
            return mp_obj_new_int(0);
        }
    }

    rc = StreamImageMemoryToHost(kFileImage);
    if (rc == 0) rc = ReloadSessionImageIfStarted();
    g_image_loaded = (rc == 0) ? 1 : 0;
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_tpu_load_image_mem_obj, 1, 2,
                                           emu_tpu_load_image_mem);

static mp_obj_t emu_tpu_image_mem_size(void) {
    return mp_obj_new_int_from_uint(g_image_mem_size);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_image_mem_size_obj,
                                 emu_tpu_image_mem_size);

static mp_obj_t emu_tpu_ready(void) {
    return mp_obj_new_bool(g_model_loaded);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_ready_obj, emu_tpu_ready);

static mp_obj_t emu_tpu_stats(void) {
    mp_obj_t out[28];
    for (uint32_t i = 0; i < 28; ++i) {
        out[i] = mp_obj_new_int_from_uint(RegRead((16u + i) * 4u));
    }
    return mp_obj_new_tuple(28, out);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_stats_obj, emu_tpu_stats);

static mp_obj_t emu_tpu_start(void) {
    if (!g_model_loaded || !g_image_loaded) {
        return mp_obj_new_int(-2);
    }
    RegWrite(0x00, kCmdSessionStart);
    if (!BridgeStatusOk()) {
        g_tpu_session_started = 0;
        return mp_obj_new_int(-3);
    }
    g_tpu_session_started = 1;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_start_obj, emu_tpu_start);

static mp_obj_t emu_tpu_stop(void) {
    RegWrite(0x00, kCmdSessionStop);
    g_tpu_session_started = 0;
    return mp_obj_new_int(BridgeStatusOk() ? 0 : -3);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_stop_obj, emu_tpu_stop);

static int InvokeOnce(void) {
    if (!g_model_loaded || !g_image_loaded) {
        return -2;
    }
    RegWrite(0x00, g_tpu_session_started ? kCmdSessionInvoke : kCmdInvoke);
    if (!BridgeStatusOk()) {
        g_last_invoke_ms = -3;
        g_last_detection_count = 0;
        return g_last_invoke_ms;
    }
    g_last_invoke_ms = (int)RegRead(0x20);
    g_last_detection_count = (int)RegRead(0x1C);
    if (g_last_detection_count < 0) g_last_detection_count = 0;
    if (g_last_detection_count > kMaxDetections) {
        g_last_detection_count = kMaxDetections;
    }
    return g_last_invoke_ms;
}

static mp_obj_t emu_tpu_invoke(void) {
    const int rc = InvokeOnce();
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_invoke_obj, emu_tpu_invoke);

static mp_obj_t emu_tpu_fps_invoke(size_t n_args, const mp_obj_t* args) {
    int runs = 5;
    if (n_args >= 1) {
        runs = mp_obj_get_int(args[0]);
    }
    if (runs <= 0) runs = 1;
    if (runs > 50) runs = 50;

    const uint32_t warmup = 1;
    for (uint32_t i = 0; i < warmup; ++i) {
        if (InvokeOnce() < 0) {
            mp_obj_t fail[6] = {
                mp_obj_new_int(0),
                mp_obj_new_int_from_uint(warmup),
                mp_obj_new_int(-1),
                mp_obj_new_int(0),
                mp_obj_new_int(0),
                mp_obj_new_int(0),
            };
            return mp_obj_new_tuple(6, fail);
        }
    }

    uint32_t completed = 0;
    uint32_t invoke_ms_sum = 0;
    const uint32_t start_ms = xTaskGetTickCount();
    for (int i = 0; i < runs; ++i) {
        const int rc = InvokeOnce();
        if (rc < 0) break;
        invoke_ms_sum += (uint32_t)rc;
        ++completed;
    }
    uint32_t measured_ms = xTaskGetTickCount() - start_ms;
    if (invoke_ms_sum > measured_ms) {
        measured_ms = invoke_ms_sum;
    }
    if (measured_ms == 0) measured_ms = 1;
    const uint32_t fps_x100 = (completed * 100000u) / measured_ms;

    mp_obj_t out[6] = {
        mp_obj_new_int_from_uint(completed),
        mp_obj_new_int_from_uint(warmup),
        mp_obj_new_int_from_uint(measured_ms),
        mp_obj_new_int_from_uint(fps_x100),
        mp_obj_new_int_from_uint(invoke_ms_sum),
        mp_obj_new_int_from_uint((uint32_t)g_last_detection_count),
    };
    return mp_obj_new_tuple(6, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_tpu_fps_invoke_obj, 0, 1,
                                           emu_tpu_fps_invoke);

static mp_obj_t emu_tpu_fps(size_t n_args, const mp_obj_t* args) {
    int runs = 5;
    if (n_args >= 1) {
        runs = mp_obj_get_int(args[0]);
    }
    if (runs <= 0) runs = 1;
    if (runs > 50) runs = 50;

    const uint32_t warmup = 1;
    RegWrite(0x04, (uint32_t)runs);
    RegWrite(0x08, warmup);
    RegWrite(0x00, g_tpu_session_started ? kCmdSessionBenchmark
                                          : kCmdBenchmark);
    if (!BridgeStatusOk()) {
        mp_obj_t fail[6] = {
            mp_obj_new_int(0),
            mp_obj_new_int_from_uint(warmup),
            mp_obj_new_int(-1),
            mp_obj_new_int(0),
            mp_obj_new_int(0),
            mp_obj_new_int(0),
        };
        return mp_obj_new_tuple(6, fail);
    }

    g_last_invoke_ms = (int)RegRead(0x20);
    g_last_detection_count = (int)RegRead(0x1C);
    if (g_last_detection_count < 0) g_last_detection_count = 0;
    if (g_last_detection_count > kMaxDetections) {
        g_last_detection_count = kMaxDetections;
    }

    mp_obj_t out[6] = {
        mp_obj_new_int_from_uint(RegRead(0x28)),
        mp_obj_new_int_from_uint(RegRead(0x2C)),
        mp_obj_new_int_from_uint(RegRead(0x20)),
        mp_obj_new_int_from_uint(RegRead(0x30)),
        mp_obj_new_int_from_uint(RegRead(0x34)),
        mp_obj_new_int_from_uint(RegRead(0x1C)),
    };
    return mp_obj_new_tuple(6, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_tpu_fps_obj, 0, 1,
                                           emu_tpu_fps);

static mp_obj_t emu_tpu_num_outputs(void) {
    return mp_obj_new_int(2);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_tpu_num_outputs_obj,
                                 emu_tpu_num_outputs);

static const mp_rom_map_elem_t emu_tpu_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_tpu)},
    {MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&emu_tpu_load_obj)},
    {MP_ROM_QSTR(MP_QSTR_load_image), MP_ROM_PTR(&emu_tpu_load_image_obj)},
    {MP_ROM_QSTR(MP_QSTR_load_image_mem),
     MP_ROM_PTR(&emu_tpu_load_image_mem_obj)},
    {MP_ROM_QSTR(MP_QSTR_image_mem_size),
     MP_ROM_PTR(&emu_tpu_image_mem_size_obj)},
    {MP_ROM_QSTR(MP_QSTR_ready), MP_ROM_PTR(&emu_tpu_ready_obj)},
    {MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&emu_tpu_stats_obj)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&emu_tpu_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&emu_tpu_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_invoke), MP_ROM_PTR(&emu_tpu_invoke_obj)},
    {MP_ROM_QSTR(MP_QSTR_fps_invoke),
     MP_ROM_PTR(&emu_tpu_fps_invoke_obj)},
    {MP_ROM_QSTR(MP_QSTR_fps), MP_ROM_PTR(&emu_tpu_fps_obj)},
    {MP_ROM_QSTR(MP_QSTR_num_outputs), MP_ROM_PTR(&emu_tpu_num_outputs_obj)},
};
static MP_DEFINE_CONST_DICT(emu_tpu_globals, emu_tpu_globals_table);

const mp_obj_module_t emu_tpu_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_tpu_globals,
};

static mp_obj_t emu_pipeline_detections(size_t n_args,
                                        const mp_obj_t* args) {
    int threshold_milli = 0;
    if (n_args >= 1) {
        threshold_milli = mp_obj_get_int(args[0]);
    }
    if (threshold_milli < 0) threshold_milli = 0;

    mp_obj_t list = mp_obj_new_list(0, NULL);
    for (int i = 0; i < g_last_detection_count; ++i) {
        RegWrite(0x04, (uint32_t)i);
        RegWrite(0x00, kCmdGetDetection);
        if (!BridgeStatusOk()) break;
        int cls = (int)RegRead(0x28);
        int score_milli = (int)RegRead(0x2C);
        if (score_milli < threshold_milli) continue;
        mp_obj_t items[6];
        items[0] = mp_obj_new_int(cls);
        items[1] = mp_obj_new_float((mp_float_t)score_milli / 1000.0f);
        items[2] =
            mp_obj_new_float((mp_float_t)((int)RegRead(0x30)) / 1000.0f);
        items[3] =
            mp_obj_new_float((mp_float_t)((int)RegRead(0x34)) / 1000.0f);
        items[4] =
            mp_obj_new_float((mp_float_t)((int)RegRead(0x38)) / 1000.0f);
        items[5] =
            mp_obj_new_float((mp_float_t)((int)RegRead(0x3C)) / 1000.0f);
        mp_obj_list_append(list, mp_obj_new_tuple(6, items));
    }
    return list;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_pipeline_detections_obj, 0, 1,
                                           emu_pipeline_detections);

static const mp_rom_map_elem_t emu_pipeline_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pipeline)},
    {MP_ROM_QSTR(MP_QSTR_detections),
     MP_ROM_PTR(&emu_pipeline_detections_obj)},
};
static MP_DEFINE_CONST_DICT(emu_pipeline_globals,
                            emu_pipeline_globals_table);

const mp_obj_module_t emu_pipeline_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_pipeline_globals,
};
