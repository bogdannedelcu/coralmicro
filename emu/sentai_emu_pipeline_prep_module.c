// Minimal emulator sentai.pipeline binding for PrepTask-only bring-up.
//
// The implementation calls the shared runtime PrepTask entry points from
// detection_task.cc.  It deliberately does not model TPU detections; the TPU
// bridge owns the separate B8 `sentai.tpu`/`pipeline.detections` surface.

#include <stdint.h>

#include "examples/sentai_runtime/sentai_prep.h"
#include "py/obj.h"
#include "py/runtime.h"

#define STR_KEY(s) mp_obj_new_str((s), sizeof(s) - 1)

extern int sentai_prep_task_start_only(void);
extern int sentai_prep_task_stop_only(void);
extern int sentai_pipeline_prep_fps_get(void);
extern void sentai_pipeline_prep_fps_set(int v);
extern void sentai_prep_stage_stats(uint32_t* frames,
                                    uint32_t* cam_grab_ms,
                                    uint32_t* pxp_ms,
                                    uint32_t* quant_ms,
                                    uint32_t* total_ms);
extern void sentai_prep_stage_reset(void);

static mp_obj_t emu_pipeline_prep_start(void) {
    int rc = sentai_prep_task_start_only();
    if (rc != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("pipeline prep_start failed (%d)"),
                          rc);
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_pipeline_prep_start_obj,
                                 emu_pipeline_prep_start);

static mp_obj_t emu_pipeline_prep_stop(void) {
    return mp_obj_new_int(sentai_prep_task_stop_only());
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_pipeline_prep_stop_obj,
                                 emu_pipeline_prep_stop);

static mp_obj_t emu_pipeline_prep_fps(size_t n_args, const mp_obj_t* args) {
    if (n_args >= 1) {
        sentai_pipeline_prep_fps_set(mp_obj_get_int(args[0]));
    }
    return mp_obj_new_int(sentai_pipeline_prep_fps_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(emu_pipeline_prep_fps_obj, 0, 1,
                                           emu_pipeline_prep_fps);

static mp_obj_t emu_pipeline_prep_stats(void) {
    uint32_t frames = 0, cam = 0, pxp = 0, quant = 0, total = 0;
    sentai_prep_stage_stats(&frames, &cam, &pxp, &quant, &total);
    sentai_prep_stats_t st;
    sentai_prep_get_stats(&st);

    mp_obj_t d = mp_obj_new_dict(9);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames), mp_obj_new_int_from_uint(frames));
    mp_obj_dict_store(d, STR_KEY("cam_grab_ms"), mp_obj_new_int_from_uint(cam));
    mp_obj_dict_store(d, STR_KEY("pxp_ms"), mp_obj_new_int_from_uint(pxp));
    mp_obj_dict_store(d, STR_KEY("quant_ms"), mp_obj_new_int_from_uint(quant));
    mp_obj_dict_store(d, STR_KEY("total_ms"), mp_obj_new_int_from_uint(total));
    mp_obj_dict_store(d, STR_KEY("prep_frames_total"),
                      mp_obj_new_int_from_uint(st.frames_total));
    mp_obj_dict_store(d, STR_KEY("prep_frames_with_aux"),
                      mp_obj_new_int_from_uint(st.frames_with_aux));

    mp_obj_t refs[SENTAI_PREP_SLOT_COUNT];
    mp_obj_t seqs[SENTAI_PREP_SLOT_COUNT];
    for (int i = 0; i < SENTAI_PREP_SLOT_COUNT; ++i) {
        refs[i] = mp_obj_new_int_from_uint(st.slot_refcount[i]);
        seqs[i] = mp_obj_new_int_from_uint(st.slot_seq[i]);
    }
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_slot_refcount),
                      mp_obj_new_tuple(SENTAI_PREP_SLOT_COUNT, refs));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_slot_seq),
                      mp_obj_new_tuple(SENTAI_PREP_SLOT_COUNT, seqs));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_pipeline_prep_stats_obj,
                                 emu_pipeline_prep_stats);

static mp_obj_t emu_pipeline_prep_reset(void) {
    sentai_prep_stage_reset();
    sentai_prep_reset_stats();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(emu_pipeline_prep_reset_obj,
                                 emu_pipeline_prep_reset);

static const mp_rom_map_elem_t emu_pipeline_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pipeline)},
    {MP_ROM_QSTR(MP_QSTR_prep_start), MP_ROM_PTR(&emu_pipeline_prep_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_prep_stop), MP_ROM_PTR(&emu_pipeline_prep_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_prep_fps), MP_ROM_PTR(&emu_pipeline_prep_fps_obj)},
    {MP_ROM_QSTR(MP_QSTR_prep_stats), MP_ROM_PTR(&emu_pipeline_prep_stats_obj)},
    {MP_ROM_QSTR(MP_QSTR_prep_reset), MP_ROM_PTR(&emu_pipeline_prep_reset_obj)},
};
static MP_DEFINE_CONST_DICT(emu_pipeline_globals, emu_pipeline_globals_table);

const mp_obj_module_t emu_pipeline_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&emu_pipeline_globals,
};
