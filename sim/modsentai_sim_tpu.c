/* modsentai_sim_tpu.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== sentai.tpu — Phase 5 SIM via pycoral helper ============================
 * Mirrors examples/sentai_runtime/modsentai_tpu.c MP_QSTR table. The C
 * entry points sentai_tpu_*() are implemented in sim/sim_tpu_shim.c and
 * forward to a pycoral daemon over /tmp/sentai_tpu.sock.
 */
#include "examples/sentai_runtime/sentai_tpu_shim.h"

static mp_obj_t sentai_tpu_load(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_tpu_load_model(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_load_obj, sentai_tpu_load);

static mp_obj_t sentai_tpu_invoke_mp(void) {
    return mp_obj_new_int(sentai_tpu_invoke());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_invoke_obj, sentai_tpu_invoke_mp);

static mp_obj_t sentai_tpu_ready_mp(void) {
    return mp_obj_new_bool(sentai_tpu_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_ready_obj, sentai_tpu_ready_mp);

static mp_obj_t sentai_tpu_num_outputs_mp(void) {
    return mp_obj_new_int(sentai_tpu_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_num_outputs_obj, sentai_tpu_num_outputs_mp);

static mp_obj_t sentai_tpu_output_size_mp(mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_size(mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_size_obj, sentai_tpu_output_size_mp);

static mp_obj_t sentai_tpu_output_mp(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int sz  = sentai_tpu_get_output_size(idx);
    const void* d = sentai_tpu_get_output_data(idx);
    if (!d || sz <= 0) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    return mp_obj_new_bytes((const byte*)d, sz);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_obj, sentai_tpu_output_mp);

static mp_obj_t sentai_tpu_output_dims_mp(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int n   = sentai_tpu_get_output_num_dims(idx);
    if (n > 8) n = 8;
    mp_obj_t items[8];
    for (int i = 0; i < n; ++i)
        items[i] = mp_obj_new_int(sentai_tpu_get_output_dim(idx, i));
    return mp_obj_new_tuple(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_dims_obj, sentai_tpu_output_dims_mp);

static mp_obj_t sentai_tpu_output_type_mp(mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_type(mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_type_obj, sentai_tpu_output_type_mp);

static mp_obj_t sentai_tpu_input_quant_mp(void) {
    float scale = 0.0f; int32_t zp = 0;
    sentai_tpu_input_quant(&scale, &zp);
    mp_obj_t items[2] = { mp_obj_new_float(scale), mp_obj_new_int(zp) };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_input_quant_obj, sentai_tpu_input_quant_mp);

static mp_obj_t sentai_tpu_output_quant_mp(mp_obj_t idx_obj) {
    float scale = 0.0f; int32_t zp = 0;
    sentai_tpu_output_quant(mp_obj_get_int(idx_obj), &scale, &zp);
    mp_obj_t items[2] = { mp_obj_new_float(scale), mp_obj_new_int(zp) };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_quant_obj, sentai_tpu_output_quant_mp);

static mp_obj_t sentai_tpu_input_type_mp(void) {
    return mp_obj_new_int(sentai_tpu_input_type());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_input_type_obj, sentai_tpu_input_type_mp);

/* sentai.tpu.set_input(bytes) — pushes a flat byte buffer into the input tensor.
 * Useful for loading a pre-resized image without going through the camera. */
static mp_obj_t sentai_tpu_set_input_mp(mp_obj_t buf_obj) {
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf_obj, &bi, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_tpu_set_input_slot(0, (const uint8_t*)bi.buf, bi.len));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_set_input_obj, sentai_tpu_set_input_mp);

static const mp_rom_map_elem_t sentai_tpu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_tpu) },
    { MP_ROM_QSTR(MP_QSTR_load),         MP_ROM_PTR(&sentai_tpu_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke),       MP_ROM_PTR(&sentai_tpu_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_ready),        MP_ROM_PTR(&sentai_tpu_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_num_outputs),  MP_ROM_PTR(&sentai_tpu_num_outputs_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size),  MP_ROM_PTR(&sentai_tpu_output_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_output),       MP_ROM_PTR(&sentai_tpu_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims),  MP_ROM_PTR(&sentai_tpu_output_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type),  MP_ROM_PTR(&sentai_tpu_output_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_quant),  MP_ROM_PTR(&sentai_tpu_input_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_quant), MP_ROM_PTR(&sentai_tpu_output_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_type),   MP_ROM_PTR(&sentai_tpu_input_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_input),    MP_ROM_PTR(&sentai_tpu_set_input_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tpu_globals, sentai_tpu_globals_table);
static const mp_obj_module_t sentai_tpu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_tpu_globals,
};
