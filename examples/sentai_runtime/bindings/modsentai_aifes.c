// ============== sentai.aifes — On-device Neural Network Training ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Uses AIfES library from Fraunhofer IMS (AGPL-3.0 license).
// See third_party/aifes/LICENSE for details.

// Forward declarations from aifes_task.h (C interface)
extern int aifes_load_model(const char* yaml_path);
extern int aifes_load_weights(const char* weights_path);
extern int aifes_save_weights(const char* weights_path);
extern float aifes_train(const float* x_data, const float* y_data,
                         int n_samples, int input_size, int output_size,
                         int epochs, int batch_size, float learning_rate,
                         int optimizer, int loss_fn, float val_split,
                         const char* log_path, float sigreg_lambda);
extern int aifes_predict(const float* input, float* output);
extern int aifes_is_loaded(void);
extern const char* aifes_get_model_name(void);
extern int aifes_get_layer_count(void);
extern int aifes_get_input_size(void);
extern int aifes_get_output_size(void);
extern int aifes_get_total_params(void);
extern const char* aifes_get_log_path(void);
extern int aifes_get_best_epoch(void);
extern float aifes_get_best_val_loss(void);
extern void aifes_unload(void);

// New invoke-style API
extern int aifes_set_input(const float* data, int size);
extern int aifes_from_tpu(int idx);
extern int aifes_from_camera(int width, int height, int grayscale);
extern int aifes_from_mic(int samples);
extern int aifes_invoke(void);
extern const float* aifes_get_output_data(void);
extern int aifes_get_output_count(void);

// Mic init/stop - use existing mic API from modsentai_hal.cc
extern int sentai_mic_start(int max_seconds);
extern int sentai_mic_stop(void);
extern int sentai_mic_is_initialized(void);

// sentai.aifes.load(yaml_path) -> bool
// Load model architecture from YAML file
static mp_obj_t mod_aifes_load(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int result = aifes_load_model(path);
    return mp_obj_new_bool(result == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_load_obj, mod_aifes_load);

// sentai.aifes.load_weights(path) -> bool
// Load pre-trained weights from binary file
static mp_obj_t mod_aifes_load_weights(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int result = aifes_load_weights(path);
    return mp_obj_new_bool(result == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_load_weights_obj, mod_aifes_load_weights);

// sentai.aifes.save_weights(path) -> bool
// Save trained weights (best model) to binary file
static mp_obj_t mod_aifes_save_weights(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    int result = aifes_save_weights(path);
    return mp_obj_new_bool(result == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_save_weights_obj, mod_aifes_save_weights);

// sentai.aifes.train(x_data, y_data, epochs=100, batch=4, lr=0.01, 
//                    optimizer=0, loss=0, val_split=0.1, log_path=None,
//                    sigreg=0.0) -> float
// Train model on provided data
// x_data: list of lists (flattened input samples)
// y_data: list of lists (flattened target samples)
// optimizer: 0=adam, 1=sgd
// loss: 0=mse, 1=crossentropy
// sigreg: Weak-SIGReg lambda (0=disabled, 0.001-0.1 typical)
//         Which layers get SIGReg is configured in YAML per-layer:
//         - [dense, 64, relu, sigreg]   # SIGReg ON
//         - [dense, 32, relu]           # SIGReg OFF
// Returns: final loss or -1 on error
static mp_obj_t mod_aifes_train(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args) {
    // Required positional args
    if (n_args < 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("x_data and y_data required"));
    }
    
    // Get x_data and y_data as lists
    mp_obj_t x_list = args[0];
    mp_obj_t y_list = args[1];
    
    size_t x_len, y_len;
    mp_obj_t* x_items;
    mp_obj_t* y_items;
    mp_obj_get_array(x_list, &x_len, &x_items);
    mp_obj_get_array(y_list, &y_len, &y_items);
    
    if (x_len == 0 || y_len == 0 || x_len != y_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("x and y must have same non-zero length"));
    }
    
    int n_samples = x_len;
    
    // Determine input/output sizes from first samples
    size_t in_len, out_len;
    mp_obj_t* in_items;
    mp_obj_t* out_items;
    mp_obj_get_array(x_items[0], &in_len, &in_items);
    mp_obj_get_array(y_items[0], &out_len, &out_items);
    
    int input_size = in_len;
    int output_size = out_len;
    
    // Flatten data into contiguous arrays
    float* x_data = m_new(float, n_samples * input_size);
    float* y_data = m_new(float, n_samples * output_size);
    
    for (int i = 0; i < n_samples; i++) {
        mp_obj_t* xi;
        mp_obj_t* yi;
        size_t xi_len, yi_len;
        mp_obj_get_array(x_items[i], &xi_len, &xi);
        mp_obj_get_array(y_items[i], &yi_len, &yi);
        
        for (size_t j = 0; j < xi_len && j < (size_t)input_size; j++) {
            x_data[i * input_size + j] = mp_obj_get_float(xi[j]);
        }
        for (size_t j = 0; j < yi_len && j < (size_t)output_size; j++) {
            y_data[i * output_size + j] = mp_obj_get_float(yi[j]);
        }
    }
    
    // Parse keyword arguments with defaults
    int epochs = 100;
    int batch_size = 4;
    float learning_rate = 0.01f;
    int optimizer = 0;  // adam
    int loss_fn = 0;    // mse
    float val_split = 0.1f;
    const char* log_path = NULL;
    float sigreg_lambda = 0.0f;  // Weak-SIGReg disabled by default
    
    // Optional positional args
    if (n_args > 2) epochs = mp_obj_get_int(args[2]);
    if (n_args > 3) batch_size = mp_obj_get_int(args[3]);
    if (n_args > 4) learning_rate = mp_obj_get_float(args[4]);
    if (n_args > 5) optimizer = mp_obj_get_int(args[5]);
    if (n_args > 6) loss_fn = mp_obj_get_int(args[6]);
    if (n_args > 7) val_split = mp_obj_get_float(args[7]);
    if (n_args > 8 && args[8] != mp_const_none) {
        log_path = mp_obj_str_get_str(args[8]);
    }
    if (n_args > 9) sigreg_lambda = mp_obj_get_float(args[9]);
    
    // Call training
    float result = aifes_train(x_data, y_data, n_samples, input_size, output_size,
                               epochs, batch_size, learning_rate,
                               optimizer, loss_fn, val_split, log_path,
                               sigreg_lambda);
    
    // Free temporary arrays
    m_del(float, x_data, n_samples * input_size);
    m_del(float, y_data, n_samples * output_size);
    
    return mp_obj_new_float(result);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_aifes_train_obj, 2, mod_aifes_train);

// sentai.aifes.predict(input) -> list
// Run inference on single input
// input: list of floats
// Returns: list of output floats
static mp_obj_t mod_aifes_predict(mp_obj_t input_obj) {
    size_t in_len;
    mp_obj_t* in_items;
    mp_obj_get_array(input_obj, &in_len, &in_items);
    
    int input_size = aifes_get_input_size();
    int output_size = aifes_get_output_size();
    
    if ((int)in_len != input_size) {
        mp_raise_ValueError(MP_ERROR_TEXT("input size mismatch"));
    }
    
    float* input = m_new(float, input_size);
    float* output = m_new(float, output_size);
    
    for (size_t i = 0; i < in_len; i++) {
        input[i] = mp_obj_get_float(in_items[i]);
    }
    
    int result = aifes_predict(input, output);
    
    mp_obj_t ret;
    if (result == 0) {
        mp_obj_t* items = m_new(mp_obj_t, output_size);
        for (int i = 0; i < output_size; i++) {
            items[i] = mp_obj_new_float(output[i]);
        }
        ret = mp_obj_new_list(output_size, items);
        m_del(mp_obj_t, items, output_size);
    } else {
        ret = mp_const_none;
    }
    
    m_del(float, input, input_size);
    m_del(float, output, output_size);
    
    return ret;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_predict_obj, mod_aifes_predict);

// sentai.aifes.ready() -> bool
// Check if model is loaded
static mp_obj_t mod_aifes_ready(void) {
    return mp_obj_new_bool(aifes_is_loaded());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_ready_obj, mod_aifes_ready);

// sentai.aifes.info() -> dict
// Get model information
static mp_obj_t mod_aifes_info(void) {
    if (!aifes_is_loaded()) {
        return mp_const_none;
    }
    
    mp_obj_t dict = mp_obj_new_dict(6);
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_name), 
                      mp_obj_new_str(aifes_get_model_name(), strlen(aifes_get_model_name())));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_layers), 
                      mp_obj_new_int(aifes_get_layer_count()));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_input_size), 
                      mp_obj_new_int(aifes_get_input_size()));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_output_size), 
                      mp_obj_new_int(aifes_get_output_size()));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_params), 
                      mp_obj_new_int(aifes_get_total_params()));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_best_epoch), 
                      mp_obj_new_int(aifes_get_best_epoch()));
    
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_info_obj, mod_aifes_info);

// sentai.aifes.unload() -> None
// Unload model and free memory
static mp_obj_t mod_aifes_unload(void) {
    aifes_unload();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_unload_obj, mod_aifes_unload);

// ============== NEW: TPU-style set_input/invoke/output API ==============

// sentai.aifes.set_input(data) -> int
// Set input data for inference from a list of floats.
// Returns 0 on success, negative on error.
static mp_obj_t mod_aifes_set_input(mp_obj_t data_obj) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(data_obj, &len, &items);
    
    float* data = m_new(float, len);
    for (size_t i = 0; i < len; i++) {
        data[i] = mp_obj_get_float(items[i]);
    }
    
    int result = aifes_set_input(data, (int)len);
    m_del(float, data, len);
    
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_set_input_obj, mod_aifes_set_input);

// sentai.aifes.from_tpu(idx) -> int
// Set input from TPU output tensor (auto-dequantizes int8/uint8).
// Returns 0 on success, negative on error.
static mp_obj_t mod_aifes_from_tpu(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int result = aifes_from_tpu(idx);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_from_tpu_obj, mod_aifes_from_tpu);

// sentai.aifes.from_camera(width, height, grayscale=False) -> int
// Capture camera, resize to width×height, normalize [0,1] and set as input.
// grayscale=True: output w*h floats. grayscale=False: output w*h*3 floats (RGB).
// Returns 0 on success, negative on error.
static mp_obj_t mod_aifes_from_camera(size_t n_args, const mp_obj_t *args) {
    int width = mp_obj_get_int(args[0]);
    int height = mp_obj_get_int(args[1]);
    int grayscale = 0;
    if (n_args >= 3) {
        grayscale = mp_obj_is_true(args[2]) ? 1 : 0;
    }
    int result = aifes_from_camera(width, height, grayscale);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_aifes_from_camera_obj, 2, 3, mod_aifes_from_camera);

// sentai.aifes.from_mic(samples) -> int
// Capture mic samples, normalize [-1,1] and set as input.
// Requires mic_init() first.
// Returns 0 on success, negative on error.
static mp_obj_t mod_aifes_from_mic(mp_obj_t samples_obj) {
    int samples = mp_obj_get_int(samples_obj);
    int result = aifes_from_mic(samples);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_aifes_from_mic_obj, mod_aifes_from_mic);

// sentai.aifes.mic_init() -> int
// Initialize microphone at 16kHz with 10s ring buffer. Returns 0 on success.
static mp_obj_t mod_aifes_mic_init(void) {
    return mp_obj_new_int(sentai_mic_start(10));  // 10 second ring buffer
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_mic_init_obj, mod_aifes_mic_init);

// sentai.aifes.mic_stop() -> int
// Stop microphone. Returns 0 on success.
static mp_obj_t mod_aifes_mic_stop(void) {
    return mp_obj_new_int(sentai_mic_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_mic_stop_obj, mod_aifes_mic_stop);

// sentai.aifes.invoke() -> int
// Run inference using previously set input.
// Returns inference time in ms, negative on error.
static mp_obj_t mod_aifes_invoke(void) {
    int result = aifes_invoke();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_invoke_obj, mod_aifes_invoke);

// sentai.aifes.output() -> list
// Get output from last inference as a list of floats.
static mp_obj_t mod_aifes_output(void) {
    const float* data = aifes_get_output_data();
    int count = aifes_get_output_count();
    
    if (!data || count <= 0) {
        return mp_const_none;
    }
    
    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(count, NULL));
    for (int i = 0; i < count; i++) {
        list->items[i] = mp_obj_new_float(data[i]);
    }
    
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_aifes_output_obj, mod_aifes_output);

// Module definition for sentai.aifes
static const mp_rom_map_elem_t sentai_aifes_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_aifes) },
    // Model loading/saving
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_aifes_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_weights), MP_ROM_PTR(&mod_aifes_load_weights_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_weights), MP_ROM_PTR(&mod_aifes_save_weights_obj) },
    { MP_ROM_QSTR(MP_QSTR_unload), MP_ROM_PTR(&mod_aifes_unload_obj) },
    { MP_ROM_QSTR(MP_QSTR_ready), MP_ROM_PTR(&mod_aifes_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_aifes_info_obj) },
    // Training
    { MP_ROM_QSTR(MP_QSTR_train), MP_ROM_PTR(&mod_aifes_train_obj) },
    // Inference (new TPU-style API)
    { MP_ROM_QSTR(MP_QSTR_set_input), MP_ROM_PTR(&mod_aifes_set_input_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_tpu), MP_ROM_PTR(&mod_aifes_from_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_camera), MP_ROM_PTR(&mod_aifes_from_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_mic), MP_ROM_PTR(&mod_aifes_from_mic_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke), MP_ROM_PTR(&mod_aifes_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_output), MP_ROM_PTR(&mod_aifes_output_obj) },
    // Microphone control
    { MP_ROM_QSTR(MP_QSTR_mic_init), MP_ROM_PTR(&mod_aifes_mic_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_mic_stop), MP_ROM_PTR(&mod_aifes_mic_stop_obj) },
    // Legacy predict (kept for compatibility)
    { MP_ROM_QSTR(MP_QSTR_predict), MP_ROM_PTR(&mod_aifes_predict_obj) },
    // Constants for optimizer/loss
    { MP_ROM_QSTR(MP_QSTR_ADAM), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_SGD), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_MSE), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_CROSSENTROPY), MP_ROM_INT(1) },
};
static MP_DEFINE_CONST_DICT(sentai_aifes_module_globals, sentai_aifes_module_globals_table);

const mp_obj_module_t sentai_aifes_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_aifes_module_globals,
};
