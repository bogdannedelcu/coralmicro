// ============== sentai.anomaly — Anomaly Detection ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Online anomaly detection using Welford's algorithm for streaming mean/covariance,
// Mahalanobis distance for multivariate scoring, and CUSUM for scalar change detection.
//
// Use after PCA for high-dim inputs: sentai.pca.transform_tpu(0) → sentai.anomaly.observe()

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== Anomaly State =====================

#define ANOMALY_MAX_DIM     256
#define ANOMALY_MIN_SAMPLES 2

static int g_anom_dim = 0;
static int g_anom_initialized = 0;
static int g_anom_n_samples = 0;
static float g_anom_threshold = 3.0f;  // default Mahalanobis threshold

// Welford online statistics
static float* g_anom_mean = NULL;       // dim floats
static float* g_anom_M2 = NULL;         // dim*dim floats (running sum of outer products)
static float* g_anom_cov = NULL;        // dim*dim floats (covariance matrix)
static float* g_anom_cov_inv = NULL;    // dim*dim floats (inverse covariance)
static int g_anom_cov_valid = 0;        // whether cov_inv is up to date

// Temp buffer
static float* g_anom_temp = NULL;       // dim floats

// CUSUM state (scalar change detection)
static int g_cusum_initialized = 0;
static float g_cusum_threshold = 5.0f;
static float g_cusum_drift = 0.5f;
static float g_cusum_s_pos = 0.0f;     // positive cumulative sum
static float g_cusum_s_neg = 0.0f;     // negative cumulative sum
static float g_cusum_mean = 0.0f;      // running mean for reference
static int g_cusum_n = 0;

// ===================== Internal Functions =====================

// Invert a symmetric positive-definite matrix using Cholesky decomposition
// a: input D×D matrix, inv: output D×D inverse, D: dimension
// Returns 0 on success, -1 if not positive definite
static int anom_cholesky_inv(const float* a, float* inv, int D) {
    // Cholesky factorization: A = L * L^T
    float* L = (float*)malloc(D * D * sizeof(float));
    if (!L) return -1;
    memset(L, 0, D * D * sizeof(float));

    for (int i = 0; i < D; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            for (int k = 0; k < j; k++)
                sum += L[i * D + k] * L[j * D + k];

            if (i == j) {
                float val = a[i * D + i] - sum;
                if (val <= 1e-8f) {
                    // Add small regularization
                    val = 1e-6f;
                }
                L[i * D + j] = sqrtf(val);
            } else {
                L[i * D + j] = (a[i * D + j] - sum) / L[j * D + j];
            }
        }
    }

    // Invert L (forward substitution)
    float* Li = (float*)malloc(D * D * sizeof(float));
    if (!Li) { free(L); return -1; }
    memset(Li, 0, D * D * sizeof(float));

    for (int i = 0; i < D; i++) {
        Li[i * D + i] = 1.0f / L[i * D + i];
        for (int j = i + 1; j < D; j++) {
            float sum = 0.0f;
            for (int k = i; k < j; k++)
                sum += L[j * D + k] * Li[k * D + i];
            Li[j * D + i] = -sum / L[j * D + j];
        }
    }

    // inv = Li^T * Li (since A^-1 = (L^T)^-1 * L^-1 = Li^T * Li)
    for (int i = 0; i < D; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            for (int k = i; k < D; k++)
                sum += Li[k * D + i] * Li[k * D + j];
            inv[i * D + j] = sum;
            inv[j * D + i] = sum;  // symmetric
        }
    }

    free(L);
    free(Li);
    return 0;
}

// Compute Mahalanobis distance: sqrt((x-mu)^T * Cov_inv * (x-mu))
static float anom_mahalanobis(const float* x, const float* mean,
                               const float* cov_inv, float* temp, int D) {
    // temp = x - mean
    for (int i = 0; i < D; i++) temp[i] = x[i] - mean[i];

    // result = temp^T * cov_inv * temp
    float dist = 0.0f;
    for (int i = 0; i < D; i++) {
        float row_sum = 0.0f;
        for (int j = 0; j < D; j++)
            row_sum += cov_inv[i * D + j] * temp[j];
        dist += temp[i] * row_sum;
    }
    return sqrtf(dist > 0.0f ? dist : 0.0f);
}

// Update covariance matrix and its inverse from M2
static int anom_update_cov(void) {
    int D = g_anom_dim;
    int N = g_anom_n_samples;
    if (N < ANOMALY_MIN_SAMPLES) return -1;

    float inv_n1 = 1.0f / (float)(N - 1);
    for (int i = 0; i < D * D; i++)
        g_anom_cov[i] = g_anom_M2[i] * inv_n1;

    // Add small regularization to diagonal for numerical stability
    for (int i = 0; i < D; i++)
        g_anom_cov[i * D + i] += 1e-6f;

    int rc = anom_cholesky_inv(g_anom_cov, g_anom_cov_inv, D);
    g_anom_cov_valid = (rc == 0);
    return rc;
}

// ===================== C API =====================

static int anom_init(int dim) {
    if (dim < 1 || dim > ANOMALY_MAX_DIM) return -1;

    if (g_anom_mean) { free(g_anom_mean); g_anom_mean = NULL; }
    if (g_anom_M2) { free(g_anom_M2); g_anom_M2 = NULL; }
    if (g_anom_cov) { free(g_anom_cov); g_anom_cov = NULL; }
    if (g_anom_cov_inv) { free(g_anom_cov_inv); g_anom_cov_inv = NULL; }
    if (g_anom_temp) { free(g_anom_temp); g_anom_temp = NULL; }

    g_anom_mean = (float*)malloc(dim * sizeof(float));
    g_anom_M2 = (float*)malloc(dim * dim * sizeof(float));
    g_anom_cov = (float*)malloc(dim * dim * sizeof(float));
    g_anom_cov_inv = (float*)malloc(dim * dim * sizeof(float));
    g_anom_temp = (float*)malloc(dim * sizeof(float));

    if (!g_anom_mean || !g_anom_M2 || !g_anom_cov || !g_anom_cov_inv || !g_anom_temp) {
        if (g_anom_mean) free(g_anom_mean);
        if (g_anom_M2) free(g_anom_M2);
        if (g_anom_cov) free(g_anom_cov);
        if (g_anom_cov_inv) free(g_anom_cov_inv);
        if (g_anom_temp) free(g_anom_temp);
        g_anom_mean = g_anom_M2 = g_anom_cov = g_anom_cov_inv = g_anom_temp = NULL;
        g_anom_initialized = 0;
        return -2;
    }

    memset(g_anom_mean, 0, dim * sizeof(float));
    memset(g_anom_M2, 0, dim * dim * sizeof(float));
    memset(g_anom_cov, 0, dim * dim * sizeof(float));
    memset(g_anom_cov_inv, 0, dim * dim * sizeof(float));

    g_anom_dim = dim;
    g_anom_n_samples = 0;
    g_anom_cov_valid = 0;
    g_anom_initialized = 1;
    g_anom_threshold = 3.0f;
    return 0;
}

// Welford online update: observe a new vector
static int anom_observe(const float* x) {
    if (!g_anom_initialized) return -1;
    int D = g_anom_dim;
    int n = g_anom_n_samples + 1;

    // delta = x - old_mean
    float* delta = g_anom_temp;
    for (int i = 0; i < D; i++) delta[i] = x[i] - g_anom_mean[i];

    // Update mean
    float inv_n = 1.0f / (float)n;
    for (int i = 0; i < D; i++) g_anom_mean[i] += delta[i] * inv_n;

    // delta2 = x - new_mean
    // M2 += outer(delta, delta2)
    for (int i = 0; i < D; i++) {
        for (int j = 0; j < D; j++) {
            g_anom_M2[i * D + j] += delta[i] * (x[j] - g_anom_mean[j]);
        }
    }

    g_anom_n_samples = n;
    g_anom_cov_valid = 0;  // invalidate cached inverse
    return 0;
}

// Compute Mahalanobis distance score for a vector
static float anom_score(const float* x) {
    if (!g_anom_initialized || g_anom_n_samples < ANOMALY_MIN_SAMPLES) return -1.0f;

    if (!g_anom_cov_valid) {
        if (anom_update_cov() < 0) return -1.0f;
    }

    return anom_mahalanobis(x, g_anom_mean, g_anom_cov_inv, g_anom_temp, g_anom_dim);
}

static int anom_save(const char* path) {
    if (!g_anom_initialized) return -1;
    int D = g_anom_dim;
    // Format: [dim:i32][n_samples:i32][threshold:f32][mean:D*f32][M2:D*D*f32]
    size_t size = 12 + D * 4 + D * D * 4;
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int32_t* hi = (int32_t*)buf;
    hi[0] = D;
    hi[1] = g_anom_n_samples;
    float* hf = (float*)&buf[8];
    hf[0] = g_anom_threshold;
    size_t off = 12;
    memcpy(&buf[off], g_anom_mean, D * sizeof(float)); off += D * 4;
    memcpy(&buf[off], g_anom_M2, D * D * sizeof(float));

    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)size);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

static int anom_load(const char* path) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 12) return -1;

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int read = sentai_fs_read(path, buf, size);
    if (read != size) { free(buf); return -3; }

    int32_t* hi = (int32_t*)buf;
    int D = hi[0];
    int n = hi[1];
    float threshold = *(float*)&buf[8];

    size_t expected = 12 + D * 4 + D * D * 4;
    if (size != (int)expected) { free(buf); return -4; }

    int rc = anom_init(D);
    if (rc < 0) { free(buf); return rc; }

    size_t off = 12;
    memcpy(g_anom_mean, &buf[off], D * sizeof(float)); off += D * 4;
    memcpy(g_anom_M2, &buf[off], D * D * sizeof(float));
    free(buf);

    g_anom_n_samples = n;
    g_anom_threshold = threshold;
    g_anom_cov_valid = 0;
    return 0;
}

// TPU interop
static int anom_read_tpu(int tpu_idx) {
    extern int sentai_tpu_get_output_size(int idx);
    extern const void* sentai_tpu_get_output_data(int idx);
    extern int sentai_tpu_get_output_type(int idx);
    extern int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);

    int size = sentai_tpu_get_output_size(tpu_idx);
    int type = sentai_tpu_get_output_type(tpu_idx);
    const void* data = sentai_tpu_get_output_data(tpu_idx);
    if (!data || size <= 0) return -5;

    int n_elements = 0;
    float scale = 1.0f;
    int32_t zero_point = 0;

    if (type == 1) { n_elements = size / 4; }
    else if (type == 9 || type == 3) {
        sentai_tpu_output_quant(tpu_idx, &scale, &zero_point);
        n_elements = size;
    } else { return -6; }

    if (n_elements != g_anom_dim) return -4;

    if (type == 1) {
        memcpy(g_anom_temp, data, n_elements * sizeof(float));
    } else if (type == 9) {
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++)
            g_anom_temp[i] = scale * ((float)idata[i] - (float)zero_point);
    } else {
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++)
            g_anom_temp[i] = scale * ((float)udata[i] - (float)zero_point);
    }
    return 0;
}

// ===================== MicroPython Bindings =====================

// sentai.anomaly.init(dim) -> int
static mp_obj_t mod_anom_init(mp_obj_t dim_obj) {
    return mp_obj_new_int(anom_init(mp_obj_get_int(dim_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_init_obj, mod_anom_init);

// sentai.anomaly.observe(vector) -> int
static mp_obj_t mod_anom_observe(mp_obj_t vec_obj) {
    if (!g_anom_initialized) return mp_obj_new_int(-1);
    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_anom_dim) return mp_obj_new_int(-4);
    for (size_t i = 0; i < list->len; i++)
        g_anom_temp[i] = mp_obj_get_float(list->items[i]);
    return mp_obj_new_int(anom_observe(g_anom_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_observe_obj, mod_anom_observe);

// sentai.anomaly.observe_tpu(idx) -> int
static mp_obj_t mod_anom_observe_tpu(mp_obj_t idx_obj) {
    if (!g_anom_initialized) return mp_obj_new_int(-1);
    int rc = anom_read_tpu(mp_obj_get_int(idx_obj));
    if (rc < 0) return mp_obj_new_int(rc);
    return mp_obj_new_int(anom_observe(g_anom_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_observe_tpu_obj, mod_anom_observe_tpu);

// sentai.anomaly.score(vector) -> float
static mp_obj_t mod_anom_score(mp_obj_t vec_obj) {
    if (!g_anom_initialized) return mp_obj_new_float(-1.0f);
    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_anom_dim) return mp_obj_new_float(-1.0f);
    for (size_t i = 0; i < list->len; i++)
        g_anom_temp[i] = mp_obj_get_float(list->items[i]);
    return mp_obj_new_float(anom_score(g_anom_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_score_obj, mod_anom_score);

// sentai.anomaly.score_tpu(idx) -> float
static mp_obj_t mod_anom_score_tpu(mp_obj_t idx_obj) {
    if (!g_anom_initialized) return mp_obj_new_float(-1.0f);
    int rc = anom_read_tpu(mp_obj_get_int(idx_obj));
    if (rc < 0) return mp_obj_new_float(-1.0f);
    return mp_obj_new_float(anom_score(g_anom_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_score_tpu_obj, mod_anom_score_tpu);

// sentai.anomaly.threshold(val) -> float (set/get threshold)
static mp_obj_t mod_anom_threshold(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) g_anom_threshold = mp_obj_get_float(args[0]);
    return mp_obj_new_float(g_anom_threshold);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_anom_threshold_obj, 0, 1, mod_anom_threshold);

// sentai.anomaly.is_anomaly(vector) -> bool
static mp_obj_t mod_anom_is_anomaly(mp_obj_t vec_obj) {
    if (!g_anom_initialized) return mp_const_false;
    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_anom_dim) return mp_const_false;
    for (size_t i = 0; i < list->len; i++)
        g_anom_temp[i] = mp_obj_get_float(list->items[i]);
    float s = anom_score(g_anom_temp);
    return mp_obj_new_bool(s > g_anom_threshold);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_is_anomaly_obj, mod_anom_is_anomaly);

// sentai.anomaly.is_anomaly_tpu(idx) -> bool
static mp_obj_t mod_anom_is_anomaly_tpu(mp_obj_t idx_obj) {
    if (!g_anom_initialized) return mp_const_false;
    int rc = anom_read_tpu(mp_obj_get_int(idx_obj));
    if (rc < 0) return mp_const_false;
    float s = anom_score(g_anom_temp);
    return mp_obj_new_bool(s > g_anom_threshold);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_is_anomaly_tpu_obj, mod_anom_is_anomaly_tpu);

// ---- CUSUM ----

// sentai.anomaly.cusum_init(threshold, drift) -> None
static mp_obj_t mod_anom_cusum_init(mp_obj_t thresh_obj, mp_obj_t drift_obj) {
    g_cusum_threshold = mp_obj_get_float(thresh_obj);
    g_cusum_drift = mp_obj_get_float(drift_obj);
    g_cusum_s_pos = 0.0f;
    g_cusum_s_neg = 0.0f;
    g_cusum_mean = 0.0f;
    g_cusum_n = 0;
    g_cusum_initialized = 1;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_anom_cusum_init_obj, mod_anom_cusum_init);

// sentai.anomaly.cusum_observe(value) -> int (0 = normal, 1 = change detected)
static mp_obj_t mod_anom_cusum_observe(mp_obj_t val_obj) {
    if (!g_cusum_initialized) return mp_obj_new_int(-1);
    float x = mp_obj_get_float(val_obj);

    // Update running mean (reference level)
    g_cusum_n++;
    g_cusum_mean += (x - g_cusum_mean) / (float)g_cusum_n;

    // CUSUM update
    float z = x - g_cusum_mean;
    g_cusum_s_pos = fmaxf(0.0f, g_cusum_s_pos + z - g_cusum_drift);
    g_cusum_s_neg = fmaxf(0.0f, g_cusum_s_neg - z - g_cusum_drift);

    int alarm = (g_cusum_s_pos > g_cusum_threshold || g_cusum_s_neg > g_cusum_threshold) ? 1 : 0;
    return mp_obj_new_int(alarm);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_cusum_observe_obj, mod_anom_cusum_observe);

// sentai.anomaly.cusum_score() -> tuple(s_pos, s_neg)
static mp_obj_t mod_anom_cusum_score(void) {
    mp_obj_t items[2] = {
        mp_obj_new_float(g_cusum_s_pos),
        mp_obj_new_float(g_cusum_s_neg)
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_anom_cusum_score_obj, mod_anom_cusum_score);

// sentai.anomaly.cusum_reset() -> None
static mp_obj_t mod_anom_cusum_reset(void) {
    g_cusum_s_pos = 0.0f;
    g_cusum_s_neg = 0.0f;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_anom_cusum_reset_obj, mod_anom_cusum_reset);

// sentai.anomaly.stats() -> dict
static mp_obj_t mod_anom_stats(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dim), mp_obj_new_int(g_anom_dim));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_samples), mp_obj_new_int(g_anom_n_samples));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_threshold), mp_obj_new_float(g_anom_threshold));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), mp_obj_new_bool(g_anom_initialized));
    if (g_anom_initialized && g_anom_n_samples > 0) {
        mp_obj_list_t* m = MP_OBJ_TO_PTR(mp_obj_new_list(g_anom_dim, NULL));
        for (int i = 0; i < g_anom_dim; i++)
            m->items[i] = mp_obj_new_float(g_anom_mean[i]);
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mean), MP_OBJ_FROM_PTR(m));
    }
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_anom_stats_obj, mod_anom_stats);

// sentai.anomaly.save(path) -> int
static mp_obj_t mod_anom_save(mp_obj_t path_obj) {
    return mp_obj_new_int(anom_save(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_save_obj, mod_anom_save);

// sentai.anomaly.load(path) -> int
static mp_obj_t mod_anom_load(mp_obj_t path_obj) {
    return mp_obj_new_int(anom_load(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_anom_load_obj, mod_anom_load);

// sentai.anomaly.info() -> dict (alias for stats)
static mp_obj_t mod_anom_info(void) { return mod_anom_stats(); }
static MP_DEFINE_CONST_FUN_OBJ_0(mod_anom_info_obj, mod_anom_info);

// sentai.anomaly.clear() -> None
static mp_obj_t mod_anom_clear(void) {
    if (g_anom_initialized) {
        memset(g_anom_mean, 0, g_anom_dim * sizeof(float));
        memset(g_anom_M2, 0, g_anom_dim * g_anom_dim * sizeof(float));
        g_anom_n_samples = 0;
        g_anom_cov_valid = 0;
    }
    g_cusum_s_pos = g_cusum_s_neg = 0.0f;
    g_cusum_n = 0;
    g_cusum_mean = 0.0f;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_anom_clear_obj, mod_anom_clear);

// ---- module table ----
static const mp_rom_map_elem_t sentai_anomaly_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_anomaly) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_anom_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_observe), MP_ROM_PTR(&mod_anom_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_observe_tpu), MP_ROM_PTR(&mod_anom_observe_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_score), MP_ROM_PTR(&mod_anom_score_obj) },
    { MP_ROM_QSTR(MP_QSTR_score_tpu), MP_ROM_PTR(&mod_anom_score_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_threshold), MP_ROM_PTR(&mod_anom_threshold_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_anomaly), MP_ROM_PTR(&mod_anom_is_anomaly_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_anomaly_tpu), MP_ROM_PTR(&mod_anom_is_anomaly_tpu_obj) },
    // CUSUM
    { MP_ROM_QSTR(MP_QSTR_cusum_init), MP_ROM_PTR(&mod_anom_cusum_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_cusum_observe), MP_ROM_PTR(&mod_anom_cusum_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_cusum_score), MP_ROM_PTR(&mod_anom_cusum_score_obj) },
    { MP_ROM_QSTR(MP_QSTR_cusum_reset), MP_ROM_PTR(&mod_anom_cusum_reset_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_anom_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_anom_load_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&mod_anom_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_anom_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&mod_anom_clear_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_anomaly_globals, sentai_anomaly_globals_table);
static const mp_obj_module_t sentai_anomaly_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_anomaly_globals,
};
