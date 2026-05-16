// ============== sentai.pca — PCA Dimensionality Reduction ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Principal Component Analysis for reducing high-dimensional embeddings
// (e.g., 1280-dim MobileNet) to low-dimensional vectors for clustering,
// anomaly detection, or visualization.
//
// Uses power iteration to find top-k eigenvectors of the covariance matrix.
// For very high dimensions (>512), consider random projection as alternative.

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== PCA State =====================

#define PCA_MAX_IN_DIM    2048
#define PCA_MAX_OUT_DIM   128
#define PCA_MAX_VECTORS   1000
#define PCA_POWER_ITER    100    // iterations for power method

static int g_pca_in_dim = 0;
static int g_pca_out_dim = 0;
static int g_pca_initialized = 0;
static int g_pca_fitted = 0;

// Training vectors: n_vectors * in_dim floats
static float* g_pca_vectors = NULL;
static int g_pca_n_vectors = 0;

// Mean vector: in_dim floats
static float* g_pca_mean = NULL;

// Projection matrix: out_dim * in_dim floats (each row is an eigenvector)
static float* g_pca_components = NULL;

// Explained variance: out_dim floats
static float* g_pca_variance = NULL;

// Temp buffer: in_dim floats
static float* g_pca_temp = NULL;

// ===================== Internal Functions =====================

// Dot product of two vectors
static float pca_dot(const float* a, const float* b, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += a[i] * b[i];
    return sum;
}

// L2 norm
static float pca_norm(const float* v, int n) {
    return sqrtf(pca_dot(v, v, n));
}

// Normalize vector in-place, return original norm
static float pca_normalize(float* v, int n) {
    float norm = pca_norm(v, n);
    if (norm > 1e-12f) {
        float inv = 1.0f / norm;
        for (int i = 0; i < n; i++) v[i] *= inv;
    }
    return norm;
}

// Subtract projection of v onto u from v: v = v - (v.u)*u
static void pca_deflate(float* v, const float* u, int n) {
    float proj = pca_dot(v, u, n);
    for (int i = 0; i < n; i++) v[i] -= proj * u[i];
}

// Multiply covariance matrix by vector: out = (1/N) * X_centered^T * X_centered * v
// X_centered is g_pca_vectors with mean subtracted
// This avoids forming the full D×D covariance matrix!
static void pca_cov_mul(const float* v, float* out, int in_dim, int n_vec,
                         const float* vectors, const float* mean) {
    // Step 1: tmp[j] = X_centered[j,:] . v  for all j (N dot products of dim D)
    // Step 2: out[i] = sum_j( tmp[j] * (X_centered[j,i] - mean[i]) ) / N
    //
    // This is O(N*D) instead of O(D^2) for matrix-vector multiply

    memset(out, 0, in_dim * sizeof(float));
    for (int j = 0; j < n_vec; j++) {
        // Compute dot product of (centered row j) with v
        float dot = 0.0f;
        for (int i = 0; i < in_dim; i++) {
            dot += (vectors[j * in_dim + i] - mean[i]) * v[i];
        }
        // Accumulate outer product contribution
        for (int i = 0; i < in_dim; i++) {
            out[i] += dot * (vectors[j * in_dim + i] - mean[i]);
        }
    }
    // Divide by N
    float inv_n = 1.0f / (float)n_vec;
    for (int i = 0; i < in_dim; i++) out[i] *= inv_n;
}

// ===================== C API =====================

static int pca_init(int in_dim, int out_dim) {
    if (in_dim < 1 || in_dim > PCA_MAX_IN_DIM) return -1;
    if (out_dim < 1 || out_dim > PCA_MAX_OUT_DIM) return -2;
    if (out_dim > in_dim) return -3;

    // Free existing
    if (g_pca_vectors) { free(g_pca_vectors); g_pca_vectors = NULL; }
    if (g_pca_mean) { free(g_pca_mean); g_pca_mean = NULL; }
    if (g_pca_components) { free(g_pca_components); g_pca_components = NULL; }
    if (g_pca_variance) { free(g_pca_variance); g_pca_variance = NULL; }
    if (g_pca_temp) { free(g_pca_temp); g_pca_temp = NULL; }

    g_pca_vectors = (float*)malloc(PCA_MAX_VECTORS * in_dim * sizeof(float));
    g_pca_mean = (float*)malloc(in_dim * sizeof(float));
    g_pca_components = (float*)malloc(out_dim * in_dim * sizeof(float));
    g_pca_variance = (float*)malloc(out_dim * sizeof(float));
    g_pca_temp = (float*)malloc(in_dim * sizeof(float));

    if (!g_pca_vectors || !g_pca_mean || !g_pca_components ||
        !g_pca_variance || !g_pca_temp) {
        if (g_pca_vectors) free(g_pca_vectors);
        if (g_pca_mean) free(g_pca_mean);
        if (g_pca_components) free(g_pca_components);
        if (g_pca_variance) free(g_pca_variance);
        if (g_pca_temp) free(g_pca_temp);
        g_pca_vectors = g_pca_mean = g_pca_components = g_pca_variance = g_pca_temp = NULL;
        g_pca_initialized = 0;
        return -4;
    }

    memset(g_pca_mean, 0, in_dim * sizeof(float));
    memset(g_pca_components, 0, out_dim * in_dim * sizeof(float));
    memset(g_pca_variance, 0, out_dim * sizeof(float));

    g_pca_in_dim = in_dim;
    g_pca_out_dim = out_dim;
    g_pca_n_vectors = 0;
    g_pca_fitted = 0;
    g_pca_initialized = 1;
    return 0;
}

static int pca_add(const float* vec) {
    if (!g_pca_initialized) return -1;
    if (g_pca_n_vectors >= PCA_MAX_VECTORS) return -2;

    memcpy(&g_pca_vectors[g_pca_n_vectors * g_pca_in_dim],
           vec, g_pca_in_dim * sizeof(float));
    g_pca_n_vectors++;
    g_pca_fitted = 0;  // invalidate fit
    return 0;
}

static int pca_fit(void) {
    if (!g_pca_initialized) return -1;
    if (g_pca_n_vectors < g_pca_out_dim) return -2;  // need at least out_dim vectors

    int D = g_pca_in_dim;
    int N = g_pca_n_vectors;
    int K = g_pca_out_dim;

    // Compute mean
    memset(g_pca_mean, 0, D * sizeof(float));
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < D; i++) {
            g_pca_mean[i] += g_pca_vectors[j * D + i];
        }
    }
    float inv_n = 1.0f / (float)N;
    for (int i = 0; i < D; i++) g_pca_mean[i] *= inv_n;

    // Allocate work vector for power iteration
    float* work = (float*)malloc(D * sizeof(float));
    if (!work) return -3;

    // Find top-K eigenvectors using deflated power iteration
    for (int k = 0; k < K; k++) {
        float* eigvec = &g_pca_components[k * D];

        // Initialize with pseudo-random direction (deterministic based on k)
        for (int i = 0; i < D; i++) {
            eigvec[i] = sinf((float)(i + 1) * (float)(k + 1) * 0.1f);
        }
        pca_normalize(eigvec, D);

        // Power iteration: repeatedly multiply by covariance and normalize
        for (int iter = 0; iter < PCA_POWER_ITER; iter++) {
            // work = Cov * eigvec  (without forming Cov explicitly)
            pca_cov_mul(eigvec, work, D, N, g_pca_vectors, g_pca_mean);

            // Deflate against previously found eigenvectors
            for (int p = 0; p < k; p++) {
                pca_deflate(work, &g_pca_components[p * D], D);
            }

            pca_normalize(work, D);
            memcpy(eigvec, work, D * sizeof(float));
        }

        // Compute eigenvalue (variance along this component)
        pca_cov_mul(eigvec, work, D, N, g_pca_vectors, g_pca_mean);
        g_pca_variance[k] = pca_dot(eigvec, work, D);
    }

    free(work);
    g_pca_fitted = 1;
    return 0;
}

// Transform a vector: out = components * (vec - mean)
// out must have out_dim floats
static int pca_transform(const float* vec, float* out) {
    if (!g_pca_fitted) return -1;
    int D = g_pca_in_dim;
    int K = g_pca_out_dim;

    for (int k = 0; k < K; k++) {
        float dot = 0.0f;
        for (int i = 0; i < D; i++) {
            dot += g_pca_components[k * D + i] * (vec[i] - g_pca_mean[i]);
        }
        out[k] = dot;
    }
    return 0;
}

// Inverse transform: reconstruct = components^T * reduced + mean
static int pca_inverse(const float* reduced, float* out) {
    if (!g_pca_fitted) return -1;
    int D = g_pca_in_dim;
    int K = g_pca_out_dim;

    for (int i = 0; i < D; i++) {
        float sum = g_pca_mean[i];
        for (int k = 0; k < K; k++) {
            sum += g_pca_components[k * D + i] * reduced[k];
        }
        out[i] = sum;
    }
    return 0;
}

static int pca_save(const char* path) {
    if (!g_pca_fitted) return -1;

    int D = g_pca_in_dim;
    int K = g_pca_out_dim;
    // Format: [in_dim:i32][out_dim:i32][mean:D*f32][components:K*D*f32][variance:K*f32]
    size_t size = 8 + D * 4 + K * D * 4 + K * 4;
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int32_t* header = (int32_t*)buf;
    header[0] = D;
    header[1] = K;
    size_t off = 8;
    memcpy(&buf[off], g_pca_mean, D * sizeof(float)); off += D * 4;
    memcpy(&buf[off], g_pca_components, K * D * sizeof(float)); off += K * D * 4;
    memcpy(&buf[off], g_pca_variance, K * sizeof(float));

    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)size);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

static int pca_load(const char* path) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 8) return -1;

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int read = sentai_fs_read(path, buf, size);
    if (read != size) { free(buf); return -3; }

    int32_t* header = (int32_t*)buf;
    int D = header[0], K = header[1];

    size_t expected = 8 + D * 4 + K * D * 4 + K * 4;
    if (size != (int)expected) { free(buf); return -4; }

    int rc = pca_init(D, K);
    if (rc < 0) { free(buf); return rc; }

    size_t off = 8;
    memcpy(g_pca_mean, &buf[off], D * sizeof(float)); off += D * 4;
    memcpy(g_pca_components, &buf[off], K * D * sizeof(float)); off += K * D * 4;
    memcpy(g_pca_variance, &buf[off], K * sizeof(float));
    free(buf);

    g_pca_fitted = 1;
    return 0;
}

// ===================== TPU Interop =====================

// Dequantize TPU output to temp buffer (reuse pattern from kmeans)
static int pca_read_tpu(int tpu_idx) {
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

    if (n_elements != g_pca_in_dim) return -4;

    if (type == 1) {
        memcpy(g_pca_temp, data, n_elements * sizeof(float));
    } else if (type == 9) {
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++)
            g_pca_temp[i] = scale * ((float)idata[i] - (float)zero_point);
    } else {
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++)
            g_pca_temp[i] = scale * ((float)udata[i] - (float)zero_point);
    }
    return 0;
}

// ===================== MicroPython Bindings =====================

// sentai.pca.init(in_dim, out_dim) -> int
static mp_obj_t mod_pca_init(mp_obj_t in_obj, mp_obj_t out_obj) {
    return mp_obj_new_int(pca_init(mp_obj_get_int(in_obj), mp_obj_get_int(out_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_pca_init_obj, mod_pca_init);

// sentai.pca.add(vector) -> int
static mp_obj_t mod_pca_add(mp_obj_t vec_obj) {
    if (!g_pca_initialized) return mp_obj_new_int(-1);
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(vec_obj, &len, &items);
    if (len != (size_t)g_pca_in_dim) return mp_obj_new_int(-4);
    for (size_t i = 0; i < len; i++)
        g_pca_temp[i] = mp_obj_get_float(items[i]);
    return mp_obj_new_int(pca_add(g_pca_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_add_obj, mod_pca_add);

// sentai.pca.from_tpu(idx) -> int
static mp_obj_t mod_pca_from_tpu(mp_obj_t idx_obj) {
    if (!g_pca_initialized) return mp_obj_new_int(-1);
    int rc = pca_read_tpu(mp_obj_get_int(idx_obj));
    if (rc < 0) return mp_obj_new_int(rc);
    return mp_obj_new_int(pca_add(g_pca_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_from_tpu_obj, mod_pca_from_tpu);

// sentai.pca.fit() -> int
static mp_obj_t mod_pca_fit(void) {
    return mp_obj_new_int(pca_fit());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_pca_fit_obj, mod_pca_fit);

// sentai.pca.transform(vector) -> list
static mp_obj_t mod_pca_transform(mp_obj_t vec_obj) {
    if (!g_pca_fitted) return mp_const_none;
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(vec_obj, &len, &items);
    if (len != (size_t)g_pca_in_dim) return mp_const_none;
    for (size_t i = 0; i < len; i++)
        g_pca_temp[i] = mp_obj_get_float(items[i]);

    float* out = (float*)malloc(g_pca_out_dim * sizeof(float));
    if (!out) return mp_const_none;
    pca_transform(g_pca_temp, out);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(g_pca_out_dim, NULL));
    for (int i = 0; i < g_pca_out_dim; i++)
        result->items[i] = mp_obj_new_float(out[i]);
    free(out);
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_transform_obj, mod_pca_transform);

// sentai.pca.transform_tpu(idx) -> list
static mp_obj_t mod_pca_transform_tpu(mp_obj_t idx_obj) {
    if (!g_pca_fitted) return mp_const_none;
    int rc = pca_read_tpu(mp_obj_get_int(idx_obj));
    if (rc < 0) return mp_const_none;

    float* out = (float*)malloc(g_pca_out_dim * sizeof(float));
    if (!out) return mp_const_none;
    pca_transform(g_pca_temp, out);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(g_pca_out_dim, NULL));
    for (int i = 0; i < g_pca_out_dim; i++)
        result->items[i] = mp_obj_new_float(out[i]);
    free(out);
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_transform_tpu_obj, mod_pca_transform_tpu);

// sentai.pca.inverse(reduced) -> list
static mp_obj_t mod_pca_inverse(mp_obj_t vec_obj) {
    if (!g_pca_fitted) return mp_const_none;
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(vec_obj, &len, &items);
    if (len != (size_t)g_pca_out_dim) return mp_const_none;

    float* reduced = (float*)malloc(g_pca_out_dim * sizeof(float));
    float* recon = (float*)malloc(g_pca_in_dim * sizeof(float));
    if (!reduced || !recon) {
        if (reduced) free(reduced);
        if (recon) free(recon);
        return mp_const_none;
    }

    for (size_t i = 0; i < len; i++)
        reduced[i] = mp_obj_get_float(items[i]);
    pca_inverse(reduced, recon);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(g_pca_in_dim, NULL));
    for (int i = 0; i < g_pca_in_dim; i++)
        result->items[i] = mp_obj_new_float(recon[i]);
    free(reduced);
    free(recon);
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_inverse_obj, mod_pca_inverse);

// sentai.pca.explained_variance() -> list
static mp_obj_t mod_pca_explained_variance(void) {
    if (!g_pca_fitted) return mp_const_none;
    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(g_pca_out_dim, NULL));
    for (int i = 0; i < g_pca_out_dim; i++)
        list->items[i] = mp_obj_new_float(g_pca_variance[i]);
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_pca_explained_variance_obj, mod_pca_explained_variance);

// sentai.pca.save(path) -> int
static mp_obj_t mod_pca_save(mp_obj_t path_obj) {
    return mp_obj_new_int(pca_save(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_save_obj, mod_pca_save);

// sentai.pca.load(path) -> int
static mp_obj_t mod_pca_load(mp_obj_t path_obj) {
    return mp_obj_new_int(pca_load(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_pca_load_obj, mod_pca_load);

// sentai.pca.info() -> dict
static mp_obj_t mod_pca_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(5));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_in_dim), mp_obj_new_int(g_pca_in_dim));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_out_dim), mp_obj_new_int(g_pca_out_dim));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_vectors), mp_obj_new_int(g_pca_n_vectors));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_fitted), mp_obj_new_bool(g_pca_fitted));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), mp_obj_new_bool(g_pca_initialized));
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_pca_info_obj, mod_pca_info);

// sentai.pca.clear() -> None
static mp_obj_t mod_pca_clear(void) {
    g_pca_n_vectors = 0;
    g_pca_fitted = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_pca_clear_obj, mod_pca_clear);

// ---- module table ----
static const mp_rom_map_elem_t sentai_pca_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pca) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_pca_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_add), MP_ROM_PTR(&mod_pca_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_tpu), MP_ROM_PTR(&mod_pca_from_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_fit), MP_ROM_PTR(&mod_pca_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_transform), MP_ROM_PTR(&mod_pca_transform_obj) },
    { MP_ROM_QSTR(MP_QSTR_transform_tpu), MP_ROM_PTR(&mod_pca_transform_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_inverse), MP_ROM_PTR(&mod_pca_inverse_obj) },
    { MP_ROM_QSTR(MP_QSTR_explained_variance), MP_ROM_PTR(&mod_pca_explained_variance_obj) },
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_pca_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_pca_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_pca_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&mod_pca_clear_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pca_globals, sentai_pca_globals_table);
static const mp_obj_module_t sentai_pca_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_pca_globals,
};
