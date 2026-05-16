// ============== sentai.kmeans — K-Means Clustering for Embeddings ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Simple k-means clustering for classifying embedding vectors.
// Useful for few-shot learning: collect embeddings from TPU, cluster them,
// then classify new embeddings by nearest centroid.
//
// Storage is in SDRAM to handle large embedding dimensions (e.g., 1280-dim MobileNet).

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== K-Means State =====================

// Maximum supported configuration
#define KMEANS_MAX_K        64      // Max clusters
#define KMEANS_MAX_DIM      2048    // Max embedding dimension
#define KMEANS_MAX_VECTORS  1000    // Max vectors during training

// State variables (stored in SDRAM via modsentai_hal.cc)
static int g_kmeans_k = 0;
static int g_kmeans_dim = 0;
static int g_kmeans_initialized = 0;

// Centroids: k * dim floats
static float* g_kmeans_centroids = NULL;

// Training vectors: max_vectors * dim floats
static float* g_kmeans_vectors = NULL;
static int* g_kmeans_labels = NULL;  // Cluster assignment for each vector
static int g_kmeans_n_vectors = 0;
static int* g_kmeans_cluster_counts = NULL;  // Count of vectors per cluster

// Temporary buffer for inference
static float* g_kmeans_temp = NULL;

// ===================== Internal Functions =====================

// Euclidean distance squared (faster than sqrt for comparisons)
static float kmeans_distance_sq(const float* a, const float* b, int dim) {
    float sum = 0.0f;
    for (int i = 0; i < dim; i++) {
        float d = a[i] - b[i];
        sum += d * d;
    }
    return sum;
}

// Find nearest centroid, return index
static int kmeans_nearest_centroid(const float* vec, int dim, int k, const float* centroids) {
    int best = 0;
    float best_dist = kmeans_distance_sq(vec, centroids, dim);
    for (int i = 1; i < k; i++) {
        float dist = kmeans_distance_sq(vec, &centroids[i * dim], dim);
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

// ===================== C API (called from MicroPython bindings) =====================

// Initialize k-means with k clusters and dimension
// Returns 0 on success, negative on error
static int kmeans_init(int k, int dim) {
    if (k < 1 || k > KMEANS_MAX_K) return -1;
    if (dim < 1 || dim > KMEANS_MAX_DIM) return -2;

    // Free existing allocations
    if (g_kmeans_centroids) { free(g_kmeans_centroids); g_kmeans_centroids = NULL; }
    if (g_kmeans_vectors) { free(g_kmeans_vectors); g_kmeans_vectors = NULL; }
    if (g_kmeans_labels) { free(g_kmeans_labels); g_kmeans_labels = NULL; }
    if (g_kmeans_cluster_counts) { free(g_kmeans_cluster_counts); g_kmeans_cluster_counts = NULL; }
    if (g_kmeans_temp) { free(g_kmeans_temp); g_kmeans_temp = NULL; }

    // Allocate new buffers
    g_kmeans_centroids = (float*)malloc(k * dim * sizeof(float));
    g_kmeans_vectors = (float*)malloc(KMEANS_MAX_VECTORS * dim * sizeof(float));
    g_kmeans_labels = (int*)malloc(KMEANS_MAX_VECTORS * sizeof(int));
    g_kmeans_cluster_counts = (int*)malloc(k * sizeof(int));
    g_kmeans_temp = (float*)malloc(dim * sizeof(float));

    if (!g_kmeans_centroids || !g_kmeans_vectors || !g_kmeans_labels || 
        !g_kmeans_cluster_counts || !g_kmeans_temp) {
        // Allocation failed - clean up
        if (g_kmeans_centroids) free(g_kmeans_centroids);
        if (g_kmeans_vectors) free(g_kmeans_vectors);
        if (g_kmeans_labels) free(g_kmeans_labels);
        if (g_kmeans_cluster_counts) free(g_kmeans_cluster_counts);
        if (g_kmeans_temp) free(g_kmeans_temp);
        g_kmeans_centroids = g_kmeans_vectors = g_kmeans_temp = NULL;
        g_kmeans_labels = g_kmeans_cluster_counts = NULL;
        g_kmeans_initialized = 0;
        return -3;
    }

    // Initialize
    memset(g_kmeans_centroids, 0, k * dim * sizeof(float));
    memset(g_kmeans_cluster_counts, 0, k * sizeof(int));
    g_kmeans_k = k;
    g_kmeans_dim = dim;
    g_kmeans_n_vectors = 0;
    g_kmeans_initialized = 1;

    return 0;
}

// Add a vector to a specific cluster (for manual labeling)
// Returns 0 on success, negative on error
static int kmeans_add(int cluster_idx, const float* vec) {
    if (!g_kmeans_initialized) return -1;
    if (cluster_idx < 0 || cluster_idx >= g_kmeans_k) return -2;
    if (g_kmeans_n_vectors >= KMEANS_MAX_VECTORS) return -3;

    // Copy vector
    memcpy(&g_kmeans_vectors[g_kmeans_n_vectors * g_kmeans_dim], 
           vec, g_kmeans_dim * sizeof(float));
    g_kmeans_labels[g_kmeans_n_vectors] = cluster_idx;
    g_kmeans_cluster_counts[cluster_idx]++;
    g_kmeans_n_vectors++;

    return 0;
}

// Compute centroids from manually added vectors
// Returns 0 on success, negative on error
static int kmeans_compute(void) {
    if (!g_kmeans_initialized) return -1;
    if (g_kmeans_n_vectors == 0) return -2;

    // Reset centroids
    memset(g_kmeans_centroids, 0, g_kmeans_k * g_kmeans_dim * sizeof(float));

    // Sum vectors for each cluster
    for (int i = 0; i < g_kmeans_n_vectors; i++) {
        int c = g_kmeans_labels[i];
        for (int d = 0; d < g_kmeans_dim; d++) {
            g_kmeans_centroids[c * g_kmeans_dim + d] += 
                g_kmeans_vectors[i * g_kmeans_dim + d];
        }
    }

    // Divide by count to get mean
    for (int c = 0; c < g_kmeans_k; c++) {
        if (g_kmeans_cluster_counts[c] > 0) {
            for (int d = 0; d < g_kmeans_dim; d++) {
                g_kmeans_centroids[c * g_kmeans_dim + d] /= g_kmeans_cluster_counts[c];
            }
        }
    }

    return 0;
}

// Run Lloyd's k-means algorithm on the stored vectors
// max_iter: maximum iterations (0 = until convergence)
// Returns number of iterations, negative on error
static int kmeans_fit(int max_iter) {
    if (!g_kmeans_initialized) return -1;
    if (g_kmeans_n_vectors < g_kmeans_k) return -2;  // Need at least k vectors

    if (max_iter <= 0) max_iter = 100;

    // Initialize centroids with first k vectors (simple initialization)
    for (int c = 0; c < g_kmeans_k; c++) {
        memcpy(&g_kmeans_centroids[c * g_kmeans_dim],
               &g_kmeans_vectors[c * g_kmeans_dim],
               g_kmeans_dim * sizeof(float));
    }

    int iter;
    for (iter = 0; iter < max_iter; iter++) {
        int changes = 0;

        // Assign vectors to nearest centroid
        memset(g_kmeans_cluster_counts, 0, g_kmeans_k * sizeof(int));
        for (int i = 0; i < g_kmeans_n_vectors; i++) {
            int new_label = kmeans_nearest_centroid(
                &g_kmeans_vectors[i * g_kmeans_dim],
                g_kmeans_dim, g_kmeans_k, g_kmeans_centroids);
            if (new_label != g_kmeans_labels[i]) {
                changes++;
                g_kmeans_labels[i] = new_label;
            }
            g_kmeans_cluster_counts[new_label]++;
        }

        // Update centroids
        memset(g_kmeans_centroids, 0, g_kmeans_k * g_kmeans_dim * sizeof(float));
        for (int i = 0; i < g_kmeans_n_vectors; i++) {
            int c = g_kmeans_labels[i];
            for (int d = 0; d < g_kmeans_dim; d++) {
                g_kmeans_centroids[c * g_kmeans_dim + d] += 
                    g_kmeans_vectors[i * g_kmeans_dim + d];
            }
        }
        for (int c = 0; c < g_kmeans_k; c++) {
            if (g_kmeans_cluster_counts[c] > 0) {
                for (int d = 0; d < g_kmeans_dim; d++) {
                    g_kmeans_centroids[c * g_kmeans_dim + d] /= g_kmeans_cluster_counts[c];
                }
            }
        }

        // Converged?
        if (changes == 0) break;
    }

    return iter + 1;
}

// Predict cluster for a vector
// Returns cluster index (0 to k-1), negative on error
static int kmeans_predict(const float* vec) {
    if (!g_kmeans_initialized) return -1;
    return kmeans_nearest_centroid(vec, g_kmeans_dim, g_kmeans_k, g_kmeans_centroids);
}

// Get distances to all centroids
// distances: output array of k floats
// Returns 0 on success
static int kmeans_distances(const float* vec, float* distances) {
    if (!g_kmeans_initialized) return -1;
    for (int c = 0; c < g_kmeans_k; c++) {
        distances[c] = sqrtf(kmeans_distance_sq(vec, &g_kmeans_centroids[c * g_kmeans_dim], g_kmeans_dim));
    }
    return 0;
}

// Get a centroid vector
// Returns pointer to centroid, NULL on error
static const float* kmeans_get_centroid(int idx) {
    if (!g_kmeans_initialized || idx < 0 || idx >= g_kmeans_k) return NULL;
    return &g_kmeans_centroids[idx * g_kmeans_dim];
}

// Save centroids to file (binary format: k, dim, then k*dim floats)
static int kmeans_save(const char* path) {
    if (!g_kmeans_initialized) return -1;

    // Build binary data: [k:int32][dim:int32][centroids:k*dim*float32]
    size_t size = 8 + g_kmeans_k * g_kmeans_dim * sizeof(float);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int32_t* header = (int32_t*)buf;
    header[0] = g_kmeans_k;
    header[1] = g_kmeans_dim;
    memcpy(&buf[8], g_kmeans_centroids, g_kmeans_k * g_kmeans_dim * sizeof(float));

    // Use LittleFS write
    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)size);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

// Load centroids from file
static int kmeans_load(const char* path) {
    // Use LittleFS read
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 8) return -1;

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int read = sentai_fs_read(path, buf, size);
    if (read != size) {
        free(buf);
        return -3;
    }

    int32_t* header = (int32_t*)buf;
    int k = header[0];
    int dim = header[1];

    // Validate
    if (size != 8 + k * dim * (int)sizeof(float)) {
        free(buf);
        return -4;
    }

    // Initialize with loaded dimensions
    int rc = kmeans_init(k, dim);
    if (rc < 0) {
        free(buf);
        return rc;
    }

    // Copy centroids
    memcpy(g_kmeans_centroids, &buf[8], k * dim * sizeof(float));
    free(buf);
    return 0;
}

// Clear training vectors (keep centroids)
static void kmeans_clear_vectors(void) {
    g_kmeans_n_vectors = 0;
    if (g_kmeans_cluster_counts) {
        memset(g_kmeans_cluster_counts, 0, g_kmeans_k * sizeof(int));
    }
}

// ===================== MicroPython Bindings =====================

// sentai.kmeans.init(k, dim) -> int
// Initialize k-means with k clusters and dimension
static mp_obj_t mod_kmeans_init(mp_obj_t k_obj, mp_obj_t dim_obj) {
    int k = mp_obj_get_int(k_obj);
    int dim = mp_obj_get_int(dim_obj);
    return mp_obj_new_int(kmeans_init(k, dim));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_kmeans_init_obj, mod_kmeans_init);

// sentai.kmeans.add(cluster_idx, vector) -> int
// Add a vector to a specific cluster
static mp_obj_t mod_kmeans_add(mp_obj_t cluster_obj, mp_obj_t vec_obj) {
    if (!g_kmeans_initialized) return mp_obj_new_int(-1);

    int cluster = mp_obj_get_int(cluster_obj);
    
    // Get vector from list
    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_kmeans_dim) {
        return mp_obj_new_int(-4);  // Dimension mismatch
    }

    // Copy to temp buffer
    for (size_t i = 0; i < list->len; i++) {
        g_kmeans_temp[i] = mp_obj_get_float(list->items[i]);
    }

    return mp_obj_new_int(kmeans_add(cluster, g_kmeans_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_kmeans_add_obj, mod_kmeans_add);

// sentai.kmeans.add_from_tpu(cluster_idx, tpu_output_idx) -> int
// Add TPU output tensor as a vector to cluster
static mp_obj_t mod_kmeans_add_from_tpu(mp_obj_t cluster_obj, mp_obj_t idx_obj) {
    if (!g_kmeans_initialized) return mp_obj_new_int(-1);

    int cluster = mp_obj_get_int(cluster_obj);
    int tpu_idx = mp_obj_get_int(idx_obj);

    // Get TPU output
    extern int sentai_tpu_get_output_size(int idx);
    extern const void* sentai_tpu_get_output_data(int idx);
    extern int sentai_tpu_get_output_type(int idx);
    extern int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);

    int size = sentai_tpu_get_output_size(tpu_idx);
    int type = sentai_tpu_get_output_type(tpu_idx);
    const void* data = sentai_tpu_get_output_data(tpu_idx);

    if (!data || size <= 0) return mp_obj_new_int(-5);

    // Calculate element count
    int n_elements = 0;
    float scale = 1.0f;
    int32_t zero_point = 0;

    if (type == 1) {  // float32
        n_elements = size / 4;
    } else if (type == 9 || type == 3) {  // int8 or uint8
        sentai_tpu_output_quant(tpu_idx, &scale, &zero_point);
        n_elements = size;
    } else {
        return mp_obj_new_int(-6);  // Unsupported type
    }

    if (n_elements != g_kmeans_dim) {
        return mp_obj_new_int(-4);  // Dimension mismatch
    }

    // Dequantize to temp buffer
    if (type == 1) {
        memcpy(g_kmeans_temp, data, n_elements * sizeof(float));
    } else if (type == 9) {  // int8
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            g_kmeans_temp[i] = scale * ((float)idata[i] - (float)zero_point);
        }
    } else {  // uint8
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            g_kmeans_temp[i] = scale * ((float)udata[i] - (float)zero_point);
        }
    }

    return mp_obj_new_int(kmeans_add(cluster, g_kmeans_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_kmeans_add_from_tpu_obj, mod_kmeans_add_from_tpu);

// sentai.kmeans.compute() -> int
// Compute centroids from added vectors
static mp_obj_t mod_kmeans_compute(void) {
    return mp_obj_new_int(kmeans_compute());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_kmeans_compute_obj, mod_kmeans_compute);

// sentai.kmeans.fit(max_iter=100) -> int
// Run Lloyd's algorithm on stored vectors
static mp_obj_t mod_kmeans_fit(size_t n_args, const mp_obj_t *args) {
    int max_iter = 100;
    if (n_args >= 1) {
        max_iter = mp_obj_get_int(args[0]);
    }
    return mp_obj_new_int(kmeans_fit(max_iter));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_kmeans_fit_obj, 0, 1, mod_kmeans_fit);

// sentai.kmeans.predict(vector) -> int
// Predict cluster for a vector
static mp_obj_t mod_kmeans_predict(mp_obj_t vec_obj) {
    if (!g_kmeans_initialized) return mp_obj_new_int(-1);

    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_kmeans_dim) {
        return mp_obj_new_int(-4);
    }

    for (size_t i = 0; i < list->len; i++) {
        g_kmeans_temp[i] = mp_obj_get_float(list->items[i]);
    }

    return mp_obj_new_int(kmeans_predict(g_kmeans_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_predict_obj, mod_kmeans_predict);

// sentai.kmeans.from_tpu(idx) -> int
// Predict cluster using TPU output tensor
static mp_obj_t mod_kmeans_from_tpu(mp_obj_t idx_obj) {
    if (!g_kmeans_initialized) return mp_obj_new_int(-1);

    int tpu_idx = mp_obj_get_int(idx_obj);

    // Get TPU output (same dequantization as add_from_tpu)
    extern int sentai_tpu_get_output_size(int idx);
    extern const void* sentai_tpu_get_output_data(int idx);
    extern int sentai_tpu_get_output_type(int idx);
    extern int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);

    int size = sentai_tpu_get_output_size(tpu_idx);
    int type = sentai_tpu_get_output_type(tpu_idx);
    const void* data = sentai_tpu_get_output_data(tpu_idx);

    if (!data || size <= 0) return mp_obj_new_int(-5);

    int n_elements = 0;
    float scale = 1.0f;
    int32_t zero_point = 0;

    if (type == 1) {
        n_elements = size / 4;
    } else if (type == 9 || type == 3) {
        sentai_tpu_output_quant(tpu_idx, &scale, &zero_point);
        n_elements = size;
    } else {
        return mp_obj_new_int(-6);
    }

    if (n_elements != g_kmeans_dim) {
        return mp_obj_new_int(-4);
    }

    if (type == 1) {
        memcpy(g_kmeans_temp, data, n_elements * sizeof(float));
    } else if (type == 9) {
        const int8_t* idata = (const int8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            g_kmeans_temp[i] = scale * ((float)idata[i] - (float)zero_point);
        }
    } else {
        const uint8_t* udata = (const uint8_t*)data;
        for (int i = 0; i < n_elements; i++) {
            g_kmeans_temp[i] = scale * ((float)udata[i] - (float)zero_point);
        }
    }

    return mp_obj_new_int(kmeans_predict(g_kmeans_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_from_tpu_obj, mod_kmeans_from_tpu);

// sentai.kmeans.distances(vector) -> list
// Get distance to each centroid
static mp_obj_t mod_kmeans_distances(mp_obj_t vec_obj) {
    if (!g_kmeans_initialized) return mp_const_none;

    mp_obj_list_t* list = MP_OBJ_TO_PTR(vec_obj);
    if (list->len != (size_t)g_kmeans_dim) {
        return mp_const_none;
    }

    for (size_t i = 0; i < list->len; i++) {
        g_kmeans_temp[i] = mp_obj_get_float(list->items[i]);
    }

    float* dists = (float*)malloc(g_kmeans_k * sizeof(float));
    if (!dists) return mp_const_none;

    kmeans_distances(g_kmeans_temp, dists);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(g_kmeans_k, NULL));
    for (int i = 0; i < g_kmeans_k; i++) {
        result->items[i] = mp_obj_new_float(dists[i]);
    }
    free(dists);

    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_distances_obj, mod_kmeans_distances);

// sentai.kmeans.centroid(idx) -> list
// Get centroid vector
static mp_obj_t mod_kmeans_centroid(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    const float* c = kmeans_get_centroid(idx);
    if (!c) return mp_const_none;

    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(g_kmeans_dim, NULL));
    for (int i = 0; i < g_kmeans_dim; i++) {
        list->items[i] = mp_obj_new_float(c[i]);
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_centroid_obj, mod_kmeans_centroid);

// sentai.kmeans.save(path) -> int
static mp_obj_t mod_kmeans_save(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(kmeans_save(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_save_obj, mod_kmeans_save);

// sentai.kmeans.load(path) -> int
static mp_obj_t mod_kmeans_load(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(kmeans_load(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_kmeans_load_obj, mod_kmeans_load);

// sentai.kmeans.clear() -> None
// Clear training vectors (keep centroids)
static mp_obj_t mod_kmeans_clear(void) {
    kmeans_clear_vectors();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_kmeans_clear_obj, mod_kmeans_clear);

// sentai.kmeans.info() -> dict
static mp_obj_t mod_kmeans_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(5));
    
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_k), 
                      mp_obj_new_int(g_kmeans_k));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dim), 
                      mp_obj_new_int(g_kmeans_dim));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_vectors), 
                      mp_obj_new_int(g_kmeans_n_vectors));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), 
                      mp_obj_new_bool(g_kmeans_initialized));
    
    // Per-cluster counts
    if (g_kmeans_initialized && g_kmeans_cluster_counts) {
        mp_obj_list_t* counts = MP_OBJ_TO_PTR(mp_obj_new_list(g_kmeans_k, NULL));
        for (int i = 0; i < g_kmeans_k; i++) {
            counts->items[i] = mp_obj_new_int(g_kmeans_cluster_counts[i]);
        }
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_counts), MP_OBJ_FROM_PTR(counts));
    }

    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_kmeans_info_obj, mod_kmeans_info);

// ---- module table ----
static const mp_rom_map_elem_t sentai_kmeans_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_kmeans) },
    // Initialization
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_kmeans_init_obj) },
    // Training (manual labeling)
    { MP_ROM_QSTR(MP_QSTR_add), MP_ROM_PTR(&mod_kmeans_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_from_tpu), MP_ROM_PTR(&mod_kmeans_add_from_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_compute), MP_ROM_PTR(&mod_kmeans_compute_obj) },
    // Training (auto clustering)
    { MP_ROM_QSTR(MP_QSTR_fit), MP_ROM_PTR(&mod_kmeans_fit_obj) },
    // Inference
    { MP_ROM_QSTR(MP_QSTR_predict), MP_ROM_PTR(&mod_kmeans_predict_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_tpu), MP_ROM_PTR(&mod_kmeans_from_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_distances), MP_ROM_PTR(&mod_kmeans_distances_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_kmeans_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_kmeans_load_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_centroid), MP_ROM_PTR(&mod_kmeans_centroid_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_kmeans_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&mod_kmeans_clear_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_kmeans_globals, sentai_kmeans_globals_table);
static const mp_obj_module_t sentai_kmeans_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_kmeans_globals,
};
