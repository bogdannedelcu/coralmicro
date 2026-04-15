// ============== sentai.dtw — Dynamic Time Warping Template Matching ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// DTW for gesture recognition, audio pattern matching, and sequence classification.
// Record templates from IMU or mic, then match live streams in real time.
// Zero training needed — just record a few examples per gesture.
//
// Uses full DTW matrix with Sakoe-Chiba band for O(T*W) instead of O(T^2).

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== DTW State =====================

#define DTW_MAX_DIM         32
#define DTW_MAX_LEN         200
#define DTW_MAX_TEMPLATES   16
#define DTW_LABEL_LEN       16

// Template storage
typedef struct {
    char label[DTW_LABEL_LEN];
    float* data;        // len * dim floats
    int len;
    int active;
} dtw_template_t;

static int g_dtw_dim = 0;
static int g_dtw_max_len = 0;
static int g_dtw_initialized = 0;

static dtw_template_t g_dtw_templates[DTW_MAX_TEMPLATES];
static int g_dtw_n_templates = 0;

// Recording state
static float* g_dtw_rec_buf = NULL;     // max_len * dim floats
static int g_dtw_rec_len = 0;
static int g_dtw_recording = 0;
static char g_dtw_rec_label[DTW_LABEL_LEN];

// DTW cost matrix (reused for each comparison)
static float* g_dtw_cost = NULL;        // max_len * max_len floats

// Temp buffer for live input
static float* g_dtw_temp = NULL;        // dim floats

// ===================== Internal Functions =====================

static float dtw_dist_sq(const float* a, const float* b, int dim) {
    float sum = 0.0f;
    for (int i = 0; i < dim; i++) {
        float d = a[i] - b[i];
        sum += d * d;
    }
    return sum;
}

// Compute DTW distance between two sequences
// Uses Sakoe-Chiba band to reduce computation
static float dtw_compute(const float* seq_a, int len_a,
                          const float* seq_b, int len_b,
                          int dim, float* cost, int max_len) {
    // Band width: max_len/4 or at least 10
    int band = max_len / 4;
    if (band < 10) band = 10;

    float INF = 1e30f;

    // Initialize cost matrix to infinity
    for (int i = 0; i < len_a * len_b; i++) cost[i] = INF;

    // Fill cost matrix with Sakoe-Chiba band
    for (int i = 0; i < len_a; i++) {
        int j_start = i - band;
        if (j_start < 0) j_start = 0;
        int j_end = i + band + 1;
        if (j_end > len_b) j_end = len_b;

        for (int j = j_start; j < j_end; j++) {
            float d = dtw_dist_sq(&seq_a[i * dim], &seq_b[j * dim], dim);

            float prev = INF;
            if (i == 0 && j == 0) {
                prev = 0.0f;
            } else {
                if (i > 0 && cost[(i-1) * len_b + j] < prev)
                    prev = cost[(i-1) * len_b + j];
                if (j > 0 && cost[i * len_b + (j-1)] < prev)
                    prev = cost[i * len_b + (j-1)];
                if (i > 0 && j > 0 && cost[(i-1) * len_b + (j-1)] < prev)
                    prev = cost[(i-1) * len_b + (j-1)];
            }

            cost[i * len_b + j] = d + prev;
        }
    }

    float result = cost[(len_a - 1) * len_b + (len_b - 1)];
    return sqrtf(result > 0.0f ? result : 0.0f);
}

// ===================== C API =====================

static int dtw_init(int dim, int max_len) {
    if (dim < 1 || dim > DTW_MAX_DIM) return -1;
    if (max_len < 2 || max_len > DTW_MAX_LEN) return -2;

    if (g_dtw_rec_buf) { free(g_dtw_rec_buf); g_dtw_rec_buf = NULL; }
    if (g_dtw_cost) { free(g_dtw_cost); g_dtw_cost = NULL; }
    if (g_dtw_temp) { free(g_dtw_temp); g_dtw_temp = NULL; }

    // Free existing template data
    for (int i = 0; i < DTW_MAX_TEMPLATES; i++) {
        if (g_dtw_templates[i].data) {
            free(g_dtw_templates[i].data);
            g_dtw_templates[i].data = NULL;
        }
        g_dtw_templates[i].active = 0;
        g_dtw_templates[i].len = 0;
    }

    g_dtw_rec_buf = (float*)malloc(max_len * dim * sizeof(float));
    g_dtw_cost = (float*)malloc(max_len * max_len * sizeof(float));
    g_dtw_temp = (float*)malloc(dim * sizeof(float));

    if (!g_dtw_rec_buf || !g_dtw_cost || !g_dtw_temp) {
        if (g_dtw_rec_buf) free(g_dtw_rec_buf);
        if (g_dtw_cost) free(g_dtw_cost);
        if (g_dtw_temp) free(g_dtw_temp);
        g_dtw_rec_buf = g_dtw_cost = g_dtw_temp = NULL;
        g_dtw_initialized = 0;
        return -3;
    }

    g_dtw_dim = dim;
    g_dtw_max_len = max_len;
    g_dtw_n_templates = 0;
    g_dtw_recording = 0;
    g_dtw_rec_len = 0;
    g_dtw_initialized = 1;
    return 0;
}

static int dtw_record_start(const char* label) {
    if (!g_dtw_initialized) return -1;
    if (g_dtw_recording) return -2;

    strncpy(g_dtw_rec_label, label, DTW_LABEL_LEN - 1);
    g_dtw_rec_label[DTW_LABEL_LEN - 1] = '\0';
    g_dtw_rec_len = 0;
    g_dtw_recording = 1;
    return 0;
}

static int dtw_record_add(const float* frame) {
    if (!g_dtw_recording) return -1;
    if (g_dtw_rec_len >= g_dtw_max_len) return -2;

    memcpy(&g_dtw_rec_buf[g_dtw_rec_len * g_dtw_dim], frame, g_dtw_dim * sizeof(float));
    g_dtw_rec_len++;
    return 0;
}

static int dtw_record_end(void) {
    if (!g_dtw_recording) return -1;
    if (g_dtw_rec_len < 2) { g_dtw_recording = 0; return -2; }

    // Find free slot
    int slot = -1;
    for (int i = 0; i < DTW_MAX_TEMPLATES; i++) {
        if (!g_dtw_templates[i].active) { slot = i; break; }
    }
    if (slot < 0) { g_dtw_recording = 0; return -3; }  // full

    g_dtw_templates[slot].data = (float*)malloc(g_dtw_rec_len * g_dtw_dim * sizeof(float));
    if (!g_dtw_templates[slot].data) { g_dtw_recording = 0; return -4; }

    memcpy(g_dtw_templates[slot].data, g_dtw_rec_buf,
           g_dtw_rec_len * g_dtw_dim * sizeof(float));
    strncpy(g_dtw_templates[slot].label, g_dtw_rec_label, DTW_LABEL_LEN);
    g_dtw_templates[slot].len = g_dtw_rec_len;
    g_dtw_templates[slot].active = 1;
    g_dtw_n_templates++;

    g_dtw_recording = 0;
    return slot;
}

// Match a sequence against all templates
// Returns best template index, fills best_dist. Returns -1 if no templates.
static int dtw_match_seq(const float* seq, int len, float* best_dist) {
    if (!g_dtw_initialized || g_dtw_n_templates == 0) return -1;
    if (len < 2) return -2;

    int best_idx = -1;
    float best = 1e30f;

    for (int t = 0; t < DTW_MAX_TEMPLATES; t++) {
        if (!g_dtw_templates[t].active) continue;

        float dist = dtw_compute(seq, len,
                                  g_dtw_templates[t].data, g_dtw_templates[t].len,
                                  g_dtw_dim, g_dtw_cost, g_dtw_max_len);
        if (dist < best) {
            best = dist;
            best_idx = t;
        }
    }

    if (best_dist) *best_dist = best;
    return best_idx;
}

static int dtw_remove(const char* label) {
    for (int i = 0; i < DTW_MAX_TEMPLATES; i++) {
        if (g_dtw_templates[i].active && strcmp(g_dtw_templates[i].label, label) == 0) {
            free(g_dtw_templates[i].data);
            g_dtw_templates[i].data = NULL;
            g_dtw_templates[i].active = 0;
            g_dtw_n_templates--;
            return 0;
        }
    }
    return -1;
}

static int dtw_save(const char* path) {
    if (!g_dtw_initialized || g_dtw_n_templates == 0) return -1;

    // Calculate total size
    // Header: [dim:i32][max_len:i32][n_templates:i32]
    // Per template: [label:16bytes][len:i32][data:len*dim*f32]
    size_t size = 12;
    for (int t = 0; t < DTW_MAX_TEMPLATES; t++) {
        if (!g_dtw_templates[t].active) continue;
        size += DTW_LABEL_LEN + 4 + g_dtw_templates[t].len * g_dtw_dim * sizeof(float);
    }

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int32_t* header = (int32_t*)buf;
    header[0] = g_dtw_dim;
    header[1] = g_dtw_max_len;
    header[2] = g_dtw_n_templates;

    size_t off = 12;
    for (int t = 0; t < DTW_MAX_TEMPLATES; t++) {
        if (!g_dtw_templates[t].active) continue;
        memcpy(&buf[off], g_dtw_templates[t].label, DTW_LABEL_LEN); off += DTW_LABEL_LEN;
        int32_t len = g_dtw_templates[t].len;
        memcpy(&buf[off], &len, 4); off += 4;
        memcpy(&buf[off], g_dtw_templates[t].data, len * g_dtw_dim * sizeof(float));
        off += len * g_dtw_dim * sizeof(float);
    }

    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)size);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

static int dtw_load(const char* path) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 12) return -1;

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int read = sentai_fs_read(path, buf, size);
    if (read != size) { free(buf); return -3; }

    int32_t* header = (int32_t*)buf;
    int dim = header[0], max_len = header[1], n_tmpl = header[2];

    int rc = dtw_init(dim, max_len);
    if (rc < 0) { free(buf); return rc; }

    size_t off = 12;
    for (int t = 0; t < n_tmpl; t++) {
        if (off + DTW_LABEL_LEN + 4 > (size_t)size) { free(buf); return -4; }
        char label[DTW_LABEL_LEN];
        memcpy(label, &buf[off], DTW_LABEL_LEN); off += DTW_LABEL_LEN;
        int32_t len;
        memcpy(&len, &buf[off], 4); off += 4;

        size_t data_size = len * dim * sizeof(float);
        if (off + data_size > (size_t)size) { free(buf); return -4; }

        // Find slot and store
        int slot = -1;
        for (int i = 0; i < DTW_MAX_TEMPLATES; i++) {
            if (!g_dtw_templates[i].active) { slot = i; break; }
        }
        if (slot < 0) { free(buf); return -5; }

        g_dtw_templates[slot].data = (float*)malloc(data_size);
        if (!g_dtw_templates[slot].data) { free(buf); return -6; }

        memcpy(g_dtw_templates[slot].data, &buf[off], data_size); off += data_size;
        memcpy(g_dtw_templates[slot].label, label, DTW_LABEL_LEN);
        g_dtw_templates[slot].len = len;
        g_dtw_templates[slot].active = 1;
        g_dtw_n_templates++;
    }

    free(buf);
    return 0;
}

// ===================== MicroPython Bindings =====================

// sentai.dtw.init(dim, max_len) -> int
static mp_obj_t mod_dtw_init(mp_obj_t dim_obj, mp_obj_t len_obj) {
    return mp_obj_new_int(dtw_init(mp_obj_get_int(dim_obj), mp_obj_get_int(len_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_dtw_init_obj, mod_dtw_init);

// sentai.dtw.record_start(label) -> int
static mp_obj_t mod_dtw_record_start(mp_obj_t label_obj) {
    return mp_obj_new_int(dtw_record_start(mp_obj_str_get_str(label_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_dtw_record_start_obj, mod_dtw_record_start);

// sentai.dtw.record_add(frame) -> int
static mp_obj_t mod_dtw_record_add(mp_obj_t frame_obj) {
    if (!g_dtw_recording) return mp_obj_new_int(-1);
    mp_obj_list_t* list = MP_OBJ_TO_PTR(frame_obj);
    if (list->len != (size_t)g_dtw_dim) return mp_obj_new_int(-4);
    for (size_t i = 0; i < list->len; i++)
        g_dtw_temp[i] = mp_obj_get_float(list->items[i]);
    return mp_obj_new_int(dtw_record_add(g_dtw_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_dtw_record_add_obj, mod_dtw_record_add);

// sentai.dtw.record_add_imu() -> int (add current IMU reading as frame)
static mp_obj_t mod_dtw_record_add_imu(void) {
    if (!g_dtw_recording) return mp_obj_new_int(-1);
    if (g_dtw_dim < 3) return mp_obj_new_int(-4);

    extern int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c);
    float x, y, z, temp;
    if (sentai_imu_read_accel(&x, &y, &z, &temp) < 0) return mp_obj_new_int(-5);

    g_dtw_temp[0] = x;
    g_dtw_temp[1] = y;
    g_dtw_temp[2] = z;
    // Fill remaining dims with 0 if dim > 3
    for (int i = 3; i < g_dtw_dim; i++) g_dtw_temp[i] = 0.0f;

    return mp_obj_new_int(dtw_record_add(g_dtw_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_record_add_imu_obj, mod_dtw_record_add_imu);

// sentai.dtw.record_add_mic() -> int (add current mic level as 1D frame)
static mp_obj_t mod_dtw_record_add_mic(void) {
    if (!g_dtw_recording) return mp_obj_new_int(-1);

    extern int sentai_mic_level(void);
    int level = sentai_mic_level();
    g_dtw_temp[0] = (float)level;
    for (int i = 1; i < g_dtw_dim; i++) g_dtw_temp[i] = 0.0f;

    return mp_obj_new_int(dtw_record_add(g_dtw_temp));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_record_add_mic_obj, mod_dtw_record_add_mic);

// sentai.dtw.record_end() -> int (template slot index)
static mp_obj_t mod_dtw_record_end(void) {
    return mp_obj_new_int(dtw_record_end());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_record_end_obj, mod_dtw_record_end);

// sentai.dtw.match(sequence, threshold=inf) -> tuple(label, distance) or None
static mp_obj_t mod_dtw_match(size_t n_args, const mp_obj_t *args) {
    if (!g_dtw_initialized || g_dtw_n_templates == 0) return mp_const_none;

    // Parse sequence: list of lists
    mp_obj_list_t* seq_list = MP_OBJ_TO_PTR(args[0]);
    int seq_len = seq_list->len;
    if (seq_len < 2 || seq_len > g_dtw_max_len) return mp_const_none;

    float* seq = (float*)malloc(seq_len * g_dtw_dim * sizeof(float));
    if (!seq) return mp_const_none;

    for (int i = 0; i < seq_len; i++) {
        mp_obj_list_t* frame = MP_OBJ_TO_PTR(seq_list->items[i]);
        if (frame->len != (size_t)g_dtw_dim) { free(seq); return mp_const_none; }
        for (int d = 0; d < g_dtw_dim; d++)
            seq[i * g_dtw_dim + d] = mp_obj_get_float(frame->items[d]);
    }

    float threshold = 1e30f;
    if (n_args >= 2) threshold = mp_obj_get_float(args[1]);

    float best_dist;
    int best_idx = dtw_match_seq(seq, seq_len, &best_dist);
    free(seq);

    if (best_idx < 0 || best_dist > threshold) return mp_const_none;

    mp_obj_t items[2] = {
        mp_obj_new_str(g_dtw_templates[best_idx].label,
                       strlen(g_dtw_templates[best_idx].label)),
        mp_obj_new_float(best_dist)
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_dtw_match_obj, 1, 2, mod_dtw_match);

// sentai.dtw.match_imu(n_frames, threshold=inf, delay_ms=20) -> tuple or None
// Capture n_frames from IMU and match against templates
static mp_obj_t mod_dtw_match_imu(size_t n_args, const mp_obj_t *args) {
    if (!g_dtw_initialized || g_dtw_n_templates == 0) return mp_const_none;
    if (g_dtw_dim < 3) return mp_const_none;

    int n_frames = mp_obj_get_int(args[0]);
    if (n_frames < 2 || n_frames > g_dtw_max_len) return mp_const_none;
    float threshold = (n_args >= 2) ? mp_obj_get_float(args[1]) : 1e30f;
    int delay_ms = (n_args >= 3) ? mp_obj_get_int(args[2]) : 20;

    extern int sentai_imu_read_accel(float* x, float* y, float* z, float* t);
    extern void sentai_sleep_ms(uint32_t ms);

    // Capture directly into rec_buf (reuse)
    for (int i = 0; i < n_frames; i++) {
        float x, y, z, temp;
        if (sentai_imu_read_accel(&x, &y, &z, &temp) < 0) return mp_const_none;
        g_dtw_rec_buf[i * g_dtw_dim + 0] = x;
        g_dtw_rec_buf[i * g_dtw_dim + 1] = y;
        g_dtw_rec_buf[i * g_dtw_dim + 2] = z;
        for (int d = 3; d < g_dtw_dim; d++) g_dtw_rec_buf[i * g_dtw_dim + d] = 0.0f;
        if (delay_ms > 0 && i < n_frames - 1) sentai_sleep_ms(delay_ms);
    }

    float best_dist;
    int best_idx = dtw_match_seq(g_dtw_rec_buf, n_frames, &best_dist);

    if (best_idx < 0 || best_dist > threshold) return mp_const_none;

    mp_obj_t items[2] = {
        mp_obj_new_str(g_dtw_templates[best_idx].label,
                       strlen(g_dtw_templates[best_idx].label)),
        mp_obj_new_float(best_dist)
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_dtw_match_imu_obj, 1, 3, mod_dtw_match_imu);

// sentai.dtw.match_mic(n_frames, threshold=inf, delay_ms=20) -> tuple or None
static mp_obj_t mod_dtw_match_mic(size_t n_args, const mp_obj_t *args) {
    if (!g_dtw_initialized || g_dtw_n_templates == 0) return mp_const_none;

    int n_frames = mp_obj_get_int(args[0]);
    if (n_frames < 2 || n_frames > g_dtw_max_len) return mp_const_none;
    float threshold = (n_args >= 2) ? mp_obj_get_float(args[1]) : 1e30f;
    int delay_ms = (n_args >= 3) ? mp_obj_get_int(args[2]) : 20;

    extern int sentai_mic_level(void);
    extern void sentai_sleep_ms(uint32_t ms);

    for (int i = 0; i < n_frames; i++) {
        g_dtw_rec_buf[i * g_dtw_dim] = (float)sentai_mic_level();
        for (int d = 1; d < g_dtw_dim; d++) g_dtw_rec_buf[i * g_dtw_dim + d] = 0.0f;
        if (delay_ms > 0 && i < n_frames - 1) sentai_sleep_ms(delay_ms);
    }

    float best_dist;
    int best_idx = dtw_match_seq(g_dtw_rec_buf, n_frames, &best_dist);

    if (best_idx < 0 || best_dist > threshold) return mp_const_none;

    mp_obj_t items[2] = {
        mp_obj_new_str(g_dtw_templates[best_idx].label,
                       strlen(g_dtw_templates[best_idx].label)),
        mp_obj_new_float(best_dist)
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_dtw_match_mic_obj, 1, 3, mod_dtw_match_mic);

// sentai.dtw.templates() -> list of dicts
static mp_obj_t mod_dtw_templates(void) {
    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int t = 0; t < DTW_MAX_TEMPLATES; t++) {
        if (!g_dtw_templates[t].active) continue;
        mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(3));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_label),
            mp_obj_new_str(g_dtw_templates[t].label, strlen(g_dtw_templates[t].label)));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_len),
            mp_obj_new_int(g_dtw_templates[t].len));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_slot),
            mp_obj_new_int(t));
        mp_obj_list_append(MP_OBJ_FROM_PTR(list), MP_OBJ_FROM_PTR(d));
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_templates_obj, mod_dtw_templates);

// sentai.dtw.remove(label) -> int
static mp_obj_t mod_dtw_remove(mp_obj_t label_obj) {
    return mp_obj_new_int(dtw_remove(mp_obj_str_get_str(label_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_dtw_remove_obj, mod_dtw_remove);

// sentai.dtw.save(path) -> int
static mp_obj_t mod_dtw_save(mp_obj_t path_obj) {
    return mp_obj_new_int(dtw_save(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_dtw_save_obj, mod_dtw_save);

// sentai.dtw.load(path) -> int
static mp_obj_t mod_dtw_load(mp_obj_t path_obj) {
    return mp_obj_new_int(dtw_load(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_dtw_load_obj, mod_dtw_load);

// sentai.dtw.info() -> dict
static mp_obj_t mod_dtw_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(5));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dim), mp_obj_new_int(g_dtw_dim));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_max_len), mp_obj_new_int(g_dtw_max_len));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_templates), mp_obj_new_int(g_dtw_n_templates));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_recording), mp_obj_new_bool(g_dtw_recording));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), mp_obj_new_bool(g_dtw_initialized));
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_info_obj, mod_dtw_info);

// sentai.dtw.clear() -> None
static mp_obj_t mod_dtw_clear(void) {
    for (int i = 0; i < DTW_MAX_TEMPLATES; i++) {
        if (g_dtw_templates[i].data) {
            free(g_dtw_templates[i].data);
            g_dtw_templates[i].data = NULL;
        }
        g_dtw_templates[i].active = 0;
        g_dtw_templates[i].len = 0;
    }
    g_dtw_n_templates = 0;
    g_dtw_recording = 0;
    g_dtw_rec_len = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_dtw_clear_obj, mod_dtw_clear);

// ---- module table ----
static const mp_rom_map_elem_t sentai_dtw_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_dtw) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_dtw_init_obj) },
    // Recording
    { MP_ROM_QSTR(MP_QSTR_record_start), MP_ROM_PTR(&mod_dtw_record_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_record_add), MP_ROM_PTR(&mod_dtw_record_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_record_add_imu), MP_ROM_PTR(&mod_dtw_record_add_imu_obj) },
    { MP_ROM_QSTR(MP_QSTR_record_add_mic), MP_ROM_PTR(&mod_dtw_record_add_mic_obj) },
    { MP_ROM_QSTR(MP_QSTR_record_end), MP_ROM_PTR(&mod_dtw_record_end_obj) },
    // Matching
    { MP_ROM_QSTR(MP_QSTR_match), MP_ROM_PTR(&mod_dtw_match_obj) },
    { MP_ROM_QSTR(MP_QSTR_match_imu), MP_ROM_PTR(&mod_dtw_match_imu_obj) },
    { MP_ROM_QSTR(MP_QSTR_match_mic), MP_ROM_PTR(&mod_dtw_match_mic_obj) },
    // Template management
    { MP_ROM_QSTR(MP_QSTR_templates), MP_ROM_PTR(&mod_dtw_templates_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove), MP_ROM_PTR(&mod_dtw_remove_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_dtw_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_dtw_load_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_dtw_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&mod_dtw_clear_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_dtw_globals, sentai_dtw_globals_table);
static const mp_obj_module_t sentai_dtw_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_dtw_globals,
};
