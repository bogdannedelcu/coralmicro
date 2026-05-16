// ============== sentai.hmm — Hidden Markov Model ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Discrete-observation HMM with Baum-Welch training and Viterbi decoding.
// Useful for sequential pattern recognition: IMU gestures, audio events,
// drone flight phases, and any discrete state-transition system.
//
// Observations are discrete integers [0, n_obs-1].
// For continuous data, first discretize (e.g., PCA→quantize or bin accelerometer).

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== HMM State =====================

#define HMM_MAX_STATES      32
#define HMM_MAX_OBS         64
#define HMM_MAX_SEQ_LEN     500
#define HMM_MAX_SEQS        100
#define HMM_LOG_ZERO        (-1e30f)

static int g_hmm_n_states = 0;
static int g_hmm_n_obs = 0;
static int g_hmm_initialized = 0;

// Model parameters (all in log-space for numerical stability)
static float* g_hmm_log_trans = NULL;    // n_states * n_states (log transition probs)
static float* g_hmm_log_emit = NULL;     // n_states * n_obs (log emission probs)
static float* g_hmm_log_prior = NULL;    // n_states (log initial state probs)

// Training sequences
static int* g_hmm_seqs = NULL;           // MAX_SEQS * MAX_SEQ_LEN observation indices
static int g_hmm_seq_lens[HMM_MAX_SEQS];
static int g_hmm_n_seqs = 0;

// Work buffers for forward-backward / Viterbi
static float* g_hmm_alpha = NULL;        // MAX_SEQ_LEN * n_states
static float* g_hmm_beta = NULL;         // MAX_SEQ_LEN * n_states
static float* g_hmm_gamma = NULL;        // MAX_SEQ_LEN * n_states
static float* g_hmm_xi_sum = NULL;       // n_states * n_states

// ===================== Internal Functions =====================

// Log-sum-exp of two values: log(exp(a) + exp(b))
static float hmm_logadd(float a, float b) {
    if (a == HMM_LOG_ZERO) return b;
    if (b == HMM_LOG_ZERO) return a;
    float mx = (a > b) ? a : b;
    return mx + logf(expf(a - mx) + expf(b - mx));
}

// Forward algorithm (alpha pass)
// alpha[t][i] = log P(o_1..o_t, s_t=i | model)
static float hmm_forward(const int* obs, int T) {
    int N = g_hmm_n_states;

    // t = 0
    for (int i = 0; i < N; i++) {
        g_hmm_alpha[0 * N + i] = g_hmm_log_prior[i] + g_hmm_log_emit[i * g_hmm_n_obs + obs[0]];
    }

    // t = 1..T-1
    for (int t = 1; t < T; t++) {
        for (int j = 0; j < N; j++) {
            float sum = HMM_LOG_ZERO;
            for (int i = 0; i < N; i++) {
                sum = hmm_logadd(sum, g_hmm_alpha[(t-1) * N + i] + g_hmm_log_trans[i * N + j]);
            }
            g_hmm_alpha[t * N + j] = sum + g_hmm_log_emit[j * g_hmm_n_obs + obs[t]];
        }
    }

    // Total log-likelihood
    float ll = HMM_LOG_ZERO;
    for (int i = 0; i < N; i++) {
        ll = hmm_logadd(ll, g_hmm_alpha[(T-1) * N + i]);
    }
    return ll;
}

// Backward algorithm (beta pass)
static void hmm_backward(const int* obs, int T) {
    int N = g_hmm_n_states;

    // t = T-1
    for (int i = 0; i < N; i++) {
        g_hmm_beta[(T-1) * N + i] = 0.0f;  // log(1)
    }

    // t = T-2..0
    for (int t = T - 2; t >= 0; t--) {
        for (int i = 0; i < N; i++) {
            float sum = HMM_LOG_ZERO;
            for (int j = 0; j < N; j++) {
                sum = hmm_logadd(sum,
                    g_hmm_log_trans[i * N + j] +
                    g_hmm_log_emit[j * g_hmm_n_obs + obs[t+1]] +
                    g_hmm_beta[(t+1) * N + j]);
            }
            g_hmm_beta[t * N + i] = sum;
        }
    }
}

// Viterbi decoding
// path: output array of T ints (most likely state sequence)
static float hmm_viterbi(const int* obs, int T, int* path) {
    int N = g_hmm_n_states;

    // Reuse alpha as viterbi scores, gamma as backpointers (cast to int)
    float* V = g_hmm_alpha;
    int* bp = (int*)g_hmm_gamma;  // safe: both are MAX_SEQ_LEN * n_states

    // t = 0
    for (int i = 0; i < N; i++) {
        V[0 * N + i] = g_hmm_log_prior[i] + g_hmm_log_emit[i * g_hmm_n_obs + obs[0]];
        bp[0 * N + i] = 0;
    }

    // t = 1..T-1
    for (int t = 1; t < T; t++) {
        for (int j = 0; j < N; j++) {
            float best = HMM_LOG_ZERO;
            int best_i = 0;
            for (int i = 0; i < N; i++) {
                float score = V[(t-1) * N + i] + g_hmm_log_trans[i * N + j];
                if (score > best) {
                    best = score;
                    best_i = i;
                }
            }
            V[t * N + j] = best + g_hmm_log_emit[j * g_hmm_n_obs + obs[t]];
            bp[t * N + j] = best_i;
        }
    }

    // Backtrack
    float best_score = HMM_LOG_ZERO;
    path[T-1] = 0;
    for (int i = 0; i < N; i++) {
        if (V[(T-1) * N + i] > best_score) {
            best_score = V[(T-1) * N + i];
            path[T-1] = i;
        }
    }
    for (int t = T - 2; t >= 0; t--) {
        path[t] = bp[(t+1) * N + path[t+1]];
    }

    return best_score;
}

// ===================== C API =====================

static int hmm_init(int n_states, int n_obs) {
    if (n_states < 1 || n_states > HMM_MAX_STATES) return -1;
    if (n_obs < 1 || n_obs > HMM_MAX_OBS) return -2;

    if (g_hmm_log_trans) { free(g_hmm_log_trans); g_hmm_log_trans = NULL; }
    if (g_hmm_log_emit) { free(g_hmm_log_emit); g_hmm_log_emit = NULL; }
    if (g_hmm_log_prior) { free(g_hmm_log_prior); g_hmm_log_prior = NULL; }
    if (g_hmm_seqs) { free(g_hmm_seqs); g_hmm_seqs = NULL; }
    if (g_hmm_alpha) { free(g_hmm_alpha); g_hmm_alpha = NULL; }
    if (g_hmm_beta) { free(g_hmm_beta); g_hmm_beta = NULL; }
    if (g_hmm_gamma) { free(g_hmm_gamma); g_hmm_gamma = NULL; }
    if (g_hmm_xi_sum) { free(g_hmm_xi_sum); g_hmm_xi_sum = NULL; }

    g_hmm_log_trans = (float*)malloc(n_states * n_states * sizeof(float));
    g_hmm_log_emit = (float*)malloc(n_states * n_obs * sizeof(float));
    g_hmm_log_prior = (float*)malloc(n_states * sizeof(float));
    g_hmm_seqs = (int*)malloc(HMM_MAX_SEQS * HMM_MAX_SEQ_LEN * sizeof(int));
    g_hmm_alpha = (float*)malloc(HMM_MAX_SEQ_LEN * n_states * sizeof(float));
    g_hmm_beta = (float*)malloc(HMM_MAX_SEQ_LEN * n_states * sizeof(float));
    g_hmm_gamma = (float*)malloc(HMM_MAX_SEQ_LEN * n_states * sizeof(float));
    g_hmm_xi_sum = (float*)malloc(n_states * n_states * sizeof(float));

    if (!g_hmm_log_trans || !g_hmm_log_emit || !g_hmm_log_prior ||
        !g_hmm_seqs || !g_hmm_alpha || !g_hmm_beta || !g_hmm_gamma || !g_hmm_xi_sum) {
        if (g_hmm_log_trans) free(g_hmm_log_trans);
        if (g_hmm_log_emit) free(g_hmm_log_emit);
        if (g_hmm_log_prior) free(g_hmm_log_prior);
        if (g_hmm_seqs) free(g_hmm_seqs);
        if (g_hmm_alpha) free(g_hmm_alpha);
        if (g_hmm_beta) free(g_hmm_beta);
        if (g_hmm_gamma) free(g_hmm_gamma);
        if (g_hmm_xi_sum) free(g_hmm_xi_sum);
        g_hmm_log_trans = g_hmm_log_emit = g_hmm_log_prior = NULL;
        g_hmm_alpha = g_hmm_beta = g_hmm_gamma = g_hmm_xi_sum = NULL;
        g_hmm_seqs = NULL;
        g_hmm_initialized = 0;
        return -3;
    }

    // Initialize with uniform distributions
    float log_trans = logf(1.0f / (float)n_states);
    float log_emit = logf(1.0f / (float)n_obs);
    for (int i = 0; i < n_states * n_states; i++) g_hmm_log_trans[i] = log_trans;
    for (int i = 0; i < n_states * n_obs; i++) g_hmm_log_emit[i] = log_emit;
    for (int i = 0; i < n_states; i++) g_hmm_log_prior[i] = log_trans;

    g_hmm_n_states = n_states;
    g_hmm_n_obs = n_obs;
    g_hmm_n_seqs = 0;
    g_hmm_initialized = 1;
    return 0;
}

static int hmm_add_seq(const int* obs, int len) {
    if (!g_hmm_initialized) return -1;
    if (g_hmm_n_seqs >= HMM_MAX_SEQS) return -2;
    if (len < 1 || len > HMM_MAX_SEQ_LEN) return -3;

    // Validate observations
    for (int i = 0; i < len; i++) {
        if (obs[i] < 0 || obs[i] >= g_hmm_n_obs) return -4;
    }

    memcpy(&g_hmm_seqs[g_hmm_n_seqs * HMM_MAX_SEQ_LEN], obs, len * sizeof(int));
    g_hmm_seq_lens[g_hmm_n_seqs] = len;
    g_hmm_n_seqs++;
    return 0;
}

// Baum-Welch training (EM algorithm)
// Returns final total log-likelihood, negative on error
static float hmm_train(int max_iter) {
    if (!g_hmm_initialized) return HMM_LOG_ZERO;
    if (g_hmm_n_seqs == 0) return HMM_LOG_ZERO;
    if (max_iter <= 0) max_iter = 20;

    int N = g_hmm_n_states;
    int M = g_hmm_n_obs;
    float total_ll = HMM_LOG_ZERO;

    // Allocate accumulators
    float* new_log_trans = (float*)malloc(N * N * sizeof(float));
    float* new_log_emit = (float*)malloc(N * M * sizeof(float));
    float* new_log_prior = (float*)malloc(N * sizeof(float));
    float* gamma_sum = (float*)malloc(N * sizeof(float));

    if (!new_log_trans || !new_log_emit || !new_log_prior || !gamma_sum) {
        if (new_log_trans) free(new_log_trans);
        if (new_log_emit) free(new_log_emit);
        if (new_log_prior) free(new_log_prior);
        if (gamma_sum) free(gamma_sum);
        return HMM_LOG_ZERO;
    }

    for (int iter = 0; iter < max_iter; iter++) {
        // Initialize accumulators
        for (int i = 0; i < N * N; i++) new_log_trans[i] = HMM_LOG_ZERO;
        for (int i = 0; i < N * M; i++) new_log_emit[i] = HMM_LOG_ZERO;
        for (int i = 0; i < N; i++) new_log_prior[i] = HMM_LOG_ZERO;
        for (int i = 0; i < N; i++) gamma_sum[i] = HMM_LOG_ZERO;
        for (int i = 0; i < N * N; i++) g_hmm_xi_sum[i] = HMM_LOG_ZERO;

        total_ll = 0.0f;

        // E-step: for each sequence
        for (int s = 0; s < g_hmm_n_seqs; s++) {
            const int* obs = &g_hmm_seqs[s * HMM_MAX_SEQ_LEN];
            int T = g_hmm_seq_lens[s];

            float ll = hmm_forward(obs, T);
            hmm_backward(obs, T);
            total_ll += ll;

            // Compute gamma[t][i] = P(s_t=i | obs, model)
            for (int t = 0; t < T; t++) {
                for (int i = 0; i < N; i++) {
                    g_hmm_gamma[t * N + i] = g_hmm_alpha[t * N + i] + g_hmm_beta[t * N + i] - ll;
                }
            }

            // Accumulate statistics
            // Prior: gamma[0]
            for (int i = 0; i < N; i++) {
                new_log_prior[i] = hmm_logadd(new_log_prior[i], g_hmm_gamma[0 * N + i]);
            }

            // Emission: sum gamma where obs[t] == m
            for (int t = 0; t < T; t++) {
                for (int i = 0; i < N; i++) {
                    new_log_emit[i * M + obs[t]] =
                        hmm_logadd(new_log_emit[i * M + obs[t]], g_hmm_gamma[t * N + i]);
                    gamma_sum[i] = hmm_logadd(gamma_sum[i], g_hmm_gamma[t * N + i]);
                }
            }

            // Transition: xi[i][j] = sum_t P(s_t=i, s_{t+1}=j | obs, model)
            for (int t = 0; t < T - 1; t++) {
                for (int i = 0; i < N; i++) {
                    for (int j = 0; j < N; j++) {
                        float xi_val = g_hmm_alpha[t * N + i] +
                                        g_hmm_log_trans[i * N + j] +
                                        g_hmm_log_emit[j * M + obs[t+1]] +
                                        g_hmm_beta[(t+1) * N + j] - ll;
                        g_hmm_xi_sum[i * N + j] =
                            hmm_logadd(g_hmm_xi_sum[i * N + j], xi_val);
                    }
                }
            }
        }

        // M-step: update parameters
        // Normalize prior
        float prior_sum = HMM_LOG_ZERO;
        for (int i = 0; i < N; i++) prior_sum = hmm_logadd(prior_sum, new_log_prior[i]);
        for (int i = 0; i < N; i++) g_hmm_log_prior[i] = new_log_prior[i] - prior_sum;

        // Normalize transition rows
        for (int i = 0; i < N; i++) {
            float row_sum = HMM_LOG_ZERO;
            for (int j = 0; j < N; j++) row_sum = hmm_logadd(row_sum, g_hmm_xi_sum[i * N + j]);
            for (int j = 0; j < N; j++) {
                g_hmm_log_trans[i * N + j] = (row_sum > HMM_LOG_ZERO + 1.0f) ?
                    g_hmm_xi_sum[i * N + j] - row_sum : logf(1.0f / (float)N);
            }
        }

        // Normalize emission rows
        for (int i = 0; i < N; i++) {
            for (int m = 0; m < M; m++) {
                g_hmm_log_emit[i * M + m] = (gamma_sum[i] > HMM_LOG_ZERO + 1.0f) ?
                    new_log_emit[i * M + m] - gamma_sum[i] : logf(1.0f / (float)M);
            }
        }
    }

    free(new_log_trans);
    free(new_log_emit);
    free(new_log_prior);
    free(gamma_sum);
    return total_ll;
}

// Predict most likely next state given current observation
static int hmm_predict(int obs) {
    if (!g_hmm_initialized) return -1;
    if (obs < 0 || obs >= g_hmm_n_obs) return -2;

    int N = g_hmm_n_states;
    // P(s | obs) ∝ P(obs | s) * P(s)
    // Then best next state = argmax_j sum_i P(s_curr=i|obs) * P(s_next=j|s_curr=i)
    float best_score = HMM_LOG_ZERO;
    int best_state = 0;

    for (int j = 0; j < N; j++) {
        float score = HMM_LOG_ZERO;
        for (int i = 0; i < N; i++) {
            float p_i = g_hmm_log_prior[i] + g_hmm_log_emit[i * g_hmm_n_obs + obs];
            score = hmm_logadd(score, p_i + g_hmm_log_trans[i * N + j]);
        }
        if (score > best_score) {
            best_score = score;
            best_state = j;
        }
    }
    return best_state;
}

static int hmm_save(const char* path) {
    if (!g_hmm_initialized) return -1;
    int N = g_hmm_n_states, M = g_hmm_n_obs;
    size_t size = 8 + N * sizeof(float) + N * N * sizeof(float) + N * M * sizeof(float);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int32_t* header = (int32_t*)buf;
    header[0] = N;
    header[1] = M;
    size_t off = 8;
    memcpy(&buf[off], g_hmm_log_prior, N * sizeof(float)); off += N * 4;
    memcpy(&buf[off], g_hmm_log_trans, N * N * sizeof(float)); off += N * N * 4;
    memcpy(&buf[off], g_hmm_log_emit, N * M * sizeof(float));

    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)size);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

static int hmm_load(const char* path) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 8) return -1;

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;

    int read = sentai_fs_read(path, buf, size);
    if (read != size) { free(buf); return -3; }

    int32_t* header = (int32_t*)buf;
    int N = header[0], M = header[1];
    if (N < 1 || N > HMM_MAX_STATES || M < 1 || M > HMM_MAX_OBS) { free(buf); return -4; }

    size_t expected = 8 + N * 4 + N * N * 4 + N * M * 4;
    if (size != (int)expected) { free(buf); return -4; }

    int rc = hmm_init(N, M);
    if (rc < 0) { free(buf); return rc; }

    size_t off = 8;
    memcpy(g_hmm_log_prior, &buf[off], N * sizeof(float)); off += N * 4;
    memcpy(g_hmm_log_trans, &buf[off], N * N * sizeof(float)); off += N * N * 4;
    memcpy(g_hmm_log_emit, &buf[off], N * M * sizeof(float));
    free(buf);
    return 0;
}

// ===================== MicroPython Bindings =====================

// sentai.hmm.init(n_states, n_obs) -> int
static mp_obj_t mod_hmm_init(mp_obj_t states_obj, mp_obj_t obs_obj) {
    return mp_obj_new_int(hmm_init(mp_obj_get_int(states_obj), mp_obj_get_int(obs_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_hmm_init_obj, mod_hmm_init);

// sentai.hmm.set_transition(i, j, prob) -> None
static mp_obj_t mod_hmm_set_transition(mp_obj_t i_obj, mp_obj_t j_obj, mp_obj_t p_obj) {
    if (!g_hmm_initialized) return mp_const_none;
    int i = mp_obj_get_int(i_obj), j = mp_obj_get_int(j_obj);
    float p = mp_obj_get_float(p_obj);
    if (i >= 0 && i < g_hmm_n_states && j >= 0 && j < g_hmm_n_states && p > 0.0f)
        g_hmm_log_trans[i * g_hmm_n_states + j] = logf(p);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_hmm_set_transition_obj, mod_hmm_set_transition);

// sentai.hmm.set_emission(state, obs, prob) -> None
static mp_obj_t mod_hmm_set_emission(mp_obj_t s_obj, mp_obj_t o_obj, mp_obj_t p_obj) {
    if (!g_hmm_initialized) return mp_const_none;
    int s = mp_obj_get_int(s_obj), o = mp_obj_get_int(o_obj);
    float p = mp_obj_get_float(p_obj);
    if (s >= 0 && s < g_hmm_n_states && o >= 0 && o < g_hmm_n_obs && p > 0.0f)
        g_hmm_log_emit[s * g_hmm_n_obs + o] = logf(p);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_hmm_set_emission_obj, mod_hmm_set_emission);

// sentai.hmm.set_prior(state, prob) -> None
static mp_obj_t mod_hmm_set_prior(mp_obj_t s_obj, mp_obj_t p_obj) {
    if (!g_hmm_initialized) return mp_const_none;
    int s = mp_obj_get_int(s_obj);
    float p = mp_obj_get_float(p_obj);
    if (s >= 0 && s < g_hmm_n_states && p > 0.0f)
        g_hmm_log_prior[s] = logf(p);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_hmm_set_prior_obj, mod_hmm_set_prior);

// sentai.hmm.add_seq(obs_list) -> int
static mp_obj_t mod_hmm_add_seq(mp_obj_t seq_obj) {
    if (!g_hmm_initialized) return mp_obj_new_int(-1);
    size_t arr_len;
    mp_obj_t *arr_items;
    mp_obj_get_array(seq_obj, &arr_len, &arr_items);
    int len = (int)arr_len;
    if (len < 1 || len > HMM_MAX_SEQ_LEN) return mp_obj_new_int(-3);

    int* obs = (int*)malloc(len * sizeof(int));
    if (!obs) return mp_obj_new_int(-5);
    for (int i = 0; i < len; i++)
        obs[i] = mp_obj_get_int(arr_items[i]);

    int rc = hmm_add_seq(obs, len);
    free(obs);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_add_seq_obj, mod_hmm_add_seq);

// sentai.hmm.train(max_iter=20) -> float (log-likelihood)
static mp_obj_t mod_hmm_train(size_t n_args, const mp_obj_t *args) {
    int max_iter = (n_args >= 1) ? mp_obj_get_int(args[0]) : 20;
    return mp_obj_new_float(hmm_train(max_iter));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_hmm_train_obj, 0, 1, mod_hmm_train);

// sentai.hmm.viterbi(obs_list) -> list (most likely state path)
static mp_obj_t mod_hmm_viterbi(mp_obj_t seq_obj) {
    if (!g_hmm_initialized) return mp_const_none;
    size_t arr_len;
    mp_obj_t *arr_items;
    mp_obj_get_array(seq_obj, &arr_len, &arr_items);
    int T = (int)arr_len;
    if (T < 1 || T > HMM_MAX_SEQ_LEN) return mp_const_none;

    int* obs = (int*)malloc(T * sizeof(int));
    int* path = (int*)malloc(T * sizeof(int));
    if (!obs || !path) {
        if (obs) free(obs);
        if (path) free(path);
        return mp_const_none;
    }

    for (int i = 0; i < T; i++) obs[i] = mp_obj_get_int(arr_items[i]);

    hmm_viterbi(obs, T, path);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(T, NULL));
    for (int i = 0; i < T; i++) result->items[i] = mp_obj_new_int(path[i]);

    free(obs);
    free(path);
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_viterbi_obj, mod_hmm_viterbi);

// sentai.hmm.predict(obs) -> int (most likely next state)
static mp_obj_t mod_hmm_predict(mp_obj_t obs_obj) {
    return mp_obj_new_int(hmm_predict(mp_obj_get_int(obs_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_predict_obj, mod_hmm_predict);

// sentai.hmm.log_likelihood(obs_list) -> float
static mp_obj_t mod_hmm_log_likelihood(mp_obj_t seq_obj) {
    if (!g_hmm_initialized) return mp_obj_new_float(HMM_LOG_ZERO);
    size_t arr_len;
    mp_obj_t *arr_items;
    mp_obj_get_array(seq_obj, &arr_len, &arr_items);
    int T = (int)arr_len;
    if (T < 1 || T > HMM_MAX_SEQ_LEN) return mp_obj_new_float(HMM_LOG_ZERO);

    int* obs = (int*)malloc(T * sizeof(int));
    if (!obs) return mp_obj_new_float(HMM_LOG_ZERO);
    for (int i = 0; i < T; i++) obs[i] = mp_obj_get_int(arr_items[i]);

    float ll = hmm_forward(obs, T);
    free(obs);
    return mp_obj_new_float(ll);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_log_likelihood_obj, mod_hmm_log_likelihood);

// sentai.hmm.from_tpu(idx, n_bins) -> int (discretized observation)
// Discretizes TPU output[idx] into n_bins uniform bins (argmax of output, binned)
static mp_obj_t mod_hmm_from_tpu(mp_obj_t idx_obj, mp_obj_t bins_obj) {
    extern int sentai_tpu_get_output_size(int idx);
    extern const void* sentai_tpu_get_output_data(int idx);
    extern int sentai_tpu_get_output_type(int idx);
    extern int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);

    int tpu_idx = mp_obj_get_int(idx_obj);
    int n_bins = mp_obj_get_int(bins_obj);
    if (n_bins < 1 || n_bins > g_hmm_n_obs) return mp_obj_new_int(-1);

    int size = sentai_tpu_get_output_size(tpu_idx);
    int type = sentai_tpu_get_output_type(tpu_idx);
    const void* data = sentai_tpu_get_output_data(tpu_idx);
    if (!data || size <= 0) return mp_obj_new_int(-5);

    // Find argmax of output tensor
    int n_elements = (type == 1) ? size / 4 : size;
    float scale = 1.0f;
    int32_t zero_point = 0;
    if (type == 9 || type == 3) sentai_tpu_output_quant(tpu_idx, &scale, &zero_point);

    float best_val = -1e30f;
    int best_idx = 0;
    for (int i = 0; i < n_elements; i++) {
        float v;
        if (type == 1) v = ((const float*)data)[i];
        else if (type == 9) v = scale * ((float)((const int8_t*)data)[i] - (float)zero_point);
        else v = scale * ((float)((const uint8_t*)data)[i] - (float)zero_point);
        if (v > best_val) { best_val = v; best_idx = i; }
    }

    // Map to bin (modulo n_bins for safety)
    int obs = best_idx % n_bins;
    return mp_obj_new_int(obs);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_hmm_from_tpu_obj, mod_hmm_from_tpu);

// sentai.hmm.from_imu(n_bins) -> int (discretized accel magnitude)
static mp_obj_t mod_hmm_from_imu(mp_obj_t bins_obj) {
    extern int sentai_imu_read_accel(float* x, float* y, float* z, float* t);
    int n_bins = mp_obj_get_int(bins_obj);
    if (n_bins < 1 || n_bins > g_hmm_n_obs) return mp_obj_new_int(-1);

    float x, y, z, temp;
    if (sentai_imu_read_accel(&x, &y, &z, &temp) < 0) return mp_obj_new_int(-5);

    // Magnitude in mg (0 ~ 4000 range for +/- 2g)
    float mag = sqrtf(x*x + y*y + z*z);
    // Quantize to [0, n_bins-1]
    int bin = (int)(mag / 4000.0f * (float)n_bins);
    if (bin < 0) bin = 0;
    if (bin >= n_bins) bin = n_bins - 1;
    return mp_obj_new_int(bin);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_from_imu_obj, mod_hmm_from_imu);

// sentai.hmm.save(path) -> int
static mp_obj_t mod_hmm_save(mp_obj_t path_obj) {
    return mp_obj_new_int(hmm_save(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_save_obj, mod_hmm_save);

// sentai.hmm.load(path) -> int
static mp_obj_t mod_hmm_load(mp_obj_t path_obj) {
    return mp_obj_new_int(hmm_load(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_hmm_load_obj, mod_hmm_load);

// sentai.hmm.info() -> dict
static mp_obj_t mod_hmm_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_states), mp_obj_new_int(g_hmm_n_states));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_observations), mp_obj_new_int(g_hmm_n_obs));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_sequences), mp_obj_new_int(g_hmm_n_seqs));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), mp_obj_new_bool(g_hmm_initialized));
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_hmm_info_obj, mod_hmm_info);

// sentai.hmm.clear() -> None
static mp_obj_t mod_hmm_clear(void) {
    g_hmm_n_seqs = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_hmm_clear_obj, mod_hmm_clear);

// ---- module table ----
static const mp_rom_map_elem_t sentai_hmm_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_hmm) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_hmm_init_obj) },
    // Manual parameter setting
    { MP_ROM_QSTR(MP_QSTR_set_transition), MP_ROM_PTR(&mod_hmm_set_transition_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_emission), MP_ROM_PTR(&mod_hmm_set_emission_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_prior), MP_ROM_PTR(&mod_hmm_set_prior_obj) },
    // Training
    { MP_ROM_QSTR(MP_QSTR_add_seq), MP_ROM_PTR(&mod_hmm_add_seq_obj) },
    { MP_ROM_QSTR(MP_QSTR_train), MP_ROM_PTR(&mod_hmm_train_obj) },
    // Inference
    { MP_ROM_QSTR(MP_QSTR_viterbi), MP_ROM_PTR(&mod_hmm_viterbi_obj) },
    { MP_ROM_QSTR(MP_QSTR_predict), MP_ROM_PTR(&mod_hmm_predict_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_likelihood), MP_ROM_PTR(&mod_hmm_log_likelihood_obj) },
    // Sensor interop
    { MP_ROM_QSTR(MP_QSTR_from_tpu), MP_ROM_PTR(&mod_hmm_from_tpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_imu), MP_ROM_PTR(&mod_hmm_from_imu_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_hmm_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_hmm_load_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_hmm_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&mod_hmm_clear_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_hmm_globals, sentai_hmm_globals_table);
static const mp_obj_module_t sentai_hmm_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_hmm_globals,
};
