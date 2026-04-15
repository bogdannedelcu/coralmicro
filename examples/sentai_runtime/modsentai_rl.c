// ============== sentai.rl — Reinforcement Learning ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Three RL algorithms for on-device adaptive control:
//   1. Q-learning (tabular): Classic S×A value table with epsilon-greedy
//   2. Multi-Armed Bandit (MAB): UCB1, epsilon-greedy, Thompson sampling
//   3. DQN (Deep Q-Network): 2-layer MLP with experience replay
//
// DQN uses a self-contained MLP (NOT the AIfES singleton) to avoid conflicts
// with user's loaded AIfES model. The MLP is ~100 lines of pure C.

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== Constants =====================

#define RL_MAX_STATES       256
#define RL_MAX_ACTIONS      16

#define DQN_MAX_STATE_DIM   64
#define DQN_MAX_HIDDEN      128
#define DQN_MAX_ACTIONS     16
#define DQN_REPLAY_SIZE     1000

// ===================== Q-Learning State =====================

static float* g_q_table = NULL;         // states * actions
static int g_q_n_states = 0;
static int g_q_n_actions = 0;
static int g_q_initialized = 0;

// ===================== MAB State =====================

#define MAB_MAX_ARMS 32
static int g_mab_n_arms = 0;
static int g_mab_initialized = 0;
static float g_mab_rewards[MAB_MAX_ARMS];    // total reward per arm
static int g_mab_counts[MAB_MAX_ARMS];       // pull count per arm
static float g_mab_sq_rewards[MAB_MAX_ARMS]; // sum of squared rewards (for Thompson)
static int g_mab_total_pulls;

// ===================== DQN State =====================

typedef struct {
    float* state;       // state_dim
    int action;
    float reward;
    float* next_state;  // state_dim
    int done;
} dqn_experience_t;

static int g_dqn_state_dim = 0;
static int g_dqn_n_actions = 0;
static int g_dqn_hidden = 0;
static int g_dqn_initialized = 0;

// MLP weights: input→hidden (W1, b1), hidden→output (W2, b2)
static float* g_dqn_W1 = NULL;     // state_dim * hidden
static float* g_dqn_b1 = NULL;     // hidden
static float* g_dqn_W2 = NULL;     // hidden * n_actions
static float* g_dqn_b2 = NULL;     // n_actions

// Replay buffer
static float* g_dqn_replay_buf = NULL;  // REPLAY_SIZE * (2*state_dim + 3) floats
static int g_dqn_replay_count = 0;
static int g_dqn_replay_pos = 0;

// Work buffers
static float* g_dqn_h = NULL;       // hidden (hidden layer activation)
static float* g_dqn_q = NULL;       // n_actions (Q-values output)
static float* g_dqn_dh = NULL;      // hidden (gradient of hidden)
static float* g_dqn_dq = NULL;      // n_actions (gradient of output)

static float g_dqn_lr = 0.001f;
static float g_dqn_gamma = 0.99f;

// Simple pseudo-random (xorshift32)
static uint32_t g_rl_rng_state = 12345;
static uint32_t rl_rand(void) {
    g_rl_rng_state ^= g_rl_rng_state << 13;
    g_rl_rng_state ^= g_rl_rng_state >> 17;
    g_rl_rng_state ^= g_rl_rng_state << 5;
    return g_rl_rng_state;
}
static float rl_randf(void) { return (float)(rl_rand() & 0x7FFFFFFF) / (float)0x7FFFFFFF; }

// ===================== DQN Internal MLP =====================

// Forward pass: state → hidden (relu) → Q-values
static void dqn_forward(const float* state, float* h, float* q) {
    int D = g_dqn_state_dim, H = g_dqn_hidden, A = g_dqn_n_actions;

    // Hidden = relu(W1^T * state + b1)
    for (int j = 0; j < H; j++) {
        float sum = g_dqn_b1[j];
        for (int i = 0; i < D; i++) sum += g_dqn_W1[i * H + j] * state[i];
        h[j] = (sum > 0.0f) ? sum : 0.0f;  // ReLU
    }

    // Q = W2^T * hidden + b2
    for (int a = 0; a < A; a++) {
        float sum = g_dqn_b2[a];
        for (int j = 0; j < H; j++) sum += g_dqn_W2[j * A + a] * h[j];
        q[a] = sum;
    }
}

// Backward pass: given target Q-values, update weights with SGD
static void dqn_backward(const float* state, const float* h, const float* q,
                           const float* target_q, int action_mask) {
    int D = g_dqn_state_dim, H = g_dqn_hidden, A = g_dqn_n_actions;
    float lr = g_dqn_lr;

    // dq = q - target_q (only for the taken action)
    memset(g_dqn_dq, 0, A * sizeof(float));
    g_dqn_dq[action_mask] = q[action_mask] - target_q[action_mask];

    // Update W2, b2
    for (int j = 0; j < H; j++) {
        for (int a = 0; a < A; a++) {
            g_dqn_W2[j * A + a] -= lr * h[j] * g_dqn_dq[a];
        }
    }
    for (int a = 0; a < A; a++) {
        g_dqn_b2[a] -= lr * g_dqn_dq[a];
    }

    // dh = W2 * dq, masked by relu derivative
    for (int j = 0; j < H; j++) {
        float sum = 0.0f;
        for (int a = 0; a < A; a++) sum += g_dqn_W2[j * A + a] * g_dqn_dq[a];
        g_dqn_dh[j] = (h[j] > 0.0f) ? sum : 0.0f;  // ReLU derivative
    }

    // Update W1, b1
    for (int i = 0; i < D; i++) {
        for (int j = 0; j < H; j++) {
            g_dqn_W1[i * H + j] -= lr * state[i] * g_dqn_dh[j];
        }
    }
    for (int j = 0; j < H; j++) {
        g_dqn_b1[j] -= lr * g_dqn_dh[j];
    }
}

// Xavier initialization
static void dqn_init_weights(void) {
    int D = g_dqn_state_dim, H = g_dqn_hidden, A = g_dqn_n_actions;

    float scale1 = sqrtf(2.0f / (float)(D + H));
    for (int i = 0; i < D * H; i++) g_dqn_W1[i] = (rl_randf() * 2.0f - 1.0f) * scale1;
    memset(g_dqn_b1, 0, H * sizeof(float));

    float scale2 = sqrtf(2.0f / (float)(H + A));
    for (int i = 0; i < H * A; i++) g_dqn_W2[i] = (rl_randf() * 2.0f - 1.0f) * scale2;
    memset(g_dqn_b2, 0, A * sizeof(float));
}

// ===================== Q-Learning C API =====================

static int q_init(int n_states, int n_actions) {
    if (n_states < 1 || n_states > RL_MAX_STATES) return -1;
    if (n_actions < 1 || n_actions > RL_MAX_ACTIONS) return -2;

    if (g_q_table) { free(g_q_table); g_q_table = NULL; }

    g_q_table = (float*)malloc(n_states * n_actions * sizeof(float));
    if (!g_q_table) return -3;

    memset(g_q_table, 0, n_states * n_actions * sizeof(float));
    g_q_n_states = n_states;
    g_q_n_actions = n_actions;
    g_q_initialized = 1;
    return 0;
}

static int q_update(int s, int a, float r, int s_next, float alpha, float gamma) {
    if (!g_q_initialized) return -1;
    if (s < 0 || s >= g_q_n_states || a < 0 || a >= g_q_n_actions) return -2;
    if (s_next < 0 || s_next >= g_q_n_states) return -2;

    // Q(s,a) = Q(s,a) + alpha * (r + gamma * max_a' Q(s',a') - Q(s,a))
    float max_q_next = g_q_table[s_next * g_q_n_actions + 0];
    for (int i = 1; i < g_q_n_actions; i++) {
        float v = g_q_table[s_next * g_q_n_actions + i];
        if (v > max_q_next) max_q_next = v;
    }

    float old_q = g_q_table[s * g_q_n_actions + a];
    g_q_table[s * g_q_n_actions + a] = old_q + alpha * (r + gamma * max_q_next - old_q);
    return 0;
}

static int q_action(int s, float epsilon) {
    if (!g_q_initialized || s < 0 || s >= g_q_n_states) return -1;

    // Epsilon-greedy
    if (rl_randf() < epsilon) {
        return (int)(rl_rand() % (uint32_t)g_q_n_actions);
    }

    // Greedy
    int best = 0;
    float best_q = g_q_table[s * g_q_n_actions + 0];
    for (int a = 1; a < g_q_n_actions; a++) {
        float v = g_q_table[s * g_q_n_actions + a];
        if (v > best_q) { best_q = v; best = a; }
    }
    return best;
}

// ===================== MAB C API =====================

static int mab_init(int n_arms) {
    if (n_arms < 1 || n_arms > MAB_MAX_ARMS) return -1;
    g_mab_n_arms = n_arms;
    memset(g_mab_rewards, 0, sizeof(g_mab_rewards));
    memset(g_mab_counts, 0, sizeof(g_mab_counts));
    memset(g_mab_sq_rewards, 0, sizeof(g_mab_sq_rewards));
    g_mab_total_pulls = 0;
    g_mab_initialized = 1;
    return 0;
}

static int mab_pull(int arm, float reward) {
    if (!g_mab_initialized || arm < 0 || arm >= g_mab_n_arms) return -1;
    g_mab_rewards[arm] += reward;
    g_mab_sq_rewards[arm] += reward * reward;
    g_mab_counts[arm]++;
    g_mab_total_pulls++;
    return 0;
}

// strategy: 0=epsilon-greedy(0.1), 1=UCB1, 2=Thompson
static int mab_select(int strategy) {
    if (!g_mab_initialized) return -1;
    int A = g_mab_n_arms;

    // First: try each arm at least once
    for (int a = 0; a < A; a++) {
        if (g_mab_counts[a] == 0) return a;
    }

    if (strategy == 0) {
        // Epsilon-greedy (eps=0.1)
        if (rl_randf() < 0.1f) return (int)(rl_rand() % (uint32_t)A);
        int best = 0;
        float best_v = g_mab_rewards[0] / (float)g_mab_counts[0];
        for (int a = 1; a < A; a++) {
            float v = g_mab_rewards[a] / (float)g_mab_counts[a];
            if (v > best_v) { best_v = v; best = a; }
        }
        return best;
    }
    else if (strategy == 1) {
        // UCB1
        float log_t = logf((float)g_mab_total_pulls);
        int best = 0;
        float best_ucb = -1e30f;
        for (int a = 0; a < A; a++) {
            float mean = g_mab_rewards[a] / (float)g_mab_counts[a];
            float ucb = mean + sqrtf(2.0f * log_t / (float)g_mab_counts[a]);
            if (ucb > best_ucb) { best_ucb = ucb; best = a; }
        }
        return best;
    }
    else {
        // Thompson sampling (Gaussian approximation)
        int best = 0;
        float best_sample = -1e30f;
        for (int a = 0; a < A; a++) {
            float mean = g_mab_rewards[a] / (float)g_mab_counts[a];
            float var = (g_mab_sq_rewards[a] / (float)g_mab_counts[a]) - mean * mean;
            if (var < 1e-6f) var = 1e-6f;
            float std = sqrtf(var / (float)g_mab_counts[a]);
            // Box-Muller approximation (one normal sample)
            float u1 = rl_randf(), u2 = rl_randf();
            if (u1 < 1e-10f) u1 = 1e-10f;
            float z = sqrtf(-2.0f * logf(u1)) * cosf(6.283185f * u2);
            float sample = mean + std * z;
            if (sample > best_sample) { best_sample = sample; best = a; }
        }
        return best;
    }
}

// ===================== DQN C API =====================

static int dqn_init(int state_dim, int n_actions, int hidden) {
    if (state_dim < 1 || state_dim > DQN_MAX_STATE_DIM) return -1;
    if (n_actions < 1 || n_actions > DQN_MAX_ACTIONS) return -2;
    if (hidden < 1 || hidden > DQN_MAX_HIDDEN) return -3;

    if (g_dqn_W1) { free(g_dqn_W1); g_dqn_W1 = NULL; }
    if (g_dqn_b1) { free(g_dqn_b1); g_dqn_b1 = NULL; }
    if (g_dqn_W2) { free(g_dqn_W2); g_dqn_W2 = NULL; }
    if (g_dqn_b2) { free(g_dqn_b2); g_dqn_b2 = NULL; }
    if (g_dqn_replay_buf) { free(g_dqn_replay_buf); g_dqn_replay_buf = NULL; }
    if (g_dqn_h) { free(g_dqn_h); g_dqn_h = NULL; }
    if (g_dqn_q) { free(g_dqn_q); g_dqn_q = NULL; }
    if (g_dqn_dh) { free(g_dqn_dh); g_dqn_dh = NULL; }
    if (g_dqn_dq) { free(g_dqn_dq); g_dqn_dq = NULL; }

    int D = state_dim, H = hidden, A = n_actions;
    int exp_size = 2 * D + 3;  // s, s', a, r, done

    g_dqn_W1 = (float*)malloc(D * H * sizeof(float));
    g_dqn_b1 = (float*)malloc(H * sizeof(float));
    g_dqn_W2 = (float*)malloc(H * A * sizeof(float));
    g_dqn_b2 = (float*)malloc(A * sizeof(float));
    g_dqn_replay_buf = (float*)malloc(DQN_REPLAY_SIZE * exp_size * sizeof(float));
    g_dqn_h = (float*)malloc(H * sizeof(float));
    g_dqn_q = (float*)malloc(A * sizeof(float));
    g_dqn_dh = (float*)malloc(H * sizeof(float));
    g_dqn_dq = (float*)malloc(A * sizeof(float));

    if (!g_dqn_W1 || !g_dqn_b1 || !g_dqn_W2 || !g_dqn_b2 ||
        !g_dqn_replay_buf || !g_dqn_h || !g_dqn_q || !g_dqn_dh || !g_dqn_dq) {
        if (g_dqn_W1) free(g_dqn_W1);
        if (g_dqn_b1) free(g_dqn_b1);
        if (g_dqn_W2) free(g_dqn_W2);
        if (g_dqn_b2) free(g_dqn_b2);
        if (g_dqn_replay_buf) free(g_dqn_replay_buf);
        if (g_dqn_h) free(g_dqn_h);
        if (g_dqn_q) free(g_dqn_q);
        if (g_dqn_dh) free(g_dqn_dh);
        if (g_dqn_dq) free(g_dqn_dq);
        g_dqn_W1 = g_dqn_b1 = g_dqn_W2 = g_dqn_b2 = NULL;
        g_dqn_replay_buf = g_dqn_h = g_dqn_q = g_dqn_dh = g_dqn_dq = NULL;
        g_dqn_initialized = 0;
        return -4;
    }

    g_dqn_state_dim = D;
    g_dqn_n_actions = A;
    g_dqn_hidden = H;
    g_dqn_replay_count = 0;
    g_dqn_replay_pos = 0;
    g_dqn_lr = 0.001f;
    g_dqn_gamma = 0.99f;
    g_dqn_initialized = 1;

    dqn_init_weights();
    return 0;
}

// Store experience in replay buffer (circular)
static int dqn_observe(const float* state, int action, float reward,
                        const float* next_state, int done) {
    if (!g_dqn_initialized) return -1;
    int D = g_dqn_state_dim;
    int exp_size = 2 * D + 3;
    float* entry = &g_dqn_replay_buf[g_dqn_replay_pos * exp_size];

    memcpy(entry, state, D * sizeof(float));
    entry[D] = (float)action;
    entry[D + 1] = reward;
    memcpy(&entry[D + 2], next_state, D * sizeof(float));
    entry[2 * D + 2] = (float)done;

    g_dqn_replay_pos = (g_dqn_replay_pos + 1) % DQN_REPLAY_SIZE;
    if (g_dqn_replay_count < DQN_REPLAY_SIZE) g_dqn_replay_count++;
    return 0;
}

// Select action using epsilon-greedy policy
static int dqn_action(const float* state, float epsilon) {
    if (!g_dqn_initialized) return -1;

    if (rl_randf() < epsilon) {
        return (int)(rl_rand() % (uint32_t)g_dqn_n_actions);
    }

    dqn_forward(state, g_dqn_h, g_dqn_q);
    int best = 0;
    for (int a = 1; a < g_dqn_n_actions; a++) {
        if (g_dqn_q[a] > g_dqn_q[best]) best = a;
    }
    return best;
}

// Train on a batch from replay buffer
static float dqn_train_batch(int batch_size) {
    if (!g_dqn_initialized || g_dqn_replay_count < batch_size) return -1.0f;

    int D = g_dqn_state_dim;
    int A = g_dqn_n_actions;
    int exp_size = 2 * D + 3;
    float total_loss = 0.0f;

    float* h_next = (float*)malloc(g_dqn_hidden * sizeof(float));
    float* q_next = (float*)malloc(A * sizeof(float));
    float* target = (float*)malloc(A * sizeof(float));
    if (!h_next || !q_next || !target) {
        if (h_next) free(h_next);
        if (q_next) free(q_next);
        if (target) free(target);
        return -1.0f;
    }

    for (int b = 0; b < batch_size; b++) {
        // Random sample from replay buffer
        int idx = (int)(rl_rand() % (uint32_t)g_dqn_replay_count);
        float* entry = &g_dqn_replay_buf[idx * exp_size];

        const float* s = entry;
        int action = (int)entry[D];
        float reward = entry[D + 1];
        const float* s_next = &entry[D + 2];
        int done = (int)entry[2 * D + 2];

        // Forward pass for current state
        dqn_forward(s, g_dqn_h, g_dqn_q);

        // Compute target
        memcpy(target, g_dqn_q, A * sizeof(float));
        if (done) {
            target[action] = reward;
        } else {
            dqn_forward(s_next, h_next, q_next);
            float max_q = q_next[0];
            for (int a = 1; a < A; a++) {
                if (q_next[a] > max_q) max_q = q_next[a];
            }
            target[action] = reward + g_dqn_gamma * max_q;
        }

        float err = g_dqn_q[action] - target[action];
        total_loss += err * err;

        // Backward pass (only updates the taken action's gradient)
        dqn_backward(s, g_dqn_h, g_dqn_q, target, action);
    }

    free(h_next);
    free(q_next);
    free(target);
    return total_loss / (float)batch_size;
}

// ===================== MicroPython Bindings — Q-Learning =====================

// sentai.rl.q_init(n_states, n_actions) -> int
static mp_obj_t mod_rl_q_init(mp_obj_t s_obj, mp_obj_t a_obj) {
    return mp_obj_new_int(q_init(mp_obj_get_int(s_obj), mp_obj_get_int(a_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_rl_q_init_obj, mod_rl_q_init);

// sentai.rl.q_update(s, a, r, s_next, alpha=0.1, gamma=0.99) -> int
static mp_obj_t mod_rl_q_update(size_t n_args, const mp_obj_t *args) {
    int s = mp_obj_get_int(args[0]);
    int a = mp_obj_get_int(args[1]);
    float r = mp_obj_get_float(args[2]);
    int sn = mp_obj_get_int(args[3]);
    float alpha = (n_args >= 5) ? mp_obj_get_float(args[4]) : 0.1f;
    float gamma = (n_args >= 6) ? mp_obj_get_float(args[5]) : 0.99f;
    return mp_obj_new_int(q_update(s, a, r, sn, alpha, gamma));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_q_update_obj, 4, 6, mod_rl_q_update);

// sentai.rl.q_action(s, epsilon=0.1) -> int
static mp_obj_t mod_rl_q_action(size_t n_args, const mp_obj_t *args) {
    int s = mp_obj_get_int(args[0]);
    float eps = (n_args >= 2) ? mp_obj_get_float(args[1]) : 0.1f;
    return mp_obj_new_int(q_action(s, eps));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_q_action_obj, 1, 2, mod_rl_q_action);

// sentai.rl.q_value(s, a) -> float
static mp_obj_t mod_rl_q_value(mp_obj_t s_obj, mp_obj_t a_obj) {
    if (!g_q_initialized) return mp_obj_new_float(0.0f);
    int s = mp_obj_get_int(s_obj), a = mp_obj_get_int(a_obj);
    if (s < 0 || s >= g_q_n_states || a < 0 || a >= g_q_n_actions) return mp_obj_new_float(0.0f);
    return mp_obj_new_float(g_q_table[s * g_q_n_actions + a]);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_rl_q_value_obj, mod_rl_q_value);

// sentai.rl.q_save(path) -> int
static mp_obj_t mod_rl_q_save(mp_obj_t path_obj) {
    if (!g_q_initialized) return mp_obj_new_int(-1);
    size_t size = 8 + g_q_n_states * g_q_n_actions * sizeof(float);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return mp_obj_new_int(-2);
    int32_t* header = (int32_t*)buf;
    header[0] = g_q_n_states;
    header[1] = g_q_n_actions;
    memcpy(&buf[8], g_q_table, g_q_n_states * g_q_n_actions * sizeof(float));
    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(mp_obj_str_get_str(path_obj), buf, (int)size);
    free(buf);
    return mp_obj_new_int(rc >= 0 ? 0 : rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_rl_q_save_obj, mod_rl_q_save);

// sentai.rl.q_load(path) -> int
static mp_obj_t mod_rl_q_load(mp_obj_t path_obj) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);
    const char* path = mp_obj_str_get_str(path_obj);
    int size = sentai_fs_size(path);
    if (size < 8) return mp_obj_new_int(-1);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return mp_obj_new_int(-2);
    if (sentai_fs_read(path, buf, size) != size) { free(buf); return mp_obj_new_int(-3); }
    int32_t* header = (int32_t*)buf;
    if (header[0] < 1 || header[0] > RL_MAX_STATES || header[1] < 1 || header[1] > RL_MAX_ACTIONS) {
        free(buf); return mp_obj_new_int(-4);
    }
    if (size != (int)(8 + header[0] * header[1] * sizeof(float))) {
        free(buf); return mp_obj_new_int(-4);
    }
    int rc = q_init(header[0], header[1]);
    if (rc < 0) { free(buf); return mp_obj_new_int(rc); }
    memcpy(g_q_table, &buf[8], header[0] * header[1] * sizeof(float));
    free(buf);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_rl_q_load_obj, mod_rl_q_load);

// sentai.rl.q_clear() -> None
static mp_obj_t mod_rl_q_clear(void) {
    if (g_q_initialized) memset(g_q_table, 0, g_q_n_states * g_q_n_actions * sizeof(float));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_rl_q_clear_obj, mod_rl_q_clear);

// ===================== MicroPython Bindings — MAB =====================

// sentai.rl.mab_init(n_arms) -> int
static mp_obj_t mod_rl_mab_init(mp_obj_t n_obj) {
    return mp_obj_new_int(mab_init(mp_obj_get_int(n_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_rl_mab_init_obj, mod_rl_mab_init);

// sentai.rl.mab_pull(arm, reward) -> int
static mp_obj_t mod_rl_mab_pull(mp_obj_t arm_obj, mp_obj_t r_obj) {
    return mp_obj_new_int(mab_pull(mp_obj_get_int(arm_obj), mp_obj_get_float(r_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_rl_mab_pull_obj, mod_rl_mab_pull);

// sentai.rl.mab_select(strategy=1) -> int  (0=eps-greedy, 1=UCB1, 2=Thompson)
static mp_obj_t mod_rl_mab_select(size_t n_args, const mp_obj_t *args) {
    int strategy = (n_args >= 1) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(mab_select(strategy));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_mab_select_obj, 0, 1, mod_rl_mab_select);

// sentai.rl.mab_stats() -> list of dicts
static mp_obj_t mod_rl_mab_stats(void) {
    if (!g_mab_initialized) return mp_const_none;
    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(g_mab_n_arms, NULL));
    for (int a = 0; a < g_mab_n_arms; a++) {
        mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(3));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_arm), mp_obj_new_int(a));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pulls), mp_obj_new_int(g_mab_counts[a]));
        float mean = (g_mab_counts[a] > 0) ? g_mab_rewards[a] / (float)g_mab_counts[a] : 0.0f;
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_mean), mp_obj_new_float(mean));
        list->items[a] = MP_OBJ_FROM_PTR(d);
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_rl_mab_stats_obj, mod_rl_mab_stats);

// sentai.rl.mab_clear() -> None
static mp_obj_t mod_rl_mab_clear(void) {
    if (g_mab_initialized) {
        memset(g_mab_rewards, 0, sizeof(g_mab_rewards));
        memset(g_mab_counts, 0, sizeof(g_mab_counts));
        memset(g_mab_sq_rewards, 0, sizeof(g_mab_sq_rewards));
        g_mab_total_pulls = 0;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_rl_mab_clear_obj, mod_rl_mab_clear);

// ===================== MicroPython Bindings — DQN =====================

// sentai.rl.dqn_init(state_dim, n_actions, hidden=64) -> int
static mp_obj_t mod_rl_dqn_init(size_t n_args, const mp_obj_t *args) {
    int sd = mp_obj_get_int(args[0]);
    int na = mp_obj_get_int(args[1]);
    int h = (n_args >= 3) ? mp_obj_get_int(args[2]) : 64;
    return mp_obj_new_int(dqn_init(sd, na, h));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_dqn_init_obj, 2, 3, mod_rl_dqn_init);

// sentai.rl.dqn_observe(state, action, reward, next_state, done) -> int
static mp_obj_t mod_rl_dqn_observe(size_t n_args, const mp_obj_t *args) {
    if (!g_dqn_initialized) return mp_obj_new_int(-1);
    int D = g_dqn_state_dim;

    size_t s_len, sn_len;
    mp_obj_t *s_items, *sn_items;
    mp_obj_get_array(args[0], &s_len, &s_items);
    int action = mp_obj_get_int(args[1]);
    float reward = mp_obj_get_float(args[2]);
    mp_obj_get_array(args[3], &sn_len, &sn_items);
    int done = mp_obj_is_true(args[4]) ? 1 : 0;

    if (s_len != (size_t)D || sn_len != (size_t)D) return mp_obj_new_int(-4);

    float* s = (float*)malloc(D * sizeof(float));
    float* sn = (float*)malloc(D * sizeof(float));
    if (!s || !sn) { if (s) free(s); if (sn) free(sn); return mp_obj_new_int(-5); }

    for (int i = 0; i < D; i++) {
        s[i] = mp_obj_get_float(s_items[i]);
        sn[i] = mp_obj_get_float(sn_items[i]);
    }

    int rc = dqn_observe(s, action, reward, sn, done);
    free(s);
    free(sn);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_dqn_observe_obj, 5, 5, mod_rl_dqn_observe);

// sentai.rl.dqn_action(state, epsilon=0.1) -> int
static mp_obj_t mod_rl_dqn_action(size_t n_args, const mp_obj_t *args) {
    if (!g_dqn_initialized) return mp_obj_new_int(-1);
    int D = g_dqn_state_dim;

    size_t s_len;
    mp_obj_t *s_items;
    mp_obj_get_array(args[0], &s_len, &s_items);
    if (s_len != (size_t)D) return mp_obj_new_int(-4);

    float eps = (n_args >= 2) ? mp_obj_get_float(args[1]) : 0.1f;
    float* s = (float*)malloc(D * sizeof(float));
    if (!s) return mp_obj_new_int(-5);
    for (int i = 0; i < D; i++) s[i] = mp_obj_get_float(s_items[i]);

    int a = dqn_action(s, eps);
    free(s);
    return mp_obj_new_int(a);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_dqn_action_obj, 1, 2, mod_rl_dqn_action);

// sentai.rl.dqn_train(batch_size=32) -> float (average loss)
static mp_obj_t mod_rl_dqn_train(size_t n_args, const mp_obj_t *args) {
    int batch = (n_args >= 1) ? mp_obj_get_int(args[0]) : 32;
    return mp_obj_new_float(dqn_train_batch(batch));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_rl_dqn_train_obj, 0, 1, mod_rl_dqn_train);

// sentai.rl.dqn_save(path) -> int
static mp_obj_t mod_rl_dqn_save(mp_obj_t path_obj) {
    if (!g_dqn_initialized) return mp_obj_new_int(-1);
    int D = g_dqn_state_dim, H = g_dqn_hidden, A = g_dqn_n_actions;
    size_t size = 12 + (D * H + H + H * A + A) * sizeof(float);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return mp_obj_new_int(-2);
    int32_t* header = (int32_t*)buf;
    header[0] = D; header[1] = A; header[2] = H;
    size_t off = 12;
    memcpy(&buf[off], g_dqn_W1, D * H * sizeof(float)); off += D * H * 4;
    memcpy(&buf[off], g_dqn_b1, H * sizeof(float)); off += H * 4;
    memcpy(&buf[off], g_dqn_W2, H * A * sizeof(float)); off += H * A * 4;
    memcpy(&buf[off], g_dqn_b2, A * sizeof(float));
    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(mp_obj_str_get_str(path_obj), buf, (int)size);
    free(buf);
    return mp_obj_new_int(rc >= 0 ? 0 : rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_rl_dqn_save_obj, mod_rl_dqn_save);

// sentai.rl.dqn_load(path) -> int
static mp_obj_t mod_rl_dqn_load(mp_obj_t path_obj) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);
    const char* path = mp_obj_str_get_str(path_obj);
    int size = sentai_fs_size(path);
    if (size < 12) return mp_obj_new_int(-1);
    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return mp_obj_new_int(-2);
    if (sentai_fs_read(path, buf, size) != size) { free(buf); return mp_obj_new_int(-3); }
    int32_t* header = (int32_t*)buf;
    int D = header[0], A = header[1], H = header[2];
    size_t expected = 12 + ((size_t)D * H + H + (size_t)H * A + A) * sizeof(float);
    if (size != (int)expected) { free(buf); return mp_obj_new_int(-4); }
    int rc = dqn_init(D, A, H);
    if (rc < 0) { free(buf); return mp_obj_new_int(rc); }
    size_t off = 12;
    memcpy(g_dqn_W1, &buf[off], D * H * sizeof(float)); off += D * H * 4;
    memcpy(g_dqn_b1, &buf[off], H * sizeof(float)); off += H * 4;
    memcpy(g_dqn_W2, &buf[off], H * A * sizeof(float)); off += H * A * 4;
    memcpy(g_dqn_b2, &buf[off], A * sizeof(float));
    free(buf);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_rl_dqn_load_obj, mod_rl_dqn_load);

// sentai.rl.dqn_clear() -> None
static mp_obj_t mod_rl_dqn_clear(void) {
    if (g_dqn_initialized) {
        dqn_init_weights();
        g_dqn_replay_count = 0;
        g_dqn_replay_pos = 0;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_rl_dqn_clear_obj, mod_rl_dqn_clear);

// sentai.rl.info() -> dict
static mp_obj_t mod_rl_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(6));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_q_initialized), mp_obj_new_bool(g_q_initialized));
    if (g_q_initialized) {
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_q_states), mp_obj_new_int(g_q_n_states));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_q_actions), mp_obj_new_int(g_q_n_actions));
    }
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mab_initialized), mp_obj_new_bool(g_mab_initialized));
    if (g_mab_initialized) {
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mab_arms), mp_obj_new_int(g_mab_n_arms));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mab_pulls), mp_obj_new_int(g_mab_total_pulls));
    }
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dqn_initialized), mp_obj_new_bool(g_dqn_initialized));
    if (g_dqn_initialized) {
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dqn_state_dim), mp_obj_new_int(g_dqn_state_dim));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dqn_actions), mp_obj_new_int(g_dqn_n_actions));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dqn_hidden), mp_obj_new_int(g_dqn_hidden));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dqn_replay), mp_obj_new_int(g_dqn_replay_count));
    }
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_rl_info_obj, mod_rl_info);

// ---- module table ----
static const mp_rom_map_elem_t sentai_rl_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rl) },
    // Q-learning
    { MP_ROM_QSTR(MP_QSTR_q_init), MP_ROM_PTR(&mod_rl_q_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_update), MP_ROM_PTR(&mod_rl_q_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_action), MP_ROM_PTR(&mod_rl_q_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_value), MP_ROM_PTR(&mod_rl_q_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_save), MP_ROM_PTR(&mod_rl_q_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_load), MP_ROM_PTR(&mod_rl_q_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_q_clear), MP_ROM_PTR(&mod_rl_q_clear_obj) },
    // Multi-Armed Bandit
    { MP_ROM_QSTR(MP_QSTR_mab_init), MP_ROM_PTR(&mod_rl_mab_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_mab_pull), MP_ROM_PTR(&mod_rl_mab_pull_obj) },
    { MP_ROM_QSTR(MP_QSTR_mab_select), MP_ROM_PTR(&mod_rl_mab_select_obj) },
    { MP_ROM_QSTR(MP_QSTR_mab_stats), MP_ROM_PTR(&mod_rl_mab_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_mab_clear), MP_ROM_PTR(&mod_rl_mab_clear_obj) },
    // DQN
    { MP_ROM_QSTR(MP_QSTR_dqn_init), MP_ROM_PTR(&mod_rl_dqn_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_observe), MP_ROM_PTR(&mod_rl_dqn_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_action), MP_ROM_PTR(&mod_rl_dqn_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_train), MP_ROM_PTR(&mod_rl_dqn_train_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_save), MP_ROM_PTR(&mod_rl_dqn_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_load), MP_ROM_PTR(&mod_rl_dqn_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_dqn_clear), MP_ROM_PTR(&mod_rl_dqn_clear_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_rl_info_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_rl_globals, sentai_rl_globals_table);
static const mp_obj_module_t sentai_rl_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_rl_globals,
};
