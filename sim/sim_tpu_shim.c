// sim_tpu_shim.c — host-side TPU C entry points for sentai_sim.
//
// Implements the sentai_tpu_* contract from sentai_tpu_shim.h by
// forwarding requests to the pycoral helper daemon listening on
// /tmp/sentai_tpu.sock (sim/scripts/sim_tpu_helper.py).
//
// All compute (model load, inference, tensor extraction) happens in the
// helper daemon — this file only marshals length-prefixed framed
// requests.  Stays in C so the same modsentai_tpu_sim.c MicroPython
// bindings drop in unchanged on top.
//
// Bounded behaviour per embeded.md: every read/write has SO_*TIMEO; the
// daemon is auto-restarted by the operator (no in-shim respawn).  On
// any wire error we log + return -1 instead of blocking.
//
// Output buffer ownership: GET_OUTPUT replies are copied into a static
// per-output buffer owned by this file (matches the sentai_tpu_get_output_data
// contract on ARM, where the pointer is into the live tflite arena).

#define _POSIX_C_SOURCE 200809L

#include "examples/sentai_runtime/sentai_tpu_shim.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>
#include <pthread.h>

#define SOCK_PATH    "/tmp/sentai_tpu.sock"
#define REQ_MAGIC    0x53545055u   // 'STPU'
#define REPLY_MAGIC  0x52545055u   // 'RTPU'

#define OP_LOAD          1
#define OP_INVOKE        2
#define OP_GET_OUTPUT    3
#define OP_GET_INFO      4
#define OP_SET_INPUT     5
#define OP_OUTPUT_DIMS   6
#define OP_OUTPUT_QUANT  7
#define OP_NUM_OUTPUTS   8
#define OP_OUTPUT_TYPE   9
#define OP_OUTPUT_HASH  10

// Maximum output we cache per index.  Single biggest model output we
// ship is yolo_5_256 at ~256 KB; double for headroom.
#define MAX_OUTPUT_BYTES (640 * 1024)
#define MAX_OUTPUTS      8

// ── Single persistent connection ────────────────────────────────────
static int           s_fd = -1;
static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;

// ── Cached output buffers ───────────────────────────────────────────
static uint8_t  s_out_data[MAX_OUTPUTS][MAX_OUTPUT_BYTES];
static int      s_out_len [MAX_OUTPUTS];
static int      s_out_dims[MAX_OUTPUTS][4];
static float    s_out_scale[MAX_OUTPUTS];
static int32_t  s_out_zp   [MAX_OUTPUTS];
static int      s_out_type [MAX_OUTPUTS];
// Cached input metadata.
static int      s_in_dims[4];
static float    s_in_scale;
static int32_t  s_in_zp;
static int      s_in_type;
static int      s_loaded = 0;
static int      s_n_outputs = 0;

// ────────────────────────────────────────────────────────────────────
// I/O helpers
// ────────────────────────────────────────────────────────────────────
static int read_full(int fd, void* dst, size_t n) {
    uint8_t* p = (uint8_t*)dst; size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r > 0) { got += (size_t)r; continue; }
        if (r < 0 && errno == EINTR) continue;
        return -1;
    }
    return 0;
}
static int write_full(int fd, const void* src, size_t n) {
    const uint8_t* p = (const uint8_t*)src; size_t put = 0;
    while (put < n) {
        ssize_t w = write(fd, p + put, n - put);
        if (w > 0) { put += (size_t)w; continue; }
        if (w < 0 && errno == EINTR) continue;
        return -1;
    }
    return 0;
}

static int connect_helper(void) {
    if (s_fd >= 0) return 0;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("[tpu] helper not running at %s — start it with:\r\n", SOCK_PATH);
        printf("       venv-coral/bin/python3 sim/scripts/sim_tpu_helper.py\r\n");
        close(fd); return -1;
    }
    // Bound every wire op (model load can take 700ms+, leave headroom).
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    s_fd = fd;
    return 0;
}

// Issue a request, write payload, read header+payload reply.  Returns
// status code from helper (0 on success, negative on error).  *result
// is filled with up to result_max bytes; *result_len gets actual size.
static int call_helper(uint32_t opcode,
                        const void* req_payload, uint32_t req_len,
                        void* result, uint32_t result_max,
                        uint32_t* result_len) {
    pthread_mutex_lock(&s_mu);
    int rc = -1;
    if (connect_helper() < 0) goto out;

    uint32_t hdr[3] = { REQ_MAGIC, opcode, req_len };
    if (write_full(s_fd, hdr, sizeof(hdr)) < 0) {
        printf("[tpu] write hdr failed (errno=%d)\r\n", errno);
        close(s_fd); s_fd = -1; goto out;
    }
    if (req_len && write_full(s_fd, req_payload, req_len) < 0) {
        printf("[tpu] write payload failed\r\n");
        close(s_fd); s_fd = -1; goto out;
    }

    uint32_t rhdr[3];
    if (read_full(s_fd, rhdr, sizeof(rhdr)) < 0) {
        printf("[tpu] read reply hdr failed (timeout?)\r\n");
        close(s_fd); s_fd = -1; goto out;
    }
    if (rhdr[0] != REPLY_MAGIC) {
        printf("[tpu] bad reply magic 0x%08x\r\n", rhdr[0]);
        close(s_fd); s_fd = -1; goto out;
    }
    int32_t status = (int32_t)rhdr[1];
    uint32_t plen  = rhdr[2];

    // Always drain the payload, then truncate into caller's buffer.
    if (plen > 0) {
        // Read into result if it fits, else read into a discard buffer.
        if (result && plen <= result_max) {
            if (read_full(s_fd, result, plen) < 0) {
                close(s_fd); s_fd = -1; goto out;
            }
            if (result_len) *result_len = plen;
        } else {
            // Drain in chunks.
            uint8_t scratch[4096];
            uint32_t left = plen;
            while (left > 0) {
                uint32_t take = left < sizeof(scratch) ? left : sizeof(scratch);
                if (read_full(s_fd, scratch, take) < 0) {
                    close(s_fd); s_fd = -1; goto out;
                }
                left -= take;
            }
            if (result_len) *result_len = 0;
        }
    } else {
        if (result_len) *result_len = 0;
    }
    rc = status;
out:
    pthread_mutex_unlock(&s_mu);
    return rc;
}

// ────────────────────────────────────────────────────────────────────
// Refresh per-output metadata (dims/quant/type) after each invoke or load.
// ────────────────────────────────────────────────────────────────────
static int refresh_output_meta(void) {
    uint32_t got;
    int n;
    int rc = call_helper(OP_NUM_OUTPUTS, NULL, 0, &n, sizeof(n), &got);
    if (rc < 0 || got != sizeof(n)) return -1;
    if (n > MAX_OUTPUTS) n = MAX_OUTPUTS;
    s_n_outputs = n;
    for (int i = 0; i < n; ++i) {
        uint32_t idx = (uint32_t)i;
        rc = call_helper(OP_OUTPUT_DIMS, &idx, sizeof(idx),
                         s_out_dims[i], sizeof(s_out_dims[i]), &got);
        if (rc < 0) return -2;
        struct { float scale; int32_t zp; } q;
        rc = call_helper(OP_OUTPUT_QUANT, &idx, sizeof(idx),
                         &q, sizeof(q), &got);
        if (rc == 0 && got == sizeof(q)) {
            s_out_scale[i] = q.scale; s_out_zp[i] = q.zp;
        } else {
            s_out_scale[i] = 0.0f; s_out_zp[i] = 0;
        }
        int t;
        rc = call_helper(OP_OUTPUT_TYPE, &idx, sizeof(idx), &t, sizeof(t), &got);
        if (rc == 0) s_out_type[i] = t; else s_out_type[i] = 0;
    }
    return 0;
}

// ────────────────────────────────────────────────────────────────────
// Public API
// ────────────────────────────────────────────────────────────────────
int sentai_tpu_load_model(const char* path) {
    if (!path) return -1;
    int rc = call_helper(OP_LOAD, path, (uint32_t)strlen(path), NULL, 0, NULL);
    if (rc < 0) { s_loaded = 0; return rc; }

    // Refresh input metadata.
    struct __attribute__((packed)) { int32_t d[4]; float scale; int32_t zp; int32_t dtype; } info;
    uint32_t got;
    int rc2 = call_helper(OP_GET_INFO, NULL, 0, &info, sizeof(info), &got);
    if (rc2 < 0 || got != sizeof(info)) return -2;
    memcpy(s_in_dims, info.d, sizeof(info.d));
    s_in_scale = info.scale; s_in_zp = info.zp; s_in_type = info.dtype;

    if (refresh_output_meta() < 0) { s_loaded = 0; return -3; }
    s_loaded = 1;
    return 0;
}

int sentai_tpu_invoke(void) {
    if (!s_loaded) return -1;
    int rc = call_helper(OP_INVOKE, NULL, 0, NULL, 0, NULL);
    if (rc != 0) return rc;
    // After invoke, output dims/types stay the same; only data changes.
    // Pull each output once into our cache for cheap re-reads.
    for (int i = 0; i < s_n_outputs; ++i) {
        uint32_t idx = (uint32_t)i;
        uint32_t got;
        int r = call_helper(OP_GET_OUTPUT, &idx, sizeof(idx),
                            s_out_data[i], sizeof(s_out_data[i]), &got);
        s_out_len[i] = (r == 0) ? (int)got : 0;
    }
    return 0;
}

int sentai_tpu_invoke_with_input(uint8_t* input_buf) {
    if (!s_loaded || !input_buf) return -1;
    // Compute total input bytes from cached dims.
    int n = s_in_dims[0] * s_in_dims[1] * s_in_dims[2] * s_in_dims[3];
    int itemsize = (s_in_type == 1 /*float32*/) ? 4 : 1;
    int total = n * itemsize;
    int rc = call_helper(OP_SET_INPUT, input_buf, (uint32_t)total, NULL, 0, NULL);
    if (rc != 0) return rc;
    return sentai_tpu_invoke();
}

int sentai_tpu_is_ready(void)            { return s_loaded; }
int sentai_tpu_num_outputs(void)         { return s_loaded ? s_n_outputs : 0; }
int sentai_tpu_get_output_size(int idx)  {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return 0;
    return s_out_len[idx];
}
const void* sentai_tpu_get_output_data(int idx) {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return NULL;
    return s_out_data[idx];
}
int sentai_tpu_get_output_num_dims(int idx) {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return 0;
    // Count non-1 dims for stable nDim semantics.
    int n = 4;
    while (n > 1 && s_out_dims[idx][n - 1] == 1) --n;
    return n;
}
int sentai_tpu_get_output_dim(int idx, int dim) {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return 0;
    if (dim < 0 || dim >= 4) return 0;
    return s_out_dims[idx][dim];
}
int sentai_tpu_get_output_type(int idx) {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return 0;
    return s_out_type[idx];
}
int sentai_tpu_input_quant(float* scale, int32_t* zp) {
    if (!s_loaded) return -1;
    if (scale) *scale = s_in_scale;
    if (zp)    *zp = s_in_zp;
    return 0;
}
int sentai_tpu_output_quant(int idx, float* scale, int32_t* zp) {
    if (!s_loaded || idx < 0 || idx >= s_n_outputs) return -1;
    if (scale) *scale = s_out_scale[idx];
    if (zp)    *zp = s_out_zp[idx];
    return 0;
}
int sentai_tpu_input_type(void) { return s_loaded ? s_in_type : 0; }

// ── Tensor info (camera→tensor glue) ────────────────────────────────
int sentai_get_tensor_info(int* w, int* h, int* ch,
                            uint8_t** buf, int* type, int* zp) {
    if (!s_loaded) return -1;
    if (w)  *w  = s_in_dims[2];   // dims = [N, H, W, C]
    if (h)  *h  = s_in_dims[1];
    if (ch) *ch = s_in_dims[3];
    if (type) *type = s_in_type;
    if (zp)   *zp = s_in_zp;
    if (buf)  *buf = NULL;        // SIM doesn't expose direct arena pointer
    return 0;
}
int sentai_cam_to_tensor(void)            { return -1; /* Phase 5.6 */ }
int sentai_cam_to_tensor_ex(const char* p, int q) { (void)p;(void)q; return -1; }

// ── Multi-slot stubs (single-slot for now) ──────────────────────────
int sentai_tpu_slot_count(void)             { return 1; }
int sentai_tpu_slot_ready(int s)            { return (s == 0) ? s_loaded : 0; }
int sentai_tpu_load_model_slot(int s, const char* p) {
    return (s == 0) ? sentai_tpu_load_model(p) : -1;
}
int sentai_tpu_invoke_slot(int s)           { return (s == 0) ? sentai_tpu_invoke() : -1; }
int sentai_tpu_invoke_slot_with_input(int s, uint8_t* b) {
    return (s == 0) ? sentai_tpu_invoke_with_input(b) : -1;
}
int sentai_tpu_num_outputs_slot(int s)      { return (s == 0) ? s_n_outputs : 0; }
int sentai_tpu_get_output_size_slot(int s, int i) {
    return (s == 0) ? sentai_tpu_get_output_size(i) : 0;
}
const void* sentai_tpu_get_output_data_slot(int s, int i) {
    return (s == 0) ? sentai_tpu_get_output_data(i) : NULL;
}
int sentai_tpu_get_output_num_dims_slot(int s, int i) {
    return (s == 0) ? sentai_tpu_get_output_num_dims(i) : 0;
}
int sentai_tpu_get_output_dim_slot(int s, int i, int d) {
    return (s == 0) ? sentai_tpu_get_output_dim(i, d) : 0;
}
int sentai_tpu_get_output_type_slot(int s, int i) {
    return (s == 0) ? sentai_tpu_get_output_type(i) : 0;
}
int sentai_tpu_output_quant_slot(int s, int i, float* sc, int32_t* z) {
    return (s == 0) ? sentai_tpu_output_quant(i, sc, z) : -1;
}
int sentai_tpu_set_input_slot(int s, const uint8_t* d, size_t n) {
    if (s != 0) return -1;
    return call_helper(OP_SET_INPUT, d, (uint32_t)n, NULL, 0, NULL);
}
uint32_t sentai_tpu_output_hash_slot(int s) {
    if (s != 0) return 0;
    uint32_t h = 0; uint32_t got;
    if (call_helper(OP_OUTPUT_HASH, NULL, 0, &h, sizeof(h), &got) < 0) return 0;
    return h;
}
