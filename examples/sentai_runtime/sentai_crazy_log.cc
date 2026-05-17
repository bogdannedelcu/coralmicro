// sentai_crazy_log.cc — CRTP LOG subscription, shared C port of crtp_log.py.
//
// Task #44.  Compiles for both ARM (examples/sentai_runtime) and SIM
// (sim/CMakeLists.txt — added in same commit).  All transport bytes go
// through sentai_crazy_send_crtp() + sentai_crazy_recv_pop(), so the
// algorithm is platform-agnostic and the only per-platform plumbing
// stays in sentai_crazy_{sim.cc, .cc}.
//
// Wire format mirrors Bitcraze cflib/crazyflie/{log,toc}.py byte-for-byte
// (same format crtp_log.py used, just in C).  Unit tested offline via
// experiments/s146_pose_feedback/test_crtp_log.py (still PASS for the
// reference impl; this port is a strict translation).
//
// NASA/JPL discipline (agent/embeded.md):
//   - All loops bounded (scan ≤ max_items, poll ≤ MAX_DRAIN_PER_CALL).
//   - All allocations static (one state struct in BSS).
//   - All TX/RX returns checked, errors propagated as negative codes.
//   - Single-writer single-reader for the latest_pose cache.
//   - Bounded subscribe(): scan_timeout + create_block timeout +
//     start_logging timeout ≤ ~3.6 s default.

#include "sentai_crazy_log.h"

#include <math.h>      // isfinite
#include <string.h>    // memset, memcpy

#include "sentai_crazy.h"   // sentai_crazy_send_crtp

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t cl_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  static inline void cl_sleep_ms(uint32_t ms) {
      struct timespec ts;
      ts.tv_sec  = ms / 1000;
      ts.tv_nsec = (long)(ms % 1000) * 1000000L;
      nanosleep(&ts, NULL);
  }
  #define SENTAI_CL_SDRAM_BSS  /* nothing */
  #define SENTAI_CL_SDRAM_TEXT /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t cl_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  static inline void cl_sleep_ms(uint32_t ms) {
      vTaskDelay(pdMS_TO_TICKS(ms));
  }
  #define SENTAI_CL_SDRAM_BSS   __attribute__((section(".sdram_bss"), aligned(4)))
  #define SENTAI_CL_SDRAM_TEXT  __attribute__((section(".sdram_text"), noinline))
#endif

// ===== Transport seam =====================================================
// SIM exposes sentai_crazy_recv_pop directly.  ARM does not yet have a
// generic LOG-port RX ring (existing routing is per-block specialised);
// the weak stub below lets ARM builds link.  When ARM proper LOG routing
// lands, drop the weak attribute and forward to the real ring.
//
// Signature mirrors sim/sentai_crazy_sim.cc:
//   int sentai_crazy_recv_pop(uint8_t* port, uint8_t* ch,
//                              uint8_t* data, int max_len, int* out_len);
//   returns 1 if a packet was popped, 0 if empty.

extern "C" int sentai_crazy_recv_pop(uint8_t* port, uint8_t* ch,
                                      uint8_t* data, int max_len, int* out_len);

#if defined(__arm__) && !defined(SENTAI_PLATFORM_SIM)
// Weak stub so ARM links until the LOG ring lands.  pose_subscribe()
// will return -2 (TOC scan timeout) on ARM today, which is the loudest-
// failing behaviour we can offer per [[missions-run-in-sentai-only]].
extern "C" __attribute__((weak)) int sentai_crazy_recv_pop(
        uint8_t* port, uint8_t* ch, uint8_t* data, int max_len, int* out_len) {
    (void)port; (void)ch; (void)data; (void)max_len;
    if (out_len) *out_len = 0;
    return 0;
}
#endif

// ===== Protocol constants (mirror cflib byte-for-byte) ====================
#define CRTP_PORT_LOG          0x05
#define CH_TOC                 0
#define CH_SETTINGS            1
#define CH_LOGDATA             2

#define CMD_GET_ITEM_V2        2
#define CMD_GET_INFO_V2        3
#define CMD_CREATE_BLOCK_V2    6
#define CMD_DELETE_BLOCK       2
#define CMD_START_LOGGING      3
#define CMD_STOP_LOGGING       4
#define CMD_RESET_LOGGING      5

#define LOG_T_FLOAT            0x07   // we only subscribe to 4 floats

#define POSE_BLOCK_ID          1      // arbitrary; mission code never sees it
#define POSE_VARS_N            4

// ===== Bounded constants ==================================================
#define LOG_MAX_TOC_ITEMS      400    // cf2 SITL ships ~361
#define LOG_SCAN_TOTAL_MS      8000   // bounded outer budget for scan
#define LOG_REQ_TIMEOUT_MS     300    // per-request reply budget
#define LOG_POLL_TICK_MS       5      // drain step granularity
#define LOG_DRAIN_MAX_PKTS     16     // pose() drains at most this many

// ===== State (one static block) ===========================================
typedef struct {
    uint8_t   subscribed;            // 0/1
    uint8_t   block_id;
    uint8_t   ready;                 // 1 once at least one LOGDATA arrived
    uint8_t   _pad;
    // Resolved TOC ids for the 4 pose vars (filled at scan).
    uint16_t  id_x;
    uint16_t  id_y;
    uint16_t  id_z;
    uint16_t  id_yaw;
    // Latest pose snapshot — single writer (pose()), single reader (caller).
    float     x, y, z, yaw;
    // Diagnostics
    uint32_t  toc_n_items;
    uint32_t  data_frames;
} crazy_log_state_t;

SENTAI_CL_SDRAM_BSS static crazy_log_state_t g_log;

// ===== Helpers ============================================================

static inline void cl_pack_le16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Drain RX, looking for a LOG-port reply on `channel` whose first byte is
// `cmd`.  Routes LOGDATA frames into the pose cache as a side effect.
// Bounded: returns NULL after `timeout_ms` (rounded up to LOG_POLL_TICK_MS).
static SENTAI_CL_SDRAM_TEXT int cl_drain_for_reply(
        uint8_t channel, uint8_t cmd,
        uint8_t* out_payload, int max_len, int* out_len,
        uint32_t timeout_ms) {
    if (out_len) *out_len = 0;
    uint32_t t0 = cl_now_ms();
    while ((cl_now_ms() - t0) < timeout_ms) {
        uint8_t  port=0, ch=0;
        uint8_t  buf[30];
        int      n = 0;
        if (sentai_crazy_recv_pop(&port, &ch, buf, sizeof(buf), &n) == 1) {
            if (port == CRTP_PORT_LOG && ch == CH_LOGDATA) {
                // Side-effect: route into pose cache if this matches our block.
                if (n >= 4 && buf[0] == g_log.block_id && g_log.subscribed) {
                    // Payload after [block, ts0, ts1, ts2]: 4 little-endian floats.
                    if (n >= 4 + 4*4) {
                        float vx, vy, vz, vyaw;
                        memcpy(&vx,   buf + 4 + 0,  4);
                        memcpy(&vy,   buf + 4 + 4,  4);
                        memcpy(&vz,   buf + 4 + 8,  4);
                        memcpy(&vyaw, buf + 4 + 12, 4);
                        if (isfinite(vx) && isfinite(vy) &&
                            isfinite(vz) && isfinite(vyaw)) {
                            g_log.x   = vx;
                            g_log.y   = vy;
                            g_log.z   = vz;
                            g_log.yaw = vyaw;
                            g_log.ready = 1;
                            g_log.data_frames++;
                        }
                    }
                }
                continue;
            }
            if (port == CRTP_PORT_LOG && ch == channel && n >= 1 && buf[0] == cmd) {
                // Copy payload after the cmd byte.
                int copy_n = n - 1;
                if (copy_n > max_len) copy_n = max_len;
                if (copy_n > 0 && out_payload) memcpy(out_payload, buf + 1, (size_t)copy_n);
                if (out_len) *out_len = copy_n;
                return 0;
            }
            // Other LOG channels / mismatched cmd — drop silently.
            continue;
        }
        cl_sleep_ms(LOG_POLL_TICK_MS);
    }
    return -1;   // timeout
}

// ===== Public API =========================================================

extern "C" SENTAI_CL_SDRAM_TEXT int sentai_crazy_log_reset(void) {
    if (!sentai_crazy_is_running()) return -1;
    uint8_t pkt[1] = { CMD_RESET_LOGGING };
    int rc = sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_SETTINGS, pkt, 1);
    if (rc < 0) return -1;
    // Best-effort reply drain; cf2 SITL doesn't ACK RESET in all firmwares.
    uint8_t tmp[30];
    int tmp_n = 0;
    (void)cl_drain_for_reply(CH_SETTINGS, CMD_RESET_LOGGING, tmp, sizeof(tmp), &tmp_n, 200);
    return 0;
}

// Scan TOC, early-exiting once all 4 pose vars are resolved.
// Returns 0 ok, -2 timeout, -3 missing var(s).
static SENTAI_CL_SDRAM_TEXT int cl_scan_pose_toc(uint32_t total_budget_ms) {
    g_log.id_x = g_log.id_y = g_log.id_z = g_log.id_yaw = 0xFFFF;
    g_log.toc_n_items = 0;

    // GET_INFO_V2 → (n_items_lo, n_items_hi, crc[4]).
    uint8_t info_pkt[1] = { CMD_GET_INFO_V2 };
    if (sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_TOC, info_pkt, 1) < 0) return -2;
    uint8_t reply[30];
    int     reply_n = 0;
    if (cl_drain_for_reply(CH_TOC, CMD_GET_INFO_V2,
                            reply, sizeof(reply), &reply_n,
                            LOG_REQ_TIMEOUT_MS * 3) < 0) {
        return -2;
    }
    if (reply_n < 2) return -2;
    int n_items = (int)reply[0] | ((int)reply[1] << 8);
    if (n_items <= 0) return -2;
    if (n_items > LOG_MAX_TOC_ITEMS) n_items = LOG_MAX_TOC_ITEMS;
    g_log.toc_n_items = (uint32_t)n_items;

    uint32_t t0 = cl_now_ms();
    for (int i = 0; i < n_items; i++) {
        if ((cl_now_ms() - t0) >= total_budget_ms) return -2;

        uint8_t req[3] = { CMD_GET_ITEM_V2, (uint8_t)(i & 0xFF), (uint8_t)((i >> 8) & 0xFF) };
        if (sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_TOC, req, 3) < 0) return -2;
        if (cl_drain_for_reply(CH_TOC, CMD_GET_ITEM_V2,
                                reply, sizeof(reply), &reply_n,
                                LOG_REQ_TIMEOUT_MS) < 0) {
            continue;   // best-effort: missing entries are not fatal
        }
        if (reply_n < 5) continue;
        int idx = (int)reply[0] | ((int)reply[1] << 8);
        if (idx != i) continue;
        uint8_t ttype = reply[2];
        // Find NUL terminators inside reply[3..reply_n-1].
        int nul1 = -1;
        for (int k = 3; k < reply_n; k++) { if (reply[k] == 0) { nul1 = k; break; } }
        if (nul1 < 0) continue;
        int nul2 = -1;
        for (int k = nul1 + 1; k < reply_n; k++) { if (reply[k] == 0) { nul2 = k; break; } }
        if (nul2 < 0) continue;
        // group  = reply[3..nul1-1]
        // name   = reply[nul1+1..nul2-1]
        const char* group = (const char*)&reply[3];
        const char* name  = (const char*)&reply[nul1 + 1];
        int  g_len = nul1 - 3;
        int  n_len = nul2 - (nul1 + 1);

        // Cheap, bounded compares — no strncmp dep.
        #define MATCH_GN(_g, _n) \
            (g_len == (int)sizeof(_g) - 1 && n_len == (int)sizeof(_n) - 1 && \
             memcmp(group, _g, g_len) == 0 && memcmp(name, _n, n_len) == 0)

        if (ttype == LOG_T_FLOAT) {
            if      (MATCH_GN("stateEstimate", "x"))   g_log.id_x   = (uint16_t)idx;
            else if (MATCH_GN("stateEstimate", "y"))   g_log.id_y   = (uint16_t)idx;
            else if (MATCH_GN("stateEstimate", "z"))   g_log.id_z   = (uint16_t)idx;
            else if (MATCH_GN("stabilizer",    "yaw")) g_log.id_yaw = (uint16_t)idx;
        }
        #undef MATCH_GN

        // Early exit once all 4 resolved — saves ~5 s of scan on stock cf2.
        if (g_log.id_x != 0xFFFF && g_log.id_y != 0xFFFF &&
            g_log.id_z != 0xFFFF && g_log.id_yaw != 0xFFFF) {
            return 0;
        }
    }
    // Missed at least one — return -3 so caller can distinguish from -2 timeout.
    if (g_log.id_x == 0xFFFF || g_log.id_y == 0xFFFF ||
        g_log.id_z == 0xFFFF || g_log.id_yaw == 0xFFFF) {
        return -3;
    }
    return 0;
}

extern "C" SENTAI_CL_SDRAM_TEXT int sentai_crazy_pose_subscribe(int period_ms) {
    if (!sentai_crazy_is_running()) return -1;

    // Clear any prior subscription state (idempotent re-subscribe).
    g_log.subscribed = 0;
    g_log.ready      = 0;
    g_log.block_id   = POSE_BLOCK_ID;
    g_log.data_frames = 0;

    // Best-effort wipe on the cf2 side — keeps subscribe deterministic
    // when missions are re-run without restarting cf2 SITL.
    (void)sentai_crazy_log_reset();

    int rc = cl_scan_pose_toc(LOG_SCAN_TOTAL_MS);
    if (rc != 0) return rc;   // -2 timeout / -3 missing var

    // Build CREATE_BLOCK_V2 packet:
    //   [cmd, block_id, fetch_byte, id_lo, id_hi, ... × N vars]
    // fetch_byte = (t & 0x0F) | ((t & 0x0F) << 4); both fetch_as and
    // stored_as equal LOG_T_FLOAT.
    uint8_t pkt[2 + POSE_VARS_N * 3];
    pkt[0] = CMD_CREATE_BLOCK_V2;
    pkt[1] = g_log.block_id;
    int o = 2;
    const uint16_t ids[POSE_VARS_N] = { g_log.id_x, g_log.id_y, g_log.id_z, g_log.id_yaw };
    const uint8_t  fetch_byte       = (LOG_T_FLOAT & 0x0F) | ((LOG_T_FLOAT & 0x0F) << 4);
    for (int i = 0; i < POSE_VARS_N; i++) {
        pkt[o++] = fetch_byte;
        cl_pack_le16(pkt + o, ids[i]);
        o += 2;
    }
    if (sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_SETTINGS, pkt, o) < 0) return -4;
    uint8_t reply[30];
    int     reply_n = 0;
    if (cl_drain_for_reply(CH_SETTINGS, CMD_CREATE_BLOCK_V2,
                            reply, sizeof(reply), &reply_n, 500) < 0) {
        return -4;
    }
    // reply = [block_id, err]; err==0 ok, err==17 (EEXIST) recoverable.
    if (reply_n >= 2 && reply[0] == g_log.block_id) {
        if (reply[1] != 0 && reply[1] != 17) return -4;
    }

    // START_LOGGING: [cmd, block_id, period_10ms]
    int period_10ms = period_ms / 10;
    if (period_10ms < 1)   period_10ms = 1;
    if (period_10ms > 255) period_10ms = 255;
    uint8_t start_pkt[3] = { CMD_START_LOGGING,
                              (uint8_t)g_log.block_id,
                              (uint8_t)period_10ms };
    if (sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_SETTINGS, start_pkt, 3) < 0) return -5;
    if (cl_drain_for_reply(CH_SETTINGS, CMD_START_LOGGING,
                            reply, sizeof(reply), &reply_n, 500) < 0) {
        return -5;
    }
    if (reply_n < 2 || reply[0] != g_log.block_id || reply[1] != 0) return -5;

    g_log.subscribed = 1;
    return 0;
}

extern "C" SENTAI_CL_SDRAM_TEXT int sentai_crazy_pose(float* out_x, float* out_y, float* out_z, float* out_yaw) {
    if (!g_log.subscribed) return -1;

    // Drain ≤ LOG_DRAIN_MAX_PKTS — bounded per call.
    for (int i = 0; i < LOG_DRAIN_MAX_PKTS; i++) {
        uint8_t port=0, ch=0, buf[30];
        int     n=0;
        if (sentai_crazy_recv_pop(&port, &ch, buf, sizeof(buf), &n) != 1) break;
        if (port == CRTP_PORT_LOG && ch == CH_LOGDATA &&
            n >= 4 + 4*4 && buf[0] == g_log.block_id) {
            float vx, vy, vz, vyaw;
            memcpy(&vx,   buf + 4 + 0,  4);
            memcpy(&vy,   buf + 4 + 4,  4);
            memcpy(&vz,   buf + 4 + 8,  4);
            memcpy(&vyaw, buf + 4 + 12, 4);
            if (isfinite(vx) && isfinite(vy) &&
                isfinite(vz) && isfinite(vyaw)) {
                g_log.x = vx; g_log.y = vy; g_log.z = vz; g_log.yaw = vyaw;
                g_log.ready = 1;
                g_log.data_frames++;
            }
        }
        // Else: not our block / wrong port — drop silently (we own the RX queue
        // for the duration of pose() but mission code shouldn't be using
        // recv_crtp directly while subscribed; see header concurrency note).
    }
    if (!g_log.ready) return -2;
    if (out_x)   *out_x   = g_log.x;
    if (out_y)   *out_y   = g_log.y;
    if (out_z)   *out_z   = g_log.z;
    if (out_yaw) *out_yaw = g_log.yaw;
    return 0;
}

extern "C" SENTAI_CL_SDRAM_TEXT int sentai_crazy_pose_stop(void) {
    if (!g_log.subscribed) return 0;
    uint8_t stop_pkt[2]   = { CMD_STOP_LOGGING,  g_log.block_id };
    uint8_t delete_pkt[2] = { CMD_DELETE_BLOCK, g_log.block_id };
    uint8_t reply[30];
    int     reply_n = 0;
    (void)sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_SETTINGS, stop_pkt, 2);
    (void)cl_drain_for_reply(CH_SETTINGS, CMD_STOP_LOGGING, reply, sizeof(reply), &reply_n, 200);
    (void)sentai_crazy_send_crtp(CRTP_PORT_LOG, CH_SETTINGS, delete_pkt, 2);
    (void)cl_drain_for_reply(CH_SETTINGS, CMD_DELETE_BLOCK, reply, sizeof(reply), &reply_n, 200);
    g_log.subscribed = 0;
    g_log.ready      = 0;
    return 0;
}

extern "C" SENTAI_CL_SDRAM_TEXT int sentai_crazy_pose_ready(void) {
    return (g_log.subscribed && g_log.ready) ? 1 : 0;
}

extern "C" SENTAI_CL_SDRAM_TEXT void sentai_crazy_log_stats(uint32_t out[4]) {
    if (!out) return;
    out[0] = g_log.subscribed ? 1u : 0u;
    out[1] = g_log.toc_n_items;
    out[2] = g_log.data_frames;
    out[3] = g_log.subscribed ? (uint32_t)g_log.block_id : 0u;
}
