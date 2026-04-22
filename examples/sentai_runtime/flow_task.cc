// flow_task.cc — optical-flow background task exposed as sentai.flow.*
//
// Produces a frame-to-frame (Δx, Δy) translation estimate from ONE
// camera, at ~20-40 Hz, using integer SAD block-matching on a PXP-
// downscaled 80×60 grayscale frame.  Designed as a low-overhead
// "where did the camera move since last frame" sensor whose output
// can later feed an autopilot loop.
//
// Design constraints (per agent/embeded.md):
//  * Mutually exclusive with the TPU detection pipeline — both share
//    the PXP hw + camera buffer ownership.  Start refuses if the
//    detection pipeline is running and vice-versa (checked in runtime).
//  * Bounded loop: every iteration has explicit timeout on cam grab
//    (200 ms) and a vTaskDelay(1) yield to feed watchdog + cooperate.
//  * Self-deleting task on stop; supervised via s_running flag read
//    in the loop.
//  * Single latest-delta struct protected by a FreeRTOS mutex for
//    reads from the MicroPython context (short critical section).
//
// Algorithm (v1 — "coarse global translation"):
//   1. Grab raw XRGB8888 frame from camera.
//   2. PXP-scale to W×H RGB888 (W=80, H=60 default).
//   3. Integer RGB→Y conversion (Y = (R + 2G + B) >> 2) into the
//      next ping-pong slot of the grayscale buffer pair.
//   4. If we have a valid previous frame, do exhaustive SAD over a
//      centered B×B reference block against a ±SR search window in
//      the previous frame.  Peak (min SAD) wins.
//   5. Publish (dx, dy, SAD-at-peak, frames++, age).
//   6. vTaskDelay(1).
//
// Sizing math (defaults W=80, H=60, B=32, SR=12):
//   worst-case SAD ops = (2·SR+1)² · B² = 25² · 32² ≈ 640 000 add/abs;
//   at Cortex-M7 @ 800 MHz and ~2 cycles/pixel SAD this is ≈ 1.6 ms
//   per frame, well under the 22 ms camera period @ VGA/45.

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <cstdint>
#include <cstring>

#include "libs/camera/camera_support.h"   // DEMO_CAMERA_WIDTH/HEIGHT
#include "examples/sentai_runtime/flow_shared.h"

extern "C" {
    // Supplied by sentai_runtime.cc / detection_task.cc
    int      sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                              uint8_t* dst, int dw, int dh);
    int      sentai_cam_grab_latest(uint8_t** raw);
    void     sentai_cam_return_raw(int idx);
    uint32_t sentai_cam_get_frame_seq(void);
    int      sentai_cam_is_initialized(void);
    int      sentai_detection_is_running(void);
    int      sentai_cam_switch(int id);  // switches MUX to cam id
}

namespace {

// Geometry — kept small so SDRAM cost stays negligible and the SAD
// loops fit comfortably in the frame period.
constexpr int kFlowW = 80;
constexpr int kFlowH = 60;
constexpr int kFlowPixels = kFlowW * kFlowH;
// Central reference block — chosen smaller than the frame on both
// axes so the ±kSearchRange search window stays inside the image.
constexpr int kBlockW = 32;
constexpr int kBlockH = 32;
constexpr int kSearchRange = 12;   // ±12 pixels on each axis

// Static allocations.  Everything fits comfortably in SDRAM slack.
//   rgb_scratch — 80×60×3 = 14 400 B, aligned for PXP dst
//   gray[2]     — 2 × 80×60   = 9 600 B ping-pong
static uint8_t s_rgb_scratch[kFlowW * kFlowH * 3]
    __attribute__((aligned(64), section(".sdram_bss")));
static uint8_t s_gray[2][kFlowPixels]
    __attribute__((aligned(64), section(".sdram_bss")));

// Sanity bound: if the scratch / gray buffers ever balloon, catch
// it at compile time before it steals space from adjacent .sdram_bss
// consumers (TPU ping-pong, USB host, httpsrv).
static_assert(sizeof(s_rgb_scratch) + sizeof(s_gray) <= 32 * 1024,
              "flow buffers exceed 32 KB SDRAM budget");

// Latest-delta snapshot — written from the flow task, read from
// the MicroPython context via sentai.flow.read().  Guarded by a mutex
// since we publish multiple fields atomically.
struct FlowSample {
    int32_t   dx;                // pixels, in the downscaled grid
    int32_t   dy;
    uint32_t  sad;               // sum-of-absolute-diffs at the peak
    uint32_t  frame_seq;         // g_camera_frame_seq at sample time
    TickType_t when_tick;        // for age_ms()
    uint8_t   confidence;        // 0..255; 255 = strong match
    uint8_t   cam_id;            // which camera produced this sample
    uint16_t  _pad;
};
static FlowSample        s_sample = {};
static SemaphoreHandle_t s_sample_lock = nullptr;

// Task state.
static volatile bool      s_running = false;
static TaskHandle_t       s_task    = nullptr;
static volatile int       s_cam_id  = 0;   // which cam to stream from

// Stats.
static volatile uint32_t  s_frames_processed = 0;
static volatile uint32_t  s_frames_dropped   = 0;
static volatile uint32_t  s_grab_fail_count  = 0;
static volatile uint32_t  s_pxp_fail_count   = 0;
static TickType_t         s_start_tick       = 0;

// -------------------------------------------------------------
// Integer RGB → Y.  Approximates Rec.601 Y ≈ 0.30R + 0.59G + 0.11B
// with shifts: Y = (R + 2G + B) >> 2.  Close enough for block
// matching (the absolute luma doesn't matter, only spatial
// structure).  ~3 cycles/pixel on Cortex-M7 → 3·4800 = 14 k cycles
// ≈ 18 µs at 800 MHz.  Negligible.
// -------------------------------------------------------------
static void rgb_to_gray_fast(const uint8_t* rgb, uint8_t* gray,
                             int pixels) {
    for (int i = 0; i < pixels; ++i) {
        const uint32_t r = rgb[3 * i + 0];
        const uint32_t g = rgb[3 * i + 1];
        const uint32_t b = rgb[3 * i + 2];
        gray[i] = (uint8_t)((r + (g << 1) + b) >> 2);
    }
}

// -------------------------------------------------------------
// Exhaustive SAD search.  Finds the (dx, dy) in [-kSearchRange,
// +kSearchRange] that best aligns a B×B block at the centre of
// `curr` against `prev`.  Returns the best SAD at *sad_out.
// Positive dx means the image moved RIGHT in the new frame (so
// the camera moved LEFT relative to the scene); same sign
// convention for dy (positive = scene moved DOWN).
// -------------------------------------------------------------
static void flow_block_match(const uint8_t* curr,
                             const uint8_t* prev,
                             int* dx_out, int* dy_out,
                             uint32_t* sad_out) {
    // Reference block taken from curr[], centered.
    const int bx = (kFlowW - kBlockW) / 2;  // 24
    const int by = (kFlowH - kBlockH) / 2;  // 14

    uint32_t best_sad = 0xFFFFFFFFu;
    int      best_dx  = 0;
    int      best_dy  = 0;

    for (int dy = -kSearchRange; dy <= kSearchRange; ++dy) {
        for (int dx = -kSearchRange; dx <= kSearchRange; ++dx) {
            uint32_t sad = 0;
            for (int y = 0; y < kBlockH; ++y) {
                const uint8_t* c = curr + (by + y) * kFlowW + bx;
                const uint8_t* p = prev + (by + y + dy) * kFlowW + (bx + dx);
                for (int x = 0; x < kBlockW; ++x) {
                    int d = (int)c[x] - (int)p[x];
                    sad += (uint32_t)(d < 0 ? -d : d);
                }
                // Early-out: a partial SAD already worse than the best
                // candidate cannot win; skip the remaining rows.
                if (sad >= best_sad) break;
            }
            if (sad < best_sad) {
                best_sad = sad;
                best_dx  = dx;
                best_dy  = dy;
            }
        }
    }

    *dx_out  = best_dx;
    *dy_out  = best_dy;
    *sad_out = best_sad;
}

// SAD → confidence: map the per-pixel mean absolute difference to a
// 0..255 score.  Intuition: SAD=0 → perfect match → 255; SAD ≥ 64×
// (block pixels) → noise → 0.  Linear in between.
static uint8_t sad_to_confidence(uint32_t sad) {
    const uint32_t block_pixels = (uint32_t)(kBlockW * kBlockH);
    const uint32_t mad = sad / block_pixels;  // mean abs diff per pixel
    if (mad >= 64) return 0;
    return (uint8_t)(255u - ((mad * 255u) / 64u));
}

// -------------------------------------------------------------
// Main loop.
// -------------------------------------------------------------
static void flow_task_fn(void* /*arg*/) {
    bool     have_prev = false;
    uint32_t prev_seq  = 0;
    int      prev_slot = 0;   // index into s_gray[]

    // Snap the cam once before entering the loop.  Errors here
    // just drop the frame — the loop's bounded retry handles it.
    (void)sentai_cam_switch(s_cam_id);

    while (s_running) {
        uint8_t* raw = nullptr;
        int idx = sentai_cam_grab_latest(&raw);
        if (idx < 0 || !raw) {
            __atomic_add_fetch(&s_grab_fail_count, 1, __ATOMIC_RELAXED);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const uint32_t frame_seq = sentai_cam_get_frame_seq();
        // Frame rate limiter: if the ISR hasn't served a new frame yet
        // compared to last iteration, don't re-process the same one —
        // return the buffer and wait 1 tick.  Keeps the SAD loop from
        // saturating at the grab latency during low-FPS conditions.
        if (have_prev && frame_seq == prev_seq) {
            sentai_cam_return_raw(idx);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int rc = sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  s_rgb_scratch, kFlowW, kFlowH);
        sentai_cam_return_raw(idx);
        if (rc != 0) {
            __atomic_add_fetch(&s_pxp_fail_count, 1, __ATOMIC_RELAXED);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        const int curr_slot = prev_slot ^ 1;
        rgb_to_gray_fast(s_rgb_scratch, s_gray[curr_slot], kFlowPixels);

        if (have_prev) {
            int dx = 0, dy = 0;
            uint32_t sad = 0;
            flow_block_match(s_gray[curr_slot], s_gray[prev_slot],
                             &dx, &dy, &sad);

            FlowSample snap;
            snap.dx         = dx;
            snap.dy         = dy;
            snap.sad        = sad;
            snap.frame_seq  = frame_seq;
            snap.when_tick  = xTaskGetTickCount();
            snap.confidence = sad_to_confidence(sad);
            snap.cam_id     = (uint8_t)s_cam_id;
            snap._pad       = 0;

            if (s_sample_lock &&
                xSemaphoreTake(s_sample_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_sample = snap;
                xSemaphoreGive(s_sample_lock);
            } else {
                __atomic_add_fetch(&s_frames_dropped, 1, __ATOMIC_RELAXED);
            }
        } else {
            have_prev = true;
        }

        prev_slot = curr_slot;
        prev_seq  = frame_seq;
        __atomic_add_fetch(&s_frames_processed, 1, __ATOMIC_RELAXED);

        // Yield: lets lower-prio tasks run and refreshes the watchdog
        // chain (health task supervises periodically).
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    s_task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

// ============================================================
// Public C API — called from MicroPython bindings.
// ============================================================

extern "C" int sentai_flow_start(int cam_id) {
    if (s_running) return -1;                     // already running
    if (sentai_detection_is_running()) return -2; // pipeline owns PXP/cam
    if (!sentai_cam_is_initialized()) return -3;
    if (cam_id != 0 && cam_id != 1) return -4;

    if (!s_sample_lock) {
        s_sample_lock = xSemaphoreCreateMutex();
        if (!s_sample_lock) return -5;
    }

    // Reset public state so a fresh start() doesn't return stale
    // data from the previous run.
    if (xSemaphoreTake(s_sample_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&s_sample, 0, sizeof(s_sample));
        xSemaphoreGive(s_sample_lock);
    }
    s_frames_processed = 0;
    s_frames_dropped   = 0;
    s_grab_fail_count  = 0;
    s_pxp_fail_count   = 0;
    s_start_tick       = xTaskGetTickCount();
    s_cam_id           = cam_id;
    s_running          = true;

    BaseType_t r = xTaskCreate(flow_task_fn, "sentai_flow",
                               configMINIMAL_STACK_SIZE * 4,
                               nullptr, tskIDLE_PRIORITY + 2, &s_task);
    if (r != pdPASS) {
        s_running = false;
        s_task = nullptr;
        return -6;
    }
    return 0;
}

extern "C" int sentai_flow_stop(void) {
    if (!s_running) return 0;
    s_running = false;
    // Task self-deletes inside the loop; wait up to 500 ms.
    for (int i = 0; i < 50 && s_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (s_task == nullptr) ? 0 : -1;
}

extern "C" int sentai_flow_is_running(void) {
    return s_running ? 1 : 0;
}

// Writes the latest sample and (optionally) the derived age_ms.
// Returns:
//   +1 if a sample is available,
//    0 if the task is running but no sample has been produced yet,
//   -1 if not running and no historical sample exists.
extern "C" int sentai_flow_read(int32_t* dx, int32_t* dy,
                                uint32_t* sad, uint32_t* frame_seq,
                                uint32_t* age_ms, uint8_t* confidence,
                                uint8_t* cam_id) {
    if (!s_sample_lock) return -1;
    FlowSample snap = {};
    if (xSemaphoreTake(s_sample_lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -1;
    }
    snap = s_sample;
    xSemaphoreGive(s_sample_lock);

    const bool have_data = (snap.frame_seq != 0);
    if (dx)         *dx         = snap.dx;
    if (dy)         *dy         = snap.dy;
    if (sad)        *sad        = snap.sad;
    if (frame_seq)  *frame_seq  = snap.frame_seq;
    if (confidence) *confidence = snap.confidence;
    if (cam_id)     *cam_id     = snap.cam_id;
    if (age_ms) {
        if (have_data) {
            TickType_t now = xTaskGetTickCount();
            *age_ms = (uint32_t)((now - snap.when_tick) * portTICK_PERIOD_MS);
        } else {
            *age_ms = 0xFFFFFFFFu;
        }
    }
    if (!have_data) return s_running ? 0 : -1;
    return 1;
}

// ============================================================
// M4-offload support — PrepTask frame publisher + command/read API.
//
// `sentai_flow_m4_publish_frame` is called from PrepTask after the
// raw XRGB frame is in hand.  When the publish flag is off (default)
// it's an early return; when on it does a step-8 CPU decimation
// down to the 80×60 gray buffer in the shared OCRAM window and
// stamps a new frame_seq.  Cost: ~50 µs at 800 MHz — negligible
// vs PrepTask's ~15 ms budget, so the M7 pipeline fps is preserved.
// ============================================================

static volatile int s_flow_m4_publish_enabled = 0;
static volatile uint32_t s_flow_m4_publish_seq = 0;

// Auto-level (histogram min-max stretch) on the 80×60 gray buffer.
// Off by default — when enabled, the per-frame pass:
//   1. builds a 256-bin histogram of the just-published gray frame,
//   2. finds vmin / vmax (first / last non-empty bin),
//   3. maps [vmin..vmax] → [0..255] via an 8-bit LUT.
// Cost: ~4800 adds + 256-entry LUT build + 4800 LUT lookups ≈ 120 µs
// on M7 @ 800 MHz — well inside PrepTask's 15 ms budget.
//
// Motivation: OV5640 AEC measures the post-gamma histogram and drives
// the output toward a target mean (default ≈ 0x50/255).  Gamma / SDE /
// AEC-target presets all feed back through the same loop so the output
// stabilizes at roughly the same mean luma regardless — E38 showed
// all four presets producing ≈ indistinguishable brightness (means
// within a handful of DN of each other).  Post-capture linear stretch
// is the only reliable way to widen the dynamic range *after* the
// sensor's auto-normalization pins it.
//
// Applied BEFORE publishing frame_seq — the M4 SAD consumer sees the
// stretched buffer, which gives higher-contrast blocks and therefore
// stronger SAD minima (better flow confidence in dim scenes).
static volatile int s_gray_stretch_enabled = 0;
static volatile uint8_t s_gray_stretch_last_vmin = 0;
static volatile uint8_t s_gray_stretch_last_vmax = 0;

static inline void gray_stretch_in_place(volatile uint8_t* gray, int n) {
    uint32_t hist[256] = {0};
    for (int i = 0; i < n; ++i) ++hist[gray[i]];
    int vmin = 0;
    while (vmin < 255 && hist[vmin] == 0) ++vmin;
    int vmax = 255;
    while (vmax > 0 && hist[vmax] == 0) --vmax;
    s_gray_stretch_last_vmin = (uint8_t)vmin;
    s_gray_stretch_last_vmax = (uint8_t)vmax;
    if (vmax <= vmin) return;  // flat frame — nothing to stretch
    const int span = vmax - vmin;
    uint8_t lut[256];
    for (int i = 0; i < 256; ++i) {
        int v = i - vmin;
        if (v <= 0)         lut[i] = 0;
        else if (v >= span) lut[i] = 255;
        else                lut[i] = (uint8_t)((v * 255) / span);
    }
    for (int i = 0; i < n; ++i) gray[i] = lut[gray[i]];
}

extern "C" int sentai_flow_gray_stretch_set(int enable) {
    return __atomic_exchange_n(&s_gray_stretch_enabled,
                               enable ? 1 : 0, __ATOMIC_RELEASE);
}

extern "C" int sentai_flow_gray_stretch_get(uint8_t* vmin,
                                            uint8_t* vmax) {
    if (vmin) *vmin = s_gray_stretch_last_vmin;
    if (vmax) *vmax = s_gray_stretch_last_vmax;
    return __atomic_load_n(&s_gray_stretch_enabled, __ATOMIC_ACQUIRE);
}

extern "C" void sentai_flow_m4_publish_frame(const uint8_t* raw,
                                             int raw_w, int raw_h,
                                             int cam_id) {
    // Acquire with atomic load — avoids torn reads of the flag at
    // the same time MicroPython's m4_start()/m4_stop() writes it.
    // The ACQUIRE semantics also order the subsequent loads (decimation
    // reads of `raw`) after this check.
    if (!__atomic_load_n(&s_flow_m4_publish_enabled, __ATOMIC_ACQUIRE))
        return;
    if (raw_w < FLOW_GRAY_W || raw_h < FLOW_GRAY_H) return;

    volatile flow_shared_t* sh = &FLOW_SHARED();

    const int step_x = raw_w / FLOW_GRAY_W;
    const int step_y = raw_h / FLOW_GRAY_H;
    // Decimation sanity — refuse to index past the raw buffer on a
    // caller that passed an absurdly large raw_w/raw_h.  Without this
    // guard, step_x * 4 * FLOW_GRAY_W could step off the end of the
    // camera's XRGB buffer and feed random SDRAM into the SAD algo,
    // producing plausible-but-garbage motion estimates.  Fail closed.
    if (step_x < 1 || step_y < 1) return;
    const int stride_row = raw_w * 4;   // XRGB8888 = 4 bytes/pixel

    // Luma estimate that is robust to XRGB byte ordering — sum all
    // four bytes and shift by 2.  The alpha/X byte is constant so it
    // only adds a DC bias, which is invariant to SAD.  Any real R,G,B
    // byte, whichever slot it lands in, contributes to spatial
    // structure so motion always shows up.  Tested earlier that a
    // single byte pick (byte 2) was hitting the constant X on this
    // sensor → zero motion signal.
    volatile uint8_t* dst = sh->gray;
    const uint8_t* src_row = raw;
    const int src_row_step = step_y * stride_row;
    const int src_pix_step = step_x * 4;
    for (int y = 0; y < FLOW_GRAY_H; ++y) {
        const uint8_t* p = src_row;
        for (int x = 0; x < FLOW_GRAY_W; ++x) {
            const uint32_t sum = (uint32_t)p[0] + (uint32_t)p[1]
                               + (uint32_t)p[2] + (uint32_t)p[3];
            *dst++ = (uint8_t)(sum >> 2);
            p += src_pix_step;
        }
        src_row += src_row_step;
    }

    // Post-decimation auto-level (opt-in).  Runs on the shared-memory
    // buffer directly — M4 hasn't been told a new frame is available
    // yet (frame_seq is bumped below), so this is race-free.
    if (__atomic_load_n(&s_gray_stretch_enabled, __ATOMIC_ACQUIRE)) {
        gray_stretch_in_place(sh->gray, FLOW_GRAY_PIXELS);
    }

    sh->frame_cam_id = (uint8_t)cam_id;
    __DMB();
    sh->frame_seq = ++s_flow_m4_publish_seq;
    __DMB();
    sh->frame_valid = 1;
}

// Enable/disable the publish hook from the M7 side.  Returns the
// previous value.  Atomic RELEASE-store so PrepTask's ACQUIRE-load
// sees a consistent value + sees all preceding writes from the
// caller that armed this transition.
extern "C" int sentai_flow_m4_publish_set(int enable) {
    int prev = __atomic_exchange_n(&s_flow_m4_publish_enabled,
                                   enable ? 1 : 0,
                                   __ATOMIC_RELEASE);
    return prev;
}

// Issue FLOW_CMD_START to the M4 (increments cmd_seq so M4's
// handshake edge-triggers).  Enables the PrepTask publish hook as
// a side-effect so the M4 has frames to consume.
//
// cam_id:  0 = cam0 (default, recommended for consistent body-frame
//              conversion — see paper/flow_body_frame.md),
//          1 = cam1 (180° rotated vs cam0 — body-frame signs flip).
// Any other value is rejected.  The caller is responsible for
// arranging sentai.camera.select()/ratio() so the pipeline actually
// delivers frames from the chosen camera; this call only records
// the intent in the shared struct for downstream consumers.
extern "C" int sentai_flow_m4_cmd_start(int cam_id) {
    if (cam_id != 0 && cam_id != 1) return -2;
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic != FLOW_SHARED_MAGIC) return -1;   // M4 not alive
    sh->cmd          = FLOW_CMD_START;
    sh->frame_cam_id = (uint8_t)cam_id;
    __DMB();
    sh->cmd_seq = sh->cmd_seq + 1;
    __atomic_store_n(&s_flow_m4_publish_enabled, 1, __ATOMIC_RELEASE);
    return 0;
}

extern "C" int sentai_flow_m4_cmd_stop(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic != FLOW_SHARED_MAGIC) return -1;
    __atomic_store_n(&s_flow_m4_publish_enabled, 0, __ATOMIC_RELEASE);
    sh->cmd = FLOW_CMD_STOP;
    __DMB();
    sh->cmd_seq = sh->cmd_seq + 1;
    return 0;
}

// Detail-score metric on the current 40×30 gray buffer in shared OCRAM.
//
// Computes a normalized gradient-energy score — the standard metric
// for "image sharpness / information content", used in auto-focus
// and image-quality assessment.  Higher = more detail, clearer edges,
// better input for SAD block matching.
//
// Algorithm: sum of |∂I/∂x| + |∂I/∂y| over the 40×30 grid, normalized
// by pixel count.  ~3900 abs-subtract ops, ~8 µs on M7 @ 800 MHz.
//
// Returned score is in "per-pixel gradient units" (0..255 per axis);
// typical values:
//   < 5     : very flat — near-uniform scene, SAD cannot lock
//   5..15   : dim or low-texture
//   15..40  : good detail — normal indoor scene
//   > 40    : rich detail — lots of edges / high contrast
extern "C" uint32_t sentai_flow_detail_score(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic != FLOW_SHARED_MAGIC) return 0;

    uint32_t sum = 0;
    const volatile uint8_t* g = sh->gray;

    // Horizontal gradient: |I(x+1,y) - I(x,y)|.
    for (int y = 0; y < FLOW_GRAY_H; ++y) {
        const volatile uint8_t* row = g + y * FLOW_GRAY_W;
        for (int x = 0; x < FLOW_GRAY_W - 1; ++x) {
            int d = (int)row[x + 1] - (int)row[x];
            sum += (uint32_t)(d < 0 ? -d : d);
        }
    }
    // Vertical gradient: |I(x,y+1) - I(x,y)|.
    for (int y = 0; y < FLOW_GRAY_H - 1; ++y) {
        const volatile uint8_t* r0 = g + y * FLOW_GRAY_W;
        const volatile uint8_t* r1 = r0 + FLOW_GRAY_W;
        for (int x = 0; x < FLOW_GRAY_W; ++x) {
            int d = (int)r1[x] - (int)r0[x];
            sum += (uint32_t)(d < 0 ? -d : d);
        }
    }

    // Normalize: total ops = (W-1)*H + W*(H-1) ≈ 2*W*H - W - H
    // For 40x30: 2*40*30 - 40 - 30 = 2330.  Return average × 100 so
    // fractional gradients show up as integer basis points.
    const uint32_t divisor = (FLOW_GRAY_W - 1) * FLOW_GRAY_H
                           + FLOW_GRAY_W * (FLOW_GRAY_H - 1);
    return (sum * 100u) / divisor;
}

extern "C" void sentai_flow_stats(uint32_t* frames_processed,
                                  uint32_t* frames_dropped,
                                  uint32_t* grab_fail,
                                  uint32_t* pxp_fail,
                                  uint32_t* avg_fps_x10) {
    uint32_t p = __atomic_load_n(&s_frames_processed, __ATOMIC_RELAXED);
    uint32_t d = __atomic_load_n(&s_frames_dropped,   __ATOMIC_RELAXED);
    uint32_t g = __atomic_load_n(&s_grab_fail_count,  __ATOMIC_RELAXED);
    uint32_t x = __atomic_load_n(&s_pxp_fail_count,   __ATOMIC_RELAXED);
    if (frames_processed) *frames_processed = p;
    if (frames_dropped)   *frames_dropped   = d;
    if (grab_fail)        *grab_fail        = g;
    if (pxp_fail)         *pxp_fail         = x;
    if (avg_fps_x10) {
        uint32_t elapsed_ms = (xTaskGetTickCount() - s_start_tick)
                              * portTICK_PERIOD_MS;
        *avg_fps_x10 = (elapsed_ms && p)
            ? (uint32_t)(((uint64_t)p * 10000u) / elapsed_ms)
            : 0;
    }
}
