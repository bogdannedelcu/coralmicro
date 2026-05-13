// sentai_aruco_shim_arm.cc — ARM implementation of the ArUco anchor
// shim.  STUB for now (build #1278): returns detected=0 unconditionally.
//
// Full M7 detector is s112+ work.  Building blocks already shipped
// in s111 (build #1278):
//   - PXP HW downscale + Y8 threshold      (1.1 ms / frame)
//   - 3×3 Sobel edge magnitude             (3.0 ms / frame)
//   - PXP HW axis-aligned rectify 60×60→32×32 (37 µs / marker)
//   - CMSIS-DSP q7 BasicMath + Statistics  (linked, ready)
//
// Remaining to build: Suzuki-Abe contour finder → quad approx →
// 4×4 bit-pattern decode → PnP pose solve (fixed-point via
// CMSIS-DSP MatrixFunctions).  Total estimated budget ~5 ms/frame
// (within the 30 ms camera-rate slack).
//
// API contract is in sentai_aruco_shim.h.  Keep this stub honest:
// when SENTAI_ARUCO_ARM_ENABLED is false (the build flag we'll add
// when real detection ships), `detected` stays 0 and downstream
// VPE forwarding correctly degrades to "no anchor available".

#include "sentai_aruco_shim.h"

#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

namespace {

// Cached latest snapshot for sentai_aruco_get_latest().  Even though
// detect() always returns detected=0 today, we still publish a fresh
// frame_seq so callers see "we ran".
static sentai_aruco_pose_t s_latest = {};
static uint32_t s_seq = 0;
static bool s_inited = false;

}  // namespace

extern "C" __attribute__((section(".sdram_text"), noinline))
int sentai_aruco_init(void) {
    if (s_inited) return 0;
    std::memset(&s_latest, 0, sizeof(s_latest));
    s_inited = true;
    return 0;
}

extern "C" __attribute__((section(".sdram_text"), noinline))
int sentai_aruco_detect(const uint8_t* /*gray*/, int /*w*/, int /*h*/,
                         sentai_aruco_pose_t* out) {
    if (!s_inited) sentai_aruco_init();
    s_seq++;
    s_latest.detected   = 0;             // stub: never detects
    s_latest.num_markers = 0;
    s_latest.frame_seq  = s_seq;
    s_latest.detect_us  = 0;             // not measured
    s_latest.src_ts_ms  = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (out) std::memcpy(out, &s_latest, sizeof(*out));
    return 0;
}

extern "C" __attribute__((section(".sdram_text"), noinline))
int sentai_aruco_get_latest(sentai_aruco_pose_t* out) {
    if (!out) return -1;
    if (!s_inited) {
        std::memset(out, 0, sizeof(*out));
        return 0;
    }
    std::memcpy(out, &s_latest, sizeof(*out));
    return 0;
}

extern "C" __attribute__((section(".sdram_text"), noinline))
void sentai_aruco_shutdown(void) {
    s_inited = false;
}
