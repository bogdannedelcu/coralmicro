// sentai_aruco_shim.h — platform-abstract ArUco anchor detection API.
//
// Architecture mirrors `sentai_pxp_shim.h` + `sentai_fft_shim.h`:
//
//   ARM target  : implementation in sentai_aruco_shim_arm.cc.
//                 Will wrap the PXP + SIMD pipeline measured in s111
//                 (~1.1 ms threshold + 3.0 ms edge + contour finder).
//                 Currently a stub returning detected=0 — full M7
//                 detector is s112+ work.
//
//   SIM target  : implementation in sim/sentai_aruco_shim_sim.c.
//                 Connects to a sidecar Python publisher
//                 (sim/scripts/aruco_pose_publisher.py) over UDS at
//                 /tmp/sentai_aruco_pose.sock.  The sidecar runs
//                 cv2.aruco + solvePnP on Gazebo camera frames and
//                 publishes the world-pose snapshot continuously.
//
// MicroPython binding `sentai.flow.mode("anchor")` flips a flag that
// causes the flow path to call `sentai_aruco_detect()` once per frame
// and forward a VISION_POSITION_ESTIMATE via `sentai.link` (PX4
// MAVLink) or `sentai.crazy` (cf2 CRTP).  The forward step is
// platform-uniform; only the detection differs.
//
// Wire-format / contract is identical on both targets so a script
// written on SIM ports unchanged to ARM.

#ifndef SENTAI_ARUCO_SHIM_H_
#define SENTAI_ARUCO_SHIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Snapshot of the latest anchor pose estimate.
//
//   detected     1 if any marker visible in the most recent frame
//   num_markers  count of markers in the frame (0..N)
//   x_m, y_m, z_m  drone position in WORLD ENU (Gazebo / lab frame).
//                 Units: metres.
//   yaw_rad      drone yaw in WORLD ENU, [-pi, +pi].
//   frame_seq    monotonic frame counter from the detector source.
//                Strictly increasing on fresh detections; stale-read
//                returns same value.
//   detect_us    last-frame detection cost (informational).
//   src_ts_ms    monotonic ms at which the detector saw this frame;
//                consumer can compute staleness vs xTaskGetTickCount.
typedef struct {
    uint8_t  detected;
    uint8_t  num_markers;
    uint16_t _pad0;
    float    x_m;
    float    y_m;
    float    z_m;
    float    yaw_rad;
    uint32_t frame_seq;
    uint32_t detect_us;
    uint32_t src_ts_ms;
    uint32_t _pad1;
} sentai_aruco_pose_t;

// One-time init.  Idempotent.
//
// ARM impl: brings up the PXP+SIMD pipeline (stub today).
// SIM impl: opens the UDS to the Python publisher, non-blocking.
//
// Returns 0 on success, negative on failure.  Caller is expected to
// proceed even on failure (sentai_aruco_get_latest will return
// detected=0).
int sentai_aruco_init(void);

// Trigger detection on the supplied gray frame.
//
// ARM:  synchronous — runs the on-chip detector and fills `out`.
// SIM:  the Python sidecar already streams pose; this is a pure read
//       of the latest UDS snapshot (no per-call detection cost).
//       The `gray`/`w`/`h` arguments may be ignored on SIM since the
//       sidecar pulls frames directly from gz transport.
//
// Returns 0 on success (even if no marker detected — `out->detected`
// is the authoritative flag).  Returns negative on infrastructure
// error (no UDS, shim not initialised, etc.).
int sentai_aruco_detect(const uint8_t* gray, int w, int h,
                        sentai_aruco_pose_t* out);

// Tear-down (rarely used; firmware is long-running).
void sentai_aruco_shutdown(void);

// Convenience: poll the latest published snapshot without re-running
// detection.  ARM: same as the cached last result.  SIM: reads UDS
// snapshot.  Useful for periodic MAVLink VPE forwarding from a
// separate task.
int sentai_aruco_get_latest(sentai_aruco_pose_t* out);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_ARUCO_SHIM_H_
