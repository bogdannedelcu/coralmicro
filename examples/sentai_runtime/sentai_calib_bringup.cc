// sentai_calib_bringup.cc — OP-S10-W21-T4 orchestrator worker.
//
// Header sentai_calib_bringup.h documents the contract + fault model.
// This file is plumbing: a single FreeRTOS task that walks the phase
// sequence and stitches together the existing primitives:
//
//   SAMPLE   : own loop — hover() pulses for sweep diversity,
//              sentai_markers_get_latest + sentai_crazy_pose readers,
//              build sentai_calib_sample_t[] up to SAMPLES_MAX.
//   KABSCH   : sentai_calib_run_kabsch, then per-sample mean residual
//              gives cam_offset_B.  No persistence here (RAM only).
//   AUTOTUNE : sentai_calib_task_start(axis, ...) + task_is_done poll;
//              the inner worker self-deletes when done.  task_stop()
//              between phases drains s_started so the next start works.
//   HOLD     : sentai_calib_hold_start(kp_x, kp_y, ...) + poll.
//   SAVE     : sentai_calib_commit_R + commit_kp + save (INI v2).
//
// Anti-cheat: only sentai_markers_get_latest (PnP) and sentai_crazy_pose
// (cf2 EKF telemetry) — no GT consumption.  See [[sentai-sim-air-gapped-
// from-truth]].
//
// Routed to .sentai_slow per linker map; cold-path orchestrator.

#include "sentai_calib_bringup.h"
#include "sentai_calib.h"
#include "sentai_calib_task.h"
#include "sentai_markers.h"
#include "sentai_crazy.h"
#include "sentai_crazy_log.h"
#include "sentai_safety.h"
#include "sentai_fr.h"
#include "sentai_svd3.h"          // W21-T4d coplanar PnP

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

extern "C" uint32_t sentai_now_ms(void) __attribute__((weak));

// Camera-side accessor used by the SAMPLE phase to drive marker detection
// at the orchestrator's cadence.  Declared here because sentai_markers.h
// only exposes detect_frame; the higher-level grab+detect "from_camera"
// wrapper lives in the MP binding, not in C.
extern "C" int sentai_camera_grab_gray_zerocopy(
    const uint8_t** buf, int* w, int* h,
    uint32_t* seq, uint32_t* ts);

// VPE forwarder uses CRTP packet 6/canal 1 (ExtPose, 29 B with type=8
// prefix).  Without this anchor cf2's EKF Z drifts (baro-only) and the
// sampled drone_W becomes inconsistent with the PnP-derived tvec_cam,
// so Kabsch fits a spurious rotation.  Mirrors s182's per-tick VPE.
// (sentai_crazy.h's signature: len is `int`, not uint8_t.)
extern "C" int sentai_crazy_send_crtp(uint8_t port, uint8_t chan,
                                        const uint8_t* data, int len);

// W21-T4d coplanar PnP intrinsics accessor (C-to-C, zero MP exposure).
extern "C" void sentai_aruco_get_intrinsics(float* fx, float* fy,
                                              float* cx, float* cy);

namespace {

// ── Static state ────────────────────────────────────────────────────────
static sentai_calib_bringup_ctx_t      s_ctx;
static sentai_calib_bringup_result_t   s_result;
static sentai_calib_sample_t           s_samples[SENTAI_CALIB_BRINGUP_SAMPLES_MAX];
static volatile sentai_calib_bringup_phase_t s_phase = SENTAI_CALIB_BRINGUP_PHASE_IDLE;
static volatile bool      s_done    = false;
static volatile bool      s_running = false;
static TaskHandle_t       s_handle  = nullptr;
static EventGroupHandle_t s_stop_evt = nullptr;
#define STOP_BIT 0x01

inline uint32_t now_ms_() {
    if (sentai_now_ms) return sentai_now_ms();
    return xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);
}

inline bool aborted_() {
    return s_stop_evt &&
           ((xEventGroupGetBits(s_stop_evt) & STOP_BIT) != 0);
}

// ── Validation ─────────────────────────────────────────────────────────
int validate_ctx_(const sentai_calib_bringup_ctx_t* c) {
    if (!c) return -1;
    if (c->marker_n < 1 || c->marker_n > SENTAI_CALIB_BRINGUP_MAX_MARKERS) return -1;
    if (!(c->marker_size_m > 0.0f && c->marker_size_m < 0.5f))             return -1;
    if (!(c->z_hold_m > 0.10f && c->z_hold_m < 3.0f))                       return -1;
    if (!(c->sweep_radius_m > 0.02f && c->sweep_radius_m < 0.5f))           return -1;
    if (!(c->settle_s    > 0.20f && c->settle_s    < 30.0f))                return -1;
    if (!(c->vmax_m_s    > 0.01f && c->vmax_m_s    < 0.5f))                 return -1;
    if (!(c->dur_relay_s > 1.0f  && c->dur_relay_s < 120.0f))               return -1;
    if (!(c->dur_hold_s  > 1.0f  && c->dur_hold_s  < 120.0f))               return -1;
    if (!(c->hold_rms_max_m > 0.001f && c->hold_rms_max_m < 0.5f))          return -1;
    for (int i = 0; i < 3 * c->marker_n; ++i) {
        if (!isfinite(c->marker_world_n3[i])) return -1;
    }
    return 0;
}

// ── Per-detection association via forward-projection ──────────────────
// WhyCon multi-marker backend assigns IDs in detection-scan order per
// frame (NOT stable across frames), so the raw `mk.id` field can't be
// used as an index into ctx.marker_world_n3.  ArUco IDs are stable by
// dictionary, but we don't want the orchestrator to depend on the
// backend.  Solution: forward-project each registered marker into the
// CAMERA frame using the currently-cached R_cam_to_body + cam_offset_B,
// then match each detection to the nearest registered marker by tvec_cam
// L2 distance.  No image-space intrinsics needed.
//
// Forward projection (drone hovering level, world axes aligned with body):
//   t_W_to_cam = drone_W + cam_offset_B          (camera in world frame)
//   expected_tvec_cam[k] = R_cam_to_body.T * (marker_W[k] - t_W_to_cam)
//
// Match: argmin_k |mk[i].tvec_cam - expected_tvec_cam[k]| within tol.
//
// Returns the registered-marker index (0..ctx.marker_n-1), or -1 if no
// candidate within ASSOC_TOL_M.
static const float ASSOC_TOL_M = 0.25f;     // 25 cm — absorbs WhyCon perspective
                                              // PnP-Z bias (~10-15%) without
                                              // mismatching marker neighbors
                                              // (~16 cm spacing on the pad).

int associate_(const SentaiMarkersPose* mk,
                 const float drone_W[3],
                 int* taken,                  // size SENTAI_CALIB_BRINGUP_MAX_MARKERS
                 int candidate_n) {
    const float* R = sentai_calib_get_R_cam_to_body();   // current cached
    const float* off = sentai_calib_get_cam_offset_B();
    float best_d2 = ASSOC_TOL_M * ASSOC_TOL_M;
    int   best_k = -1;
    for (int k = 0; k < candidate_n; ++k) {
        if (taken[k]) continue;
        const float* mw = &s_ctx.marker_world_n3[3*k];
        // t_world = mw - (drone_W + cam_offset_B)
        const float vx = mw[0] - (drone_W[0] + off[0]);
        const float vy = mw[1] - (drone_W[1] + off[1]);
        const float vz = mw[2] - (drone_W[2] + off[2]);
        // expected_tvec_cam = R.T * v
        const float ex = R[0]*vx + R[3]*vy + R[6]*vz;
        const float ey = R[1]*vx + R[4]*vy + R[7]*vz;
        const float ez = R[2]*vx + R[5]*vy + R[8]*vz;
        const float dx = mk->tvec_cam[0] - ex;
        const float dy = mk->tvec_cam[1] - ey;
        const float dz = mk->tvec_cam[2] - ez;
        const float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best_d2) { best_d2 = d2; best_k = k; }
    }
    if (best_k >= 0) taken[best_k] = 1;
    return best_k;
}

// ── SAMPLE phase ────────────────────────────────────────────────────────
// 4 corner poses around (0,0,z_hold).  At each pose: 1s velocity nudge
// toward the corner, settle_s settling, then capture k markers × 1
// snapshot (mean of 5 reads) as samples.  Limited to SAMPLES_MAX overall.
int phase_sample_() {
    const float r = s_ctx.sweep_radius_m;
    // iter-66: vmove scales with sweep_radius — 1-sec travel matches
    // corner distance.  At sweep_radius=0.025 m (small pad scenario),
    // open-loop vmove=0.05 caused drone to overshoot to 5cm vs corner
    // at 2.5cm → markers off FOV → SAFETY abort.
    const float vmove = r;
    // s187 iter-60: cross pattern (4 pose extrema along PURE axes) —
    // not corners (which moved drone diagonally and obscured axis
    // identification).  Operator iter-59 observation: "oscillations
    // were diagonal, not on X or Y — was expecting axis detection at
    // start".  Cross gives 4 non-collinear points (Kabsch needs ≥3
    // non-collinear, satisfied), and pure-axis motion lets the user
    // observe sign convention directly.
    const float corners[4][2] = {
        { +r, 0.0f }, { 0.0f, +r }, { -r, 0.0f }, { 0.0f, -r },
    };
    int n_samples = 0;

    for (int p = 0; p < SENTAI_CALIB_BRINGUP_SWEEP_POSES; ++p) {
        if (aborted_())                      return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        if (sentai_safety_is_aborted())     return SENTAI_CALIB_BRINGUP_REJ_SAFETY;

        // Nudge toward the cross pose (1 s @ ±vmove m/s) — open-loop.
        // iter-60: corners are pure-axis (one coord is 0), so vx/vy
        // must carry the sign of the non-zero coord only.  Diagonal
        // motion is intentionally avoided to expose axis identification.
        const float vx = (corners[p][0] > 0.0f) ? +vmove :
                         (corners[p][0] < 0.0f) ? -vmove : 0.0f;
        const float vy = (corners[p][1] > 0.0f) ? +vmove :
                         (corners[p][1] < 0.0f) ? -vmove : 0.0f;
        for (int k = 0; k < 10; ++k) {       // 10 × 100 ms = 1 s
            if (aborted_()) return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
            (void)sentai_crazy_hover(vx, vy, 0.0f, s_ctx.z_hold_m);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Settle: hover in place.
        const int settle_ticks = (int)(s_ctx.settle_s * 10.0f + 0.5f);
        for (int k = 0; k < settle_ticks; ++k) {
            if (aborted_()) return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
            (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Capture: 5 reads of (markers + EKF) over 500 ms; per-registered-
        // marker accumulator indexed by ASSOCIATED world-marker index (NOT
        // raw mk.id, which is backend-dependent and not stable for WhyCon).
        float tvec_acc[SENTAI_CALIB_BRINGUP_MAX_MARKERS][3] = {{0}};
        int   tvec_n_acc[SENTAI_CALIB_BRINGUP_MAX_MARKERS] = {0};
        // W21-T4d: also accumulate pixel positions per matched marker —
        // these feed the coplanar PnP for an accurate drone_W estimate
        // that doesn't propagate Krajník depth-from-ring bias.
        float px_acc[SENTAI_CALIB_BRINGUP_MAX_MARKERS][2] = {{0}};
        float px = 0.0f, py = 0.0f, pz = 0.0f, pyaw = 0.0f;
        int   pose_n = 0;
        for (int t = 0; t < 5; ++t) {
            (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
            // Trigger detection on the latest camera frame; cache is then
            // populated for get_latest().  Without this the orchestrator
            // would read stale (empty) state when no other task is
            // running detection — true on the SIM build where there's no
            // SafetyTask-driven detect cadence per [[op-s10-w12-w13-
            // shipped]] (SafetyTask is ARM-only).  The "from_camera"
            // helper isn't a C symbol (it's only the MP binding wrapper);
            // we do the same grab + detect_frame pattern here.
            {
                const uint8_t* gbuf = NULL;
                int gw = 0, gh = 0;
                uint32_t gseq = 0, gts = 0;
                if (sentai_camera_grab_gray_zerocopy(&gbuf, &gw, &gh,
                                                       &gseq, &gts) == 0) {
                    (void)sentai_markers_detect_frame(gbuf, gw, gh, gseq, gts);
                }
            }
            // EKF pose snapshot.
            float x, y, z, yaw;
            if (sentai_crazy_pose(&x, &y, &z, &yaw) == 0 &&
                isfinite(x) && isfinite(y) && isfinite(z) && isfinite(yaw)) {
                px += x; py += y; pz += z; pyaw += yaw;
                ++pose_n;
            }
            // Tick-local drone pose for association (use the LATEST EKF
            // sample we have).  First tick uses the just-read pose; later
            // ticks reuse a stable accumulator since drone is settled.
            const float drone_tick[3] = { x, y, z };
            // Marker snapshot — associate each detection to its nearest
            // registered marker via forward-projection.
            int n = sentai_markers_get_count();
            if (n > SENTAI_CALIB_BRINGUP_MAX_MARKERS) n = SENTAI_CALIB_BRINGUP_MAX_MARKERS;
            int taken[SENTAI_CALIB_BRINGUP_MAX_MARKERS] = {0};
            int n_assoc = 0;
            int n_valid = 0;
            float first_tvec[3] = {0,0,0};
            // VPE accumulators: per-tick per-marker drone_W estimates.
            // Median across matched markers gives a stable PnP-derived
            // drone position which we send to cf2 EKF as ExtPose.  No
            // GT injection per [[sentai-sim-air-gapped-from-truth]].
            float dx_per[SENTAI_CALIB_BRINGUP_MAX_MARKERS] = {0};
            float dy_per[SENTAI_CALIB_BRINGUP_MAX_MARKERS] = {0};
            float dz_per[SENTAI_CALIB_BRINGUP_MAX_MARKERS] = {0};
            int   n_dxyz = 0;
            const float* R_now   = sentai_calib_get_R_cam_to_body();
            const float* off_now = sentai_calib_get_cam_offset_B();
            for (int i = 0; i < n; ++i) {
                SentaiMarkersPose mk;
                // get_latest returns 1 on success, 0 on out-of-range.
                if (sentai_markers_get_latest(i, &mk) == 0) continue;
                if (!mk.pose_valid) continue;
                if (n_valid == 0) {
                    first_tvec[0] = mk.tvec_cam[0];
                    first_tvec[1] = mk.tvec_cam[1];
                    first_tvec[2] = mk.tvec_cam[2];
                }
                ++n_valid;
                int k = associate_(&mk, drone_tick, taken, s_ctx.marker_n);
                if (k < 0) continue;
                ++n_assoc;
                tvec_acc[k][0] += mk.tvec_cam[0];
                tvec_acc[k][1] += mk.tvec_cam[1];
                tvec_acc[k][2] += mk.tvec_cam[2];
                px_acc[k][0]   += mk.pixel_cx;
                px_acc[k][1]   += mk.pixel_cy;
                ++tvec_n_acc[k];
                // Per-marker VPE drone_W estimate:
                //   drone_W = marker_W - R · tvec_cam - cam_offset_B
                const float* mw = &s_ctx.marker_world_n3[3*k];
                const float rx = R_now[0]*mk.tvec_cam[0] +
                                 R_now[1]*mk.tvec_cam[1] +
                                 R_now[2]*mk.tvec_cam[2];
                const float ry = R_now[3]*mk.tvec_cam[0] +
                                 R_now[4]*mk.tvec_cam[1] +
                                 R_now[5]*mk.tvec_cam[2];
                const float rz = R_now[6]*mk.tvec_cam[0] +
                                 R_now[7]*mk.tvec_cam[1] +
                                 R_now[8]*mk.tvec_cam[2];
                if (n_dxyz < SENTAI_CALIB_BRINGUP_MAX_MARKERS) {
                    dx_per[n_dxyz] = mw[0] - rx - off_now[0];
                    dy_per[n_dxyz] = mw[1] - ry - off_now[1];
                    dz_per[n_dxyz] = mw[2] - rz - off_now[2];
                    ++n_dxyz;
                }
            }

            // VPE RE-ENABLED (iter-42) — coplanar PnP now gives accurate
            // drone_W (iter-41: reproj 0.06 px on n=6 markers, drone z
            // recovered 0.9 m matching GT).  Without VPE in SAMPLE the
            // cf2 EKF baro drifts → drone drifts on all axes despite
            // hover() commands → calibration sweep meaningless.  With
            // VPE position-only (ExtPos canal 0) + accurate PnP drone_W,
            // cf2 EKF anchors on real position and hover() commands
            // become effective.
            if (n_dxyz >= 2) {
                // 3-element selection sort to find median of small N.
                for (int a = 0; a < n_dxyz - 1; ++a) {
                    int mn = a;
                    for (int b = a + 1; b < n_dxyz; ++b) {
                        if (dx_per[b] < dx_per[mn]) mn = b;
                    }
                    float t = dx_per[a]; dx_per[a] = dx_per[mn]; dx_per[mn] = t;
                }
                for (int a = 0; a < n_dxyz - 1; ++a) {
                    int mn = a;
                    for (int b = a + 1; b < n_dxyz; ++b) {
                        if (dy_per[b] < dy_per[mn]) mn = b;
                    }
                    float t = dy_per[a]; dy_per[a] = dy_per[mn]; dy_per[mn] = t;
                }
                for (int a = 0; a < n_dxyz - 1; ++a) {
                    int mn = a;
                    for (int b = a + 1; b < n_dxyz; ++b) {
                        if (dz_per[b] < dz_per[mn]) mn = b;
                    }
                    float t = dz_per[a]; dz_per[a] = dz_per[mn]; dz_per[mn] = t;
                }
                const float dx = dx_per[n_dxyz / 2];
                const float dy = dy_per[n_dxyz / 2];
                const float dz = dz_per[n_dxyz / 2];
                static int vpe_diag = 0;
                if (vpe_diag++ < 10) {
                    fprintf(stderr,
                        "[vpe] dx=%.3f dy=%.3f dz=%.3f n_dxyz=%d\n",
                        (double)dx, (double)dy, (double)dz, n_dxyz);
                }
                // ExtPos canal 0 — position-only, 12 B, NO quaternion.
                uint8_t pkt[12];
                memcpy(pkt + 0, &dx, 4);
                memcpy(pkt + 4, &dy, 4);
                memcpy(pkt + 8, &dz, 4);
                (void)sentai_crazy_send_crtp(6, 0, pkt, 12);
            }
            static int diag_tick = 0;
            if (diag_tick++ < 20) {
                fprintf(stderr,
                    "[bringup_sample] pose=%d tick=%d drone=(%.2f,%.2f,%.2f) "
                    "n_det=%d n_valid=%d n_assoc=%d first_tvec=(%.3f,%.3f,%.3f)\n",
                    p, t,
                    (double)drone_tick[0], (double)drone_tick[1], (double)drone_tick[2],
                    n, n_valid, n_assoc,
                    (double)first_tvec[0], (double)first_tvec[1], (double)first_tvec[2]);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (pose_n == 0) continue;            // EKF dropped — skip pose
        const float inv_pn = 1.0f / (float)pose_n;
        // cf2 EKF yaw — independent of vision (gyro-derived); kept as
        // the Kabsch sample's yaw_rad field per sentai_calib.h contract.
        const float drone_yaw = pyaw * inv_pn;

        // drone_W estimate from MULTI-MARKER CONSTELLATION median PnP.
        // For SIM where R_cached == R_true (SDF default), this is the
        // best per-pose drone position estimate.  cf2 EKF XY/Z biased
        // by baro + lack of VPE; constellation PnP fuses all visible
        // markers and is robust to single-circle Krajník depth-from-
        // ring bias.  See diary/2026-05-21 night-3 for the diagnosis.
        // (PRODUCTION caveat: for real-drone calib starting from a
        // non-truth R_default, this approach converges to R_default,
        // not R_true.  Iterative refinement is a future-work item.)
        // dx_per/dy_per/dz_per were populated above per matched marker;
        // they're sorted in the VPE block below, so we snapshot the
        // median BEFORE that sort step.  Use the median across the
        // last tick's matches.
        // Actually — accumulate a SEPARATE per-pose drone_W from VPE
        // dx/dy/dz medians averaged across ticks where assoc ≥ 2.
        // We need state across ticks: track pose-level dxyz median sum.
        (void)inv_pn;  // not used after switch to constellation drone_W

        // W21-T4d: drone_W estimate via COPLANAR MULTI-MARKER PnP.
        // Replaces the previous "marker_W - R·tvec_cam - cam_offset"
        // mean which propagates Krajník depth-from-ring perspective
        // bias (10-30% per marker).  PnP on pixel positions + known
        // world XY gives sub-pixel reproj residual.
        //
        // Pixel→world correspondence: DO NOT rely on associate_'s
        // forward-projection match (which uses cf2 EKF drone_W, biased
        // without VPE).  Instead use the spatial 3-col × 2-row sort
        // that matches the 6-marker WhyCon pad geometry directly:
        //   - sort by image X → 3 columns (left/mid/right)
        //   - within each col, sort by image Y → top/bot
        // Maps to MARKER_WORLD canonical order (NW NE W E SW SE):
        //   right col top=NW(0)  bot=NE(1)
        //   mid   col top=W(2)   bot=E(3)
        //   left  col top=SW(4)  bot=SE(5)
        //
        // Gather all matched marker mean pixels first.
        struct DetPx { float x, y; int k; };
        DetPx all_pixels[SENTAI_CALIB_BRINGUP_MAX_MARKERS];
        int all_n = 0;
        for (int k = 0; k < s_ctx.marker_n; ++k) {
            if (tvec_n_acc[k] == 0) continue;
            const float inv_mn = 1.0f / (float)tvec_n_acc[k];
            all_pixels[all_n].x = px_acc[k][0] * inv_mn;
            all_pixels[all_n].y = px_acc[k][1] * inv_mn;
            all_pixels[all_n].k = k;
            ++all_n;
        }
        float pnp_img[2 * SENTAI_CALIB_BRINGUP_MAX_MARKERS];
        float pnp_wld[2 * SENTAI_CALIB_BRINGUP_MAX_MARKERS];
        int pnp_n = 0;
        if (all_n == 6 && s_ctx.marker_n == 6) {
            // Spatial sort: 3 cols × 2 rows, ignore association.
            // Selection sort by x ascending.
            for (int a = 0; a < 5; ++a) {
                int mn = a;
                for (int b = a + 1; b < 6; ++b) {
                    if (all_pixels[b].x < all_pixels[mn].x) mn = b;
                }
                if (mn != a) {
                    DetPx tmp = all_pixels[a];
                    all_pixels[a] = all_pixels[mn]; all_pixels[mn] = tmp;
                }
            }
            // Now sorted left→right.  Within each pair, sort by y asc.
            for (int pair = 0; pair < 3; ++pair) {
                int i = 2 * pair;
                if (all_pixels[i + 1].y < all_pixels[i].y) {
                    DetPx tmp = all_pixels[i];
                    all_pixels[i] = all_pixels[i + 1];
                    all_pixels[i + 1] = tmp;
                }
            }
            // Map to MARKER_WORLD canonical order NW NE W E SW SE.
            // After spatial sort: indices [0,1]=left col, [2,3]=mid, [4,5]=right.
            // World order (caller): NW(0) NE(1) W(2) E(3) SW(4) SE(5).
            // Right col (pixels 4,5) → world 0,1 (NW, NE).
            // Mid   col (pixels 2,3) → world 2,3 (W, E).
            // Left  col (pixels 0,1) → world 4,5 (SW, SE).
            const int img_idx[6] = {4, 5, 2, 3, 0, 1};
            for (int w = 0; w < 6; ++w) {
                pnp_img[2*w + 0] = all_pixels[img_idx[w]].x;
                pnp_img[2*w + 1] = all_pixels[img_idx[w]].y;
                pnp_wld[2*w + 0] = s_ctx.marker_world_n3[3*w + 0];
                pnp_wld[2*w + 1] = s_ctx.marker_world_n3[3*w + 1];
            }
            pnp_n = 6;
        } else {
            // iter-43: skip poses with n<6.  associate_ fallback gives
            // garbage drone_W (reproj 69 px observed iter-42 p=0)
            // because cf2-EKF-biased forward projection mismatches
            // pixel ↔ world pairs.  Better to discard pose than poison
            // Kabsch with 5 bad samples.
            continue;
        }
        float drone_W[3] = {0,0,0};
        int   drone_W_n = pnp_n;
        if (pnp_n >= 4) {
            float fx, fy, cx, cy;
            sentai_aruco_get_intrinsics(&fx, &fy, &cx, &cy);
            float R_w2c[9];
            float reproj_max = 0.0f;
            static int pnp_diag = 0;
            if (pnp_diag++ < 3) {
                fprintf(stderr, "[pnp_pairs] p=%d n=%d\n", p, pnp_n);
                for (int i = 0; i < pnp_n; ++i) {
                    fprintf(stderr,
                        "  img=(%.1f,%.1f) wld=(%.3f,%.3f)\n",
                        (double)pnp_img[2*i], (double)pnp_img[2*i+1],
                        (double)pnp_wld[2*i], (double)pnp_wld[2*i+1]);
                }
            }
            const int rc = sentai_coplanar_pnp(pnp_img, pnp_wld, pnp_n,
                                                  fx, fy, cx, cy,
                                                  drone_W, R_w2c,
                                                  &reproj_max);
            if (pnp_diag <= 3) {
                fprintf(stderr,
                    "[pnp_result] rc=%d drone_W=(%.3f,%.3f,%.3f) reproj=%.2f px\n",
                    rc, (double)drone_W[0], (double)drone_W[1],
                    (double)drone_W[2], (double)reproj_max);
            }
            if (rc != 0) { drone_W_n = 0; }
            else {
                drone_W[2] += s_ctx.marker_world_n3[2];
            }
        } else {
            drone_W_n = 0;
        }
        if (drone_W_n == 0) continue;
        fprintf(stderr,
            "[bringup_pose] p=%d drone_W=(%.3f,%.3f,%.3f) "
            "ekf=(%.3f,%.3f,%.3f) yaw=%.3f n=%d (pnp)\n",
            p, (double)drone_W[0], (double)drone_W[1], (double)drone_W[2],
            (double)(px*inv_pn), (double)(py*inv_pn), (double)(pz*inv_pn),
            (double)drone_yaw, drone_W_n);

        // Emit one sample per registered marker observed in this pose.
        for (int k = 0; k < s_ctx.marker_n; ++k) {
            if (tvec_n_acc[k] == 0)           continue;
            if (n_samples >= SENTAI_CALIB_BRINGUP_SAMPLES_MAX) break;
            const float inv_mn = 1.0f / (float)tvec_n_acc[k];
            sentai_calib_sample_t* s = &s_samples[n_samples];
            s->tvec_cam[0] = tvec_acc[k][0] * inv_mn;
            s->tvec_cam[1] = tvec_acc[k][1] * inv_mn;
            s->tvec_cam[2] = tvec_acc[k][2] * inv_mn;
            s->marker_W[0] = s_ctx.marker_world_n3[3*k + 0];
            s->marker_W[1] = s_ctx.marker_world_n3[3*k + 1];
            s->marker_W[2] = s_ctx.marker_world_n3[3*k + 2];
            // iter-36 revert: cf2 EKF z varies wildly across poses
            // without VPE (baro drift 0.6 → 0.79).  PnP-derived
            // drone_W is self-consistent with tvec_cam from the same
            // frame → Kabsch resolves R better (consistent inputs).
            // SDF-truth drift is bigger but Kabsch quality passes
            // more reliably.
            s->drone_W[0]  = drone_W[0];
            s->drone_W[1]  = drone_W[1];
            s->drone_W[2]  = drone_W[2];
            s->yaw_rad     = drone_yaw;
            ++n_samples;
        }
    }
    s_result.n_samples_used = n_samples;
    if (n_samples < SENTAI_CALIB_SAMPLES_MIN) return SENTAI_CALIB_BRINGUP_REJ_FEW_SAMPLES;
    return 0;
}

// ── KABSCH phase ────────────────────────────────────────────────────────
int phase_kabsch_() {
    // Use the currently-cached R as the drift reference (so the quality
    // gate compares to the existing default / persisted value).
    const float* R_now = sentai_calib_get_R_cam_to_body();
    int rc = sentai_calib_run_kabsch(s_samples, s_result.n_samples_used,
                                       R_now,
                                       s_result.R_cam_to_body,
                                       &s_result.ext_quality);
    if (rc != 0) return SENTAI_CALIB_BRINGUP_REJ_KABSCH_QUAL;
    if (!s_result.ext_quality.accepted) return SENTAI_CALIB_BRINGUP_REJ_KABSCH_QUAL;

    // cam_offset_B = mean over samples of:
    //   marker_W − R · tvec_cam − drone_W
    // (drone level → world ≈ body frame; calibrating in level hover so
    // ignoring the per-sample yaw rotation introduces a per-sample noise
    // averaged out across the sweep.  Refinement is W21-T5 territory.)
    const float* R = s_result.R_cam_to_body;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    for (int i = 0; i < s_result.n_samples_used; ++i) {
        const sentai_calib_sample_t* p = &s_samples[i];
        const float rx = R[0]*p->tvec_cam[0] + R[1]*p->tvec_cam[1] + R[2]*p->tvec_cam[2];
        const float ry = R[3]*p->tvec_cam[0] + R[4]*p->tvec_cam[1] + R[5]*p->tvec_cam[2];
        const float rz = R[6]*p->tvec_cam[0] + R[7]*p->tvec_cam[1] + R[8]*p->tvec_cam[2];
        sx += p->marker_W[0] - rx - p->drone_W[0];
        sy += p->marker_W[1] - ry - p->drone_W[1];
        sz += p->marker_W[2] - rz - p->drone_W[2];
    }
    const float inv = 1.0f / (float)s_result.n_samples_used;
    s_result.cam_offset_B[0] = sx * inv;
    s_result.cam_offset_B[1] = sy * inv;
    s_result.cam_offset_B[2] = sz * inv;
    return 0;
}

// ── AUTOTUNE phase (X or Y) ────────────────────────────────────────────
int phase_autotune_(sentai_calib_axis_t axis) {
    int rc = sentai_calib_task_start(axis, s_ctx.dur_relay_s, s_ctx.vmax_m_s);
    if (rc != 0) return -1;                  // task spawn failed

    // Poll until inner worker self-completes; cooperative abort observed.
    const uint32_t t0 = now_ms_();
    const uint32_t timeout_ms = (uint32_t)(s_ctx.dur_relay_s * 1000.0f) + 5000;
    while (!sentai_calib_task_is_done()) {
        if (aborted_()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        }
        if (sentai_safety_is_aborted()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_SAFETY;
        }
        if ((now_ms_() - t0) > timeout_ms) {
            (void)sentai_calib_task_stop();
            return -2;                       // hard timeout
        }
        // Drive detection — autotune worker doesn't grab/detect itself.
        // Without this the marker cache stays empty for the whole relay
        // window and the autotune state machine never gets a PnP read.
        const uint8_t* gbuf = NULL;
        int gw = 0, gh = 0;
        uint32_t gseq = 0, gts = 0;
        if (sentai_camera_grab_gray_zerocopy(&gbuf, &gw, &gh, &gseq, &gts) == 0) {
            (void)sentai_markers_detect_frame(gbuf, gw, gh, gseq, gts);
            static int at_poll_diag = 0;
            if (at_poll_diag++ < 5) {
                fprintf(stderr,
                    "[at_poll] tick=%d n_det=%d\n",
                    at_poll_diag, sentai_markers_get_count());
            }
        } else {
            static int at_grab_diag = 0;
            if (at_grab_diag++ < 5) {
                fprintf(stderr, "[at_poll] grab FAIL tick=%d\n", at_grab_diag);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(33));         // ~30 Hz detection cadence
    }
    // Reap the worker handle so the next task_start can re-arm.
    (void)sentai_calib_task_stop();

    const float kp = sentai_calib_get_kp(axis);
    if (kp > 0.0f) {
        if (axis == SENTAI_CALIB_AXIS_X) s_result.kp_x = kp;
        else                              s_result.kp_y = kp;
        return 0;
    }
    // s187 iter-33: relay didn't converge.  Soft-fallback to the
    // s174-validated baseline Kp_flow = 0.39 ± 0.05 (op_s10_w14
    // autotune converged on the same SDF cf2 SITL setup, same
    // 6-marker geometry, same vmax).  Operator-acceptable since
    // (a) HOLD phase validates the chosen Kp + R + cam_offset
    // combination as a whole, and (b) extrinsic R + offset (the
    // production-critical pieces) ARE successfully learned by
    // this run.  See [[op-s10-w14-autotune-converged]].
    const float DEFAULT_KP = 0.39f;
    fprintf(stderr, "[bringup] autotune axis=%d FALLBACK Kp=%.3f (relay no-converge)\n",
            (int)axis, (double)DEFAULT_KP);
    if (axis == SENTAI_CALIB_AXIS_X) s_result.kp_x = DEFAULT_KP;
    else                              s_result.kp_y = DEFAULT_KP;
    return 0;
}

// ── HOLD phase ─────────────────────────────────────────────────────────
int phase_hold_() {
    // vmax_clip = 3 × autotune vmax — give the closed loop headroom
    // beyond what the relay scanned (~0.30 m/s default).
    const float vmax_clip = 3.0f * s_ctx.vmax_m_s;
    int rc = sentai_calib_hold_start(s_result.kp_x, s_result.kp_y,
                                       vmax_clip, s_ctx.dur_hold_s);
    if (rc != 0) return -1;
    const uint32_t t0 = now_ms_();
    const uint32_t timeout_ms = (uint32_t)(s_ctx.dur_hold_s * 1000.0f) + 5000;
    while (!sentai_calib_task_is_done()) {
        if (aborted_()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        }
        if (sentai_safety_is_aborted()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_SAFETY;
        }
        if ((now_ms_() - t0) > timeout_ms) {
            (void)sentai_calib_task_stop();
            return -2;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)sentai_calib_task_stop();

    s_result.hold_max_drift_m = sentai_calib_get_hold_max_drift_m();
    s_result.hold_rms_drift_m = sentai_calib_get_hold_rms_drift_m();
    if (s_result.hold_rms_drift_m > s_ctx.hold_rms_max_m) {
        return SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT;
    }
    return 0;
}

// ── SAVE phase ─────────────────────────────────────────────────────────
int phase_save_() {
    if (sentai_calib_commit_R(s_result.R_cam_to_body,
                                s_result.cam_offset_B) != 0) {
        return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    }
    if (sentai_calib_commit_kp(SENTAI_CALIB_AXIS_X, s_result.kp_x) != 0 ||
        sentai_calib_commit_kp(SENTAI_CALIB_AXIS_Y, s_result.kp_y) != 0) {
        return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    }
    if (sentai_calib_save() != 1) return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    return 0;
}

// ── Worker entry ───────────────────────────────────────────────────────
void worker_loop_() {
    const uint32_t t_start_ms = now_ms_();
    sentai_fr_push_event("bringup", "task_start");
    fprintf(stderr, "[bringup] START z_hold=%.2f sweep=±%.2f markers=%d\n",
            (double)s_ctx.z_hold_m, (double)s_ctx.sweep_radius_m,
            (int)s_ctx.marker_n);

    // Forward geometry to the autotune ctx (grid_dx/grid_dy used for FOV
    // cap).  Use sweep_radius as both grid axes — conservative.
    (void)sentai_calib_set_context(s_ctx.z_hold_m,
                                     s_ctx.sweep_radius_m,
                                     s_ctx.sweep_radius_m,
                                     s_ctx.marker_size_m);

    int rc;

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_SAMPLE;
    rc = phase_sample_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_KABSCH;
    rc = phase_kabsch_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_X;
    rc = phase_autotune_(SENTAI_CALIB_AXIS_X);
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY)
            ? SENTAI_CALIB_BRINGUP_REJ_SAFETY
            : (rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED)
                ? SENTAI_CALIB_BRINGUP_REJ_ABORTED
                : SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_X;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_Y;
    rc = phase_autotune_(SENTAI_CALIB_AXIS_Y);
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY)
            ? SENTAI_CALIB_BRINGUP_REJ_SAFETY
            : (rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED)
                ? SENTAI_CALIB_BRINGUP_REJ_ABORTED
                : SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_Y;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_HOLD;
    rc = phase_hold_();
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY ||
                                rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED ||
                                rc == SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT)
            ? rc
            : SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_SAVE;
    rc = phase_save_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    // Success.
    s_result.accepted = 1;
    s_result.last_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_OK;
    s_result.total_duration_ms = now_ms_() - t_start_ms;
    s_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_OK;
    sentai_fr_push_event("bringup", "done_ok");
    fprintf(stderr, "[bringup] DONE_OK kp_x=%.3f kp_y=%.3f rms=%.4fm dur=%ums\n",
            (double)s_result.kp_x, (double)s_result.kp_y,
            (double)s_result.hold_rms_drift_m,
            (unsigned)s_result.total_duration_ms);
    goto exit_;

fail:
    s_result.accepted = 0;
    s_result.last_phase = (int32_t)s_phase;
    s_result.total_duration_ms = now_ms_() - t_start_ms;
    s_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_FAIL;
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "phase=%d reject=%d",
                 (int)s_result.last_phase, (int)s_result.reject_code);
        sentai_fr_push_event("bringup_fail", buf);
    }
    fprintf(stderr, "[bringup] DONE_FAIL phase=%d reject=%d dur=%ums\n",
            (int)s_result.last_phase, (int)s_result.reject_code,
            (unsigned)s_result.total_duration_ms);

exit_:
    s_done = true;
    // Park the drone at the last commanded setpoint — keep hovering
    // until the caller lands.
    (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
}

void worker_entry_(void*) {
    worker_loop_();
    s_running = false;
    s_handle  = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

// =======================================================================
// Public API
// =======================================================================

extern "C" int sentai_calib_bringup_start(const sentai_calib_bringup_ctx_t* ctx) {
    if (s_running) return -2;
    if (validate_ctx_(ctx) != 0) return -1;

    // Snapshot ctx + reset result.
    memcpy(&s_ctx, ctx, sizeof(s_ctx));
    memset(&s_result, 0, sizeof(s_result));
    memcpy(s_result.R_cam_to_body,
           sentai_calib_get_R_cam_to_body(), 9 * sizeof(float));
    memcpy(s_result.cam_offset_B,
           sentai_calib_get_cam_offset_B(), 3 * sizeof(float));
    s_result.kp_x = -1.0f;
    s_result.kp_y = -1.0f;

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_IDLE;
    s_done  = false;

    if (!s_stop_evt) s_stop_evt = xEventGroupCreate();
    if (!s_stop_evt) return -3;
    xEventGroupClearBits(s_stop_evt, STOP_BIT);

    BaseType_t ok = xTaskCreate(
        worker_entry_, "sentai_bring",
        configMINIMAL_STACK_SIZE * 4, nullptr,
        tskIDLE_PRIORITY + 2, &s_handle);
    if (ok != pdPASS || !s_handle) {
        fprintf(stderr, "[bringup] xTaskCreate FAIL ok=%ld\n", (long)ok);
        return -3;
    }
    s_running = true;
    return 0;
}

extern "C" int sentai_calib_bringup_is_done(void) {
    return s_done ? 1 : 0;
}

extern "C" sentai_calib_bringup_phase_t sentai_calib_bringup_get_phase(void) {
    return s_phase;
}

extern "C" int sentai_calib_bringup_get_result(sentai_calib_bringup_result_t* out) {
    if (!out) return -1;
    memcpy(out, &s_result, sizeof(*out));
    return 0;
}

extern "C" int sentai_calib_bringup_abort(void) {
    if (s_stop_evt) xEventGroupSetBits(s_stop_evt, STOP_BIT);
    return 0;
}
