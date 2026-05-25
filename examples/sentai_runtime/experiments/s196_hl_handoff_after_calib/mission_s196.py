# mission_s196 - TD-S10-A3 clean uncalibrated visual-servo calibration path.
#
# This mission is intentionally not a fork of s194's control algorithm.
# s194 remains useful as a negative-control experiment and launcher precedent.
# s195 starts from image-space WhyCon features and delays pose/body-frame
# decisions until the camera/body response has been measured.

import sentai
import struct


JOURNAL_NAME = "mission_s196_journal.txt"
SUMMARY_NAME = "mission_s196_summary.json"

IMG_W = 320
IMG_H = 240
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0
MARKER_DIAMETER_M = 0.0544

# Canonical A3 small-pad layout, including the asymmetric seventh marker.
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),  # NW
    (+0.08, +0.08, 0.005),  # NE
    (-0.06,  0.00, 0.005),  # W
    (+0.06,  0.00, 0.005),  # E
    (-0.08, -0.08, 0.005),  # SW
    (+0.08, -0.08, 0.005),  # SE
    (+0.02, +0.10, 0.005),  # N, asymmetric
)

FULL_VIS_MARGIN_PX = 2.0
MIN_FULL_MARKERS = 4
PREFLIGHT_SAMPLES = 20
TICK_MS = 33

# First flight smoke: neutral RPY, bounded thrust ramp, no lateral hold.
ZERO_UNLOCK_S = 1.5
ARM_PRE_ZERO_S = 0.4
ARM_RETRY_ZERO_S = 0.5
RAMP_S = 8.0
MAX_RAMP_S = 14.0
T_BASE_U16 = 30000
T_MAX_U16 = 34000
ACQ_FULL_MARKERS = 7
LOCK_CONSEC_TICKS = 10
ACQ_LOCK_BRAKE_THRUST_U16 = 30500
MIN_LOCK_RADIUS_PX = 5.0
MANUAL_DESCENT_S = 1.8

# Brake vertical inertia immediately after 7x10 lock, before Z-hold.
POST_LOCK_BRAKE_MAX_S = 1.2
POST_LOCK_BRAKE_MIN_TICKS = 8
POST_LOCK_BRAKE_THRUST_U16 = 30500
POST_LOCK_SETTLE_THRUST_U16 = 32000
POST_LOCK_VZ_OK_M_S = 0.035
POST_LOCK_VZ_OK_TICKS = 4

# Visual-Z hold smoke.  Still no lateral control and no EKF/yaw decision path.
# Hold a slightly higher calibration band so future X/Y pulses have FOV margin.
Z_HOLD_TARGET_M = 0.55
Z_HOLD_DURATION_S = 2.5
Z_HOLD_MIN_FULL_MARKERS = 7
Z_HOLD_LOST_MAX_TICKS = 15
T_HOVER_VISUAL_U16 = 32800
T_HOLD_MIN_U16 = 30000
T_HOLD_MAX_U16 = 34500
KP_THRUST_PER_M = 6500.0
KD_THRUST_PER_M_S = 5200.0
VZ_LPF_ALPHA = 0.25

# First axis-response smoke.  No R commit here; just image response logging.
AXIS_PULSE_DEG = 1.0
AXIS_PULSE_S = 0.25
AXIS_SETTLE_S = 0.35
AXIS_MIN_FULL_MARKERS = 7
AXIS_RETURN_MAX_PX = 12.0
AXIS_RECENTER_MAX_PASSES = 0
AXIS_RECENTER_PULSE_DEG = 0.6
AXIS_RECENTER_PULSE_S = 0.20
AXIS_SMOKE_MAX_AXES = 2

# Final cleanup after both independent axis probes.  This uses the measured
# local image response as a tiny image-Jacobian estimate, only to recentre
# before landing; it is not persisted as calibration.
FINAL_RECENTER_MAX_PASSES = 0
FINAL_RECENTER_TOL_PX = 10.0
FINAL_RECENTER_MAX_DEG = 0.45
FINAL_RECENTER_PULSE_S = 0.18
FINAL_RECENTER_SETTLE_S = 0.22
FINAL_RECENTER_RESPONSE_MIN_PX = 2.0
FINAL_RECENTER_DET_MIN = 4.0
FINAL_RECENTER_WORSE_MARGIN_PX = 2.0
AXIS_DOMINANCE_RATIO_MIN = 2.0
AXIS_ORTHOGONAL_DOT_MAX_NORM = 0.35

# Validation after axis/sign discovery.  This is a conservative IBVS smoke:
# use the measured local response to reduce centroid error, then accept only
# if the centroid moves in the predicted direction without losing marker lock.
CENTROID_VALIDATION_S = 2.0
CENTROID_VALIDATION_KP = 0.08
CENTROID_VALIDATION_KD = 0.0
CENTROID_VALIDATION_MAX_DEG = 0.45
CENTROID_VALIDATION_MIN_IMPROVE_PX = 3.0
CENTROID_VALIDATION_WORSE_MAX_PX = 8.0
CENTROID_VALIDATION_TOL_PX = 12.0
CENTROID_VALIDATION_INVERT_IMPULSE_J = True
CENTROID_VALIDATION_DEADBAND_PX = 6.0

CANDIDATE_SCORE_ACCEPT_MAX = 0.1
CANDIDATE_SCORE_MARGIN_MIN = 1.0

FINAL_VALIDATE_PULSE_DEG = 0.6
FINAL_VALIDATE_PULSE_S = 0.20
FINAL_VALIDATE_SETTLE_S = 0.25
FINAL_VALIDATE_RETURN_MAX_PX = 14.0


def _ser_val(v):
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, str):
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return '"' + s + '"'
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join(
            ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        ) + "}"
    return '"<%s>"' % type(v).__name__


def _write_summary(summary):
    sentai.fs.write(SUMMARY_NAME, _ser_val(summary))


def _j(event, data=None):
    if data is None:
        data = ""
    try:
        sentai.sim.journal_write(event, data)
    except Exception:
        pass


def _rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16):
    sentai.crazy.send_crtp(
        3, 0, struct.pack("<fffH",
                          roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16))


def _marker_world_bytes():
    buf = b""
    for x, y, z in MARKER_WORLD:
        buf += struct.pack("<fff", x, y, z)
    return buf


def _phase_setup(summary):
    _j("phase_setup", "start")
    sentai.calib.init()

    # Keep the old calibration out of the decision path.  A3 discovers and
    # commits a fresh R only after response validation.
    try:
        sentai.calib.load()
        summary["previous_R_loaded"] = True
    except Exception:
        summary["previous_R_loaded"] = False

    sentai.camera.init()
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)

    try:
        sentai.markers.clear_cam_extrinsics()
        summary["cam_extrinsics"] = "identity"
    except Exception:
        summary["cam_extrinsics"] = "identity_assumed"

    rc_mw = sentai.markers.set_marker_world(_marker_world_bytes())
    summary["marker_world_count"] = len(MARKER_WORLD)
    summary["set_marker_world_rc"] = rc_mw

    sentai.safety.init()
    sentai.crazy.init()
    # iter7/iter8 fix: subscribe to cf2 EKF pose so sentai.crazy.pose() returns
    # valid data.  Without this, ekf_vz_stable_streak in extpos_warmup_full
    # can never increment → warmup times out even when EKF is converged.
    # iter7 saw rc=-2 ("not ready") on first call.  cf2 CRTP TOC takes
    # ~hundreds of ms to populate after crazy.init().  s183/s192/s194 wrap
    # this in a retry loop — copy that pattern (~2 s ceiling).
    pose_sub_rc = -1
    pose_sub_tries = 0
    for _try in range(8):
        pose_sub_tries = _try + 1
        try:
            pose_sub_rc = sentai.crazy.pose_subscribe(33)
            if pose_sub_rc == 0:
                break
        except (AttributeError, RuntimeError, TypeError) as e:
            summary["pose_subscribe_error"] = str(e)[:120]
        sentai.rtos.sleep_ms(250)
    summary["pose_subscribe_rc"] = pose_sub_rc
    summary["pose_subscribe_tries"] = pose_sub_tries

    _j("phase_setup", {
        "markers": len(MARKER_WORLD),
        "set_marker_world_rc": rc_mw,
        "intrinsics": (FX, FY, CX, CY),
        "marker_diameter_m": MARKER_DIAMETER_M,
        "pose_subscribe_rc": pose_sub_rc,
        "pose_subscribe_tries": pose_sub_tries,
    })
    return rc_mw == 0


def _is_full_visible(det):
    # Tuple layout documented in modsentai_markers.c:
    # (id, cx, cy, axis_a, axis_b, angle, comp_id, tx, ty, tz, rx, ry, rz,
    #  reproj, backend, pose_valid, geometry_valid, radius_outer)
    cx = float(det[1])
    cy = float(det[2])
    r = float(det[17])
    geom_ok = int(det[16]) == 1
    if not geom_ok or r <= 0.0:
        return False
    return (cx - r >= FULL_VIS_MARGIN_PX and
            cy - r >= FULL_VIS_MARGIN_PX and
            cx + r < (IMG_W - FULL_VIS_MARGIN_PX) and
            cy + r < (IMG_H - FULL_VIS_MARGIN_PX))


def _detect_features():
    n = sentai.markers.detect_from_camera()
    detections = []
    full = []
    pose_z = []
    radius_sum = 0.0
    cx_sum = 0.0
    cy_sum = 0.0

    if n < 0:
        return {
            "n_raw": n,
            "n_full": 0,
            "centroid_px": None,
            "radius_mean_px": 0.0,
            "z_cam_mean_m": 0.0,
            "detections": [],
        }

    for i in range(n):
        d = sentai.markers.get_detection_tuple(i)
        if d is None:
            continue
        is_full = _is_full_visible(d)
        item = {
            "idx": i,
            "id": int(d[0]),
            "cx": float(d[1]),
            "cy": float(d[2]),
            "axis_a": float(d[3]),
            "axis_b": float(d[4]),
            "angle_rad": float(d[5]),
            "tx": float(d[7]),
            "ty": float(d[8]),
            "tz": float(d[9]),
            "pose_valid": int(d[15]),
            "geometry_valid": int(d[16]),
            "radius_outer": float(d[17]),
            "full_visible": is_full,
        }
        detections.append(item)
        if is_full:
            full.append(item)
            cx_sum += item["cx"]
            cy_sum += item["cy"]
            radius_sum += item["radius_outer"]
            if item["pose_valid"] == 1 and item["tz"] > 0.0:
                pose_z.append(item["tz"])

    nf = len(full)
    z_mean = 0.0
    if len(pose_z) > 0:
        z_mean = sum(pose_z) / len(pose_z)

    return {
        "n_raw": n,
        "n_full": nf,
        "centroid_px": ((cx_sum / nf), (cy_sum / nf)) if nf > 0 else None,
        "radius_mean_px": (radius_sum / nf) if nf > 0 else 0.0,
        "z_cam_mean_m": z_mean,
        "detections": detections,
    }


def _phase_preflight_features(summary):
    _j("phase_preflight_features", "start")
    ok_full_ticks = 0
    n_full_max = 0
    radius_max = 0.0
    z_max = 0.0
    for k in range(PREFLIGHT_SAMPLES):
        f = _detect_features()
        tick = {
            "k": k,
            "n_raw": f["n_raw"],
            "n_full": f["n_full"],
            "centroid_px": f["centroid_px"],
            "radius_mean_px": f["radius_mean_px"],
            "z_cam_mean_m": f["z_cam_mean_m"],
        }
        if f["n_full"] >= MIN_FULL_MARKERS:
            ok_full_ticks += 1
        if f["n_full"] > n_full_max:
            n_full_max = f["n_full"]
        if f["radius_mean_px"] > radius_max:
            radius_max = f["radius_mean_px"]
        if f["z_cam_mean_m"] > z_max:
            z_max = f["z_cam_mean_m"]
        _j("feature_tick", tick)
        sentai.rtos.sleep_ms(TICK_MS)

    summary["preflight_ok_ticks"] = ok_full_ticks
    summary["preflight_n_full_max"] = n_full_max
    summary["preflight_radius_max_px"] = radius_max
    summary["preflight_z_cam_max_m"] = z_max
    summary["preflight_feature_lock"] = (
        ok_full_ticks >= (PREFLIGHT_SAMPLES // 2))
    _j("phase_preflight_features", {
        "ok_ticks": ok_full_ticks,
        "samples": PREFLIGHT_SAMPLES,
        "lock": summary["preflight_feature_lock"],
    })
    return True


def _stream_zero_thrust(duration_s):
    n_ticks = int(duration_s * 1000 / TICK_MS)
    for _ in range(n_ticks):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)


def _phase_arm_zero_unlock(summary):
    _j("phase_arm_zero_unlock", "start")
    _stream_zero_thrust(ARM_PRE_ZERO_S)
    try:
        sentai.crazy.arm()
        summary["armed"] = True
    except Exception as e:
        summary["armed"] = False
        summary["arm_error"] = str(e)
        _j("arm_error", {"err": str(e)})
        return False

    _stream_zero_thrust(ZERO_UNLOCK_S)
    try:
        # cf2 SITL can report "Can not fly" briefly after spawn; a late arm
        # retry makes the raw RPYT path robust without using pose/GT feedback.
        sentai.crazy.arm()
        summary["arm_retry"] = True
    except Exception as e:
        summary["arm_retry"] = False
        summary["arm_retry_error"] = str(e)
    _stream_zero_thrust(ARM_RETRY_ZERO_S)
    _j("phase_arm_zero_unlock", {
        "pre_zero_s": ARM_PRE_ZERO_S,
        "zero_unlock_s": ZERO_UNLOCK_S,
        "retry_zero_s": ARM_RETRY_ZERO_S,
        "arm_retry": summary.get("arm_retry", False),
    })
    return True


def _feature_has_lock(f):
    return (f["n_full"] >= ACQ_FULL_MARKERS and
            f["radius_mean_px"] >= MIN_LOCK_RADIUS_PX and
            f["centroid_px"] is not None)


def _phase_thrust_only_marker_acquisition(summary):
    _j("phase_thrust_only_marker_acquisition", "start")
    lock_streak = 0
    ticks = int(MAX_RAMP_S * 1000 / TICK_MS)
    lock = None
    n_full_max = 0
    radius_max = 0.0
    z_min = 999.0
    z_max = 0.0
    z_last = 0.0
    first_seen_tick = None
    last_thrust = T_BASE_U16

    for k in range(ticks):
        t_s = (k * TICK_MS) / 1000.0
        if lock_streak > 0:
            # Once the full 7-marker band appears, stop accelerating upward
            # while collecting the remaining consecutive confirmation frames.
            thrust = ACQ_LOCK_BRAKE_THRUST_U16
        else:
            alpha = t_s / RAMP_S
            if alpha > 1.0:
                alpha = 1.0
            thrust = int(T_BASE_U16 + alpha * (T_MAX_U16 - T_BASE_U16))
        last_thrust = thrust
        _rpyt(0.0, 0.0, 0.0, thrust)

        f = _detect_features()
        has_lock = _feature_has_lock(f)
        if has_lock:
            lock_streak += 1
        else:
            lock_streak = 0
        if f["n_full"] > n_full_max:
            n_full_max = f["n_full"]
        if f["radius_mean_px"] > radius_max:
            radius_max = f["radius_mean_px"]
        if f["z_cam_mean_m"] > 0.0:
            z_last = f["z_cam_mean_m"]
            if f["z_cam_mean_m"] < z_min:
                z_min = f["z_cam_mean_m"]
            if f["z_cam_mean_m"] > z_max:
                z_max = f["z_cam_mean_m"]
        if first_seen_tick is None and f["n_full"] > 0:
            first_seen_tick = k

        tick = {
            "k": k,
            "t_s": t_s,
            "thrust": thrust,
            "n_raw": f["n_raw"],
            "n_full": f["n_full"],
            "centroid_px": f["centroid_px"],
            "radius_mean_px": f["radius_mean_px"],
            "z_cam_mean_m": f["z_cam_mean_m"],
            "lock_streak": lock_streak,
        }
        if k % 10 == 0 or lock_streak > 0:
            _j("acq_tick", tick)

        if lock_streak >= LOCK_CONSEC_TICKS:
            lock = tick
            break
        sentai.rtos.sleep_ms(TICK_MS)

    summary["marker_acquisition"] = {
        "locked": lock is not None,
        "lock_tick": lock,
        "last_thrust": last_thrust,
        "required_full_markers": ACQ_FULL_MARKERS,
        "lock_consec_required": LOCK_CONSEC_TICKS,
        "lock_brake_thrust": ACQ_LOCK_BRAKE_THRUST_U16,
        "n_full_max": n_full_max,
        "radius_max_px": radius_max,
        "z_cam_min_m": 0.0 if z_min == 999.0 else z_min,
        "z_cam_max_m": z_max,
        "z_cam_last_m": z_last,
        "first_seen_tick": first_seen_tick,
    }
    _j("phase_thrust_only_marker_acquisition", summary["marker_acquisition"])
    return lock is not None, last_thrust


def _phase_post_lock_brake(summary, start_thrust):
    _j("phase_post_lock_brake", {
        "start_thrust": start_thrust,
        "brake_thrust": POST_LOCK_BRAKE_THRUST_U16,
        "settle_thrust": POST_LOCK_SETTLE_THRUST_U16,
    })
    ticks = int(POST_LOCK_BRAKE_MAX_S * 1000 / TICK_MS)
    ok_vz_ticks = 0
    valid_ticks = 0
    lost_ticks = 0
    z_prev = 0.0
    z_last = 0.0
    z_min = 999.0
    z_max = 0.0
    vz_filt = 0.0
    thrust = POST_LOCK_BRAKE_THRUST_U16
    abort_reason = ""

    for k in range(ticks):
        f = _detect_features()
        valid = (f["n_full"] >= Z_HOLD_MIN_FULL_MARKERS and
                 f["z_cam_mean_m"] > 0.0)
        if valid:
            valid_ticks += 1
            lost_ticks = 0
            z = f["z_cam_mean_m"]
            if z_prev > 0.0:
                vz = (z - z_prev) * (1000.0 / TICK_MS)
                vz_filt = (1.0 - VZ_LPF_ALPHA) * vz_filt + VZ_LPF_ALPHA * vz
            z_prev = z
            z_last = z
            if z < z_min:
                z_min = z
            if z > z_max:
                z_max = z

            if vz_filt > POST_LOCK_VZ_OK_M_S:
                thrust = POST_LOCK_BRAKE_THRUST_U16
                ok_vz_ticks = 0
            else:
                thrust = POST_LOCK_SETTLE_THRUST_U16
                ok_vz_ticks += 1
        else:
            lost_ticks += 1
            thrust = POST_LOCK_SETTLE_THRUST_U16
            if lost_ticks >= Z_HOLD_LOST_MAX_TICKS:
                abort_reason = "marker_loss"

        _rpyt(0.0, 0.0, 0.0, thrust)
        if k % 3 == 0 or lost_ticks > 0:
            _j("post_lock_brake_tick", {
                "k": k,
                "thrust": thrust,
                "n_full": f["n_full"],
                "z_cam_mean_m": f["z_cam_mean_m"],
                "vz_filt_m_s": vz_filt,
                "ok_vz_ticks": ok_vz_ticks,
                "lost_ticks": lost_ticks,
            })

        if abort_reason:
            break
        if k >= POST_LOCK_BRAKE_MIN_TICKS and ok_vz_ticks >= POST_LOCK_VZ_OK_TICKS:
            break
        sentai.rtos.sleep_ms(TICK_MS)

    ok = (abort_reason == "" and valid_ticks > 0 and z_last > 0.0)
    summary["post_lock_brake"] = {
        "ok": ok,
        "abort_reason": abort_reason,
        "valid_ticks": valid_ticks,
        "z_cam_min_m": 0.0 if z_min == 999.0 else z_min,
        "z_cam_max_m": z_max,
        "z_cam_last_m": z_last,
        "vz_filt_last_m_s": vz_filt,
        "thrust_last": thrust,
        "ok_vz_ticks": ok_vz_ticks,
    }
    _j("phase_post_lock_brake", summary["post_lock_brake"])
    return ok, thrust


def _phase_visual_z_hold(summary, start_thrust):
    _j("phase_visual_z_hold", {
        "target_z_m": Z_HOLD_TARGET_M,
        "duration_s": Z_HOLD_DURATION_S,
        "start_thrust": start_thrust,
    })
    ticks = int(Z_HOLD_DURATION_S * 1000 / TICK_MS)
    lost_ticks = 0
    valid_ticks = 0
    z_min = 999.0
    z_max = 0.0
    z_last = 0.0
    z_prev = 0.0
    vz_filt = 0.0
    thrust = start_thrust
    thrust_min = start_thrust
    thrust_max = start_thrust
    abort_reason = ""

    for k in range(ticks):
        f = _detect_features()
        valid = (f["n_full"] >= Z_HOLD_MIN_FULL_MARKERS and
                 f["z_cam_mean_m"] > 0.0)
        if valid:
            valid_ticks += 1
            lost_ticks = 0
            z = f["z_cam_mean_m"]
            if z_prev > 0.0:
                vz = (z - z_prev) * (1000.0 / TICK_MS)
                vz_filt = (1.0 - VZ_LPF_ALPHA) * vz_filt + VZ_LPF_ALPHA * vz
            z_prev = z
            z_last = z
            if z < z_min:
                z_min = z
            if z > z_max:
                z_max = z

            err = Z_HOLD_TARGET_M - z
            cmd = (T_HOVER_VISUAL_U16 +
                   KP_THRUST_PER_M * err -
                   KD_THRUST_PER_M_S * vz_filt)
            thrust = int(cmd)
            if thrust < T_HOLD_MIN_U16:
                thrust = T_HOLD_MIN_U16
            if thrust > T_HOLD_MAX_U16:
                thrust = T_HOLD_MAX_U16
        else:
            lost_ticks += 1
            # Fail soft: keep near-hover briefly.  In iter3, lowering thrust
            # on partial marker loss pulled the drone down and made n_full
            # worse; do not turn a temporary 3-marker edge case into descent.
            thrust = T_HOVER_VISUAL_U16
            if thrust < T_HOLD_MIN_U16:
                thrust = T_HOLD_MIN_U16
            if lost_ticks >= Z_HOLD_LOST_MAX_TICKS:
                abort_reason = "marker_loss"

        if thrust < thrust_min:
            thrust_min = thrust
        if thrust > thrust_max:
            thrust_max = thrust

        _rpyt(0.0, 0.0, 0.0, thrust)
        if k % 5 == 0 or lost_ticks > 0:
            _j("zhold_tick", {
                "k": k,
                "thrust": thrust,
                "n_full": f["n_full"],
                "z_cam_mean_m": f["z_cam_mean_m"],
                "vz_filt_m_s": vz_filt,
                "lost_ticks": lost_ticks,
            })

        if abort_reason:
            break
        sentai.rtos.sleep_ms(TICK_MS)

    ok = (abort_reason == "" and valid_ticks >= (ticks // 2) and
          z_last > 0.0)
    summary["visual_z_hold"] = {
        "ok": ok,
        "abort_reason": abort_reason,
        "target_z_m": Z_HOLD_TARGET_M,
        "valid_ticks": valid_ticks,
        "ticks_requested": ticks,
        "z_cam_min_m": 0.0 if z_min == 999.0 else z_min,
        "z_cam_max_m": z_max,
        "z_cam_last_m": z_last,
        "vz_filt_last_m_s": vz_filt,
        "thrust_min": thrust_min,
        "thrust_max": thrust_max,
        "thrust_last": thrust,
    }
    _j("phase_visual_z_hold", summary["visual_z_hold"])
    return ok, thrust


def _z_thrust_from_feature(f, z_prev, vz_filt):
    z = f["z_cam_mean_m"]
    if z > 0.0 and z_prev > 0.0:
        vz = (z - z_prev) * (1000.0 / TICK_MS)
        vz_filt = (1.0 - VZ_LPF_ALPHA) * vz_filt + VZ_LPF_ALPHA * vz
    elif z > 0.0:
        vz_filt = 0.0
    if z > 0.0:
        z_prev = z
        err = Z_HOLD_TARGET_M - z
        cmd = (T_HOVER_VISUAL_U16 +
               KP_THRUST_PER_M * err -
               KD_THRUST_PER_M_S * vz_filt)
        thrust = int(cmd)
    else:
        thrust = T_HOVER_VISUAL_U16
    if thrust < T_HOLD_MIN_U16:
        thrust = T_HOLD_MIN_U16
    if thrust > T_HOLD_MAX_U16:
        thrust = T_HOLD_MAX_U16
    return thrust, z_prev, vz_filt


def _feature_compact(f):
    c = f["centroid_px"]
    return {
        "n_full": f["n_full"],
        "cx": 0.0 if c is None else c[0],
        "cy": 0.0 if c is None else c[1],
        "radius_mean_px": f["radius_mean_px"],
        "z_cam_mean_m": f["z_cam_mean_m"],
    }


def _stream_axis_segment(label, roll_deg, pitch_deg, duration_s, z_prev, vz_filt):
    ticks = int(duration_s * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1
    first = None
    last = None
    n_full_min = 99
    for k in range(ticks):
        f = _detect_features()
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(roll_deg, pitch_deg, 0.0, thrust)
        fc = _feature_compact(f)
        if first is None:
            first = fc
        last = fc
        if f["n_full"] < n_full_min:
            n_full_min = f["n_full"]
        if k == 0 or k == ticks - 1:
            _j("axis_segment_tick", {
                "label": label,
                "k": k,
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "thrust": thrust,
                "feature": fc,
                "vz_filt_m_s": vz_filt,
            })
        sentai.rtos.sleep_ms(TICK_MS)
    return first, last, n_full_min, z_prev, vz_filt, thrust


def _phase_axis_recenter(axis_name, axis_base, comp_dx, comp_dy,
                         z_prev, vz_filt):
    """Try to bring image centroid near this axis' local baseline.

    This is still a smoke-stage recover helper: it uses the just-observed
    complementary direction as a local image response estimate.  It does not
    commit calibration or encode a permanent mapping.
    """
    f = _detect_features()
    cur = _feature_compact(f)
    err_x = cur["cx"] - axis_base["cx"]
    err_y = cur["cy"] - axis_base["cy"]
    err = (err_x * err_x + err_y * err_y) ** 0.5
    thrust = T_HOVER_VISUAL_U16
    n_full_min = f["n_full"]
    passes = 0

    for p in range(AXIS_RECENTER_MAX_PASSES):
        if err <= AXIS_RETURN_MAX_PX:
            break
        # Project image error onto the measured axis response.  If the current
        # centroid drift is in the same direction as +axis response, command
        # -axis to undo it; otherwise command +axis.
        dot = err_x * comp_dx + err_y * comp_dy
        sign = -1.0 if dot > 0.0 else 1.0
        roll = 0.0
        pitch = 0.0
        if axis_name == "roll":
            roll = sign * AXIS_RECENTER_PULSE_DEG
        else:
            pitch = sign * AXIS_RECENTER_PULSE_DEG
        first, last, mn, z_prev, vz_filt, thrust = _stream_axis_segment(
            axis_name + "_recenter_%d" % p,
            roll, pitch, AXIS_RECENTER_PULSE_S,
            z_prev, vz_filt)
        if mn < n_full_min:
            n_full_min = mn
        # Short neutral settle after the corrective pulse.
        first_s, last_s, mn_s, z_prev, vz_filt, thrust = _stream_axis_segment(
            axis_name + "_recenter_settle_%d" % p,
            0.0, 0.0, AXIS_RECENTER_PULSE_S,
            z_prev, vz_filt)
        if mn_s < n_full_min:
            n_full_min = mn_s
        cur = last_s
        err_x = cur["cx"] - axis_base["cx"]
        err_y = cur["cy"] - axis_base["cy"]
        err = (err_x * err_x + err_y * err_y) ** 0.5
        passes += 1
        _j("axis_recenter", {
            "axis": axis_name,
            "pass": p,
            "cmd": (roll, pitch),
            "err_px": err,
            "feature": cur,
            "n_full_min": n_full_min,
        })

    return {
        "passes": passes,
        "err_px": err,
        "ok": err <= AXIS_RETURN_MAX_PX,
        "feature": cur,
        "n_full_min": n_full_min,
        "thrust_last": thrust,
    }, z_prev, vz_filt, thrust


def _phase_axis_response_smoke(summary, start_thrust):
    _j("phase_axis_response_smoke", {
        "pulse_deg": AXIS_PULSE_DEG,
        "pulse_s": AXIS_PULSE_S,
        "settle_s": AXIS_SETTLE_S,
    })
    f0 = _detect_features()
    baseline = _feature_compact(f0)
    z_prev = f0["z_cam_mean_m"]
    vz_filt = 0.0
    thrust = start_thrust
    axes = (
        ("pitch", 0.0, AXIS_PULSE_DEG),
        ("roll", AXIS_PULSE_DEG, 0.0),
    )
    results = []
    ok = True

    for axis_idx, (name, roll_cmd, pitch_cmd) in enumerate(axes):
        if axis_idx >= AXIS_SMOKE_MAX_AXES:
            break
        f_axis_base = _detect_features()
        axis_base = _feature_compact(f_axis_base)
        if axis_base["n_full"] < AXIS_MIN_FULL_MARKERS:
            ok = False
        pos_first, pos_last, pos_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_pos",
                                 roll_cmd, pitch_cmd, AXIS_PULSE_S,
                                 z_prev, vz_filt))
        neg_first, neg_last, neg_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_neg",
                                 -roll_cmd, -pitch_cmd, AXIS_PULSE_S,
                                 z_prev, vz_filt))
        settle_first, settle_last, settle_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_settle",
                                 0.0, 0.0, AXIS_SETTLE_S,
                                 z_prev, vz_filt))

        dx_pos = pos_last["cx"] - baseline["cx"]
        dy_pos = pos_last["cy"] - baseline["cy"]
        dx_neg = neg_last["cx"] - pos_last["cx"]
        dy_neg = neg_last["cy"] - pos_last["cy"]
        min_full = pos_min
        if neg_min < min_full:
            min_full = neg_min
        if settle_min < min_full:
            min_full = settle_min
        if min_full < AXIS_MIN_FULL_MARKERS:
            ok = False

        pos_seg_dx = pos_last["cx"] - pos_first["cx"]
        pos_seg_dy = pos_last["cy"] - pos_first["cy"]
        neg_seg_dx = neg_last["cx"] - neg_first["cx"]
        neg_seg_dy = neg_last["cy"] - neg_first["cy"]
        # Complementary response estimate.  Negative pulse response should be
        # opposite sign; subtracting it cancels some drift/delay bias.
        comp_dx = 0.5 * (pos_seg_dx - neg_seg_dx)
        comp_dy = 0.5 * (pos_seg_dy - neg_seg_dy)
        dominant_image_axis = "x"
        dominant_value = comp_dx
        secondary_value = comp_dy
        if abs(comp_dy) > abs(comp_dx):
            dominant_image_axis = "y"
            dominant_value = comp_dy
            secondary_value = comp_dx
        response_strength = (comp_dx * comp_dx + comp_dy * comp_dy) ** 0.5
        dominance_ratio = 999.0
        if abs(secondary_value) > 0.0001:
            dominance_ratio = abs(dominant_value) / abs(secondary_value)
        dominant_sign = 1 if dominant_value >= 0.0 else -1
        axis_sign_ok = (response_strength >= FINAL_RECENTER_RESPONSE_MIN_PX and
                        dominance_ratio >= AXIS_DOMINANCE_RATIO_MIN)
        pre_return_dx = settle_last["cx"] - axis_base["cx"]
        pre_return_dy = settle_last["cy"] - axis_base["cy"]
        pre_return_err_px = (pre_return_dx * pre_return_dx +
                             pre_return_dy * pre_return_dy) ** 0.5
        recenter, z_prev, vz_filt, thrust = _phase_axis_recenter(
            name, axis_base, comp_dx, comp_dy, z_prev, vz_filt)
        if recenter["n_full_min"] < min_full:
            min_full = recenter["n_full_min"]
        return_dx = recenter["feature"]["cx"] - axis_base["cx"]
        return_dy = recenter["feature"]["cy"] - axis_base["cy"]
        return_err_px = recenter["err_px"]
        return_ok = recenter["ok"]
        if not return_ok:
            ok = False
        if not axis_sign_ok:
            ok = False

        r = {
            "axis": name,
            "axis_baseline": axis_base,
            "cmd_pos": (roll_cmd, pitch_cmd),
            "cmd_neg": (-roll_cmd, -pitch_cmd),
            "pos_delta_from_phase_baseline_px": (dx_pos, dy_pos),
            "neg_delta_from_pos_end_px": (dx_neg, dy_neg),
            "pos_segment_delta_px": (pos_seg_dx, pos_seg_dy),
            "neg_segment_delta_px": (neg_seg_dx, neg_seg_dy),
            "complementary_delta_px": (comp_dx, comp_dy),
            "dominant_image_axis": dominant_image_axis,
            "dominant_sign": dominant_sign,
            "dominance_ratio": dominance_ratio,
            "response_strength_px": response_strength,
            "axis_sign_ok": axis_sign_ok,
            "pre_recenter_return_err_px": pre_return_err_px,
            "settle_delta_from_axis_baseline_px": (return_dx, return_dy),
            "return_err_px": return_err_px,
            "return_ok": return_ok,
            "recenter": recenter,
            "min_full_markers": min_full,
            "z_last_m": recenter["feature"]["z_cam_mean_m"],
            "thrust_last": thrust,
        }
        results.append(r)
        _j("axis_response", r)
        if not return_ok or not axis_sign_ok:
            break

    orthogonality = None
    pitch_vec = None
    roll_vec = None
    for r in results:
        comp = r.get("complementary_delta_px") or (0.0, 0.0)
        if r.get("axis") == "pitch":
            pitch_vec = (float(comp[0]), float(comp[1]))
        if r.get("axis") == "roll":
            roll_vec = (float(comp[0]), float(comp[1]))
    if pitch_vec is not None and roll_vec is not None:
        ps = _response_strength(pitch_vec)
        rs = _response_strength(roll_vec)
        dot_norm = 1.0
        if ps > 0.0001 and rs > 0.0001:
            dot_norm = ((pitch_vec[0] * roll_vec[0] +
                         pitch_vec[1] * roll_vec[1]) / (ps * rs))
        orthogonality = {
            "dot_norm": dot_norm,
            "ok": abs(dot_norm) <= AXIS_ORTHOGONAL_DOT_MAX_NORM,
        }
        if not orthogonality["ok"]:
            ok = False

    summary["axis_response_smoke"] = {
        "ok": ok,
        "baseline": baseline,
        "pulse_deg": AXIS_PULSE_DEG,
        "pulse_s": AXIS_PULSE_S,
        "settle_s": AXIS_SETTLE_S,
        "return_max_px": AXIS_RETURN_MAX_PX,
        "max_axes": AXIS_SMOKE_MAX_AXES,
        "dominance_ratio_min": AXIS_DOMINANCE_RATIO_MIN,
        "orthogonality": orthogonality,
        "results": results,
        "thrust_last": thrust,
    }
    _j("phase_axis_response_smoke", summary["axis_response_smoke"])
    return ok, thrust


def _response_strength(v):
    return (v[0] * v[0] + v[1] * v[1]) ** 0.5


def _axis_response_vectors(axis_summary):
    results = (axis_summary or {}).get("results") or ()
    roll_vec = None
    pitch_vec = None
    for r in results:
        comp = r.get("complementary_delta_px") or (0.0, 0.0)
        if r.get("axis") == "roll":
            roll_vec = (float(comp[0]), float(comp[1]))
        elif r.get("axis") == "pitch":
            pitch_vec = (float(comp[0]), float(comp[1]))
    return roll_vec, pitch_vec


def _axis_result(axis_summary, name):
    for r in (axis_summary or {}).get("results") or ():
        if r.get("axis") == name:
            return r
    return None


def _solve_image_response(roll_vec, pitch_vec, target_dx, target_dy):
    det = roll_vec[0] * pitch_vec[1] - pitch_vec[0] * roll_vec[1]
    if abs(det) < FINAL_RECENTER_DET_MIN:
        return None
    units_roll = (target_dx * pitch_vec[1] - pitch_vec[0] * target_dy) / det
    units_pitch = (roll_vec[0] * target_dy - target_dx * roll_vec[1]) / det
    return units_roll, units_pitch, det


def _clip(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _phase_centroid_pd_validation(summary, start_thrust):
    axis = summary.get("axis_response_smoke") or {}
    _j("phase_centroid_pd_validation", {
        "duration_s": CENTROID_VALIDATION_S,
        "kp": CENTROID_VALIDATION_KP,
        "kd": CENTROID_VALIDATION_KD,
        "max_deg": CENTROID_VALIDATION_MAX_DEG,
    })
    if not axis.get("ok", False):
        val = {
            "ok": False,
            "reason": "skipped_axis_response_not_accepted",
            "thrust_last": start_thrust,
        }
        summary["centroid_pd_validation"] = val
        _j("phase_centroid_pd_validation", val)
        return False, start_thrust

    roll_vec, pitch_vec = _axis_response_vectors(axis)
    roll_result = _axis_result(axis, "roll")
    pitch_result = _axis_result(axis, "pitch")
    if roll_vec is None or pitch_vec is None:
        val = {
            "ok": False,
            "reason": "missing_axis_response",
            "thrust_last": start_thrust,
        }
        summary["centroid_pd_validation"] = val
        _j("phase_centroid_pd_validation", val)
        return False, start_thrust

    f0 = _detect_features()
    cur = _feature_compact(f0)
    if cur["n_full"] < AXIS_MIN_FULL_MARKERS or cur["z_cam_mean_m"] <= 0.0:
        val = {
            "ok": False,
            "reason": "no_marker_lock",
            "initial_feature": cur,
            "thrust_last": start_thrust,
        }
        summary["centroid_pd_validation"] = val
        _j("phase_centroid_pd_validation", val)
        return False, start_thrust

    err_x = CX - cur["cx"]
    err_y = CY - cur["cy"]
    err = (err_x * err_x + err_y * err_y) ** 0.5
    err_initial = err
    err_min = err
    err_max = err
    feature_initial = cur
    feature_min = cur
    z_prev = cur["z_cam_mean_m"]
    vz_filt = 0.0
    prev_err_x = err_x
    prev_err_y = err_y
    thrust = start_thrust
    n_full_min = cur["n_full"]
    abort_reason = ""
    first_cmd = None
    last_cmd = None
    ticks = int(CENTROID_VALIDATION_S * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1

    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        if cur["n_full"] < n_full_min:
            n_full_min = cur["n_full"]
        if cur["n_full"] < AXIS_MIN_FULL_MARKERS or cur["z_cam_mean_m"] <= 0.0:
            abort_reason = "marker_loss"
            _rpyt(0.0, 0.0, 0.0, T_HOVER_VISUAL_U16)
            break

        err_x = CX - cur["cx"]
        err_y = CY - cur["cy"]
        derr_x = err_x - prev_err_x
        derr_y = err_y - prev_err_y
        prev_err_x = err_x
        prev_err_y = err_y
        roll_cmd = 0.0
        pitch_cmd = 0.0
        if abs(err_x) > CENTROID_VALIDATION_DEADBAND_PX:
            sign = 1.0
            if roll_result is not None and roll_result.get("dominant_sign", 1) < 0:
                sign = -1.0
            if CENTROID_VALIDATION_INVERT_IMPULSE_J:
                sign = -sign
            strength = _response_strength(roll_vec)
            if strength > 0.0001:
                roll_cmd = CENTROID_VALIDATION_KP * err_x / (sign * strength)
        if abs(err_y) > CENTROID_VALIDATION_DEADBAND_PX:
            sign = 1.0
            if pitch_result is not None and pitch_result.get("dominant_sign", 1) < 0:
                sign = -1.0
            if CENTROID_VALIDATION_INVERT_IMPULSE_J:
                sign = -sign
            strength = _response_strength(pitch_vec)
            if strength > 0.0001:
                pitch_cmd = CENTROID_VALIDATION_KP * err_y / (sign * strength)
        roll_cmd = _clip(roll_cmd,
                         -CENTROID_VALIDATION_MAX_DEG,
                         CENTROID_VALIDATION_MAX_DEG)
        pitch_cmd = _clip(pitch_cmd,
                          -CENTROID_VALIDATION_MAX_DEG,
                          CENTROID_VALIDATION_MAX_DEG)
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)

        err = (err_x * err_x + err_y * err_y) ** 0.5
        if err < err_min:
            err_min = err
            feature_min = cur
        if err > err_max:
            err_max = err
        cmd = {
            "k": k,
            "roll_deg": roll_cmd,
            "pitch_deg": pitch_cmd,
            "err_px": err,
            "feature": cur,
        }
        if first_cmd is None:
            first_cmd = cmd
        last_cmd = cmd
        if k % 5 == 0 or err > err_initial + CENTROID_VALIDATION_WORSE_MAX_PX:
            _j("centroid_pd_tick", cmd)
        if err > err_initial + CENTROID_VALIDATION_WORSE_MAX_PX:
            abort_reason = "centroid_error_worse"
            break
        sentai.rtos.sleep_ms(TICK_MS)

    _rpyt(0.0, 0.0, 0.0, thrust)
    f_last = _detect_features()
    last_feature = _feature_compact(f_last)
    if last_feature["n_full"] < n_full_min:
        n_full_min = last_feature["n_full"]
    last_err_x = CX - last_feature["cx"]
    last_err_y = CY - last_feature["cy"]
    err_last = (last_err_x * last_err_x + last_err_y * last_err_y) ** 0.5
    if err_last < err_min:
        err_min = err_last
        feature_min = last_feature
    if err_last > err_max:
        err_max = err_last

    improvement = err_initial - err_last
    ok = (abort_reason == "" and
          n_full_min >= AXIS_MIN_FULL_MARKERS and
          (err_last <= CENTROID_VALIDATION_TOL_PX or
           improvement >= CENTROID_VALIDATION_MIN_IMPROVE_PX) and
          err_max <= err_initial + CENTROID_VALIDATION_WORSE_MAX_PX)
    val = {
        "ok": ok,
        "reason": "" if ok else (abort_reason or "insufficient_improvement"),
        "target_px": (CX, CY),
        "initial_err_px": err_initial,
        "final_err_px": err_last,
        "min_err_px": err_min,
        "max_err_px": err_max,
        "improvement_px": improvement,
        "initial_feature": feature_initial,
        "best_feature": feature_min,
        "final_feature": last_feature,
        "n_full_min": n_full_min,
        "first_cmd": first_cmd,
        "last_cmd": last_cmd,
        "roll_vec_px": roll_vec,
        "pitch_vec_px": pitch_vec,
        "thrust_last": thrust,
    }
    summary["centroid_pd_validation"] = val
    _j("phase_centroid_pd_validation", val)
    return ok, thrust


def _det3(R):
    return (R[0] * (R[4] * R[8] - R[5] * R[7]) -
            R[1] * (R[3] * R[8] - R[5] * R[6]) +
            R[2] * (R[3] * R[7] - R[4] * R[6]))


def _discrete_rotation_candidates():
    rows = (
        (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
    )
    cands = []
    for rx in rows:
        for ry in rows:
            dot = rx[0] * ry[0] + rx[1] * ry[1] + rx[2] * ry[2]
            if abs(dot) > 0.0001:
                continue
            rz = (
                rx[1] * ry[2] - rx[2] * ry[1],
                rx[2] * ry[0] - rx[0] * ry[2],
                rx[0] * ry[1] - rx[1] * ry[0],
            )
            R = (rx[0], rx[1], rx[2],
                 ry[0], ry[1], ry[2],
                 rz[0], rz[1], rz[2])
            if _det3(R) > 0.5:
                cands.append(R)
    return cands


def _expected_from_row(row):
    ax = "x"
    val = row[0]
    if abs(row[1]) > abs(val):
        ax = "y"
        val = row[1]
    return ax, (1 if val >= 0.0 else -1)


def _score_candidate(R, obs):
    score = 0.0
    details = []
    specs = (("pitch", 0), ("roll", 1))
    for axis_name, row_idx in specs:
        o = obs.get(axis_name)
        row = (R[row_idx * 3], R[row_idx * 3 + 1], R[row_idx * 3 + 2])
        exp_axis, exp_sign = _expected_from_row(row)
        axis_penalty = 0.0 if exp_axis == o["axis"] else 10.0
        sign_penalty = 0.0 if exp_sign == o["sign"] else 1.0
        score += axis_penalty + sign_penalty
        details.append({
            "body_axis": axis_name,
            "expected_image_axis": exp_axis,
            "expected_sign": exp_sign,
            "observed_image_axis": o["axis"],
            "observed_sign": o["sign"],
            "axis_penalty": axis_penalty,
            "sign_penalty": sign_penalty,
        })
    return score, details


def _phase_candidate_scoring(summary):
    axis = summary.get("axis_response_smoke") or {}
    _j("phase_candidate_scoring", "start")
    if not axis.get("ok", False):
        out = {
            "ok": False,
            "reason": "skipped_axis_response_not_accepted",
        }
        summary["candidate_scoring"] = out
        _j("phase_candidate_scoring", out)
        return False

    obs = {}
    for r in axis.get("results") or ():
        if r.get("axis") in ("pitch", "roll"):
            obs[r.get("axis")] = {
                "axis": r.get("dominant_image_axis"),
                "sign": int(r.get("dominant_sign", 0)),
                "dominance_ratio": float(r.get("dominance_ratio") or 0.0),
                "response_strength_px": float(r.get("response_strength_px") or 0.0),
            }
    if "pitch" not in obs or "roll" not in obs:
        out = {"ok": False, "reason": "missing_axis_observation"}
        summary["candidate_scoring"] = out
        _j("phase_candidate_scoring", out)
        return False

    ranked = []
    for idx, R in enumerate(_discrete_rotation_candidates()):
        score, details = _score_candidate(R, obs)
        ranked.append({
            "idx": idx,
            "score": score,
            "R_cam_to_body": R,
            "details": details,
            "det": _det3(R),
        })
    ranked.sort(key=lambda x: x["score"])
    best = ranked[0]
    second = ranked[1] if len(ranked) > 1 else None
    margin = 999.0 if second is None else (second["score"] - best["score"])
    ok = (best["score"] <= CANDIDATE_SCORE_ACCEPT_MAX and
          margin >= CANDIDATE_SCORE_MARGIN_MIN)
    out = {
        "ok": ok,
        "reason": "" if ok else "weak_candidate_margin_or_residual",
        "observation": obs,
        "best": best,
        "second": second,
        "margin": margin,
        "accept_score_max": CANDIDATE_SCORE_ACCEPT_MAX,
        "accept_margin_min": CANDIDATE_SCORE_MARGIN_MIN,
        "ranking_top": ranked[:6],
        "committed": False,
        "saved": False,
    }
    summary["candidate_scoring"] = out
    _j("phase_candidate_scoring", out)
    return ok


def _axis_observation_from_comp(comp_dx, comp_dy):
    dominant_axis = "x"
    dominant_value = comp_dx
    secondary_value = comp_dy
    if abs(comp_dy) > abs(comp_dx):
        dominant_axis = "y"
        dominant_value = comp_dy
        secondary_value = comp_dx
    strength = (comp_dx * comp_dx + comp_dy * comp_dy) ** 0.5
    dominance = 999.0
    if abs(secondary_value) > 0.0001:
        dominance = abs(dominant_value) / abs(secondary_value)
    sign = 1 if dominant_value >= 0.0 else -1
    return dominant_axis, sign, dominance, strength


def _phase_final_candidate_validation(summary, start_thrust):
    cand = summary.get("candidate_scoring") or {}
    _j("phase_final_candidate_validation", {
        "pulse_deg": FINAL_VALIDATE_PULSE_DEG,
        "pulse_s": FINAL_VALIDATE_PULSE_S,
        "settle_s": FINAL_VALIDATE_SETTLE_S,
    })
    if not cand.get("ok", False):
        out = {
            "ok": False,
            "reason": "skipped_candidate_not_accepted",
            "thrust_last": start_thrust,
        }
        summary["final_candidate_validation"] = out
        _j("phase_final_candidate_validation", out)
        return False, start_thrust

    best = cand.get("best") or {}
    R = best.get("R_cam_to_body")
    if R is None or len(R) != 9:
        out = {
            "ok": False,
            "reason": "missing_candidate_R",
            "thrust_last": start_thrust,
        }
        summary["final_candidate_validation"] = out
        _j("phase_final_candidate_validation", out)
        return False, start_thrust

    z_prev = 0.0
    vz_filt = 0.0
    thrust = start_thrust
    results = []
    ok = True
    specs = (
        ("pitch", 0.0, FINAL_VALIDATE_PULSE_DEG, 0),
        ("roll", FINAL_VALIDATE_PULSE_DEG, 0.0, 1),
    )

    for name, roll_cmd, pitch_cmd, row_idx in specs:
        f_axis_base = _detect_features()
        axis_base = _feature_compact(f_axis_base)
        if axis_base["n_full"] < AXIS_MIN_FULL_MARKERS:
            ok = False
        if axis_base["z_cam_mean_m"] > 0.0:
            z_prev = axis_base["z_cam_mean_m"]

        pos_first, pos_last, pos_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment("final_" + name + "_pos",
                                 roll_cmd, pitch_cmd,
                                 FINAL_VALIDATE_PULSE_S,
                                 z_prev, vz_filt))
        neg_first, neg_last, neg_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment("final_" + name + "_neg",
                                 -roll_cmd, -pitch_cmd,
                                 FINAL_VALIDATE_PULSE_S,
                                 z_prev, vz_filt))
        settle_first, settle_last, settle_min, z_prev, vz_filt, thrust = (
            _stream_axis_segment("final_" + name + "_settle",
                                 0.0, 0.0,
                                 FINAL_VALIDATE_SETTLE_S,
                                 z_prev, vz_filt))

        pos_seg_dx = pos_last["cx"] - pos_first["cx"]
        pos_seg_dy = pos_last["cy"] - pos_first["cy"]
        neg_seg_dx = neg_last["cx"] - neg_first["cx"]
        neg_seg_dy = neg_last["cy"] - neg_first["cy"]
        comp_dx = 0.5 * (pos_seg_dx - neg_seg_dx)
        comp_dy = 0.5 * (pos_seg_dy - neg_seg_dy)
        obs_axis, obs_sign, dominance, strength = _axis_observation_from_comp(
            comp_dx, comp_dy)

        row = (float(R[row_idx * 3]),
               float(R[row_idx * 3 + 1]),
               float(R[row_idx * 3 + 2]))
        exp_axis, exp_sign = _expected_from_row(row)
        return_dx = settle_last["cx"] - axis_base["cx"]
        return_dy = settle_last["cy"] - axis_base["cy"]
        return_err = (return_dx * return_dx + return_dy * return_dy) ** 0.5
        min_full = pos_min
        if neg_min < min_full:
            min_full = neg_min
        if settle_min < min_full:
            min_full = settle_min
        axis_ok = (obs_axis == exp_axis and obs_sign == exp_sign and
                   dominance >= AXIS_DOMINANCE_RATIO_MIN and
                   strength >= FINAL_RECENTER_RESPONSE_MIN_PX and
                   min_full >= AXIS_MIN_FULL_MARKERS and
                   return_err <= FINAL_VALIDATE_RETURN_MAX_PX)
        if not axis_ok:
            ok = False
        r = {
            "axis": name,
            "axis_baseline": axis_base,
            "cmd_pos": (roll_cmd, pitch_cmd),
            "cmd_neg": (-roll_cmd, -pitch_cmd),
            "expected_image_axis": exp_axis,
            "expected_sign": exp_sign,
            "observed_image_axis": obs_axis,
            "observed_sign": obs_sign,
            "dominance_ratio": dominance,
            "response_strength_px": strength,
            "complementary_delta_px": (comp_dx, comp_dy),
            "return_err_px": return_err,
            "min_full_markers": min_full,
            "ok": axis_ok,
            "thrust_last": thrust,
        }
        results.append(r)
        _j("final_candidate_axis", r)

    out = {
        "ok": ok,
        "reason": "" if ok else "validation_motion_mismatch",
        "candidate_R": R,
        "candidate_idx": best.get("idx"),
        "pulse_deg": FINAL_VALIDATE_PULSE_DEG,
        "pulse_s": FINAL_VALIDATE_PULSE_S,
        "settle_s": FINAL_VALIDATE_SETTLE_S,
        "return_max_px": FINAL_VALIDATE_RETURN_MAX_PX,
        "results": results,
        "committed": False,
        "saved": False,
        "thrust_last": thrust,
    }
    summary["final_candidate_validation"] = out
    _j("phase_final_candidate_validation", out)
    return ok, thrust


def _phase_final_centroid_recenter(summary, start_thrust):
    _j("phase_final_centroid_recenter", {
        "target_px": (CX, CY),
        "tol_px": FINAL_RECENTER_TOL_PX,
        "max_passes": FINAL_RECENTER_MAX_PASSES,
    })
    if FINAL_RECENTER_MAX_PASSES <= 0:
        rec = {
            "ok": False,
            "reason": "disabled",
            "passes": 0,
            "err_px": 0.0,
            "thrust_last": start_thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, start_thrust

    axis = summary.get("axis_response_smoke") or {}
    results = axis.get("results") or ()
    roll_vec = None
    pitch_vec = None
    for r in results:
        comp = r.get("complementary_delta_px") or (0.0, 0.0)
        if r.get("axis") == "roll":
            roll_vec = (float(comp[0]), float(comp[1]))
        elif r.get("axis") == "pitch":
            pitch_vec = (float(comp[0]), float(comp[1]))

    thrust = start_thrust
    z_prev = 0.0
    vz_filt = 0.0
    if roll_vec is None or pitch_vec is None:
        rec = {
            "ok": False,
            "reason": "missing_axis_response",
            "passes": 0,
            "err_px": 999.0,
            "thrust_last": thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, thrust

    roll_strength = _response_strength(roll_vec)
    pitch_strength = _response_strength(pitch_vec)
    det = roll_vec[0] * pitch_vec[1] - pitch_vec[0] * roll_vec[1]
    if (roll_strength < FINAL_RECENTER_RESPONSE_MIN_PX or
            pitch_strength < FINAL_RECENTER_RESPONSE_MIN_PX or
            abs(det) < FINAL_RECENTER_DET_MIN):
        rec = {
            "ok": False,
            "reason": "weak_or_degenerate_response",
            "passes": 0,
            "roll_vec_px": roll_vec,
            "pitch_vec_px": pitch_vec,
            "det_px2": det,
            "roll_strength_px": roll_strength,
            "pitch_strength_px": pitch_strength,
            "err_px": 999.0,
            "thrust_last": thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, thrust

    passes = 0
    n_full_min = 99
    commands = []
    f = _detect_features()
    cur = _feature_compact(f)
    if f["n_full"] < n_full_min:
        n_full_min = f["n_full"]
    if cur["n_full"] >= AXIS_MIN_FULL_MARKERS and cur["z_cam_mean_m"] > 0.0:
        z_prev = cur["z_cam_mean_m"]
    err_x = CX - cur["cx"]
    err_y = CY - cur["cy"]
    err = (err_x * err_x + err_y * err_y) ** 0.5
    err_initial = err
    abort_reason = ""

    for p in range(FINAL_RECENTER_MAX_PASSES):
        if err <= FINAL_RECENTER_TOL_PX:
            break
        if cur["n_full"] < AXIS_MIN_FULL_MARKERS:
            abort_reason = "marker_loss_before_command"
            break

        # Solve [roll_vec pitch_vec] * units = image_error.  One unit is the
        # previously measured AXIS_PULSE_DEG command; clamp hard for safety.
        units_roll = (err_x * pitch_vec[1] - pitch_vec[0] * err_y) / det
        units_pitch = (roll_vec[0] * err_y - err_x * roll_vec[1]) / det
        roll_cmd = units_roll * AXIS_PULSE_DEG
        pitch_cmd = units_pitch * AXIS_PULSE_DEG
        if roll_cmd > FINAL_RECENTER_MAX_DEG:
            roll_cmd = FINAL_RECENTER_MAX_DEG
        if roll_cmd < -FINAL_RECENTER_MAX_DEG:
            roll_cmd = -FINAL_RECENTER_MAX_DEG
        if pitch_cmd > FINAL_RECENTER_MAX_DEG:
            pitch_cmd = FINAL_RECENTER_MAX_DEG
        if pitch_cmd < -FINAL_RECENTER_MAX_DEG:
            pitch_cmd = -FINAL_RECENTER_MAX_DEG

        first, last, mn, z_prev, vz_filt, thrust = _stream_axis_segment(
            "final_recenter_%d" % p,
            roll_cmd, pitch_cmd, FINAL_RECENTER_PULSE_S,
            z_prev, vz_filt)
        if mn < n_full_min:
            n_full_min = mn
        first_s, last_s, mn_s, z_prev, vz_filt, thrust = _stream_axis_segment(
            "final_recenter_settle_%d" % p,
            0.0, 0.0, FINAL_RECENTER_SETTLE_S,
            z_prev, vz_filt)
        if mn_s < n_full_min:
            n_full_min = mn_s
        cur = last_s
        err_x = CX - cur["cx"]
        err_y = CY - cur["cy"]
        err = (err_x * err_x + err_y * err_y) ** 0.5
        passes += 1
        cmd = {
            "pass": p,
            "roll_deg": roll_cmd,
            "pitch_deg": pitch_cmd,
            "err_px": err,
            "feature": cur,
        }
        commands.append(cmd)
        _j("final_recenter_tick", cmd)
        if err > err_initial + FINAL_RECENTER_WORSE_MARGIN_PX:
            abort_reason = "worse_after_cleanup_pulse"
            break

    ok = (err <= FINAL_RECENTER_TOL_PX and
          n_full_min >= AXIS_MIN_FULL_MARKERS)
    rec = {
        "ok": ok,
        "reason": "" if ok else (abort_reason or "not_centered_or_marker_loss"),
        "passes": passes,
        "target_px": (CX, CY),
        "initial_err_px": err_initial,
        "err_px": err,
        "feature": cur,
        "n_full_min": n_full_min,
        "roll_vec_px": roll_vec,
        "pitch_vec_px": pitch_vec,
        "det_px2": det,
        "commands": commands,
        "thrust_last": thrust,
    }
    summary["final_centroid_recenter"] = rec
    _j("phase_final_centroid_recenter", rec)
    return ok, thrust


def _phase_manual_descend_disarm(summary, start_thrust):
    _j("phase_manual_descend_disarm", {"start_thrust": start_thrust})
    n_ticks = int(MANUAL_DESCENT_S * 1000 / TICK_MS)
    if n_ticks < 1:
        n_ticks = 1
    for k in range(n_ticks):
        alpha = 1.0 - (float(k + 1) / float(n_ticks))
        thrust = int(start_thrust * alpha)
        if thrust < 0:
            thrust = 0
        _rpyt(0.0, 0.0, 0.0, thrust)
        sentai.rtos.sleep_ms(TICK_MS)
    _stream_zero_thrust(0.3)
    try:
        sentai.crazy.disarm()
        summary["disarmed"] = True
    except Exception as e:
        summary["disarmed"] = False
        summary["disarm_error"] = str(e)
    _j("phase_manual_descend_disarm", {"done": True,
                                        "disarmed": summary["disarmed"]})


def _phase_commit_R(summary):
    """s196 NEW: commit the R discovered by candidate_scoring so subsequent
    PnP get_drone_pose_tuple() returns valid drone position in pad frame.
    Required input for ExtPos warmup → cf2 Kalman anchoring → HL handoff."""
    cand = summary.get("candidate_scoring") or {}
    best = cand.get("best") or {}
    R = best.get("R_cam_to_body")
    out = {"committed": False, "rc": None, "R": R}
    if R is None or len(R) != 9:
        out["reason"] = "no_candidate_R"
        summary["commit_R"] = out
        _j("phase_commit_R", out)
        return False
    try:
        rc = sentai.calib.commit_R(R)
        out["committed"] = (rc == 0)
        out["rc"] = rc
    except (AttributeError, RuntimeError, ValueError) as e:
        out["reason"] = str(e)[:120]
    summary["commit_R"] = out
    _j("phase_commit_R", out)
    return out["committed"]


# s196 ExtPos warmup constants
EXTPOS_WARMUP_DUR_S      = 2.5
EXTPOS_WARMUP_VZ_TOL_M_S = 0.10
EXTPOS_WARMUP_STABLE_REQ = 6
EXTPOS_WARMUP_TIMEOUT_S  = 4.0
EXTPOS_MIN_FULL_MARKERS  = 5
# iter6 — separate PD gains for warmup (stronger than validation PD).
# Validation PD (Kp=0.08, max=0.45°) is calibrated for small corrections
# from near-centered start.  Warmup runs after axis-ID + validation
# accumulate ~5cm XY drift → need recovery authority.  iter5 saw drone
# drift +51cm in 5.8s with validation-strength PD.
WARMUP_PD_KP             = 0.20      # was 0.08 (2.5x stronger)
WARMUP_PD_MAX_DEG        = 1.20      # was 0.45 (recovery authority)
WARMUP_PD_DEADBAND_PX    = 4.0       # was 6.0 (act sooner)


def _phase_extpos_warmup_full(summary, start_thrust):
    """s196 NEW: stream full (x, y, z) ExtPos derived from committed-R PnP
    to cf2 Kalman.  Wait for ekf_vz to converge before declaring done.
    Anchors cf2 world frame to pad frame so HL go_to/takeoff works correctly.

    iter5 fix: keep centroid PD active during warmup (was open-loop XY in
    iter1-4 → drone drifted out of FOV in 2-4s → warmup never converged).
    Use roll_vec / pitch_vec from axis_response_smoke for image-Jacobian
    inverse, same logic as _phase_centroid_pd_validation.
    """
    _j("phase_extpos_warmup_full", {
        "dur_s": EXTPOS_WARMUP_DUR_S,
        "vz_tol_m_s": EXTPOS_WARMUP_VZ_TOL_M_S,
        "stable_req": EXTPOS_WARMUP_STABLE_REQ,
        "timeout_s": EXTPOS_WARMUP_TIMEOUT_S,
    })
    # iter5: load axis response vectors for centroid PD hold
    axis = summary.get("axis_response_smoke") or {}
    roll_vec, pitch_vec = _axis_response_vectors(axis)
    roll_result = _axis_result(axis, "roll")
    pitch_result = _axis_result(axis, "pitch")
    have_axis_pd = (roll_vec is not None and pitch_vec is not None)
    roll_strength = _response_strength(roll_vec) if roll_vec else 0.0
    pitch_strength = _response_strength(pitch_vec) if pitch_vec else 0.0
    z_prev = 0.0
    vz_filt = 0.0
    ekf_z_prev = None
    ekf_vz_stable_streak = 0
    converged = False
    n_full_min = 99
    last_pose = None
    thrust = start_thrust
    n_ticks = int(EXTPOS_WARMUP_TIMEOUT_S * 1000 / TICK_MS)
    for k in range(n_ticks):
        f = _detect_features()
        if f["n_full"] < n_full_min:
            n_full_min = f["n_full"]
        cf2_yaw = 0.0
        try:
            p_yaw = sentai.crazy.pose()
            if p_yaw is not None and len(p_yaw) >= 4:
                cf2_yaw = float(p_yaw[3])
        except (AttributeError, RuntimeError, TypeError):
            pass
        pose = None
        try:
            pose = sentai.markers.get_drone_pose_tuple(cf2_yaw)
        except (AttributeError, RuntimeError, TypeError):
            pose = None
        # Z PD continues so altitude is held during warmup
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        if (pose is not None and f["n_full"] >= EXTPOS_MIN_FULL_MARKERS
                and pose[2] > 0.1):
            last_pose = (float(pose[0]), float(pose[1]), float(pose[2]),
                          float(pose[3]) if len(pose) >= 4 else 0.0)
            try:
                sentai.crazy.send_extpos(last_pose[0], last_pose[1], last_pose[2])
            except (AttributeError, RuntimeError, TypeError):
                pass
        # iter5/6: centroid PD for XY hold during warmup.
        # iter6: use WARMUP_PD_* (stronger) instead of CENTROID_VALIDATION_*
        # (which are tuned for small near-centered corrections).
        roll_cmd = 0.0
        pitch_cmd = 0.0
        c = f["centroid_px"]
        if (have_axis_pd and c is not None and
                f["n_full"] >= EXTPOS_MIN_FULL_MARKERS):
            err_x = CX - c[0]
            err_y = CY - c[1]
            if abs(err_x) > WARMUP_PD_DEADBAND_PX:
                sign = 1.0
                if roll_result is not None and roll_result.get("dominant_sign", 1) < 0:
                    sign = -1.0
                if CENTROID_VALIDATION_INVERT_IMPULSE_J:
                    sign = -sign
                if roll_strength > 0.0001:
                    roll_cmd = WARMUP_PD_KP * err_x / (sign * roll_strength)
            if abs(err_y) > WARMUP_PD_DEADBAND_PX:
                sign = 1.0
                if pitch_result is not None and pitch_result.get("dominant_sign", 1) < 0:
                    sign = -1.0
                if CENTROID_VALIDATION_INVERT_IMPULSE_J:
                    sign = -sign
                if pitch_strength > 0.0001:
                    pitch_cmd = WARMUP_PD_KP * err_y / (sign * pitch_strength)
            roll_cmd = _clip(roll_cmd, -WARMUP_PD_MAX_DEG, WARMUP_PD_MAX_DEG)
            pitch_cmd = _clip(pitch_cmd, -WARMUP_PD_MAX_DEG, WARMUP_PD_MAX_DEG)
        _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)
        # cf2 Kalman stability via finite-diff on ekf_z
        ekf_x = ekf_y = ekf_z_now = None
        try:
            p = sentai.crazy.pose()
            if p is not None and len(p) >= 3:
                ekf_x, ekf_y, ekf_z_now = float(p[0]), float(p[1]), float(p[2])
        except (AttributeError, RuntimeError, TypeError):
            pass
        if ekf_z_now is not None and ekf_z_prev is not None:
            ekf_vz = (ekf_z_now - ekf_z_prev) * (1000.0 / TICK_MS)
            if abs(ekf_vz) < EXTPOS_WARMUP_VZ_TOL_M_S:
                ekf_vz_stable_streak += 1
            else:
                ekf_vz_stable_streak = 0
        ekf_z_prev = ekf_z_now
        if k % 8 == 0:
            _j("extpos_warmup_tick", {
                "k": k, "pose": last_pose,
                "ekf_xyz": (ekf_x, ekf_y, ekf_z_now),
                "stable_streak": ekf_vz_stable_streak,
                "n_full": f["n_full"],
                "thrust": thrust,
            })
        if (ekf_vz_stable_streak >= EXTPOS_WARMUP_STABLE_REQ
                and k * TICK_MS / 1000.0 >= EXTPOS_WARMUP_DUR_S):
            converged = True
            break
        sentai.rtos.sleep_ms(TICK_MS)
    out = {
        "ok": converged,
        "last_pose": last_pose,
        "n_full_min": n_full_min,
        "ekf_stable_streak": ekf_vz_stable_streak,
        "thrust_last": thrust,
    }
    summary["extpos_warmup"] = out
    _j("phase_extpos_warmup_full", out)
    return converged, thrust, last_pose


def _phase_hl_takeoff(summary, last_pose):
    """s196 NEW: notifySetpointsStop then HL takeoff() — operator-confirmed
    s193 lesson: takeoff() reads cf2 Kalman via CRTP handler so trajectory
    starts from a valid position.  go_to() with planner.state==IDLE returns
    traj_eval_invalid() = NaN → drone tumbles."""
    out = {"started": False}
    if last_pose is None:
        out["reason"] = "no_pose_available"
        summary["hl_takeoff"] = out
        _j("phase_hl_takeoff", out)
        return False
    # Relax priority so HL planner can take over
    try:
        sentai.crazy.send_crtp(7, 1, b'\x00')
        out["relaxed_priority"] = True
    except (AttributeError, RuntimeError, TypeError) as e:
        out["relax_error"] = str(e)[:120]
        out["relaxed_priority"] = False
    sentai.rtos.sleep_ms(150)
    # Use last_pose Z as target so HL doesn't ascend/descend at handoff
    target_z = last_pose[2]
    if target_z < 0.3:
        target_z = 0.3
    if target_z > 0.7:
        target_z = 0.7
    try:
        sentai.crazy.takeoff(target_z, 2.0)
        out["takeoff_started"] = True
        out["target_z"] = target_z
        out["started"] = True
    except (AttributeError, RuntimeError, TypeError, ValueError) as e:
        out["takeoff_error"] = str(e)[:120]
        out["takeoff_started"] = False
    summary["hl_takeoff"] = out
    _j("phase_hl_takeoff", out)
    # Hover briefly so HL planner stabilizes
    if out["started"]:
        sentai.rtos.sleep_ms(2500)
    return out["started"]


def _phase_hl_land(summary):
    """s196 NEW: HL land() — clean controlled descent vs manual thrust ramp."""
    out = {"landed": False}
    try:
        sentai.crazy.land(2.5)
        sentai.rtos.sleep_ms(3000)
        out["landed"] = True
    except (AttributeError, RuntimeError, TypeError, ValueError) as e:
        out["land_error"] = str(e)[:120]
    try:
        sentai.crazy.disarm()
        out["disarmed"] = True
    except (AttributeError, RuntimeError, TypeError) as e:
        out["disarm_error"] = str(e)[:120]
        out["disarmed"] = False
    summary["hl_land"] = out
    _j("phase_hl_land", out)
    return out["landed"]


def _flight_plan_placeholders(summary):
    # These are deliberately explicit so nobody mistakes this scaffold for a
    # completed flight controller.
    summary["flight_phases_enabled"] = False
    summary["next_phases"] = (
        "thrust_only_marker_acquisition",
        "post_lock_brake",
        "candidate_scoring",
        "validation",
        "commit_persist",
    )


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    summary = {
        "experiment": "s196_hl_handoff_after_calib",
        "task": "TD-S10-B3",
        "status": "ERROR",
        "phase": "start",
        "world_expected": "sentai_whycon_small",
        "image": {"w": IMG_W, "h": IMG_H},
        "fully_visible_min_markers": MIN_FULL_MARKERS,
        "acquisition_full_markers": ACQ_FULL_MARKERS,
        "acquisition_consecutive_frames": LOCK_CONSEC_TICKS,
        "uses_yaw_for_decision": False,
        "uses_ekf_for_decision": False,
        "uses_get_drone_pose_tuple": False,
    }

    try:
        summary["phase"] = "setup"
        if not _phase_setup(summary):
            summary["status"] = "SETUP_FAIL"
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        summary["phase"] = "preflight_features"
        _phase_preflight_features(summary)

        summary["phase"] = "arm_zero_unlock"
        if not _phase_arm_zero_unlock(summary):
            summary["status"] = "ARM_FAIL"
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        summary["phase"] = "thrust_only_marker_acquisition"
        locked, last_thrust = _phase_thrust_only_marker_acquisition(summary)

        zhold_ok = False
        axis_ok = False
        brake_ok = False
        centroid_validation_ok = False
        candidate_ok = False
        final_validation_ok = False
        if locked:
            summary["phase"] = "post_lock_brake"
            brake_ok, last_thrust = _phase_post_lock_brake(summary, last_thrust)

        if locked and brake_ok:
            summary["phase"] = "visual_z_hold"
            zhold_ok, last_thrust = _phase_visual_z_hold(summary, last_thrust)

        if locked and brake_ok and zhold_ok:
            summary["phase"] = "axis_response_smoke"
            axis_ok, last_thrust = _phase_axis_response_smoke(summary, last_thrust)

        if locked and brake_ok and zhold_ok and axis_ok:
            summary["phase"] = "centroid_pd_validation"
            centroid_validation_ok, last_thrust = _phase_centroid_pd_validation(
                summary, last_thrust)

        if locked and brake_ok and zhold_ok and axis_ok and centroid_validation_ok:
            summary["phase"] = "candidate_scoring"
            candidate_ok = _phase_candidate_scoring(summary)

        if (locked and brake_ok and zhold_ok and axis_ok and
                centroid_validation_ok and candidate_ok):
            summary["phase"] = "final_candidate_validation"
            final_validation_ok, last_thrust = _phase_final_candidate_validation(
                summary, last_thrust)

        if (locked and brake_ok and zhold_ok and axis_ok and
                centroid_validation_ok and candidate_ok and final_validation_ok):
            summary["phase"] = "final_centroid_recenter"
            _phase_final_centroid_recenter(summary, last_thrust)
            last_thrust = (summary.get("final_centroid_recenter") or {}).get(
                "thrust_last", last_thrust)
        elif locked and brake_ok and zhold_ok:
            reason = "skipped_axis_response_not_accepted"
            if axis_ok and not centroid_validation_ok:
                reason = "skipped_centroid_validation_not_accepted"
            elif axis_ok and centroid_validation_ok and not candidate_ok:
                reason = "skipped_candidate_not_accepted"
            elif axis_ok and centroid_validation_ok and candidate_ok and not final_validation_ok:
                reason = "skipped_final_validation_not_accepted"
            summary["final_centroid_recenter"] = {
                "ok": False,
                "reason": reason,
                "passes": 0,
                "thrust_last": last_thrust,
            }

        # s196 NEW: post-FINAL_VALIDATION_OK HL handoff sequence.
        # Pre-conditions for HL: all s195 phases passed.  If any failed,
        # bypass HL and use manual descent so drone still lands safely.
        commit_R_ok = False
        extpos_ok = False
        hl_takeoff_ok = False
        hl_land_ok = False
        last_pose = None
        s195_ok = (locked and brake_ok and zhold_ok and axis_ok
                   and centroid_validation_ok and candidate_ok
                   and final_validation_ok)

        if s195_ok:
            summary["phase"] = "commit_R"
            commit_R_ok = _phase_commit_R(summary)

        if s195_ok and commit_R_ok:
            summary["phase"] = "extpos_warmup_full"
            extpos_ok, last_thrust, last_pose = _phase_extpos_warmup_full(
                summary, last_thrust)

        if s195_ok and commit_R_ok and extpos_ok:
            summary["phase"] = "hl_takeoff"
            hl_takeoff_ok = _phase_hl_takeoff(summary, last_pose)

        if s195_ok and commit_R_ok and extpos_ok and hl_takeoff_ok:
            summary["phase"] = "hl_land"
            hl_land_ok = _phase_hl_land(summary)
        else:
            # Fallback: manual descent.  Either s195 path failed or HL chain
            # broke — either way, land safely without HL.
            summary["phase"] = "manual_descend_disarm"
            _phase_manual_descend_disarm(summary, last_thrust)

        summary["phase"] = "post_acquisition"
        _flight_plan_placeholders(summary)
        if not locked:
            summary["status"] = "MARKER_ACQ_TIMEOUT"
        elif not brake_ok:
            summary["status"] = "POST_LOCK_BRAKE_FAIL"
        elif not zhold_ok:
            summary["status"] = "VISUAL_Z_HOLD_FAIL"
        elif not axis_ok:
            summary["status"] = "AXIS_RESPONSE_FAIL"
        elif not centroid_validation_ok:
            summary["status"] = "CENTROID_VALIDATION_FAIL"
        elif not candidate_ok:
            summary["status"] = "CANDIDATE_SCORING_FAIL"
        elif not final_validation_ok:
            summary["status"] = "FINAL_VALIDATION_FAIL"
        elif not commit_R_ok:
            summary["status"] = "COMMIT_R_FAIL"
        elif not extpos_ok:
            summary["status"] = "EXTPOS_WARMUP_FAIL"
        elif not hl_takeoff_ok:
            summary["status"] = "HL_TAKEOFF_FAIL"
        elif not hl_land_ok:
            summary["status"] = "HL_LAND_FAIL"
        else:
            summary["status"] = "HL_HANDOFF_OK"
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = str(e)
        _j("mission_exception", {"err": str(e)})
        try:
            for _ in range(10):
                _rpyt(0.0, 0.0, 0.0, 0)
                sentai.rtos.sleep_ms(TICK_MS)
            sentai.crazy.disarm()
        except Exception:
            pass

    _j("mission_done", {"status": summary["status"],
                         "phase": summary["phase"]})
    _write_summary(summary)
    sentai.sim.journal_close()
    return summary
