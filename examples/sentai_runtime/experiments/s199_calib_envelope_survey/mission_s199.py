# mission_s199 - TD-S10-A3 calibration plus visual axis-envelope survey.
#
# This mission is intentionally not a fork of s194's control algorithm.
# s194 remains useful as a negative-control experiment and launcher precedent.
# s199 starts from image-space WhyCon features and delays pose/body-frame
# decisions until the camera/body response has been measured.

import sentai
import struct


JOURNAL_NAME = "mission_s199_journal.txt"
SUMMARY_NAME = "mission_s199_summary.json"
CALIB_ARTIFACT_NAME = "mission_s199_calibration.json"
CALIB_INI_NAME = "calib.ini"

IMG_W = 320
IMG_H = 240
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0
MARKER_DIAMETER_M = 0.0544

# Canonical A3 small-pad layout, including the asymmetric seventh marker.
# The XY centroid of these seven marker centers is the world origin.
MARKER_WORLD = (
    (-0.082857143, +0.065714286, 0.005),  # NW
    (+0.077142857, +0.065714286, 0.005),  # NE
    (-0.062857143, -0.014285714, 0.005),  # W
    (+0.057142857, -0.014285714, 0.005),  # E
    (-0.082857143, -0.094285714, 0.005),  # SW
    (+0.077142857, -0.094285714, 0.005),  # SE
    (+0.017142857, +0.085714286, 0.005),  # N, asymmetric
)

FULL_VIS_MARGIN_PX = 2.0
MIN_FULL_MARKERS = 4
PREFLIGHT_SAMPLES = 20
TICK_MS = 33
MARKER_COUNT_AVG_WINDOW = 10

# First flight smoke: neutral RPY, bounded thrust ramp, no lateral hold.
ZERO_UNLOCK_S = 1.5
ARM_PRE_ZERO_S = 0.4
ARM_RETRY_ZERO_S = 0.5
RAMP_S = 8.0
MAX_RAMP_S = 14.0
T_BASE_U16 = 30000
T_MAX_U16 = 34000
ACQ_FULL_MARKERS = 7
MARKER_AVG_FULL_LOCK = ACQ_FULL_MARKERS - 1.0
LOCK_CONSEC_TICKS = 10
ACQ_LOCK_BRAKE_THRUST_U16 = 30500
MIN_LOCK_RADIUS_PX = 5.0
MANUAL_DESCENT_S = 2.6
CENTER_HOLD_DESCENT_S = 14.0
CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS = 4
CENTER_HOLD_DESCENT_THRUST_FLOOR_U16 = 29500
CENTER_HOLD_DESCENT_KP = 0.10
CENTER_HOLD_DESCENT_MAX_DEG = 0.55
CENTER_HOLD_DESCENT_DEADBAND_SIGMA_MULT = 2.0
CENTER_HOLD_DESCENT_DEADBAND_FLOOR_PX = 0.75
CENTER_HOLD_DESCENT_TARGET_VZ_M_S = -0.09
CENTER_HOLD_DESCENT_KD_THRUST_PER_M_S = 5200.0
CENTER_HOLD_DESCENT_LOST_MAX_TICKS = 5
AXIS_ENVELOPE_LEG_S = 8.0
AXIS_ENVELOPE_CENTER_S = 8.0
AXIS_ENVELOPE_MAX_DEG = 0.55
AXIS_ENVELOPE_KP = 0.075
AXIS_ENVELOPE_ORTH_RECOVERY_KP_MULT = 2.0
AXIS_ENVELOPE_EDGE_FULL_MARKERS = ACQ_FULL_MARKERS - 1.0
AXIS_ENVELOPE_CENTER_TOL_PX = 18.0
AXIS_ENVELOPE_ORTH_RECOVERY_MIN_TICKS = MARKER_COUNT_AVG_WINDOW

# Brake vertical inertia immediately after 7x10 lock, before Z-hold.
POST_LOCK_BRAKE_MAX_S = 1.2
POST_LOCK_BRAKE_MIN_TICKS = 8
POST_LOCK_BRAKE_THRUST_U16 = 30500
POST_LOCK_SETTLE_THRUST_U16 = 32000
POST_LOCK_VZ_OK_M_S = 0.035
POST_LOCK_VZ_OK_TICKS = 4

# Visual-Z hold smoke.  Still no lateral control and no EKF/yaw decision path.
# Hold a slightly higher calibration band so future X/Y pulses have FOV margin.
Z_HOLD_TARGET_M = 0.64
Z_CALIB_ALTITUDE_GAIN = 1.30
Z_HOLD_TARGET_MAX_M = 1.05
Z_HOLD_CONTINUE_IF_TARGET_SEEN = True
Z_HOLD_TARGET_TOL_M = 0.04
Z_HOLD_DURATION_S = 4.2
Z_HOLD_MIN_FULL_MARKERS = 7
Z_HOLD_LOST_MAX_TICKS = 15
T_HOVER_VISUAL_U16 = 32800
T_HOLD_MIN_U16 = 30000
T_HOLD_MAX_U16 = 34500
KP_THRUST_PER_M = 6500.0
KD_THRUST_PER_M_S = 5200.0
VZ_LPF_ALPHA = 0.25
RUNTIME_Z_TARGET_M = Z_HOLD_TARGET_M

# First axis-response smoke.  No R commit here; just image response logging.
AXIS_PULSE_DEG = 1.0
AXIS_PULSE_S = 0.25
AXIS_SETTLE_S = 0.35
AXIS_RESPONSE_MAX_ATTEMPTS = 2
AXIS_RESPONSE_RETRY_SETTLE_S = 1.0
AXIS_MIN_FULL_MARKERS = 7
AXIS_MIN_AVG_FULL_MARKERS = AXIS_MIN_FULL_MARKERS - 1.0
AXIS_HARD_MIN_FULL_MARKERS = 4
AXIS_RETURN_MAX_PX = 12.0
AXIS_RECENTER_MAX_PASSES = 0
AXIS_RECENTER_PULSE_DEG = 0.6
AXIS_RECENTER_PULSE_S = 0.20
AXIS_SMOKE_MAX_AXES = 2

# Final return-to-center after orientation validation.  This is the proof step
# before landing: use the discovered command/image signs to drive the marker
# constellation centroid back toward the image center, with visual-Z hold active.
FINAL_RECENTER_DURATION_S = 4.5
FINAL_RECENTER_TOL_PX = 15.0
FINAL_RECENTER_MIN_IMPROVE_PX = 8.0
FINAL_RECENTER_MAX_DEG = 0.70
FINAL_RECENTER_KP = 0.075
FINAL_RECENTER_DEADBAND_PX = 5.0
FINAL_RECENTER_WORSE_MAX_PX = 10.0
FINAL_CENTER_HOVER_S = 1.2
FINAL_CENTER_HOVER_MAX_S = 3.0
FINAL_CENTER_HOVER_TOL_PX = 18.0
FINAL_CENTER_HOVER_WORSE_MAX_PX = 8.0
FINAL_RECENTER_RESPONSE_MIN_PX = 2.0
FINAL_RECENTER_DET_MIN = 4.0
AXIS_DOMINANCE_RATIO_MIN = 2.0
AXIS_ORTHOGONAL_DOT_MAX_NORM = 0.35
IBVS_DAMPING_PX_PER_DEG = 1.5
IBVS_SUSTAINED_RESPONSE_SIGN = -1.0
IBVS_Z_REF_M = 0.64
IBVS_Z_GAIN_MIN = 0.65
IBVS_Z_GAIN_MAX = 1.35

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
OPTICAL_AXIS_EXPECT_BODY_Z_IN_CAMERA_Z_SIGN = -1
OPTICAL_AXIS_MIN_POSE_VALID = 7
OPTICAL_AXIS_MIN_MEAN_TZ_M = 0.20

FINAL_VALIDATE_PULSE_DEG = 0.7
FINAL_VALIDATE_RETRY_PULSES_DEG = (1.0, 1.3)
FINAL_VALIDATE_PULSE_S = 0.20
FINAL_VALIDATE_SETTLE_S = 0.25
FINAL_VALIDATE_RETURN_MAX_PX = 14.0
FINAL_VALIDATE_MAX_PULSE_DEG = 1.3
FINAL_VALIDATE_NOISE_SAMPLES = 12
FINAL_VALIDATE_NOISE_SIGMA_MULT = 3.0
FINAL_VALIDATE_NOISE_FLOOR_PX = 0.6


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


def _compact_attempt(a):
    noise = a.get("noise") or {}
    return {
        "axis": a.get("axis"),
        "attempt": a.get("attempt"),
        "pulse_deg": a.get("pulse_deg"),
        "expected_image_axis": a.get("expected_image_axis"),
        "expected_sign": a.get("expected_sign"),
        "observed_image_axis": a.get("observed_image_axis"),
        "observed_sign": a.get("observed_sign"),
        "dominance_ratio": a.get("dominance_ratio"),
        "response_strength_px": a.get("response_strength_px"),
        "noise_gate_px": a.get("noise_gate_px"),
        "noise_sigma_px": noise.get("sigma_px"),
        "complementary_delta_px": a.get("complementary_delta_px"),
        "return_err_px": a.get("return_err_px"),
        "min_full_markers": a.get("min_full_markers"),
        "avg_full_markers": a.get("avg_full_markers"),
        "marker_lock_ok": a.get("marker_lock_ok"),
        "consistent": a.get("consistent"),
        "observable": a.get("observable"),
        "ok": a.get("ok"),
        "reason": a.get("reason", ""),
    }


def _compact_final_validation(v):
    if not v:
        return v
    results = []
    for r in v.get("results") or ():
        attempts = r.get("attempts") or ()
        noise = r.get("noise") or {}
        accepted = r.get("accepted_attempt") or {}
        results.append({
            "axis": r.get("axis"),
            "ok": r.get("ok"),
            "reason": r.get("reason", ""),
            "attempts_count": len(attempts),
            "noise_sigma_px": noise.get("sigma_px"),
            "noise_gate_px": noise.get("gate_px"),
            "accepted_attempt": _compact_attempt(accepted) if accepted else None,
            "attempts": [_compact_attempt(a) for a in attempts],
        })
    return {
        "ok": v.get("ok"),
        "reason": v.get("reason", ""),
        "candidate_R": v.get("candidate_R"),
        "candidate_idx": v.get("candidate_idx"),
        "pulse_deg": v.get("pulse_deg"),
        "retry_pulses_deg": v.get("retry_pulses_deg"),
        "pulse_s": v.get("pulse_s"),
        "settle_s": v.get("settle_s"),
        "return_max_px": v.get("return_max_px"),
        "noise_samples": v.get("noise_samples"),
        "noise_sigma_mult": v.get("noise_sigma_mult"),
        "noise_floor_px": v.get("noise_floor_px"),
        "results": results,
        "committed": v.get("committed", False),
        "saved": v.get("saved", False),
        "thrust_last": v.get("thrust_last"),
    }


def _compact_summary_for_write(summary):
    if "final_candidate_validation" in summary:
        summary["final_candidate_validation"] = _compact_final_validation(
            summary.get("final_candidate_validation") or {})
    if summary.get("abort_reason") == "final_candidate_validation_failed":
        summary["abort_detail"] = summary.get("final_candidate_validation") or {}
    return summary


def _write_summary(summary):
    # Keep MP memory usage bounded: detailed forensics are append-only journal
    # events; summary is only a compact verdict surface for host post-process.
    compact = _compact_summary_for_write(summary)
    try:
        sentai.fs.write(SUMMARY_NAME, _ser_val(compact))
    except MemoryError:
        sentai.fs.write(SUMMARY_NAME, _ser_val({
            "status": summary.get("status", "SUMMARY_MEMORY_ERROR"),
            "phase": summary.get("phase", ""),
            "abort_reason": summary.get("abort_reason", "summary_memory_error"),
            "summary_error": "memory_error_compact_summary",
        }))


def _compact_axis_response_for_artifact(axis):
    out = {}
    for name in ("roll", "pitch"):
        r = _axis_result(axis, name)
        if not r:
            continue
        out[name] = {
            "complementary_delta_px": r.get("complementary_delta_px"),
            "dominant_image_axis": r.get("dominant_image_axis"),
            "dominant_sign": r.get("dominant_sign"),
            "dominance_ratio": r.get("dominance_ratio"),
            "response_strength_px": r.get("response_strength_px"),
            "min_full_markers": r.get("min_full_markers"),
            "return_err_px": r.get("return_err_px"),
            "return_ok": r.get("return_ok"),
            "z_last_m": r.get("z_last_m"),
        }
    out["orthogonality"] = (axis or {}).get("orthogonality")
    out["pulse_deg"] = (axis or {}).get("pulse_deg")
    out["pulse_s"] = (axis or {}).get("pulse_s")
    out["settle_s"] = (axis or {}).get("settle_s")
    return out


def _calib_artifact(summary):
    cand = summary.get("candidate_scoring") or {}
    best = cand.get("best") or {}
    axis = summary.get("axis_response_smoke") or {}
    final_val = _compact_final_validation(
        summary.get("final_candidate_validation") or {})
    final_rec = summary.get("final_centroid_recenter") or {}
    envelope = summary.get("axis_envelope_survey") or {}
    center_desc = summary.get("center_hold_descend") or {}
    zhold = summary.get("visual_z_hold") or {}

    return {
        "schema_version": 1,
        "task_id": "TD-S10-B3",
        "experiment": "s199_calib_envelope_survey",
        "status": summary.get("status", ""),
        "accepted": bool(
            final_val.get("ok") and
            final_rec.get("ok") and
            envelope.get("ok") and
            center_desc.get("ok")),
        "world": summary.get("world_expected", "sentai_whycon_small"),
        "layout_id": "sentai_whycon_small_centroid_7_marker_v1",
        "image": {
            "w": IMG_W,
            "h": IMG_H,
            "fx": FX,
            "fy": FY,
            "cx": CX,
            "cy": CY,
        },
        "marker": {
            "diameter_m": MARKER_DIAMETER_M,
            "world": MARKER_WORLD,
            "count": len(MARKER_WORLD),
        },
        "R_cam_to_body": best.get("R_cam_to_body"),
        "candidate": {
            "idx": best.get("idx"),
            "score": best.get("score"),
            "margin": cand.get("margin"),
            "det": best.get("det"),
        },
        "axis_response": _compact_axis_response_for_artifact(axis),
        "validation": {
            "final_candidate": final_val,
            "final_recenter_ok": final_rec.get("ok"),
            "final_recenter_reason": final_rec.get("reason", ""),
            "axis_envelope_survey_ok": envelope.get("ok"),
            "axis_envelope_survey_reason": envelope.get("reason", ""),
            "axis_envelope_survey_legs": [
                {
                    "label": leg.get("label"),
                    "ok": leg.get("ok"),
                    "trigger": leg.get("trigger"),
                    "min_fov_margin_px": leg.get("min_fov_margin_px"),
                    "avg_full_markers": leg.get("avg_full_markers"),
                    "err_last_px": leg.get("err_last_px"),
                }
                for leg in (envelope.get("legs") or ())
            ],
            "center_hover_ok": final_rec.get("center_hover_ok"),
            "center_hover_err_last_px": final_rec.get("center_hover_err_last_px"),
            "center_hold_descend_ok": center_desc.get("ok"),
            "center_hold_descend_reason": center_desc.get("reason", ""),
            "center_hold_descend_trigger": center_desc.get("disarm_trigger", ""),
            "camera_referenced_landing_ok": center_desc.get("ok"),
            "camera_referenced_landing_contract":
                "center_hold_active_until_avg_full_markers_reaches_disarm_threshold",
        },
        "safety_envelope": {
            "z_target_m": zhold.get("target_z_m"),
            "z_max_m": zhold.get("z_cam_max_m"),
            "z_last_m": zhold.get("z_cam_last_m"),
            "max_pulse_deg": FINAL_VALIDATE_MAX_PULSE_DEG,
            "final_recenter_max_deg": FINAL_RECENTER_MAX_DEG,
            "center_hold_descent_max_deg": CENTER_HOLD_DESCENT_MAX_DEG,
            "center_hold_descent_target_vz_m_s":
                CENTER_HOLD_DESCENT_TARGET_VZ_M_S,
            "center_hold_descent_thrust_floor":
                CENTER_HOLD_DESCENT_THRUST_FLOOR_U16,
            "axis_min_full_markers": AXIS_MIN_FULL_MARKERS,
            "disarm_full_markers": CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
        },
    }


def _write_calib_artifact(summary):
    artifact = _calib_artifact(summary)
    try:
        sentai.fs.write(CALIB_ARTIFACT_NAME, _ser_val(artifact))
        _j("calibration_artifact_written", {
            "path": CALIB_ARTIFACT_NAME,
            "accepted": artifact.get("accepted"),
            "status": artifact.get("status"),
        })
    except MemoryError:
        sentai.fs.write(CALIB_ARTIFACT_NAME, _ser_val({
            "schema_version": 1,
            "task_id": "TD-S10-B3",
            "status": summary.get("status", ""),
            "accepted": False,
            "error": "memory_error_calibration_artifact",
        }))
        _j("calibration_artifact_write_failed", {
            "reason": "memory_error",
        })


def _write_calib_ini(summary):
    artifact = _calib_artifact(summary)
    if not artifact.get("accepted"):
        return False
    r = artifact.get("R_cam_to_body") or ()
    validation = artifact.get("validation") or {}
    lines = [
        "[sentai.calib]",
        "schema_version=1",
        "task_id=TD-S10-B3",
        "experiment=s199_calib_envelope_survey",
        "layout_id=%s" % artifact.get("layout_id", ""),
        "accepted=true",
        "camera_referenced_landing_ok=%s" %
            ("true" if validation.get("camera_referenced_landing_ok") else "false"),
        "axis_envelope_survey_ok=%s" %
            ("true" if validation.get("axis_envelope_survey_ok") else "false"),
        "r_cam_to_body=%s" % ",".join([str(x) for x in r]),
        "fx=%s" % FX,
        "fy=%s" % FY,
        "cx=%s" % CX,
        "cy=%s" % CY,
        "marker_count=%d" % len(MARKER_WORLD),
        "marker_diameter_m=%s" % MARKER_DIAMETER_M,
        "disarm_full_markers=%s" % CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
        "marker_avg_window=%s" % MARKER_COUNT_AVG_WINDOW,
        "axis_envelope_policy=active_axis_to_fov_margin_other_axis_centered",
        "",
    ]
    sentai.fs.write(CALIB_INI_NAME, "\n".join(lines))
    _j("calib_ini_written", {"path": CALIB_INI_NAME, "lines": len(lines)})
    return True


def _j(event, data=None):
    if data is None:
        data = ""
    try:
        sentai.sim.journal_write(event, data)
    except Exception:
        pass


def _set_phase(summary, phase, reason=""):
    prev = summary.get("phase", "")
    summary["phase"] = phase
    _j("phase_transition", {
        "from": prev,
        "to": phase,
        "reason": reason,
    })


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

    # A3/s199 is an online calibration run: do not load a previously persisted
    # calibration into the decision path. This mission writes its compact
    # artifact at the end, and a later task can decide how to reload it.
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

    _j("phase_setup", {
        "markers": len(MARKER_WORLD),
        "set_marker_world_rc": rc_mw,
        "intrinsics": (FX, FY, CX, CY),
        "marker_diameter_m": MARKER_DIAMETER_M,
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
    fov_margin_min = 999.0

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
        if item["geometry_valid"] == 1 and item["radius_outer"] > 0.0:
            margin = min(
                item["cx"] - item["radius_outer"],
                item["cy"] - item["radius_outer"],
                IMG_W - (item["cx"] + item["radius_outer"]),
                IMG_H - (item["cy"] + item["radius_outer"]))
            if margin < fov_margin_min:
                fov_margin_min = margin
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
        "fov_margin_min_px": 0.0 if fov_margin_min == 999.0 else fov_margin_min,
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


def _feature_has_lock(f, marker_stats=None):
    marker_ok = f["n_full"] > (ACQ_FULL_MARKERS - 1)
    if marker_stats is not None:
        marker_ok = _marker_avg_lock_ok(marker_stats)
    return (marker_ok and
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
    marker_win = _marker_window_new(LOCK_CONSEC_TICKS)
    marker_stats = _marker_window_stats(marker_win)

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
        marker_stats = _marker_window_update(marker_win, f["n_full"])
        has_lock = _feature_has_lock(f, marker_stats)
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
            "n_full_avg": marker_stats["avg_full"],
            "n_full_avg_ready": marker_stats["ready"],
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
        "lock_policy": "running_average",
        "lock_avg_window": LOCK_CONSEC_TICKS,
        "lock_avg_threshold": MARKER_AVG_FULL_LOCK,
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
    marker_win = _marker_window_new()

    for k in range(ticks):
        f = _detect_features()
        marker_stats = _marker_window_update(marker_win, f["n_full"])
        valid = (_marker_avg_lock_ok(marker_stats) and
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
            if (_marker_avg_unsafe(marker_stats) or
                    lost_ticks >= Z_HOLD_LOST_MAX_TICKS):
                abort_reason = "marker_loss"

        _rpyt(0.0, 0.0, 0.0, thrust)
        if k % 3 == 0 or lost_ticks > 0:
            _j("post_lock_brake_tick", {
                "k": k,
                "thrust": thrust,
                "n_full": f["n_full"],
                "n_full_avg": marker_stats["avg_full"],
                "n_full_avg_ready": marker_stats["ready"],
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
    global RUNTIME_Z_TARGET_M
    acq = summary.get("marker_acquisition") or {}
    brake = summary.get("post_lock_brake") or {}
    target_z = Z_HOLD_TARGET_M * Z_CALIB_ALTITUDE_GAIN
    for src in (acq, brake):
        z_src = float(src.get("z_cam_max_m") or 0.0)
        z_src_target = z_src * Z_CALIB_ALTITUDE_GAIN
        if z_src_target > target_z:
            target_z = z_src_target
    if target_z > Z_HOLD_TARGET_MAX_M:
        target_z = Z_HOLD_TARGET_MAX_M
    RUNTIME_Z_TARGET_M = target_z
    _j("phase_visual_z_hold", {
        "target_z_m": target_z,
        "target_policy": "climb_or_hold_upper_visual_z_before_axis_exercises",
        "base_target_z_m": Z_HOLD_TARGET_M,
        "calib_altitude_gain": Z_CALIB_ALTITUDE_GAIN,
        "max_target_z_m": Z_HOLD_TARGET_MAX_M,
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
    marker_win = _marker_window_new()

    for k in range(ticks):
        f = _detect_features()
        marker_stats = _marker_window_update(marker_win, f["n_full"])
        valid = (_marker_avg_lock_ok(marker_stats) and
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

            err = target_z - z
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
            if (_marker_avg_unsafe(marker_stats) or
                    lost_ticks >= Z_HOLD_LOST_MAX_TICKS):
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
                "n_full_avg": marker_stats["avg_full"],
                "n_full_avg_ready": marker_stats["ready"],
                "z_cam_mean_m": f["z_cam_mean_m"],
                "vz_filt_m_s": vz_filt,
                "lost_ticks": lost_ticks,
            })

        if abort_reason:
            break
        sentai.rtos.sleep_ms(TICK_MS)

    target_reached_last = z_last >= (target_z - Z_HOLD_TARGET_TOL_M)
    target_reached_peak = z_max >= (target_z - Z_HOLD_TARGET_TOL_M)
    target_reached = target_reached_last or (
        Z_HOLD_CONTINUE_IF_TARGET_SEEN and target_reached_peak)
    ok = (abort_reason == "" and valid_ticks >= (ticks // 2) and
          z_last > 0.0 and target_reached)
    summary["visual_z_hold"] = {
        "ok": ok,
        "abort_reason": abort_reason,
        "target_z_m": target_z,
        "base_target_z_m": Z_HOLD_TARGET_M,
        "calib_altitude_gain": Z_CALIB_ALTITUDE_GAIN,
        "max_target_z_m": Z_HOLD_TARGET_MAX_M,
        "target_tol_m": Z_HOLD_TARGET_TOL_M,
        "target_reached": target_reached,
        "target_reached_last": target_reached_last,
        "target_reached_peak": target_reached_peak,
        "continue_if_target_seen": Z_HOLD_CONTINUE_IF_TARGET_SEEN,
        "target_policy": "climb_or_hold_upper_visual_z_before_axis_exercises",
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
        err = RUNTIME_Z_TARGET_M - z
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


def _descent_thrust_from_feature(f, z_prev, vz_filt, base_thrust):
    return _vertical_rate_thrust_from_feature(
        f, z_prev, vz_filt, base_thrust,
        CENTER_HOLD_DESCENT_TARGET_VZ_M_S)


def _vertical_rate_thrust_from_feature(f, z_prev, vz_filt, base_thrust,
                                       target_vz_m_s):
    z = f["z_cam_mean_m"]
    if z > 0.0 and z_prev > 0.0:
        vz = (z - z_prev) * (1000.0 / TICK_MS)
        vz_filt = (1.0 - VZ_LPF_ALPHA) * vz_filt + VZ_LPF_ALPHA * vz
    elif z > 0.0:
        vz_filt = 0.0
    if z > 0.0:
        z_prev = z
        vz_err = vz_filt - target_vz_m_s
        cmd = (base_thrust -
               CENTER_HOLD_DESCENT_KD_THRUST_PER_M_S * vz_err)
        thrust = int(cmd)
    else:
        thrust = CENTER_HOLD_DESCENT_THRUST_FLOOR_U16
    if thrust < CENTER_HOLD_DESCENT_THRUST_FLOOR_U16:
        thrust = CENTER_HOLD_DESCENT_THRUST_FLOOR_U16
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
        "fov_margin_min_px": f.get("fov_margin_min_px", 0.0),
    }


def _marker_window_new(size=MARKER_COUNT_AVG_WINDOW):
    return {
        "size": size,
        "values": [0] * size,
        "idx": 0,
        "count": 0,
        "sum": 0.0,
        "min": 99,
    }


def _marker_window_update(w, n_full):
    idx = w["idx"]
    if w["count"] >= w["size"]:
        w["sum"] -= w["values"][idx]
    else:
        w["count"] += 1
    w["values"][idx] = n_full
    w["sum"] += n_full
    w["idx"] = (idx + 1) % w["size"]
    if n_full < w["min"]:
        w["min"] = n_full
    return _marker_window_stats(w)


def _marker_window_stats(w):
    avg = 0.0
    if w["count"] > 0:
        avg = w["sum"] / float(w["count"])
    return {
        "count": w["count"],
        "size": w["size"],
        "avg_full": avg,
        "min_full": 0 if w["min"] == 99 else w["min"],
        "ready": w["count"] >= w["size"],
    }


def _marker_avg_lock_ok(stats, threshold=MARKER_AVG_FULL_LOCK):
    return stats.get("ready", False) and stats.get("avg_full", 0.0) > threshold


def _marker_avg_unsafe(stats, threshold=MIN_FULL_MARKERS):
    return stats.get("ready", False) and stats.get("avg_full", 0.0) < threshold


def _stream_axis_segment(label, roll_deg, pitch_deg, duration_s, z_prev, vz_filt):
    ticks = int(duration_s * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1
    first = None
    last = None
    n_full_min = 99
    n_full_sum = 0.0
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
        n_full_sum += f["n_full"]
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
    stats = {
        "ticks": ticks,
        "min_full": 0 if n_full_min == 99 else n_full_min,
        "avg_full": n_full_sum / float(ticks),
        "avg_threshold": AXIS_MIN_AVG_FULL_MARKERS,
        "hard_min_threshold": AXIS_HARD_MIN_FULL_MARKERS,
    }
    stats["marker_lock_ok"] = (
        stats["min_full"] >= AXIS_HARD_MIN_FULL_MARKERS and
        stats["avg_full"] > AXIS_MIN_AVG_FULL_MARKERS)
    _j("axis_segment_summary", {
        "label": label,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "stats": stats,
    })
    return first, last, stats, z_prev, vz_filt, thrust


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
        first, last, seg, z_prev, vz_filt, thrust = _stream_axis_segment(
            axis_name + "_recenter_%d" % p,
            roll, pitch, AXIS_RECENTER_PULSE_S,
            z_prev, vz_filt)
        if seg["min_full"] < n_full_min:
            n_full_min = seg["min_full"]
        # Short neutral settle after the corrective pulse.
        first_s, last_s, seg_s, z_prev, vz_filt, thrust = _stream_axis_segment(
            axis_name + "_recenter_settle_%d" % p,
            0.0, 0.0, AXIS_RECENTER_PULSE_S,
            z_prev, vz_filt)
        if seg_s["min_full"] < n_full_min:
            n_full_min = seg_s["min_full"]
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
        if axis_base["n_full"] < AXIS_HARD_MIN_FULL_MARKERS:
            ok = False
            _j("axis_response_abort", {
                "axis": name,
                "reason": "weak_marker_lock_before_axis_pulse",
                "feature": axis_base,
            })
            break
        pos_first, pos_last, pos_stats, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_pos",
                                 roll_cmd, pitch_cmd, AXIS_PULSE_S,
                                 z_prev, vz_filt))
        neg_first, neg_last, neg_stats, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_neg",
                                 -roll_cmd, -pitch_cmd, AXIS_PULSE_S,
                                 z_prev, vz_filt))
        settle_first, settle_last, settle_stats, z_prev, vz_filt, thrust = (
            _stream_axis_segment(name + "_settle",
                                 0.0, 0.0, AXIS_SETTLE_S,
                                 z_prev, vz_filt))

        dx_pos = pos_last["cx"] - baseline["cx"]
        dy_pos = pos_last["cy"] - baseline["cy"]
        dx_neg = neg_last["cx"] - pos_last["cx"]
        dy_neg = neg_last["cy"] - pos_last["cy"]
        min_full = pos_stats["min_full"]
        if neg_stats["min_full"] < min_full:
            min_full = neg_stats["min_full"]
        if settle_stats["min_full"] < min_full:
            min_full = settle_stats["min_full"]
        total_ticks = (pos_stats["ticks"] + neg_stats["ticks"] +
                       settle_stats["ticks"])
        avg_full = ((pos_stats["avg_full"] * pos_stats["ticks"] +
                     neg_stats["avg_full"] * neg_stats["ticks"] +
                     settle_stats["avg_full"] * settle_stats["ticks"]) /
                    float(total_ticks))
        marker_lock_ok = (
            min_full >= AXIS_HARD_MIN_FULL_MARKERS and
            avg_full > AXIS_MIN_AVG_FULL_MARKERS)
        if not marker_lock_ok:
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
            "avg_full_markers": avg_full,
            "marker_lock_ok": marker_lock_ok,
            "segment_marker_stats": {
                "pos": pos_stats,
                "neg": neg_stats,
                "settle": settle_stats,
            },
            "z_last_m": recenter["feature"]["z_cam_mean_m"],
            "thrust_last": thrust,
        }
        results.append(r)
        _j("axis_response", r)
        if not axis_sign_ok:
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


def _phase_axis_response_retry_settle(summary, start_thrust, attempt):
    _j("phase_axis_response_retry_settle", {
        "attempt": attempt,
        "duration_s": AXIS_RESPONSE_RETRY_SETTLE_S,
        "reason": "axis_response_not_accepted",
    })
    ticks = int(AXIS_RESPONSE_RETRY_SETTLE_S * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1
    f0 = _detect_features()
    z_prev = f0.get("z_cam_mean_m", 0.0)
    vz_filt = 0.0
    thrust = start_thrust
    n_full_min = 99
    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        if cur["n_full"] < n_full_min:
            n_full_min = cur["n_full"]
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(0.0, 0.0, 0.0, thrust)
        if k % 10 == 0:
            _j("axis_response_retry_settle_tick", {
                "attempt": attempt,
                "k": k,
                "feature": cur,
                "thrust": thrust,
                "vz_filt_m_s": vz_filt,
            })
        sentai.rtos.sleep_ms(TICK_MS)
    out = {
        "attempt": attempt,
        "ticks": ticks,
        "n_full_min": 0 if n_full_min == 99 else n_full_min,
        "thrust_last": thrust,
    }
    _j("phase_axis_response_retry_settle", out)
    return thrust


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


def _ibvs_centroid_command_to_target(cur, roll_vec, pitch_vec, target_cx,
                                     target_cy, gain, max_deg, deadband_px):
    err_x = target_cx - cur["cx"]
    err_y = target_cy - cur["cy"]
    err = (err_x * err_x + err_y * err_y) ** 0.5
    cmd_err_x = err_x if abs(err_x) > deadband_px else 0.0
    cmd_err_y = err_y if abs(err_y) > deadband_px else 0.0

    z = cur.get("z_cam_mean_m", 0.0)
    z_gain = 1.0
    if z > 0.0:
        z_gain = z / IBVS_Z_REF_M
        z_gain = _clip(z_gain, IBVS_Z_GAIN_MIN, IBVS_Z_GAIN_MAX)

    # The pulse response is an image-velocity proxy.  Longer lateral hold has
    # opposite settled-position sign in this simulator, so use the sustained
    # response convention for centering and landing.
    rx = IBVS_SUSTAINED_RESPONSE_SIGN * roll_vec[0]
    ry = IBVS_SUSTAINED_RESPONSE_SIGN * roll_vec[1]
    px = IBVS_SUSTAINED_RESPONSE_SIGN * pitch_vec[0]
    py = IBVS_SUSTAINED_RESPONSE_SIGN * pitch_vec[1]

    target_x = gain * z_gain * cmd_err_x
    target_y = gain * z_gain * cmd_err_y
    lam2 = IBVS_DAMPING_PX_PER_DEG * IBVS_DAMPING_PX_PER_DEG

    # Damped least squares: u = J^T (J J^T + lambda^2 I)^-1 e.
    a11 = rx * rx + px * px + lam2
    a12 = rx * ry + px * py
    a22 = ry * ry + py * py + lam2
    det = a11 * a22 - a12 * a12
    roll_cmd = 0.0
    pitch_cmd = 0.0
    if abs(det) > 0.0001:
        y0 = (a22 * target_x - a12 * target_y) / det
        y1 = (-a12 * target_x + a11 * target_y) / det
        roll_cmd = rx * y0 + ry * y1
        pitch_cmd = px * y0 + py * y1

    roll_cmd = _clip(roll_cmd, -max_deg, max_deg)
    pitch_cmd = _clip(pitch_cmd, -max_deg, max_deg)
    return {
        "roll_deg": roll_cmd,
        "pitch_deg": pitch_cmd,
        "err_x_px": err_x,
        "err_y_px": err_y,
        "err_px": err,
        "target_cx": target_cx,
        "target_cy": target_cy,
        "target_delta_px": (target_x, target_y),
        "z_gain": z_gain,
        "dls_det": det,
        "response_roll_px_per_deg": (rx, ry),
        "response_pitch_px_per_deg": (px, py),
    }


def _ibvs_centroid_command(cur, roll_vec, pitch_vec, gain, max_deg, deadband_px):
    return _ibvs_centroid_command_to_target(
        cur, roll_vec, pitch_vec, CX, CY, gain, max_deg, deadband_px)


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
    if cur["n_full"] < AXIS_HARD_MIN_FULL_MARKERS or cur["z_cam_mean_m"] <= 0.0:
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
    err_win = []
    err_win = []
    feature_initial = cur
    feature_min = cur
    z_prev = cur["z_cam_mean_m"]
    vz_filt = 0.0
    prev_err_x = err_x
    prev_err_y = err_y
    thrust = start_thrust
    n_full_min = cur["n_full"]
    marker_win = _marker_window_new()
    marker_stats = _marker_window_update(marker_win, cur["n_full"])
    abort_reason = ""
    first_cmd = None
    last_cmd = None
    ticks = int(CENTROID_VALIDATION_S * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1

    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        marker_stats = _marker_window_update(marker_win, cur["n_full"])
        if cur["n_full"] < n_full_min:
            n_full_min = cur["n_full"]
        if (_marker_avg_unsafe(marker_stats) or
                cur["z_cam_mean_m"] <= 0.0):
            abort_reason = "marker_loss"
            _rpyt(0.0, 0.0, 0.0, T_HOVER_VISUAL_U16)
            break

        ctrl = _ibvs_centroid_command(
            cur, roll_vec, pitch_vec,
            CENTROID_VALIDATION_KP,
            CENTROID_VALIDATION_MAX_DEG,
            CENTROID_VALIDATION_DEADBAND_PX)
        err_x = ctrl["err_x_px"]
        err_y = ctrl["err_y_px"]
        derr_x = err_x - prev_err_x
        derr_y = err_y - prev_err_y
        prev_err_x = err_x
        prev_err_y = err_y
        roll_cmd = ctrl["roll_deg"]
        pitch_cmd = ctrl["pitch_deg"]
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)

        err = (err_x * err_x + err_y * err_y) ** 0.5
        if err < err_min:
            err_min = err
            feature_min = cur
        if err > err_max:
            err_max = err
        err_win.append(err)
        if len(err_win) > MARKER_COUNT_AVG_WINDOW:
            err_win.pop(0)
        cmd = {
            "k": k,
            "roll_deg": roll_cmd,
            "pitch_deg": pitch_cmd,
            "err_px": err,
            "feature": cur,
            "ibvs": ctrl,
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
    marker_lock_ok = (n_full_min >= AXIS_HARD_MIN_FULL_MARKERS and
                      marker_stats["avg_full"] > AXIS_MIN_AVG_FULL_MARKERS)
    ok = (abort_reason == "" and
          marker_lock_ok and
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
        "avg_full_markers": marker_stats["avg_full"],
        "marker_lock_ok": marker_lock_ok,
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
    # A3 observes command responses, then scores rows of R_cam_to_body.  The
    # row labels below name the body-axis hypothesis being tested; the command
    # labels record which RPYT perturbation produced the image evidence.
    specs = (
        ("pitch", "body_x_from_pitch_response", 0),
        ("roll", "body_y_from_roll_response", 1),
    )
    for command_axis, body_axis_hypothesis, row_idx in specs:
        o = obs.get(command_axis)
        row = (R[row_idx * 3], R[row_idx * 3 + 1], R[row_idx * 3 + 2])
        exp_axis, exp_sign = _expected_from_row(row)
        axis_penalty = 0.0 if exp_axis == o["axis"] else 10.0
        sign_penalty = 0.0 if exp_sign == o["sign"] else 1.0
        score += axis_penalty + sign_penalty
        details.append({
            "command_axis": command_axis,
            "body_axis_hypothesis": body_axis_hypothesis,
            "candidate_row": row_idx,
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


def _phase_optical_axis_validation(summary):
    cand = summary.get("candidate_scoring") or {}
    _j("phase_optical_axis_validation", {
        "expected_body_z_camera_z_sign":
            OPTICAL_AXIS_EXPECT_BODY_Z_IN_CAMERA_Z_SIGN,
    })
    if not cand.get("ok", False):
        out = {
            "ok": False,
            "reason": "skipped_candidate_not_accepted",
        }
        summary["optical_axis_validation"] = out
        _j("phase_optical_axis_validation", out)
        return False

    best = cand.get("best") or {}
    R = best.get("R_cam_to_body")
    if R is None or len(R) != 9:
        out = {
            "ok": False,
            "reason": "missing_candidate_R",
        }
        summary["optical_axis_validation"] = out
        _j("phase_optical_axis_validation", out)
        return False

    f = _detect_features()
    pose_valid = 0
    tz_sum = 0.0
    tz_min = 999.0
    for d in f.get("detections") or ():
        if d.get("full_visible") and d.get("pose_valid") == 1 and d.get("tz", 0.0) > 0.0:
            pose_valid += 1
            tz = float(d.get("tz") or 0.0)
            tz_sum += tz
            if tz < tz_min:
                tz_min = tz
    tz_mean = tz_sum / pose_valid if pose_valid > 0 else 0.0
    body_z_row = (float(R[6]), float(R[7]), float(R[8]))
    body_z_camera_z_sign = 1 if body_z_row[2] >= 0.0 else -1
    axis_ok = (body_z_camera_z_sign ==
               OPTICAL_AXIS_EXPECT_BODY_Z_IN_CAMERA_Z_SIGN)
    pose_ok = (pose_valid > (OPTICAL_AXIS_MIN_POSE_VALID - 1) and
               tz_mean >= OPTICAL_AXIS_MIN_MEAN_TZ_M)
    out = {
        "ok": axis_ok and pose_ok,
        "reason": "" if (axis_ok and pose_ok) else "optical_axis_or_pose_tz_mismatch",
        "candidate_R": R,
        "body_z_row": body_z_row,
        "body_z_camera_z_sign": body_z_camera_z_sign,
        "expected_body_z_camera_z_sign":
            OPTICAL_AXIS_EXPECT_BODY_Z_IN_CAMERA_Z_SIGN,
        "axis_ok": axis_ok,
        "pose_valid_full_markers": pose_valid,
        "pose_valid_min_required": OPTICAL_AXIS_MIN_POSE_VALID,
        "tz_mean_m": tz_mean,
        "tz_min_m": 0.0 if tz_min == 999.0 else tz_min,
        "tz_mean_min_m": OPTICAL_AXIS_MIN_MEAN_TZ_M,
        "pose_ok": pose_ok,
        "feature": _feature_compact(f),
    }
    summary["optical_axis_validation"] = out
    _j("phase_optical_axis_validation", out)
    return out["ok"]


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


def _measure_centroid_noise(label, z_prev, vz_filt):
    ticks = FINAL_VALIDATE_NOISE_SAMPLES
    if ticks < 2:
        ticks = 2
    samples = []
    n_full_min = 99
    n_full_sum = 0.0
    thrust = T_HOVER_VISUAL_U16
    for k in range(ticks):
        f = _detect_features()
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(0.0, 0.0, 0.0, thrust)
        fc = _feature_compact(f)
        if fc["n_full"] < n_full_min:
            n_full_min = fc["n_full"]
        n_full_sum += fc["n_full"]
        if fc["n_full"] >= AXIS_HARD_MIN_FULL_MARKERS:
            samples.append((fc["cx"], fc["cy"]))
        if k == 0 or k == ticks - 1:
            _j("final_validation_noise_tick", {
                "label": label,
                "k": k,
                "feature": fc,
                "thrust": thrust,
                "vz_filt_m_s": vz_filt,
            })
        sentai.rtos.sleep_ms(TICK_MS)

    if len(samples) < 2:
        out = {
            "ok": False,
            "samples": len(samples),
            "n_full_min": n_full_min,
            "avg_full_markers": n_full_sum / float(ticks),
            "sigma_px": 999.0,
            "gate_px": 999.0,
            "reason": "insufficient_marker_lock_for_noise_estimate",
        }
        _j("final_validation_noise", out)
        return out, z_prev, vz_filt, thrust

    n = len(samples)
    mean_x = sum([p[0] for p in samples]) / n
    mean_y = sum([p[1] for p in samples]) / n
    raw_var = 0.0
    for p in samples:
        dx = p[0] - mean_x
        dy = p[1] - mean_y
        raw_var += dx * dx + dy * dy
    raw_var = raw_var / (n - 1)
    raw_sigma = raw_var ** 0.5

    drift_x_per_tick = (samples[-1][0] - samples[0][0]) / float(n - 1)
    drift_y_per_tick = (samples[-1][1] - samples[0][1]) / float(n - 1)
    drift_px_per_s = ((drift_x_per_tick * drift_x_per_tick +
                       drift_y_per_tick * drift_y_per_tick) ** 0.5 *
                      (1000.0 / TICK_MS))

    # Noise gate must capture jitter, not slow natural drift.  Remove the
    # linear trend observed during neutral RPY before estimating sigma.
    var = 0.0
    for i, p in enumerate(samples):
        pred_x = samples[0][0] + drift_x_per_tick * float(i)
        pred_y = samples[0][1] + drift_y_per_tick * float(i)
        dx = p[0] - pred_x
        dy = p[1] - pred_y
        var += dx * dx + dy * dy
    var = var / (n - 1)
    sigma = var ** 0.5
    gate = FINAL_VALIDATE_NOISE_SIGMA_MULT * sigma
    if gate < FINAL_VALIDATE_NOISE_FLOOR_PX:
        gate = FINAL_VALIDATE_NOISE_FLOOR_PX
    avg_full = n_full_sum / float(ticks)
    marker_lock_ok = (n_full_min >= AXIS_HARD_MIN_FULL_MARKERS and
                      avg_full > AXIS_MIN_AVG_FULL_MARKERS)
    out = {
        "ok": marker_lock_ok,
        "samples": len(samples),
        "n_full_min": n_full_min,
        "avg_full_markers": avg_full,
        "marker_lock_ok": marker_lock_ok,
        "mean_px": (mean_x, mean_y),
        "sigma_px": sigma,
        "raw_sigma_px": raw_sigma,
        "drift_px_per_s": drift_px_per_s,
        "drift_px_per_tick": (drift_x_per_tick, drift_y_per_tick),
        "sigma_mult": FINAL_VALIDATE_NOISE_SIGMA_MULT,
        "floor_px": FINAL_VALIDATE_NOISE_FLOOR_PX,
        "gate_px": gate,
        "reason": "" if marker_lock_ok else "weak_marker_lock_during_noise_estimate",
    }
    _j("final_validation_noise", out)
    return out, z_prev, vz_filt, thrust


def _final_validate_attempt(name, pulse_deg, row_idx, R, noise_gate_px,
                            z_prev, vz_filt):
    roll_cmd = pulse_deg if name == "roll" else 0.0
    pitch_cmd = pulse_deg if name == "pitch" else 0.0

    f_axis_base = _detect_features()
    axis_base = _feature_compact(f_axis_base)
    if axis_base["n_full"] < AXIS_HARD_MIN_FULL_MARKERS:
        return {
            "axis": name,
            "axis_baseline": axis_base,
            "pulse_deg": pulse_deg,
            "ok": False,
            "reason": "weak_marker_lock_before_validation_pulse",
        }, z_prev, vz_filt, T_HOVER_VISUAL_U16
    if axis_base["z_cam_mean_m"] > 0.0:
        z_prev = axis_base["z_cam_mean_m"]

    pos_first, pos_last, pos_stats, z_prev, vz_filt, thrust = (
        _stream_axis_segment("final_" + name + "_pos_%.1f" % pulse_deg,
                             roll_cmd, pitch_cmd,
                             FINAL_VALIDATE_PULSE_S,
                             z_prev, vz_filt))
    neg_first, neg_last, neg_stats, z_prev, vz_filt, thrust = (
        _stream_axis_segment("final_" + name + "_neg_%.1f" % pulse_deg,
                             -roll_cmd, -pitch_cmd,
                             FINAL_VALIDATE_PULSE_S,
                             z_prev, vz_filt))
    settle_first, settle_last, settle_stats, z_prev, vz_filt, thrust = (
        _stream_axis_segment("final_" + name + "_settle_%.1f" % pulse_deg,
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
    min_full = pos_stats["min_full"]
    if neg_stats["min_full"] < min_full:
        min_full = neg_stats["min_full"]
    if settle_stats["min_full"] < min_full:
        min_full = settle_stats["min_full"]
    total_ticks = (pos_stats["ticks"] + neg_stats["ticks"] +
                   settle_stats["ticks"])
    avg_full = ((pos_stats["avg_full"] * pos_stats["ticks"] +
                 neg_stats["avg_full"] * neg_stats["ticks"] +
                 settle_stats["avg_full"] * settle_stats["ticks"]) /
                float(total_ticks))
    marker_lock_ok = (
        min_full >= AXIS_HARD_MIN_FULL_MARKERS and
        avg_full > AXIS_MIN_AVG_FULL_MARKERS)

    consistent = (obs_axis == exp_axis and obs_sign == exp_sign and
                  dominance >= AXIS_DOMINANCE_RATIO_MIN and
                  marker_lock_ok and
                  return_err <= FINAL_VALIDATE_RETURN_MAX_PX)
    observable = strength >= noise_gate_px
    axis_ok = consistent and observable

    r = {
        "axis": name,
        "axis_baseline": axis_base,
        "cmd_pos": (roll_cmd, pitch_cmd),
        "cmd_neg": (-roll_cmd, -pitch_cmd),
        "pulse_deg": pulse_deg,
        "expected_image_axis": exp_axis,
        "expected_sign": exp_sign,
        "observed_image_axis": obs_axis,
        "observed_sign": obs_sign,
        "dominance_ratio": dominance,
        "response_strength_px": strength,
        "noise_gate_px": noise_gate_px,
        "complementary_delta_px": (comp_dx, comp_dy),
        "return_err_px": return_err,
        "min_full_markers": min_full,
        "avg_full_markers": avg_full,
        "marker_lock_ok": marker_lock_ok,
        "segment_marker_stats": {
            "pos": pos_stats,
            "neg": neg_stats,
            "settle": settle_stats,
        },
        "consistent": consistent,
        "observable": observable,
        "ok": axis_ok,
        "thrust_last": thrust,
    }
    if not consistent:
        r["reason"] = "validation_motion_mismatch"
    elif not observable:
        r["reason"] = "response_below_noise_gate"
    else:
        r["reason"] = ""
    return r, z_prev, vz_filt, thrust


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
        ("pitch", 0, 0),
        ("roll", 1, 1),
    )
    pulse_schedule = (FINAL_VALIDATE_PULSE_DEG,) + FINAL_VALIDATE_RETRY_PULSES_DEG

    for name, command_col, row_idx in specs:
        axis_ok = False
        axis_attempts = []
        noise, z_prev, vz_filt, thrust = _measure_centroid_noise(
            "final_" + name, z_prev, vz_filt)
        if not noise.get("ok", False):
            axis_result = {
                "axis": name,
                "ok": False,
                "attempts": [],
                "noise": noise,
                "accepted_attempt": None,
                "reason": noise.get("reason", "noise_estimate_failed"),
            }
            results.append(axis_result)
            _j("final_candidate_axis", axis_result)
            ok = False
            break
        for pulse_deg in pulse_schedule:
            if pulse_deg > FINAL_VALIDATE_MAX_PULSE_DEG:
                continue
            r, z_prev, vz_filt, thrust = _final_validate_attempt(
                name, pulse_deg, row_idx, R, noise.get("gate_px", 999.0),
                z_prev, vz_filt)
            r["attempt"] = len(axis_attempts) + 1
            r["command_col"] = command_col
            r["noise"] = noise
            axis_attempts.append(r)
            _j("final_candidate_axis_attempt", r)
            if r.get("ok", False):
                axis_ok = True
                break
            if r.get("consistent", False) and len(axis_attempts) >= 2:
                prev = axis_attempts[-2]
                repeat_consistent = (
                    prev.get("consistent", False) and
                    prev.get("observed_image_axis") == r.get("observed_image_axis") and
                    prev.get("observed_sign") == r.get("observed_sign") and
                    float(r.get("response_strength_px") or 0.0) >=
                    1.10 * float(prev.get("response_strength_px") or 0.0) and
                    float(r.get("response_strength_px") or 0.0) >=
                    0.75 * float(r.get("noise_gate_px") or 999.0)
                )
                if repeat_consistent:
                    r["ok"] = True
                    r["observable"] = True
                    r["reason"] = "accepted_repeat_consistent_adaptive_excitation"
                    r["acceptance_mode"] = "repeat_consistent_adaptive_excitation"
                    axis_ok = True
                    break
            if not r.get("consistent", False):
                break
            _j("final_candidate_axis_retry", {
                "axis": name,
                "reason": r.get("reason", ""),
                "response_strength_px": r.get("response_strength_px", 0.0),
                "noise_gate_px": r.get("noise_gate_px", 0.0),
                "next_attempt": len(axis_attempts) + 1,
            })

        axis_result = {
            "axis": name,
            "ok": axis_ok,
            "attempts": axis_attempts,
            "noise": noise,
            "accepted_attempt": axis_attempts[-1] if axis_ok else None,
            "reason": "" if axis_ok else axis_attempts[-1].get("reason", "validation_failed"),
        }
        results.append(axis_result)
        _j("final_candidate_axis", axis_result)
        if not axis_ok:
            ok = False
            break

    out = {
        "ok": ok,
        "reason": "" if ok else "validation_motion_mismatch",
        "candidate_R": R,
        "candidate_idx": best.get("idx"),
        "pulse_deg": FINAL_VALIDATE_PULSE_DEG,
        "retry_pulses_deg": FINAL_VALIDATE_RETRY_PULSES_DEG,
        "pulse_s": FINAL_VALIDATE_PULSE_S,
        "settle_s": FINAL_VALIDATE_SETTLE_S,
        "return_max_px": FINAL_VALIDATE_RETURN_MAX_PX,
        "noise_samples": FINAL_VALIDATE_NOISE_SAMPLES,
        "noise_sigma_mult": FINAL_VALIDATE_NOISE_SIGMA_MULT,
        "noise_floor_px": FINAL_VALIDATE_NOISE_FLOOR_PX,
        "results": results,
        "committed": False,
        "saved": False,
        "thrust_last": thrust,
    }
    summary["final_candidate_validation"] = out
    _j("phase_final_candidate_validation", out)
    return ok, thrust


def _phase_final_centroid_recenter(summary, start_thrust):
    axis = summary.get("axis_response_smoke") or {}
    final_val = summary.get("final_candidate_validation") or {}
    _j("phase_final_centroid_recenter", {
        "target_px": (CX, CY),
        "tol_px": FINAL_RECENTER_TOL_PX,
        "duration_s": FINAL_RECENTER_DURATION_S,
    })
    if not final_val.get("ok", False):
        rec = {
            "ok": False,
            "reason": "skipped_final_validation_not_accepted",
            "ticks": 0,
            "err_px": 999.0,
            "thrust_last": start_thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, start_thrust

    roll_vec, pitch_vec = _axis_response_vectors(axis)
    if roll_vec is None or pitch_vec is None:
        rec = {
            "ok": False,
            "reason": "missing_axis_response",
            "ticks": 0,
            "err_px": 999.0,
            "thrust_last": start_thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, start_thrust

    roll_strength = _response_strength(roll_vec)
    pitch_strength = _response_strength(pitch_vec)
    if (roll_strength < FINAL_RECENTER_RESPONSE_MIN_PX or
            pitch_strength < FINAL_RECENTER_RESPONSE_MIN_PX):
        rec = {
            "ok": False,
            "reason": "weak_response",
            "ticks": 0,
            "roll_vec_px": roll_vec,
            "pitch_vec_px": pitch_vec,
            "roll_strength_px": roll_strength,
            "pitch_strength_px": pitch_strength,
            "err_px": 999.0,
            "thrust_last": start_thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, start_thrust

    f0 = _detect_features()
    cur = _feature_compact(f0)
    if cur["n_full"] < AXIS_HARD_MIN_FULL_MARKERS or cur["z_cam_mean_m"] <= 0.0:
        rec = {
            "ok": False,
            "reason": "no_marker_lock",
            "initial_feature": cur,
            "ticks": 0,
            "err_px": 999.0,
            "thrust_last": start_thrust,
        }
        summary["final_centroid_recenter"] = rec
        _j("phase_final_centroid_recenter", rec)
        return False, start_thrust

    err_x = CX - cur["cx"]
    err_y = CY - cur["cy"]
    err = (err_x * err_x + err_y * err_y) ** 0.5
    err_initial = err
    err_min = err
    err_max = err
    err_win = []
    feature_initial = cur
    feature_best = cur
    feature_last = cur
    z_prev = cur["z_cam_mean_m"]
    vz_filt = 0.0
    thrust = start_thrust
    n_full_min = cur["n_full"]
    marker_win = _marker_window_new()
    marker_stats = _marker_window_update(marker_win, cur["n_full"])
    abort_reason = ""
    first_cmd = None
    last_cmd = None
    ticks_done = 0
    ticks = int(FINAL_RECENTER_DURATION_S * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1

    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        marker_stats = _marker_window_update(marker_win, cur["n_full"])
        if cur["n_full"] < n_full_min:
            n_full_min = cur["n_full"]
        if (_marker_avg_unsafe(marker_stats) or
                cur["z_cam_mean_m"] <= 0.0):
            abort_reason = "marker_loss"
            _rpyt(0.0, 0.0, 0.0, T_HOVER_VISUAL_U16)
            break

        ctrl = _ibvs_centroid_command(
            cur, roll_vec, pitch_vec,
            FINAL_RECENTER_KP,
            FINAL_RECENTER_MAX_DEG,
            FINAL_RECENTER_DEADBAND_PX)
        err_x = ctrl["err_x_px"]
        err_y = ctrl["err_y_px"]
        err = ctrl["err_px"]
        roll_cmd = ctrl["roll_deg"]
        pitch_cmd = ctrl["pitch_deg"]
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)

        if err < err_min:
            err_min = err
            feature_best = cur
        if err > err_max:
            err_max = err
        err_win.append(err)
        if len(err_win) > MARKER_COUNT_AVG_WINDOW:
            err_win.pop(0)
        cmd = {
            "k": k,
            "roll_deg": roll_cmd,
            "pitch_deg": pitch_cmd,
            "err_px": err,
            "feature": cur,
            "ibvs": ctrl,
        }
        if first_cmd is None:
            first_cmd = cmd
        last_cmd = cmd
        feature_last = cur
        ticks_done = k + 1
        if k % 5 == 0 or err <= FINAL_RECENTER_TOL_PX:
            _j("final_recenter_tick", cmd)
        sentai.rtos.sleep_ms(TICK_MS)

    _rpyt(0.0, 0.0, 0.0, thrust)
    f_last = _detect_features()
    last_feature = _feature_compact(f_last)
    if last_feature["n_full"] < n_full_min:
        n_full_min = last_feature["n_full"]
    if last_feature["n_full"] > 0:
        last_err_x = CX - last_feature["cx"]
        last_err_y = CY - last_feature["cy"]
        err_last = (last_err_x * last_err_x + last_err_y * last_err_y) ** 0.5
        feature_last = last_feature
    else:
        err_last = 999.0
    if err_last < err_min:
        err_min = err_last
        feature_best = last_feature
    if err_last > err_max:
        err_max = err_last

    improvement = err_initial - err_last
    recenter_trend_ok = (err_last <= err_initial or err_min < err_initial)
    if len(err_win) >= MARKER_COUNT_AVG_WINDOW:
        half = MARKER_COUNT_AVG_WINDOW // 2
        old_avg = sum(err_win[:half]) / float(half)
        new_avg = sum(err_win[half:]) / float(len(err_win) - half)
        recenter_trend_ok = recenter_trend_ok or new_avg <= old_avg
    marker_lock_ok = (n_full_min >= AXIS_HARD_MIN_FULL_MARKERS and
                      marker_stats["avg_full"] > AXIS_MIN_AVG_FULL_MARKERS)
    recenter_ok = (abort_reason == "" and
                   marker_lock_ok and
                   recenter_trend_ok)
    hover_ok = False
    hover_ticks = 0
    hover_stable_ticks = 0
    hover_err_max = 0.0
    hover_err_min = err_last
    hover_err_last = err_last
    hover_err_win = []
    hover_reason = ""
    if recenter_ok:
        hover_ticks_req = int(FINAL_CENTER_HOVER_S * 1000 / TICK_MS)
        if hover_ticks_req < 1:
            hover_ticks_req = 1
        hover_ticks_max = int(FINAL_CENTER_HOVER_MAX_S * 1000 / TICK_MS)
        if hover_ticks_max < hover_ticks_req:
            hover_ticks_max = hover_ticks_req
        for hk in range(hover_ticks_max):
            f = _detect_features()
            cur = _feature_compact(f)
            marker_stats = _marker_window_update(marker_win, cur["n_full"])
            if cur["n_full"] < n_full_min:
                n_full_min = cur["n_full"]
            if (_marker_avg_unsafe(marker_stats) or
                    cur["z_cam_mean_m"] <= 0.0):
                hover_reason = "marker_loss_during_center_hover"
                _rpyt(0.0, 0.0, 0.0, T_HOVER_VISUAL_U16)
                break
            ctrl = _ibvs_centroid_command(
                cur, roll_vec, pitch_vec,
                FINAL_RECENTER_KP,
                FINAL_RECENTER_MAX_DEG,
                FINAL_RECENTER_DEADBAND_PX)
            err_x = ctrl["err_x_px"]
            err_y = ctrl["err_y_px"]
            hover_err_last = ctrl["err_px"]
            if hover_err_last > hover_err_max:
                hover_err_max = hover_err_last
            if hover_err_last < hover_err_min:
                hover_err_min = hover_err_last
            hover_err_win.append(hover_err_last)
            if len(hover_err_win) > MARKER_COUNT_AVG_WINDOW:
                hover_err_win.pop(0)
            roll_cmd = ctrl["roll_deg"]
            pitch_cmd = ctrl["pitch_deg"]
            thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
            _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)
            hover_ticks = hk + 1
            feature_last = cur
            if hk % 5 == 0:
                _j("final_center_hover_tick", {
                    "k": hk,
                    "roll_deg": roll_cmd,
                    "pitch_deg": pitch_cmd,
                    "err_px": hover_err_last,
                    "feature": cur,
                    "ibvs": ctrl,
                    "stable_ticks": hover_stable_ticks,
                    "required_stable_ticks": hover_ticks_req,
                })
            if hover_err_last <= err_last or hover_err_last <= hover_err_min:
                hover_stable_ticks += 1
            sentai.rtos.sleep_ms(TICK_MS)
        _rpyt(0.0, 0.0, 0.0, thrust)
        hover_trend_ok = (hover_err_last <= err_last)
        if len(hover_err_win) >= MARKER_COUNT_AVG_WINDOW:
            half = MARKER_COUNT_AVG_WINDOW // 2
            old_avg = sum(hover_err_win[:half]) / float(half)
            new_avg = sum(hover_err_win[half:]) / float(len(hover_err_win) - half)
            hover_trend_ok = hover_trend_ok or new_avg <= old_avg
        if hover_reason == "" and not hover_trend_ok:
            hover_reason = "center_hover_not_converging"
        hover_ok = (hover_reason == "" and
                    hover_trend_ok and
                    marker_stats["avg_full"] > AXIS_MIN_AVG_FULL_MARKERS)

    ok = recenter_ok and hover_ok
    rec = {
        "ok": ok,
        "reason": "" if ok else (
            abort_reason or hover_reason or "not_centered_or_insufficient_improvement"),
        "target_px": (CX, CY),
        "initial_err_px": err_initial,
        "final_err_px": err_last,
        "min_err_px": err_min,
        "max_err_px": err_max,
        "improvement_px": improvement,
        "recenter_trend_ok": recenter_trend_ok,
        "center_hover_ok": hover_ok,
        "center_hover_s": FINAL_CENTER_HOVER_S,
        "center_hover_tol_px": FINAL_CENTER_HOVER_TOL_PX,
        "center_hover_max_s": FINAL_CENTER_HOVER_MAX_S,
        "center_hover_ticks": hover_ticks,
        "center_hover_stable_ticks": hover_stable_ticks,
        "center_hover_err_max_px": hover_err_max,
        "center_hover_err_min_px": hover_err_min,
        "center_hover_err_last_px": hover_err_last,
        "center_hover_trend_ok": hover_ok,
        "initial_feature": feature_initial,
        "best_feature": feature_best,
        "final_feature": feature_last,
        "n_full_min": n_full_min,
        "ticks": ticks_done,
        "first_cmd": first_cmd,
        "last_cmd": last_cmd,
        "roll_vec_px": roll_vec,
        "pitch_vec_px": pitch_vec,
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


def _stream_axis_envelope_target(label, target_cx, target_cy, roll_vec,
                                 pitch_vec, start_thrust, edge_mode,
                                 active_axis):
    duration_s = AXIS_ENVELOPE_LEG_S if edge_mode else AXIS_ENVELOPE_CENTER_S
    ticks = int(duration_s * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1
    z_prev = 0.0
    vz_filt = 0.0
    marker_win = _marker_window_new()
    marker_stats = _marker_window_stats(marker_win)
    first = None
    last = None
    min_full = 99
    min_fov_margin = 999.0
    err_min = 999.0
    err_last = 0.0
    stable_ticks = 0
    orth_recovery_ticks = 0
    orth_recovery_events = 0
    orth_err_win = []
    orth_err_best = 999.0
    active_target_used = 0
    recovery_target_used = 0
    trigger = "timeout"
    thrust = start_thrust
    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        if first is None:
            first = cur
        last = cur
        marker_stats = _marker_window_update(marker_win, cur["n_full"])
        if cur["n_full"] < min_full:
            min_full = cur["n_full"]
        fov_margin = cur.get("fov_margin_min_px", 0.0)
        if fov_margin < min_fov_margin:
            min_fov_margin = fov_margin

        active_on_target_side = True
        if edge_mode and active_axis == "x":
            active_on_target_side = (
                cur["cx"] >= CX if target_cx > CX else cur["cx"] <= CX)
        elif edge_mode and active_axis == "y":
            active_on_target_side = (
                cur["cy"] >= CY if target_cy > CY else cur["cy"] <= CY)
        if edge_mode and marker_stats["ready"] and active_on_target_side:
            if (fov_margin <= FULL_VIS_MARGIN_PX or
                    marker_stats["avg_full"] <= AXIS_ENVELOPE_EDGE_FULL_MARKERS):
                trigger = "visual_envelope_reached"
                break
        if (not edge_mode and marker_stats["ready"] and
                marker_stats["avg_full"] > AXIS_MIN_AVG_FULL_MARKERS):
            err_to_center = ((CX - cur["cx"]) * (CX - cur["cx"]) +
                             (CY - cur["cy"]) * (CY - cur["cy"])) ** 0.5
            if err_to_center <= AXIS_ENVELOPE_CENTER_TOL_PX:
                stable_ticks += 1
                if stable_ticks >= MARKER_COUNT_AVG_WINDOW:
                    trigger = "center_hover_stable"
                    break
            else:
                stable_ticks = 0

        orth_err = abs(CY - cur["cy"]) if active_axis == "x" else abs(CX - cur["cx"])
        if orth_err < orth_err_best:
            orth_err_best = orth_err
        orth_err_win.append(orth_err)
        if len(orth_err_win) > MARKER_COUNT_AVG_WINDOW:
            orth_err_win.pop(0)
        orth_worsening = False
        orth_improving = False
        if len(orth_err_win) >= MARKER_COUNT_AVG_WINDOW:
            half = MARKER_COUNT_AVG_WINDOW // 2
            old_avg = sum(orth_err_win[:half]) / float(half)
            new_avg = sum(orth_err_win[half:]) / float(len(orth_err_win) - half)
            orth_worsening = new_avg > old_avg
            orth_improving = new_avg < old_avg

        active_target_cx = target_cx
        active_target_cy = target_cy
        recovery_mode = False
        if edge_mode:
            if active_axis == "x":
                active_target_cy = CY
                if orth_recovery_ticks > 0 or orth_worsening:
                    active_target_cx = cur["cx"]
                    recovery_mode = True
            elif active_axis == "y":
                active_target_cx = CX
                if orth_recovery_ticks > 0 or orth_worsening:
                    active_target_cy = cur["cy"]
                    recovery_mode = True
            if orth_worsening and orth_recovery_ticks == 0:
                orth_recovery_events += 1
                orth_recovery_ticks = AXIS_ENVELOPE_ORTH_RECOVERY_MIN_TICKS
            elif orth_recovery_ticks > 0:
                orth_recovery_ticks -= 1
                if orth_improving and orth_recovery_ticks < (AXIS_ENVELOPE_ORTH_RECOVERY_MIN_TICKS // 2):
                    orth_recovery_ticks = 0

        ctrl_gain = AXIS_ENVELOPE_KP
        if recovery_mode:
            ctrl_gain = AXIS_ENVELOPE_KP * AXIS_ENVELOPE_ORTH_RECOVERY_KP_MULT

        ctrl = _ibvs_centroid_command_to_target(
            cur, roll_vec, pitch_vec, active_target_cx, active_target_cy,
            ctrl_gain, AXIS_ENVELOPE_MAX_DEG,
            FINAL_RECENTER_DEADBAND_PX)
        if edge_mode and (active_target_cx != target_cx or
                          active_target_cy != target_cy):
            recovery_target_used += 1
        else:
            active_target_used += 1
        err_last = ctrl["err_px"]
        if err_last < err_min:
            err_min = err_last
        thrust, z_prev, vz_filt = _z_thrust_from_feature(f, z_prev, vz_filt)
        _rpyt(ctrl["roll_deg"], ctrl["pitch_deg"], 0.0, thrust)
        if k % 5 == 0:
            _j("axis_envelope_tick", {
                "label": label,
                "k": k,
                "target": (target_cx, target_cy),
                "command_target": (active_target_cx, active_target_cy),
                "edge_mode": edge_mode,
                "active_axis": active_axis,
                "active_on_target_side": active_on_target_side,
                "orth_err_px": orth_err,
                "orth_err_best_px": 0.0 if orth_err_best == 999.0 else orth_err_best,
                "orth_recovery_ticks": orth_recovery_ticks,
                "recovery_mode": recovery_mode,
                "ctrl_gain": ctrl_gain,
                "orth_worsening": orth_worsening,
                "feature": cur,
                "fov_margin_px": fov_margin,
                "n_full_avg": marker_stats["avg_full"],
                "n_full_avg_ready": marker_stats["ready"],
                "err_px": err_last,
                "roll_deg": ctrl["roll_deg"],
                "pitch_deg": ctrl["pitch_deg"],
                "thrust": thrust,
                "ibvs": ctrl,
            })
        sentai.rtos.sleep_ms(TICK_MS)

    out = {
        "label": label,
        "ok": trigger in ("visual_envelope_reached", "center_hover_stable"),
        "trigger": trigger,
        "target": (target_cx, target_cy),
        "active_axis": active_axis,
        "target_side_required": edge_mode,
        "edge_mode": edge_mode,
        "ticks": k + 1,
        "first": first,
        "last": last,
        "min_full_markers": 0 if min_full == 99 else min_full,
        "avg_full_markers": marker_stats["avg_full"],
        "min_fov_margin_px": 0.0 if min_fov_margin == 999.0 else min_fov_margin,
        "err_min_px": 0.0 if err_min == 999.0 else err_min,
        "err_last_px": err_last,
        "stable_ticks": stable_ticks,
        "orth_err_best_px": 0.0 if orth_err_best == 999.0 else orth_err_best,
        "orth_err_last_px": orth_err,
        "orth_recovery_events": orth_recovery_events,
        "orth_recovery_gain_mult": AXIS_ENVELOPE_ORTH_RECOVERY_KP_MULT,
        "active_target_ticks": active_target_used,
        "recovery_target_ticks": recovery_target_used,
        "thrust_last": thrust,
    }
    _j("axis_envelope_leg", out)
    return out, thrust


def _phase_axis_envelope_survey(summary, start_thrust):
    _j("phase_axis_envelope_survey", {
        "leg_s": AXIS_ENVELOPE_LEG_S,
        "center_s": AXIS_ENVELOPE_CENTER_S,
        "max_deg": AXIS_ENVELOPE_MAX_DEG,
        "policy": "move_active_axis_to_visual_envelope_keep_other_axis_centered",
    })
    axis = summary.get("axis_response_smoke") or {}
    roll_vec, pitch_vec = _axis_response_vectors(axis)
    if roll_vec is None or pitch_vec is None:
        out = {
            "ok": False,
            "reason": "missing_axis_response",
            "thrust_last": start_thrust,
        }
        summary["axis_envelope_survey"] = out
        _j("phase_axis_envelope_survey", out)
        return False, start_thrust

    thrust = start_thrust
    legs = []
    sequence = (
        ("pre_center", CX, CY, False, "xy"),
        ("x_max", IMG_W, CY, True, "x"),
        ("x_center_after_max", CX, CY, False, "xy"),
        ("x_min", 0.0, CY, True, "x"),
        ("x_center", CX, CY, False, "xy"),
        ("y_max", CX, IMG_H, True, "y"),
        ("y_center_after_max", CX, CY, False, "xy"),
        ("y_min", CX, 0.0, True, "y"),
        ("y_center", CX, CY, False, "xy"),
    )
    ok = True
    reason = ""
    for label, tx, ty, edge_mode, active_axis in sequence:
        leg, thrust = _stream_axis_envelope_target(
            label, tx, ty, roll_vec, pitch_vec, thrust, edge_mode,
            active_axis)
        legs.append(leg)
        if not leg.get("ok"):
            ok = False
            reason = "leg_failed_" + label
            break

    out = {
        "ok": ok,
        "reason": reason,
        "legs": legs,
        "thrust_last": thrust,
        "fov_margin_policy":
            "min_marker_center_radius_distance_to_image_bounds",
        "orthogonal_axis_policy":
            "active_axis_moves_to_envelope_other_axis_targeted_to_image_center",
    }
    summary["axis_envelope_survey"] = out
    _j("phase_axis_envelope_survey", out)
    return ok, thrust


def _phase_center_hold_descend_disarm(summary, start_thrust):
    _j("phase_center_hold_descend_disarm", {
        "start_thrust": start_thrust,
        "duration_s": CENTER_HOLD_DESCENT_S,
        "disarm_full_markers": CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
    })
    axis = summary.get("axis_response_smoke") or {}
    roll_vec, pitch_vec = _axis_response_vectors(axis)
    roll_strength = _response_strength(roll_vec or (0.0, 0.0))
    pitch_strength = _response_strength(pitch_vec or (0.0, 0.0))
    if (roll_vec is None or pitch_vec is None or
            roll_strength < FINAL_RECENTER_RESPONSE_MIN_PX or
            pitch_strength < FINAL_RECENTER_RESPONSE_MIN_PX):
        out = {
            "ok": False,
            "reason": "missing_or_weak_axis_response",
            "disarm_trigger": "",
            "thrust_last": start_thrust,
        }
        summary["center_hold_descend"] = out
        _j("phase_center_hold_descend_disarm", out)
        _phase_manual_descend_disarm(summary, start_thrust)
        return False

    ticks = int(CENTER_HOLD_DESCENT_S * 1000 / TICK_MS)
    if ticks < 1:
        ticks = 1
    z_prev = 0.0
    vz_filt = 0.0
    noise, z_prev, vz_filt, thrust = _measure_centroid_noise(
        "center_hold_descend", z_prev, vz_filt)
    z_hold_prev = z_prev
    vz_hold_filt = vz_filt
    deadband_px = CENTER_HOLD_DESCENT_DEADBAND_FLOOR_PX
    if noise.get("ok", False):
        measured_deadband = (CENTER_HOLD_DESCENT_DEADBAND_SIGMA_MULT *
                             float(noise.get("sigma_px") or 0.0))
        if measured_deadband > deadband_px:
            deadband_px = measured_deadband
    _j("center_hold_descend_deadband", {
        "noise": noise,
        "deadband_px": deadband_px,
        "sigma_mult": CENTER_HOLD_DESCENT_DEADBAND_SIGMA_MULT,
        "floor_px": CENTER_HOLD_DESCENT_DEADBAND_FLOOR_PX,
    })
    _j("center_hold_descend_rate_control", {
        "target_vz_m_s": CENTER_HOLD_DESCENT_TARGET_VZ_M_S,
        "base_thrust": start_thrust,
        "kd_thrust_per_m_s": CENTER_HOLD_DESCENT_KD_THRUST_PER_M_S,
        "pause_policy":
            "pause_descent_when_lateral_control_saturates_and_error_worsens",
        "center_error_policy":
            "forensic_only_while_marker_lock_is_present",
        "pause_signal":
            "command_saturation_plus_10_frame_error_trend_no_pixel_threshold",
        "thrust_floor": CENTER_HOLD_DESCENT_THRUST_FLOOR_U16,
    })
    n_full_min = 99
    err_max = 0.0
    err_last = 0.0
    trigger = ""
    last_feature = None
    thrust = start_thrust
    ticks_done = 0
    lost_ticks = 0
    z_min = 999.0
    z_max = 0.0
    z_last = 0.0
    marker_win = _marker_window_new()
    marker_stats = _marker_window_stats(marker_win)
    err_win = []
    descent_pause_ticks = 0
    for k in range(ticks):
        f = _detect_features()
        cur = _feature_compact(f)
        marker_stats = _marker_window_update(marker_win, cur["n_full"])
        last_feature = cur
        if cur["n_full"] < n_full_min:
            n_full_min = cur["n_full"]
        if cur["z_cam_mean_m"] > 0.0:
            z_last = cur["z_cam_mean_m"]
            if z_last < z_min:
                z_min = z_last
            if z_last > z_max:
                z_max = z_last
        if (marker_stats["ready"] and
                marker_stats["avg_full"] <= CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS):
            trigger = "full_marker_count_reached_disarm_threshold"
            _j("center_hold_descend_trigger", {
                "k": k,
                "feature": cur,
                "threshold": CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
                "n_full_avg": marker_stats["avg_full"],
                "avg_window": marker_stats["size"],
            })
            break
        if cur["n_full"] <= 0:
            lost_ticks += 1
            if lost_ticks >= CENTER_HOLD_DESCENT_LOST_MAX_TICKS:
                trigger = "marker_centroid_lost_before_threshold"
                _j("center_hold_descend_trigger", {
                    "k": k,
                    "feature": cur,
                    "threshold": CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
                    "lost_ticks": lost_ticks,
                    "n_full_avg": marker_stats["avg_full"],
                })
                break
        else:
            lost_ticks = 0

        ctrl = _ibvs_centroid_command(
            cur, roll_vec, pitch_vec,
            CENTER_HOLD_DESCENT_KP,
            CENTER_HOLD_DESCENT_MAX_DEG,
            deadband_px)
        err_last = ctrl["err_px"]
        if err_last > err_max:
            err_max = err_last
        err_win.append(err_last)
        if len(err_win) > MARKER_COUNT_AVG_WINDOW:
            err_win.pop(0)

        roll_cmd = ctrl["roll_deg"]
        pitch_cmd = ctrl["pitch_deg"]

        z_thrust, z_hold_prev, vz_hold_filt = _z_thrust_from_feature(
            f, z_hold_prev, vz_hold_filt)
        descend_thrust, z_prev, vz_filt = _descent_thrust_from_feature(
            f, z_prev, vz_filt, start_thrust)
        hold_thrust, z_prev, vz_filt = _vertical_rate_thrust_from_feature(
            f, z_prev, vz_filt, start_thrust, 0.0)

        roll_saturated = (
            abs(abs(roll_cmd) - CENTER_HOLD_DESCENT_MAX_DEG) < 0.0001)
        pitch_saturated = (
            abs(abs(pitch_cmd) - CENTER_HOLD_DESCENT_MAX_DEG) < 0.0001)
        error_trend = 0.0
        error_trend_ready = (len(err_win) >= MARKER_COUNT_AVG_WINDOW)
        if error_trend_ready:
            error_trend = err_win[-1] - err_win[0]
        lateral_authority_limited = (
            (roll_saturated or pitch_saturated) and
            error_trend_ready and
            error_trend > 0.0)
        descent_paused = (
            marker_stats["ready"] and
            marker_stats["avg_full"] > CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS and
            lateral_authority_limited)
        if descent_paused:
            thrust = hold_thrust
            descent_pause_ticks += 1
        else:
            thrust = descend_thrust
        _rpyt(roll_cmd, pitch_cmd, 0.0, thrust)
        ticks_done = k + 1
        if k % 5 == 0:
            _j("center_hold_descend_tick", {
                "k": k,
                "thrust": thrust,
                "descend_thrust": descend_thrust,
                "hold_thrust": hold_thrust,
                "z_hold_thrust": z_thrust,
                "descent_paused": descent_paused,
                "descent_pause_ticks": descent_pause_ticks,
                "lateral_authority_limited": lateral_authority_limited,
                "roll_saturated": roll_saturated,
                "pitch_saturated": pitch_saturated,
                "error_trend_px_per_window": error_trend,
                "error_trend_ready": error_trend_ready,
                "deadband_px": deadband_px,
                "n_full_avg": marker_stats["avg_full"],
                "n_full_avg_ready": marker_stats["ready"],
                "target_vz_m_s": CENTER_HOLD_DESCENT_TARGET_VZ_M_S,
                "vz_filt_m_s": vz_filt,
                "z_hold_vz_filt_m_s": vz_hold_filt,
                "roll_deg": roll_cmd,
                "pitch_deg": pitch_cmd,
                "err_px": err_last,
                "feature": cur,
                "ibvs": ctrl,
            })
        sentai.rtos.sleep_ms(TICK_MS)

    if trigger == "":
        trigger = "timeout"
    _stream_zero_thrust(0.2)
    try:
        sentai.crazy.disarm()
        summary["disarmed"] = True
    except Exception as e:
        summary["disarmed"] = False
        summary["disarm_error"] = str(e)
    ok = (trigger == "full_marker_count_reached_disarm_threshold" and
          summary.get("disarmed", False))
    out = {
        "ok": ok,
        "reason": "" if ok else trigger,
        "disarm_trigger": trigger,
        "threshold_full_markers": CENTER_HOLD_DESCENT_DISARM_FULL_MARKERS,
        "n_full_min": n_full_min,
        "avg_full_markers": marker_stats["avg_full"],
        "avg_window": marker_stats["size"],
        "err_max_px": err_max,
        "err_last_px": err_last,
        "descent_pause_ticks": descent_pause_ticks,
        "ticks": ticks_done,
        "feature_last": last_feature,
        "z_cam_min_m": 0.0 if z_min == 999.0 else z_min,
        "z_cam_max_m": z_max,
        "z_cam_last_m": z_last,
        "vz_filt_last_m_s": vz_filt,
        "target_vz_m_s": CENTER_HOLD_DESCENT_TARGET_VZ_M_S,
        "thrust_last": thrust,
        "disarmed": summary.get("disarmed", False),
    }
    summary["center_hold_descend"] = out
    _j("phase_center_hold_descend_disarm", out)
    return ok


def _mission_abort_detail(summary, abort_reason):
    if abort_reason == "marker_acquisition_timeout":
        return summary.get("marker_acquisition") or {}
    if abort_reason == "post_lock_brake_failed":
        return summary.get("post_lock_brake") or {}
    if abort_reason == "visual_z_hold_failed":
        return summary.get("visual_z_hold") or {}
    if abort_reason == "axis_response_failed":
        return summary.get("axis_response_smoke") or {}
    if abort_reason == "centroid_validation_failed":
        return summary.get("centroid_pd_validation") or {}
    if abort_reason == "candidate_scoring_failed":
        return summary.get("candidate_scoring") or {}
    if abort_reason == "optical_axis_validation_failed":
        return summary.get("optical_axis_validation") or {}
    if abort_reason == "final_candidate_validation_failed":
        return summary.get("final_candidate_validation") or {}
    if abort_reason == "return_to_center_failed":
        return summary.get("final_centroid_recenter") or {}
    if abort_reason == "axis_envelope_survey_failed":
        return summary.get("axis_envelope_survey") or {}
    return {}


def _set_abort_reason(summary, reason):
    summary["abort_reason"] = reason
    detail = _mission_abort_detail(summary, reason)
    summary["abort_detail"] = detail
    _j("abort_decision", {
        "reason": reason,
        "phase": summary.get("phase", ""),
        "detail": detail,
    })


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
        "experiment": "s199_calib_envelope_survey",
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
        _set_phase(summary, "setup", "mission_start")
        if not _phase_setup(summary):
            summary["status"] = "SETUP_FAIL"
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "preflight_features", "setup_ok")
        _phase_preflight_features(summary)

        _set_phase(summary, "arm_zero_unlock", "preflight_sampled")
        if not _phase_arm_zero_unlock(summary):
            summary["status"] = "ARM_FAIL"
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "thrust_only_marker_acquisition", "armed")
        locked, last_thrust = _phase_thrust_only_marker_acquisition(summary)

        zhold_ok = False
        axis_ok = False
        brake_ok = False
        centroid_validation_ok = False
        candidate_ok = False
        optical_axis_ok = False
        final_validation_ok = False
        final_recenter_ok = False
        axis_envelope_ok = False
        center_hold_descend_ok = False
        if locked:
            _set_phase(summary, "post_lock_brake", "marker_lock_acquired")
            brake_ok, last_thrust = _phase_post_lock_brake(summary, last_thrust)

        if locked and brake_ok:
            _set_phase(summary, "visual_z_hold", "post_lock_brake_ok")
            zhold_ok, last_thrust = _phase_visual_z_hold(summary, last_thrust)

        if locked and brake_ok and zhold_ok:
            _set_phase(summary, "axis_response_smoke", "visual_z_hold_ok")
            axis_attempts = []
            for attempt in range(1, AXIS_RESPONSE_MAX_ATTEMPTS + 1):
                axis_ok, last_thrust = _phase_axis_response_smoke(
                    summary, last_thrust)
                attempt_summary = summary.get("axis_response_smoke") or {}
                attempt_summary["attempt"] = attempt
                axis_attempts.append(attempt_summary)
                summary["axis_response_attempts"] = axis_attempts
                if axis_ok:
                    break
                if attempt < AXIS_RESPONSE_MAX_ATTEMPTS:
                    last_thrust = _phase_axis_response_retry_settle(
                        summary, last_thrust, attempt)

        if locked and brake_ok and zhold_ok and axis_ok:
            _set_phase(summary, "centroid_pd_validation", "axis_response_ok")
            centroid_validation_ok, last_thrust = _phase_centroid_pd_validation(
                summary, last_thrust)

        if locked and brake_ok and zhold_ok and axis_ok:
            _set_phase(summary, "candidate_scoring", "axis_response_ok")
            candidate_ok = _phase_candidate_scoring(summary)

        if locked and brake_ok and zhold_ok and axis_ok and candidate_ok:
            _set_phase(summary, "optical_axis_validation", "candidate_ok")
            optical_axis_ok = _phase_optical_axis_validation(summary)

        if (locked and brake_ok and zhold_ok and axis_ok and candidate_ok and
                optical_axis_ok):
            _set_phase(summary, "final_candidate_validation",
                       "optical_axis_ok")
            final_validation_ok, last_thrust = _phase_final_candidate_validation(
                summary, last_thrust)

        if (locked and brake_ok and zhold_ok and axis_ok and candidate_ok and
                optical_axis_ok and final_validation_ok):
            _set_phase(summary, "final_centroid_recenter",
                       "final_candidate_validation_ok")
            final_recenter_ok, last_thrust = _phase_final_centroid_recenter(
                summary, last_thrust)
            last_thrust = (summary.get("final_centroid_recenter") or {}).get(
                "thrust_last", last_thrust)
        elif locked and brake_ok and zhold_ok:
            reason = "skipped_axis_response_not_accepted"
            if axis_ok and not candidate_ok:
                reason = "skipped_candidate_not_accepted"
            elif axis_ok and candidate_ok and not optical_axis_ok:
                reason = "skipped_optical_axis_validation_not_accepted"
            elif axis_ok and candidate_ok and optical_axis_ok and not final_validation_ok:
                reason = "skipped_final_validation_not_accepted"
            summary["final_centroid_recenter"] = {
                "ok": False,
                "reason": reason,
                "passes": 0,
                "thrust_last": last_thrust,
            }

        if final_recenter_ok:
            _set_phase(summary, "axis_envelope_survey",
                       "final_recenter_ok")
            axis_envelope_ok, last_thrust = _phase_axis_envelope_survey(
                summary, last_thrust)

        _set_phase(summary, "manual_descend_disarm",
                   "select_recovery_or_landing")
        if not locked:
            _set_abort_reason(summary, "marker_acquisition_timeout")
        elif not brake_ok:
            _set_abort_reason(summary, "post_lock_brake_failed")
        elif not zhold_ok:
            _set_abort_reason(summary, "visual_z_hold_failed")
        elif not axis_ok:
            _set_abort_reason(summary, "axis_response_failed")
        elif not candidate_ok:
            _set_abort_reason(summary, "candidate_scoring_failed")
        elif not optical_axis_ok:
            _set_abort_reason(summary, "optical_axis_validation_failed")
        elif not final_validation_ok:
            _set_abort_reason(summary, "final_candidate_validation_failed")
        elif not final_recenter_ok:
            _set_abort_reason(summary, "return_to_center_failed")
        elif not axis_envelope_ok:
            _set_abort_reason(summary, "axis_envelope_survey_failed")
        else:
            summary["abort_reason"] = ""
            summary["abort_detail"] = {}
        if final_recenter_ok and axis_envelope_ok:
            _set_phase(summary, "center_hold_descend_disarm",
                       "axis_envelope_survey_ok")
            center_hold_descend_ok = _phase_center_hold_descend_disarm(
                summary, last_thrust)
        else:
            _phase_manual_descend_disarm(summary, last_thrust)

        _set_phase(summary, "post_acquisition", "flight_sequence_done")
        _flight_plan_placeholders(summary)
        if not locked:
            summary["status"] = "MARKER_ACQ_TIMEOUT"
        elif not brake_ok:
            summary["status"] = "POST_LOCK_BRAKE_FAIL"
        elif not zhold_ok:
            summary["status"] = "VISUAL_Z_HOLD_FAIL"
        elif not axis_ok:
            summary["status"] = "AXIS_RESPONSE_FAIL"
        elif not candidate_ok:
            summary["status"] = "CANDIDATE_SCORING_FAIL"
        elif not optical_axis_ok:
            summary["status"] = "OPTICAL_AXIS_VALIDATION_FAIL"
        elif not final_validation_ok:
            summary["status"] = "FINAL_VALIDATION_FAIL"
        elif not final_recenter_ok:
            summary["status"] = "RETURN_TO_CENTER_FAIL"
        elif not axis_envelope_ok:
            summary["status"] = "AXIS_ENVELOPE_SURVEY_FAIL"
        elif not center_hold_descend_ok:
            summary["status"] = "CENTER_HOLD_DESCENT_FAIL"
        else:
            summary["status"] = "FINAL_VALIDATION_OK"
        if final_validation_ok:
            _write_calib_artifact(summary)
            _write_calib_ini(summary)
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = str(e)
        summary["abort_reason"] = "exception"
        summary["abort_detail"] = {
            "phase": summary.get("phase", ""),
            "exception": str(e),
        }
        _j("mission_exception", {"err": str(e)})
        _j("abort_decision", {
            "reason": "exception",
            "phase": summary.get("phase", ""),
            "detail": summary["abort_detail"],
        })
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
