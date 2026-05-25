# mission_s201 - TD-S10-B4 minimal in-flight commander handoff smoke.
#
# Scope: consume accepted A3 calib.ini, take off with conservative RPYT,
# center/hold on WhyCon, warm up ExtPos, release RPYT without disarming,
# hold with generic hover for a few seconds, then land.

import sentai
import struct


JOURNAL_NAME = "mission_s201_journal.txt"
SUMMARY_NAME = "mission_s201_summary.json"

IMG_W = 320
IMG_H = 240
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0
MARKER_DIAMETER_M = 0.0544
EXPECTED_LAYOUT = "sentai_whycon_small_centroid_7_marker_v1"
EXPECTED_R = (0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0)

MARKER_WORLD = (
    (-0.08285714, +0.06571429, 0.005),
    (+0.07714286, +0.06571429, 0.005),
    (-0.06285715, -0.01428571, 0.005),
    (+0.05714286, -0.01428571, 0.005),
    (-0.08285714, -0.09428572, 0.005),
    (+0.07714286, -0.09428572, 0.005),
    (+0.01714286, +0.08571428, 0.005),
)

FULL_VIS_MARGIN_PX = 2.0
MIN_FULL_MARKERS = 4
LOCK_FULL_MARKERS = 7
LOCK_AVG_WINDOW = 10
TICK_MS = 33
TAKEOFF_Z_TARGET_SCALE = 1.5

ZERO_UNLOCK_S = 1.5
RAMP_MAX_S = 12.0
T_BASE_U16 = 30000
T_MAX_U16 = 34500
T_HOVER_VISUAL_U16 = 32700
T_HOLD_MIN_U16 = 30000
T_HOLD_MAX_U16 = 34500
KP_THRUST_PER_M = 6500.0
KD_THRUST_PER_M_S = 5200.0
VZ_LPF_ALPHA = 0.25

CENTER_HOLD_S = 2.0
CENTER_HOLD_GAIN = 0.08
CENTER_HOLD_MAX_DEG = 0.45
IBVS_DAMPING_PX_PER_DEG = 1.5
IBVS_SUSTAINED_RESPONSE_SIGN = -1.0
IBVS_Z_REF_M = 0.64
IBVS_Z_GAIN_MIN = 0.65
IBVS_Z_GAIN_MAX = 1.35
EXTPOS_PNP_Z_RATIO_MIN = 0.50
EXTPOS_PNP_Z_RATIO_MAX = 1.50
EXTPOS_SIGN_X = -1.0
EXTPOS_SIGN_Y = -1.0
EXTPOS_SIGN_Z = 1.0
EXTPOS_WARMUP_MIN_S = 2.5
EXTPOS_WARMUP_MAX_S = 12.0
EXTPOS_CONVERGED_ERR_FRACTION = 0.20
EXTPOS_DIVERGED_ERR_FRACTION = 1.00
HOVER_HOLD_S = 4.0
HL_ENGAGE_DUR_S = 0.8
HL_HOLD_MIN_DUR_S = 1.5
HL_HOLD_MAX_DUR_S = 3.5
HL_LATERAL_SPEED_LIMIT_M_S = 0.03
HL_AXIS_STEP_FOV_FRACTION = 0.20
HL_AXIS_STEP_MIN_M = 0.04
HL_AXIS_STEP_MAX_M = 0.08
HL_SEGMENT_SETTLE_S = 0.5
HL_AXIS_LINEAR = 1
HL_LAND_DUR_S = 3.0

CRTP_PORT_PARAM = 0x02
PARAM_TOC_CH = 0
PARAM_WRITE_CH = 2
PARAM_TOC_GET_ITEM_V2 = 2
PARAM_TOC_GET_INFO_V2 = 3


def _ser_val(v):
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
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


def _set_phase(summary, phase, reason=""):
    prev = summary.get("phase", "")
    summary["phase"] = phase
    _j("phase_transition", {"from": prev, "to": phase, "reason": reason})


def _clip(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16):
    sentai.crazy.send_crtp(
        3, 0, struct.pack("<fffH",
                          roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16))


def _crtp_drain(limit=32):
    recv = getattr(sentai.crazy, "recv_crtp", None)
    if recv is None:
        return 0
    n = 0
    for _ in range(limit):
        pkt = recv()
        if pkt is None:
            break
        n += 1
    return n


def _crtp_wait(port, ch, first_byte=None, timeout_ms=600):
    recv = getattr(sentai.crazy, "recv_crtp", None)
    if recv is None:
        return None
    ticks = max(1, int(timeout_ms / 10))
    for _ in range(ticks):
        pkt = recv()
        if pkt is not None:
            p, c, data = pkt
            if int(p) == port and int(c) == ch:
                if first_byte is None or (len(data) > 0 and data[0] == first_byte):
                    return data
        sentai.rtos.sleep_ms(10)
    return None


def _param_send_wait(ch, payload, first_byte, timeout_ms=800):
    _crtp_drain()
    rc = sentai.crazy.send_crtp(CRTP_PORT_PARAM, ch, payload)
    if rc != 0:
        return None, rc
    return _crtp_wait(CRTP_PORT_PARAM, ch, first_byte, timeout_ms), 0


def _param_toc_count():
    data, rc = _param_send_wait(
        PARAM_TOC_CH, bytes([PARAM_TOC_GET_INFO_V2]), PARAM_TOC_GET_INFO_V2, 1000)
    if data is None or len(data) < 3:
        return None, rc
    return int(data[1]) | (int(data[2]) << 8), 0


def _ascii_from_bytes(data, start, end):
    out = ""
    for i in range(start, end):
        b = int(data[i])
        if b == 0:
            break
        out += chr(b)
    return out


def _param_find(group_want, name_want):
    count, rc = _param_toc_count()
    if count is None:
        return None, "toc_info_fail_%d" % rc
    for pid in range(count):
        req = bytes([PARAM_TOC_GET_ITEM_V2, pid & 0xFF, (pid >> 8) & 0xFF])
        data, rc = _param_send_wait(PARAM_TOC_CH, req, PARAM_TOC_GET_ITEM_V2, 500)
        if data is None or len(data) < 6:
            continue
        nul1 = -1
        nul2 = -1
        for i in range(4, len(data)):
            if data[i] == 0:
                if nul1 < 0:
                    nul1 = i
                else:
                    nul2 = i
                    break
        if nul1 < 0:
            continue
        group = _ascii_from_bytes(data, 4, nul1)
        end = nul2 if nul2 >= 0 else len(data)
        name = _ascii_from_bytes(data, nul1 + 1, end)
        if group == group_want and name == name_want:
            return pid, "ok"
    return None, "not_found"


def _param_write_u8(pid, value):
    payload = bytes([pid & 0xFF, (pid >> 8) & 0xFF, int(value) & 0xFF])
    data, rc = _param_send_wait(PARAM_WRITE_CH, payload, None, 700)
    return rc if data is not None or rc != 0 else -2


def _kalman_reset_before_extpos():
    out = {
        "supported": hasattr(sentai.crazy, "recv_crtp"),
        "stabilizer_estimator_id": None,
        "kalman_reset_id": None,
        "set_estimator_rc": None,
        "reset_1_rc": None,
        "reset_0_rc": None,
        "ok": False,
    }
    if not out["supported"]:
        out["reason"] = "recv_crtp_unavailable"
        return out
    est_id, est_reason = _param_find("stabilizer", "estimator")
    reset_id, reset_reason = _param_find("kalman", "resetEstimation")
    out["stabilizer_estimator_id"] = est_id
    out["stabilizer_estimator_reason"] = est_reason
    out["kalman_reset_id"] = reset_id
    out["kalman_reset_reason"] = reset_reason
    if est_id is not None:
        out["set_estimator_rc"] = _param_write_u8(est_id, 2)
    if reset_id is not None:
        out["reset_1_rc"] = _param_write_u8(reset_id, 1)
        sentai.rtos.sleep_ms(100)
        out["reset_0_rc"] = _param_write_u8(reset_id, 0)
    out["ok"] = (
        (est_id is None or out["set_estimator_rc"] == 0) and
        reset_id is not None and
        out["reset_1_rc"] == 0 and
        out["reset_0_rc"] == 0)
    return out


def _release_rpyt_without_disarm(remain_valid_ms=100):
    fn = getattr(sentai.crazy, "attitude_release_no_disarm", None)
    if fn is not None:
        return fn()
    # SIM s201 streams RPYT manually via raw CRTP, so there is no local
    # background attitude task to stop.  The firmware-side release primitive is
    # Generic Commander notifySetpointsStop: type=0, remainValidMillisecs.
    return sentai.crazy.send_crtp(7, 1, struct.pack("<BI", 0, remain_valid_ms))


def _marker_world_bytes():
    buf = b""
    for x, y, z in MARKER_WORLD:
        buf += struct.pack("<fff", x, y, z)
    return buf


def _read_fs_text(path):
    try:
        return sentai.fs.read_str(path)
    except Exception:
        return ""


def _ini_map(txt):
    out = {}
    for raw in txt.split("\n"):
        line = raw.strip()
        if not line or line[0] in "#;[":
            continue
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _csv_floats(v):
    out = []
    for part in v.split(","):
        if part.strip():
            out.append(float(part.strip()))
    return out


def _phase_setup(summary):
    global EXTPOS_SIGN_X, EXTPOS_SIGN_Y, EXTPOS_SIGN_Z
    _j("phase_setup", "start")
    sentai.calib.init()

    ini_txt = _read_fs_text("/system/calib.ini")
    ini = _ini_map(ini_txt)
    summary["calib_ini_present"] = bool(ini_txt)
    summary["calib_ini_bytes"] = len(ini_txt)
    summary["calib_ini_layout_id"] = ini.get("layout_id", "")
    summary["calib_ini_source_status"] = ini.get("source_status", "")

    load_ok = False
    try:
        load_ok = bool(sentai.calib.load())
    except Exception as e:
        summary["calib_load_error"] = str(e)[:120]
    summary["calib_load_ok"] = load_ok

    r_loaded = ()
    try:
        r_loaded = sentai.calib.get_R_cam_to_body()
    except Exception as e:
        summary["calib_get_R_error"] = str(e)[:120]
    summary["R_loaded"] = r_loaded
    cam_offset = ()
    try:
        cam_offset = sentai.calib.get_cam_offset_B()
    except Exception as e:
        summary["calib_get_cam_offset_error"] = str(e)[:120]
    summary["cam_offset_B_loaded"] = cam_offset
    r_ok = load_ok and len(r_loaded) == 9
    if r_ok:
        max_abs = 0.0
        for a, b in zip(r_loaded, EXPECTED_R):
            d = abs(float(a) - float(b))
            if d > max_abs:
                max_abs = d
        summary["R_expected_max_abs_diff"] = max_abs
        r_ok = max_abs < 0.001
    summary["R_loaded_matches_seed"] = r_ok

    layout_ok = ini.get("layout_id", "") == EXPECTED_LAYOUT
    summary["calib_layout_ok"] = layout_ok

    sentai.camera.init()
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)
    marker_extrinsics_ok = False
    if len(r_loaded) == 9 and len(cam_offset) == 3:
        try:
            sentai.markers.set_cam_extrinsics_matrix(
                float(cam_offset[0]), float(cam_offset[1]), float(cam_offset[2]),
                r_loaded)
            marker_extrinsics_ok = True
        except Exception as e:
            summary["marker_extrinsics_error"] = str(e)[:120]
    summary["marker_extrinsics_from_calib_ini"] = marker_extrinsics_ok
    rc_mw = sentai.markers.set_marker_world(_marker_world_bytes())
    summary["set_marker_world_rc"] = rc_mw

    sentai.safety.init()
    sentai.crazy.init()
    pose_attempts = []
    pose_rc = -999
    for attempt in range(3):
        try:
            pose_rc = sentai.crazy.pose_subscribe(33)
        except Exception as e:
            pose_rc = -998
            pose_attempts.append({"attempt": attempt, "rc": pose_rc,
                                  "error": str(e)[:120]})
            sentai.rtos.sleep_ms(500)
            continue
        pose_attempts.append({"attempt": attempt, "rc": pose_rc})
        if pose_rc == 0:
            break
        sentai.rtos.sleep_ms(500)
    summary["pose_subscribe_rc"] = pose_rc
    summary["pose_subscribe_attempts"] = pose_attempts

    axis = {
        "roll_sign": -1.0,
        "roll_strength_px": 4.0,
        "pitch_sign": -1.0,
        "pitch_strength_px": 4.0,
    }
    try:
        EXTPOS_SIGN_X = float(ini.get("extpos_sign_x", str(EXTPOS_SIGN_X)))
        EXTPOS_SIGN_Y = float(ini.get("extpos_sign_y", str(EXTPOS_SIGN_Y)))
        EXTPOS_SIGN_Z = float(ini.get("extpos_sign_z", str(EXTPOS_SIGN_Z)))
        axis["roll_sign"] = float(ini.get("axis_roll_sign", "-1"))
        axis["pitch_sign"] = float(ini.get("axis_pitch_sign", "-1"))
        axis["roll_strength_px"] = float(ini.get("axis_roll_strength_px", "4"))
        axis["pitch_strength_px"] = float(ini.get("axis_pitch_strength_px", "4"))
        rv = _csv_floats(ini.get("axis_roll_vec_px", ""))
        pv = _csv_floats(ini.get("axis_pitch_vec_px", ""))
        if len(rv) == 2:
            axis["roll_vec_px"] = (rv[0], rv[1])
        if len(pv) == 2:
            axis["pitch_vec_px"] = (pv[0], pv[1])
    except Exception:
        pass
    summary["axis_seed"] = axis
    summary["extpos_signs"] = (EXTPOS_SIGN_X, EXTPOS_SIGN_Y, EXTPOS_SIGN_Z)

    ok = load_ok and r_ok and layout_ok and marker_extrinsics_ok and rc_mw == 0
    _j("phase_setup", {"ok": ok, "load_ok": load_ok, "layout_ok": layout_ok})
    return ok, axis


def _is_full_visible(det):
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
    full = []
    cx_sum = 0.0
    cy_sum = 0.0
    radius_sum = 0.0
    if n < 0:
        n = 0
    for i in range(n):
        d = sentai.markers.get_detection_tuple(i)
        if d is None:
            continue
        if _is_full_visible(d):
            cx = float(d[1])
            cy = float(d[2])
            r = float(d[17])
            full.append((cx, cy, r))
            cx_sum += cx
            cy_sum += cy
            radius_sum += r
    nf = len(full)
    radius_mean = (radius_sum / nf) if nf else 0.0
    # Visual altitude proxy from apparent marker diameter.  This must stay in
    # camera optical geometry, independent of whether marker tvecs are later
    # transformed into body frame by calibrated extrinsics.
    z_visual = (FX * MARKER_DIAMETER_M / (2.0 * radius_mean)) if radius_mean > 0.0 else 0.0
    return {
        "n_raw": n,
        "n_full": nf,
        "centroid_px": ((cx_sum / nf), (cy_sum / nf)) if nf else None,
        "radius_mean_px": radius_mean,
        "z_cam_mean_m": z_visual,
    }


def _avg_push(buf, v):
    buf.append(float(v))
    if len(buf) > LOCK_AVG_WINDOW:
        buf.pop(0)
    return sum(buf) / len(buf)


def _feature_compact(f):
    c = f["centroid_px"]
    return {
        "n_full": f["n_full"],
        "cx": c[0] if c else None,
        "cy": c[1] if c else None,
        "z_cam_mean_m": f["z_cam_mean_m"],
        "radius_mean_px": f["radius_mean_px"],
    }


def _pose_error_m(pnp_pose, estimator_pose):
    if pnp_pose is None or estimator_pose is None:
        return None
    dx = float(estimator_pose[0]) - float(pnp_pose[0])
    dy = float(estimator_pose[1]) - float(pnp_pose[1])
    dz = float(estimator_pose[2]) - float(pnp_pose[2])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _pose_to_extpos(pose):
    if pose is None:
        return None
    return (
        EXTPOS_SIGN_X * float(pose[0]),
        EXTPOS_SIGN_Y * float(pose[1]),
        EXTPOS_SIGN_Z * float(pose[2]),
    )


def _cf_pose_yaw():
    try:
        p = sentai.crazy.pose()
        if p is not None and len(p) >= 4:
            return p, float(p[3])
    except Exception:
        pass
    return None, 0.0


def _hl_hold_duration_from_pose(est_pose):
    if est_pose is None:
        return HL_HOLD_MAX_DUR_S
    dx = float(est_pose[0])
    dy = float(est_pose[1])
    dist = (dx * dx + dy * dy) ** 0.5
    dur = dist / HL_LATERAL_SPEED_LIMIT_M_S if HL_LATERAL_SPEED_LIMIT_M_S > 0.0 else HL_HOLD_MAX_DUR_S
    return _clip(dur, HL_HOLD_MIN_DUR_S, HL_HOLD_MAX_DUR_S)


def _layout_half_extents_m():
    max_x = 0.0
    max_y = 0.0
    for x, y, _ in MARKER_WORLD:
        ax = abs(float(x))
        ay = abs(float(y))
        if ax > max_x:
            max_x = ax
        if ay > max_y:
            max_y = ay
    return max_x, max_y


def _hl_axis_step_from_z(z_m):
    if z_m is None or z_m <= 0.0:
        return HL_AXIS_STEP_MIN_M, {
            "source": "fallback_min",
            "step_m": HL_AXIS_STEP_MIN_M,
        }
    layout_x, layout_y = _layout_half_extents_m()
    half_fov_x = float(z_m) * (CX / FX)
    half_fov_y = float(z_m) * (CY / FY)
    margin_x = half_fov_x - layout_x
    margin_y = half_fov_y - layout_y
    margin = margin_x if margin_x < margin_y else margin_y
    if margin <= 0.0:
        step = HL_AXIS_STEP_MIN_M
    else:
        step = HL_AXIS_STEP_FOV_FRACTION * margin
    step = _clip(step, HL_AXIS_STEP_MIN_M, HL_AXIS_STEP_MAX_M)
    return step, {
        "source": "fov_layout_margin",
        "z_m": z_m,
        "layout_half_x_m": layout_x,
        "layout_half_y_m": layout_y,
        "half_fov_x_m": half_fov_x,
        "half_fov_y_m": half_fov_y,
        "margin_x_m": margin_x,
        "margin_y_m": margin_y,
        "fraction": HL_AXIS_STEP_FOV_FRACTION,
        "step_m": step,
    }


def _hl_duration_for_distance(distance_m):
    if HL_LATERAL_SPEED_LIMIT_M_S <= 0.0:
        return HL_HOLD_MAX_DUR_S
    return _clip(float(distance_m) / HL_LATERAL_SPEED_LIMIT_M_S,
                 HL_HOLD_MIN_DUR_S,
                 HL_HOLD_MAX_DUR_S)


def _send_extpos_from_current_markers():
    f = _detect_features()
    pose = None
    extpos = None
    pose_ok = False
    pose_reason = "no_pose"
    cf_pose, cf_yaw = _cf_pose_yaw()
    try:
        pose = sentai.markers.get_drone_pose_tuple(cf_yaw)
        pose_ok, pose_reason = _pose_plausible_for_extpos(pose, f)
        if pose_ok:
            extpos = _pose_to_extpos(pose)
            rc = sentai.crazy.send_extpos(float(extpos[0]),
                                          float(extpos[1]),
                                          float(extpos[2]))
            send_ok = (rc == 0)
        else:
            send_ok = False
    except Exception:
        pose_reason = "exception"
        send_ok = False
    est = None
    if cf_pose is not None and len(cf_pose) >= 3:
        est = (float(cf_pose[0]), float(cf_pose[1]), float(cf_pose[2]))
    return {
        "feature": f,
        "pose": pose,
        "extpos": extpos,
        "pose_ok": pose_ok,
        "pose_reason": pose_reason,
        "pose_diag": _pose_diag(pose),
        "cf_yaw_rad": cf_yaw,
        "send_ok": send_ok,
        "est_pose": est,
    }


def _pose_diag(pose):
    if pose is None or not isinstance(pose, (tuple, list)) or len(pose) < 9:
        return None
    return {
        "raw_xyz": (float(pose[0]), float(pose[1]), float(pose[2])),
        "yaw_rad": float(pose[3]),
        "res_max_m": float(pose[4]),
        "n_used": int(pose[5]),
        "flip_x": int(pose[6]),
        "flip_y": int(pose[7]),
        "flip_z": int(pose[8]),
    }


def _dist3(a, b):
    if a is None or b is None:
        return None
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _attitude_sample():
    try:
        att = sentai.crazy.attitude_get(40)
        if att is not None and len(att) >= 3:
            return (float(att[0]), float(att[1]), float(att[2]))
    except Exception:
        pass
    return None


def _att_stats_new():
    return {
        "samples": 0,
        "roll_abs_max_deg": 0.0,
        "pitch_abs_max_deg": 0.0,
        "tilt_abs_max_deg": 0.0,
        "roll_sum_sq": 0.0,
        "pitch_sum_sq": 0.0,
        "last": None,
    }


def _att_stats_push(stats, att):
    if att is None:
        return
    roll = float(att[0])
    pitch = float(att[1])
    tilt = (roll * roll + pitch * pitch) ** 0.5
    stats["samples"] += 1
    if abs(roll) > stats["roll_abs_max_deg"]:
        stats["roll_abs_max_deg"] = abs(roll)
    if abs(pitch) > stats["pitch_abs_max_deg"]:
        stats["pitch_abs_max_deg"] = abs(pitch)
    if tilt > stats["tilt_abs_max_deg"]:
        stats["tilt_abs_max_deg"] = tilt
    stats["roll_sum_sq"] += roll * roll
    stats["pitch_sum_sq"] += pitch * pitch
    stats["last"] = att


def _att_stats_finish(stats):
    out = {
        "samples": stats["samples"],
        "roll_abs_max_deg": stats["roll_abs_max_deg"],
        "pitch_abs_max_deg": stats["pitch_abs_max_deg"],
        "tilt_abs_max_deg": stats["tilt_abs_max_deg"],
        "last": stats["last"],
    }
    if stats["samples"] > 0:
        out["roll_rms_deg"] = (stats["roll_sum_sq"] / stats["samples"]) ** 0.5
        out["pitch_rms_deg"] = (stats["pitch_sum_sq"] / stats["samples"]) ** 0.5
    else:
        out["roll_rms_deg"] = None
        out["pitch_rms_deg"] = None
    return out


def _pose_plausible_for_extpos(pose, f):
    if pose is None:
        return False, "no_pose"
    if f["n_full"] < MIN_FULL_MARKERS:
        return False, "weak_markers"
    z_visual = float(f["z_cam_mean_m"])
    z_pose = float(pose[2])
    if z_visual <= 0.0 or z_pose <= 0.0:
        return False, "bad_z"
    ratio = z_pose / z_visual
    if ratio < EXTPOS_PNP_Z_RATIO_MIN or ratio > EXTPOS_PNP_Z_RATIO_MAX:
        return False, "z_visual_mismatch"
    return True, "ok"


def _z_thrust_from_feature(f, z_target, z_prev, vz_filt, thrust_base):
    z = f["z_cam_mean_m"]
    if z <= 0.0:
        return thrust_base, z_prev, vz_filt
    vz = 0.0
    if z_prev > 0.0:
        vz = (z - z_prev) * (1000.0 / TICK_MS)
    vz_filt = VZ_LPF_ALPHA * vz + (1.0 - VZ_LPF_ALPHA) * vz_filt
    err = z_target - z
    thrust = int(thrust_base + KP_THRUST_PER_M * err - KD_THRUST_PER_M_S * vz_filt)
    thrust = _clip(thrust, T_HOLD_MIN_U16, T_HOLD_MAX_U16)
    return thrust, z, vz_filt


def _center_cmd(f, axis):
    c = f["centroid_px"]
    if c is None or f["n_full"] < MIN_FULL_MARKERS:
        return 0.0, 0.0
    err_x = CX - c[0]
    err_y = CY - c[1]
    roll_vec = axis.get("roll_vec_px")
    pitch_vec = axis.get("pitch_vec_px")
    if roll_vec is not None and pitch_vec is not None:
        z = f.get("z_cam_mean_m", 0.0)
        z_gain = 1.0
        if z > 0.0:
            z_gain = _clip(z / IBVS_Z_REF_M, IBVS_Z_GAIN_MIN, IBVS_Z_GAIN_MAX)

        # Same convention as the accepted A3/s197 recentering path.  The
        # short pulse response has opposite settled-position sign in this sim.
        rx = IBVS_SUSTAINED_RESPONSE_SIGN * float(roll_vec[0])
        ry = IBVS_SUSTAINED_RESPONSE_SIGN * float(roll_vec[1])
        px = IBVS_SUSTAINED_RESPONSE_SIGN * float(pitch_vec[0])
        py = IBVS_SUSTAINED_RESPONSE_SIGN * float(pitch_vec[1])
        target_x = CENTER_HOLD_GAIN * z_gain * err_x
        target_y = CENTER_HOLD_GAIN * z_gain * err_y
        lam2 = IBVS_DAMPING_PX_PER_DEG * IBVS_DAMPING_PX_PER_DEG
        a11 = rx * rx + px * px + lam2
        a12 = rx * ry + px * py
        a22 = ry * ry + py * py + lam2
        det = a11 * a22 - a12 * a12
        if abs(det) > 0.0001:
            y0 = (a22 * target_x - a12 * target_y) / det
            y1 = (-a12 * target_x + a11 * target_y) / det
            roll = rx * y0 + ry * y1
            pitch = px * y0 + py * y1
            return (_clip(roll, -CENTER_HOLD_MAX_DEG, CENTER_HOLD_MAX_DEG),
                    _clip(pitch, -CENTER_HOLD_MAX_DEG, CENTER_HOLD_MAX_DEG))

    # Fallback if an older calib.ini lacks response vectors.
    roll_strength = axis.get("roll_strength_px", 4.0)
    pitch_strength = axis.get("pitch_strength_px", 4.0)
    roll_sign = axis.get("roll_sign", -1.0)
    pitch_sign = axis.get("pitch_sign", -1.0)
    if abs(roll_strength) < 0.001:
        roll_strength = 4.0
    if abs(pitch_strength) < 0.001:
        pitch_strength = 4.0
    roll = CENTER_HOLD_GAIN * err_x / (roll_sign * roll_strength)
    pitch = CENTER_HOLD_GAIN * err_y / (pitch_sign * pitch_strength)
    return (_clip(roll, -CENTER_HOLD_MAX_DEG, CENTER_HOLD_MAX_DEG),
            _clip(pitch, -CENTER_HOLD_MAX_DEG, CENTER_HOLD_MAX_DEG))


def _stream_zero_thrust(duration_s):
    ticks = int(duration_s * 1000 / TICK_MS)
    for _ in range(ticks):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)


def _phase_acquire(summary):
    _j("phase_acquire", "start")
    _stream_zero_thrust(ZERO_UNLOCK_S)
    n_avg_buf = []
    lock_avg_ticks = 0
    best = {"n_full_avg": 0.0, "feature": None}
    z_peak = 0.0
    ticks = int(RAMP_MAX_S * 1000 / TICK_MS)
    for k in range(ticks):
        alpha = float(k) / float(max(1, ticks - 1))
        thrust = int(T_BASE_U16 + alpha * (T_MAX_U16 - T_BASE_U16))
        _rpyt(0.0, 0.0, 0.0, thrust)
        f = _detect_features()
        n_avg = _avg_push(n_avg_buf, f["n_full"])
        if n_avg > best["n_full_avg"]:
            best = {"n_full_avg": n_avg, "feature": _feature_compact(f)}
        if f["z_cam_mean_m"] > z_peak:
            z_peak = f["z_cam_mean_m"]
        if n_avg > (LOCK_FULL_MARKERS - 1):
            lock_avg_ticks += 1
        else:
            lock_avg_ticks = 0
        if k % 10 == 0:
            _j("acquire_tick", {
                "k": k, "thrust": thrust, "n_avg": n_avg,
                "feature": _feature_compact(f),
            })
        if lock_avg_ticks >= LOCK_AVG_WINDOW:
            z_lock = z_peak
            z_target = z_lock * TAKEOFF_Z_TARGET_SCALE
            out = {
                "ok": True,
                "k": k,
                "thrust_last": thrust,
                "z_lock_m": z_lock,
                "z_target_scale": TAKEOFF_Z_TARGET_SCALE,
                "z_target_m": z_target,
                "best": best,
            }
            summary["acquire"] = out
            _j("phase_acquire", out)
            return True, thrust, z_target
        sentai.rtos.sleep_ms(TICK_MS)
    out = {"ok": False, "reason": "marker_lock_timeout", "best": best}
    summary["acquire"] = out
    _j("phase_acquire", out)
    return False, T_BASE_U16, z_peak


def _phase_center_hold(summary, axis, start_thrust, z_target):
    _j("phase_center_hold", {"z_target_m": z_target})
    z_prev = 0.0
    vz_filt = 0.0
    thrust = start_thrust
    n_min = 99
    err_last = 999.0
    att_stats = _att_stats_new()
    ticks = int(CENTER_HOLD_S * 1000 / TICK_MS)
    for k in range(ticks):
        f = _detect_features()
        _att_stats_push(att_stats, _attitude_sample())
        if f["n_full"] < n_min:
            n_min = f["n_full"]
        thrust, z_prev, vz_filt = _z_thrust_from_feature(
            f, z_target, z_prev, vz_filt, T_HOVER_VISUAL_U16)
        roll, pitch = _center_cmd(f, axis)
        c = f["centroid_px"]
        if c is not None:
            dx = CX - c[0]
            dy = CY - c[1]
            err_last = (dx * dx + dy * dy) ** 0.5
        _rpyt(roll, pitch, 0.0, thrust)
        if k % 10 == 0:
            _j("center_hold_tick", {
                "k": k, "roll": roll, "pitch": pitch, "thrust": thrust,
                "err_px": err_last, "feature": _feature_compact(f),
            })
        sentai.rtos.sleep_ms(TICK_MS)
    ok = n_min >= MIN_FULL_MARKERS
    out = {
        "ok": ok,
        "n_full_min": n_min,
        "err_last_px": err_last,
        "thrust_last": thrust,
        "z_target_m": z_target,
        "attitude_stats": _att_stats_finish(att_stats),
    }
    summary["center_hold"] = out
    _j("phase_center_hold", out)
    return ok, thrust


def _phase_extpos_warmup(summary, axis, start_thrust, z_target):
    _j("phase_extpos_warmup", {
        "min_s": EXTPOS_WARMUP_MIN_S,
        "max_s": EXTPOS_WARMUP_MAX_S,
    })
    kalman_reset = _kalman_reset_before_extpos()
    summary["kalman_reset_before_extpos"] = kalman_reset
    _j("kalman_reset_before_extpos", kalman_reset)
    z_prev = 0.0
    vz_filt = 0.0
    thrust = start_thrust
    n_min = 99
    n_avg_buf = []
    last_pose = None
    last_extpos = None
    last_est_pose = None
    send_ok = 0
    pose_rejects = 0
    reject_reasons = {}
    last_pose_candidate = None
    last_pose_reject_reason = ""
    err_first = None
    err_last = None
    err_min = None
    err_max = 0.0
    err_sum = 0.0
    err_count = 0
    err_recent = []
    vel_last = None
    att_last = None
    converged = False
    diverged = False
    marker_visibility_lost = False
    convergence_reason = ""
    min_ticks = int(EXTPOS_WARMUP_MIN_S * 1000 / TICK_MS)
    max_ticks = int(EXTPOS_WARMUP_MAX_S * 1000 / TICK_MS)
    for k in range(max_ticks):
        f = _detect_features()
        n_avg = _avg_push(n_avg_buf, f["n_full"])
        if f["n_full"] < n_min:
            n_min = f["n_full"]
        cf_yaw = 0.0
        try:
            p = sentai.crazy.pose()
            if p is not None and len(p) >= 4:
                cf_yaw = float(p[3])
        except Exception:
            pass
        try:
            pose = sentai.markers.get_drone_pose_tuple(cf_yaw)
        except Exception:
            pose = None
        pose_candidate = None
        if pose is not None:
            pose_candidate = (float(pose[0]), float(pose[1]), float(pose[2]))
            last_pose_candidate = pose_candidate
        extpos_candidate = _pose_to_extpos(pose_candidate)
        pose_ok, pose_reason = _pose_plausible_for_extpos(pose, f)
        if pose_ok:
            last_pose = last_pose_candidate
            last_extpos = extpos_candidate
            try:
                if sentai.crazy.send_extpos(
                        last_extpos[0], last_extpos[1], last_extpos[2]) == 0:
                    send_ok += 1
            except Exception:
                pass
        else:
            pose_rejects += 1
            last_pose_reject_reason = pose_reason
            reject_reasons[pose_reason] = reject_reasons.get(pose_reason, 0) + 1
        try:
            est = sentai.crazy.pose()
            if est is not None and len(est) >= 3:
                last_est_pose = (float(est[0]), float(est[1]), float(est[2]))
        except Exception:
            last_est_pose = None
        e = _pose_error_m(last_extpos, last_est_pose)
        if e is not None:
            if err_first is None:
                err_first = e
            err_last = e
            err_sum += e
            err_count += 1
            err_recent.append(e)
            if len(err_recent) > 10:
                err_recent.pop(0)
            if err_min is None or e < err_min:
                err_min = e
            if e > err_max:
                err_max = e
            if (k >= min_ticks and len(err_recent) >= LOCK_AVG_WINDOW and
                    z_target > 0.0):
                recent_mean_now = sum(err_recent) / len(err_recent)
                if recent_mean_now <= EXTPOS_CONVERGED_ERR_FRACTION * z_target:
                    converged = True
                    convergence_reason = "recent_error_below_visual_z_fraction"
                elif recent_mean_now >= EXTPOS_DIVERGED_ERR_FRACTION * z_target:
                    diverged = True
                    convergence_reason = "recent_error_above_visual_z_fraction"
        try:
            vel = sentai.crazy.velocity(80)
            if vel is not None and len(vel) >= 3:
                vel_last = (float(vel[0]), float(vel[1]), float(vel[2]))
        except Exception:
            pass
        try:
            att = sentai.crazy.attitude_get(80)
            if att is not None and len(att) >= 3:
                att_last = (float(att[0]), float(att[1]), float(att[2]))
        except Exception:
            pass
        thrust, z_prev, vz_filt = _z_thrust_from_feature(
            f, z_target, z_prev, vz_filt, T_HOVER_VISUAL_U16)
        roll, pitch = _center_cmd(f, axis)
        _rpyt(roll, pitch, 0.0, thrust)
        if k % 10 == 0:
            _j("extpos_warmup_tick", {
                "k": k, "pose": last_pose, "extpos": last_extpos,
                "pose_candidate": pose_candidate,
                "extpos_candidate": extpos_candidate,
                "pose_valid": pose_ok, "pose_reject_reason": pose_reason,
                "send_ok": send_ok,
                "est_pose": last_est_pose, "est_minus_pnp_m": e,
                "n_avg": n_avg,
                "converged": converged, "diverged": diverged,
                "est_velocity": vel_last, "est_attitude_deg": att_last,
                "feature": _feature_compact(f),
            })
        if len(n_avg_buf) >= LOCK_AVG_WINDOW and n_avg < MIN_FULL_MARKERS:
            marker_visibility_lost = True
            convergence_reason = "marker_visibility_lost"
            _j("extpos_warmup_marker_abort", {
                "k": k,
                "n_recent_avg": n_avg,
                "window": LOCK_AVG_WINDOW,
                "min_full_markers": MIN_FULL_MARKERS,
            })
            break
        if converged:
            break
        if diverged:
            break
        sentai.rtos.sleep_ms(TICK_MS)
    recent_mean = None
    if err_recent:
        recent_mean = sum(err_recent) / len(err_recent)
    err_mean = (err_sum / err_count) if err_count else None
    improving = False
    if err_first is not None and err_last is not None:
        improving = err_last <= err_first or (err_min is not None and err_min < err_first)
    ok = (last_pose is not None and send_ok > 5 and converged and
          not diverged and not marker_visibility_lost and n_avg_buf and
          (sum(n_avg_buf) / len(n_avg_buf)) > (MIN_FULL_MARKERS - 1))
    out = {
        "ok": ok,
        "ticks": k + 1,
        "converged": converged,
        "diverged": diverged,
        "marker_visibility_lost": marker_visibility_lost,
        "convergence_reason": convergence_reason,
        "last_pose": last_pose,
        "last_extpos": last_extpos,
        "last_est_pose": last_est_pose,
        "extpos_signs": (EXTPOS_SIGN_X, EXTPOS_SIGN_Y, EXTPOS_SIGN_Z),
        "n_full_min": n_min,
        "n_full_recent_avg": (sum(n_avg_buf) / len(n_avg_buf)) if n_avg_buf else 0.0,
        "send_ok": send_ok,
        "pose_rejects": pose_rejects,
        "pose_reject_reasons": reject_reasons,
        "last_pose_reject_reason": last_pose_reject_reason,
        "thrust_last": thrust,
        "kalman_crosscheck": {
            "samples": err_count,
            "err_first_m": err_first,
            "err_last_m": err_last,
            "err_mean_m": err_mean,
            "err_recent10_mean_m": recent_mean,
            "err_min_m": err_min,
            "err_max_m": err_max,
            "trend_improving": improving,
            "velocity_last_m_s": vel_last,
            "attitude_last_deg": att_last,
        },
    }
    summary["extpos_warmup"] = out
    _j("phase_extpos_warmup", out)
    return ok, thrust, last_pose, last_est_pose


def _phase_handoff_hover(summary, z_target, last_pose, last_est_pose):
    _j("phase_handoff_hover", {
        "z_target_m": z_target,
        "last_pose": last_pose,
        "last_est_pose": last_est_pose,
    })
    hover_z = z_target
    hover_z_source = "visual_z_target"
    if last_est_pose is not None and last_est_pose[2] > 0.2:
        # Generic hover z is interpreted by Crazyflie in its estimator frame.
        # Use stateEstimate.z at handoff to avoid a Z step if ExtPos/PNP and
        # the onboard EKF disagree.
        hover_z = last_est_pose[2]
        hover_z_source = "cf_state_estimate_z"
    elif last_pose is not None and last_pose[2] > 0.2:
        hover_z = last_pose[2]
        hover_z_source = "pnp_pose_z_fallback"
    hold_pose = (0.0, 0.0, hover_z)
    hold_pose_source = "marker_pad_origin_with_cf_z"
    hl_hold_dur = _hl_hold_duration_from_pose(last_est_pose)
    hl_takeoff_rc = None
    try:
        # Engage the HL planner while RPYT still owns commander priority.
        # Prior s193/s194 analysis showed that a direct go_to from an IDLE HL
        # planner after RPYT can produce invalid/unstable trajectories.  A
        # takeoff-to-current-height trajectory seeds planner state from the
        # current Kalman pose without commanding an XY move yet.
        hl_takeoff_rc = sentai.crazy.takeoff(hover_z, HL_ENGAGE_DUR_S, 0.0, 1, 0)
    except Exception as e:
        summary["handoff_hl_takeoff_error"] = str(e)[:120]
    sentai.rtos.sleep_ms(15)
    release_rc = _release_rpyt_without_disarm()
    hl_go_to_rc = None
    try:
        hl_go_to_rc = sentai.crazy.go_to(float(hold_pose[0]),
                                         float(hold_pose[1]),
                                         float(hold_pose[2]),
                                         0.0,
                                         hl_hold_dur,
                                         0,
                                         0,
                                         0)
    except Exception as e:
        summary["handoff_hl_go_to_error"] = str(e)[:120]
    n_min = 99
    n_avg_buf = []
    n_recent_avg = 0.0
    est_last = last_est_pose
    send_ok = 0
    pose_rejects = 0
    reject_reasons = {}
    att_stats = _att_stats_new()
    ticks = int(HOVER_HOLD_S * 1000 / TICK_MS)
    for k in range(ticks):
        pump = _send_extpos_from_current_markers()
        f = pump["feature"]
        att = _attitude_sample()
        _att_stats_push(att_stats, att)
        n_recent_avg = _avg_push(n_avg_buf, f["n_full"])
        if f["n_full"] < n_min:
            n_min = f["n_full"]
        pose = pump["pose"]
        extpos = pump["extpos"]
        pose_ok = pump["pose_ok"]
        pose_reason = pump["pose_reason"]
        if pump["send_ok"]:
            send_ok += 1
        else:
            pose_rejects += 1
            reject_reasons[pose_reason] = reject_reasons.get(pose_reason, 0) + 1
        if pump["est_pose"] is not None:
            est_last = pump["est_pose"]
        if k % 10 == 0:
            _j("handoff_hover_tick", {
                "k": k, "hover_z": hover_z, "n_full": f["n_full"],
                "n_recent_avg": n_recent_avg,
                "feature": _feature_compact(f),
                "hold_pose": hold_pose,
                "hold_pose_source": hold_pose_source,
                "est_pose": est_last,
                "hl_takeoff_rc": hl_takeoff_rc,
                "hl_go_to_rc": hl_go_to_rc,
                "hl_hold_dur_s": hl_hold_dur,
                "attitude_deg": att,
                "pose": pose,
                "extpos": extpos,
                "cf_yaw_rad": pump.get("cf_yaw_rad"),
                "pose_valid": pose_ok,
                "pose_reject_reason": pose_reason,
                "extpos_send_ok": send_ok,
            })
        if len(n_avg_buf) >= LOCK_AVG_WINDOW and n_recent_avg < MIN_FULL_MARKERS:
            _j("handoff_hover_marker_abort", {
                "k": k,
                "n_recent_avg": n_recent_avg,
                "window": LOCK_AVG_WINDOW,
            })
            break
        sentai.rtos.sleep_ms(TICK_MS)
    ok = (release_rc == 0 and hl_takeoff_rc == 0 and hl_go_to_rc == 0 and
          n_recent_avg >= MIN_FULL_MARKERS)
    out = {
        "ok": ok,
        "release_rc": release_rc,
        "hl_takeoff_rc": hl_takeoff_rc,
        "hl_engage_dur_s": HL_ENGAGE_DUR_S,
        "hl_go_to_rc": hl_go_to_rc,
        "hl_hold_dur_s": hl_hold_dur,
        "hl_lateral_speed_limit_m_s": HL_LATERAL_SPEED_LIMIT_M_S,
        "hold_pose": hold_pose,
        "hold_pose_source": hold_pose_source,
        "last_est_pose": est_last,
        "extpos_send_ok": send_ok,
        "pose_rejects": pose_rejects,
        "pose_reject_reasons": reject_reasons,
        "hover_z": hover_z,
        "hover_z_source": hover_z_source,
        "n_full_min": n_min,
        "n_full_recent_avg": n_recent_avg,
        "attitude_stats": _att_stats_finish(att_stats),
    }
    summary["handoff_hover"] = out
    _j("phase_handoff_hover", out)
    return ok


def _phase_hl_axis_motion(summary):
    hover = summary.get("handoff_hover") or {}
    hover_z = hover.get("hover_z")
    if hover_z is None:
        hover_z = (hover.get("last_est_pose") or (0.0, 0.0, 0.6))[2]
    hover_z = float(hover_z)
    step_m, step_meta = _hl_axis_step_from_z(hover_z)
    waypoints = (
        ("plus_x", +step_m, 0.0, hover_z),
        ("minus_x", -step_m, 0.0, hover_z),
        ("center_after_x", 0.0, 0.0, hover_z),
        ("plus_y", 0.0, +step_m, hover_z),
        ("minus_y", 0.0, -step_m, hover_z),
        ("center_after_y", 0.0, 0.0, hover_z),
    )
    _j("phase_hl_axis_motion", {
        "z_m": hover_z,
        "step": step_meta,
        "speed_limit_m_s": HL_LATERAL_SPEED_LIMIT_M_S,
        "settle_s": HL_SEGMENT_SETTLE_S,
    })

    segments = []
    last_target = (0.0, 0.0, hover_z)
    n_global_min = 99
    n_avg_buf = []
    n_recent_avg = 0.0
    total_send_ok = 0
    total_pose_rejects = 0
    reject_reasons = {}
    abort_reason = ""
    last_est = hover.get("last_est_pose")

    for label, x, y, z in waypoints:
        target = (float(x), float(y), float(z))
        dist = _dist3(last_target, target)
        duration = _hl_duration_for_distance(dist if dist is not None else step_m)
        go_to_rc = None
        try:
            go_to_rc = sentai.crazy.go_to(target[0], target[1], target[2],
                                          0.0, duration, 0, HL_AXIS_LINEAR, 0)
        except Exception as e:
            go_to_rc = -999
            summary["hl_axis_motion_go_to_error"] = str(e)[:120]

        ticks = int((duration + HL_SEGMENT_SETTLE_S) * 1000 / TICK_MS)
        seg_send_ok = 0
        seg_pose_rejects = 0
        seg_n_min = 99
        seg_n_avg_last = 0.0
        seg_est_first = last_est
        seg_est_last = last_est
        seg_err_first = _dist3(seg_est_first, target)
        seg_err_last = seg_err_first
        seg_att_stats = _att_stats_new()

        for k in range(ticks):
            pump = _send_extpos_from_current_markers()
            att = _attitude_sample()
            _att_stats_push(seg_att_stats, att)
            f = pump["feature"]
            n_recent_avg = _avg_push(n_avg_buf, f["n_full"])
            seg_n_avg_last = n_recent_avg
            if f["n_full"] < n_global_min:
                n_global_min = f["n_full"]
            if f["n_full"] < seg_n_min:
                seg_n_min = f["n_full"]
            if pump["send_ok"]:
                seg_send_ok += 1
                total_send_ok += 1
            else:
                seg_pose_rejects += 1
                total_pose_rejects += 1
                reason = pump["pose_reason"]
                reject_reasons[reason] = reject_reasons.get(reason, 0) + 1
            if pump["est_pose"] is not None:
                seg_est_last = pump["est_pose"]
                last_est = pump["est_pose"]
                seg_err_last = _dist3(seg_est_last, target)
            if k % 10 == 0:
                _j("hl_axis_motion_tick", {
                    "segment": label,
                    "k": k,
                    "target": target,
                    "duration_s": duration,
                    "linear": HL_AXIS_LINEAR,
                    "feature": _feature_compact(f),
                    "n_recent_avg": n_recent_avg,
                    "pose_valid": pump["pose_ok"],
                    "pose_reject_reason": pump["pose_reason"],
                    "extpos": pump["extpos"],
                    "est_pose": pump["est_pose"],
                    "attitude_deg": att,
                    "cf_yaw_rad": pump.get("cf_yaw_rad"),
                    "est_target_err_m": seg_err_last,
                })
            if len(n_avg_buf) >= LOCK_AVG_WINDOW and n_recent_avg < MIN_FULL_MARKERS:
                abort_reason = "marker_visibility_lost_%s" % label
                _j("hl_axis_motion_marker_abort", {
                    "segment": label,
                    "k": k,
                    "n_recent_avg": n_recent_avg,
                    "window": LOCK_AVG_WINDOW,
                })
                break
            sentai.rtos.sleep_ms(TICK_MS)

        seg = {
            "label": label,
            "target": target,
            "from_target": last_target,
            "distance_m": dist,
            "duration_s": duration,
            "go_to_rc": go_to_rc,
            "linear": HL_AXIS_LINEAR,
            "ticks": ticks,
            "extpos_send_ok": seg_send_ok,
            "pose_rejects": seg_pose_rejects,
            "n_full_min": seg_n_min,
            "n_full_recent_avg": seg_n_avg_last,
            "est_first": seg_est_first,
            "est_last": seg_est_last,
            "est_target_err_first_m": seg_err_first,
            "est_target_err_last_m": seg_err_last,
            "attitude_stats": _att_stats_finish(seg_att_stats),
        }
        segments.append(seg)
        _j("hl_axis_motion_segment", seg)
        last_target = target
        if abort_reason:
            break

    ok = (not abort_reason and segments and
          all(seg.get("go_to_rc") == 0 for seg in segments) and
          n_recent_avg >= MIN_FULL_MARKERS)
    out = {
        "ok": ok,
        "abort_reason": abort_reason,
        "step": step_meta,
        "speed_limit_m_s": HL_LATERAL_SPEED_LIMIT_M_S,
        "segments": segments,
        "last_est_pose": last_est,
        "n_full_min": n_global_min,
        "n_full_recent_avg": n_recent_avg,
        "extpos_send_ok": total_send_ok,
        "pose_rejects": total_pose_rejects,
        "pose_reject_reasons": reject_reasons,
    }
    summary["hl_axis_motion"] = out
    _j("phase_hl_axis_motion", out)
    return ok


def _phase_land(summary):
    _j("phase_land", {"dur_s": HL_LAND_DUR_S})
    out = {"land_rc": None, "disarm_rc": None}
    try:
        try:
            out["pre_land_relax_rc"] = _release_rpyt_without_disarm()
        except Exception as e:
            out["pre_land_relax_error"] = str(e)[:120]
        out["land_rc"] = sentai.crazy.land(0.0, HL_LAND_DUR_S)
        ticks = int((HL_LAND_DUR_S + 0.8) * 1000 / TICK_MS)
        n_avg_buf = []
        n_recent_avg = 0.0
        n_min = 99
        send_ok = 0
        pose_rejects = 0
        reject_reasons = {}
        est_last = None
        att_stats = _att_stats_new()
        for k in range(ticks):
            pump = _send_extpos_from_current_markers()
            att = _attitude_sample()
            _att_stats_push(att_stats, att)
            f = pump["feature"]
            n_recent_avg = _avg_push(n_avg_buf, f["n_full"])
            if f["n_full"] < n_min:
                n_min = f["n_full"]
            if pump["send_ok"]:
                send_ok += 1
            else:
                pose_rejects += 1
                reason = pump["pose_reason"]
                reject_reasons[reason] = reject_reasons.get(reason, 0) + 1
            if pump["est_pose"] is not None:
                est_last = pump["est_pose"]
            if k % 10 == 0:
                _j("land_tick", {
                    "k": k,
                    "feature": _feature_compact(f),
                    "n_recent_avg": n_recent_avg,
                    "pose_valid": pump["pose_ok"],
                    "pose_reject_reason": pump["pose_reason"],
                    "extpos": pump["extpos"],
                    "est_pose": pump["est_pose"],
                    "attitude_deg": att,
                    "cf_yaw_rad": pump.get("cf_yaw_rad"),
                })
            sentai.rtos.sleep_ms(TICK_MS)
        out["extpos_send_ok"] = send_ok
        out["pose_rejects"] = pose_rejects
        out["pose_reject_reasons"] = reject_reasons
        out["n_full_min"] = n_min
        out["n_full_recent_avg"] = n_recent_avg
        out["last_est_pose"] = est_last
        out["attitude_stats"] = _att_stats_finish(att_stats)
    except Exception as e:
        out["land_error"] = str(e)[:120]
    try:
        out["disarm_rc"] = sentai.crazy.disarm()
    except Exception as e:
        out["disarm_error"] = str(e)[:120]
    out["ok"] = out.get("land_rc") == 0
    summary["land"] = out
    _j("phase_land", out)
    return out["ok"]


def _safe_stop(summary, reason):
    _j("safe_stop", {"reason": reason})
    summary["safe_stop_reason"] = reason
    try:
        sentai.crazy.fly_stop()
    except Exception:
        pass
    sentai.rtos.sleep_ms(500)


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    summary = {
        "experiment": "s201_inflight_handoff_hl_marker_control",
        "task": "TD-S10-B4",
        "status": "ERROR",
        "phase": "start",
        "world_expected": "sentai_whycon_small",
        "goal": "A3 RPYT bootstrap -> load calib.ini -> ExtPos warmup -> HL handoff -> +x/-x/center/+y/-y/center -> land",
    }
    try:
        _set_phase(summary, "setup")
        setup_ok, axis = _phase_setup(summary)
        if not setup_ok:
            summary["status"] = "SETUP_FAIL"
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "acquire")
        ok, thrust, z_target = _phase_acquire(summary)
        if not ok:
            summary["status"] = "ACQUIRE_FAIL"
            _safe_stop(summary, "acquire_fail")
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "center_hold")
        ok, thrust = _phase_center_hold(summary, axis, thrust, z_target)
        if not ok:
            summary["status"] = "CENTER_HOLD_FAIL"
            _safe_stop(summary, "center_hold_fail")
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "extpos_warmup")
        ok, thrust, last_pose, last_est_pose = _phase_extpos_warmup(
            summary, axis, thrust, z_target)
        if not ok:
            summary["status"] = "EXTPOS_WARMUP_FAIL"
            _safe_stop(summary, "extpos_warmup_fail")
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "handoff_hover")
        ok = _phase_handoff_hover(summary, z_target, last_pose, last_est_pose)
        if not ok:
            summary["status"] = "HANDOFF_HOVER_FAIL"
            _safe_stop(summary, "handoff_hover_fail")
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "hl_axis_motion")
        ok = _phase_hl_axis_motion(summary)
        if not ok:
            summary["status"] = "HL_AXIS_MOTION_FAIL"
            _safe_stop(summary, "hl_axis_motion_fail")
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        _set_phase(summary, "land")
        land_ok = _phase_land(summary)
        summary["status"] = "HL_AXIS_MOTION_OK" if land_ok else "LAND_FAIL"
        _write_summary(summary)
        sentai.sim.journal_close()
        return summary
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = str(e)[:180]
        _safe_stop(summary, "exception")
        _write_summary(summary)
        try:
            sentai.sim.journal_close()
        except Exception:
            pass
        return summary
