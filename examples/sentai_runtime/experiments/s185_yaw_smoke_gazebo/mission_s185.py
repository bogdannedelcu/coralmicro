# mission_s185 -- OP-S10-W19-T7 yaw smoke in Gazebo SIM.
#
# First mission-level test of sentai.markers.get_drone_pose, the
# runtime port of the yaw-anchored Kabsch picker shipped under
# OP-S10-W19-T6b (commit 631db542).
#
# Anti-cheat: sentai_sim sees only camera frames + cf2 CRTP telemetry.
# No GT injection.  cf2_yaw fed into the picker is the cf2 EKF yaw via
# CRTP LOG -- independent of marker observations, valid yaw anchor.
#
# Flight plan:
#   1. markers.init('whycon') + intrinsics + extrinsics + marker_world
#   2. crazy.init + arm + takeoff(Z_HOLD)
#   3. hl_stop + low-level hover-pin (s174 pattern)
#   4. Yaw sweep: hover(0, 0, YAW_RATE, Z_HOLD) for SWEEP_TICKS ticks
#      -> total rotation ~360 deg
#   5. Per tick (10 Hz): cf2 pose snapshot, detect_from_camera,
#      get_drone_pose_tuple(cf2_yaw), journal
#   6. Stop yaw + land + disarm

import sentai

JOURNAL_NAME = "mission_s185_journal.txt"
SUMMARY_NAME = "mission_s185_summary.json"

# Camera intrinsics + marker geometry — verbatim from mission_s183.
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0
MARKER_DIAMETER_M = 0.1088

# 6-marker square pad (verbatim from s183 + s184).  Symmetric under
# R_180z about (0,0) — the picker's job is exactly to disambiguate.
MARKER_WORLD = (
    (-0.16, +0.16, 0.005),   # 0  NW
    (+0.16, +0.16, 0.005),   # 1  NE
    (-0.12,  0.00, 0.005),   # 2  W
    (+0.12,  0.00, 0.005),   # 3  E
    (-0.16, -0.16, 0.005),   # 4  SW
    (+0.16, -0.16, 0.005),   # 5  SE
)

# Flight timing.  s183 values preserved where possible.
Z_HOLD            = 0.78
TAKEOFF_DUR       = 2.5
LAND_DUR          = 2.5
SETTLE_S          = 4.0
PIN_TICKS         = 5
TICK_INTERVAL_MS  = 100   # 10 Hz log cadence

# Pre-sweep stationary log: prove picker fires at yaw=0 before rotation.
HOVER_TICKS_PRE   = 30    # 3 s at yaw=0
# Yaw sweep: 10 deg/s for ~36 s -> ~360 deg total.
YAW_RATE_RAD_S    = 0.1745   # 10 deg/s in radians
SWEEP_TICKS       = 360      # 36 s at 10 Hz
# Post-sweep stationary log: prove picker still works after rotation.
HOVER_TICKS_POST  = 30


def _j(event, payload):
    sentai.sim.journal_write(event, payload)


def _sleep_after_cmd(dur_s, extra_ms=200):
    sentai.rtos.sleep_ms(int(dur_s * 1000) + extra_ms)


def _ser_val(v):
    if v is None:               return "null"
    if isinstance(v, bool):     return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, str):
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return '"' + s + '"'
    if isinstance(v, list):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join(
            ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        ) + "}"
    return '"<%s>"' % type(v).__name__


def _write_summary(summary):
    sentai.fs.write(SUMMARY_NAME, _ser_val(summary))


def _read_cf2_pose():
    try:
        return sentai.crazy.pose()
    except (AttributeError, RuntimeError):
        return None


def _pack_marker_world():
    # bytes-of-floats packer (MicroPython embed has no struct).  6 markers
    # * 3 coords * 4 bytes = 72 bytes total.
    import struct
    out = b""
    for (x, y, z) in MARKER_WORLD:
        out += struct.pack("<fff", x, y, z)
    return out


def _log_tick(phase, k, yaw_cmd_rate):
    """One logging tick: read cf2 pose, run detection, call picker,
    journal everything."""
    cf2 = _read_cf2_pose()
    n = sentai.markers.detect_from_camera()
    cf2_yaw = cf2[3] if (cf2 is not None) else 0.0

    pose = None
    if n >= 3:
        pose = sentai.markers.get_drone_pose_tuple(cf2_yaw)
        # tuple: (x, y, z, yaw_rad, res_max, n_used, flip_x, flip_y, flip_z)

    _j("tick", {
        "phase":   phase,
        "k":       k,
        "yaw_cmd": yaw_cmd_rate,
        "cf2":     cf2,
        "n":       n,
        "pose":    pose,
    })


def run():
    sentai.sim.journal_open(JOURNAL_NAME)

    summary = {
        "name":          "mission_s185",
        "version":       sentai.version(),
        "status":        "STARTED",
        "phases_done":   [],
        "z_hold":        Z_HOLD,
        "ticks_logged":  0,
        "errors":        [],
    }

    try:
        # ---- markers setup ---------------------------------------
        rc = sentai.markers.init("whycon")
        _j("markers_init", {"rc": rc, "backend": sentai.markers.backend()})
        if rc != 0:
            summary["errors"].append("markers.init('whycon') rc=%d" % rc)
            summary["status"] = "FAIL_MARKERS_INIT"
            return summary
        sentai.markers.set_intrinsics(FX, FY, CX, CY)
        sentai.markers.set_marker_size(MARKER_DIAMETER_M)

        # cf2 SDF cam mount → tvec in body frame (T5).
        import math as _math
        sentai.markers.set_cam_extrinsics(
            -0.04, 0.0, -0.02,
            0.0, _math.pi / 2.0, _math.pi)
        _j("markers_extrinsics_set", {
            "t": (-0.04, 0.0, -0.02),
            "rpy": (0.0, _math.pi / 2.0, _math.pi),
        })

        # Register marker world geometry for the picker.
        rc_w = sentai.markers.set_marker_world(_pack_marker_world())
        n_w = sentai.markers.get_marker_world_count()
        _j("marker_world_registered", {"rc": rc_w, "n": n_w})
        if n_w != len(MARKER_WORLD):
            summary["errors"].append(
                "marker_world register mismatch %d vs %d" % (n_w, len(MARKER_WORLD)))
            summary["status"] = "FAIL_WORLD_REGISTER"
            return summary

        summary["phases_done"].append("markers_init")

        # ---- crazy.init + arm + pose_subscribe -------------------
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["errors"].append("crazy.init rc=%d" % rc)
            summary["status"] = "FAIL_CRAZY_INIT"
            return summary

        sentai.crazy.arm()
        _j("crazy_arm", {})
        sentai.rtos.sleep_ms(300)

        sub_rc = -3
        _try = 0
        for _try in range(30):
            try:
                sub_rc = sentai.crazy.pose_subscribe(50)
            except (AttributeError, RuntimeError) as ex:
                _j("pose_subscribe_err", {"try": _try, "err": repr(ex)})
                sub_rc = -99
            if sub_rc == 0:
                break
            sentai.rtos.sleep_ms(200)
        _j("pose_subscribe", {"rc": sub_rc, "period_ms": 50, "tries": _try + 1})
        sentai.rtos.sleep_ms(500)

        # ---- takeoff + settle + hl_stop pin ----------------------
        rc = sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)
        _j("takeoff", {"h": Z_HOLD, "rc": rc})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        sentai.rtos.sleep_ms(int(SETTLE_S * 1000))
        _j("takeoff_settled", {})

        try:
            _j("crazy_hl_stop", {"rc": sentai.crazy.hl_stop()})
        except (AttributeError, RuntimeError) as ex:
            _j("hl_stop_err", {"err": repr(ex)})

        for _ in range(PIN_TICKS):
            try:
                sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
            except (AttributeError, RuntimeError):
                pass
            sentai.rtos.sleep_ms(30)
        _j("hover_pin_ready", {"alt": Z_HOLD})
        summary["phases_done"].append("takeoff")

        # ---- Phase A: stationary hover at yaw=0 -----------------
        _j("phase_A_start", {"ticks": HOVER_TICKS_PRE})
        tick_idx = 0
        for k in range(HOVER_TICKS_PRE):
            if k % 5 == 0:
                try:
                    sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
                except (AttributeError, RuntimeError):
                    pass
            _log_tick("A_pre", tick_idx, 0.0)
            tick_idx += 1
            sentai.rtos.sleep_ms(TICK_INTERVAL_MS)
        _j("phase_A_done", {"ticks": HOVER_TICKS_PRE})

        # ---- Phase B: yaw sweep at YAW_RATE_RAD_S ---------------
        _j("phase_B_start", {"yaw_rate_rad_s": YAW_RATE_RAD_S,
                              "ticks": SWEEP_TICKS})
        for k in range(SWEEP_TICKS):
            # Continuously command yaw rotation; cf2 EKF integrates.
            try:
                sentai.crazy.hover(0.0, 0.0, YAW_RATE_RAD_S, Z_HOLD)
            except (AttributeError, RuntimeError):
                pass
            _log_tick("B_sweep", tick_idx, YAW_RATE_RAD_S)
            tick_idx += 1
            sentai.rtos.sleep_ms(TICK_INTERVAL_MS)
        _j("phase_B_done", {"ticks": SWEEP_TICKS})

        # ---- Phase C: stationary again, yaw=0 -------------------
        _j("phase_C_start", {"ticks": HOVER_TICKS_POST})
        for k in range(HOVER_TICKS_POST):
            if k % 5 == 0:
                try:
                    sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
                except (AttributeError, RuntimeError):
                    pass
            _log_tick("C_post", tick_idx, 0.0)
            tick_idx += 1
            sentai.rtos.sleep_ms(TICK_INTERVAL_MS)
        _j("phase_C_done", {"ticks": HOVER_TICKS_POST})

        summary["ticks_logged"] = tick_idx
        summary["phases_done"].append("yaw_sweep")

        # ---- land + disarm ---------------------------------------
        rc = sentai.crazy.land(0.0, LAND_DUR)
        _j("land", {"rc": rc})
        _sleep_after_cmd(LAND_DUR)
        sentai.crazy.disarm()
        _j("disarm", {})
        summary["phases_done"].append("land")

        summary["status"] = "OK"
    except Exception as e:
        summary["errors"].append(repr(e))
        summary["status"] = "EXCEPTION"
        try:
            sentai.crazy.land(0.0, 1.5)
            sentai.rtos.sleep_ms(1800)
            sentai.crazy.disarm()
        except Exception:
            pass
    finally:
        _write_summary(summary)
        sentai.sim.journal_close()
    return summary


if __name__ == "__main__":
    print(repr(run()))
