# mission_s192 — minimal WhyCon detection regression in cf2 flight.
#
# Goal: prove that the T10 OpenCV-parity WhyCon detector
# (commit b4eb37ce, sentai_sim build #558+) reliably sees the
# 6-marker pad during actual cf2 hover.  Yesterday's s191 returned
# n=0 / z_pnp=-1.0 for hundreds of climb ticks — this is the
# regression companion of T10's 368-frame static ablation.
#
# Phases:
#   1. setup (markers, intrinsics, marker_world, safety, crazy, arm)
#   2. zero-unlock RPYT
#   3. thrust ramp until first valid PnP
#   4. PD altitude lock (Z stable at ramp-handoff altitude)
#   5. DETECTION MEASUREMENT WINDOW (5 s) — record n, pnp_xyz, yaw per tick
#   6. RPYT ramp-down land + disarm
#
# Pass criterion: ≥80% of measurement-window ticks have n>=4, at
# least one tick reaches n==6 (full pad in FOV).
#
# WBS: OP-S10-W21-T12 regression.

import sentai
import struct
import math


# ---- Names + paths ----------------------------------------------------
JOURNAL_NAME = "mission_s192_journal.txt"
SUMMARY_NAME = "mission_s192_summary.json"

# ---- WhyCon pad geometry (sentai_whycon_small.sdf — verified 2026-05-23)
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),  # NW
    (+0.08, +0.08, 0.005),  # NE
    (-0.06,  0.00, 0.005),  # W
    (+0.06,  0.00, 0.005),  # E
    (-0.08, -0.08, 0.005),  # SW
    (+0.08, -0.08, 0.005),  # SE
)

# ---- Camera intrinsics ----
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.0544

# ---- RPYT ramp + PD altitude params (proven in s190 iter-21) ---------
T_BASE_U16       = 30000
T_HOVER_NOMINAL  = 36000
T_MIN_U16        = 22000
T_MAX_HOLD_U16   = 40000
T_MAX_U16        = 37000           # ramp cap
RAMP_S           = 6.0
TICK_MS          = 33              # ≈ 30 Hz
MAX_RAMP_S       = 12.0
N_STABLE_FRAMES  = 2

KP_Z_THRUST_PER_M       = 10000.0
KD_Z_THRUST_PER_M_PER_S = 8000.0
VZ_LPF_ALPHA            = 0.25

PD_LOCK_MAX_S    = 4.0
V_STABLE_TOL_M_S = 0.10

# ---- Detection measurement window ----
MEASURE_WINDOW_S        = 5.0
PASS_FRAC_N_GE_4        = 0.80     # ≥80% ticks with n>=4
REQUIRE_FULL_PAD_AT_LEAST_ONCE = True   # at least 1 tick with n>=6

# ---- Landing ----
LAND_DUR         = 2.5
ZERO_UNLOCK_S    = 1.5


def _j(event, payload):
    sentai.sim.journal_write(event, payload)


def _ser_val(v):
    if v is None:               return "null"
    if isinstance(v, bool):     return "true" if v else "false"
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


# ---- CRTP helpers -----------------------------------------------------

def _rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16):
    """Classic Commander setpoint (CRTP port 3 ch 0)."""
    if roll_deg  >  15.0:  roll_deg  =  15.0
    if roll_deg  < -15.0:  roll_deg  = -15.0
    if pitch_deg >  15.0:  pitch_deg =  15.0
    if pitch_deg < -15.0:  pitch_deg = -15.0
    sentai.crazy.send_crtp(3, 0,
        struct.pack('<fffH', roll_deg, pitch_deg, yaw_rate_deg_s,
                    thrust_u16))


def _try_pnp():
    """(n, x, y, z, yaw_rad) or (n, -1, -1, -1, 0) on PnP failure."""
    n = sentai.markers.detect_from_camera()
    if n < 4:
        return n, -1.0, -1.0, -1.0, 0.0
    try:
        p = sentai.crazy.pose()
        cf2_yaw = float(p[3]) if (p is not None and len(p) >= 4) else 0.0
    except (AttributeError, RuntimeError, TypeError):
        cf2_yaw = 0.0
    pose = sentai.markers.get_drone_pose_tuple(cf2_yaw)
    if pose is None:
        return n, -1.0, -1.0, -1.0, 0.0
    x, y, z, yaw_rad = float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3])
    if not (x == x and y == y and z == z and yaw_rad == yaw_rad):
        return n, -1.0, -1.0, -1.0, 0.0
    if z < 0.10:
        return n, -1.0, -1.0, -1.0, 0.0
    return n, x, y, z, yaw_rad


# ============================ PHASES =================================

def _phase1_setup():
    _j("phase1_setup", "start")
    sentai.calib.init()
    sentai.calib.clear()
    sentai.camera.init()
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)

    mw_bytes = b''
    for (x, y, z) in MARKER_WORLD:
        mw_bytes += struct.pack('<fff', x, y, z)
    rc_mw = sentai.markers.set_marker_world(mw_bytes)
    _j("set_marker_world", {"n": len(MARKER_WORLD), "rc": rc_mw})

    sentai.safety.init()
    sentai.crazy.init()
    sentai.crazy.arm()
    sentai.rtos.sleep_ms(300)

    sub_rc = -3
    for _try in range(30):
        try:
            sub_rc = sentai.crazy.pose_subscribe(50)
        except (AttributeError, RuntimeError):
            sub_rc = -99
        if sub_rc == 0:
            break
        sentai.rtos.sleep_ms(200)
    _j("pose_subscribe", {"rc": sub_rc, "tries": _try + 1})
    sentai.rtos.sleep_ms(500)
    _j("phase1_setup", "done")


def _phase2_zero_unlock():
    _j("phase2_zero_unlock", {"dur_s": ZERO_UNLOCK_S})
    n_ticks = int(ZERO_UNLOCK_S * 1000 / TICK_MS)
    for _ in range(n_ticks):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)
    _j("phase2_zero_unlock", "done")


def _phase3_thrust_ramp():
    _j("phase3_thrust_ramp", {
        "T_BASE": T_BASE_U16, "T_MAX": T_MAX_U16, "RAMP_S": RAMP_S,
        "MAX_RAMP_S": MAX_RAMP_S, "N_STABLE": N_STABLE_FRAMES,
    })
    n_ticks_ramp = int(RAMP_S * 1000 / TICK_MS)
    max_ticks    = int(MAX_RAMP_S * 1000 / TICK_MS)
    stable_streak = 0
    last_pnp = None
    thrust = T_BASE_U16

    for ti in range(max_ticks):
        if ti < n_ticks_ramp:
            thrust = T_BASE_U16 + int(
                (T_MAX_U16 - T_BASE_U16) * ti / n_ticks_ramp)
        else:
            thrust = T_MAX_U16

        _rpyt(0.0, 0.0, 0.0, thrust)
        n, px, py, pz, pyaw = _try_pnp()
        pnp_valid = (n >= 4) and (pz > 0.0)

        if (ti % 8) == 0:
            _j("ramp_tick", {"t": ti, "thrust": thrust, "n": n,
                              "pnp_xyz": (px, py, pz)})

        if pnp_valid:
            stable_streak += 1
            last_pnp = (px, py, pz, pyaw)
            if stable_streak >= N_STABLE_FRAMES:
                _j("ramp_handoff", {
                    "t": ti, "thrust": thrust,
                    "stable_streak": stable_streak,
                    "pnp_xyz": (px, py, pz), "yaw_rad": pyaw,
                })
                return thrust, last_pnp
        else:
            stable_streak = 0

        sentai.rtos.sleep_ms(TICK_MS)

    _j("ramp_timeout", {"final_thrust": thrust})
    return None, None


def _phase4_pd_altitude_lock(first_pnp):
    z_target = first_pnp[2]
    z_prev   = first_pnp[2]
    vz_filt  = 0.0
    last_xyz_yaw = first_pnp
    stable_count = 0
    max_ticks = int(PD_LOCK_MAX_S * 1000 / TICK_MS)
    thrust = T_HOVER_NOMINAL

    _j("phase4_pd_alt", {"z_target": z_target,
                           "Kp_z": KP_Z_THRUST_PER_M,
                           "Kd_z": KD_Z_THRUST_PER_M_PER_S})

    for ti in range(max_ticks):
        n, px, py, pz, pyaw = _try_pnp()
        if n >= 4 and pz > 0.0:
            z_now = pz
            last_xyz_yaw = (px, py, pz, pyaw)
        else:
            z_now = z_prev

        dt_s = TICK_MS / 1000.0
        vz_raw  = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt

        err_z  = z_target - z_now
        thrust = T_HOVER_NOMINAL + int(
            KP_Z_THRUST_PER_M * err_z - KD_Z_THRUST_PER_M_PER_S * vz_filt)
        if thrust < T_MIN_U16:    thrust = T_MIN_U16
        if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16

        _rpyt(0.0, 0.0, 0.0, thrust)

        if (ti % 3) == 0:
            _j("pd_alt_tick", {"t": ti, "n": n, "z_now": z_now,
                                "vz_filt": vz_filt, "thrust": thrust,
                                "stable": stable_count})

        if (abs(vz_filt) < V_STABLE_TOL_M_S
                and n >= 4 and pz > 0.0):
            stable_count += 1
            if stable_count >= 4:
                _j("phase4_pd_alt", {"stable": True,
                                       "final_xyz_yaw": last_xyz_yaw,
                                       "final_thrust": thrust})
                return last_xyz_yaw, thrust, vz_filt
        else:
            stable_count = 0

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase4_pd_alt", {"stable": False, "timed_out": True})
    return last_xyz_yaw, thrust, vz_filt


def _phase5_detect_window(pd_state):
    """Measurement window: hold altitude, log n + pnp every tick.
    This is the regression target — pre-T10 returned n=0 here."""
    pd_xyz_yaw, _, pd_vz_filt = pd_state
    z_target = pd_xyz_yaw[2]
    z_prev   = pd_xyz_yaw[2]
    vz_filt  = pd_vz_filt
    last_xyz_yaw = pd_xyz_yaw

    _j("phase5_detect_window", {
        "window_s": MEASURE_WINDOW_S,
        "z_target": z_target,
        "pass_frac": PASS_FRAC_N_GE_4,
    })

    n_ticks = int(MEASURE_WINDOW_S * 1000 / TICK_MS)
    n_total      = 0
    n_ge4        = 0
    n_ge6        = 0
    n_pnp_valid  = 0
    n_histogram  = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
    pnp_x_sum = pnp_y_sum = pnp_z_sum = 0.0
    pnp_x_min = pnp_y_min = pnp_z_min =  1e9
    pnp_x_max = pnp_y_max = pnp_z_max = -1e9

    for ti in range(n_ticks):
        n, px, py, pz, pyaw = _try_pnp()
        n_total += 1
        if n >= 8:
            n_histogram[8] += 1
        elif n in n_histogram:
            n_histogram[n] += 1
        if n >= 4: n_ge4 += 1
        if n >= 6: n_ge6 += 1

        if n >= 4 and pz > 0.0:
            n_pnp_valid += 1
            last_xyz_yaw = (px, py, pz, pyaw)
            z_now = pz
            pnp_x_sum += px; pnp_y_sum += py; pnp_z_sum += pz
            if px < pnp_x_min: pnp_x_min = px
            if px > pnp_x_max: pnp_x_max = px
            if py < pnp_y_min: pnp_y_min = py
            if py > pnp_y_max: pnp_y_max = py
            if pz < pnp_z_min: pnp_z_min = pz
            if pz > pnp_z_max: pnp_z_max = pz
        else:
            z_now = z_prev

        dt_s = TICK_MS / 1000.0
        vz_raw  = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt
        err_z  = z_target - z_now
        thrust = T_HOVER_NOMINAL + int(
            KP_Z_THRUST_PER_M * err_z - KD_Z_THRUST_PER_M_PER_S * vz_filt)
        if thrust < T_MIN_U16:    thrust = T_MIN_U16
        if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16
        _rpyt(0.0, 0.0, 0.0, thrust)

        if (ti % 5) == 0:
            _j("measure_tick", {"t": ti, "n": n,
                                 "pnp": (px, py, pz) if n >= 4 else (-1, -1, -1)})

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    pnp_mean = (-1.0, -1.0, -1.0)
    if n_pnp_valid > 0:
        pnp_mean = (pnp_x_sum / n_pnp_valid,
                    pnp_y_sum / n_pnp_valid,
                    pnp_z_sum / n_pnp_valid)

    frac_ge4 = n_ge4 / n_total if n_total else 0.0
    frac_ge6 = n_ge6 / n_total if n_total else 0.0
    pass_ge4 = frac_ge4 >= PASS_FRAC_N_GE_4
    pass_full = (n_ge6 > 0) if REQUIRE_FULL_PAD_AT_LEAST_ONCE else True
    passed   = pass_ge4 and pass_full

    result = {
        "n_total":     n_total,
        "n_ge4":       n_ge4,
        "n_ge6":       n_ge6,
        "n_pnp_valid": n_pnp_valid,
        "frac_ge4":    frac_ge4,
        "frac_ge6":    frac_ge6,
        "n_histogram": n_histogram,
        "pnp_mean":    pnp_mean,
        "pnp_x_range": (pnp_x_min, pnp_x_max) if n_pnp_valid else (-1, -1),
        "pnp_y_range": (pnp_y_min, pnp_y_max) if n_pnp_valid else (-1, -1),
        "pnp_z_range": (pnp_z_min, pnp_z_max) if n_pnp_valid else (-1, -1),
        "pass_ge4":    pass_ge4,
        "pass_full":   pass_full,
        "pass":        passed,
    }
    _j("phase5_detect_window_done", result)
    return result, last_xyz_yaw


def _phase10_land():
    _j("phase10_land", "start")
    n_ticks = int(LAND_DUR * 1000 / TICK_MS)
    T_START = T_HOVER_NOMINAL - 2000
    T_END   = 0
    for ti in range(n_ticks):
        thrust = T_START - int((T_START - T_END) * ti / n_ticks)
        _rpyt(0.0, 0.0, 0.0, thrust)
        sentai.rtos.sleep_ms(TICK_MS)
    for _ in range(10):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)
    sentai.crazy.disarm()
    _j("phase10_land", "done")


# ============================ ENTRY ===================================

def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"wbs": "OP-S10-W21-T12-regression",
                          "iter": 1,
                          "n_markers_world": len(MARKER_WORLD)})

    summary = {
        "status":            "ERROR",
        "phase_reached":     -1,
        "ramp_handoff":      None,
        "pd_alt_stable":     False,
        "detect_window":     None,
        "pass":              False,
    }

    try:
        _phase1_setup()
        summary["phase_reached"] = 1

        _phase2_zero_unlock()
        summary["phase_reached"] = 2

        thrust, first_pnp = _phase3_thrust_ramp()
        summary["phase_reached"] = 3
        if first_pnp is None:
            summary["status"] = "RAMP_TIMEOUT"
            _write_summary(summary)
            _phase10_land()
            sentai.sim.journal_close()
            return summary
        summary["ramp_handoff"] = {"thrust": thrust, "pnp": first_pnp}

        pd_xyz_yaw, pd_thrust, pd_vz_filt = _phase4_pd_altitude_lock(first_pnp)
        summary["phase_reached"] = 4
        summary["pd_alt_stable"] = True

        detect, last_xyz_yaw = _phase5_detect_window(
            (pd_xyz_yaw, pd_thrust, pd_vz_filt))
        summary["phase_reached"] = 5
        summary["detect_window"] = detect
        summary["pass"] = detect["pass"]
        summary["status"] = "PASS" if detect["pass"] else "DETECT_FAIL"

        _phase10_land()
        summary["phase_reached"] = 10

    except (RuntimeError, OSError, AttributeError, TypeError, ValueError) as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = repr(e)[:200]
        _j("mission_exception", summary["exception"])
        try:
            _phase10_land()
        except (RuntimeError, OSError, AttributeError, TypeError, ValueError):
            pass

    _write_summary(summary)
    _j("mission_end", summary)
    sentai.sim.journal_close()
    return summary
