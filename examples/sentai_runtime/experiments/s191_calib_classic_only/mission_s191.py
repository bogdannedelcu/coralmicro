# mission_s191 — OP-S10-W21-T12 calib RPYT-only cascaded PD.
#
# Classic Commander (CRTP port 3 ch 0) throughout — NO HL handoff.
# Mission-side cascaded PD on X/Y/Z/Yaw using vision PnP feedback.
# YAW LOCK eliminates the helical drift observed in s190.
#
# Iter-1 scope: phases 1-5 only.  Phase 5 = PD navigate to (0,0,Z_HOLD)
# + hover for 5 s.  Pass criterion: no helical drift; |xyz_err|<5cm
# stable for 4 consecutive seconds.  Cross sweep + Kabsch + save in
# iter-2+ once PD nav is validated.
#
# WBS: OP-S10-W21-T12.  See README.md.

import sentai
import struct
import math


# ---- Names + paths ----------------------------------------------------
JOURNAL_NAME = "mission_s191_journal.txt"
SUMMARY_NAME = "mission_s191_summary.json"

# ---- WhyCon pad geometry (must match sentai_whycon_small.sdf) --------
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),
    (+0.08, +0.08, 0.005),
    (-0.06,  0.00, 0.005),
    (+0.06,  0.00, 0.005),
    (-0.08, -0.08, 0.005),
    (+0.08, -0.08, 0.005),
)

# ---- Camera intrinsics ----
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.0544

# ---- RPYT ramp + PD altitude params (REUSE from s190 iter-21) --------
T_BASE_U16       = 30000
T_HOVER_NOMINAL  = 36000
T_MIN_U16        = 22000
T_MAX_HOLD_U16   = 40000
T_MAX_U16        = 37000           # ramp cap
RAMP_S           = 6.0
TICK_MS          = 33              # ≈ 30 Hz
MAX_RAMP_S       = 12.0
N_STABLE_FRAMES  = 2

# Z PD gains (validated in s190 iter-21).
KP_Z_THRUST_PER_M       = 10000.0
KD_Z_THRUST_PER_M_PER_S = 8000.0

# ---- NEW T12: X/Y/Yaw PD gains — CONSERVATIVE (half of Z) ----
# Z's Kp_z=10k, Kd_z=8k worked with mass m=0.0282kg, hover thrust 36k.
# X/Y use ROLL/PITCH (degrees) instead of thrust units.  Different
# scale entirely.  Quadrotor: tilt α radians → horizontal accel ≈ g·sin(α).
# For 1 m/s² lateral acceleration: α ≈ 0.1 rad ≈ 5.8°.
# Outer loop: position error 0.1m → 1 m/s² accel → 5.8° tilt
# → Kp_xy = 58 deg/m.  Conservative half: Kp_xy = 30 deg/m.
# Kd_xy similar order — for 1 m/s velocity → counter with 5° tilt.
KP_XY_DEG_PER_M       = 10.0       # iter-4: lowered 30→10 (very gentle)
KD_XY_DEG_PER_M_PER_S = 10.0

# Iter-4 lesson: get_drone_pose_tuple yaw oscillates between mirror
# solutions across frames (e.g. 1.57 ↔ -1.71).  This breaks any
# body→world rotation that depends on yaw.  Lower Kp_yaw further +
# accept that yaw lock is weak.  Could swap to cf2's gyro-derived yaw
# (sentai.crazy.pose()[3]) later if PnP yaw stays unstable.
KP_YAW_RATE_PER_RAD   = 20.0

# Roll/pitch safety limits (degrees) — cap to prevent flips.
MAX_ROLL_DEG          = 15.0
MAX_PITCH_DEG         = 15.0
MAX_YAW_RATE_DEG      = 60.0

VZ_LPF_ALPHA          = 0.25
VXY_LPF_ALPHA         = 0.25

# Sample / settle thresholds.
SAMPLE_MAX_S     = 2.0
Z_STABLE_TOL_M   = 0.05
V_STABLE_TOL_M_S = 0.10

# ---- Mission envelope ----
Z_HOLD             = 0.78          # target hover altitude
NAV_TARGET_X       = 0.0
NAV_TARGET_Y       = 0.0
NAV_HOLD_S         = 5.0           # hold time at (0,0,Z_HOLD) after nav

# Pass criterion for iter-1: stable hold within tolerance for N seconds
HOLD_TOL_M         = 0.05
HOLD_STABLE_S_REQ  = 4.0

# Other consts copied from s190.
ZERO_UNLOCK_S      = 1.5
LAND_DUR           = 2.5


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


# ---- Raw CRTP helpers -------------------------------------------------

def _rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16):
    """Classic Commander setpoint (CRTP port 3 ch 0).  Bypasses HL/Generic
    entirely.  cf2 stabilizer rate-only attitude control + raw thrust."""
    # Clamp safety limits.
    if roll_deg  >  MAX_ROLL_DEG:  roll_deg  =  MAX_ROLL_DEG
    if roll_deg  < -MAX_ROLL_DEG:  roll_deg  = -MAX_ROLL_DEG
    if pitch_deg >  MAX_PITCH_DEG: pitch_deg =  MAX_PITCH_DEG
    if pitch_deg < -MAX_PITCH_DEG: pitch_deg = -MAX_PITCH_DEG
    if yaw_rate_deg_s >  MAX_YAW_RATE_DEG: yaw_rate_deg_s =  MAX_YAW_RATE_DEG
    if yaw_rate_deg_s < -MAX_YAW_RATE_DEG: yaw_rate_deg_s = -MAX_YAW_RATE_DEG
    sentai.crazy.send_crtp(3, 0,
        struct.pack('<fffH', roll_deg, pitch_deg, yaw_rate_deg_s,
                    thrust_u16))


def _extpos(x, y, z):
    """ExtPos observation to cf2 Kalman (port 6 ch 0).  Position-only
    (12 B), no quaternion — avoids yaw forcing per iter-13 crash."""
    sentai.crazy.send_crtp(6, 0, struct.pack('<fff', x, y, z))


# ---- PnP helper -------------------------------------------------------

def _try_pnp():
    """Returns (n, x, y, z, yaw_rad) or (n, -1,-1,-1, 0) on failure.
    NEW T12: also extract yaw from get_drone_pose_tuple for yaw lock."""
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

    x       = float(pose[0])
    y       = float(pose[1])
    z       = float(pose[2])
    yaw_rad = float(pose[3])
    if not (x == x and y == y and z == z and yaw_rad == yaw_rad):
        return n, -1.0, -1.0, -1.0, 0.0
    if z < 0.10:
        return n, -1.0, -1.0, -1.0, 0.0

    return n, x, y, z, yaw_rad


# ============================ PHASES =================================

def _phase1_setup():
    """Reuse from s190: markers + crazy + safety + FR + set_marker_world."""
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

    fr_frames = (
        "/home/bogdan/work/coralmicro/examples/sentai_runtime/"
        "experiments/s191_calib_classic_only/fr_current/frames")
    try:
        rc_fr = sentai.fr.open("frames", fr_frames)
        _j("fr_open_frames", {"rc": rc_fr, "path": fr_frames})
    except (AttributeError, RuntimeError):
        pass

    sentai.safety.init()
    sentai.crazy.init()

    _j("arm", "start")
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
    """Reuse from s190: RPYT(0,0,0,0) × 60 to unlock cf2 thrust."""
    _j("phase2_zero_unlock", {"dur_s": ZERO_UNLOCK_S})
    n_ticks = int(ZERO_UNLOCK_S * 1000 / TICK_MS)
    for _ in range(n_ticks):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)
    _j("phase2_zero_unlock", "done")


def _phase3_thrust_ramp():
    """Reuse from s190: open-loop thrust ramp until first valid PnP."""
    _j("phase3_thrust_ramp", {
        "T_BASE": T_BASE_U16, "T_MAX": T_MAX_U16, "RAMP_S": RAMP_S,
        "MAX_RAMP_S": MAX_RAMP_S, "N_STABLE": N_STABLE_FRAMES,
    })

    n_ticks_ramp = int(RAMP_S * 1000 / TICK_MS)
    max_ticks    = int(MAX_RAMP_S * 1000 / TICK_MS)
    stable_streak = 0
    last_pnp = None

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
    """Reuse from s190 iter-21: PD altitude controller, vz-only stable.
    Returns (last_xyz_yaw, last_thrust, vz_filt).  Caller continues PD
    in the navigation phase."""
    z_target = first_pnp[2]
    z_prev   = first_pnp[2]
    vz_filt  = 0.0
    last_xyz_yaw = first_pnp
    stable_count = 0
    max_ticks = int(SAMPLE_MAX_S * 1000 / TICK_MS)

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
                                       "final_thrust": thrust,
                                       "final_vz_filt": vz_filt})
                return last_xyz_yaw, thrust, vz_filt
        else:
            stable_count = 0

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase4_pd_alt", {"stable": False, "timed_out": True,
                           "final_xyz_yaw": last_xyz_yaw})
    return last_xyz_yaw, thrust, vz_filt


def _phase5_pd_navigate(pd_state):
    """NEW T12: Cascaded PD on X/Y/Z/Yaw to navigate to (0,0,Z_HOLD)
    and HOLD there for NAV_HOLD_S.  Classic Commander RPYT only.
    Pass: |xyz_err| < HOLD_TOL_M stable for HOLD_STABLE_S_REQ seconds."""
    pd_xyz_yaw, pd_thrust, pd_vz_filt = pd_state
    target_x = NAV_TARGET_X
    target_y = NAV_TARGET_Y
    target_z = Z_HOLD
    # Iter-2: lock yaw to INITIAL value (frame convention offset).
    target_yaw = pd_xyz_yaw[3]

    _j("phase5_pd_navigate", {
        "target": (target_x, target_y, target_z),
        "target_yaw": target_yaw,
        "Kp_xy": KP_XY_DEG_PER_M, "Kd_xy": KD_XY_DEG_PER_M_PER_S,
        "Kp_yaw": KP_YAW_RATE_PER_RAD,
        "start_xyz_yaw": pd_xyz_yaw,
    })

    x_prev    = pd_xyz_yaw[0]
    y_prev    = pd_xyz_yaw[1]
    z_prev    = pd_xyz_yaw[2]
    vx_filt   = 0.0
    vy_filt   = 0.0
    vz_filt   = pd_vz_filt
    last_xyz_yaw = pd_xyz_yaw

    stable_acc_s = 0.0
    NAV_MAX_S = NAV_HOLD_S + 4.0       # 4s to converge + hold time
    max_ticks = int(NAV_MAX_S * 1000 / TICK_MS)

    for ti in range(max_ticks):
        n, px, py, pz, pyaw = _try_pnp()
        if n >= 4 and pz > 0.0:
            x_now = px
            y_now = py
            z_now = pz
            yaw_now = pyaw
            last_xyz_yaw = (px, py, pz, pyaw)
        else:
            # Use last-known if no PnP this frame.
            x_now = x_prev
            y_now = y_prev
            z_now = z_prev
            yaw_now = last_xyz_yaw[3]

        dt_s = TICK_MS / 1000.0
        vx_raw  = (x_now - x_prev) / dt_s
        vy_raw  = (y_now - y_prev) / dt_s
        vz_raw  = (z_now - z_prev) / dt_s
        vx_filt = VXY_LPF_ALPHA * vx_raw + (1.0 - VXY_LPF_ALPHA) * vx_filt
        vy_filt = VXY_LPF_ALPHA * vy_raw + (1.0 - VXY_LPF_ALPHA) * vy_filt
        vz_filt = VZ_LPF_ALPHA  * vz_raw + (1.0 - VZ_LPF_ALPHA)  * vz_filt

        # Z PD → thrust (same as phase 4).
        err_z  = target_z - z_now
        thrust = T_HOVER_NOMINAL + int(
            KP_Z_THRUST_PER_M * err_z - KD_Z_THRUST_PER_M_PER_S * vz_filt)
        if thrust < T_MIN_U16:    thrust = T_MIN_U16
        if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16

        # Iter-6: XY PD DISABLED.  We DON'T YET know the pitch/roll →
        # X/Y mapping (axes may be swapped, signs unknown).  Just hold
        # roll=pitch=0 and let drone drift naturally.  Z PD keeps
        # altitude.  Use this as baseline to measure natural drift,
        # then iter-7 will add axis identification via test pulses.
        # Operator-stated: "calibrarea trebuie sa determine axele, nu
        # hardcoda — vreau sa ne dam seama chiar in zbor daca axa X e
        # cumva inversata cu Y".
        err_x = target_x - x_now
        err_y = target_y - y_now
        roll_deg  = 0.0
        pitch_deg = 0.0

        # Yaw control DISABLED (iter-5).  Iter-4 showed PnP yaw flips
        # between mirror solutions across frames (1.57 ↔ -1.57) due to
        # Kabsch+yaw-anchor picker selecting different branches when
        # PnP is noisy.  Trying to lock to a flipping target produces
        # saturated yaw_rate=±60°/s commands → drone genuinely spins.
        # cf2 stabilizer's gyro-yaw drift is slow (~1°/s); for 10s
        # calibration that's 10° drift — acceptable.  yaw_rate=0 lets
        # cf2 do its job.  Future fix: use cf2 gyro yaw (sentai.crazy.
        # pose()[3]) which is mirror-free, OR add hysteresis on PnP
        # yaw to reject mirror flips.
        yaw_rate_deg = 0.0

        _rpyt(roll_deg, pitch_deg, yaw_rate_deg, thrust)

        # Stability check: all 3 axes within HOLD_TOL_M.
        err_norm = math.sqrt(err_x*err_x + err_y*err_y + err_z*err_z)
        if (err_norm < HOLD_TOL_M and n >= 4 and pz > 0.0):
            stable_acc_s += dt_s
        else:
            stable_acc_s = 0.0

        if (ti % 3) == 0:
            _j("nav_tick", {
                "t": ti, "n": n,
                "xyz": (x_now, y_now, z_now), "yaw": yaw_now,
                "err": (err_x, err_y, err_z),
                "vxyz": (vx_filt, vy_filt, vz_filt),
                "cmd": (roll_deg, pitch_deg, yaw_rate_deg, thrust),
                "stable_s": stable_acc_s,
            })

        # Also stream ExtPos to cf2 EKF (keeps cf2 Kalman anchored —
        # may help with attitude estimate if cf2 uses pos for some
        # internal logic).
        if n >= 4 and pz > 0.0:
            _extpos(px, py, pz)

        if stable_acc_s >= HOLD_STABLE_S_REQ:
            _j("phase5_pd_navigate", {
                "stable": True, "stable_s": stable_acc_s,
                "final_xyz_yaw": last_xyz_yaw,
                "final_thrust": thrust,
            })
            return last_xyz_yaw, thrust, vz_filt, True

        x_prev = x_now
        y_prev = y_now
        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase5_pd_navigate", {
        "stable": False, "timed_out": True,
        "final_xyz_yaw": last_xyz_yaw,
    })
    return last_xyz_yaw, thrust, vz_filt, False


def _phase10_land():
    """RPYT thrust ramp-down — no HL, no Generic Commander.
    Gradually reduce thrust from T_HOVER to 0 over 2 s, then disarm."""
    _j("phase10_land", "start")
    n_ticks = int(LAND_DUR * 1000 / TICK_MS)
    T_START = T_HOVER_NOMINAL - 2000   # below hover so drone descends
    T_END   = 0
    for ti in range(n_ticks):
        thrust = T_START - int((T_START - T_END) * ti / n_ticks)
        _rpyt(0.0, 0.0, 0.0, thrust)
        sentai.rtos.sleep_ms(TICK_MS)
    # A few final zero-thrust packets.
    for _ in range(10):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)
    sentai.crazy.disarm()
    _j("phase10_land", "done")


# ============================ ENTRY ===================================

def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"wbs": "OP-S10-W21-T12", "iter": 1})

    summary = {
        "status":         "ERROR",
        "phase_reached":  -1,
        "handoff_thrust": 0,
        "handoff_xyz_yaw": None,
        "nav_stable":     False,
        "nav_stable_s":   0.0,
        "nav_final_xyz_yaw": None,
    }

    try:
        _phase1_setup()
        summary["phase_reached"] = 1

        _phase2_zero_unlock()
        summary["phase_reached"] = 2

        locked_thrust, first_pnp = _phase3_thrust_ramp()
        if locked_thrust is None:
            summary["status"] = "RAMP_TIMEOUT"
            for _ in range(10):
                _rpyt(0.0, 0.0, 0.0, 0)
                sentai.rtos.sleep_ms(TICK_MS)
            sentai.crazy.disarm()
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary
        summary["phase_reached"] = 3
        summary["handoff_thrust"] = locked_thrust
        summary["handoff_xyz_yaw"] = first_pnp

        pd_xyz_yaw, pd_thrust, pd_vz_filt = _phase4_pd_altitude_lock(first_pnp)
        summary["phase_reached"] = 4

        nav_result = _phase5_pd_navigate(
            (pd_xyz_yaw, pd_thrust, pd_vz_filt))
        last_xyz_yaw, last_thrust, last_vz, nav_stable = nav_result
        summary["phase_reached"]      = 5
        summary["nav_stable"]         = nav_stable
        summary["nav_final_xyz_yaw"]  = last_xyz_yaw

        if nav_stable:
            summary["status"] = "PASS"
        else:
            summary["status"] = "NAV_TIMEOUT"

        _phase10_land()
    except Exception as e:
        _j("mission_exception", {"err": str(e)})
        summary["status"] = "EXCEPTION"
        try:
            for _ in range(10):
                _rpyt(0.0, 0.0, 0.0, 0)
                sentai.rtos.sleep_ms(TICK_MS)
            sentai.crazy.disarm()
        except Exception:
            pass

    _j("mission_done", {"status": summary["status"]})
    _write_summary(summary)
    sentai.sim.journal_close()
    return summary
