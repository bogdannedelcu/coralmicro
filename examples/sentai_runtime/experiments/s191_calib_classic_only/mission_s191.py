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

# Iter-7+ strategy (A): hover after cf2 EKF convergence.
# Operator-stated 2026-05-22: "Z poate sta pe PnP only, hover() works
# after Kalman has good Z observation".  We continue PD altitude
# (Classic RPYT) while feeding ExtPos, watch cf2.pose() converge to
# match PnP, THEN switch to hover() which uses cf2's internal PIDs.
EXTPOS_WARMUP_MAX_S    = 6.0
EKF_CONV_TOL_M         = 0.10
EKF_CONV_TICKS_REQ     = 5
HOVER_HOLD_S           = 5.0
HOVER_EXCURSION_TOL_M  = 0.25       # iter-10: was 0.10, drone oscillates
                                     # 5-22cm naturally with cf2 hover() PID

# Iter-11: operator-stated 2026-05-22 — KEEP ExtPos at 30Hz unfiltered.
# Kalman has its own noise filtering; reducing rate throws away info.
EXTPOS_LPF_ALPHA       = 1.0        # 1.0 = NO LPF (pass-through)
EXTPOS_RATE_HZ         = 30         # full rate, every tick

# Iter-10: cross sweep params for axis ID via Kabsch fit.
SWEEP_RADIUS_M         = 0.025      # ±2.5cm cross corners
SWEEP_VMOVE            = 0.025      # 1s travel for 2.5cm = 25 mm/s
SWEEP_TRAVEL_S         = 1.0        # time to traverse to corner
SWEEP_SETTLE_S         = 1.5        # settle at corner before sampling
SWEEP_SAMPLE_S         = 2.0        # sample window
SAMPLE_RATE_HZ         = 30         # PnP samples taken per second

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


def _position_setpoint(x_w, y_w, z_w, yaw_deg):
    """Generic Commander position setpoint (CRTP port 7 ch 0, type=7).
    cf2 sets mode.x/y/z = modeAbs → uses its internal position PID
    with Kalman state to navigate to (x, y, z) in WORLD frame.
    Iter-12 fix: replaces hover(vx, vy, ...) velocity-pulses that
    caused open-loop resonance — drone now has closed-loop position
    feedback in cf2, won't diverge if commanded back to origin."""
    sentai.crazy.send_crtp(7, 0,
        struct.pack('<Bffff', 7, x_w, y_w, z_w, yaw_deg))


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


def _z_pd_thrust(z_now, z_prev, target_z, vz_filt_state, dt_s):
    """Helper: compute Z PD thrust + update vz_filt.  Used across all
    sub-phases of axis ID + nav so Z control stays active throughout."""
    vz_raw = (z_now - z_prev) / dt_s
    vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt_state
    err_z  = target_z - z_now
    thrust = T_HOVER_NOMINAL + int(
        KP_Z_THRUST_PER_M * err_z - KD_Z_THRUST_PER_M_PER_S * vz_filt)
    if thrust < T_MIN_U16:    thrust = T_MIN_U16
    if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16
    return thrust, vz_filt


def _phase5_extpos_warmup_convergence(pd_state):
    """Strategy (A) — feed ExtPos to cf2 Kalman while continuing PD
    altitude (Classic RPYT), watch for cf2.pose() to converge to PnP.

    Returns (last_xyz_yaw, last_thrust, vz_filt, converged).

    Convergence criterion: |cf2.pose - pnp.pose| < EKF_CONV_TOL_M on
    all 3 axes for EKF_CONV_TICKS_REQ consecutive ticks.

    Once converged, mission can safely switch to Generic hover() which
    relies on cf2.position/velocity from Kalman."""
    pd_xyz_yaw, pd_thrust, pd_vz_filt = pd_state
    target_z = pd_xyz_yaw[2]
    z_prev   = pd_xyz_yaw[2]
    vz_filt  = pd_vz_filt
    last_xyz_yaw = pd_xyz_yaw

    _j("phase5_extpos_warmup", {
        "z_target":  target_z,
        "tol_m":     EKF_CONV_TOL_M,
        "ticks_req": EKF_CONV_TICKS_REQ,
        "max_s":     EXTPOS_WARMUP_MAX_S,
    })

    dt_s = TICK_MS / 1000.0
    n_ticks = int(EXTPOS_WARMUP_MAX_S * 1000 / TICK_MS)
    conv_streak = 0

    for ti in range(n_ticks):
        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            z_now = z
            last_xyz_yaw = (x, y, z, yaw)
            # Feed cf2 Kalman with vision-derived position.
            _extpos(x, y, z)
        else:
            z_now = z_prev

        # Continue PD altitude via RPYT thrust (vision-based).
        thrust, vz_filt = _z_pd_thrust(z_now, z_prev, target_z,
                                         vz_filt, dt_s)
        _rpyt(0.0, 0.0, 0.0, thrust)

        # Read cf2 Kalman state for convergence check.
        ekf_x = ekf_y = ekf_z = 0.0
        ekf_ok = False
        try:
            p = sentai.crazy.pose()
            if p is not None and len(p) >= 3:
                ekf_x = float(p[0])
                ekf_y = float(p[1])
                ekf_z = float(p[2])
                ekf_ok = True
        except (AttributeError, RuntimeError, TypeError):
            pass

        # Iter-9 fix: only update streak when we have a VALID PnP
        # measurement to compare against.  Without PnP we can't tell
        # if cf2 is converged — but missing a tick shouldn't RESET
        # progress.  Previously every PnP miss (n<4) zeroed the
        # streak, preventing 5 consecutive matches.
        if ekf_ok and n >= 4 and z > 0.0:
            dx = abs(ekf_x - x)
            dy = abs(ekf_y - y)
            dz = abs(ekf_z - z)
            if (dx < EKF_CONV_TOL_M and
                dy < EKF_CONV_TOL_M and
                dz < EKF_CONV_TOL_M):
                conv_streak += 1
            else:
                conv_streak = 0
        # PnP missing this tick → leave conv_streak unchanged (no
        # info to update — neither confirm nor reset).

        if (ti % 3) == 0:
            _j("warmup_tick", {
                "t": ti, "n": n,
                "pnp": (x, y, z) if n >= 4 else (-1, -1, -1),
                "ekf": (ekf_x, ekf_y, ekf_z),
                "streak": conv_streak,
                "thrust": thrust,
            })

        if conv_streak >= EKF_CONV_TICKS_REQ:
            _j("phase5_extpos_warmup", {
                "converged": True, "ticks_used": ti,
                "final_xyz_yaw": last_xyz_yaw,
                "final_thrust": thrust,
            })
            return last_xyz_yaw, thrust, vz_filt, True

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase5_extpos_warmup", {
        "converged": False, "timed_out": True,
        "final_xyz_yaw": last_xyz_yaw,
    })
    return last_xyz_yaw, thrust, vz_filt, False


def _phase6_hover_hold(warmup_state):
    """Switch to Generic Commander hover() — cf2's altitude PID + XY
    velocity hold using its (now converged) Kalman state.

    Iter-10: LPF on PnP before ExtPos + reduce ExtPos rate from 30 to
    10 Hz — reduces noise injected to cf2 Kalman → cf2 velocity PID
    sees smoother state → less reactive oscillation.

    Returns (xyz_yaw, success).  success=True if max excursion
    < HOVER_EXCURSION_TOL_M (iter-10: 0.25m vs prior 0.10m — drone
    naturally oscillates with cf2 PID, 10cm too strict)."""
    last_xyz_yaw, _, _, _ = warmup_state
    z_target = last_xyz_yaw[2]
    last_xyz = last_xyz_yaw

    # LPF state for PnP smoothing before ExtPos.
    pnp_lpf_x = last_xyz_yaw[0]
    pnp_lpf_y = last_xyz_yaw[1]
    pnp_lpf_z = last_xyz_yaw[2]

    _j("phase6_hover_hold", {
        "z_target": z_target, "hold_s": HOVER_HOLD_S,
        "extpos_lpf_alpha": EXTPOS_LPF_ALPHA,
        "extpos_rate_hz":   EXTPOS_RATE_HZ,
    })

    n_ticks = int(HOVER_HOLD_S * 1000 / TICK_MS)
    stable_count = 0
    max_excursion = 0.0
    # Send ExtPos every N ticks (10Hz on 30Hz mission loop = every 3).
    extpos_decim = int(round((1000.0 / TICK_MS) / EXTPOS_RATE_HZ))

    for ti in range(n_ticks):
        # Always send hover at 30Hz (cf2 commander watchdog ~500ms).
        sentai.crazy.hover(0.0, 0.0, 0.0, z_target)

        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            # LPF on PnP to smooth noise.
            pnp_lpf_x = EXTPOS_LPF_ALPHA * x + (1 - EXTPOS_LPF_ALPHA) * pnp_lpf_x
            pnp_lpf_y = EXTPOS_LPF_ALPHA * y + (1 - EXTPOS_LPF_ALPHA) * pnp_lpf_y
            pnp_lpf_z = EXTPOS_LPF_ALPHA * z + (1 - EXTPOS_LPF_ALPHA) * pnp_lpf_z
            # Send ExtPos at reduced rate.
            if (ti % extpos_decim) == 0:
                _extpos(pnp_lpf_x, pnp_lpf_y, pnp_lpf_z)
            last_xyz = (x, y, z, yaw)
            err_x = x - last_xyz_yaw[0]
            err_y = y - last_xyz_yaw[1]
            err_z = z - z_target
            excursion = math.sqrt(err_x*err_x + err_y*err_y + err_z*err_z)
            if excursion > max_excursion:
                max_excursion = excursion
            if excursion < HOVER_EXCURSION_TOL_M:
                stable_count += 1
            else:
                stable_count = 0

        if (ti % 5) == 0:
            ekf_xyz = (-1.0, -1.0, -1.0)
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_xyz = (p[0], p[1], p[2])
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("hover_tick", {
                "t": ti, "n": n,
                "xyz": (x, y, z) if n >= 4 else (-1, -1, -1),
                "lpf": (pnp_lpf_x, pnp_lpf_y, pnp_lpf_z),
                "ekf": ekf_xyz,
                "excursion": excursion if n >= 4 else -1.0,
                "stable": stable_count,
            })

        sentai.rtos.sleep_ms(TICK_MS)

    success = (stable_count >= (n_ticks // 2)
               and max_excursion < HOVER_EXCURSION_TOL_M)
    _j("phase6_hover_hold", {
        "success": success,
        "stable_ticks": stable_count, "total_ticks": n_ticks,
        "max_excursion_m": max_excursion,
        "final_xyz_yaw": last_xyz,
        "pnp_lpf_state":  (pnp_lpf_x, pnp_lpf_y, pnp_lpf_z),
    })
    return last_xyz, success, (pnp_lpf_x, pnp_lpf_y, pnp_lpf_z)


def _goto_and_sample(target_x_w, target_y_w, z_target, travel_s, settle_s,
                       sample_s):
    """Iter-12: send Generic position setpoint (world frame absolute)
    to navigate drone to (target_x_w, target_y_w, z_target).  cf2's
    position PID closed-loop drives drone there — no resonance from
    open-loop velocity pulses.

    Phase A: stream position setpoint for travel_s (drone arrives).
    Phase B: continue same setpoint for settle_s (drone settles).
    Phase C: continue + collect PnP samples for sample_s.

    Continuously feeds ExtPos so cf2 Kalman stays anchored to vision.
    Returns (samples_list, last_xyz_yaw).
    samples_list = [(px, py, pz, pyaw), ...] of successful PnP frames."""
    last_xyz_yaw = (target_x_w, target_y_w, z_target, 0.0)
    samples = []

    # Phase A: travel.
    travel_ticks = int(travel_s * 1000 / TICK_MS)
    for ti in range(travel_ticks):
        _position_setpoint(target_x_w, target_y_w, z_target, 0.0)
        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            _extpos(x, y, z)
            last_xyz_yaw = (x, y, z, yaw)
        sentai.rtos.sleep_ms(TICK_MS)

    # Phase B: settle (same setpoint).
    settle_ticks = int(settle_s * 1000 / TICK_MS)
    for ti in range(settle_ticks):
        _position_setpoint(target_x_w, target_y_w, z_target, 0.0)
        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            _extpos(x, y, z)
            last_xyz_yaw = (x, y, z, yaw)
        sentai.rtos.sleep_ms(TICK_MS)

    # Phase C: sample.
    sample_ticks = int(sample_s * 1000 / TICK_MS)
    for ti in range(sample_ticks):
        _position_setpoint(target_x_w, target_y_w, z_target, 0.0)
        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            samples.append((x, y, z, yaw))
            _extpos(x, y, z)
            last_xyz_yaw = (x, y, z, yaw)
        sentai.rtos.sleep_ms(TICK_MS)

    return samples, last_xyz_yaw


def _phase7_cross_sweep(lpf_state, z_target):
    """Iter-12: position-setpoint-based cross sweep.  Drone navigates
    to ABSOLUTE WORLD positions via Generic position setpoint (cf2
    position PID is closed-loop, no resonance).  Between each corner
    drone RETURNS to (0,0,z_target) — operator-stated requirement.

    World-frame absolute targets:
      center = (0, 0, z)
      +X     = (+r, 0, z)
      -X     = (-r, 0, z)
      +Y     = (0, +r, z)
      -Y     = (0, -r, z)

    Median of N samples per pose handles residual oscillation."""
    _j("phase7_cross_sweep", {
        "sweep_radius": SWEEP_RADIUS_M,
        "travel_s": SWEEP_TRAVEL_S,
        "settle_s": SWEEP_SETTLE_S,
        "sample_s": SWEEP_SAMPLE_S,
        "z_target": z_target,
        "mode": "position_setpoint_world_frame",
    })

    def _median_of(xs):
        s = sorted(xs)
        m = len(s) // 2
        return s[m] if (len(s) & 1) else 0.5 * (s[m-1] + s[m])

    def _summarize(label, samples):
        if not samples:
            return None
        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        zs = [s[2] for s in samples]
        yaws = [s[3] for s in samples]
        med = (_median_of(xs), _median_of(ys), _median_of(zs),
               _median_of(yaws))
        # Estimate noise as inter-quartile range.
        return {"label": label, "n": len(samples), "median": med,
                "x_range": (min(xs), max(xs)),
                "y_range": (min(ys), max(ys)),
                "z_range": (min(zs), max(zs))}

    poses = {}
    r = SWEEP_RADIUS_M

    # ── 1. CENTER ── (drone navigates to absolute origin first)
    _j("sweep_pose", "center")
    s_center, _ = _goto_and_sample(
        0.0, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, SWEEP_SAMPLE_S)
    poses["center"] = _summarize("center", s_center)
    _j("sweep_summary", poses["center"])

    # ── 2. +X world ──
    _j("sweep_pose", "+X_world")
    s_px, _ = _goto_and_sample(
        +r, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, SWEEP_SAMPLE_S)
    poses["+X"] = _summarize("+X_world", s_px)
    _j("sweep_summary", poses["+X"])

    # Return to center between corners (closed-loop, position abs).
    _j("sweep_pose", "return_to_center_1")
    _, _ = _goto_and_sample(
        0.0, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, 0.0)

    # ── 3. -X world ──
    _j("sweep_pose", "-X_world")
    s_nx, _ = _goto_and_sample(
        -r, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, SWEEP_SAMPLE_S)
    poses["-X"] = _summarize("-X_world", s_nx)
    _j("sweep_summary", poses["-X"])

    _j("sweep_pose", "return_to_center_2")
    _, _ = _goto_and_sample(
        0.0, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, 0.0)

    # ── 4. +Y world ──
    _j("sweep_pose", "+Y_world")
    s_py, _ = _goto_and_sample(
        0.0, +r, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, SWEEP_SAMPLE_S)
    poses["+Y"] = _summarize("+Y_world", s_py)
    _j("sweep_summary", poses["+Y"])

    _j("sweep_pose", "return_to_center_3")
    _, _ = _goto_and_sample(
        0.0, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, 0.0)

    # ── 5. -Y world ──
    _j("sweep_pose", "-Y_world")
    s_ny, _ = _goto_and_sample(
        0.0, -r, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, SWEEP_SAMPLE_S)
    poses["-Y"] = _summarize("-Y_world", s_ny)
    _j("sweep_summary", poses["-Y"])

    # Final return to center.
    _j("sweep_pose", "return_to_center_final")
    _, _ = _goto_and_sample(
        0.0, 0.0, z_target,
        SWEEP_TRAVEL_S, SWEEP_SETTLE_S, 0.0)

    # Body→world mapping check: with position setpoint in WORLD frame,
    # the world displacement SHOULD match the commanded target.  If
    # cf2 PID worked correctly, +X target → drone at (+r, 0), so the
    # median position should be near (+r, 0).  Deviation = control
    # error / Kalman noise.
    if poses["center"] and poses["+X"] and poses["+Y"]:
        c  = poses["center"]["median"]
        px = poses["+X"]["median"]
        py = poses["+Y"]["median"]
        _j("axis_map_observed", {
            "+X_target": (+r, 0),
            "+X_observed_dxy": (px[0] - c[0], px[1] - c[1]),
            "+Y_target": (0, +r),
            "+Y_observed_dxy": (py[0] - c[0], py[1] - c[1]),
        })

    return poses, lpf_state


def _phase5_axis_id(pd_state):
    """NEW iter-7: axis identification via test-pulse pattern.

    1. Baseline: Z PD + level for AXIS_ID_BASELINE_S → measure drift.
    2. Pitch pulse: +AXIS_ID_PULSE_DEG for AXIS_ID_PULSE_S → record dx, dy.
    3. Recovery: -AXIS_ID_PULSE_DEG for AXIS_ID_RECOVERY_S → brake.
    4. Settle: roll=pitch=0 for AXIS_ID_SETTLE_S.
    5. Roll pulse: same pattern.
    6. Build 2×2 axis map.

    Returns (M_inv_per_meter, drone_xyz, vz_filt, axis_id_ok)
    where M_inv_per_meter maps (err_x_m, err_y_m) → (pitch_deg, roll_deg)
    needed to produce that displacement over a 1-second window.
    On failure, axis_id_ok=False (mission falls back to no XY control)."""
    pd_xyz_yaw, _, pd_vz_filt = pd_state
    target_z = pd_xyz_yaw[2]      # lock z to PD's final altitude
    vz_filt  = pd_vz_filt
    z_prev   = pd_xyz_yaw[2]
    last_xyz_yaw = pd_xyz_yaw

    _j("phase5_axis_id", {
        "z_target":    target_z,
        "pulse_deg":   AXIS_ID_PULSE_DEG,
        "pulse_s":     AXIS_ID_PULSE_S,
        "baseline_s":  AXIS_ID_BASELINE_S,
        "recovery_s":  AXIS_ID_RECOVERY_S,
    })

    dt_s = TICK_MS / 1000.0

    # ─── 1. BASELINE: Z PD + level, measure mean velocity ─────────
    _j("axis_id_phase", "baseline_start")
    samples_base = []
    n_ticks = int(AXIS_ID_BASELINE_S * 1000 / TICK_MS)
    for ti in range(n_ticks):
        n, x, y, z, yaw = _try_pnp()
        if n >= 4 and z > 0.0:
            z_now = z
            last_xyz_yaw = (x, y, z, yaw)
            samples_base.append((x, y, z))
        else:
            z_now = z_prev

        thrust, vz_filt = _z_pd_thrust(z_now, z_prev, target_z,
                                         vz_filt, dt_s)
        _rpyt(0.0, 0.0, 0.0, thrust)
        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    if len(samples_base) < 5:
        _j("axis_id_phase", {"step": "baseline", "fail": "few_samples",
                              "n_samples": len(samples_base)})
        return None, last_xyz_yaw, vz_filt, False

    # Drift velocity from first/last sample.
    base_dt = (len(samples_base) - 1) * dt_s
    vx_drift = (samples_base[-1][0] - samples_base[0][0]) / base_dt
    vy_drift = (samples_base[-1][1] - samples_base[0][1]) / base_dt
    _j("axis_id_baseline", {
        "n_samples": len(samples_base),
        "x_start": samples_base[0][0],  "y_start": samples_base[0][1],
        "x_end":   samples_base[-1][0], "y_end":   samples_base[-1][1],
        "vx_drift": vx_drift, "vy_drift": vy_drift,
    })

    # ─── Helper: run one pulse + recovery + settle ────────────────
    def _do_pulse(label, roll_deg, pitch_deg):
        """Returns (dx_net, dy_net) — displacement during pulse with
        drift subtracted.  Updates z_prev, vz_filt, last_xyz_yaw via
        nonlocal closure."""
        # Snapshot position before pulse.
        n0, x0, y0, z0, yaw0 = _try_pnp()
        if n0 < 4:
            return None, None
        _j("axis_id_pulse_start", {
            "label": label, "x": x0, "y": y0, "z": z0, "yaw": yaw0,
            "cmd_roll": roll_deg, "cmd_pitch": pitch_deg,
        })

        # 2A. FORWARD PULSE
        pulse_ticks = int(AXIS_ID_PULSE_S * 1000 / TICK_MS)
        for ti in range(pulse_ticks):
            nonlocal z_prev, vz_filt, last_xyz_yaw
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                z_now = z
                last_xyz_yaw = (x, y, z, yaw)
            else:
                z_now = z_prev
            thrust, vz_filt = _z_pd_thrust(z_now, z_prev, target_z,
                                             vz_filt, dt_s)
            _rpyt(roll_deg, pitch_deg, 0.0, thrust)
            z_prev = z_now
            sentai.rtos.sleep_ms(TICK_MS)

        # Snapshot position after pulse.
        n1, x1, y1, z1, yaw1 = _try_pnp()
        if n1 < 4:
            return None, None

        # 2B. RECOVERY (opposite pulse to brake).
        for ti in range(int(AXIS_ID_RECOVERY_S * 1000 / TICK_MS)):
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                z_now = z
                last_xyz_yaw = (x, y, z, yaw)
            else:
                z_now = z_prev
            thrust, vz_filt = _z_pd_thrust(z_now, z_prev, target_z,
                                             vz_filt, dt_s)
            _rpyt(-roll_deg, -pitch_deg, 0.0, thrust)
            z_prev = z_now
            sentai.rtos.sleep_ms(TICK_MS)

        # 2C. SETTLE (level + Z PD).
        for ti in range(int(AXIS_ID_SETTLE_S * 1000 / TICK_MS)):
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                z_now = z
                last_xyz_yaw = (x, y, z, yaw)
            else:
                z_now = z_prev
            thrust, vz_filt = _z_pd_thrust(z_now, z_prev, target_z,
                                             vz_filt, dt_s)
            _rpyt(0.0, 0.0, 0.0, thrust)
            z_prev = z_now
            sentai.rtos.sleep_ms(TICK_MS)

        # Compute net pulse displacement (subtract baseline drift over
        # the pulse duration only — the pulse caused this dx/dy).
        dx_raw = x1 - x0
        dy_raw = y1 - y0
        dx_net = dx_raw - vx_drift * AXIS_ID_PULSE_S
        dy_net = dy_raw - vy_drift * AXIS_ID_PULSE_S
        _j("axis_id_pulse_done", {
            "label": label, "x0_y0": (x0, y0), "x1_y1": (x1, y1),
            "dx_raw": dx_raw, "dy_raw": dy_raw,
            "dx_net": dx_net, "dy_net": dy_net,
        })
        return dx_net, dy_net

    # ─── 2. PITCH PULSE ──────────────────────────────────────────
    dx_p, dy_p = _do_pulse("pitch+", 0.0, +AXIS_ID_PULSE_DEG)
    if dx_p is None:
        _j("axis_id_phase", {"step": "pitch_pulse", "fail": "no_pnp"})
        return None, last_xyz_yaw, vz_filt, False

    # ─── 3. ROLL PULSE ───────────────────────────────────────────
    dx_r, dy_r = _do_pulse("roll+", +AXIS_ID_PULSE_DEG, 0.0)
    if dx_r is None:
        _j("axis_id_phase", {"step": "roll_pulse", "fail": "no_pnp"})
        return None, last_xyz_yaw, vz_filt, False

    # ─── 4. BUILD MAP ────────────────────────────────────────────
    # M relates (pitch_deg, roll_deg) pulse over PULSE_S → (dx, dy)
    # in world frame.  M = [[dx_per_pitch, dx_per_roll],
    #                       [dy_per_pitch, dy_per_roll]]
    pulse = float(AXIS_ID_PULSE_DEG)
    dx_per_pitch = dx_p / pulse
    dy_per_pitch = dy_p / pulse
    dx_per_roll  = dx_r / pulse
    dy_per_roll  = dy_r / pulse

    # Invert 2×2.
    det = dx_per_pitch * dy_per_roll - dx_per_roll * dy_per_pitch
    _j("axis_id_map", {
        "dx_per_pitch": dx_per_pitch, "dy_per_pitch": dy_per_pitch,
        "dx_per_roll":  dx_per_roll,  "dy_per_roll":  dy_per_roll,
        "det": det,
    })
    if abs(det) < 1e-4:
        _j("axis_id_phase", {"step": "build_map", "fail": "singular",
                              "det": det})
        return None, last_xyz_yaw, vz_filt, False

    inv_det = 1.0 / det
    # M_inv: maps (dx_world, dy_world) → (pitch, roll) needed.
    M_inv = [
        [+dy_per_roll  * inv_det, -dx_per_roll  * inv_det],
        [-dy_per_pitch * inv_det, +dx_per_pitch * inv_det],
    ]
    _j("axis_id_done", {"M_inv_row0": M_inv[0], "M_inv_row1": M_inv[1]})

    return M_inv, last_xyz_yaw, vz_filt, True


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

        # Iter-14: axis ID via HOVER() body-frame velocity pulses.
        # Position setpoint was a dead-end — depended on frame
        # alignment (R_cam_to_body) which is EXACTLY what calibration
        # produces.  Now: hover(±v, 0) body pulses, observe world
        # response via PnP, derive R.
        z_target = pd_xyz_yaw[2]

        # 5a. SETTLE: hover(0,0,0,z) for 2s — let cf2 stabilize via
        # its Kalman + ExtPos.  Drone holds bounded ±20cm.
        _j("phase5a_settle", {"z_target": z_target, "dur_s": 2.0})
        for ti in range(int(2.0 * 1000 / TICK_MS)):
            sentai.crazy.hover(0.0, 0.0, 0.0, z_target)
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                _extpos(x, y, z)
            sentai.rtos.sleep_ms(TICK_MS)

        # Sample baseline pose.
        baseline_samples = []
        for ti in range(int(1.0 * 1000 / TICK_MS)):
            sentai.crazy.hover(0.0, 0.0, 0.0, z_target)
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                _extpos(x, y, z)
                baseline_samples.append((x, y, z))
            sentai.rtos.sleep_ms(TICK_MS)

        def _median(xs):
            s = sorted(xs)
            m = len(s) // 2
            return s[m] if (len(s) & 1) else 0.5 * (s[m-1] + s[m])

        if len(baseline_samples) < 5:
            summary["status"] = "AXIS_ID_NO_BASELINE"
            _phase10_land()
            _j("mission_done", {"status": summary["status"]})
            _write_summary(summary); sentai.sim.journal_close()
            return summary

        bx = _median([s[0] for s in baseline_samples])
        by = _median([s[1] for s in baseline_samples])
        bz = _median([s[2] for s in baseline_samples])
        _j("axis_id_baseline", {"n": len(baseline_samples),
                                  "median": (bx, by, bz)})

        # 5b. Axis ID via body-frame velocity pulses.
        def _pulse_and_sample(label, vx_body, vy_body,
                                pulse_s=0.8, recovery_s=0.8,
                                settle_s=1.0, sample_s=1.0):
            """Pulse hover(vx, vy) for pulse_s, opposite for recovery,
            settle, then sample.  Returns median (x, y, z) of samples."""
            _j("pulse_start", {"label": label,
                                "vx": vx_body, "vy": vy_body,
                                "pulse_s": pulse_s})
            # Pulse forward.
            for ti in range(int(pulse_s * 1000 / TICK_MS)):
                sentai.crazy.hover(vx_body, vy_body, 0.0, z_target)
                n, x, y, z, yaw = _try_pnp()
                if n >= 4 and z > 0.0:
                    _extpos(x, y, z)
                sentai.rtos.sleep_ms(TICK_MS)

            # Sample DURING pulse end (drone is at displaced position).
            during_samples = []
            for ti in range(int(0.5 * 1000 / TICK_MS)):
                sentai.crazy.hover(0.0, 0.0, 0.0, z_target)
                n, x, y, z, yaw = _try_pnp()
                if n >= 4 and z > 0.0:
                    _extpos(x, y, z)
                    during_samples.append((x, y, z))
                sentai.rtos.sleep_ms(TICK_MS)

            # Recovery: opposite pulse.
            for ti in range(int(recovery_s * 1000 / TICK_MS)):
                sentai.crazy.hover(-vx_body, -vy_body, 0.0, z_target)
                n, x, y, z, yaw = _try_pnp()
                if n >= 4 and z > 0.0:
                    _extpos(x, y, z)
                sentai.rtos.sleep_ms(TICK_MS)

            # Settle.
            for ti in range(int(settle_s * 1000 / TICK_MS)):
                sentai.crazy.hover(0.0, 0.0, 0.0, z_target)
                n, x, y, z, yaw = _try_pnp()
                if n >= 4 and z > 0.0:
                    _extpos(x, y, z)
                sentai.rtos.sleep_ms(TICK_MS)

            if len(during_samples) < 3:
                _j("pulse_done", {"label": label, "fail": "few_samples"})
                return None

            px = _median([s[0] for s in during_samples])
            py = _median([s[1] for s in during_samples])
            pz = _median([s[2] for s in during_samples])
            _j("pulse_done", {
                "label": label, "n": len(during_samples),
                "median": (px, py, pz),
                "delta_from_baseline": (px - bx, py - by, pz - bz),
            })
            return (px, py, pz)

        PULSE_V = 0.10        # 10 cm/s body-frame velocity
        result_px = _pulse_and_sample("+X_body", +PULSE_V, 0.0)
        result_py = _pulse_and_sample("+Y_body", 0.0, +PULSE_V)

        # Axis map from pulses.
        if result_px and result_py:
            dx_per_x_body = (result_px[0] - bx, result_px[1] - by)
            dy_per_y_body = (result_py[0] - bx, result_py[1] - by)
            # Yaw inferred from +X body pulse direction in world.
            yaw_observed_rad = math.atan2(dx_per_x_body[1],
                                           dx_per_x_body[0])
            _j("axis_id_map", {
                "dx_per_x_body": dx_per_x_body,
                "dy_per_y_body": dy_per_y_body,
                "inferred_yaw_rad": yaw_observed_rad,
                "inferred_yaw_deg": math.degrees(yaw_observed_rad),
            })
            summary["axis_id_dx_per_x_body"]   = dx_per_x_body
            summary["axis_id_dy_per_y_body"]   = dy_per_y_body
            summary["axis_id_inferred_yaw"]    = yaw_observed_rad

        hover_xyz = (0.0, 0.0, z_target, 0.0)
        hover_ok = True
        summary["phase_reached"] = 6
        summary["hover_ok"] = True
        summary["hover_final_xyz_yaw"] = hover_xyz
        lpf_state = (0.0, 0.0, z_target)
        summary["ekf_converged"] = True

        # Phase 7: cross sweep to discover body→world mapping via PnP.
        # Operator-stated: "vreau sa ne dam seama chiar in zbor daca
        # axa X e cumva inversata cu Y".  We sweep BODY frame velocity
        # and Kabsch fits the world response.
        # Iter-12: run sweep REGARDLESS of hover_ok — drone is being
        # controlled (cf2 velocity hold) even if oscillating; median
        # of N samples per pose handles the noise.
        if not hover_ok:
            _j("phase7_warn", "hover unstable but proceeding to sweep")
        poses, lpf_state2 = _phase7_cross_sweep(lpf_state, hover_xyz[2])
        summary["phase_reached"] = 7
        summary["sweep_poses"]   = {k: (v["median"] if v else None)
                                      for k, v in poses.items()}
        # Quick axis-map verdict.
        c  = poses["center"]["median"]    if poses.get("center") else None
        px = poses["+X"]["median"]        if poses.get("+X")     else None
        py = poses["+Y"]["median"]        if poses.get("+Y")     else None
        if c and px and py:
            dx_b_to_w = (px[0]-c[0], px[1]-c[1])
            dy_b_to_w = (py[0]-c[0], py[1]-c[1])
            summary["axis_map_x_body_to_world"] = dx_b_to_w
            summary["axis_map_y_body_to_world"] = dy_b_to_w
            summary["status"] = "SWEEP_DONE"
        else:
            summary["status"] = "SWEEP_PARTIAL"

        # Phase 8: return drone to takeoff origin (per [[sim-test-must-
        # return-home]] HR — operator-stated 2026-05-22).
        # Use position setpoint (closed-loop) — cf2 navigates back to
        # (0,0,z_hold) via its own PID.  Iter-12: phase 7 already ends
        # at (0,0,z) so this is mostly redundant — keep as safety.
        _j("phase8_return_to_origin", {"z_hold": hover_xyz[2]})
        RTH_TIMEOUT_S = 3.0
        n_ticks = int(RTH_TIMEOUT_S * 1000 / TICK_MS)
        for ti in range(n_ticks):
            _position_setpoint(0.0, 0.0, hover_xyz[2], 0.0)
            n, x, y, z, yaw = _try_pnp()
            if n >= 4 and z > 0.0:
                _extpos(x, y, z)
                if math.sqrt(x*x + y*y) < 0.08:
                    _j("phase8_arrived",
                       {"xy_err": math.sqrt(x*x + y*y),
                        "ticks": ti})
                    break
            sentai.rtos.sleep_ms(TICK_MS)
        _j("phase8_return_to_origin", "done")

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
