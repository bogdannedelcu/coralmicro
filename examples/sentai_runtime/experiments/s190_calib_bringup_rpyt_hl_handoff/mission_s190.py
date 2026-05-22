# mission_s190 — OP-S10-W21-T7 calib bringup via RPYT→HL handoff.
#
# Anti-cheat clean: NO GT injected into cf2 SITL.  Drone climbs open-loop
# with RPYT (port 3 ch 0, Classic Commander, IMU-stabilized attitude+
# thrust — same path as XBox controller flight on real CF Brushless per
# UART_RPYT_AGENT_PROMPT.md).  Once markers come into view, ExtPos
# (port 6 ch 0, vision-PnP-derived) feeds cf2 Kalman.  After Kalman
# converges, meta-cmd notifySetpointsStop (port 7 ch 1, byte 0) relaxes
# commander priority, and HL go_to navigates to bringup start.  Then
# sentai.calib.run_bringup() runs unchanged.
#
# WBS: OP-S10-W21-T7.  See README.md for claim + pass criteria.
# Anti-cheat: [[sentai-sim-air-gapped-from-truth]].

import sentai
import struct


# ---- Names + paths ----------------------------------------------------
JOURNAL_NAME = "mission_s190_journal.txt"
SUMMARY_NAME = "mission_s190_summary.json"

# ---- WhyCon pad geometry (must match sentai_whycon_small.sdf) --------
# 6 markers on a 0.5× pad: corners at ±0.08 m, mid-bars at ±0.06 m, all
# at z=0.005 m (top of mount box).
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),
    (+0.08, +0.08, 0.005),
    (-0.06,  0.00, 0.005),
    (+0.06,  0.00, 0.005),
    (-0.08, -0.08, 0.005),
    (+0.08, -0.08, 0.005),
)

# ---- Camera intrinsics — bridge downsamples to 320×240 ---------------
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.0544        # 0.5× WhyCon outer ring

# ---- RPYT thrust ramp parameters.
# Iter-14: operator visual feedback — drone climbs through marker-visible
# band in just 1-2 frames at T_MAX=40000.  Lower T_MAX to hover thrust
# (~36500 empirical from iter-12/13 handoff thrust), extend ramp, add
# explicit hover hold to give detection multiple stable frames.
T_BASE_U16       = 30000           # initial thrust above ESC threshold
T_HOVER_U16      = 36500           # near-hover thrust (iter-12/13 empirical)
T_MAX_U16        = 37000           # iter-19: lowered cap from 38000.  Less
                                    # over-hover → drone arrives at marker
                                    # band with lower vz (less inertia for
                                    # PD to brake).
T_HOVER_NOMINAL  = 36000            # empirical hover (iter-12..17 lift-off
                                    # observed at 35500-36500).  SDF physics
                                    # predicts 41900 but CrazySim motor plugin
                                    # diverges from formula at ~15% lower.
T_MIN_U16        = 22000            # floor — below this drone free-falls
T_MAX_HOLD_U16   = 40000            # cap during PD hold
# Iter-18: PD altitude controller TUNED FROM PHYSICS.
# Drone parameters from model.sdf.jinja:
#   m = 0.025 (base) + 4 × 0.0008 (props) = 0.0282 kg
#   F_max = 4 × motorConstant × maxRotVelocity²
#         = 4 × 1.8145e-8 × 3052² = 0.676 N
#   F_hover = m·g = 0.277 N
# Local linearization at hover:
#   F(T) ≈ F_max × (T/T_max)²  →  dF/dT @ hover ≈ 1.1e-5 N/unit
# Critically-damped 2nd-order with ω_n=1.5 rad/s (~0.7s settle), ζ=0.8:
#   Kp = m · ω_n² / (dF/dT) = 0.0282 × 2.25 / 1.1e-5 ≈ 5800 units/m
#   Kd = 2ζ·sqrt(m·Kp_in_N/m·m) / (dF/dT)
#      = 2·0.8·sqrt(2.25·m²) / (dF/dT)
#      = 1.6·m·ω_n / (dF/dT)
#      ≈ 1.6·0.0282·1.5 / 1.1e-5 ≈ 6150 units/(m/s)
# Iter-17 used Kp=20000, Kd=10000 — too aggressive, drone oscillated.
KP_THRUST_PER_M       = 10000.0
KD_THRUST_PER_M_PER_S = 8000.0
VZ_LPF_ALPHA          = 0.25       # iter-19: low-pass filter on PnP-derived
                                    # velocity.  Iter-18 saw vz=1.25 m/s
                                    # spuriously when real vz≈0.3 m/s (PnP
                                    # frame-to-frame jitter).  α=0.25 blends
                                    # in only 25% of each new sample.
RAMP_S           = 6.0
TICK_MS          = 33              # ≈ 30 Hz; cf2 watchdog ~1 s
MAX_RAMP_S       = 12.0            # safety budget for the open-loop ramp
SAMPLE_MAX_S     = 2.0              # PD-controller max duration before forced
                                    # handoff (even if Z not stabilized)
Z_STABLE_TOL_M   = 0.05             # |Z − target| < 5cm = stable
V_STABLE_TOL_M_S = 0.10             # |Vz| < 10cm/s = stable
N_STABLE_REQ     = 4                # 4 consecutive frames within both tols
MAX_EKF_Z_M      = 999.0           # DISABLED: ekf_z is IMU-integration drift
N_STABLE_FRAMES  = 2               # 2 consecutive valid PnP frames before
                                    # triggering hold phase
ZERO_UNLOCK_S    = 1.5             # zero-thrust packets before ramp (unlock cf2)
EXTPOS_WARMUP_S  = 1.0             # Kalman convergence time after ExtPos starts

# ---- Bringup envelope (orchestrator args) ----------------------------
Z_HOLD             = 0.78          # bringup hover altitude.  Iter-26:
                                    # PD+warmup land drone at ~0.85-0.95m
                                    # consistently; Z_HOLD=0.6 forced HL
                                    # to descend 0.3m+ which destabilized
                                    # cf2 commander.  Match Z_HOLD to where
                                    # the drone actually settles → HL go_to
                                    # is a small XY correction, no descent.
SWEEP_RADIUS_M     = 0.025
SETTLE_S           = 2.0
VMAX_M_S           = 0.06
DUR_RELAY_S        = 30.0
DUR_HOLD_S         = 10.0
HOLD_RMS_MAX_M     = 0.030
LAND_DUR           = 2.5
PHASE_POLL_MS      = 500
PHASE_TIMEOUT_S    = 180.0

PHASE_NAMES = {
    0: "IDLE", 1: "SAMPLE", 2: "KABSCH", 3: "AUTOTUNE_X",
    4: "AUTOTUNE_Y", 5: "HOLD", 6: "SAVE",
    7: "DONE_OK", 8: "DONE_FAIL",
}
REJECT_NAMES = {
    0: "OK", 1: "INVALID_CTX", 2: "FEW_SAMPLES", 3: "KABSCH_QUAL",
    4: "AUTOTUNE_X", 5: "AUTOTUNE_Y", 6: "HOLD_DRIFT", 7: "SAVE",
    8: "SAFETY", 9: "ABORTED",
}


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
    """Classic Commander setpoint (CRTP port 3 ch 0).  IMU-stabilized
    attitude + raw thrust to motor mixer.  NO position controller in
    the loop — drone follows raw thrust regardless of Kalman state."""
    sentai.crazy.send_crtp(3, 0,
        struct.pack('<fffH', roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16))


def _extpos(x, y, z):
    """ExtPos observation (CRTP port 6 ch 0, crtp_localization_service).
    cf2 Kalman fuses as position measurement with stdDev=0.01 m.
    HW-parity: same path on SIM and real cf2."""
    sentai.crazy.send_crtp(6, 0, struct.pack('<fff', x, y, z))


def _relax_priority():
    """Meta-cmd notifySetpointsStop (CRTP port 7 ch 1, byte 0).
    In cf2 firmware: commanderRelaxPriority() → priority queue back
    to LOWEST=1, HL planner gets current state, can take over."""
    sentai.crazy.send_crtp(7, 1, b'\x00')


# ---- PnP helper -------------------------------------------------------
# Iter-5 bug: mission passed `img_pts` in WhyCon scan-order (unstable
# across frames) against `wld_pts` in MARKER_WORLD canonical order →
# correspondence mismatch → garbage homography (PnP-z=0.008 when GT
# said drone was at 1.5 m).
#
# Iter-6 fix: use `sentai.markers.get_drone_pose_tuple(yaw)` which the
# W19-T6b runtime port already wires up to do association internally
# (permutation search + Kabsch + yaw-anchor mirror picker).  Requires
# `set_marker_world(packed_bytes)` called once at setup.

def _try_pnp():
    """Returns (n_markers, drone_x, drone_y, drone_z) or (n, -1,-1,-1)
    if pose not valid this frame.  Uses get_drone_pose_tuple which
    does association + Kabsch internally.

    Iter-14: require n>=4 for non-degenerate Kabsch.  Iter-13 showed
    z=0.005 (pad-plane degenerate) when only 2-3 markers fit.

    Iter-15: also REJECT obviously-degenerate z<0.1m results — when
    Kabsch picks the Z-reflection branch wrong, output collapses to
    pad-plane z=0.005.  Drone at handoff time is at real ~0.3-0.5m,
    so any pnp_z under 0.10 is rejected as noise."""
    n = sentai.markers.detect_from_camera()
    if n < 4:
        return n, -1.0, -1.0, -1.0

    # Use cf2 yaw if available (for the symmetric-pad mirror picker);
    # default 0.0 if pose subscribe not yet up.
    try:
        p = sentai.crazy.pose()
        cf2_yaw = float(p[3]) if (p is not None and len(p) >= 4) else 0.0
    except (AttributeError, RuntimeError, TypeError):
        cf2_yaw = 0.0

    pose = sentai.markers.get_drone_pose_tuple(cf2_yaw)
    if pose is None:
        return n, -1.0, -1.0, -1.0

    x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
    # NaN check via self-comparison
    if not (x == x and y == y and z == z):
        return n, -1.0, -1.0, -1.0

    # Iter-15: reject Kabsch degenerate solution (z collapses to pad
    # plane ~0.005m).  Drone is REALLY at ~0.3-0.5m when markers come
    # into view, so any z<0.10m is a wrong-branch fit.
    if z < 0.10:
        return n, -1.0, -1.0, -1.0

    return n, x, y, z


# ---- Phases -----------------------------------------------------------

def _phase1_setup():
    """Markers + crazy + safety init; arm cf2; pose_subscribe retry."""
    _j("phase1_setup", "start")
    sentai.calib.init()
    sentai.calib.clear()
    sentai.camera.init()
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)

    # Register MARKER_WORLD for get_drone_pose internal association.
    # Binding wants packed bytes (N * 3 * float32).  MicroPython embed
    # doesn't have bytearray — concat bytes from struct.pack.
    mw_bytes = b''
    for (x, y, z) in MARKER_WORLD:
        mw_bytes += struct.pack('<fff', x, y, z)
    rc_mw = sentai.markers.set_marker_world(mw_bytes)
    _j("set_marker_world", {"n": len(MARKER_WORLD), "rc": rc_mw})

    # Open the FR frames channel so handoff-time frames are saved for
    # post-mortem visual inspection (iter-5 lesson: validate marker
    # count against actual image, not against ekf_z).  MP binding takes
    # channel NAME (string), not enum int — iter-7 bug.
    fr_frames = (
        "/home/bogdan/work/coralmicro/examples/sentai_runtime/"
        "experiments/s190_calib_bringup_rpyt_hl_handoff/fr_current/frames")
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
    """Stream RPYT(0,0,0,0) for ZERO_UNLOCK_S — cf2 firmware requires
    a zero-thrust packet before nonzero thrust (per crtp_commander_rpyt.c
    + UART_RPYT_AGENT_PROMPT.md)."""
    _j("phase2_zero_unlock", {"dur_s": ZERO_UNLOCK_S})
    n_ticks = int(ZERO_UNLOCK_S * 1000 / TICK_MS)
    for _ in range(n_ticks):
        _rpyt(0.0, 0.0, 0.0, 0)
        sentai.rtos.sleep_ms(TICK_MS)
    _j("phase2_zero_unlock", "done")


def _phase3_thrust_ramp():
    """Ramp thrust from T_BASE to T_MAX, watching for markers.  Returns
    (locked_thrust, last_pnp_xyz) on success or (None, None) on
    timeout."""
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

        n, px, py, pz = _try_pnp()
        pnp_valid = (n >= 4) and (pz > 0.0)

        ekf_z = -1.0
        try:
            p = sentai.crazy.pose()
            if p is not None and len(p) >= 3:
                ekf_z = p[2]
        except (AttributeError, RuntimeError, TypeError):
            pass

        if (ti % 8) == 0:
            _j("ramp_tick", {"t": ti, "thrust": thrust, "n": n,
                              "pnp_xyz": (px, py, pz), "ekf_z": ekf_z})

        # Safety: drone climbed too high without seeing markers — abort.
        # ekf_z is IMU-integration drift since no Z source yet; treat as
        # an upper-bound altitude indicator only.
        if ekf_z > MAX_EKF_Z_M:
            _j("ramp_safety_abort", {"t": ti, "thrust": thrust,
                                       "ekf_z": ekf_z, "reason": "MAX_EKF_Z"})
            return None, None

        if pnp_valid:
            stable_streak += 1
            last_pnp = (px, py, pz)
            if stable_streak >= N_STABLE_FRAMES:
                _j("ramp_handoff", {
                    "t": ti, "thrust": thrust,
                    "stable_streak": stable_streak,
                    "pnp_xyz": last_pnp, "ekf_z": ekf_z,
                })
                return thrust, last_pnp
        else:
            stable_streak = 0

        sentai.rtos.sleep_ms(TICK_MS)

    _j("ramp_timeout", {"final_thrust": thrust, "last_pnp": last_pnp})
    return None, None


def _phase3b_pd_hold(first_pnp):
    """Iter-17: PD altitude controller using vision feedback.

    Algorithm (classic quadrotor altitude hold, Bouabdallah 2004):
        z_target = first_pnp[2]                # lock onto first valid Z
        each frame:
            vz       = (z_now − z_prev) / dt   # finite-diff velocity
            err_z    = z_target − z_now
            thrust   = T_HOVER + Kp·err_z − Kd·vz
            thrust   = clamp(thrust, T_MIN, T_MAX)
            send RPYT(0,0,0, thrust)
        STABLE when |err_z| < 5cm AND |vz| < 10cm/s for N_STABLE_REQ frames

    Operator-stated requirement: 'cumva redu thrust cu cresterea lui Z' +
    'sa oprim acceleratia cand incepem sa percepem distanta'.  PD does
    this exactly — Kd term is viscous damping (proportional to velocity).

    NO ExtPos sent during this phase ('fara acel Z stabil nu putem
    comunica la drona').  ExtPos warmup starts in phase 4 once stable.

    Returns (x, y, z) tuple of the locked position, or None on timeout."""
    z_target  = first_pnp[2]
    z_prev    = first_pnp[2]
    vz_filt   = 0.0           # iter-19: low-pass-filtered velocity
    last_xyz  = first_pnp
    stable_count = 0
    max_ticks = int(SAMPLE_MAX_S * 1000 / TICK_MS)

    _j("phase3b_pd_hold", {"z_target": z_target,
                              "Kp": KP_THRUST_PER_M,
                              "Kd": KD_THRUST_PER_M_PER_S,
                              "T_HOVER": T_HOVER_NOMINAL})

    for ti in range(max_ticks):
        n, px, py, pz = _try_pnp()
        # If PnP failed this frame, use last known z (extrapolation).
        if n >= 4 and pz > 0.0:
            z_now = pz
            last_xyz = (px, py, pz)
        else:
            z_now = z_prev

        # Velocity estimate (m/s) from finite difference + LPF for noise.
        dt_s = TICK_MS / 1000.0
        vz_raw = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt

        # PD controller in thrust units.
        err_z = z_target - z_now
        thrust = T_HOVER_NOMINAL + int(
            KP_THRUST_PER_M * err_z - KD_THRUST_PER_M_PER_S * vz_filt)
        if thrust < T_MIN_U16:    thrust = T_MIN_U16
        if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16

        _rpyt(0.0, 0.0, 0.0, thrust)

        if (ti % 3) == 0:
            ekf_z = -1.0
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_z = p[2]
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("pd_tick", {"t": ti, "n": n, "z_now": z_now,
                            "vz_raw": vz_raw, "vz_filt": vz_filt,
                            "err_z": err_z, "thrust": thrust,
                            "ekf_z": ekf_z, "stable": stable_count})

        # Stability check — VELOCITY-ONLY (iter-20).  Iter-19 showed drone
        # stabilized at z=0.70m vs target z=0.37m (33cm overshoot from P
        # weakness).  For handoff we only need vz≈0 — HL go_to handles
        # any tracking to final hover altitude after handoff.
        if (abs(vz_filt) < V_STABLE_TOL_M_S
                and n >= 4 and pz > 0.0):
            stable_count += 1
            if stable_count >= N_STABLE_REQ:
                _j("phase3b_pd_hold", {"stable": True,
                                          "final_xyz": last_xyz,
                                          "z_target": z_target,
                                          "final_thrust": thrust,
                                          "final_vz_filt": vz_filt})
                return last_xyz, thrust, vz_filt
        else:
            stable_count = 0

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase3b_pd_hold", {"stable": False, "timed_out": True,
                              "final_xyz": last_xyz,
                              "final_thrust": thrust,
                              "final_vz_filt": vz_filt})
    return last_xyz, thrust, vz_filt   # iter-21: also return last thrust
                                        # so warmup can continue from where
                                        # PD left off (no thrust step-up
                                        # that re-accelerates drone)


def _phase4_extpos_warmup(pd_state):
    """Iter-21: CONTINUE the PD loop while streaming ExtPos.  Iter-20
    bug: switching to fixed T_HOVER=36500 caused +4000 thrust jump from
    PD's stable thrust (~32500) → drone re-accelerated upward → Z drift
    +0.16m during warmup → bringup orchestrator saw wrong altitude →
    'o ia razna' (operator visual).

    Stream ExtPos with PnP-derived (x,y,z) at 30 Hz.  cf2 Kalman locks.
    PD keeps drone steady throughout. """
    pd_xyz, pd_thrust, pd_vz_filt = pd_state
    z_target = pd_xyz[2]            # lock target to PD's final altitude
    z_prev   = pd_xyz[2]
    vz_filt  = pd_vz_filt            # inherit from PD
    last_xyz = pd_xyz
    _j("phase4_extpos_warmup", {"start_thrust": pd_thrust,
                                  "z_target": z_target,
                                  "dur_s": EXTPOS_WARMUP_S})
    n_ticks = int(EXTPOS_WARMUP_S * 1000 / TICK_MS)
    for ti in range(n_ticks):
        # Try a fresh PnP this tick
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            z_now = pz
            last_xyz = (px, py, pz)
            _extpos(px, py, pz)
        else:
            z_now = z_prev
            _extpos(last_xyz[0], last_xyz[1], last_xyz[2])

        # Continue PD with same gains.
        dt_s = TICK_MS / 1000.0
        vz_raw = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt
        err_z = z_target - z_now
        thrust = T_HOVER_NOMINAL + int(
            KP_THRUST_PER_M * err_z - KD_THRUST_PER_M_PER_S * vz_filt)
        if thrust < T_MIN_U16:    thrust = T_MIN_U16
        if thrust > T_MAX_HOLD_U16: thrust = T_MAX_HOLD_U16

        _rpyt(0.0, 0.0, 0.0, thrust)
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            _extpos(px, py, pz)
            last_xyz = (px, py, pz)

        if (ti % 8) == 0:
            ekf_xyz = (-1.0, -1.0, -1.0)
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_xyz = (p[0], p[1], p[2])
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("warmup_tick", {"t": ti, "n": n,
                                "pnp_xyz": (px, py, pz),
                                "ekf_xyz": ekf_xyz})
        sentai.rtos.sleep_ms(TICK_MS)
    _j("phase4_extpos_warmup", {"last_xyz": last_xyz})
    return last_xyz


def _phase5_handoff_to_hl(last_xyz):
    """STOP RPYT stream; engage HL planner BEFORE relaxing priority;
    smoothly hand off to HL go_to.

    Iter-26 firmware-analysis fix: previous order (relax → sleep → go_to)
    created a ~30-100ms nullSetpoint gap during which HL task wrote
    nullSetpoint (motors off) at priority=1, accepted because priority
    just dropped to 1.  cf2 motors cut briefly → drone destabilizes →
    bringup never recovers.

    Insight from crtp_commander_high_level.c source:
    - `go_to` (CRTP port 8 ch 0) is a META-COMMAND that calls
      `plan_go_to_from()` directly — it modifies planner.state to
      FLYING WITHOUT going through commanderSetSetpoint/priority queue.
    - So we can engage HL planner FIRST while RPYT priority=2 is still
      active.  HL writes setpoints with priority=1 but they're rejected
      (1 < 2), so the previous RPYT setpoint stays in queue (drone
      continues last commanded thrust).
    - Once priority is relaxed (notifySetpointsStop), HL's setpoints
      (now valid because planner.state=FLYING evaluates the trajectory)
      are accepted (1 >= 1).  No gap, no nullSetpoint.
    """
    _j("phase5_handoff", {"last_xyz": last_xyz, "target_z": Z_HOLD})

    # Step 1: send go_to FIRST.  Changes planner.state to FLYING,
    # builds trajectory from HL's cached pos (refreshed each HL tick
    # while RPYT was driving).  This is a meta-command — does NOT
    # write to the commander setpoint queue.
    try:
        sentai.crazy.go_to(0.0, 0.0, Z_HOLD, 0.0, 2.0)
        _j("go_to_engage", {"target": (0.0, 0.0, Z_HOLD)})
    except (AttributeError, RuntimeError) as e:
        _j("go_to_fail", {"err": str(e)})

    # Step 2: wait one HL tick (~10ms is the HL period in firmware).
    # During this, HL evaluates the trajectory and tries to write the
    # first setpoint at priority=1 — STILL REJECTED because mission's
    # RPYT priority=2 is current.  No harm: last RPYT setpoint still
    # holds drone.  Mainly ensures HL has time to compute the
    # trajectory's first sample.
    sentai.rtos.sleep_ms(15)

    # Step 3: NOW relax priority.  HL's already-pending valid setpoint
    # (next tick) writes with priority=1 → 1 >= 1 → ACCEPTED.  Smooth
    # transition with no nullSetpoint gap.
    _relax_priority()
    _j("relax_priority", "sent")

    # Step 4: pump ExtPos at 30 Hz while go_to executes.  HL writes
    # its setpoints at ~100 Hz from cf2 firmware side; we don't compete.
    n_ticks = int(2.5 * 1000 / TICK_MS)
    for ti in range(n_ticks):
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            _extpos(px, py, pz)
        if (ti % 8) == 0:
            ekf_xyz = (-1.0, -1.0, -1.0)
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_xyz = (p[0], p[1], p[2])
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("goto_tick", {"t": ti, "n": n,
                              "pnp_xyz": (px, py, pz),
                              "ekf_xyz": ekf_xyz})
        sentai.rtos.sleep_ms(TICK_MS)
    _j("phase5_handoff", "done")


def _phase6_bringup(summary):
    """Run the existing C-side orchestrator unchanged.  ExtPos stream is
    handed over to the orchestrator's internal VPE forwarder (if any) or
    we maintain it here in parallel via the safety task."""
    # Iter-23+: use the backend-agnostic enable_markers (sentai.markers
    # backend was init'd as 'whycon' in setup; the check counts whatever
    # the dispatcher returns — enable_aruco was misnamed legacy).
    _j("safety_enable",     {"rc": sentai.safety.enable_markers(4, 4.0)})
    _j("safety_task_start", {"rc": sentai.safety.task_start()})

    _j("bringup_start", {
        "z_hold":       Z_HOLD,
        "sweep_radius": SWEEP_RADIUS_M,
        "settle_s":     SETTLE_S,
        "vmax":         VMAX_M_S,
        "dur_relay":    DUR_RELAY_S,
        "dur_hold":     DUR_HOLD_S,
    })
    rc = sentai.calib.run_bringup(
        MARKER_WORLD,
        marker_size_m  = MARKER_DIAMETER_M,
        z_hold         = Z_HOLD,
        sweep_radius   = SWEEP_RADIUS_M,
        settle_s       = SETTLE_S,
        vmax           = VMAX_M_S,
        dur_relay      = DUR_RELAY_S,
        dur_hold       = DUR_HOLD_S,
        hold_rms_max   = HOLD_RMS_MAX_M,
    )
    _j("bringup_rc", {"rc": rc})
    if rc != 0:
        _j("bringup_fail", "spawn rc != 0")
        summary["status"] = "SPAWN_FAIL"
        return False

    last_phase = -1
    t_acc_ms = 0
    done = False
    # Iter-23: log marker count + ekf pose every 100ms during bringup so
    # we can see EXACTLY when markers disappear and what cf2 EKF thinks
    # at that moment.  Operator-stated observation: 'dupa o secunda de
    # stabilizare incepe sa driftuiasca pe z foarte mult'.
    POLL_FAST_MS = 100
    while t_acc_ms < int(PHASE_TIMEOUT_S * 1000):
        if sentai.calib.bringup_is_done():
            done = True
            break
        phase = sentai.calib.bringup_get_phase()
        if phase != last_phase:
            _j("bringup_phase", {"phase": phase,
                                  "name": PHASE_NAMES.get(phase, "?")})
            last_phase = phase

        # Sample marker count + EKF pose for diagnostic.  This does NOT
        # trigger a detect (safety_task already does that internally at
        # its period).  Just reads the cached count.
        try:
            n_cache = sentai.markers.get_count()
        except (AttributeError, RuntimeError):
            n_cache = -1
        try:
            p = sentai.crazy.pose()
            ekf_xyz = (p[0], p[1], p[2]) if (p is not None and len(p) >= 3) else (-1, -1, -1)
        except (AttributeError, RuntimeError, TypeError):
            ekf_xyz = (-1, -1, -1)
        try:
            aborted = sentai.safety.aborted()
        except (AttributeError, RuntimeError):
            aborted = -1
        _j("bringup_tick", {
            "t_ms":   t_acc_ms,
            "phase":  phase,
            "n_cache": n_cache,
            "ekf_xyz": ekf_xyz,
            "safety_aborted": aborted,
        })

        sentai.rtos.sleep_ms(POLL_FAST_MS)
        t_acc_ms += POLL_FAST_MS

    r = sentai.calib.bringup_get_result()
    summary["phase_reached"] = r["last_phase"]
    summary["reject_code"]   = r["reject_code"]
    summary["reject_name"]   = REJECT_NAMES.get(r["reject_code"], "?")
    summary["accepted"]      = bool(r["accepted"])
    summary["kp_x"]          = r["kp_x"]
    summary["kp_y"]          = r["kp_y"]
    summary["hold_rms"]      = r["hold_rms_drift_m"]
    summary["hold_max"]      = r["hold_max_drift_m"]
    summary["n_samples"]     = r["n_samples_used"]
    summary["duration_ms"]   = r["total_duration_ms"]
    summary["R"]             = list(r["R_cam_to_body"])
    summary["cam_offset_B"]  = list(r["cam_offset_B"])
    summary["ext_quality"]   = r["ext_quality"]
    _j("bringup_result", summary)
    return done


def _phase7_land():
    """HL land + disarm.  Drone returns to home via final go_to first."""
    _j("phase7_land", "start")
    try:
        sentai.crazy.go_to(0.0, 0.0, Z_HOLD, 0.0, 2.0)
    except (AttributeError, RuntimeError):
        pass
    sentai.rtos.sleep_ms(2500)
    sentai.crazy.land(LAND_DUR)
    sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
    sentai.crazy.disarm()
    _j("phase7_land", "done")


# ---- Entry point ------------------------------------------------------

def run():
    """Operator entry.  Returns dict with status + bringup metrics."""
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"wbs": "OP-S10-W21-T7", "iter": 1})

    summary = {
        "status":         "ERROR",
        "phase_reached":  -1,
        "reject_code":    -1,
        "reject_name":    "",
        "accepted":       False,
        "kp_x":           0.0,
        "kp_y":           0.0,
        "hold_rms":       0.0,
        "hold_max":       0.0,
        "n_samples":      0,
        "duration_ms":    0,
        "R":              None,
        "cam_offset_B":   None,
        "ext_quality":    None,
        "assert_calibrated": False,
        "handoff_xyz":    None,
        "handoff_thrust": 0,
    }

    try:
        _phase1_setup()
        _phase2_zero_unlock()
        locked_thrust, last_pnp = _phase3_thrust_ramp()
        if locked_thrust is None:
            summary["status"] = "RAMP_TIMEOUT"
            # Kill thrust safely before any land attempt.
            for _ in range(10):
                _rpyt(0.0, 0.0, 0.0, 0)
                sentai.rtos.sleep_ms(TICK_MS)
            sentai.crazy.disarm()
            _write_summary(summary)
            sentai.sim.journal_close()
            return summary

        summary["handoff_thrust"] = locked_thrust
        summary["handoff_xyz"]    = last_pnp

        # Iter-17: PD altitude hold — lock target_z to first detected z,
        # apply PD controller with vision feedback until vz_filt ≈ 0.
        # Iter-21: returns (xyz, thrust, vz_filt) so warmup can continue
        # PD seamlessly instead of jumping thrust.
        pd_state = _phase3b_pd_hold(last_pnp)
        last_pnp = pd_state[0]

        warmup_xyz = _phase4_extpos_warmup(pd_state)
        _phase5_handoff_to_hl(warmup_xyz)

        bringup_done = _phase6_bringup(summary)

        try:
            sentai.calib.assert_calibrated()
            summary["assert_calibrated"] = True
        except RuntimeError:
            summary["assert_calibrated"] = False

        if summary["accepted"]:
            summary["status"] = "PASS"
        elif not bringup_done:
            summary["status"] = "TIMEOUT"
        else:
            summary["status"] = "FAIL"

        _phase7_land()
    except Exception as e:
        _j("mission_exception", {"err": str(e)})
        summary["status"] = "EXCEPTION"
        # Best-effort safe state.
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
