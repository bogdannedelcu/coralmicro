# mission_s194 — OP-S10-W21-T15 closed-loop PD per-axis ID + level-settle.
#
# Forked from mission_s193 iter18 (open-loop pulse axis ID — failed at
# SAMPLE due to drift-contaminated R: pitch_disp and roll_disp ended up
# nearly anti-parallel after 1.2 s passive settle).  s194 replaces the
# `_phase4_5_axis_id_rpyt` open-loop body with `_phase4_5_axis_id_rpyt_pd`:
#
#   For each body axis (pitch, then roll):
#     1. Exploratory 4°/250 ms pulse → measure Δp_pad → d_axis unit vec
#     2. PD-park drone at p0 + δ·d_axis (δ=6 cm) — closed loop until
#        |err_along| < 1 cm AND |vel_along| < 5 cm/s for 5 ticks
#     3. Level-settle: stream rpyt(0,0,0,thrust_PD) 150 ms (cf2 attitude
#        PID @ 1 kHz brings drone to true level)
#     4. Capture median PnP over 0.4 s (drone level + stationary)
#     5. PD-park at p0 - δ·d_axis, level-settle, capture p_-
#     6. axis_disp_pad = (p_+ - p_-) / 2  ← drift-canceled by symmetry
#
# All other phases preserved from s193: setup → zero-unlock → RPYT thrust
# ramp → PD altitude lock → ExtPos warmup → AXIS ID (new) → handoff →
# bringup → land.
#
# Anti-cheat clean: NO GT injected into cf2 SITL.  Drone climbs open-loop
# with RPYT (port 3 ch 0, Classic Commander).  Once markers come into
# view, ExtPos (port 6 ch 0, vision-PnP-derived) feeds cf2 Kalman.  After
# Kalman converges, notifySetpointsStop relaxes commander priority, and
# HL go_to navigates to bringup start.  sentai.calib.run_bringup() then
# runs unchanged with R already committed by the new axis ID phase.
#
# WBS: OP-S10-W21-T15.  See README.md for claim + pass criteria.
# Anti-cheat: [[sentai-sim-air-gapped-from-truth]].

import sentai
import struct


# ---- Names + paths ----------------------------------------------------
JOURNAL_NAME = "mission_s194_journal.txt"
SUMMARY_NAME = "mission_s194_summary.json"

# ---- WhyCon pad geometry (must match sentai_whycon_small.sdf) --------
# 7 markers on a 0.5× pad — 6 symmetric corners/mid-bars at z=0.005 m
# (top of mount box) PLUS the asymmetric marker N at (+0.02, +0.10).
# The N marker breaks rectangular symmetry → unique PnP solution.
# Added to upstream sentai_whycon_small.sdf 2026-05-23 — see
# ideas/external_patches.md and B2 dataset bench
# dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/.
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),   # NW
    (+0.08, +0.08, 0.005),   # NE
    (-0.06,  0.00, 0.005),   # W
    (+0.06,  0.00, 0.005),   # E
    (-0.08, -0.08, 0.005),   # SW
    (+0.08, -0.08, 0.005),   # SE
    (+0.02, +0.10, 0.005),   # N  ← asymmetric, breaks PnP ambiguity
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
T_MAX_U16        = 34000           # iter9: 37000→34000.  With T_HOVER=32500
                                    # margin to T_MAX = 1500 units is enough
                                    # to ramp drone airborne but no big over-
                                    # over-hover → drone arrives at marker
                                    # band with lower vz (less inertia for
                                    # PD to brake).
T_HOVER_NOMINAL  = 32500            # iter8 — empirically recalibrated.
                                    # iter1-7 used 36000 (estimated from LIFT-
                                    # OFF, not hover).  iter7 evidence: thrust
                                    # ≈32400 with PID showed drone falling
                                    # ~5 cm/s → real hover < 32500.  Drone
                                    # mass ~0.028 kg, motor const matches
                                    # F(T)≈0.676·(T/65535)² → T_hover ideal
                                    # ≈41900 but CrazySim plugin is ~20%
                                    # lower than formula → 32500 fits data.
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
KP_THRUST_PER_M       = 10000.0    # iter8: 6k→10k (T_HOVER now correct, agresivnow safe)
KD_THRUST_PER_M_PER_S = 10000.0    # iter8: 12k→10k (less overdamped)
# iter8: T_HOVER_NOMINAL recalibrated 36000→32500 to match real cf2 SITL
# hover thrust.  With T_HOVER correct, P term alone gives small steady-state
# error (~5 cm for 500-unit residual bias), I term only handles transient
# disturbances → smaller Ki + smaller I_max, faster response.
KI_THRUST_PER_M_S     = 2000.0     # iter8: 5k→2k (less integral wind-up)
I_INTEGRAL_MAX        = 0.5        # iter8: 2.0→0.5 m·s (limits over-correction)

# iter5 NEW: XY position hold during Phase 3b + Phase 4.
# Naive identity assumption: pitch ≡ pad+x, roll ≡ pad+y.  For our 90° mount
# rotation, this is WRONG by 90° → naive PD would push drone in the
# perpendicular direction.  Mitigated by VERY SMALL gains + tight saturation:
# even wrong-direction commands at 1.5° max produce lateral accel ≈ 0.26 m/s²;
# combined with KD it bounds drift to ~5 cm even if direction is inverted.
# Axis ID proper (Phase 4.5) discovers the real R afterwards.
KP_XY_PER_M           = 2.0        # deg / m
KD_XY_PER_M_S         = 8.0        # deg / (m/s)
ATT_XY_CMD_MAX_DEG    = 1.5        # ±1.5° hard saturation (safe for wrong direction)
VXY_HOLD_LPF_ALPHA    = 0.3        # LPF on PnP-derived XY velocity
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
# iter3: Phase 3b stable check tightened — require BOTH position and velocity
# inside tolerance (was velocity-only in iter1/iter2 from s193 inheritance).
# iter4: relaxed pos tol 3→5 cm to match Z gate (iter3 PID converged to
# 4.9 cm steady-state — just outside the 3 cm threshold, timed out).
Z_STABLE_TOL_M   = 0.05             # iter4: 3→5 cm (matches Z-gate threshold)
V_STABLE_TOL_M_S = 0.05             # iter5: 3→5 cm/s (matches softer Z PID overshoot)
N_STABLE_REQ     = 12               # iter5: 15→12 ticks (400 ms — faster decision)
PHASE3B_TIMEOUT_S = 6.0             # iter9: 4→6 s safety margin (PID with
                                    # T_HOVER fixed converges fast, but PnP
                                    # intermittent n=3 ticks reset stable
                                    # counter — extra time absorbs jitter)

# iter5 NEW: ramp handoff requires PnP-z near target hover altitude.
# iter4 saw ramp handoff with pnp_z=0.30 but ekf_z=0.54 — Phase 3b then
# had to fight a 24 cm overshoot in EKF state.  Enforce tighter gate.
RAMP_HANDOFF_Z_TARGET = 0.30        # m
RAMP_HANDOFF_Z_TOL    = 0.08        # m — accept handoff when |pnp_z - target| < 8 cm
MAX_EKF_Z_M      = 999.0           # DISABLED: ekf_z is IMU-integration drift
N_STABLE_FRAMES  = 2               # 2 consecutive valid PnP frames before
                                    # triggering hold phase
ZERO_UNLOCK_S    = 1.5             # zero-thrust packets before ramp (unlock cf2)
EXTPOS_WARMUP_S       = 0.5         # legacy fallback time

# iter8: REVERT smart convergence (iter7 unstable — drone oscillated 8s).
# iter6 with warmup=0.5s was best (land 74cm, no crash).  Try even
# shorter to minimize time in unstable RPYT↔PnP feedback loop.
EXTPOS_WARMUP_MAX_S   = 0.3         # iter8: 200ms after PD lock (was 8s)
EKF_VZ_STABLE_M_S     = 0.10
EKF_STABLE_TICKS_REQ  = 3           # very fast exit (3 ticks ≈ 100ms)

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


def _fr_scalar(label, value):
    """Best-effort FR scalar push.  FR task is auto-started at
    sentai_sim boot, channels default to $SENTAI_FR_DIR/*.csv.
    Async-buffered — no impact on mission tick budget."""
    try:
        sentai.fr.push_scalar(label, float(value))
    except (AttributeError, RuntimeError, TypeError, ValueError):
        pass


def _fr_event(etype, text):
    """Best-effort FR event push for phase boundaries + anomalies."""
    try:
        sentai.fr.push_event(etype, text)
    except (AttributeError, RuntimeError, TypeError):
        pass


def _fr_pnp_ekf(px, py, pz, n, thrust=None):
    """Compact per-tick FR scalars: PnP xyz + ekf xyz + n + optional thrust.
    Streams to fr_current/scalars.csv at whatever rate caller invokes —
    intended @30 Hz from phases 4, 5, 6 for post-mortem time-series."""
    _fr_scalar("pnp_x", px); _fr_scalar("pnp_y", py); _fr_scalar("pnp_z", pz)
    _fr_scalar("n_pnp", n)
    ekf_x = ekf_y = ekf_z = -9.99
    try:
        p = sentai.crazy.pose()
        if p is not None and len(p) >= 3:
            ekf_x, ekf_y, ekf_z = float(p[0]), float(p[1]), float(p[2])
    except (AttributeError, RuntimeError, TypeError):
        pass
    _fr_scalar("ekf_x", ekf_x); _fr_scalar("ekf_y", ekf_y); _fr_scalar("ekf_z", ekf_z)
    if thrust is not None:
        _fr_scalar("thrust", thrust)


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
    HW-parity: same path on SIM and real cf2.

    WARNING: cf2 firmware accepts this at stdDev=0.01m → near-truth
    weight.  Single bad packet (e.g. PnP mirror branch z=0.28 when
    drone is at z=0.80) corrupts Kalman almost instantly.  Use
    `_extpos_guarded` for normal mission code.  Direct `_extpos`
    only for trusted contexts (e.g. pre-arm warmup at known ground)."""
    sentai.crazy.send_crtp(6, 0, struct.pack('<fff', x, y, z))


# ── Innovation-gated ExtPos (s193 iter4 — fix A + B from iter3 PM) ────
# cf2 has NO other Z source (no baro, no TOF cheat, no flow_deck).
# Gating ExtPos = trusting IMU integration for the rejected ticks.
# IMU drift over 165 ms ≈ 1.4 mm position, 17 mm/s velocity — safe.
# Force-accept after 5 consecutive rejects = bound the worst case.
EXTPOS_MAX_INNOV_M       = 0.30   # innov gate: reject if |delta| > 30 cm
EXTPOS_REJECT_OVERRIDE   = 5      # after 5 rejects, force-accept (Kalman is the one wrong)

# Mutable container so the helper can update its state without `global`
# (MicroPython embed sometimes has issues with module-level globals).
_extpos_state = {"reject_streak": 0}


def _extpos_reset_gate():
    """Call at the start of any phase that streams ExtPos so the
    reject-streak counter doesn't carry across phases."""
    _extpos_state["reject_streak"] = 0


def _extpos_guarded(x, y, z):
    """ExtPos with innovation gate + override-on-streak.  Returns True
    if sent, False if rejected.  Logs FR scalar `extpos_reject` (0/1)
    per call and FR event on each force-accept override.

    Algorithm (s193 iter4):
      ekf := sentai.crazy.pose() current state
      innov := max(|x-ekf.x|, |y-ekf.y|, |z-ekf.z|)
      if innov > 0.30m AND reject_streak < 5:
          reject (don't send), increment streak
      else:
          send, reset streak; log event if it was an override
    """
    try:
        p = sentai.crazy.pose()
        if p is not None and len(p) >= 3:
            ekx, eky, ekz = float(p[0]), float(p[1]), float(p[2])
            innov = abs(x - ekx)
            iy = abs(y - eky)
            iz = abs(z - ekz)
            if iy > innov: innov = iy
            if iz > innov: innov = iz
            if innov > EXTPOS_MAX_INNOV_M:
                if _extpos_state["reject_streak"] < EXTPOS_REJECT_OVERRIDE:
                    _extpos_state["reject_streak"] += 1
                    _fr_scalar("extpos_reject", 1)
                    _fr_scalar("extpos_innov", innov)
                    return False
                # Streak overflow → force accept (Kalman is corrupted,
                # let it snap to PnP).
                _fr_event("extpos_override",
                           "force-accept after %d rejects innov=%.3f xyz=(%.3f,%.3f,%.3f) ekf=(%.3f,%.3f,%.3f)" %
                           (_extpos_state["reject_streak"], innov,
                            x, y, z, ekx, eky, ekz))
    except (AttributeError, RuntimeError, TypeError):
        pass     # if pose() unavailable, send unconditionally
    _extpos(x, y, z)
    _fr_scalar("extpos_reject", 0)
    _extpos_state["reject_streak"] = 0
    return True


# ── Last-resort Z fallback (s193 iter4 — operator-requested 2026-05-23) ─
# Simple constant — 0.5 m default altitude.  Used when PnP fails but
# markers ARE visible (n >= 1), so we know drone is somewhere over the
# pad.  Better than letting cf2 Kalman drift on IMU-only Z.
# Operator: "nu ceva elaborat, doar o constanta... pune 0.5m default
# last resort".
Z_FALLBACK_M = 0.5


def _extpos_force_z(z):
    """Z-only ExtPos: echoes current Kalman x/y, only Z changes.
    XY innovation = 0 by construction → bypasses the innovation gate.
    Use for fallback during takeoff / PnP outages.
    Returns True if sent, False if Kalman pose unavailable."""
    try:
        p = sentai.crazy.pose()
        if p is None or len(p) < 3:
            return False
        ekx, eky = float(p[0]), float(p[1])
    except (AttributeError, RuntimeError, TypeError):
        return False
    _extpos(ekx, eky, z)
    _fr_scalar("z_fallback_used", 1)
    return True


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
    pnp_yaw_rad = float(pose[3]) if len(pose) >= 4 else 0.0
    # NaN check via self-comparison
    if not (x == x and y == y and z == z):
        return n, -1.0, -1.0, -1.0

    # iter12 — REVERTED iter11 yaw reject.  Operator-corrected:
    # the WHOLE POINT of calibration is to discover R_cam_to_body
    # (which encodes the body↔pad XY rotation).  PnP yaw being ±90°
    # at handoff is EXPECTED — SDF GT has R_cam_to_body=[[0,1,0],
    # [1,0,0],[0,0,-1]] (camera mounted 90° rotated).  Calib's
    # SAMPLE phase discovers this via body-frame motion + pad-frame
    # observation.  Rejecting PnP based on yaw breaks calib.
    # Keep PnP yaw logging for visibility.
    _fr_scalar("pnp_yaw_deg", pnp_yaw_rad * 57.29577951)
    _fr_scalar("cf2_yaw_deg", cf2_yaw * 57.29577951)

    # Iter-15: reject Kabsch degenerate solution (z collapses to pad
    # plane ~0.005m).
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
        "experiments/s193_calib_full_postT10/iter2/fr_current/frames")
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

        # iter4 fix: PAD VISIBLE (n>=4) but PnP returned invalid pose
        # (pose=None, NaN, or z<0.10 sentinel) → feed cf2 Kalman a
        # default Z so it has SOME Z anchor instead of pure IMU drift.
        # Operator-requested 2026-05-23: "presupunem ca daca N>=4 atunci
        # facem last resort, doar o constanta 0.5m default".
        if (n >= 4) and not pnp_valid:
            _extpos_force_z(Z_FALLBACK_M)

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
    """iter3: PID altitude controller (was PD in iter1/iter2).

    Adds integral term to eliminate steady-state error caused by
    T_HOVER_NOMINAL bias.  With P-only, steady state has err_z ≠ 0 to
    balance (T_real_hover − T_HOVER_NOMINAL) / Kp.  With I term:
        thrust = T_HOVER + Kp·err + Ki·∫err·dt − Kd·vz
    Anti-windup: integral frozen when thrust output is clipped to bounds.

    Operator-stated requirement 2026-05-24: "pai nu facem echilibrarea
    p Z cu PID?  Chiar daca avem constanta aia tot trebuie sa tinem
    stabilizarea pe Z" + scope of sentai.calib: "fara sa prabusim drona,
    incet incet sa ii descoperim caracteristicile".

    Stable check tightened to BOTH position AND velocity:
        |err_z| < Z_STABLE_TOL_M (3 cm) AND |vz_filt| < V_STABLE_TOL_M_S (3 cm/s)
    for N_STABLE_REQ (15) consecutive ticks ≈ 500 ms.

    Returns (last_xyz, thrust, vz_filt, err_z_integral, x0, y0) for chaining
    PID state forward.  iter5: also returns (x0, y0) = handoff XY snapshot
    used as the "stay-here" reference for XY hold in Phase 4 and axis ID."""
    # iter5: FIXED target altitude — don't track ramp's z, drive to hover band.
    z_target  = RAMP_HANDOFF_Z_TARGET
    z_prev    = first_pnp[2]
    vz_filt   = 0.0
    err_z_integral = 0.0
    last_xyz  = first_pnp
    x0, y0    = first_pnp[0], first_pnp[1]   # iter5: XY anchor for hold
    px_prev   = first_pnp[0]
    py_prev   = first_pnp[1]
    vx_filt   = 0.0
    vy_filt   = 0.0
    stable_count = 0
    max_ticks = int(PHASE3B_TIMEOUT_S * 1000 / TICK_MS)

    _j("phase3b_pid_hold", {"z_target": z_target,
                              "Kp": KP_THRUST_PER_M,
                              "Ki": KI_THRUST_PER_M_S,
                              "Kd": KD_THRUST_PER_M_PER_S,
                              "I_max": I_INTEGRAL_MAX,
                              "T_HOVER": T_HOVER_NOMINAL,
                              "z_tol_m": Z_STABLE_TOL_M,
                              "v_tol_m_s": V_STABLE_TOL_M_S,
                              "n_stable_req": N_STABLE_REQ,
                              "timeout_s": PHASE3B_TIMEOUT_S,
                              "xy_anchor": (x0, y0),
                              "Kp_xy": KP_XY_PER_M,
                              "Kd_xy": KD_XY_PER_M_S,
                              "att_xy_max_deg": ATT_XY_CMD_MAX_DEG,
                              "first_pnp_z": first_pnp[2]})

    for ti in range(max_ticks):
        n, px, py, pz = _try_pnp()
        # iter9: accept n>=3 (was n>=4) — pad has 7 markers but at low z
        # PnP often momentarily drops to 3 visible.  Position estimate
        # with 3 markers is still valid (overdetermined for 6-DOF PnP
        # is anything ≥3).  Was: gate at n>=4.
        if n >= 3 and pz > 0.0:
            z_now = pz
            last_xyz = (px, py, pz)
            _extpos_force_z(pz)
        else:
            z_now = z_prev

        dt_s = TICK_MS / 1000.0
        vz_raw = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt

        err_z = z_target - z_now

        # PID controller in thrust units (anti-windup applied after clip)
        i_term = KI_THRUST_PER_M_S * err_z_integral
        thrust_raw = (T_HOVER_NOMINAL
                      + int(KP_THRUST_PER_M * err_z
                            + i_term
                            - KD_THRUST_PER_M_PER_S * vz_filt))
        clipped = False
        if thrust_raw < T_MIN_U16:
            thrust = T_MIN_U16
            clipped = True
        elif thrust_raw > T_MAX_HOLD_U16:
            thrust = T_MAX_HOLD_U16
            clipped = True
        else:
            thrust = thrust_raw

        # Anti-windup: only accumulate integral when not clipped AND with
        # fresh measurement (n>=3 — iter9 relaxed from n>=4 for low-altitude
        # FOV; _try_pnp internally still requires n>=4 today, so this is
        # forward-compat for future PnP relaxation).
        if not clipped and n >= 3 and pz > 0.0:
            err_z_integral += err_z * dt_s
            if err_z_integral >  I_INTEGRAL_MAX: err_z_integral =  I_INTEGRAL_MAX
            if err_z_integral < -I_INTEGRAL_MAX: err_z_integral = -I_INTEGRAL_MAX

        # iter6: XY hold REMOVED — iter5 evidence showed naive identity
        # mapping pushed drone in WRONG direction for 90° mount, plus
        # noisy PnP-derived velocity caused 1.5° saturation oscillation.
        # iter6 strategy: keep Phase 3b SHORT (3s) — accept small natural
        # drift — then do early R discovery via open-loop pulses.
        _rpyt(0.0, 0.0, 0.0, thrust)

        if (ti % 3) == 0:
            ekf_z = -1.0
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_z = p[2]
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("pid_tick", {"t": ti, "n": n, "z_now": z_now,
                              "vz_filt": vz_filt, "err_z": err_z,
                              "i_integral": err_z_integral, "i_term": i_term,
                              "thrust": thrust, "clipped": clipped,
                              "ekf_z": ekf_z, "stable": stable_count,
                              "px": last_xyz[0], "py": last_xyz[1]})

        # iter3: BOTH position AND velocity check for true stability.
        # iter9: removed n>=4 gate — drone is physically stable if err_z and
        # vz_filt are both small, regardless of whether THIS tick had fresh
        # PnP.  iter8 evidence: PID converged (err=0.028, vz=-0.002) but
        # PnP intermittent n=3 reset stable_count every other tick → never
        # crossed N_STABLE_REQ.  Removed gate; rely on z_prev tracking
        # across short PnP drops (≤6 ticks).  Will still detect false
        # stability if PnP dies for long stretches because vz_filt drops
        # to zero only after sustained z_prev = z_now (which only happens
        # with stale data → drone could be drifting).  Acceptable risk.
        if (abs(err_z) < Z_STABLE_TOL_M
                and abs(vz_filt) < V_STABLE_TOL_M_S):
            stable_count += 1
            if stable_count >= N_STABLE_REQ:
                _j("phase3b_pid_hold", {"stable": True,
                                          "final_xyz": last_xyz,
                                          "z_target": z_target,
                                          "final_thrust": thrust,
                                          "final_vz_filt": vz_filt,
                                          "final_err_z": err_z,
                                          "final_i_integral": err_z_integral,
                                          "xy_drift_cm": (
                                              (last_xyz[0]-x0)*100.0,
                                              (last_xyz[1]-y0)*100.0),
                                          "ticks": ti + 1})
                return last_xyz, thrust, vz_filt, err_z_integral, x0, y0
        else:
            stable_count = 0

        z_prev = z_now
        sentai.rtos.sleep_ms(TICK_MS)

    _j("phase3b_pid_hold", {"stable": False, "timed_out": True,
                              "final_xyz": last_xyz,
                              "final_thrust": thrust,
                              "final_vz_filt": vz_filt,
                              "final_err_z": z_target - z_prev,
                              "final_i_integral": err_z_integral,
                              "xy_drift_cm": (
                                  (last_xyz[0]-x0)*100.0,
                                  (last_xyz[1]-y0)*100.0)})
    return last_xyz, thrust, vz_filt, err_z_integral, x0, y0


def _phase4_extpos_warmup(pd_state):
    """iter7: SMART convergence-based warmup.  Operator-stated 2026-05-23:
    "handover doar dupa ce Kalman de pe drona e suficient de stabil".

    Streams ExtPos @ 30Hz while continuing PD-RPYT.  Monitors cf2 Kalman
    vz via finite-diff on ekf_z: when |ekf_vz| < 10cm/s for 6 consecutive
    ticks (~200 ms), declares CONVERGED and exits.  Falls back to time
    timeout EXTPOS_WARMUP_MAX_S=8s if Kalman never settles.

    Without firmware/binding changes, finite-diff is our only velocity
    proxy.  Future improvement: add binding for kalman.statePZ + varZ
    direct LOG subscription for richer stability metric.

    iter5: also inherits XY anchor (x0, y0) from Phase 3b for XY hold."""
    # iter5: unpack 6-tuple (xyz, thrust, vz_filt, integral, x0, y0)
    pd_xyz, pd_thrust, pd_vz_filt, pd_i, x0, y0 = pd_state
    z_target = RAMP_HANDOFF_Z_TARGET  # iter5: fixed target, not pd_xyz[2]
    z_prev   = pd_xyz[2]
    vz_filt  = pd_vz_filt            # inherit from PID
    err_z_integral = pd_i            # inherit integral — no PID reset
    last_xyz = pd_xyz
    # iter5: XY hold continues from Phase 3b state
    px_prev  = pd_xyz[0]
    py_prev  = pd_xyz[1]
    vx_filt  = 0.0
    vy_filt  = 0.0
    _j("phase4_extpos_warmup", {"start_thrust": pd_thrust,
                                  "z_target": z_target,
                                  "max_s": EXTPOS_WARMUP_MAX_S,
                                  "vz_stable_tol": EKF_VZ_STABLE_M_S,
                                  "ticks_req": EKF_STABLE_TICKS_REQ,
                                  "mode": "smart_convergence"})
    _fr_event("phase", "p4_smart_warmup_start")
    _extpos_reset_gate()      # reset reject streak — Kalman is OK from PD lock

    ekf_z_prev = None
    stable_streak = 0
    converged = False
    n_ticks = int(EXTPOS_WARMUP_MAX_S * 1000 / TICK_MS)
    for ti in range(n_ticks):
        # Try a fresh PnP this tick
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            z_now = pz
            last_xyz = (px, py, pz)
            _extpos_force_z(pz)
        elif n >= 4:
            # iter4 fix C: PAD VISIBLE (n>=4) but PnP returned invalid
            # pose → feed cf2 Kalman default 0.5m via z-only ExtPos.
            # Operator: "doar o constanta, 0.5m default last resort".
            z_now = z_prev
            _extpos_force_z(Z_FALLBACK_M)
        else:
            # iter4 fix B: NO stale-fallback _extpos(last_xyz).  When
            # PnP fails AND n<4, leave Kalman to IMU integration for
            # 33 ms (~0.06 mm drift), no harm.
            z_now = z_prev

        # iter3: continue PID with inherited integral.
        dt_s = TICK_MS / 1000.0
        vz_raw = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt
        err_z = z_target - z_now
        i_term = KI_THRUST_PER_M_S * err_z_integral
        thrust_raw = (T_HOVER_NOMINAL
                      + int(KP_THRUST_PER_M * err_z
                            + i_term
                            - KD_THRUST_PER_M_PER_S * vz_filt))
        clipped = False
        if thrust_raw < T_MIN_U16:
            thrust = T_MIN_U16; clipped = True
        elif thrust_raw > T_MAX_HOLD_U16:
            thrust = T_MAX_HOLD_U16; clipped = True
        else:
            thrust = thrust_raw
        if not clipped and n >= 4 and pz > 0.0:
            err_z_integral += err_z * dt_s
            if err_z_integral >  I_INTEGRAL_MAX: err_z_integral =  I_INTEGRAL_MAX
            if err_z_integral < -I_INTEGRAL_MAX: err_z_integral = -I_INTEGRAL_MAX

        # iter6: XY hold removed from Phase 4 too (same reason as Phase 3b).
        _rpyt(0.0, 0.0, 0.0, thrust)
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            _extpos_force_z(pz)
            last_xyz = (px, py, pz)
        # FR per-tick stream — see when PnP / ekf diverge.
        _fr_pnp_ekf(px if (n >= 4 and pz > 0.0) else last_xyz[0],
                     py if (n >= 4 and pz > 0.0) else last_xyz[1],
                     pz if (n >= 4 and pz > 0.0) else last_xyz[2],
                     n, thrust)

        # ── Kalman stability check via finite-diff on ekf_z ──
        ekf_x = ekf_y = ekf_z_now = None
        try:
            p = sentai.crazy.pose()
            if p is not None and len(p) >= 3:
                ekf_x = float(p[0]); ekf_y = float(p[1]); ekf_z_now = float(p[2])
        except (AttributeError, RuntimeError, TypeError):
            pass

        if ekf_z_now is not None and ekf_z_prev is not None:
            ekf_vz = (ekf_z_now - ekf_z_prev) / dt_s
            if abs(ekf_vz) < EKF_VZ_STABLE_M_S:
                stable_streak += 1
            else:
                stable_streak = 0
            _fr_scalar("ekf_vz", ekf_vz)
            _fr_scalar("ekf_stable_streak", stable_streak)
        ekf_z_prev = ekf_z_now

        if (ti % 8) == 0:
            _j("warmup_tick", {"t": ti, "n": n,
                                "pnp_xyz": (px, py, pz),
                                "ekf_xyz": (ekf_x, ekf_y, ekf_z_now),
                                "streak": stable_streak})

        if stable_streak >= EKF_STABLE_TICKS_REQ:
            converged = True
            _fr_event("phase", "p4_smart_warmup_converged")
            _j("phase4_extpos_warmup", {"converged": True, "tick": ti,
                                          "last_xyz": last_xyz,
                                          "ekf_z": ekf_z_now,
                                          "stable_streak": stable_streak,
                                          "final_i_integral": err_z_integral,
                                          "xy_drift_cm": (
                                              (last_xyz[0]-x0)*100.0,
                                              (last_xyz[1]-y0)*100.0)})
            return last_xyz, vz_filt, err_z_integral, x0, y0

        sentai.rtos.sleep_ms(TICK_MS)
    _fr_event("phase", "p4_smart_warmup_timeout")
    _j("phase4_extpos_warmup", {"timed_out": True,
                                  "max_s": EXTPOS_WARMUP_MAX_S,
                                  "last_xyz": last_xyz,
                                  "final_streak": stable_streak,
                                  "final_i_integral": err_z_integral,
                                  "xy_drift_cm": (
                                      (last_xyz[0]-x0)*100.0,
                                      (last_xyz[1]-y0)*100.0)})
    return last_xyz, vz_filt, err_z_integral, x0, y0


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
    _fr_event("phase", "p5_handoff_start")

    # iter13 MAJOR FIX: use takeoff() instead of go_to() to engage HL.
    # Root cause (operator-found 2026-05-23): planner.state==IDLE
    # because mission used RPYT throughout, never engaging HL.  Then
    # plan_current_goal() returns traj_eval_invalid() = pos NaN, and
    # plan_go_to_from() computes trajectory polynomials from NaN curr
    # → cf2 PID reads NaN setpoint → motor commands undefined → yaw
    # spike + crash.  See planner.c:106 plan_current_goal +
    # pptraj.c:280 traj_eval_invalid.
    #
    # plan_takeoff() (planner.c:149) takes curr_pos EXPLICITLY from cf2
    # Kalman state in the firmware handler (crtp_commander_high_level.c)
    # — same meta-command pattern as go_to, but with valid trajectory
    # start.  use_current_yaw=1 takes cf2 yaw (gyro) as target → no
    # rotation commanded.
    try:
        sentai.crazy.takeoff(Z_HOLD, 0.8, 0.0, 1)  # iter16: dur 2.0→0.8 to minimize HL/mission setpoint race
        _j("takeoff_engage", {"height": Z_HOLD, "dur": 0.8})
        _fr_event("cmd", "takeoff_engaged height=%.2f dur=0.8 use_current_yaw=1" %
                   Z_HOLD)
    except (AttributeError, RuntimeError) as e:
        _j("takeoff_fail", {"err": str(e)})
        _fr_event("err", "takeoff_fail")

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
    _fr_event("cmd", "notifySetpointsStop_sent priority=2_to_1")

    # Step 4: pump ExtPos at 30 Hz while go_to executes.  HL writes
    # its setpoints at ~100 Hz from cf2 firmware side; we don't compete.
    n_ticks = int(1.0 * 1000 / TICK_MS)   # iter15: 2.5s→1.0s to minimize IMU drift window
    for ti in range(n_ticks):
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            _extpos_force_z(pz)
        # FR per-tick stream — track exactly when ekf diverges from PnP.
        _fr_pnp_ekf(px, py, pz, n)
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
    _fr_event("phase", "p5_handoff_done")


def _phase4_5_axis_id_rpyt(pd_state):
    """s194 iter6 — SIMPLIFIED OPEN-LOOP axis ID (no PD-park).

    Operator path 2026-05-24: PD-park (iter1-5) was unstable due to (1)
    PnP velocity jitter ±1.5 m/s spurious feedback, (2) wrong direction
    for 90° mount with naive identity assumption.

    Strategy: discover R via simple open-loop pulses, same algorithm as
    s193 iter17 but with iter3 PID Z in background.  No XY closed loop.
    Drone just responds to pulses; PnP captures positions; we compute R.

    For each body axis (pitch, then roll):
      1. Capture baseline p0 (PnP median over 0.5 s, drone level)
      2. +pulse (2°, 150 ms)
      3. -brake (−2°, 100 ms — partial cancellation)
      4. Coast 0.8 s (drone settles via cf2 attitude PID; some drift)
      5. Capture p_axis (PnP median 0.5 s)
      6. d_axis = (p_axis − p0)  (NOT normalized yet — we want magnitude)

    Then:
      bx_pad = d_pitch / |d_pitch|       (body+x direction in pad-XY)
      by_pad = Gram-Schmidt(d_roll vs bx_pad), normalize
      bz_z   = bx × by  (sign auto-detected for mount orientation)
      R = column-stack(bx_pad, by_pad, body_z)
      commit_R(R)

    Z PID continues in background through all phases via _do_tick().
    """
    import math
    # iter5/6: pd_state is 6-tuple (xyz, thrust, vz_filt, integral, x0, y0)
    pd_xyz, pd_thrust, pd_vz_filt, pd_i, x0, y0 = pd_state
    z_target = RAMP_HANDOFF_Z_TARGET
    z_prev   = pd_xyz[2]
    vz_filt  = pd_vz_filt
    err_z_integral = pd_i
    last_xyz = pd_xyz

    PULSE_DEG     = 2.0
    PULSE_DUR_S   = 0.15
    BRAKE_DUR_S   = 0.10
    SETTLE_DUR_S  = 0.8
    CAPTURE_DUR_S = 0.5
    MIN_DISP_M    = 0.010   # 10 mm minimum displacement to be valid

    _j("phase4_5_axis_id_simple", {
        "pulse_deg": PULSE_DEG, "pulse_dur_s": PULSE_DUR_S,
        "brake_dur_s": BRAKE_DUR_S, "settle_dur_s": SETTLE_DUR_S,
        "capture_dur_s": CAPTURE_DUR_S, "min_disp_m": MIN_DISP_M,
    })
    _fr_event("phase", "p4_5_axis_id_simple_start")
    _extpos_reset_gate()

    def _do_tick(roll_deg, pitch_deg):
        """Z PID + Z-only ExtPos + rpyt send.  iter6: no XY closed loop."""
        nonlocal z_prev, vz_filt, last_xyz, err_z_integral
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            z_now = pz
            last_xyz = (px, py, pz)
            _extpos_force_z(pz)
        else:
            z_now = z_prev
        dt_s = TICK_MS / 1000.0
        vz_raw = (z_now - z_prev) / dt_s
        vz_filt = VZ_LPF_ALPHA * vz_raw + (1.0 - VZ_LPF_ALPHA) * vz_filt
        err_z = z_target - z_now
        i_term = KI_THRUST_PER_M_S * err_z_integral
        thrust_raw = (T_HOVER_NOMINAL
                      + int(KP_THRUST_PER_M * err_z
                            + i_term
                            - KD_THRUST_PER_M_PER_S * vz_filt))
        clipped = False
        if thrust_raw < T_MIN_U16:
            thrust = T_MIN_U16; clipped = True
        elif thrust_raw > T_MAX_HOLD_U16:
            thrust = T_MAX_HOLD_U16; clipped = True
        else:
            thrust = thrust_raw
        if not clipped and n >= 4 and pz > 0.0:
            err_z_integral += err_z * dt_s
            if err_z_integral >  I_INTEGRAL_MAX: err_z_integral =  I_INTEGRAL_MAX
            if err_z_integral < -I_INTEGRAL_MAX: err_z_integral = -I_INTEGRAL_MAX
        _rpyt(roll_deg, pitch_deg, 0.0, thrust)
        z_prev = z_now
        return n, px, py, pz

    def _stream(roll_deg, pitch_deg, dur_s):
        n_ticks = int(dur_s * 1000 / TICK_MS)
        for _ in range(n_ticks):
            _do_tick(roll_deg, pitch_deg)
            sentai.rtos.sleep_ms(TICK_MS)

    def _capture_median(dur_s=CAPTURE_DUR_S):
        n_ticks = int(dur_s * 1000 / TICK_MS)
        xs, ys, zs = [], [], []
        for _ in range(n_ticks):
            n, px, py, pz = _do_tick(0.0, 0.0)
            if n >= 4 and pz > 0.0:
                xs.append(px); ys.append(py); zs.append(pz)
            sentai.rtos.sleep_ms(TICK_MS)
        # iter9: 3→2 minimum samples (PnP intermittent at low z + 0.5s window
        # may only yield 2 valid reads; median of 2 is still useful here).
        if len(xs) < 2:
            return None
        xs.sort(); ys.sort(); zs.sort()
        return (xs[len(xs)//2], ys[len(ys)//2], zs[len(zs)//2])

    # ── Baseline ──────────────────────────────────────────────────────
    p0 = _capture_median()
    if p0 is None:
        _j("axis_id_simple", "FAIL_baseline_pnp")
        _fr_event("phase", "p4_5_axis_id_FAIL_p0")
        return False
    _j("axis_id_p0", {"pnp": p0})

    # ── Pitch axis: +pulse → brake → settle → capture ────────────────
    _stream(0.0, +PULSE_DEG, PULSE_DUR_S)
    _stream(0.0, -PULSE_DEG, BRAKE_DUR_S)
    _stream(0.0,  0.0,       SETTLE_DUR_S)
    p_pitch = _capture_median()
    if p_pitch is None:
        _j("axis_id_simple", "FAIL_pitch_capture")
        _fr_event("phase", "p4_5_axis_id_FAIL_pitch")
        return False
    _j("axis_id_p_pitch", {"pnp": p_pitch})

    # ── Roll axis: +pulse → brake → settle → capture ────────────────
    _stream(+PULSE_DEG, 0.0, PULSE_DUR_S)
    _stream(-PULSE_DEG, 0.0, BRAKE_DUR_S)
    _stream( 0.0,       0.0, SETTLE_DUR_S)
    p_roll = _capture_median()
    if p_roll is None:
        _j("axis_id_simple", "FAIL_roll_capture")
        _fr_event("phase", "p4_5_axis_id_FAIL_roll")
        return False
    _j("axis_id_p_roll", {"pnp": p_roll})

    # ── Compute body axes in pad frame ───────────────────────────────
    dx_p = p_pitch[0] - p0[0]
    dy_p = p_pitch[1] - p0[1]
    mag_p = math.sqrt(dx_p*dx_p + dy_p*dy_p)
    dx_r = p_roll[0]  - p0[0]
    dy_r = p_roll[1]  - p0[1]
    mag_r = math.sqrt(dx_r*dx_r + dy_r*dy_r)
    _j("axis_id_vectors", {
        "pitch_disp": (dx_p, dy_p), "mag_p": mag_p,
        "roll_disp":  (dx_r, dy_r), "mag_r": mag_r,
    })
    if mag_p < MIN_DISP_M or mag_r < MIN_DISP_M:
        _j("axis_id_simple", "FAIL_small_displacement")
        _fr_event("phase", "p4_5_axis_id_FAIL_disp")
        return False

    bx_pad = (dx_p / mag_p, dy_p / mag_p)
    # Gram-Schmidt: orthogonalize roll against pitch
    dot = (dx_r * bx_pad[0] + dy_r * bx_pad[1])
    by_perp = (dx_r - dot * bx_pad[0], dy_r - dot * bx_pad[1])
    by_mag = math.sqrt(by_perp[0]**2 + by_perp[1]**2)
    if by_mag < MIN_DISP_M:
        _j("axis_id_simple", {"fail": "roll_collinear_with_pitch",
                                "abs_dot": abs(dot/mag_r)})
        _fr_event("phase", "p4_5_axis_id_FAIL_collinear")
        return False
    by_pad = (by_perp[0] / by_mag, by_perp[1] / by_mag)

    # body_z component (sign from cross product in XY plane)
    bz_z = bx_pad[0] * by_pad[1] - bx_pad[1] * by_pad[0]

    # R_body_to_pad row-major
    R = (
        bx_pad[0], by_pad[0], 0.0,
        bx_pad[1], by_pad[1], 0.0,
        0.0,       0.0,       bz_z,
    )
    try:
        rc = sentai.calib.commit_R(R)
    except (AttributeError, RuntimeError, ValueError) as e:
        rc = -99
        _j("axis_id_commit_err", str(e)[:120])

    theta_p = math.atan2(bx_pad[1], bx_pad[0]) * 57.29577951
    theta_r = math.atan2(by_pad[1], by_pad[0]) * 57.29577951
    theta_delta = theta_r - theta_p
    while theta_delta >  180.0: theta_delta -= 360.0
    while theta_delta < -180.0: theta_delta += 360.0
    # Compute |dot| of unnormalized disp vectors (collinearity diagnostic)
    abs_dot_pre_gs = abs(dx_p * dx_r + dy_p * dy_r) / (mag_p * mag_r)
    _j("axis_id_R_committed", {
        "R": list(R),
        "rc_commit": rc,
        "theta_pitch_deg": theta_p,
        "theta_roll_deg":  theta_r,
        "theta_delta_deg": theta_delta,
        "abs_dot_unit_pre_gs": abs_dot_pre_gs,
        "bz_z": bz_z,
        "mag_p": mag_p, "mag_r": mag_r,
    })
    _fr_event("phase",
               "p4_5_axis_id_simple_done θ_p=%.1f° θ_r=%.1f° Δ=%.1f° |dot|=%.2f bz=%+.2f rc=%d" %
               (theta_p, theta_r, theta_delta, abs_dot_pre_gs, bz_z, rc))
    return True

def _phase5_5_learn_R():
    """iter14 (operator-requested 2026-05-23): MP-side axis ID via 2
    body-velocity moves on body_x.  Commits the learned R via
    sentai.calib.commit_R() so orchestrator's SAMPLE has a sensible
    R_cam_to_body starting point instead of identity.

    Algorithm:
      1. Hover center, capture pnp_p0
      2. Stream hover(vx=+0.05, vy=0, z=Z_HOLD) for 1s
         → drone moves +body_x by ~5cm in body frame
      3. Settle 0.5s, capture pnp_p1
      4. body_x_in_pad = p1 - p0  (direction vector)
      5. Rotation angle θ = atan2(body_x_in_pad_y, body_x_in_pad_x)
      6. R_body_to_pad = [[cosθ,-sinθ,0],[sinθ,cosθ,0],[0,0,-1]]
         (z flip for camera-down mount)
      7. commit_R(R)

    Returns True on success, False if not enough valid PnP samples.
    Skips R commit on failure — orchestrator uses defaults.
    """
    import math
    _j("phase5_5_axis_id", "start")
    _fr_event("phase", "p5_5_axis_id_start")

    def _capture_pnp_median(n=8):
        xs = []; ys = []; zs = []
        for _ in range(n):
            nm, x, y, z = _try_pnp()
            if nm >= 4 and z > 0.0:
                xs.append(x); ys.append(y); zs.append(z)
            sentai.rtos.sleep_ms(40)
        if len(xs) < 3:
            return None
        xs.sort(); ys.sort(); zs.sort()
        return (xs[len(xs)//2], ys[len(ys)//2], zs[len(zs)//2])

    def _stream_hover_z_extpos(vx, vy, dur_s):
        n_ticks = int(dur_s * 1000 / TICK_MS)
        for _ in range(n_ticks):
            try:
                sentai.crazy.hover(vx, vy, 0.0, Z_HOLD)
            except (AttributeError, RuntimeError):
                pass
            # Keep cf2 Kalman z anchored throughout (z-only ExtPos).
            nm, px, py, pz = _try_pnp()
            if nm >= 4 and pz > 0.0:
                _extpos_force_z(pz)
            sentai.rtos.sleep_ms(TICK_MS)

    # ── 1. Hover at center, capture starting position ─────────────────
    _stream_hover_z_extpos(0.0, 0.0, 0.8)
    p0 = _capture_pnp_median()
    _j("axis_id_p0", {"pnp": p0})

    # ── 2. Move body+x for 1s @ 5cm/s ─────────────────────────────────
    _stream_hover_z_extpos(+0.05, 0.0, 1.0)
    _stream_hover_z_extpos(0.0, 0.0, 0.5)
    p1 = _capture_pnp_median()
    _j("axis_id_p1", {"pnp": p1, "cmd": "+body_x 0.05*1s"})

    # ── 3. Compute body_x_in_pad direction ────────────────────────────
    if p0 is None or p1 is None:
        _j("axis_id", "FAIL_no_pnp")
        _fr_event("phase", "p5_5_axis_id_FAIL_pnp")
        return False

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    mag = math.sqrt(dx*dx + dy*dy)
    _j("axis_id_displacement", {"dx_pad": dx, "dy_pad": dy, "mag": mag})

    if mag < 0.01:
        # Drone didn't move enough → can't determine direction
        _j("axis_id", {"fail": "displacement_too_small"})
        _fr_event("phase", "p5_5_axis_id_FAIL_small")
        return False

    cos_t = dx / mag
    sin_t = dy / mag
    theta_deg = math.atan2(sin_t, cos_t) * 57.29577951
    _fr_scalar("axis_id_theta_deg", theta_deg)

    # ── 4. Construct R_body_to_pad assuming pure z rotation ───────────
    # R takes body vector and outputs pad-world vector.  Camera mount
    # also flips z (drone is below, camera looks down).  For axis ID
    # we assume the rotation is pure yaw around z.
    R = (
        cos_t, -sin_t, 0.0,
        sin_t,  cos_t, 0.0,
        0.0,    0.0,  -1.0,
    )
    try:
        rc = sentai.calib.commit_R(R)
    except (AttributeError, RuntimeError, ValueError) as e:
        rc = -99
        _j("axis_id_commit_err", str(e)[:120])
    _j("axis_id_done", {"theta_deg": theta_deg,
                         "cos_t": cos_t, "sin_t": sin_t,
                         "rc_commit": rc})
    _fr_event("phase",
               "p5_5_axis_id_done theta=%.1f° rc=%d" % (theta_deg, rc))
    return True


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
    _fr_event("phase", "p6_bringup_start")
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
    # s193 iter2: poll @ 30 Hz so we can ALSO stream mission-side ExtPos
    # in parallel with the orchestrator's own ExtPos.  Critical for
    # cf2 EKF anchoring through SAMPLE phase, where orchestrator only
    # sends ExtPos during the 0.5s capture windows (out of 14s total),
    # leaving ~12s of pure hover() with IMU-only state estimation →
    # drone drifts off pad ([[s190]] 22-May journal: "drone wanders
    # off pad after handoff").  Two ExtPos streams (mission @30Hz +
    # orchestrator's bursty) don't conflict: both encode the same
    # PnP-derived drone position, so cf2 Kalman just gets more
    # frequent updates from a consistent source.
    POLL_FAST_MS = 33
    DIAG_DECIM   = 3       # log diagnostic every 3 ticks ≈ 10 Hz (was 10 Hz)
    diag_ti = 0
    safety_emitted = False
    while t_acc_ms < int(PHASE_TIMEOUT_S * 1000):
        if sentai.calib.bringup_is_done():
            done = True
            break
        phase = sentai.calib.bringup_get_phase()
        if phase != last_phase:
            _j("bringup_phase", {"phase": phase,
                                  "name": PHASE_NAMES.get(phase, "?")})
            _fr_event("bringup_phase", PHASE_NAMES.get(phase, "?"))
            _fr_scalar("bringup_phase_id", phase)
            last_phase = phase

        # ── Mission ExtPos stream — keeps cf2 Kalman anchored to PnP
        # through SAMPLE phase gaps.  PnP via get_drone_pose_tuple
        # (W19-T6b — internal correspondence + Kabsch + yaw-anchor
        # mirror picker, handles the 7-marker layout from
        # set_marker_world).
        n, px, py, pz = _try_pnp()
        if n >= 4 and pz > 0.0:
            _extpos_force_z(pz)
        # FR per-tick stream — 30 Hz time-series of pnp vs ekf vs
        # bringup phase.  Lets us pinpoint the EXACT tick the drone
        # starts misbehaving (loss of PnP, ekf vs pnp divergence,
        # phase transition coincident with chaos).
        _fr_pnp_ekf(px, py, pz, n)

        # Cheap per-tick safety check — emit an FR event the FIRST tick
        # the abort flag goes True, so post-mortem can pinpoint it
        # against the FR scalar stream.
        try:
            aborted_now = sentai.safety.aborted()
        except (AttributeError, RuntimeError):
            aborted_now = -1
        if aborted_now is True and not safety_emitted:
            _fr_event("safety", "safety_aborted_TRUE phase=%s" %
                      PHASE_NAMES.get(phase, "?"))
            safety_emitted = True

        # Sample marker count + EKF pose for diagnostic.  Decimated to
        # ~10 Hz to keep journal size reasonable.
        if diag_ti >= DIAG_DECIM:
            diag_ti = 0
            try:
                n_cache = sentai.markers.get_count()
            except (AttributeError, RuntimeError):
                n_cache = -1
            try:
                p = sentai.crazy.pose()
                ekf_xyz = (p[0], p[1], p[2]) if (p is not None and len(p) >= 3) else (-1, -1, -1)
            except (AttributeError, RuntimeError, TypeError):
                ekf_xyz = (-1, -1, -1)
            aborted = aborted_now
            _j("bringup_tick", {
                "t_ms":   t_acc_ms,
                "phase":  phase,
                "n_pnp":  n,
                "pnp_xyz": (px, py, pz) if n >= 4 else (-1, -1, -1),
                "n_cache": n_cache,
                "ekf_xyz": ekf_xyz,
                "safety_aborted": aborted,
            })
        diag_ti += 1

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
    _j("mission_start", {"wbs": "OP-S10-W21-T15", "experiment": "s194"})

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

        # iter5: PID altitude hold + XY anchor hold.  Returns 6-tuple
        # (xyz, thrust, vz_filt, err_z_integral, x0, y0).
        pd_state = _phase3b_pd_hold(last_pnp)
        last_pnp = pd_state[0]

        # ExtPos warmup — same PID + XY hold continues from Phase 3b state.
        # Returns (last_xyz, vz_filt, err_z_integral, x0, y0).
        warmup_xyz, vz_filt_post, i_post, x0_post, y0_post = (
            _phase4_extpos_warmup(pd_state))
        # Repack pd_state for axis ID (6-tuple, same shape).
        pd_state = (warmup_xyz, pd_state[1], vz_filt_post, i_post,
                    x0_post, y0_post)

        # iter17: RPYT-based axis ID BEFORE handover (operator-requested).
        # Drone stays in proven-stable RPYT mode during axis ID.
        axis_id_ok = _phase4_5_axis_id_rpyt(pd_state)
        summary["axis_id_ok"] = bool(axis_id_ok)

        # iter3 (operator request 2026-05-24): "pentru moment oprim
        # handoff, facem experimentul doar pana acolo.  Daca am
        # identificat corect axele si drona e stabila in zbor consideram
        # succes".  Skip phase5/phase6 until axis ID is proven reliable.
        # Just land cleanly.
        if axis_id_ok:
            summary["status"] = "AXIS_ID_OK"
        else:
            summary["status"] = "AXIS_ID_FAIL"

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
