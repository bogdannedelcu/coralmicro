# s174 mission — T21 step-wise yaw rotation via cf2 HL go_to.
#
# Operator-proposed 2026-05-18: continuous yaw_rate command via
# hover() + ExtPose quat is fundamentally unstable in our setup
# (positive feedback through VPE — see T19/T20 notes in wbs.md).
# T21 replaces it with the SOTA approach: many small steps using
# cf2's HL trajectory generator (go_to with yaw target = absolute
# angle in radians).  Each step:
#   1. Issue go_to(x=0, y=0, z=Z_HOLD, yaw=cumulative_target_rad,
#                   duration=STEP_DUR, relative=0)
#      cf2 runs a minimum-jerk trajectory to that yaw target +
#      holds position at world (0,0,Z_HOLD).
#   2. Wait STEP_DUR + SETTLE for the trajectory to complete.
#   3. Check safety (n_dets >= 4).  Abort on safety trigger.
#   4. Next step.
#
# Total: N_STEPS × STEP_DEG = 360° rotation over
# (N_STEPS × (STEP_DUR + SETTLE)) seconds.
#
# Stays in HL mode the whole flight; no hover() conflict.

import sentai
import math

# ── Mission config ────────────────────────────────────────────────
Z_HOLD             = 0.60   # m above ground for hover
TAKEOFF_DUR        = 2.0    # s
LAND_DUR           = 2.0    # s
SETTLE_S           = 1.0    # s after takeoff settle

# Step-wise rotation parameters (operator 2026-05-18: smaller steps
# to minimise z drift between go_to calls).
N_STEPS            = 36             # 36 × 10° = 360°
STEP_DEG           = 10.0
STEP_DUR_S         = 0.8            # cf2 trajectory time per step
SETTLE_PER_STEP_S  = 0.4            # dwell after each go_to

# Known marker world positions (must match KNOWN_POS_M in
# sentai_calib_task.cc + SDF aruco_id0..3).
KNOWN_POS = [
    (+0.12, +0.20, 0.010),    # id 0
    (-0.12, +0.20, 0.010),    # id 1
    (-0.12, -0.20, 0.010),    # id 2
    (+0.12, -0.20, 0.010),    # id 3
]

GRID_DX_M    = 0.20
GRID_DY_M    = 0.15
MARKER_SIZE  = 0.094

SAFETY_N_MIN      = 4
SAFETY_MAX_LOSS_S = 4.0
POLL_MS           = 100

FR_FRAMES_DIR  = "/tmp/s174_yaw_aruco_baseline/fr_current/frames"
FR_EVENTS_FILE = "/tmp/s174_yaw_aruco_baseline/fr_current/events.csv"
FR_SCALARS_FILE = "/tmp/s174_yaw_aruco_baseline/fr_current/scalars.csv"
JOURNAL_NAME   = "mission_yaw_step_journal.txt"


def _ser_val(v):
    if v is None: return "null"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, float)): return str(v)
    if isinstance(v, str): return '"' + v + '"'
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        kvs = ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        return "{" + ", ".join(kvs) + "}"
    return '"<%s>"' % type(v).__name__


def _journal(event, payload=None):
    try: sentai.sim.journal_write(event, payload)
    except Exception: pass


def _pump_vpe():
    """Pull latest PnP markers, average drone-world position
    (R_cam_to_body ≈ identity for SIM downward camera), and send
    ExtPos to cf2 EKF.  Position-only — no quaternion (T19/T20
    showed quaternion feedback causes resonance during yaw)."""
    try:
        mks = sentai.aruco.get_latest()
    except Exception:
        return None
    if not mks:
        return None
    sx = 0.0; sy = 0.0; sz = 0.0; n = 0
    for m in mks:
        mid = m.get("marker_id", -1)
        if mid < 0 or mid >= len(KNOWN_POS):
            continue
        tv = m.get("tvec_cam")
        if not tv or len(tv) < 3:
            continue
        mw = KNOWN_POS[mid]
        # drone_world = marker_world - R_cam_to_body * tvec_cam.
        # R ≈ identity for our downward camera + drone yaw=0
        # convention (yaw is small here since cf2 in HL go_to
        # tracks our target yaw — approximation good enough for
        # the position fix to stop the runaway z climb).
        sx += mw[0] - tv[0]
        sy += mw[1] - tv[1]
        sz += mw[2] - tv[2]
        n += 1
    if n == 0:
        return None
    dx = sx / n
    dy = sy / n
    dz = sz / n
    # T21 trial 1: send ExtPos position-only.  cf2 yaw measurement
    # missing → HL go_to(yaw) didn't propagate; drone only yawed 7°.
    # T21 trial 2: ExtPose with identity quat.  cf2 accepts yaw=0
    # measurement but then OVER-ROTATES (1049° for 6 commanded steps)
    # AND climbs altitude (13 m) — different feedback path.  Neither
    # ExtPos nor ExtPose-identity works for HL go_to step-wise yaw in
    # CrazySim.  This requires deeper cf2 firmware investigation.
    sentai.crazy.send_extpos(dx, dy, dz)
    return (dx, dy, dz, n)


def run():
    summary = {
        "status":               "RUNNING",
        "phases_done":          [],
        "errors":               [],
        "is_done":              False,
        "aborted_by_safety":    False,
        "step_dur_s":           STEP_DUR_S,
        "step_deg":             STEP_DEG,
        "n_steps":              N_STEPS,
        "steps_completed":      0,
        "last_target_deg":      0.0,
    }
    _j = _journal

    try:
        try: sentai.sim.journal_open(JOURNAL_NAME)
        except Exception: pass
        sentai.camera.init()
        _j("camera_init_ok", {})

        _j("fr_init",         {"rc": sentai.fr.init()})
        _j("fr_open_frames",  {"rc": sentai.fr.open("frames",  FR_FRAMES_DIR)})
        _j("fr_open_events",  {"rc": sentai.fr.open("events",  FR_EVENTS_FILE)})
        _j("fr_open_scalars", {"rc": sentai.fr.open("scalars", FR_SCALARS_FILE)})
        _j("fr_task_start",   {"rc": sentai.fr.task_start()})

        _j("safety_init", {"rc": sentai.safety.init()})

        _j("calib_set_context",
           {"rc": sentai.calib.set_context(Z_HOLD, GRID_DX_M,
                                            GRID_DY_M, MARKER_SIZE)})
        summary["phases_done"].append("inited")

        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["errors"].append("crazy.init rc=%d" % rc)
            summary["status"] = "INIT_FAIL"
            return summary

        _j("crazy_arm",     {"rc": sentai.crazy.arm()})
        _j("crazy_takeoff", {"rc": sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        sentai.rtos.sleep_ms(int(SETTLE_S * 1000))
        _j("takeoff_settled", {})
        summary["phases_done"].append("takeoff")

        # Stay in HL mode — do NOT call hl_stop.  go_to commands
        # below produce minimum-jerk trajectories handled internally
        # by cf2's HL commander, which also enforces position hold
        # at x=0, y=0 between yaw transitions.

        _j("safety_enable_aruco",
           {"rc": sentai.safety.enable_aruco(SAFETY_N_MIN,
                                              SAFETY_MAX_LOSS_S)})
        _j("safety_task_start", {"rc": sentai.safety.task_start()})
        summary["phases_done"].append("safety_armed")

        # ── Step-wise rotation loop ────────────────────────────────
        summary["phases_done"].append("rotation_started")
        for step in range(N_STEPS):
            target_deg = (step + 1) * STEP_DEG
            target_rad = target_deg * math.pi / 180.0
            _j("step_go_to",
               {"step": step + 1, "of": N_STEPS,
                "target_deg": target_deg,
                "target_rad": target_rad})
            # Absolute target: x=0, y=0, z=Z_HOLD, yaw=target_rad.
            # relative=0 means cf2 interprets all as absolute world
            # coordinates from take-off origin.
            rc = sentai.crazy.go_to(0.0, 0.0, Z_HOLD,
                                     target_rad, STEP_DUR_S, 0, 0, 0)
            _j("step_go_to_rc", {"step": step + 1, "rc": rc})

            # Wait for trajectory + settle.  Pump VPE ExtPos at
            # ~30 Hz throughout — without this cf2's z estimate
            # drifts (no baro fusion in SITL) and drone climbs.
            sleep_total_ms = int((STEP_DUR_S + SETTLE_PER_STEP_S) * 1000)
            poll_ms = 33
            slept = 0
            while slept < sleep_total_ms:
                _pump_vpe()
                if sentai.safety.aborted():
                    _j("safety_aborted_in_step",
                       {"step": step + 1,
                        "reason": sentai.safety.reason()})
                    summary["aborted_by_safety"] = True
                    summary["last_target_deg"]   = target_deg
                    raise StopIteration   # break double loop
                sentai.rtos.sleep_ms(poll_ms)
                slept += poll_ms

            summary["steps_completed"] = step + 1
            summary["last_target_deg"] = target_deg

        summary["is_done"] = True

    except StopIteration:
        pass     # safety abort — fall through to land
    except Exception as e:
        summary["status"] = "EXC"
        summary["errors"].append("exception: " + str(e))

    # ── Land ──────────────────────────────────────────────────────
    try:
        _j("crazy_land", {"rc": sentai.crazy.land(0.0, LAND_DUR)})
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        summary["phases_done"].append("landed")
    except Exception as e:
        summary["errors"].append("land exception: " + str(e))

    # ── Teardown ──────────────────────────────────────────────────
    try:
        _j("safety_task_stop", {"rc": sentai.safety.task_stop()})
        _j("safety_disable",   {"rc": sentai.safety.disable_aruco()})
        _j("fr_task_stop",     {"rc": sentai.fr.task_stop()})
        _j("fr_close_frames",  {"rc": sentai.fr.close("frames")})
        _j("fr_close_events",  {"rc": sentai.fr.close("events")})
        _j("fr_close_scalars", {"rc": sentai.fr.close("scalars")})
        summary["phases_done"].append("teardown")
    except Exception as e:
        summary["errors"].append("teardown exception: " + str(e))

    if summary["status"] == "RUNNING":
        summary["status"] = "OK"
    _j("mission_done", summary)

    return summary
