# mission_yaw_aruco.py — OP-S10-W14-T16 YawArucoBaseline.
#
# Operator-proposed 2026-05-18: drone hovers at z_hold using the
# autotune-identified Kp_x = Kp_y = 0.39 (X+Y closed-loop P-control)
# AND commands a continuous yaw rotation while keeping markers in
# FOV.  Validates that the calibration generalises to a more
# demanding manoeuvre (lateral hold under camera rotation).
#
# Runs INSIDE sentai_sim per [[missions-run-in-sentai-only]].
# Compute lives in C++ (sentai.calib.hold_yaw_start spawns the
# worker task at tskIDLE_PRIORITY+2).  MP only:
#   - 5 floats to start
#   - bool poll @ 10 Hz
#   - 2 floats read at end
#
# VPE format auto-switches inside the C++ worker:
#   yaw_rate==0 → ExtPose (lock yaw, T13 path)
#   yaw_rate!=0 → ExtPos (yaw managed by cf2 gyro + hover.yaw_rate)

import sentai

KP_X = 0.39
KP_Y = 0.39

Z_HOLD       = 0.90
TAKEOFF_DUR  = 2.5
LAND_DUR     = 2.5
SETTLE_S     = 4.0

GRID_DX_M    = 0.24
GRID_DY_M    = 0.40
MARKER_SIZE  = 0.125

HOLD_DUR_S    = 24.0
HOLD_VMAX     = 0.30
# Iter 3 (after cf2 firmware audit found `attitudeRate.yaw =
# -values->yawrate` — firmware NEGATES the value): use very small
# magnitude to test the hypothesis "altitude climb is proportional
# to commanded yaw_rate, not VPE format".
# Iter 1 (+30 → actual -15°/s, altitude → 2.2 m):   FAIL altitude
# Iter 2 (-15 → actual +7°/s,  altitude → 1.6 m):   FAIL altitude
# Iter 3 (-3  → actual ~+1.5°/s, expect altitude stable since
#         yaw torque is tiny → motor thrust differential minimal).
# T20  (T18+T19 stack): cmd -3 → actual +0.5°/s = 17 % authority.
#       Need ~15°/s actual to hit 360° in 24 s.  Try cmd = -90
#       (10×-ish): if ratio holds, expect actual ≈ -15°/s.
YAW_RATE_DEG_S = -3.0   # T20: calib_task auto-stops at 360° integrated rotation

SAFETY_N_MIN      = 4
SAFETY_MAX_LOSS_S = 4.0

POLL_MS         = 100
DEADLINE_MS     = int((HOLD_DUR_S + 15.0) * 1000.0)
JOURNAL_NAME    = "mission_yaw_aruco_journal.txt"
SUMMARY_NAME    = "mission_yaw_aruco_summary.json"

FR_DIR          = "/tmp/s174_yaw_aruco_baseline/fr_current"
FR_FRAMES_DIR   = FR_DIR + "/frames"
FR_EVENTS_FILE  = FR_DIR + "/events.csv"
FR_SCALARS_FILE = FR_DIR + "/scalars.csv"


def _ser_val(v):
    if v is None: return "null"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, float)): return str(v)
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, list):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        kvs = ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        return "{" + ", ".join(kvs) + "}"
    return '"<%s>"' % type(v).__name__


def _j(event, payload=None):
    try: sentai.sim.journal_write(event, payload)
    except Exception: pass


def run():
    summary = {
        "status": "INIT",
        "phases_done": [],
        "kp_x": KP_X, "kp_y": KP_Y,
        "yaw_rate_deg_s": YAW_RATE_DEG_S,
        "hold_dur_s": HOLD_DUR_S,
        "hold_max_drift_m": -1.0,
        "hold_rms_drift_m": -1.0,
        "is_done": False,
        "aborted_by_safety": False,
        "errors": [],
    }
    try:
        try: sentai.sim.journal_open(JOURNAL_NAME)
        except Exception: pass
        _j("smoke_start", {"kp_x": KP_X, "kp_y": KP_Y,
                            "yaw_rate": YAW_RATE_DEG_S,
                            "hold_dur_s": HOLD_DUR_S})

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
        _j("crazy_arm", {"rc": sentai.crazy.arm()})
        _j("crazy_takeoff", {"rc": sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        sentai.rtos.sleep_ms(int(SETTLE_S * 1000))
        _j("takeoff_settled", {})
        summary["phases_done"].append("takeoff")

        _j("crazy_hl_stop", {"rc": sentai.crazy.hl_stop()})
        for _ in range(5):
            sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
            sentai.rtos.sleep_ms(30)

        _j("safety_enable_aruco",
           {"rc": sentai.safety.enable_aruco(SAFETY_N_MIN,
                                              SAFETY_MAX_LOSS_S)})
        _j("safety_task_start", {"rc": sentai.safety.task_start()})
        summary["phases_done"].append("safety_armed")

        # ── Spawn rotating-hold C++ task
        _j("calib_hold_yaw_start",
           {"rc": sentai.calib.hold_yaw_start(KP_X, KP_Y,
                                                HOLD_VMAX, HOLD_DUR_S,
                                                YAW_RATE_DEG_S),
            "kp_x": KP_X, "kp_y": KP_Y,
            "vmax": HOLD_VMAX, "dur_s": HOLD_DUR_S,
            "yaw_rate": YAW_RATE_DEG_S})
        summary["phases_done"].append("rotation_started")

        elapsed_ms = 0
        while elapsed_ms < DEADLINE_MS:
            if sentai.calib.is_done():
                _j("calib_is_done", {"elapsed_ms": elapsed_ms})
                summary["is_done"] = True
                break
            if sentai.safety.aborted():
                _j("safety_aborted", {"reason": sentai.safety.reason()})
                summary["aborted_by_safety"] = True
                break
            sentai.rtos.sleep_ms(POLL_MS)
            elapsed_ms += POLL_MS

        _j("calib_task_stop", {"rc": sentai.calib.task_stop()})

        summary["hold_max_drift_m"] = sentai.calib.get_hold_max()
        summary["hold_rms_drift_m"] = sentai.calib.get_hold_rms()
        _j("hold_results",
           {"max_m": summary["hold_max_drift_m"],
            "rms_m": summary["hold_rms_drift_m"]})

        _j("crazy_land", {"rc": sentai.crazy.land(0.0, LAND_DUR)})
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        summary["phases_done"].append("landed")

        _j("safety_task_stop",  {"rc": sentai.safety.task_stop()})
        _j("safety_disable",    {"rc": sentai.safety.disable_aruco()})
        _j("fr_task_stop",      {"rc": sentai.fr.task_stop()})
        _j("fr_close_frames",   {"rc": sentai.fr.close("frames")})
        _j("fr_close_events",   {"rc": sentai.fr.close("events")})
        _j("fr_close_scalars",  {"rc": sentai.fr.close("scalars")})
        summary["phases_done"].append("teardown")

        summary["status"] = "OK"
        _j("mission_ok", summary)
    except Exception as e:
        summary["status"] = "EXC"
        summary["errors"].append("exception: " + str(e))
        _j("exception", {"err": str(e)})
        try: sentai.crazy.land(0.0, LAND_DUR); sentai.rtos.sleep_ms(int(LAND_DUR * 1000))
        except Exception: pass
        try: sentai.calib.task_stop()
        except Exception: pass
        try: sentai.safety.task_stop()
        except Exception: pass
        try: sentai.fr.task_stop()
        except Exception: pass
    finally:
        try: sentai.fs.write(SUMMARY_NAME, _ser_val(summary))
        except Exception as e: _j("summary_write_FAIL", {"err": str(e)})
        try: sentai.sim.journal_close()
        except Exception: pass
    return summary
