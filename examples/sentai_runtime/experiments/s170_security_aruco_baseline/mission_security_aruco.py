# mission_security_aruco.py — OP-S10-W12-T6 SecurityArucoBaseline.
#
# Operator-named 2026-05-18 ("SecurityArucoBaseline").
# Mission runs ENTIRELY in MP inside sentai_sim per
# [[missions-run-in-sentai-only]].  Steps:
#   1. init crazy + camera + sentai.fr + sentai.safety
#   2. arm safety (aruco, n_min=4, max_loss_s=1.0)
#   3. start SafetyTask + FR drain task
#   4. arm + takeoff to 0.6 m
#   5. hover 20 s OR until sentai.safety.aborted() (poll @ 10 Hz)
#      Every second, journal a `hover_tick` AND push to sentai.fr.
#   6. land + safety teardown
#
# All progress goes to `sentai.sim.journal_*` (post-mortem if mission hangs)
# AND `sentai.fr` (multi-channel flight recorder per OP-S10-W13):
#   - frames channel: SafetyTask pushes every processed frame as PGM
#     into FR_FRAMES_DIR/t<ms>_n<n_dets>_f<seq>.pgm
#   - events channel: text events CSV (`ts_ms,type,text`)

import sentai

# ---- Knobs --------------------------------------------------------
TAKEOFF_HEIGHT     = 0.60
TAKEOFF_DUR        = 2.0
LAND_DUR           = 2.5
HOVER_DEADLINE_S   = 20
POLL_SLEEP_MS      = 100        # safety-poll cadence (10 Hz)
TICK_JOURNAL_EVERY = 10         # journal every 10 polls = ~1 s

# Lateral drift command — operator spec 2026-05-18: "ridicam drona
# 0.5-1m apoi navigam intr-o parte asa incat sa dispara markerii".
# After 3s settle, command 80 cm forward over 4s.  At z=0.6m the FOV
# is ~0.83×0.62 m so 0.4 m+ of drift moves markers fully out of view.
DRIFT_DX_M     = 0.80
DRIFT_DY_M     = 0.00
DRIFT_DUR_S    = 4.0
# Short pre-drift settle — long enough for takeoff to stabilise,
# short enough that cf2 hover noise (no VPE → ~1s till natural marker
# loss) doesn't trip safety before the FORCED drift starts.
PRE_DRIFT_HOLD_S = 0.5

SAFETY_N_MIN       = 4
SAFETY_MAX_LOSS_S  = 1.0

JOURNAL_NAME = "mission_security_aruco_journal.txt"
SUMMARY_NAME = "mission_security_aruco_summary.json"

# Flight Recorder paths.  run.sh prepares /tmp/.../fr_current as a
# symlink to the per-trial dir, so the mission hardcodes this stable
# path (FS path = only contract between host + mission).
FR_DIR          = "/tmp/s170_security_aruco_baseline/fr_current"
FR_FRAMES_DIR   = FR_DIR + "/frames"
FR_EVENTS_FILE  = FR_DIR + "/events.csv"
FR_SCALARS_FILE = FR_DIR + "/scalars.csv"


# ---- Tiny JSON serialiser (MP embed has no `json`) ----------------
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
    try:
        sentai.sim.journal_write(event, payload)
    except Exception:
        pass


def _fr_event(type_, text):
    try:
        sentai.fr.push_event(type_, text)
    except Exception:
        pass


def run():
    summary = {
        "status": "INIT",
        "errors": [],
        "phases_done": [],
        "aborted": False,
        "abort_reason": "",
        "hover_elapsed_s": 0.0,
        "loop_iters": 0,
        "tick_journal_writes": 0,
    }
    try:
        try: sentai.sim.journal_open(JOURNAL_NAME)
        except Exception: pass

        _j("smoke_start", {"takeoff_h": TAKEOFF_HEIGHT,
                            "deadline_s": HOVER_DEADLINE_S,
                            "n_min": SAFETY_N_MIN,
                            "max_loss_s": SAFETY_MAX_LOSS_S})

        # ---- Camera init -------------------------------------------
        try:
            sentai.camera.init()
            _j("camera_init", {"rc": 0})
        except Exception as e:
            _j("camera_init_FAIL", {"err": str(e)})
            summary["errors"].append("camera.init: " + str(e))

        # ---- Flight Recorder open + start (BEFORE SafetyTask so the
        #      task's first push_frame lands in an open channel) -----
        _j("fr_init",         {"rc": sentai.fr.init()})
        _j("fr_open_frames",  {"rc": sentai.fr.open("frames",  FR_FRAMES_DIR)})
        _j("fr_open_events",  {"rc": sentai.fr.open("events",  FR_EVENTS_FILE)})
        _j("fr_open_scalars", {"rc": sentai.fr.open("scalars", FR_SCALARS_FILE)})
        _j("fr_task_start",   {"rc": sentai.fr.task_start()})
        _fr_event("mission_start",
                  "n_min=%d max_loss_s=%s deadline=%ds"
                  % (SAFETY_N_MIN, SAFETY_MAX_LOSS_S, HOVER_DEADLINE_S))

        # ---- Safety prep (init only; ARM later after takeoff so
        #      cf2 takeoff-transient marker loss does not trip it) --
        _j("safety_init", {"rc": sentai.safety.init()})
        summary["phases_done"].append("safety_inited")

        # ---- Crazy connect + arm + takeoff -------------------------
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["errors"].append("crazy.init rc=%d" % rc)
            summary["status"] = "INIT_FAIL"
            return summary

        _j("crazy_arm", {"rc": sentai.crazy.arm()})
        summary["phases_done"].append("armed")
        _fr_event("phase", "armed")

        _j("crazy_takeoff", {"h": TAKEOFF_HEIGHT, "dur": TAKEOFF_DUR,
                              "rc": sentai.crazy.takeoff(TAKEOFF_HEIGHT,
                                                          TAKEOFF_DUR)})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        _j("takeoff_settled", {})
        summary["phases_done"].append("takeoff")
        _fr_event("phase", "takeoff_done h=%s" % TAKEOFF_HEIGHT)

        # ---- ARM safety NOW (post-takeoff, drone stable) ----------
        _j("safety_enable_aruco", {"rc": sentai.safety.enable_aruco(
                                     SAFETY_N_MIN, SAFETY_MAX_LOSS_S),
                                    "n_min": SAFETY_N_MIN,
                                    "max_loss_s": SAFETY_MAX_LOSS_S})
        _j("safety_task_start", {"rc": sentai.safety.task_start()})
        summary["phases_done"].append("safety_armed")
        _fr_event("phase", "safety_armed")

        # ---- DRIFT lateral to push markers out of FOV ------------
        # Operator spec 2026-05-18: forced drift so safety MUST fire.
        _j("drift_start", {"dx": DRIFT_DX_M, "dy": DRIFT_DY_M,
                            "dur_s": DRIFT_DUR_S})
        _fr_event("phase", "drift_start dx=%s dy=%s dur=%s"
                  % (DRIFT_DX_M, DRIFT_DY_M, DRIFT_DUR_S))
        # sentai.crazy.go_to(x, y, z, yaw, dur, relative=, rel_pos=, rel_yaw=)
        # Absolute target = current_drone_pose + drift.  Since cf2 EKF
        # starts at (0,0,h), target = (dx, dy, h).  go_to handles the
        # trajectory; cf2 HL Commander interpolates over `dur`.
        rc = sentai.crazy.go_to(DRIFT_DX_M, DRIFT_DY_M, TAKEOFF_HEIGHT,
                                 0.0, DRIFT_DUR_S, 0, 0, 0)
        _j("drift_cmd", {"rc": rc})
        summary["phases_done"].append("drift_commanded")
        # Don't `sleep_ms(DRIFT_DUR)` here — the hover loop below polls
        # safety at 10 Hz; if markers leave FOV during the drift, safety
        # fires within ~1 s and we abort mid-drift.

        # ---- Hover loop --------------------------------------------
        max_iters = int(HOVER_DEADLINE_S * 1000 / POLL_SLEEP_MS)
        _j("hover_loop_start", {"max_iters": max_iters,
                                 "deadline_s": HOVER_DEADLINE_S})
        _fr_event("phase", "hover_loop_start deadline_s=%d" % HOVER_DEADLINE_S)
        aborted_at = -1
        for i in range(max_iters):
            ab = sentai.safety.aborted()
            if i % TICK_JOURNAL_EVERY == 0:
                _j("hover_tick", {"iter": i,
                                   "elapsed_ms": i * POLL_SLEEP_MS,
                                   "aborted": ab})
                _fr_event("hover_tick",
                          "i=%d ms=%d aborted=%d"
                          % (i, i * POLL_SLEEP_MS, 1 if ab else 0))
                summary["tick_journal_writes"] += 1
            if ab:
                aborted_at = i
                reason = sentai.safety.reason()
                elapsed_s = (i * POLL_SLEEP_MS) / 1000.0
                _j("safety_abort_caught",
                    {"iter": i, "elapsed_s": elapsed_s, "reason": reason})
                _fr_event("safety_abort",
                          "iter=%d elapsed_s=%s reason=%s"
                          % (i, elapsed_s, reason))
                summary["aborted"] = True
                summary["abort_reason"] = reason
                summary["hover_elapsed_s"] = elapsed_s
                summary["loop_iters"] = i
                summary["phases_done"].append("aborted_mid_hover")
                break
            sentai.rtos.sleep_ms(POLL_SLEEP_MS)
        if aborted_at < 0:
            summary["hover_elapsed_s"] = float(HOVER_DEADLINE_S)
            summary["loop_iters"] = max_iters
            summary["phases_done"].append("hover_done")
            _j("hover_complete", {"elapsed_s": HOVER_DEADLINE_S})
            _fr_event("hover_complete", "elapsed_s=%d" % HOVER_DEADLINE_S)

        # ---- Land --------------------------------------------------
        _j("crazy_land", {"dur": LAND_DUR,
                           "rc": sentai.crazy.land(0.0, LAND_DUR)})
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        _j("land_settled", {})
        summary["phases_done"].append("landed")
        _fr_event("phase", "landed")

        # ---- Safety + FR teardown ----------------------------------
        _fr_event("mission_end",
                  "status=OK aborted=%d" % (1 if summary["aborted"] else 0))
        _j("safety_task_stop", {"rc": sentai.safety.task_stop()})
        _j("safety_disable",   {"rc": sentai.safety.disable_aruco()})
        _j("safety_clear",     {"rc": sentai.safety.clear()})
        _j("fr_task_stop",     {"rc": sentai.fr.task_stop()})
        # FR stats snapshot — pure scalars, no JSON.
        try:
            st = sentai.fr.stats("frames")
            _j("fr_stats_frames", {"pushes_total": st[0],
                                    "pushes_accepted": st[1],
                                    "drops_full": st[2],
                                    "writes_ok": st[3],
                                    "writes_fail": st[4],
                                    "worst_queue_depth": st[6]})
            st = sentai.fr.stats("events")
            _j("fr_stats_events", {"pushes_total": st[0],
                                    "writes_ok": st[3]})
        except Exception as e:
            _j("fr_stats_fail", {"err": str(e)})
        _j("fr_close_frames",  {"rc": sentai.fr.close("frames")})
        _j("fr_close_events",  {"rc": sentai.fr.close("events")})
        _j("fr_close_scalars", {"rc": sentai.fr.close("scalars")})
        summary["phases_done"].append("teardown")

        summary["status"] = "OK"
        _j("mission_ok", summary)
    except Exception as e:
        summary["status"] = "EXC"
        summary["errors"].append("exception: " + str(e))
        _j("exception", {"err": str(e)})
        _fr_event("exception", str(e))
        try: sentai.crazy.land(0.0, LAND_DUR); sentai.rtos.sleep_ms(int(LAND_DUR * 1000))
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
