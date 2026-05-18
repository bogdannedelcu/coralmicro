# mission_flow_autotune.py — OP-S10-W14-T6 SIM smoke for the Flow
# loop autotuner.  Mirrors s170 mission shape; runs INSIDE sentai_sim
# MP per [[missions-run-in-sentai-only]].
#
# Steps:
#   1. init sentai.fr + sentai.safety + sentai.calib
#   2. sentai.calib.set_context(z, gdx, gdy, msize)
#   3. cf2 takeoff to z_hold (0.6 m)
#   4. arm safety (n_min=4, max_loss_s=1.0)
#   5. sentai.calib.task_start("x", 30.0, 0.10)
#   6. poll is_done() at 10 Hz; abort on safety
#   7. record final Kp + land + teardown
#
# Verdict (host-side post-mortem): PASS iff Kp ∈ [0.5, 5.0],
# state=DONE_OK from FR events, drone landed ≤ 10 cm of origin.

import sentai

Z_HOLD       = 0.90    # 50% above s127/s170 baseline (0.60m) — operator
                       # 2026-05-18 ("altitudine mai mare cu 50% ca sa poti
                       # avea mai mult loc de manevra")
TAKEOFF_DUR  = 2.5    # Iter #12: longer takeoff so cf2 reaches z_hold
                       # FULLY before hl_stop releases HL Commander.
LAND_DUR     = 2.5
SETTLE_S     = 4.0    # Iter #12: was 1.2 s.  GT showed cf2 still at
                       # z=0.5 m after 1.2 s — takeoff trajectory
                       # incomplete.  Longer settle gives altitude
                       # PID time to converge to 0.9 m before relay
                       # disturbs it.

# Marker geometry — matches the doubled 2026-05-18 SDF layout
# (sentai_crazysim.sdf aruco_id0..3 at ±0.12, ±0.20; 0.12 m face).
GRID_DX_M    = 0.24       # center-to-center between id0 and id1 (=2×0.12)
GRID_DY_M    = 0.40       # center-to-center between id0 and id3 (=2×0.20)
MARKER_SIZE  = 0.125      # matches sentai_aruco.cc s_marker_size_m

# Autotune knobs
AT_AXIS      = "x"
AT_DUR_S     = 30.0
# Iter #17 reverted iter #16 reduction: vmax=0.03 produced WORSE
# lateral oscillation (±18 cm), not better — at low vmax the relay
# signal drops below the drift-bias floor and the system wanders
# instead of oscillating cleanly.  ZN method has an inherent
# minimum vmax for SNR.  0.06 is the empirical sweet spot:
# converges (iter #15: Kp=1.58, Tu=2.0 s) with bounded ±6-10 cm
# oscillation.
AT_VMAX      = 0.06

# Safety
SAFETY_N_MIN      = 4
# Operator 2026-05-18 ("4 secunde e ok sa stam fara markeri in FOV
# nu e grav"): give autotune room to overshoot momentarily.  120
# frames at 30 FPS = 4.0 s — relaxed from baseline s170's 1.0 s.
SAFETY_MAX_LOSS_S = 4.0

POLL_MS         = 100
DEADLINE_MS     = int((AT_DUR_S + 10.0) * 1000.0)
JOURNAL_NAME    = "mission_flow_autotune_journal.txt"
SUMMARY_NAME    = "mission_flow_autotune_summary.json"

FR_DIR          = "/tmp/s172_flow_autotune_baseline/fr_current"
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
        "kp_x": -1.0,
        "kp_y": -1.0,
        "is_done": False,
        "aborted_by_safety": False,
        "errors": [],
    }
    try:
        try: sentai.sim.journal_open(JOURNAL_NAME)
        except Exception: pass
        _j("smoke_start", {"axis": AT_AXIS, "dur": AT_DUR_S,
                            "vmax": AT_VMAX, "z_hold": Z_HOLD})

        # ── Bring-up ────────────────────────────────────────────────
        sentai.camera.init()
        _j("camera_init_ok", {})

        # FR open BEFORE task_start so autotune push_event/scalar lands
        # in an open channel (silent no-op otherwise).
        _j("fr_init",         {"rc": sentai.fr.init()})
        # Frames channel: SafetyTask pushes a PGM per processed frame
        # → we can pinpoint the exact frame that triggered abort.
        _j("fr_open_frames",  {"rc": sentai.fr.open("frames",  FR_FRAMES_DIR)})
        _j("fr_open_events",  {"rc": sentai.fr.open("events",  FR_EVENTS_FILE)})
        _j("fr_open_scalars", {"rc": sentai.fr.open("scalars", FR_SCALARS_FILE)})
        _j("fr_task_start",   {"rc": sentai.fr.task_start()})

        _j("safety_init", {"rc": sentai.safety.init()})

        # Persistent context (operator: "altitudinea, marker grid, dimensiuni").
        _j("calib_set_context",
           {"rc": sentai.calib.set_context(Z_HOLD, GRID_DX_M,
                                            GRID_DY_M, MARKER_SIZE)})
        summary["phases_done"].append("inited")

        # ── Crazy connect + takeoff ─────────────────────────────────
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

        # Release HL Commander so the autotune's hover() commands
        # take effect (otherwise HL position-hold setpoints win).
        # OP-S10-W14 iter #11 — without this, hover() is ignored
        # post-takeoff.  After hl_stop(), CALIB worker MUST send
        # hover() at 30 Hz continuously or cf2 motors cut.
        _j("crazy_hl_stop", {"rc": sentai.crazy.hl_stop()})
        # Prime hover at z_hold before starting autotune task so the
        # cf2 Commander watchdog has fresh Generic Setpoints to keep.
        for _ in range(5):
            sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
            sentai.rtos.sleep_ms(30)

        # ── Arm safety BEFORE autotune (4-marker invariant) ────────
        _j("safety_enable_aruco",
           {"rc": sentai.safety.enable_aruco(SAFETY_N_MIN,
                                              SAFETY_MAX_LOSS_S)})
        _j("safety_task_start", {"rc": sentai.safety.task_start()})
        summary["phases_done"].append("safety_armed")

        # ── Launch autotune task ───────────────────────────────────
        _j("calib_task_start",
           {"rc": sentai.calib.task_start(AT_AXIS, AT_DUR_S, AT_VMAX)})
        summary["phases_done"].append("autotune_started")

        # ── Poll ────────────────────────────────────────────────────
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
        if not summary["is_done"] and not summary["aborted_by_safety"]:
            _j("autotune_timeout", {"elapsed_ms": elapsed_ms})

        # ── Stop task ───────────────────────────────────────────────
        _j("calib_task_stop", {"rc": sentai.calib.task_stop()})

        # ── Read results ────────────────────────────────────────────
        kp_x = sentai.calib.get_kp("x")
        kp_y = sentai.calib.get_kp("y")
        td   = sentai.calib.get_td_ms()
        summary["kp_x"] = kp_x
        summary["kp_y"] = kp_y
        summary["td_ms"] = td
        _j("calib_results", {"kp_x": kp_x, "kp_y": kp_y, "td_ms": td})
        summary["phases_done"].append("results_read")

        # ── Land + teardown ─────────────────────────────────────────
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
