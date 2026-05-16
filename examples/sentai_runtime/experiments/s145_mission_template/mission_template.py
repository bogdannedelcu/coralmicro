# mission_template.py — canonical MP mission template (Task #40).
#
# Copy this file to `build-sim/sentai_fs_root/mission_<name>.py`, rename
# the `JOURNAL_NAME`/`SUMMARY_NAME` constants, and adjust phases.  Run
# from the SIM REPL with a single line:
#
#     import mission_<name>; mission_<name>.run()
#
# Or, if the host wants the drone to actually fly in Gazebo, use the
# launcher in `run.sh` (handles SITL bring-up + summary capture).
#
# The template is INTENTIONALLY MINIMAL — it does NOT use camera, ArUco,
# pose feedback, or L6 explore FSM.  Those are added by missions that
# inherit this shape (s146+).  The job of this template is to PROVE
# that a pure-MP mission can drive cf2 SITL via sentai.crazy.* with no
# host Python in the control loop.

import sentai

# ─── Configuration ──────────────────────────────────────────────────
JOURNAL_NAME    = "mission_template_journal.txt"
SUMMARY_NAME    = "mission_template_summary.json"

TAKEOFF_HEIGHT  = 0.75    # m
TAKEOFF_DUR     = 2.0    # s
LAND_DUR        = 2.0    # s
WAYPOINT_DUR    = 3.0    # s (cf2 onboard trajectory planner takes this long)
HOVER_DWELL_MS  = 1000

# Out-and-back waypoint sequence (last point must be near origin per
# [[sim-test-must-return-home]] — the verdict gates closure).
WAYPOINTS = [
    (0.3,  0.0, TAKEOFF_HEIGHT),     # forward 30 cm
    (0.0,  0.0, TAKEOFF_HEIGHT),     # back to (0, 0)
]

# ─── Helpers ───────────────────────────────────────────────────────

def _j(event, payload):
    """Write one event to the journal.  Always called inside `run()`
    so journal_open has already happened."""
    sentai.sim.journal_write(event, payload)


def _sleep_after_cmd(dur_s, extra_ms=200):
    """Wait for cf2's onboard trajectory planner to complete a command.
    The +200 ms is settling time (cf2 drops to hover at the end of a
    HL Commander trajectory, but the wire-time of the next command is
    non-zero)."""
    sentai.rtos.sleep_ms(int(dur_s * 1000) + extra_ms)


def _go_to(x, y, z, dur=WAYPOINT_DUR):
    """Absolute go_to via cf2 HL commander.  Returns rc from send."""
    rc = sentai.crazy.go_to(x, y, z, 0.0, dur, 0, 0, 0)
    _j("go_to_cmd", {"x": x, "y": y, "z": z, "dur": dur, "rc": rc})
    _sleep_after_cmd(dur)
    return rc


def _ser_val(v):
    """Tiny JSON-ish serializer (MP embed has no `json` module).  Handles
    int / float / str / bool / None / list / dict.  Strings are
    minimally escaped (backslash + quote)."""
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, str):
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return '"' + s + '"'
    if isinstance(v, list):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        kvs = ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        return "{" + ", ".join(kvs) + "}"
    return '"<%s>"' % type(v).__name__


def _write_summary(summary):
    """Serialize summary dict and persist via sentai.fs.write so the
    host can read it after the mission terminates."""
    sentai.fs.write(SUMMARY_NAME, _ser_val(summary))


# ─── Mission entry point ───────────────────────────────────────────

def run():
    """Top-level mission entry.  Returns the summary dict so the REPL
    can `print(repr(...))` it, AND writes summary.json for the host."""
    sentai.sim.journal_open(JOURNAL_NAME)

    summary = {
        "name":             "mission_template",
        "version":          sentai.version(),
        "status":           "STARTED",
        "phases_done":      [],
        "phase_count":      0,
        "waypoints_visited": 0,
        "crazy_init_rc":    None,
        "errors":           [],
    }

    try:
        # ---- Phase 1: open transport to cf2 SITL --------------------
        rc = sentai.crazy.init()
        summary["crazy_init_rc"] = rc
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["errors"].append("crazy.init failed rc=%d" % rc)
            summary["status"] = "FAIL_INIT"
            return summary
        summary["phases_done"].append("init")
        summary["phase_count"] += 1

        # ---- Phase 2: arm ------------------------------------------
        arm_rc = sentai.crazy.arm()
        _j("crazy_arm", {"rc": arm_rc})
        sentai.rtos.sleep_ms(200)
        summary["phases_done"].append("arm")
        summary["phase_count"] += 1

        # ---- Phase 3: takeoff --------------------------------------
        t_rc = sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        _j("crazy_takeoff", {"h": TAKEOFF_HEIGHT, "dur": TAKEOFF_DUR, "rc": t_rc})
        _sleep_after_cmd(TAKEOFF_DUR)
        summary["phases_done"].append("takeoff")
        summary["phase_count"] += 1

        # ---- Phase 4: waypoint sequence ----------------------------
        for i, (x, y, z) in enumerate(WAYPOINTS):
            _j("waypoint_start", {"idx": i, "x": x, "y": y, "z": z})
            _go_to(x, y, z)
            sentai.rtos.sleep_ms(HOVER_DWELL_MS)
            summary["waypoints_visited"] += 1
            _j("waypoint_done", {"idx": i})
        summary["phases_done"].append("waypoints")
        summary["phase_count"] += 1

        # ---- Phase 5: land -----------------------------------------
        l_rc = sentai.crazy.land(0.0, LAND_DUR)
        _j("crazy_land", {"dur": LAND_DUR, "rc": l_rc})
        _sleep_after_cmd(LAND_DUR)
        summary["phases_done"].append("land")
        summary["phase_count"] += 1

        # ---- Phase 6: disarm ---------------------------------------
        d_rc = sentai.crazy.disarm()
        _j("crazy_disarm", {"rc": d_rc})
        summary["phases_done"].append("disarm")
        summary["phase_count"] += 1

        summary["status"] = "DONE"

    except Exception as e:
        # Any unhandled MP error: capture + continue to cleanup so the
        # summary is still written and the journal closed.  Host
        # verdict will see status != DONE and fail loudly.
        summary["status"] = "EXCEPTION"
        summary["errors"].append("exc: " + str(e))
        _j("exception", {"msg": str(e)})

    finally:
        # Best-effort cleanup — even on failure, leave a recoverable
        # state: write summary, close journal, drop UDP socket.
        try:
            _write_summary(summary)
        except Exception:
            pass
        try:
            sentai.sim.journal_close()
        except Exception:
            pass
        try:
            sentai.crazy.stop()
        except Exception:
            pass

    return summary
