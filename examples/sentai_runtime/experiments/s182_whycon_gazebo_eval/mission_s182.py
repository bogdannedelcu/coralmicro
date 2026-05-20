# mission_s182 — WhyCon evaluation in Gazebo SIM (anti-cheat compliant).
#
# Runs INSIDE sentai_sim per [[missions-run-in-sentai-only]].  Host
# only launches the SITL stack + does post-mortem analysis.  sentai_sim
# sees ONLY: camera frames over /tmp/sentai_cam.sock, cf2 telemetry
# over CRTP (sentai.crazy.pose), MAVLink (sentai.link).  NO ground
# truth crosses the air-gap.
#
# Flight plan:
#   1. crazy.init + arm
#   2. markers.init('whycon') with set_intrinsics + set_marker_size
#   3. takeoff to 0.6 m
#   4. Altitude sweep: hover at z ∈ {0.4, 0.5, 0.6, 0.7, 0.8, 1.0} m
#      For each altitude, dwell HOVER_TICKS frames and per-tick log:
#        - cf2 EKF pose (sentai.crazy.pose) — drone's onboard estimate
#        - For each WhyCon detection, tvec_cam / rvec_cam / pixel_cx,cy
#   5. land + disarm
#
# Host verdict.py pairs the journal with gt_recorder JSONL (drone +
# all 4 marker GT poses) and produces the X/Y/Z est-vs-GT plots.
#
# WBS: OP-S10-W19-T4 step 2 (the canonical Gazebo eval).

import sentai

JOURNAL_NAME = "mission_s182_journal.txt"
SUMMARY_NAME = "mission_s182_summary.json"

# Camera intrinsics — 320x240 downward cam in sentai_whycon world.
# Values match sim/scripts/aruco_to_vision_estimate.py + the existing
# ArUco bench convention (fx=fy=240 in pixels).
FX = 240.0
FY = 240.0
CX = 160.0
CY = 120.0

# Marker physical outer-ring diameter (metres).  PNG has 24 px white
# margin on a 256 px half-frame, mapped onto a 0.12 m box face, so
# R_world = 232/256 * 0.06 = 0.0544 m → diameter = 0.1088 m.
MARKER_DIAMETER_M = 0.1088

# Flight plan.
TAKEOFF_HEIGHT = 0.6
TAKEOFF_DUR    = 2.5
LAND_DUR       = 2.5
HOVER_WAIT_S   = 2.0    # let cf2 settle before logging
HOVER_TICKS    = 30     # ~3s @ 10 Hz logging
TICK_INTERVAL_MS = 100  # ~10 Hz logging cadence

ALT_SWEEP = [0.40, 0.50, 0.60, 0.70, 0.80, 1.00]


def _j(event, payload):
    sentai.sim.journal_write(event, payload)


def _sleep_after_cmd(dur_s, extra_ms=200):
    sentai.rtos.sleep_ms(int(dur_s * 1000) + extra_ms)


def _ser_val(v):
    if v is None:               return "null"
    if isinstance(v, bool):     return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, str):
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return '"' + s + '"'
    if isinstance(v, list):
        return "[" + ", ".join(_ser_val(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join(
            ['"%s": %s' % (k, _ser_val(val)) for k, val in v.items()]
        ) + "}"
    return '"<%s>"' % type(v).__name__


def _write_summary(summary):
    sentai.fs.write(SUMMARY_NAME, _ser_val(summary))


def _read_cf2_pose():
    """Read cf2 EKF pose snapshot via CRTP LOG (anti-cheat-compliant —
    this is what cf2 thinks its pose is, NOT ground truth).
    Returns a 7-tuple (x, y, z, qx, qy, qz, qw) or None if unavailable."""
    try:
        return sentai.crazy.pose()
    except (AttributeError, RuntimeError):
        return None


def _log_tick(tick_idx, target_alt):
    """One logging tick: run WhyCon detection on the current camera
    frame, log cf2 EKF pose + per-marker tvec_cam.  All numbers
    journalled for the host verdict to consume."""
    cf2 = _read_cf2_pose()
    n = sentai.markers.detect_from_camera()
    dets = []
    if n > 0:
        for i in range(n):
            t = sentai.markers.get_pose_tuple(i)
            if t is None:
                continue
            # (id, pixel_cx, pixel_cy, tx, ty, tz, rx, ry, rz, reproj, backend, valid)
            dets.append({
                "i":    i,
                "px":   t[1],
                "py":   t[2],
                "tx":   t[3],
                "ty":   t[4],
                "tz":   t[5],
                "rx":   t[6],
                "ry":   t[7],
                "rz":   t[8],
                "rep":  t[9],
                "v":    t[11],
            })
    _j("tick", {
        "i":      tick_idx,
        "alt":    target_alt,
        "ts_ms":  sentai.rtos.ticks_ms(),
        "cf2":    cf2,
        "n":      n,
        "dets":   dets,
    })


def run():
    sentai.sim.journal_open(JOURNAL_NAME)

    summary = {
        "name":          "mission_s182",
        "version":       sentai.version(),
        "status":        "STARTED",
        "phases_done":   [],
        "alt_sweep":     ALT_SWEEP,
        "ticks_logged":  0,
        "frames_w_det":  0,
        "errors":        [],
    }

    try:
        # ---- markers setup (FIRST — needs to be live before takeoff
        #      so the very first hover loop has something to detect).
        rc = sentai.markers.init("whycon")
        _j("markers_init", {"rc": rc, "backend": sentai.markers.backend()})
        if rc != 0:
            summary["errors"].append("markers.init('whycon') rc=%d" % rc)
            summary["status"] = "FAIL_MARKERS_INIT"
            return summary
        sentai.markers.set_intrinsics(FX, FY, CX, CY)
        sentai.markers.set_marker_size(MARKER_DIAMETER_M)
        summary["phases_done"].append("markers_init")

        # ---- crazy.init -------------------------------------------
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["errors"].append("crazy.init rc=%d" % rc)
            summary["status"] = "FAIL_CRAZY_INIT"
            return summary
        summary["phases_done"].append("crazy_init")

        sentai.crazy.arm()
        _j("crazy_arm", {})
        sentai.rtos.sleep_ms(300)

        # ---- takeoff to working altitude --------------------------
        rc = sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        _j("takeoff", {"h": TAKEOFF_HEIGHT, "rc": rc})
        _sleep_after_cmd(TAKEOFF_DUR)
        summary["phases_done"].append("takeoff")

        # ---- altitude sweep ---------------------------------------
        tick_idx = 0
        for alt in ALT_SWEEP:
            rc = sentai.crazy.go_to(0.0, 0.0, alt, 0.0, 2.0, 0, 0, 0)
            _j("goto", {"z": alt, "rc": rc})
            _sleep_after_cmd(2.0)
            # Let cf2 settle before sampling.
            sentai.rtos.sleep_ms(int(HOVER_WAIT_S * 1000))
            _j("hover_start", {"alt": alt})
            for _ in range(HOVER_TICKS):
                _log_tick(tick_idx, alt)
                tick_idx += 1
                sentai.rtos.sleep_ms(TICK_INTERVAL_MS)
            _j("hover_done", {"alt": alt, "ticks": HOVER_TICKS})
        summary["phases_done"].append("alt_sweep")
        summary["ticks_logged"] = tick_idx

        # ---- land + disarm ----------------------------------------
        rc = sentai.crazy.land(0.0, LAND_DUR)
        _j("land", {"rc": rc})
        _sleep_after_cmd(LAND_DUR)
        sentai.crazy.disarm()
        _j("disarm", {})
        summary["phases_done"].append("land")
        summary["status"] = "OK"
    except Exception as e:
        summary["errors"].append(repr(e))
        summary["status"] = "EXCEPTION"
        try:
            sentai.crazy.land(0.0, 1.5)
            sentai.rtos.sleep_ms(1800)
            sentai.crazy.disarm()
        except Exception:
            pass
    finally:
        _write_summary(summary)
        sentai.sim.journal_close()
    return summary


if __name__ == "__main__":
    print(repr(run()))
