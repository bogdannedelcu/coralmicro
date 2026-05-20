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

# Flight Recorder channel paths (per [[op-s10-w13-shipped]] pattern).
# On SIM, sentai.fr uses native POSIX fopen with literal host paths.
# Must be host-writable + parent must already exist (mkdir_p is
# single-level only).  /tmp pattern mirrors s174.
FR_DIR           = "/tmp/s182_whycon_gazebo/fr_current"
FR_FRAMES_DIR    = FR_DIR + "/frames"
FR_EVENTS_FILE   = FR_DIR + "/events.csv"
FR_SCALARS_FILE  = FR_DIR + "/scalars.csv"

# Camera intrinsics — derived from cf2 SDF downward_cam horizontal_fov
# = 1.0123 rad (cf2 model.sdf.jinja).  At 640×480 native:
#   fx = (640/2) / tan(1.0123/2) ≈ 576.6
# Bridge downsamples nearest 640×480 → 320×240 (camera_bridge_recv +
# sim_camera_grab_gray_zerocopy resize), so the effective intrinsics
# at the buffer sentai.markers actually receives are HALVED:
#   fx = fy ≈ 288 ;  cx = 160 ; cy = 120
# Caught the hard way 2026-05-20 — initial fx=240 underestimated by
# 17 % giving 0.5+ m Z bias even after the WhyCon annulus correction.
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0

# Marker physical outer-ring diameter (metres).  PNG has 24 px white
# margin on a 256 px half-frame, mapped onto a 0.12 m box face, so
# R_world = 232/256 * 0.06 = 0.0544 m → diameter = 0.1088 m.
MARKER_DIAMETER_M = 0.1088

# Flight plan.  iter-9b: raised TAKEOFF_HEIGHT 0.6→1.0 m.  Without
# VPE feedback cf2 drifts on Z and we end up below the altitude
# where the full H pattern is visible — at z<0.4 m the Y extent
# (0.34 m) starts clipping out of FOV → fewer markers detected,
# Kabsch degenerates to collinear-column subsets → X-bimodal flips.
TAKEOFF_HEIGHT = 1.0
TAKEOFF_DUR    = 3.0
LAND_DUR       = 3.0
HOVER_WAIT_S   = 2.0    # let cf2 settle before logging
HOVER_TICKS    = 30     # ~3s @ 10 Hz logging
TICK_INTERVAL_MS = 100  # ~10 Hz logging cadence

ALT_SWEEP = [0.40, 0.50, 0.60, 0.70, 0.80, 1.00]

# VPE control.  iter-6 lesson (2026-05-20): sending VPE est_z from
# per-frame median tz creates a POSITIVE FEEDBACK LOOP — cf2 EKF
# accepts the new z, the next frame's tz reflects the NEW (wrong)
# altitude, cf2 jumps further.  After 2 ticks the drone is at z=2 m
# commanded 0.4 m.  Until we have a proper Kabsch-based XYZ in-mission
# (W19-T3), VPE stays OFF.  Drone will slowly drift on X/Y/Z under
# cf2's open-loop control — still good enough for ~3-5 s of capture
# over the H pattern.
ENABLE_VPE = False

# Known marker world positions (must match sentai_whycon.sdf).  Used
# in-mission to feed VPE back into cf2 EKF — sentai_sim is the
# perception source, cf2 EKF fuses VPE with IMU, drone stays
# stabilised on Z without the host doing any vision work.
# Per [[missions-run-in-sentai-only]] this is THE pattern (the old
# host-side aruco_to_vision_estimate.py is deprecated).
# iter-8: east column shifted from +0.16 → +0.12 to break the
# X-mirror symmetry of the H pattern.  See verdict_sota.py header.
MARKER_WORLD = (
    (-0.16, +0.20, 0.005),   # NW
    (+0.12, +0.20, 0.005),   # NE
    (-0.16,  0.00, 0.005),   # W   ← cross-bar (left)
    (+0.12,  0.00, 0.005),   # E   ← cross-bar (right)
    (-0.16, -0.14, 0.005),   # SW
    (+0.12, -0.14, 0.005),   # SE
)
MARKER_NAMES = ("NW", "NE", "W", "E", "SW", "SE")


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
    Returns a tuple (x, y, z, yaw) or None if pose not yet subscribed
    or first frame not yet arrived."""
    try:
        return sentai.crazy.pose()
    except (AttributeError, RuntimeError):
        return None


def _associate_and_estimate(dets, cf2):
    """In-mission marker-to-world association + per-detection drone
    pose estimate.  Same logic as the host verdict but runs INSIDE
    sentai_sim so we can also feed VPE back to cf2 EKF.

    Strategy: forward-project each known marker into image space
    using cf2 EKF pose (cf2 looking straight down, yaw≈0), then
    match each detection to its nearest projected marker.  Discard
    detections whose nearest marker is > MAX_ASSOC_PX away or whose
    tz is wildly inconsistent with cf2.z.
    """
    if cf2 is None:
        return []
    cx_w, cy_w, cz_w, _yaw = cf2
    # Forward-project each marker.
    projected = []
    for i in range(len(MARKER_WORLD)):
        mx, my, mz = MARKER_WORLD[i]
        z = cz_w - mz
        if z <= 0.01:
            projected.append(None)
            continue
        # body_xform = (-1, 0, 0, +1) ⇒ image LEFT (+x_body) maps to
        # cam +X being on the -x_body axis.  See verdict.py.
        u = CX - FX * (mx - cx_w) / z
        v = CY + FY * (my - cy_w) / z
        projected.append((u, v, mx, my, mz, i))

    out = []
    used = set()
    for d in dets:
        if not d.get("v"):
            continue
        tz = d["tz"]
        if cz_w > 0.05:
            rel = tz / cz_w
            if rel < 0.5 or rel > 1.5:
                continue
        # Closest unused projected marker.
        best_i = -1
        best_dist2 = 80 * 80   # MAX_ASSOC_PX squared
        for p in projected:
            if p is None:
                continue
            u, v, _mx, _my, _mz, idx = p
            if idx in used:
                continue
            dx = d["px"] - u
            dy = d["py"] - v
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_i = idx
        if best_i < 0:
            continue
        used.add(best_i)
        mx, my, mz = MARKER_WORLD[best_i]
        # Drone world pose estimate.  Sign convention chosen so the
        # plot matches GT (verified visually in iter #2):
        #   drone_world.x = marker.x - tvec.x
        #   drone_world.y = marker.y - tvec.y
        #   drone_world.z = marker.z + tvec.z   (cam +Z is depth = world DOWN)
        est_x = mx - d["tx"]
        est_y = my - d["ty"]
        est_z = mz + d["tz"]
        out.append((best_i, est_x, est_y, est_z, d))
    return out


def _log_tick(tick_idx, target_alt):
    """One logging tick: run WhyCon detection on the current camera
    frame, associate detections with known markers, AVERAGE the per-
    marker drone-world estimates, send the result back to cf2 EKF
    via send_extpos so cf2 stays stabilised on Z.  Log cf2 EKF pose
    + per-marker estimates + the VPE we sent.
    """
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

    # ---- Iter-7: VPE permanently OFF until Kabsch-in-mission (W19-T3).
    # See ENABLE_VPE comment above for the iter-6 positive-feedback
    # loop that this iteration retires.
    vpe_sent = None
    # sentai.sim.journal_write auto-prefixes the line with the host
    # monotonic timestamp (see line "286089204 takeoff ..." in journal),
    # so we don't add ts_ms here — sentai.rtos has only sleep_ms in
    # the SIM build, no ticks_ms (caught the hard way 2026-05-20 in
    # the s182 first run).
    _j("tick", {
        "i":      tick_idx,
        "alt":    target_alt,
        "cf2":    cf2,
        "n":      n,
        "dets":   dets,
        "vpe":    vpe_sent,
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

        # ---- crazy pose subscribe — retry loop until CRTP TOC ready.
        # Iter-5 finding (2026-05-20): a single subscribe call at
        # arm+300ms returns rc=-3 (no TOC yet); cf2 TOC download takes
        # ~1-3 s post-link.  Retry up to 30 attempts × 200 ms = 6 s.
        sub_rc = -3
        for _try in range(30):
            try:
                sub_rc = sentai.crazy.pose_subscribe(50)
            except (AttributeError, RuntimeError) as ex:
                _j("pose_subscribe_err", {"try": _try, "err": repr(ex)})
                sub_rc = -99
            if sub_rc == 0:
                break
            sentai.rtos.sleep_ms(200)
        _j("pose_subscribe", {"rc": sub_rc, "period_ms": 50, "tries": _try + 1})
        sentai.rtos.sleep_ms(500)

        # ---- Flight Recorder — open channels so per-tick frames are
        #      saved via sentai_markers_detect_frame's sentai_fr_push_frame
        #      hook.  Used by s182_replay.py post-mortem to compare
        #      drone-side detection vs cv2 vs local numpy.
        _j("fr_init",         {"rc": sentai.fr.init()})
        _j("fr_open_frames",  {"rc": sentai.fr.open("frames",  FR_FRAMES_DIR)})
        _j("fr_open_events",  {"rc": sentai.fr.open("events",  FR_EVENTS_FILE)})
        _j("fr_open_scalars", {"rc": sentai.fr.open("scalars", FR_SCALARS_FILE)})
        _j("fr_task_start",   {"rc": sentai.fr.task_start()})

        # ---- takeoff to working altitude --------------------------
        rc = sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        _j("takeoff", {"h": TAKEOFF_HEIGHT, "rc": rc})
        _sleep_after_cmd(TAKEOFF_DUR)
        summary["phases_done"].append("takeoff")

        # ---- hover-and-log loop -----------------------------------
        # Iter-7: skip the multi-altitude sweep; cf2 drifts in 1-2 s
        # without VPE so a long altitude sweep mostly logs frames AWAY
        # from the H pattern.  Instead hover at takeoff height with
        # repeated set_position re-commands and log @ 10 Hz for ~6 s.
        # Verdict picks up the few ticks where cf2 is still over the
        # pattern.
        HOVER_TICKS_TOTAL = 80      # 8 s @ 10 Hz
        REASSERT_EVERY = 5          # re-issue go_to every 5 ticks (~0.5 s)
        tick_idx = 0
        alt = TAKEOFF_HEIGHT
        _j("hover_start", {"alt": alt})
        for k in range(HOVER_TICKS_TOTAL):
            if k % REASSERT_EVERY == 0:
                try:
                    sentai.crazy.go_to(0.0, 0.0, alt, 0.0, 0.6, 0, 0, 0)
                except (AttributeError, RuntimeError):
                    pass
            _log_tick(tick_idx, alt)
            tick_idx += 1
            sentai.rtos.sleep_ms(TICK_INTERVAL_MS)
        _j("hover_done", {"alt": alt, "ticks": HOVER_TICKS_TOTAL})
        summary["phases_done"].append("hover_log")
        summary["ticks_logged"] = tick_idx

        # ---- land + disarm ----------------------------------------
        rc = sentai.crazy.land(0.0, LAND_DUR)
        _j("land", {"rc": rc})
        _sleep_after_cmd(LAND_DUR)
        sentai.crazy.disarm()
        _j("disarm", {})
        summary["phases_done"].append("land")

        # ---- Stop + close FR channels -----------------------------
        try:
            _j("fr_task_stop",     {"rc": sentai.fr.task_stop()})
            for ch in ("frames", "events", "scalars"):
                try:
                    st = sentai.fr.stats(ch)
                    _j("fr_stats", {"ch": ch, "stats": st})
                except (AttributeError, RuntimeError):
                    pass
            _j("fr_close_frames",  {"rc": sentai.fr.close("frames")})
            _j("fr_close_events",  {"rc": sentai.fr.close("events")})
            _j("fr_close_scalars", {"rc": sentai.fr.close("scalars")})
        except (AttributeError, RuntimeError) as ex:
            _j("fr_close_err", {"err": repr(ex)})

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
