# mission_s187 — OP-S10-W21-T4 calib bringup, Gazebo end-to-end.
#
# Production-bringup mission: same code that runs on a real drone bench
# also runs here on cf2 SITL.  Mission's only job is to wrap the C-side
# orchestrator with takeoff + land; the heavy lifting (sample sweep,
# Kabsch, autotune X/Y, hold validation, persist) all lives in
# sentai_calib_bringup.cc.
#
# Per [[sentai-calib-is-production-bringup]]: no auto-flight, operator
# triggers via REPL by calling `run()`.  Per [[missions-run-in-sentai-
# only]]: ALL flight commands issued inside sentai_sim; host runs only
# the launcher + gt_recorder (post-mortem only, never injected).
#
# Anti-cheat: orchestrator consumes only sentai.markers (PnP via WhyCon)
# and sentai.crazy.pose (cf2 EKF telemetry) — see
# [[sentai-sim-air-gapped-from-truth]].
#
# WBS: OP-S10-W21-T4 phase-2 (acceptance gate per design doc).

import sentai


JOURNAL_NAME = "mission_s187_journal.txt"
SUMMARY_NAME = "mission_s187_summary.json"

# ---- Flight Recorder channels (per [[op-s10-w13-shipped]] pattern).
# On SIM, sentai.fr uses POSIX fopen with literal host paths.  The
# "frames" channel captures the gray buffer alongside each
# sentai_markers_detect_frame call (via sentai_fr_push_frame hook in
# sentai_markers.cc:310).  Operator-suggested 2026-05-21 for s187 to
# enable post-mortem replay of what the camera actually sees.
FR_DIR           = ("/home/bogdan/work/coralmicro/examples/sentai_runtime/"
                    "experiments/s187_calib_bringup_gazebo/fr_current")
FR_FRAMES_DIR    = FR_DIR + "/frames"
FR_EVENTS_FILE   = FR_DIR + "/events.csv"
FR_SCALARS_FILE  = FR_DIR + "/scalars.csv"

# ---- WhyCon pad geometry (must match sentai_whycon.sdf, iter-11 square).
# 6 markers: 4 corners at ±0.16 m, 2 mid-bars at ±0.12 m, all at
# z=0.005 (top of mount box).  Operator-specified square layout
# (proportions 1:1:0.75 with unit 0.16 m).
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),
    (+0.08, +0.08, 0.005),
    (-0.06,  0.00, 0.005),
    (+0.06,  0.00, 0.005),
    (-0.08, -0.08, 0.005),
    (+0.08, -0.08, 0.005),
)

# ---- Camera intrinsics — bridge downsamples to 320x240 -------------
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.0544   # iter-64: 0.5× WhyCon outer ring
                              # (was 0.1088 = sentai_whycon big pad)

# ---- Bringup envelope ------------------------------------------------
Z_HOLD             = 0.60    # iter-68: s172-validated baseline (was 0.9 then,
                              # 0.6 for autotune).  At z=0.6 + small pad 16cm,
                              # WhyCon ring is 26 px — comfortable detection.
SWEEP_RADIUS_M     = 0.025   # iter-64b: scaled with 0.5× pad — keep
                              # drone within FOV (pad 16cm wide now)
SETTLE_S           = 2.0
VMAX_M_S           = 0.06    # s172 baseline (operator iter #17: 0.06 is
                              # the empirical ZN SNR sweet spot)
DUR_RELAY_S        = 30.0   # s174 baseline (autotune-validated)
DUR_HOLD_S         = 10.0
HOLD_RMS_MAX_M     = 0.030
TAKEOFF_DUR        = 2.5    # iter-57: back to iter-53 baseline (best so far:
                              # R 0.38°, hold 39mm/62mm).  z_stabilize wait
                              # + slow takeoff = best stability.  Faster
                              # takeoff degrades quality more than it saves
                              # time.
LAND_DUR           = 2.5
PHASE_POLL_MS      = 500
PHASE_TIMEOUT_S    = 180.0   # generous: sweep + 2×30s relay + 10s hold + slack

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


def _setup_markers():
    # No set_marker_world here — the bringup orchestrator owns the
    # registered-pad layout (passed via run_bringup arg) and does its
    # own forward-projection-based detection→world association,
    # independent of any mk.id field.  WhyCon multi-marker IDs are
    # scan-order per frame and not stable, so direct ID lookup would
    # fail; the orchestrator's spatial assoc handles that backend-
    # agnostically.
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)


def _poll_bringup(timeout_s):
    """Block until orchestrator hits DONE_OK or DONE_FAIL.  Logs every
    phase transition to the journal so the host can reconstruct the
    timeline post-mortem.  Returns (done, last_phase)."""
    last_phase = -1
    t_acc_ms = 0
    while t_acc_ms < int(timeout_s * 1000):
        if sentai.calib.bringup_is_done():
            return True, sentai.calib.bringup_get_phase()
        phase = sentai.calib.bringup_get_phase()
        if phase != last_phase:
            _j("bringup_phase", {"phase": phase,
                                  "name": PHASE_NAMES.get(phase, "?")})
            last_phase = phase
        sentai.rtos.sleep_ms(PHASE_POLL_MS)
        t_acc_ms += PHASE_POLL_MS
    return False, sentai.calib.bringup_get_phase()


def run():
    """Operator entry point.  Returns a dict matching mission_s187_summary
    so the host run.sh can grep status without parsing journal lines."""
    # Journal path is resolved relative to FS_ROOT (per sim_fs_resolve);
    # bare filename only — absolute paths fail the resolver.
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"wbs": "OP-S10-W21-T4"})

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
    }

    # ── Stack init.  SafetyTask is backend-agnostic (per
    # sentai_safety_task.cc:194 "the safety task is backend-agnostic")
    # — the "aruco" suffix in enable_aruco is historical naming; the
    # check just counts markers via sentai_markers_get_count which
    # respects whichever backend was init'd.  W21-T4d iter-47 patch
    # ensures safety_task_start doesn't override a mission-set backend.
    _j("setup", "start")
    try:
        sentai.calib.init()
        sentai.calib.clear()
        sentai.camera.init()
        _setup_markers()                 # WhyCon backend init + intrinsics
        sentai.safety.init()
        sentai.crazy.init()
    except Exception as e:
        _j("setup_fail", {"err": str(e)})
        summary["status"] = "SETUP_FAIL"
        _write_summary(summary); return summary
    _j("setup", "ok")

    # ── Flight Recorder — open channels so detect_frame auto-saves
    # gray buffers + per-tick scalars.  Operator-suggested for s187.
    _j("fr_init",         {"rc": sentai.fr.init()})
    _j("fr_open_frames",  {"rc": sentai.fr.open("frames",  FR_FRAMES_DIR)})
    _j("fr_open_events",  {"rc": sentai.fr.open("events",  FR_EVENTS_FILE)})
    _j("fr_open_scalars", {"rc": sentai.fr.open("scalars", FR_SCALARS_FILE)})
    _j("fr_task_start",   {"rc": sentai.fr.task_start()})

    # iter-73: send extPos (0,0,0.014) PRE-ARM at 30Hz for 2s.  cf2
    # stock no-baro firmware (operator confirm): accel z integration
    # without correction → Kalman bounds breach → reset loop.
    # Spamming extPos before arm gives Kalman a Z anchor → converges
    # → arm + takeoff work like in real-world deployment with
    # flow_deck / lighthouse positioning.
    import struct
    _j("extpos_warmup_pre_arm", "start")
    for _ in range(60):                # 60 × 30 ms = 1.8 s
        try:
            sentai.crazy.send_crtp(6, 0, struct.pack("<fff", 0.0, 0.0, 0.014))
        except (AttributeError, RuntimeError):
            pass
        sentai.rtos.sleep_ms(30)
    _j("extpos_warmup_pre_arm", "done")

    # ── Arm + retry pose_subscribe + takeoff ─────────────────────────
    # pose_subscribe at arm+300ms returns -3 (cf2 TOC not ready, 1-3s
    # post-link).  Retry 30×200ms.  Pattern from s182 iter-5 (2026-05-20).
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

    # iter-83 — VIRTUAL Z-RANGER via SENSOR_TOF_SIM CRTP injection.
    # cf2 SITL listens on CRTP_PORT_SETPOINT_SIM (port 9) for sensor sim
    # packets; sensors_sitl.c case SENSOR_TOF_SIM (type=5) calls
    # rangeEnqueueDownRangeInEstimator → Kalman fuses TOF as authoritative
    # Z anchor (like a flow_deck VL53L1x).  We feed PnP-derived z when
    # markers visible, else operator-known ground z=0.014.
    # Net effect: cf2 thinks it has a Z-ranger deck → HL takeoff/hover
    # work normally without baro.  Anti-cheat clean (z from PnP, not GT).
    import struct

    def _send_tof(z_m):
        # iter-85: Gazebo plugin now publishes GT z as SENSOR_TOF_SIM
        # (flow_deck VL53L1x emulation).  MP-side injection deprecated —
        # plugin runs at 200Hz with GT, dominates Kalman fusion.
        pass

    # Stage 1: prime cf2 with ground TOF (1.5s, drone on ground)
    _j("tof_warmup_ground", "start")
    for _ in range(50):           # 1.5s @ 30ms
        _send_tof(0.014)
        sentai.rtos.sleep_ms(30)
    _j("tof_warmup_ground", "done")

    # Stage 2: HL takeoff().  Keep sending TOF=0.014 (REAL drone z on
    # ground) — DO NOT ramp.  cf2 sees z_setpoint=Z_HOLD vs z_est=0.014
    # → error 0.586 → cf2 commands climb thrust → drone lifts.
    _j("takeoff", {"z": Z_HOLD, "dur": TAKEOFF_DUR})
    sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)
    # Send ground TOF during takeoff (cf2 trusts → drone physically lifts)
    ramp_ticks = int(TAKEOFF_DUR * 1000 / 30) + 5
    for ti in range(ramp_ticks):
        _send_tof(0.014)
        sentai.rtos.sleep_ms(30)
    sentai.crazy.hl_stop()
    _j("takeoff_done", {"z_target": Z_HOLD})

    # Stage 3: hover + busy-poll markers; once n>=6 switch TOF to PnP-z
    _j("climb_seen", "start")
    POLL_BUDGET_S = 8.0
    found = False
    pnp_seen = 0
    for ti in range(int(POLL_BUDGET_S * 1000 / 30)):
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
        n = sentai.markers.detect_from_camera()
        z_pnp = -1.0
        if n >= 4:
            # Build pixel + world correspondence for first n detected markers.
            img_pts = []
            wld_pts = []
            for k in range(min(n, 6)):
                t = sentai.markers.get_pose_tuple(k)
                if t is None:
                    img_pts = None; break
                img_pts.append((t[1], t[2]))
                wld_pts.append((MARKER_WORLD[k][0], MARKER_WORLD[k][1]))
            if img_pts is not None and len(img_pts) >= 4:
                try:
                    r = sentai.markers.coplanar_pnp(
                        img_pts, wld_pts, FX, FY, CX, CY)
                except (AttributeError, RuntimeError, ValueError):
                    r = None
                if r is not None:
                    cw = r["cam_world"]
                    px_pnp = float(cw[0])
                    py_pnp = float(cw[1])
                    pz_pnp = float(cw[2])
                    if pz_pnp == pz_pnp and pz_pnp > 0.05:
                        z_pnp = pz_pnp
                        pnp_seen += 1
                        # Send extPos (x, y, z) for XY anchor — plugin TOF
                        # handles Z but XY drifts via accel integration.
                        if (px_pnp == px_pnp and py_pnp == py_pnp):
                            sentai.crazy.send_crtp(6, 0,
                                struct.pack("<fff", px_pnp, py_pnp, pz_pnp))
        if (ti % 8) == 0:
            ekf_z = -1.0
            try:
                p = sentai.crazy.pose()
                if p is not None and len(p) >= 3:
                    ekf_z = p[2]
            except (AttributeError, RuntimeError, TypeError):
                pass
            _j("climb_tick",
               {"t": ti, "ekf_z": ekf_z, "n": n, "z_pnp": z_pnp})
        # Relaxed gate: 4+ markers + drone above 0.3m → calibration ready
        if n >= 4 and z_pnp > 0.0:
            _j("climb_seen_ok", {"t": ti, "n": n, "z_pnp": z_pnp})
            found = True
            break
        sentai.rtos.sleep_ms(30)
    if not found:
        _j("climb_seen_timeout", {"pnp_seen": pnp_seen})
        sentai.crazy.land(LAND_DUR)
        sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
        sentai.crazy.disarm()
        summary["status"] = "ASCENT_FAIL"
        _write_summary(summary)
        sentai.sim.journal_close()
        return summary

    # Stage 4: VPE warmup with PnP-derived z (1s of stable anchor).
    # MP embed: use coplanar_pnp (dict) instead of get_drone_pose (buffer).
    _j("vpe_warmup", "start")
    vpe_sent = 0
    for ti in range(33):                # 33 × 30 ms = 1 s
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
        n = sentai.markers.detect_from_camera()
        z_now = Z_HOLD
        if n >= 6:
            img_pts = []
            wld_pts = []
            for k in range(6):
                t = sentai.markers.get_pose_tuple(k)
                if t is None:
                    img_pts = None; break
                img_pts.append((t[1], t[2]))
                wld_pts.append((MARKER_WORLD[k][0], MARKER_WORLD[k][1]))
            if img_pts is not None:
                try:
                    r = sentai.markers.coplanar_pnp(
                        img_pts, wld_pts, FX, FY, CX, CY)
                except (AttributeError, RuntimeError, ValueError):
                    r = None
                if r is not None:
                    cw = r["cam_world"]
                    vx = float(cw[0]); vy = float(cw[1]); vz = float(cw[2])
                    if vx == vx and vy == vy and vz == vz:
                        z_now = vz
                        sentai.crazy.send_crtp(
                            6, 0, struct.pack("<fff", vx, vy, vz))
                        vpe_sent += 1
        _send_tof(z_now)
        sentai.rtos.sleep_ms(30)
    _j("vpe_warmup_done", {"sent_total": vpe_sent})

    Z_HOLD_RUN = Z_HOLD
    _j("takeoff_settled", {"z_hold_run": Z_HOLD_RUN})
    sentai.crazy.hl_stop()
    for _ in range(5):
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD_RUN)
        sentai.rtos.sleep_ms(30)

    # Now that drone is at altitude with markers visible, arm SafetyTask.
    # It drives detection at 30 Hz throughout SAMPLE + AUTOTUNE so the
    # orchestrator's inner workers see fresh markers each tick.
    _j("safety_enable",      {"rc": sentai.safety.enable_aruco(4, 4.0)})  # tolerant 4s
    _j("safety_task_start",  {"rc": sentai.safety.task_start()})

    # iter-65: REMOVED post_ascent_soak — cf2 z drifts up ~1.5m in 5s
    # without VPE.  Jump directly into bringup so SAMPLE engages VPE
    # immediately (within 500ms) and anchors z via PnP.
    # Operator iter-64b: "ai continuat sa urci, destul de mult".
    _j("post_ascent_soak", "skipped")

    # ── Run bringup ───────────────────────────────────────────────────
    _j("bringup_start", {
        "z_hold":       Z_HOLD_RUN,
        "sweep_radius": SWEEP_RADIUS_M,
        "settle_s":     SETTLE_S,
        "vmax":         VMAX_M_S,
        "dur_relay":    DUR_RELAY_S,
        "dur_hold":     DUR_HOLD_S,
    })
    rc = sentai.calib.run_bringup(
        MARKER_WORLD,
        marker_size_m  = MARKER_DIAMETER_M,
        z_hold         = Z_HOLD_RUN,
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
        # Land anyway — keep cf2 controlled.
        sentai.crazy.land(LAND_DUR)
        sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
        sentai.crazy.disarm()
        _write_summary(summary); return summary

    done, phase = _poll_bringup(PHASE_TIMEOUT_S)
    if not done:
        _j("bringup_timeout", {"last_phase": phase})
        sentai.calib.bringup_abort()
        sentai.rtos.sleep_ms(2000)

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

    # ── Post-bringup ──────────────────────────────────────────────────
    # T6 guard sanity — assert should succeed iff bringup accepted.
    try:
        sentai.calib.assert_calibrated()
        cal_ok = True
    except RuntimeError:
        cal_ok = False
    summary["assert_calibrated"] = cal_ok
    _j("assert_calibrated", {"ok": cal_ok})

    if summary["accepted"]:
        summary["status"] = "PASS"
    elif not done:
        summary["status"] = "TIMEOUT"
    else:
        summary["status"] = "FAIL"

    # ── Return-to-home + land ────────────────────────────────────────
    # iter-60: SIM mission valid iff drone lands ≤10cm of takeoff
    # origin [[sim-test-must-return-home]].  Autotune sweep displaced
    # cf2 ~25cm (iter-59 GT measure).  Use high-level go_to to navigate
    # back to (0,0) at Z_HOLD_RUN, settle, THEN land.
    _j("rth", {"x": 0.0, "y": 0.0, "z": Z_HOLD_RUN})
    try:
        sentai.crazy.go_to(0.0, 0.0, Z_HOLD_RUN, 0.0, 3.0)
    except (AttributeError, RuntimeError):
        # Fallback: hover toward origin with proportional velocity.
        for _ in range(40):                  # 4 s closed-loop
            try:
                px, py, pz, pyaw = sentai.crazy.pose()
            except (AttributeError, RuntimeError):
                break
            vx = max(-0.15, min(0.15, -px * 0.5))
            vy = max(-0.15, min(0.15, -py * 0.5))
            sentai.crazy.hover(vx, vy, 0.0, Z_HOLD_RUN)
            sentai.rtos.sleep_ms(100)
    sentai.rtos.sleep_ms(3500)               # let go_to/closed-loop settle
    sentai.crazy.hl_stop()
    for _ in range(10):
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD_RUN)
        sentai.rtos.sleep_ms(50)

    _j("land", {"dur": LAND_DUR})
    sentai.crazy.land(LAND_DUR)
    sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
    sentai.crazy.disarm()

    _j("mission_done", {"status": summary["status"]})
    _write_summary(summary)
    sentai.sim.journal_close()
    return summary
