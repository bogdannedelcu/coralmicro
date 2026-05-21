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
    (-0.16, +0.16, 0.005),
    (+0.16, +0.16, 0.005),
    (-0.12,  0.00, 0.005),
    (+0.12,  0.00, 0.005),
    (-0.16, -0.16, 0.005),
    (+0.16, -0.16, 0.005),
)

# ---- Camera intrinsics — bridge downsamples to 320x240 -------------
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.1088   # WhyCon outer ring

# ---- Bringup envelope ------------------------------------------------
Z_HOLD             = 0.78    # iter-20-baseline (replicable Kabsch convergence)
SWEEP_RADIUS_M     = 0.05    # small offset → markers stay near image center
SETTLE_S           = 2.0
VMAX_M_S           = 0.10    # iter-29 baseline (Kabsch drift 0.33° PASS)
                              # Lower (0.04) tested in iter-30: didn't help
                              # autotune convergence; reverted.
DUR_RELAY_S        = 30.0   # s174 baseline (autotune-validated)
DUR_HOLD_S         = 10.0
HOLD_RMS_MAX_M     = 0.030
TAKEOFF_DUR        = 2.5
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

    # ── Stack init (mirrors s182 robust sequence) ─────────────────────
    _j("setup", "start")
    try:
        sentai.calib.init()           # load /system/calib.ini if present
        # iter-37: clear prior persisted calib so Kabsch's
        # drift_from_persisted check uses SDF defaults (not a prior
        # run's possibly-stochastically-off R).  In production this
        # would be conditional on operator's "fresh-calib" gesture.
        sentai.calib.clear()
        # Markers FIRST so detection backend is live before takeoff (s182).
        _setup_markers()
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

    _j("takeoff", {"z": Z_HOLD, "dur": TAKEOFF_DUR})
    sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)
    # Long settle: takeoff_dur + 0.5s + 4s soak (s174/s182 pattern).
    sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
    sentai.rtos.sleep_ms(4000)
    _j("takeoff_settled", {})
    sentai.crazy.hl_stop()
    for _ in range(5):
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
        sentai.rtos.sleep_ms(30)

    # ── Run bringup ───────────────────────────────────────────────────
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

    # ── Land + disarm ─────────────────────────────────────────────────
    _j("land", {"dur": LAND_DUR})
    sentai.crazy.land(LAND_DUR)
    sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
    sentai.crazy.disarm()

    _j("mission_done", {"status": summary["status"]})
    _write_summary(summary)
    sentai.sim.journal_close()
    return summary
