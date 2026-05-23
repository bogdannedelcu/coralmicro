# mission_s193 — OP-S10-W21-T4 calib bringup with 7-marker world (post-T10).
#
# This is the canonical sentai.calib.run_bringup() acceptance test,
# rerun against the now-working WhyCon detector (T10, commit b4eb37ce)
# AND the new 7-marker asymmetric pad layout (whycon_N added to
# upstream sentai_whycon_small.sdf — see ideas/external_patches.md
# 2026-05-23).
#
# Same code that runs here on cf2 SITL is the production bringup
# method on a real drone bench (per
# [[sentai-calib-is-production-bringup]]).
#
# Anti-cheat: orchestrator consumes only sentai.markers (PnP via
# WhyCon) + sentai.crazy.pose (cf2 EKF telemetry over CRTP LOG).
# No GT injection.  [[sentai-sim-air-gapped-from-truth]].
#
# WBS: OP-S10-W21-T4 phase-3 (rerun on 7-marker pad).
# Predecessors: s187 (blocked by detector), s192 (detection unblocked).

import sentai
import struct


JOURNAL_NAME = "mission_s193_journal.txt"
SUMMARY_NAME = "mission_s193_summary.json"

# ---- WhyCon 7-marker pad geometry (matches upstream sentai_whycon_small.sdf
# post-2026-05-23 + dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/).
# The 7th marker `N` breaks rectangular symmetry → unique PnP solution.
MARKER_WORLD = (
    (-0.08, +0.08, 0.005),   # NW
    (+0.08, +0.08, 0.005),   # NE
    (-0.06,  0.00, 0.005),   # W
    (+0.06,  0.00, 0.005),   # E
    (-0.08, -0.08, 0.005),   # SW
    (+0.08, -0.08, 0.005),   # SE
    (+0.02, +0.10, 0.005),   # N  ← asymmetric
)

# ---- Camera intrinsics — bridge downsamples to 320x240 -------------
FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.0544     # small pad (0.5× WhyCon outer ring)

# ---- Bringup envelope (s187 iter-83 baseline) -----------------------
Z_HOLD             = 0.60
SWEEP_RADIUS_M     = 0.025
SETTLE_S           = 2.0
VMAX_M_S           = 0.06
DUR_RELAY_S        = 30.0
DUR_HOLD_S         = 10.0
HOLD_RMS_MAX_M     = 0.030
TAKEOFF_DUR        = 2.5
LAND_DUR           = 2.5
PHASE_POLL_MS      = 500
PHASE_TIMEOUT_S    = 180.0       # generous: sweep + 2×30s relay + 10s hold

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
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)


def _poll_bringup(timeout_s):
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
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"wbs": "OP-S10-W21-T4-phase3",
                          "marker_n": len(MARKER_WORLD)})

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

    # ── Stack init ──────────────────────────────────────────────────────
    _j("setup", "start")
    try:
        sentai.calib.init()
        sentai.calib.clear()
        sentai.camera.init()
        _setup_markers()
        sentai.safety.init()
        sentai.crazy.init()
    except (RuntimeError, OSError, AttributeError, TypeError) as e:
        _j("setup_fail", {"err": repr(e)[:200]})
        summary["status"] = "SETUP_FAIL"
        _write_summary(summary); return summary
    _j("setup", "ok")

    # ── Pre-arm extPos warmup so cf2 Kalman has a Z anchor before
    # takeoff (no baro, no TOF cheat — only extPos until markers come
    # into view).  s187 iter-73 pattern.
    _j("extpos_warmup_pre_arm", "start")
    for _ in range(60):                # 1.8 s @ 30 ms
        try:
            sentai.crazy.send_crtp(6, 0, struct.pack("<fff", 0.0, 0.0, 0.014))
        except (AttributeError, RuntimeError):
            pass
        sentai.rtos.sleep_ms(30)
    _j("extpos_warmup_pre_arm", "done")

    # ── Arm + retry pose_subscribe ─────────────────────────────────────
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

    # ── HL takeoff (post-pre-arm-warmup, Kalman has converged Z) ──────
    _j("takeoff", {"z": Z_HOLD, "dur": TAKEOFF_DUR})
    sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)
    for _ in range(int(TAKEOFF_DUR * 1000 / 30) + 5):
        sentai.rtos.sleep_ms(30)
    sentai.crazy.hl_stop()
    _j("takeoff_done", {"z_target": Z_HOLD})

    # ── Hover + busy-poll markers; once n>=4 send extPos from PnP ─────
    _j("climb_seen", "start")
    POLL_BUDGET_S = 8.0
    found = False
    pnp_seen = 0
    for ti in range(int(POLL_BUDGET_S * 1000 / 30)):
        sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
        n = sentai.markers.detect_from_camera()
        z_pnp = -1.0
        if n >= 4:
            # Use the W19-shipped get_drone_pose_tuple which does
            # internal correspondence + Kabsch + yaw-anchor mirror
            # picker — handles the 7-marker layout via set_marker_world.
            try:
                p = sentai.crazy.pose()
                cf2_yaw = float(p[3]) if (p is not None and len(p) >= 4) else 0.0
            except (AttributeError, RuntimeError, TypeError):
                cf2_yaw = 0.0
            pose = sentai.markers.get_drone_pose_tuple(cf2_yaw)
            if pose is not None:
                px_pnp = float(pose[0])
                py_pnp = float(pose[1])
                pz_pnp = float(pose[2])
                if (px_pnp == px_pnp and py_pnp == py_pnp and pz_pnp == pz_pnp
                        and pz_pnp > 0.05):
                    z_pnp = pz_pnp
                    pnp_seen += 1
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
            _j("climb_tick", {"t": ti, "ekf_z": ekf_z, "n": n, "z_pnp": z_pnp})
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
        return summary

    # ── Register the 7-marker layout with sentai.markers so
    # get_drone_pose_tuple uses the correct correspondence search.
    mw_bytes = b''
    for (x, y, z) in MARKER_WORLD:
        mw_bytes += struct.pack('<fff', x, y, z)
    try:
        rc_mw = sentai.markers.set_marker_world(mw_bytes)
        _j("set_marker_world", {"n": len(MARKER_WORLD), "rc": rc_mw})
    except (AttributeError, RuntimeError):
        _j("set_marker_world", {"err": "binding-missing"})

    # ── Run bringup ────────────────────────────────────────────────────
    _j("bringup_start", {
        "z_hold":       Z_HOLD,
        "sweep_radius": SWEEP_RADIUS_M,
        "settle_s":     SETTLE_S,
        "vmax":         VMAX_M_S,
        "dur_relay":    DUR_RELAY_S,
        "dur_hold":     DUR_HOLD_S,
        "marker_n":     len(MARKER_WORLD),
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

    # ── Return-to-home + land ─────────────────────────────────────────
    try:
        sentai.crazy.go_to(0.0, 0.0, Z_HOLD, 0.0, 3.0)
        sentai.rtos.sleep_ms(3500)
    except (AttributeError, RuntimeError, TypeError):
        pass
    _j("land", "start")
    sentai.crazy.land(LAND_DUR)
    sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
    sentai.crazy.disarm()
    _j("land", "done")

    _write_summary(summary)
    _j("mission_end", summary)
    sentai.sim.journal_close()
    return summary
