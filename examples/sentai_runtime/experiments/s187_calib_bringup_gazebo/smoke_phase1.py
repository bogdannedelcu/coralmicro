# s187 phase-1 smoke — no Gazebo, no drone.
#
# Validates the MicroPython binding surface for OP-S10-W21-T4:
#   - run_bringup(marker_world, **opts) spawns the orchestrator task
#   - bringup_get_phase() reports live phase
#   - bringup_abort() cooperatively cancels the worker
#   - bringup_is_done() flips once the worker exits
#   - bringup_get_result() returns the expected dict shape
#
# This is the smoke a developer runs after touching modsentai_calib.c
# bindings or sentai_calib_bringup.{h,cc} to confirm the surface is wired
# end-to-end BEFORE booting Gazebo.
#
# Pass: all 7 assertions PASS, abort observed within 10 s.

import sentai


def sleep_s(s):
    sentai.rtos.sleep_ms(int(s * 1000))

# Bogus 4-marker pad — geometry doesn't matter for this smoke; the
# orchestrator will reach SAMPLE phase, fail to see real markers (no
# camera frames in this headless smoke), and we abort it.
MARKER_WORLD = [
    (-0.16, +0.16, 0.005),
    (+0.16, +0.16, 0.005),
    (-0.16, -0.16, 0.005),
    (+0.16, -0.16, 0.005),
]


def assert_eq(label, got, want):
    ok = (got == want)
    print("  {}  {}: got={!r} want={!r}".format("PASS" if ok else "FAIL", label, got, want))
    assert ok, label


def assert_keys(label, d, keys):
    missing = [k for k in keys if k not in d]
    ok = not missing
    print("  {}  {}: missing={!r}".format("PASS" if ok else "FAIL", label, missing))
    assert ok, label


def main():
    print("=== s187 smoke_phase1 ===")

    # 1. Pre-spawn state — IDLE.
    assert_eq("phase pre-spawn",   sentai.calib.bringup_get_phase(), 0)
    assert_eq("done pre-spawn",    sentai.calib.bringup_is_done(), False)

    # 2. Spawn with short timeouts so the abort window is small.
    rc = sentai.calib.run_bringup(
        MARKER_WORLD,
        marker_size_m=0.094,
        z_hold=0.78,
        sweep_radius=0.10,
        settle_s=0.5,
        vmax=0.10,
        dur_relay=5.0,
        dur_hold=2.0,
        hold_rms_max=0.030,
    )
    assert_eq("run_bringup rc", rc, 0)

    # 3. Phase should advance to SAMPLE (=1) within a second.
    sleep_s(1.0)
    phase = sentai.calib.bringup_get_phase()
    in_sample = (phase in (1, 2))   # SAMPLE or KABSCH (if super fast)
    print("  {}  phase advanced to SAMPLE/KABSCH: phase={}".format("PASS" if in_sample else "FAIL", phase))
    assert in_sample

    # 4. Abort and wait for the worker to wind down.
    sentai.calib.bringup_abort()
    for _ in range(30):
        if sentai.calib.bringup_is_done():
            break
        sleep_s(0.5)
    assert_eq("done after abort", sentai.calib.bringup_is_done(), True)

    # 5. Result dict shape — check the documented keys are all present.
    r = sentai.calib.bringup_get_result()
    print("  result keys: {}".format(sorted(r.keys())))
    assert_keys("result keys", r, [
        "accepted", "reject_code", "last_phase",
        "R_cam_to_body", "cam_offset_B",
        "kp_x", "kp_y",
        "hold_max_drift_m", "hold_rms_drift_m",
        "n_samples_used", "total_duration_ms",
        "ext_quality",
    ])
    assert_eq("R length",     len(r["R_cam_to_body"]), 9)
    assert_eq("off length",   len(r["cam_offset_B"]), 3)

    # 6. Final phase should be DONE_FAIL (=8) — aborted runs don't accept.
    assert_eq("final phase DONE_FAIL", sentai.calib.bringup_get_phase(), 8)
    assert_eq("accepted",              r["accepted"],                    False)

    # 7. reject_code should be REJ_ABORTED (=9) OR REJ_FEW_SAMPLES (=2)
    #    if the sweep finished before abort took effect (race-free either
    #    way — both are valid "no real markers visible" outcomes).
    rj = r["reject_code"]
    accepts = (rj in (2, 9))
    print("  {}  reject_code in (ABORTED, FEW_SAMPLES): rj={}".format("PASS" if accepts else "FAIL", rj))
    assert accepts

    print("=== ALL PASS ===")


main()
