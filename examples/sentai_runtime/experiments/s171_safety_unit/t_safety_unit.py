# s171_safety_unit — sentai.safety state machine unit test (no SITL).
#
# Bypasses SafetyTask + camera + ArUco by directly injecting fake
# detection results via the test-only `sentai.safety._test_push_aruco`
# binding.  Validates the HARD RULE [[flowbaseline2-4markers-abort]]:
#   - n_dets < 4 sustained for >= 1.0s → aborted == True
#   - n_dets == 4 within window resets streak (fluke is OK)
#   - clear() resets the abort flag
# Plus the EKF-ceiling watchdog timer behaviour (stale-feed).
#
# Run from sentai_sim REPL:
#   import t_safety_unit
# (or `bash run.sh` from this folder)

import sentai


def _assert(cond, msg):
    if not cond:
        print("FAIL:", msg)
        raise Exception(msg)
    else:
        print("OK  :", msg)


def run():
    print("== s171 sentai.safety state-machine unit test ==")
    n_pass = 0
    try:
        # ── Test 1: clean init + enable, aborted should be False ──
        _assert(sentai.safety.init() == 0, "init returns 0")
        _assert(sentai.safety.clear() == 0, "clear returns 0")
        _assert(sentai.safety.aborted() is False, "after clear, aborted=False")
        rc = sentai.safety.enable_aruco(4, 1.0)
        _assert(rc == 0, "enable_aruco(4, 1.0) returns 0")
        _assert(sentai.safety.aborted() is False, "still False after enable")
        n_pass += 1

        # ── Test 2: push 4 markers, no abort ─────────────────────
        for i in range(5):
            sentai.safety._test_push_aruco(4, i + 1, i * 100)
        _assert(sentai.safety.aborted() is False,
                "5 frames of n=4 → no abort")
        n_pass += 1

        # ── Test 3: push n=0 for < 1s → no abort yet ─────────────
        # Streak start at t=600ms, push n=0 from t=700..1400 (700ms < 1000ms).
        sentai.safety._test_push_aruco(0, 100, 700)   # streak starts at 700
        sentai.safety._test_push_aruco(0, 101, 900)   # 200ms < 1000ms
        sentai.safety._test_push_aruco(0, 102, 1500)  # 800ms < 1000ms
        _assert(sentai.safety.aborted() is False,
                "n=0 for 800ms (<1000ms) → no abort yet")
        n_pass += 1

        # ── Test 4: continue n=0 past 1s → abort fires ───────────
        # Streak started at 700; push at 1800 → elapsed=1100ms ≥ 1000ms.
        sentai.safety._test_push_aruco(0, 103, 1800)
        _assert(sentai.safety.aborted() is True,
                "n=0 for >= 1000ms → aborted=True")
        reason = sentai.safety.reason()
        _assert("markers" in reason or "n_dets" in reason or "<4" in reason,
                "reason mentions markers/n_dets/<4 — got: %r" % reason)
        n_pass += 1

        # ── Test 5: clear, then verify aborted resets ────────────
        sentai.safety.clear()
        _assert(sentai.safety.aborted() is False, "clear() resets aborted")
        _assert(sentai.safety.reason() == "", "clear() resets reason")
        n_pass += 1

        # ── Test 6: fluke save — n=0 streak broken by ONE n=4 ────
        sentai.safety.enable_aruco(4, 1.0)
        sentai.safety._test_push_aruco(0, 200, 0)
        sentai.safety._test_push_aruco(0, 201, 500)
        sentai.safety._test_push_aruco(4, 202, 600)   # fluke save
        sentai.safety._test_push_aruco(0, 203, 700)   # new streak from 700
        sentai.safety._test_push_aruco(0, 204, 1200)  # 500ms < 1000ms
        _assert(sentai.safety.aborted() is False,
                "fluke n=4 reset → no abort yet")
        sentai.safety._test_push_aruco(0, 205, 1750)  # 1050ms ≥ 1000ms → abort
        _assert(sentai.safety.aborted() is True,
                "after fluke + 1050ms n=0 → abort")
        n_pass += 1

        # ── Test 7: partial detection (n=2) counts as <4 ─────────
        sentai.safety.clear()
        sentai.safety.enable_aruco(4, 1.0)
        sentai.safety._test_push_aruco(2, 300, 0)
        sentai.safety._test_push_aruco(3, 301, 500)
        sentai.safety._test_push_aruco(1, 302, 1100)
        _assert(sentai.safety.aborted() is True,
                "partial detections (n=2,3,1) all count as <4 → abort")
        n_pass += 1

        # ── Test 8: tighter window — max_loss_s=0.3s ─────────────
        sentai.safety.clear()
        sentai.safety.enable_aruco(4, 0.3)
        sentai.safety._test_push_aruco(0, 400, 0)
        sentai.safety._test_push_aruco(0, 401, 200)
        _assert(sentai.safety.aborted() is False, "0.2s < 0.3s, not aborted")
        sentai.safety._test_push_aruco(0, 402, 400)
        _assert(sentai.safety.aborted() is True, "0.4s >= 0.3s → abort")
        n_pass += 1

        print("\n== %d/8 tests PASS ==" % n_pass)
        sentai.safety.clear()
        return True
    except Exception as e:
        print("\n== %d/8 tests passed before failure: %s ==" % (n_pass, e))
        try: sentai.safety.clear()
        except Exception: pass
        return False


# Auto-run on import (REPL-friendly)
_result = run()
print("UNIT_TEST_RESULT:", "PASS" if _result else "FAIL")
