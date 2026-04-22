# _e25_flow_motion.py — first smoke test for sentai.flow.
#
# Three phases, ~18s total.  Prints the LED state that the user
# should observe, and records per-phase statistics on the (dx, dy)
# deltas reported by sentai.flow.read():
#
#   Phase A  (5s):  LED OFF — camera still.  Expected |Δ| ≈ 0.
#   Phase B (10s):  LED ON  — USER MOVES THE BOARD in their hand.
#                   Expected |Δ| clearly non-zero, confidence high.
#   Phase C  (3s):  LED OFF — camera still again.  Expected back to
#                   near-zero; confirms flow re-acquires baseline.
#
# Output: per-phase table of (fps, |dx| mean, |dy| mean, |dx| max,
# |dy| max, confidence mean).  Also dumps raw samples as CSV to
# the session dir.

import sentai, gc

gc.collect()

# Refuse if pipeline is running — they share PXP + cam buffers.
if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.camera.ratio(0, 0)         # no auto-alternate; cam0 only
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.select(0)

# LED should be OFF at start so the user sees the transition.
sentai.io.led_off()

from diag._session import _session, begin, end, _save_path, _record, _save_desc
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned:
    begin("e25_flow")

PHASE_A_MS = 5000
PHASE_B_MS = 10000
PHASE_C_MS = 3000
POLL_MS    = 50     # poll rate for sentai.flow.read()

# Launch the background flow task.
rc = sentai.flow.start(0)
print("[E25] flow.start(cam0) rc=%d — running=%s" % (rc, sentai.flow.running()))

def _sample_phase(label, duration_ms):
    samples = []                 # (t_ms, dx, dy, sad, conf, frame_seq, age)
    start_t = _ticks()
    last_seq = 0
    while _ticks() - start_t < duration_ms:
        r = sentai.flow.read()
        # Only record a sample when the flow task produced a new frame
        # (frame_seq advances).  Otherwise we'd get duplicates during
        # the 50ms poll loop.
        if r["status"] == 1 and r["frame_seq"] != last_seq:
            last_seq = r["frame_seq"]
            samples.append((
                _ticks() - start_t,
                r["dx"], r["dy"], r["sad"],
                r["confidence"], r["frame_seq"], r["age_ms"]))
        sentai.rtos.sleep_ms(POLL_MS)

    if not samples:
        print("[E25:%s] NO samples collected" % label)
        return []

    def _absmean(lst):
        return sum(abs(v) for v in lst) // len(lst)
    def _absmax(lst):
        return max(abs(v) for v in lst)

    dxs = [s[1] for s in samples]
    dys = [s[2] for s in samples]
    confs = [s[4] for s in samples]
    dur_ms = samples[-1][0] - samples[0][0] or 1
    fps = len(samples) * 1000.0 / dur_ms if len(samples) > 1 else 0
    print("[E25:%s] N=%d fps=%.1f  |dx|=%d/%d  |dy|=%d/%d  conf=%d (mean/max)" % (
        label, len(samples), fps,
        _absmean(dxs), _absmax(dxs),
        _absmean(dys), _absmax(dys),
        sum(confs) // len(confs)))
    return samples

# ── Phase A: fixed camera baseline ──────────────────────────────
print("== Phase A (5s): hold camera still. LED should be OFF. ==")
samples_a = _sample_phase("A_still", PHASE_A_MS)

# ── Phase B: user moves the camera ──────────────────────────────
sentai.io.led_on()
print("== Phase B (10s): LED ON — MOVE THE BOARD NOW! ==")
samples_b = _sample_phase("B_moving", PHASE_B_MS)

# ── Phase C: fixed again ────────────────────────────────────────
sentai.io.led_off()
print("== Phase C (3s): stop moving.  LED OFF.  Confirm deltas drop. ==")
samples_c = _sample_phase("C_still_again", PHASE_C_MS)

sentai.flow.stop()

# ── Stats summary ───────────────────────────────────────────────
st = sentai.flow.stats()
print()
print("=== E25 FLOW MOTION SUMMARY ===")
print("  frames_processed = %d" % st["frames_processed"])
print("  frames_dropped   = %d" % st["frames_dropped"])
print("  grab_fail        = %d" % st["grab_fail"])
print("  pxp_fail         = %d" % st["pxp_fail"])
print("  avg_fps          = %.1f" % (st["avg_fps_x10"] / 10.0))

# ── Persist samples for post-mortem ─────────────────────────────
csv_path = _save_path("e25_flow")
rows = []
for (phase, samples) in (("A", samples_a), ("B", samples_b), ("C", samples_c)):
    for (t, dx, dy, sad, conf, seq, age) in samples:
        rows.append((phase, t, dx, dy, sad, conf, seq, age))
save_csv(csv_path,
    ["phase", "t_ms", "dx", "dy", "sad", "confidence", "frame_seq", "age_ms"],
    rows)
_save_desc(csv_path,
    "E25 - optical flow motion detection smoke test.\n"
    "Phase A (5s): camera still -> baseline |Δ| near 0.\n"
    "Phase B (10s): user moves board -> deltas visibly non-zero.\n"
    "Phase C (3s): still again -> deltas return to baseline.\n",
    params={
        "downsample": "80x60",
        "block": "32x32",
        "search_range": 12,
        "poll_ms": POLL_MS,
        "frames_processed": st["frames_processed"],
        "avg_fps_x10": st["avg_fps_x10"],
    })
_record("e25_flow", csv_path,
        "frames=%d fps=%.1f" % (st["frames_processed"], st["avg_fps_x10"] / 10.0))

if owned:
    end()
