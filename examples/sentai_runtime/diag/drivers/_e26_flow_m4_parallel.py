# _e26_flow_m4_parallel.py — M4-offload flow running in parallel with
# the M7 detection pipeline.  End-to-end test for the goal statement
# "don't affect pipeline fps while flow runs in parallel".
#
# Three phases, LED toggled to cue the user when to move the board:
#
#   Phase A (5s):  pipeline running,  flow running,  board STILL (LED off).
#                  Expected: pipeline fps near baseline, flow |Δ| ≈ 0.
#   Phase B (10s): pipeline running,  flow running,  board MOVING (LED on).
#                  Expected: pipeline fps still near baseline, flow |Δ|
#                  clearly non-zero.
#   Phase C (3s):  pipeline running,  flow running,  board STILL (LED off).
#                  Expected: flow back to near-zero; pipeline untouched.
#
# Baseline (no M4) came from E22:
#   A fixed cam0:               24.97 fps
# Target: phase fps within ±5% of that.

import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.ratio(0, 0)          # fixed cam0 — clean flow input
sentai.camera.select(0)
sentai.pipeline.direct_tensor(1)   # production path
sentai.camera.switch_drain(1)

sentai.io.led_off()

from diag._session import _session, begin, end, _save_path, _record, _save_desc
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned:
    begin("e26_flow_m4")

# Bring up the M4 if it isn't already (idempotent: multiple calls
# after success are harmless — StartM4() only runs once).
rc_enable = sentai.flow.m4_enable()
hb0 = sentai.flow.m4_heartbeat()
print("[E26] m4_enable rc=%d  alive=%s  hb=%d" %
      (rc_enable, hb0["alive"], hb0["heartbeat"]))
if not hb0["alive"]:
    print("[E26] ABORT: M4 not alive")
    if owned: end()
    raise SystemExit

# Start the detection pipeline first — otherwise the PrepTask hook
# that publishes frames to the M4 never runs.
sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)   # warmup

# Tell the M4 worker to start consuming.
sentai.flow.m4_start()

# ── helpers ─────────────────────────────────────────────────────
def _phase(label, duration_ms, led_on):
    if led_on: sentai.io.led_on()
    else:      sentai.io.led_off()

    # Pipeline fps meter: walk the queue and time the deltas.
    t_start = _ticks()
    wall = []; t_prev = _ticks()

    # Flow samples: per unique M4 frame_seq.
    flow_seen = {}   # frame_seq -> (dx, dy, sad, conf, age_ms)
    last_m4_seq = 0

    while _ticks() - t_start < duration_ms:
        r = sentai.pipeline.get_ex(50)      # 50ms max wait per frame
        t_now = _ticks()
        if r is not None:
            wall.append(t_now - t_prev); t_prev = t_now
        # Sample flow without stalling the loop.
        f = sentai.flow.m4_read()
        if f["alive"] and f["frame_seq"] != last_m4_seq:
            last_m4_seq = f["frame_seq"]
            flow_seen[f["frame_seq"]] = (
                f["dx"], f["dy"], f["sad"], f["confidence"])

    # Pipeline fps.
    mean_wall = sum(wall) / len(wall) if wall else 0
    fps = 1000.0 / mean_wall if mean_wall else 0

    # Flow aggregate.
    n = len(flow_seen)
    if n:
        dxs = [v[0] for v in flow_seen.values()]
        dys = [v[1] for v in flow_seen.values()]
        cfs = [v[3] for v in flow_seen.values()]
        def _absmean(l): return sum(abs(x) for x in l) // len(l)
        def _absmax(l):  return max(abs(x) for x in l)
        print(("[E26:%s] pipe_fps=%.2f  flow_n=%d  "
               "|dx|=%d/%d  |dy|=%d/%d  conf=%d") %
              (label, fps, n,
               _absmean(dxs), _absmax(dxs),
               _absmean(dys), _absmax(dys),
               sum(cfs) // len(cfs)))
    else:
        print("[E26:%s] pipe_fps=%.2f  flow_n=0 (no samples)" % (label, fps))

    return {"label": label, "fps": fps, "wall": wall,
            "flow": dict(flow_seen)}

# ── Phase A ─────────────────────────────────────────────────────
print("== Phase A (5s): BOARD STILL.  LED off. ==")
A = _phase("A_still", 5000, led_on=False)

# ── Phase B ─────────────────────────────────────────────────────
print("== Phase B (10s): LED ON → MOVE THE BOARD. ==")
B = _phase("B_moving", 10000, led_on=True)

# ── Phase C ─────────────────────────────────────────────────────
sentai.io.led_off()
print("== Phase C (3s): BOARD STILL AGAIN.  LED off. ==")
C = _phase("C_still_again", 3000, led_on=False)

sentai.flow.m4_stop()
sentai.pipeline.stop()

# ── summary ─────────────────────────────────────────────────────
m4_stats = sentai.flow.m4_read()
print()
print("=== E26 SUMMARY ===")
print("  A  (still):  pipe %.2f fps   flow n=%d" %
      (A["fps"], len(A["flow"])))
print("  B  (moving): pipe %.2f fps   flow n=%d" %
      (B["fps"], len(B["flow"])))
print("  C  (still):  pipe %.2f fps   flow n=%d" %
      (C["fps"], len(C["flow"])))
print("  m4.frames_processed = %d" % m4_stats["frames_processed"])
print("  m4.avg_compute_us   = %d" % m4_stats["avg_compute_us"])

# Persist CSV for post-mortem.
csv_path = _save_path("e26_flow_m4")
rows = []
for phase, data in (("A", A), ("B", B), ("C", C)):
    for seq, (dx, dy, sad, conf) in data["flow"].items():
        rows.append((phase, seq, dx, dy, sad, conf))
save_csv(csv_path,
    ["phase", "frame_seq", "dx", "dy", "sad", "confidence"],
    rows)
_save_desc(csv_path,
    "E26 - M4-offload optical flow running in parallel with M7 "
    "detection pipeline.\n"
    "Gray 40x30 from step-16 decimation; SAD 16x16 block, +-6 search.\n",
    params={
        "pipeline_fps_A": round(A["fps"], 2),
        "pipeline_fps_B": round(B["fps"], 2),
        "pipeline_fps_C": round(C["fps"], 2),
        "m4_frames_processed": m4_stats["frames_processed"],
        "m4_avg_compute_us":  m4_stats["avg_compute_us"],
        "flow_samples_A": len(A["flow"]),
        "flow_samples_B": len(B["flow"]),
        "flow_samples_C": len(C["flow"]),
    })
_record("e26_flow_m4", csv_path,
        "A=%.2f B=%.2f C=%.2f fps  m4=%d frames" %
        (A["fps"], B["fps"], C["fps"], m4_stats["frames_processed"]))

if owned: end()
