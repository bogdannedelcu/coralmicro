# _e34_flow_square_dualcap.py — square walk + DUAL capture at each corner.
#
# Identical protocol to E32/E33 (LED flicker = hold, LED solid = move),
# but each WAIT (corner) phase captures BOTH:
#
#   1. A full-resolution color JPEG from cam0 (640×480, quality 80).
#      Requires briefly pausing the detection pipeline because the
#      camera is owned by PrepTask while the pipeline is running; the
#      pause is ~200 ms per corner, invisible to the user.
#
#   2. A 40×30 gray PGM = the exact buffer the M4 SAD algorithm sees.
#      This is a pure memory read from the shared window (no pause
#      needed) — captures whatever M7 PrepTask last published before
#      the pipeline was stopped, which is fine for a visual reference.
#
# Between WAITs, the pipeline runs normally and M4 processes motion.
# The trace CSV is built from m4_body_read() samples collected only
# during MOVE phases (so we don't contaminate with pause artifacts).
#
# Output:
#   /diags/sNNN_e34/001_e34_flow_square.csv  (trace for plot)
#   /diags/sNNN_e34/000_e34_corners_frames/  (JPEG + PGM per corner)

import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)
sentai.io.led_off()

from diag._session import _session, begin, end, _save_path, _record, _save_desc, _photos_dir
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned:
    begin("e34_flow_square_dualcap")

# Bring up M4 and start pipeline once.
rc = sentai.flow.m4_enable()
hb = sentai.flow.m4_heartbeat()
print("[E34] m4_enable rc=%d alive=%s" % (rc, hb["alive"]))
if not hb["alive"]:
    print("[E34] ABORT: M4 not alive")
    if owned: end()
    raise SystemExit

shots_dir = _photos_dir("e34_corners") or "/diags/e34_corners"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

POLL_MS = 20

PHASES = [
    # E35 measured the dual-capture cycle at ~500 ms (stop 40 + jpeg
    # 100 + gray 0 + start 100 + lfs 120 + settle 140 ms).  A 3 s WAIT
    # leaves 2.5 s of "hold still" for the operator, more than enough.
    # Total experiment: 5×3 + 4×4 = 31 s, well under the 120 s REPL-
    # idle watchdog ceiling.
    ("WAIT_start", 3000, "flicker"),
    ("MOVE_FW",    4000, "solid"),
    ("WAIT_c1",    3000, "flicker"),
    ("MOVE_R",     4000, "solid"),
    ("WAIT_c2",    3000, "flicker"),
    ("MOVE_BACK",  4000, "solid"),
    ("WAIT_c3",    3000, "flicker"),
    ("MOVE_L",     4000, "solid"),
    ("WAIT_end",   3000, "flicker"),
]

def _flick(pstart, now):
    on = (((now - pstart) // 100) & 1) == 0
    if on: sentai.io.led_on()
    else:  sentai.io.led_off()

def _capture_corner(tag):
    """Grab a color JPEG + a gray PGM at a corner while pipeline is
    paused.  Returns the two (ok, size) tuples for summary.

    Pause/resume rhythm:
      1. pipeline.stop()            — ~50ms, frees camera
      2. small settle (100ms) so the CSI queue drains
      3. camera.jpeg(80)            — ~120ms, full-res color
      4. flow.m4_gray_snap(buf)     — pure memory read, ~50us
      5. pipeline.start(...)        — ~200ms, re-creates PrepTask
    """
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(100)
    jpg_ok = 0
    gray_ok = 0
    try:
        jpg = sentai.camera.jpeg(80)
        p = "%s/%s_cam0.jpg" % (shots_dir, tag)
        sentai.fs.write(p, jpg)
        jpg_ok = len(jpg)
        jpg = None; gc.collect()
    except Exception as e:
        print("  WARN jpeg %s: %s" % (tag, e))
    try:
        # MP embed on this board omits `bytearray` (MICROPY_PY_BUILTINS_
        # BYTEARRAY off), so we take the alloc-path of m4_gray_snap —
        # 1200 bytes per corner × 5 corners = 6 KB of heap churn,
        # trivial with the 1 MB MP GC heap.
        gray_bytes = sentai.flow.m4_gray_snap()
        p = "%s/%s_gray40x30.pgm" % (shots_dir, tag)
        sentai.fs.write(p, b"P5\n40 30\n255\n" + gray_bytes)
        gray_ok = len(gray_bytes)
        gray_bytes = None
    except Exception as e:
        print("  WARN gray %s: %s" % (tag, e))
    # Restart pipeline + flow.
    sentai.pipeline.prep_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)
    sentai.flow.m4_start(0)
    sentai.rtos.sleep_ms(150)
    return jpg_ok, gray_ok

# Start pipeline + flow before the countdown so MOVE phases have
# active flow ingest from frame 1.
sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)
sentai.flow.m4_start(0)
sentai.rtos.sleep_ms(200)

# Countdown blinks
print("[E34] FLICKER=stai, SOLID=misca.  3..2..1..")
for _ in range(3):
    sentai.io.led_on();  sentai.rtos.sleep_ms(100)
    sentai.io.led_off(); sentai.rtos.sleep_ms(900)

samples = []
pos_fw = pos_left = 0
last_seq = 0
t0 = _ticks()

for phase_idx, (name, dur, mode) in enumerate(PHASES):
    print("[E34] phase %d %s (%d ms, %s)" % (phase_idx, name, dur, mode))
    pstart = _ticks()
    shot_done = False
    if mode == "solid":
        sentai.io.led_on()
    while _ticks() - pstart < dur:
        if mode == "flicker":
            _flick(pstart, _ticks())
            # Take the dual snapshot once, ~1s in (user holding still).
            if (not shot_done) and (_ticks() - pstart > 1000):
                j, g = _capture_corner(name)
                print("  shot %s: jpg=%dB gray=%dB" % (name, j, g))
                shot_done = True
                # Flow is off during the pause; don't sample during this
                # window — skip to end of phase cleanly.
        if mode == "solid":
            r = sentai.flow.m4_body_read()
            if r["alive"] and r["frame_seq"] != last_seq and r["frame_seq"] != 0:
                last_seq = r["frame_seq"]
                dx, dy = r["dx"], r["dy"]
                bfw, bleft = r["body_fw"], r["body_left"]
                pos_fw   += bfw
                pos_left += bleft
                r2 = sentai.flow.m4_read()
                conf = r2["confidence"]
                samples.append((_ticks() - t0, phase_idx, name, last_seq,
                                dx, dy, bfw, bleft, pos_fw, pos_left, conf))
        sentai.rtos.sleep_ms(POLL_MS)

sentai.io.led_off()
sentai.flow.m4_stop()
sentai.pipeline.stop()

n = len(samples)
m4 = sentai.flow.m4_read()
print("[E34] N=%d  m4_frames=%d" % (n, m4["frames_processed"]))
if n:
    fwmin = min(s[8] for s in samples); fwmax = max(s[8] for s in samples)
    lfmin = min(s[9] for s in samples); lfmax = max(s[9] for s in samples)
    print("  pos_fw   [%d .. %d]" % (fwmin, fwmax))
    print("  pos_left [%d .. %d]" % (lfmin, lfmax))
    print("  close-loop: pos_fw=%d pos_left=%d"
          % (samples[-1][8], samples[-1][9]))

csv_path = _save_path("e34_flow_square_dualcap")
save_csv(csv_path,
    ["t_ms", "phase_idx", "phase", "frame_seq",
     "dx", "dy", "body_fw", "body_left",
     "pos_fw", "pos_left", "confidence"],
    samples)
_save_desc(csv_path,
    "E34 - Square walk with dual corner capture.\n"
    "MOVE phases sampled via flow.m4_body_read (25 Hz); WAIT phases\n"
    "briefly pause pipeline to grab cam0 JPEG + M4 gray snapshot.\n"
    "Corners at shots_dir; CSV here for 2D trace plotting.\n",
    params={"poll_ms": POLL_MS, "samples": n,
            "m4_frames_processed": m4["frames_processed"],
            "shots_dir": shots_dir})
_record("e34_flow_square_dualcap", csv_path, "N=%d" % n)

if owned: end()
