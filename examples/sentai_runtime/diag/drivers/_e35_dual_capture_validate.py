# _e35_dual_capture_validate.py — validate dual JPEG + gray capture
# cycle WITHOUT any square walk.  Goal: check that we can repeatedly
# stop the pipeline, snapshot BOTH a color JPEG and the 40×30 gray
# that M4 sees, restart the pipeline, and continue — without the
# board hanging or the MicroPython REPL-idle watchdog firing.
#
# Protocol (5 iterations):
#   1. Solid LED 1s  (you can move the board a bit between frames)
#   2. Flicker 200ms (settle)
#   3. pipeline.stop()
#   4. sentai.flow.m4_gray_snap(buf)     → PGM
#   5. sentai.camera.jpeg(80)             → JPEG
#   6. pipeline.start() + flow.m4_start(0)
#   7. Print timings (so we see if any phase is pathological)
#
# Each phase has a timing budget; anything past the budget is flagged
# so we can diagnose which call in the sequence is slow.

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

from diag._session import _session, begin, end, _photos_dir
from diag._util import _ticks

owned = (_session is None)
if owned:
    begin("e35_dual_capture")

shots_dir = _photos_dir("e35_shots") or "/diags/e35_shots"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

ITERS = 5

# ── Bring-up M4 once ────────────────────────────────────────────
t0 = _ticks()
rc = sentai.flow.m4_enable()
print("[E35] m4_enable rc=%d elapsed=%dms" % (rc, _ticks() - t0))

# MicroPython embed on this board doesn't expose `bytearray`, so we
# fall back to the alloc path of m4_gray_snap (fresh bytes per call).
# At 5 iterations the heap cost is trivial.

for it in range(ITERS):
    print("[E35] === iter %d ===" % it)

    # Pipeline running: collect some motion samples into M4.
    t0 = _ticks()
    sentai.pipeline.prep_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)
    sentai.flow.m4_start(0)
    print("  start+m4_start : %dms" % (_ticks() - t0))

    # Let the board breathe for ~1s so scene has fresh frames.
    sentai.io.led_on()
    sentai.rtos.sleep_ms(1000)
    sentai.io.led_off()

    # Check M4 is processing real frames (frame_seq should advance).
    pre = sentai.flow.m4_read()
    print("  m4 frame_seq pre=%d conf=%d" %
          (pre["frame_seq"], pre["confidence"]))

    # Pipeline stop — measure.  If this ever exceeds ~5s it's a bug.
    t0 = _ticks()
    sentai.pipeline.stop()
    t_stop = _ticks() - t0
    print("  pipeline.stop  : %dms %s" %
          (t_stop, "" if t_stop < 5000 else "!!! TOO SLOW"))

    # Gray snap — pure memory read, should be microseconds.
    t0 = _ticks()
    gray_bytes = sentai.flow.m4_gray_snap()   # returns bytes(1200)
    t_gray = _ticks() - t0
    print("  gray_snap      : %dms (%d bytes)" %
          (t_gray, len(gray_bytes)))

    # JPEG full-res.
    t0 = _ticks()
    try:
        jpg = sentai.camera.jpeg(80)
        t_jpg = _ticks() - t0
        print("  camera.jpeg    : %dms (%d bytes)" % (t_jpg, len(jpg)))
    except Exception as e:
        jpg = None
        print("  camera.jpeg    : FAILED %s" % e)

    # Write both files AFTER the timing-sensitive section (LFS writes
    # are slow but not time-critical here).
    t0 = _ticks()
    try:
        sentai.fs.write("%s/i%02d_gray.pgm" % (shots_dir, it),
                        b"P5\n40 30\n255\n" + gray_bytes)
    except Exception as e:
        print("  gray.pgm write fail: %s" % e)
    if jpg is not None:
        try:
            sentai.fs.write("%s/i%02d_cam0.jpg" % (shots_dir, it), jpg)
        except Exception as e:
            print("  jpg write fail: %s" % e)
    jpg = None; gc.collect()
    print("  lfs writes     : %dms" % (_ticks() - t0))

# ── Final cleanup ───────────────────────────────────────────────
if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.flow.m4_stop()
print("[E35] done. shots at %s" % shots_dir)

if owned: end()
