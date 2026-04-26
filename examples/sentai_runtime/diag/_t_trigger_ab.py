# _t_trigger_ab.py — synchronous "trigger-like" capture using
# MUX-set + wait-for-fresh-frame_seq in streaming mode.
# Tag is IMPLICIT from the camera we explicitly selected, so no
# inference from buffer-tag arrays.  Goal: visual A/B should match
# 8/8 because we never alternate MUX behind the back of capture.
import sentai
sentai.verbose(1)


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception:
        sid = 1
    try:
        sentai.fs.write(counter, str(sid))
    except Exception:
        pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def wait_fresh_frames(n=2, timeout_ms=400):
    """Wait until n new sensor frames have been captured."""
    start = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    while (sentai.camera.frame_count() - start) < n:
        if (sentai.rtos.ticks_ms() - t0) > timeout_ms:
            return False
        sentai.rtos.sleep_ms(2)
    return True


print("=== boot ===")
print("version=", sentai.version())

# Streaming on, NO ratio (manual MUX), NO drain-on-grab (we wait
# explicitly via frame_seq).
sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

sess_dir = _session_dir("trigger_ab")
print("session dir:", sess_dir)
build_id = sentai.version().split("build")[1].split()[0]

# Capture 4 from cam0, then 4 from cam1.  Tag is implicit from the
# select() we just did — guaranteed correct.
print("--- explicit cam0 captures ---")
sentai.camera.select(0)
sentai.rtos.sleep_ms(100)
wait_fresh_frames(2)  # let MUX settle + 1 fresh frame
for i in range(4):
    wait_fresh_frames(1)
    final = "%s/b%s_cam0_i%d.jpg" % (sess_dir, build_id, i)
    sentai.camera.save_jpeg(final, 30)
    sentai.diag.repl_kick()
    print("  cam0 i=%d -> %s" % (i, final))

print("--- explicit cam1 captures ---")
sentai.camera.select(1)
sentai.rtos.sleep_ms(100)
wait_fresh_frames(2)
for i in range(4):
    wait_fresh_frames(1)
    final = "%s/b%s_cam1_i%d.jpg" % (sess_dir, build_id, i)
    sentai.camera.save_jpeg(final, 30)
    sentai.diag.repl_kick()
    print("  cam1 i=%d -> %s" % (i, final))

print("=== done ===")
