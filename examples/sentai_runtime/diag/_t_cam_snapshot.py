# _t_cam_snapshot.py -- one JPEG per camera, identifiable by filename.
#
# Output:
#   /diags/sNNN_cam_snap/cam0_front.jpg
#   /diags/sNNN_cam_snap/cam1_back.jpg
#
# Drains a few frames after each camera.select to let the freshly-armed
# camera produce a clean (non-mixed) frame before capture.
import sentai

DRAIN_FRAMES   = 10        # frame_count() ticks (FB2-gated) ~= 20 sensor frames
SETTLE_MS      = 800       # extra wall sleep after select before grabs
GRAB_PURGE_N   = 8         # explicit buffer queue purge (grab+return)
JPEG_Q         = 80


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception:
        sid = 1
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d, sid


def _drain(n):
    # Wait for n fresh sensor frames from the new camera.
    fc0 = sentai.camera.frame_count()
    deadline = sentai.rtos.ticks_ms() + 2000
    while sentai.camera.frame_count() - fc0 < n and sentai.rtos.ticks_ms() < deadline:
        sentai.rtos.sleep_ms(20)


def _purge_buffer_queue(n):
    # Force CameraTask's buffer pool to rotate by explicitly grabbing
    # and returning n frames.  After this, the most-recent buffer is
    # guaranteed to come from the post-switch camera (since fewer than
    # n buffers are in the pool, all stale ones are flushed).
    for _ in range(n):
        try:
            sentai.camera.discard()
        except AttributeError:
            # No discard binding; fall back to to_tensor() which is a
            # buffer-consuming no-op for our purpose.
            try:
                sentai.camera.to_tensor()
            except Exception:
                pass
        sentai.rtos.sleep_ms(5)


def main():
    sess, sid = _session_dir("cam_snap")
    print("session:", sess)

    sentai.usb.ip(1)
    rc = sentai.camera.init(1)
    print("camera.init:", rc)
    if rc != 0 and rc != -11:
        print("FAIL camera.init=%d" % rc)
        print("=== done ==="); return

    for cam_id, label in [(0, "front"), (1, "back")]:
        rc = sentai.camera.select(cam_id)
        print("select(%d) rc=%s" % (cam_id, rc))
        _drain(DRAIN_FRAMES)
        sentai.rtos.sleep_ms(SETTLE_MS)
        _purge_buffer_queue(GRAB_PURGE_N)
        # Confirm the next frame really is from cam_id by reading the
        # per-buffer tag (set by CSI ISR at frame completion).
        try:
            tag = sentai.camera.grabbed_id()
            print("  grabbed_id post-purge=%s (expected %d)" % (tag, cam_id))
        except Exception as e:
            print("  grabbed_id unavailable: %s" % e)
        path = "%s/cam%d_%s.jpg" % (sess, cam_id, label)
        try:
            sentai.camera.save_jpeg(path, JPEG_Q)
            print("saved:", path)
        except Exception as e:
            print("FAIL save cam%d: %s" % (cam_id, e))

    print("session=%s sid=%d" % (sess, sid))
    print("=== done ===")


main()
