# _t_visual_ab.py — capture cam0 + cam1 tagged JPEGs at alt 1:1.
# SELF-CONTAINED: inlines a minimal session helper.
#
# IMPORTANT (build #905+): LFS sustained-write throughput is below
# the camera capture rate at VGA45.  Doing save_jpeg → read → write
# → remove per iteration in a tight loop wedges the LFS task after
# 2-3 frames.  Two-phase design:
#   PHASE A: capture-all-to-RAM via sentai.camera.jpeg() — returns
#            JPEG bytes directly, NO LFS WRITE during capture.
#            Tag the cam_id immediately after each grab.
#   PHASE B: drain to LFS at the end with a generous sleep between
#            writes (200 ms) so the lfs_task has time to flush.
import sentai
sentai.verbose(1)


def _wipe_prior_visual_ab():
    removed_dirs = 0
    removed_files = 0
    try:
        entries = sentai.fs.ls("/diags")
    except Exception:
        return 0, 0
    for name, kind, _size in entries:
        if kind != 2:
            continue
        if not name.endswith("_visual_ab"):
            continue
        path = "/diags/" + name
        try:
            for f in sentai.fs.ls(path):
                try:
                    sentai.fs.remove(path + "/" + f[0])
                    removed_files += 1
                except Exception:
                    pass
            sentai.fs.remove(path)
            removed_dirs += 1
        except Exception:
            pass
    return removed_dirs, removed_files


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


print("=== boot ===")
print("version=", sentai.version())

print("--- pre-clean LFS ---")
rd, rf = _wipe_prior_visual_ab()
print("  removed_dirs=%d  removed_files=%d" % (rd, rf))

sentai.camera.init(1)
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(800)

sess_dir = _session_dir("visual_ab")
print("session dir:", sess_dir)

build_id = sentai.version().split("build")[1].split()[0]
N = 8

# ===== PHASE A: capture-all-to-RAM (no LFS pressure) =====
print("--- PHASE A: %d captures to RAM ---" % N)
caps = []  # list of (i, cam_id, jpeg_bytes)
for i in range(N):
    data = sentai.camera.jpeg(30)        # bytes — no LFS write
    cam = sentai.camera.grabbed_id()
    caps.append((i, cam, data))
    print("  i=%d cam=%d (%d bytes)" % (i, cam, len(data)))
    sentai.diag.repl_kick()
    sentai.rtos.sleep_ms(60)             # frame interval ~33 ms at VGA45

print("--- buf_id snapshot (before LFS drain) ---")
d = sentai.camera.buf_id_dump()
print("  ISR  slots=(%d,%d,%d,%d) writes=%d miss=%d skip_both=%d" %
      (d[0], d[1], d[2], d[3], d[4], d[5], d[6]))
print("  TASK slots=(%d,%d,%d,%d) writes=%d unknown=%d" %
      (d[7], d[8], d[9], d[10], d[11], d[12]))
print("  CSI  slots=(%d,%d,%d,%d) hook_fires=%d" %
      (d[13], d[14], d[15], d[16], d[17]))

# Stop alternating now — saves bus bandwidth during the LFS drain.
sentai.camera.ratio(0, 0)
sentai.camera.select(0)

# ===== PHASE B: drain to LFS at human-paced rate =====
print("--- PHASE B: drain %d JPEGs to LFS ---" % N)
for (i, cam, data) in caps:
    final = "%s/b%s_i%d_cam%d.jpg" % (sess_dir, build_id, i, cam)
    try:
        sentai.fs.write(final, data)
        print("  wrote %s (%d B)" % (final, len(data)))
    except Exception as e:
        print("  i=%d write failed: %s" % (i, e))
    sentai.diag.repl_kick()
    sentai.rtos.sleep_ms(200)            # let lfs_task flush

print("=== done ===")
