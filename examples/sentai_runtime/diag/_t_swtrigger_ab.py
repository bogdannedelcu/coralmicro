# _t_swtrigger_ab.py — software trigger via OV5640 reg 0x3008.
# 0x42 = standby (sensor stopped, no streaming)
# 0x02 = normal operation (sensor streaming)
#
# Sequence per capture:
#   1. Stop both sensors (0x42 on whoever's active)
#   2. Set MUX to target cam
#   3. Start that sensor (0x02)
#   4. Wait for one fresh frame
#   5. Grab + save
#   6. Stop sensor (0x42)
#
# cam_id is GUARANTEED correct by construction — we explicitly chose
# which camera streamed, no race possible.
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


def cam_stop(cid):
    sentai.camera.reg_write(cid, 0x3008, 0x42)


def cam_start(cid):
    sentai.camera.reg_write(cid, 0x3008, 0x02)


def wait_n_frames(n=1, timeout_ms=400):
    start = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    while (sentai.camera.frame_count() - start) < n:
        if (sentai.rtos.ticks_ms() - t0) > timeout_ms:
            return False
        sentai.rtos.sleep_ms(2)
    return True


print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.rtos.sleep_ms(500)

# Initial state: both sensors stopped
sentai.camera.select(0); sentai.rtos.sleep_ms(50); cam_stop(0)
sentai.camera.select(1); sentai.rtos.sleep_ms(50); cam_stop(1)
print("--- both sensors stopped ---")

sess_dir = _session_dir("swtrigger_ab")
print("session dir:", sess_dir)
build_id = sentai.version().split("build")[1].split()[0]


def snap_one(cid, label):
    sentai.camera.select(cid)
    sentai.rtos.sleep_ms(50)
    cam_start(cid)
    wait_n_frames(2)  # let AEC settle, drain initial frame
    final = "%s/b%s_cam%d_%s.jpg" % (sess_dir, build_id, cid, label)
    sentai.camera.save_jpeg(final, 30)
    cam_stop(cid)
    sentai.diag.repl_kick()
    print("  cam%d %s -> %s" % (cid, label, final))


# Alternate cam0/cam1 like real alt-mode would, but with explicit
# stop-set-start-grab-stop sequence per capture.
print("--- alternating snapshots ---")
for i in range(4):
    snap_one(0, "i%d" % i)
    snap_one(1, "i%d" % i)

print("=== done ===")
