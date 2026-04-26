# _t_pattern_save.py — capture 1 JPEG per camera per pattern,
# save to LFS for host-side byte inspection with PIL.
import sentai
sentai.verbose(1)


def _wipe(prefix="/diags/pattern_"):
    try:
        for n,k,_ in sentai.fs.ls("/diags"):
            if k == 2 and n.endswith("_pattern_save"):
                p = "/diags/" + n
                try:
                    for f in sentai.fs.ls(p): sentai.fs.remove(p+"/"+f[0])
                    sentai.fs.remove(p)
                except Exception: pass
    except Exception: pass


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


print("=== boot ===")
print("version=", sentai.version())
_wipe()
sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

sd = _session_dir("pattern_save")
print("session dir:", sd)


def snap(cam_id, mode, name):
    sentai.camera.test_pattern(cam_id, mode)
    sentai.camera.select(cam_id)
    sentai.rtos.sleep_ms(400)
    # drain stale frames
    for _ in range(3):
        sentai.camera.peek_row(8)
    f = "%s/%s.jpg" % (sd, name)
    sentai.camera.save_jpeg(f, 80)         # quality 80 — preserve fidelity
    print("  saved %s (mode=%d cam=%d)" % (f, mode, cam_id))
    sentai.diag.repl_kick()
    sentai.rtos.sleep_ms(150)


# Each combination — name encodes (cam, mode_label)
print("--- captures ---")
snap(0, 0, "cam0_real")
snap(1, 0, "cam1_real")
snap(0, 1, "cam0_BARS")
snap(1, 1, "cam1_BARS")
snap(0, 2, "cam0_HBAND")
snap(1, 2, "cam1_HBAND")

# Restore
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)
print("=== done ===")
