# _t_fps_pipeline.py - probe-driven TPU pipeline benchmark per fps.
#
# Uses sentai.pipeline.probe_ratios() to auto-discover which (a,b)
# ratios produce a captured cam0:cam1 distribution within tolerance
# for the current (model, fps, sensor period) tuple, then runs a
# full calibrate() on each working ratio plus the 1cam (no-switch)
# baseline.  Aliased ratios (e.g. VGA60+2:1) are reported as
# excluded with reason — no consumer-side workaround.
#
# Self-correcting init via the same -11 -> sys.reset() idiom as
# _t_fps_bench.py.  Persistent flash REQUIRED.
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
NB    = 100  # frames per calibrate
PROBE_FRAMES = 50
TOL_PCT      = 10


def _sd(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    c = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(c, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _calibrate_at(label, ra, rb):
    sentai.camera.ratio(ra, rb)
    sentai.camera.switch_drain(1)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    res = sentai.pipeline.calibrate(MODEL, NB, 2000)
    sentai.rtos.sleep_ms(50)
    return res


try: _fps = int(_target_fps)  # noqa: F821
except Exception: _fps = 30

print("=== boot ===")
print("--- target fps:", _fps, "---")
rc = sentai.camera.init(1, _fps)
print("init(1, %d) =" % _fps, rc)
if rc == -11:
    print("mismatch -> reset")
    sentai.rtos.sleep_ms(200)
    sentai.sys.reset()
elif rc != 0:
    print("FAIL init=%d" % rc)
    print("=== done ===")
else:
    sentai.rtos.sleep_ms(800)
    sentai.camera.select(0); sentai.rtos.sleep_ms(300)

    # ------ Probe ratios first, pick only the working ones ------
    print("--- probing ratios @ fps=%d ---" % _fps)
    probe = sentai.pipeline.probe_ratios(MODEL, PROBE_FRAMES, TOL_PCT)
    for p in probe:
        a, b = p["ratio"]
        flag = "OK" if p["ok"] else "SKIP"
        print("  %d:%d %4s cam0:cam1=%d:%d (%d frames) reason=%s" %
              (a, b, flag, p["cam0"], p["cam1"], p["frames"], p["reason"]))

    # ------ Run full pipeline calibrate on 1cam + working ratios ------
    res = []
    print("--- pipeline 1cam ---"); res.append(("1cam", 0, 0, _calibrate_at("1cam", 0, 0)))
    for p in probe:
        if not p["ok"]: continue
        a, b = p["ratio"]
        label = "%d:%d" % (a, b)
        print("--- pipeline %s ---" % label)
        res.append((label, a, b, _calibrate_at(label, a, b)))

    sentai.pipeline.stop()
    sentai.camera.ratio(0, 0)

    print("--- summary fps=%d ---" % _fps)
    print("  mode | frames | cam0:cam1 | invoke ms (avg/min/max) | total ms (avg) | pipeline fps")
    for label, _a, _b, r in res:
        n   = r.get("frames", 0) or 1
        c0  = r.get("cam0", 0)
        c1  = r.get("cam1", 0)
        ia  = (r.get("invoke_ms_sum", 0) // n)
        imn = r.get("invoke_ms_min", 0)
        imx = r.get("invoke_ms_max", 0)
        ta  = (r.get("total_ms_sum", 0) // n)
        f100 = r.get("fps_x100", 0)
        print("  %-4s | %6d | %3d:%-3d   | %3d / %3d / %3d         | %3d            | %d.%02d" %
              (label, n, c0, c1, ia, imn, imx, ta, f100 // 100, f100 % 100))

    sd = _sd("pipeline_%d" % _fps)
    L = ["fps,mode,a,b,frames,cam0,cam1,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\n"]
    for label, a, b, r in res:
        n = r.get("frames", 0) or 1
        L.append("%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" %
                 (_fps, label, a, b, n, r.get("cam0", 0), r.get("cam1", 0),
                  r.get("invoke_ms_sum", 0) // n,
                  r.get("invoke_ms_min", 0),
                  r.get("invoke_ms_max", 0),
                  r.get("total_ms_sum", 0) // n,
                  r.get("fps_x100", 0)))
    # also save the probe report
    P = ["fps,a,b,ok,cam0,cam1,frames,reason\n"]
    for p in probe:
        a, b = p["ratio"]
        P.append("%d,%d,%d,%d,%d,%d,%d,%s\n" %
                 (_fps, a, b, 1 if p["ok"] else 0,
                  p["cam0"], p["cam1"], p["frames"], p["reason"]))
    sentai.fs.write(sd + "/pipeline.csv", "".join(L))
    sentai.fs.write(sd + "/probe.csv", "".join(P))
    print("log:", sd + "/pipeline.csv")
    print("=== done ===")
