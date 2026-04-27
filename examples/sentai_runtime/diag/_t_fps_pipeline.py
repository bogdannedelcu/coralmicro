# _t_fps_pipeline.py — full TPU pipeline benchmark per fps × ratio.
#
# Companion to _t_fps_bench.py.  After camera parity is validated by
# _t_fps_bench (cam_id correctness ceiling 100/100), this driver
# measures the END-TO-END pipeline FPS — PrepTask (PXP scale + cam
# alternation) + InferTask (TPU yolo_1 512×512) — at the same
# fps × ratio matrix.  Output goes alongside the camera-parity
# numbers so the user sees how much TPU subtracts from the raw
# sensor + drain ceiling.
#
# Self-correcting init: if the firmware booted at a different fps,
# init(1, _target_fps) returns -11 → script calls sys.reset(); the
# host runner re-issues the same command after re-enumeration.
#
# Persistent flash REQUIRED (sys.reset workflow).
#
# How to drive:
#   python3 diag/_host_run_with_var.py \
#       --file /lib/diag/_t_fps_pipeline.py \
#       --set _target_fps=30 --timeout 240
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
NB = 100  # frames per ratio


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
    # calibrate auto-loads the model + auto-starts the pipeline.
    # calibrate is a positional-only MP binding: (model, frames,
    # timeout_ms, delay_ms, conf, iou).  Keyword form raises
    # TypeError on this firmware.
    res = sentai.pipeline.calibrate(MODEL, NB, 2000)
    sentai.diag.repl_kick()
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
    # No test patterns this time — pipeline runs on whatever the
    # cameras see (synthetic scene, lab bench, etc.).  cam_id
    # correctness is already validated by _t_fps_bench, so this
    # driver focuses purely on throughput.
    sentai.camera.select(0); sentai.rtos.sleep_ms(300)

    res = []
    print("--- pipeline 1cam ---"); res.append(("1cam", _calibrate_at("1cam", 0, 0)))
    print("--- pipeline 1:1 ---");  res.append(("1:1",  _calibrate_at("1:1",  1, 1)))
    print("--- pipeline 2:1 ---");  res.append(("2:1",  _calibrate_at("2:1",  2, 1)))
    print("--- pipeline 3:1 ---");  res.append(("3:1",  _calibrate_at("3:1",  3, 1)))

    sentai.pipeline.stop()
    sentai.camera.ratio(0, 0)

    print("--- summary fps=%d ---" % _fps)
    print("  mode | frames | cam0:cam1 | invoke ms (avg/min/max) | total ms (avg) | pipeline fps")
    for label, r in res:
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
    L = ["fps,mode,frames,cam0,cam1,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\n"]
    for label, r in res:
        n = r.get("frames", 0) or 1
        L.append("%d,%s,%d,%d,%d,%d,%d,%d,%d,%d\n" %
                 (_fps, label, n, r.get("cam0", 0), r.get("cam1", 0),
                  r.get("invoke_ms_sum", 0) // n,
                  r.get("invoke_ms_min", 0),
                  r.get("invoke_ms_max", 0),
                  r.get("total_ms_sum", 0) // n,
                  r.get("fps_x100", 0)))
    sentai.fs.write(sd + "/pipeline.csv", "".join(L))
    print("log:", sd + "/pipeline.csv")
    print("=== done ===")
