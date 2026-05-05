# _t_two_model_pipeline_alt.py — 2-model alternation IN PIPELINE.
#
# Companion to _t_two_model_alt.py (pure-TPU).  Same A/B rotation but
# inside pipeline.calibrate (load + start + 100 frames + stop).
# 5 cycles × 2 models = 10 calibrations.
#
# Question: does rapid 2-model alternation in pipeline degrade FPS or
# carry a different swap cost than the 3-model rotation seen in
# _t_pipeline_rotation.py (~100 ms/swap)?  And does the MSBlock
# slow-reload anomaly we saw pure-TPU also appear here?
#
# Self-contained per agent.md §5.1.2 + §5.1.5.
import sentai
sentai.verbose(1)

A = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
B = ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
FPS    = 45
CYCLES = 5
NB     = 100


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


sess = _sd("two_model_pipeline_alt")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "cycle,slot,label,wall_ms,frames,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\r\n")
print("=== session:", sess, "fps=%d cycles=%d ===" % (FPS, CYCLES))

rc = sentai.camera.init(1, FPS)
print("init(1, %d) =" % FPS, rc)
if rc == -11:
    sentai.rtos.sleep_ms(200); sentai.sys.reset()
elif rc != 0:
    print("FAIL init=%d" % rc); print("=== done ==="); raise SystemExit

sentai.rtos.sleep_ms(800)
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)


def _phase(label, path, cycle, slot):
    print("--- cycle=%d slot=%d %s ---" % (cycle, slot, label))
    try:
        if sentai.pipeline.running(): sentai.pipeline.stop()
    except Exception: pass
    sentai.rtos.sleep_ms(50)
    t0 = sentai.rtos.ticks_ms()
    res = sentai.pipeline.calibrate(path, NB, 5000)
    wall = sentai.rtos.ticks_ms() - t0
    n = res.get("frames", 0) or 1
    ia = res.get("invoke_ms_sum", 0) // n
    ta = res.get("total_ms_sum", 0) // n
    f100 = res.get("fps_x100", 0)
    print("  wall=%d ms frames=%d invoke avg/min/max=%d/%d/%d total_avg=%d fps=%d.%02d" %
          (wall, n, ia, res.get("invoke_ms_min", 0), res.get("invoke_ms_max", 0),
           ta, f100 // 100, f100 % 100))
    sentai.fs.append(csv_path,
        "%d,%d,%s,%d,%d,%d,%d,%d,%d,%d\r\n" %
        (cycle, slot, label, wall, n, ia,
         res.get("invoke_ms_min", 0), res.get("invoke_ms_max", 0),
         ta, f100))


for c in range(CYCLES):
    print("\n========== cycle %d/%d ==========" % (c + 1, CYCLES))
    _phase(A[0], A[1], c, 0)
    _phase(B[0], B[1], c, 1)

try: sentai.pipeline.stop()
except Exception: pass
print("\nCSV:", csv_path)
print("=== done ===")
