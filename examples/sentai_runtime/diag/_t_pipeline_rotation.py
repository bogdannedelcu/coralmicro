# _t_pipeline_rotation.py — rotate 3 p3p4 models through the pipeline
# in a SINGLE boot (no sys.reset between cycles) to see whether
# repeated start/stop/load cycles degrade performance.
#
# Sequence: 3 cycles × [MSBlock → C2f → GELAN], each cycle calls
# pipeline.calibrate(model, NB, 5000) which internally does
# tpu.load + pipeline.start + sample N frames + pipeline.stop.
#
# Mode: 1cam (no MUX flips — isolates pure TPU+camera throughput),
# VGA45 sensor.  CSV columns: cycle, slot, model, calibrate_wall_ms,
# frames, invoke_avg, invoke_min, invoke_max, total_avg, fps_x100.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.
import sentai
sentai.verbose(1)

MODELS = [
    ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite"),
]
FPS    = 45
CYCLES = 3
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


sess = _sd("pipeline_rotation")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "cycle,slot,model,wall_ms,frames,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\r\n")
print("=== session:", sess, "fps=%d cycles=%d ===" % (FPS, CYCLES))

# Camera up at VGA45, no alt — 1cam mode, ratio(0,0).
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

for cycle in range(CYCLES):
    print("\n========== cycle %d/%d ==========" % (cycle + 1, CYCLES))
    for slot, (label, path) in enumerate(MODELS):
        print("--- cycle=%d slot=%d %s ---" % (cycle, slot, label))
        # Make sure pipeline is fully stopped (idempotent if not running).
        try:
            if sentai.pipeline.running(): sentai.pipeline.stop()
        except Exception as e: print("  stop exc:", e)
        sentai.rtos.sleep_ms(50)
        # Wall-clock for the whole calibrate call (load + start + N frames + stop).
        t0 = sentai.rtos.ticks_ms()
        res = sentai.pipeline.calibrate(path, NB, 5000)
        wall = sentai.rtos.ticks_ms() - t0
        n   = res.get("frames", 0) or 1
        ia  = res.get("invoke_ms_sum", 0) // n
        ta  = res.get("total_ms_sum", 0) // n
        f100 = res.get("fps_x100", 0)
        print("  wall=%d ms frames=%d invoke avg/min/max=%d/%d/%d total_avg=%d fps=%d.%02d" %
              (wall, n, ia, res.get("invoke_ms_min", 0), res.get("invoke_ms_max", 0),
               ta, f100 // 100, f100 % 100))
        sentai.fs.append(csv_path,
            "%d,%d,%s,%d,%d,%d,%d,%d,%d,%d\r\n" %
            (cycle, slot, label, wall, n, ia,
             res.get("invoke_ms_min", 0), res.get("invoke_ms_max", 0),
             ta, f100))
        sentai.rtos.sleep_ms(100)

# Final stop.
try: sentai.pipeline.stop()
except Exception: pass

print("\nCSV:", csv_path)
print("=== done ===")
