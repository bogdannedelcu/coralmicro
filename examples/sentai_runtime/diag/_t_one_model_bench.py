# _t_one_model_bench.py — bench EXACTLY ONE model and append to a
# shared CSV.  Caller seeds REPL globals before exec'ing this file:
#   _target_model = "/models/..."
#   _target_w     = 640
#   _target_h     = 480
#   _csv_path     = "/diags/sXXX_models_bench/results.csv"  (must exist
#                   with header; this driver only appends)
#
# Per agent.md §2.9: cross-test contamination is real — each model runs
# on a fresh-flashed board.  This driver loads the requested model
# only, runs warmup + 30 timed invokes, appends one CSV row, and exits.
# A wedge is contained to that single boot — host orchestrator
# detects the disconnect, marks the model as failed, reflashes and
# moves on.
import sentai

sentai.verbose(1)
build_id = sentai.version().split("build")[1].split()[0]
print("=== one_model_bench start (build %s) ===" % build_id)
print("model:", _target_model, "input:", _target_w, "x", _target_h)

WARMUP = 3
N      = 30

label = _target_model.split("/")[-1].replace(".tflite", "")
note  = ""

try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

try: sentai.camera.set_resolution(_target_w, _target_h)
except Exception as e: note += "set_res:%s " % str(e).replace(",", " ")

try:
    if sentai.camera.frame_count() == 0: sentai.camera.init(1)
except Exception as e: note += "cam_init:%s " % str(e).replace(",", " ")

# Load model.
load_ms = -1
t0 = sentai.rtos.ticks_ms()
try:
    rc_load = sentai.tpu.load(_target_model)
    load_ms = sentai.rtos.ticks_ms() - t0
    print("tpu.load rc=%s in %dms" % (rc_load, load_ms))
except Exception as e:
    note += "load_exc:%s " % str(e).replace(",", " ")
    print("tpu.load EXC:", e)
    sentai.fs.append(_csv_path,
        '%s,%s,%d,%d,,,,,,,,,LOAD_EXC %s\r\n' %
        (label, _target_model, _target_w, _target_h, note.strip()))
    print("=== done ===")

else:
    try: sentai.camera.to_tensor()
    except Exception as e: note += "to_tensor:%s " % str(e).replace(",", " ")
    sentai.rtos.sleep_ms(50)
    # Warmup.
    warm_fail = 0
    for i in range(WARMUP):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception as e:
            rc = -1; warm_fail += 1
            print("warmup #%d invoke EXC: %s" % (i, e))
        t1 = sentai.rtos.ticks_ms()
        print("warmup #%d: %dms rc=%s" % (i, t1 - t0, rc))
        sentai.diag.repl_kick()
    if warm_fail >= WARMUP:
        sentai.fs.append(_csv_path,
            '%s,%s,%d,%d,,,,,,,,,WARMUP_ALL_FAIL %s\r\n' %
            (label, _target_model, _target_w, _target_h, note.strip()))
        print("=== done ===")
    else:
        try:
            sentai.diag.tpu_call_stats(1)
            s_pre = sentai.diag.tpu_call_stats()
        except Exception:
            s_pre = {'p_calls':0,'i_calls':0,'in_calls':0,
                     'p_bytes':0,'i_bytes':0,'in_bytes':0}
        # Bench.
        samples_ms = []
        invoke_fail = 0
        for i in range(N):
            t0 = sentai.rtos.ticks_ms()
            try: rc = sentai.tpu.invoke()
            except Exception:
                rc = -1
            t1 = sentai.rtos.ticks_ms()
            if isinstance(rc, int) and rc < 0:
                invoke_fail += 1
            samples_ms.append(t1 - t0)
            if (i & 0x07) == 0: sentai.diag.repl_kick()
            if invoke_fail > 5:
                note += "ABORT_after_5_invoke_fails "
                break
        try: s_post = sentai.diag.tpu_call_stats()
        except Exception: s_post = s_pre
        samples_ms.sort()
        n_real = len(samples_ms)
        if n_real == 0:
            sentai.fs.append(_csv_path,
                '%s,%s,%d,%d,,,,,,,,%d,NO_SAMPLES %s\r\n' %
                (label, _target_model, _target_w, _target_h,
                 invoke_fail, note.strip()))
        else:
            median = samples_ms[n_real // 2]
            p99    = samples_ms[min(n_real - 1, int(n_real * 0.99))]
            mn     = sum(samples_ms) // n_real
            fps    = (1000 // median) if median > 0 else 0
            p_per   = (s_post['p_bytes']  - s_pre['p_bytes'])  // n_real
            i_per   = (s_post['i_bytes']  - s_pre['i_bytes'])  // n_real
            in_per  = (s_post['in_bytes'] - s_pre['in_bytes']) // n_real
            print("N=%d med=%dms p99=%dms mean=%dms fail=%d fps≈%d" %
                  (n_real, median, p99, mn, invoke_fail, fps))
            print("USB/invoke: p=%dB i=%dB in=%dB total=%dB" %
                  (p_per, i_per, in_per, p_per+i_per+in_per))
            sentai.fs.append(_csv_path,
                '%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\r\n' %
                (label, _target_model, _target_w, _target_h,
                 median, p99, mn, fps, p_per, i_per, in_per,
                 invoke_fail, note.strip()))
        print("=== done ===")
