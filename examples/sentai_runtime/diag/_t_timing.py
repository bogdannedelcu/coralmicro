# Detailed per-stage timing breakdown with variance across runs
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_RUNS = 5       # repeat each measurement N times for variance
N_INVOKES = 30   # invokes per measurement

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(5): sentai.tpu.invoke()  # warm
print("warmed")

def measure_pure_tpu():
    sentai.diag.tpu_perf(1)  # reset DWT counters
    t0 = sentai.rtos.ticks_ms()
    for i in range(N_INVOKES):
        sentai.tpu.invoke()
    t1 = sentai.rtos.ticks_ms()
    perf = sentai.diag.tpu_perf()
    # DWT cycles @ 800 MHz = ms × 800_000
    # Per-invoke breakdowns
    n = perf['n_ins']
    def cyc_to_ms(total_cyc, n_calls):
        if n_calls == 0: return 0
        return (total_cyc / 800000.0) / N_INVOKES
    return {
        'total_ms': (t1 - t0) / N_INVOKES,
        'input_ms':  cyc_to_ms(perf['cyc_input'],  perf['n_input']),
        'params_ms': cyc_to_ms(perf['cyc_params'], perf['n_params']),
        'ins_ms':    cyc_to_ms(perf['cyc_ins'],    perf['n_ins']),
        'output_ms': cyc_to_ms(perf['cyc_output'], perf['n_output']),
        'event_ms':  cyc_to_ms(perf['cyc_event'],  perf['n_event']),
        'by_input':  perf['by_input']//N_INVOKES,
        'by_ins':    perf['by_ins']//N_INVOKES,
        'by_output': perf['by_output']//N_INVOKES,
    }

def measure_pipeline(duration_ms=5000):
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(duration_ms)
    pstats = sentai.pipeline.prep_stats()
    istats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(200)
    f = max(pstats['frames'], 1)
    return {
        'prep_fps':    pstats['frames']/(duration_ms/1000.0),
        'infer_fps':   istats['ok']/(duration_ms/1000.0),
        'cam_grab':    pstats['cam_grab_ms_sum']/f,
        'pxp':         pstats['pxp_ms_sum']/f,
        'quant':       pstats['quant_ms_sum']/f,
        'sem_wait':    pstats['sem_wait_ms_sum']/f,
        'total_prep':  pstats['total_ms_sum']/f,
        'invoke_avg':  istats['ms_sum']/max(istats['ok'],1),
        'fails':       istats['fail'],
    }

# PURE TPU — N_RUNS measurements
print("\n=== PURE TPU variance (N=%d, %d invokes each) ===" % (N_RUNS, N_INVOKES))
pure_runs = []
for r in range(N_RUNS):
    pure_runs.append(measure_pure_tpu())
    x = pure_runs[-1]
    print("run %d: total=%.2fms  input=%.2f  params=%.2f  ins=%.2f  output=%.2f  event=%.3f" % (
        r, x['total_ms'], x['input_ms'], x['params_ms'],
        x['ins_ms'], x['output_ms'], x['event_ms']))

# bytes (invariant across runs)
b = pure_runs[0]
print("bytes/invoke: input=%dB ins=%dB output=%dB" % (
    b['by_input'], b['by_ins'], b['by_output']))

# PIPELINE — N_RUNS measurements
print("\n=== PIPELINE variance (N=%d, 5s each) ===" % N_RUNS)
pipe_runs = []
for r in range(N_RUNS):
    pipe_runs.append(measure_pipeline(5000))
    x = pipe_runs[-1]
    print("run %d: prep=%.1ffps infer=%.1ffps  cam=%.1f pxp=%.1f quant=%.1f wait=%.1f total_prep=%.1f invoke=%.1f fails=%d" % (
        r, x['prep_fps'], x['infer_fps'],
        x['cam_grab'], x['pxp'], x['quant'], x['sem_wait'],
        x['total_prep'], x['invoke_avg'], x['fails']))

print("=== done ===")
