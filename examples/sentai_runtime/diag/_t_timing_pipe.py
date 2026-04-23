# Pipeline timing breakdown — 5 runs for variance, on fresh boot
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_RUNS = 5

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

print("=== PIPELINE 5 runs of 5s each ===")
for r in range(N_RUNS):
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    t0 = sentai.rtos.ticks_ms()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(5000)
    pstats = sentai.pipeline.prep_stats()
    istats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    dt = sentai.rtos.ticks_ms() - t0
    f = max(pstats['frames'], 1)
    ok = max(istats['ok'], 1)
    print("run%d dur=%dms  prep=%dframes(%.1ffps) infer_ok=%d(%.1ffps) fail=%d" % (
        r, dt, pstats['frames'], pstats['frames']*1000.0/dt,
        istats['ok'], istats['ok']*1000.0/dt, istats['fail']))
    print("  per-frame ms: cam=%.2f pxp=%.2f quant=%.2f sem_wait=%.2f total_prep=%.2f invoke=%.2f" % (
        pstats['cam_grab_ms_sum']/f,
        pstats['pxp_ms_sum']/f,
        pstats['quant_ms_sum']/f,
        pstats['sem_wait_ms_sum']/f,
        pstats['total_ms_sum']/f,
        istats['ms_sum']/ok))
    sentai.rtos.sleep_ms(500)  # settle between runs

print("=== done ===")
