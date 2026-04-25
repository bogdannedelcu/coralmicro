# Path A — early sem release test. Compare OFF vs ON with variance.
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(label, early, n_runs=3, dur=5000):
    print("--- %s (early_release=%d) ---" % (label, early))
    sentai.pipeline.early_release(early)
    for r in range(n_runs):
        sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        istats = sentai.pipeline.infer_stats()
        pstats = sentai.pipeline.prep_stats()
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(300)
        f = max(pstats['frames'], 1); ok = max(istats['ok'], 1)
        print("  run %d: infer_ok=%d (%.1f fps) fail=%d | avg_invoke=%d ms | prep=%d (%.1f fps) sem_wait=%d" % (
            r, istats['ok'], istats['ok']*1000.0/dur, istats['fail'],
            istats['ms_sum']//ok,
            pstats['frames'], pstats['frames']*1000.0/dur,
            pstats['sem_wait_ms_sum']//f))

run("BASELINE (serial)", 0)
run("EARLY RELEASE", 1)

sentai.pipeline.early_release(0)  # restore
print("=== done ===")
