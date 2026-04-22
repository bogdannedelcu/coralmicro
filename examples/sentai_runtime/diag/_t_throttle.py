# Test convention: prep 30 FPS, tpu 60 FPS (1:2 ratio)
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(prep_fps, target_fps):
    print("--- PREP=%d TPU=%d ---" % (prep_fps, target_fps))
    sentai.pipeline.prep_fps(prep_fps)
    sentai.pipeline.target_fps(target_fps)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAIL rc=%s" % rc)
        return
    sentai.rtos.sleep_ms(5000)
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    print("  @5s infer ok=%d fail=%d | prep=%d" % (
        istats['ok'], istats['fail'], pstats['frames']))
    if istats['ok']:
        print("    avg=%d ms  e2e_fps=%.1f  success_rate=%.1f%%" % (
            istats['ms_sum'] // istats['ok'],
            istats['ok']/5.0,
            100.0 * istats['ok'] / max(1, istats['ok']+istats['fail'])))
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(300)

# Start with throttled to avoid board hang
run(prep_fps=30, target_fps=60)
run(prep_fps=20, target_fps=40)
run(prep_fps=15, target_fps=30)
run(prep_fps=10, target_fps=20)

# Reset defaults
sentai.pipeline.prep_fps(0)
sentai.pipeline.target_fps(45)
print("=== done ===")
