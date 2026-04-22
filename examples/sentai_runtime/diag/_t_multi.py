# Multi-patch simulation: camera 30 FPS + N invokes per frame
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(label, prep_fps, ipf, dur=5000):
    print("--- %s: prep_fps=%d invokes_per_frame=%d ---" % (label, prep_fps, ipf))
    sentai.pipeline.prep_fps(prep_fps)
    sentai.pipeline.target_fps(0)  # unlimited InferTask
    sentai.pipeline.invokes_per_frame(ipf)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAIL %s" % rc); return
    sentai.rtos.sleep_ms(dur)
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(200)
    if pstats['frames']:
        print("  prep: %d frames (%.1f FPS)  cam_grab=%d pxp=%d quant=%d (ms/frame)" % (
            pstats['frames'], 1000*pstats['frames']/dur,
            pstats['cam_grab_ms_sum']//pstats['frames'],
            pstats['pxp_ms_sum']//pstats['frames'],
            pstats['quant_ms_sum']//pstats['frames']))
    if istats['ok']:
        invokes_per_cam_frame = istats['ok']/max(pstats['frames'],1)
        print("  infer: ok=%d fail=%d  fps=%.1f  avg_invoke=%d ms  inv/prep=%.2f" % (
            istats['ok'], istats['fail'], istats['ok']/(dur/1000.0),
            istats['ms_sum']//istats['ok'], invokes_per_cam_frame))
    else:
        print("  infer: 0 ok, %d fail" % istats['fail'])

# Baseline free-run
run("baseline free", 0, 1, 4000)
# Camera 30 FPS, 1 invoke = like pure throttle
run("cam30 ipf=1", 30, 1, 4000)
# Camera 30 FPS, 2 invokes per frame — the target scenario
run("cam30 ipf=2", 30, 2, 4000)
# Camera 20 FPS, 3 invokes per frame
run("cam20 ipf=3", 20, 3, 4000)
# Camera 15 FPS, 4 invokes per frame
run("cam15 ipf=4", 15, 4, 4000)

# Reset
sentai.pipeline.prep_fps(0); sentai.pipeline.target_fps(45)
sentai.pipeline.invokes_per_frame(1)
print("=== done ===")
