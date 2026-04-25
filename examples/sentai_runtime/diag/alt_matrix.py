import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
for _ in range(3): sentai.tpu.invoke()

# Camera alt 1:1
sentai.camera.ratio(1, 1)

def bench(tag, prep_fps, inv_per_frame, dur=3000):
    try:
        sentai.pipeline.prep_fps(prep_fps)
        sentai.pipeline.invokes_per_frame(inv_per_frame)
        sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        s = sentai.pipeline.infer_stats()
        p = sentai.pipeline.prep_stats()
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(400)
        log("%s prep=%d ipf=%d  prepFPS=%.1f inferFPS=%.1f fail=%d invoke_ms=%d" % (
            tag, prep_fps, inv_per_frame,
            p.get('frames',0)/(dur/1000),
            s['ok']/(dur/1000),
            s['fail'],
            s['ms_sum']//max(s['ok'],1)))
    except Exception as e:
        log("%s EXC=%s" % (tag, e))
        try: sentai.pipeline.stop()
        except: pass

log("=== ALT 1:1 ===")
# Reduced matrix — 9 cells total, ~30s
for pf in [0, 30, 15]:
    for ipf in [1, 2, 4]:
        bench("alt11", pf, ipf)

# Reset
sentai.pipeline.prep_fps(0)
sentai.pipeline.invokes_per_frame(1)
sentai.camera.ratio(0, 0)

sentai.fs.write('/alt_matrix.txt', "\n".join(results) + "\n")
print("=== RESULTS ===")
for r in results: print(r)
print("=== END ===")
