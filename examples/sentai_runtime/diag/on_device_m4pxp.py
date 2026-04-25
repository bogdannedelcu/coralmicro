import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
if sentai.camera.frame_count() == 0: sentai.camera.init(1)
sentai.rtos.sleep_ms(500)

# Start M4 flow core (it handles both flow + PXP channel now)
try:
    rc = sentai.flow.m4_start(0)
    log("m4_start=%s" % rc)
except Exception as e:
    log("m4_start_exc=%s" % e)

sentai.rtos.sleep_ms(800)
for i in range(3): sentai.tpu.invoke()
log("warmup done")

def bench(tag, pxp_on_m4, async_ring, dur=3000):
    try:
        sentai.pipeline.pxp_on_m4(pxp_on_m4)
        sentai.pipeline.async_ring(async_ring)
        sentai.pipeline.debug_prep_mode(0)
        sentai.pipeline.debug_no_invoke(0)
        sentai.pipeline.prep_reset()
        sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        s = sentai.pipeline.infer_stats()
        p = sentai.pipeline.prep_stats()
        sentai.pipeline.stop()
        sentai.pipeline.pxp_on_m4(0)
        sentai.pipeline.async_ring(0)
        sentai.rtos.sleep_ms(300)
        fps = s['ok']/(dur/1000)
        log("%s ok=%d fail=%d fps=%.1f prep_frames=%d pxp_ms=%d" % (
            tag, s['ok'], s['fail'], fps,
            p.get('frames',0), p.get('pxp_ms_sum',0)))
    except Exception as e:
        log("%s EXC=%s" % (tag, e))
        try:
            sentai.pipeline.stop()
            sentai.pipeline.pxp_on_m4(0)
            sentai.pipeline.async_ring(0)
        except: pass

bench("LEGACY_M7PXP",     pxp_on_m4=0, async_ring=0)
bench("LEGACY_M4PXP",     pxp_on_m4=1, async_ring=0)
bench("V23_M7PXP",        pxp_on_m4=0, async_ring=1)
bench("V23_M4PXP",        pxp_on_m4=1, async_ring=1)

out = "\n".join(results) + "\n"
sentai.fs.write('/m4_results.txt', out)
print("=== M4 PXP RESULTS ===")
for r in results: print(r)
print("=== END ===")
