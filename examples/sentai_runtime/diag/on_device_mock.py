import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

# Setup
rc = sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
log("tpu_load=%s" % rc)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
for i in range(3): sentai.tpu.invoke()
log("warmup done")

try:
    ALL_ASYNC = hasattr(sentai.diag, 'async_stats')
except:
    ALL_ASYNC = False

def bench(tag, async_ring, prep_mode, no_invoke, dur=3000):
    try:
        sentai.pipeline.async_ring(async_ring)
        sentai.pipeline.debug_prep_mode(prep_mode)
        sentai.pipeline.debug_no_invoke(no_invoke)
        sentai.pipeline.ar_mode(0)
        sentai.pipeline.ar_infer_wait(5)
        sentai.pipeline.prep_reset()
        sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        s = sentai.pipeline.infer_stats()
        ar = sentai.pipeline.ar_stats() if async_ring else (0,0,0,0,0,0,0)
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(300)
        # reset toggles
        sentai.pipeline.debug_prep_mode(0)
        sentai.pipeline.debug_no_invoke(0)
        fps = s['ok']/(dur/1000)
        extra = ""
        if ALL_ASYNC:
            try:
                a = sentai.diag.async_stats()
                extra = " async=%d/%d/%d/%d" % (
                    a.get('bo_ok',0), a.get('bo_send',0),
                    a.get('take_timeout',0), a.get('urb_cancelled',0))
            except: pass
        log("%s ok=%d fail=%d fps=%.1f last_rc=%d prep=%d infer=%d ovw=%d%s" % (
            tag, s['ok'], s['fail'], fps, s.get('last_rc',-999),
            ar[0], ar[1], ar[5], extra))
    except Exception as e:
        log("%s EXC=%s" % (tag, e))
        try:
            sentai.pipeline.stop()
            sentai.pipeline.debug_prep_mode(0)
            sentai.pipeline.debug_no_invoke(0)
        except: pass

# === ISOLATION MATRIX ===
# T0 baseline legacy full
bench("T0_LEGACY_full",          async_ring=0, prep_mode=0, no_invoke=0)
# T1 legacy MOCK prep + invoke → isolates if legacy infra OK
bench("T1_LEGACY_mock_inv",      async_ring=0, prep_mode=1, no_invoke=0)
# T2 legacy full + NO invoke → isolates if prep path itself stable
bench("T2_LEGACY_full_noinv",    async_ring=0, prep_mode=0, no_invoke=1)

# T3 V23 baseline (known fail)
bench("T3_V23_full",             async_ring=1, prep_mode=0, no_invoke=0)
# T4 V23 MOCK prep + invoke → no PXP/cam SDRAM traffic, just invoke on pre-ring
bench("T4_V23_mock_inv",         async_ring=1, prep_mode=1, no_invoke=0)
# T5 V23 full + NO invoke → prep loads ring, no TPU traffic
bench("T5_V23_full_noinv",       async_ring=1, prep_mode=0, no_invoke=1)
# T6 V23 MOCK + NO invoke → baseline "nothing happening"
bench("T6_V23_mock_noinv",       async_ring=1, prep_mode=1, no_invoke=1)

# T7 V23 CAM only (grab but skip PXP, zero tensor dst)
bench("T7_V23_camonly_inv",      async_ring=1, prep_mode=2, no_invoke=0)

# Save + print
out = "\n".join(results) + "\n"
sentai.fs.write('/mock_results.txt', out)
print("=== MOCK RESULTS ===")
for r in results: print(r)
print("=== END ===")
