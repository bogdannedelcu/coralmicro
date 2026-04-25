# TPU phase timing DURING pipeline run — reveals which phase suffers
# from SEMC contention with CSI/PXP.
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()

MHZ = 996
print("=== Baseline: Pure TPU (N=30, no pipeline) ===")
sentai.diag.tpu_perf(True)
for i in range(30): sentai.tpu.invoke()
p = sentai.diag.tpu_perf()
def us(cyc, n): return cyc // n // MHZ
N = 30
print("  params=%4d us  input=%4d us  ins=%4d us  output=%4d us  event=%4d us  TOTAL=%5d us" % (
    us(p['cyc_params'],N), us(p['cyc_input'],N), us(p['cyc_ins'],N),
    us(p['cyc_output'],N), us(p['cyc_event'],N),
    (p['cyc_params']+p['cyc_input']+p['cyc_ins']+p['cyc_output']+p['cyc_event'])//N//MHZ))

print("\n=== Pipeline active: SAME per-phase timing ===")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(500)  # let pipeline stabilize
sentai.diag.tpu_perf(True)
sentai.rtos.sleep_ms(5000)  # 5s of pipeline
istats = sentai.pipeline.infer_stats()
p = sentai.diag.tpu_perf()
sentai.pipeline.stop()

n_invoke = istats['ok']
print("  pipeline run: %d invokes in 5s = %.1f FPS" % (n_invoke, n_invoke / 5.0))
if n_invoke > 0:
    print("  params=%4d us  input=%4d us  ins=%4d us  output=%4d us  event=%4d us  TOTAL=%5d us" % (
        us(p['cyc_params'], n_invoke), us(p['cyc_input'], n_invoke), us(p['cyc_ins'], n_invoke),
        us(p['cyc_output'], n_invoke), us(p['cyc_event'], n_invoke),
        (p['cyc_params']+p['cyc_input']+p['cyc_ins']+p['cyc_output']+p['cyc_event'])//n_invoke//MHZ))

print("\n=== done ===")
