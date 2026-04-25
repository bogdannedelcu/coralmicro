# Pure TPU phase timing breakdown via DWT cycle counters
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)

sentai.diag.tpu_perf(True)  # reset counters
N = 30
for i in range(N):
    sentai.tpu.invoke()

p = sentai.diag.tpu_perf()
MHZ = 996  # M7 overclocked to 996 MHz this build

def us(cyc): return cyc // N // MHZ

print("=== Pure TPU, %d invokes (no pipeline), M7 @ %d MHz ===" % (N, MHZ))
print("  cyc_params  %6d us/inv" % us(p['cyc_params']))
print("  cyc_input   %6d us/inv" % us(p['cyc_input']))
print("  cyc_ins     %6d us/inv" % us(p['cyc_ins']))
print("  cyc_output  %6d us/inv" % us(p['cyc_output']))
print("  cyc_event   %6d us/inv" % us(p['cyc_event']))
total = p['cyc_params']+p['cyc_input']+p['cyc_ins']+p['cyc_output']+p['cyc_event']
print("  TOTAL       %6d us/inv (%.1f FPS ceiling)" % (us(total), 1e6/us(total) if us(total) else 0))
print("=== done ===")
