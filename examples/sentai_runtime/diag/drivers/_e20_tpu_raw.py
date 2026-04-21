# _e20_tpu_raw.py — isolated TPU benchmark.
#
# Bypasses the camera + pipeline paths entirely.  Loads the current YOLO
# model, builds a fixed input tensor via sentai.tpu.load_image (or
# falls back to letting the last pipeline invoke set it), and calls
# sentai.tpu.invoke() N times back-to-back.  Reports per-call ms.
#
# Goal: separate the TPU+USB cost from camera/PXP/pipeline variance so
# any firmware-driver change can be A/B'd with tight confidence.

import sentai, gc

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_WARMUP = 3
N_SAMPLES = 30

if sentai.pipeline.running():
    sentai.pipeline.stop()

# Ensure model loaded; load_image call fills the input tensor with whatever
# sits in /diags/<some jpeg>.  We don't care what the pixels are — the TPU
# path timing is independent of input content.
if not sentai.tpu.ready():
    rc = sentai.tpu.load(MODEL)
    print("tpu.load rc=", rc)

# Fill the input tensor with camera frame so TPU has valid data.  Keeping
# the same frame across all invokes so we measure ONLY the invoke path
# (no camera grab, no PXP, no memcpy).
sentai.camera.set_resolution(512, 512)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.select(0)
sentai.camera.to_tensor()   # fills input tensor once

gc.collect()

# Warmup so cache state + USB pipe is primed
for _ in range(N_WARMUP):
    sentai.tpu.invoke()

dts = []
t0 = sentai.rtos.ticks_ms()
for _ in range(N_SAMPLES):
    t = sentai.rtos.ticks_ms()
    rc = sentai.tpu.invoke()
    dt = sentai.rtos.ticks_ms() - t
    dts.append((dt, rc))
wall_total = sentai.rtos.ticks_ms() - t0

rcs = [r for (_d, r) in dts]
times = [d for (d, _r) in dts]
ok = [d for (d, r) in dts if r >= 0]
print("E20 TPU RAW BENCH (%d invokes, no cam/PXP):" % N_SAMPLES)
if ok:
    print("  invoke ms: min=%d mean=%.1f max=%d" % (
        min(ok), sum(ok)/len(ok), max(ok)))
    print("  effective FPS (pure invoke): %.1f" % (1000.0 * N_SAMPLES / wall_total))
print("  invoke rcs: min=%d max=%d fails=%d" % (
    min(rcs), max(rcs), sum(1 for r in rcs if r < 0)))
print("  wall total: %d ms" % wall_total)
