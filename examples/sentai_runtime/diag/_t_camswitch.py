# Camera switch performance: latency + first-post-switch cleanliness
# Runs 3 scenarios:
#   A) cold switch (no pipeline) — bare latency of sentai.camera.select()
#   B) switch between pipeline runs — stop/switch/start overhead
#   C) switch while pipeline is RUNNING — most interesting case
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(500)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def ms(): return sentai.rtos.ticks_ms()

print("--- A: 10 cold switches, no pipeline ---")
durs = []
for i in range(10):
    target = (i + 1) & 1   # alternate 1,0,1,0,...
    t0 = ms()
    rc = sentai.camera.select(target)
    t1 = ms()
    durs.append(t1 - t0)
    if rc != 0:
        print("  switch(%d) failed rc=%s" % (target, rc))
    sentai.rtos.sleep_ms(200)  # settle between
avg = sum(durs) / len(durs)
mx = max(durs); mn = min(durs)
print("  10 switches: min=%d avg=%d max=%d ms" % (mn, avg, mx))

# Leave camera at 0
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

print("--- B: switch between pipeline runs ---")
# 3 cycles: start -> run 2s -> stop -> switch -> start
for cycle in range(3):
    target = cycle & 1
    sentai.camera.select(target)
    sentai.rtos.sleep_ms(200)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    t0 = ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(2000)
    istats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    t1 = ms()
    print("  cycle %d cam=%d: pipe_dur=%d ms | infer ok=%d fail=%d fps=%.1f" % (
        cycle, target, t1 - t0,
        istats['ok'], istats['fail'], istats['ok'] / 2.0))

print("--- C: switch DURING running pipeline ---")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(1000)
print("  baseline before switch:")
pre = sentai.pipeline.infer_stats()
print("    infer ok=%d fail=%d" % (pre['ok'], pre['fail']))

# Do a few in-flight switches and measure impact
for cycle in range(5):
    target = (cycle + 1) & 1
    t0 = ms()
    rc = sentai.camera.select(target)
    t1 = ms()
    sentai.rtos.sleep_ms(500)  # observe post-switch effects
    cur = sentai.pipeline.infer_stats()
    print("  sw%d -> cam%d latency=%d ms | since last: ok+%d fail+%d" % (
        cycle, target, t1 - t0,
        cur['ok'] - pre['ok'], cur['fail'] - pre['fail']))
    pre = cur
sentai.pipeline.stop()
post = sentai.pipeline.infer_stats()
print("  stats: ok=%d fail=%d" % (post['ok'], post['fail']))

# cam switch diagnostics (fault counters)
try:
    stats = sentai.diag.cam_stats() if hasattr(sentai.diag, 'cam_stats') else None
    print("  cam_stats:", stats)
except Exception as e:
    print("  cam_stats err:", e)

print("=== done ===")
