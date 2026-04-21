# _e22_alt_parallel.py — alternating cam0/cam1 in the PARALLEL pipeline
# (direct_tensor ping-pong + ratio(1,1) ISR-driven auto-alternate).
import sentai, gc

def bench(label, reps=40):
    sentai.pipeline.prep_reset()
    if sentai.pipeline.running():
        sentai.pipeline.stop()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)  # warmup
    wall = []; inv = []; seqs = []
    t_prev = sentai.rtos.ticks_ms()
    for _ in range(reps):
        r = sentai.pipeline.get_ex(2000)
        t_now = sentai.rtos.ticks_ms()
        if r is None:
            continue
        _d, inv_ms, _tot, fseq, _mcp, _nms = r
        wall.append(t_now - t_prev); t_prev = t_now
        inv.append(inv_ms); seqs.append(fseq)
    sentai.pipeline.stop()
    ps = sentai.pipeline.prep_stats()
    f = ps["frames"] or 1
    mean_wall = sum(wall) / len(wall) if wall else 0
    fps = 1000.0 / mean_wall if mean_wall else 0
    delta = [seqs[i] - seqs[i-1] for i in range(1, len(seqs))]
    print("[E22:%s] fps=%.2f wall=%.1fms invoke=%.1fms prep=%.1fms "
          "fseq_delta=%.1f (min=%d max=%d)" % (
          label, fps, mean_wall, sum(inv)/len(inv),
          ps["total_ms_sum"]/f,
          sum(delta)/len(delta) if delta else 0,
          min(delta) if delta else 0, max(delta) if delta else 0))
    return fps

if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)
gc.collect()

# A: single cam0 (baseline)
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
gc.collect()
fps_a = bench("A fixed cam0")

# B: ISR ratio alternating 1:1
sentai.camera.ratio(1, 1)
gc.collect()
fps_b = bench("B ratio(1,1) alt 1:1")

# C: ratio 1:1 + drain=2 (conservative)
sentai.camera.switch_drain(2)
gc.collect()
fps_c = bench("C ratio(1,1) drain=2")

# Restore defaults
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(2)
print()
print("=== E22 SUMMARY parallel pipeline VGA@45 ===")
print("  A fixed cam0:               %.2f fps" % fps_a)
print("  B alt ratio(1,1) drain=1:   %.2f fps" % fps_b)
print("  C alt ratio(1,1) drain=2:   %.2f fps" % fps_c)
