# Full per-stage benchmark using pipeline.get_ex(): exposes invoke_ms, total_ms,
# frame_seq AND the new memcpy_ms + nms_ms sub-stage fields.
import sentai

sentai.verbose(0)
if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.select(0)
sentai.camera.set_resolution(512, 512)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.to_tensor()

sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)  # warmup

N = 20
wall = []; inv = []; tot = []; mcp = []; nms = []; seqs = []
t_prev = sentai.rtos.ticks_ms()
for i in range(N):
    r = sentai.pipeline.get_ex(2000)
    t_now = sentai.rtos.ticks_ms()
    if r is None:
        continue
    _d, inv_ms, tot_ms, fseq, mcp_ms, nms_ms = r
    wall.append(t_now - t_prev); t_prev = t_now
    inv.append(inv_ms); tot.append(tot_ms)
    mcp.append(mcp_ms); nms.append(nms_ms); seqs.append(fseq)

sentai.pipeline.stop()
sentai.verbose(1)

def st(xs):
    if not xs: return "--"
    return "mean=%.1f min=%d max=%d" % (sum(xs)/len(xs), min(xs), max(xs))

print()
print("E15 PER-STAGE BENCH (%d frames):" % len(wall))
print("  wall_interval:  %s  ms" % st(wall))
print("  invoke (TPU):   %s  ms" % st(inv))
print("  memcpy stg->t:  %s  ms" % st(mcp))
print("  NMS (tpu_det):  %s  ms" % st(nms))
print("  infer_total:    %s  ms" % st(tot))
if wall and tot:
    gap = sum(wall)/len(wall) - sum(tot)/len(tot)
    print("  gap wall-total: %.1f ms" % gap)
if tot and inv and mcp and nms:
    other = sum(tot)/len(tot) - sum(inv)/len(inv) - sum(mcp)/len(mcp) - sum(nms)/len(nms)
    print("  unaccounted in total (bookkeeping+queue): %.1f ms" % other)
if seqs and len(seqs) > 1:
    sds = [seqs[i] - seqs[i-1] for i in range(1, len(seqs))]
    print("  frame_seq delta:  mean=%.1f min=%d max=%d (cam frames per pipeline output)" % (
        sum(sds)/len(sds), min(sds), max(sds)))
print("  FPS wall:       %.1f" % (1000.0 * len(wall) / sum(wall) if wall else 0))
p, dr, f = sentai.pipeline.stats()
print("  fw: processed=%d dropped=%d avg_fps=%.1f" % (p, dr, f))
