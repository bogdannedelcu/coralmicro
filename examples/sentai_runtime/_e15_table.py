# Run E15 bench 10 times and print a summary table.
# Each run: 20 frames (after warmup). Uses pipeline.get_ex for per-stage data.
import sentai

def bench(n_frames=20):
    sentai.verbose(0)
    if sentai.pipeline.running():
        sentai.pipeline.stop()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)  # warmup
    wall = []; inv = []; tot = []; mcp = []; nms = []; seqs = []
    tp = sentai.rtos.ticks_ms()
    for i in range(n_frames):
        r = sentai.pipeline.get_ex(2000)
        tn = sentai.rtos.ticks_ms()
        if r is None:
            continue
        _d, inv_ms, tot_ms, fseq, mcp_ms, nms_ms = r
        wall.append(tn - tp); tp = tn
        inv.append(inv_ms); tot.append(tot_ms)
        mcp.append(mcp_ms); nms.append(nms_ms); seqs.append(fseq)
    sentai.pipeline.stop()
    sentai.verbose(1)
    def m(xs): return sum(xs)/len(xs) if xs else 0
    fps = 1000.0 * len(wall) / sum(wall) if wall else 0
    seq_delta = (seqs[-1] - seqs[0]) / max(1, len(seqs) - 1) if len(seqs) > 1 else 0
    return {
        "n": len(wall),
        "wall": m(wall),
        "inv":  m(inv),
        "mcp":  m(mcp),
        "nms":  m(nms),
        "tot":  m(tot),
        "fps":  fps,
        "seq_d": seq_delta,
    }

# Setup once
sentai.verbose(0)
if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.select(0)
sentai.camera.set_resolution(512, 512)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.to_tensor()
sentai.verbose(1)

print()
print("E15 x10 runs (20 frames each, yolo_1_class_512_1_upsample... @ 512x512)")
print("run | wall  inv   mcp   nms   tot  | FPS  | seq_d")
print("----+------+-----+-----+-----+-----+------+------")
results = []
for i in range(10):
    r = bench(20)
    results.append(r)
    print(" %2d | %4.1f %4.1f %4.1f %4.1f %4.1f | %4.1f | %4.2f" % (
        i+1, r["wall"], r["inv"], r["mcp"], r["nms"], r["tot"], r["fps"], r["seq_d"]))

print("----+------+-----+-----+-----+-----+------+------")
def mm(k):
    vs = [r[k] for r in results]
    return sum(vs)/len(vs), min(vs), max(vs)
w_m, w_mi, w_ma = mm("wall")
i_m, _, _       = mm("inv")
m_m, _, _       = mm("mcp")
n_m, _, _       = mm("nms")
t_m, _, _       = mm("tot")
f_m, f_mi, f_ma = mm("fps")
print("mean| %4.1f %4.1f %4.1f %4.1f %4.1f | %4.1f |      ms per stage" % (
    w_m, i_m, m_m, n_m, t_m, f_m))
print("FPS stats: mean=%.2f  min=%.2f  max=%.2f" % (f_m, f_mi, f_ma))
