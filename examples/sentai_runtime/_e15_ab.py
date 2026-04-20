# A/B benchmark: same firmware, toggle eDMA memcpy on/off via runtime flag.
# Runs 10 × (20 frames) per configuration and prints a side-by-side table.
import sentai

def bench(n_frames=20):
    sentai.verbose(0)
    if sentai.pipeline.running():
        sentai.pipeline.stop()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)  # warmup
    wall = []; inv = []; mcp = []; nms = []; tot = []; seqs = []
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
    return {"n": len(wall), "wall": m(wall), "inv": m(inv), "mcp": m(mcp),
            "nms": m(nms), "tot": m(tot), "fps": fps}

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

def run_block(label, dma_flag):
    sentai.pipeline.dma_memcpy(dma_flag)
    print()
    print("%s (dma_memcpy=%d)" % (label, dma_flag))
    print("run | wall  inv   mcp   nms   tot  | FPS")
    print("----+------+-----+-----+-----+-----+------")
    rs = []
    for i in range(10):
        r = bench(20); rs.append(r)
        print(" %2d | %4.1f %4.1f %4.1f %4.1f %4.1f | %4.1f" % (
            i+1, r["wall"], r["inv"], r["mcp"], r["nms"], r["tot"], r["fps"]))
    def mm(k):
        vs=[r[k] for r in rs]; return sum(vs)/len(vs), min(vs), max(vs)
    w_m,_,_ = mm("wall"); i_m,_,_=mm("inv"); m_m,_,_=mm("mcp")
    n_m,_,_ = mm("nms");  t_m,_,_=mm("tot"); f_m,f_mi,f_ma=mm("fps")
    print("----+------+-----+-----+-----+-----+------")
    print("mean| %4.1f %4.1f %4.1f %4.1f %4.1f | %4.2f  (min=%.2f max=%.2f)" % (
        w_m, i_m, m_m, n_m, t_m, f_m, f_mi, f_ma))
    return {"label": label, "wall": w_m, "inv": i_m, "mcp": m_m, "nms": n_m,
            "tot": t_m, "fps": f_m, "fps_min": f_mi, "fps_max": f_ma}

A = run_block("BASELINE (CPU memcpy)", 0)
B = run_block("OPTIMISED (eDMA 32-B bursts)", 1)

print()
print("================ A/B SUMMARY ================")
print("%-28s %7s %7s %7s %7s %7s %7s" % (
    "config", "wall", "invoke", "memcpy", "nms", "total", "FPS"))
for X in (A, B):
    print("%-28s %7.1f %7.1f %7.1f %7.1f %7.1f %7.2f" % (
        X["label"], X["wall"], X["inv"], X["mcp"], X["nms"], X["tot"], X["fps"]))
dmcp = A["mcp"] - B["mcp"]
dfps = B["fps"] - A["fps"]
print()
print("memcpy saving: %.1f ms/frame  (%.0f%% faster)" % (
    dmcp, 100.0*dmcp/A["mcp"] if A["mcp"] else 0))
print("FPS improvement: +%.2f  (%.1f%% relative)" % (
    dfps, 100.0*dfps/A["fps"] if A["fps"] else 0))
