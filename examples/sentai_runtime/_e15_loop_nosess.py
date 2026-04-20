import gc, diag, sentai
sentai.verbose(0)
N = 20
results = []
for i in range(N):
    gc.collect()
    r = diag.e15_pipeline_parallel_512(repetitions=20, save=False)
    results.append(r["summary"]["fps_wall"] if r else None)
sentai.verbose(1)
for i, f in enumerate(results):
    print("run %2d fps=%s" % (i + 1, f))
