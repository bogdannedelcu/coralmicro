import sentai
sentai.verbose(0)

# Don't init camera.  Don't start pipeline.  Load model only.
rc = sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')

results = []
def log(m): results.append(str(m))

log("tpu_load=%s" % rc)
log("cam_fc=%d (camera NOT initialized)" % sentai.camera.frame_count())
log("pipeline_running=%s" % sentai.pipeline.running())

# Warmup
for i in range(5): sentai.tpu.invoke()
log("warmup x5 done")

# Tight invoke loop from REPL (MicroPython main task)
N = 100
t0 = sentai.rtos.ticks_ms()
fails = 0
for i in range(N):
    r = sentai.tpu.invoke()
    if r < 0: fails += 1
t1 = sentai.rtos.ticks_ms()
dt = t1 - t0
log("pure_invoke: N=%d dt=%dms avg=%.2fms/inv fps=%.1f fails=%d" % (
    N, dt, dt/N, 1000*N/dt, fails))

# Repeat 3x for stability
for rep in range(3):
    t0 = sentai.rtos.ticks_ms()
    fails = 0
    for i in range(N):
        r = sentai.tpu.invoke()
        if r < 0: fails += 1
    t1 = sentai.rtos.ticks_ms()
    dt = t1 - t0
    log("run%d: fps=%.1f avg=%.2fms fails=%d" % (
        rep, 1000*N/dt, dt/N, fails))

sentai.fs.write('/invoke_only.txt', "\n".join(results) + "\n")
print("=== INVOKE-ONLY RESULTS ===")
for r in results: print(r)
print("=== END ===")
