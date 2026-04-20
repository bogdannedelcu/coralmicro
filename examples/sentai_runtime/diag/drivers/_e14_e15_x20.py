# Drive E14 (COCO yolo26n @ 320x320) and E15 (1-class @ 512x512) 20 times each,
# each experiment wrapped in its own diag session.  verbose=0 the whole time
# so CDC-ACM TX doesn't saturate under the print volume of ~40 consecutive
# pipeline runs.  Results land as CSVs under /diags/<session>/.
import gc, diag, sentai

N = 20
REPS_PER_RUN = 20
MODEL_E14 = "/yolo26n.edgetpu_1.tflite"
MODEL_E15 = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.verbose(0)                          # silence ALL firmware prints

def short(label, i, r):
    if r is None:
        return "%s %2d STALL" % (label, i + 1)
    s = r["summary"]
    iv = s["interval"]
    return "%s %2d iv=%.1f fps=%.1f fw=%.1f dets=%d" % (
        label, i + 1, iv["mean"], s["fps_wall"],
        s["fw_avg_fps"], s["dets_total"])

# ---- E14 block --------------------------------------------------------
e14_tag = diag.begin("e14_x20")                       # returns session tag
e14_dir = "/diags/" + e14_tag
e14_results = []
for i in range(N):
    gc.collect()
    r = diag.e14_pipeline_parallel(
        MODEL_E14, camera_id=0, width=320, height=320,
        conf=0.25, iou=0.45, max_det=50,
        repetitions=REPS_PER_RUN, save=True,
    )
    e14_results.append(r)
diag.end()

# ---- E15 block --------------------------------------------------------
e15_tag = diag.begin("e15_x20")
e15_dir = "/diags/" + e15_tag
e15_results = []
for i in range(N):
    gc.collect()
    r = diag.e15_pipeline_parallel_512(
        MODEL_E15, camera_id=0, repetitions=REPS_PER_RUN, save=True)
    e15_results.append(r)
diag.end()

sentai.verbose(1)                          # restore
print()
print("E14 @ 320x320 results:")
for i, r in enumerate(e14_results):
    print(" ", short("E14", i, r))
print()
print("E15 @ 512x512 results:")
for i, r in enumerate(e15_results):
    print(" ", short("E15", i, r))
print()
print("saved: E14 ->", e14_dir)
print("saved: E15 ->", e15_dir)
