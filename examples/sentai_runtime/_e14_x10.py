# Run E14 ten times and print a compact table.  verbose=0 during experiment.
import gc, diag, sentai

MODEL = "/yolo26n.edgetpu_1.tflite"
REPS = 8
N = 10

rows = []
for i in range(N):
    gc.collect()
    r = diag.e14_pipeline_parallel(MODEL, camera_id=0, width=320, height=320,
                                    conf=0.25, iou=0.45, repetitions=REPS,
                                    save=False)
    s  = r["summary"]; iv = s["interval"]
    rows.append((i + 1, iv["mean"], iv["min"], iv["max"], iv["p95"],
                 s["fps_wall"], s["fw_avg_fps"],
                 s["fw_processed"], s["fw_dropped"], s["timeouts"]))

print()
print("| run | mean_ms | min_ms | max_ms | p95_ms | fps_wall | fw_fps | fw_proc | fw_drop | tout |")
print("|-----|--------:|-------:|-------:|-------:|---------:|-------:|--------:|--------:|-----:|")
for r in rows:
    print("| %3d | %7.1f | %6d | %6d | %6.1f | %8.1f | %6.1f | %7d | %7d | %4d |" % r)
ivs = [r[1] for r in rows]
fps = [r[5] for r in rows]
print()
print("mean_of_mean=%.1f ms  fps_wall_mean=%.1f  min_of_means=%.1f  max_of_means=%.1f" % (
    sum(ivs)/N, sum(fps)/N, min(ivs), max(ivs)))
