#!/usr/bin/env python3
"""s237 - per-export-route EdgeTPU inference benchmark on the Coral USB stick.

Benchmarks the 3 edgetpu-compiled variants of the PyTorch-route c2f_thick
champion (MLflow run d62bb783, experiment sentai_revision_cover_v1) on the
physical Coral USB Accelerator (full libedgetpu, host-side). Measures pure TPU
invoke latency (the model is resident; we feed a fixed input and time invoke()
only), separated from the one-time parameter/instruction upload on first run.

This is the host-side reference number; the on-board (i.MX RT1176) figure is
USB2-bound and lower (see s236). Pass criterion: report mean/median/p95/min
latency + FPS per model, and the host-decode-free output shape.

Run:  venv-coral39/bin/python3 bench_usbcoral.py
"""
import glob, os, time, statistics, csv, sys
import numpy as np
from pycoral.utils.edgetpu import make_interpreter, list_edge_tpus

WARMUP = 20
ITERS = 200
HERE = os.path.dirname(os.path.abspath(__file__))

LABELS = {
    "A_main_int8_edgetpu.tflite":   "PyTorch export.py route (decode on-TPU)",
    "B_silu_int8_edgetpu.tflite":   "SiLU-activation experiment",
    "C_onnx2tf_int8_edgetpu.tflite": "onnx2tf export route",
}


def bench(path):
    it = make_interpreter(path)          # loads libedgetpu delegate on the USB TPU
    it.allocate_tensors()
    ind = it.get_input_details()[0]
    outs = it.get_output_details()
    ishape = list(ind["shape"])
    # deterministic mid-range input (uint8); content does not affect TPU timing
    x = np.full(ishape, 127, dtype=ind["dtype"])
    it.set_tensor(ind["index"], x)

    for _ in range(WARMUP):
        it.invoke()
    samples = []
    for _ in range(ITERS):
        t0 = time.perf_counter()
        it.invoke()
        samples.append((time.perf_counter() - t0) * 1e3)  # ms

    samples.sort()
    mean = statistics.mean(samples)
    return {
        "in_shape": ishape,
        "out_shape": [list(o["shape"]) for o in outs],
        "size_b": os.path.getsize(path),
        "mean_ms": mean,
        "median_ms": statistics.median(samples),
        "p95_ms": samples[int(0.95 * len(samples)) - 1],
        "min_ms": samples[0],
        "fps_mean": 1000.0 / mean,
    }


def main():
    tpus = list_edge_tpus()
    print("Edge TPUs:", tpus)
    if not tpus:
        print("ERROR: no Edge TPU found. Is the Coral USB stick plugged in?")
        sys.exit(1)

    rows = []
    for f in sorted(glob.glob(os.path.join(HERE, "models", "*_edgetpu.tflite"))):
        name = os.path.basename(f)
        print("\n=== %s — %s ===" % (name, LABELS.get(name, "")))
        try:
            r = bench(f)
        except Exception as e:
            print("  FAILED:", e)
            continue
        print("  in=%s out=%s size=%dB" % (r["in_shape"], r["out_shape"], r["size_b"]))
        print("  mean=%.3f ms  median=%.3f  p95=%.3f  min=%.3f  -> %.1f FPS"
              % (r["mean_ms"], r["median_ms"], r["p95_ms"], r["min_ms"], r["fps_mean"]))
        r["model"] = name
        r["label"] = LABELS.get(name, "")
        rows.append(r)

    out_csv = os.path.join(HERE, "results.csv")
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["model", "label", "size_b", "in_shape", "out_shape",
                    "mean_ms", "median_ms", "p95_ms", "min_ms", "fps_mean"])
        for r in rows:
            w.writerow([r["model"], r["label"], r["size_b"], r["in_shape"],
                        r["out_shape"], "%.3f" % r["mean_ms"], "%.3f" % r["median_ms"],
                        "%.3f" % r["p95_ms"], "%.3f" % r["min_ms"], "%.2f" % r["fps_mean"]])
    print("\nwrote", out_csv)


if __name__ == "__main__":
    main()
