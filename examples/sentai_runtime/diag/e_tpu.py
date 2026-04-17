# diag/e_tpu.py — E1: TPU invoke latency, E2: TPU model load

import sentai
from diag._util import (stats, time_call, save_csv, snapshot_meta,
                         snapshot_heap, _print_stats, _print_heap)
from diag._session import _save_path, _record, _save_desc


def e1_tpu_invoke(model_path, image_path=None, use_camera=False,
                  repetitions=100, save=True):
    """Measure sentai.tpu.invoke() latency.
    Model must be loaded first, or provide model_path to auto-load.
    If image_path given, loads that image. If use_camera, calls to_tensor().
    """
    print("[E1] TPU invoke latency — %d reps" % repetitions)

    sentai.tpu.load(model_path)
    if image_path:
        sentai.tpu.load_image(image_path)
    elif use_camera:
        sentai.camera.to_tensor()

    # Warm-up
    sentai.tpu.invoke()

    samples = []
    for i in range(repetitions):
        if use_camera:
            sentai.camera.to_tensor()
        ms = sentai.tpu.invoke()
        samples.append(ms)
        if (i + 1) % 25 == 0:
            print("  ... %d/%d" % (i + 1, repetitions))

    st = stats(samples)
    meta = snapshot_meta("E1_tpu_invoke", model_path=model_path,
                         image_path=image_path or "", use_camera=use_camera)

    print("[E1] Done.")
    _print_stats("invoke", st)

    result = {"experiment": "E1_tpu_invoke", "meta": meta,
              "params": {"model_path": model_path, "repetitions": repetitions,
                         "image_path": image_path, "use_camera": use_camera},
              "samples": samples, "summary": st}

    if save:
        path = _save_path("e1_tpu_invoke")
        save_csv(path, ["run_index", "edgetpu_invoke_ms"], [(i, s) for i, s in enumerate(samples)])
        _save_desc(path,
            "E1 — EdgeTPU invoke latency.\n"
            "Measures how long sentai.tpu.invoke() takes to run one forward pass on the EdgeTPU.\n"
            "\nColumns:\n"
            "  run_index          : repetition number (0-based)\n"
            "  edgetpu_invoke_ms  : wall time of tpu.invoke() in milliseconds\n"
            "\nNote: first run is warm-up and excluded. Subsequent runs use the same loaded model.",
            params={"model_path": model_path, "repetitions": repetitions,
                    "use_camera": use_camera, "image_path": image_path})
        _record("e1_tpu_invoke", path, "mean=%.1f p95=%.1f ms" % (st["mean"], st["p95"]))
        print("  Saved: %s" % path)

    return result


def e2_tpu_load(model_path, repetitions=10, save=True):
    """Measure sentai.tpu.load() time and memory impact."""
    print("[E2] TPU model load — %d reps" % repetitions)

    heap_before = snapshot_heap()
    samples = []
    for i in range(repetitions):
        elapsed, rc = time_call(sentai.tpu.load, model_path)
        samples.append(elapsed)
        print("  run %d: %d ms (rc=%s)" % (i, elapsed, rc))
    heap_after = snapshot_heap()

    st = stats(samples)
    meta = snapshot_meta("E2_tpu_load", model_path=model_path)

    print("[E2] Done.")
    _print_stats("load", st)
    _print_heap("before", heap_before)
    _print_heap("after", heap_after)

    result = {"experiment": "E2_tpu_load", "meta": meta,
              "params": {"model_path": model_path, "repetitions": repetitions},
              "samples": samples, "summary": st,
              "extra": {"heap_before": heap_before, "heap_after": heap_after}}

    if save:
        path = _save_path("e2_tpu_load")
        save_csv(path, ["run_index", "model_load_ms"], [(i, s) for i, s in enumerate(samples)])
        _save_desc(path,
            "E2 — EdgeTPU model load latency.\n"
            "Measures how long sentai.tpu.load() takes to load a .tflite model into EdgeTPU SRAM.\n"
            "\nColumns:\n"
            "  run_index      : repetition number (0-based)\n"
            "  model_load_ms  : wall time of tpu.load() in milliseconds\n"
            "\nNote: heap snapshots before/after are in the result dict but not in the CSV.",
            params={"model_path": model_path, "repetitions": repetitions})
        _record("e2_tpu_load", path, "mean=%.1f ms" % st["mean"])
        print("  Saved: %s" % path)

    return result
