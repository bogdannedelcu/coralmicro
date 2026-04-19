# diag/e_system.py — E10: memory snapshot, E11: CPU/task state, E12: live loop

import sentai
from diag._util import (stats, save_csv, snapshot_meta, snapshot_heap,
                         snapshot_cpu, snapshot_tasks, _ticks, _print_stats)
from diag._session import _save_path, _record, _save_desc


def e10_memory(scenario="idle", save=True):
    """Collect heap snapshot under a labeled scenario."""
    print("[E10] Memory snapshot — '%s'" % scenario)

    heap  = snapshot_heap()
    tasks = snapshot_tasks()
    meta  = snapshot_meta("E10_memory", scenario=scenario)

    from diag._util import _print_heap
    _print_heap(scenario, heap)
    print("  tasks: %d" % len(tasks))

    result = {"experiment": "E10_memory", "meta": meta,
              "params": {"scenario": scenario},
              "samples": [heap],
              "extra": {"tasks": tasks}}

    if save:
        path = _save_path("e10_mem_%s" % scenario)
        save_csv(path,
                 ["scenario_name", "rtos_free_heap_bytes", "gc_total_bytes", "gc_used_bytes",
                  "gc_free_bytes", "gc_largest_free_block_bytes"],
                 [[scenario, heap["rtos_free"], heap["gc_total"],
                   heap["gc_used"], heap["gc_free"], heap["gc_max_free"]]])
        _save_desc(path,
            "E10 — Memory snapshot.\n"
            "One-row snapshot of RAM state under a labeled scenario.\n"
            "\nColumns:\n"
            "  scenario_name                 : user label for when the snapshot was taken\n"
            "  rtos_free_heap_bytes          : FreeRTOS heap free (available for C malloc/tasks)\n"
            "  gc_total_bytes                : total MicroPython GC heap size\n"
            "  gc_used_bytes                 : GC heap currently allocated by Python objects\n"
            "  gc_free_bytes                 : GC heap free (available for new Python allocations)\n"
            "  gc_largest_free_block_bytes   : largest contiguous free block in GC heap",
            params={"scenario": scenario})
        _record("e10_memory", path,
                "%s rtos_free=%d" % (scenario, heap["rtos_free"]))
        print("  Saved: %s" % path)

    return result


def e11_cpu(scenario="idle", duration_ms=2000, save=True):
    """Collect CPU usage and task info over a measurement window."""
    print("[E11] CPU/task state — '%s' for %dms" % (scenario, duration_ms))

    cpu_before = snapshot_cpu()
    tasks_snap = snapshot_tasks()

    sentai.rtos.sleep_ms(duration_ms)

    cpu_after = snapshot_cpu()
    meta = snapshot_meta("E11_cpu", scenario=scenario, duration_ms=duration_ms)

    print("[E11] Done. Tasks:")
    for name, state, prio, hwm in tasks_snap:
        print("  %-16s state=%-8s prio=%d hwm=%d" % (name, state, prio, hwm))
    print("  CPU usage (after):")
    for name, pct in cpu_after:
        if pct > 0:
            print("    %-16s %d%%" % (name, pct))

    result = {"experiment": "E11_cpu", "meta": meta,
              "params": {"scenario": scenario, "duration_ms": duration_ms},
              "samples": {"cpu_before": cpu_before, "cpu_after": cpu_after},
              "extra": {"tasks": tasks_snap}}

    if save:
        path = _save_path("e11_cpu_%s" % scenario)
        cpu_map = {n: p for n, p in cpu_after}
        save_csv(path, ["task_name", "rtos_state", "rtos_priority", "stack_hwm_bytes", "cpu_usage_pct"],
                 [[name, state, prio, hwm, cpu_map.get(name, 0)]
                  for name, state, prio, hwm in tasks_snap])
        _save_desc(path,
            "E11 — CPU usage and RTOS task state.\n"
            "One row per FreeRTOS task, snapshot taken over a measurement window.\n"
            "\nColumns:\n"
            "  task_name        : FreeRTOS task name\n"
            "  rtos_state       : task state: Running/Ready/Blocked/Suspended/Deleted\n"
            "  rtos_priority    : FreeRTOS task priority (higher = more urgent)\n"
            "  stack_hwm_bytes  : stack high-water mark = smallest free stack seen (low = risk of overflow)\n"
            "  cpu_usage_pct    : CPU usage % measured over the %d ms window" % duration_ms,
            params={"scenario": scenario, "measurement_window_ms": duration_ms})
        _record("e11_cpu", path,
                "%s %d tasks" % (scenario, len(tasks_snap)))
        print("  Saved: %s" % path)

    return result


def e12_live_loop(model_path, camera_id=0, width=320, height=320,
                  repetitions=50, save=True):
    """Measure full perception loop: to_tensor + invoke + output read."""
    import gc
    print("[E12] Live loop — cam%d %dx%d, model=%s, %d reps" % (
        camera_id, width, height, model_path, repetitions))

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    sentai.camera.to_tensor()
    sentai.tpu.invoke()
    gc.collect()

    tensor_times = []
    invoke_times = []
    loop_times   = []

    for i in range(repetitions):
        t_loop = _ticks()

        t0 = _ticks(); sentai.camera.to_tensor(); tensor_times.append(_ticks() - t0)
        invoke_times.append(sentai.tpu.invoke())
        # tpu.output(0) returns a ~176 KB bytes object; without explicit GC the
        # micropython heap fragments and the next call raises MemoryError.
        sentai.tpu.output(0)
        gc.collect()
        loop_times.append(_ticks() - t_loop)

        if (i + 1) % 25 == 0:
            print("  ... %d/%d" % (i + 1, repetitions))

    st_tensor = stats(tensor_times)
    st_invoke = stats(invoke_times)
    st_loop   = stats(loop_times)
    fps = 1000.0 / st_loop["mean"] if st_loop["mean"] > 0 else 0
    meta = snapshot_meta("E12_live_loop", camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         model_path=model_path)

    print("[E12] Done.")
    _print_stats("to_tensor",  st_tensor)
    _print_stats("invoke",     st_invoke)
    _print_stats("loop_total", st_loop)
    print("  effective FPS: %.1f" % fps)

    result = {"experiment": "E12_live_loop", "meta": meta,
              "params": {"camera_id": camera_id, "width": width,
                         "height": height, "model_path": model_path,
                         "repetitions": repetitions},
              "samples": {"tensor_ms": tensor_times,
                          "invoke_ms": invoke_times,
                          "loop_ms": loop_times},
              "summary": {"tensor": st_tensor, "invoke": st_invoke,
                          "loop": st_loop, "fps": round(fps, 1)}}

    if save:
        path = _save_path("e12_loop_cam%d_%dx%d" % (camera_id, width, height))
        save_csv(path, ["run_index", "camera_to_tensor_ms", "edgetpu_invoke_ms", "total_loop_ms"],
                 [(i, tensor_times[i], invoke_times[i], loop_times[i])
                  for i in range(repetitions)])
        _save_desc(path,
            "E12 — Full perception loop latency.\n"
            "Measures the complete pipeline per frame: camera to_tensor + EdgeTPU invoke + output read.\n"
            "\nColumns:\n"
            "  run_index              : repetition number (0-based)\n"
            "  camera_to_tensor_ms    : time for camera.to_tensor() — PXP resize + quantize + DMA to TPU\n"
            "  edgetpu_invoke_ms      : time for tpu.invoke() — neural network forward pass on EdgeTPU\n"
            "  total_loop_ms          : total = to_tensor + invoke + tpu.output(0) read\n"
            "\nEffective FPS = 1000 / mean(total_loop_ms). Does NOT include sensor frame-wait time.",
            params={"camera_id": camera_id, "width": width, "height": height,
                    "model_path": model_path, "repetitions": repetitions})
        _record("e12_live_loop", path,
                "cam%d %dx%d loop=%.1f ms fps=%.1f" % (
                    camera_id, width, height, st_loop["mean"], fps))
        print("  Saved: %s" % path)

    return result
