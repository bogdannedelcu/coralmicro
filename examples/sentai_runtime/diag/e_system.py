# diag/e_system.py — E10: memory snapshot, E11: CPU/task state, E12: live loop

import sentai
from diag._util import (stats, save_csv, snapshot_meta, snapshot_heap,
                         snapshot_cpu, snapshot_tasks, _ticks, _print_stats)
from diag._session import _save_path, _record


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
                 ["scenario", "rtos_free", "gc_total", "gc_used",
                  "gc_free", "gc_max_free"],
                 [[scenario, heap["rtos_free"], heap["gc_total"],
                   heap["gc_used"], heap["gc_free"], heap["gc_max_free"]]])
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
        save_csv(path, ["task", "state", "priority", "stack_hwm", "cpu_pct"],
                 [[name, state, prio, hwm, cpu_map.get(name, 0)]
                  for name, state, prio, hwm in tasks_snap])
        _record("e11_cpu", path,
                "%s %d tasks" % (scenario, len(tasks_snap)))
        print("  Saved: %s" % path)

    return result


def e12_live_loop(model_path, camera_id=0, width=320, height=320,
                  repetitions=50, save=True):
    """Measure full perception loop: to_tensor + invoke + output read."""
    print("[E12] Live loop — cam%d %dx%d, model=%s, %d reps" % (
        camera_id, width, height, model_path, repetitions))

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    sentai.camera.to_tensor()
    sentai.tpu.invoke()

    tensor_times = []
    invoke_times = []
    loop_times   = []

    for i in range(repetitions):
        t_loop = _ticks()

        t0 = _ticks(); sentai.camera.to_tensor(); tensor_times.append(_ticks() - t0)
        invoke_times.append(sentai.tpu.invoke())
        sentai.tpu.output(0)
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
        save_csv(path, ["run", "tensor_ms", "invoke_ms", "loop_ms"],
                 [(i, tensor_times[i], invoke_times[i], loop_times[i])
                  for i in range(repetitions)])
        _record("e12_live_loop", path,
                "cam%d %dx%d loop=%.1f ms fps=%.1f" % (
                    camera_id, width, height, st_loop["mean"], fps))
        print("  Saved: %s" % path)

    return result
