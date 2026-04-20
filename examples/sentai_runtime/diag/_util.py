# diag/_util.py — pure utility functions (no session state)
# Imported by _session.py and all e_*.py modules.

import sentai
import math


def _ticks():
    return sentai.rtos.ticks_ms()


# Path-keyed guard for sentai.tpu.load().  Reloading the same model
# twice in a row bloats the EdgeTpuManager package cache (empirically
# drops FPS from 15 to 6 on the 20-run loop) — see paper/memcpy.md.
# Keep a single module-level cache of "last path loaded"; skip the
# reload if the caller is asking for the same file.  Calling with a
# different path (or after a tpu.reset()) forces a fresh load.
_last_tpu_model_path = None

def _ensure_model_loaded(model_path):
    """Idempotent sentai.tpu.load — loads only if the path changed
    since the last call.  Returns True if a load actually happened."""
    global _last_tpu_model_path
    if _last_tpu_model_path == model_path:
        return False
    sentai.tpu.load(model_path)
    _last_tpu_model_path = model_path
    return True


def percentile(samples, p):
    """Compute p-th percentile (0-100) from sorted copy."""
    s = sorted(samples)
    n = len(s)
    if n == 0:
        return 0
    k = (p / 100.0) * (n - 1)
    f = int(k)
    c = f + 1 if f + 1 < n else f
    d = k - f
    return s[f] + d * (s[c] - s[f])


def stats(samples):
    """Compute summary statistics dict from a list of numbers."""
    n = len(samples)
    if n == 0:
        return {"n": 0, "mean": 0, "min": 0, "max": 0,
                "median": 0, "p95": 0, "std": 0}
    s = sorted(samples)
    total = sum(s)
    mean = total / n
    variance = sum((x - mean) ** 2 for x in s) / n if n > 1 else 0
    return {
        "n": n,
        "mean": round(mean, 3),
        "min": s[0],
        "max": s[-1],
        "median": percentile(s, 50),
        "p95": percentile(s, 95),
        "std": round(math.sqrt(variance), 3),
    }


def time_call(fn, *args, **kwargs):
    """Measure one call in ms. Returns (elapsed_ms, return_value)."""
    t0 = _ticks()
    rv = fn(*args, **kwargs)
    t1 = _ticks()
    return (t1 - t0, rv)


def ensure_dir(path="/diags"):
    """Create output directory if missing."""
    if not sentai.fs.exists(path):
        sentai.fs.mkdir(path)


def save_csv(path, header, rows):
    """Save rows (list of lists/tuples) to CSV."""
    parent = path.rsplit("/", 1)[0] if "/" in path else "/diags"
    ensure_dir(parent)
    lines = [",".join(str(c) for c in header)]
    for row in rows:
        lines.append(",".join(str(c) for c in row))
    sentai.fs.write(path, "\n".join(lines) + "\n")


def snapshot_meta(experiment, **extra):
    """Return common metadata dict."""
    d = {
        "experiment": experiment,
        "uptime_ms": sentai.rtos.uptime(),
    }
    d.update(extra)
    return d


def snapshot_heap():
    return sentai.rtos.heap_info()


def snapshot_cpu():
    return sentai.rtos.cpu_usage()


def snapshot_tasks():
    return sentai.rtos.tasks()


def _print_stats(name, st):
    """Pretty-print summary stats."""
    print("  %s: mean=%.1f min=%.1f max=%.1f med=%.1f p95=%.1f std=%.1f ms (n=%d)" % (
        name, st["mean"], st["min"], st["max"],
        st["median"], st["p95"], st["std"], st["n"]))


def _print_heap(label, h):
    print("  %s: rtos_free=%d gc_used=%d gc_free=%d" % (
        label, h["rtos_free"], h["gc_used"], h["gc_free"]))
