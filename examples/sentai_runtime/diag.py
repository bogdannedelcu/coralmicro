# diag.py — sentai.diag diagnostics and benchmarking module
# Upload to device as /lib/diag.py, then: import diag
# All experiments are REPL-driven: call manually, see results live.
#
# Workflow:
#   diag.begin("cam0_yolo26n")     # opens session s004_cam0_yolo26n
#   diag.e1_tpu_invoke(...)        # saves 001_e1_tpu_invoke.csv
#   diag.e3_camera_tensor(...)     # saves 002_e3_camera_tensor.csv
#   diag.end()                     # writes manifest, closes session
#
# Sessions live under /diags/sNNN_name/ and never overwrite each other.

import sentai
import math

# ─────────────────────────────────────────────────────────
# Session manager
# ─────────────────────────────────────────────────────────

_session = None  # active session state

class Session:
    def __init__(self, name, sid):
        self.name = name
        self.sid = sid
        self.tag = "s%03d_%s" % (sid, name)
        self.dir = "/diags/%s" % self.tag
        self.seq = 0
        self.log = []  # list of (seq, experiment, csv_path, summary_line)
        self.t_start = sentai.rtos.uptime()
        self.heap_start = sentai.rtos.heap_info()

    def next_path(self, experiment):
        """Return next CSV path inside session dir."""
        self.seq += 1
        fname = "%03d_%s.csv" % (self.seq, experiment)
        return "%s/%s" % (self.dir, fname)

    def record(self, experiment, csv_path, summary_line=""):
        """Log an experiment completion."""
        self.log.append((self.seq, experiment, csv_path, summary_line))

def _next_session_id():
    """Read and increment persistent counter in /diags/.counter."""
    ensure_dir("/diags")
    counter_path = "/diags/.counter"
    sid = 1
    if sentai.fs.exists(counter_path):
        try:
            raw = sentai.fs.read_str(counter_path)
            sid = int(raw.strip()) + 1
        except:
            sid = 1
    sentai.fs.write(counter_path, str(sid))
    return sid

def begin(name="experiment"):
    """Start a new diagnostics session.
    Creates /diags/sNNN_name/ directory.
    All subsequent experiments save into this directory.
    """
    global _session
    if _session is not None:
        print("WARNING: session '%s' still open — closing it first." % _session.tag)
        end()

    sid = _next_session_id()
    _session = Session(name, sid)
    sentai.fs.mkdir(_session.dir)

    # Write manifest header
    sentai.fs.write("%s/manifest.csv" % _session.dir,
                    "seq,experiment,file,summary\n")

    print("=" * 50)
    print("SESSION %s started" % _session.tag)
    print("  dir: %s" % _session.dir)
    print("=" * 50)
    return _session.tag

def end():
    """Close the active session. Write final manifest and summary."""
    global _session
    if _session is None:
        print("No active session.")
        return

    elapsed = sentai.rtos.uptime() - _session.t_start
    heap_end = sentai.rtos.heap_info()

    # Write full manifest
    lines = ["seq,experiment,file,summary"]
    for seq, exp, path, summ in _session.log:
        lines.append("%d,%s,%s,%s" % (seq, exp, path, summ.replace(",", ";")))
    sentai.fs.write("%s/manifest.csv" % _session.dir, "\n".join(lines) + "\n")

    # Write session summary
    summary_lines = [
        "session: %s" % _session.tag,
        "experiments: %d" % len(_session.log),
        "duration_ms: %d" % elapsed,
        "heap_start_rtos_free: %d" % _session.heap_start["rtos_free"],
        "heap_end_rtos_free: %d" % heap_end["rtos_free"],
        "heap_start_gc_used: %d" % _session.heap_start["gc_used"],
        "heap_end_gc_used: %d" % heap_end["gc_used"],
    ]
    sentai.fs.write("%s/summary.txt" % _session.dir,
                    "\n".join(summary_lines) + "\n")

    print("=" * 50)
    print("SESSION %s closed" % _session.tag)
    print("  experiments: %d" % len(_session.log))
    print("  duration: %d ms" % elapsed)
    print("  dir: %s" % _session.dir)
    print("=" * 50)

    tag = _session.tag
    _session = None
    return tag

def status():
    """Show active session status."""
    if _session is None:
        print("No active session. Call diag.begin('name') to start.")
        return
    elapsed = sentai.rtos.uptime() - _session.t_start
    print("Session: %s  |  experiments: %d  |  elapsed: %d ms" % (
        _session.tag, len(_session.log), elapsed))
    for seq, exp, path, _ in _session.log:
        print("  %03d  %s" % (seq, exp))

def _save_path(experiment):
    """Get save path: use session dir if active, else /diags/ flat."""
    if _session:
        return _session.next_path(experiment)
    ensure_dir("/diags")
    return "/diags/%s.csv" % experiment

def _record(experiment, csv_path, summary_line=""):
    """Record experiment in session log if active."""
    if _session:
        _session.record(experiment, csv_path, summary_line)

# ─────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────

def _ticks():
    return sentai.rtos.ticks_ms()

def _sort(lst):
    """In-place sort for MicroPython (list.sort() exists)."""
    lst.sort()
    return lst

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
    # ensure parent dir exists (session dir or /diags/)
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

# ─────────────────────────────────────────────────────────
# E1 — TPU invoke latency
# ─────────────────────────────────────────────────────────

def e1_tpu_invoke(model_path, image_path=None, use_camera=False,
                  repetitions=100, save=True):
    """Measure sentai.tpu.invoke() latency.
    Model must be loaded first, or provide model_path to auto-load.
    If image_path given, loads that image. If use_camera, calls to_tensor().
    """
    print("[E1] TPU invoke latency — %d reps" % repetitions)

    # Setup
    sentai.tpu.load(model_path)
    if image_path:
        sentai.tpu.load_image(image_path)
    elif use_camera:
        sentai.camera.to_tensor()

    # Warm-up
    sentai.tpu.invoke()

    # Measure
    samples = []
    for i in range(repetitions):
        if use_camera:
            sentai.camera.to_tensor()
        ms = sentai.tpu.invoke()  # returns elapsed ms directly
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
        save_csv(path, ["run", "invoke_ms"], [(i, s) for i, s in enumerate(samples)])
        _record("e1_tpu_invoke", path, "mean=%.1f p95=%.1f ms" % (st["mean"], st["p95"]))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E2 — TPU model loading
# ─────────────────────────────────────────────────────────

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
        save_csv(path, ["run", "load_ms"], [(i, s) for i, s in enumerate(samples)])
        _record("e2_tpu_load", path, "mean=%.1f ms" % st["mean"])
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E3 — Camera-to-tensor
# ─────────────────────────────────────────────────────────

def e3_camera_tensor(camera_id=0, width=1280, height=720,
                     repetitions=50, save=True):
    """Measure camera frame rate and to_tensor() cost (PXP+memcpy).
    Three measurements:
      1) Bulk to_tensor() — N calls back-to-back, total/N = PXP+memcpy cost
      2) Sensor FPS — poll frame_count() for N new frames
      3) to_tensor per new frame — wait for new frame then to_tensor()
    """
    print("[E3] Camera tensor — cam%d %dx%d, %d reps" % (
        camera_id, width, height, repetitions))

    # to_tensor() requires TPU loaded (needs interpreter input tensor)
    if not sentai.tpu.ready():
        sentai.tpu.load("/yolo26n.edgetpu_1.tflite")
        print("  [E3] loaded model for to_tensor")

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    # Warm-up — wait for first real frame
    sentai.rtos.sleep_ms(200)
    sentai.camera.to_tensor()

    # --- Method 1: bulk to_tensor (same buffer, measures PXP+memcpy) ---
    n_bulk = max(repetitions, 100)
    t_start = _ticks()
    for _ in range(n_bulk):
        sentai.camera.to_tensor()
    t_end = _ticks()
    bulk_total_ms = t_end - t_start
    bulk_per_call = bulk_total_ms / n_bulk if n_bulk > 0 else 0
    print("  to_tensor bulk: %d calls in %d ms = %.2f ms/call (PXP+memcpy)" % (
        n_bulk, bulk_total_ms, bulk_per_call))

    # --- Method 2: sensor FPS via frame_count polling ---
    n_fps = min(repetitions, 30)
    fc0 = sentai.camera.frame_count()
    t0 = _ticks()
    for _ in range(n_fps):
        fc = sentai.camera.frame_count()
        while sentai.camera.frame_count() == fc:
            pass
    t1 = _ticks()
    sensor_total = t1 - t0
    sensor_per_frame = sensor_total / n_fps if n_fps > 0 else 0
    sensor_fps = 1000.0 / sensor_per_frame if sensor_per_frame > 0 else 0
    print("  sensor: %d frames in %d ms = %.1f ms/frame = %.1f FPS" % (
        n_fps, sensor_total, sensor_per_frame, sensor_fps))

    # --- Method 3: wait + to_tensor (full pipeline per frame) ---
    pipe_times = []
    tensor_times = []
    for i in range(min(repetitions, 20)):
        fc = sentai.camera.frame_count()
        t0 = _ticks()
        while sentai.camera.frame_count() == fc:
            pass
        t_got = _ticks()
        sentai.camera.to_tensor()
        t_done = _ticks()
        pipe_times.append(t_done - t0)
        tensor_times.append(t_done - t_got)

    st_pipe = stats(pipe_times)
    st_tens = stats(tensor_times)
    fps_pipe = 1000.0 / st_pipe["mean"] if st_pipe["mean"] > 0 else 0

    res = sentai.camera.resolution()
    meta = snapshot_meta("E3_camera_tensor", camera_id=camera_id,
                         resolution="%dx%d" % (res[0], res[1]))

    print("[E3] Done.")
    _print_stats("pipe(wait+tensor)", st_pipe)
    _print_stats("to_tensor_only", st_tens)
    print("  pipeline FPS: %.1f" % fps_pipe)

    result = {"experiment": "E3_camera_tensor", "meta": meta,
              "params": {"camera_id": camera_id, "width": width,
                         "height": height, "repetitions": repetitions},
              "samples": pipe_times,
              "summary": st_pipe,
              "extra": {"bulk_total_ms": bulk_total_ms,
                        "bulk_per_call_ms": bulk_per_call,
                        "bulk_n": n_bulk,
                        "sensor_fps": int(sensor_fps),
                        "sensor_per_frame_ms": sensor_per_frame,
                        "tensor_only": st_tens,
                        "fps_pipe": int(fps_pipe)}}

    if save:
        path = _save_path("e3_cam%d_%dx%d" % (camera_id, width, height))
        save_csv(path, ["run", "pipe_ms", "tensor_ms"],
                 [(i, pipe_times[i], tensor_times[i])
                  for i in range(len(pipe_times))])
        _record("e3_camera_tensor", path,
                "cam%d %dx%d bulk=%.2fms sensor=%.0ffps pipe=%.0ffps" % (
                    camera_id, width, height, bulk_per_call,
                    sensor_fps, fps_pipe))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E4 — JPEG capture
# ─────────────────────────────────────────────────────────

def e4_jpeg(camera_id=0, width=1280, height=720, quality=75,
            repetitions=30, save=True):
    """Measure sentai.camera.jpeg() and save_jpeg() latency + sizes."""
    print("[E4] JPEG capture — cam%d %dx%d q=%d, %d reps" % (
        camera_id, width, height, quality, repetitions))

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.camera.jpeg(quality)  # warm-up

    jpeg_times = []
    jpeg_sizes = []
    save_times = []

    for i in range(repetitions):
        # jpeg() — returns bytes
        elapsed, data = time_call(sentai.camera.jpeg, quality)
        jpeg_times.append(elapsed)
        jpeg_sizes.append(len(data))

        # save_jpeg()
        elapsed2, sz = time_call(sentai.camera.save_jpeg,
                                 "/diags/_e4_tmp.jpg", quality)
        save_times.append(elapsed2)

    st_jpeg = stats(jpeg_times)
    st_save = stats(save_times)
    st_size = stats(jpeg_sizes)
    meta = snapshot_meta("E4_jpeg", camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         quality=quality)

    print("[E4] Done.")
    _print_stats("jpeg()", st_jpeg)
    _print_stats("save_jpeg()", st_save)
    print("  sizes: mean=%.0f min=%d max=%d bytes" % (
        st_size["mean"], st_size["min"], st_size["max"]))

    result = {"experiment": "E4_jpeg", "meta": meta,
              "params": {"camera_id": camera_id, "width": width,
                         "height": height, "quality": quality,
                         "repetitions": repetitions},
              "samples": {"jpeg_ms": jpeg_times, "save_ms": save_times,
                          "jpeg_bytes": jpeg_sizes},
              "summary": {"jpeg": st_jpeg, "save": st_save, "sizes": st_size}}

    if save:
        path = _save_path("e4_jpeg_cam%d_q%d" % (camera_id, quality))
        save_csv(path, ["run", "jpeg_ms", "save_ms", "jpeg_bytes"],
                 [(i, jpeg_times[i], save_times[i], jpeg_sizes[i])
                  for i in range(repetitions)])
        _record("e4_jpeg", path, "cam%d q=%d jpeg=%.1f save=%.1f ms" % (camera_id, quality, st_jpeg["mean"], st_save["mean"]))
        print("  Saved: %s" % path)

    # Clean up temp file
    sentai.fs.remove("/diags/_e4_tmp.jpg")
    return result

# ─────────────────────────────────────────────────────────
# E5 — Camera switch
# ─────────────────────────────────────────────────────────

def e5_camera_switch(from_cam=0, to_cam=1, repetitions=20, save=True):
    """Measure camera switch round-trip: select(to) + drain N frames + select(back).
    Each rep measures the full overhead of switching cameras and getting a
    fresh frame from the new camera (not a stale DMA buffer).
    """
    print("[E5] Camera switch %d->%d, %d reps" % (from_cam, to_cam, repetitions))

    # to_tensor() requires TPU loaded
    if not sentai.tpu.ready():
        sentai.tpu.load("/yolo26n.edgetpu_1.tflite")
        print("  [E5] loaded model for to_tensor")

    sentai.camera.init(1)
    sentai.camera.select(from_cam)
    # drain warm-up frames
    for _ in range(5):
        sentai.camera.to_tensor()

    switch_times = []
    first_fresh_times = []
    roundtrip_times = []

    for i in range(repetitions):
        # Ensure we're on from_cam with a fresh frame
        sentai.camera.select(from_cam)
        sentai.camera.to_tensor()
        sentai.rtos.sleep_ms(50)

        # Measure: switch + wait for truly new frame via frame_count
        t_all = _ticks()

        t0 = _ticks()
        sentai.camera.select(to_cam)
        switch_times.append(_ticks() - t0)

        # Wait for a genuinely new frame from the new camera
        t0 = _ticks()
        fc = sentai.camera.frame_count()
        while sentai.camera.frame_count() == fc:
            pass
        sentai.camera.to_tensor()
        first_fresh_times.append(_ticks() - t0)

        roundtrip_times.append(_ticks() - t_all)

    st_switch = stats(switch_times)
    st_fresh = stats(first_fresh_times)
    st_rt = stats(roundtrip_times)
    meta = snapshot_meta("E5_camera_switch",
                         from_cam=from_cam, to_cam=to_cam)

    print("[E5] Done.")
    _print_stats("select()", st_switch)
    _print_stats("wait+tensor", st_fresh)
    _print_stats("roundtrip", st_rt)

    result = {"experiment": "E5_camera_switch", "meta": meta,
              "params": {"from_cam": from_cam, "to_cam": to_cam,
                         "repetitions": repetitions},
              "samples": {"switch_ms": switch_times,
                          "wait_tensor_ms": first_fresh_times,
                          "roundtrip_ms": roundtrip_times},
              "summary": {"switch": st_switch, "wait_tensor": st_fresh,
                          "roundtrip": st_rt}}

    if save:
        path = _save_path("e5_switch_%dto%d" % (from_cam, to_cam))
        save_csv(path, ["run", "switch_ms", "wait_tensor_ms", "roundtrip_ms"],
                 [(i, switch_times[i], first_fresh_times[i], roundtrip_times[i])
                  for i in range(repetitions)])
        _record("e5_camera_switch", path,
                "%d->%d switch=%.1f roundtrip=%.1f ms" % (
                    from_cam, to_cam, st_switch["mean"], st_rt["mean"]))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E5b — Alternating 1:1 camera switch under load
# ─────────────────────────────────────────────────────────

def e5b_alternating(model_path, cam_nadir=0, cam_forward=1,
                    width=320, height=320, cycles=20, save=True):
    """Alternate between nadir and forward cameras every frame.
    Each cycle: select→to_tensor→invoke on cam A, then same on cam B.
    Measures switch overhead, to_tensor, and invoke for each camera.
    """
    print("[E5b] Alternating 1:1 — cam%d/cam%d, %d cycles" % (
        cam_nadir, cam_forward, cycles))

    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    # Warm-up on nadir
    sentai.camera.select(cam_nadir)
    sentai.camera.to_tensor()
    sentai.tpu.invoke()

    nadir_switch = []
    nadir_tensor = []
    nadir_invoke = []
    fwd_switch = []
    fwd_tensor = []
    fwd_invoke = []

    for i in range(cycles):
        # --- nadir frame ---
        t0 = _ticks()
        sentai.camera.select(cam_nadir)
        t1 = _ticks()
        nadir_switch.append(t1 - t0)

        t0 = _ticks()
        sentai.camera.to_tensor()
        t1 = _ticks()
        nadir_tensor.append(t1 - t0)

        ms = sentai.tpu.invoke()
        nadir_invoke.append(ms)

        # --- forward frame ---
        t0 = _ticks()
        sentai.camera.select(cam_forward)
        t1 = _ticks()
        fwd_switch.append(t1 - t0)

        t0 = _ticks()
        sentai.camera.to_tensor()
        t1 = _ticks()
        fwd_tensor.append(t1 - t0)

        ms = sentai.tpu.invoke()
        fwd_invoke.append(ms)

        if (i + 1) % 10 == 0:
            print("  ... %d/%d cycles" % (i + 1, cycles))

    st_ns = stats(nadir_switch)
    st_nt = stats(nadir_tensor)
    st_ni = stats(nadir_invoke)
    st_fs = stats(fwd_switch)
    st_ft = stats(fwd_tensor)
    st_fi = stats(fwd_invoke)

    # Total per-cycle time
    cycle_times = [nadir_switch[i] + nadir_tensor[i] + nadir_invoke[i] +
                   fwd_switch[i] + fwd_tensor[i] + fwd_invoke[i]
                   for i in range(cycles)]
    st_cycle = stats(cycle_times)
    fps = 1000.0 / st_cycle["mean"] if st_cycle["mean"] > 0 else 0

    meta = snapshot_meta("E5b_alternating", cam_nadir=cam_nadir,
                         cam_forward=cam_forward, model_path=model_path,
                         resolution="%dx%d" % (width, height))

    print("[E5b] Done.")
    print("  Nadir (cam%d):" % cam_nadir)
    _print_stats("  switch", st_ns)
    _print_stats("  tensor", st_nt)
    _print_stats("  invoke", st_ni)
    print("  Forward (cam%d):" % cam_forward)
    _print_stats("  switch", st_fs)
    _print_stats("  tensor", st_ft)
    _print_stats("  invoke", st_fi)
    print("  Cycle: mean=%.1f ms  fps=%.1f (both cameras)" % (
        st_cycle["mean"], fps))

    result = {"experiment": "E5b_alternating", "meta": meta,
              "params": {"cam_nadir": cam_nadir, "cam_forward": cam_forward,
                         "width": width, "height": height, "cycles": cycles,
                         "model_path": model_path},
              "samples": {
                  "nadir_switch": nadir_switch, "nadir_tensor": nadir_tensor,
                  "nadir_invoke": nadir_invoke,
                  "fwd_switch": fwd_switch, "fwd_tensor": fwd_tensor,
                  "fwd_invoke": fwd_invoke, "cycle_ms": cycle_times},
              "summary": {
                  "nadir_switch": st_ns, "nadir_tensor": st_nt,
                  "nadir_invoke": st_ni,
                  "fwd_switch": st_fs, "fwd_tensor": st_ft,
                  "fwd_invoke": st_fi,
                  "cycle": st_cycle, "fps": round(fps, 1)}}

    if save:
        path = _save_path("e5b_alt_1to1")
        save_csv(path,
                 ["cycle", "nadir_sw", "nadir_tens", "nadir_inv",
                  "fwd_sw", "fwd_tens", "fwd_inv", "cycle_ms"],
                 [(i, nadir_switch[i], nadir_tensor[i], nadir_invoke[i],
                   fwd_switch[i], fwd_tensor[i], fwd_invoke[i], cycle_times[i])
                  for i in range(cycles)])
        _record("e5b_alternating", path,
                "1:1 cycle=%.1f ms fps=%.1f" % (st_cycle["mean"], fps))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E5c — Asymmetric duty cycle (nadir continuous, forward at N Hz)
# ─────────────────────────────────────────────────────────

def e5c_asymmetric(model_path, cam_nadir=0, cam_forward=1,
                   width=320, height=320,
                   fwd_every_n=5, total_nadir_frames=50, save=True):
    """Nadir camera runs every frame; forward camera interleaved every N frames.
    Models the real flight pattern: nadir (down) is primary for continuous
    detection/tracking, forward (horizontal) is sampled less frequently
    for situational awareness.

    fwd_every_n=5 means 1 forward frame per 5 nadir frames (~ratio 1:5).
    fwd_every_n=10 means ~1 Hz forward if nadir runs at ~10 fps.

    Measures:
    - nadir-only frames (no switch overhead)
    - nadir frames right before a forward switch
    - forward switch + capture + return to nadir overhead
    - total frame-to-frame timing to compute effective nadir FPS
    """
    fwd_hz_approx = "~%.0f Hz" % (10.0 / fwd_every_n) if fwd_every_n > 0 else "off"
    print("[E5c] Asymmetric — nadir=cam%d continuous, fwd=cam%d every %d frames (%s)" % (
        cam_nadir, cam_forward, fwd_every_n, fwd_hz_approx))
    print("  %d total nadir frames, model=%s" % (total_nadir_frames, model_path))

    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    # Start on nadir
    sentai.camera.select(cam_nadir)
    sentai.camera.to_tensor()
    sentai.tpu.invoke()

    # Per-frame records
    frame_type = []      # 'nadir' or 'fwd_round'
    frame_total_ms = []   # wall-clock per logical frame
    nadir_tensor_ms = []
    nadir_invoke_ms = []
    # Forward round-trip records (only on fwd frames)
    fwd_switch_to_ms = []
    fwd_tensor_ms = []
    fwd_invoke_ms = []
    fwd_switch_back_ms = []
    fwd_roundtrip_ms = []

    nadir_count = 0

    for i in range(total_nadir_frames):
        t_frame = _ticks()

        # --- nadir frame (always) ---
        t0 = _ticks()
        sentai.camera.to_tensor()
        t1 = _ticks()
        nt = t1 - t0

        inv = sentai.tpu.invoke()
        nadir_tensor_ms.append(nt)
        nadir_invoke_ms.append(inv)
        nadir_count += 1

        # --- forward interleave? ---
        if fwd_every_n > 0 and (i + 1) % fwd_every_n == 0:
            t_rt = _ticks()

            # switch to forward
            t0 = _ticks()
            sentai.camera.select(cam_forward)
            t1 = _ticks()
            sw_to = t1 - t0

            # capture + invoke on forward
            t0 = _ticks()
            sentai.camera.to_tensor()
            t1 = _ticks()
            ft = t1 - t0

            fi = sentai.tpu.invoke()

            # switch back to nadir
            t0 = _ticks()
            sentai.camera.select(cam_nadir)
            t1 = _ticks()
            sw_back = t1 - t0

            rt = _ticks() - t_rt

            fwd_switch_to_ms.append(sw_to)
            fwd_tensor_ms.append(ft)
            fwd_invoke_ms.append(fi)
            fwd_switch_back_ms.append(sw_back)
            fwd_roundtrip_ms.append(rt)

            frame_type.append("fwd_round")
            frame_total_ms.append(_ticks() - t_frame)
        else:
            frame_type.append("nadir")
            frame_total_ms.append(_ticks() - t_frame)

        if (i + 1) % 25 == 0:
            print("  ... %d/%d nadir frames" % (i + 1, total_nadir_frames))

    # Separate nadir-only frame times from frames that included a fwd round-trip
    nadir_only_total = [frame_total_ms[i] for i in range(len(frame_type))
                        if frame_type[i] == "nadir"]
    fwd_frame_total = [frame_total_ms[i] for i in range(len(frame_type))
                       if frame_type[i] == "fwd_round"]

    st_nt = stats(nadir_tensor_ms)
    st_ni = stats(nadir_invoke_ms)
    st_nadir_only = stats(nadir_only_total)
    st_fwd_frame = stats(fwd_frame_total)
    st_fwd_rt = stats(fwd_roundtrip_ms)
    st_fwd_sw_to = stats(fwd_switch_to_ms)
    st_fwd_sw_back = stats(fwd_switch_back_ms)
    st_all_frames = stats(frame_total_ms)

    nadir_fps = 1000.0 / st_nadir_only["mean"] if st_nadir_only["mean"] > 0 else 0
    effective_fps = 1000.0 / st_all_frames["mean"] if st_all_frames["mean"] > 0 else 0
    fwd_count = len(fwd_roundtrip_ms)

    meta = snapshot_meta("E5c_asymmetric", cam_nadir=cam_nadir,
                         cam_forward=cam_forward, fwd_every_n=fwd_every_n,
                         model_path=model_path,
                         resolution="%dx%d" % (width, height))

    print("[E5c] Done.")
    print("  Nadir (cam%d) — %d frames:" % (cam_nadir, nadir_count))
    _print_stats("  to_tensor", st_nt)
    _print_stats("  invoke", st_ni)
    _print_stats("  nadir-only frame", st_nadir_only)
    print("  nadir-only FPS: %.1f" % nadir_fps)
    if fwd_count > 0:
        print("  Forward (cam%d) — %d round-trips:" % (cam_forward, fwd_count))
        _print_stats("  switch_to", st_fwd_sw_to)
        _print_stats("  switch_back", st_fwd_sw_back)
        _print_stats("  roundtrip", st_fwd_rt)
        _print_stats("  frame w/ fwd", st_fwd_frame)
    print("  Effective FPS (all frames): %.1f" % effective_fps)
    print("  FPS penalty from fwd: %.1f%%" % (
        (1.0 - effective_fps / nadir_fps) * 100 if nadir_fps > 0 else 0))

    result = {"experiment": "E5c_asymmetric", "meta": meta,
              "params": {"cam_nadir": cam_nadir, "cam_forward": cam_forward,
                         "fwd_every_n": fwd_every_n,
                         "total_nadir_frames": total_nadir_frames,
                         "width": width, "height": height,
                         "model_path": model_path},
              "samples": {
                  "frame_type": frame_type, "frame_total_ms": frame_total_ms,
                  "nadir_tensor_ms": nadir_tensor_ms,
                  "nadir_invoke_ms": nadir_invoke_ms,
                  "fwd_switch_to_ms": fwd_switch_to_ms,
                  "fwd_tensor_ms": fwd_tensor_ms,
                  "fwd_invoke_ms": fwd_invoke_ms,
                  "fwd_switch_back_ms": fwd_switch_back_ms,
                  "fwd_roundtrip_ms": fwd_roundtrip_ms},
              "summary": {
                  "nadir_tensor": st_nt, "nadir_invoke": st_ni,
                  "nadir_only_frame": st_nadir_only,
                  "fwd_roundtrip": st_fwd_rt,
                  "fwd_frame": st_fwd_frame,
                  "all_frames": st_all_frames,
                  "nadir_fps": round(nadir_fps, 1),
                  "effective_fps": round(effective_fps, 1),
                  "fwd_count": fwd_count}}

    if save:
        path = _save_path("e5c_asym_1to%d" % fwd_every_n)
        rows = []
        fi = 0  # forward index
        for i in range(len(frame_type)):
            if frame_type[i] == "fwd_round" and fi < fwd_count:
                rows.append((i, frame_type[i], frame_total_ms[i],
                             nadir_tensor_ms[i], nadir_invoke_ms[i],
                             fwd_switch_to_ms[fi], fwd_tensor_ms[fi],
                             fwd_invoke_ms[fi], fwd_switch_back_ms[fi],
                             fwd_roundtrip_ms[fi]))
                fi += 1
            else:
                rows.append((i, frame_type[i], frame_total_ms[i],
                             nadir_tensor_ms[i], nadir_invoke_ms[i],
                             "", "", "", "", ""))
        save_csv(path,
                 ["frame", "type", "total_ms", "nadir_tens", "nadir_inv",
                  "fwd_sw_to", "fwd_tens", "fwd_inv", "fwd_sw_back", "fwd_rt"],
                 rows)
        _record("e5c_asymmetric", path,
                "1:%d nadir_fps=%.1f eff_fps=%.1f penalty=%.1f%%" % (
                    fwd_every_n, nadir_fps, effective_fps,
                    (1.0 - effective_fps / nadir_fps) * 100 if nadir_fps > 0 else 0))
        print("  Saved: %s" % path)

    return result

def e5c_sweep(model_path, cam_nadir=0, cam_forward=1,
              ratios=None, total_nadir_frames=50, save=True):
    """Run E5c across multiple forward-camera duty cycles.
    Default ratios: [0, 2, 5, 10, 20] meaning forward every N nadir frames.
    ratio=0 means nadir-only (baseline, no switching).
    """
    if ratios is None:
        ratios = [0, 2, 5, 10, 20]
    print("[E5c sweep] Ratios: %s" % ratios)

    results = {}
    for r in ratios:
        tag = "baseline" if r == 0 else "1to%d" % r
        results[tag] = e5c_asymmetric(
            model_path, cam_nadir, cam_forward,
            fwd_every_n=r, total_nadir_frames=total_nadir_frames, save=save)

    # Print comparison table
    print("\n  %-12s  %8s  %8s  %8s" % ("Ratio", "Nadir FPS", "Eff FPS", "Penalty"))
    print("  " + "-" * 42)
    for r in ratios:
        tag = "baseline" if r == 0 else "1to%d" % r
        s = results[tag]["summary"]
        nfps = s.get("nadir_fps", s.get("effective_fps", 0))
        efps = s["effective_fps"]
        pen = (1.0 - efps / nfps) * 100 if nfps > 0 else 0
        label = "nadir only" if r == 0 else "1:%d" % r
        print("  %-12s  %8.1f  %8.1f  %7.1f%%" % (label, nfps, efps, pen))

    return results

# ─────────────────────────────────────────────────────────
# E6 — Filesystem read
# ─────────────────────────────────────────────────────────

def e6_fs_read(path, repetitions=20, save=True):
    """Measure sentai.fs.read() latency and throughput."""
    print("[E6] FS read — '%s', %d reps" % (path, repetitions))

    file_size = sentai.fs.size(path)
    if file_size < 0:
        print("  ERROR: file not found")
        return None

    samples = []
    for i in range(repetitions):
        elapsed, data = time_call(sentai.fs.read, path)
        samples.append(elapsed)

    st = stats(samples)
    throughput_kbs = (file_size / 1024.0) / (st["mean"] / 1000.0) if st["mean"] > 0 else 0
    meta = snapshot_meta("E6_fs_read", path=path, file_size=file_size)

    print("[E6] Done. file_size=%d bytes" % file_size)
    _print_stats("read", st)
    print("  throughput: %.1f KB/s" % throughput_kbs)

    result = {"experiment": "E6_fs_read", "meta": meta,
              "params": {"path": path, "repetitions": repetitions},
              "samples": samples, "summary": st,
              "extra": {"file_size": file_size, "throughput_kbs": round(throughput_kbs, 1)}}

    if save:
        csv_path = _save_path("e6_fs_read")
        save_csv(csv_path, ["run", "read_ms"],
                 [(i, s) for i, s in enumerate(samples)])
        _record("e6_fs_read", csv_path, "%.1f KB/s mean=%.1f ms" % (throughput_kbs, st["mean"]))
        print("  Saved: %s" % csv_path)

    return result

# ─────────────────────────────────────────────────────────
# E7 — Filesystem write
# ─────────────────────────────────────────────────────────

def e7_fs_write(size=4096, repetitions=20, save=True):
    """Measure sentai.fs.write() latency and throughput."""
    print("[E7] FS write — %d bytes, %d reps" % (size, repetitions))

    data = b'\xAA' * size
    test_path = "/diags/_e7_tmp.bin"

    samples = []
    for i in range(repetitions):
        elapsed, _ = time_call(sentai.fs.write, test_path, data)
        samples.append(elapsed)

    st = stats(samples)
    throughput_kbs = (size / 1024.0) / (st["mean"] / 1000.0) if st["mean"] > 0 else 0
    meta = snapshot_meta("E7_fs_write", size=size)

    print("[E7] Done.")
    _print_stats("write", st)
    print("  throughput: %.1f KB/s" % throughput_kbs)

    result = {"experiment": "E7_fs_write", "meta": meta,
              "params": {"size": size, "repetitions": repetitions},
              "samples": samples, "summary": st,
              "extra": {"data_size": size, "throughput_kbs": round(throughput_kbs, 1)}}

    if save:
        csv_path = _save_path("e7_fs_write_%db" % size)
        save_csv(csv_path, ["run", "write_ms"],
                 [(i, s) for i, s in enumerate(samples)])
        _record("e7_fs_write", csv_path, "%dB %.1f KB/s mean=%.1f ms" % (size, throughput_kbs, st["mean"]))
        print("  Saved: %s" % csv_path)

    sentai.fs.remove(test_path)
    return result

# ─────────────────────────────────────────────────────────
# E8 — IMU read
# ─────────────────────────────────────────────────────────

def e8_imu(repetitions=100, save=True):
    """Measure sentai.imu.read(), degrees(), radians() latency."""
    print("[E8] IMU read — %d reps" % repetitions)

    sentai.imu.init()

    read_times = []
    deg_times = []
    rad_times = []

    for i in range(repetitions):
        elapsed, _ = time_call(sentai.imu.read)
        read_times.append(elapsed)
        elapsed, _ = time_call(sentai.imu.degrees)
        deg_times.append(elapsed)
        elapsed, _ = time_call(sentai.imu.radians)
        rad_times.append(elapsed)

    st_read = stats(read_times)
    st_deg = stats(deg_times)
    st_rad = stats(rad_times)
    meta = snapshot_meta("E8_imu")

    print("[E8] Done.")
    _print_stats("read()", st_read)
    _print_stats("degrees()", st_deg)
    _print_stats("radians()", st_rad)

    result = {"experiment": "E8_imu", "meta": meta,
              "params": {"repetitions": repetitions},
              "samples": {"read_ms": read_times, "degrees_ms": deg_times,
                          "radians_ms": rad_times},
              "summary": {"read": st_read, "degrees": st_deg, "radians": st_rad}}

    if save:
        path = _save_path("e8_imu")
        save_csv(path, ["run", "read_ms", "degrees_ms", "radians_ms"],
                 [(i, read_times[i], deg_times[i], rad_times[i])
                  for i in range(repetitions)])
        _record("e8_imu", path, "read=%.1f deg=%.1f rad=%.1f ms" % (st_read["mean"], st_deg["mean"], st_rad["mean"]))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E9 — Microphone
# ─────────────────────────────────────────────────────────

def e9_mic(seconds=2, repetitions=5, save=True):
    """Measure mic.start(), level(), save_mp3() latency."""
    print("[E9] Microphone — %ds recording, %d reps" % (seconds, repetitions))

    start_times = []
    level_times = []
    save_times = []
    mp3_sizes = []

    for i in range(repetitions):
        # start ring recording
        elapsed, _ = time_call(sentai.mic.start, seconds)
        start_times.append(elapsed)

        # ring buffer records continuously; wait for duration then stop
        sentai.rtos.sleep_ms(seconds * 1000 + 200)
        sentai.mic.stop()

        # level (measures last recorded buffer level)
        elapsed, _ = time_call(sentai.mic.level)
        level_times.append(elapsed)

        # save_mp3
        elapsed, fname = time_call(sentai.mic.save_mp3)
        save_times.append(elapsed)
        if fname:
            sz = sentai.fs.size(fname)
            mp3_sizes.append(sz if sz > 0 else 0)
        else:
            mp3_sizes.append(0)

        print("  run %d: start=%dms save=%dms file=%s" % (
            i, start_times[-1], save_times[-1], fname or "None"))

    st_start = stats(start_times)
    st_level = stats(level_times)
    st_save = stats(save_times)
    meta = snapshot_meta("E9_mic", seconds=seconds)

    print("[E9] Done.")
    _print_stats("start()", st_start)
    _print_stats("level()", st_level)
    _print_stats("save_mp3()", st_save)

    result = {"experiment": "E9_mic", "meta": meta,
              "params": {"seconds": seconds, "repetitions": repetitions},
              "samples": {"start_ms": start_times, "level_ms": level_times,
                          "save_ms": save_times, "mp3_bytes": mp3_sizes},
              "summary": {"start": st_start, "level": st_level, "save": st_save}}

    if save:
        path = _save_path("e9_mic_%ds" % seconds)
        save_csv(path, ["run", "start_ms", "level_ms", "save_ms", "mp3_bytes"],
                 [(i, start_times[i], level_times[i], save_times[i], mp3_sizes[i])
                  for i in range(repetitions)])
        _record("e9_mic", path, "%ds start=%.1f save=%.1f ms" % (seconds, st_start["mean"], st_save["mean"]))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E10 — Memory-state snapshot
# ─────────────────────────────────────────────────────────

def e10_memory(scenario="idle", save=True):
    """Collect heap snapshot under a labeled scenario."""
    print("[E10] Memory snapshot — '%s'" % scenario)

    heap = snapshot_heap()
    tasks = snapshot_tasks()
    meta = snapshot_meta("E10_memory", scenario=scenario)

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
        _record("e10_memory", path, "%s rtos_free=%d" % (scenario, heap["rtos_free"]))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E11 — CPU/task-state
# ─────────────────────────────────────────────────────────

def e11_cpu(scenario="idle", duration_ms=2000, save=True):
    """Collect CPU usage and task info over a measurement window."""
    print("[E11] CPU/task state — '%s' for %dms" % (scenario, duration_ms))

    cpu_before = snapshot_cpu()
    tasks_snap = snapshot_tasks()

    # Let the system run for duration_ms to accumulate runtime counters
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
        _record("e11_cpu", path, "%s %d tasks" % (scenario, len(tasks_snap)))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# E12 — End-to-end live loop
# ─────────────────────────────────────────────────────────

def e12_live_loop(model_path, camera_id=0, width=320, height=320,
                  repetitions=50, save=True):
    """Measure full perception loop: to_tensor + invoke + value read."""
    print("[E12] Live loop — cam%d %dx%d, model=%s, %d reps" % (
        camera_id, width, height, model_path, repetitions))

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    # Warm-up
    sentai.camera.to_tensor()
    sentai.tpu.invoke()

    tensor_times = []
    invoke_times = []
    loop_times = []

    for i in range(repetitions):
        t_loop = _ticks()

        t0 = _ticks()
        sentai.camera.to_tensor()
        t1 = _ticks()
        tensor_times.append(t1 - t0)

        inv_ms = sentai.tpu.invoke()
        invoke_times.append(inv_ms)

        # Read first output to complete the loop
        sentai.tpu.output(0)

        loop_times.append(_ticks() - t_loop)

        if (i + 1) % 25 == 0:
            print("  ... %d/%d" % (i + 1, repetitions))

    st_tensor = stats(tensor_times)
    st_invoke = stats(invoke_times)
    st_loop = stats(loop_times)
    fps = 1000.0 / st_loop["mean"] if st_loop["mean"] > 0 else 0
    meta = snapshot_meta("E12_live_loop", camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         model_path=model_path)

    print("[E12] Done.")
    _print_stats("to_tensor", st_tensor)
    _print_stats("invoke", st_invoke)
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
        _record("e12_live_loop", path, "cam%d %dx%d loop=%.1f ms fps=%.1f" % (camera_id, width, height, st_loop["mean"], fps))
        print("  Saved: %s" % path)

    return result

# ─────────────────────────────────────────────────────────
# Batch runners
# ─────────────────────────────────────────────────────────

def run_all_quick(model_path, camera_id=0):
    """Run all experiments with minimal repetitions for a quick check."""
    begin("quick_cam%d" % camera_id)

    results = {}
    results["e1"] = e1_tpu_invoke(model_path, use_camera=True, repetitions=10)
    results["e2"] = e2_tpu_load(model_path, repetitions=3)
    results["e3"] = e3_camera_tensor(camera_id, repetitions=10)
    results["e4"] = e4_jpeg(camera_id, repetitions=10)
    results["e5"] = e5_camera_switch(repetitions=5)
    results["e8"] = e8_imu(repetitions=20)
    results["e10"] = e10_memory("quick_all")
    results["e11"] = e11_cpu("quick_all", duration_ms=1000)
    results["e12"] = e12_live_loop(model_path, camera_id, repetitions=10)

    end()
    return results

def run_ablation_cameras(model_path, repetitions=30):
    """Run E3, E4, E12 on both cameras for comparison."""
    begin("ablation_cameras")

    results = {}
    for cam in [0, 1]:
        results["e3_cam%d" % cam] = e3_camera_tensor(cam, repetitions=repetitions)
        results["e4_cam%d" % cam] = e4_jpeg(cam, repetitions=repetitions)
        results["e12_cam%d" % cam] = e12_live_loop(model_path, cam,
                                                     repetitions=repetitions)

    end()
    return results

def run_ablation_resolutions(model_path, camera_id=0, repetitions=20):
    """Run E3 and E12 across multiple resolutions."""
    begin("ablation_res_cam%d" % camera_id)

    resolutions = [(320, 240), (320, 320), (640, 480)]
    results = {}
    for w, h in resolutions:
        tag = "%dx%d" % (w, h)
        results["e3_%s" % tag] = e3_camera_tensor(camera_id, w, h,
                                                    repetitions=repetitions)
        results["e12_%s" % tag] = e12_live_loop(model_path, camera_id, w, h,
                                                  repetitions=repetitions)

    end()
    return results

# ─────────────────────────────────────────────────────────
print("diag module loaded. Usage: diag.e1_tpu_invoke('/models/yolo.tflite')")
