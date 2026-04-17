# diag/e_camera.py — E3: camera tensor, E4: JPEG, E5/E5b/E5c: camera switch

import sentai
from diag._util import (stats, time_call, save_csv, snapshot_meta,
                         _ticks, _print_stats)
from diag._session import _save_path, _record


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

    if not sentai.tpu.ready():
        sentai.tpu.load("/yolo26n.edgetpu_1.tflite")
        print("  [E3] loaded model for to_tensor")

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.rtos.sleep_ms(200)
    sentai.camera.to_tensor()

    # --- Method 1: bulk to_tensor ---
    n_bulk = max(repetitions, 100)
    t_start = _ticks()
    for _ in range(n_bulk):
        sentai.camera.to_tensor()
    t_end = _ticks()
    bulk_total_ms = t_end - t_start
    bulk_per_call = bulk_total_ms / n_bulk if n_bulk > 0 else 0
    print("  to_tensor bulk: %d calls in %d ms = %.2f ms/call (PXP+memcpy)" % (
        n_bulk, bulk_total_ms, bulk_per_call))

    # --- Method 2: sensor FPS ---
    n_fps = min(repetitions, 30)
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

    # --- Method 3: wait + to_tensor ---
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
        elapsed, data = time_call(sentai.camera.jpeg, quality)
        jpeg_times.append(elapsed)
        jpeg_sizes.append(len(data))

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
        _record("e4_jpeg", path,
                "cam%d q=%d jpeg=%.1f save=%.1f ms" % (
                    camera_id, quality, st_jpeg["mean"], st_save["mean"]))
        print("  Saved: %s" % path)

    sentai.fs.remove("/diags/_e4_tmp.jpg")
    return result


def e5_camera_switch(from_cam=0, to_cam=1, repetitions=20, save=True):
    """Measure camera switch round-trip: select(to) + drain N frames + select(back)."""
    print("[E5] Camera switch %d->%d, %d reps" % (from_cam, to_cam, repetitions))

    if not sentai.tpu.ready():
        sentai.tpu.load("/yolo26n.edgetpu_1.tflite")
        print("  [E5] loaded model for to_tensor")

    sentai.camera.init(1)
    sentai.camera.select(from_cam)
    for _ in range(5):
        sentai.camera.to_tensor()

    switch_times = []
    first_fresh_times = []
    roundtrip_times = []

    for i in range(repetitions):
        sentai.camera.select(from_cam)
        sentai.camera.to_tensor()
        sentai.rtos.sleep_ms(50)

        t_all = _ticks()

        t0 = _ticks()
        sentai.camera.select(to_cam)
        switch_times.append(_ticks() - t0)

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


def e5b_alternating(model_path, cam_nadir=0, cam_forward=1,
                    width=320, height=320, cycles=20, save=True):
    """Alternate between nadir and forward cameras every frame (1:1 duty cycle)."""
    print("[E5b] Alternating 1:1 — cam%d/cam%d, %d cycles" % (
        cam_nadir, cam_forward, cycles))

    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

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
        t0 = _ticks(); sentai.camera.select(cam_nadir); nadir_switch.append(_ticks() - t0)
        t0 = _ticks(); sentai.camera.to_tensor();        nadir_tensor.append(_ticks() - t0)
        nadir_invoke.append(sentai.tpu.invoke())

        t0 = _ticks(); sentai.camera.select(cam_forward); fwd_switch.append(_ticks() - t0)
        t0 = _ticks(); sentai.camera.to_tensor();          fwd_tensor.append(_ticks() - t0)
        fwd_invoke.append(sentai.tpu.invoke())

        if (i + 1) % 10 == 0:
            print("  ... %d/%d cycles" % (i + 1, cycles))

    st_ns = stats(nadir_switch);  st_nt = stats(nadir_tensor);  st_ni = stats(nadir_invoke)
    st_fs = stats(fwd_switch);    st_ft = stats(fwd_tensor);    st_fi = stats(fwd_invoke)

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
    _print_stats("  switch", st_ns); _print_stats("  tensor", st_nt); _print_stats("  invoke", st_ni)
    print("  Forward (cam%d):" % cam_forward)
    _print_stats("  switch", st_fs); _print_stats("  tensor", st_ft); _print_stats("  invoke", st_fi)
    print("  Cycle: mean=%.1f ms  fps=%.1f (both cameras)" % (st_cycle["mean"], fps))

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
                  "nadir_switch": st_ns, "nadir_tensor": st_nt, "nadir_invoke": st_ni,
                  "fwd_switch": st_fs, "fwd_tensor": st_ft, "fwd_invoke": st_fi,
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


def e5c_asymmetric(model_path, cam_nadir=0, cam_forward=1,
                   width=320, height=320,
                   fwd_every_n=5, total_nadir_frames=50, save=True):
    """Nadir continuous, forward interleaved every N nadir frames.
    fwd_every_n=0 → nadir-only baseline (no switching).
    """
    fwd_hz_approx = "~%.0f Hz" % (10.0 / fwd_every_n) if fwd_every_n > 0 else "off"
    print("[E5c] Asymmetric — nadir=cam%d continuous, fwd=cam%d every %d frames (%s)" % (
        cam_nadir, cam_forward, fwd_every_n, fwd_hz_approx))
    print("  %d total nadir frames, model=%s" % (total_nadir_frames, model_path))

    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    sentai.camera.select(cam_nadir)
    sentai.camera.to_tensor()
    sentai.tpu.invoke()

    frame_type = []
    frame_total_ms = []
    nadir_tensor_ms = []
    nadir_invoke_ms = []
    fwd_switch_to_ms = []
    fwd_tensor_ms = []
    fwd_invoke_ms = []
    fwd_switch_back_ms = []
    fwd_roundtrip_ms = []

    for i in range(total_nadir_frames):
        t_frame = _ticks()

        t0 = _ticks(); sentai.camera.to_tensor(); nt = _ticks() - t0
        inv = sentai.tpu.invoke()
        nadir_tensor_ms.append(nt)
        nadir_invoke_ms.append(inv)

        if fwd_every_n > 0 and (i + 1) % fwd_every_n == 0:
            t_rt = _ticks()
            t0 = _ticks(); sentai.camera.select(cam_forward); sw_to = _ticks() - t0
            t0 = _ticks(); sentai.camera.to_tensor();          ft = _ticks() - t0
            fi = sentai.tpu.invoke()
            t0 = _ticks(); sentai.camera.select(cam_nadir);    sw_back = _ticks() - t0
            rt = _ticks() - t_rt

            fwd_switch_to_ms.append(sw_to)
            fwd_tensor_ms.append(ft)
            fwd_invoke_ms.append(fi)
            fwd_switch_back_ms.append(sw_back)
            fwd_roundtrip_ms.append(rt)
            frame_type.append("fwd_round")
        else:
            frame_type.append("nadir")

        frame_total_ms.append(_ticks() - t_frame)

        if (i + 1) % 25 == 0:
            print("  ... %d/%d nadir frames" % (i + 1, total_nadir_frames))

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
    print("  Nadir (cam%d) — %d frames:" % (cam_nadir, total_nadir_frames))
    _print_stats("  to_tensor", st_nt); _print_stats("  invoke", st_ni)
    _print_stats("  nadir-only frame", st_nadir_only)
    print("  nadir-only FPS: %.1f" % nadir_fps)
    if fwd_count > 0:
        print("  Forward (cam%d) — %d round-trips:" % (cam_forward, fwd_count))
        _print_stats("  switch_to", st_fwd_sw_to)
        _print_stats("  switch_back", st_fwd_sw_back)
        _print_stats("  roundtrip", st_fwd_rt)
        _print_stats("  frame w/ fwd", st_fwd_frame)
    print("  Effective FPS (all frames): %.1f" % effective_fps)
    pen = (1.0 - effective_fps / nadir_fps) * 100 if nadir_fps > 0 else 0
    print("  FPS penalty from fwd: %.1f%%" % pen)

    result = {"experiment": "E5c_asymmetric", "meta": meta,
              "params": {"cam_nadir": cam_nadir, "cam_forward": cam_forward,
                         "fwd_every_n": fwd_every_n,
                         "total_nadir_frames": total_nadir_frames,
                         "width": width, "height": height,
                         "model_path": model_path},
              "samples": {
                  "frame_type": frame_type, "frame_total_ms": frame_total_ms,
                  "nadir_tensor_ms": nadir_tensor_ms, "nadir_invoke_ms": nadir_invoke_ms,
                  "fwd_switch_to_ms": fwd_switch_to_ms, "fwd_tensor_ms": fwd_tensor_ms,
                  "fwd_invoke_ms": fwd_invoke_ms,
                  "fwd_switch_back_ms": fwd_switch_back_ms,
                  "fwd_roundtrip_ms": fwd_roundtrip_ms},
              "summary": {
                  "nadir_tensor": st_nt, "nadir_invoke": st_ni,
                  "nadir_only_frame": st_nadir_only,
                  "fwd_roundtrip": st_fwd_rt, "fwd_frame": st_fwd_frame,
                  "all_frames": st_all_frames,
                  "nadir_fps": round(nadir_fps, 1),
                  "effective_fps": round(effective_fps, 1),
                  "fwd_count": fwd_count}}

    if save:
        path = _save_path("e5c_asym_1to%d" % fwd_every_n)
        rows = []
        fi = 0
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
                    fwd_every_n, nadir_fps, effective_fps, pen))
        print("  Saved: %s" % path)

    return result


def e5c_sweep(model_path, cam_nadir=0, cam_forward=1,
              ratios=None, total_nadir_frames=50, save=True):
    """Run E5c across multiple forward-camera duty cycles.
    Default ratios: [0, 2, 5, 10, 20].  ratio=0 = nadir-only baseline.
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
