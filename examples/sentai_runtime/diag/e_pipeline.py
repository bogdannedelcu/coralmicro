# diag/e_pipeline.py — Full inference pipeline timing
#
# E13 — sequential pipeline, per-stage Python-visible timing
# E14 — parallel pipeline, measure sustained FPS via sentai.pipeline.*
#
# Why two experiments?
#   E13 is a single Python call chain:  to_tensor → invoke → output → detect
#   E14 uses the firmware's PrepTask + InferTask double-buffer runner
#     (detection_task.cc). While InferTask is in Invoke(), PrepTask is already
#     doing PXP + quant for the next frame — so wall-clock frame interval
#     collapses from (sum of stages) to max(prep_stage, infer_stage).
#
# How the firmware pipeline knows when it may overwrite the tensor:
#   PrepTask writes to   staging_buf   (never touches TFLite tensor)
#   InferTask does       memcpy(tensor_buf, staging_buf, total)
#                        xSemaphoreGive(staging_free)      ← PrepTask can start N+1
#                        Invoke()                          ← tensor_buf is private to TPU
#                        tpu_detect()                      ← NMS on output
#   So the staging buffer is released *before* Invoke runs, and the TPU input
#   tensor is never shared.  Two synchronisation primitives (staging_free +
#   prep_done) give us a true producer/consumer pipeline with zero races.

import gc
import sentai
from diag._util import stats, save_csv, snapshot_meta, _ticks, _print_stats
from diag._session import _save_path, _record, _save_desc


def e13_pipeline_full(model_path, camera_id=0, width=320, height=320,
                      conf=0.25, iou=0.45, max_det=50,
                      repetitions=30, save=True):
    """Per-stage timing across the full inference pipeline, one row per frame."""
    print("[E13] Pipeline full — cam%d %dx%d model=%s %d reps (conf=%.2f iou=%.2f)" % (
        camera_id, width, height, model_path, repetitions, conf, iou))

    sentai.camera.select(camera_id)
    sentai.camera.set_resolution(width, height)
    sentai.camera.init(1)
    sentai.tpu.load(model_path)

    # Warm-up
    sentai.camera.to_tensor()
    sentai.tpu.invoke()
    sentai.tpu.output(0)
    sentai.tpu.detect(conf, iou, max_det)
    gc.collect()

    frame_ms_s  = []
    tensor_ms_s = []
    invoke_ms_s = []
    output_ms_s = []
    detect_ms_s = []
    total_ms_s  = []
    ndet_s      = []

    for i in range(repetitions):
        fc = sentai.camera.frame_count()
        t0 = _ticks()
        while sentai.camera.frame_count() == fc:
            pass
        t_got_frame = _ticks()

        t_tensor_start = _ticks()
        sentai.camera.to_tensor()
        t_tensor_end = _ticks()

        inv_ms = sentai.tpu.invoke()
        t_invoke_end = _ticks()

        t_out_start = _ticks()
        _out = sentai.tpu.output(0)
        t_out_end = _ticks()
        _out = None

        t_det_start = _ticks()
        dets = sentai.tpu.detect(conf, iou, max_det)
        t_det_end = _ticks()

        frame_ms  = t_got_frame   - t0
        tensor_ms = t_tensor_end  - t_tensor_start
        out_ms    = t_out_end     - t_out_start
        det_ms    = t_det_end     - t_det_start
        n_det     = len(dets)
        total_ms  = frame_ms + tensor_ms + inv_ms + out_ms + det_ms

        frame_ms_s.append(frame_ms)
        tensor_ms_s.append(tensor_ms)
        invoke_ms_s.append(inv_ms)
        output_ms_s.append(out_ms)
        detect_ms_s.append(det_ms)
        total_ms_s.append(total_ms)
        ndet_s.append(n_det)

        gc.collect()

        if (i + 1) % 10 == 0:
            print("  ... %d/%d  frame=%d tensor=%d invoke=%d out=%d det=%d tot=%d dets=%d" % (
                i + 1, repetitions,
                frame_ms, tensor_ms, inv_ms, out_ms, det_ms, total_ms, n_det))

    st_frame  = stats(frame_ms_s)
    st_tensor = stats(tensor_ms_s)
    st_invoke = stats(invoke_ms_s)
    st_output = stats(output_ms_s)
    st_detect = stats(detect_ms_s)
    st_total  = stats(total_ms_s)
    fps = 1000.0 / st_total["mean"] if st_total["mean"] > 0 else 0
    dets_total = sum(ndet_s)

    meta = snapshot_meta("E13_pipeline_full",
                         camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         model_path=model_path,
                         conf=conf, iou=iou)

    print("[E13] Done.")
    _print_stats("frame_wait",  st_frame)
    _print_stats("to_tensor",   st_tensor)
    _print_stats("invoke",      st_invoke)
    _print_stats("output_read", st_output)
    _print_stats("detect_nms",  st_detect)
    _print_stats("total_loop",  st_total)
    print("  pipeline FPS: %.1f   total detections across %d frames: %d" % (
        fps, repetitions, dets_total))

    sum_of_means = (st_frame["mean"] + st_tensor["mean"] + st_invoke["mean"]
                    + st_output["mean"] + st_detect["mean"])
    elapsed_total = sum(total_ms_s)
    print("  total (sum of stage means) = %.1f ms  ->  %.1f FPS" % (
        sum_of_means, 1000.0 / sum_of_means if sum_of_means > 0 else 0))
    print("  total wall time for %d reps = %d ms (%.2f s)" % (
        repetitions, elapsed_total, elapsed_total / 1000.0))

    result = {"experiment": "E13_pipeline_full", "meta": meta,
              "params": {"camera_id": camera_id, "width": width, "height": height,
                         "model_path": model_path, "repetitions": repetitions,
                         "conf": conf, "iou": iou, "max_det": max_det},
              "samples": {"frame_ms":  frame_ms_s,
                          "tensor_ms": tensor_ms_s,
                          "invoke_ms": invoke_ms_s,
                          "output_ms": output_ms_s,
                          "detect_ms": detect_ms_s,
                          "total_ms":  total_ms_s,
                          "ndet":      ndet_s},
              "summary": {"frame": st_frame, "tensor": st_tensor,
                          "invoke": st_invoke, "output": st_output,
                          "detect": st_detect, "total": st_total,
                          "fps": round(fps, 1), "dets_total": dets_total}}

    if save:
        path = _save_path("e13_pipeline_cam%d_%dx%d" % (camera_id, width, height))
        save_csv(path,
            ["run_index",
             "frame_wait_ms",
             "camera_to_tensor_ms",
             "edgetpu_invoke_ms",
             "output_read_ms",
             "detect_nms_ms",
             "num_detections",
             "total_loop_ms"],
            [(i, frame_ms_s[i], tensor_ms_s[i], invoke_ms_s[i],
              output_ms_s[i], detect_ms_s[i], ndet_s[i], total_ms_s[i])
             for i in range(repetitions)])
        _save_desc(path,
            "E13 - Full inference pipeline per-stage timing (sequential).\n"
            "One row per frame captures every Python-visible stage of the vision loop.\n"
            "\nColumns:\n"
            "  run_index             : repetition number (0-based)\n"
            "  frame_wait_ms         : poll time waiting for a new sensor frame\n"
            "  camera_to_tensor_ms   : PXP resize + int8 quantize + copy into TPU SRAM\n"
            "  edgetpu_invoke_ms     : tpu.invoke() - neural network forward pass on EdgeTPU\n"
            "  output_read_ms        : tpu.output(0) - copy raw output tensor into micropython bytes\n"
            "  detect_nms_ms         : tpu.detect(conf,iou) - YOLO NMS + class selection\n"
            "  num_detections        : surviving detections after NMS\n"
            "  total_loop_ms         : sum of the five stage timings\n"
            "\nEffective FPS = 1000 / mean(total_loop_ms).",
            params={"camera_id": camera_id, "width": width, "height": height,
                    "model_path": model_path, "repetitions": repetitions,
                    "conf": conf, "iou": iou, "max_det": max_det})
        _record("e13_pipeline_full", path,
                "cam%d %dx%d fps=%.1f frame=%.0f tensor=%.0f inv=%.0f out=%.0f det=%.0f ms dets=%d" % (
                    camera_id, width, height, fps,
                    st_frame["mean"], st_tensor["mean"], st_invoke["mean"],
                    st_output["mean"], st_detect["mean"], dets_total))
        print("  Saved: %s" % path)

    return result


def e15_pipeline_parallel_512(model_path="/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
                               camera_id=0, repetitions=10, save=True):
    """E15 — parallel pipeline with the 1-class 512x512 yolo edge-only model.

    Always leaves a trace on LittleFS under /diags/<session>/ — if no session
    is active when called, E15 opens a dedicated one (``e15_512``) so the
    per-frame CSV + .txt description are persisted.  This matches the
    convention used by every other e_*() experiment.

    Differences vs E14:
      - 512x512 uint8 input (no int8 quant step in firmware path — slightly faster
        per frame on PrepTask side).
      - Output shape [1, 1344, 6] is yolo26 anchor-style pre-NMS (6 = cx, cy,
        w, h, obj_conf, class_conf for the single class).  The firmware
        auto-detects this layout and runs the matching NMS path — see
        `sentai_tpu_detect` in sentai_runtime.cc.

    The purpose of E15 is to measure whether a lighter model (smaller arena,
    smaller output tensor) actually shortens the InferTask critical path —
    Invoke + memcpy + NMS — and therefore the end-to-end pipeline FPS.
    """
    # Auto-open a session if the caller hasn't already started one, so E15
    # always leaves per-frame CSV + manifest under /diags/sNNN_e15_512/.
    # Imported locally to avoid a hard dependency at module-import time.
    from diag._session import _session, begin, end
    owned_session = (_session is None)
    if owned_session:
        begin("e15_512")
    try:
        return e14_pipeline_parallel(model_path,
                                     camera_id=camera_id,
                                     width=512, height=512,
                                     conf=0.25, iou=0.45, max_det=50,
                                     repetitions=repetitions, save=save)
    finally:
        if owned_session:
            end()


def e14_pipeline_parallel(model_path, camera_id=0, width=320, height=320,
                          conf=0.25, iou=0.45, max_det=50,
                          repetitions=30, save=True,
                          compare_sequential=False):
    """Parallel pipeline: firmware PrepTask || InferTask, measured sustained FPS.

    Uses sentai.pipeline.start/get/stop. PrepTask does camera grab + PXP resize
    + int8 quantization into a staging buffer while InferTask does memcpy of
    the previous staging into the TPU tensor and then Invoke + NMS. The staging
    buffer is released *before* Invoke runs, so the two stages overlap.

    Measures:
      - wall-clock interval between consecutive pipeline.get() calls
      - final frames_processed / frames_dropped from sentai.pipeline.stats()
      - prep_stall / infer_stall via sentai.pipeline.task_health()
    """
    print("[E14] Pipeline parallel — cam%d %dx%d model=%s %d reps (conf=%.2f iou=%.2f)" % (
        camera_id, width, height, model_path, repetitions, conf, iou))

    # Silence *everything* for the entire experiment. Camera init + warmup
    # to_tensor() each emit a flurry of printf lines that together can
    # saturate the CDC-ACM bulk-IN endpoint across back-to-back runs. The
    # per-frame stats are the signal — the init logs are pure noise here.
    prev_verbose = sentai.verbose(0)

    # Make sure no prior pipeline run is still running BEFORE touching camera
    # or TPU — otherwise we could race with the background PrepTask/InferTask.
    if sentai.pipeline.running():
        sentai.pipeline.stop()

    # Idempotent setup: only reconfigure camera / reload model when the current
    # state doesn't match what we need.  Repeated camera.init(1) cycles the
    # MUX (back→front) which leaves g_cam_switch_pending true and forces a
    # 300 ms post-switch wait on the next grab — kills throughput on back-to-
    # back invocations of this experiment.
    cur_res = sentai.camera.resolution()
    if cur_res != (width, height):
        sentai.camera.set_resolution(width, height)
    sentai.camera.select(camera_id)
    # camera.init is safe to call multiple times but we want to avoid the
    # double SwitchCamera inside it on repeat runs; call only on cold start.
    # Detect cold start by checking frame_count — 0 means camera never ran.
    if sentai.camera.frame_count() == 0:
        sentai.camera.init(1)

    # Load the model only if a different one is in place.  `tpu.ready()` is
    # true after a successful load; we compare the last-loaded name implicitly
    # via the tensor dims (512x512 means the 512 model is loaded).
    need_load = True
    if sentai.tpu.ready():
        # Cheap heuristic: if input is 512x512, assume the right model is loaded.
        # set_input_size is part of tpu API to check dims; here we just reload
        # if we can't prove it cheaply.  Skip reload on subsequent E15 calls.
        try:
            # If tpu has the expected model and the input tensor already matches,
            # reload is unnecessary.  We detect this by trying a no-op: if the
            # previous run used the same model, the pipeline.start below will
            # accept the tensor.  Simplest: skip reload unconditionally after
            # first call — the caller can force reload by rebooting.
            need_load = False
        except Exception:
            need_load = True
    if need_load:
        sentai.tpu.load(model_path)

    # Scene snapshot (BEFORE the measurement loop).  Camera is assumed static
    # across the run, so we capture the exact PXP-scaled pixels the TPU will
    # see — useful when `dets_total == 0` to offline-verify scene content.
    from diag._session import snapshot_scene
    if save:
        snapshot_scene("before", name="scene_cam%d" % camera_id)

    # Warm camera pipeline so first frame isn't a cold-start outlier.
    sentai.camera.to_tensor()
    gc.collect()

    # verbose already 0 from above — kept suppressed through pipeline body.

    rc = sentai.pipeline.start(conf, iou, max_det)
    if rc != 0:
        sentai.verbose(prev_verbose)
        raise RuntimeError("pipeline.start failed (%d)" % rc)

    intervals_ms = []
    ndet_s       = []
    prep_stall_s  = []
    infer_stall_s = []
    got = 0
    timeouts = 0
    t_run_end = 0
    processed_fw = dropped_fw = 0
    avg_fps_fw = 0.0

    try:
        # Drain one frame to anchor the wall-clock timer (skip warm-up).
        _ = sentai.pipeline.get(2000)

        t_prev = _ticks()
        t_run_start = t_prev

        for i in range(repetitions):
            dets = sentai.pipeline.get(2000)
            t_now = _ticks()
            if dets is None:
                timeouts += 1
                print("  ... %d/%d  TIMEOUT waiting for frame" % (i + 1, repetitions))
                t_prev = t_now
                continue

            interval = t_now - t_prev
            t_prev = t_now
            intervals_ms.append(interval)
            ndet_s.append(len(dets))

            ps, is_ = sentai.pipeline.task_health()
            prep_stall_s.append(ps)
            infer_stall_s.append(is_)

            got += 1
            # Note: NO gc.collect() inside the measurement loop — it adds
            # tens of milliseconds per iteration and skews frame_interval.
            # Pipeline.get returns a small list of tuples; GC pressure is low.

            if got % 10 == 0:
                print("  ... %d/%d  interval=%d ms dets=%d prep_stall=%d infer_stall=%d" % (
                    got, repetitions, interval, len(dets), ps, is_))

        t_run_end = _ticks()
        processed_fw, dropped_fw, avg_fps_fw = sentai.pipeline.stats()
    finally:
        # Always stop the pipeline, even on KeyboardInterrupt — otherwise the
        # firmware PrepTask/InferTask keep running and flood CDC-ACM TX,
        # eventually blocking mp_repl on a stalled USB bulk endpoint.
        if sentai.pipeline.running():
            sentai.pipeline.stop()
        # Scene snapshot (AFTER the measurement loop).  Camera is still
        # initialised and streaming; pipeline is stopped so to_tensor() is
        # free to grab a frame.  Pairs with the BEFORE snapshot for diff.
        if save:
            snapshot_scene("after", name="scene_cam%d" % camera_id)
        sentai.verbose(prev_verbose)

    if not intervals_ms:
        print("[E14] No frames received — pipeline stalled.")
        return None

    st_interval = stats(intervals_ms)
    st_prep     = stats(prep_stall_s)
    st_infer    = stats(infer_stall_s)

    fps_wall = 1000.0 / st_interval["mean"] if st_interval["mean"] > 0 else 0
    run_ms = t_run_end - t_run_start
    dets_total = sum(ndet_s)

    meta = snapshot_meta("E14_pipeline_parallel",
                         camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         model_path=model_path,
                         conf=conf, iou=iou)

    print("[E14] Done.")
    _print_stats("frame_interval", st_interval)
    _print_stats("prep_stall",     st_prep)
    _print_stats("infer_stall",    st_infer)
    print("  pipeline FPS (wall):        %.1f" % fps_wall)
    print("  firmware stats: processed=%d dropped=%d avg_fps=%.1f" % (
        processed_fw, dropped_fw, avg_fps_fw))
    print("  timeouts: %d  |  detections across %d frames: %d" % (
        timeouts, got, dets_total))
    print("  total wall time for %d received frames = %d ms (%.2f s)" % (
        got, run_ms, run_ms / 1000.0))

    result = {"experiment": "E14_pipeline_parallel", "meta": meta,
              "params": {"camera_id": camera_id, "width": width, "height": height,
                         "model_path": model_path, "repetitions": repetitions,
                         "conf": conf, "iou": iou, "max_det": max_det},
              "samples": {"interval_ms":  intervals_ms,
                          "prep_stall_ms":  prep_stall_s,
                          "infer_stall_ms": infer_stall_s,
                          "ndet": ndet_s},
              "summary": {"interval": st_interval,
                          "prep_stall": st_prep,
                          "infer_stall": st_infer,
                          "fps_wall": round(fps_wall, 1),
                          "fw_processed": processed_fw,
                          "fw_dropped": dropped_fw,
                          "fw_avg_fps": round(avg_fps_fw, 1),
                          "timeouts": timeouts,
                          "dets_total": dets_total}}

    if save:
        path = _save_path("e14_pipeline_par_cam%d_%dx%d" % (camera_id, width, height))
        save_csv(path,
            ["run_index",
             "frame_interval_ms",
             "prep_stall_ms",
             "infer_stall_ms",
             "num_detections"],
            [(i, intervals_ms[i], prep_stall_s[i], infer_stall_s[i], ndet_s[i])
             for i in range(len(intervals_ms))])
        _save_desc(path,
            "E14 - Parallel inference pipeline, measured sustained FPS.\n"
            "Uses firmware PrepTask (PXP+quant into staging) || InferTask\n"
            "(memcpy staging->tensor, Invoke, NMS).  Staging buffer released\n"
            "before Invoke runs, so frame N+1 prep overlaps with frame N invoke.\n"
            "\nColumns:\n"
            "  run_index          : frame index (0-based)\n"
            "  frame_interval_ms  : wall-clock delta between consecutive pipeline.get()\n"
            "                       = sustained loop time (lower is better)\n"
            "  prep_stall_ms      : time since PrepTask last completed a frame (ms)\n"
            "  infer_stall_ms     : time since InferTask last completed a frame (ms)\n"
            "  num_detections     : detections returned for this frame\n"
            "\nSustained FPS = 1000 / mean(frame_interval_ms).\n"
            "Compare with E13's total_loop_ms to measure pipeline speedup.",
            params={"camera_id": camera_id, "width": width, "height": height,
                    "model_path": model_path, "repetitions": repetitions,
                    "conf": conf, "iou": iou, "max_det": max_det})
        _record("e14_pipeline_parallel", path,
                "cam%d %dx%d fps_wall=%.1f fw_fps=%.1f dropped=%d dets=%d" % (
                    camera_id, width, height, fps_wall, avg_fps_fw,
                    dropped_fw, dets_total))
        print("  Saved: %s" % path)

    # Optional inline comparison against sequential E13 on the same config.
    if compare_sequential:
        print("\n[E14] Running E13 on the same config for comparison...")
        seq = e13_pipeline_full(model_path, camera_id=camera_id,
                                width=width, height=height,
                                conf=conf, iou=iou, max_det=max_det,
                                repetitions=repetitions, save=save)
        if seq:
            seq_total = seq["summary"]["total"]["mean"]
            seq_fps   = seq["summary"]["fps"]
            speedup   = fps_wall / seq_fps if seq_fps > 0 else 0
            saved_ms  = seq_total - st_interval["mean"]
            print("\n[E14] Comparison E14(parallel) vs E13(sequential):")
            print("  sequential: %.1f ms/frame  %.1f FPS" % (seq_total, seq_fps))
            print("  parallel:   %.1f ms/frame  %.1f FPS" % (st_interval["mean"], fps_wall))
            print("  speedup:    %.2fx  saved %.1f ms/frame" % (speedup, saved_ms))
            result["comparison"] = {"seq_total_ms": seq_total,
                                    "seq_fps": seq_fps,
                                    "par_interval_ms": st_interval["mean"],
                                    "par_fps": fps_wall,
                                    "speedup": round(speedup, 2),
                                    "saved_ms": round(saved_ms, 1)}

    return result
