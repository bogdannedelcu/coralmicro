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
from diag._util import (stats, save_csv, snapshot_meta, _ticks,
                        _print_stats, _ensure_model_loaded)
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

    # BEFORE snapshots — both cameras.  Sequential experiment, so nothing
    # else owns the MUX and we are free to flip it.  Restores `camera_id`.
    from diag._session import snapshot_both_cameras
    if save:
        snapshot_both_cameras("before")
        sentai.camera.select(camera_id)
        sentai.camera.to_tensor()  # drain stale frames from the restore switch

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

    # AFTER snapshots — both cameras.  Pairs with BEFORE for offline diff.
    if save:
        snapshot_both_cameras("after")
        try:
            sentai.camera.select(camera_id)
        except Exception:
            pass

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
      - Output shape [1, 1344, 6] is YOLOv5-enhanced pre-NMS anchor format
        (6 = cx, cy, w, h, obj_conf, class_conf for the single class).  The
        firmware auto-detects this layout and runs the matching NMS path —
        see `sentai_tpu_detect` in sentai_runtime.cc.

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
                                     repetitions=repetitions, save=save,
                                     _experiment_tag="E15")
    finally:
        if owned_session:
            end()


def e16_camera_switch_512(
        model_path="/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
        cam_a=0, cam_b=1, repetitions=40, save=True,
        resolution=(512, 512)):
    """E16 — alternating cam_a/cam_b per frame, sequential pipeline.

    `resolution=(w,h)` picks the native sensor output size — default
    512×512 for the TPU model.  Pass (640,480) for VGA or (320,240) for
    QVGA to quantify how sensor resolution trades against per-frame PXP/
    JPEG cost.  Both cameras get the same resolution (the CSI-2 receiver
    is shared).

    Measures the latency cost of a camera MUX switch on the SentAI board.
    Each iteration:
      1. `sentai.camera.select(cam)`  — MUX flip + g_cam_switch_pending
      2. `sentai.camera.to_tensor()`  — grab raw + PXP + quant into TPU input
      3. `sentai.tpu.invoke()`        — EdgeTPU inference
      4. `sentai.tpu.detect(conf,iou)` — YOLO NMS (auto-detects v5_like layout)

    Camera is toggled between `cam_a` and `cam_b` on EACH call, so every
    frame pays the post-switch overhead (`sentai_cam_get_raw_with_recovery`
    hits the slow path: drain stale queued frames then wait for ≥ 2 fresh
    ISR frames from the new sensor).  Compare against E13 (fixed camera,
    sequential) or E15 (fixed camera, pipeline) to read off the per-switch
    cost.

    Sequential (no `sentai.pipeline.start`) because the pipeline owns the
    camera MUX during a run — mid-pipeline `camera.select` would race with
    PrepTask's `cam_grab_latest` and blow up determinism.

    Always leaves a trace under /diags/<session>/ with per-frame CSV that
    includes the camera id so the operator can align timing spikes with
    the switch direction (cam_a→cam_b vs cam_b→cam_a) offline.
    """
    from diag._session import _session, begin, end, snapshot_both_cameras

    owned_session = (_session is None)
    if owned_session:
        begin("e16_camswitch_512")

    # Keep the noisy firmware per-frame prints off — see memcpy.md for why
    # verbose=1 during a measurement loop saturates CDC-ACM and masks host.
    prev_verbose = sentai.verbose(0)

    # Defensive: if a prior E14/E15 left PrepTask running, mid-loop
    # camera.select would race with cam_grab_latest.  Stop it first.
    if sentai.pipeline.running():
        sentai.pipeline.stop()

    # Setup: both cameras powered, model loaded, streaming running.
    w, h = resolution
    sentai.camera.set_resolution(w, h)
    if sentai.camera.frame_count() == 0:
        sentai.camera.init(1)
    _ensure_model_loaded(model_path)

    # One full warmup on each camera so the first measured frame isn't a
    # cold-start outlier — we want the steady-state switch cost, not the
    # first-ever-switch cost.
    for c in (cam_a, cam_b):
        sentai.camera.select(c)
        sentai.camera.to_tensor()
        sentai.tpu.invoke()

    # BEFORE snapshot — shoot both cameras so the session dir documents
    # exactly what each sensor saw just before the switch loop began.
    if save:
        snapshot_both_cameras("before")
    # End on cam_a so the loop starts cleanly on a known sensor.
    sentai.camera.select(cam_a)
    sentai.camera.to_tensor()  # drain stale frames from the restore switch

    cam_ids = []
    sel_ms  = []
    ten_ms  = []
    inv_ms  = []
    det_ms  = []
    ndet_s  = []
    tot_ms  = []

    try:
        for i in range(repetitions):
            cam = cam_a if (i % 2 == 0) else cam_b
            t_start = _ticks()

            t0 = _ticks(); sentai.camera.select(cam);   sel  = _ticks() - t0
            t0 = _ticks(); sentai.camera.to_tensor();   tens = _ticks() - t0
            inv = sentai.tpu.invoke()                   # returns ms or <0 on err
            t0 = _ticks(); dets = sentai.tpu.detect(0.25, 0.45, 50); det = _ticks() - t0

            tot = _ticks() - t_start

            cam_ids.append(cam)
            sel_ms.append(sel)
            ten_ms.append(tens)
            inv_ms.append(inv if inv >= 0 else 0)
            det_ms.append(det)
            ndet_s.append(len(dets) if dets else 0)
            tot_ms.append(tot)
    finally:
        # AFTER snapshot — both cameras.  Pairs with BEFORE for offline diff.
        if save:
            snapshot_both_cameras("after")
        sentai.verbose(prev_verbose)

    # ---- aggregate ---------------------------------------------------
    st_sel  = stats(sel_ms)
    st_ten  = stats(ten_ms)
    st_inv  = stats(inv_ms)
    st_det  = stats(det_ms)
    st_tot  = stats(tot_ms)
    fps     = 1000.0 / st_tot["mean"] if st_tot["mean"] > 0 else 0
    dets_total = sum(ndet_s)

    # Split by camera for asymmetric scenes (one camera covered, etc.)
    a_idx = [i for i, c in enumerate(cam_ids) if c == cam_a]
    b_idx = [i for i, c in enumerate(cam_ids) if c == cam_b]
    def pick(xs, idx): return [xs[i] for i in idx]
    st_tot_a = stats(pick(tot_ms, a_idx)) if a_idx else None
    st_tot_b = stats(pick(tot_ms, b_idx)) if b_idx else None

    meta = snapshot_meta("E16_camera_switch",
                         cam_a=cam_a, cam_b=cam_b,
                         resolution="512x512",
                         model_path=model_path,
                         repetitions=repetitions)

    print("[E16] Done. cam%d<->cam%d, %d frames" % (cam_a, cam_b, repetitions))
    _print_stats("select",    st_sel)
    _print_stats("to_tensor", st_ten)
    _print_stats("invoke",    st_inv)
    _print_stats("detect",    st_det)
    _print_stats("total",     st_tot)
    if st_tot_a: _print_stats("  cam%d total" % cam_a, st_tot_a)
    if st_tot_b: _print_stats("  cam%d total" % cam_b, st_tot_b)
    print("  effective FPS (w/ switch every frame): %.1f" % fps)
    print("  dets total across %d frames: %d" % (repetitions, dets_total))

    result = {
        "experiment": "E16_camera_switch", "meta": meta,
        "params": {"cam_a": cam_a, "cam_b": cam_b,
                   "width": 512, "height": 512,
                   "model_path": model_path, "repetitions": repetitions},
        "samples": {"cam_id": cam_ids, "select_ms": sel_ms,
                    "tensor_ms": ten_ms, "invoke_ms": inv_ms,
                    "detect_ms": det_ms, "total_ms": tot_ms, "ndet": ndet_s},
        "summary": {"select": st_sel, "tensor": st_ten, "invoke": st_inv,
                    "detect": st_det, "total": st_tot, "fps": round(fps, 1),
                    "dets_total": dets_total,
                    "total_cam_a": st_tot_a, "total_cam_b": st_tot_b},
    }

    if save:
        path = _save_path("e16_camswitch_cam%d_cam%d" % (cam_a, cam_b))
        save_csv(path,
            ["run_index", "cam_id",
             "select_ms", "to_tensor_ms", "invoke_ms", "detect_ms",
             "num_detections", "total_frame_ms"],
            [(i, cam_ids[i], sel_ms[i], ten_ms[i], inv_ms[i],
              det_ms[i], ndet_s[i], tot_ms[i]) for i in range(repetitions)])
        _save_desc(path,
            "E16 - Alternating camera switch, sequential pipeline, 512x512.\n"
            "Each frame flips the MUX to the other camera before capturing,\n"
            "so every measurement pays the post-switch drain + first-fresh-\n"
            "frame wait cost.  Compare against E13/E15 on a fixed camera to\n"
            "read the per-switch overhead.\n"
            "\nColumns:\n"
            "  run_index        : frame index (0-based)\n"
            "  cam_id           : which camera produced this frame (cam_a/cam_b)\n"
            "  select_ms        : time to change MUX to the target camera\n"
            "  to_tensor_ms     : grab raw + PXP + quant + copy into TPU input\n"
            "                     (includes drain + fresh-frame wait on switch)\n"
            "  invoke_ms        : EdgeTPU inference time\n"
            "  detect_ms        : YOLO NMS post-processing\n"
            "  num_detections   : surviving detections after NMS\n"
            "  total_frame_ms   : sum of the four stages (full cycle time)",
            params={"cam_a": cam_a, "cam_b": cam_b,
                    "resolution": "512x512",
                    "model_path": model_path, "repetitions": repetitions,
                    "dets_total": dets_total})
        _record("e16_camera_switch", path,
                "cam%d<->cam%d fps=%.1f sel=%.1f ten=%.1f inv=%.1f" % (
                    cam_a, cam_b, fps, st_sel["mean"],
                    st_ten["mean"], st_inv["mean"]))
        print("  Saved: %s" % path)

    if owned_session:
        end()
    return result


def e18_camera_switch_headtail(
        model_path="/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
        cam_a=0, cam_b=1, repetitions=40, save=True,
        resolution=(512, 512)):
    """E18 — Head-to-tail camera-switch benchmark, sequential pipeline.

    `resolution=(w,h)` sets the native sensor size for both cameras
    (shared CSI-2 receiver — see paper/cam_switch.md §"Per-camera
    resolution").  Default 512×512 matches the TPU model; pass (640,480)
    or (320,240) to compare PXP/JPEG cost scaling.

    Runs THREE back-to-back sub-sweeps at the current switch_drain and
    ratio settings (all three use the same model, resolution, scene,
    thermal state — only the camera-selection pattern changes):

        A) fixed cam_a:   no switches, same 4-stage loop, baseline
        B) fixed cam_b:   no switches, baseline for the other sensor
        C) alternating:   cam_a↔cam_b every frame, the actual switch test

    Reports per-stage means for each sweep (select, to_tensor, invoke,
    detect, total) and the derived per-switch overhead as
    `C_total − max(A_total, B_total)`.  The three CSVs are saved into
    the active session so later analysis can re-compute deltas offline.

    Why three sweeps, not just one?  E16 gives only the alternating
    number; without a fixed-camera baseline measured under identical
    conditions it is impossible to separate the "switch cost" from the
    "grab-one-frame cost" in a defensible way.  E18 pays two extra
    baseline sweeps (~3 s each at 30 fps) to make that separation clean.

    Warm-up: pipeline is stopped if running; both cameras are powered
    and warmed with a single `to_tensor` before any measurement starts.
    Scene snapshots are taken before/after across both cameras via
    `snapshot_both_cameras()` just like the other E1x experiments.
    """
    from diag._session import _session, begin, end, snapshot_both_cameras

    owned_session = (_session is None)
    if owned_session:
        begin("e18_cam_switch_headtail")

    prev_verbose = sentai.verbose(0)

    if sentai.pipeline.running():
        sentai.pipeline.stop()

    try:
        w, h = resolution
        sentai.camera.set_resolution(w, h)
        if sentai.camera.frame_count() == 0:
            sentai.camera.init(1)

        _ensure_model_loaded(model_path)

        # One full warm cycle on each camera so the first measurement
        # sample is already in steady state (drain path has run at least
        # once for both sensors).
        for c in (cam_a, cam_b):
            sentai.camera.select(c)
            sentai.camera.to_tensor()
            sentai.tpu.invoke()

        if save:
            snapshot_both_cameras("before")
            sentai.camera.select(cam_a)
            sentai.camera.to_tensor()

        drain = sentai.camera.switch_drain()
        ra, rb = sentai.camera.ratio()
        print("[E18] 512x512, reps=%d per sweep, drain=%d, ratio=(%d,%d)" % (
            repetitions, drain, ra, rb))

        # ------------------------------------------------------------
        # One sub-sweep.  Returns a dict of per-stage lists + summary.
        # Defined inline rather than at module scope because it closes
        # over `repetitions` and the measurement helpers, and is only
        # meaningful inside this experiment.
        # ------------------------------------------------------------
        def _sweep(label, cam_pattern):
            sel_ms = []; ten_ms = []; inv_ms = []; det_ms = []
            tot_ms = []; ndet_s = []; cam_ids = []
            for i in range(repetitions):
                cam = cam_pattern(i)
                t_start = _ticks()
                t0 = _ticks(); sentai.camera.select(cam);   sel  = _ticks() - t0
                t0 = _ticks(); sentai.camera.to_tensor();   tens = _ticks() - t0
                inv = sentai.tpu.invoke()
                t0 = _ticks(); dets = sentai.tpu.detect(0.25, 0.45, 50); det = _ticks() - t0
                tot = _ticks() - t_start
                cam_ids.append(cam); sel_ms.append(sel); ten_ms.append(tens)
                inv_ms.append(inv if inv >= 0 else 0)
                det_ms.append(det); ndet_s.append(len(dets) if dets else 0)
                tot_ms.append(tot)
            # Drop first sample as final warm-up guard — the prior sweep
            # may have left a switch pending.
            samples = (sel_ms[1:], ten_ms[1:], inv_ms[1:], det_ms[1:],
                       tot_ms[1:], ndet_s[1:], cam_ids[1:])
            s_sel, s_ten, s_inv, s_det, s_tot, _, _ = samples
            summary = {"select": stats(s_sel),
                       "to_tensor": stats(s_ten),
                       "invoke": stats(s_inv),
                       "detect": stats(s_det),
                       "total": stats(s_tot)}
            fps = 1000.0 / summary["total"]["mean"] if summary["total"]["mean"] > 0 else 0
            summary["fps"] = round(fps, 2)
            print("[E18:%s] total=%.1fms fps=%.2f  (sel=%.1f ten=%.1f inv=%.1f det=%.1f)" % (
                label, summary["total"]["mean"], fps,
                summary["select"]["mean"], summary["to_tensor"]["mean"],
                summary["invoke"]["mean"], summary["detect"]["mean"]))
            return {"label": label,
                    "samples": {"cam_id": cam_ids, "select_ms": sel_ms,
                                "to_tensor_ms": ten_ms, "invoke_ms": inv_ms,
                                "detect_ms": det_ms, "total_frame_ms": tot_ms,
                                "num_detections": ndet_s},
                    "summary": summary}

        # --- Sweep A: fixed cam_a ---
        sentai.camera.select(cam_a)
        sentai.camera.to_tensor()  # post-switch drain before timing
        a = _sweep("A_fixed_cam%d" % cam_a, lambda i: cam_a)

        # --- Sweep B: fixed cam_b ---
        sentai.camera.select(cam_b)
        sentai.camera.to_tensor()
        b = _sweep("B_fixed_cam%d" % cam_b, lambda i: cam_b)

        # --- Sweep C: alternating ---
        sentai.camera.select(cam_a)
        sentai.camera.to_tensor()
        c = _sweep("C_alt_cam%d_cam%d" % (cam_a, cam_b),
                   lambda i: cam_a if (i % 2 == 0) else cam_b)

        # --- Head-to-tail summary ---
        a_tot = a["summary"]["total"]["mean"]
        b_tot = b["summary"]["total"]["mean"]
        c_tot = c["summary"]["total"]["mean"]
        base = a_tot if a_tot > b_tot else b_tot   # slower baseline
        overhead = c_tot - base
        print("[E18] HEAD-TO-TAIL SUMMARY")
        print("  A fixed cam%d: %.1f ms/frame, %.2f fps"
              % (cam_a, a_tot, a["summary"]["fps"]))
        print("  B fixed cam%d: %.1f ms/frame, %.2f fps"
              % (cam_b, b_tot, b["summary"]["fps"]))
        print("  C alternating:  %.1f ms/frame, %.2f fps"
              % (c_tot, c["summary"]["fps"]))
        print("  => per-switch overhead = %.1f ms (C - max(A,B))" % overhead)
        print("  => overhead as fraction of a single frame budget: %.1f%%"
              % (100.0 * overhead / base if base > 0 else 0))

        result = {
            "experiment": "E18_camera_switch_headtail",
            "meta": snapshot_meta("E18_cam_switch_headtail",
                                  cam_a=cam_a, cam_b=cam_b,
                                  resolution="512x512",
                                  model_path=model_path,
                                  repetitions=repetitions,
                                  switch_drain=drain,
                                  ratio=(ra, rb)),
            "sweeps": {"A": a, "B": b, "C": c},
            "summary": {
                "A_fixed_cam_a": a["summary"],
                "B_fixed_cam_b": b["summary"],
                "C_alternating": c["summary"],
                "per_switch_overhead_ms": round(overhead, 2),
                "overhead_pct_of_baseline": round(100.0 * overhead / base, 1)
                    if base > 0 else 0,
            },
        }

        if save:
            for sweep in (a, b, c):
                path = _save_path("e18_%s" % sweep["label"])
                s = sweep["samples"]
                save_csv(path,
                    ["run_index", "cam_id", "select_ms", "to_tensor_ms",
                     "invoke_ms", "detect_ms", "num_detections", "total_frame_ms"],
                    [(i, s["cam_id"][i], s["select_ms"][i], s["to_tensor_ms"][i],
                      s["invoke_ms"][i], s["detect_ms"][i],
                      s["num_detections"][i], s["total_frame_ms"][i])
                     for i in range(repetitions)])
                _save_desc(path,
                    "E18 sub-sweep %s.\n"
                    "\nColumns:\n"
                    "  run_index, cam_id, select_ms, to_tensor_ms,\n"
                    "  invoke_ms, detect_ms, num_detections, total_frame_ms\n"
                    % sweep["label"],
                    params={"cam_a": cam_a, "cam_b": cam_b,
                            "switch_drain": drain, "ratio": (ra, rb),
                            "mean_total_ms": sweep["summary"]["total"]["mean"],
                            "fps": sweep["summary"]["fps"]})
                _record("e18_%s" % sweep["label"], path,
                        "total=%.1fms fps=%.2f" % (
                            sweep["summary"]["total"]["mean"],
                            sweep["summary"]["fps"]))
            snapshot_both_cameras("after")
            sentai.camera.select(cam_a)

        return result
    finally:
        sentai.verbose(prev_verbose)
        if owned_session:
            end()


def e17_switch_drain_visual(cam_a=0, cam_b=1, repetitions=16,
                            quality=70, save=True,
                            resolution=(512, 512)):
    """E17 — alternating camera JPEG capture at current switch_drain threshold.

    `resolution=(w,h)` sets native sensor size (default 512×512).  JPEGs
    are encoded at that size, so smaller resolutions produce smaller
    files and faster encode.

    Purpose: visually inspect whether post-switch frames contain artifacts
    (mixed pixels, AEC/AGC glitches, rolling-shutter tears) depending on
    the threshold set via `sentai.camera.switch_drain(n)`.  At n=2 the
    drain path waits for two fresh ISR frames after each MUX flip; at n=1
    it returns after one fresh frame (saves ~67 ms but may capture a
    not-yet-stable frame).

    The hot measurement loop does NOT touch LFS — every JPEG is kept in
    MicroPython heap as `bytes` and all are written to
    `<session>/e17_t<N>_frames/NNN_camX_YYYms.jpg` after the loop ends.
    This keeps the per-iteration timing representative of pure
    select + capture cost, undistorted by NAND write latency.

    Memory budget: at 512x512 quality=70 a JPEG is ~25–40 KB.  Default
    reps=16 -> ~500 KB on the MP heap (total heap 512 KB per
    sentai_runtime.cc:MP_GC_HEAP_SIZE).  Raising reps or quality may OOM
    — bail out cleanly rather than crashing the REPL.

    Usage (from REPL):
        import diag
        diag.begin("e17_probe")
        sentai.camera.switch_drain(2)        # baseline
        r2 = diag.e17_switch_drain_visual(repetitions=16)
        sentai.camera.switch_drain(1)        # fast path
        r1 = diag.e17_switch_drain_visual(repetitions=16)
        diag.end()
    """
    from diag._session import _session, begin, end

    owned_session = (_session is None)
    if owned_session:
        begin("e17_switch_drain")

    prev_verbose = sentai.verbose(0)

    # Defensive: if a prior pipeline run left PrepTask running, an in-loop
    # camera.select would race with cam_grab_latest.  Stop it first.
    if sentai.pipeline.running():
        sentai.pipeline.stop()

    try:
        w, h = resolution
        sentai.camera.set_resolution(w, h)
        if sentai.camera.frame_count() == 0:
            sentai.camera.init(1)

        # Warm-up both cameras so the first measured iteration is not a
        # cold-start outlier — we want steady-state post-switch cost.
        for c in (cam_a, cam_b):
            sentai.camera.select(c)
            sentai.camera.to_tensor()

        thresh = sentai.camera.switch_drain()
        print("[E17] switch_drain=%d  reps=%d  quality=%d  512x512" % (
            thresh, repetitions, quality))

        # Hot loop: flip MUX, encode JPEG to RAM, time.  NO disk I/O.
        # `sentai.camera.jpeg(q)` returns bytes from the MCU JPEG encoder;
        # it internally calls the same get_raw_with_recovery drain path
        # that to_tensor uses, so the measured `dt` here reflects the real
        # switch cost at the current threshold.
        captures = []  # (index, cam_id, elapsed_ms, jpeg_bytes)
        t_run_start = _ticks()
        for i in range(repetitions):
            cam = cam_a if (i % 2 == 0) else cam_b
            t0 = _ticks()
            sentai.camera.select(cam)
            jpg = sentai.camera.jpeg(quality)
            dt = _ticks() - t0
            captures.append((i, cam, dt, jpg))
        t_run_wall = _ticks() - t_run_start

        # Cold phase: write all buffered JPEGs to LFS.  Timing-sensitive
        # numbers have already been captured — the LFS write cost does
        # not leak into the per-frame statistics.
        saved_dir = None
        if save:
            base = _session.dir if _session else "/diags"
            saved_dir = "%s/e17_t%d_frames" % (base, thresh)
            try:
                sentai.fs.mkdir(saved_dir)
            except Exception:
                pass  # already exists — fine
            for (i, cam, dt, jpg) in captures:
                path = "%s/%03d_cam%d_%dms.jpg" % (saved_dir, i, cam, dt)
                try:
                    sentai.fs.write(path, jpg)
                except Exception as e:
                    print("  WARN failed to save %s: %s" % (path, e))

        # Aggregate + report.  Skip the very first sample per camera so
        # the reported stats reflect steady-state alternation, not the
        # artificial head of the run.
        times_all = [dt for (_i, _c, dt, _j) in captures]
        cam0_t = [dt for (_i, c, dt, _j) in captures
                  if c == cam_a][1:]
        cam1_t = [dt for (_i, c, dt, _j) in captures
                  if c == cam_b][1:]
        sizes = [len(j) for (_i, _c, _dt, j) in captures]

        st_all = stats(times_all)
        print("[E17] hot-loop total wall: %d ms (%d reps)" % (
            t_run_wall, repetitions))
        _print_stats("per-frame", st_all)
        if cam0_t:
            _print_stats("  cam%d" % cam_a, stats(cam0_t))
        if cam1_t:
            _print_stats("  cam%d" % cam_b, stats(cam1_t))
        print("  jpeg size: min=%d max=%d mean=%d bytes" % (
            min(sizes), max(sizes), sum(sizes) // len(sizes)))
        if saved_dir:
            print("  saved %d JPEGs to %s" % (len(captures), saved_dir))

        # Summary CSV so the session has a machine-readable record of the
        # timing even when the visual JPEGs are the primary artefact.
        if save:
            csv_path = _save_path("e17_switch_drain_t%d" % thresh)
            save_csv(csv_path,
                ["run_index", "cam_id", "elapsed_ms", "jpeg_bytes"],
                [(c[0], c[1], c[2], len(c[3])) for c in captures])
            _save_desc(csv_path,
                "E17 - Alternating camera JPEG capture at runtime-set\n"
                "switch_drain threshold.  Hot loop keeps JPEGs in RAM;\n"
                "LFS writes happen after timing measurement completes.\n"
                "\nColumns:\n"
                "  run_index    : frame index (0-based)\n"
                "  cam_id       : cam_a on even, cam_b on odd\n"
                "  elapsed_ms   : select + jpeg encode total (wall)\n"
                "  jpeg_bytes   : size of encoded JPEG in bytes\n",
                params={"cam_a": cam_a, "cam_b": cam_b,
                        "repetitions": repetitions, "quality": quality,
                        "switch_drain": thresh,
                        "hot_wall_ms": t_run_wall,
                        "jpeg_dir": saved_dir})
            _record("e17_switch_drain", csv_path,
                    "thr=%d reps=%d mean=%.1fms" % (
                        thresh, repetitions, st_all["mean"]))

        return {"experiment": "E17_switch_drain_visual",
                "params": {"cam_a": cam_a, "cam_b": cam_b,
                           "repetitions": repetitions, "quality": quality,
                           "switch_drain": thresh},
                "samples": {"elapsed_ms": times_all,
                            "cam_id": [c[1] for c in captures],
                            "jpeg_bytes": sizes},
                "summary": {"per_frame": st_all,
                            "cam_a_after_skip": stats(cam0_t) if cam0_t else None,
                            "cam_b_after_skip": stats(cam1_t) if cam1_t else None,
                            "hot_wall_ms": t_run_wall,
                            "jpeg_dir": saved_dir}}
    finally:
        sentai.verbose(prev_verbose)
        if owned_session:
            end()


def e14_pipeline_parallel(model_path, camera_id=0, width=320, height=320,
                          conf=0.25, iou=0.45, max_det=50,
                          repetitions=30, save=True,
                          compare_sequential=False,
                          _experiment_tag="E14"):
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
    tag = _experiment_tag                      # e.g. "E14" or "E15"
    tag_lo = tag.lower()                       # used in file-name prefix
    print("[%s] Pipeline parallel — cam%d %dx%d model=%s %d reps (conf=%.2f iou=%.2f)" % (
        tag, camera_id, width, height, model_path, repetitions, conf, iou))

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

    # Path-keyed reload: only call tpu.load() when the requested model differs
    # from whatever is currently resident.  Two reasons:
    #  1. Correctness — skipping the load purely on `tpu.ready()` silently
    #     ran the previous experiment's model with the new experiment's
    #     tensor layout, which took weeks of debugging off someone's life.
    #  2. Memory — the EdgeTpuManager package cache keyed by model-data
    #     pointer accumulates an entry per load; reloading the same model
    #     20 times burns heap until SDRAM pressure slows the pipeline from
    #     15 FPS down to 6 FPS (observed).
    _ensure_model_loaded(model_path)

    # Warm camera pipeline so first frame isn't a cold-start outlier.
    # Has to run *before* the BEFORE snapshot — otherwise snapshot_scene
    # sees frame_count==0 (camera streaming hasn't produced a frame yet)
    # and silently skips, leaving the session without a BEFORE image.
    sentai.camera.to_tensor()
    gc.collect()

    # Scene snapshot (BEFORE the measurement loop).  Camera is assumed static
    # across the run, so we capture the exact PXP-scaled pixels the TPU will
    # see — useful when `dets_total == 0` to offline-verify scene content.
    # We snapshot BOTH cameras (cam0 + cam1) so the session dir documents
    # what each sensor saw even when only one is exercised here — cheap
    # (~100–500 ms for two MUX flips) and pays off when diffing offline.
    from diag._session import snapshot_both_cameras
    if save:
        snapshot_both_cameras("before")
        # snapshot_both_cameras leaves cam1 selected — restore experiment cam.
        sentai.camera.select(camera_id)
        sentai.camera.to_tensor()  # drain stale frames from the switch

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
        # Scene snapshots (AFTER the measurement loop).  Pipeline is stopped
        # so camera.select + to_tensor() are free to grab frames.  Shoot both
        # cameras to pair with the BEFORE pair — diff offline if the scene
        # drifted or an LED blinked mid-run.
        if save:
            snapshot_both_cameras("after")
            # Leave experiment camera selected on exit.
            try:
                sentai.camera.select(camera_id)
            except Exception:
                pass
        sentai.verbose(prev_verbose)

    if not intervals_ms:
        print("[%s] No frames received — pipeline stalled." % tag)
        return None

    st_interval = stats(intervals_ms)
    st_prep     = stats(prep_stall_s)
    st_infer    = stats(infer_stall_s)

    fps_wall = 1000.0 / st_interval["mean"] if st_interval["mean"] > 0 else 0
    run_ms = t_run_end - t_run_start
    dets_total = sum(ndet_s)

    meta = snapshot_meta("%s_pipeline_parallel" % tag,
                         camera_id=camera_id,
                         resolution="%dx%d" % (width, height),
                         model_path=model_path,
                         conf=conf, iou=iou,
                         repetitions=repetitions)

    print("[%s] Done." % tag)
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

    result = {"experiment": "%s_pipeline_parallel" % tag, "meta": meta,
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
        path = _save_path("%s_pipeline_par_cam%d_%dx%d" % (
            tag_lo, camera_id, width, height))
        save_csv(path,
            ["run_index",
             "frame_interval_ms",
             "prep_stall_ms",
             "infer_stall_ms",
             "num_detections"],
            [(i, intervals_ms[i], prep_stall_s[i], infer_stall_s[i], ndet_s[i])
             for i in range(len(intervals_ms))])
        _save_desc(path,
            "%s - Parallel inference pipeline, measured sustained FPS.\n"
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
            "\nSustained FPS = 1000 / mean(frame_interval_ms)." % tag,
            params={"camera_id": camera_id, "width": width, "height": height,
                    "model_path": model_path, "repetitions": repetitions,
                    "conf": conf, "iou": iou, "max_det": max_det,
                    "frames_received": got,
                    "frames_timeouts": timeouts,
                    "fw_processed": processed_fw,
                    "fw_dropped": dropped_fw})
        _record("%s_pipeline_parallel" % tag_lo, path,
                "cam%d %dx%d fps_wall=%.1f fw_fps=%.1f dropped=%d dets=%d" % (
                    camera_id, width, height, fps_wall, avg_fps_fw,
                    dropped_fw, dets_total))
        print("  Saved: %s" % path)

    # Optional inline comparison against sequential E13 on the same config.
    if compare_sequential:
        print("\n[%s] Running E13 on the same config for comparison..." % tag)
        seq = e13_pipeline_full(model_path, camera_id=camera_id,
                                width=width, height=height,
                                conf=conf, iou=iou, max_det=max_det,
                                repetitions=repetitions, save=save)
        if seq:
            seq_total = seq["summary"]["total"]["mean"]
            seq_fps   = seq["summary"]["fps"]
            speedup   = fps_wall / seq_fps if seq_fps > 0 else 0
            saved_ms  = seq_total - st_interval["mean"]
            print("\n[%s] Comparison %s(parallel) vs E13(sequential):" % (tag, tag))
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
