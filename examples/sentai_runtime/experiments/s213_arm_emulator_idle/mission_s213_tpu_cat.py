import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
IMAGE = "/images/cat_640x480.bmp"
FR_EVENTS = "/fr/events.csv"
FR_SCALARS = "/fr/scalars.csv"
TIMING_SUMMARY = "/tpu_timing_summary.txt"


def _ticks_ms():
    return int(sentai.rtos.ticks_ms())


def _dt_ms(start):
    return (_ticks_ms() - start) & 0x3fffffff


def _kv_text(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    return str(v)


def _write_summary(summary):
    lines = []
    for k in (
            "model_size",
            "image_size",
            "model_stage_ms",
            "image_fs_to_mem_ms",
            "host_preload_image_ms",
            "tpu_start_ms",
            "first_invoke_ms",
            "first_invoke_repl_ms",
            "first_params_bytes",
            "first_input_bytes",
            "first_output_bytes",
            "steady_completed",
            "steady_total_ms",
            "steady_fps_x100",
            "steady_fps",
            "steady_invoke_ms_sum",
            "steady_input_bytes_last",
            "steady_output_bytes_last",
            "detections_count",
            "fr_events_size",
            "fr_scalars_size",
            "status"):
        if k in summary:
            lines.append(k + "=" + _kv_text(summary[k]))
    sentai.fs.write(TIMING_SUMMARY, "\n".join(lines) + "\n")


def _j(event, data=None):
    text = ""
    if isinstance(data, dict):
        parts = []
        for k, v in data.items():
            parts.append(str(k) + "=" + _kv_text(v))
        text = " ".join(parts)
    elif data is not None:
        text = str(data)
    try:
        sentai.fr.push_event(str(event), text)
    except Exception:
        pass


def _scalar(label, value):
    ts = _ticks_ms()
    try:
        sentai.fr.push_scalar(str(label), value, ts)
    except Exception:
        pass


STATS_NAMES = (
    "params_calls", "params_bytes", "params_ticks",
    "input_calls", "input_bytes", "input_ticks",
    "ins_calls", "ins_bytes", "ins_ticks",
    "output_calls", "output_bytes", "output_ticks",
    "event_calls", "event_ticks",
    "usb_out_calls", "usb_out_req", "usb_out_done", "usb_out_us",
    "usb_in_calls", "usb_in_req", "usb_in_done", "usb_in_us",
    "usb_event_calls", "usb_event_req", "usb_event_done", "usb_event_us",
    "usb_timeouts", "usb_failed",
)


def _stats_dict():
    values = sentai.tpu.stats()
    out = {}
    for i in range(len(STATS_NAMES)):
        out[STATS_NAMES[i]] = values[i]
    return out


def _log_stats(prefix):
    stats = _stats_dict()
    print("TPU_STATS_" + prefix,
          "params_calls", stats["params_calls"],
          "params_bytes", stats["params_bytes"],
          "input_calls", stats["input_calls"],
          "input_bytes", stats["input_bytes"],
          "ins_calls", stats["ins_calls"],
          "ins_bytes", stats["ins_bytes"],
          "output_calls", stats["output_calls"],
          "output_bytes", stats["output_bytes"],
          "event_calls", stats["event_calls"],
          "usb_out_us", stats["usb_out_us"],
          "usb_in_us", stats["usb_in_us"],
          "usb_event_us", stats["usb_event_us"])
    _j("tpu_stats_" + prefix, {
        "params_calls": stats["params_calls"],
        "params_bytes": stats["params_bytes"],
        "input_bytes": stats["input_bytes"],
        "ins_bytes": stats["ins_bytes"],
        "output_bytes": stats["output_bytes"],
        "event_calls": stats["event_calls"],
        "usb_out_us": stats["usb_out_us"],
        "usb_in_us": stats["usb_in_us"],
        "usb_event_us": stats["usb_event_us"],
    })
    for name in (
            "params_bytes", "input_bytes", "ins_bytes", "output_bytes",
            "usb_out_us", "usb_in_us", "usb_event_us"):
        _scalar(prefix + "_" + name, stats[name])
    return stats


def run():
    print("MISSION_TPU_CAT_BEGIN")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMAGE_SIZE", sentai.fs.size(IMAGE))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("TPU_LOAD_IMAGE", sentai.tpu.load_image(IMAGE))
    print("TPU_INVOKE", sentai.tpu.invoke())
    print("TPU_OUTPUTS", sentai.tpu.num_outputs())
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS_COUNT", len(dets))
    print("DETECTIONS", dets)
    print("DETECTIONS_WRITTEN",
          sentai.fs.write("/detections.txt", repr(dets)))
    print("MISSION_TPU_CAT_DONE")
    return len(dets)


def fps(runs=5):
    print("MISSION_TPU_FPS_BEGIN")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMAGE_SIZE", sentai.fs.size(IMAGE))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("TPU_LOAD_IMAGE", sentai.tpu.load_image(IMAGE))
    result = sentai.tpu.fps(runs)
    print("TPU_FPS_RESULT", result)
    completed, warmup, measured_ms, fps_x100, invoke_ms_sum, det_count = result
    print("TPU_FPS_COMPLETED", completed)
    print("TPU_FPS_WARMUP", warmup)
    print("TPU_FPS_MEASURED_MS", measured_ms)
    print("TPU_FPS_X100", fps_x100)
    print("TPU_FPS", fps_x100 / 100)
    print("TPU_FPS_INVOKE_MS_SUM", invoke_ms_sum)
    print("DETECTIONS_COUNT", det_count)
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS", dets)
    print("MISSION_TPU_FPS_DONE")
    return fps_x100


def fps_mem(runs=5):
    print("MISSION_TPU_FPS_MEM_BEGIN")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMAGE_SIZE", sentai.fs.size(IMAGE))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("TPU_LOAD_IMAGE_MEM", sentai.tpu.load_image_mem(IMAGE))
    print("IMAGE_MEM_SIZE", sentai.tpu.image_mem_size())
    result = sentai.tpu.fps(runs)
    print("TPU_FPS_MEM_RESULT", result)
    completed, warmup, measured_ms, fps_x100, invoke_ms_sum, det_count = result
    print("TPU_FPS_MEM_COMPLETED", completed)
    print("TPU_FPS_MEM_WARMUP", warmup)
    print("TPU_FPS_MEM_MEASURED_MS", measured_ms)
    print("TPU_FPS_MEM_X100", fps_x100)
    print("TPU_FPS_MEM", fps_x100 / 100)
    print("TPU_FPS_MEM_INVOKE_MS_SUM", invoke_ms_sum)
    print("DETECTIONS_COUNT", det_count)
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS", dets)
    print("MISSION_TPU_FPS_MEM_DONE")
    return fps_x100


def fps_mem_invoke(runs=5):
    print("MISSION_TPU_FPS_MEM_LOOP_BEGIN")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMAGE_SIZE", sentai.fs.size(IMAGE))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("TPU_LOAD_IMAGE_MEM", sentai.tpu.load_image_mem(IMAGE))
    print("IMAGE_MEM_SIZE", sentai.tpu.image_mem_size())
    result = sentai.tpu.fps_invoke(runs)
    print("TPU_FPS_MEM_LOOP_RESULT", result)
    completed, warmup, measured_ms, fps_x100, invoke_ms_sum, det_count = result
    print("TPU_FPS_MEM_LOOP_COMPLETED", completed)
    print("TPU_FPS_MEM_LOOP_WARMUP", warmup)
    print("TPU_FPS_MEM_LOOP_MEASURED_MS", measured_ms)
    print("TPU_FPS_MEM_LOOP_X100", fps_x100)
    print("TPU_FPS_MEM_LOOP", fps_x100 / 100)
    print("TPU_FPS_MEM_LOOP_INVOKE_MS_SUM", invoke_ms_sum)
    print("DETECTIONS_COUNT", det_count)
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS", dets)
    print("MISSION_TPU_FPS_MEM_LOOP_DONE")
    return fps_x100


def fps_mem_session(runs=5):
    print("MISSION_TPU_FPS_MEM_SESSION_BEGIN")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMAGE_SIZE", sentai.fs.size(IMAGE))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("TPU_LOAD_IMAGE_MEM", sentai.tpu.load_image_mem(IMAGE))
    print("IMAGE_MEM_SIZE", sentai.tpu.image_mem_size())
    print("TPU_START", sentai.tpu.start())
    print("TPU_FPS_MEM_SESSION_MODE", "aggregate")
    result = sentai.tpu.fps(runs)
    print("TPU_FPS_MEM_SESSION_RESULT", result)
    completed, warmup, measured_ms, fps_x100, invoke_ms_sum, det_count = result
    print("TPU_FPS_MEM_SESSION_COMPLETED", completed)
    print("TPU_FPS_MEM_SESSION_WARMUP", warmup)
    print("TPU_FPS_MEM_SESSION_MEASURED_MS", measured_ms)
    print("TPU_FPS_MEM_SESSION_X100", fps_x100)
    print("TPU_FPS_MEM_SESSION", fps_x100 / 100)
    print("TPU_FPS_MEM_SESSION_INVOKE_MS_SUM", invoke_ms_sum)
    print("DETECTIONS_COUNT", det_count)
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS", dets)
    print("TPU_STOP", sentai.tpu.stop())
    print("MISSION_TPU_FPS_MEM_SESSION_DONE")
    return fps_x100


def timing_mem_session(runs=5):
    print("MISSION_TPU_TIMING_BEGIN")
    summary = {
        "status": "STARTED",
        "model_size": sentai.fs.size(MODEL),
        "image_size": sentai.fs.size(IMAGE),
    }

    sentai.fr.init()
    print("FR_OPEN_EVENTS", sentai.fr.open("events", FR_EVENTS))
    print("FR_OPEN_SCALARS", sentai.fr.open("scalars", FR_SCALARS))
    print("FR_TASK_START", sentai.fr.task_start())
    _j("mission_start", {
        "name": "s213_tpu_timing",
        "runs": runs,
        "model_size": summary["model_size"],
        "image_size": summary["image_size"],
    })

    stop_rc = -999
    try:
        print("MODEL_SIZE", summary["model_size"])
        print("IMAGE_SIZE", summary["image_size"])

        t0 = _ticks_ms()
        rc = sentai.tpu.load(MODEL)
        summary["model_stage_ms"] = _dt_ms(t0)
        print("TPU_LOAD", rc)
        print("MODEL_STAGE_MS", summary["model_stage_ms"])
        _j("model_stage", {"rc": rc, "ms": summary["model_stage_ms"]})
        _scalar("model_stage_ms", summary["model_stage_ms"])
        if rc != 0:
            summary["status"] = "FAIL_TPU_LOAD"
            return summary

        t0 = _ticks_ms()
        rc = sentai.tpu.load_image_mem(IMAGE, False)
        summary["image_fs_to_mem_ms"] = _dt_ms(t0)
        summary["image_mem_size"] = sentai.tpu.image_mem_size()
        print("TPU_IMAGE_FS_TO_MEM", rc)
        print("TPU_IMAGE_FS_TO_MEM_MS", summary["image_fs_to_mem_ms"])
        print("IMAGE_MEM_SIZE", summary["image_mem_size"])
        _j("image_fs_to_mem", {
            "rc": rc,
            "ms": summary["image_fs_to_mem_ms"],
            "bytes": summary["image_mem_size"],
        })
        _scalar("image_fs_to_mem_ms", summary["image_fs_to_mem_ms"])
        if rc != 0:
            summary["status"] = "FAIL_IMAGE_FS_TO_MEM"
            return summary

        t0 = _ticks_ms()
        rc = sentai.tpu.load_image_mem("", True)
        summary["host_preload_image_ms"] = _dt_ms(t0)
        print("HOST_PRELOAD_IMAGE", rc)
        print("HOST_PRELOAD_IMAGE_MS", summary["host_preload_image_ms"])
        _j("host_preload_image", {
            "rc": rc,
            "ms": summary["host_preload_image_ms"],
        })
        _scalar("host_preload_image_ms", summary["host_preload_image_ms"])
        if rc != 0:
            summary["status"] = "FAIL_IMAGE_HOST_STAGE"
            return summary

        t0 = _ticks_ms()
        rc = sentai.tpu.start()
        summary["tpu_start_ms"] = _dt_ms(t0)
        print("TPU_START", rc)
        print("TPU_START_MS", summary["tpu_start_ms"])
        _j("tpu_start", {"rc": rc, "ms": summary["tpu_start_ms"]})
        _scalar("tpu_start_ms", summary["tpu_start_ms"])
        if rc != 0:
            summary["status"] = "FAIL_TPU_START"
            return summary

        t0 = _ticks_ms()
        first_ms = sentai.tpu.invoke()
        summary["first_invoke_repl_ms"] = _dt_ms(t0)
        summary["first_invoke_ms"] = first_ms
        print("TPU_FIRST_INVOKE", first_ms)
        print("TPU_FIRST_INVOKE_REPL_MS", summary["first_invoke_repl_ms"])
        _j("first_invoke", {
            "invoke_ms": first_ms,
            "repl_ms": summary["first_invoke_repl_ms"],
        })
        _scalar("first_invoke_ms", first_ms)
        _scalar("first_invoke_repl_ms", summary["first_invoke_repl_ms"])
        if first_ms < 0:
            summary["status"] = "FAIL_FIRST_INVOKE"
            return summary
        first_stats = _log_stats("FIRST")
        summary["first_params_bytes"] = first_stats["params_bytes"]
        summary["first_input_bytes"] = first_stats["input_bytes"]
        summary["first_output_bytes"] = first_stats["output_bytes"]

        steady_start = _ticks_ms()
        completed = 0
        invoke_ms_sum = 0
        last_stats = {}
        for i in range(runs):
            t_frame = _ticks_ms()
            invoke_ms = sentai.tpu.invoke()
            repl_ms = _dt_ms(t_frame)
            if invoke_ms < 0:
                break
            completed += 1
            invoke_ms_sum += invoke_ms
            print("TPU_STEADY_INVOKE", i, invoke_ms)
            print("TPU_STEADY_INVOKE_REPL_MS", i, repl_ms)
            _j("steady_invoke_one", {
                "i": i,
                "invoke_ms": invoke_ms,
                "repl_ms": repl_ms,
            })
            _scalar("steady_invoke_ms", invoke_ms)
            _scalar("steady_invoke_repl_ms", repl_ms)
            last_stats = _log_stats("STEADY_" + str(i))

        measured_ms = _dt_ms(steady_start)
        if invoke_ms_sum > measured_ms:
            measured_ms = invoke_ms_sum
        if measured_ms == 0:
            measured_ms = 1
        fps_x100 = int((completed * 100000) / measured_ms)
        det_count = 0
        try:
            det_count = len(sentai.pipeline.detections(100))
        except Exception:
            det_count = 0
        summary["steady_completed"] = completed
        summary["steady_total_ms"] = measured_ms
        summary["steady_fps_x100"] = fps_x100
        summary["steady_fps"] = fps_x100 / 100
        summary["steady_invoke_ms_sum"] = invoke_ms_sum
        summary["detections_count"] = det_count
        if last_stats:
            summary["steady_input_bytes_last"] = last_stats["input_bytes"]
            summary["steady_output_bytes_last"] = last_stats["output_bytes"]
        print("TPU_STEADY_COMPLETED", completed)
        print("TPU_STEADY_TOTAL_MS", measured_ms)
        print("TPU_STEADY_FPS_X100", fps_x100)
        print("TPU_STEADY_FPS", fps_x100 / 100)
        print("TPU_STEADY_INVOKE_MS_SUM", invoke_ms_sum)
        print("DETECTIONS_COUNT", det_count)
        _j("steady_invoke", {
            "completed": completed,
            "total_ms": measured_ms,
            "fps_x100": fps_x100,
            "invoke_ms_sum": invoke_ms_sum,
            "detections": det_count,
        })
        _scalar("steady_total_ms", measured_ms)
        _scalar("steady_fps_x100", fps_x100)
        _scalar("steady_invoke_ms_sum", invoke_ms_sum)
        _scalar("detections_count", det_count)

        summary["status"] = "OK"
        return summary
    finally:
        stop_rc = sentai.tpu.stop()
        print("TPU_STOP", stop_rc)
        _j("tpu_stop", {"rc": stop_rc})
        summary["tpu_stop_rc"] = stop_rc
        summary["fr_events_size"] = sentai.fs.size(FR_EVENTS)
        summary["fr_scalars_size"] = sentai.fs.size(FR_SCALARS)
        _write_summary(summary)
        print("FR_EVENTS_SIZE", summary["fr_events_size"])
        print("FR_SCALARS_SIZE", summary["fr_scalars_size"])
        print("TIMING_SUMMARY_WRITTEN",
              sentai.fs.write("/timing_summary_copy.txt",
                              sentai.fs.read_str(TIMING_SUMMARY)))
        print("FR_TASK_STOP", sentai.fr.task_stop())
        print("FR_EVENTS_PATH", FR_EVENTS)
        print("FR_SCALARS_PATH", FR_SCALARS)
        print("TIMING_SUMMARY_PATH", TIMING_SUMMARY)
        print("MISSION_TPU_TIMING_DONE")
