import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
RUN_ID = int(globals().get("_run_id", 0))
DRAIN = int(globals().get("_drain", 1))
REPS = int(globals().get("_reps", 40))
WARMUP = int(globals().get("_warmup", 3))
CHUNK_KB = int(globals().get("_chunk_kb", 128))
OUT_DIR = "/diags/s235_b10_camera_switch"

HEADER = (
    "version,run,model,chunk_kb,drain,rep,target_cam,select_rc,"
    "select_us,to_tensor_rc,to_tensor_us,total_us,start_frame,end_frame,"
    "frame_delta,grabbed_id,current_id\r\n"
)


def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff


def csv_escape(s):
    s = str(s)
    if "," in s or '"' in s or "\n" in s or "\r":
        return '"' + s.replace('"', '""') + '"'
    return s


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass


def camera_state_value(name, default=-1):
    try:
        return int(getattr(sentai.camera, name)())
    except Exception:
        return default


def write_row(path, vals, first):
    if first:
        sentai.fs.write(path, HEADER)
    sentai.fs.append(path, ",".join(csv_escape(v) for v in vals) + "\r\n")


def configure():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10.5 camera switch latency once")
    print("version=%s" % sentai.version())
    print("run=%d drain=%d reps=%d warmup=%d" % (RUN_ID, DRAIN, REPS, WARMUP))
    try:
        sentai.tpu.desc_cache(0)
    except Exception as e:
        print("desc_cache_disable_failed", repr(e))
    print("chunk_prev=%s chunk_now=%s" %
          (str(sentai.tpu.chunk_size()), str(sentai.tpu.chunk_size(CHUNK_KB * 1024))))
    print("model_size=%s" % str(sentai.fs.size(MODEL)))
    print("tpu_load=%s" % str(sentai.tpu.load(MODEL)))
    print("tpu_ready=%s" % str(sentai.tpu.ready()))
    try:
        print("pipeline_stop_pre=%s" % str(sentai.pipeline.stop()))
    except Exception as e:
        print("pipeline_stop_pre_failed", repr(e))
    try:
        print("camera_ratio_prev=%s" % str(sentai.camera.ratio(0, 0)))
    except Exception as e:
        print("camera_ratio_failed", repr(e))
    print("switch_drain_prev=%s" % str(sentai.camera.switch_drain(DRAIN)))
    print("camera_select0=%s" % str(sentai.camera.select(0)))
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.rtos.sleep_ms(1000)
    print("camera_frame_count=%s" % str(sentai.camera.frame_count()))
    for i in range(WARMUP):
        rc = sentai.camera.to_tensor()
        print("warmup_to_tensor[%d]=%s" % (i, str(rc)))
        sentai.rtos.sleep_ms(20)


def run_once():
    out_csv = "%s/run_%02d_drain%d.csv" % (OUT_DIR, RUN_ID, DRAIN)
    first = True
    for rep in range(REPS):
        target = rep & 1
        start_frame = int(sentai.camera.frame_count())
        t0 = sentai.rtos.micros()
        select_rc = sentai.camera.select(target)
        t1 = sentai.rtos.micros()
        tensor_rc = sentai.camera.to_tensor()
        t2 = sentai.rtos.micros()
        end_frame = int(sentai.camera.frame_count())
        grabbed = camera_state_value("grabbed_id")
        current = camera_state_value("current_id")
        vals = (
            sentai.version(), RUN_ID, MODEL, CHUNK_KB, DRAIN, rep, target,
            int(select_rc), du32(t0, t1), int(tensor_rc), du32(t1, t2),
            du32(t0, t2), start_frame, end_frame, end_frame - start_frame,
            grabbed, current,
        )
        write_row(out_csv, vals, first)
        first = False
        print(
            "row rep=%d target=%d select_us=%d to_tensor_us=%d total_us=%d "
            "rc=%d grabbed=%d current=%d frame_delta=%d" %
            (rep, target, du32(t0, t1), du32(t1, t2), du32(t0, t2),
             int(tensor_rc), grabbed, current, end_frame - start_frame)
        )
    try:
        print("fs_sync=%s" % str(sentai.fs.sync()))
    except Exception as e:
        print("fs_sync_failed", repr(e))
    print("csv=%s" % out_csv)


configure()
run_once()
print("=== done ===")
