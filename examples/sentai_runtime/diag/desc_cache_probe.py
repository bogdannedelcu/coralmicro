import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
CHUNK_KB = 128
N_INVOKES = 20
WARMUP = 3
OUT_DIR = "/diags/s235_b10_desc_cache"
OUT_CSV = OUT_DIR + "/desc_cache_probe.csv"

TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
FIELDS = (
    "calls", "callbacks", "bytes_req", "bytes_done",
    "submit_cyc", "callback_cyc", "wait_cyc",
    "errors", "timeouts", "submit_fail",
    "submit_us", "callback_us", "wait_us",
)


def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass
    if not sentai.fs.exists(OUT_CSV):
        sentai.fs.write(
            OUT_CSV,
            "version,phase,desc_cache,chunk_kb,invokes,wall_us,"
            "wall_ms_per_invoke,fps,failures,hash0,mismatches,"
            "sent_params,sent_ins,skip_params,skip_ins,"
            "phase_name,calls,bytes_req,bytes_done,wait_ms_per_invoke,"
            "errors,timeouts,submit_fail\r\n",
        )


def output_hash():
    data = sentai.tpu.output(0)
    h = 2166136261
    for b in data:
        h = ((h ^ int(b)) * 16777619) & 0xffffffff
    return h


def urb(reset=False):
    hz, rows = sentai.tpu.urb_stats(1 if reset else 0)
    out = {}
    for i, tag in enumerate(TAGS):
        row = rows[i]
        d = {}
        for j, field in enumerate(FIELDS):
            d[field] = row[j]
        out[tag] = d
    return hz, out


def append_rows(phase, enabled, invokes, wall_us, failures, hash0,
                mismatches, cache_stats, rows):
    wall_ms_per = float(wall_us) / (1000.0 * invokes)
    fps = (1000000.0 * invokes) / float(wall_us) if wall_us else 0.0
    sent_params = int(cache_stats[1])
    sent_ins = int(cache_stats[2])
    skip_params = int(cache_stats[3])
    skip_ins = int(cache_stats[4])
    for tag in TAGS:
        r = rows[tag]
        line = (
            "%s,%s,%d,%d,%d,%d,%.6f,%.6f,%d,%08x,%d,"
            "%d,%d,%d,%d,%s,%d,%d,%d,%.6f,%d,%d,%d\r\n"
        ) % (
            sentai.version(),
            phase,
            int(enabled),
            CHUNK_KB,
            invokes,
            wall_us,
            wall_ms_per,
            fps,
            failures,
            hash0,
            mismatches,
            sent_params,
            sent_ins,
            skip_params,
            skip_ins,
            tag,
            int(r["calls"]),
            int(r["bytes_req"]),
            int(r["bytes_done"]),
            float(r["wait_us"]) / (1000.0 * invokes),
            int(r["errors"]),
            int(r["timeouts"]),
            int(r["submit_fail"]),
        )
        sentai.fs.append(OUT_CSV, line)
    try:
        sentai.fs.sync()
    except Exception:
        pass


def run_phase(name, enabled, hash0):
    sentai.tpu.desc_cache(1 if enabled else 0)
    sentai.tpu.desc_cache_stats(1)
    urb(True)
    t0 = sentai.rtos.micros()
    failures = 0
    mismatches = 0
    first_hash = hash0
    for _ in range(N_INVOKES):
        rc = sentai.tpu.invoke()
        if rc < 0:
            failures += 1
            continue
        h = output_hash()
        if first_hash is None:
            first_hash = h
        elif h != first_hash:
            mismatches += 1
    t1 = sentai.rtos.micros()
    _, rows = urb(False)
    stats = sentai.tpu.desc_cache_stats()
    wall_us = du32(t0, t1)
    append_rows(name, enabled, N_INVOKES, wall_us, failures, first_hash,
                mismatches, stats, rows)
    print(
        "result phase=%s enabled=%d wall_us=%d ms_per=%.6f fps=%.6f "
        "failures=%d hash=%08x mismatches=%d stats=%s" % (
            name,
            int(enabled),
            wall_us,
            float(wall_us) / (1000.0 * N_INVOKES),
            (1000000.0 * N_INVOKES) / float(wall_us) if wall_us else 0.0,
            failures,
            first_hash,
            mismatches,
            str(stats),
        )
    )
    print("phase,calls,bytes_req,bytes_done,wait_ms_per_invoke,errors,timeouts,submit_fail")
    for tag in TAGS:
        r = rows[tag]
        print("%s,%d,%d,%d,%.6f,%d,%d,%d" % (
            tag,
            int(r["calls"]),
            int(r["bytes_req"]),
            int(r["bytes_done"]),
            float(r["wait_us"]) / (1000.0 * N_INVOKES),
            int(r["errors"]),
            int(r["timeouts"]),
            int(r["submit_fail"]),
        ))
    return first_hash


sentai.verbose(1)
ensure_out()
print("S235 B10 desc_cache validation")
print("version=%s" % sentai.version())
print("model=%s" % MODEL)
print("chunk_kb=%d invokes=%d warmup=%d" % (CHUNK_KB, N_INVOKES, WARMUP))
print("desc_cache_initial=%s stats=%s" %
      (str(sentai.tpu.desc_cache()), str(sentai.tpu.desc_cache_stats(1))))

sentai.tpu.chunk_size(CHUNK_KB * 1024)
sentai.tpu.desc_cache(0)
rc = sentai.tpu.load(MODEL)
print("load_rc=%s" % str(rc))
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)

hash0 = None
for i in range(WARMUP):
    rc = sentai.tpu.invoke()
    h = output_hash() if rc >= 0 else 0
    if hash0 is None and rc >= 0:
        hash0 = h
    print("warmup%d=%s hash=%08x" % (i, str(rc), h))

hash0 = run_phase("off", 0, hash0)

print("enable_desc_cache")
sentai.tpu.desc_cache(1)
sentai.tpu.desc_cache_stats(1)
prime = sentai.tpu.invoke()
prime_hash = output_hash() if prime >= 0 else 0
print("prime=%s hash=%08x stats=%s" %
      (str(prime), prime_hash, str(sentai.tpu.desc_cache_stats())))

run_phase("on_after_prime", 1, hash0)
sentai.tpu.desc_cache(0)
print("csv=%s" % OUT_CSV)
print("=== s235_b10_desc_cache_probe_done ===")
