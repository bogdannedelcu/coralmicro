# diag/e_fs.py — E6: filesystem read, E7: filesystem write

import sentai
from diag._util import stats, time_call, save_csv, snapshot_meta, _print_stats
from diag._session import _save_path, _record


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
              "extra": {"file_size": file_size,
                        "throughput_kbs": round(throughput_kbs, 1)}}

    if save:
        csv_path = _save_path("e6_fs_read")
        save_csv(csv_path, ["run", "read_ms"],
                 [(i, s) for i, s in enumerate(samples)])
        _record("e6_fs_read", csv_path,
                "%.1f KB/s mean=%.1f ms" % (throughput_kbs, st["mean"]))
        print("  Saved: %s" % csv_path)

    return result


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
              "extra": {"data_size": size,
                        "throughput_kbs": round(throughput_kbs, 1)}}

    if save:
        csv_path = _save_path("e7_fs_write_%db" % size)
        save_csv(csv_path, ["run", "write_ms"],
                 [(i, s) for i, s in enumerate(samples)])
        _record("e7_fs_write", csv_path,
                "%dB %.1f KB/s mean=%.1f ms" % (size, throughput_kbs, st["mean"]))
        print("  Saved: %s" % csv_path)

    sentai.fs.remove(test_path)
    return result
