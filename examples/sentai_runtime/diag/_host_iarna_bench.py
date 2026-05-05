#!/usr/bin/env python3
"""Bench iarna candidates + on-board yolo_1 reference on Coral USB.

Adds INSTRUCTION-STREAM size extraction (the per-invoke USB payload
that libedgetpu's SendInstructions ships every invoke).  Computed two
ways and reported:

  - "Inst (Δcompile)": (compiled .tflite size) − (pre-compile .tflite
    weights size) when both files exist.  Approximate; includes
    EdgeTPU graph metadata.
  - "Inst (custom-op)": exact size of the buffer attached to the
    edgetpu-custom-op CUSTOM operator's input tensor in the compiled
    .tflite.  This is what libedgetpu hands to SendInstructions
    (sans the cached-parameter portion if param caching is active).
  - "OnChip params": from the EdgeTPU compile log (cached, NOT
    re-sent per invoke).
  - "Per-invoke USB": the bytes that actually move on USB each
    invoke = "Inst (custom-op)" − "OnChip params".  This is the
    bandwidth headline.
"""
import os, re, sys, time, statistics
import numpy as np
import tflite_runtime.interpreter as tflite
from pycoral.utils import edgetpu

MODELS = [
    # name, folder, override edgetpu-tflite path, override pre-compile path
    ("p2p4-5ep(YOLO base)",
        None, "models/iarna_p2p4_5ep_export_640x480_uint8.tflite", None),
    ("yolo_1_512_1up_inloc_P5 (CANON on-board)",
        None,
        "models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
        None),
    ("yolo_1_512_1up (alt)",
        None, "models/yolo_1_class_512_1_upsample.tflite", None),
    ("C2f",
        "models/iarna_p3p4_C2f_1ep/export_uint8_480x640", None, None),
    ("GELAN",
        "models/iarna_p3p4_GELAN_1ep/export_uint8_480x640", None, None),
    ("MSBlock",
        "models/iarna_p3p4_MSBlock_1ep/export_uint8_480x640", None, None),
]

def find_one(folder, pattern):
    if folder is None: return None
    for f in os.listdir(folder):
        if re.search(pattern, f):
            return os.path.join(folder, f)
    return None

def parse_param_count_txt(tflite_path):
    """When a .txt is saved alongside (yolov5 export prints), extract
    the 'NNN parameters' count.  Used as the fall-back estimate of
    on-chip cached params for yolo_1 variants (no compile log)."""
    txt = os.path.splitext(tflite_path)[0] + ".txt"
    if not os.path.exists(txt): return None
    with open(txt, "r", errors="ignore") as f:
        text = f.read()
    m = re.search(r"([\d,]+)\s+parameters", text)
    if m:
        return int(m.group(1).replace(",", ""))
    return None

def parse_compile_log(path):
    on_chip = off_chip = ops = None
    if not path or not os.path.exists(path): return on_chip, off_chip, ops
    text = open(path, "r").read()
    m = re.search(r"On-chip memory used for caching model parameters:\s*([\d.]+)([KMG]i?B)", text)
    if m:
        v = float(m.group(1))
        u = m.group(2)
        if u.startswith("M"): v *= 1024
        elif u.startswith("G"): v *= 1024 * 1024
        on_chip = v
    m = re.search(r"Off-chip memory used for streaming uncached model parameters:\s*([\d.]+)\s*([KMGB])", text)
    if m:
        v = float(m.group(1)); u = m.group(2)
        if u == "M": v *= 1024
        elif u == "G": v *= 1024 * 1024
        elif u == "B": v /= 1024
        off_chip = v
    m = re.search(r"Total number of operations:\s*(\d+)", text)
    if m: ops = int(m.group(1))
    return on_chip, off_chip, ops

def extract_edgetpu_blob_bytes(tflite_path):
    """Return the size of the EdgeTPU executable blob in bytes.

    The EdgeTPU compiler emits ONE CUSTOM operator named
    `edgetpu-custom-op` whose `custom_options` field carries the
    entire EdgeTPU executable (parameters + instructions packed
    together with a small header).  That payload is what libedgetpu's
    `EdgeTpuExecutable` parses; it splits at runtime into:
      - the parameter blob, sent ONCE at first invoke and then cached
        on-chip (size reported as 'On-chip memory ...' in the compile log);
      - the instruction blob, sent EVERY invoke via SendInstructions.

    Per-invoke USB cost ≈ (custom_options size) − (on-chip params).  """
    import tflite
    with open(tflite_path, "rb") as f:
        buf = f.read()
    model = tflite.Model.GetRootAsModel(buf, 0)
    sg = model.Subgraphs(0)
    blob = 0
    for op_i in range(sg.OperatorsLength()):
        op = sg.Operators(op_i)
        oc = model.OperatorCodes(op.OpcodeIndex())
        cc = oc.CustomCode()
        cc_bytes = bytes(cc) if cc is not None else b""
        if cc_bytes.startswith(b"edgetpu"):
            ln = op.CustomOptionsLength()
            if ln > blob: blob = ln
    return blob if blob > 0 else None

def fmt_kb(b):
    if b is None: return "n/a"
    return f"{b/1024.0:.1f}KB"

def fmt_dim(shape):
    return "x".join(str(int(s)) for s in shape)

def bench(name, folder, override_path, override_pre):
    if override_path:
        edgetpu_tfl = override_path
        plain_tfl = override_pre
        log_compile = None
        # Try to find compile log near the override path
        base = os.path.splitext(override_path)[0]
        for cand in (base + "_edgetpu_compile.log",
                     base.replace("_edgetpu", "") + "_edgetpu_compile.log"):
            if os.path.exists(cand):
                log_compile = cand
                break
    else:
        edgetpu_tfl = find_one(folder, r"_edgetpu\.tflite$")
        plain_tfl = find_one(folder, r"_int8\.tflite$") or \
                    find_one(folder, r"_uint8.*\.tflite$")
        log_compile = find_one(folder, r"_edgetpu_compile\.log$")
    if not edgetpu_tfl:
        print(f"[{name}] missing tflite"); return None

    on_chip_kb, off_chip_kb, ops_count = parse_compile_log(log_compile)
    plain_size = os.path.getsize(plain_tfl) if plain_tfl and os.path.exists(plain_tfl) else None
    edgetpu_size = os.path.getsize(edgetpu_tfl)

    # EdgeTPU custom_options blob = parameters + instructions, packed.
    custom_buf = extract_edgetpu_blob_bytes(edgetpu_tfl)

    # Bench
    interp = edgetpu.make_interpreter(edgetpu_tfl)
    interp.allocate_tensors()
    in_det = interp.get_input_details()[0]
    out_det = interp.get_output_details()
    in_shape = in_det["shape"]
    if np.dtype(in_det["dtype"]).kind == "u":
        x = np.random.randint(0, 256, size=tuple(in_shape), dtype=in_det["dtype"])
    elif np.dtype(in_det["dtype"]).kind == "i":
        x = np.random.randint(-128, 128, size=tuple(in_shape), dtype=in_det["dtype"])
    else:
        x = np.random.rand(*tuple(in_shape)).astype(in_det["dtype"])
    interp.set_tensor(in_det["index"], x)
    for _ in range(5): interp.invoke()
    N = 100
    samples_us = []
    for _ in range(N):
        t0 = time.perf_counter_ns()
        interp.invoke()
        samples_us.append((time.perf_counter_ns() - t0) / 1000.0)
    samples_us.sort()
    median_ms = samples_us[N // 2] / 1000.0
    p99_ms = samples_us[int(N * 0.99)] / 1000.0
    fps = 1000.0 / median_ms if median_ms > 0 else 0.0

    outs = ", ".join(f"{fmt_dim(o['shape'])}/{np.dtype(o['dtype']).name}"
                     for o in out_det)

    # Per-invoke USB = custom_buf - cached_params (on_chip * 1024).
    # Source 1: compile log (precise).  Source 2: yolov5 .txt param
    # count (estimate, marked with ~).
    per_invoke = None
    per_invoke_estimated = False
    if custom_buf is not None and on_chip_kb is not None:
        per_invoke = custom_buf - int(on_chip_kb * 1024)
        if per_invoke < 0: per_invoke = None
    elif custom_buf is not None:
        nparams = parse_param_count_txt(edgetpu_tfl)
        if nparams is not None:
            # int8 quant: 1 byte per param, +small alignment overhead.
            est_params = nparams
            per_invoke = custom_buf - est_params
            if per_invoke > 0:
                per_invoke_estimated = True
                # Also fill on_chip_kb for table display.
                on_chip_kb = est_params / 1024.0

    return {
        "name": name,
        "input": fmt_dim(in_shape) + "/" + np.dtype(in_det["dtype"]).name,
        "outputs": outs,
        "tflite_kb": edgetpu_size / 1024.0,
        "pre_kb": (plain_size / 1024.0) if plain_size else None,
        "on_chip_kb": on_chip_kb,
        "ops": ops_count,
        "custom_buf_kb": (custom_buf / 1024.0) if custom_buf else None,
        "per_invoke_kb": (per_invoke / 1024.0) if per_invoke is not None else None,
        "per_invoke_est": per_invoke_estimated,
        "median_ms": median_ms,
        "p99_ms": p99_ms,
        "fps": fps,
    }

results = []
for tup in MODELS:
    print(f"\n=== {tup[0]} ===", flush=True)
    r = bench(*tup)
    if r:
        for k, v in r.items():
            print(f"  {k:>16s}: {v}")
        results.append(r)

print("\n" + "=" * 152)
hdr = f"{'Model':<42} {'Input':<18} {'TFlite':>9} {'CustOp':>9} {'OnChip':>9} {'Per-inv':>9} {'Ops':>5} {'Med ms':>8} {'p99':>7} {'FPS':>6}"
print(hdr); print("=" * 152)
for r in results:
    print(f"{r['name']:<42} "
          f"{r['input']:<18} "
          f"{r['tflite_kb']:>7.1f}KB "
          f"{(fmt_kb(r['custom_buf_kb']*1024) if r['custom_buf_kb'] else 'n/a'):>9} "
          f"{(fmt_kb(r['on_chip_kb']*1024) if r['on_chip_kb'] else 'n/a'):>9} "
          f"{((('~' if r['per_invoke_est'] else '') + fmt_kb(r['per_invoke_kb']*1024)) if r['per_invoke_kb'] else 'n/a'):>9} "
          f"{(str(r['ops']) if r['ops'] is not None else 'n/a'):>5} "
          f"{r['median_ms']:>7.2f} "
          f"{r['p99_ms']:>6.2f} "
          f"{r['fps']:>5.1f}")
