#!/usr/bin/env python3
"""Render a matplotlib figure that estimates per-invoke USB time + FPS
for each model in the inventory, calibrated from the two real
measurements we have today:

  yolo_1 512×512 : invoke_ms ≈ 13,  cyc_input 4.3 ms / 811 KB → 188 MB/s
                                    cyc_ins   2.3 ms / 371 KB → 161 MB/s
                                    TPU silicon compute ≈  6 ms
  yolo26 768×512 : invoke_ms ≈ 90,  cyc_input 6.8 ms / 1202 KB → 177 MB/s
                                    cyc_ins   8.1 ms / 1257 KB → 155 MB/s
                                    TPU silicon compute ≈ 75 ms

Effective throughputs average ~180 MB/s (input path) and ~158 MB/s (ins
path) — these are the numbers EHCI achieves with QTD pipelining at
36 KB chunk size.  Params + output + event sum to ~0.3 ms for small
outputs so we treat it as a flat add-on.

TPU compute is not estimable from static .tflite fields alone (it's
silicon-bound and depends on actual FLOPs per conv).  We show two
bounds:
  - USB-only floor: what invoke would be if compute were zero
  - Calibrated-compute: linear fit compute ≈ k × ins_KB, k chosen to
    hit the yolo_1 + yolo26 measurements (halfway, since only 2 pts)

Usage:
    venv/bin/python scripts/plot_model_perf.py --out paper/model_perf.png
"""
import argparse, os, struct, glob, sys
from tensorflow.lite.python import schema_py_generated as sch
from flatbuffers.table import Table
import flatbuffers.flexbuffers as flex
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

TENSOR_TYPE = {0:'F32', 1:'F16', 2:'I32', 3:'U8', 4:'I64',
               7:'I16', 9:'I8', 10:'CPLX64', 16:'UI16', 17:'UI32'}
EXE_TYPE = {0:'STANDALONE', 1:'PARAM_CACHING', 2:'EXEC_ONLY'}
DTYPE_BYTES = {9:1, 3:1, 7:2, 2:4, 0:4, 1:2}

# Calibration constants measured on device 2026-04-24
USB_IN_MB_S  = 180.0   # input activations effective throughput
USB_INS_MB_S = 158.0   # instructions effective throughput
USB_FIXED_MS = 0.3     # params+output+event floor (small models)

# Calibrated compute model: compute_ms ≈ k_compute × ins_KB
# yolo_1:  371 KB ins, compute ≈  6 ms   → k ≈ 0.0162
# yolo26: 1228 KB ins, compute ≈ 75 ms   → k ≈ 0.0611
# Take geometric mean as a mid-estimate; plot bands instead of a line.
K_COMPUTE_LOW  = 0.016   # mobilenet-like (lean ops)
K_COMPUTE_HIGH = 0.061   # yolo-like (deep, many channels)

# Real measurement anchors
MEASURED = {
    'yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite': 13.0,
    'yolo26_736_512_1_class_1_downsample_2conv.tflite': 90.0,
}


def parse_custom_blob(path):
    with open(path, 'rb') as f: buf = bytearray(f.read())
    m = sch.Model.GetRootAsModel(buf, 0)
    sg = m.Subgraphs(0)
    blob = None
    for i in range(sg.OperatorsLength()):
        op = sg.Operators(i)
        oc = m.OperatorCodes(op.OpcodeIndex())
        if oc.CustomCode() and oc.CustomCode().decode() == 'edgetpu-custom-op':
            n = op.CustomOptionsLength()
            blob = bytes(op.CustomOptions(k) for k in range(n)); break
    return sg, blob

def input_shape_bytes(sg):
    if sg.InputsLength() == 0: return ([], 0, '?')
    t = sg.Tensors(sg.Inputs(0))
    sh = [t.Shape(k) for k in range(t.ShapeLength())]
    sz = 1
    for d in sh: sz *= max(d, 1)
    return sh, sz * DTYPE_BYTES.get(t.Type(), 1), TENSOR_TYPE.get(t.Type(),'?')

def parse_exe(exe_bytes):
    exe = bytearray(exe_bytes)
    ro = struct.unpack_from('<I', exe, 0)[0]
    t = Table(exe, ro)
    def vlen(slot):
        o = t.Offset(4+2*slot); return t.VectorLen(o) if o else 0
    def voff(slot):
        o = t.Offset(4+2*slot); return t.Vector(o) if o else 0
    def fshort(slot):
        o = t.Offset(4+2*slot); return struct.unpack_from('<h', exe, t.Pos + o)[0] if o else None
    n_ins = vlen(5); v = voff(5)
    total = 0
    for i in range(n_ins):
        pos = v + 4*i
        ib_off = pos + struct.unpack_from('<I', exe, pos)[0]
        ib = Table(exe, ib_off)
        bs = ib.VectorLen(ib.Offset(4+2*0)) if ib.Offset(4+2*0) else 0
        total += bs
    return EXE_TYPE.get(fshort(13), '?'), total, vlen(6)

def parse_pkg(blob):
    r = flex.GetRoot(blob).AsMap
    keys = [r.Keys[i].AsKey for i in range(len(r.Keys))]
    if '4' not in keys: return []
    v = r['4']
    pb = bytes(v.AsBlob.Bytes) if v.IsBlob else bytes(v.AsStringBytes)
    pkg = bytearray(pb)
    pr = struct.unpack_from('<I', pkg, 0)[0]
    pt = Table(pkg, pr)
    sme_o = pt.Offset(4+2*1)
    sme_len = pt.VectorLen(sme_o); sme_off = pt.Vector(sme_o)
    mex = bytearray(pkg[sme_off : sme_off+sme_len])
    mr = struct.unpack_from('<I', mex, 0)[0]
    mt = Table(mex, mr)
    n = mt.VectorLen(mt.Offset(4+2*0))
    vv = mt.Vector(mt.Offset(4+2*0))
    out = []
    for i in range(n):
        p = vv + 4*i
        so = p + struct.unpack_from('<I', mex, p)[0]
        sl = struct.unpack_from('<I', mex, so)[0]
        out.append(parse_exe(bytes(mex[so+4:so+4+sl])))
    return out

def inspect(path):
    sg, blob = parse_custom_blob(path)
    sh, in_b, dt = input_shape_bytes(sg)
    if not blob: return None  # CPU-only
    # Image models only: NHWC with 3 or 1 channels, spatial dims ≥ 32
    if len(sh) != 4 or sh[-1] not in (1, 3) or sh[1] < 32 or sh[2] < 32:
        return None
    exes = parse_pkg(blob)
    ins_total = sum(sz for (t, sz, _) in exes if t in ('EXEC_ONLY','STANDALONE'))
    if ins_total == 0: return None  # trivial
    return dict(
        name=os.path.basename(path),
        shape=sh, input_bytes=in_b, input_dtype=dt,
        ins_bytes=ins_total,
    )


def estimate(input_B, ins_B, k_compute):
    usb_in_ms  = input_B / (USB_IN_MB_S  * 1024 * 1024) * 1000
    usb_ins_ms = ins_B   / (USB_INS_MB_S * 1024 * 1024) * 1000
    usb_ms = usb_in_ms + usb_ins_ms + USB_FIXED_MS
    compute_ms = k_compute * (ins_B / 1024)
    total_ms = usb_ms + compute_ms
    fps = 1000.0 / total_ms if total_ms > 0 else 0.0
    return usb_ms, compute_ms, total_ms, fps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default='models/')
    ap.add_argument('--out', default='examples/sentai_runtime/paper/model_perf.png')
    a = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(a.folder, '*.tflite')))
    rows = []
    for p in paths:
        r = inspect(p)
        if r: rows.append(r)

    rows.sort(key=lambda r: r['input_bytes'] + r['ins_bytes'])

    names = [r['name'].replace('_edgetpu.tflite','').replace('.tflite','') for r in rows]
    in_kb = np.array([r['input_bytes']/1024 for r in rows])
    ins_kb = np.array([r['ins_bytes']/1024 for r in rows])
    usb_ms = np.array([estimate(r['input_bytes'], r['ins_bytes'], 0)[0] for r in rows])
    low_ms = np.array([estimate(r['input_bytes'], r['ins_bytes'], K_COMPUTE_LOW)[2]  for r in rows])
    high_ms = np.array([estimate(r['input_bytes'], r['ins_bytes'], K_COMPUTE_HIGH)[2] for r in rows])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # --- LEFT: per-model stacked time bars + measured anchors ---
    y = np.arange(len(rows))
    usb_in_ms = np.array([r['input_bytes']/(USB_IN_MB_S*1048576)*1000 for r in rows])
    usb_ins_ms = np.array([r['ins_bytes']/(USB_INS_MB_S*1048576)*1000 for r in rows])

    ax1.barh(y, usb_in_ms, color='#4C9FE0', label='USB input transfer')
    ax1.barh(y, usb_ins_ms, left=usb_in_ms, color='#E08A4C', label='USB instructions transfer')
    compute_low = low_ms - usb_in_ms - usb_ins_ms - USB_FIXED_MS
    compute_high = high_ms - usb_in_ms - usb_ins_ms - USB_FIXED_MS
    ax1.barh(y, compute_high - compute_low, left=low_ms,
             color='#8FCB8F', alpha=0.35, label=f'TPU compute range (k={K_COMPUTE_LOW}..{K_COMPUTE_HIGH} ms/KB)')
    ax1.barh(y, compute_low, left=usb_in_ms + usb_ins_ms + USB_FIXED_MS,
             color='#4C8F4C', alpha=0.5, label=f'TPU compute low estimate')

    # Real measurements
    for i, r in enumerate(rows):
        if r['name'] in MEASURED:
            m = MEASURED[r['name']]
            ax1.plot([m], [i], marker='D', color='red', markersize=10, zorder=5,
                     label='measured' if i == next(j for j,rr in enumerate(rows) if rr['name'] in MEASURED) else None)
            ax1.annotate(f'{m:.0f} ms', (m, i), xytext=(m+2, i), va='center', color='red', fontsize=9)

    ax1.set_yticks(y)
    ax1.set_yticklabels([n[:42] for n in names], fontsize=8)
    ax1.set_xlabel('estimated invoke time (ms)')
    ax1.set_title('Per-model invoke-time estimate\n(USB I/O calibrated @ ~180 MB/s input, ~158 MB/s ins)')
    ax1.legend(loc='lower right', fontsize=8)
    ax1.grid(axis='x', alpha=0.3)

    # --- RIGHT: scatter input × ins, colored by estimated total ms ---
    pts = ax2.scatter(in_kb, ins_kb, c=high_ms, s=80, cmap='viridis', edgecolors='black')
    cbar = plt.colorbar(pts, ax=ax2)
    cbar.set_label('estimated invoke ms (high compute)')

    # Annotate measured points
    for r in rows:
        if r['name'] in MEASURED:
            ax2.plot(r['input_bytes']/1024, r['ins_bytes']/1024, marker='D',
                     markersize=14, markerfacecolor='none', markeredgecolor='red', markeredgewidth=2)
            short = r['name'].replace('_edgetpu.tflite','').replace('.tflite','')[:20]
            ax2.annotate(f'{short}\n{MEASURED[r["name"]]:.0f} ms meas',
                         (r['input_bytes']/1024, r['ins_bytes']/1024),
                         xytext=(10, 10), textcoords='offset points', fontsize=8, color='red')

    # Iso-FPS contours from the USB-only floor (lower bound)
    in_range = np.linspace(10, max(in_kb)*1.1, 80)
    ins_range = np.linspace(10, max(ins_kb)*1.1, 80)
    X, Y = np.meshgrid(in_range, ins_range)
    Z_usb = X/(USB_IN_MB_S*1024)*1000 + Y/(USB_INS_MB_S*1024)*1000 + USB_FIXED_MS  # ms using KB
    cs = ax2.contour(X, Y, 1000.0/Z_usb, levels=[5,10,20,30,50,75,100,150],
                     colors='gray', alpha=0.45, linestyles='dashed')
    ax2.clabel(cs, fmt='%d FPS', fontsize=8)

    ax2.set_xlabel('input activations (KB)')
    ax2.set_ylabel('ins bytes per invoke (KB)')
    ax2.set_title('Model landscape: input × ins  (color = est. invoke ms)\n'
                  'gray dashed = USB-only FPS ceiling (no compute)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(a.out, dpi=140, bbox_inches='tight')
    print(f'wrote {a.out}', file=sys.stderr)

if __name__ == '__main__':
    main()
