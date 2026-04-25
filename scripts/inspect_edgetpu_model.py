#!/usr/bin/env python3
"""Inspect the structure of an EdgeTPU-compiled .tflite model.

Shows:
  - Input/output tensor shapes + dtype + quantization
  - Each executable (EXEC_ONLY / PARAM_CACHING / STAND_ALONE)
  - Compiled instruction chunks (size + patch fields per chunk)
  - DMA hint breakdown
  - Per-invoke USB footprint estimate (how many bytes get uploaded per invoke
    for instructions, input, params, output)

Usage:
    scripts/inspect_edgetpu_model.py path/to/model.tflite

Requires venv with tensorflow + flatbuffers:
    venv/bin/python scripts/inspect_edgetpu_model.py models/foo.tflite
"""
import argparse, os, struct, sys
from tensorflow.lite.python import schema_py_generated as sch
from flatbuffers.table import Table
import flatbuffers.flexbuffers as flex

TENSOR_TYPE = {0:'F32', 1:'F16', 2:'I32', 3:'U8', 4:'I64',
               7:'I16', 9:'I8', 10:'CPLX64', 16:'UI16', 17:'UI32'}
EXE_TYPE = {0:'STAND_ALONE', 1:'PARAM_CACHING', 2:'EXEC_ONLY'}

def parse_custom_blob(path):
    with open(path, 'rb') as f:
        buf = bytearray(f.read())
    m = sch.Model.GetRootAsModel(buf, 0)
    sg = m.Subgraphs(0)
    blob = None
    for i in range(sg.OperatorsLength()):
        op = sg.Operators(i)
        oc = m.OperatorCodes(op.OpcodeIndex())
        if oc.CustomCode() and oc.CustomCode().decode() == 'edgetpu-custom-op':
            n = op.CustomOptionsLength()
            blob = bytes(op.CustomOptions(k) for k in range(n))
            break
    return m, sg, blob

def dump_tensors(sg):
    for role, idxs in (('INPUT', [sg.Inputs(i) for i in range(sg.InputsLength())]),
                       ('OUTPUT',[sg.Outputs(i) for i in range(sg.OutputsLength())])):
        for idx in idxs:
            t = sg.Tensors(idx)
            shape = [t.Shape(k) for k in range(t.ShapeLength())]
            q = t.Quantization()
            s = [q.Scale(k) for k in range(q.ScaleLength())] if q else []
            z = [q.ZeroPoint(k) for k in range(q.ZeroPointLength())] if q else []
            name = t.Name().decode() if t.Name() else ''
            print(f'{role:6} [{idx}] {name!r}  shape={shape}  type={TENSOR_TYPE.get(t.Type(),t.Type())}  scale={s}  zp={z}')

def dump_exe(exe_bytes, eid):
    exe = bytearray(exe_bytes)
    root_off = struct.unpack_from('<I', exe, 0)[0]
    t = Table(exe, root_off)
    def vlen(slot):
        o = t.Offset(4 + 2*slot); return t.VectorLen(o) if o else 0
    def voff(slot):
        o = t.Offset(4 + 2*slot); return t.Vector(o) if o else 0
    def fshort(slot):
        o = t.Offset(4 + 2*slot); return struct.unpack_from('<h', exe, t.Pos + o)[0] if o else None
    def fint(slot):
        o = t.Offset(4 + 2*slot); return struct.unpack_from('<i', exe, t.Pos + o)[0] if o else None
    n_ins = vlen(5); v_off = voff(5)
    t_enum = fshort(13)
    typename = EXE_TYPE.get(t_enum, f'?{t_enum}')
    n_params = vlen(6)
    print(f'  exe[{eid}] type={typename}  scratch={fint(4)} B  instruction_chunks={n_ins}  params_bytes={n_params}')
    sizes = []
    for i in range(n_ins):
        elem_pos = v_off + 4*i
        ib_off = elem_pos + struct.unpack_from('<I', exe, elem_pos)[0]
        ib = Table(exe, ib_off)
        bs_len = ib.VectorLen(ib.Offset(4+2*0)) if ib.Offset(4+2*0) else 0
        n_fo = ib.VectorLen(ib.Offset(4+2*1)) if ib.Offset(4+2*1) else 0
        sizes.append((bs_len, n_fo))
    tot = sum(s[0] for s in sizes)
    totf = sum(s[1] for s in sizes)
    for i, (b, f) in enumerate(sizes):
        print(f'    chunk[{i}] bitstream={b:>9} B  patch_fields={f}')
    if sizes:
        print(f'    TOTAL bitstreams = {tot} B  ({tot//1024} KB)   patch_fields={totf}')
    # DMA hints
    dh_off_slot = t.Offset(4+2*7)
    if dh_off_slot:
        dh_pos = t.Indirect(t.Pos + dh_off_slot)
        dh_tab = Table(exe, dh_pos)
        h_off = dh_tab.Offset(4+2*0)
        if h_off:
            n_hints = dh_tab.VectorLen(h_off)
            v = dh_tab.Vector(h_off)
            cnt = {1:0,2:0,3:0,4:0}
            for i in range(n_hints):
                pos = v + 4*i
                ho = pos + struct.unpack_from('<I', exe, pos)[0]
                ht = Table(exe, ho)
                tt_off = ht.Offset(4+2*0)
                tt = struct.unpack_from('<B', exe, ht.Pos + tt_off)[0] if tt_off else 0
                cnt[tt] = cnt.get(tt,0) + 1
            print(f'    dma_hints total={n_hints}  DmaDesc={cnt.get(1,0)}  Instr={cnt.get(2,0)}  Int={cnt.get(3,0)}  Fence={cnt.get(4,0)}')
    return typename, tot, n_params

def dump_package(blob):
    root = flex.GetRoot(blob)
    m = root.AsMap
    keys = [m.Keys[i].AsKey for i in range(len(m.Keys))]
    print(f'  flex map keys: {keys}  (EdgeTPU convention: 4=Package)')
    v = m['4']
    pkg_bytes = bytes(v.AsBlob.Bytes) if v.IsBlob else bytes(v.AsStringBytes)
    pkg = bytearray(pkg_bytes)
    p_root = struct.unpack_from('<I', pkg, 0)[0]
    pt = Table(pkg, p_root)
    sme_o = pt.Offset(4+2*1)
    sme_len = pt.VectorLen(sme_o)
    sme_off = pt.Vector(sme_o)
    print(f'  Package: serialized_multi_executable = {sme_len} B')
    mex = bytearray(pkg[sme_off:sme_off+sme_len])
    mr = struct.unpack_from('<I', mex, 0)[0]
    mt = Table(mex, mr)
    n_exe = mt.VectorLen(mt.Offset(4+2*0))
    vo = mt.Vector(mt.Offset(4+2*0))
    print(f'  MultiExecutable: {n_exe} executables')
    summaries = []
    for i in range(n_exe):
        pos = vo + 4*i
        s_off = pos + struct.unpack_from('<I', mex, pos)[0]
        s_len = struct.unpack_from('<I', mex, s_off)[0]
        exe_bytes = bytes(mex[s_off+4 : s_off+4+s_len])
        summaries.append(dump_exe(exe_bytes, i))
    return summaries

def main():
    ap = argparse.ArgumentParser(description='Inspect an EdgeTPU .tflite structure')
    ap.add_argument('model', help='path to .tflite (edgetpu-compiled)')
    a = ap.parse_args()
    if not os.path.exists(a.model):
        print(f'Not found: {a.model}'); sys.exit(1)
    file_sz = os.path.getsize(a.model)
    print(f'===== {a.model} =====')
    print(f'File size: {file_sz} B  ({file_sz//1024} KB)')
    _, sg, blob = parse_custom_blob(a.model)
    dump_tensors(sg)
    if not blob:
        print('No edgetpu-custom-op found — this model is not EdgeTPU-compiled.')
        return
    print(f'custom-op blob: {len(blob)} B')
    summaries = dump_package(blob)
    # Summary for per-invoke USB footprint
    ins_per_invoke = 0; params_once = 0
    for typ, tot, p in summaries:
        if typ == 'EXEC_ONLY' or typ == 'STAND_ALONE':
            ins_per_invoke += tot
        if typ == 'PARAM_CACHING' or typ == 'STAND_ALONE':
            params_once += p
    # Input tensor size
    input_size = 0
    for i in range(sg.InputsLength()):
        t = sg.Tensors(sg.Inputs(i))
        sh = [t.Shape(k) for k in range(t.ShapeLength())]
        sz = 1
        for d in sh: sz *= d
        # int8/uint8 = 1 B; float32 = 4 B; int16 = 2 B
        el = {9:1, 3:1, 7:2, 2:4, 0:4, 1:2}.get(t.Type(), 1)
        input_size += sz * el
    print(f'\nEstimated per-invoke USB footprint:')
    print(f'  instructions (re-sent every invoke) : {ins_per_invoke:>10} B  ({ins_per_invoke//1024} KB)')
    print(f'  input activations                   : {input_size:>10} B  ({input_size//1024} KB)')
    print(f'  parameters uploaded once at load    : {params_once:>10} B  ({params_once//1024} KB)')

if __name__ == '__main__':
    main()
