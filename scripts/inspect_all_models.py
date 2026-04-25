#!/usr/bin/env python3
"""Batch-inspect all .tflite models in a folder and emit a markdown table.

Usage:
    venv/bin/python scripts/inspect_all_models.py models/ > paper/models.md
"""
import argparse, os, struct, sys, glob
from tensorflow.lite.python import schema_py_generated as sch
from flatbuffers.table import Table
import flatbuffers.flexbuffers as flex

TENSOR_TYPE = {0:'F32', 1:'F16', 2:'I32', 3:'U8', 4:'I64',
               7:'I16', 9:'I8', 10:'CPLX64', 16:'UI16', 17:'UI32'}
EXE_TYPE = {0:'STANDALONE', 1:'PARAM_CACHING', 2:'EXEC_ONLY'}
DTYPE_BYTES = {9:1, 3:1, 7:2, 2:4, 0:4, 1:2}

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

def tensor_info(sg):
    def tdesc(idx):
        t = sg.Tensors(idx)
        sh = [t.Shape(k) for k in range(t.ShapeLength())]
        return {
            'shape': sh,
            'dtype': TENSOR_TYPE.get(t.Type(), str(t.Type())),
            'bytes': _bytes_of(sh, t.Type()),
        }
    ins = [tdesc(sg.Inputs(i)) for i in range(sg.InputsLength())]
    outs = [tdesc(sg.Outputs(i)) for i in range(sg.OutputsLength())]
    return ins, outs

def _bytes_of(shape, tf_type):
    sz = 1
    for d in shape: sz *= max(d, 1)
    return sz * DTYPE_BYTES.get(tf_type, 1)

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
    chunks = []
    for i in range(n_ins):
        pos = v + 4*i
        ib_off = pos + struct.unpack_from('<I', exe, pos)[0]
        ib = Table(exe, ib_off)
        bs = ib.VectorLen(ib.Offset(4+2*0)) if ib.Offset(4+2*0) else 0
        chunks.append(bs)
    # dma_hints
    dh = t.Offset(4+2*7)
    n_instr = n_dma = 0
    if dh:
        dht = Table(exe, t.Indirect(t.Pos + dh))
        h_off = dht.Offset(4+2*0)
        if h_off:
            nh = dht.VectorLen(h_off)
            vo = dht.Vector(h_off)
            for i in range(nh):
                p = vo + 4*i
                ho = p + struct.unpack_from('<I', exe, p)[0]
                ht = Table(exe, ho)
                to = ht.Offset(4+2*0)
                tt = struct.unpack_from('<B', exe, ht.Pos + to)[0] if to else 0
                if tt == 2: n_instr += 1
                elif tt == 1: n_dma += 1
    return {
        'type': EXE_TYPE.get(fshort(13), f'?{fshort(13)}'),
        'ins_chunks': chunks,
        'ins_total': sum(chunks),
        'params_bytes': vlen(6),
        'n_instr_hints': n_instr,
        'n_dma_hints': n_dma,
    }

def parse_package(blob):
    root = flex.GetRoot(blob)
    m = root.AsMap
    # EdgeTPU convention: key '4' -> Package flatbuffer
    keys = [m.Keys[i].AsKey for i in range(len(m.Keys))]
    if '4' not in keys:
        return []
    v = m['4']
    pkg_bytes = bytes(v.AsBlob.Bytes) if v.IsBlob else bytes(v.AsStringBytes)
    pkg = bytearray(pkg_bytes)
    pr = struct.unpack_from('<I', pkg, 0)[0]
    pt = Table(pkg, pr)
    sme_o = pt.Offset(4+2*1)
    sme_len = pt.VectorLen(sme_o)
    sme_off = pt.Vector(sme_o)
    mex = bytearray(pkg[sme_off : sme_off+sme_len])
    mr = struct.unpack_from('<I', mex, 0)[0]
    mt = Table(mex, mr)
    n = mt.VectorLen(mt.Offset(4+2*0))
    v = mt.Vector(mt.Offset(4+2*0))
    out = []
    for i in range(n):
        p = v + 4*i
        so = p + struct.unpack_from('<I', mex, p)[0]
        sl = struct.unpack_from('<I', mex, so)[0]
        exe_bytes = bytes(mex[so+4 : so+4+sl])
        out.append(parse_exe(exe_bytes))
    return out

def inspect(path):
    try:
        m, sg, blob = parse_custom_blob(path)
    except Exception as e:
        return {'error': str(e), 'file_bytes': os.path.getsize(path)}
    file_bytes = os.path.getsize(path)
    ins_tensors, out_tensors = tensor_info(sg)
    row = {
        'file_bytes': file_bytes,
        'inputs': ins_tensors,
        'outputs': out_tensors,
        'has_edgetpu_op': blob is not None,
        'exes': [],
    }
    if blob:
        try:
            row['exes'] = parse_package(blob)
        except Exception as e:
            row['parse_error'] = str(e)
    return row

def fmt_kb(b):
    return f'{b//1024}' if b else '0'

def fmt_shape(sh):
    return '×'.join(str(d) for d in sh) if sh else '?'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder')
    ap.add_argument('--out', help='write to file instead of stdout')
    a = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(a.folder, '*.tflite')))
    rows = []
    for p in paths:
        r = inspect(p)
        r['name'] = os.path.basename(p)
        rows.append(r)

    # Format markdown
    lines = []
    lines.append(f'# TFLite model inventory — `{a.folder}`')
    lines.append('')
    lines.append(f'Generated by `scripts/inspect_all_models.py`. Inspected {len(rows)} models.')
    lines.append('')
    lines.append('Columns:')
    lines.append('')
    lines.append('- **File** — total .tflite file size')
    lines.append('- **Input** — input tensor shape (NHWC or similar) + dtype')
    lines.append('- **Output(s)** — output tensor shape(s)')
    lines.append('- **Params (1×)** — weights cached on TPU at load time (from PARAM_CACHING executable; "n/a" when STAND_ALONE)')
    lines.append('- **Ins/invoke** — compiled TPU bytecode streamed every invoke (from EXEC_ONLY or STAND_ALONE executable). Dominant USB-I/O cost.')
    lines.append('- **Chunks** — number of instruction bitstream chunks the compiler emits (~260 KB cap per chunk)')
    lines.append('- **Hints** — dma_hints breakdown (`D`ma desc + `I`nstruction)')
    lines.append('- **Input B/invoke** — input activations pushed every invoke (computed from tensor shape × dtype)')
    lines.append('')
    lines.append('| Model | File | Input | Output(s) | Params (1×) | Ins/invoke | Chunks | Hints D/I | Input B/invoke |')
    lines.append('|---|---:|---|---|---:|---:|---:|---|---:|')
    for r in rows:
        n = r['name']
        fb = r['file_bytes']
        if not r.get('has_edgetpu_op'):
            if 'error' in r:
                lines.append(f'| `{n}` | {fmt_kb(fb)} KB | *parse-error* | — | — | — | — | — | — |')
            else:
                # CPU-only tflite
                inp = r['inputs'][0] if r['inputs'] else {'shape':[], 'dtype':'?', 'bytes':0}
                out_strs = [f"{fmt_shape(t['shape'])} {t['dtype']}" for t in r['outputs']]
                lines.append(f'| `{n}` | {fmt_kb(fb)} KB | {fmt_shape(inp["shape"])} {inp["dtype"]} | {"; ".join(out_strs)} | *CPU-only* | — | — | — | {fmt_kb(inp["bytes"])} KB |')
            continue
        inp = r['inputs'][0] if r['inputs'] else {'shape':[], 'dtype':'?', 'bytes':0}
        outs = r['outputs']
        out_strs = [f"{fmt_shape(t['shape'])} {t['dtype']}" for t in outs]
        # Sum across EXEC_ONLY/STANDALONE for ins; PARAM_CACHING for params
        ins_total = sum(e['ins_total'] for e in r['exes'] if e['type'] in ('EXEC_ONLY','STANDALONE'))
        # Also include STANDALONE params if that's the form
        params_total = 0
        for e in r['exes']:
            if e['type'] == 'PARAM_CACHING' or e['type'] == 'STANDALONE':
                params_total += e['params_bytes']
        n_chunks = sum(len(e['ins_chunks']) for e in r['exes'] if e['type'] in ('EXEC_ONLY','STANDALONE'))
        n_instr_hints = sum(e['n_instr_hints'] for e in r['exes'] if e['type'] in ('EXEC_ONLY','STANDALONE'))
        n_dma_hints  = sum(e['n_dma_hints'] for e in r['exes'] if e['type'] in ('EXEC_ONLY','STANDALONE'))
        # Params display: if STANDALONE has params inlined, show that; if PARAM_CACHING is separate, show its bytes
        params_str = f'{fmt_kb(params_total)} KB' if params_total else '—'
        lines.append(f'| `{n}` | {fmt_kb(fb)} KB | {fmt_shape(inp["shape"])} {inp["dtype"]} | {"; ".join(out_strs)} | {params_str} | {fmt_kb(ins_total)} KB | {n_chunks} | {n_dma_hints}/{n_instr_hints} | {fmt_kb(inp["bytes"])} KB |')

    lines.append('')
    lines.append('## Per-chunk details')
    lines.append('')
    for r in rows:
        if not r.get('has_edgetpu_op'): continue
        n = r['name']
        lines.append(f'### `{n}`')
        lines.append('')
        for i, e in enumerate(r['exes']):
            lines.append(f'- `exe[{i}]` type=**{e["type"]}**  chunks={len(e["ins_chunks"])}  ins_total={fmt_kb(e["ins_total"])} KB  params={fmt_kb(e["params_bytes"])} KB  hints D/I={e["n_dma_hints"]}/{e["n_instr_hints"]}')
            if e['ins_chunks']:
                csv = ', '.join(str(b) for b in e['ins_chunks'])
                lines.append(f'  - chunk bytes: `[{csv}]`')
        lines.append('')

    out = '\n'.join(lines) + '\n'
    if a.out:
        with open(a.out, 'w') as f: f.write(out)
        print(f'Wrote {a.out}', file=sys.stderr)
    else:
        sys.stdout.write(out)

if __name__ == '__main__':
    main()
