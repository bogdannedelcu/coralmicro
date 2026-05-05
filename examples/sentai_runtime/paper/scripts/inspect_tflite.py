#!/usr/bin/env python3
"""inspect_tflite.py - Linux-side inspection of an EdgeTPU-compiled .tflite.

Reports the byte budget per top-level region: TF Lite metadata,
operator custom_options (= EdgeTPU darwinn::ExecutablePackage), and
buffer data.  Used to sanity-check what would need to fit in
on-board OCRAM if we wanted to relocate the per-invoke USB Bulk-OUT
sources (Params / Instructions / Inputs) off SDRAM.

Sub-fields of the EdgeTPU custom_options (parameter_executable +
inference_executable, each with parameters[] and
instruction_bitstreams[]) are stored as a darwinn flatbuffer that
needs the libedgetpu schema to fully decode.  This script reports
the OUTER size; for per-component sizes run the model on the board
and read sentai.diag.tpu_call_stats() — see paper/edgetpu_sizing.md.

Usage (host):
    python3 paper/scripts/inspect_tflite.py <path/to/model.tflite>

Requires: tensorflow (any 2.x) for the TF Lite Python schema.
The `venv-coral/` virtualenv at the repo root has it pinned.
"""
import os, sys, importlib

if len(sys.argv) < 2:
    print(__doc__); sys.exit(2)
path = sys.argv[1]
if not os.path.exists(path):
    print(f"Not found: {path}"); sys.exit(1)

try:
    sch = importlib.import_module("tensorflow.lite.python.schema_py_generated")
except ImportError as e:
    print("Cannot import TF Lite schema. Activate venv-coral/ first:")
    print("  source venv-coral/bin/activate")
    print(f"Error: {e}")
    sys.exit(1)

with open(path, "rb") as f: buf = bytearray(f.read())
total = len(buf)
m = sch.Model.GetRootAsModel(buf, 0)

print(f"=== {os.path.basename(path)} ===")
print(f"File size: {total:>10,} bytes ({total/1024/1024:.2f} MB)")
print(f"Subgraphs: {m.SubgraphsLength()}")

# Operator codes
print("\nOperator codes:")
for i in range(m.OperatorCodesLength()):
    oc = m.OperatorCodes(i)
    cc = oc.CustomCode()
    cc_str = bytes(cc).decode() if cc else f"builtin#{oc.BuiltinCode()}"
    print(f"  [{i}] {cc_str}")

# Subgraph 0 operators — find edgetpu-custom-op and report size
sg = m.Subgraphs(0)
print(f"\nSubgraph[0] operators: {sg.OperatorsLength()}")
edgetpu_total = 0
for op_i in range(sg.OperatorsLength()):
    op = sg.Operators(op_i)
    oc = m.OperatorCodes(op.OpcodeIndex())
    cc = oc.CustomCode()
    name = bytes(cc).decode() if cc else f"builtin#{oc.BuiltinCode()}"
    co_len = op.CustomOptionsLength()
    if cc and b"edgetpu" in bytes(cc):
        edgetpu_total += co_len
        print(f"  [{op_i}] {name}  custom_options = {co_len:,} bytes "
              f"(~{co_len/1024:.1f} KB)")
        print(f"        ↑ contains darwinn ExecutablePackage flatbuffer:")
        print(f"          parameter_executable + inference_executable")
        print(f"          each with parameters[] + instruction_bitstreams[].")
        print(f"          Per-component sizes require running the model")
        print(f"          on-board and reading sentai.diag.tpu_call_stats().")

# Buffers (TF Lite-side weights — usually empty for fully-edgetpu-compiled
# models because everything is in custom_options).
total_buf = 0
nb = 0
for i in range(m.BuffersLength()):
    b = m.Buffers(i)
    if b.DataLength():
        total_buf += b.DataLength()
        nb += 1
print(f"\nTF Lite buffers with data: {nb}, total {total_buf:,} bytes")

# Recap
print(f"\nRecap:")
print(f"  EdgeTPU custom_options total: {edgetpu_total:>10,} bytes "
      f"({edgetpu_total/total*100:.1f}% of file)")
print(f"  TF Lite buffer data:          {total_buf:>10,} bytes")
print(f"  TF Lite metadata + structure: {total - edgetpu_total - total_buf:>10,} bytes")
