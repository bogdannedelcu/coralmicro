# _t_iarna_inspect.py — inspect a TFLite model on-board.
#
# Loads /iarna_p2p4_5ep_export_640x480_uint8.tflite, prints:
#   - input dims/dtype/quant
#   - num_outputs, per-output dims/dtype/size/quant
#   - arena used (printed by firmware on load)
#   - per-call USB Bulk-OUT byte budgets (params/instructions/inputs)
#     via sentai.diag.tpu_call_stats() after a single invoke.
#
# Self-contained per agent.md §5.1.2.  No imports from diag/ siblings.
import sentai
sentai.verbose(1)

MODEL = "/iarna_p2p4_5ep_export_640x480_uint8.tflite"


def _sd(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    c = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(c, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


print("=== boot ===")
print("--- model:", MODEL, "---")

# Filesystem-side size (raw .tflite on flash)
try:
    fsize = sentai.fs.size(MODEL)
    print("file size :", fsize, "bytes")
except Exception as e:
    print("FAIL fs.size:", e); print("=== done ==="); raise SystemExit

# Load — firmware prints "Model loaded: N bytes", input/output schema,
# arena usage etc. via verbose path
rc = sentai.tpu.load(MODEL)
print("tpu.load  :", rc)
if rc != 0:
    print("FAIL load=%d" % rc); print("=== done ==="); raise SystemExit

# I/O introspection via MP API
print()
print("--- input ---")
try:
    print("input_type :", sentai.tpu.input_type())
except Exception as e:
    print("input_type FAIL:", e)
try:
    print("input_quant:", sentai.tpu.input_quant())
except Exception as e:
    print("input_quant FAIL:", e)

print()
print("--- outputs ---")
nout = sentai.tpu.num_outputs()
print("num_outputs:", nout)
for i in range(nout):
    try: dims = sentai.tpu.output_dims(i)
    except Exception as e: dims = "FAIL %s" % e
    try: typ  = sentai.tpu.output_type(i)
    except Exception as e: typ  = "FAIL %s" % e
    try: sz   = sentai.tpu.output_size(i)
    except Exception as e: sz   = "FAIL %s" % e
    try: q    = sentai.tpu.output_quant(i)
    except Exception as e: q    = "FAIL %s" % e
    print("  [%d] dims=%s type=%s size=%s quant=%s" % (i, dims, typ, sz, q))

# Run one invoke to populate call stats — input tensor has whatever
# was preloaded in OCRAM (zeros after fresh load); we don't care
# about output content here, just the byte budget on USB Bulk-OUT.
print()
print("--- invoke (1x for byte-budget probe) ---")
try:
    sentai.diag.tpu_call_stats(1)  # reset counters before probe
except Exception as e:
    print("tpu_call_stats reset FAIL:", e)

t0 = sentai.rtos.ticks_ms()
try:
    irc = sentai.tpu.invoke()
except Exception as e:
    irc = "exc %s" % e
t1 = sentai.rtos.ticks_ms()
print("invoke    :", irc, "in", (t1 - t0), "ms")

# Read post-invoke call stats — tuple of
# (param_calls, param_bytes, instr_calls, instr_bytes,
#  input_calls, input_bytes, input_data_bytes_or_dummy)
try:
    cs = sentai.diag.tpu_call_stats(0)
    print()
    print("--- tpu_call_stats (after 1 invoke) ---")
    print("raw       :", cs)
    if isinstance(cs, (list, tuple)) and len(cs) >= 6:
        pc, pb, ic, ib, inc, inb = cs[0], cs[1], cs[2], cs[3], cs[4], cs[5]
        print("Params       calls=%d total=%d bytes (avg %d B/call)" %
              (pc, pb, pb // max(pc, 1)))
        print("Instructions calls=%d total=%d bytes (avg %d B/call)" %
              (ic, ib, ib // max(ic, 1)))
        print("Inputs       calls=%d total=%d bytes (avg %d B/call)" %
              (inc, inb, inb // max(inc, 1)))
        if len(cs) >= 7:
            print("[6]          %d (extra)" % cs[6])
except Exception as e:
    print("tpu_call_stats read FAIL:", e)

# Persist to a session dir so the host can pull via curl
sd = _sd("iarna_inspect")
build_id = sentai.version().split("build")[1].split()[0] if "build" in sentai.version() else "?"
L = []
L.append("model,%s\n" % MODEL)
L.append("build_id,%s\n" % build_id)
L.append("file_size,%d\n" % fsize)
try:
    L.append("input_type,%s\n" % sentai.tpu.input_type())
    L.append("input_quant,%s\n" % str(sentai.tpu.input_quant()))
except Exception:
    pass
L.append("num_outputs,%d\n" % nout)
for i in range(nout):
    try:
        L.append("out%d,dims=%s,type=%s,size=%s,quant=%s\n" % (
            i,
            sentai.tpu.output_dims(i),
            sentai.tpu.output_type(i),
            sentai.tpu.output_size(i),
            str(sentai.tpu.output_quant(i)),
        ))
    except Exception:
        pass
try:
    cs = sentai.diag.tpu_call_stats(0)
    L.append("call_stats,%s\n" % str(cs))
except Exception:
    pass
sentai.fs.write(sd + "/info.csv", "".join(L))
print()
print("log:", sd + "/info.csv")
print("=== done ===")
