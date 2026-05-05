# _t_three_slots.py — validate the multi-slot TPU API (Phase 1).
#
# Hypothesis to validate:
#   3 tflite::MicroInterpreter instances can coexist on the same
#   EdgeTpuContext on the SentAI board, each producing distinct
#   outputs from the same input — proving the firmware can keep 3
#   models simultaneously resident, mirroring the host pycoral result.
#
# Method:
#   1. Load 3 p3p4 candidates into slots 0, 1, 2.
#   2. Build a fixed deterministic input (size matches model input bytes).
#   3. set_input_slot + invoke_slot each, hash the outputs.
#   4. Confirm 3 distinct hashes → 3 different inference graphs running.
#   5. Alternate invokes (0,1,2,0,1,2,...) for 10 cycles, verify each
#      hash always matches that slot's expected hash.
#   6. Print PASS/FAIL.
#
# Per agent.md §5.1.2 + §5.1.5: self-contained, no diag/* imports,
# parameter list embedded in the file (no host seeding).
import sentai

A = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
B = ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
C = ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite")
SLOTS = [(0, A), (1, B), (2, C)]
ALT_CYCLES = 10
INPUT_BYTES = 480 * 640 * 3   # uint8[1,480,640,3] for all three


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


sess = _sd("three_slots")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "phase,slot,label,load_ms,invoke_ms,output_hash,expected_hash,ok\r\n")
print("=== session:", sess, " slot_count:", sentai.tpu.slot_count(), "===")

# Pipeline must NOT be running.
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

# 1) Load all 3 slots.
print("\n--- load 3 slots ---")
for slot, (label, path) in SLOTS:
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.tpu.load_slot(slot, path)
    ms = sentai.rtos.ticks_ms() - t0
    print("  slot=%d %s rc=%s in %d ms ready=%s" %
          (slot, label, rc, ms, sentai.tpu.slot_ready(slot)))
    sentai.fs.append(csv_path,
        "load,%d,%s,%d,,,,\r\n" % (slot, label, ms))

# Sanity: all slots ready.
all_ready = all(sentai.tpu.slot_ready(s) for s, _ in SLOTS)
if not all_ready:
    print("FAIL: not all slots loaded")
    print("=== done ==="); raise SystemExit

# 2) Skip explicit input injection.  MicroPython on this firmware has
#    a ~256-512 KB heap which CANNOT hold a 921 KB bytes object — both
#    `bytearray(N)` (not built into this MP config) and
#    `bytes(seed * N)` (alloc fails) are out.  Each slot's input
#    tensor is zero-initialized by AllocateTensors and untouched
#    across invokes, so the input is implicitly identical (all zeros)
#    across all 3 slots.  The MODELS have different weights, so
#    different outputs from the same input prove 3 different graphs
#    execute — same logic as the host pycoral verification.
print("\n--- input: zero-initialized arena (identical across slots) ---")
shared_input = None  # signal to skip set_input_slot below

# 3) Solo invokes: zero input → 3 different output hashes if 3
#    different models are running.
print("\n--- solo: invoke_slot per slot, hash outputs ---")
expected = {}
for slot, (label, _) in SLOTS:
    if shared_input is not None:
        rc = sentai.tpu.set_input_slot(slot, shared_input)
        if rc != 0:
            print("  slot=%d set_input_slot rc=%s — abort" % (slot, rc))
            print("=== done ==="); raise SystemExit
    ms = sentai.tpu.invoke_slot(slot)
    h = sentai.tpu.output_hash(slot)
    print("  slot=%d %s invoke=%d ms output_hash=0x%08x" % (slot, label, ms, h))
    expected[slot] = h
    sentai.fs.append(csv_path,
        "solo,%d,%s,,%d,0x%08x,0x%08x,1\r\n" % (slot, label, ms, h, h))

distinct = set(expected.values())
print("  distinct hashes: %d / 3" % len(distinct))
if len(distinct) != 3:
    print("FAIL: expected 3 distinct output hashes, got %d" % len(distinct))
    print("(slots may be aliasing the same model; investigate.)")
    print("=== done ==="); raise SystemExit
print("  PASS — 3 distinct outputs, 3 different models running")

# 4) Alternation: invoke 0,1,2,0,1,2,... for ALT_CYCLES, verify hash
#    always matches the slot's expected.  Re-set input each invoke so
#    nothing is left over from previous calls in any slot.
print("\n--- alternation: %d cycles × 3 slots ---" % ALT_CYCLES)
mismatches = 0
for cycle in range(ALT_CYCLES):
    for slot, (label, _) in SLOTS:
        if shared_input is not None:
            sentai.tpu.set_input_slot(slot, shared_input)
        ms = sentai.tpu.invoke_slot(slot)
        h = sentai.tpu.output_hash(slot)
        ok = (h == expected[slot])
        if not ok: mismatches += 1
        sentai.fs.append(csv_path,
            "alt,%d,%s,,%d,0x%08x,0x%08x,%d\r\n" %
            (slot, label, ms, h, expected[slot], 1 if ok else 0))
        if (cycle == 0) or (not ok):
            print("  cycle=%d slot=%d %s invoke=%d hash=0x%08x %s" %
                  (cycle, slot, label, ms, h, "OK" if ok else "FAIL"))
        if cycle == 0 and slot == 2:
            sentai.diag.repl_kick()

print("\n=== Verdict ===")
print("  Total invokes: %d × 3 = %d" % (ALT_CYCLES, ALT_CYCLES * 3))
print("  Mismatches: %d" % mismatches)
print("  Result: %s" % ("PASS" if mismatches == 0 else "FAIL"))
print("CSV:", csv_path)
print("=== done ===")
