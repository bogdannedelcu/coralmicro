# _t_pattern_31.py — diagnose tag/content inversion in alt mode.
#
# Builds on _t_pattern_init.py but adds two diagnostic phases:
#
#   PHASE A (single-cam baseline, no MUX flip)
#     cam0 ← BARS  (0x503D = 0x80) — col-40 B byte ≈ 0xFF (WHITE bar)
#     cam1 ← HBAND (0x503D = 0x88) — col-40 B byte ≈ 0x1F (dark band)
#     select(0); 10 frames; tally BARS/HBAND  → expect 10×BARS
#     select(1); 10 frames; tally BARS/HBAND  → expect 10×HBAND
#     If this is wrong, the firmware's label-vs-content mapping is
#     STATICALLY inverted (test_pattern routing or MUX wiring), and
#     the alt-mode tag is fine.
#
#   PHASE B (alt 3:1, dynamic switching)
#     ratio(3, 1); 100 frames; tally per (cam_id tag, content).
#     Expected at 3:1 if firmware is correct:
#       cam0 tag count ≈ 75 → 75×BARS, 0×HBAND
#       cam1 tag count ≈ 25 →  0×BARS, 25×HBAND
#     If we instead see ~75×cam0_tag with mostly HBAND, the bug is
#     in the dynamic tag association at MUX flip — and the proportion
#     should track the parity, just inverted.
#
# All bookkeeping in RAM.  Single LFS write at the very end with the
# full per-frame log so a host can audit if needed.
import sentai
sentai.verbose(1)


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _classify(b40):
    if b40 >= 0xA0: return "BARS"
    if b40 <= 0x80: return "HBAND"
    return "AMBIG"


# Build #935 — single-dequeue 5-row sampling.  peek5_b40 dequeues
# ONE buffer, samples B-channel at col 40 from rows
# {0, H/4, H/2, 3H/4, H-1}, returns (cam_tag, b0..b4).  Earlier
# pattern_31 had a bug: it called peek_row_at five times, which
# dequeued FIVE different buffers — in alt-mode those buffers came
# from different cameras, producing FALSE "scrambled" detections.
# This single-grab API gives a true mid-frame-mix signal.
def _classify_5b(samples5):
    """samples5 = (b0,b1,b2,b3,b4) from the SAME buffer.
    Return ('BARS'|'HBAND'|'AMBIG'|'SCRAMBLED', row0_class)."""
    classes = tuple(_classify(b) for b in samples5)
    row0 = classes[0]
    if all(c == "BARS" for c in classes):  return ("BARS", row0)
    if all(c == "HBAND" for c in classes): return ("HBAND", row0)
    if all(c == "AMBIG" for c in classes): return ("AMBIG", row0)
    return ("SCRAMBLED", row0)


print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

# Set patterns ONCE at init.
print("--- pattern init: cam0=BARS, cam1=HBAND ---")
sentai.camera.test_pattern(0, 1)
sentai.camera.test_pattern(1, 2)

# Warm-up: visit each camera once so the pattern register settles.
sentai.camera.select(0); sentai.rtos.sleep_ms(500)
sentai.camera.select(1); sentai.rtos.sleep_ms(500)

# ─────────────────── PHASE A: single-cam baseline ───────────────────
phaseA = []                                  # (sel, cam_tag, frame_class, row0_class, b40_row0, samples5)
print("--- PHASE A: single-cam baseline (single-grab 5-row) ---")
for sel in (0, 1):
    sentai.camera.select(sel)
    sentai.rtos.sleep_ms(300)
    # drain a few stale frames
    for _ in range(3):
        sentai.camera.peek_row(8)
    for _ in range(10):
        r = sentai.camera.peek5_b40()  # (tag, b0,b1,b2,b3,b4)
        cam = r[0]
        samples5 = r[1:6]
        frame_cls, row0_cls = _classify_5b(samples5)
        phaseA.append((sel, cam, frame_cls, row0_cls, samples5[0], samples5))
    sentai.diag.repl_kick()

# ─────────────────── PHASE B: alt 3:1 + drain=1 (baseline) ────────
sentai.camera.ratio(3, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

NB = 100
phaseB = []  # (i, cam_tag, frame_class, row0_class, samples5)
ram_jpegs = []   # [(i, cam_tag, samples5, jpeg_bytes)]
SCRAMBLED_TO_SAVE = 4
print("--- PHASE B: alt 3:1, %d frames (silent, single-grab 5-row) ---" % NB)
for i in range(NB):
    r = sentai.camera.peek5_b40()
    cam = r[0]
    samples5 = r[1:6]
    frame_cls, row0_cls = _classify_5b(samples5)
    phaseB.append((i, cam, frame_cls, row0_cls, samples5))
    if frame_cls == "SCRAMBLED" and len(ram_jpegs) < SCRAMBLED_TO_SAVE:
        try:
            jpeg = sentai.camera.jpeg(80)
            ram_jpegs.append((i, cam, samples5, jpeg))
        except Exception:
            pass
    if (i % 10) == 0:
        sentai.diag.repl_kick()

# Restore camera state BEFORE the LFS write so a hiccup can't strand us.
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)

# ─────────────────── tallies + summary ──────────────────────────────
_BUCKETS = ("BARS", "HBAND", "AMBIG", "SCRAMBLED")

def _empty_bucket():
    return {k: 0 for k in _BUCKETS}

def _tally_phaseA(records):
    out = {}
    for rec in records:
        sel, cam, frame_cls = rec[0], rec[1], rec[2]
        key = "sel=%d" % sel
        d = out.setdefault(key, dict(_empty_bucket(),
                                      tag0=0, tag1=0, tag_other=0))
        d[frame_cls] += 1
        if cam == 0: d["tag0"] += 1
        elif cam == 1: d["tag1"] += 1
        else: d["tag_other"] += 1
    return out

def _tally_phaseB(records):
    cam0 = _empty_bucket()
    cam1 = _empty_bucket()
    other = 0
    scrambled_total = 0
    for (_i, cam, frame_cls, _row0_cls, _b) in records:
        if frame_cls == "SCRAMBLED": scrambled_total += 1
        if cam == 0: cam0[frame_cls] += 1
        elif cam == 1: cam1[frame_cls] += 1
        else: other += 1
    return cam0, cam1, other, scrambled_total

print("--- summary PHASE A (single-cam baseline, 5-row sampled) ---")
A = _tally_phaseA(phaseA)
for k in sorted(A.keys()):
    d = A[k]
    print("  %s  BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d   tags: cam0=%d cam1=%d other=%d" %
          (k, d["BARS"], d["HBAND"], d["AMBIG"], d["SCRAMBLED"],
           d["tag0"], d["tag1"], d["tag_other"]))

try:
    fs = sentai.camera.flip_stats()
    print("--- ISR + flip stats (build #933+) ---")
    print("  vblank_flips=%d deferred=%d streak=%d forced=%d" % fs[:4])
    print("  isr_count=%d  us last=%d  max=%d  avg=%d" % fs[4:8])
    print("  isr histogram us:  <50=%d 50-99=%d 100-199=%d 200-499=%d 500-999=%d >=1000=%d" % fs[8:14])
    print("  skip_both (IRQ-delivery jitter, both flags set at entry) = %d" % fs[14])
except AttributeError:
    pass

print("--- summary PHASE B (alt 3:1, 5-row sampled) ---")
b_cam0, b_cam1, b_other, b_scrambled = _tally_phaseB(phaseB)
def _sum_bucket(d): return d["BARS"]+d["HBAND"]+d["AMBIG"]+d["SCRAMBLED"]
print("  total tag distribution: cam0=%d cam1=%d other=%d  | scrambled total=%d" %
      (_sum_bucket(b_cam0), _sum_bucket(b_cam1), b_other, b_scrambled))
print("  cam0 tag → BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d" %
      (b_cam0["BARS"], b_cam0["HBAND"], b_cam0["AMBIG"], b_cam0["SCRAMBLED"]))
print("  cam1 tag → BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d" %
      (b_cam1["BARS"], b_cam1["HBAND"], b_cam1["AMBIG"], b_cam1["SCRAMBLED"]))

# Single LFS write at the very end.
sd = _session_dir("pattern_31")
log_path = sd + "/log.csv"
parts = ["phase,sel_or_i,cam_tag,frame_class,row0_class,b40_row0,b40_q1,b40_mid,b40_q3,b40_last\n"]
for (sel, cam, fc, r0c, _b0, s5) in phaseA:
    parts.append("A,%d,%d,%s,%s,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n" %
                 (sel, cam, fc, r0c, s5[0], s5[1], s5[2], s5[3], s5[4]))
for (i, cam, fc, r0c, s5) in phaseB:
    parts.append("B,%d,%d,%s,%s,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n" %
                 (i, cam, fc, r0c, s5[0], s5[1], s5[2], s5[3], s5[4]))
parts.append("# PHASE A summary\n")
for k in sorted(A.keys()):
    d = A[k]
    parts.append("# %s BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d tag0=%d tag1=%d other=%d\n" %
                 (k, d["BARS"], d["HBAND"], d["AMBIG"], d["SCRAMBLED"],
                  d["tag0"], d["tag1"], d["tag_other"]))
parts.append("# PHASE B summary (alt 3:1)\n")
parts.append("# cam0_tag BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d\n" %
             (b_cam0["BARS"], b_cam0["HBAND"], b_cam0["AMBIG"], b_cam0["SCRAMBLED"]))
parts.append("# cam1_tag BARS=%d HBAND=%d AMBIG=%d SCRAMBLED=%d\n" %
             (b_cam1["BARS"], b_cam1["HBAND"], b_cam1["AMBIG"], b_cam1["SCRAMBLED"]))
parts.append("# tag_other=%d  scrambled_total=%d\n" % (b_other, b_scrambled))
sentai.fs.write(log_path, "".join(parts))
print("log saved:", log_path)

# Drain RAM-buffered scrambled JPEGs to LFS for host inspection.
for (i, cam, s5, jpeg) in ram_jpegs:
    fname = "%s/scr_i%d_cam%d_b%02X-%02X-%02X-%02X-%02X.jpg" % (
        sd, i, cam, s5[0], s5[1], s5[2], s5[3], s5[4])
    try:
        sentai.fs.write(fname, jpeg)
        print("scrambled jpeg saved:", fname)
    except Exception as e:
        print("scrambled jpeg save FAILED:", fname, e)

print("=== done ===")
