# _t_pattern_init.py — high-speed switch test with ground-truth
# verification via synthetic patterns:
#   cam0 ← BARS  (0x503D = 0x80) — uniform 8 vertical color bars
#   cam1 ← HBAND (0x503D = 0x88) — bars + horizontal brightness gradient
#
# Run alt 1:1 for N=100 captures.  For each grabbed frame:
#   - read row 0, sample B byte at col 40 (WHITE bar centre)
#   - classify pattern: BARS if B≥0xA0, HBAND if B≤0x80
#   - compare classification with cam_id tag (sentai.camera.grabbed_id)
#
# IMPORTANT: do NOT write to LFS or print during the capture loop —
# both perturb timing.  All per-iteration records are kept in RAM and
# flushed to a single LFS log file ONLY at the very end.
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


print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

# Set patterns ONCE at init, never toggle during the run.
print("--- pattern init: cam0=BARS, cam1=HBAND ---")
sentai.camera.test_pattern(0, 1)
sentai.camera.test_pattern(1, 2)

# Warmup: visit each cam so pattern settles.  Use sleep ONLY (no
# peek_row drain — that triggered drain-timeout when CSI receiver
# was settling pattern injection).
sentai.camera.select(0); sentai.rtos.sleep_ms(500)
sentai.camera.select(1); sentai.rtos.sleep_ms(500)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
print("  patterns applied + warmed up")

# Engage alt 1:1 for the high-speed test.
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

N = 100
records = []                                # (i, cam, seen, b40) — RAM only
print("--- alt 1:1, %d frames (silent) ---" % N)
for i in range(N):
    b = sentai.camera.peek_row(200)         # ≥ 50 px
    cam = sentai.camera.grabbed_id()
    b40 = b[40 * 4]                          # B at col 40 (in WHITE bar)
    if b40 >= 0xA0:
        seen = "BARS"
    elif b40 <= 0x80:
        seen = "HBAND"
    else:
        seen = "AMBIG"
    records.append((i, cam, seen, b40))
    if (i % 10) == 0:
        sentai.diag.repl_kick()

# Restore camera state BEFORE LFS work (so any LFS hiccup doesn't
# leave the camera in alt-1:1).
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)

# Tally
passed = 0
failed = 0
ambig = 0
unknown_tag = 0
mismatches = []
cam0 = {"BARS": 0, "HBAND": 0, "AMBIG": 0}
cam1 = {"BARS": 0, "HBAND": 0, "AMBIG": 0}
for (i, cam, seen, b40) in records:
    if cam == 0:
        cam0[seen] += 1
        expected = "BARS"
    elif cam == 1:
        cam1[seen] += 1
        expected = "HBAND"
    else:
        unknown_tag += 1
        continue
    if seen == "AMBIG":
        ambig += 1
    elif seen == expected:
        passed += 1
    else:
        failed += 1
        if len(mismatches) < 10:
            mismatches.append((i, cam, seen, b40))

print("--- summary ---")
print("  passed=%d failed=%d ambig=%d unknown_tag=%d  total=%d" %
      (passed, failed, ambig, unknown_tag, N))
print("  cam0:  BARS=%d HBAND=%d AMBIG=%d" %
      (cam0["BARS"], cam0["HBAND"], cam0["AMBIG"]))
print("  cam1:  BARS=%d HBAND=%d AMBIG=%d" %
      (cam1["BARS"], cam1["HBAND"], cam1["AMBIG"]))
if mismatches:
    print("  first mismatches: %s" % mismatches[:5])

# Single LFS write at the very end — full log so host can download
# and audit the per-frame correspondence.
sd = _session_dir("pattern_init")
log_path = sd + "/log.csv"
lines = ["i,cam_id,seen,b40\n"]
for (i, cam, seen, b40) in records:
    lines.append("%d,%d,%s,0x%02X\n" % (i, cam, seen, b40))
lines.append("# summary\n")
lines.append("# passed=%d failed=%d ambig=%d unknown_tag=%d total=%d\n" %
             (passed, failed, ambig, unknown_tag, N))
lines.append("# cam0 BARS=%d HBAND=%d AMBIG=%d\n" %
             (cam0["BARS"], cam0["HBAND"], cam0["AMBIG"]))
lines.append("# cam1 BARS=%d HBAND=%d AMBIG=%d\n" %
             (cam1["BARS"], cam1["HBAND"], cam1["AMBIG"]))
sentai.fs.write(log_path, "".join(lines))
print("log saved:", log_path)
print("=== done ===")
