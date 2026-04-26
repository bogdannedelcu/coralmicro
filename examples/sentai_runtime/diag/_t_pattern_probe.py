# _t_pattern_probe.py — verify OV5640 synthetic patterns produce
# RECOGNIZABLE STRUCTURAL signatures, regardless of ISP scaling
# (AWB/CMX/Gamma all in default auto mode).
#
# Modes:
#   0 = OFF   (real scene)
#   1 = BARS  (0x503D = 0x80) — 8 vertical color bars
#             First row of pixels has 7-8 abrupt B-channel
#             transitions across the 640-pixel width
#   2 = HBAND (0x503D = 0x88) — horizontal gradient (gradual
#             change at horizontal): each row is UNIFORM color,
#             gradient is row-to-row.  First row = constant,
#             0-1 transitions max.
#
# Discriminator (ISP-scaling invariant):
#   transitions(row0) ≥ 5  →  BARS
#   transitions(row0) ≤ 1  →  HBAND
#   else                    →  real scene / ambiguous
import sentai
sentai.verbose(1)


def hex_dump(b, n=32):
    return " ".join("%02X" % x for x in b[:n])


def count_transitions(buf, threshold=24, n_pixels=160):
    """Count B-channel jumps > threshold across first n_pixels.
    XRGB8888 layout: B[0] G[1] R[2] X[3] per pixel."""
    transitions = 0
    prev_b = buf[0]
    for i in range(1, n_pixels):
        cur_b = buf[i * 4]
        if abs(cur_b - prev_b) > threshold:
            transitions += 1
        prev_b = cur_b
    return transitions


def row_variance(buf, n_pixels=160):
    """Mean + max-min of B channel across first n_pixels."""
    bs = [buf[i * 4] for i in range(n_pixels)]
    avg = sum(bs) // n_pixels
    return avg, max(bs) - min(bs)


print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

PASS = []
FAIL = []


def sample_8_bars(buf):
    """Return B-channel sample at the 8 bar centres (pixels
    40, 120, 200, 280, 360, 440, 520, 600).  Each bar in the
    OV5640 standard pattern is W/8 = 80 px wide at VGA640."""
    centers = [40, 120, 200, 280, 360, 440, 520, 600]
    return [buf[c * 4] for c in centers]


def probe(cam_id, mode, label, expect_sig):
    """expect_sig: 'BARS' | 'HBAND' | 'ANY'"""
    sentai.camera.test_pattern(cam_id, mode)
    sentai.camera.select(cam_id)
    sentai.rtos.sleep_ms(400)
    for _ in range(4):
        b = sentai.camera.peek_row(640 * 4)  # full first row
    bars = sample_8_bars(b)                  # B at each bar centre
    # count distinct values across the 8 bar centres
    distinct = len(set(bars))
    trans640 = count_transitions(b, threshold=24, n_pixels=640)
    avg, span = row_variance(b, n_pixels=640)
    if expect_sig == "BARS":
        # BARS should show >= 4 distinct B values across the 8 bars
        # (8 colors collapse to fewer in B channel, but at least 4)
        verdict = "PASS" if distinct >= 4 else "FAIL"
    elif expect_sig == "HBAND":
        # HBAND row uniform → all 8 samples the same (or near-same)
        verdict = "PASS" if distinct <= 2 and span < 0x40 else "FAIL"
    else:
        verdict = "info"
    print("  cam%d %-10s mode=%d  trans640=%d  distinct8=%d  avg=0x%02X  span=0x%02X  %s" %
          (cam_id, label, mode, trans640, distinct, avg, span, verdict))
    print("    8 bar centres B: " + " ".join("%02X" % v for v in bars))
    if verdict == "PASS": PASS.append(label + "/cam%d" % cam_id)
    elif verdict == "FAIL": FAIL.append(label + "/cam%d" % cam_id)


print("--- step 1: real scene baseline ---")
probe(0, 0, "real", "ANY")
probe(1, 0, "real", "ANY")

print("--- step 2: cam0=BARS, cam1=HBAND ---")
probe(0, 1, "BARS", "BARS")
probe(1, 2, "HBAND", "HBAND")

print("--- step 3: swap (cam0=HBAND, cam1=BARS) ---")
probe(0, 2, "HBAND", "HBAND")
probe(1, 1, "BARS", "BARS")

print("--- step 4: restore real-scene ---")
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)

print("--- summary ---")
print("  PASS=%d  FAIL=%d" % (len(PASS), len(FAIL)))
if FAIL:
    print("  failed cases:", FAIL)
print("=== done ===")
