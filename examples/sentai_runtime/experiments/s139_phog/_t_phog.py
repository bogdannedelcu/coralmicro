# s139 — PHOG descriptor SIM smoke test.
#
# Verifies algebraic properties of sentai.places.compute_phog on 3
# synthetic images.  Pass criteria — see README.md.

import sentai

P = sentai.places
PASS = 0
FAIL = 0
W, H = 64, 64
BINS = 8
N_CELLS = 1 + 4 + 16
DIM = BINS * N_CELLS  # 168

def chk(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  %s" % label)
    else:
        FAIL += 1
        print("  FAIL  %s" % label)

def mk_uniform(val=128):
    return bytes([val] * (W * H))

def mk_vertical_edges():
    # Left half = 0, right half = 255 → vertical edge at x=W/2.
    buf = [0] * (W * H)
    half = W // 2
    for y in range(H):
        for x in range(W):
            if x >= half:
                buf[y * W + x] = 255
    return bytes(buf)

def mk_horizontal_edges():
    # Top half = 0, bottom half = 255 → horizontal edge at y=H/2.
    buf = [0] * (W * H)
    half = H // 2
    for y in range(half, H):
        for x in range(W):
            buf[y * W + x] = 255
    return bytes(buf)

def sum_per_bin(desc):
    """Sum energy per orientation bin across all cells."""
    s = [0.0] * BINS
    for cell in range(N_CELLS):
        for b in range(BINS):
            s[b] += desc[cell * BINS + b]
    return s

def cell_l2(desc, cell_idx):
    """L2 norm of a single cell histogram."""
    s2 = 0.0
    for b in range(BINS):
        v = desc[cell_idx * BINS + b]
        s2 += v * v
    return s2 ** 0.5

print("=== s139 PHOG SIM smoke ===")

# ---------- Surface ----------
print("[1] Module surface")
chk(hasattr(P, "compute_phog"), "places.compute_phog present")

# ---------- Uniform image: no gradient ----------
print("[2] Uniform gray → no gradient")
d1 = P.compute_phog(mk_uniform(128), W, H)
chk(d1 is not None, "compute_phog returns non-None for valid input")
chk(len(d1) == DIM, "len == %d" % DIM)
chk(all(v == v for v in d1), "no NaN in descriptor")  # NaN != NaN
chk(all(v >= 0.0 for v in d1), "all non-negative")
# Uniform image: every L2 norm should be 0 (no gradient at all).
all_zero = sum(abs(v) for v in d1) < 1e-3
chk(all_zero, "uniform image → all-zero descriptor (sum < 1e-3)")

# ---------- Vertical edges → bin 0 dominant ----------
print("[3] Vertical edges → horizontal gradient peak (bin 0)")
d2 = P.compute_phog(mk_vertical_edges(), W, H)
chk(d2 is not None and len(d2) == DIM, "len ok")
sb = sum_per_bin(d2)
print("     per-bin energy: " + " ".join("%.2f" % v for v in sb))
peak_bin = sb.index(max(sb))
chk(peak_bin == 0 or peak_bin == BINS - 1,
    "peak bin in {0, BINS-1} (got %d) — vertical edges → horizontal gradient" % peak_bin)
# L2 norm of L0 cell should be ≈ 1
l2_l0 = cell_l2(d2, 0)
chk(0.9 < l2_l0 < 1.1, "L0 cell L2 norm ≈ 1 (got %.3f)" % l2_l0)

# ---------- Horizontal edges → bin BINS/2 dominant ----------
print("[4] Horizontal edges → vertical gradient peak (~bin BINS/2)")
d3 = P.compute_phog(mk_horizontal_edges(), W, H)
chk(d3 is not None and len(d3) == DIM, "len ok")
sb = sum_per_bin(d3)
print("     per-bin energy: " + " ".join("%.2f" % v for v in sb))
peak_bin = sb.index(max(sb))
chk(BINS // 2 - 1 <= peak_bin <= BINS // 2 + 1,
    "peak bin ≈ BINS/2 (got %d) — horizontal edges → vertical gradient" % peak_bin)

# ---------- All cells either ~0 or ≈ 1 (L2-normalized) ----------
print("[5] L2 normalization invariant per cell")
ok_norm = True
for c in range(N_CELLS):
    nrm = cell_l2(d2, c)
    if not (nrm < 1e-3 or 0.9 < nrm < 1.1):
        ok_norm = False
        print("     FAIL cell %d L2 = %.3f" % (c, nrm))
        break
chk(ok_norm, "every cell L2 ∈ {0} ∪ [0.9, 1.1]")

# ---------- Invalid input rejected ----------
print("[6] Invalid input rejected")
chk(P.compute_phog(b"\x00\x00\x00", 1, 1) is None,
    "tiny image (w<8) → None")
chk(P.compute_phog(b"\x00" * 10, 0, 10) is None,
    "w=0 → None")
chk(P.compute_phog(b"\x00" * 5, 10, 10) is None,
    "buffer < w*h → None")

# ---------- Determinism ----------
print("[7] Determinism")
d4 = P.compute_phog(mk_vertical_edges(), W, H)
match = all(abs(d4[i] - d2[i]) < 1e-6 for i in range(DIM))
chk(match, "same input → identical descriptor")

print("=== s139 PHOG: %d PASS / %d FAIL ===" % (PASS, FAIL))
if FAIL == 0:
    print("VERDICT: PASS")
else:
    print("VERDICT: FAIL")
