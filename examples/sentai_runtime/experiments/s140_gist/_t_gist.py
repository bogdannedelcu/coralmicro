# s140 — GIST-lite descriptor SIM smoke.
# Verifies algebraic properties of sentai.places.compute_gist on synthetic images.

import sentai

P = sentai.places
PASS = 0
FAIL = 0
W, H = 64, 64
ORIENT = 4        # DX, DY, D45, D135
CELLS  = 16       # 4×4 grid
DIM    = ORIENT * CELLS  # 64

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
    """Left half = 0, right half = 255 → strong DX response."""
    buf = [0] * (W * H)
    half = W // 2
    for y in range(H):
        for x in range(half, W):
            buf[y * W + x] = 255
    return bytes(buf)

def mk_horizontal_edges():
    """Top half = 0, bottom half = 255 → strong DY response."""
    buf = [0] * (W * H)
    half = H // 2
    for y in range(half, H):
        for x in range(W):
            buf[y * W + x] = 255
    return bytes(buf)

def mk_diagonal_nwse():
    """Triangle below diagonal y > x = 255 → strong D45 response.
    D45 = 2*(p22 - p00): NW dark, SE bright → positive D45.
    """
    buf = [0] * (W * H)
    for y in range(H):
        for x in range(W):
            if y > x:
                buf[y * W + x] = 255
    return bytes(buf)

def orient_total(desc, o):
    """Sum of energy across all 16 cells for orientation o."""
    s = 0.0
    for c in range(CELLS):
        s += desc[o * CELLS + c]
    return s

print("=== s140 GIST-lite SIM smoke ===")

print("[1] Module surface")
chk(hasattr(P, "compute_gist"), "places.compute_gist present")

print("[2] Uniform gray → no gradient")
d1 = P.compute_gist(mk_uniform(128), W, H)
chk(d1 is not None, "non-None for valid input")
chk(len(d1) == DIM, "len == %d" % DIM)
chk(all(v == v for v in d1), "no NaN")
chk(all(v >= 0.0 for v in d1), "non-negative")
chk(sum(abs(v) for v in d1) < 1e-3, "uniform → all-zero descriptor")

print("[3] Vertical edges → DX dominant (orientation 0)")
d2 = P.compute_gist(mk_vertical_edges(), W, H)
chk(d2 is not None and len(d2) == DIM, "len ok")
totals = [orient_total(d2, o) for o in range(ORIENT)]
labels = ["DX", "DY", "D45", "D135"]
print("     totals: " + " ".join("%s=%.3f" % (l, t) for l, t in zip(labels, totals)))
peak = totals.index(max(totals))
chk(peak == 0, "DX is peak orientation (idx=0)")
chk(totals[0] > 0.5 * (totals[1] + totals[2] + totals[3] + 0.01),
    "DX clearly dominates")

print("[4] Horizontal edges → DY dominant (orientation 1)")
d3 = P.compute_gist(mk_horizontal_edges(), W, H)
totals = [orient_total(d3, o) for o in range(ORIENT)]
print("     totals: " + " ".join("%s=%.3f" % (l, t) for l, t in zip(labels, totals)))
peak = totals.index(max(totals))
chk(peak == 1, "DY is peak orientation (idx=1)")

print("[5] Diagonal NW-SE edge → diagonal-orientation response present")
# An edge along the NW-SE diagonal has its GRADIENT perpendicular to
# the edge (NE-SW direction = D135 in this code's naming).  So D135 is
# the expected peak, NOT D45 — gradients are perpendicular to edges.
d4 = P.compute_gist(mk_diagonal_nwse(), W, H)
totals = [orient_total(d4, o) for o in range(ORIENT)]
print("     totals: " + " ".join("%s=%.3f" % (l, t) for l, t in zip(labels, totals)))
# Diagonal images have non-trivial response in all 4 orientations.
# Key check: ratio between diagonal responses must reflect edge sense.
chk(totals[3] > totals[2] * 1.05, "D135 > D45 (gradient perpendicular to NW-SE edge)")
chk(totals[3] > 1.0, "D135 has substantial response to NW-SE edge")

print("[6] L2 norm per CELL across 4 orientations ≈ 1")
ok_norm = True
for c in range(CELLS):
    s2 = sum(d2[o*CELLS + c] ** 2 for o in range(ORIENT))
    n = s2 ** 0.5
    if not (n < 1e-3 or 0.9 < n < 1.1):
        ok_norm = False
        print("     FAIL cell %d L2 = %.3f" % (c, n))
        break
chk(ok_norm, "every cell's 4-orient vector L2 ∈ {0} ∪ [0.9, 1.1]")

print("[7] Determinism")
d5 = P.compute_gist(mk_vertical_edges(), W, H)
match = all(abs(d5[i] - d2[i]) < 1e-6 for i in range(DIM))
chk(match, "same input → identical descriptor")

print("[8] Invalid input rejected")
chk(P.compute_gist(b"\x00" * 10, 1, 1) is None, "w<16 → None")
chk(P.compute_gist(b"\x00" * 5, 64, 64) is None, "buffer < w*h → None")

print("=== s140 GIST-lite: %d PASS / %d FAIL ===" % (PASS, FAIL))
print("VERDICT: %s" % ("PASS" if FAIL == 0 else "FAIL"))
