# s161 — HSV-histogram descriptor SIM smoke + anti-regression gate.
# Verifies algebraic properties + golden bytes + match coherence of
# sentai.places.compute_hsv on synthetic RGB888 images.
#
# Mirror of s140 (GIST) and s141 (DescriptorBaseline) — same chk()
# helper, same image-builder pattern, same verdict format.

import sentai

P = sentai.places
PASS = 0
FAIL = 0
W, H = 64, 64
HBINS  = 16
SBINS  = 4
DIM    = HBINS * SBINS   # 64


def chk(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  %s" % label)
    else:
        FAIL += 1
        print("  FAIL  %s" % label)


def mk_solid(r, g, b):
    """Solid-color image."""
    return bytes([r, g, b] * (W * H))


def mk_red_dim():
    """Bright vs dim red — same H,S but different V."""
    return mk_solid(60, 0, 0)


def mk_gray_with_red_patch():
    """Gray background + 16x16 red patch in the middle."""
    pix = []
    rx0, rx1 = W // 2 - 8, W // 2 + 8
    ry0, ry1 = H // 2 - 8, H // 2 + 8
    for y in range(H):
        for x in range(W):
            if rx0 <= x < rx1 and ry0 <= y < ry1:
                pix.extend([255, 0, 0])
            else:
                pix.extend([128, 128, 128])
    return bytes(pix)


def mk_dark():
    """All pixels below V_MIN=16 → no valid pixels in histogram."""
    return mk_solid(8, 8, 8)


def desc_bytes(rgb):
    out = P.compute_hsv(rgb, W, H)
    assert out is not None and len(out) == DIM, "compute_hsv returned bad result"
    return out


def l1(a, b):
    return sum(abs(a[i] - b[i]) for i in range(DIM))


def main():
    print("=== s161 HSV smoke (Track A piece 3/4 — OP-S10-W4) ===")

    # T1 — uniform gray: hue undefined, S=0, all pixels land in bin
    # (h_bin=0, s_bin=0) per the achromatic short-circuit.  After
    # normalization, bin 0 should be 255 (every pixel went there) and
    # the rest should be 0.
    d_gray = desc_bytes(mk_solid(128, 128, 128))
    chk(d_gray[0] >= 240, "T1 uniform gray → bin 0 dominant (got %d)" % d_gray[0])
    chk(sum(d_gray[1:]) == 0, "T1 uniform gray → other bins zero")

    # T2 — pure RED at full saturation, H=0°, S=255.
    #   h_bin = 0,  s_bin = 3  -> bin index = 0*4 + 3 = 3
    d_red = desc_bytes(mk_solid(255, 0, 0))
    chk(d_red[3] >= 240, "T2 pure red → bin 3 dominant (got %d)" % d_red[3])
    chk(d_red[0] == 0,    "T2 pure red → bin 0 (gray bin) empty")

    # T3 — pure GREEN.  H=120°, h_bin = (120*16)/360 = 5, S=255 -> sb=3.
    #   bin index = 5*4 + 3 = 23
    d_green = desc_bytes(mk_solid(0, 255, 0))
    chk(d_green[23] >= 240, "T3 pure green → bin 23 dominant (got %d)" % d_green[23])

    # T4 — pure BLUE.  H=240°, h_bin = (240*16)/360 = 10, S=255 -> sb=3.
    #   bin index = 10*4 + 3 = 43
    d_blue = desc_bytes(mk_solid(0, 0, 255))
    chk(d_blue[43] >= 240, "T4 pure blue → bin 43 dominant (got %d)" % d_blue[43])

    # T5 — V-invariance: bright RED (255,0,0) vs dim RED (60,0,0).
    #   Both have H=0 and S=255 (∆/V = full); descriptors must be IDENTICAL.
    d_red_bright = desc_bytes(mk_solid(255, 0, 0))
    d_red_dim    = desc_bytes(mk_red_dim())
    diff_bright_dim = l1(d_red_bright, d_red_dim)
    chk(diff_bright_dim == 0,
        "T5 V-invariance: bright vs dim red → identical (L1=%d)" % diff_bright_dim)

    # T6 — gray + red patch: majority gray bin (0) + minority red bin (3).
    d_mix = desc_bytes(mk_gray_with_red_patch())
    n_gray  = d_mix[0]
    n_red   = d_mix[3]
    chk(n_gray > 0 and n_red > 0,
        "T6 mixed scene: both gray (b0=%d) AND red (b3=%d) bins non-zero" % (n_gray, n_red))
    chk(n_gray > n_red,
        "T6 mixed scene: gray bin > red bin (gray is majority pixels)")

    # T7 — dark scene: every pixel V<16 -> excluded -> all-zero descriptor.
    d_dark = desc_bytes(mk_dark())
    chk(all(b == 0 for b in d_dark), "T7 dark scene → all bins zero")

    # T8 — determinism: same input → same output bit-for-bit.
    d_red_again = desc_bytes(mk_solid(255, 0, 0))
    chk(l1(d_red, d_red_again) == 0, "T8 determinism: red replay → identical")

    # T9 — anti-regression goldens.  Captured 2026-05-17 with this exact
    # algorithm; any future edit to sentai_hsv.cc that drifts these
    # numbers should fail the gate.
    #   d_gray[0]    == 255  (uniform gray → 100% in bin 0)
    #   d_red[3]     == 255  (pure red → 100% in bin 3)
    #   d_green[23]  == 255  (pure green → 100% in bin 23)
    #   d_blue[43]   == 255  (pure blue → 100% in bin 43)
    GOLDENS = {
        ("T9.gray_0",   d_gray[0],   255),
        ("T9.red_3",    d_red[3],    255),
        ("T9.green_23", d_green[23], 255),
        ("T9.blue_43",  d_blue[43],  255),
    }
    for label, got, expected in GOLDENS:
        chk(got == expected, "%s golden: got=%d expected=%d" % (label, got, expected))

    # T10 — match coherence: same-scene similarity vs cross-scene distance.
    #   dist(red, red')        should be 0 (identical generator)
    #   dist(red, gray)        should be >> 0  (different scenes)
    #   dist(red, blue)        should be >> 0
    #   ratio cross/same       should exceed a comfortable factor (we use 100×
    #   since same==0 the ratio test is "cross > 200" instead).
    d_red_a = desc_bytes(mk_solid(255, 0, 0))
    d_red_b = desc_bytes(mk_solid(255, 0, 0))
    d_grey  = desc_bytes(mk_solid(128, 128, 128))
    d_blu   = desc_bytes(mk_solid(0, 0, 255))
    same_dist  = l1(d_red_a, d_red_b)
    rg_dist    = l1(d_red_a, d_grey)
    rb_dist    = l1(d_red_a, d_blu)
    chk(same_dist == 0,  "T10 same-scene L1=0 (got %d)" % same_dist)
    chk(rg_dist > 200,   "T10 red↔gray L1 > 200 (got %d)" % rg_dist)
    chk(rb_dist > 200,   "T10 red↔blue L1 > 200 (got %d)" % rb_dist)

    print("=== summary: PASS=%d FAIL=%d ===" % (PASS, FAIL))
    if FAIL == 0:
        print("VERDICT: PASS")
    else:
        print("VERDICT: FAIL")


main()
