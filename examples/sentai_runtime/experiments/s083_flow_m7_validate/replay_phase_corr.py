#!/usr/bin/env python3
"""Phase-correlation replay over bulk_gray.bin.

Reads the same 80x60 gray frames as replay_full.py (firmware SAD), but
computes (dx, dy) via FFT-based phase correlation:
    R = F1 . conj(F2) / |F1 . conj(F2)|
    corr = IFFT(R)
    peak position -> (dx, dy)
    parabolic fit on 3x3 neighbourhood -> sub-pixel.

Output:
    /tmp/s083_phase.csv   per-frame dx_q1000 dy_q1000 conf phase side  (vs firmware)
    /tmp/s083_phase_traj.png   trajectory rendered on scene_start.jpg
    stdout: cumulative closure + per-frame max diff vs firmware
"""

import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

_HERE = Path(__file__).parent
BULK    = sys.argv[1] if len(sys.argv) > 1 else str(_HERE / "bulk_gray.bin")
OUT_CSV = sys.argv[2] if len(sys.argv) > 2 else str(_HERE / "phase_corr_per_frame.csv")
OUT_PNG = sys.argv[3] if len(sys.argv) > 3 else str(_HERE / "trajectory_phase_corr.png")

GW, GH = 80, 60
F = GW * GH

# Tukey window (cosine-tapered rectangle) instead of full Hann.
# Hann attenuates mid-frame content cosinusoidally over 100% of width,
# which kills the spectral signal of large motions (peak smears, drops
# below noise).  Tukey with alpha=0.25 keeps the central 75% flat at 1.0
# and tapers only the outer 12.5% on each side -- enough to suppress
# wrap-around bleed without sacrificing motion-spectrum content.
# Empirically recovers idx=92 motion (-6.1, -8.6) that Hann missed.
def tukey(n, alpha):
    """1-D Tukey window of length n with taper fraction alpha."""
    w = np.ones(n, dtype=np.float32)
    edge = int(alpha * (n - 1) / 2)
    if edge > 0:
        ramp = 0.5 * (1.0 + np.cos(np.pi * (np.arange(edge) / edge - 1.0)))
        w[:edge] = ramp.astype(np.float32)
        w[n - edge:] = ramp[::-1].astype(np.float32)
    return w

TUKEY_ALPHA = 0.25
T_Y = tukey(GH, TUKEY_ALPHA).reshape(GH, 1)
T_X = tukey(GW, TUKEY_ALPHA).reshape(1, GW)
WINDOW = (T_Y * T_X).astype(np.float32)


def parse_bulk(path):
    raw = open(path, "rb").read()
    i = raw.find(b"\n") + 1
    out = []
    while i < len(raw):
        nl = raw.find(b"\n", i)
        if nl < 0:
            break
        hdr = raw[i:nl].decode("ascii", errors="replace")
        i = nl + 1
        if i + F > len(raw):
            break
        gray = np.frombuffer(raw[i:i + F], dtype=np.uint8).reshape(GH, GW)
        i += F
        parts = hdr.split()
        if len(parts) < 9 or parts[0] != "F":
            continue
        out.append({
            "idx":  int(parts[1]),
            "t_ms": int(parts[2]),
            "seq":  int(parts[3]),
            "fw_dx_q1000": int(parts[4]),
            "fw_dy_q1000": int(parts[5]),
            "fw_conf":     int(parts[6]),
            "phase":       parts[7],
            "side":        int(parts[8]),
            "gray":        gray,
        })
    return out


def foroosh_q1000(a, b, c):
    """Sub-pixel offset for PHASE CORRELATION peak (Foroosh & Zerubia
    2002).  Phase-corr peak shape is a sharp delta plus side-lobes
    proportional to the actual sub-pixel shift -- closed-form:
        delta = +c/(b+c)   if c > a (peak shifted toward +1 sample)
        delta = -a/(a+b)   if a > c (peak shifted toward -1 sample)
    Both branches give bounded result in (-0.5, 0.5).  Robust where the
    classical parabolic fit fails because the peak isn't a parabola.
    Returns milli-grid-pixels (range [-500, 500])."""
    a = float(a); b = float(b); c = float(c)
    if b <= 0:
        return 0
    if c >= a:
        denom = b + c
        if denom <= 0: return 0
        d = c / denom
    else:
        denom = a + b
        if denom <= 0: return 0
        d = -a / denom
    # bounded; clamp paranoia
    if d > 0.5: d = 0.5
    if d < -0.5: d = -0.5
    return int(d * 1000.0)


def phase_correlate(prev_g, curr_g):
    """Return (dx_q1000, dy_q1000, peak_ratio).

    peak_ratio = peak_value / mean(|corr|) — proxy for confidence.
    Higher = sharper peak = more reliable.
    """
    p = (prev_g.astype(np.float32) - 128.0) * WINDOW
    c = (curr_g.astype(np.float32) - 128.0) * WINDOW

    F1 = np.fft.fft2(p)
    F2 = np.fft.fft2(c)
    # Cross-power spectrum.  conj(F2) so a forward shift in curr -> peak
    # at +dx (matches our SAD convention where dx>0 means curr moved
    # right vs prev).
    cross = F1 * np.conj(F2)
    cross /= (np.abs(cross) + 1e-10)

    corr = np.fft.ifft2(cross).real
    # corr is GH x GW with origin at (0,0); negative shifts wrap to high
    # indices.  fftshift to center origin at (GH/2, GW/2).
    corr_s = np.fft.fftshift(corr)

    py, px = np.unravel_index(int(np.argmax(corr_s)), corr_s.shape)
    cy0, cx0 = GH // 2, GW // 2
    dy_int = py - cy0
    dx_int = px - cx0

    # Sub-pixel parabolic refinement on the shifted correlation.
    delta_x = 0
    if 0 < px < GW - 1:
        a = corr_s[py, px - 1]
        b = corr_s[py, px]
        c2 = corr_s[py, px + 1]
        delta_x = foroosh_q1000(a, b, c2)
    delta_y = 0
    if 0 < py < GH - 1:
        a = corr_s[py - 1, px]
        b = corr_s[py, px]
        c2 = corr_s[py + 1, px]
        delta_y = foroosh_q1000(a, b, c2)

    dx_q1000 = dx_int * 1000 + delta_x
    dy_q1000 = dy_int * 1000 + delta_y

    # Confidence: peak height / mean.  Phase correlation peak is bounded
    # in [0,1] for a perfect shift; lower => more spectral disagreement.
    peak_val = float(corr_s[py, px])
    mean_abs = float(np.mean(np.abs(corr_s))) + 1e-10
    peak_ratio = peak_val / mean_abs  # typically 5-50 for clean shifts

    return dx_q1000, dy_q1000, peak_ratio


def main():
    print("loading frames...")
    t0 = time.time()
    frames = parse_bulk(BULK)
    print(f"  {len(frames)} frames in {time.time() - t0:.2f}s")

    print("running phase correlation on every consecutive pair...")
    t0 = time.time()
    out = []
    prev = frames[0]["gray"]
    for n, f in enumerate(frames[1:], start=1):
        dx_q, dy_q, conf_ratio = phase_correlate(prev, f["gray"])
        out.append({
            "idx":  f["idx"],
            "t_ms": f["t_ms"],
            "phase":f["phase"],
            "side": f["side"],
            "pc_dx_q1000": dx_q,
            "pc_dy_q1000": dy_q,
            "pc_conf":     conf_ratio,
            "fw_dx_q1000": f["fw_dx_q1000"],
            "fw_dy_q1000": f["fw_dy_q1000"],
            "fw_conf":     f["fw_conf"],
        })
        prev = f["gray"]
        if n % 100 == 0:
            print(f"  {n}/{len(frames)-1}  pc=({dx_q/1000:+.3f},{dy_q/1000:+.3f}) conf={conf_ratio:.1f}  "
                  f"fw=({f['fw_dx_q1000']/1000:+.3f},{f['fw_dy_q1000']/1000:+.3f}) conf={f['fw_conf']}")
    dt = time.time() - t0
    print(f"phase corr done in {dt:.2f}s  ({len(out)/dt:.0f} pairs/s)")

    fw_cx = sum(o["fw_dx_q1000"] for o in out) / 1000.0
    fw_cy = sum(o["fw_dy_q1000"] for o in out) / 1000.0
    pc_cx = sum(o["pc_dx_q1000"] for o in out) / 1000.0
    pc_cy = sum(o["pc_dy_q1000"] for o in out) / 1000.0

    print()
    print(f"=== Cumulative comparison over {len(out)} frame pairs ===")
    print(f"  FIRMWARE SAD     : cum_dx={fw_cx:+.3f} cum_dy={fw_cy:+.3f}  closure={(fw_cx**2+fw_cy**2)**0.5:.2f} gp")
    print(f"  PHASE CORR       : cum_dx={pc_cx:+.3f} cum_dy={pc_cy:+.3f}  closure={(pc_cx**2+pc_cy**2)**0.5:.2f} gp")

    diffs_x = [abs(o["fw_dx_q1000"] - o["pc_dx_q1000"]) for o in out]
    diffs_y = [abs(o["fw_dy_q1000"] - o["pc_dy_q1000"]) for o in out]
    print(f"  per-frame |fw_dx - pc_dx| max={max(diffs_x)}  mean={sum(diffs_x)/len(diffs_x):.0f} mgp")
    print(f"  per-frame |fw_dy - pc_dy| max={max(diffs_y)}  mean={sum(diffs_y)/len(diffs_y):.0f} mgp")

    # Frames where phase corr disagrees strongly with firmware
    disagree = sorted(out, key=lambda o:
                      abs(o["fw_dx_q1000"] - o["pc_dx_q1000"]) +
                      abs(o["fw_dy_q1000"] - o["pc_dy_q1000"]),
                      reverse=True)[:5]
    print(f"\n  top-5 disagreement frames:")
    for o in disagree:
        print(f"    idx={o['idx']} t={o['t_ms']}ms phase={o['phase']} side={o['side']} "
              f"pc=({o['pc_dx_q1000']/1000:+.3f},{o['pc_dy_q1000']/1000:+.3f}) "
              f"fw=({o['fw_dx_q1000']/1000:+.3f},{o['fw_dy_q1000']/1000:+.3f})")

    # Save CSV
    with open(OUT_CSV, "w") as fp:
        fp.write("idx,t_ms,phase,side,pc_dx_q1000,pc_dy_q1000,pc_conf,fw_dx_q1000,fw_dy_q1000,fw_conf\n")
        for o in out:
            fp.write(f"{o['idx']},{o['t_ms']},{o['phase']},{o['side']},"
                     f"{o['pc_dx_q1000']},{o['pc_dy_q1000']},{o['pc_conf']:.2f},"
                     f"{o['fw_dx_q1000']},{o['fw_dy_q1000']},{o['fw_conf']}\n")
    print(f"\nwrote {OUT_CSV}")

    # Render trajectory on the SAME clean canvas style as
    # trajectory_clean.png (1600x1200 white + grid + corner markers,
    # autofit so trail spans most of the canvas).
    W, H = 1600, 1200
    bg = Image.new("RGB", (W, H), (250, 250, 250))
    d = ImageDraw.Draw(bg, "RGBA")
    GX, GY = 10, 8
    for i in range(1, GX): d.line([(i*W/GX, 0), (i*W/GX, H)], fill=(220,220,220), width=1)
    for j in range(1, GY): d.line([(0, j*H/GY), (W, j*H/GY)], fill=(220,220,220), width=1)

    # Build cumulative trails for both methods in grid-px units.
    pc_trail = [(0.0, 0.0)]
    fw_trail = [(0.0, 0.0)]
    ax = ay = 0.0
    fx = fy = 0.0
    for o in out:
        ax += o["pc_dx_q1000"] / 1000.0
        ay += o["pc_dy_q1000"] / 1000.0
        pc_trail.append((ax, ay))
        fx += o["fw_dx_q1000"] / 1000.0
        fy += o["fw_dy_q1000"] / 1000.0
        fw_trail.append((fx, fy))

    # Autofit: pick scale + center so BOTH trails fit with margins.
    all_xy = pc_trail + fw_trail
    margin = 80
    xs = [p[0] for p in all_xy]; ys = [p[1] for p in all_xy]
    span_x = max(xs) - min(xs) + 1e-6
    span_y = max(ys) - min(ys) + 1e-6
    s_x = (W - 2*margin) / span_x
    s_y = (H - 2*margin) / span_y
    s = min(s_x, s_y)
    cx_off = (W - s * (max(xs) + min(xs))) / 2
    cy_off = (H - s * (max(ys) + min(ys))) / 2
    def to_px(p):
        return (cx_off + s * p[0], cy_off + s * p[1])

    pts_pc = [to_px(p) for p in pc_trail]
    pts_fw = [to_px(p) for p in fw_trail]
    sx, sy = pts_pc[0]  # same for both -- both start at (0,0)

    # Try to load DejaVu for header.
    try:
        from PIL import ImageFont
        F1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
        F2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 36)
    except Exception:
        F1 = F2 = None

    # Firmware SAD: orange (matches trajectory_clean.png MOVE color toned down).
    for i in range(1, len(pts_fw)):
        d.line([pts_fw[i-1], pts_fw[i]], fill=(220, 140, 0, 220), width=4)
    # Phase correlation: cyan/teal.
    for i in range(1, len(pts_pc)):
        d.line([pts_pc[i-1], pts_pc[i]], fill=(0, 180, 200, 230), width=4)

    # START marker.
    d.ellipse((sx-18, sy-18, sx+18, sy+18), outline=(0,160,220), width=4)
    d.line([(sx-30, sy), (sx+30, sy)], fill=(0,160,220), width=3)
    d.line([(sx, sy-30), (sx, sy+30)], fill=(0,160,220), width=3)
    if F2: d.text((sx+22, sy-32), "START", fill=(0,160,220), font=F2)

    # END markers (one per method).
    ex_pc, ey_pc = pts_pc[-1]
    ex_fw, ey_fw = pts_fw[-1]
    d.rectangle((ex_pc-22, ey_pc-22, ex_pc+22, ey_pc+22), outline=(0,180,200), width=4)
    if F2: d.text((ex_pc+28, ey_pc-14), "END (PC)", fill=(0,140,160), font=F2)
    d.rectangle((ex_fw-22, ey_fw-22, ex_fw+22, ey_fw+22), outline=(220,140,0), width=4)
    if F2: d.text((ex_fw+28, ey_fw-14), "END (SAD)", fill=(180,100,0), font=F2)

    # Corner markers (1-4) at last sample of each side, using PHASE-CORR trail.
    sides = [o["side"] for o in out]
    for sn in range(0, 4):
        last_idx = max((i for i, sd in enumerate(sides) if sd == sn),
                       default=-1)
        if last_idx < 0: continue
        x, y = pts_pc[last_idx + 1]
        d.ellipse((x-22, y-22, x+22, y+22), fill=(255,50,50,240),
                  outline=(0,0,0), width=3)
        if F2: d.text((x-12, y-22), str(sn+1), fill=(255,255,255), font=F2)

    # Header bar with metrics.
    pc_clos = (pc_trail[-1][0]**2 + pc_trail[-1][1]**2) ** 0.5
    fw_clos = (fw_trail[-1][0]**2 + fw_trail[-1][1]**2) ** 0.5
    hdr = (f"rows={len(out)}   PHASE-CORR closure={pc_clos:.2f} gp ({pc_clos*8:.0f} raw-px)"
           f"   |   FIRMWARE SAD closure={fw_clos:.2f} gp ({fw_clos*8:.0f} raw-px)")
    d.rectangle((0, 0, W, 40), fill=(0,0,0,200))
    if F1: d.text((10, 8), hdr, fill=(255,255,255), font=F1)

    # Footer legend.
    d.rectangle((0, H-40, W, H), fill=(0,0,0,180))
    if F1: d.text((10, H-32),
                  "CYAN=phase correlation  ORANGE=firmware SAD  RED1-4=corners (PC)  CYAN-CIRCLE=start",
                  fill=(255,255,255), font=F1)

    bg.save(OUT_PNG)
    print(f"wrote {OUT_PNG}  (clean canvas; cyan=phase corr, orange=firmware SAD)")


if __name__ == "__main__":
    main()
