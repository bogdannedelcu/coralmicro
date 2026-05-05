#!/usr/bin/env python3
"""Render phase-correlation trajectory from s003 trace.csv on a clean
1600x1200 canvas with grid + corner markers, matching the visual style
of experiments/s083_flow_m7_validate/trajectory_clean.png.

Default inputs/outputs are this directory; no args needed:
    python3 draw_trajectory.py
"""
import csv
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).parent
TRACE_CSV = HERE / "trace.csv"
OUT_PNG   = HERE / "trajectory_clean.png"


def font(sz):
    for p in [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    ]:
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            continue
    return None


def parse_trace(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                "t_ms":  int(r["t_ms"]),
                "seq":   int(r["frame_seq"]),
                "dx":    int(r["dx_q1000"]) / 1000.0,
                "dy":    int(r["dy_q1000"]) / 1000.0,
                "conf":  int(r["conf"]),
                "phase": r["phase"],
                "side":  int(r["side_idx"]),
            })
    return rows


def autofit(W, H, margin, trail):
    xs = [p[0] for p in trail]
    ys = [p[1] for p in trail]
    span_x = max(xs) - min(xs) + 1e-6
    span_y = max(ys) - min(ys) + 1e-6
    s_x = (W - 2 * margin) / span_x
    s_y = (H - 2 * margin) / span_y
    s = min(s_x, s_y)
    cx = (W - s * (max(xs) + min(xs))) / 2
    cy = (H - s * (max(ys) + min(ys))) / 2
    return lambda p: (cx + s * p[0], cy + s * p[1])


def main():
    rows = parse_trace(TRACE_CSV)
    if not rows:
        raise SystemExit("trace.csv empty")

    cum_x = cum_y = 0.0
    trail = [(0.0, 0.0)]
    phases = []
    sides  = []
    for r in rows:
        cum_x += r["dx"]
        cum_y += r["dy"]
        trail.append((cum_x, cum_y))
        phases.append(r["phase"])
        sides.append(r["side"])

    W, H = 1600, 1200
    img = Image.new("RGB", (W, H), (250, 250, 250))
    d = ImageDraw.Draw(img, "RGBA")
    GX, GY = 10, 8
    for i in range(1, GX):
        d.line([(i * W / GX, 0), (i * W / GX, H)], fill=(220, 220, 220), width=1)
    for j in range(1, GY):
        d.line([(0, j * H / GY), (W, j * H / GY)], fill=(220, 220, 220), width=1)

    to_px = autofit(W, H, 80, trail)
    pts = [to_px(p) for p in trail]
    sx, sy = to_px((0, 0))
    F1, F2 = font(22), font(36)

    # Path coloured by phase: green = MOVE, orange = HOLD.
    for i in range(1, len(pts)):
        if phases[i - 1] == "MOVE":
            col = (40, 200, 40, 240)
        else:
            col = (240, 150, 0, 200)
        d.line([pts[i - 1], pts[i]], fill=col, width=5)

    # START
    d.ellipse((sx - 18, sy - 18, sx + 18, sy + 18), outline=(0, 160, 220), width=4)
    d.line([(sx - 30, sy), (sx + 30, sy)], fill=(0, 160, 220), width=3)
    d.line([(sx, sy - 30), (sx, sy + 30)], fill=(0, 160, 220), width=3)
    if F2:
        d.text((sx + 22, sy - 32), "START", fill=(0, 160, 220), font=F2)

    # END
    ex, ey = pts[-1]
    d.rectangle((ex - 22, ey - 22, ex + 22, ey + 22), outline=(0, 0, 0), width=4)
    if F2:
        d.text((ex + 28, ey - 14), "END", fill=(0, 0, 0), font=F2)

    # Corner markers (1-4) at the LAST sample of each side (i.e.
    # transition into HOLD).
    for s in range(0, 4):
        last_idx = max((i for i, sd in enumerate(sides) if sd == s),
                       default=-1)
        if last_idx < 0:
            continue
        x, y = pts[last_idx + 1]
        d.ellipse((x - 22, y - 22, x + 22, y + 22),
                  fill=(255, 50, 50, 240), outline=(0, 0, 0), width=3)
        if F2:
            d.text((x - 12, y - 22), str(s + 1),
                   fill=(255, 255, 255), font=F2)

    closure_gp  = (cum_x ** 2 + cum_y ** 2) ** 0.5
    closure_raw = closure_gp * 8

    hdr = ("rows=%d  cum_dx=%.2f gp  cum_dy=%.2f gp  "
           "closure=%.2f gp = %.0f raw-px"
           % (len(rows), cum_x, cum_y, closure_gp, closure_raw))
    d.rectangle((0, 0, W, 40), fill=(0, 0, 0, 200))
    if F1:
        d.text((10, 8), hdr, fill=(255, 255, 255), font=F1)

    legend = ("GREEN=MOVE  ORANGE=HOLD  CYAN=start  "
              "RED1-4=corners  BLACK=end")
    d.rectangle((0, H - 40, W, H), fill=(0, 0, 0, 180))
    if F1:
        d.text((10, H - 32), legend, fill=(255, 255, 255), font=F1)

    img.save(OUT_PNG)
    print(f"wrote {OUT_PNG}")
    print(f"  closure = {closure_gp:.2f} grid-px = {closure_raw:.0f} raw-px")
    print(f"  rows = {len(rows)}, MOVE = {sum(1 for p in phases if p == 'MOVE')}, "
          f"HOLD = {sum(1 for p in phases if p == 'HOLD')}")


if __name__ == "__main__":
    main()
