#!/usr/bin/env python3
"""Draw the camera trajectory from the s002 v1139 motion run.
Builds three views in /tmp/s002_motion_v1139/:
  trajectory_clean.png      -- path on auto-fit grid canvas
  trajectory_on_gray.png    -- path overlaid on the first 80x60 gray
                                  frame (upscaled 16x) so you can see
                                  the actual scene the SAD operated on
  trajectory_per_side.png   -- per-side colour bands + side markers
"""
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path("/tmp/s002_motion_v1139")
GW, GH = 80, 60
F = GW * GH


def parse_bulk(path):
    raw = open(path, "rb").read()
    i = raw.find(b"\n") + 1
    out = []
    while i < len(raw):
        nl = raw.find(b"\n", i)
        if nl < 0: break
        hdr = raw[i:nl].decode("ascii", errors="replace")
        i = nl + 1
        if i + F > len(raw): break
        gray = raw[i:i+F]
        i += F
        parts = hdr.split()
        if len(parts) < 9 or parts[0] != "F":
            continue
        out.append({
            "idx": int(parts[1]),
            "t_ms": int(parts[2]),
            "fw_dx": int(parts[4]) / 1000.0,
            "fw_dy": int(parts[5]) / 1000.0,
            "conf": int(parts[6]),
            "phase": parts[7],
            "side": int(parts[8]),
            "gray": gray,
        })
    return out


frames = parse_bulk(OUT/"bulk_gray.bin")
print("frames:", len(frames))

# Build cumulative path in grid units.
trail = [(0.0, 0.0)]
phases = []
sides = []
confs = []
cum_x = cum_y = 0.0
for f in frames:
    cum_x += f["fw_dx"]
    cum_y += f["fw_dy"]
    trail.append((cum_x, cum_y))
    phases.append(f["phase"])
    sides.append(f["side"])
    confs.append(f["conf"])

# Auto-fit canvas + helpers
def autofit(W, H, margin, pts):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    gp_min_x, gp_max_x = min(xs), max(xs)
    gp_min_y, gp_max_y = min(ys), max(ys)
    gw = max(1.0, gp_max_x - gp_min_x)
    gh = max(1.0, gp_max_y - gp_min_y)
    scale = min((W - 2*margin) / gw, (H - 2*margin) / gh)
    def to_px(g):
        return (margin + (g[0] - gp_min_x) * scale,
                margin + (g[1] - gp_min_y) * scale)
    return to_px


def font(size):
    try:
        return ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
    except Exception:
        return ImageFont.load_default()


# Per-side palette (B, G, R, alpha)
SIDE_COLORS = [
    (40,  220, 40,  240),   # 0 = green
    (240, 150, 0,   240),   # 1 = orange
    (40,  130, 240, 240),   # 2 = blue
    (220, 40,  220, 240),   # 3 = magenta
]


# === View 1: clean canvas ===
def make_clean():
    W, H = 1600, 1200
    img = Image.new("RGB", (W, H), (250, 250, 250))
    d = ImageDraw.Draw(img, "RGBA")
    GX, GY = 10, 8
    for i in range(1, GX): d.line([(i*W/GX, 0), (i*W/GX, H)], fill=(220,220,220), width=1)
    for j in range(1, GY): d.line([(0, j*H/GY), (W, j*H/GY)], fill=(220,220,220), width=1)
    to_px = autofit(W, H, 80, trail)
    pts = [to_px(p) for p in trail]
    sx, sy = to_px((0, 0))
    F1, F2 = font(22), font(36)
    # path coloured by phase (MOVE = solid, HOLD = lighter)
    for i in range(1, len(pts)):
        if phases[i-1] == "MOVE":
            col = (40, 200, 40, 240)
        else:
            col = (240, 150, 0, 200)
        d.line([pts[i-1], pts[i]], fill=col, width=5)
    # start
    d.ellipse((sx-18, sy-18, sx+18, sy+18), outline=(0,160,220), width=4)
    d.line([(sx-30, sy), (sx+30, sy)], fill=(0,160,220), width=3)
    d.line([(sx, sy-30), (sx, sy+30)], fill=(0,160,220), width=3)
    d.text((sx+22, sy-32), "START", fill=(0,160,220), font=F2)
    # end
    ex, ey = pts[-1]
    d.rectangle((ex-22, ey-22, ex+22, ey+22), outline=(0,0,0), width=4)
    d.text((ex+28, ey-14), "END", fill=(0,0,0), font=F2)
    # side markers at MOVE->HOLD transitions
    for s in range(0, 4):
        last_idx = max((i for i, sd in enumerate(sides) if sd == s),
                       default=-1)
        if last_idx < 0: continue
        x, y = pts[last_idx + 1]
        d.ellipse((x-22, y-22, x+22, y+22), fill=(255,50,50,240),
                  outline=(0,0,0), width=3)
        d.text((x-12, y-22), str(s+1), fill=(255,255,255), font=F2)
    closure = ((pts[-1][0]-sx)**2 + (pts[-1][1]-sy)**2)**0.5
    hdr = ("rows=%d  cum_dx=%.2f gp  cum_dy=%.2f gp  closure=%.2f gp = %.0f raw-px"
           % (len(frames), cum_x, cum_y,
              (cum_x**2+cum_y**2)**0.5, 8*((cum_x**2+cum_y**2)**0.5)))
    d.rectangle((0, 0, W, 40), fill=(0,0,0,200))
    d.text((10, 8), hdr, fill=(255,255,255), font=F1)
    d.rectangle((0, H-40, W, H), fill=(0,0,0,180))
    d.text((10, H-32), "GREEN=MOVE  ORANGE=HOLD  CYAN=start  RED1-4=corners  BLACK=end",
           fill=(255,255,255), font=F1)
    return img


# === View 2: path on first gray frame (upscaled) ===
def make_on_gray():
    SCALE = 16
    W, H = GW*SCALE, GH*SCALE
    # Use the first frame's gray as background, upscaled NN.
    g0 = frames[0]["gray"]
    bg = Image.frombytes("L", (GW, GH), bytes(g0))
    bg = bg.resize((W, H), Image.NEAREST).convert("RGB")
    d = ImageDraw.Draw(bg, "RGBA")
    # Place start at scene center; map each grid-px of motion to
    # SCALE pixels (1:1 with the gray buffer's own scale).
    cx, cy = W/2, H/2
    F1, F2 = font(20), font(28)
    # Faint grid overlay
    GX, GY = 10, 8
    for i in range(1, GX): d.line([(i*W/GX, 0), (i*W/GX, H)], fill=(255,255,255,80), width=1)
    for j in range(1, GY): d.line([(0, j*H/GY), (W, j*H/GY)], fill=(255,255,255,80), width=1)
    pts = [(cx + p[0]*SCALE, cy + p[1]*SCALE) for p in trail]
    for i in range(1, len(pts)):
        col = (40, 220, 40, 240) if phases[i-1] == "MOVE" else (240, 150, 0, 220)
        d.line([pts[i-1], pts[i]], fill=col, width=4)
    # start
    d.ellipse((cx-15, cy-15, cx+15, cy+15), outline=(0,200,255), width=4)
    d.text((cx+18, cy-26), "S", fill=(0,200,255), font=F2)
    # end
    ex, ey = pts[-1]
    d.rectangle((ex-18, ey-18, ex+18, ey+18), outline=(255,255,0), width=4)
    d.text((ex+22, ey-12), "E", fill=(255,255,0), font=F2)
    # corners
    for s in range(0, 4):
        last_idx = max((i for i, sd in enumerate(sides) if sd == s),
                       default=-1)
        if last_idx < 0: continue
        x, y = pts[last_idx + 1]
        d.ellipse((x-15, y-15, x+15, y+15), fill=(255,50,50,240),
                  outline=(0,0,0), width=2)
        d.text((x-8, y-15), str(s+1), fill=(255,255,255), font=F2)
    return bg


# === View 3: per-side coloured + diagnostic panel ===
def make_per_side():
    W, H = 1600, 900
    PANEL = 480
    img = Image.new("RGB", (W, H), (245, 245, 245))
    d = ImageDraw.Draw(img, "RGBA")
    F1, F2 = font(20), font(30)
    to_px = autofit(W - PANEL, H, 60, trail)
    pts = [to_px(p) for p in trail]
    sx, sy = to_px((0, 0))
    # Per-side path
    for i in range(1, len(pts)):
        s = sides[i-1]
        col = SIDE_COLORS[s % 4]
        if phases[i-1] == "HOLD":
            col = (col[0], col[1], col[2], 100)  # lighter for HOLD
        d.line([pts[i-1], pts[i]], fill=col, width=6)
    d.ellipse((sx-16, sy-16, sx+16, sy+16), outline=(0,0,0), width=3)
    d.text((sx+20, sy-24), "S", fill=(0,0,0), font=F2)
    ex, ey = pts[-1]
    d.rectangle((ex-16, ey-16, ex+16, ey+16), outline=(0,0,0), width=3)
    d.text((ex+20, ey-12), "E", fill=(0,0,0), font=F2)
    # Side panel: mini stats per side
    px0 = W - PANEL + 20
    d.text((px0, 20), "Per-side breakdown (gp)", fill=(0,0,0), font=F2)
    side_sums = {0: [0,0,0,0], 1: [0,0,0,0], 2: [0,0,0,0], 3: [0,0,0,0]}
    side_n = {0: [0,0], 1: [0,0], 2: [0,0], 3: [0,0]}
    for f in frames:
        s = f["side"]
        ph_idx = 0 if f["phase"] == "MOVE" else 1
        side_sums[s][ph_idx*2]   += f["fw_dx"]
        side_sums[s][ph_idx*2+1] += f["fw_dy"]
        side_n[s][ph_idx]        += 1
    y = 70
    for s in range(4):
        col = SIDE_COLORS[s]
        d.rectangle((px0, y, px0+20, y+20), fill=col[:3] + (255,))
        ms = side_sums[s]
        ns = side_n[s]
        line = ("Side %d   MOVE n=%d dx=%+5.1f dy=%+5.1f   HOLD n=%d dx=%+5.1f dy=%+5.1f"
                % (s+1, ns[0], ms[0], ms[1], ns[1], ms[2], ms[3]))
        d.text((px0+30, y), line, fill=(0,0,0), font=F1)
        y += 35
    # Header
    closure = (cum_x**2 + cum_y**2)**0.5
    d.rectangle((0, 0, W-PANEL, 35), fill=(0,0,0,200))
    d.text((10, 6), "rows=%d  cum=(%+.2f, %+.2f) gp  closure=%.2f gp = %.0f raw-px"
           % (len(frames), cum_x, cum_y, closure, 8*closure),
           fill=(255,255,255), font=F1)
    return img


for name, builder in [("trajectory_clean.png", make_clean),
                      ("trajectory_on_gray.png", make_on_gray),
                      ("trajectory_per_side.png", make_per_side)]:
    img = builder()
    img.save(OUT/name)
    print(name, img.size)
