#!/usr/bin/env python3
"""Render the camera trajectory from /tmp/s003_motion.bin.
Two paths are drawn for visual cross-check:
  - FIRMWARE (red): cumulative integration of fw_dx/dy taken from
    bulk headers, with duplicate-seq samples filtered out.
  - OFFLINE (green): cumulative integration of SAD recomputed
    locally on every consecutive unique-seq gray pair.
Per-side ground-truth colour bands keep the side-0/1/2/3 boundaries
visible.  Output: /tmp/s003_motion_trajectory.png  (1600x1200)."""
import sys, numpy as np
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, '/tmp')
import importlib.util
spec = importlib.util.spec_from_file_location('rf', '/tmp/replay_full.py')
rf = importlib.util.module_from_spec(spec); spec.loader.exec_module(rf)

frames = rf.parse_bulk('/tmp/s003_motion.bin')

# Dedup by seq + run offline SAD per unique-seq pair.
unique = []
last = -1
for f in frames:
    if f['seq'] != last:
        unique.append(f); last = f['seq']
print('unique seq:', len(unique))

fw_traj = [(0.0, 0.0)]
off_traj = [(0.0, 0.0)]
sides = [-1]
fx = fy = ox = oy = 0.0
prev_g = unique[0]['gray']
for f in unique[1:]:
    fx += f['fw_dx_q1000']/1000.0
    fy += f['fw_dy_q1000']/1000.0
    surf = rf.sad_surface(f['gray'], prev_g)
    flat = int(np.argmin(surf))
    bdy, bdx = divmod(flat, surf.shape[1])
    bdy -= rf.SR; bdx -= rf.SR
    best = int(surf[bdy+rf.SR, bdx+rf.SR])
    conf = rf.conf_from_sad(best)
    delta_x = delta_y = 0
    if -rf.SR < bdx < rf.SR:
        a = surf[bdy+rf.SR, bdx+rf.SR-1]; b = best; c = surf[bdy+rf.SR, bdx+rf.SR+1]
        delta_x = rf.parabolic_q1000(a,b,c)
    if -rf.SR < bdy < rf.SR:
        a = surf[bdy+rf.SR-1, bdx+rf.SR]; b = best; c = surf[bdy+rf.SR+1, bdx+rf.SR]
        delta_y = rf.parabolic_q1000(a,b,c)
    raw_dx = bdx*1000+delta_x; raw_dy = bdy*1000+delta_y
    if conf < 150: raw_dx = raw_dy = 0
    if abs(raw_dx) < 50 and abs(raw_dy) < 50: raw_dx = raw_dy = 0
    ox += raw_dx/1000.0; oy += raw_dy/1000.0
    fw_traj.append((fx, fy))
    off_traj.append((ox, oy))
    sides.append(f['side'])
    prev_g = f['gray']

# Auto-fit canvas
W, H = 1600, 1200
margin = 80
all_pts = fw_traj + off_traj
xs = [p[0] for p in all_pts]; ys = [p[1] for p in all_pts]
gp_min_x, gp_max_x = min(xs), max(xs)
gp_min_y, gp_max_y = min(ys), max(ys)
gw = max(1.0, gp_max_x - gp_min_x); gh = max(1.0, gp_max_y - gp_min_y)
scale = min((W-2*margin)/gw, (H-2*margin)/gh)
def to_px(g):
    return (margin + (g[0] - gp_min_x)*scale,
            margin + (g[1] - gp_min_y)*scale)

img = Image.new('RGB', (W, H), (250, 250, 250))
d = ImageDraw.Draw(img, 'RGBA')
# Faint grid
for i in range(1, 10): d.line([(i*W/10, 0), (i*W/10, H)], fill=(220,220,220), width=1)
for i in range(1, 8): d.line([(0, i*H/8), (W, i*H/8)], fill=(220,220,220), width=1)

try:
    F1 = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 22)
    F2 = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 32)
except Exception:
    F1 = F2 = ImageFont.load_default()

# Start marker
sx, sy = to_px((0, 0))
d.line([(sx-30, sy),(sx+30, sy)], fill=(0, 160, 220), width=4)
d.line([(sx, sy-30),(sx, sy+30)], fill=(0, 160, 220), width=4)
d.ellipse((sx-18, sy-18, sx+18, sy+18), outline=(0,160,220), width=4)
d.text((sx+22, sy-32), 'START', fill=(0,160,220), font=F2)

# Firmware trajectory in RED
fw_px = [to_px(p) for p in fw_traj]
for i in range(1, len(fw_px)):
    d.line([fw_px[i-1], fw_px[i]], fill=(220, 30, 30, 220), width=4)

# Offline trajectory in GREEN
off_px = [to_px(p) for p in off_traj]
for i in range(1, len(off_px)):
    d.line([off_px[i-1], off_px[i]], fill=(20, 180, 20, 200), width=3)

# Side transitions: mark each MOVE→HOLD transition with side number
for s in range(0, 4):
    # find last index with this side
    end_idx = max(i for i, sd in enumerate(sides) if sd == s)
    px, py = fw_px[end_idx]
    d.ellipse((px-22, py-22, px+22, py+22), fill=(255, 50, 50, 240), outline=(0,0,0), width=3)
    d.text((px-12, py-22), str(s+1), fill=(255,255,255), font=F2)

# End marker = black square
ex, ey = fw_px[-1]
d.rectangle((ex-22, ey-22, ex+22, ey+22), outline=(0,0,0), width=4)
d.text((ex+28, ey-14), 'END', fill=(0,0,0), font=F1)

# Header
fw_cl = (fw_px[-1][0]-sx)**2 + (fw_px[-1][1]-sy)**2
fw_cl = fw_cl ** 0.5
fw_x_gp = fw_traj[-1][0]; fw_y_gp = fw_traj[-1][1]
off_x_gp = off_traj[-1][0]; off_y_gp = off_traj[-1][1]
hdr = (
    f"unique-seq frames: {len(unique)}  "
    f"FW cum_dx={fw_x_gp:+.2f} cum_dy={fw_y_gp:+.2f} closure={(fw_x_gp**2+fw_y_gp**2)**0.5:.2f} gp  |  "
    f"OFF cum_dx={off_x_gp:+.2f} cum_dy={off_y_gp:+.2f} closure={(off_x_gp**2+off_y_gp**2)**0.5:.2f} gp"
)
d.rectangle((0, 0, W, 40), fill=(0,0,0,200))
d.text((10, 8), hdr, fill=(255,255,255), font=F1)
d.rectangle((0, H-40, W, H), fill=(0,0,0,180))
d.text((10, H-32), "RED=firmware trajectory  GREEN=offline replay  CYAN=start  RED1-4=corners  BLACK=end",
       fill=(255,255,255), font=F1)

img.save('/tmp/s003_motion_trajectory.png')
print('wrote /tmp/s003_motion_trajectory.png', img.size)
