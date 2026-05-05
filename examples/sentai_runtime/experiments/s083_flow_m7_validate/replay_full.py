#!/usr/bin/env python3
"""Replay SAD on EVERY consecutive frame pair (numpy-vectorized).
Outputs per-frame offline (dx, dy, conf) so we can compare 1:1 with
firmware values from headers."""
import sys, time, numpy as np
from pathlib import Path

PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/s003_motion.bin"
OUT_CSV = sys.argv[2] if len(sys.argv) > 2 else "/tmp/s003_offline_full.csv"

GW, GH = 80, 60
F = GW * GH
BW, BH = 32, 32
BX, BY = (GW - BW) // 2, (GH - BH) // 2
SR = 12

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
        gray = np.frombuffer(raw[i:i+F], dtype=np.uint8).reshape(GH, GW)
        i += F
        parts = hdr.split()
        if len(parts) < 9 or parts[0] != "F":
            continue
        out.append({
            "idx": int(parts[1]),
            "t_ms": int(parts[2]),
            "seq": int(parts[3]),
            "fw_dx_q1000": int(parts[4]),
            "fw_dy_q1000": int(parts[5]),
            "conf": int(parts[6]),
            "phase": parts[7],
            "side": int(parts[8]),
            "gray": gray,
        })
    return out

def sad_surface(curr, prev):
    """Build 25x25 SAD surface using numpy.  Each entry = SAD at
    (dy, dx) shift of the center 32x32 block."""
    block = curr[BY:BY+BH, BX:BX+BW].astype(np.int32)
    surf = np.zeros((2*SR+1, 2*SR+1), dtype=np.uint32)
    for dy in range(-SR, SR+1):
        for dx in range(-SR, SR+1):
            ref = prev[BY+dy:BY+dy+BH, BX+dx:BX+dx+BW].astype(np.int32)
            surf[dy+SR, dx+SR] = np.abs(block - ref).sum()
    return surf

def parabolic_q1000(a, b, c):
    denom = int(a) + int(c) - 2*int(b)
    if denom <= 0: return 0
    arm_max = max(int(a)-int(b), int(c)-int(b))
    if denom * 8 < arm_max: return 0
    num = int(a) - int(c)
    d = (num * 1000) // (2 * denom)
    if d > 500 or d < -500: return 0
    return d

def conf_from_sad(sad):
    bp = BW * BH
    mad = int(sad) // bp
    if mad >= 64: return 0
    return 255 - ((mad * 255) // 64)

DEADBAND = 50  # 0.05 grid-px = 50 milli-grid-px
CONF_FLOOR = 150

print("loading frames...")
t0 = time.time()
frames = parse_bulk(PATH)
print(f"  {len(frames)} frames in {time.time()-t0:.1f}s")

print("running SAD on every consecutive pair...")
t0 = time.time()
out = []
prev_g = frames[0]["gray"]
for n, f in enumerate(frames[1:], start=1):
    surf = sad_surface(f["gray"], prev_g)
    flat_min_idx = int(np.argmin(surf))
    bdy, bdx = divmod(flat_min_idx, surf.shape[1])
    bdy -= SR; bdx -= SR
    best = int(surf[bdy+SR, bdx+SR])
    conf = conf_from_sad(best)
    # parabolic
    delta_x = 0
    if -SR < bdx < SR:
        a = surf[bdy+SR, bdx+SR-1]; b = best; c = surf[bdy+SR, bdx+SR+1]
        delta_x = parabolic_q1000(a, b, c)
    delta_y = 0
    if -SR < bdy < SR:
        a = surf[bdy+SR-1, bdx+SR]; b = best; c = surf[bdy+SR+1, bdx+SR]
        delta_y = parabolic_q1000(a, b, c)
    raw_dx = bdx*1000 + delta_x
    raw_dy = bdy*1000 + delta_y
    if conf < CONF_FLOOR:
        raw_dx = 0; raw_dy = 0
    if abs(raw_dx) < DEADBAND and abs(raw_dy) < DEADBAND:
        raw_dx = 0; raw_dy = 0
    out.append({
        "idx": f["idx"],
        "t_ms": f["t_ms"],
        "off_dx_q1000": raw_dx,
        "off_dy_q1000": raw_dy,
        "off_conf": conf,
        "fw_dx_q1000": f["fw_dx_q1000"],
        "fw_dy_q1000": f["fw_dy_q1000"],
        "fw_conf": f["conf"],
        "phase": f["phase"],
        "side": f["side"],
    })
    prev_g = f["gray"]
    if n % 100 == 0:
        print(f"  {n}/{len(frames)-1}  off=({raw_dx/1000:+.3f},{raw_dy/1000:+.3f})  fw=({f['fw_dx_q1000']/1000:+.3f},{f['fw_dy_q1000']/1000:+.3f})")

print(f"\nfull SAD done in {time.time()-t0:.1f}s")

# Cumulatives
fw_cx = sum(o["fw_dx_q1000"] for o in out) / 1000.0
fw_cy = sum(o["fw_dy_q1000"] for o in out) / 1000.0
off_cx = sum(o["off_dx_q1000"] for o in out) / 1000.0
off_cy = sum(o["off_dy_q1000"] for o in out) / 1000.0

# Single-shot (last full frame vs first)
gL = frames[-1]["gray"]; g0 = frames[0]["gray"]
ss_surf = sad_surface(gL, g0)
ss_min = int(np.argmin(ss_surf))
ss_bdy, ss_bdx = divmod(ss_min, ss_surf.shape[1])
ss_bdy -= SR; ss_bdx -= SR
ss_conf = conf_from_sad(int(ss_surf[ss_bdy+SR, ss_bdx+SR]))

print(f"\n=== Comparison: every-frame replay (s={len(out)} samples) ===")
print(f"FIRMWARE cumsum  : cum_dx={fw_cx:+.2f} cum_dy={fw_cy:+.2f} closure={(fw_cx**2+fw_cy**2)**0.5:.2f} gp")
print(f"OFFLINE cumsum   : cum_dx={off_cx:+.2f} cum_dy={off_cy:+.2f} closure={(off_cx**2+off_cy**2)**0.5:.2f} gp")
print(f"SINGLE-SHOT GT   : cum_dx={ss_bdx:+d} cum_dy={ss_bdy:+d} (integer-only) conf={ss_conf}")

# Per-frame max diff between firmware and offline (sanity)
diffs = [abs(o["fw_dx_q1000"] - o["off_dx_q1000"]) for o in out]
print(f"\nper-frame |fw_dx - off_dx| max={max(diffs)} milli-gp, mean={sum(diffs)/len(diffs):.0f}")

# Save CSV
with open(OUT_CSV, "w") as fp:
    fp.write("idx,t_ms,off_dx_q1000,off_dy_q1000,off_conf,fw_dx_q1000,fw_dy_q1000,fw_conf,phase,side\n")
    for o in out:
        fp.write(f"{o['idx']},{o['t_ms']},{o['off_dx_q1000']},{o['off_dy_q1000']},{o['off_conf']},{o['fw_dx_q1000']},{o['fw_dy_q1000']},{o['fw_conf']},{o['phase']},{o['side']}\n")
print(f"\nwrote {OUT_CSV}")
