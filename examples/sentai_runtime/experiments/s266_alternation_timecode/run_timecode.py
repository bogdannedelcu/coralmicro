#!/usr/bin/env python3
"""EXP-s266 host driver: alternate the cameras 1:1 and read the monitor time code from every
dequeued frame on the board; validate identity (stripe order vs firmware tag) and currency
(counter step) and the capture rate.

The on-board loop uses `sentai.camera.peek_col(40)` -> (tag_task, tag_isr, bytes[480], idx,
ts_us, now_us).  The 480-row column is split into 14 equal bands (marker, 12 bits, marker);
each band is averaged over its central 60 %; threshold = midpoint of the two markers.  White
marker in the first band => upright (cam0); black first => rotated 180 (cam1).

Usage: python3 run_timecode.py --out iter1 [--seconds 60] [--exp-lines 90] [--bits 12]
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's265_camera_color_calibration'))
from board_repl import Repl  # noqa: E402

BOARD_LOOP = r"""
def _tc(nframes, bands):
    # bands = {tag: [(row_a, row_b), ...]} rows of column 40 that fall inside each stripe band
    # (computed on the host from the s265 homographies); mean B value per band
    import sentai
    out = []
    for _ in range(nframes):
        r = sentai.camera.peek_col(40)
        col = r[2]
        tb = bands.get(r[0]) or bands.get(0)
        vals = []
        for (a, b) in tb:
            s = 0
            for k in range(a, b):
                s += col[k]
            vals.append(s // (b - a) if b > a else -1)
        out.append((r[4], r[0], r[1], r[3], vals))
    return out
"""


def band_rows(loc_cam, nbands, y0f, y1f, x=40, cam_h=480, keep=0.6):
    """Rows of camera column x that fall in each stripe band, from the camera->screen homography
    (pattern px, 1920x1080).  Each band keeps its central `keep` fraction of rows."""
    Hinv = np.asarray(loc_cam['H_cam_to_mon'])
    rows = np.arange(cam_h)
    P = np.stack([np.full(cam_h, float(x)), rows.astype(float), np.ones(cam_h)])
    Q = Hinv @ P
    Y = Q[1] / Q[2]
    X = Q[0] / Q[2]
    y0, y1 = y0f * 1080, y1f * 1080
    bh = (y1 - y0) / nbands
    band = np.floor((Y - y0) / bh).astype(int)
    ok = (Y >= y0) & (Y < y1) & (X >= 0) & (X < 1920)
    table = []
    for i in range(nbands):
        rr = rows[ok & (band == i)]
        if len(rr) < 4:
            return None, i
        a, b = rr.min(), rr.max()
        m = int((b - a + 1) * (1 - keep) / 2)
        table.append((int(a + m), int(b - m + 1)))
    return table, None


def decode(vals, nb):
    """vals = 14 band means top->bottom.  Returns (code, orientation, contrast)."""
    top, bot = vals[0], vals[-1]
    if abs(top - bot) < 40:
        return None, 'ambiguous', abs(top - bot)
    thr = (top + bot) / 2.0
    if top > bot:
        orient = 'upright'
        bits = vals[1:-1]
    else:
        orient = 'rotated'
        bits = vals[-2:0:-1]
    gray = 0
    for v in bits:
        gray = (gray << 1) | (1 if v > thr else 0)
    b = gray
    shift = 1
    while (gray >> shift):
        b ^= gray >> shift
        shift += 1
    return b, orient, abs(top - bot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--bits', type=int, default=10)
    ap.add_argument('--y0', type=float, default=46 / 1080)
    ap.add_argument('--y1', type=float, default=969 / 1080)
    ap.add_argument('--locate', default=os.path.join(HERE, '..', 's265_camera_color_calibration', 'iter3_kamvas', 'locate.json'))
    ap.add_argument('--exp-lines', type=int, default=90, help='manual exposure in lines (~3 ms at VGA30)')
    ap.add_argument('--gain', type=int, default=0x60, help='AGC in 1/16 (0x60 = 6x)')
    ap.add_argument('--batch', type=int, default=120, help='frames per on-board call')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
    loc = json.load(open(a.locate))
    bands = {}
    for cam in (0, 1):
        t, miss = band_rows(loc['cams'][str(cam)], a.bits + 2, a.y0, a.y1)
        if t is None:
            print('cam%d does not see band %d of the stack: shrink --y0/--y1' % (cam, miss)); return
        bands[cam] = t
        print('cam%d bands (rows of column 40): %s' % (cam, t))
    r.cmd('exec(%r)' % BOARD_LOOP, 10)
    r.cmd('_bands = %r' % bands, 10)
    for cam in (0, 1):
        v = a.exp_lines << 4
        r.regw(cam, 0x3503, 0x03)
        r.regw(cam, 0x3500, (v >> 16) & 0xFF); r.regw(cam, 0x3501, (v >> 8) & 0xFF); r.regw(cam, 0x3502, v & 0xFF)
        r.regw(cam, 0x350A, (a.gain >> 8) & 0xFF); r.regw(cam, 0x350B, a.gain & 0xFF)
    print('ratio(1,1) ->', r.value('sentai.camera.ratio(1, 1)'), ' ratio_unit', r.value('sentai.camera.ratio_unit()'),
          ' stats phase_us', r.value('sentai.camera.stats()["phase_us"]'))
    time.sleep(1.0)
    frames = []
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        out = r.cmd('print(_tc(%d, _bands))' % a.batch, 60)
        line = [l for l in out.replace('\r', '').split('\n') if l.strip().startswith('[')]
        if not line:
            print('no data:', out[:200]); break
        frames += eval(line[-1])
    r.value('sentai.camera.ratio(1, 0)')
    r.close()
    json.dump(frames, open(os.path.join(a.out, 'frames.json'), 'w'))
    # ---- validation --------------------------------------------------------------
    rows = []
    for ts, tag_t, tag_i, idx, vals in frames:
        code, orient, c = decode(vals, a.bits)
        rows.append((ts, tag_t, tag_i, idx, code, orient, c))
    n = len(rows)
    ts = np.array([x[0] for x in rows], np.int64)
    dt = np.diff(ts) & 0xFFFFFFFF
    codes = [x[4] for x in rows]
    orient = [x[5] for x in rows]
    tags = [x[1] for x in rows]
    # the band tables are per camera and already in canonical (marker-first) order, so every
    # frame decoded with the table of its firmware tag must read 'upright'; a frame coming from
    # the other camera reads its markers reversed ('rotated') or ambiguous
    exp_orient = {0: 'upright', 1: 'upright'}
    mis = sum(1 for t, o in zip(tags, orient) if o != 'upright')
    amb = orient.count('ambiguous')
    steps = [((b - a_) % (1 << a.bits)) for a_, b in zip(codes, codes[1:]) if a_ is not None and b is not None]
    rep = sum(1 for s in steps if s == 0)
    big = sum(1 for s in steps if s > 3)
    print('frames %d | interval mean %.2f ms sd %.2f (%.1f fps) | code step: mean %.2f, repeats %d, >3 %d | '
          'orientation vs tag mismatches %d, ambiguous %d' % (
              n, dt.mean() / 1000, dt.std() / 1000, 1e6 / dt.mean(), np.mean(steps), rep, big, mis, amb))
    if steps:
        print('display frames per capture (from code): %.3f  -> implied capture rate %.2f fps at 60 Hz' % (
            np.mean(steps), 60.0 / np.mean(steps)))
    cams = [exp_orient.get(t) for t in tags]
    print('tag sequence (first 24):', ''.join('0' if t == 0 else '1' for t in tags[:24]),
          ' orientation (first 24):', ''.join('U' if o == 'upright' else 'R' if o == 'rotated' else '?' for o in orient[:24]))
    print('codes (first 16):', codes[:16])
    # per-camera statistics (the firmware tag selects the band table, so this is the real test)
    percam = {}
    for cam in (0, 1):
        sel = [i for i in range(n) if tags[i] == cam and codes[i] is not None]
        c = [codes[i] for i in sel]; t = ts[sel]
        st = [((b - a_) % (1 << a.bits)) for a_, b in zip(c, c[1:])]
        hist = {}
        for s in st:
            hist[s] = hist.get(s, 0) + 1
        dtc = (np.diff(t) & 0xFFFFFFFF) / 1000.0 if len(t) > 1 else np.array([0.0])
        exp_step = 2 * 60.0 / (1e6 / dt.mean()) if dt.mean() > 0 else 0
        bad = sum(v for k, v in hist.items() if k == 0 or k > exp_step + 2)
        percam[cam] = {'frames': len(sel), 'interval_ms': float(dtc.mean()), 'step_hist': {int(k): int(v) for k, v in sorted(hist.items())},
                       'expected_step': round(exp_step, 2), 'stale_or_skipped': int(bad)}
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:4]
        print('cam%d: %d frames, interval %.2f ms, code steps %s (expected %.2f), stale(0) or skipped(>%.0f): %d' % (
            cam, len(sel), dtc.mean(), dict(top), exp_step, exp_step + 2, bad))
    verdict = {'per_camera': percam, 'frames': n, 'interval_ms': float(dt.mean() / 1000), 'fps': float(1e6 / dt.mean()),
               'code_step_mean': float(np.mean(steps)) if steps else None, 'repeats': rep, 'big_steps': big,
               'mismatches': mis, 'ambiguous': amb}
    json.dump(verdict, open(os.path.join(a.out, 'verdict.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
