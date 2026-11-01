#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- can the RT1176 CSI store MIPI YUV422 at 2 bytes/pixel?

Background: the MIPI CSI-2 -> VIDEO_MUX pixel link always delivers 24-bit pixels,
so the legacy receiver (24-bit bus + PARALLEL24_EN) stores XYUV8888 = 4 B/px
(ERR051248 is only about RAW / YUV422-10bit).  CSICR18 documents a MIPI-aware
packing path (DATA_FROM_MIPI, MIPI_DATA_FORMAT, MIPI_DOUBLE_CMPNT, MIPI_YU_SWAP)
that the Linux imx7-media-csi driver uses on the same CSI IP (i.MX7D / 8MM) to
store YUYV at 2 B/px.  The SDK never programs it.  This sweep tries it on the
Coral Dev Board Micro through `sentai.camera.csi_mode(bus, cr18_or)`.

Each variant runs from a fresh boot (the receiver cannot be re-initialised with a
different bus width in place):
  yuv_mode(1); csi_mode(bus, bits); init(1, 30); OV5640 colour bars; peek the first
  4096 bytes of a frame; then bars off and peek a natural frame.

Pass criteria for a 2 B/px variant:
  * camera init rc 0 and sensor frames advancing at ~30 fps;
  * colour bars: the Y lane of row 0 shows 8 flat bars (7 transitions, 80 px each)
    with Y monotonic white -> black, and rows 0..2 (1280 B each) are identical
    (bars are vertical, so a wrong DMA pitch/width would shift them);
  * natural scene: the Y lane of row 0 correlates (> 0.8) with the Y lane of a
    legacy XYUV capture of the same static scene.

Usage: python3 run_csi16.py [--variants A,B,...] [--fps 30] [--tag x]
"""
import argparse, ast, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402
RESET = '/tmp/claude-1000/-home-bogdan-work-coralmicro/0213fcbf-a574-4225-a0ea-7da5f47ce784/scratchpad/board_reset.py'

DATA_FROM_MIPI = 1 << 22
YU_SWAP        = 1 << 21
DOUBLE_CMPNT   = 1 << 20
def FMT(dt): return dt << 25
YUV422_8B = 0x1E

# name: (bus, cr18_or, bytes/px, what it tests)
VARIANTS = {
    'L':  (24, 0, 4, 'legacy XYUV8888 reference'),
    'A':  (16, DATA_FROM_MIPI | FMT(YUV422_8B) | DOUBLE_CMPNT, 2, 'Linux UYVY8_1X16 recipe'),
    'B':  (16, DATA_FROM_MIPI | FMT(YUV422_8B) | DOUBLE_CMPNT | YU_SWAP, 2, 'A + YU swap'),
    'C':  (16, DATA_FROM_MIPI | FMT(YUV422_8B), 2, 'single component per clock, 16-bit bus'),
    'D':  (16, 0, 2, 'control: 16-bit bus, no MIPI bits (2026-04-25 dead end)'),
    'E':  (8,  DATA_FROM_MIPI | FMT(YUV422_8B), 2, '8-bit bus, 2 cycles per pixel'),
}


class Board:
    def __init__(self, log):
        import glob
        B.PORT = (sorted(glob.glob('/dev/ttyACM*')) or [B.PORT])[0]
        self.ser = B.open_repl(); B.cmd(self.ser, 'import sentai', 5); self.log = log

    def close(self):
        try: self.ser.close()
        except Exception: pass

    def q(self, line, t=20):
        out = B.cmd(self.ser, line, t)
        self.log.write('>>> %s\n%s\n' % (line, out)); self.log.flush()
        return out

    def last(self, out, key):
        for l in out.split('\n'):
            if l.strip().startswith(key): return l.strip()
        return ''


def peek(bd, n=4096):
    out = bd.q('print("PK", repr(sentai.camera.peek_row(%d)))' % n, 30)
    l = bd.last(out, 'PK')
    if not l: return b''
    try: return ast.literal_eval(l[3:].strip())
    except Exception: return b''


def lanes(buf, bpp, w):
    """Return per-lane byte sequences of row 0 (lane k = byte k of each pixel)."""
    row = buf[:w * bpp]
    return [row[k::bpp] for k in range(bpp)]


def transitions(seq, thr=24):
    return sum(1 for a, b in zip(seq, seq[1:]) if abs(a - b) > thr)


def bar_means(seq, nbars=8):
    n = len(seq) // nbars
    return [sum(seq[i * n:(i + 1) * n]) / n for i in range(nbars)]


def corr(a, b):
    n = min(len(a), len(b)); a = a[:n]; b = b[:n]
    ma = sum(a) / n; mb = sum(b) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = sum((x - ma) ** 2 for x in a) ** 0.5; db = sum((y - mb) ** 2 for y in b) ** 0.5
    return num / (da * db) if da and db else 0.0


def run_variant(name, fps, log, w=640):
    bus, bits, bpp, what = VARIANTS[name]
    print('\n=== variant %s: bus %d bits 0x%08X (%s)' % (name, bus, bits, what))
    subprocess.run(['python3', RESET], check=False, capture_output=True)
    time.sleep(1.0)
    bd = Board(log)
    res = {'name': name, 'bus': bus, 'bits': bits, 'bpp': bpp}
    try:
        bd.q('print("YUV", sentai.camera.yuv_mode(1))', 5)
        print(bd.last(bd.q('print("CM", sentai.camera.csi_mode(%d, %d))' % (bus, bits), 5), 'CM'))
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 60)
        init = bd.last(out, 'INIT'); print(init)
        for l in out.split('\n'):
            if 'CSI probe' in l or 'CAMERA_' in l: print('   ', l.strip())
        res['init'] = init
        regs = bd.last(bd.q('print("CM", sentai.camera.csi_mode())', 5), 'CM'); print(regs)
        res['regs'] = regs
        time.sleep(0.5)
        f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        time.sleep(2.0)
        f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        try:
            fps_m = (int(f1.split()[1]) - int(f0.split()[1])) / 2.0
        except Exception:
            fps_m = -1
        res['fps'] = fps_m
        print('sensor fps ~ %.1f' % fps_m)

        # colour bars
        print(bd.last(bd.q('print("TP", sentai.camera.test_pattern(sentai.camera.current_id(), 1))', 5), 'TP'))
        time.sleep(0.6)
        bars = peek(bd)
        res['bars_len'] = len(bars)
        if len(bars) >= w * bpp:
            L = lanes(bars, bpp, w)
            for k, lane in enumerate(L):
                print('  bars lane %d: transitions=%d means=%s' % (
                    k, transitions(lane), [int(m) for m in bar_means(lane)]))
            res['bars_lanes'] = [(transitions(l), [int(m) for m in bar_means(l)]) for l in L]
            rowb = w * bpp
            nrows = len(bars) // rowb
            same = [bars[:rowb] == bars[i * rowb:(i + 1) * rowb] for i in range(1, nrows)]
            print('  rows identical to row 0: %s (of %d rows)' % (same, nrows))
            res['rows_same'] = same
        print(bd.last(bd.q('print("TP", sentai.camera.test_pattern(sentai.camera.current_id(), 0))', 5), 'TP'))
        time.sleep(0.8)
        nat = peek(bd)
        res['nat'] = nat
        res['nat_lanes'] = lanes(nat, bpp, w) if len(nat) >= w * bpp else []
        for k, lane in enumerate(res['nat_lanes']):
            lane = list(lane)
            print('  natural lane %d: mean=%.1f std=%.1f min=%d max=%d' % (
                k, sum(lane) / len(lane), (sum((x - sum(lane) / len(lane)) ** 2 for x in lane) / len(lane)) ** 0.5,
                min(lane), max(lane)))
        res['stats'] = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        print(res['stats'][:300])
    finally:
        bd.close()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variants', default='L,A,B,C,D,E')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    logp = os.path.join(HERE, 'csi16_%s.log' % (a.tag or time.strftime('%H%M%S')))
    results = []
    with open(logp, 'w') as log:
        for v in a.variants.split(','):
            v = v.strip()
            if v: results.append(run_variant(v, a.fps, log))
    # natural-scene cross check against the legacy reference (Y lane)
    ref = next((r for r in results if r['name'] == 'L' and r.get('nat_lanes')), None)
    print('\n=== summary')
    print('%-3s %-4s %-10s %-8s %-6s %-40s %s' % ('var', 'bus', 'bits', 'init', 'fps', 'bars (lane: transitions)', 'Y corr vs L'))
    for r in results:
        bl = r.get('bars_lanes') or []
        bars = ' '.join('%d:%d' % (k, t) for k, (t, _) in enumerate(bl))
        c = ''
        if ref is not None and r.get('nat_lanes') and r['name'] != 'L':
            # legacy XYUV lanes: find the Y lane as the one with the largest variance
            refY = max(ref['nat_lanes'], key=lambda l: max(l) - min(l))
            cs = ['%d:%.2f' % (k, corr(list(l), list(refY))) for k, l in enumerate(r['nat_lanes'])]
            c = ' '.join(cs)
        print('%-3s %-4d 0x%08X %-8s %-6.1f %-40s %s' % (
            r['name'], r['bus'], r['bits'], (r.get('init') or '')[5:12], r.get('fps', -1), bars, c))
    print('log:', logp)


if __name__ == '__main__':
    main()
