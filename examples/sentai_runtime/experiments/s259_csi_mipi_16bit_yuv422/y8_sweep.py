#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- 1 byte/pixel luma and 2 bytes/pixel packed YUV420.

Follow-up to fmt_sweep.py: the OV5640 Y8 format (0x4300=0x10) reaches memory as
[Y,0,0,0] (24-bit receiver) or [Y,0] (16-bit receiver), i.e. Y sits on pixel-link
bits [7:0].  This sweep tries to store it at 1 B/px with the 8-bit receiver (one cycle
per pixel; CSICR20.10BIT_NEW_EN selects data[7:0] instead of data[9:2]), tries the
CSI's own gray-scale converter (CSICR20.QRCODE_EN, QR_DATA_FORMAT=4 "YUV one cycle
per pixel") on the normal YUV422 stream, and tries the MIPI-only YUV420 stream with the
receiver width halved (two pixels per pixel-link cycle -> 2 B/px packed [C,Y1,Y2,0]).
A final single-boot check correlates the Y8 row with the legacy Y lane of the same
static scene.

Usage: python3 y8_sweep.py [--configs ...] [--fps 30] [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET, peek, transitions, bar_means, corr  # noqa: E402

QRCODE_EN = 1 << 31
TEN_BIT_NEW = 1 << 29
def QR_FMT(f): return f << 9

# id: (bus, cr20_or, wdiv, 0x4300, note)
CONFIGS = {
    'y8_8_new':  (8, TEN_BIT_NEW, 1, 0x10, 'Y8 sensor, 8-bit receiver, data[7:0] -> expect 1 B/px luma'),
    'y8_8_raw':  (8, 0, 1, 0x10, 'Y8 sensor, 8-bit receiver, data[9:2]'),
    'gray_8':    (8, QRCODE_EN | QR_FMT(4) | TEN_BIT_NEW, 1, 0x3F, 'YUV422 stream, CSI gray-scale converter, data[7:0]'),
    'gray_8b':   (8, QRCODE_EN | QR_FMT(4), 1, 0x3F, 'YUV422 stream, CSI gray-scale converter, data[9:2]'),
    'gray_24':   (24, QRCODE_EN | QR_FMT(4), 1, 0x3F, 'YUV422 stream, gray-scale converter, 24-bit receiver'),
    '420_24_w2': (24, 0, 2, 0x5F, 'YUV420-MIPI stream, 24-bit receiver, half width -> packed 2 B/px?'),
    '420_16_w2': (16, 0, 2, 0x5F, 'YUV420-MIPI stream, 16-bit receiver, half width'),
    '420e_24_w2': (24, 0, 2, 0x5E, 'YUV420-MIPI (VYY/UYY) stream, 24-bit receiver, half width'),
    'y8_corr':   (24, 0, 1, 0x10, 'legacy receiver: Y lane of YUV422 vs Y8 row, same scene'),
}


def frame_rate(bd, dt=2.0):
    f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
    time.sleep(dt)
    f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
    try: return (int(f1.split()[1]) - int(f0.split()[1])) / dt
    except Exception: return -1


def report(buf, rowb, bpp, tag, nrows=4):
    n = len(buf) // rowb
    print('  %s: %d B = %d rows of %d B; first 32: %s' % (tag, len(buf), n, rowb, buf[:32].hex()))
    for r in range(min(n, nrows)):
        row = buf[r * rowb:(r + 1) * rowb]
        print('    row %d: %s' % (r, ' | '.join('L%d t=%d m=%s' % (k, transitions(row[k::bpp]), [int(x) for x in bar_means(row[k::bpp])]) for k in range(bpp))))
    print('    rows == row0: %s' % [buf[:rowb] == buf[i * rowb:(i + 1) * rowb] for i in range(1, n)])


def run(cid, fps, log, w=640):
    bus, cr20, wdiv, fmt, note = CONFIGS[cid]
    bpp = {24: 4, 16: 2, 8: 1}[bus]
    rowb = (w // wdiv) * bpp
    print('\n=== %s: bus %d cr20 0x%08X wdiv %d 0x4300<-0x%02X (%s)' % (cid, bus, cr20, wdiv, fmt, note))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log); r = {'id': cid}
    try:
        bd.q('sentai.camera.yuv_mode(1)', 5)
        bd.q('print("CM", sentai.camera.csi_mode(%d, 0, %d, %d))' % (bus, cr20, wdiv), 5)
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 60)
        r['init'] = bd.last(out, 'INIT'); print(r['init'])
        for l in out.split('\n'):
            if 'CSI probe' in l: print('   ', l.strip())
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5); time.sleep(0.5)
        if cid == 'y8_corr':
            bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5); time.sleep(0.8)
            a = peek(bd)
            refY = list(a[2:w * 4:4])
            bd.q('sentai.camera.reg_write(%s, 0x4300, 0x10)' % cam, 5); time.sleep(0.8)
            b = peek(bd)
            y8 = list(b[0:w * 4:4])
            print('  legacy Y lane mean %.1f, Y8 lane mean %.1f, corr %.3f' % (sum(refY) / w, sum(y8) / w, corr(refY, y8)))
            # also against a 2x2 average of the next row? just print ratio
            r['corr'] = corr(refY, y8)
            return r
        if fmt != 0x3F:
            print('  0x4300 <-', bd.last(bd.q('print("WR", sentai.camera.reg_write(%s, 0x4300, 0x%02X))' % (cam, fmt), 5), 'WR'))
        time.sleep(0.6)
        r['fps'] = frame_rate(bd); print('  frame rate ~ %.1f' % r['fps'])
        bars = peek(bd)
        if len(bars) >= rowb:
            report(bars, rowb, bpp, 'bars')
        else:
            print('  no frame (%d B)' % len(bars))
        bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5); time.sleep(0.8)
        nat = peek(bd)
        if len(nat) >= rowb:
            for rr in range(min(len(nat) // rowb, 2)):
                row = nat[rr * rowb:(rr + 1) * rowb]
                st = []
                for k in range(bpp):
                    lane = list(row[k::bpp]); mu = sum(lane) / len(lane)
                    st.append('L%d %.0f±%.0f' % (k, mu, (sum((x - mu) ** 2 for x in lane) / len(lane)) ** 0.5))
                print('  natural row %d: %s' % (rr, '  '.join(st)))
        r['stats'] = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
    finally:
        bd.close()
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--configs', default=','.join(CONFIGS))
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    logp = os.path.join(HERE, 'y8_%s.log' % (a.tag or time.strftime('%H%M%S')))
    res = []
    with open(logp, 'w') as log:
        for c in a.configs.split(','):
            c = c.strip()
            if c: res.append(run(c, a.fps, log))
    print('\n=== summary')
    for r in res:
        print('%-11s init %-6s fps %5.1f %s' % (r['id'], (r.get('init') or '')[5:11], r.get('fps', -1),
                                             ('corr %.3f' % r['corr']) if 'corr' in r else ''))
    print('log:', logp)


if __name__ == '__main__':
    main()
