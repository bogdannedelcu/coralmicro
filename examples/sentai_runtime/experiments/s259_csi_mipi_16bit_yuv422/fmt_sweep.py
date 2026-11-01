#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- OV5640 output-format sweep (fewer bits per pixel).

The OV5640 FORMAT CONTROL 00 (0x4300[7:4]) offers, besides YUV422 (0x3), Y8 (0x1),
YUV420 (0x4) and "YUV420 (for MIPI only)" (0x5).  The RT1176 MIPI RX core documents
YUV420 8-bit data types (0x18/0x1C/0x1A) and VIDEO_MUX has a YUV420_CTRL register, so
the SoC side anticipates them.  Question: what lands in memory, at how many bytes per
pixel, when the sensor is switched to one of these formats while streaming?

Per config (fresh boot): yuv_mode(1); csi_mode(bus, 0); init(1, fps); write 0x4300;
measure the sensor-frame rate, colour-bar lanes of rows 0..3 and their equality,
natural-scene lane statistics.

Usage: python3 fmt_sweep.py [--configs ...] [--fps 30] [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET, peek, transitions, bar_means  # noqa: E402

# id: (bus, 0x4300 value, note)
CONFIGS = {
    'y8_24':   (24, 0x10, 'Y8 into the legacy 24-bit receiver'),
    'y8_16':   (16, 0x10, 'Y8 into the 16-bit receiver'),
    'y8_8':    (8,  0x10, 'Y8 into the 8-bit receiver (2 cycles/px)'),
    '420m_24': (24, 0x5F, 'YUV420 MIPI-only (seq F) into the legacy receiver'),
    '420m_16': (16, 0x5F, 'YUV420 MIPI-only (seq F) into the 16-bit receiver'),
    '420me_24': (24, 0x5E, 'YUV420 MIPI-only (seq E, VYY/UYY) into the legacy receiver'),
    '420me_16': (16, 0x5E, 'YUV420 MIPI-only (seq E) into the 16-bit receiver'),
    '420m_8':  (8,  0x5F, 'YUV420 MIPI-only (seq F) into the 8-bit receiver'),
    '420_24':  (24, 0x4F, 'YUV420 (seq F) into the legacy receiver'),
    '420_16':  (16, 0x4F, 'YUV420 (seq F) into the 16-bit receiver'),
    '422_24':  (24, 0x3F, 'control: YUV422 UYVY legacy'),
}


def row_report(buf, w, bpp, tag):
    rowb = w * bpp
    nrows = len(buf) // rowb
    print('  %s: %d bytes, %d rows of %d B; first 32: %s' % (tag, len(buf), nrows, rowb, buf[:32].hex()))
    for r in range(min(nrows, 4)):
        row = buf[r * rowb:(r + 1) * rowb]
        lane_txt = []
        for k in range(bpp):
            lane = row[k::bpp]
            lane_txt.append('L%d t=%d m=%s' % (k, transitions(lane), [int(x) for x in bar_means(lane)]))
        print('    row %d: %s' % (r, ' | '.join(lane_txt)))
    same = [buf[:rowb] == buf[i * rowb:(i + 1) * rowb] for i in range(1, nrows)]
    print('    rows == row0: %s' % same)
    return same


def run(cid, fps, log, w=640):
    bus, fmt, note = CONFIGS[cid]
    bpp = 4 if bus == 24 else 2
    print('\n=== config %s: bus %d 0x4300<-0x%02X (%s)' % (cid, bus, fmt, note))
    subprocess.run(['python3', RESET], check=False, capture_output=True)
    time.sleep(1.0)
    bd = Board(log)
    r = {'id': cid, 'bus': bus, 'fmt': fmt}
    try:
        bd.q('print("YUV", sentai.camera.yuv_mode(1))', 5)
        bd.q('print("CM", sentai.camera.csi_mode(%d, 0))' % bus, 5)
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 60)
        r['init'] = bd.last(out, 'INIT'); print(r['init'])
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        print('  0x4300 before:', bd.last(bd.q('print("F0", hex(sentai.camera.reg_read(%s, 0x4300)))' % cam, 5), 'F0'))
        print('  write:', bd.last(bd.q('print("WR", sentai.camera.reg_write(%s, 0x4300, 0x%02X))' % (cam, fmt), 5), 'WR'),
              bd.last(bd.q('print("F1", hex(sentai.camera.reg_read(%s, 0x4300)))' % cam, 5), 'F1'))
        time.sleep(0.5)
        f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        time.sleep(2.0)
        f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        try: r['fps'] = (int(f1.split()[1]) - int(f0.split()[1])) / 2.0
        except Exception: r['fps'] = -1
        print('  sensor frame rate ~ %.1f' % r['fps'])
        bd.q('print("TP", sentai.camera.test_pattern(%s, 1))' % cam, 5)
        time.sleep(0.6)
        bars = peek(bd)
        if len(bars) >= w * bpp:
            r['rows_same'] = row_report(bars, w, bpp, 'bars')
        else:
            print('  no frame (%d bytes)' % len(bars))
        bd.q('print("TP", sentai.camera.test_pattern(%s, 0))' % cam, 5)
        time.sleep(0.8)
        nat = peek(bd)
        if len(nat) >= w * bpp:
            rowb = w * bpp
            for rr in range(min(len(nat) // rowb, 2)):
                row = nat[rr * rowb:(rr + 1) * rowb]
                st = []
                for k in range(bpp):
                    lane = list(row[k::bpp]); mu = sum(lane) / len(lane)
                    st.append('L%d %.0f±%.0f' % (k, mu, (sum((x - mu) ** 2 for x in lane) / len(lane)) ** 0.5))
                print('  natural row %d: %s' % (rr, '  '.join(st)))
        r['stats'] = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        print('  ', r['stats'][:160])
    finally:
        bd.close()
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--configs', default=','.join(CONFIGS))
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    logp = os.path.join(HERE, 'fmt_%s.log' % (a.tag or time.strftime('%H%M%S')))
    res = []
    with open(logp, 'w') as log:
        for c in a.configs.split(','):
            c = c.strip()
            if c: res.append(run(c, a.fps, log))
    print('\n=== summary')
    for r in res:
        print('%-9s bus %2d 0x4300=0x%02X init %-6s fps %5.1f rows_same %s' % (
            r['id'], r['bus'], r['fmt'], (r.get('init') or '')[5:11], r.get('fps', -1), r.get('rows_same')))
    print('log:', logp)


if __name__ == '__main__':
    main()
