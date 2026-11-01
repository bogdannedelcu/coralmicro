#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- MIPI data-type override sweep.

The OV5640 can tag its packets with any CSI-2 data type (0x4814 MIPI_CTRL14[5:0]).
The RT1176 MIPI CSI-2 RX core + VIDEO_MUX pixel reformatter decide from that tag how
the bytes are presented on the 24-bit pixel link that feeds the CSI block:
  YUV422 8-bit (0x1E)  -> expanded to one 24-bit YUV444 pixel per clock (what we have:
                           XYUV8888 = 4 B/px in memory; 16-bit CSI bus sees only {U,V})
  RAW8 (0x2A) / user-defined (0x30) -> ERR051248 says "not supported" for RAW; what
                           does the hardware actually do?  If the bytes pass through one
                           per clock, an 8-bit CSI bus with 2 cycles/pixel stores the
                           sensor's YUYV stream verbatim at 2 B/px.
  RGB565 (0x22)        -> RX presents 16-bit, reformatter expands to RGB888; the 16-bit
                           CSI bus keeps {G8,B8} which contain byte0 (= Y for YUYV) exactly.

Each config boots fresh: yuv_mode(1); csi_mode(bus, cr18); init(1, fps); optional
CSI CR20 poke; then 0x4814 <- DT with the sensor streaming; measure sensor-frame rate,
colour-bar lanes of row 0 and row-to-row consistency, natural-scene lane stats.

Usage: python3 dt_sweep.py [--configs 1,2,...] [--fps 30] [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET, peek, lanes, transitions, bar_means  # noqa: E402

CSI_CR20 = 0x40800000 + 0x50
CR20_10BIT_NEW_EN = 1 << 29

# id: (bus, cr18_or, cr20_or, dt, bytes/px expected, note)
CONFIGS = {
    '1': (8, 0, 0, 0x2A, 2, 'RAW8 tag, 8-bit bus reading data[9:2]'),
    '2': (8, 0, CR20_10BIT_NEW_EN, 0x2A, 2, 'RAW8 tag, 8-bit bus reading data[7:0]'),
    '3': (8, 0, 0, 0x30, 2, 'user-defined 8-bit tag, data[9:2]'),
    '4': (8, 0, CR20_10BIT_NEW_EN, 0x30, 2, 'user-defined 8-bit tag, data[7:0]'),
    '5': (24, 0, 0, 0x2A, 4, 'RAW8 tag into the legacy 24-bit receiver'),
    '6': (16, 0, 0, 0x22, 2, 'RGB565 tag, 16-bit bus keeps {G8,B8}'),
    '7': (24, 0, 0, 0x22, 4, 'RGB565 tag into the legacy 24-bit receiver'),
    '8': (16, 0, 0, 0x1E, 2, 'control: YUV422 tag explicit, 16-bit bus'),
}


def run(cid, fps, log, w=640):
    bus, cr18, cr20, dt, bpp, note = CONFIGS[cid]
    print('\n=== config %s: bus %d cr18 0x%X cr20 0x%X DT 0x%02X (%s)' % (cid, bus, cr18, cr20, dt, note))
    subprocess.run(['python3', RESET], check=False, capture_output=True)
    time.sleep(1.0)
    bd = Board(log)
    r = {'id': cid, 'bus': bus, 'dt': dt, 'bpp': bpp}
    try:
        bd.q('print("YUV", sentai.camera.yuv_mode(1))', 5)
        bd.q('print("CM", sentai.camera.csi_mode(%d, %d))' % (bus, cr18), 5)
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 60)
        r['init'] = bd.last(out, 'INIT'); print(r['init'])
        for l in out.split('\n'):
            if 'CSI probe' in l: print('   ', l.strip())
        if cr20:
            bd.q('sentai.sys.poke32(0x%X, sentai.sys.peek32(0x%X) | 0x%X)' % (CSI_CR20, CSI_CR20, cr20), 5)
        print('  CR20 =', bd.last(bd.q('print("R20", hex(sentai.sys.peek32(0x%X)))' % CSI_CR20, 5), 'R20'))
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        before = bd.last(bd.q('print("DT0", hex(sentai.camera.reg_read(%s, 0x4814)))' % cam, 5), 'DT0')
        print('  0x4814 before:', before)
        print('  write 0x4814 <- 0x%02X:' % dt, bd.last(bd.q('print("WR", sentai.camera.reg_write(%s, 0x4814, 0x%02X))' % (cam, dt), 5), 'WR'))
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
        r['bars_len'] = len(bars)
        if len(bars) >= w * bpp:
            L = lanes(bars, bpp, w)
            r['bars_lanes'] = []
            for k, lane in enumerate(L):
                t = transitions(lane); m = [int(x) for x in bar_means(lane)]
                r['bars_lanes'].append((t, m))
                print('  bars lane %d: transitions=%d means=%s' % (k, t, m))
            rowb = w * bpp; nrows = len(bars) // rowb
            same = [bars[:rowb] == bars[i * rowb:(i + 1) * rowb] for i in range(1, nrows)]
            print('  rows identical to row 0: %s' % same)
            r['rows_same'] = same
            print('  first 32 bytes:', bars[:32].hex())
        else:
            print('  no frame (%d bytes)' % len(bars))
        bd.q('print("TP", sentai.camera.test_pattern(%s, 0))' % cam, 5)
        time.sleep(0.8)
        nat = peek(bd)
        if len(nat) >= w * bpp:
            for k, lane in enumerate(lanes(nat, bpp, w)):
                lane = list(lane); mu = sum(lane) / len(lane)
                print('  natural lane %d: mean=%.1f std=%.1f' % (k, mu, (sum((x - mu) ** 2 for x in lane) / len(lane)) ** 0.5))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        r['stats'] = st
        print('  ', st[:200])
    finally:
        bd.close()
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--configs', default='1,2,3,4,5,6,7,8')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    logp = os.path.join(HERE, 'dt_%s.log' % (a.tag or time.strftime('%H%M%S')))
    res = []
    with open(logp, 'w') as log:
        for c in a.configs.split(','):
            c = c.strip()
            if c: res.append(run(c, a.fps, log))
    print('\n=== summary')
    for r in res:
        bl = r.get('bars_lanes') or []
        print('cfg %s bus %2d DT 0x%02X init %-6s fps %5.1f bars %s rows_same %s' % (
            r['id'], r['bus'], r['dt'], (r.get('init') or '')[5:11], r.get('fps', -1),
            ' '.join('%d:%d' % (k, t) for k, (t, _) in enumerate(bl)), r.get('rows_same')))
    print('log:', logp)


if __name__ == '__main__':
    main()
