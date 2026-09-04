#!/usr/bin/env python3
"""s246 iter2, step 1 — characterise the two sensor test patterns.

One-off exploration: read the FULL first row (640 px x 4 B = 2560 B, pitch has
LINE_PADDING=0) once per camera and print a downsampled profile, so we can see
what BARS and HBAND actually look like at row 0 and design a classifier.

This is the only step that pulls a whole row through the REPL.  The classifier
itself must run ON THE BOARD and return scalars — 2560 bytes per frame through
MicroPython would violate [[no-heavy-data-through-mp]] and could not keep up at
45 fps anyway.

Patterns (sentai_runtime.cc:3365):
  test_pattern(cam, 0) -> 0x00 disable
  test_pattern(cam, 1) -> 0x80 BARS   (colour bars)
  test_pattern(cam, 2) -> 0x88 HBAND  (horizontal bands, darker)

Buffer layout is XRGB8888: byte 0,1,2 = colour channels, byte 3 = X (unused).
"""
import sys, os, re, time, json

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

WIDTH   = 640
BPP     = 4
ROW_LEN = WIDTH * BPP        # 2560, fits under peek_row's 4096 cap
CHUNK   = 128                # bytes per REPL read; 512 overflows the serial line


def N(x):
    m = re.findall(r'-?\d+', x)
    return int(m[-1]) if m else -999


def conn():
    for _ in range(20):
        for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
            try:
                B.PORT = p
                s = B.open_repl()
                if 'OK' in B.cmd(s, 'import sentai; print("OK")', 3):
                    return s
                s.close()
            except Exception:
                pass
        time.sleep(1.5)
    return None


def read_row(ser):
    """Full first row as a list of ints, read in chunks.

    peek_row(n) always returns the first n bytes of the SAME grab, but each
    call re-grabs, so chunks would come from different frames.  Instead read
    the whole row in one call and slice it on the board, printing one chunk at
    a time -- the row is captured once, into a board-side variable.
    """
    B.cmd(ser, 'r=sentai.camera.peek_row(%d)' % ROW_LEN, 15)
    n = N(B.cmd(ser, 'print(len(r))', 10))
    if n != ROW_LEN:
        print('   read_row: board holds %d bytes, expected %d' % (n, ROW_LEN))
        return None
    out = []
    for off in range(0, ROW_LEN, CHUNK):
        txt = B.cmd(ser, 'print(list(r[%d:%d]))' % (off, off + CHUNK), 20)
        got = None
        for m in re.finditer(r'\[([0-9,\s]+)\]', txt):
            v = [int(x) for x in m.group(1).replace(' ', '').split(',') if x != '']
            if got is None or len(v) > len(got):
                got = v
        if not got or len(got) != CHUNK:
            print('   read_row: chunk at offset %d returned %s'
                  % (off, ('%d values' % len(got)) if got else 'nothing'))
            print('   raw tail: %r' % txt[-160:])
            return None
        out.extend(got)
    return out


def profile(row, nbins=32):
    """Downsample the row to nbins, reporting mean of each colour channel."""
    px = [(row[i], row[i+1], row[i+2]) for i in range(0, len(row), BPP)]
    per = max(1, len(px) // nbins)
    bins = []
    for b in range(nbins):
        seg = px[b*per:(b+1)*per]
        if not seg:
            break
        n = len(seg)
        bins.append((sum(p[0] for p in seg)//n,
                     sum(p[1] for p in seg)//n,
                     sum(p[2] for p in seg)//n))
    return px, bins


def main():
    ser = conn()
    if not ser:
        print('NO REPL')
        return 1
    print(B.cmd(ser, 'print(sentai.version())', 5).strip().split('\n')[-2][:50])
    print('init ->', N(B.cmd(ser, 'print(sentai.camera.init(1,45))', 25)))

    report = {}
    # cam0 = BARS (bright colour bars), cam1 = HBAND (darker bands) -- the
    # pairing used by s235/iter10.
    for cam, mode, name in ((0, 1, 'BARS'), (1, 2, 'HBAND')):
        print('\n=== cam%d : %s ===' % (cam, name))
        print(' test_pattern ->', N(B.cmd(ser, 'print(sentai.camera.test_pattern(%d,%d))' % (cam, mode), 10)))
        print(' select ->', N(B.cmd(ser, 'print(sentai.camera.select(%d))' % cam, 12)))
        time.sleep(0.6)
        print(' grabbed_id ->', N(B.cmd(ser, 'print(sentai.camera.grabbed_id())', 8)))

        row = read_row(ser)
        if not row:
            print(' FAIL: could not read row')
            continue
        px, bins = profile(row)
        vals = [sum(p)//3 for p in px]
        print(' row: %d px  luma min=%d max=%d mean=%d'
              % (len(px), min(vals), max(vals), sum(vals)//len(vals)))
        print(' 32-bin profile (mean of ch0,ch1,ch2 per bin):')
        for i, (a, b_, c) in enumerate(bins):
            bar = '#' * (((a + b_ + c) // 3) // 8)
            print('   bin%02d  %3d %3d %3d  %s' % (i, a, b_, c, bar))
        report[name] = {'cam': cam, 'bins': bins,
                        'luma_min': min(vals), 'luma_max': max(vals),
                        'luma_mean': sum(vals)//len(vals),
                        'first_16_px': px[:16]}

    # leave the patterns ON: iter2 needs them for the oracle
    json.dump(report, open(os.path.join(HERE, 'pattern_profiles.json'), 'w'), indent=2)
    print('\nwrote pattern_profiles.json  (patterns left ENABLED for the oracle)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
