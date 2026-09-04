#!/usr/bin/env python3
"""s246 iter1 — can we get a per-frame provenance oracle out of the sensor?

Goal of s246: alternate between the two cameras at FULL sensor rate without
losing a frame or mixing two cameras into one.  To claim that, we need an
oracle that answers, per frame: which camera produced it, and was any frame
dropped?  Today the only content-based check is the left-column pixel
heuristic from s235/iter10, which its own README admits breaks on real scenes.

OV5640 can emit "embedded data lines": per-frame metadata (frame counter, AGC,
exposure) placed in the first bytes of the image.  `modsentai_camera.c:346`
says this is gated by `0x501F` bit 7, and `sentai.camera.peek_row()` already
exposes those bytes.  If it works, it is a far better oracle:

  - the two sensors free-run with INDEPENDENT frame counters that drift apart,
    so the counter value is itself a camera fingerprint;
  - consecutive frames from one camera have consecutive counters, so a gap
    proves a dropped frame;
  - it is read from the image data itself — no heuristic, no scene dependence.

## What this probe tests

  A. BASELINE   — peek_row() across N frames with 0x501F untouched.  Establish
                  what the first bytes normally look like and whether anything
                  there already varies frame to frame.
  B. ENABLE     — set 0x501F bit 7, repeat.  Did the bytes change at all?
  C. COUNTER    — look for a byte position that increments by ~1 between
                  consecutive frames.  That is the frame counter.
  D. RESTORE    — put 0x501F back and confirm.

Honest caveat recorded up front: the camera path is XRGB8888, so a raw
embedded line is carried through the ISP and the CSI as if it were pixels.
That may mangle it.  A negative result here is informative, not a failure —
it just means this oracle is unavailable and s246 needs a different one.

Outputs: embedded_bytes.csv, verdict.json, run_probe.log
"""
import sys, os, re, json, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV     = os.path.join(HERE, 'embedded_bytes.csv')
VERDICT = os.path.join(HERE, 'verdict.json')

FMT_MUX = 0x501F
NFRAMES = 12       # frames sampled per phase
NBYTES  = 32       # bytes of the first row to inspect


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


class BD:
    def __init__(self):
        self.ser = conn()

    def c(self, cmd, t=8):
        return B.cmd(self.ser, cmd, t)

    def n(self, cmd, t=6):
        return N(self.c(cmd, t))

    def rd(self, cam, reg):
        return self.n('print(sentai.camera.reg_read(%d,0x%04X))' % (cam, reg))

    def wr(self, cam, reg, val):
        return self.n('print(sentai.camera.reg_write(%d,0x%04X,0x%02X))'
                      % (cam, reg, val))

    def row(self, nbytes=NBYTES):
        """peek_row -> list of ints.

        Printed via list() rather than .hex(): this MicroPython build has no
        bytes.hex(), and printing the raw bytes object gives escaped literals
        that are painful to parse unambiguously.  list() yields plain ints.
        """
        txt = self.c('print(list(sentai.camera.peek_row(%d)))' % nbytes, 12)
        for m in re.finditer(r'\[([0-9,\s]+)\]', txt):
            try:
                v = [int(x) for x in m.group(1).replace(' ', '').split(',') if x != '']
            except ValueError:
                continue
            if len(v) == nbytes:
                return v
        return None

    def frame_seq(self):
        return self.n('print(sentai.camera.frame_count())', 6)


def sample(bd, P, label, nframes=NFRAMES):
    """Collect one first-row sample per NEW sensor frame."""
    rows = []
    last = bd.frame_seq()
    tries = 0
    while len(rows) < nframes and tries < nframes * 12:
        tries += 1
        cur = bd.frame_seq()
        if cur == last:
            time.sleep(0.01)
            continue
        last = cur
        r = bd.row()
        if r is not None:
            rows.append((cur, r))
    P('  %s: captured %d frames' % (label, len(rows)))
    return rows


def find_counter(rows):
    """Return byte positions whose value behaves like a frame counter:
    it changes across frames and its deltas are mostly small positive steps
    (mod 256).  A constant byte or pure noise is rejected."""
    if len(rows) < 4:
        return []
    width = min(len(r) for _, r in rows)
    hits = []
    for pos in range(width):
        vals = [r[pos] for _, r in rows]
        if len(set(vals)) < 2:
            continue                       # constant -> not a counter
        deltas = [((vals[i+1] - vals[i]) % 256) for i in range(len(vals) - 1)]
        small = sum(1 for d in deltas if 1 <= d <= 4)
        if small >= int(0.7 * len(deltas)):
            hits.append((pos, vals, deltas))
    return hits


def main():
    log = open(os.path.join(HERE, 'run_probe.log'), 'w')

    def P(*a):
        s = ' '.join(str(x) for x in a)
        print(s)
        log.write(s + '\n')
        log.flush()

    bd = BD()
    if not bd.ser:
        P('NO REPL')
        return 1

    ver = bd.c('print(sentai.version())', 5)
    build = [l for l in ver.strip().split('\n') if 'build' in l]
    P('build:', build[-1].strip() if build else '?')

    rc = bd.n('print(sentai.camera.init(1,45))', 25)
    P('camera.init(1,45) ->', rc, '| fps', bd.n('print(sentai.camera.fps())'))
    if rc != 0:
        P('FAIL: camera not initialized')
        return 1

    probe = bd.row()
    if probe is None:
        P('FAIL: peek_row() unreadable — the instrument does not work, so any')
        P('negative result below would be meaningless.')
        return 1
    P('peek_row sanity: %s ...' % ' '.join('%02x' % b for b in probe[:12]))

    results, cam_reports = {}, {}
    for cam in (0, 1):
        P('\n=== cam%d ===' % cam)
        # cam_id maps 1:1 to the MUX level (cam_mux.h), so select the camera
        # we are about to reconfigure — otherwise we would read cam A's pixels
        # while writing cam B's registers.
        bd.n('print(sentai.camera.select(%d))' % cam, 10)
        time.sleep(0.3)

        orig = bd.rd(cam, FMT_MUX)
        P('0x501F original = 0x%02X' % orig)

        P(' phase A: baseline (0x501F untouched)')
        base = sample(bd, P, 'baseline')

        want = (orig | 0x80) & 0xFF
        bd.wr(cam, FMT_MUX, want)
        back = bd.rd(cam, FMT_MUX)
        P(' phase B: wrote 0x501F=0x%02X -> reads 0x%02X  %s'
          % (want, back, 'BIT7 STUCK' if back == want else 'BIT7 REJECTED'))
        bit7_sticks = (back == want)
        emb = sample(bd, P, 'embedded-on')

        bd.wr(cam, FMT_MUX, orig)
        restored = (bd.rd(cam, FMT_MUX) == orig)
        P(' phase D: restored 0x501F -> %s' % ('OK' if restored else 'MISMATCH!'))

        # Did enabling the bit change the first bytes at all?
        b0 = base[0][1] if base else []
        e0 = emb[0][1] if emb else []
        changed = (b0 != e0)

        hits_base = find_counter(base)
        hits_emb  = find_counter(emb)
        P(' counter-like bytes: baseline=%s embedded=%s'
          % ([h[0] for h in hits_base], [h[0] for h in hits_emb]))
        for pos, vals, deltas in hits_emb[:3]:
            P('   byte[%d] values %s deltas %s' % (pos, vals[:8], deltas[:7]))

        cam_reports[cam] = {
            'orig_501F': orig, 'bit7_sticks': bit7_sticks,
            'restored': restored, 'first_bytes_changed': changed,
            'counter_bytes_baseline': [h[0] for h in hits_base],
            'counter_bytes_embedded': [h[0] for h in hits_emb],
            'baseline_rows': [[s, r] for s, r in base],
            'embedded_rows': [[s, r] for s, r in emb],
        }
        results[cam] = (bit7_sticks, changed, [h[0] for h in hits_emb])

    P('\n--- Verdict ---')
    any_counter = any(len(r[2]) > 0 for r in results.values())
    any_sticks  = any(r[0] for r in results.values())
    any_changed = any(r[1] for r in results.values())

    P('0x501F bit7 accepted by sensor : %s' % ('yes' if any_sticks else 'NO'))
    P('first bytes changed when set   : %s' % ('yes' if any_changed else 'NO'))
    P('counter-like byte found        : %s' % ('YES' if any_counter else 'no'))

    if any_counter:
        concl = ('USABLE — a per-frame counter is readable from the image data; '
                 's246 has a content-based provenance oracle')
    elif any_sticks and not any_changed:
        concl = ('NOT USABLE — bit7 is writable but changes nothing visible in '
                 'the first row; the ISP/XRGB path most likely consumes or '
                 'overwrites the embedded line')
    elif not any_sticks:
        concl = ('NOT USABLE — the sensor rejects 0x501F bit7, so embedded data '
                 'lines cannot be enabled this way on this part')
    else:
        concl = 'NOT USABLE — bytes changed but no counter-like behaviour found'
    P('CONCLUSION:', concl)

    with open(CSV, 'w') as f:
        f.write('cam,phase,frame_seq,' + ','.join('b%d' % i for i in range(NBYTES)) + '\n')
        for cam, rep in cam_reports.items():
            for phase, key in (('baseline', 'baseline_rows'), ('embedded', 'embedded_rows')):
                for seq, r in rep[key]:
                    f.write('%d,%s,%d,%s\n' % (cam, phase, seq,
                                               ','.join(str(b) for b in r)))

    json.dump({'cams': cam_reports, 'conclusion': concl},
              open(VERDICT, 'w'), indent=2)
    P('\nwrote %s\nwrote %s' % (CSV, VERDICT))
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
