#!/usr/bin/env python3
"""s245 iter3 — functional test: is GPIO8_IO27 routed to an OV5640 pad?

iter2 established that nothing DRIVES GPIO8_IO27, but could not exclude a
high-impedance input (an OV5640 FREX input looks exactly like an absent trace).
This iter closes that gap from the other side: instead of listening for a
driver, we make the SENSOR drive, and watch whether our pin notices.

## The test

OV5640 `0x3016` (PAD_OUTPUT_ENABLE00) enables auxiliary pads as outputs, and
`0x3019` (PAD_OUTPUT_VALUE00) sets their level.  So for each candidate bit:

    0x3016 = (1 << b)     enable that pad as an output
    0x3019 = 0x00         drive it low   -> read our pin
    0x3019 = 0xFF         drive it high  -> read our pin

If our pin's reading FOLLOWS 0x3019, the pad is routed to GPIO8_IO27.  That is
a *correlated* response, not merely "the pin looks driven" — coincidence is
ruled out, so a positive here is conclusive.

This deliberately needs no knowledge of FREX bit semantics (which the OV5640
datasheet holds and this repo does not document), and it produces a static DC
level rather than a per-frame strobe pulse that a millisecond-scale sampler
could easily miss.

## Safety

- The pin is parked as a high-impedance INPUT for the entire sweep
  (`pin_probe(1)`, added in build 1596).  An MCU output driving 0 against a
  sensor output driving 1 would be a short through both drivers.
- `0x3017` / `0x3018` are NOT touched: the NXP driver sets them to 0x00 to put
  the DVP pads in high-Z for MIPI mode (fsl_ov5640.c:997), so writing them
  would break the MIPI link.
- Every config is restored immediately, and camera health (frame_count
  advancing + I2C responding) is re-checked after each step.  The sweep aborts
  on the first sign that the sensor stopped streaming.

Outputs (this folder): frex_functional.csv, verdict.json, run_probe.log
"""
import sys, os, re, json, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV     = os.path.join(HERE, 'frex_functional.csv')
VERDICT = os.path.join(HERE, 'verdict.json')

PAD_EN   = 0x3016      # PAD_OUTPUT_ENABLE00
PAD_VAL  = 0x3019      # PAD_OUTPUT_VALUE00
BITS     = [0, 1, 2, 3, 4, 5, 6, 7]
CAMS     = [0, 1]


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

    def probe(self, park=1):
        """pin_probe(park) -> park pin as input; return (nopull, pullup, pulldown).

        The REPL echoes the command before printing the result, so
        `pin_probe(1)` puts a literal "(1)" in the stream ahead of the real
        tuple.  Take the LONGEST parenthesised group of numbers, not the
        first — the first one is the echo of our own argument.
        """
        txt = self.c('print(sentai.io.pin_probe(%d))' % park, 15)
        best = None
        for m in re.finditer(r'\(([-0-9,\s]+)\)', txt):
            try:
                v = [int(x) for x in m.group(1).replace(' ', '').split(',') if x != '']
            except ValueError:
                continue
            if best is None or len(v) > len(best):
                best = v
        if best is None or len(best) < 11:
            return None
        return (best[0], best[1], best[2])

    def frames_advancing(self):
        a = self.n('print(sentai.camera.frame_count())', 6)
        time.sleep(0.3)
        b = self.n('print(sentai.camera.frame_count())', 6)
        return b > a, (a, b)


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
    build = build[-1].strip() if build else '?'
    P('build:', build)
    if '1596' not in build:
        P('WARNING: expected build 1596 (pin_probe park_input arg)')

    rc = bd.n('print(sentai.camera.init(1,45))', 25)
    P('camera.init(1,45) ->', rc)
    if rc != 0:
        P('FAIL: camera not initialized')
        return 1

    ok, cnt = bd.frames_advancing()
    P('baseline streaming:', ok, cnt)
    if not ok:
        P('FAIL: camera not streaming at baseline; nothing to compare against')
        return 1

    base = bd.probe()
    P('baseline pin (nopull, pullup, pulldown):', base)
    if base is None:
        P('FAIL: cannot read the pin at baseline — the instrument is not')
        P('working, so any "no response" below would be meaningless.')
        return 1
    P('  -> pin parked as INPUT for the whole sweep\n')

    # ---- PRE-FLIGHT: do the register writes actually take? -------------
    # Without this, a sensor that silently ignores 0x3016 would produce a
    # clean-looking "no response" sweep that in fact tested nothing.
    P('--- pre-flight: which bits of 0x3016 / 0x3019 are implemented? ---')
    implemented = {}
    for reg, nm in ((PAD_EN, 'PAD_OUTPUT_ENABLE00'), (PAD_VAL, 'PAD_OUTPUT_VALUE00')):
        orig = bd.rd(0, reg)
        mask = 0
        for pat in (0xFF, 0x55, 0xAA):
            bd.wr(0, reg, pat)
            mask |= bd.rd(0, reg)          # bits that can be set at all
        bd.wr(0, reg, orig)
        implemented[reg] = mask
        P('0x%04X %-20s writable bits = 0x%02X %s'
          % (reg, nm, mask,
             '(all 8)' if mask == 0xFF else '(only bits %s)'
             % ','.join(str(i) for i in range(8) if mask & (1 << i))))
    P('  -> only the writable bits of 0x3016 can possibly enable a pad;')
    P('     bits outside that mask are not a gap in coverage, they do not exist.\n')

    rows, hits = [], []
    aborted = None

    for cam in CAMS:
        orig_en  = bd.rd(cam, PAD_EN)
        orig_val = bd.rd(cam, PAD_VAL)
        P('--- cam%d: saving 0x3016=0x%02X 0x3019=0x%02X ---'
          % (cam, orig_en, orig_val))

        for b in BITS:
            en = 1 << b
            reads = {}
            for drive in (0x00, 0xFF):
                bd.wr(cam, PAD_EN, en)
                bd.wr(cam, PAD_VAL, drive)
                time.sleep(0.05)
                pr = bd.probe()
                reads[drive] = pr

            # restore before judging, so a bad config is live as briefly
            # as possible
            bd.wr(cam, PAD_EN, orig_en)
            bd.wr(cam, PAD_VAL, orig_val)

            lo, hi = reads[0x00], reads[0xFF]
            if lo is None or hi is None:
                P('cam%d bit%d: PROBE FAIL' % (cam, b))
                continue

            # Routed iff the pin reading tracks what the sensor drives.
            #
            # Judge ONLY on the pull-up / pull-down columns.  The no-pull
            # column is meaningless on an undriven pad: a floating CMOS
            # input holds residual charge and drifts, so comparing it
            # manufactures false positives.  (It did exactly that on the
            # first run of this sweep — it "responded" at 0x3016 bit4, a
            # bit the pre-flight had just shown is not even implemented.)
            #
            # A genuinely driven pad forces BOTH pull readings to the
            # driven level, which is the signature we test for.  The
            # no-pull column is still recorded, as evidence of floating.
            responded = (lo[1:] != hi[1:])
            rows.append((cam, b, en, lo, hi, responded))
            P('cam%d 0x3016=0x%02X : drive_low=%s drive_high=%s  %s'
              % (cam, en, lo, hi, 'RESPONDED <<<' if responded else '-'))
            if responded:
                hits.append((cam, b, en, lo, hi))

            ok, cnt = bd.frames_advancing()
            if not ok:
                aborted = 'cam%d bit%d stopped the sensor streaming %s' % (cam, b, cnt)
                P('ABORT:', aborted)
                break

        # verify the restore actually took
        back_en, back_val = bd.rd(cam, PAD_EN), bd.rd(cam, PAD_VAL)
        P('cam%d restored: 0x3016=0x%02X (was 0x%02X) 0x3019=0x%02X (was 0x%02X) %s\n'
          % (cam, back_en, orig_en, back_val, orig_val,
             'OK' if (back_en == orig_en and back_val == orig_val) else 'MISMATCH!'))
        if aborted:
            break

    # Park the pin back to its boot state (output driving 0) now that no
    # sensor pad is enabled any more.
    bd.probe(0)

    ok, cnt = bd.frames_advancing()
    P('final streaming check:', ok, cnt)

    P('\n--- Verdict ---')
    if hits:
        concl = ('ROUTED — the pin tracked 0x3019 on %s'
                 % ', '.join('cam%d bit%d' % (c, b) for c, b, _, _, _ in hits))
        P('CONCLUSIVE POSITIVE:', concl)
        P('A hardware trigger line to the sensor therefore EXISTS.')
    else:
        concl = ('NO RESPONSE — no 0x3016 bit made GPIO8_IO27 track 0x3019 on '
                 'either sensor')
        P(concl)
        P('Combined with iter2 (nothing drives the net), the reasonable reading')
        P('is that GPIO8_IO27 is NOT routed to the OV5640 pads: the name is')
        P('inherited from upstream coralmicro and means nothing on this board.')
        P('Coverage note: 0x3016 implements only bits 0x%02X, so the sweep did'
          % implemented[PAD_EN])
        P('exercise every pad-enable bit that physically exists — the masked')
        P('bits are not untested, they are absent.')
        P('Still not a mathematical proof: a FREX INPUT on the sensor never')
        P('drives anything, so it stays invisible to a test that listens.')
    if aborted:
        P('NOTE: sweep aborted early —', aborted)

    with open(CSV, 'w') as f:
        f.write('cam,bit,pad_en,read_drive_low,read_drive_high,responded\n')
        for cam, b, en, lo, hi, r in rows:
            f.write('%d,%d,0x%02X,"%s","%s",%d\n' % (cam, b, en, lo, hi, 1 if r else 0))

    json.dump({
        'build': build, 'baseline_pin': base,
        'rows': [{'cam': c, 'bit': b, 'pad_en': en,
                  'drive_low': lo, 'drive_high': hi, 'responded': r}
                 for c, b, en, lo, hi, r in rows],
        'hits': hits, 'aborted': aborted, 'conclusion': concl,
        'implemented_bits': {('0x%04X' % k): v for k, v in implemented.items()},
    }, open(VERDICT, 'w'), indent=2)

    P('\nwrote %s' % CSV)
    P('wrote %s' % VERDICT)
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
