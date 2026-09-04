#!/usr/bin/env python3
"""s245 iter2 — is GPIO8_IO27 (Gpio::kCameraTrigger) routed to anything?

Drives the on-board probe `sentai.io.pin_probe()` added in build 1595
(implementation + full method notes in sentai_runtime.cc).

Method: weak-pull flip test.  Configure the pad as an input and read it under
the internal pull-up, then under the internal pull-down.  A pad with nothing on
it follows the pull (1 then 0).  A pad tied to a rail or driven by another
device ignores the ~50 kOhm internal pull and reads the same both times.

Methodology control: the identical measurement runs on Gpio::kEdgeTpuPgood
(GPIO8_IO26, pad GPIO_EMC_B2_16) — the physically adjacent pad in the same
bank, known routed to the EdgeTPU power-good signal.

LIMITATION, stated up front: a high-impedance INPUT on the far end — exactly
what an OV5640 FREX trigger input is — ALSO follows the internal pull.  So
"follows the pulls" means FLOATING-OR-HIGH-Z-LOAD and does not by itself prove
the trace is absent.  Only the TIED/DRIVEN outcome is conclusive.  The charge
timing in column `chg_cyc` is a secondary hint: a routed trace carries more
capacitance and charges measurably slower than a bare pad.

Outputs (this folder): pin_probe.csv, verdict.json, run_probe.log
"""
import sys, os, re, json, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV     = os.path.join(HERE, 'pin_probe.csv')
VERDICT = os.path.join(HERE, 'verdict.json')

PINS = [
    ('kCameraTrigger', 'GPIO8_IO27', 'GPIO_EMC_B2_17', 'UNDER TEST'),
    ('kEdgeTpuPgood',  'GPIO8_IO26', 'GPIO_EMC_B2_16', 'CONTROL (known routed)'),
]
N_REPEAT = 5   # the probe is cheap; repeat to catch an unstable reading


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


def parse_tuple(txt):
    """Pull the (a, b, c, ...) tuple out of the REPL echo."""
    m = re.search(r'\(([-0-9,\s]+)\)', txt)
    if not m:
        return None
    try:
        return [int(x) for x in m.group(1).replace(' ', '').split(',') if x != '']
    except ValueError:
        return None


def main():
    log = open(os.path.join(HERE, 'run_probe.log'), 'w')

    def P(*a):
        s = ' '.join(str(x) for x in a)
        print(s)
        log.write(s + '\n')
        log.flush()

    ser = conn()
    if not ser:
        P('NO REPL — is the board enumerated as 1fc9:c0a1?')
        return 1

    ver = B.cmd(ser, 'print(sentai.version())', 5)
    build = [l for l in ver.strip().split('\n') if 'build' in l]
    build = build[-1].strip() if build else '?'
    P('build:', build)

    runs = []
    for k in range(N_REPEAT):
        txt = B.cmd(ser, 'print(sentai.io.pin_probe())', 15)
        r = parse_tuple(txt)
        if r is None or len(r) < 11:
            P('run %d: PARSE FAIL -> %r' % (k, txt[-160:]))
            continue
        runs.append(r)
        P('run %d: %s' % (k, r))

    if not runs:
        P('FAIL: no usable runs')
        return 1

    P('\n--- Per-pin results (%d runs) ---' % len(runs))
    P('%-16s %-14s %6s %6s %6s %10s  %s'
      % ('pin', 'pad', 'nopull', 'pullup', 'pulldn', 'chg_cyc', 'verdict'))

    rows, results = [], {}
    for p, (name, gpio, pad, role) in enumerate(PINS):
        nop = [r[p * 5 + 0] for r in runs]
        pu  = [r[p * 5 + 1] for r in runs]
        pd  = [r[p * 5 + 2] for r in runs]
        chg = [r[p * 5 + 3] for r in runs]
        flt = [r[p * 5 + 4] for r in runs]

        stable = (len(set(pu)) == 1 and len(set(pd)) == 1)
        follows = (all(v == 1 for v in flt))

        if not stable:
            verdict = 'UNSTABLE'
        elif follows:
            verdict = 'FLOATING-or-HIGH-Z'   # ambiguous by construction
        else:
            verdict = 'TIED/DRIVEN'          # conclusive

        chg_valid = [c for c in chg if c >= 0]
        chg_med = sorted(chg_valid)[len(chg_valid) // 2] if chg_valid else -1

        P('%-16s %-14s %6s %6s %6s %10s  %s'
          % (name, pad, nop[0], pu[0], pd[0],
             chg_med if chg_med >= 0 else 'n/a', verdict))
        rows.append((name, gpio, pad, role, nop[0], pu[0], pd[0], chg_med, verdict))
        results[name] = {
            'gpio': gpio, 'pad': pad, 'role': role,
            'nopull': nop, 'pullup': pu, 'pulldown': pd,
            'charge_cycles': chg, 'charge_cycles_median': chg_med,
            'stable': stable, 'verdict': verdict,
        }

    sdram_bad = [r[10] for r in runs]
    P('\nSDRAM canary mismatches per run:', sdram_bad,
      '->', 'INTACT' if all(v == 0 for v in sdram_bad) else 'CORRUPTED')

    # ---- interpretation ------------------------------------------------
    ut = results['kCameraTrigger']['verdict']
    ct = results['kEdgeTpuPgood']['verdict']
    P('\n--- Interpretation ---')
    P('control pin (kEdgeTpuPgood) reads:', ct)
    if ct == 'TIED/DRIVEN':
        P('  -> method VALIDATED: a genuinely routed+driven pin does resist')
        P('     the internal pulls, so the test can tell the two apart.')
        method_ok = True
    else:
        P('  -> method NOT validated: the known-routed control ALSO follows the')
        P('     pulls (TPU may be unpowered, leaving its PGOOD output high-Z).')
        P('     A "floating" reading on the pin under test proves nothing here.')
        method_ok = False

    P('pin under test (kCameraTrigger) reads:', ut)
    if ut == 'TIED/DRIVEN':
        concl = 'ROUTED — something on the board holds this net'
    elif not method_ok:
        concl = 'INCONCLUSIVE — control did not validate the method'
    else:
        concl = ('NO STRONG DRIVER — floating, or routed to a high-Z input '
                 '(an OV5640 FREX input would look exactly like this)')
    P('CONCLUSION:', concl)

    chg_ut = results['kCameraTrigger']['charge_cycles_median']
    chg_ct = results['kEdgeTpuPgood']['charge_cycles_median']
    if chg_ut >= 0 and chg_ct >= 0:
        P('charge-time hint: under-test %d cyc vs control %d cyc' % (chg_ut, chg_ct))
        P('  (more cycles = more capacitance = more copper/pins on the net)')

    with open(CSV, 'w') as f:
        f.write('pin,gpio,pad,role,nopull,pullup,pulldown,charge_cycles_median,verdict\n')
        for r in rows:
            f.write('%s,%s,%s,"%s",%d,%d,%d,%d,%s\n' % r)

    json.dump({
        'build': build, 'runs': len(runs), 'raw': runs,
        'pins': results,
        'sdram_canary_mismatches': sdram_bad,
        'sdram_intact': all(v == 0 for v in sdram_bad),
        'method_validated_by_control': method_ok,
        'conclusion': concl,
    }, open(VERDICT, 'w'), indent=2)

    P('\nwrote %s' % CSV)
    P('wrote %s' % VERDICT)
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
