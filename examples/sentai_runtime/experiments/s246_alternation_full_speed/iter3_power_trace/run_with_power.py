#!/usr/bin/env python3
"""s246 iter3 — run the alternation sweep while watching the supply rail.

The sweep resets the board partway through under sustained 45 fps dual-camera
capture.  The board is stable when idle (uptime climbs for minutes), so the
reset is load-induced.  Two candidate causes look identical from the REPL:

  BROWNOUT      supply sags -> the rail collapses before the CPU stops.
                Signature: V drops measurably BEFORE the current falls.
  SOFTWARE FAULT watchdog / hard fault.  Signature: V stays flat and the
                current drops abruptly at the reset instant.

A power trace separates them.  The board is powered through an FNIRSI FNB48S,
and `tools/fnirsi_power.py` (from ../ch569w) has been logging it at 100 SPS
continuously since 2026-08-18 into /tmp/power_log.jsonl.  We do NOT start a
second logger -- we mark the window and slice the existing log afterwards.

The driver itself touches NO filesystem (operator directive): results come back
over stdout as "CSV|" lines and are written here, on the host.

Usage:  python3 run_with_power.py
Outputs (this folder): results.csv, power_window.csv, run.log, verdict.json
"""
import sys, os, re, csv, json, time, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

POWER_LOG = '/tmp/power_log.jsonl'
DASH      = 'http://localhost:8081'
DRIVER    = '/lib/diag/_t_s246_alt.py'
RUN_MAX_S = 600


def mark(label):
    try:
        urllib.request.urlopen('%s/mark?label=%s' % (DASH, label), timeout=3).read()
    except Exception as e:
        print('  (mark %s failed: %s)' % (label, e))


def tail_power(t_start, t_end, path=POWER_LOG):
    """Slice [t_start, t_end] out of the rolling JSONL log.

    The file is ~11 GB, so seek near the end and scan backwards in blocks
    rather than reading it whole.
    """
    want, size = [], os.path.getsize(path)
    block = 8 << 20
    pos = size
    with open(path, 'rb') as f:
        while pos > 0 and (not want or want[0]['t'] > t_start):
            pos = max(0, pos - block)
            f.seek(pos)
            chunk = f.read(min(block, size - pos)).decode('utf-8', 'replace')
            if pos:                      # drop the partial first line
                chunk = chunk.split('\n', 1)[1] if '\n' in chunk else ''
            rows = []
            for line in chunk.split('\n'):
                line = line.strip()
                if not line.startswith('{'):
                    continue
                try:
                    d = json.loads(line)
                except Exception:
                    continue
                if t_start <= d.get('t', 0) <= t_end:
                    rows.append(d)
            want = rows + want
            if pos == 0:
                break
    return want


def main():
    log = open(os.path.join(HERE, 'run.log'), 'w')

    def P(*a):
        s = ' '.join(str(x) for x in a)
        print(s)
        log.write(s + '\n')
        log.flush()

    ser = B.open_repl()
    B.cmd(ser, 'import sentai', 5)
    P('build:', B.cmd(ser, 'print(sentai.version())', 8).strip().split('\n')[-2][:48])

    mark('s246_start')
    t0 = time.time()
    P('driver start t=%.3f' % t0)

    ser.write(('exec(sentai.fs.read_str("%s"))\r\n' % DRIVER).encode())
    acc, err, done = bytearray(), None, False
    try:
        while time.time() - t0 < RUN_MAX_S:
            c = ser.read(4096)
            if c:
                acc.extend(c)
                if b'=== done ===' in acc:
                    done = True
                    break
            else:
                time.sleep(0.05)
    except Exception as e:
        err = str(e)
    t1 = time.time()
    mark('s246_end')
    P('driver end   t=%.3f  elapsed=%.1fs  done=%s  err=%s'
      % (t1, t1 - t0, done, err))

    txt = acc.decode(errors='replace')
    log.write('\n----- board output -----\n' + txt + '\n')
    i = txt.find('PHASE0')
    P('\n----- board output (from PHASE0) -----')
    P(txt[i if i > 0 else 0:][:4000])

    # results emitted as "CSV|" lines -> write the csv here, host-side
    rows = [l[4:].strip() for l in txt.split('\n') if l.strip().startswith('CSV|')]
    if rows:
        with open(os.path.join(HERE, 'results.csv'), 'w') as f:
            f.write('\n'.join(rows) + '\n')
        P('\nwrote results.csv (%d rows)' % (len(rows) - 1))
    else:
        P('\nno CSV| rows -- the sweep did not reach its report step')

    # ---- power window ------------------------------------------------
    time.sleep(1.5)                       # let the logger flush past t1
    pw = tail_power(t0 - 2.0, t1 + 2.0)
    P('power samples in window: %d' % len(pw))
    verdict = {'done': done, 'err': err, 'elapsed_s': round(t1 - t0, 2),
               'n_power_samples': len(pw)}
    if pw:
        with open(os.path.join(HERE, 'power_window.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_rel', 'v', 'i', 'p', 'temp'])
            for d in pw:
                w.writerow([round(d['t'] - t0, 3), d['v'], d['i'], d['p'],
                            d.get('temp')])
        vs = [d['v'] for d in pw]
        ps = [d['p'] for d in pw]
        vmin = min(vs)
        vmin_t = [d['t'] - t0 for d in pw if d['v'] == vmin][0]
        P('V: min=%.3f mean=%.3f max=%.3f   (min at t+%.2fs)'
          % (vmin, sum(vs) / len(vs), max(vs), vmin_t))
        P('P: min=%.3f mean=%.3f max=%.3f W' % (min(ps), sum(ps) / len(ps), max(ps)))
        verdict.update(v_min=vmin, v_min_t=round(vmin_t, 3),
                       v_mean=round(sum(vs) / len(vs), 4), v_max=max(vs),
                       p_mean=round(sum(ps) / len(ps), 4), p_max=max(ps))

        # If the board reset, the tail of the window brackets it.  Report the
        # last second before the end so a sag is visible without plotting.
        tail = [d for d in pw if (t1 - d['t']) <= 1.0]
        if tail:
            tv = [d['v'] for d in tail]
            P('last 1.0 s before end: V min=%.3f max=%.3f | I min=%.3f max=%.3f'
              % (min(tv), max(tv), min(d['i'] for d in tail),
                 max(d['i'] for d in tail)))
            verdict['tail_v_min'] = min(tv)

        # Brownout call.  The rail is nominally ~5.05 V; a genuine sag shows up
        # as a clear dip, not 20 mV of ripple.
        if vmin < 4.75:
            verdict['diagnosis'] = ('SUPPLY SAG — rail dipped to %.3f V; '
                                    'brownout is plausible' % vmin)
        else:
            verdict['diagnosis'] = ('RAIL HELD — min %.3f V, no brownout; '
                                    'a reset here is a software fault '
                                    '(watchdog / hard fault), not power' % vmin)
        P('DIAGNOSIS:', verdict['diagnosis'])
    json.dump(verdict, open(os.path.join(HERE, 'verdict.json'), 'w'), indent=2)
    P('wrote verdict.json')
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
