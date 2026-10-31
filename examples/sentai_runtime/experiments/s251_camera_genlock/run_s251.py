#!/usr/bin/env python3
"""EXP-s251 host runner: push diag/_t_s251_genlock.py to the board, exec it,
keep the log and the CSV| lines.

Usage:  python3 run_s251.py [--no-push] [--tag iterN]
Outputs: run_<tag>.log, results_<tag>.csv (frame rows F,... and summaries S,...)
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

DRIVER = '_t_s251_genlock.py'
REMOTE = '/lib/diag/' + DRIVER


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-push', action='store_true')
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--timeout', type=float, default=420.0)
    a = ap.parse_args()

    if not a.no_push:
        p = subprocess.run([sys.executable, os.path.join(RT, 'diag', '_host_upload_repl.py'),
                            '--file', DRIVER], cwd=RT, capture_output=True, text=True)
        print(p.stdout.strip().split('\n')[-1] if p.stdout.strip() else p.stderr[-300:])
        if p.returncode != 0:
            print('upload failed'); return 2
        time.sleep(1.0)

    ser = B.open_repl()
    B.cmd(ser, 'import sentai', 5)
    print('build:', B.cmd(ser, 'print(sentai.version())', 8).strip().split('\n')[-2][:60])
    log_path = os.path.join(HERE, 'run_%s.log' % a.tag)
    with open(log_path, 'w') as log:
        done, txt = B.run_driver(ser, REMOTE, timeout_s=a.timeout, log=log)
    print('done=%s log=%s' % (done, log_path))
    rows = [l[4:].strip() for l in txt.split('\n') if l.strip().startswith('CSV|')]
    csv_path = os.path.join(HERE, 'results_%s.csv' % a.tag)
    with open(csv_path, 'w') as f:
        f.write('\n'.join(rows) + '\n')
    print('rows=%d csv=%s' % (len(rows), csv_path))
    for l in txt.split('\n'):
        s = l.strip()
        if s.startswith('---') or s.startswith('  ') and ('n=' in s or 'attempt' in s or 'aligned' in s or 'cam' in s[:6]):
            print(s[:200])
    return 0 if done else 1


if __name__ == '__main__':
    sys.exit(main())
