#!/usr/bin/env python3
"""EXP-s249 — crash-text record round trip (OP-S10-W23-T8, build >= 1645).

Deliberately raises a precise BusFault on the board (volatile read of an
unmapped address through sentai.sys.peek32), lets the fault handler write
its record into the .sdram_boot_log crash-text slot and reset, re-loads the
RAM image (the board returns to ROM SDP after any reset on this board rev),
then reads back /log/crash_prev.txt and the GPR breadcrumb.

NOT part of repl_regression.py: it resets the board and needs a re-flash.

Run:
    python3 crash_text_probe.py [--addr 0x00400000] [--no-flash]

Pass: /log/crash_prev.txt exists after the reboot and contains
"*** CRASH: code=0x00000FFF" (HardFault, bus faults are escalated) or
"code=0x00000FF4" (BusFault enabled), a PC/LR/CFSR line and a decode
line (PRECISERR / IMPRECISERR / FORCED).
"""
import argparse
import glob
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import repl_regression as rr  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))


def find_port():
    ports = sorted(glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--addr', default='0x00400000',
                    help='unmapped address to read (precise BusFault)')
    ap.add_argument('--no-flash', action='store_true',
                    help='do not re-flash; assume the board comes back by itself')
    a = ap.parse_args()

    port = find_port()
    if not port:
        print('no /dev/ttyACM* — board not up'); return 2
    r = rr.Repl(port)
    print(r.one('print(sentai.version())'))
    r.one('sentai.fs.remove("/log/crash_prev.txt")', 6)
    print('sync:', r.one('print(sentai.fs.sync())', 10))
    print('triggering BusFault at', a.addr, '...')
    try:
        r.raw(('sentai.sys.peek32(%s)\r' % a.addr).encode(), 2.0)
    except Exception as e:  # the port vanishes on reset
        print('  (port error during fault, expected):', e)
    time.sleep(3.0)
    gone = find_port() is None
    print('board left USB after the fault:', gone)

    if not a.no_flash:
        cmd = [os.path.join(REPO, 'venv-flash/bin/python'),
               os.path.join(REPO, 'scripts/flashtool.py'), '-e', 'sentai_runtime', '--ram']
        print('re-loading RAM image ...')
        p = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
        print('  flashtool rc', p.returncode, p.stdout.strip().split('\n')[-1])
        time.sleep(8.0)

    port = find_port()
    if not port:
        print('board did not come back'); return 3
    r = rr.Repl(port)
    print(r.one('print(sentai.version())'))
    print('--- /log/crash_prev.txt ---')
    # Repl.one() keeps the last output line only: flatten the record first.
    txt = r.one('print(sentai.fs.read_str("/log/crash_prev.txt").replace("\\r\\n", " | "))', 10)
    print(txt)
    ok = ('*** CRASH: code=' in txt) and ('PC=0x' in txt) and ('CFSR=0x' in txt)
    print('--- breadcrumb ---')
    print(r.one('print(sentai.rtos.dmesg()[:400])', 8))
    print('crash_text_probe:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
