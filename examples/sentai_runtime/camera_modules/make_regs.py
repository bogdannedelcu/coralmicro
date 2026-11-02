#!/usr/bin/env python3
"""Export the production register set of each connector from modules.json to the board
format (/cam/cam<N>.regs: "ADDR VAL" hex per line) and optionally upload it over the REPL.

The AEC/AGC freeze registers (0x3500-0x3503, 0x350A-0x350B) are always dropped so the sensor
keeps auto exposure (measured 2026-11-02: auto exposure + the tuned set reproduces the
calibration, dE00 8.7 / 12.4 vs 9.1 / 13.9 with the frozen exposure).  The tuned manual AWB
gains are KEPT by default (--awb manual): the sensor's own AWB on these modules gives dE00
17.9 / 17.8 (greys 15-21) against 8.7 / 12.4 with the gains -- out/measure_v2_*.  --awb auto
drops 0x3400-0x3406 for a field illuminant very different from the calibration display.

Usage:
  python3 make_regs.py --out out/                  # write out/cam0.regs, out/cam1.regs
  python3 make_regs.py --upload                    # also write them to the board (/cam/)
  python3 make_regs.py --upload --awb manual
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'experiments', 's265_camera_color_calibration'))

EXPOSURE = set(range(0x3500, 0x3504)) | {0x350A, 0x350B}
AWB_MANUAL = set(range(0x3400, 0x3407))


def export(module, awb, keep_exposure=False):
    prod = [s for s in module['register_sets'] if s.get('production')][0]
    regs = json.load(open(os.path.join(HERE, prod['file'])))
    lines = ['# %s on %s/%s — %s (%s), exported %s by make_regs.py' % (
        module['id'], module['placement']['board'], module['placement']['connector'],
        prod['reference'], prod['date'], time.strftime('%Y-%m-%d')),
        '# AEC/AGC %s; AWB %s' % ('FROZEN at the tuning exposure (diagnostic only)' if keep_exposure else 'freeze dropped (auto exposure)', 'manual gains kept' if awb == 'manual' else 'auto (manual block dropped)')]
    n = 0
    for k in sorted(regs, key=lambda s: int(s, 16)):
        a = int(k, 16)
        if a in EXPOSURE and not keep_exposure:
            continue
        if awb != 'manual' and a in AWB_MANUAL:
            continue
        lines.append('%04X %02X' % (a, int(regs[k])))
        n += 1
    return '\n'.join(lines) + '\n', n


def upload(path_on_board, text):
    from board_repl import Repl
    r = Repl()
    r.value('sentai.fs.mkdir("/cam")') if r.value('sentai.fs.exists("/cam")') != 'True' else None
    data = text.encode()
    # chunked hex append (REPL line limit); the first write creates / truncates the file
    r.value('sentai.fs.write(%r, b"")' % path_on_board)
    for i in range(0, len(data), 96):
        chunk = data[i:i + 96].hex()
        r.cmd('_h=%r; sentai.fs.append(%r, bytes(int(_h[k:k+2], 16) for k in range(0, len(_h), 2)))' % (chunk, path_on_board), 10)
    size = r.value('sentai.fs.size(%r)' % path_on_board)
    r.value('sentai.fs.sync()')
    r.close()
    return int(size) if size.isdigit() else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'out'))
    ap.add_argument('--awb', choices=['auto', 'manual'], default='manual',
                    help='manual (default, measured best: keeps the tuned AWB gains) or auto (drop 0x3400-0x3406)')
    ap.add_argument('--upload', action='store_true')
    ap.add_argument('--keep-exposure', action='store_true', help='diagnostic: keep the AEC/AGC freeze of the tuning session')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    reg = json.load(open(os.path.join(HERE, 'modules.json')))
    for m in reg['modules']:
        conn = m['placement']['connector']          # cam0 / cam1
        text, n = export(m, a.awb, a.keep_exposure)
        p = os.path.join(a.out, '%s.regs' % conn)
        open(p, 'w').write(text)
        print('%s -> %s (%d registers)' % (m['id'], p, n))
        if a.upload:
            size = upload('/cam/%s.regs' % conn, text)
            print('   uploaded /cam/%s.regs: %d bytes (%s)' % (conn, size, 'ok' if size == len(text.encode()) else 'SIZE MISMATCH'))


if __name__ == '__main__':
    main()
