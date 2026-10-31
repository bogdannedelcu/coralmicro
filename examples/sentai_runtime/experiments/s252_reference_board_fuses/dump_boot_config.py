#!/usr/bin/env python3
"""EXP-s252 — boot-configuration dump: SRC boot/reset registers + OCOTP fuse
shadow words, read from the running firmware with sentai.sys.peek32.

Purpose (OP-S10-W22-T4/T9): compare a board that boots from flash (fuses
burnt by the manufacturer / colleague) with the bring-up unit that always
drops into the ROM serial downloader.  The dump is read-only; the firmware
image is RAM-loaded and nothing on the reference board's flash or fuses is
touched.  OCOTP_ReadFuseShadowRegister is not needed: the shadow words are
memory-mapped at OCOTP_BASE + 0x800 + 16*n (MIMXRT1176_cm7.h FUSEN[144]).

Run (board alone on USB, build >= 1648 RAM-loaded):
    python3 dump_boot_config.py --label good_board
    python3 dump_boot_config.py --label bringup_unit
    python3 dump_boot_config.py --diff good_board.csv bringup_unit.csv

Interesting words (i.MX RT1170 fusemap): BOOT_CFG (0x960/0x970 region ->
shadow words 0x18/0x1C per the RM), BT_FUSE_SEL; SRC_SBMR2 bit decode per
EXP-s248.  We dump everything and let the diff speak.
"""
import argparse
import csv
import glob
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 's249_hardening_regression'))
import repl_regression as rr  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_BASE = 0x40C04000
SRC_REGS = [('SCR', 0x0), ('SRMR', 0x4), ('SBMR1', 0x8), ('SBMR2', 0xC), ('SRSR', 0x10)]
OCOTP_FUSE0 = 0x40CAC800
N_FUSES = 144


def dump(label):
    ports = sorted(glob.glob('/dev/ttyACM*'))
    if not ports:
        print('no /dev/ttyACM* -- board not up'); return 2
    r = rr.Repl(ports[0])
    ver = r.one('print(sentai.version())')
    print(ver)
    rows = [('version', '', ver)]
    for name, off in SRC_REGS:
        v = r.one('print(hex(sentai.sys.peek32(0x%08X)))' % (SRC_BASE + off), 6)
        rows.append(('SRC_' + name, '0x%08X' % (SRC_BASE + off), v))
        print('SRC_%-6s %s' % (name, v))
    # GPR1..20 (boot counter, breadcrumbs) for context
    for i in range(20):
        v = r.one('print(hex(sentai.sys.peek32(0x%08X)))' % (SRC_BASE + 0x14 + 4 * i), 6)
        rows.append(('SRC_GPR%d' % (i + 1), '0x%08X' % (SRC_BASE + 0x14 + 4 * i), v))
    # Fuse shadow words, one REPL round trip each (~0.1 s; a pasted on-board
    # loop lost its output on the first attempt -- keep it simple).  The
    # OCOTP clock is on in this firmware (CTRL reads 0x8AD), so the shadow
    # window is readable without enabling anything.
    got = {}
    for n in range(N_FUSES):
        v = r.one('print(hex(sentai.sys.peek32(0x%08X)))' % (OCOTP_FUSE0 + 16 * n), 6)
        if v.startswith('0x'):
            got[n] = v
    missing = [n for n in range(N_FUSES) if n not in got]
    for n in range(N_FUSES):
        rows.append(('OCOTP_FUSE%03d' % n, '0x%08X' % (OCOTP_FUSE0 + 16 * n), got.get(n, '')))
    print('fuse words read: %d/%d%s' % (len(got), N_FUSES,
          ('  missing: %s' % missing[:10]) if missing else ''))
    nz = [(n, v) for n, v in sorted(got.items()) if v not in ('0x0', '0x00000000')]
    print('non-zero fuse words (%d):' % len(nz))
    for n, v in nz:
        print('  FUSE%03d @0x%08X = %s' % (n, OCOTP_FUSE0 + 16 * n, v))
    path = os.path.join(HERE, label + '.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['name', 'address', 'value'])
        w.writerows(rows)
    print('wrote', path)
    return 0


def diff(a, b):
    def load(p):
        with open(p) as f:
            return {r['name']: r['value'] for r in csv.DictReader(f)}
    da, db = load(a), load(b)
    print('%-16s %-14s %-14s' % ('register', os.path.basename(a), os.path.basename(b)))
    for k in da:
        if k == 'version':
            continue
        if da[k] != db.get(k, ''):
            print('%-16s %-14s %-14s' % (k, da[k], db.get(k, '')))
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='board')
    ap.add_argument('--diff', nargs=2, metavar=('A.csv', 'B.csv'))
    a = ap.parse_args()
    sys.exit(diff(*a.diff) if a.diff else dump(a.label))
