#!/usr/bin/env python3
"""OP-S10-W15 -- ARM memory budget report for the SentAI firmware ELF.

Prints how much of every physical region the link consumed and how much is
left, from `arm-none-eabi-size -A` plus the region table of the linker script
(MIMXRT1176xxxxx_cm7_ram_mp.ld).  Exit code 1 when any region's free space
drops below its guard (default 4 KB for ITCM/DTCM/OCRAM) so a CI step or a
pre-commit run catches "the next buffer will not fit" before the link fails
with an opaque `region overflowed` message.

Usage:
  python3 scripts/mem_budget.py [build/examples/sentai_runtime/sentai_runtime] [--guard-kb 4] [--top N]

Section -> region mapping follows the linker script: ITCM = .interrupts/.text/
.ramfunc/.ARM/.init_array/.fini_array; DTCM = .ncache (fixed 32 KB window) +
.data/.noinit_boot_persist/.bss/.stack/.noinit_crash_text; OCRAM = everything
at 0x2024xxxx..; SDRAM code+bss = 0x80E00000.. (m_sdram), heap 0x80000000,
camera window 0x82000000.
"""
import argparse, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIZE = os.path.join(ROOT, 'third_party', 'toolchain-linux', 'gcc-arm-none-eabi-9-2020-q2-update', 'bin', 'arm-none-eabi-size')
NM = SIZE.replace('arm-none-eabi-size', 'arm-none-eabi-nm')

# (name, origin, length) -- keep in sync with the MEMORY block of the .ld.
NCACHE = 0x8000
HEAP = 0x00E00000
RPMSG = 0x4000
REGIONS = [
    ('ITCM  m_text',     0x00000800, 0x00000400 + 0x0003F400),
    ('DTCM  m_ncache',   0x20000000, NCACHE),
    ('DTCM  m_data',     0x20000000 + NCACHE, 0x00040000 - NCACHE),
    ('OCRAM m_ocram',    0x20240000, 0x00100000 - RPMSG),
    ('OCRAM rpmsg',      0x20340000 - RPMSG, RPMSG),
    ('SDRAM m_heap',     0x80000000, HEAP),
    ('SDRAM m_sdram',    0x80000000 + HEAP, 0x02000000 - HEAP),
    ('SDRAM m_ncamera',  0x82000000, 0x01800000),
]


def sections(elf):
    out = subprocess.check_output([SIZE, '-A', elf], text=True)
    secs = []
    for line in out.splitlines()[2:]:
        p = line.split()
        if len(p) != 3 or p[0] == 'Total':
            continue
        name, size, addr = p[0], int(p[1]), int(p[2])
        if size == 0 or name.startswith('.debug') or name in ('.comment', '.ARM.attributes', '.stab', '.stabstr'):
            continue
        secs.append((name, size, addr))
    return secs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', nargs='?', default=os.path.join(ROOT, 'build', 'examples', 'sentai_runtime', 'sentai_runtime'))
    ap.add_argument('--guard-kb', type=float, default=4.0, help='minimum free space in ITCM/DTCM/OCRAM')
    ap.add_argument('--top', type=int, default=0, help='also list the N largest symbols of the tight regions')
    a = ap.parse_args()
    secs = sections(a.elf)
    rc = 0
    print('%-16s %10s %10s %10s %6s   sections' % ('region', 'size', 'used', 'free', 'use%'))
    for name, org, ln in REGIONS:
        used = [(s, sz) for s, sz, addr in secs if org <= addr < org + ln]
        tot = sum(sz for _, sz in used)
        free = ln - tot
        pct = 100.0 * tot / ln if ln else 0
        flag = ''
        if name.split()[0] in ('ITCM', 'DTCM', 'OCRAM') and free < a.guard_kb * 1024 and 'rpmsg' not in name and 'ncache' not in name:
            flag = '  <-- below guard'
            rc = 1
        print('%-16s %10d %10d %10d %5.1f%%   %s%s' % (name, ln, tot, free, pct, ' '.join(s for s, _ in used)[:60], flag))
    if a.top:
        nm = subprocess.check_output([NM, '-S', '-C', '--size-sort', a.elf], text=True)
        rows = []
        for l in nm.splitlines():
            p = l.split(None, 3)
            if len(p) < 4:
                continue
            try:
                rows.append((int(p[0], 16), int(p[1], 16), p[2], p[3]))
            except ValueError:
                pass
        for name, org, ln in REGIONS[:4]:
            sel = sorted([r for r in rows if org <= r[0] < org + ln], key=lambda r: -r[1])[:a.top]
            print('\n-- %s: %d largest symbols' % (name, a.top))
            for addr, sz, t, sym in sel:
                print('   %8d  %s  %s' % (sz, t, sym[:80]))
    return rc


if __name__ == '__main__':
    sys.exit(main())
