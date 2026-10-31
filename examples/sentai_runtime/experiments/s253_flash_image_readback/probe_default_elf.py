#!/usr/bin/env python3
"""EXP-s253: what does the NAND system partition hold in /default.elf?
Reads via sentai.sys.sysfs_* (build >= 1665): size, all "20xx-xx-xx hh:mm:ss"
build timestamps found in the stored ELF, SentAI version strings.
Usage: python3 probe_default_elf.py [--tag label]"""
import argparse, os, sys, time, re
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B
ap = argparse.ArgumentParser(); ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S')); a = ap.parse_args()
ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
log = open(os.path.join(HERE, 'probe_%s.log' % a.tag), 'w')
def q(line, t=20):
    out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush()
    return out
print(q('print("RUNNING", sentai.version())').strip().split('\n')[-2][:80])
print(q('print("SIZE", sentai.sys.sysfs_size("/default.elf"))').strip().split('\n')[-2][:80])
out = q('exec("p=\'/default.elf\'\\ns=0\\nhits=[]\\nwhile len(hits)<6:\\n  o=sentai.sys.sysfs_find(p,b\'2026-\',s)\\n  if o<0: break\\n  hits.append((o,sentai.sys.sysfs_read(p,o,19)))\\n  s=o+1\\nprint(\'HITS\',hits)")', 240)
for l in out.split('\n'):
    if l.strip().startswith('HITS'): print(l.strip()[:400])
out = q('exec("p=\'/default.elf\'\\no=sentai.sys.sysfs_find(p,b\'SentAI v1.\',0)\\nprint(\'VER\',o,sentai.sys.sysfs_read(p,max(o,0),64) if o>=0 else None)")', 240)
for l in out.split('\n'):
    if l.strip().startswith('VER'): print(l.strip()[:300])
