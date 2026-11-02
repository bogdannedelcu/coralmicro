#!/usr/bin/env python3
"""Merge a tune_colors.py result (incremental `tuned_regs_cam<N>.json`, only the registers the
stages changed) onto the module's current production set and register the new set in
modules.json (W24-T22 registry).

Usage:
  python3 merge_set.py --tuned <dir with tuned_regs_cam0/1.json> --name kamvas45 --date 2026-11-02 \
                       --reference "Kamvas 16 sRGB at 45 cm, grid central 70 %" [--mode grid] [--no-production]
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tuned', required=True)
    ap.add_argument('--name', required=True)
    ap.add_argument('--date', required=True)
    ap.add_argument('--reference', required=True)
    ap.add_argument('--mode', default='grid')
    ap.add_argument('--no-production', action='store_true', help='register the set without making it the production one')
    ap.add_argument('--quality', default='{}', help='JSON dict of quality numbers to attach, e.g. {"de00": 6.1}')
    a = ap.parse_args()
    reg_path = os.path.join(HERE, 'modules.json')
    reg = json.load(open(reg_path))
    for m in reg['modules']:
        cam = int(m['placement']['connector'][-1])
        tuned_path = os.path.join(a.tuned, 'tuned_regs_cam%d.json' % cam)
        if not os.path.exists(tuned_path):
            print('%s: no %s, skipped' % (m['id'], tuned_path)); continue
        base_entry = [s for s in m['register_sets'] if s.get('production')][0]
        base = json.load(open(os.path.join(HERE, base_entry['file'])))
        delta = json.load(open(tuned_path))
        merged = dict(base)
        changed = 0
        for k, v in delta.items():
            kk = '0x%04X' % int(k, 16)
            if merged.get(kk) != int(v):
                changed += 1
            merged[kk] = int(v)
        out_rel = 'regs/%s_%s_%s.json' % (m['id'], a.name, a.date)
        json.dump(merged, open(os.path.join(HERE, out_rel), 'w'), indent=1)
        entry = {'file': out_rel, 'reference': a.reference, 'mode': a.mode, 'regs': len(merged), 'date': a.date,
                 'production': not a.no_production, 'based_on': base_entry['file'], 'changed_vs_base': changed,
                 'quality': json.loads(a.quality)}
        if not a.no_production:
            for s in m['register_sets']:
                s['production'] = False
        m['register_sets'].append(entry)
        print('%s: %s (%d regs, %d changed vs %s)%s' % (m['id'], out_rel, len(merged), changed, base_entry['file'],
                                                       '' if a.no_production else ' -> PRODUCTION'))
    json.dump(reg, open(reg_path, 'w'), indent=2); open(reg_path, 'a').write('\n')


if __name__ == '__main__':
    main()
