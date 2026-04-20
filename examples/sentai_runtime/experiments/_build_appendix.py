#!/usr/bin/env python3
"""Generate appendix-ready markdown tables from the experiments/ tree."""
import csv, io, pathlib, statistics, re

ROOT = pathlib.Path('/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments')

def read_rows(p):
    try: return list(csv.DictReader(p.read_text().splitlines()))
    except Exception: return []

def col_int(rows, col, skip=0):
    out = []
    for r in rows[skip:]:
        try: out.append(int(r[col]))
        except (KeyError, ValueError, TypeError):
            try: out.append(int(float(r[col])))
            except: pass
    return out

def stats(xs):
    if not xs: return None
    return {'mean':statistics.mean(xs),
            'stdev':statistics.stdev(xs) if len(xs)>1 else 0,
            'min':min(xs), 'max':max(xs),
            'median':statistics.median(xs), 'n':len(xs)}

def csv_link(sess_name, fname):
    return '[`%s`](%s/%s)' % (fname, sess_name, fname)

def e18_session(sess):
    def sw(name):
        p = sess / name
        if not p.exists(): return None
        rows = read_rows(p)[1:]
        return {
            'sel': stats(col_int(rows, 'select_ms')),
            'ten': stats(col_int(rows, 'to_tensor_ms')),
            'inv': stats(col_int(rows, 'invoke_ms')),
            'det': stats(col_int(rows, 'detect_ms')),
            'tot': stats(col_int(rows, 'total_frame_ms')),
            'path': name,
        }
    return {
        'A': sw('001_e18_A_fixed_cam0.csv'),
        'B': sw('002_e18_B_fixed_cam1.csv'),
        'C': sw('003_e18_C_alt_cam0_cam1.csv'),
    }

def e16_session(sess):
    p = sess / '002_e16_camswitch_cam0_cam1.csv'
    if not p.exists():
        p = sess / '001_e16_camswitch_cam0_cam1.csv'
    if not p.exists(): return None
    rows = read_rows(p)[1:]
    cam0 = [r for r in rows if r.get('cam_id') == '0']
    cam1 = [r for r in rows if r.get('cam_id') == '1']
    return {
        'sel': stats(col_int(rows, 'select_ms')),
        'ten': stats(col_int(rows, 'to_tensor_ms')),
        'inv': stats(col_int(rows, 'invoke_ms')),
        'det': stats(col_int(rows, 'detect_ms')),
        'tot': stats(col_int(rows, 'total_frame_ms')),
        'cam0_tot': stats(col_int(cam0, 'total_frame_ms')),
        'cam1_tot': stats(col_int(cam1, 'total_frame_ms')),
        'path': p.name,
    }

def e15_session(sess):
    p = sess / '001_e15_pipeline_par_cam0_512x512.csv'
    if not p.exists(): return None
    rows = read_rows(p)
    if len(rows) > 2: rows = rows[2:]
    return {
        'iv': stats(col_int(rows, 'frame_interval_ms')),
        'ps': stats(col_int(rows, 'prep_stall_ms')),
        'is': stats(col_int(rows, 'infer_stall_ms')),
        'path': p.name,
    }

def e17_session(sess):
    out = {}
    for thr in (1, 2):
        found = None
        for cand in sorted(sess.glob('*_e17_switch_drain_t%d.csv' % thr)):
            found = cand; break
        if not found: continue
        rows = read_rows(found)
        fdir = sess / ('e17_t%d_frames' % thr)
        out['t%d' % thr] = {
            'el': stats(col_int(rows, 'elapsed_ms')),
            'sz': stats(col_int(rows, 'jpeg_bytes')),
            'path': found.name,
            'n_frames': len(list(fdir.glob('*.jpg'))) if fdir.exists() else 0,
        }
    return out

def e14_session(sess):
    csvs = sorted(sess.glob('*_e14_pipeline_par_*.csv'))
    if not csvs: return None
    per_run = []
    for p in csvs:
        rows = read_rows(p)
        if len(rows) > 2: rows = rows[2:]
        iv = col_int(rows, 'frame_interval_ms')
        s = stats(iv)
        if s and s['mean'] > 0:
            per_run.append((p.name, s['mean'], 1000/s['mean']))
    return per_run

sessions = sorted(ROOT.glob('s[0-9][0-9][0-9]_*'))
by_group = {'pre_e15': [], 'e15': [], 'e16': [], 'e17': [], 'e18': [], 'mixed': []}
for s in sessions:
    n = s.name
    if re.search(r'e15_vs_e16', n):       by_group['mixed'].append(s)
    elif re.search(r'e1[5]_(512|x20)', n):  by_group['e15'].append(s)
    elif re.search(r'e16', n):             by_group['e16'].append(s)
    elif re.search(r'e17', n):             by_group['e17'].append(s)
    elif re.search(r'e18', n):             by_group['e18'].append(s)
    else:                                   by_group['pre_e15'].append(s)

out = []
A = out.append

A('## Appendix A — Pre-E15 groundwork sessions (subsystem probes)')
A('')
A('Early probes on individual subsystems before the parallel vision')
A('pipeline was established.  All sessions that involve a model use the')
A('80-class `yolo26n.edgetpu_1.tflite`.  Each table row is a single')
A('experiment inside that session; the `Summary` column is verbatim from')
A("the `manifest.csv` on the device.")
A('')
for sess in by_group['pre_e15']:
    m = read_rows(sess / 'manifest.csv')
    A('### `%s`' % sess.name)
    A('')
    if not m:
        A('*Empty session (no experiments recorded).*')
        A('')
        continue
    A('| # | Experiment | Summary | CSV |')
    A('|---|---|---|---|')
    for row in m:
        fname = row['file'].replace('/diags/%s/' % sess.name, '')
        A('| %s | `%s` | %s | [`%s`](%s/%s) |' % (
            row['seq'], row['experiment'],
            row['summary'] if row['summary'] else '—',
            fname, sess.name, fname))
    A('')

A('')
A('## Appendix B — E14 parallel-pipeline at 320×320, yolo26n 80-class')
A('')
A('E14 runs `sentai.pipeline.start/get/stop` with the firmware')
A('`PrepTask + InferTask` double-buffer.  Measurement is the wall-clock')
A('interval between successive `pipeline.get()` returns.  All sessions')
A('below use `yolo26n.edgetpu_1.tflite` at 320×320, 20 repetitions each')
A('inside a run, and multiple runs per session.  The 80-class model is')
A('slower than the 1-class 512×512 model that later replaces it in E15 —')
A('these runs consistently land around **6.4 FPS** because the 80-class')
A('output head does not fit entirely on the EdgeTPU and falls back to')
A('CPU execution for several layers.  That was the motivation for the')
A('model swap in E15.')
A('')
for sess in sessions:
    if not re.search(r'e14_x20', sess.name): continue
    runs = e14_session(sess)
    if not runs: continue
    A('### `%s` — %d runs' % (sess.name, len(runs)))
    A('')
    A('| Run file | Mean frame_interval (ms) | FPS |')
    A('|---|---:|---:|')
    for fname, ms, fps in runs:
        A('| %s | %.2f | %.2f |' % (csv_link(sess.name, fname), ms, fps))
    A('')

A('')
A('## Appendix C — E15 parallel pipeline at 512×512, 1-class model')
A('')
A('E15 replays E14 on the faster single-class model')
A('`yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`.')
A('This model is fully edgetpu-compiled — no CPU fallback — so the')
A('bottleneck shifts from inference to the SDRAM→tensor memcpy.')
A('')
A('**Two eras in the data below:**')
A('')
A('| Era | Sessions | Typical FPS | Why |')
A('|---|---|---:|---|')
A('| Pre-eDMA (CPU memcpy 24 ms) | `s016`-`s020` (single-shot), `s024`, `s026` (x20) | ~6.4 | Invoke is fast but the 786 KB `memcpy(tensor, staging)` on CPU through the D-cache burns ~24 ms per frame |')
A('| Post-eDMA (32-byte AXI bursts) | `s027`-`s030` (single-shot), `s032` (x20) | ~14–15 | 786 KB copy drops to ~14.6 ms; pipeline hits sensor rate ([paper/memcpy.md](../paper/memcpy.md) § "Optimised") |')
A('')
A('')
for sess in sorted(by_group['e15']):
    s = e15_session(sess)
    if not s: continue
    A('### `%s`' % sess.name)
    A('')
    A('| Metric | Mean ± σ | min / max | n |')
    A('|---|---:|---:|---:|')
    for label, key in [('frame_interval_ms', 'iv'),
                       ('prep_stall_ms', 'ps'),
                       ('infer_stall_ms', 'is')]:
        v = s[key]
        if v: A('| %s | %.2f ± %.2f | %d / %d | %d |' %
                (label, v['mean'], v['stdev'], v['min'], v['max'], v['n']))
    if s['iv']:
        A('| **FPS** (1000/mean_interval) | **%.2f** | — | — |' %
          (1000/s['iv']['mean']))
    A('')
    A('Raw: ' + csv_link(sess.name, s['path']))
    A('')

A('')
A('## Appendix D — E15 + E16 head-to-head sessions (pre/post Fix A)')
A('')
A('Both E15 (fixed camera) and E16 (alternating cam0↔cam1) run in the')
A('same session so the numbers are measured under identical scene,')
A('thermal state, and firmware.  `s034` is pre-Fix A, `s035` is post.')
A('The ~65 ms directional asymmetry in cam1 − cam0 was discovered here.')
A('')
for sess in by_group['mixed']:
    A('### `%s`' % sess.name)
    A('')
    s15 = e15_session(sess)
    if s15:
        A('**E15 — fixed cam0 parallel pipeline**')
        A('')
        A('| Metric | Mean ± σ | min / max | n |')
        A('|---|---:|---:|---:|')
        for label, key in [('frame_interval_ms', 'iv'), ('prep_stall_ms', 'ps'), ('infer_stall_ms', 'is')]:
            v = s15[key]
            if v: A('| %s | %.2f ± %.2f | %d / %d | %d |' % (label, v['mean'], v['stdev'], v['min'], v['max'], v['n']))
        if s15['iv']: A('| **FPS** | **%.2f** | — | — |' % (1000/s15['iv']['mean']))
        A('')
        A('Raw: ' + csv_link(sess.name, s15['path']))
        A('')
    s16 = e16_session(sess)
    if s16:
        A('**E16 — alternating cam0↔cam1 sequential**')
        A('')
        A('| Stage | Mean ± σ | min / max | n |')
        A('|---|---:|---:|---:|')
        for label, key in [('select_ms','sel'),('to_tensor_ms','ten'),('invoke_ms','inv'),('detect_ms','det'),('total_frame_ms','tot')]:
            v = s16[key]
            if v: A('| %s | %.2f ± %.2f | %d / %d | %d |' % (label, v['mean'], v['stdev'], v['min'], v['max'], v['n']))
        if s16['tot']: A('| **FPS** | **%.2f** | — | — |' % (1000/s16['tot']['mean']))
        A('')
        A('**Directional split:**')
        A('')
        A('| Direction | Mean total ms | n |')
        A('|---|---:|---:|')
        for d, key in [('→ cam0', 'cam0_tot'), ('→ cam1', 'cam1_tot')]:
            v = s16[key]
            if v: A('| %s | %.2f | %d |' % (d, v['mean'], v['n']))
        if s16['cam0_tot'] and s16['cam1_tot']:
            A('| **asymmetry (cam1 − cam0)** | **%+.2f ms** | — |' %
              (s16['cam1_tot']['mean'] - s16['cam0_tot']['mean']))
        A('')
        A('Raw: ' + csv_link(sess.name, s16['path']))
        A('')

A('')
A('## Appendix E — E16 alternating on Fix B (30 fps + flip-on-EOF)')
A('')
A('`s042_e16_eof_30fps_x40` — E16 alternating cam0↔cam1 at 30 fps with')
A('flip-on-EOF active.  First session where the directional asymmetry')
A('collapsed to ≤ 1 ms.')
A('')
sess = ROOT / 's042_e16_eof_30fps_x40'
if sess.exists():
    s = e16_session(sess)
    if s:
        A('| Stage | Mean ± σ | min / max | n |')
        A('|---|---:|---:|---:|')
        for label, key in [('select_ms','sel'),('to_tensor_ms','ten'),('invoke_ms','inv'),('detect_ms','det'),('total_frame_ms','tot')]:
            v = s[key]
            if v: A('| %s | %.2f ± %.2f | %d / %d | %d |' % (label, v['mean'], v['stdev'], v['min'], v['max'], v['n']))
        if s['tot']: A('| **FPS** | **%.2f** | — | — |' % (1000/s['tot']['mean']))
        A('')
        A('| Direction | Mean total ms | n |')
        A('|---|---:|---:|')
        for d, key in [('→ cam0', 'cam0_tot'), ('→ cam1', 'cam1_tot')]:
            v = s[key]
            if v: A('| %s | %.2f | %d |' % (d, v['mean'], v['n']))
        if s['cam0_tot'] and s['cam1_tot']:
            A('| **asymmetry (cam1 − cam0)** | **%+.2f ms** | — |' %
              (s['cam1_tot']['mean'] - s['cam0_tot']['mean']))
        A('')
        A('Raw: ' + csv_link(sess.name, s['path']))
        A('')

A('')
A('## Appendix F — E17 per-switch JPEGs (visual inspection)')
A('')
A('E17 captures alternating cam0↔cam1 JPEGs into MicroPython heap')
A('during the timing loop, then writes them to LFS after the loop.')
A('Two threshold values are tested per session (`switch_drain` 2 and 1).')
A('')
for sess_name in ('s036_e17_drain_ab','s037_e17_drain_ab','s038_e17_drain_ab',
                  's039_e17_drain_ab','s040_e17_drain_ab','s041_e17_eof_check'):
    sess = ROOT / sess_name
    if not sess.exists(): continue
    data = e17_session(sess)
    if not data:
        A('### `%s`' % sess_name)
        A('')
        A('*Empty session (driver failed to record frames).*')
        A('')
        continue
    A('### `%s`' % sess_name)
    A('')
    A('| Threshold | elapsed_ms mean ± σ | JPEG bytes mean | Frames saved | CSV |')
    A('|---:|---:|---:|---:|---|')
    for thr in (2, 1):
        v = data.get('t%d' % thr)
        if not v: continue
        el = v['el']; sz = v['sz']
        A('| %d | %.1f ± %.1f | %.0f | %d | %s |' % (
            thr,
            el['mean'] if el else 0, el['stdev'] if el else 0,
            sz['mean'] if sz else 0, v['n_frames'],
            csv_link(sess_name, v['path'])))
    A('')

A('')
A('## Appendix G — E18 head-to-tail (three-sweep benchmark)')
A('')
A('E18 runs three back-to-back sweeps in one session at identical')
A('firmware + scene + model: (A) fixed cam0, (B) fixed cam1,')
A('(C) alternating.  Per-switch overhead = C_total − max(A_total, B_total).')
A('')
for sess_name in ('s043_e18_headtail_drain2','s044_e18_headtail_drain2','s045_e18_post_refactor'):
    sess = ROOT / sess_name
    if not sess.exists(): continue
    d = e18_session(sess)
    if not (d['A'] and d['B'] and d['C']): continue
    A('### `%s`' % sess_name)
    A('')
    A('| Sweep | `select` | `to_tensor` | `invoke` | `detect` | `total` | FPS |')
    A('|---|---:|---:|---:|---:|---:|---:|')
    for label, key in [('A — fixed cam0','A'), ('B — fixed cam1','B'), ('C — alternating','C')]:
        s = d[key]
        if not s: continue
        A('| %s | %.1f | %.1f | %.1f | %.1f | **%.1f** | %.2f |' % (
            label, s['sel']['mean'], s['ten']['mean'], s['inv']['mean'],
            s['det']['mean'], s['tot']['mean'], 1000/s['tot']['mean']))
    base = max(d['A']['tot']['mean'], d['B']['tot']['mean'])
    oh = d['C']['tot']['mean'] - base
    A('')
    A('**Per-switch overhead** = C − max(A,B) = **%.1f ms** (%.1f %% of baseline)' %
      (oh, 100*oh/base))
    A('')
    A('CSVs: ' + csv_link(sess_name, '001_e18_A_fixed_cam0.csv') +
        ' · ' + csv_link(sess_name, '002_e18_B_fixed_cam1.csv') +
        ' · ' + csv_link(sess_name, '003_e18_C_alt_cam0_cam1.csv'))
    A('')

(ROOT / '_appendix.md').write_text('\n'.join(out))
print("wrote %d lines" % len(out))
