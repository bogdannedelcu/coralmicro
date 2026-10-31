#!/usr/bin/env python3
# Merge s241 on-board speed (results_all_vga45.csv) with MLflow float-model
# mAP (sentai_revision_relu_v1 exp8 / sentai_yuv420_v1 exp13) into the paper
# table. Emits paper_table.md (markdown) + paper_table.tex (LaTeX booktabs).
import csv, os
HERE = os.path.dirname(os.path.abspath(__file__))

# MLflow float-model mAP@.5 / mAP@.5:.95 (imgsz 640x480, act=relu)
MAP = {
 ('RGB','msblock'):(0.645,0.228),('RGB','c3'):(0.683,0.261),('RGB','c2f'):(0.702,0.278),
 ('RGB','c2f_pan2'):(0.707,0.276),('RGB','gelan'):(0.690,0.266),('RGB','c2f_thick'):(0.803,0.360),
 ('RGB','gelan_pan2'):(0.661,0.255),('RGB','c2f_deep'):(0.705,0.283),
 ('YUV','msblock'):(0.522,0.158),('YUV','c3'):(0.700,0.258),('YUV','c2f'):(0.720,0.275),
 ('YUV','c2f_pan2'):(0.707,0.269),('YUV','gelan'):(0.734,0.285),('YUV','c2f_thick'):(0.823,0.368),
 ('YUV','gelan_pan2'):(0.712,0.261),('YUV','c2f_deep'):(0.736,0.282),
}
# c2f_deep RGB: pipeline unstable on-board (scratch-spill 153600 wedges the pipeline
# under camera+prep contention); isolated invoke measured 35.0+-0.0 ms (n=30).
UNSTABLE = {('RGB','c2f_deep'): 35.0}
ORDER = ['msblock','c3','c2f','c2f_pan2','gelan','c2f_thick','gelan_pan2','c2f_deep']

rows = {}
with open(os.path.join(HERE,'results_all_vga45.csv')) as f:
    for r in csv.DictReader(f):
        rows[(r['fmt'], r['model'])] = r

def cell(fmt, arch):
    key = (fmt, arch)
    m5, m95 = MAP.get(key, (None,None))
    if key in rows:
        r = rows[key]
        inv = f"{float(r['invoke_mean_ms']):.1f}±{float(r['invoke_std_ms']):.1f}"
        fps = f"{float(r['fps_mean']):.1f}±{float(r['fps_std']):.2f}"
    elif key in UNSTABLE:
        inv = f"{UNSTABLE[key]:.1f}±0.0"; fps = "unstable*"
    else:
        inv = "—"; fps = "—"
    return m5, m95, inv, fps

# ---- markdown ----
md = ["# s241 — ReLU (RGB) vs YUV420 detector performance on VGA45 (build 1590)\n",
      "On-board i.MX RT1176, single-EP EdgeTPU/USB2, camera 640×480@45fps (verified).",
      "Isolated invoke: N=30 GPT1-µs-timed invokes. Pipeline FPS: M=3 trials × 6 s,",
      "frame_count/wall, full camera→PXP→TPU pipeline live. mean±σ.\n",
      "| arch | fmt | mAP@.5 | mAP@.5:.95 | invoke ms | pipeline FPS |",
      "|---|---|--:|--:|--:|--:|"]
for fmt in ('RGB','YUV'):
    for a in ORDER:
        m5,m95,inv,fps = cell(fmt,a)
        md.append(f"| {a} | {fmt} | {m5:.3f} | {m95:.3f} | {inv} | {fps} |")
md.append("\n\\* c2f_deep RGB: pipeline unstable (scratch-spill 153600 B wedges the pipeline "
          "under camera+prep contention); isolated invoke 35.0±0.0 ms. Its YUV counterpart "
          "(smaller, no spill) runs fine at 38.9 fps.\n")
# per-arch RGB->YUV comparison
md.append("## Per-architecture RGB → YUV (same weights, half the input transfer)\n")
md.append("| arch | ΔmAP@.5 | RGB FPS | YUV FPS | FPS speedup |")
md.append("|---|--:|--:|--:|--:|")
for a in ORDER:
    r=rows.get(('RGB',a)); y=rows.get(('YUV',a))
    dm = MAP[('YUV',a)][0]-MAP[('RGB',a)][0]
    rf = f"{float(r['fps_mean']):.1f}" if r else "n/a"
    yf = f"{float(y['fps_mean']):.1f}" if y else "n/a"
    sp = f"{float(y['fps_mean'])/float(r['fps_mean']):.2f}×" if (r and y) else "—"
    md.append(f"| {a} | {dm:+.3f} | {rf} | {yf} | {sp} |")
open(os.path.join(HERE,'paper_table.md'),'w').write("\n".join(md)+"\n")

# ---- LaTeX ----
tex = [r"% s241 ReLU-RGB vs YUV420, VGA45, i.MX RT1176 build 1590",
       r"\begin{table}[t]\centering",
       r"\caption{On-board detector performance (i.MX RT1176, EdgeTPU/USB2, 640$\times$480@45\,fps). "
       r"Isolated invoke and full-pipeline FPS, mean$\pm\sigma$ ($N{=}30$ invokes, $M{=}3$ FPS trials). "
       r"mAP is the float model (person, 1 class).}",
       r"\label{tab:s241}",
       r"\begin{tabular}{llrrrr}",
       r"\toprule",
       r"arch & fmt & mAP$_{.5}$ & mAP$_{.5:.95}$ & invoke (ms) & FPS \\",
       r"\midrule"]
for fmt in ('RGB','YUV'):
    for a in ORDER:
        m5,m95,inv,fps = cell(fmt,a)
        inv=inv.replace('±',r'$\pm$'); fps=fps.replace('±',r'$\pm$').replace('unstable*',r'unstable$^{*}$')
        tex.append(f"{a.replace('_',chr(92)+'_')} & {fmt} & {m5:.3f} & {m95:.3f} & {inv} & {fps} \\\\")
    if fmt=='RGB': tex.append(r"\midrule")
tex += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
open(os.path.join(HERE,'paper_table.tex'),'w').write("\n".join(tex)+"\n")

print(open(os.path.join(HERE,'paper_table.md')).read())
print("\n[written paper_table.md + paper_table.tex]")
