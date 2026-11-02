#!/usr/bin/env python3
"""Side-by-side colour comparison from a measurement directory (tune_colors.py --stages
measure): for every ColorChecker patch, the sRGB reference swatch next to the ACTUAL pixels
cropped from each camera's capture (no synthetic rendering of the camera colour), with the
per-camera dE00.  Layout = the chart (4 rows x 6 columns); each cell = [ref | cam0 | cam1].

Usage: python3 compare_patches.py out/measure_v2_manual [--out out/compare_v2_manual.png]
"""
import argparse
import json
import os

from PIL import Image, ImageDraw, ImageFont

ORDER = ['dark skin', 'light skin', 'blue sky', 'foliage', 'blue flower', 'bluish green',
         'orange', 'purplish blue', 'moderate red', 'purple', 'yellow green', 'orange yellow',
         'blue', 'green', 'red', 'yellow', 'magenta', 'cyan',
         'white', 'neutral 8', 'neutral 6.5', 'neutral 5', 'neutral 3.5', 'black']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--out')
    ap.add_argument('--tile', type=int, default=72)
    ap.add_argument('--title', default='')
    a = ap.parse_args()
    cams = {}
    for c in (0, 1):
        js = os.path.join(a.dir, 'measure_patches_cam%d.json' % c)
        cap = [f for f in sorted(os.listdir(a.dir)) if f.startswith('tune_cam%d_' % c) and f.endswith('.jpg')]
        cams[c] = (json.load(open(js)), Image.open(os.path.join(a.dir, cap[-1])).convert('RGB'))
    T = a.tile; gap = 6; label_h = 30; cols, rows = 6, 4
    cell_w, cell_h = 3 * T + 2 * 2, T + label_h
    W = cols * cell_w + (cols + 1) * gap
    H = rows * cell_h + (rows + 1) * gap + 44
    img = Image.new('RGB', (W, H), (24, 24, 24))
    dr = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 11)
        font_b = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 13)
    except Exception:
        font = font_b = ImageFont.load_default()
    title = a.title or ('%s — each cell: sRGB reference | cam0 capture crop | cam1 capture crop (dE00 cam0 / cam1)' % os.path.basename(a.dir.rstrip('/')))
    dr.text((gap, 8), title, fill=(230, 230, 230), font=font_b)
    de = {0: [], 1: []}
    for i, name in enumerate(ORDER):
        r, c = divmod(i, cols)
        x0 = gap + c * (cell_w + gap); y0 = 44 + gap + r * (cell_h + gap)
        tgt = tuple(int(v) for v in cams[0][0][name]['target'])
        dr.rectangle([x0, y0, x0 + T - 1, y0 + T - 1], fill=tgt)
        for k, cam in enumerate((0, 1)):
            d, im = cams[cam]
            bx = d[name]['bbox']
            crop = im.crop((int(round(bx[0])), int(round(bx[1])), int(round(bx[2])), int(round(bx[3]))))
            crop = crop.resize((T, T), Image.BILINEAR)
            img.paste(crop, (x0 + (k + 1) * (T + 2), y0))
            de[cam].append(d[name]['dE00'])
        dr.text((x0, y0 + T + 3), name, fill=(220, 220, 220), font=font)
        dr.text((x0, y0 + T + 16), 'dE00 %.1f / %.1f' % (cams[0][0][name]['dE00'], cams[1][0][name]['dE00']),
                fill=(190, 190, 190), font=font)
    m0 = sum(de[0]) / len(de[0]); m1 = sum(de[1]) / len(de[1])
    dr.text((gap, 26), 'mean dE00: cam0 %.1f, cam1 %.1f   (crops are the real pixels of the captures; each tile %d px)' % (m0, m1, T),
            fill=(180, 180, 180), font=font)
    out = a.out or os.path.join(a.dir, 'compare.png')
    img.save(out)
    print(out, img.size, 'mean dE00 cam0 %.1f cam1 %.1f' % (m0, m1))


if __name__ == '__main__':
    main()
