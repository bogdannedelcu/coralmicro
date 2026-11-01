#!/usr/bin/env python3
"""Printable A4 (landscape) colour chart: the 24 ColorChecker Classic patches at their sRGB
values, 6 x 4, on a mid-grey background, with four black corner fiducials (for homography
from the corners instead of Gray-code stripes), patch labels and the reference values table.

Caveat printed on the sheet: an office printer is not calibrated; the printed patches are a
reflective target under real illumination, not guaranteed sRGB.  For absolute colour use an
X-Rite / Calibrite ColorChecker.

Usage: python3 make_chart_pdf.py [--out patterns/colorchecker_a4.pdf] [--dpi 300]
"""
import argparse
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from show_pattern import PATCHES  # noqa: E402

A4_MM = (297.0, 210.0)  # landscape


def font(size):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'patterns', 'colorchecker_a4.pdf'))
    ap.add_argument('--dpi', type=int, default=300)
    a = ap.parse_args()
    px = lambda mm: int(round(mm / 25.4 * a.dpi))  # noqa: E731
    W, H = px(A4_MM[0]), px(A4_MM[1])
    img = Image.new('RGB', (W, H), (118, 118, 118))
    d = ImageDraw.Draw(img)
    # chart area: 6 x 4 patches of 32 mm, 3 mm gaps, centred; fiducials 12 mm squares in the corners
    pw, gap, cols, rows = px(32), px(3), 6, 4
    cw, ch = cols * pw + (cols - 1) * gap, rows * pw + (rows - 1) * gap
    x0, y0 = (W - cw) // 2, px(24)
    fid = px(12)
    rects = []
    for i, (name, rgb) in enumerate(PATCHES):
        c, r = i % cols, i // cols
        x, y = x0 + c * (pw + gap), y0 + r * (pw + gap)
        d.rectangle([x, y, x + pw - 1, y + pw - 1], fill=rgb)
        d.text((x + px(1.5), y + pw - px(5)), '%d' % (i + 1), fill=(255, 255, 255) if sum(rgb) < 380 else (0, 0, 0), font=font(px(3)))
        rects.append({'name': name, 'srgb': rgb, 'rect_mm': [round((x - x0) / a.dpi * 25.4, 1), round((y - y0) / a.dpi * 25.4, 1), 32, 32]})
    # fiducials: black squares outside the chart corners, with a white inner dot for sub-pixel centring
    for fx, fy in ((x0 - fid - px(4), y0 - fid - px(4)), (x0 + cw + px(4), y0 - fid - px(4)),
                   (x0 - fid - px(4), y0 + ch + px(4)), (x0 + cw + px(4), y0 + ch + px(4))):
        d.rectangle([fx, fy, fx + fid, fy + fid], fill=(0, 0, 0))
        d.ellipse([fx + fid * 0.4, fy + fid * 0.4, fx + fid * 0.6, fy + fid * 0.6], fill=(255, 255, 255))
    # title + table
    f, fs = font(px(4)), font(px(2.6))
    d.text((x0, px(10)), 'SentAI s265 colour chart - ColorChecker Classic sRGB values, 32 mm patches, 300 dpi',
           fill=(0, 0, 0), font=f)
    ty = y0 + ch + px(22)
    d.text((x0, ty), 'Reference sRGB (R,G,B) per patch, row by row:', fill=(0, 0, 0), font=fs)
    for r in range(rows):
        line = '   '.join('%2d %s %s' % (r * cols + c + 1, PATCHES[r * cols + c][0], PATCHES[r * cols + c][1]) for c in range(cols))
        d.text((x0, ty + px(4) * (r + 1)), line, fill=(0, 0, 0), font=fs)
    d.text((x0, ty + px(4) * 5.5), 'Print at 100 % scale (no fit-to-page), matte paper. An office printer is not calibrated: '
           'the patches are a reflective target under real light, not guaranteed sRGB.', fill=(0, 0, 0), font=fs)
    d.text((x0, ty + px(4) * 6.3), 'For absolute colour use an X-Rite / Calibrite ColorChecker.', fill=(0, 0, 0), font=fs)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    img.save(a.out, 'PDF', resolution=a.dpi)
    img.save(a.out.replace('.pdf', '.png'))
    json.dump({'patch_mm': 32, 'gap_mm': 3, 'fiducial_mm': 12, 'cols': cols, 'rows': rows, 'patches': rects},
              open(a.out.replace('.pdf', '.json'), 'w'), indent=1)
    print(a.out, '%dx%d px' % (W, H))


if __name__ == '__main__':
    main()
