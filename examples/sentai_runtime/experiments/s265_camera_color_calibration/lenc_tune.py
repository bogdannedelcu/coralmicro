#!/usr/bin/env python3
"""LENC (lens / colour shading) tuning by measured Jacobian on a uniform field.

Registers: green matrix 0x5800-0x5823 (6x6, 6-bit, luminance gain per cell, matrix row 0 =
image bottom), BR matrix 0x5824-0x583C (5x5, high nibble = B gain, low nibble = R gain).
Procedure per camera: show a uniform field (grey on the screen, or white behind a paper
diffuser), measure mean R,G,B on a 6x6 grid of the frame, perturb every register once
(G +8, B nibble +3, R nibble +3 = 86 captures, ~5 min), build the Jacobian of the error
vector [G/Gc - target_lum, R/G - (R/G)c, B/G - (B/G)c] per cell (c = centre cells), solve a
damped least-squares update, clip to the register ranges, apply, verify.  The luminance target
is `--corner` of the centre at the corners (default 0.85, i.e. do not fully flatten luminance:
full flattening would amplify noise); colour ratios are fully flattened.

Usage: python3 lenc_tune.py --out iter1/lenc --cams 1 [--grey 150] [--rounds 1] [--apply file.json]
Output: <out>/lenc_cam<N>_{before,after}.json (cell stats), lenc_regs_cam<N>.json (register
set), lenc_cam<N>.jsonl (every write / capture).
"""
import argparse
import io
import json
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from board_repl import Repl  # noqa: E402
from shading_map import cell_stats  # noqa: E402

G_REGS = list(range(0x5800, 0x5824))
BR_REGS = list(range(0x5824, 0x583D))


def show_solid(hexrgb):
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--solid', hexrgb],
                   check=True, stdout=subprocess.DEVNULL)


class Lenc:
    def __init__(self, r, cam, out, corner=0.85):
        self.r, self.cam, self.out, self.corner = r, cam, out, corner
        self.jl = open(os.path.join(out, 'lenc_cam%d.jsonl' % cam), 'a')

    def log(self, **kw):
        kw['t'] = round(time.time(), 2)
        self.jl.write(json.dumps(kw) + '\n')
        self.jl.flush()

    def cells(self, tag=None):
        time.sleep(0.6)
        jpg = self.r.snap_cur('/s265_l.jpg', 95)
        if tag:
            open(os.path.join(self.out, 'lenc_cam%d_%s.jpg' % (self.cam, tag)), 'wb').write(jpg)
        st = cell_stats(Image.open(io.BytesIO(jpg)), 6, 6)
        if st[..., 1].max() < 60 or st[..., 1].max() > 250:
            raise RuntimeError('field not usable (G max %.0f): pattern hidden or clipping' % st[..., 1].max())
        return st

    def err(self, st):
        """Error vector over the 36 cells: luminance vs target, R/G and B/G vs centre."""
        g = st[..., 1]
        rg, bg = st[..., 0] / g, st[..., 2] / g
        c = (slice(2, 4), slice(2, 4))
        gc, rgc, bgc = g[c].mean(), rg[c].mean(), bg[c].mean()
        # luminance target: 1.0 at the centre falling to `corner` at the corners (radial)
        yy, xx = np.mgrid[0:6, 0:6]
        rad = np.sqrt(((xx - 2.5) / 2.5) ** 2 + ((yy - 2.5) / 2.5) ** 2) / np.sqrt(2)
        lum_t = 1 - (1 - self.corner) * rad
        e_lum = g / gc - lum_t
        e_rg = rg - rgc
        e_bg = bg - bgc
        return np.concatenate([e_lum.ravel(), 2 * e_rg.ravel(), 2 * e_bg.ravel()]), (gc, rgc, bgc)

    def read_all(self):
        return self.r.regs(self.cam, G_REGS), self.r.regs(self.cam, BR_REGS)

    def write(self, reg, val):
        rc = self.r.regw(self.cam, reg, int(val))
        self.log(ev='write', reg='0x%04X' % reg, val=int(val), rc=rc)

    def params(self, g, br):
        """Parameter vector: 36 G values, 25 B nibbles, 25 R nibbles."""
        return np.array(list(g) + [v >> 4 for v in br] + [v & 0x0F for v in br], float)

    def apply(self, p):
        p = np.round(p).astype(int)
        g = np.clip(p[:36], 0, 63)
        b = np.clip(p[36:61], 0, 15)
        rr = np.clip(p[61:86], 0, 15)
        for reg, v in zip(G_REGS, g):
            self.write(reg, v)
        for reg, bv, rv in zip(BR_REGS, b, rr):
            self.write(reg, (int(bv) << 4) | int(rv))
        return {'0x%04X' % reg: int(v) for reg, v in zip(G_REGS, g)} | \
               {'0x%04X' % reg: (int(bv) << 4) | int(rv) for reg, bv, rv in zip(BR_REGS, b, rr)}

    def jacobian(self, p0, e0, steps=(8, 3, 3)):
        J = np.zeros((len(e0), 86))
        g0, br0 = list(map(int, p0[:36])), None
        for k in range(86):
            if k < 36:
                reg, step, hi = G_REGS[k], steps[0], 63
                old = int(p0[k]); new = old + step if old + step <= hi else old - step
                self.write(reg, new)
            elif k < 61:
                reg, step = BR_REGS[k - 36], steps[1]
                bo, ro = int(p0[k]), int(p0[k + 25])
                new = bo + step if bo + step <= 15 else bo - step
                self.write(reg, (new << 4) | ro)
                old = bo
            else:
                reg, step = BR_REGS[k - 61], steps[2]
                bo, ro = int(p0[k - 25]), int(p0[k])
                new = ro + step if ro + step <= 15 else ro - step
                self.write(reg, (bo << 4) | new)
                old = ro
            ek, _ = self.err(self.cells())
            # restore
            if k < 36:
                self.write(reg, old)
            elif k < 61:
                self.write(reg, (old << 4) | int(p0[k + 25]))
            else:
                self.write(reg, (int(p0[k - 25]) << 4) | old)
            J[:, k] = (ek - e0) / (new - old)
            if k % 10 == 9:
                print('    jacobian %d/86' % (k + 1), flush=True)
        return J

    def run(self, rounds=1, damp=0.8, lam_rel=0.02):
        g, br = self.read_all()
        p = self.params(g, br)
        st = self.cells('before')
        json.dump(st.round(1).tolist(), open(os.path.join(self.out, 'lenc_cam%d_before.json' % self.cam), 'w'))
        e0, (gc, rgc, bgc) = self.err(st)
        self.log(ev='start', g=g, br=br, err_rms=float(np.sqrt(np.mean(e0 ** 2))))
        print('  cam%d LENC start: G %.0f..%.0f (falloff %.0f %%), R/G %.3f..%.3f, B/G %.3f..%.3f, err rms %.3f' % (
            self.cam, st[..., 1].min(), st[..., 1].max(), 100 * (1 - st[..., 1].min() / st[..., 1].max()),
            (st[..., 0] / st[..., 1]).min(), (st[..., 0] / st[..., 1]).max(),
            (st[..., 2] / st[..., 1]).min(), (st[..., 2] / st[..., 1]).max(), np.sqrt(np.mean(e0 ** 2))))
        regs = None
        for rd in range(rounds):
            J = self.jacobian(p, e0)
            lam = lam_rel * np.trace(J.T @ J) / 86
            # smoothness prior over neighbouring G cells (6x6) and BR cells (5x5)
            def lap(n):
                D = []
                for i in range(n):
                    for j in range(n):
                        if j + 1 < n:
                            row = np.zeros(n * n); row[i * n + j] = 1; row[i * n + j + 1] = -1; D.append(row)
                        if i + 1 < n:
                            row = np.zeros(n * n); row[i * n + j] = 1; row[(i + 1) * n + j] = -1; D.append(row)
                return np.array(D)
            S = np.zeros((0, 86))
            Dg, Db = lap(6), lap(5)
            S = np.vstack([np.hstack([Dg, np.zeros((len(Dg), 50))]),
                           np.hstack([np.zeros((len(Db), 36)), Db, np.zeros((len(Db), 25))]),
                           np.hstack([np.zeros((len(Db), 61)), Db])])
            A = J.T @ J + lam * np.eye(86) + 0.3 * lam * S.T @ S
            dp = np.linalg.solve(A, -J.T @ e0)
            p_new = p + damp * dp
            regs = self.apply(p_new)
            p = self.params([regs['0x%04X' % r] for r in G_REGS], [regs['0x%04X' % r] for r in BR_REGS])
            st = self.cells('round%d' % rd)
            e0, _ = self.err(st)
            self.log(ev='round', rd=rd, err_rms=float(np.sqrt(np.mean(e0 ** 2))))
            print('  cam%d LENC round %d: G %.0f..%.0f (falloff %.0f %%), R/G %.3f..%.3f, B/G %.3f..%.3f, err rms %.3f' % (
                self.cam, rd, st[..., 1].min(), st[..., 1].max(), 100 * (1 - st[..., 1].min() / st[..., 1].max()),
                (st[..., 0] / st[..., 1]).min(), (st[..., 0] / st[..., 1]).max(),
                (st[..., 2] / st[..., 1]).min(), (st[..., 2] / st[..., 1]).max(), np.sqrt(np.mean(e0 ** 2))))
        st = self.cells('after')
        json.dump(st.round(1).tolist(), open(os.path.join(self.out, 'lenc_cam%d_after.json' % self.cam), 'w'))
        if regs:
            json.dump(regs, open(os.path.join(self.out, 'lenc_regs_cam%d.json' % self.cam), 'w'), indent=1)
        return regs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='1')
    ap.add_argument('--grey', type=int, default=150)
    ap.add_argument('--hex', help='field colour (e.g. ffffff with a paper diffuser)')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--corner', type=float, default=0.85)
    ap.add_argument('--apply')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    show_solid(a.hex or '%02x%02x%02x' % ((a.grey,) * 3))
    time.sleep(1.5)
    r = Repl()
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        L = Lenc(r, cam, a.out, a.corner)
        if a.apply:
            regs = json.load(open(a.apply))
            for k, v in regs.items():
                L.write(int(k, 16), v)
            st = L.cells('applied')
            print('cam%d applied: G falloff %.0f %%, R/G %.3f..%.3f, B/G %.3f..%.3f' % (
                cam, 100 * (1 - st[..., 1].min() / st[..., 1].max()),
                (st[..., 0] / st[..., 1]).min(), (st[..., 0] / st[..., 1]).max(),
                (st[..., 2] / st[..., 1]).min(), (st[..., 2] / st[..., 1]).max()))
            continue
        L.run(rounds=a.rounds)
    r.close()
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--grid', '--region', '843,229,555,653',
                    '--layout', '4x6'], check=True, stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()
