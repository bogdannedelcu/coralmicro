#!/usr/bin/env python3
"""Closed-loop OV5640 colour tuning against the grid shown on the monitor.

Prerequisites: the grid is on the monitor (show_pattern.py --grid --region ... --layout ...)
and <out>/locate.json exists (locate.py).  The script works on one camera at a time:

  stage expo : freeze AEC/AGC (0x3503 = 0x03), keep exposure at its current value and
               drive the AGC gain (0x350A/B, 1/16 units) until the 'neutral 8' patch
               (sRGB 200) reads TARGET_N8 on the green channel.  Green is used because
               the AWB green gain is fixed at 1.0x, so this stage is independent of AWB.
  stage awb  : freeze AWB (0x3406 = 0x01) and scale the R and B gains (0x3400-0x3405,
               12-bit, 0x400 = 1.0x) so that the mid greys ('neutral 6.5', 'neutral 5')
               read R = G = B.  Multiplicative update, damped, 6 iterations max.
  stage tone : switches the monitor to the 24-step grey ramp (show_pattern.py --ramp) and
               runs Gauss-Newton on the 15 gamma-curve bytes (0x5481-0x548F, Y output at
               the OV5640 knot inputs 8..254): Jacobian by +STEP perturbation of each byte,
               least-squares update on (measured luma - target) over the 24 greys, damped,
               clipped to 0..255 and forced monotonic.  2 rounds.  Restores the grid after.
  stage cmx  : Gauss-Newton on the 9 colour-matrix magnitudes (0x5381-0x5389; sign bits
               0x538A/0x538B are left as found).  A Jacobian is measured by perturbing
               each register by +STEP and capturing; the least-squares update minimising
               the RGB error over all 24 patches (greys weighted x2) is applied, clipped
               to [0, 255].  2 rounds by default.

Every capture, register write and measurement is appended to <out>/tune_cam<N>.jsonl so the
run can be audited; the final register list goes to <out>/tuned_regs_cam<N>.json and can be
replayed with --apply on any camera of the same model.

Usage:
  python3 tune_colors.py --out iter1 --cams 0,1 --stages expo,awb
  python3 tune_colors.py --out iter1 --cams 0 --apply iter1/tuned_regs_cam0.json
"""
import argparse
import io
import json
import os
import sys
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from board_repl import Repl  # noqa: E402
from analyze import measure, de76  # noqa: E402

TARGET_N8 = 200.0           # sRGB value of the 'neutral 8' patch
MIN_WHITE = 150.0           # visibility guard: monitor 150; a dim printed chart needs ~50 (--min-white)
GREYS = ('neutral 6.5', 'neutral 5')
ALL_GREYS = ('white', 'neutral 8', 'neutral 6.5', 'neutral 5', 'neutral 3.5', 'black')
CMX_REGS = list(range(0x5381, 0x538A))
GAMMA_REGS = list(range(0x5481, 0x5490))
# NXP SDK init-table values (fsl_ov5640.c) for the blocks the tuner touches: `reset` stage
SDK_DEFAULTS = {
    0x5381: 0x1E, 0x5382: 0x5B, 0x5383: 0x08, 0x5384: 0x0A, 0x5385: 0x7E, 0x5386: 0x88,
    0x5387: 0x7C, 0x5388: 0x6C, 0x5389: 0x10, 0x538A: 0x01, 0x538B: 0x98,
    0x5481: 0x08, 0x5482: 0x14, 0x5483: 0x28, 0x5484: 0x51, 0x5485: 0x65, 0x5486: 0x71,
    0x5487: 0x7D, 0x5488: 0x87, 0x5489: 0x91, 0x548A: 0x9A, 0x548B: 0xAA, 0x548C: 0xB8,
    0x548D: 0xCD, 0x548E: 0xDD, 0x548F: 0xEA, 0x5490: 0x1D,
    0x5580: 0x02, 0x5583: 0x40, 0x5584: 0x10, 0x5589: 0x10, 0x558A: 0x00, 0x558B: 0xF8,
}


def show(kind, region, layout):
    """Put the grid or the grey ramp on the monitor; returns the patch list shown."""
    import subprocess
    cmd = [sys.executable, os.path.join(HERE, 'show_pattern.py'), '--' + kind, '--layout', layout]
    if region:
        cmd += ['--region', region]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    time.sleep(1.5)
    return json.load(open(os.path.join(HERE, 'patterns', 'pattern_%s.json' % kind)))


class Tuner:
    def __init__(self, r, cam, Hm, patches, out, shrink=0.3, region=None, layout='4x6'):
        self.r, self.cam, self.H, self.patches, self.shrink = r, cam, Hm, patches, shrink
        self.region, self.layout = region, layout
        self.out = out
        self.jl = open(os.path.join(out, 'tune_cam%d.jsonl' % cam), 'a')
        # capture counter continues across runs so file names stay chronological
        import glob
        self.n = len(glob.glob(os.path.join(out, 'tune_cam%d_*.jpg' % cam)))

    def log(self, **kw):
        kw['t'] = round(time.time(), 2)
        self.jl.write(json.dumps(kw) + '\n')
        self.jl.flush()

    def write(self, reg, val):
        rc = self.r.regw(self.cam, reg, val)
        self.log(ev='write', reg='0x%04X' % reg, val='0x%02X' % val, rc=rc)
        if rc != 0:
            raise RuntimeError('reg_write(0x%04X) -> %d' % (reg, rc))

    def read(self, reg):
        return self.r.reg(self.cam, reg)

    # --- centre mode: one patch at a time at the camera's frame centre --------------
    # Removes field position (lens / colour shading, LCD viewing angle) from the colour
    # measurement: every patch is shown as a `size` px square at the screen point the
    # camera's optical centre looks at, on the mid-grey background, and measured alone.
    def enable_centre(self, loc_cam, size=220, bg=(118, 118, 118), settle=0.7):
        Hinv = np.asarray(loc_cam['H_cam_to_mon'])
        cw, ch = loc_cam['cam_size']
        p = np.array([cw / 2, ch / 2, 1.0]) @ Hinv.T
        cx, cy = p[0] / p[2], p[1] / p[2]
        self.centre_rect = [int(cx - size / 2), int(cy - size / 2), size, size]
        self.centre_dir = os.path.join(HERE, 'patterns', 'centre_cam%d' % self.cam)
        os.makedirs(self.centre_dir, exist_ok=True)
        self.centre_settle = settle
        for p in self.patches:
            img = Image.new('RGB', (1920, 1080), bg)
            img.paste(Image.new('RGB', (size, size), tuple(p['srgb'])), (self.centre_rect[0], self.centre_rect[1]))
            img.save(os.path.join(self.centre_dir, p['name'].replace(' ', '_') + '.png'))
        self.log(ev='centre_mode', rect=self.centre_rect, size=size)

    def _capture_centre(self, tag):
        import subprocess
        d = {}
        for p in self.patches:
            png = os.path.join(self.centre_dir, p['name'].replace(' ', '_') + '.png')
            subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--png', png],
                           check=True, stdout=subprocess.DEVNULL)
            time.sleep(self.centre_settle)
            jpg = self.r.snap_cur('/s265_tune.jpg', 90)
            one = [{'name': p['name'], 'srgb': p['srgb'], 'rect': self.centre_rect}]
            m = measure(Image.open(io.BytesIO(jpg)), self.H, one, 0.4)[0]
            if m:
                d[p['name']] = m
                # visibility guard in centre mode: a bright patch must read bright
                if p['srgb'][0] >= 230 and p['srgb'][1] >= 230 and np.mean(m['mean']) < MIN_WHITE:
                    raise RuntimeError('pattern not visible (patch %s reads %.0f): a dialog or the desktop '
                                       'is covering the display' % (p['name'], np.mean(m['mean'])))
        self.n += 1
        json.dump(d, open(os.path.join(self.out, 'tune_cam%d_%03d_%s_centre.json' % (self.cam, self.n, tag)), 'w'))
        return d

    def capture(self, tag):
        time.sleep(0.5)                     # let the sensor settle after register writes
        if getattr(self, 'centre_rect', None):
            d = self._capture_centre(tag)
            self.log(ev='capture', tag=tag, centre=True,
                     dE_mean=round(float(np.mean([m['dE76'] for m in d.values()])), 2),
                     greys={k: d[k]['mean'] for k in ALL_GREYS if k in d})
            return d
        jpg = self.r.snap_cur('/s265_tune.jpg', 90)
        self.n += 1
        p = os.path.join(self.out, 'tune_cam%d_%03d_%s.jpg' % (self.cam, self.n, tag))
        open(p, 'wb').write(jpg)
        meas = measure(Image.open(io.BytesIO(jpg)), self.H, self.patches, self.shrink)
        d = {m['name']: m for m in meas if m}
        # pattern-visible guard (s265 run 1 was ruined by a GNOME dialog over the grid):
        # the white patch must be bright and brighter than the black one by a wide margin
        if 'white' in d and 'black' in d:
            wv, bv = np.mean(d['white']['mean']), np.mean(d['black']['mean'])
            if wv < MIN_WHITE or wv - bv < MIN_WHITE * 0.5:
                self.log(ev='pattern_not_visible', tag=tag, white=wv, black=bv, file=os.path.basename(p))
                raise RuntimeError('pattern not visible on the monitor (white %.0f, black %.0f) at %s — '
                                   'a dialog or the desktop is covering it; see %s' % (wv, bv, tag, p))
        self.log(ev='capture', tag=tag, file=os.path.basename(p),
                 dE_mean=round(float(np.mean([m['dE76'] for m in d.values()])), 2),
                 greys={k: d[k]['mean'] for k in ALL_GREYS if k in d})
        return d

    # --- stages ---------------------------------------------------------
    def stage_reset(self):
        """Back to the SDK defaults for CMX, gamma and SDE; AWB/AEC back to auto."""
        for reg, val in SDK_DEFAULTS.items():
            self.write(reg, val)
        self.write(0x3406, 0x00)
        self.write(0x3503, 0x00)
        time.sleep(2.0)                     # let AEC/AWB re-converge
        return {}

    def stage_expo(self, iters=6, tol=3.0):
        self.write(0x3503, 0x03)            # manual AEC + manual AGC, keep current values
        gain = (self.read(0x350A) << 8) | self.read(0x350B)
        exp = (self.read(0x3500) << 16) | (self.read(0x3501) << 8) | self.read(0x3502)
        self.log(ev='expo_start', agc=gain, exposure=exp)
        for i in range(iters):
            d = self.capture('expo%d' % i)
            g = d['neutral 8']['mean'][1]
            print('  cam%d expo it%d: AGC %d (%.2fx) neutral8 G=%.1f target %.0f'
                  % (self.cam, i, gain, gain / 16.0, g, TARGET_N8))
            if abs(g - TARGET_N8) <= tol:
                break
            # sensor is ~linear before gamma; gamma ~2.2 makes output ∝ gain^(1/2.2)
            ratio = (TARGET_N8 / max(g, 1.0)) ** 2.2
            ratio = min(max(ratio, 0.5), 2.0)
            gain = int(round(min(max(gain * ratio, 0x10), 0x3FF)))
            self.write(0x350A, gain >> 8)
            self.write(0x350B, gain & 0xFF)
        return {'0x3503': 0x03, '0x350A': gain >> 8, '0x350B': gain & 0xFF}

    @staticmethod
    def _secant_exp(prev, gain, out, default):
        """Exponent e in out ∝ gain^e, estimated from the previous (gain, out) pair."""
        if prev is None or prev[0] == gain or prev[1] <= 1 or out <= 1:
            return default
        e = np.log(out / prev[1]) / np.log(gain / prev[0])
        return float(min(max(e, 0.3), 2.0))

    def stage_awb(self, iters=8, tol=2.0, damp=0.7):
        rg = (self.read(0x3400) << 8) | self.read(0x3401)
        bg = (self.read(0x3404) << 8) | self.read(0x3405)
        self.write(0x3406, 0x01)            # manual AWB, freeze the current gains
        self.log(ev='awb_start', r=rg, b=bg)
        prev_r = prev_b = None
        for i in range(iters):
            d = self.capture('awb%d' % i)
            m = np.mean([d[k]['mean'] for k in GREYS], axis=0)
            print('  cam%d awb it%d: R gain %d B gain %d  mid-grey RGB %.1f %.1f %.1f'
                  % (self.cam, i, rg, bg, *m))
            if abs(m[0] - m[1]) <= tol and abs(m[2] - m[1]) <= tol:
                break
            # measured response of the output to the AWB gain is close to linear (the
            # first run with a gamma-2.2 assumption oscillated); estimate the exponent
            # from the previous step (secant in log domain), damp the update
            er = self._secant_exp(prev_r, rg, m[0], 1.0)
            eb = self._secant_exp(prev_b, bg, m[2], 1.0)
            prev_r, prev_b = (rg, m[0]), (bg, m[2])
            kr = (m[1] / max(m[0], 1.0)) ** (1.0 / er)
            kb = (m[1] / max(m[2], 1.0)) ** (1.0 / eb)
            rg = int(round(min(max(rg * (1 + damp * (kr - 1)), 0x100), 0xFFF)))
            bg = int(round(min(max(bg * (1 + damp * (kb - 1)), 0x100), 0xFFF)))
            self.log(ev='awb_step', exp_r=round(er, 2), exp_b=round(eb, 2), r=rg, b=bg)
            self.write(0x3400, rg >> 8); self.write(0x3401, rg & 0xFF)
            self.write(0x3404, bg >> 8); self.write(0x3405, bg & 0xFF)
        return {'0x3406': 0x01, '0x3400': rg >> 8, '0x3401': rg & 0xFF,
                '0x3402': 0x04, '0x3403': 0x00, '0x3404': bg >> 8, '0x3405': bg & 0xFF}

    def stage_tone(self, rounds=2, step=12, damp=0.7):
        grid_patches = self.patches
        self.patches = show('ramp', self.region, self.layout)
        names = [p['name'] for p in self.patches]
        target = np.array([p['srgb'][0] for p in self.patches], float)

        def luma(d):
            return np.array([np.mean(d[n]['mean']) if n in d else np.nan for n in names])

        cur = [self.read(r) for r in GAMMA_REGS]
        self.log(ev='tone_start', gamma=cur)
        try:
            for rd in range(rounds):
                d0 = self.capture('tone%d_base' % rd)
                y0 = luma(d0)
                ok = ~np.isnan(y0)
                e0 = y0[ok] - target[ok]
                print('  cam%d tone round %d: luma rms %.1f  gamma %s' % (
                    self.cam, rd, np.sqrt(np.mean(e0 ** 2)), ['%02X' % v for v in cur]))
                J = np.zeros((ok.sum(), len(GAMMA_REGS)))
                for k, reg in enumerate(GAMMA_REGS):
                    s = step if cur[k] + step <= 255 else -step
                    self.write(reg, cur[k] + s)
                    yk = luma(self.capture('tone%d_j%d' % (rd, k)))
                    self.write(reg, cur[k])
                    J[:, k] = (yk[ok] - y0[ok]) / s
                lam = 0.05 * np.trace(J.T @ J) / len(GAMMA_REGS)
                # smoothness prior on the update (second differences)
                D = np.diff(np.eye(len(GAMMA_REGS)), 2, axis=0)
                dx = np.linalg.solve(J.T @ J + lam * np.eye(len(GAMMA_REGS)) + lam * D.T @ D, -J.T @ e0)
                new = np.clip(np.array(cur) + damp * dx, 0, 255)
                new = np.maximum.accumulate(new)        # gamma curve must be monotonic
                new = [int(round(v)) for v in new]
                self.log(ev='tone_update', old=cur, new=new, dx=dx.round(2).tolist())
                for reg, v in zip(GAMMA_REGS, new):
                    self.write(reg, v)
                cur = new
            d = self.capture('tone_final')
            y = luma(d)
            ok = ~np.isnan(y)
            print('  cam%d tone final: luma rms %.1f' % (self.cam, np.sqrt(np.mean((y[ok] - target[ok]) ** 2))))
            print('    ramp target  ' + ' '.join('%3d' % t for t in target))
            print('    ramp measured' + ' '.join('%3.0f' % v for v in y))
        finally:
            self.patches = grid_patches
            show('grid', self.region, self.layout)
        return {'0x%04X' % r: v for r, v in zip(GAMMA_REGS, cur)}

    def stage_tone_centre(self, rounds=3, damp=0.7):
        """Tone curve with every grey shown alone at the frame centre (no field position in the
        measurement) and no Jacobian: the measured output m of a grey g tells which part of the
        curve produced it (the knots whose output is near m), and those knots are moved by
        (g - m), interpolated over the knot index, damped and kept monotonic.  One centre-mode
        ramp measurement (24 captures) per round."""
        from show_pattern import RAMP
        grid_patches = self.patches
        if not getattr(self, 'centre_rect', None):
            raise RuntimeError('tone_centre needs --centre')
        ramp = [{'name': n, 'srgb': list(rgb), 'rect': self.centre_rect} for n, rgb in RAMP]
        self.patches = ramp
        for p in ramp:                                     # PNGs for the ramp greys
            png = os.path.join(self.centre_dir, p['name'].replace(' ', '_') + '.png')
            if not os.path.exists(png):
                img = Image.new('RGB', (1920, 1080), (118, 118, 118))
                img.paste(Image.new('RGB', (self.centre_rect[2], self.centre_rect[3]), tuple(p['srgb'])),
                          (self.centre_rect[0], self.centre_rect[1]))
                img.save(png)
        cur = np.array([self.read(r) for r in GAMMA_REGS], float)
        self.log(ev='tone_centre_start', gamma=cur.tolist())
        try:
            for rd in range(rounds):
                d = self.capture('tonec%d' % rd)
                g = np.array([p['srgb'][0] for p in ramp if p['name'] in d], float)
                m = np.array([np.mean(d[p['name']]['mean']) for p in ramp if p['name'] in d])
                err = g - m
                print('  cam%d tone_centre round %d: luma rms %.1f  gamma %s' % (
                    self.cam, rd, np.sqrt(np.mean(err ** 2)), ['%02X' % int(v) for v in cur]))
                print('    target   ' + ' '.join('%3d' % v for v in g))
                print('    measured ' + ' '.join('%3.0f' % v for v in m))
                if np.sqrt(np.mean(err ** 2)) < 3.0:
                    break
                # knot k currently outputs cur[k]; the correction it needs is err at the grey
                # whose measured output is closest to cur[k] (interpolated along m)
                order = np.argsort(m)
                corr = np.interp(cur, m[order], err[order])
                new = np.clip(cur + damp * corr, 0, 255)
                new = np.maximum.accumulate(new)
                new = np.round(new)
                self.log(ev='tone_centre_update', old=cur.tolist(), new=new.tolist())
                for reg, v in zip(GAMMA_REGS, new):
                    self.write(reg, int(v))
                cur = new
        finally:
            self.patches = grid_patches
        return {'0x%04X' % r: int(v) for r, v in zip(GAMMA_REGS, cur)}

    def _err_vec(self, d):
        """Stacked (measured - target) RGB over the patches, greys weighted x2."""
        v, w = [], []
        for p in self.patches:
            m = d.get(p['name'])
            if not m:
                continue
            v.append(np.array(m['mean']) - np.array(p['srgb'], float))
            w.append(2.0 if p['name'] in ALL_GREYS else 1.0)
        return np.concatenate(v), np.repeat(w, 3)

    def _gauss_newton(self, name, regs, rounds, step, damp, lam_rel, lo=0, hi=255):
        """Generic measured-Jacobian Gauss-Newton on a list of 8-bit registers, error =
        weighted RGB delta over all grid patches.  Returns the final register values."""
        cur = [self.read(r) for r in regs]
        self.log(ev=name + '_start', regs=['0x%04X' % r for r in regs], vals=cur)
        for rd in range(rounds):
            d0 = self.capture('%s%d_base' % (name, rd))
            e0, w = self._err_vec(d0)
            print('  cam%d %s round %d: RGB rms %.1f  %s' % (self.cam, name, rd, np.sqrt(np.mean(e0 ** 2)), cur))
            J = np.zeros((len(e0), len(regs)))
            for k, reg in enumerate(regs):
                s = step if cur[k] + step <= hi else -step
                self.write(reg, cur[k] + s)
                dk = self.capture('%s%d_j%d' % (name, rd, k))
                self.write(reg, cur[k])
                ek, _ = self._err_vec(dk)
                J[:, k] = (ek - e0) / s
            Wm = np.sqrt(w)[:, None]
            A = J * Wm
            b = -e0 * Wm[:, 0]
            lam = lam_rel * np.trace(A.T @ A) / len(regs)
            dx = np.linalg.solve(A.T @ A + lam * np.eye(len(regs)), A.T @ b)
            new = [int(round(min(max(c + damp * x, lo), hi))) for c, x in zip(cur, dx)]
            self.log(ev=name + '_update', old=cur, new=new, dx=dx.round(2).tolist())
            for reg, v in zip(regs, new):
                self.write(reg, v)
            cur = new
        d = self.capture(name + '_final')
        e, _ = self._err_vec(d)
        print('  cam%d %s final: RGB rms %.1f  %s' % (self.cam, name, np.sqrt(np.mean(e ** 2)), cur))
        return {'0x%04X' % r: v for r, v in zip(regs, cur)}

    def stage_cmx(self, rounds=3, step=20, damp=0.8, lam_rel=0.01):
        """Colour matrix magnitudes 0x5381-0x5389 (signs 0x538A/B left as found).  Run 2
        with step 16 / damp 0.7 / lam 0.05 barely moved (rms 26.0 -> 23.8 on cam1)."""
        return self._gauss_newton('cmx', CMX_REGS, rounds, step, damp, lam_rel)

    # --- analytic colour-correction matrix -------------------------------
    # The sensor outputs YUV = CMX · rgb_cam (magnitudes in 1/128, sign bits 0x538A/B),
    # the firmware converts YUV → RGB with the BT.601 inverse K⁻¹, so out = K⁻¹·CMX·rgb_cam.
    # A 3x3 correction C fitted on the measured patches (out → target, white preserving)
    # therefore maps to CMX' = K·C·K⁻¹·CMX.  One capture + one solve per round instead of
    # 9 Jacobian captures.  Fit in the encoded (output) domain: that is where the relation
    # is linear if the gamma block precedes the matrix, and it is what we can measure.
    K = np.array([[0.299, 0.587, 0.114],
                  [-0.169, -0.331, 0.500],
                  [0.500, -0.419, -0.081]])
    SIGN_BITS = {0x5381: (0x538B, 0), 0x5382: (0x538B, 1), 0x5383: (0x538B, 2), 0x5384: (0x538B, 3),
                 0x5385: (0x538B, 4), 0x5386: (0x538B, 5), 0x5387: (0x538B, 6), 0x5388: (0x538B, 7),
                 0x5389: (0x538A, 0)}

    def read_cmx(self):
        mag = np.array([self.read(r) for r in CMX_REGS], float)
        s_a, s_b = self.read(0x538A), self.read(0x538B)
        sign = np.ones(9)
        for i, reg in enumerate(CMX_REGS):
            sreg, bit = self.SIGN_BITS[reg]
            if ((s_a if sreg == 0x538A else s_b) >> bit) & 1:
                sign[i] = -1
        return (mag * sign / 128.0).reshape(3, 3), (s_a, s_b)

    def write_cmx(self, M):
        flat = (M * 128.0).ravel()
        mags = np.clip(np.abs(flat).round(), 0, 255).astype(int)
        s_a, s_b = self.read(0x538A) & ~0x01, self.read(0x538B) & 0x00
        for i, reg in enumerate(CMX_REGS):
            sreg, bit = self.SIGN_BITS[reg]
            if flat[i] < 0:
                if sreg == 0x538A:
                    s_a |= 1 << bit
                else:
                    s_b |= 1 << bit
        for reg, v in zip(CMX_REGS, mags):
            self.write(reg, int(v))
        self.write(0x538A, s_a)
        self.write(0x538B, s_b)
        return {'0x%04X' % r: int(v) for r, v in zip(CMX_REGS, mags)} | {'0x538A': s_a, '0x538B': s_b}

    def stage_ccm(self, rounds=2, damp=0.9, clip_lo=6, clip_hi=250):
        Kinv = np.linalg.inv(self.K)
        tuned = {}
        for rd in range(rounds):
            d = self.capture('ccm%d_base' % rd)
            M, signs = self.read_cmx()
            out, tgt = [], []
            for p in self.patches:
                m = d.get(p['name'])
                if not m:
                    continue
                if min(m['mean']) < clip_lo or max(m['mean']) > clip_hi:
                    continue            # clipped channel: not linear, leave it out
                out.append(m['mean'])
                tgt.append(p['srgb'])
            out, tgt = np.array(out, float), np.array(tgt, float)
            # least squares C · out ≈ tgt, then row-normalise so greys stay neutral
            C, *_ = np.linalg.lstsq(out, tgt, rcond=None)
            C = C.T
            C = C / C.sum(1, keepdims=True)
            C = np.eye(3) + damp * (C - np.eye(3))
            e_before = np.sqrt(np.mean((out - tgt) ** 2))
            e_pred = np.sqrt(np.mean((out @ C.T - tgt) ** 2))
            Mnew = self.K @ C @ Kinv @ M
            print('  cam%d ccm round %d: %d patches, RGB rms %.1f -> predicted %.1f; C=%s' % (
                self.cam, rd, len(out), e_before, e_pred, np.array2string(C, precision=3, separator=',').replace('\n', '')))
            self.log(ev='ccm_update', C=C.round(4).tolist(), cmx_old=M.round(4).tolist(), cmx_new=Mnew.round(4).tolist())
            if np.abs(Mnew * 128).max() > 255:
                print('  cam%d ccm: magnitude overflow (%.0f/128), scaling U/V rows down' % (self.cam, np.abs(Mnew * 128).max()))
                Mnew[1:] *= 255 / np.abs(Mnew[1:] * 128).max()
            tuned = self.write_cmx(Mnew)
        d = self.capture('ccm_final')
        e, _ = self._err_vec(d)
        print('  cam%d ccm final: RGB rms %.1f' % (self.cam, np.sqrt(np.mean(e ** 2))))
        return tuned

    def stage_sde(self, rounds=2, step=12, damp=0.8, lam_rel=0.01):
        """SDE saturation U (0x5583) and V (0x5584): global chroma scale per axis.  SDE
        saturation enable (0x5580 bit1) is on in the SDK init (0x5580 = 0x02)."""
        return self._gauss_newton('sde', [0x5583, 0x5584], rounds, step, damp, lam_rel, lo=0x08, hi=0xF0)


def main():
    global TARGET_N8, MIN_WHITE
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--stages', default='expo,awb')
    ap.add_argument('--apply', help='replay a tuned_regs json instead of tuning')
    ap.add_argument('--grid-json', default=os.path.join(HERE, 'patterns', 'pattern_grid.json'))
    ap.add_argument('--region', default='843,229,555,653', help='screen rect of the grid')
    ap.add_argument('--layout', default='4x6')
    ap.add_argument('--n8', type=float, default=200.0, help="exposure target for the 'neutral 8' patch (green)")
    ap.add_argument('--centre', action='store_true',
                    help='measure each patch alone at the camera frame centre (24 captures per measurement)')
    ap.add_argument('--min-white', type=float, default=150.0,
                    help='visibility guard: minimum mean of the white patch (monitor 150, dim print ~50)')
    a = ap.parse_args()
    TARGET_N8 = a.n8
    MIN_WHITE = a.min_white
    loc = json.load(open(os.path.join(a.out, 'locate.json')))
    if a.grid_json != os.path.join(HERE, 'patterns', 'pattern_grid.json'):
        patches = json.load(open(a.grid_json))      # printed chart (or any other target): no monitor
    else:
        patches = show('grid', a.region, a.layout)
    r = Repl(log=open(os.path.join(a.out, 'tune_repl.log'), 'a'))
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        t = Tuner(r, cam, loc['cams'][str(cam)]['H_mon_to_cam'], patches, a.out,
                  region=a.region, layout=a.layout)
        if a.centre:
            t.enable_centre(loc['cams'][str(cam)])
        if a.apply:
            regs = json.load(open(a.apply))
            for k, v in regs.items():
                t.write(int(k, 16), int(v))
            d = t.capture('applied')
            print('cam%d applied %d regs: mean dE76 %.1f' % (
                cam, len(regs), np.mean([m['dE76'] for m in d.values()])))
            continue
        d = t.capture('before')
        before = np.mean([m['dE76'] for m in d.values()])
        if a.stages == 'measure':
            json.dump(d, open(os.path.join(a.out, 'measure%s_patches_cam%d.json' % (
                '_centre' if a.centre else '', cam)), 'w'), indent=1)
            print('cam%d measure: mean dE76 %.1f; %s' % (cam, before, ' | '.join(
                '%s %s dE %.0f' % (k, tuple(int(round(v)) for v in m['mean']), m['dE76'])
                for k, m in sorted(d.items(), key=lambda kv: -kv[1]['dE76'])[:6])))
            continue
        tuned = {}
        for st in a.stages.split(','):
            print('cam%d stage %s' % (cam, st))
            tuned.update(getattr(t, 'stage_' + st)())
        d = t.capture('after')
        after = np.mean([m['dE76'] for m in d.values()])
        greys = np.mean([d[k]['dE76'] for k in ALL_GREYS if k in d])
        print('cam%d: mean dE76 %.1f -> %.1f (greys %.1f); worst %s %.1f' % (
            cam, before, after, greys, *max(((k, m['dE76']) for k, m in d.items()), key=lambda x: x[1])))
        tuned_path = os.path.join(a.out, 'tuned_regs_cam%d.json' % cam)
        old = json.load(open(tuned_path)) if os.path.exists(tuned_path) else {}
        old.update({k: v for k, v in tuned.items()})
        json.dump(old, open(tuned_path, 'w'), indent=1)
        json.dump(d, open(os.path.join(a.out, 'tune_after_patches_cam%d.json' % cam), 'w'), indent=1)
    r.close()


if __name__ == '__main__':
    main()
