#!/usr/bin/env python3
"""EXP-s267 (OP-S10-W24-T20): REPL smoke test of every camera-frame reader that used to take
its SOURCE geometry from the compile-time VGA constants, in the three capture modes the
firmware supports (VGA 30 legacy, SXGA 15 legacy, SXGA 30 packed YUV420).

For each mode: reset the board, select the geometry / layout, init the camera, then check
native_res(), resolution() / set_resolution(), peek_col rows, save_jpeg (size, and the
expected refusal in packed mode), prep_once() (aux slots), flow.start/stop (standalone
publisher, expected refusal in packed mode), peek5_b40 / peek5_tags, yuv420_test refusal,
and the dmesg tail for the new SERR codes (0x0448 detection, 0x0E21 flow).

Usage: python3 smoke.py --out iter1 [--modes vga30,sxga15,sxga30p]
"""
import argparse
import glob
import json
import os
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's265_camera_color_calibration'))
from board_repl import Repl  # noqa: E402

MODES = {
    'vga30':   dict(hw=(640, 480),   packed=0, fps=30, exp_h=480),
    'sxga15':  dict(hw=(1280, 960),  packed=0, fps=15, exp_h=960),
    'sxga30p': dict(hw=(1280, 960),  packed=1, fps=30, exp_h=960),
}


def reset_board():
    r = Repl()
    r.ser.write(b'sentai.sys.reset()\r')
    time.sleep(0.5)
    r.close()
    t0 = time.time()
    while time.time() - t0 < 40:
        time.sleep(1)
        try:
            s = serial.Serial(sorted(glob.glob('/dev/ttyACM*'))[0], 115200, timeout=0.5)
            s.close()
            break
        except Exception:
            pass
    time.sleep(3)


def expect_raise(r, expr, text):
    """Run expr; return True when the board raised an exception containing `text`."""
    out = r.cmd(expr, 30)
    return (text in out), out.replace('\r', '').strip().split('\n')[-1][:120]


def run_mode(name, m, log):
    res = {'mode': name, 'checks': {}}
    ck = res['checks']

    def check(key, ok, detail):
        ck[key] = {'ok': bool(ok), 'detail': detail}
        print('  [%s] %-28s %s' % ('PASS' if ok else 'FAIL', key, detail))

    reset_board()
    r = Repl(log=log)
    ver = r.value('sentai.version()')
    print('== %s  (%s)' % (name, ver))
    res['version'] = ver
    w, h = m['hw']
    if (w, h) != (640, 480):
        check('set_hw', r.value('sentai.camera.set_hw(%d, %d)' % (w, h)) == '0', 'set_hw(%d,%d)' % (w, h))
    if m['packed']:
        check('packed420', r.value('sentai.camera.packed420(1)') == '1', 'packed420(1) -> 1 (returns the mode)')
    init = r.cmd('print(sentai.camera.init(1, %d))' % m['fps'], 40).replace('\r', '').strip().split('\n')[-1]
    check('init', init.strip() == '0', 'init(1, %d) -> %s' % (m['fps'], init.strip()))
    time.sleep(2.0)

    # 1. geometry reporting
    nat = r.value('sentai.camera.native_res()')
    check('native_res', nat == '(%d, %d)' % (w, h), nat)
    rs = r.value('sentai.camera.resolution()')
    check('resolution_default', rs == '(%d, %d)' % (w, h), rs)

    # 2. column reader: rows = frame height, byte 1 accepted
    byte = 1 if m['packed'] else 0
    rows = r.value('len(sentai.camera.peek_col(40, %d)[2])' % byte)
    check('peek_col_rows', rows == str(m['exp_h']), 'rows %s (byte %d)' % (rows, byte))
    # bottom half of the frame must carry live data: not all-equal bytes.  A 2-frame
    # comparison of rows >= h/2 (cache fix #1) is done by the s266 time code, not here.
    spread = r.value('(lambda c: max(c[len(c)//2:]) - min(c[len(c)//2:]))(sentai.camera.peek_col(40, %d)[2])' % byte)
    check('peek_col_bottom_live', spread.isdigit() and int(spread) > 0, 'max-min of rows >= h/2 = %s' % spread)

    # 3. JPEG capture (legacy: VGA-sized JPEG; packed: refused with -7 -> RuntimeError)
    if m['packed']:
        ok, last = expect_raise(r, 'sentai.camera.save_jpeg("/s267.jpg")', 'cam capture fail')
        check('save_jpeg_refused_packed', ok, last)
    else:
        sz = r.cmd('print(sentai.camera.save_jpeg("/s267.jpg"))', 40).replace('\r', '').strip().split('\n')[-1]
        check('save_jpeg', sz.strip().isdigit() and int(sz) > 5000, 'bytes %s' % sz.strip())

    # 4. set_resolution() is OUTPUT only
    check('set_resolution_320', r.value('sentai.camera.set_resolution(320, 240)') == '0', 'set_resolution(320,240)')
    rs2 = r.value('sentai.camera.resolution()')
    check('resolution_after', rs2 == '(320, 240)', rs2)
    nat2 = r.value('sentai.camera.native_res()')
    check('native_res_unchanged', nat2 == nat, nat2)
    if not m['packed']:
        sz2 = r.cmd('print(sentai.camera.save_jpeg("/s267_small.jpg"))', 40).replace('\r', '').strip().split('\n')[-1]
        check('save_jpeg_320', sz2.strip().isdigit() and 0 < int(sz2) < 60000, 'bytes %s' % sz2.strip())
    rows2 = r.value('len(sentai.camera.peek_col(40, %d)[2])' % byte)
    check('peek_col_rows_after_set_res', rows2 == str(m['exp_h']), 'rows %s' % rows2)
    check('set_resolution_restore', r.value('sentai.camera.set_resolution(%d, %d)' % (w, h)) == '0', 'back to sensor size')
    check('set_resolution_too_big', r.value('sentai.camera.set_resolution(%d, %d)' % (w * 2, h)) == '-1', 'w*2 refused')

    # 5. aux prep slots (aruco / places / flow grey) from the raw frame: enable the
    # GRAY_NATIVE (0) and RGB_64 (1) slots so the producers fire; legacy modes must
    # advance the slot sequence, packed mode must skip the producers (SERR 0x0448)
    r.value('sentai.camera.prep_enable(0)'); r.value('sentai.camera.prep_enable(1)')
    seq0 = r.value('sentai.camera.prep_stats()["slot_seq"][0]')
    p = r.value('sentai.camera.prep_once()')
    seq1 = r.value('sentai.camera.prep_stats()["slot_seq"][0]')
    check('prep_once', p == '0', 'prep_once -> %s' % p)
    if m['packed']:
        check('prep_slots_skipped_packed', seq0 == seq1, 'GRAY_NATIVE seq %s -> %s' % (seq0, seq1))
    else:
        check('prep_slots_advance', seq0 != seq1, 'GRAY_NATIVE seq %s -> %s' % (seq0, seq1))
    r.value('sentai.camera.prep_disable(0)'); r.value('sentai.camera.prep_disable(1)')

    # 6. standalone flow publisher: legacy runs, packed refused (-4 -> RuntimeError)
    if m['packed']:
        ok, last = expect_raise(r, 'sentai.flow.start(0)', 'flow.start failed (-4)')
        check('flow_start_refused_packed', ok, last)
    else:
        fs = r.value('sentai.flow.start(0)')
        check('flow_start', fs == '0', 'flow.start(0) -> %s' % fs)
        time.sleep(1.5)
        ps = r.value('sentai.flow.pub_stats()')
        rd = r.value('sentai.flow.read()')
        check('flow_publishes', 'frames' in ps and "'frames': 0" not in ps, ps[:100])
        res['flow_read'] = rd
        r.value('sentai.flow.stop()')

    # 7. diag oracles and the VGA-only YUV test
    p5 = r.value('sentai.camera.peek5_b40()')
    check('peek5_b40', p5.startswith('(') or p5.lstrip('-').isdigit(), p5[:80])
    p5t = r.value('sentai.camera.peek5_tags()')
    check('peek5_tags', p5t.startswith('('), p5t[:80])
    yt = r.value('sentai.camera.yuv420_test("/s267_y.bin", "/s267_uv.bin")')
    if (w, h) == (640, 480) and not m['packed']:
        # VGA legacy XRGB (yuv_mode off): the PXP split still runs on the XRGB bytes
        check('yuv420_test_vga', yt in ('0', '-2'), 'yuv420_test -> %s' % yt)
    else:
        check('yuv420_test_refused', yt in ('-5', '-7'), 'yuv420_test -> %s' % yt)

    # 8. dmesg tail: the new codes must appear only where expected
    dm = r.cmd('print(sentai.rtos.dmesg(4000))', 20)   # whole text, not the last line
    res['dmesg_tail'] = dm[-1500:]
    has_det = '0448' in dm
    has_flow = '0E21' in dm or '0e21' in dm
    if m['packed']:
        check('serr_det_cam_format_logged', has_det, 'SERR 0x0448 (prep producers skipped) %s' % ('present' if has_det else 'absent'))
        check('serr_flow_cam_format_logged', has_flow, 'SERR 0x0E21 %s' % ('present' if has_flow else 'absent'))
    else:
        check('no_cam_format_serr', not has_det and not has_flow, 'no 0x0448 / 0x0E21')
    r.close()
    res['pass'] = all(c['ok'] for c in ck.values())
    print('== %s: %s (%d/%d)' % (name, 'PASS' if res['pass'] else 'FAIL',
                                 sum(c['ok'] for c in ck.values()), len(ck)))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--modes', default='vga30,sxga15,sxga30p')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, 'repl.log'), 'a')
    results = [run_mode(n, MODES[n], log) for n in a.modes.split(',')]
    json.dump(results, open(os.path.join(a.out, 'verdict.json'), 'w'), indent=1)
    print('\nOVERALL:', 'PASS' if all(x['pass'] for x in results) else 'FAIL')
    # leave the board at VGA30 legacy, cameras initialised
    if a.modes.split(',')[-1] != 'vga30':
        reset_board()


if __name__ == '__main__':
    main()
