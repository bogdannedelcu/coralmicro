#!/usr/bin/env python3
"""EXP-s258 / OP-S10-W24-T11 -- Edge TPU speed at 1280x960 YUV420 on the board.

Phase A (camera VGA 45 fps, native YUV):
  1. replica check: c2f_yuv420_vga (random weights, built by build_model.py) vs the
     trained s257 c2f model -- isolated `tpu.invoke()` ms and pipeline fps must match
     (speed is graph-shaped, not weight-shaped).
  2. SXGA model isolated invokes with RANDOM input: `tpu.set_input_slot(0, rnd)` is
     too heavy through MicroPython for 1.8 MB, so the planes are filled on the board
     with `sentai.tpu.fill_input` if present, else left as the arena content (random
     enough -- the TPU time does not depend on pixel values).
Phase B (fresh boot, camera SXGA 1280x960 at --sxga-fps, native YUV):
  3. load the SXGA model, run the detection pipeline for --seconds, report fps,
     invoke ms, PXP ms, frames dropped, URB counters, TPU/EHCI errors.
  4. same with the stride-2 space-to-depth variant.

Usage: python3 run_sxga.py [--seconds 15] [--n 30] [--sxga-fps 15] [--tag x] [--skip-a]
"""
import argparse, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402
RESET = '/tmp/claude-1000/-home-bogdan-work-coralmicro/0213fcbf-a574-4225-a0ea-7da5f47ce784/scratchpad/board_reset.py'


def kv(s, pre=''):
    d = {}
    m = re.search(r"\{.*\}", s, re.S)
    if m:
        for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(0)):
            d[pre + k] = int(v)
    return d


class Board:
    def __init__(self, log):
        # the port name can change (ttyACM0 -> ttyACM1) if a handle was still open
        # across a reset, so re-resolve it and always close before resetting.
        import glob
        B.PORT = (sorted(glob.glob('/dev/ttyACM*')) or [B.PORT])[0]
        self.ser = B.open_repl(); B.cmd(self.ser, 'import sentai', 5); self.log = log

    def close(self):
        try: self.ser.close()
        except Exception: pass

    def q(self, line, t=20):
        out = B.cmd(self.ser, line, t)
        self.log.write('>>> %s\n%s\n' % (line, out)); self.log.flush()
        return out

    def last(self, out, key):
        for l in out.split('\n'):
            if l.strip().startswith(key): return l.strip()
        return ''


def isolated(bd, n):
    bd.q('sentai.tpu.invoke()', 30)   # parameter upload on the first invoke after a load
    out = bd.q('print("ISO", [sentai.tpu.invoke() for _ in range(%d)])' % n, 240)
    m = re.search(r'ISO \[(.*)\]', out)
    v = [int(x) for x in m.group(1).split(',')] if m else []
    ok = [x for x in v if x >= 0]
    return (min(ok), sum(ok) / len(ok), max(ok), len(v) - len(ok)) if ok else (-1, -1, -1, len(v))


def pipeline(bd, seconds):
    bd.q('sentai.pipeline.infer_reset()', 5)
    bd.q('print("START", sentai.pipeline.start(0.5))', 30)
    t0 = time.time(); time.sleep(seconds)
    st = bd.q('print("STATS", sentai.pipeline.stats())', 10)
    ist = kv(bd.q('print("ISTATS", sentai.pipeline.infer_stats())', 10), 'i_')
    wall = time.time() - t0
    bd.q('print("STOP", sentai.pipeline.stop())', 30)
    pst = bd.q('print("PSTATS", sentai.pipeline.prep_stats())', 10) if 'prep_stats' in bd.q('print(dir(sentai.pipeline))', 5) else ''
    frames = ist.get('i_ok', 0); fail = ist.get('i_fail', 0)
    ms = ist.get('i_ms_sum', 0) / frames if frames else 0
    return dict(fps=round(frames / wall, 1), invoke_ms=round(ms, 2), frames=frames, fail=fail,
                cycle_ms=round(ist.get('i_cycle_ms_sum', 0) / frames, 1) if frames else 0,
                wait_ms=round(ist.get('i_infer_wait_ms_sum', 0) / frames, 1) if frames else 0,
                first_fail=ist.get('i_first_fail_idx', -1), stats=bd.last(st, 'STATS')[:80],
                pstats=bd.last(pst, 'PSTATS')[:160])


def urb(bd):
    d = kv(bd.q('print(sentai.tpu.urb_sync_stats())', 8))
    return {k: d.get(k, 0) for k in ('take_failed', 'csr_timeouts', 'desync_count', 'usb_detach')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--sxga-fps', type=int, default=15)
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--skip-a', action='store_true')
    a = ap.parse_args()
    log = open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w')
    rows = []

    def report(phase, model, iso, pipe, u):
        row = (phase, model, iso, pipe, u); rows.append(row)
        print('%-7s %-32s iso min/avg/max/fail = %s | pipe %s | urb %s' %
              (phase, model, iso if iso else '-', pipe if pipe else '-', u))

    if not a.skip_a:
        subprocess.run(['python3', RESET], check=False, capture_output=True)
        bd = Board(log)
        print(bd.q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
        bd.q('print("YUV", sentai.camera.yuv_mode(1))', 5)
        print(bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 60), 'INIT'))
        for name, path in (('c2f (trained, s257)', '/models/c2f_yuv420_int8_edgetpu.tflite'),
                           ('c2f_vga_relu replica', '/models/c2f_yuv420_vga_relu_int8_edgetpu.tflite')):
            out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % path, 90)
            if 'LOAD 0' not in out: print('LOAD FAILED', name); continue
            report('A-vga', name, isolated(bd, a.n), pipeline(bd, a.seconds), urb(bd))
        for name, path in (('c2f_sxga_relu (stride-2 stem)', '/models/c2f_yuv420_sxga_relu_int8_edgetpu.tflite'),
                           ('c2f_sxga_s2d_relu', '/models/c2f_yuv420_sxga_s2d_relu_int8_edgetpu.tflite')):
            out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % path, 90)
            if 'LOAD 0' not in out: print('LOAD FAILED', name); continue
            print('   ', [l.strip()[:70] for l in out.split('\n') if l.strip().startswith('in[')])
            report('A-iso', name, isolated(bd, a.n), None, urb(bd))
        bd.close()

    # ---- Phase B: camera at SXGA ----
    subprocess.run(['python3', RESET], check=False, capture_output=True)
    bd = Board(log)
    print(bd.q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    bd.q('print("YUV", sentai.camera.yuv_mode(1))', 5)
    print('set_hw:', bd.last(bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10), 'HW'))
    out = bd.q('print("INIT", sentai.camera.init(1, %d))' % a.sxga_fps, 90)
    print('SXGA', bd.last(out, 'INIT'), [l.strip()[:80] for l in out.split('\n') if '[cam]' in l][:3])
    if 'INIT 0' not in out:
        print('camera SXGA init failed -> stop'); return 1
    time.sleep(1.0)
    print('cam fps:', bd.last(bd.q('print("FPS", sentai.camera.fps())', 10), 'FPS') if 'fps' in bd.q('print(dir(sentai.camera))', 5) else '?')
    print('alive:', 'OK' in bd.q('print("OK")', 5))
    for name, path in (('c2f_sxga_relu (stride-2 stem)', '/models/c2f_yuv420_sxga_relu_int8_edgetpu.tflite'),
                       ('c2f_sxga_s2d_relu', '/models/c2f_yuv420_sxga_s2d_relu_int8_edgetpu.tflite')):
        out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % path, 90)
        if 'LOAD 0' not in out: print('LOAD FAILED', name, out[-200:]); continue
        report('B-sxga', name, None, pipeline(bd, a.seconds), urb(bd))
        print('   health:', bd.q('print(sentai.sys.health())', 8).strip().split('\n')[-2][:120])
        if not ('OK' in bd.q('print("OK")', 5)): print('REPL dead'); break
    out = bd.q('print(sentai.rtos.dmesg())', 10)
    print('events:', ' '.join(re.findall(r'E:0[0-9A-F]{3}:\d+', out))[:400])
    bd.close()
    with open(os.path.join(HERE, 'results_%s.txt' % a.tag), 'w') as f:
        for r in rows: f.write(repr(r) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
