#!/usr/bin/env python3
"""EXP-s257 / OP-S10-W24-T3 -- model-switch stress on the Edge TPU (hypothesis:
"switching models is buggy", raised 2026-10-31 after the pass-1 silence hit at
the 3rd/4th `tpu.load` of a boot).

One boot, camera in native YUV at --fps, then --switches rounds of:

  1. `tpu.load_slot(0, <next model>)`  (= `tpu.load` + slot-0 mirror; cycling the six YUV420 models; with --same the
     same model is reloaded every round)
  2. determinism probe: two `tpu.invoke()` on the arena content as-is ->
     `tpu.output_hash(0)` must be identical (same model, same input bytes)
  3. `pipeline.start` for --seconds, then `pipeline.infer_stats()`
  4. `pipeline.stop`
  5. `rtos.heap_info()` and `tpu.urb_sync_stats()` after every round

Also, before the loop, --cold-loads back-to-back loads WITHOUT inference
(the sequence that leaked packages before build 1695).

Pass criteria: every load rc 0, 0 invoke failures, 0 URB timeouts, both
probe hashes equal every round, FreeRTOS free heap not trending down by more
than --heap-tol bytes over the run, REPL alive at the end.

Usage: python3 model_switch_stress.py [--switches 36] [--seconds 3] [--fps 45]
                                      [--cold-loads 12] [--same] [--tag x]
"""
import argparse, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

MODELS = ['msblock', 'c3', 'c2f', 'c2f_pan2', 'gelan', 'c2f_thick']


def kv(s, pre=''):
    d = {}
    m = re.search(r"\{.*\}", s, re.S)
    if m:
        for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(0)):
            d[pre + k] = int(v)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--switches', type=int, default=36)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--fps', type=int, default=45)
    ap.add_argument('--cold-loads', type=int, default=12)
    ap.add_argument('--same', action='store_true')
    ap.add_argument('--heap-tol', type=int, default=64 * 1024)
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'switch_%s.log' % a.tag), 'w')

    def q(line, t=20):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush(); return out

    print(q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    q('print("YUVMODE", sentai.camera.yuv_mode(1))', 5)
    out = q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 60)
    print([l.strip()[:80] for l in out.split('\n') if l.strip().startswith('INIT')])
    heap0 = kv(q('print(sentai.rtos.heap_info())', 8))
    print('heap at start:', heap0)
    bad = []

    # --- phase A: back-to-back loads, no inference -----------------------
    t_loads = []
    for i in range(a.cold_loads):
        name = MODELS[0 if a.same else i % len(MODELS)]
        t0 = time.time()
        out = q('print("LOAD", sentai.tpu.load("/models/%s_yuv420_int8_edgetpu.tflite"))' % name, 90)
        t_loads.append(round(time.time() - t0, 2))
        if 'LOAD 0' not in out:
            bad.append('cold load %d (%s) rc: %s' % (i, name, out.strip()[-80:])); break
    heapA = kv(q('print(sentai.rtos.heap_info())', 8))
    print('phase A: %d cold loads, load s = %s, heap = %s' % (len(t_loads), t_loads, heapA))

    # --- phase B: switch + probe + pipeline -----------------------------
    print('%4s %-10s %6s %10s %10s %6s %6s %6s %8s %9s' %
          ('rnd', 'model', 'load_s', 'hash1', 'hash2', 'frames', 'fail', 'ms', 'heapfree', 'urb_to'))
    heap_free = []
    for r in range(a.switches):
        name = MODELS[0 if a.same else (r + 1) % len(MODELS)]
        t0 = time.time()
        # load_slot(0, ...) == tpu.load() + the slot-0 mirror that output_hash(0) needs.
        out = q('print("LOAD", sentai.tpu.load_slot(0, "/models/%s_yuv420_int8_edgetpu.tflite"))' % name, 90)
        dt = round(time.time() - t0, 2)
        if 'LOAD 0' not in out:
            bad.append('round %d load %s: %s' % (r, name, out.strip()[-80:])); print('LOAD FAILED', name); break
        p1 = q('print("P1", sentai.tpu.invoke(), hex(sentai.tpu.output_hash(0)))', 20)
        p2 = q('print("P2", sentai.tpu.invoke(), hex(sentai.tpu.output_hash(0)))', 20)
        h1 = re.search(r'P1 (-?\d+) (0x[0-9a-f]+)', p1); h2 = re.search(r'P2 (-?\d+) (0x[0-9a-f]+)', p2)
        if not h1 or not h2:
            bad.append('round %d probe no answer: %r %r' % (r, p1[-60:], p2[-60:])); print('PROBE SILENT', name); break
        if h1.group(2) != h2.group(2) or int(h1.group(1)) < 0 or int(h2.group(1)) < 0:
            bad.append('round %d %s probe mismatch/err: %s %s' % (r, name, h1.groups(), h2.groups()))
        q('sentai.pipeline.infer_reset()', 5)
        q('print("START", sentai.pipeline.start(0.5))', 30)
        time.sleep(a.seconds)
        st = kv(q('print("STATS", sentai.pipeline.stats())', 10))
        ist = kv(q('print("ISTATS", sentai.pipeline.infer_stats())', 10), 'i_')
        q('print("STOP", sentai.pipeline.stop())', 30)
        hp = kv(q('print(sentai.rtos.heap_info())', 8))
        urb = kv(q('print(sentai.tpu.urb_sync_stats())', 8))
        # pipeline.stats() is a tuple on this build; infer_stats() carries ok/fail/ms_sum.
        frames = ist.get('i_ok', 0); fail = ist.get('i_fail', 0)
        ms = round(ist.get('i_ms_sum', 0) / frames, 1) if frames else 0
        free = hp.get('free', hp.get('rtos_free', 0))
        heap_free.append(free)
        to = urb.get('take_failed', 0) + urb.get('csr_timeouts', 0)
        print('%4d %-10s %6s %10s %10s %6d %6d %6s %8d %9d' %
              (r, name, dt, h1.group(2), h2.group(2), frames, fail, ms, free, to))
        if fail or to or frames == 0:
            bad.append('round %d %s: frames=%d fail=%d urb_timeouts=%d' % (r, name, frames, fail, to))
        if ist.get('i_recover_attempts', 0):
            bad.append('round %d auto-recover fired: %s' % (r, ist))
    out = q('print(sentai.rtos.dmesg())', 10)
    print('events:', ' '.join(re.findall(r'E:0[AB4][0-9A-F]{2}:\d+', out))[:400])
    print('heap keys:', list(kv(q('print(sentai.rtos.heap_info())', 8)).keys()))
    if len(heap_free) >= 4 and heap_free[0] - heap_free[-1] > a.heap_tol:
        bad.append('heap free fell %d B over %d rounds' % (heap_free[0] - heap_free[-1], len(heap_free)))
    alive = 'OK' in q('print("OK")', 5)
    if not alive: bad.append('REPL dead at end')
    print('VERDICT:', 'PASS' if not bad else 'FAIL'); [print('  -', b) for b in bad]
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
