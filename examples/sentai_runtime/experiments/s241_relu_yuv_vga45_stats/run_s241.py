#!/usr/bin/env python3
# s241 — ReLU (RGB) + YUV420 detector performance on VGA45, WITH STATISTICS.
#
# Post-fix build 1590: camera VGA45 verified 45 fps (registers + camera.fps()),
# InferTask FPS throttle fixed to GPT1 us (was tick-starved -> spurious delay).
# Supersedes s239 (build 1560, camera-regressed, camera-bound ~27 fps).
#
# Per model: isolated invoke (N invokes, GPT1 ms) -> mean/std/min/max/median,
#            pipeline FPS (M trials x DUR s, frame_count/wall) -> mean/std.
# Writes results_vga45.csv incrementally (one row per model) so a mid-run USB
# re-enum/wedge does not lose completed models; re-running skips done models.
import sys, time, re, os, statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV = os.path.join(HERE, 'results_vga45.csv')
N_INV   = 30     # isolated invokes per model (after 5 warmup)
M_TRIAL = 3      # pipeline FPS trials
DUR     = 6.0    # seconds measured per pipeline trial
WARM    = 2.0    # pipeline warmup before measuring

RGB_MODELS = ['msblock','c3','c2f','c2f_pan2','gelan','c2f_thick','gelan_pan2','c2f_deep']
YUV_MODELS = ['c2f_yuv420']

def N(x):
    m = re.findall(r'-?\d+', x)
    return int(m[-1]) if m else -999

def conn():
    for _ in range(20):
        for p in ['/dev/ttyACM0','/dev/ttyACM1']:
            try:
                B.PORT = p; s = B.open_repl()
                if 'OK' in B.cmd(s, 'import sentai; print("OK")', 3):
                    B.cmd(s, 'sentai.verbose(1)'); return s
                s.close()
            except Exception: pass
        time.sleep(1.5)
    return None

def done_models():
    if not os.path.exists(CSV): return set()
    return {ln.split(',')[0] for ln in open(CSV).read().splitlines()[1:] if ln.strip()}

def ensure_header():
    if not os.path.exists(CSV):
        with open(CSV,'w') as f:
            f.write('model,fmt,n_inputs,invoke_n,invoke_mean_ms,invoke_std_ms,'
                    'invoke_min_ms,invoke_max_ms,invoke_median_ms,'
                    'fps_trials,fps_mean,fps_std,fps_min,fps_max,per_ms_frame\n')

class Board:
    def __init__(s): s.ser = conn()
    def c(s, cmd, t=8):
        for _ in range(2):
            try: return B.cmd(s.ser, cmd, t)
            except Exception:
                time.sleep(3); s.ser = conn()
                if not s.ser: raise
        return ''
    def n(s, cmd, t=6): return N(s.c(cmd, t))

def isolated(bd):
    # 5 warmup, then N timed invokes; invoke() returns real GPT1 ms.
    bd.c('print([sentai.tpu.invoke() for _ in range(5)])', 12)
    vals = []
    # batch in chunks of 15 to keep REPL replies small
    got = 0
    while got < N_INV:
        k = min(15, N_INV - got)
        r = bd.c('print([sentai.tpu.invoke() for _ in range(%d)])' % k, 15)
        nums = [int(x) for x in re.findall(r'-?\d+', r.split('[')[-1].split(']')[0])] if '[' in r else []
        vals += [v for v in nums if v >= 0]
        got += k
    return vals

def pipeline_fps(bd):
    fps = []
    for _ in range(M_TRIAL):
        bd.c('sentai.pipeline.infer_reset()', 2)
        if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))', 5) != 0:
            bd.c('sentai.pipeline.stop()', 3); continue
        time.sleep(WARM)
        fc0 = bd.n('print(sentai.pipeline.frame_count())', 3); t0 = time.time()
        time.sleep(DUR)
        fc1 = bd.n('print(sentai.pipeline.frame_count())', 3); wall = time.time() - t0
        bd.c('sentai.pipeline.stop()', 3); time.sleep(0.5)
        if fc1 > fc0 and wall > 0: fps.append((fc1 - fc0) / wall)
    return fps

def bench(bd, name, models, yuv):
    print('\n=== %s group (yuv_mode=%d) ===' % ('YUV' if yuv else 'RGB', yuv))
    bd.c('sentai.camera.yuv_mode(%d)' % yuv, 2)
    bd.c('sentai.camera.select(0)', 2); bd.c('sentai.camera.set_resolution(640,480)', 2)
    print('cam init(1,45):', bd.n('print("R",sentai.camera.init(1,45))', 6)); time.sleep(1.5)
    print('camera.fps:', bd.c('print("F",sentai.camera.fps())', 6).split('F')[-1].strip()[:8])
    for m in models:
        if m in done_models(): print('skip', m, '(done)'); continue
        # load with retry (handles transient re-enum)
        ok = False; ni = -1
        for _ in range(3):
            r = bd.c('print("LD",sentai.tpu.load("/%s.tflite"))' % m, 14)
            if 'LD 0' in r: ok = True; break
            time.sleep(2)
        if not ok: print('FAIL load', m); continue
        ni = bd.n('print("NI",sentai.tpu.num_inputs())', 3)
        time.sleep(0.5)
        inv = isolated(bd)
        fps = pipeline_fps(bd)
        if len(inv) < 5 or len(fps) < 1:
            print('FAIL measure', m, 'inv=%d fps=%d' % (len(inv), len(fps))); continue
        imean, istd = st.mean(inv), (st.stdev(inv) if len(inv) > 1 else 0.0)
        fmean = st.mean(fps); fstd = (st.stdev(fps) if len(fps) > 1 else 0.0)
        row = [m, 'YUV' if yuv else 'RGB', ni, len(inv),
               '%.2f'%imean, '%.2f'%istd, min(inv), max(inv),
               '%.1f'%st.median(inv), len(fps), '%.2f'%fmean, '%.2f'%fstd,
               '%.2f'%min(fps), '%.2f'%max(fps), '%.2f'%(1000.0/fmean if fmean else 0)]
        ensure_header()
        with open(CSV,'a') as f: f.write(','.join(str(x) for x in row) + '\n')
        print('  %-12s in=%2d  invoke=%.1f+-%.1fms  FPS=%.1f+-%.2f (%d trials)'
              % (m, ni, imean, istd, fmean, fstd, len(fps)))

def main():
    bd = Board()
    if not bd.ser: print('NO REPL'); return
    print('build:', bd.c('print(sentai.version())', 3).strip().split('\n')[-2][:45])
    print('exists:', bd.c('print(sentai.fs.exists("/"))', 3).strip().split('\n')[-2][:10])
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)')
    bd.c('sentai.pipeline.invokes_per_frame(1)')
    ensure_header()
    bench(bd, 'RGB', RGB_MODELS, 0)
    bench(bd, 'YUV', YUV_MODELS, 1)
    print('\n=== DONE. CSV: %s ===' % CSV)
    print(open(CSV).read())
    bd.ser.close()

if __name__ == '__main__':
    main()
