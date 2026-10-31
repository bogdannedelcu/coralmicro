#!/usr/bin/env python3
# s241 YUV sweep — the 8 YUV420 dual-input detectors (MLflow sentai_yuv420_v1)
# on VGA45, build 1590 (throttle fixed). Same protocol as run_s241.py.
# Models on board as <arch>_y.tflite. Writes results_yuv_vga45.csv incrementally.
import sys, time, re, os, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV = os.path.join(HERE, 'results_yuv_vga45.csv')
N_INV, M_TRIAL, DUR, WARM = 30, 3, 6.0, 2.0
MODELS = ['msblock','c3','c2f','c2f_pan2','gelan','c2f_thick','gelan_pan2','c2f_deep']

def N(x):
    m = re.findall(r'-?\d+', x); return int(m[-1]) if m else -999
def conn():
    for _ in range(20):
        for p in ['/dev/ttyACM0','/dev/ttyACM1']:
            try:
                B.PORT=p; s=B.open_repl()
                if 'OK' in B.cmd(s,'import sentai; print("OK")',3):
                    B.cmd(s,'sentai.verbose(1)'); return s
                s.close()
            except Exception: pass
        time.sleep(1.5)
    return None
def done_models():
    if not os.path.exists(CSV): return set()
    return {ln.split(',')[0] for ln in open(CSV).read().splitlines()[1:] if ln.strip()}
def ensure_header():
    if not os.path.exists(CSV):
        open(CSV,'w').write('model,fmt,n_inputs,invoke_n,invoke_mean_ms,invoke_std_ms,'
            'invoke_min_ms,invoke_max_ms,invoke_median_ms,fps_trials,fps_mean,fps_std,'
            'fps_min,fps_max,per_ms_frame\n')
class BD:
    def __init__(s): s.ser=conn()
    def c(s,cmd,t=8):
        for _ in range(3):
            try: return B.cmd(s.ser,cmd,t)
            except Exception:
                time.sleep(4); s.ser=conn()
                if not s.ser: raise
        return ''
    def n(s,cmd,t=6): return N(s.c(cmd,t))
def isolated(bd):
    bd.c('print([sentai.tpu.invoke() for _ in range(5)])',12)
    vals=[]; got=0
    while got<N_INV:
        k=min(15,N_INV-got)
        r=bd.c('print([sentai.tpu.invoke() for _ in range(%d)])'%k,15)
        vals+=[int(x) for x in re.findall(r'-?\d+',r.split('[')[-1].split(']')[0])] if '[' in r else []
        got+=k
    return [v for v in vals if 0<v<200][:N_INV]
def pipeline_fps(bd):
    fps=[]
    for _ in range(M_TRIAL):
        bd.c('sentai.pipeline.infer_reset()',2)
        if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))',5)!=0:
            bd.c('sentai.pipeline.stop()',3); continue
        time.sleep(WARM)
        fc0=bd.n('print(sentai.pipeline.frame_count())',3); t0=time.time()
        time.sleep(DUR)
        fc1=bd.n('print(sentai.pipeline.frame_count())',3); wall=time.time()-t0
        bd.c('sentai.pipeline.stop()',3); time.sleep(0.5)
        if fc1>fc0 and wall>0: fps.append((fc1-fc0)/wall)
    return fps
def main():
    bd=BD()
    if not bd.ser: print('NO REPL'); return
    print('build:', bd.c('print(sentai.version())',3).strip().split('\n')[-2][:45])
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)')
    bd.c('sentai.pipeline.invokes_per_frame(1)')
    bd.c('sentai.camera.yuv_mode(1)',2); bd.c('sentai.camera.select(0)',2)
    bd.c('sentai.camera.set_resolution(640,480)',2)
    print('cam init(1,45):', bd.n('print("R",sentai.camera.init(1,45))',6)); time.sleep(1.5)
    print('camera.fps:', bd.c('print("F",sentai.camera.fps())',6).split('F')[-1].strip()[:8])
    ensure_header()
    for m in MODELS:
        if m in done_models(): print('skip',m); continue
        ok=False
        for _ in range(3):
            if 'LD 0' in bd.c('print("LD",sentai.tpu.load("/%s_y.tflite"))'%m,15): ok=True; break
            time.sleep(2)
        if not ok: print('FAIL load',m); continue
        ni=bd.n('print("NI",sentai.tpu.num_inputs())',3); time.sleep(0.5)
        inv=isolated(bd); fps=pipeline_fps(bd)
        if len(inv)<5 or len(fps)<1: print('FAIL measure',m,'inv=%d fps=%d'%(len(inv),len(fps))); continue
        im,istd=st.mean(inv),(st.stdev(inv) if len(inv)>1 else 0)
        fm,fstd=st.mean(fps),(st.stdev(fps) if len(fps)>1 else 0)
        row=[m,'YUV',ni,len(inv),'%.2f'%im,'%.2f'%istd,min(inv),max(inv),'%.1f'%st.median(inv),
             len(fps),'%.2f'%fm,'%.2f'%fstd,'%.2f'%min(fps),'%.2f'%max(fps),'%.2f'%(1000/fm if fm else 0)]
        open(CSV,'a').write(','.join(str(x) for x in row)+'\n')
        print('  %-12s in=%d invoke=%.1f+-%.1fms FPS=%.1f+-%.2f (%d)'%(m,ni,im,istd,fm,fstd,len(fps)))
    print('\n=== DONE ===\n'+open(CSV).read())
    bd.ser.close()
if __name__=='__main__': main()
