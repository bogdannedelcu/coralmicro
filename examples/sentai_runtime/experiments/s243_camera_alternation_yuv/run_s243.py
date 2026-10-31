#!/usr/bin/env python3
# s243 — camera ALTERNATION throughput with the detection pipeline, YUV only.
# Redoes the experiment.md #1077 alt-mode table (cam0:cam1 ratios) but with:
#   - YUV420 dual-input model (c2f_thick_yuv420, one model on BOTH cameras via
#     slot 0 / primary interpreter — multi-slot dual-input is not wired),
#   - the full detection pipeline live,
#   - build 1590 (invoke-timing + FPS-throttle tick bugs fixed).
# camera.ratio(a,b): CSI ISR flips the MUX so cam0 owns a / cam1 owns b frames
# per (a+b) cycle (glitch-free EOF-VBLANK). a or b = 0 -> 1-cam (no MUX flips).
# The MUX flip costs a drain/dirty-buffer skip, so 1:1 (flip every frame) is the
# worst case; higher ratios amortize it. VGA45.
import sys, time, re, os, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV = os.path.join(HERE, 'results_alt_yuv_vga45.csv')
M_TRIAL, DUR, WARM = 4, 7.0, 2.0
RATIOS = [(1,0),(1,1),(1,2),(1,3),(1,5)]   # (1,0)=1cam baseline

def N(x):
    m=re.findall(r'-?\d+',x); return int(m[-1]) if m else -999
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

def trial(bd):
    bd.c('sentai.pipeline.infer_reset()',2)
    if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))',5)!=0:
        bd.c('sentai.pipeline.stop()',3); return None
    time.sleep(WARM)
    up0=bd.n('print(sentai.rtos.ticks_ms())',3); fc0=bd.n('print(sentai.pipeline.frame_count())',3); t0=time.time()
    time.sleep(DUR)
    fc1=bd.n('print(sentai.pipeline.frame_count())',3); up1=bd.n('print(sentai.rtos.ticks_ms())',3); wall=time.time()-t0
    bd.c('sentai.pipeline.stop()',3); time.sleep(0.6)
    reset = 1 if (up1>0 and up0>0 and up1<up0) else 0
    return ((fc1-fc0)/wall if fc1>fc0 and wall>0 else 0, reset)

def main():
    bd=BD()
    if not bd.ser: print("NO REPL"); return
    print("build:", bd.c('print(sentai.version())',3).strip().split('\n')[-2][:42])
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)'); bd.c('sentai.pipeline.invokes_per_frame(1)')
    bd.c('sentai.camera.yuv_mode(1)',2); bd.c('sentai.camera.select(0)',2); bd.c('sentai.camera.set_resolution(640,480)',2)
    print("cam init(1,45):", bd.n('print("R",sentai.camera.init(1,45))',6)); time.sleep(1.5)
    print("camera.fps:", bd.c('print("F",sentai.camera.fps())',6).split('F')[-1].strip()[:6])
    ok=False
    for _ in range(3):
        if 'LD 0' in bd.c('print("LD",sentai.tpu.load("/c2f_thick_y.tflite"))',15): ok=True; break
        time.sleep(2)
    print("load c2f_thick_y:", ok)
    if not ok: return
    if not os.path.exists(CSV):
        open(CSV,'w').write('ratio,cam0,cam1,fps_trials,fps_mean,fps_std,fps_min,fps_max,resets\n')
    for a,bb in RATIOS:
        tag = "1cam" if bb==0 else "%d:%d"%(a,bb)
        rc = bd.n('print("RT",sentai.camera.ratio(%d,%d))'%(a,bb),4)
        time.sleep(1.5)   # let the MUX cadence settle
        print("\n--- ratio %s (set rc parts: %s) ---"%(tag, bd.c('print(sentai.camera.ratio())',3).strip().split('\n')[-2][:20]))
        fps=[]; resets=0
        for tr in range(M_TRIAL):
            r=trial(bd)
            if not r: print("  trial %d: start fail"%tr); continue
            f,rs=r; resets+=rs
            if f>0: fps.append(f)
            print("  trial %d: %.1f fps reset=%d"%(tr,f,rs))
        m=st.mean(fps) if fps else 0; s=st.stdev(fps) if len(fps)>1 else 0
        row=[tag,a,bb,len(fps),'%.2f'%m,'%.2f'%s,'%.2f'%(min(fps) if fps else 0),'%.2f'%(max(fps) if fps else 0),resets]
        open(CSV,'a').write(','.join(str(x) for x in row)+'\n')
        print("  >>> %s: %.1f +- %.2f fps (resets %d/%d)"%(tag,m,s,resets,M_TRIAL))
    # back to 1cam
    bd.c('sentai.camera.ratio(1,0)',3)
    print("\n=== DONE ===\n"+open(CSV).read())
    bd.ser.close()
if __name__=='__main__': main()
