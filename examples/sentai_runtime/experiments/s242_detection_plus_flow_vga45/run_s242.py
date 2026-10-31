#!/usr/bin/env python3
# s242 — detection pipeline + optical flow CONCURRENT, one model
# (c2f_thick_yuv420, the s241 champion), VGA45, build 1590.
#
# Flow consumes PrepTask's shared FLOW_GRAY_80x60 Y slot (use_prep_slot=1):
# ONE camera read feeds BOTH detection and flow. Question: how much does
# concurrent flow cost detection FPS, what flow rate is achieved, and is it
# stable (mem: flow+detection can saturate the M7 SEMC/OCRAM buses -> resets)?
#
# Conditions (M trials each, mean+-std): detect-only | +flow(M7) | +flow(M4 offload).
import sys, time, re, os, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV = os.path.join(HERE, 'results_flow_vga45.csv')
MODEL = 'c2f_thick_y'   # c2f_thick_yuv420
M_TRIAL, DUR, WARM = 4, 8.0, 2.0
SLOT_DIV = 2

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

def flow_seq(bd):
    r = bd.c('print("FS",sentai.flow.read_tuple())', 4)
    # tuple (alive,state,dx,dy,confidence,frame_seq,cam_id)
    nums = re.findall(r'-?\d+', r.split('FS')[-1])
    return int(nums[5]) if len(nums)>=6 else -1
def flow_sample(bd):
    r = bd.c('print("FR",sentai.flow.read())', 4)
    d={}
    for k,v in re.findall(r"'(\w+)':\s*(-?\d+)", r): d[k]=int(v)
    return d

def trial(bd, mode):
    # mode: 'det' | 'm7' | 'm4'
    bd.c('sentai.pipeline.infer_reset()',2)
    if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))',5)!=0:
        bd.c('sentai.pipeline.stop()',3); return None
    if mode in ('m7','m4'):
        bd.c('print(sentai.flow.enable())',3)
        bd.c('sentai.flow.use_prep_slot(1)',2); bd.c('sentai.flow.slot_div(%d)'%SLOT_DIV,2)
        bd.c('print(sentai.flow.start(0))',4)
        if mode=='m4': bd.c('print(sentai.flow.m4_start(1500))',6)
    time.sleep(WARM)
    up0=bd.n('print(sentai.rtos.ticks_ms())',3)
    fc0=bd.n('print(sentai.pipeline.frame_count())',3)
    fs0=flow_seq(bd) if mode!='det' else 0
    t0=time.time()
    time.sleep(DUR)
    fc1=bd.n('print(sentai.pipeline.frame_count())',3)
    fs1=flow_seq(bd) if mode!='det' else 0
    up1=bd.n('print(sentai.rtos.ticks_ms())',3)
    wall=time.time()-t0
    samp=flow_sample(bd) if mode!='det' else {}
    reset = 1 if (up1>0 and up0>0 and up1<up0) else 0
    if mode=='m4': bd.c('sentai.flow.m4_stop()',4)
    if mode!='det': bd.c('sentai.flow.stop()',3)
    bd.c('sentai.pipeline.stop()',3); time.sleep(0.6)
    det_fps = (fc1-fc0)/wall if (fc1>fc0 and wall>0) else 0
    flow_fps = (fs1-fs0)/wall if (fs1>fs0 and wall>0) else 0
    return dict(det_fps=det_fps, flow_fps=flow_fps, reset=reset,
                conf=samp.get('confidence',-1), dx=samp.get('dx',0), dy=samp.get('dy',0),
                sad=samp.get('sad',-1))

def run_mode(bd, name, mode):
    print("\n=== %s ==="%name)
    dets=[]; flows=[]; resets=0; last=None
    for tr in range(M_TRIAL):
        r=trial(bd,mode)
        if not r: print("  trial %d: start failed"%tr); continue
        last=r; resets+=r['reset']
        if r['det_fps']>0: dets.append(r['det_fps'])
        if mode!='det' and r['flow_fps']>0: flows.append(r['flow_fps'])
        print("  trial %d: det=%.1f fps  flow=%.1f fps  reset=%d  conf=%d dx=%d dy=%d"
              %(tr,r['det_fps'],r['flow_fps'],r['reset'],r['conf'],r['dx'],r['dy']))
    dmean=st.mean(dets) if dets else 0; dstd=st.stdev(dets) if len(dets)>1 else 0
    fmean=st.mean(flows) if flows else 0; fstd=st.stdev(flows) if len(flows)>1 else 0
    row=[name, len(dets), '%.2f'%dmean,'%.2f'%dstd, len(flows), '%.2f'%fmean,'%.2f'%fstd,
         resets, last['conf'] if last else -1]
    hdr='mode,det_trials,det_fps_mean,det_fps_std,flow_trials,flow_fps_mean,flow_fps_std,resets,last_conf\n'
    if not os.path.exists(CSV): open(CSV,'w').write(hdr)
    open(CSV,'a').write(','.join(str(x) for x in row)+'\n')
    print("  => det %.1f+-%.2f fps | flow %.1f+-%.2f fps | resets %d/%d"
          %(dmean,dstd,fmean,fstd,resets,M_TRIAL))
    return dmean

def main():
    bd=BD()
    if not bd.ser: print("NO REPL"); return
    print("build:", bd.c('print(sentai.version())',3).strip().split('\n')[-2][:45])
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)')
    bd.c('sentai.pipeline.invokes_per_frame(1)')
    bd.c('sentai.camera.yuv_mode(1)',2); bd.c('sentai.camera.select(0)',2)
    bd.c('sentai.camera.set_resolution(640,480)',2)
    print("cam init(1,45):", bd.n('print("R",sentai.camera.init(1,45))',6)); time.sleep(1.5)
    print("cam fps:", bd.c('print("F",sentai.camera.fps())',6).split('F')[-1].strip()[:6])
    ok=False
    for _ in range(3):
        if 'LD 0' in bd.c('print("LD",sentai.tpu.load("/%s.tflite"))'%MODEL,15): ok=True; break
        time.sleep(2)
    print("load %s: %s (slot_div=%d)"%(MODEL,ok,SLOT_DIV))
    if not ok: return
    base=run_mode(bd,'detect_only','det')
    m7=run_mode(bd,'detect+flow_M7','m7')
    m4=run_mode(bd,'detect+flow_M4','m4')
    print("\n=== SUMMARY (c2f_thick_yuv420 @ VGA45) ===")
    print("detect-only:      %.1f fps"%base)
    print("detect+flow(M7):  %.1f fps  (%.0f%% of baseline)"%(m7,100*m7/base if base else 0))
    print("detect+flow(M4):  %.1f fps  (%.0f%% of baseline)"%(m4,100*m4/base if base else 0))
    print("\n"+open(CSV).read())
    bd.ser.close()
if __name__=='__main__': main()
