#!/usr/bin/env python3
# s244 TASK 1 — multi-patch: single-camera pipeline with N invokes/frame (N patches
# of the same frame), N=1..4, RGB (c2f_thick) + YUV (c2f_thick_yuv420). Build 1590,
# VGA 640x480@45. Method as s241: isolated invoke = 5 warmup + 30 GPT1-us invokes;
# pipeline = M=3 x 6 s live. -> Table 12.
import sys, time, re, os, statistics as st
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B
CSV = os.path.join(HERE, 'task1_multipatch.csv')
M_TRIAL, DUR = 3, 6.0
def N(x):
    m=re.findall(r'-?\d+',x); return int(m[-1]) if m else -999
def conn():
    for _ in range(20):
        for p in ['/dev/ttyACM0','/dev/ttyACM1']:
            try:
                B.PORT=p; s=B.open_repl()
                if 'OK' in B.cmd(s,'import sentai; print("OK")',3): B.cmd(s,'sentai.verbose(1)'); return s
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
        return ''
    def n(s,cmd,t=6): return N(s.c(cmd,t))
    def stat(s,key):  # from infer_stats dict
        r=s.c('print("IS",sentai.pipeline.infer_stats())',4); m=re.search(r"'%s':\s*(-?\d+)"%key,r)
        return int(m.group(1)) if m else -1
def isolated(bd):
    bd.c('sentai.pipeline.invokes_per_frame(1)',2)
    bd.c('print([sentai.tpu.invoke() for _ in range(5)])',12)
    v=[]; got=0
    while got<30:
        k=min(15,30-got); r=bd.c('print([sentai.tpu.invoke() for _ in range(%d)])'%k,15)
        v+=[int(x) for x in re.findall(r'-?\d+',r.split('[')[-1].split(']')[0])] if '[' in r else []
        got+=k
    return [x for x in v if 0<x<300][:30]
def bench(bd, fmt, model, yuv):
    print("\n=== %s (%s, yuv_mode=%d) ==="%(fmt,model,yuv))
    bd.c('sentai.camera.yuv_mode(%d)'%yuv,2); bd.c('sentai.camera.select(0)',2); bd.c('sentai.camera.set_resolution(640,480)',2)
    print("cam init(1,45):", bd.n('print("R",sentai.camera.init(1,45))',6)); time.sleep(1.5)
    print("camera.fps:", bd.c('print("F",sentai.camera.fps())',6).split('F')[-1].strip()[:5])
    ok=False
    for _ in range(3):
        if 'LD 0' in bd.c('print("LD",sentai.tpu.load("/%s.tflite"))'%model,15): ok=True; break
        time.sleep(2)
    if not ok: print("load FAIL"); return
    inv=isolated(bd)
    iso_ms = st.mean(inv) if inv else 0; iso_sd = st.stdev(inv) if len(inv)>1 else 0
    print("isolated invoke: %.2f +- %.2f ms (n=%d)"%(iso_ms,iso_sd,len(inv)))
    for Np in [1,2,3,4]:
        bd.c('sentai.pipeline.invokes_per_frame(%d)'%Np,2)
        ftp=[]; itp=[]; ipf=[]; tos=[]; fails=[]
        for _ in range(M_TRIAL):
            bd.c('sentai.pipeline.infer_reset()',2); bd.c('sentai.pipeline.prep_reset()',2)
            if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))',5)!=0: bd.c('sentai.pipeline.stop()',3); continue
            time.sleep(2)
            fc0=bd.n('print(sentai.pipeline.frame_count())',3); ok0=bd.stat('ok'); to0=bd.n('print(sentai.pipeline.prep_buf_timeout())',3); t0=time.time()
            time.sleep(DUR)
            fc1=bd.n('print(sentai.pipeline.frame_count())',3); ok1=bd.stat('ok'); fl1=bd.stat('fail'); to1=bd.n('print(sentai.pipeline.prep_buf_timeout())',3); wall=time.time()-t0
            bd.c('sentai.pipeline.stop()',3); time.sleep(0.6)
            if fc1>fc0 and wall>0:
                ftp.append((fc1-fc0)/wall); itp.append((ok1-ok0)/wall)
                ipf.append((ok1-ok0)/(fc1-fc0)); tos.append(max(0,to1-to0)); fails.append(fl1)
        if not ftp: print("  N=%d: FAIL"%Np); continue
        row=[Np, '%.2f'%st.mean(ipf), fmt, '%.2f'%st.mean(ftp),
             '%.2f'%(st.stdev(ftp) if len(ftp)>1 else 0), '%.2f'%st.mean(itp),
             '%.2f'%(st.stdev(itp) if len(itp)>1 else 0), int(st.mean(tos)), int(st.mean(fails)),
             '%.2f'%iso_ms, '%.2f'%iso_sd]
        hdr='patches_per_frame,invokes_per_frame_measured,input,frame_tp_fps,frame_tp_std,invoke_tp_ips,invoke_tp_std,timeouts,infer_failures,isolated_invoke_ms,isolated_invoke_std\n'
        if not os.path.exists(CSV): open(CSV,'w').write(hdr)
        open(CSV,'a').write(','.join(str(x) for x in row)+'\n')
        print("  N=%d: inv/frame=%.2f  frame_tp=%.1f fps  invoke_tp=%.1f ips  timeouts=%d fails=%d"
              %(Np, st.mean(ipf), st.mean(ftp), st.mean(itp), int(st.mean(tos)), int(st.mean(fails))))
def main():
    bd=BD()
    if not bd.ser: print("NO REPL"); return
    print("build:", bd.c('print(sentai.version())',3).strip().split('\n')[-2][:42])
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)')
    bench(bd,'RGB','c2f_thick',0)
    bench(bd,'YUV','c2f_thick_y',1)
    bd.c('sentai.pipeline.invokes_per_frame(1)',2)
    print("\n=== DONE ===\n"+open(CSV).read())
    bd.ser.close()
if __name__=='__main__': main()
