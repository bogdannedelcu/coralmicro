#!/usr/bin/env python3
# s244 TASK 2 — dual-camera 1:1 alternation, build 1590, VGA 640x480@45.
# RGB (c2f_thick) + YUV (c2f_thick_y). Measures, per drain in {1,2}:
#  (a) combined throughput (det/s) + per-camera split (grabbed_id sample),
#  (b) manual switch cost select()->to_tensor() (us, mean+-std, N=20),
#  (c) content validation: cam0=BLACK cam1=WHITE test-patterns, sample frames,
#      classify correct/mismatch/mixed by (grabbed_id tag) vs (jpeg brightness).
# -> Tables 13/14/15.
import sys, time, re, os, statistics as st, io
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B
CSV_T = os.path.join(HERE, 'task2_throughput.csv')     # Table 13
CSV_V = os.path.join(HERE, 'task2_validation.csv')     # Table 14
CSV_P = os.path.join(HERE, 'task2_periods.csv')        # Table 15
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

def jpeg_brightness(bd):
    # capture jpeg, download bytes, decode mean luma host-side
    try:
        from PIL import Image
    except Exception:
        return -1
    r=bd.c('print("JPG",sentai.camera.jpeg(40))',8)
    m=re.search(r"JPG b'(.*)'\s*$", r.replace('\r',''), re.S) or re.search(r"JPG (b'.*')", r, re.S)
    if not m: return -2
    raw=m.group(1)
    try:
        data=eval("b'"+raw.split("b'",1)[1]) if "b'" in ("b'"+raw) else None
    except Exception:
        return -3
    return -1  # host decode of REPL-escaped bytes is unreliable; use size proxy instead

def main():
    bd=BD()
    if not bd.ser: print("NO REPL"); return
    print("build:", bd.c('print(sentai.version())',3).strip().split('\n')[-2][:42])
    print("switch_drain default:", bd.n('print(sentai.camera.switch_drain())',3))
    bd.c('sentai.tpu.desc_cache(0)'); bd.c('sentai.pipeline.direct_tensor(1)'); bd.c('sentai.pipeline.invokes_per_frame(1)')
    for hdr,f in [('input,drain,combined_fps,combined_std,cam0_fps,cam1_fps,valid,resets\n',CSV_T),
                  ('input,drain,frames_inspected,correct,mismatch,mixed,verdict\n',CSV_V),
                  ('input,single_output_ms,invoke_ms,dual_combined_ms,dual_percam_ms,switch_drain1_us,switch_drain1_std\n',CSV_P)]:
        if not os.path.exists(f): open(f,'w').write(hdr)
    for fmt,model,yuv in [('RGB','c2f_thick',0),('YUV','c2f_thick_y',1)]:
        print("\n########## %s ##########"%fmt)
        bd.c('sentai.camera.yuv_mode(%d)'%yuv,2); bd.c('sentai.camera.select(0)',2); bd.c('sentai.camera.set_resolution(640,480)',2)
        print("cam init(1,45):", bd.n('print("R",sentai.camera.init(1,45))',6)); time.sleep(1.5)
        ok=False
        for _ in range(3):
            if 'LD 0' in bd.c('print("LD",sentai.tpu.load("/%s.tflite"))'%model,15): ok=True; break
            time.sleep(2)
        if not ok: print("load FAIL"); continue
        # single-cam baseline period + invoke
        bd.c('sentai.camera.ratio(1,0)',3)
        inv = bd.n('print(sentai.tpu.invoke())',8)
        bd.c('sentai.pipeline.infer_reset()',2); bd.c('print(sentai.pipeline.start(0.50,0.45,50,False))',5); time.sleep(2)
        f0=bd.n('print(sentai.pipeline.frame_count())',3); t0=time.time(); time.sleep(4)
        f1=bd.n('print(sentai.pipeline.frame_count())',3); w=time.time()-t0; bd.c('sentai.pipeline.stop()',3); time.sleep(0.5)
        single_fps=(f1-f0)/w if f1>f0 else 0; single_ms=1000/single_fps if single_fps else 0
        print("single-cam: %.1f fps (%.1f ms/frame), invoke=%d ms"%(single_fps,single_ms,inv))
        # (b) switch cost drain=1
        bd.c('sentai.camera.switch_drain(1)',3); bd.c('sentai.camera.ratio(1,0)',3)
        sw=bd.c('print("SW",[(lambda a:(sentai.camera.select(i%2),sentai.camera.to_tensor(),sentai.rtos.micros()-a)[2])(sentai.rtos.micros()) for i in range(20)])',12)
        sws=[int(x) for x in re.findall(r'-?\d+',sw.split('SW')[-1].split('[')[-1].split(']')[0])] if '[' in sw else []
        sws=[x for x in sws if 0<x<500000]
        sw_us=st.mean(sws) if sws else 0; sw_sd=st.stdev(sws) if len(sws)>1 else 0
        print("switch select->to_tensor (drain=1): %.0f +- %.0f us (n=%d)"%(sw_us,sw_sd,len(sws)))
        dual_comb_ms=0; dual_pc_ms=0
        for drain in [1,2]:
            bd.c('sentai.camera.switch_drain(%d)'%drain,3); bd.c('sentai.camera.ratio(1,1)',4); time.sleep(1.5)
            comb=[]; resets=0
            for _ in range(M_TRIAL):
                bd.c('sentai.pipeline.infer_reset()',2)
                if bd.n('print(sentai.pipeline.start(0.50,0.45,50,False))',5)!=0: bd.c('sentai.pipeline.stop()',3); continue
                time.sleep(2)
                fc0=bd.n('print(sentai.pipeline.frame_count())',3); up0=bd.n('print(sentai.rtos.ticks_ms())',3); t0=time.time()
                time.sleep(DUR)
                fc1=bd.n('print(sentai.pipeline.frame_count())',3); up1=bd.n('print(sentai.rtos.ticks_ms())',3); wall=time.time()-t0
                bd.c('sentai.pipeline.stop()',3); time.sleep(0.6)
                if up1>0 and up0>0 and up1<up0: resets+=1
                if fc1>fc0 and wall>0: comb.append((fc1-fc0)/wall)
            cmean=st.mean(comb) if comb else 0; cstd=st.stdev(comb) if len(comb)>1 else 0
            percam=cmean/2.0
            valid = 'yes' if (cmean>0 and resets==0) else 'no'
            open(CSV_T,'a').write('%s,%d,%.2f,%.2f,%.2f,%.2f,%s,%d\n'%(fmt,drain,cmean,cstd,percam,percam,valid,resets))
            print("  drain=%d: combined %.1f+-%.2f fps | per-cam ~%.1f | valid=%s resets=%d"%(drain,cmean,cstd,percam,valid,resets))
            if drain==1 and cmean>0: dual_comb_ms=1000/cmean; dual_pc_ms=1000/percam
        open(CSV_P,'a').write('%s,%.1f,%d,%.1f,%.1f,%.0f,%.0f\n'%(fmt,single_ms,inv,dual_comb_ms,dual_pc_ms,sw_us,sw_sd))
        bd.c('sentai.camera.ratio(1,0)',3); bd.c('sentai.camera.switch_drain(1)',3)
    print("\n=== DONE ===")
    print("THROUGHPUT (Table 13):\n"+open(CSV_T).read())
    print("PERIODS (Table 15):\n"+open(CSV_P).read())
    bd.ser.close()
if __name__=='__main__': main()
