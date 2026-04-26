# A/B test: DEFER vs REARM at SXGA30 alt 1:1, with call-stat diff per run.
# Goal: see exactly what each mode does at the SendParameters /
# SendInstructions / SendInputs level.  Confirms or refutes our
# hypothesis about why REARM's avg invoke is 2× DEFER's.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(200)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(label, n, mode, ratio, drain, dur):
    sentai.pipeline.invokes_per_frame(n)
    sentai.pipeline.multi_invoke_mode(mode)
    sentai.camera.switch_drain(drain)
    sentai.camera.ratio(ratio[0], ratio[1])
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    sentai.diag.tpu_call_stats(1)        # reset call counters
    pre = sentai.diag.tpu_call_stats()
    fc0 = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("--- %s START FAIL rc=%s ---" % (label, rc)); return
    sentai.rtos.sleep_ms(dur)
    fc1 = sentai.camera.frame_count()
    t1 = sentai.rtos.ticks_ms()
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    post = sentai.diag.tpu_call_stats()
    sentai.pipeline.stop()
    dt = t1 - t0
    ok = istats['ok']
    # Diff post-pre for call counters
    dp_c = post['p_calls']  - pre['p_calls']
    di_c = post['i_calls']  - pre['i_calls']
    dn_c = post['in_calls'] - pre['in_calls']
    dp_b = post['p_bytes']  - pre['p_bytes']
    di_b = post['i_bytes']  - pre['i_bytes']
    dn_b = post['in_bytes'] - pre['in_bytes']
    dnd  = post['in_done']  - pre['in_done']
    print("--- %s ---" % label)
    print("  ok=%d/%.1ffps fail=%d avg_invoke=%dms" % (
        ok, 1000.0*ok/dt, istats['fail'],
        istats['ms_sum']//ok if ok else 0))
    if ok:
        print("  per_invoke: P=%.2fx/%dB  I=%.2fx/%dB  In=%.2fx/%dB  in_done=%.2f" % (
            dp_c/ok, dp_b//ok, di_c/ok, di_b//ok, dn_c/ok, dn_b//ok, dnd/ok))
    sentai.rtos.sleep_ms(400)

DUR=5000
run("REF n=1 single",      1, 0, (0,0), 1, DUR)
run("REF n=1 alt 1:1",     1, 0, (1,1), 1, DUR)
run("A:DEFER n=2 alt 1:1", 2, 1, (1,1), 1, DUR)
run("B:REARM n=2 alt 1:1", 2, 2, (1,1), 1, DUR)

sentai.pipeline.invokes_per_frame(1); sentai.pipeline.multi_invoke_mode(0)
sentai.camera.ratio(0,0); sentai.camera.select(0); sentai.camera.switch_drain(1)
print("=== done ===")
