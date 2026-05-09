# _t_flow_inject.py -- LED-cued flow CAPTURE + INJECT to drone EKF.
#
# Combines _t_flow_capture.py (LED schedule + OBS lines) with the
# inject path from _t_flow_to_drone.py (sentai.crazy.send_flow on
# CH=1).  Each new flow frame is BOTH printed as OBS and shipped to
# the drone EKF using the empirically-verified body_xform for cam0:
#   body_fw   = -1.0 * dx
#   body_left = +1.0 * dy
#
# Output stream (on REPL via _host_paste_bench):
#   CUE:<phase>             LED-cued phase boundary
#   OBS:t,dx,dy,conf,seq    raw flow read (mgp / 1000=grid-px)
#   INJ:t,dpx,dpy,dt,std,rc body-frame inject result (rc=0 OK)
#   STAT:sent,ok,fail,rej   final summary
#   === done ===
#
# Static-drone smoke test (first):  drone counter `deck.sentaiFlow`
# should advance by ~OBS count on the radio side, with `sentaiFlBad
# = sentaiUcrc = 0`.  Then re-run with motion.

import sentai

# Empirical body_xform[0] for cam0 + vflip=1 (verified 2026-05-07):
FW_FROM_DX  = -1.0
FW_FROM_DY  =  0.0
LF_FROM_DX  =  0.0
LF_FROM_DY  =  1.0
GRID_TO_RAW = 8.0       # 1 grid-px = 8 raw-px (PXP step-8 from VGA)
EKF_COMP    = 10.0      # PMW3901 units: drone EKF *0.1 (FLOW_RESOLUTION)


def _conf_to_std(c):
    if c >= 200: return 1.0
    if c >= 128: return 2.0
    if c >=  64: return 4.0
    return 8.0


def go(secs=16, period_ms=100, cam_id=0):
    sentai.crazy.init()
    sentai.camera.init()
    sentai.flow.enable()
    sentai.flow.start(cam_id)
    sentai.rtos.sleep_ms(400)

    schedule = [
        (    0,  1000, 'STARTUP', None),
        ( 1000,  3000, 'STILL_1', False),
        ( 3000,  6000, 'FORWARD', True),
        ( 6000,  8000, 'STILL_2', False),
        ( 8000, 11000, 'BACK',    True),
        (11000, 13000, 'STILL_3', False),
        (13000, 16000, 'LEFT',    True),
    ]
    total_ms = secs * 1000

    print('=== inject secs=%d cam_id=%d ===' % (secs, cam_id))
    t0 = sentai.rtos.ticks_ms()

    print('CUE:STARTUP')
    for _ in range(5):
        sentai.io.led_on(); sentai.rtos.sleep_ms(80)
        sentai.io.led_off(); sentai.rtos.sleep_ms(80)
    sentai.io.led_off()

    cur_idx = -1
    last_seq = -1
    last_t = sentai.rtos.ticks_ms()
    sent = ok = fail = 0

    while True:
        t_now = sentai.rtos.ticks_ms() - t0
        if t_now >= total_ms:
            break
        new_idx = cur_idx
        while (new_idx + 1 < len(schedule) and
               schedule[new_idx + 1][0] <= t_now):
            new_idx += 1
        if new_idx != cur_idx and new_idx >= 0:
            cur_idx = new_idx
            phase = schedule[cur_idx]
            if phase[3] is True:    sentai.io.led_on()
            elif phase[3] is False: sentai.io.led_off()
            print('CUE:%s' % phase[2])

        d = sentai.flow.read()
        seq = d['frame_seq']
        if seq != last_seq:
            last_seq = seq
            print('OBS:%d,%d,%d,%d,%d' %
                  (t_now, d['dx'], d['dy'], d['confidence'], seq))

            # Compute body-frame and inject
            now = sentai.rtos.ticks_ms()
            dt = (now - last_t) / 1000.0
            last_t = now
            if dt <= 0.001: dt = 0.001
            if dt > 0.20:   dt = 0.20

            dx_g = d['dx'] / 1000.0
            dy_g = d['dy'] / 1000.0
            fw_g = FW_FROM_DX * dx_g + FW_FROM_DY * dy_g
            lf_g = LF_FROM_DX * dx_g + LF_FROM_DY * dy_g
            dpx = fw_g * GRID_TO_RAW * EKF_COMP
            dpy = lf_g * GRID_TO_RAW * EKF_COMP
            std = _conf_to_std(d['confidence'])

            rc = sentai.crazy.send_flow(dpx, dpy, dt, std)
            sent += 1
            if rc == 0: ok += 1
            else:       fail += 1
            print('INJ:%d,%.2f,%.2f,%.4f,%.2f,%d' %
                  (t_now, dpx, dpy, dt, std, rc))

        sentai.rtos.sleep_ms(period_ms)

    sentai.io.led_off()
    sentai.flow.stop()
    print('STAT:%d,%d,%d,0' % (sent, ok, fail))
    print('=== done ===')


try:
    _ = _target_fps  # noqa: F821
    go(secs=16, cam_id=0)
except NameError:
    pass
