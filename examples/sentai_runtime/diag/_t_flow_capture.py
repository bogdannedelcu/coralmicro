# _t_flow_capture.py -- minimal LED-cued flow capture for orientation calib.
#
# Standalone bench script: bring up flow, run a 16 s schedule with LED
# phase cues, print OBS lines suitable for host-side analysis.
# Kept short (no transform / send_flow / verify_orientation etc.) so the
# REPL paste path stays under ~3 KB and streams quickly.
#
# Run via:  diag/_host_paste_bench.py --file diag/_t_flow_capture.py --fps -1

import sentai


def capture(secs=16, period_ms=100, cam_id=0):
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
        (16000, 18000, 'STILL_4', False),
        (18000, 21000, 'RIGHT',   True),
    ]
    total_ms = secs * 1000

    print('=== capture secs=%d cam_id=%d ===' % (secs, cam_id))
    t0 = sentai.rtos.ticks_ms()

    print('CUE:STARTUP')
    for _ in range(5):
        sentai.io.led_on(); sentai.rtos.sleep_ms(80)
        sentai.io.led_off(); sentai.rtos.sleep_ms(80)
    sentai.io.led_off()

    cur_idx = -1
    last_seq = -1
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
            led = phase[3]
            if led is True:
                sentai.io.led_on()
            elif led is False:
                sentai.io.led_off()
            print('CUE:%s' % phase[2])
        d = sentai.flow.read()
        seq = d['frame_seq']
        if seq != last_seq:
            last_seq = seq
            print('OBS:%d,%d,%d,%d,%d' %
                  (t_now, d['dx'], d['dy'], d['confidence'], seq))
        sentai.rtos.sleep_ms(period_ms)

    sentai.io.led_off()
    sentai.flow.stop()
    print('=== done ===')


# Bench harness dispatch
try:
    _ = _target_fps  # noqa: F821
    if _target_fps == -2:
        capture(secs=21, cam_id=0)
    else:
        capture(secs=16, cam_id=0)
except NameError:
    pass
