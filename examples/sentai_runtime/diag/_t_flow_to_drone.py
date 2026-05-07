# _t_flow_to_drone.py -- feed sentai.flow output to drone EKF on CH=1.
#
# Pulls the latest phase-correlation result from sentai.flow.read(),
# converts mgp->raw-px, applies the cam0/cam1 -> body-frame transform
# (per paper/flow_body_frame.md), and ships it as a 16-byte
# flow_pkt_t over CH=1. Drone deck driver hands the packet to
# estimatorEnqueueFlow() -- the EKF treats us as a legitimate flow
# source.
#
# Usage:
#   import diag._t_flow_to_drone as f
#   f.run(secs=30, cam_id=0)
#
# Side channel: while this runs, host can interrogate drone via
# radio for `deck.sentaiFlow` PARAM (counter increments) and use
# `$sentai.flow.read()` over the radio bridge to see live values.

import sentai

# 1 grid-px = 8 raw-px (FLOW_GRAY_W=80 from raw VGA 640x480)
GRID_TO_RAW_PX = 8.0


def _conf_to_std(conf):
    """Cheap heuristic: low confidence -> high std. Bitcraze flow_v2
    uses ~0.5..4 px range; we pick a similar band keyed off our 0..255
    confidence score so the EKF weights low-quality matches less."""
    if conf >= 200:
        return 1.0
    if conf >= 128:
        return 2.0
    if conf >= 64:
        return 4.0
    return 8.0   # very low conf: still send, but the EKF will mostly ignore


def _body_xy(d):
    """mgp dx/dy from sentai.flow.read() -> (body_fw_px, body_left_px)
    in raw pixels per the cam0/cam1 sign convention."""
    cam = d['cam_id']
    dx_grid = d['dx'] / 1000.0
    dy_grid = d['dy'] / 1000.0
    if cam == 0:
        body_fw_grid   = -dx_grid
        body_left_grid = +dy_grid
    else:
        body_fw_grid   = +dx_grid
        body_left_grid = -dy_grid
    return (body_fw_grid * GRID_TO_RAW_PX,
            body_left_grid * GRID_TO_RAW_PX)


def run(secs=30, cam_id=0, send_every_n=1, max_dt_s=0.20):
    """Run the flow-to-drone publisher loop for `secs` seconds.

    secs          - total wall-time budget
    cam_id        - 0 (front) or 1 (back); see flow_body_frame.md
    send_every_n  - send every Nth read (e.g. 2 = 50 Hz from a 100 Hz
                    flow, when we want to throttle bandwidth)
    max_dt_s      - clamp dt at this ceiling to prevent the EKF from
                    seeing a giant gap if the flow stalled
    """
    # Bring up bridge + camera + flow.  Camera must be init'd before
    # flow.start or the publisher_task wakes against a silent CSI ISR
    # and SERR_FLOW_NOTIFY_TIMEOUT (0E30) fires within ~500 ms.
    sentai.crazy.init()
    sentai.camera.init(1)        # 1 = on; default VGA/30
    sentai.flow.enable()
    sentai.flow.start(cam_id)
    sentai.rtos.sleep_ms(400)   # let the publisher prime

    sent = 0
    sent_ok = 0
    sent_fail = 0
    last_seq = 0
    last_t_ms = sentai.rtos.ticks_ms()
    t_end = sentai.rtos.ticks_ms() + secs * 1000

    while sentai.rtos.ticks_ms() < t_end:
        d = sentai.flow.read()
        seq = d['frame_seq']
        if seq == last_seq:
            sentai.rtos.sleep_ms(5)   # no new frame yet
            continue
        last_seq = seq

        if (seq % send_every_n) != 0:
            continue

        now_ms = sentai.rtos.ticks_ms()
        dt = (now_ms - last_t_ms) / 1000.0
        last_t_ms = now_ms
        if dt <= 0.001:
            dt = 0.001        # drone rejects dt <= 0
        if dt > max_dt_s:
            dt = max_dt_s     # clamp to avoid huge "snap"

        dpx, dpy = _body_xy(d)
        std = _conf_to_std(d['confidence'])

        rc = sentai.crazy.send_flow(dpx, dpy, dt, std)
        sent += 1
        if rc == 0:
            sent_ok += 1
        else:
            sent_fail += 1

    print('flow->drone: sent=%d ok=%d fail=%d' % (sent, sent_ok, sent_fail))
    print('  last d=%r' % d)
    print('  last (dpx,dpy,dt,std)=(%.3f, %.3f, %.4f, %.2f)' %
          (dpx, dpy, dt, std))
    return (sent, sent_ok, sent_fail)


# Bench harness: when streamed via _host_paste_bench.py, _target_fps is set
# but unused here; we just run a single 6 s pass and print '=== done ===' so
# the harness exits cleanly.
try:
    _ = _target_fps  # noqa: F821 - injected by _host_paste_bench
    run(secs=6, cam_id=0)
    print('=== done ===')
except NameError:
    pass
