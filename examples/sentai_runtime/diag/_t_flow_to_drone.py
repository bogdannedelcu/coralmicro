# _t_flow_to_drone.py -- feed sentai.flow output to drone EKF on CH=1.
#
# Pulls the latest phase-correlation result from sentai.flow.read(),
# converts our 80x60 flow grid pixel deltas to PMW3901-equivalent
# pixel deltas (so the drone EKF, which assumes Bitcraze flow_v2
# geometry, predicts correctly), applies a configurable image -> body
# frame transform, and ships a 16-byte flow_pkt_t over CH=1.  Drone
# deck driver hands the packet to estimatorEnqueueFlow().
#
# DESIGN: every magic number lives in DEFAULTS so we can tune them at
# the REPL without rebuilding.  Defaults are the BASELINE established
# 2026-05-07 -- see "Board + image-axis convention" below.  Reconfigure
# any of them via run() kwargs without touching this file.
#
# === Board + image-axis convention (BASELINE 2026-05-07) ====================
#
# Hardware mount on the Crazyflie 2.1:
#   * cam0 is the camera physically closest to the SentAI board's USB-C
#     port ("back" of the assembly).
#   * cam1 is the camera at the other end of the board.
#   * Drone body +x (FORWARD) = direction from cam0 toward cam1.
#   * Drone body +y (LEFT), +z (UP) follow the standard right-hand frame.
#
# Camera orientation chosen for FLOW (cam_hflip=0, cam_vflip=1):
#   * vflip=1 was set so the captured image visually matches the natural
#     reading orientation of the calibration target (verified by photo
#     pair _orientation_cam0_default.jpg vs _orientation_cam0_vflip.jpg).
#   * hflip=0 (no horizontal mirror).
#
# In that vflip=1 buffer, the image-axis -> body-axis convention is:
#   * Image LEFT   ←→  body FORWARD  (+x)
#   * Image RIGHT  ←→  body BACKWARD (-x)
#   * Image TOP    ←→  body RIGHT    (-y)
#   * Image BOTTOM ←→  body LEFT     (+y)
#
# Phase-correlation sign convention (standard): +dx, +dy in the
# sentai.flow output are the displacement of features in the BUFFER
# frame between consecutive frames.  +dx = features moved RIGHT in
# buffer; +dy = features moved DOWN in buffer.
#
# Combining both: when the drone moves PHYSICALLY,
#   forward (+body_x):  features move toward image RIGHT  →  flow dx > 0
#   left    (+body_y):  features move toward image TOP    →  flow dy < 0
# so the body-frame components are
#   body_fw   = +1.0 * dx + 0.0 * dy
#   body_left =  0.0 * dx + (-1.0) * dy
# encoded in DEFAULTS['body_xform'][0] = (+1.0, 0.0, 0.0, -1.0).
#
# cam1 is mounted at the opposite end of the board; its sign convention
# may differ (rotation around board long axis).  Verify with
# verify_orientation(cam_id=1) before trusting cam1 in flight.
#
# === Lens / FOV note ======================================================
# FOV cannot be read from camera registers -- it is a property of the
# LENS (M12 mount + plastic optic), not the OV5640 silicon.  The OV5640
# reports only its active pixel array dimensions.  Defaults below are
# from paper/flow_altitude.md (empirical measurement, 2026-04-21).
#
# Usage (normal injection):
#   import diag._t_flow_to_drone as f
#   f.run(secs=30, cam_id=0)
#
# Usage (override any default):
#   f.run(secs=10, cam_id=0,
#         fov_h_deg=70.0,           # different lens
#         body_xform={0: (+1.0, 0.0, 0.0, +1.0)},   # identity
#         cam_pos_xyz=(0.0, -0.02, 0.005),
#         apply_cam_pos=True)
#
# Usage (orientation calibration -- no drone send, just print):
#   f.verify_orientation(cam_id=0, secs=20)

import math
import sentai


# === Default config ===========================================================
# Override at run() time via kwargs; or mutate DEFAULTS in place if you want
# the change to stick across calls in the same REPL session.

DEFAULTS = {
    # --- sensor + lens geometry (read empirically; cannot be queried from regs)
    # Source: paper/flow_altitude.md.
    'fov_h_deg':       58.0,    # field of view along the long sensor axis (640 px)
    'fov_v_deg':       45.0,    # along the short axis (480 px)
    'grid_w':          80,      # flow grid width (post PXP step-8 from VGA)
    'grid_h':          60,      # flow grid height
    # focal length in grid pixels is derived: f = (grid_w / 2) / tan(fov_h/2)
    # ~72.3 grid-px for 58°/80, ~72.4 for 45°/60 -- consistent w/ a single
    # physical lens.  Recompute via _focal_px_grid(cfg) if you change FOV.

    # --- drone EKF geometry constants (from src/modules/src/kalman_core/mm_flow.c)
    # Don't change these unless you patched mm_flow.c on the drone side.
    'drone_npix':              35.0,        # PMW3901 pixel count along one axis
    'drone_thetapix_rad':      0.71674,     # 2 * sin(42°/2) -- 42° FOV
    'drone_flow_resolution':   0.10,        # FLOW_RESOLUTION constant: 0.10

    # --- image axis -> body frame mapping ------------------------------------
    # See header "Board + image-axis convention" for the derivation.
    # Encoding: (fw_from_dx, fw_from_dy, left_from_dx, left_from_dy).
    # Body component = fw_from_dx * dx + fw_from_dy * dy etc.
    #
    # cam0 baseline (vflip=1, USB-side camera, forward = cam0->cam1):
    #   body_fw   = +1.0 * dx                  (drone forward → +dx)
    #   body_left =                -1.0 * dy   (drone left    → -dy)
    #
    # cam1 NOT YET VERIFIED -- placeholder copy of cam0; run
    # verify_orientation(cam_id=1) before using cam1 in flight.
    'body_xform': {
        0: (+1.0,  0.0,   0.0, -1.0),   # BASELINE 2026-05-07 (cam0 + vflip=1)
        1: (+1.0,  0.0,   0.0, -1.0),   # placeholder; verify before flight
    },

    # --- camera position relative to drone centre of mass --------------------
    # Body frame, METRES, +x forward / +y left / +z up.
    # Drone EKF lever-arm correction:
    #   v_cam_bx_add = omegay * pos_z - omegaz * pos_y
    #   v_cam_by_add = omegaz * pos_x - omegax * pos_z
    # If camera is on the rotation axis (pos = 0,0,0), yaw rate produces
    # zero apparent flow -- safe but wrong if the camera is offset.
    'cam_pos_xyz': (0.0, 0.0, 0.0),
    'apply_cam_pos': False,  # opt-in: writes flowdeck.flowdeckPos_x/y/z
                             # via cflib (host-side), since the bridge
                             # has no PARAM SET path yet.

    # --- timing --------------------------------------------------------------
    'min_dt_s': 0.001,   # drone rejects dt <= 0; clamp small
    'max_dt_s': 0.20,    # absorb short flow stalls without huge snaps

    # --- confidence -> std mapping ------------------------------------------
    'std_floor':         1.0,   # at conf >= conf_thresh_high
    'std_low':           2.0,   # at conf >= conf_thresh_mid
    'std_mid':           4.0,   # at conf >= conf_thresh_low
    'std_ceil':          8.0,   # otherwise
    'conf_thresh_high': 200,
    'conf_thresh_mid':  128,
    'conf_thresh_low':   64,

    # --- bandwidth throttle --------------------------------------------------
    'send_every_n':  1,  # 1 = every flow frame; 2 = half rate, etc.

    # --- camera init -- applied at sentai.camera.init() at run() startup -----
    # Defaults reflect "raw sensor, no transformations" -- which is what we
    # want for flow experimentation so the body_xform alone determines axis
    # mapping, with no hidden ISP work in between.  Note: changing these
    # also affects ANY OTHER consumer of the camera (visualization, ML),
    # because the OV5640 register state is global.
    'cam_streaming':  1,        # 1 = streaming, 0 = trigger
    'cam_fps':        30,       # 15/30/45/60/90 from fsl_ov5640.c table
    'cam_hflip':      0,        # 0 = no mirror, 1 = mirror, -1 = leave OV5640 default (mirror ON)
    'cam_vflip':      1,        # BASELINE 2026-05-07: vflip ON for both cams.
                                # 0 = no flip, -1 = leave OV5640 default (flip OFF).

    # --- observability -------------------------------------------------------
    'log_altitude':  False,    # query drone altitude after run()
    'print_scale':   True,     # one-line print of derived scale factors at startup
}


def _focal_px_grid(cfg):
    """Focal length in grid-pixels, derived from FOV + grid dim.
    Returns (f_x, f_y).  Sanity-check: both should be similar for a
    correctly characterized lens.
    """
    fx = cfg['grid_w'] / (2.0 * math.tan(math.radians(cfg['fov_h_deg']) / 2.0))
    fy = cfg['grid_h'] / (2.0 * math.tan(math.radians(cfg['fov_v_deg']) / 2.0))
    return (fx, fy)


def _scale_to_drone_units(cfg):
    """Conversion factor: our grid-pixels -> drone-EKF dpixel units.

    The drone's EKF treats incoming dpixelx as PMW3901 pixels times 10
    (FLOW_RESOLUTION = 0.10).  PMW3901 pixel angular size is
    `drone_thetapix / drone_npix`; ours is `our_thetapix / our_grid`.
    For the EKF's predicted flow to match our measured flow at the same
    body velocity / altitude, we need:

        sent_dpixel * FLOW_RESOLUTION * (drone_thetapix / drone_npix)
        == our_grid_px * (our_thetapix / our_grid)

    Solving:

        sent_dpixel = our_grid_px * scale
        scale = (our_thetapix * drone_npix)
              / (our_grid * drone_flow_resolution * drone_thetapix)

    Returns (scale_x, scale_y) in drone_dpixel per our_grid_px.
    Typical values for SentAI defaults: ~6.18 and ~6.39.
    """
    Np = cfg['drone_npix']
    th = cfg['drone_thetapix_rad']
    fr = cfg['drone_flow_resolution']
    sx = (math.radians(cfg['fov_h_deg']) * Np) / (cfg['grid_w'] * fr * th)
    sy = (math.radians(cfg['fov_v_deg']) * Np) / (cfg['grid_h'] * fr * th)
    return (sx, sy)


def _conf_to_std(conf, cfg):
    if conf >= cfg['conf_thresh_high']:
        return cfg['std_floor']
    if conf >= cfg['conf_thresh_mid']:
        return cfg['std_low']
    if conf >= cfg['conf_thresh_low']:
        return cfg['std_mid']
    return cfg['std_ceil']


def _body_xy(d, cfg, scale_x, scale_y):
    """Apply image -> body transform, then geometry-correct scaling.

    Returns (dpx, dpy) in drone EKF dpixel units (PMW3901-equivalent
    after FLOW_RESOLUTION).
    """
    cam = d['cam_id']
    xform = cfg['body_xform'].get(cam, cfg['body_xform'][0])
    fw_dx, fw_dy, lf_dx, lf_dy = xform
    # Convert mgp -> grid-px first; scaling to drone units happens after
    # applying the body axis selection so each axis can use its own
    # angular pixel size (FOV_H vs FOV_V).
    dx_grid = d['dx'] / 1000.0
    dy_grid = d['dy'] / 1000.0
    # Pick which image axis becomes which body component, then scale
    # each component using the FOV that ITS image source has.
    fw_grid_from_dx   = fw_dx * dx_grid
    fw_grid_from_dy   = fw_dy * dy_grid
    left_grid_from_dx = lf_dx * dx_grid
    left_grid_from_dy = lf_dy * dy_grid
    body_fw   = fw_grid_from_dx * scale_x   + fw_grid_from_dy * scale_y
    body_left = left_grid_from_dx * scale_x + left_grid_from_dy * scale_y
    return (body_fw, body_left)


def _push_cam_pos(cfg):
    """The board-side bridge has only PARAM GET (CH=2) at the moment;
    PARAM SET would need a new opcode.  Until then the user must set
    flowdeck.flowdeckPos_{x,y,z} via cfclient on the host side."""
    x, y, z = cfg['cam_pos_xyz']
    print('cam_pos_xyz=(%.3f, %.3f, %.3f) m' % (x, y, z))
    print('  TODO: bridge has no PARAM SET yet --')
    print('  set flowdeck.flowdeckPos_x/y/z via cfclient on the host side.')


def _bringup(cam_id, cfg):
    sentai.crazy.init()
    rc = sentai.camera.init(cfg['cam_streaming'], cfg['cam_fps'],
                            cfg['cam_hflip'], cfg['cam_vflip'])
    if rc != 0:
        # rc=0 means already-initialized at same fps (still OK); other
        # negatives propagate as failure but don't abort -- the user may
        # be re-running and the existing camera state is fine.
        print('camera.init returned %d (already initialized at different '
              'orientation? sys.reset() to reapply)' % rc)
    sentai.flow.enable()
    sentai.flow.start(cam_id)
    sentai.rtos.sleep_ms(400)
    if cfg.get('apply_cam_pos'):
        _push_cam_pos(cfg)


def run(secs=30, cam_id=0, **kwargs):
    """Run the flow-to-drone publisher loop for `secs` seconds.

    Any DEFAULTS key may be overridden via kwargs.  Examples:

      f.run(10, 0, fov_h_deg=70)
      f.run(10, 0, body_xform={0: (-1.0, 0, 0, +1.0)})
      f.run(10, 0, drone_flow_resolution=1.0)   # if drone-side patched
    """
    cfg = dict(DEFAULTS)
    cfg.update(kwargs)
    if 'body_xform' in kwargs:
        # ensure both cam ids exist; fall back to identity if user only
        # provided one
        cfg['body_xform'].setdefault(0, (+1.0, 0.0, 0.0, +1.0))
        cfg['body_xform'].setdefault(1, (+1.0, 0.0, 0.0, +1.0))

    fx, fy = _focal_px_grid(cfg)
    scale_x, scale_y = _scale_to_drone_units(cfg)
    if cfg['print_scale']:
        print('focal_px_grid = (%.2f, %.2f)  scale_to_drone = (%.3f, %.3f)' %
              (fx, fy, scale_x, scale_y))

    _bringup(cam_id, cfg)

    sent = 0
    sent_ok = 0
    sent_fail = 0
    last_seq = 0
    last_t_ms = sentai.rtos.ticks_ms()
    t_end = sentai.rtos.ticks_ms() + secs * 1000

    d = None
    dpx = 0.0
    dpy = 0.0
    dt = 0.0
    std = 0.0

    while sentai.rtos.ticks_ms() < t_end:
        d = sentai.flow.read()
        seq = d['frame_seq']
        if seq == last_seq:
            sentai.rtos.sleep_ms(5)
            continue
        last_seq = seq

        if (seq % cfg['send_every_n']) != 0:
            continue

        now_ms = sentai.rtos.ticks_ms()
        dt = (now_ms - last_t_ms) / 1000.0
        last_t_ms = now_ms
        if dt <= cfg['min_dt_s']:
            dt = cfg['min_dt_s']
        if dt > cfg['max_dt_s']:
            dt = cfg['max_dt_s']

        dpx, dpy = _body_xy(d, cfg, scale_x, scale_y)
        std = _conf_to_std(d['confidence'], cfg)

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
    if cfg['log_altitude']:
        try:
            print('  drone altitude=%.3f m' % sentai.crazy.altitude())
        except Exception as e:
            print('  altitude query failed: %r' % e)
    return (sent, sent_ok, sent_fail)


def verify_orientation(cam_id=0, secs=20, period_ms=200):
    """Print raw flow dx/dy continuously so you can physically slide
    the board over a textured surface and confirm / re-derive the sign
    convention.

    Baseline 2026-05-07 expectation (cam0, vflip=1, board forward =
    cam0 -> cam1):

      slide board FORWARD  →  flow dx > 0   (features go to image RIGHT)
      slide board BACKWARD →  flow dx < 0
      slide board LEFT     →  flow dy < 0   (features go to image TOP)
      slide board RIGHT    →  flow dy > 0

    Confidence column matters: ignore conf<64 outliers, score sign
    only when conf>=128.

    If the observed signs disagree with the table above on this
    camera, build a corrected body_xform that makes:

        body_fw   POSITIVE on FORWARD motion
        body_left POSITIVE on LEFT motion

    and pass it to run() via the body_xform kwarg.

    Cam1 baseline is currently a placeholder (same as cam0).  Re-run
    verify_orientation(cam_id=1) to derive its true mapping before
    using cam1 in flight.
    """
    cfg = dict(DEFAULTS)
    sentai.camera.init(cfg['cam_streaming'], cfg['cam_fps'],
                       cfg['cam_hflip'], cfg['cam_vflip'])
    sentai.flow.enable()
    sentai.flow.start(cam_id)
    sentai.rtos.sleep_ms(400)
    print('=== verify_orientation cam_id=%d for %ds ===' % (cam_id, secs))
    print('Baseline (cam0, vflip=1, fw=cam0->cam1):')
    print('  forward  -> dx>0   left  -> dy<0')
    print('  backward -> dx<0   right -> dy>0')
    print('Score only when conf>=128; ignore conf<64.')
    print('')
    print('   t  | conf | dx_grid  dy_grid')
    print('------+------+------------------')
    t0 = sentai.rtos.ticks_ms()
    t_end = t0 + secs * 1000
    last_seq = 0
    while sentai.rtos.ticks_ms() < t_end:
        d = sentai.flow.read()
        seq = d['frame_seq']
        if seq == last_seq:
            sentai.rtos.sleep_ms(20)
            continue
        last_seq = seq
        dx_g = d['dx'] / 1000.0
        dy_g = d['dy'] / 1000.0
        elapsed = (sentai.rtos.ticks_ms() - t0) / 1000.0
        print('%5.1fs |  %3d | %+7.3f  %+7.3f' %
              (elapsed, d['confidence'], dx_g, dy_g))
        sentai.rtos.sleep_ms(period_ms)
    sentai.flow.stop()
    print('=== verify_orientation done ===')


# Bench harness: when streamed via _host_paste_bench.py, _target_fps is set
# but unused here; we just run a single 6 s pass and print '=== done ===' so
# the harness exits cleanly.
try:
    _ = _target_fps  # noqa: F821 - injected by _host_paste_bench
    run(secs=6, cam_id=0)
    print('=== done ===')
except NameError:
    pass
