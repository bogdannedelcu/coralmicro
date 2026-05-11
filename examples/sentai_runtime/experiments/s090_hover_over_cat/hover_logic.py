# hover_logic.py — runs the hover-over loop inside sentai_sim.
#
# Loaded from REPL via:  import hover_logic
# (resolved through sim_fs_resolve() in main_sim.c → MP lexer streams
# this file from the SIM virtual FS, no source-string heap alloc.)
#
# Emits one "STATE=" line per iteration with the parsed pose + chosen
# control output, which the host-side Python wrapper reads and forwards
# to cflib.  The drone takeoff/landing is owned by the wrapper.
import sentai

# Note: MobileNet V2 COCO17 oscillates between classes when shown the
# cat picture (bear/person/dog/cat — model isn't sure).  Loosen filter:
# accept any CONFIRMED track above MIN_CONF; the cat picture is the only
# salient object in the scene so whatever wins is what we hover over.
TARGET_CLASS  = None      # None = accept any class
IMG_W = 300
IMG_H = 300
MIN_CONF      = 300       # permille
GAIN          = 4         # pixel error * GAIN / 1000 = m/s
V_MAX         = 200       # mm/s
RATE_MS       = 200       # 5 Hz
N_ITER        = 500       # 500*200ms = 100s — enough headroom for slow staged climb
COAST_FRAMES  = 15        # ~3s @ 5Hz — keep moving toward last bbox after detection drops
CENTER_THRESH = 20        # px — within ±20 of image centre = "centered"
CENTERED_HOLD = 15        # frames consecutively centered to declare HOVER_CENTERED

def clamp(v, lo, hi):
    return lo if v<lo else hi if v>hi else v

print("HOVER_LOGIC_READY")
last_diag = -10
last_flow_seq = -1
# Coast + centered-hold state
last_cx = last_cy = None       # last seen bbox centroid
last_vx = last_vy = 0           # last commanded velocity (for coast)
coast_left = 0                  # frames remaining of coast-after-loss
centered_count = 0              # consecutive frames within CENTER_THRESH
for i in range(N_ITER):
    tracks = sentai.pipeline.tracker_tracks()
    n_tracks = len(tracks)
    f = sentai.flow.read()  # (seq, dx_q, dy_q, conf, lat, dz_q, dz_conf)
    flow_fresh = (len(f) >= 7 and f[0] != last_flow_seq)
    if flow_fresh:
        last_flow_seq = f[0]
    fvx = f[1] if flow_fresh else 0
    fvy = f[2] if flow_fresh else 0
    best = None
    for t in tracks:
        if len(t) < 10: continue
        if TARGET_CLASS is not None and t[1] != TARGET_CLASS: continue
        if t[7] != 1: continue
        if t[6] < MIN_CONF: continue
        if best is None or t[6] > best[6]:
            best = t
    if best is not None:
        cx = (best[2] + best[4]) // 2
        cy = (best[3] + best[5]) // 2
        err_x = cx - IMG_W // 2
        err_y = cy - IMG_H // 2
        # Sign convention (empirically verified — cf2 SITL inverts vy):
        vx = clamp( err_y * GAIN, -V_MAX, V_MAX)
        vy = clamp(-err_x * GAIN, -V_MAX, V_MAX)
        # Memorise for coast + center-hold logic.
        last_cx, last_cy = cx, cy
        last_vx, last_vy = vx, vy
        coast_left = COAST_FRAMES
        # Track centered-hold: error magnitude within threshold on BOTH axes.
        if abs(err_x) <= CENTER_THRESH and abs(err_y) <= CENTER_THRESH:
            centered_count += 1
            if centered_count == CENTERED_HOLD:
                print("HOVER_CENTERED iter={} cxy=({},{}) err=({},{})".format(
                    i, cx, cy, err_x, err_y))
        else:
            centered_count = 0
        print("STATE=", (i, best[0], best[1], best[6], cx, cy, err_x, err_y,
                          vx, vy, fvx, fvy, best[2], best[3], best[4], best[5]))
    elif coast_left > 0 and last_cx is not None:
        # Coast — detection dropped, keep commanding toward last known cx/cy
        # for COAST_FRAMES ticks.  Don't increment centered_count (need a
        # fresh observation to confirm we ARE there).
        coast_left -= 1
        # Re-compute err from last memorised bbox so the command tracks
        # any motion the drone has done since.
        err_x = last_cx - IMG_W // 2
        err_y = last_cy - IMG_H // 2
        # Decay velocity a bit each tick so we don't blast through target.
        decay = float(coast_left) / COAST_FRAMES
        vx = int(last_vx * decay)
        vy = int(last_vy * decay)
        print("STATE=", (i, -1, -1, 0, last_cx, last_cy, err_x, err_y,
                          vx, vy, fvx, fvy, 0, 0, 0, 0))
        if coast_left == 0:
            print("HOVER_COAST_END iter={} (detection lost, coast expired)".format(i))
    else:
        # No detection, no coast → stationary, no command.
        centered_count = 0
        print("STATE=", (i, 0, 0, 0, 0, 0, 0, 0, 0, 0, fvx, fvy, 0, 0, 0, 0))
        # Diagnostic every 1s — when no target, dump tracks + raw SSD
        # detections so the host log shows whether SSD sees anything at
        # all and which filter rejects it (class / state / conf).
        if i - last_diag >= 5:
            last_diag = i
            stats = sentai.pipeline.stats()
            raw_dets = sentai.pipeline.detections(100)
            print("HOVER_LOGIC_DIAG iter={} n_tracks={} stats={}".format(
                i, n_tracks, stats))
            if n_tracks:
                print("HOVER_LOGIC_TRACKS=", tracks)
            if raw_dets:
                print("HOVER_LOGIC_DETS=", raw_dets)
    sentai.rtos.sleep_ms(RATE_MS)
print("HOVER_LOGIC_DONE")
