# _t_s246_alt.py — EXP-s246: dual-camera alternation at FULL sensor rate,
# with no lost frames and no frame mixed from both cameras.
#
# Self-contained on-board driver per [[no-complex-serial-orchestration]]:
# the whole sweep lives here, the host only pushes this file, execs it once,
# waits for "=== done ===" and downloads the CSV.  No host-side branching,
# no seeded globals.
#
# ORACLE (house method, from _t_pattern_31.py + s235/iter10):
#   cam0 <- BARS  (0x503D=0x80): col-40 B byte ~ 0xFF (white bar)
#   cam1 <- HBAND (0x503D=0x88): col-40 B byte ~ 0x1F (dark band)
#   peek5_b40() does a SINGLE dequeue and samples the B channel at column 40
#   from rows {0, H/4, H/2, 3H/4, H-1} of THAT ONE buffer, plus its cam tag.
#   Sampling several rows of one buffer is what detects a horizontal seam --
#   a frame cut across two cameras.  (Calling peek_row_at five times instead
#   dequeues five DIFFERENT buffers and manufactures false SCRAMBLED, which
#   is the documented bug _t_pattern_31.py was written to fix.)
#
# WHAT WE MEASURE, per config:
#   fps      - distinct sensor frames actually observed per second
#   SCRAMBLED- frames mixed from both cameras   (must be 0)
#   mismatch - buffer tag disagrees with content (must be 0)
#   balance  - cam0 vs cam1 share at ratio(1,1)  (must be ~50/50)
#
# KNOBS SWEPT (the suspects identified in s245 for the missing 2x):
#   switch_drain(n)  - post-flip frames to wait before a frame is "clean"
#   dirty_skip_n(n)  - FB-done buffers marked dirty after each flip
import sentai

NB_BASE = 40          # frames per single-cam baseline
NB_SWEEP = 120        # frames per alternation config
DRAINS = (0, 1, 2)
DIRTYS = (0, 1, 2)


def _classify(b40):
    if b40 >= 0xA0: return "BARS"
    if b40 <= 0x80: return "HBAND"
    return "AMBIG"


def _classify_5b(s5):
    """s5 = (b0..b4) from ONE buffer -> BARS | HBAND | AMBIG | SCRAMBLED."""
    c = tuple(_classify(b) for b in s5)
    if all(x == "BARS" for x in c):  return "BARS"
    if all(x == "HBAND" for x in c): return "HBAND"
    if all(x == "AMBIG" for x in c): return "AMBIG"
    return "SCRAMBLED"


def _collect(nframes, budget_ms=0):
    """Sample distinct sensor frames, deduped by sensor_frames().

    Dedup is mandatory: without it one sensor frame is logged 2-3x and every
    rate inflates ([[dedup-log-by-seq]]).  We dedup on sensor_frames(), NOT
    frame_count() -- the latter is fb2-gated and ticks at half the sensor
    rate (PHASE0), so it cannot see every frame and cannot detect a drop.

    The guard is a TIME budget, not an iteration count.  An iteration count
    is the bug that made the first run of this driver return n=0 for every
    config: polling the counter costs microseconds, so a few thousand spins
    elapse long before one 22 ms frame period does, and the loop gave up
    before a single frame arrived -- then reported "0 scrambled" as CLEAN.

    Returns (records, sensor_span, wall_ms), records = [(seq, tag, cls)].
    """
    if budget_ms <= 0:
        budget_ms = 400 + nframes * 40     # generous: ~22 ms/frame expected
    recs = []
    fails = 0
    t0 = sentai.rtos.ticks_ms()
    seq0 = sentai.camera.sensor_frames()
    last = seq0
    while len(recs) < nframes:
        if (sentai.rtos.ticks_ms() - t0) > budget_ms:
            break
        cur = sentai.camera.sensor_frames()
        if cur == last:
            # MUST yield.  A bare `continue` here spins MicroPython flat out
            # and starves the other tasks -- including the watchdog kicker and
            # the USB stack.  That reset the board mid-run (USB re-enumerated,
            # host saw "device reports readiness to read but returned no
            # data").  1 ms against a ~22 ms frame period costs no resolution.
            sentai.rtos.sleep_ms(1)
            continue                       # same sensor frame -> skip
        last = cur
        # peek5_b40 raises when the grab fails outright -- with dirty_skip_n
        # high enough every queued buffer is marked dirty, the recovery loop
        # runs out, and the C side returns an error.  That is a legitimate
        # measurement outcome for that config, not a reason to kill the whole
        # sweep, so count it and move on.  (An uncaught raise here aborted the
        # first full run at drain=0/dirty=2.)
        try:
            # (idx, tag_task, tag_isr, tag_csi, b0..b4) -- score content against
            # EVERY tag source, not just the one the consumer happens to use.
            r = sentai.camera.peek5_tags()
        except Exception:
            fails += 1
            if fails > 20:
                break                      # this config cannot deliver frames
            sentai.rtos.sleep_ms(2)
            continue
        recs.append((cur, r[1], r[2], r[3], _classify_5b(r[4:9])))
        if (len(recs) % 20) == 0:
            sentai.rtos.repl_kick()
    wall = sentai.rtos.ticks_ms() - t0
    return recs, (last - seq0), wall, fails


def _bad(tag, cls):
    """True when this tag contradicts the content."""
    return (tag == 0 and cls == "HBAND") or (tag == 1 and cls == "BARS")


def _row(phase, ka, kb, fps, n, span, t, tag0, tag1,
         mism, mism_isr, mism_csi, csi_unt, fails, lag1=0, lead1=0):
    """Build one CSV line by joining fields, never by a positional format
    string -- an arity slip there aborted the first iter4 run at PHASE A
    ("format string didn't convert all arguments")."""
    return ",".join([phase, ka, kb, "%.2f" % fps] +
                    [str(x) for x in (n, span, t["BARS"], t["HBAND"],
                                      t["AMBIG"], t["SCRAMBLED"], tag0, tag1,
                                      mism, mism_isr, mism_csi, csi_unt,
                                      fails, lag1, lead1)])


def _tally(recs):
    t = {"BARS": 0, "HBAND": 0, "AMBIG": 0, "SCRAMBLED": 0}
    tag0 = tag1 = 0
    mism_task = mism_isr = mism_csi = 0
    csi_untagged = 0
    for (_s, tg_task, tg_isr, tg_csi, cls) in recs:
        t[cls] += 1
        if tg_task == 0: tag0 += 1
        elif tg_task == 1: tag1 += 1
        if _bad(tg_task, cls): mism_task += 1
        if _bad(tg_isr, cls):  mism_isr += 1
        if tg_csi == 0xFF:     csi_untagged += 1
        elif _bad(tg_csi, cls): mism_csi += 1
    # LAG/LEAD scoring -- the actual hypothesis under test.
    #
    # At drain=0/dirty=0 the tags alternate exactly 60/60 while the CONTENT
    # does not (46/73), and frames are clean (1/120 scrambled).  A clean image
    # carrying the wrong label is what you get when the first buffer after a
    # MUX flip still holds the PREVIOUS camera's frame -- it was already in
    # flight through the CSI pipeline when the MUX moved.
    #
    # If that is the mechanism, the fix is to DELAY the tag by one frame, not
    # to discard the frame: we would keep full rate AND label correctly.  So
    # score content against the neighbouring frames' tags as well.  A near-zero
    # lag-1 score confirms it; if lag and lead are both as bad as lag-0, the
    # tag is not merely shifted and something else is wrong.
    mism_lag1 = mism_lead1 = 0
    for i in range(len(recs)):
        cls = recs[i][4]
        if i > 0 and _bad(recs[i - 1][1], cls):
            mism_lag1 += 1
        if i + 1 < len(recs) and _bad(recs[i + 1][1], cls):
            mism_lead1 += 1
    return (t, tag0, tag1, mism_task, mism_isr, mism_csi, csi_untagged,
            mism_lag1, mism_lead1)


print("=== boot ===")
print("version=", sentai.version())

sentai.camera.init(1, 45)
sentai.camera.ratio(0, 0)
sentai.rtos.sleep_ms(500)
print("fps_configured=", sentai.camera.fps())

print("--- pattern init: cam0=BARS, cam1=HBAND ---")
sentai.camera.test_pattern(0, 1)
sentai.camera.test_pattern(1, 2)
sentai.camera.select(0); sentai.rtos.sleep_ms(400)
sentai.camera.select(1); sentai.rtos.sleep_ms(400)

rows = []          # csv rows
notes = []

# ---- PHASE 0: what does frame_count() actually count? -----------------
# The ISR comment says it is gated on FB2-done and matches the sensor rate;
# auto-memory [[csi-isr-is-timing-sensitive]] says it ticks at HALF the
# sensor rate.  Any "full speed" claim depends on which is true, so measure
# it against the wall clock at a known 45 fps before anything else.
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)
t0 = sentai.rtos.ticks_ms()
f0 = sentai.camera.frame_count()
n0 = sentai.camera.sensor_frames()
sentai.rtos.sleep_ms(2000)
f1 = sentai.camera.frame_count()
n1 = sentai.camera.sensor_frames()
t1 = sentai.rtos.ticks_ms()
dt = max(1, t1 - t0)
fc_hz = (f1 - f0) * 1000.0 / dt
sf_hz = (n1 - n0) * 1000.0 / dt
print("PHASE0 frame_count=%.1f Hz  sensor_frames=%.1f Hz  over %d ms "
      "(sensor configured 45)" % (fc_hz, sf_hz, dt))
notes.append("frame_count=%.0f Hz, sensor_frames=%.0f Hz (configured 45)"
             % (fc_hz, sf_hz))
if sf_hz > 1.5 * fc_hz:
    notes.append("CONFIRMED: frame_count is fb2-gated at HALF rate; "
                 "ratio() keys off it, so ratio(1,1) flips per TWO frames")
rows.append("phase0,frame_count,sensor_frames,%.2f,%.2f" % (fc_hz, sf_hz)
            + "," * 14)

# ---- PHASE A: single-cam baseline -- validate the oracle itself -------
# If the oracle cannot classify a STATIC single camera at 100%, nothing it
# says about alternation can be trusted.
print("--- PHASE A: single-cam baseline ---")
for sel, want in ((0, "BARS"), (1, "HBAND")):
    sentai.camera.select(sel)
    sentai.rtos.sleep_ms(400)
    recs, span, wall, fails = _collect(NB_BASE)
    (t, tag0, tag1, mism, mism_isr, mism_csi, csi_unt,
     mism_lag1, mism_lead1) = _tally(recs)
    fps = len(recs) * 1000.0 / max(1, wall)
    ok = t[want]
    if not recs:
        notes.append("baseline sel=%d observed NO frames - oracle not running" % sel)
    print("  sel=%d n=%d fps=%.1f %s=%d scrambled=%d ambig=%d mismatch=%d"
          % (sel, len(recs), fps, want, ok, t["SCRAMBLED"], t["AMBIG"], mism))
    rows.append(_row("baseline", "sel%d" % sel, "", fps, len(recs),
                     span, t, tag0, tag1, mism, mism_isr, mism_csi,
                     csi_unt, 0, mism_lag1, mism_lead1))
    if ok < len(recs):
        notes.append("baseline sel=%d classified only %d/%d as %s"
                     % (sel, ok, len(recs), want))

# ---- PHASE B: alternation sweep --------------------------------------
print("--- PHASE B: ratio(1,1) sweep drain x dirty ---")
best = None
for dr in DRAINS:
    for dy in DIRTYS:
        sentai.camera.ratio(0, 0)
        sentai.camera.select(0)
        sentai.rtos.sleep_ms(200)
        sentai.camera.switch_drain(dr)
        sentai.camera.dirty_skip_n(dy)
        sentai.camera.ratio(1, 1)
        sentai.rtos.sleep_ms(300)

        recs, span, wall, fails = _collect(NB_SWEEP)
        (t, tag0, tag1, mism, mism_isr, mism_csi, csi_unt,
         mism_lag1, mism_lead1) = _tally(recs)
        fps = len(recs) * 1000.0 / max(1, wall)
        # n == 0 means the oracle never ran; calling that "clean" is the
        # trap this driver already fell into once.
        valid = (len(recs) >= NB_SWEEP // 2)
        clean = valid and t["SCRAMBLED"] == 0 and mism == 0
        clean_isr = valid and t["SCRAMBLED"] == 0 and mism_isr == 0
        dropped = span - len(recs)
        print("  drain=%d dirty=%d fps=%.1f bars=%d hband=%d ambig=%d "
              "SCRAM=%d mism_task=%d mism_isr=%d LAG1=%d LEAD1=%d "
              "tag0=%d tag1=%d %s"
              % (dr, dy, fps, t["BARS"], t["HBAND"], t["AMBIG"],
                 t["SCRAMBLED"], mism, mism_isr, mism_lag1, mism_lead1, tag0, tag1,
                 ("CLEAN dropped=%d" % dropped) if clean
                 else ("INVALID n=%d grabfail=%d" % (len(recs), fails))
                 if not valid else ("grabfail=%d" % fails)))
        rows.append(_row("sweep", "drain%d" % dr, "dirty%d" % dy, fps,
                         len(recs), span, t, tag0, tag1, mism,
                         mism_isr, mism_csi, csi_unt, fails,
                         mism_lag1, mism_lead1))
        if clean and (best is None or fps > best[0]):
            best = (fps, dr, dy)
        sentai.rtos.repl_kick()

# ---- restore camera state before reporting ---------------------------
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.camera.dirty_skip_n(1)
sentai.camera.select(0)
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)

print("--- summary ---")
for n in notes:
    print("  note:", n)
if best:
    print("  BEST CLEAN: fps=%.1f at drain=%d dirty=%d" % best)
else:
    print("  BEST CLEAN: none -- every config lost or mixed frames")

# NAND is deliberately NOT touched by this driver (operator directive
# 2026-09-04).  Everything above is bookkept in RAM and the results are
# emitted on stdout with a "CSV|" prefix; the host reconstructs the file.
#
# Why: FxUserMaybeIdleSync() runs from CombinedWatchdogTask every 5 s and,
# whenever an unflushed write is older than 2 s, performs a SYNCHRONOUS
# FxUserSync() costing 200-500 ms ([[idle-autosync-brownout]]).  A NAND
# program of that length landing inside a sustained 45 fps dual-camera
# capture is a prime suspect for the mid-sweep resets, so this run removes
# the filesystem from the experiment entirely rather than reasoning about it.
hdr = ("phase,knob_a,knob_b,fps,n_frames,seq_span,bars,hband,"
       "ambig,scrambled,tag0,tag1,mism_task,mism_isr,mism_csi,"
       "csi_untagged,grabfail,mism_lag1,mism_lead1")
print("CSV|" + hdr)
for _r in rows:
    print("CSV|" + _r)

print("=== done ===")
