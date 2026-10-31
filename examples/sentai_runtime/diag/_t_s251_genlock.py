# _t_s251_genlock.py — EXP-s251 iter2: dual-camera alternation, frame identity,
# frame AGE and ordering (OP-S10-W23-T9).
#
# Self-contained on-board driver per [[no-complex-serial-orchestration]]: the
# host pushes this file, execs it once, waits for "=== done ===" and keeps the
# "CSV|" lines.  No filesystem writes from here.
#
# ORACLE v2 — OV5640 ROLLING colour bars (0x503D = 0xC0) on both sensors;
# cam1 additionally runs the SDE "negative" effect (0x5580 bit 6):
#   column 40 is the white bar: cam0 -> B = 255 everywhere except the rolling
#   stripe (8 rows, B = 0); cam1 -> B = 0 everywhere except the stripe (255).
#   identity  = plateau level (255 -> cam0, 0 -> cam1)
#   stripe s  = first row of the inverted band; it moves 8 rows per sensor
#               frame (measured build 1650) -> an in-picture FRAME COUNTER:
#               for a camera seen at sensor_frames() = q, s must equal
#               (s_ref + 8*(q - q_ref)) mod 480.  lag = how many frames the
#               content is OLDER than that -> 0 = fresh, >= 1 = stale buffer.
#   mixed     = inverted rows not one contiguous band (two cameras / seam)
#   age_us    = now - FB-done time of the buffer (frame-complete hook)
# The s246 question "is the first frame after a MUX flip mislabelled, or is
# it an OLD frame?" is answered by lag and age_us directly.
import sentai

COL = 40
H = 480
STEP = 8            # stripe advance per sensor frame (rows)
NB_STATIC = 30
NB_ALT = 120


def _analyse(b):
    """b = 480-byte column.  Returns (cam, s, nstripe, mixed)."""
    hi = 0
    for i in range(H):
        if b[i] >= 128:
            hi += 1
    cam = 0 if hi > H // 2 else 1
    plateau = 255 if cam == 0 else 0
    inv = []
    for i in range(H):
        d = b[i] - plateau
        if d < 0:
            d = -d
        if d > 100:
            inv.append(i)
    n = len(inv)
    if n == 0:
        return cam, -1, 0, False
    # contiguous band allowing wrap at 479 -> 0
    s = inv[0]
    contiguous = True
    for k in range(1, n):
        if inv[k] != inv[k - 1] + 1:
            contiguous = False
            break
    if not contiguous and inv[0] == 0 and inv[-1] == H - 1:
        # wrapped band: find the gap and take the row after it as the start
        gap = None
        for k in range(1, n):
            if inv[k] != inv[k - 1] + 1:
                if gap is None:
                    gap = k
                else:
                    gap = -1      # second gap -> really not contiguous
                    break
        if gap is not None and gap > 0:
            s = inv[gap]
            contiguous = True
    mixed = (not contiguous) or n > 24
    return cam, s, n, mixed


class _Ref:
    """per-camera stripe reference (s_ref at sensor frame q_ref)."""
    def __init__(self):
        self.s = [None, None]
        self.q = [None, None]

    def lag(self, cam, s, q):
        if s < 0:
            return -1
        if self.s[cam] is None:
            self.s[cam] = s
            self.q[cam] = q
            return 0
        exp = (self.s[cam] + STEP * (q - self.q[cam])) % H
        # how many frames behind expectation (positive = older content)
        d = (exp - s) % H
        return (d + STEP // 2) // STEP


def _collect(nframes, ref, budget_ms=0):
    """Sample distinct sensor frames (dedup on sensor_frames()), time budget.
    Returns records (q, ms, cam, s, nstripe, mixed, lag, age_us, tag_task,
    tag_isr, idx), span, wall, fails.  See s246 for the guard/sleep rationale."""
    if budget_ms <= 0:
        budget_ms = 1500 + nframes * 60
    recs = []
    fails = 0
    t0 = sentai.rtos.ticks_ms()
    q0 = sentai.camera.sensor_frames()
    last = q0
    while len(recs) < nframes:
        if (sentai.rtos.ticks_ms() - t0) > budget_ms:
            break
        q = sentai.camera.sensor_frames()
        if q == last:
            sentai.rtos.sleep_ms(1)
            continue
        last = q
        try:
            tg_task, tg_isr, b, idx, ts, now = sentai.camera.peek_col(COL)
        except Exception:
            fails += 1
            if fails > 20:
                break
            sentai.rtos.sleep_ms(2)
            continue
        ms = sentai.rtos.ticks_ms() - t0
        cam, s, n, mixed = _analyse(b)
        lag = ref.lag(cam, s, q) if not mixed else -1
        age = now - ts
        if age < 0:
            age += 4294967296
        recs.append((q, ms, cam, s, n, 1 if mixed else 0, lag, age, tg_task, tg_isr, idx))
        if (len(recs) % 20) == 0:
            sentai.rtos.repl_kick()
    wall = sentai.rtos.ticks_ms() - t0
    return recs, (last - q0), wall, fails


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return None
    return s[n // 2]


def _summary(tag, recs, span, wall, fails):
    n = len(recs)
    fps = (n * 1000.0 / wall) if wall > 0 else 0.0
    c0 = sum(1 for r in recs if r[2] == 0)
    c1 = n - c0
    mixed = sum(1 for r in recs if r[5])
    mism_task = sum(1 for r in recs if r[8] in (0, 1) and r[8] != r[2])
    mism_isr = sum(1 for r in recs if r[9] in (0, 1) and r[9] != r[2])
    lag0 = sum(1 for r in recs if r[6] == 0)
    lag1 = sum(1 for r in recs if r[6] == 1)
    lag2 = sum(1 for r in recs if r[6] >= 2)
    ages = [r[7] for r in recs]
    med_age = _median(ages)
    max_age = max(ages) if ages else None
    # stale frames among the mislabelled ones: is the wrong tag an OLD frame?
    mism_stale = sum(1 for r in recs if r[8] in (0, 1) and r[8] != r[2] and r[6] >= 1)
    gaps = [recs[i][1] - recs[i - 1][1] for i in range(1, n)]
    print("  %s: n=%d span=%d fps=%.1f cam0=%d cam1=%d mixed=%d mism_task=%d "
          "mism_isr=%d (stale %d) lag0=%d lag1=%d lag2+=%d age_us med=%s max=%s "
          "gap_ms=%s fails=%d"
          % (tag, n, span, fps, c0, c1, mixed, mism_task, mism_isr, mism_stale,
             lag0, lag1, lag2, med_age, max_age, _median(gaps), fails))
    print("CSV|" + ",".join(str(x) for x in (
        "S", tag, n, span, "%.1f" % fps, c0, c1, mixed, mism_task, mism_isr,
        mism_stale, lag0, lag1, lag2, med_age, max_age, _median(gaps), fails)))


def _dump(tag, recs, limit=200):
    for r in recs[:limit]:
        print("CSV|" + ",".join(str(x) for x in ("F", tag) + r))


def _flog(tag):
    """Dump the FB-done log and return the inter-sensor phase estimate.

    At ratio(1,1) the MUX owner changes every 2 frames.  The FB-done interval
    of the first frame after a flip TO cam1 is T + phi, after a flip TO cam0
    it is T - phi, where phi = cam1 frame start relative to cam0 (us).
    Returns (phi_us, T_us, n_flips)."""
    log = sentai.camera.frame_log(64)
    prev = None
    prev_cam = None
    to1 = []
    to0 = []
    steady = []
    for (ts, idx, cam, seq) in log:
        dt = (ts - prev) if prev is not None else 0
        if dt < 0:
            dt += 4294967296          # GPT1 wrap (rare); keeps dt a small int
        print("CSV|L,%s,%d,%d,%d,%d,%d" % (tag, ts, dt, idx, cam, seq))
        if prev is not None and dt < 60000:
            if prev_cam is not None and cam != prev_cam:
                (to1 if cam == 1 else to0).append(dt)
            else:
                steady.append(dt)
        prev = ts
        prev_cam = cam
    T = _median(steady)
    phi = None
    if to1 and T is not None:
        # first frame after a flip TO cam1 completes T + phi later (phi > 0:
        # cam1 starts later than cam0).  When the CSI misses the new
        # sensor's SOF it waits a whole frame, so reduce modulo T and map to
        # (-T/2, T/2].  Cross-check with the cam0 side: its excess is -phi.
        e1 = (_median(to1) - T) % T
        phi = e1 if e1 <= T // 2 else e1 - T
        if to0:
            e0 = (_median(to0) - T) % T
            phi0 = -(e0 if e0 <= T // 2 else e0 - T)
            if abs(phi0 - phi) > 300:
                print("  (phase cross-check disagrees: from cam1 flips %s us, from cam0 flips %s us)" % (phi, phi0))
    print("  flip timing %s: T=%s us, dt->cam1=%s dt->cam0=%s  => phi(cam1-cam0)=%s us (n=%d/%d)"
          % (tag, T, _median(to1), _median(to0), phi, len(to1), len(to0)))
    print("CSV|P,%s,%s,%s,%s,%s" % (tag, T, _median(to1), _median(to0), phi))
    return phi, T, len(to1) + len(to0)


print("=== boot ===")
print("version=", sentai.version())
sentai.camera.init(1, 45)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(0)
sentai.camera.dirty_skip_n(0)
sentai.rtos.sleep_ms(500)

# ---- patterns: both 0xC0 rolling bars; cam1 + SDE negative ------------------
sentai.camera.reg_write(0, 0x503D, 0xC0)
sentai.camera.reg_write(1, 0x503D, 0xC0)
_e = sentai.camera.reg_read(1, 0x5001)
_v = sentai.camera.reg_read(1, 0x5580) & ~0x40
sentai.camera.reg_write(1, 0x5001, _e | 0x80)
sentai.camera.reg_write(1, 0x5580, _v | 0x40)
sentai.rtos.sleep_ms(400)

print("--- PHASE A: static reference, each camera ---")
for _c in (0, 1):
    sentai.camera.select(_c)
    sentai.rtos.sleep_ms(300)
    ref = _Ref()
    recs, span, wall, fails = _collect(NB_STATIC, ref)
    _summary("A_cam%d" % _c, recs, span, wall, fails)
    _dump("A%d" % _c, recs, 40)

print("--- PHASE B: ratio(1,1) drain=0 dirty=0 ---")
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.rtos.sleep_ms(200)
sentai.camera.switch_drain(0)
sentai.camera.dirty_skip_n(0)
sentai.camera.ratio(1, 1)
sentai.rtos.sleep_ms(300)
ref = _Ref()
recs, span, wall, fails = _collect(NB_ALT, ref)
phiB, TB, _n = _flog("B")
_summary("B_d0_y0", recs, span, wall, fails)
_dump("B", recs)

print("--- PHASE B1: ratio(1,1) drain=0 dirty=1 (production) ---")
sentai.camera.ratio(0, 0)
sentai.rtos.sleep_ms(200)
sentai.camera.dirty_skip_n(1)
sentai.camera.ratio(1, 1)
sentai.rtos.sleep_ms(300)
ref = _Ref()
recs, span, wall, fails = _collect(NB_ALT, ref)
_flog("B1")
_summary("B1_d0_y1", recs, span, wall, fails)
_dump("B1", recs, 80)

def _regs(cam):
    rd = sentai.camera.reg_read
    return ((rd(cam, 0x380E) << 8) | rd(cam, 0x380F), (rd(cam, 0x380C) << 8) | rd(cam, 0x380D))


def _set_vts(cam, vts):
    sentai.camera.reg_write(cam, 0x380E, (vts >> 8) & 0xFF)
    sentai.camera.reg_write(cam, 0x380F, vts & 0xFF)


def _probe_phase(tag):
    """Alternate 1:1 WHILE CONSUMING frames, then read the FB-done log.

    The frame-complete hook only runs when the NXP driver queues a frame; with
    nobody dequeuing, the 3-deep queue fills after ~3 frames and the log
    freezes (iter4/iter5: every probe returned the same 64 entries).  So the
    probe must drain frames like the B phase does."""
    sentai.camera.ratio(0, 0)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(200)
    sentai.camera.switch_drain(0)
    sentai.camera.dirty_skip_n(0)
    sentai.camera.ratio(1, 1)
    sentai.rtos.sleep_ms(300)
    _recs, _span, _wall, _fails = _collect(70, _Ref())
    phi, T, n = _flog(tag)
    sentai.camera.ratio(0, 0)
    sentai.rtos.sleep_ms(200)
    return phi, T


print("--- PHASE C: genlock -- bring the two sensors IN PHASE (phi -> 0) ---")
# Both sensors share the 24 MHz XCLK, so a one-time phase shift holds for the
# whole session.  Geometry (iter3e): a MUX flip at FB-done of the old sensor
# is clean only if the new sensor's frame start lies inside the old one's
# vertical blanking.  With phi = cam1 - cam0 frame-start offset and
# VBLANK = (VTS - 960) lines ~ 543 us, the margins are VBLANK - phi - delta
# (flip to cam0) and VBLANK + phi - delta (flip to cam1): both are maximal
# at phi = 0.  Shift = run one sensor with VTS + 1 for N frames -> it moves
# N lines later; delaying cam0 lowers phi, delaying cam1 raises it.
VTS0, HTS0 = _regs(0)
VTS1, HTS1 = _regs(1)
print("  cam0 VTS=%d HTS=%d  cam1 VTS=%d HTS=%d" % (VTS0, HTS0, VTS1, HTS1))
line_ns = (TB * 1000) // VTS0 if (TB and VTS0) else 22640   # one sensor line, ns
frame_ms = (TB // 1000) if TB else 22
tol_us = 60
phi = phiB
for attempt in range(4):
    if phi is None:
        print("  no phase estimate -- stop")
        break
    if abs(phi) < tol_us:
        print("  in phase: phi=%s us (|phi| < %d us) after %d shift(s)" % (phi, tol_us, attempt))
        break
    nlines = (abs(phi) * 1000 + line_ns // 2) // line_ns
    if nlines == 0:
        break
    if nlines > 200:
        nlines = 200                   # keep a single tweak well below one frame (984 lines)
    cam = 0 if phi > 0 else 1          # phi > 0: cam1 is late -> delay cam0 to meet it
    vts = VTS0 if cam == 0 else VTS1
    print("  attempt %d: phi=%s us -> delay cam%d by %d lines (VTS %d+%d for one frame)" % (attempt, phi, cam, nlines, vts, nlines))
    _set_vts(cam, vts + nlines)
    sentai.rtos.sleep_ms(frame_ms + frame_ms // 2)     # ~1.5 frames: exactly one long frame
    _set_vts(cam, vts)
    sentai.rtos.sleep_ms(100)
    phi, T = _probe_phase("C%d" % attempt)
    print("CSV|C,%d,%d,%d,%s" % (attempt, cam, nlines, phi))

print("--- PHASE D: ratio(1,1) drain=0 dirty=0 after genlock ---")
sentai.camera.select(0)
sentai.rtos.sleep_ms(200)
sentai.camera.switch_drain(0)
sentai.camera.dirty_skip_n(0)
sentai.camera.ratio(1, 1)
sentai.rtos.sleep_ms(300)
ref = _Ref()
recs, span, wall, fails = _collect(NB_ALT, ref)
_flog("D")
_summary("D_d0_y0", recs, span, wall, fails)
_dump("D", recs)

print("--- PHASE E: idle 1 s, then the first grabs (stale-frame check) ---")
sentai.camera.ratio(0, 0)
sentai.camera.dirty_skip_n(0)
sentai.camera.select(0)
sentai.rtos.sleep_ms(1000)
for k in range(5):
    tg_task, tg_isr, b, idx, ts, now = sentai.camera.peek_col(COL)
    cam, s, n, mixed = _analyse(b)
    age = now - ts
    if age < 0:
        age += 4294967296
    print("CSV|E,%d,%d,%d,%d,%d,%d,%d" % (k, cam, s, age, idx, tg_task, tg_isr))

# ---- restore ---------------------------------------------------------------
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.camera.dirty_skip_n(1)
sentai.camera.reg_write(1, 0x5580, _v)
sentai.camera.reg_write(0, 0x503D, 0x00)
sentai.camera.reg_write(1, 0x503D, 0x00)
sentai.camera.select(0)
print("=== done ===")
