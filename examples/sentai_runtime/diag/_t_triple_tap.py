# _t_triple_tap.py -- IMU double-tap detector with non-blocking LED + train().
#
# Pure-REPL software detector on top of sentai.imu.read() (~195 Hz useful).
# LIS2DU12 has hardware double-tap; if this software path proves too laggy
# we promote it to a C++ FreeRTOS task that consumes TAP_SRC.double_tap_ia
# (see examples/camera_streaming_http/camera_streaming_http.cc:AccelTask).
#
# Workflow:
#   import diag._t_triple_tap as tt
#   tt.train()                    # tap 5x at the strength you want to use
#   tt.run()                      # 60s, LED-only
#   tt.run(spin_motors=True)      # also spin Crazyflie motors
#
# Latency notes (tuned 2026-05-05):
#   * No print() during detection -- only on trigger + summary. Per-tap
#     prints over USB-CDC stretch the loop and dominate visible latency.
#   * LED is non-blocking: turn ON at trigger, schedule OFF at led_until.
#     The main loop checks led_until each iteration -> tap N+1 still seen.
#   * sleep_ms(1) only on None reads (yields to scheduler without idling).
import sentai

# --- Defaults (override via run() / train() args) ---
NEED_TAPS       = 2        # 2 = double-tap
TAP_MG          = 500      # peak |a-baseline| to register a tap (mg)
REFRACTORY_MS   = 150      # suppress double-counts from one impact ring
WINDOW_MS       = 800      # all NEED_TAPS must arrive inside this window
COOLDOWN_MS     = 400      # silence after a trigger fires
BASELINE_MS     = 250      # gravity calibration duration
LED_FLASH_MS    = 250      # LED on-time per trigger (visual ack)
SPIN_POWER      = 6553     # 10% — same as validated test_fly call
SPIN_MS         = 1500

# Set by train(); used by run() unless overridden.
_LEARNED_TAP_MG = None


def _calibrate(ms):
    t0 = sentai.rtos.ticks_ms()
    sx = sy = sz = 0.0
    n = 0
    while sentai.rtos.ticks_ms() - t0 < ms:
        d = sentai.imu.read()
        if d is None:
            sentai.rtos.sleep_ms(1)
            continue
        sx += d['x']; sy += d['y']; sz += d['z']
        n += 1
    if n == 0:
        return (0.0, 0.0, 0.0)
    return (sx / n, sy / n, sz / n)


def _abs(v):
    return v if v >= 0 else -v


def _max3(a, b, c):
    m = a
    if b > m: m = b
    if c > m: m = c
    return m


def train(n_taps=5, deadline_ms=15000, refractory_ms=200, floor_mg=180):
    """Learn this board's tap profile. Tap N times at the strength you want.

    Returns the chosen tap_mg threshold and stores it as the module default.
    Strategy: scan |a-baseline| for floor_mg-or-greater spikes, take the
    minimum peak across n_taps taps, set threshold = 0.55 * min_peak so
    that softer real taps still register.
    """
    global _LEARNED_TAP_MG
    sentai.imu.init()
    sentai.io.led_off()
    print('[train] calibrating baseline %d ms ...' % BASELINE_MS)
    bx, by, bz = _calibrate(BASELINE_MS)
    print('[train] tap %d times now (you have %d s) ...'
          % (n_taps, deadline_ms // 1000))

    peaks = []
    cur_peak = 0
    in_event = False
    last_tap_t = -1000000
    t_end = sentai.rtos.ticks_ms() + deadline_ms
    sentai.io.led_on()  # cue: training started
    while len(peaks) < n_taps:
        now = sentai.rtos.ticks_ms()
        if now >= t_end:
            break
        d = sentai.imu.read()
        if d is None:
            sentai.rtos.sleep_ms(1)
            continue
        peak = _max3(_abs(d['x'] - bx), _abs(d['y'] - by), _abs(d['z'] - bz))
        if peak >= floor_mg:
            if not in_event and (now - last_tap_t) >= refractory_ms:
                in_event = True
                cur_peak = peak
            elif in_event and peak > cur_peak:
                cur_peak = peak
        else:
            if in_event:
                peaks.append(cur_peak)
                last_tap_t = now
                in_event = False
                # Quick LED blink to confirm we saw the tap.
                sentai.io.led_off(); sentai.rtos.sleep_ms(60)
                sentai.io.led_on()
                print('[train] tap %d/%d  peak=%d mg' % (len(peaks), n_taps, int(cur_peak)))
    sentai.io.led_off()

    if not peaks:
        print('[train] FAIL: no taps detected (try harder, or lower floor_mg)')
        return None
    pmin = min(peaks)
    pmed = sorted(peaks)[len(peaks)//2]
    # 55% of the softest tap -> headroom for slightly weaker real taps,
    # still well above noise (~12 mg).
    chosen = max(int(0.55 * pmin), 200)
    _LEARNED_TAP_MG = chosen
    print('[train] peaks=%s  min=%d  median=%d  -> tap_mg=%d (saved)'
          % (peaks, int(pmin), int(pmed), chosen))
    return chosen


def run(run_secs=60, spin_motors=False, power=SPIN_POWER, spin_ms=SPIN_MS,
        need_taps=NEED_TAPS, tap_mg=None, window_ms=WINDOW_MS,
        refractory_ms=REFRACTORY_MS, cooldown_ms=COOLDOWN_MS,
        led_flash_ms=LED_FLASH_MS, verbose=False):
    """Listen for an N-tap gesture; flash LED (and optionally spin motors).

    Pass verbose=True to log each registered tap (slower / more latency).
    """
    sentai.imu.init()
    sentai.io.led_off()
    if spin_motors:
        sentai.crazy.init()  # no-op if already running

    if tap_mg is None:
        tap_mg = _LEARNED_TAP_MG if _LEARNED_TAP_MG is not None else TAP_MG

    print('[tap] calibrating %d ms ...' % BASELINE_MS)
    bx, by, bz = _calibrate(BASELINE_MS)
    print('[tap] baseline x=%.1f y=%.1f z=%.1f mg' % (bx, by, bz))
    print('[tap] armed: need=%d  tap_mg=%d  win=%d  led=%d  spin=%s  run=%ds  v=%d'
          % (need_taps, tap_mg, window_ms, led_flash_ms,
             ('YES @%d/%dms' % (power, spin_ms)) if spin_motors else 'no',
             run_secs, 1 if verbose else 0))

    t_end       = sentai.rtos.ticks_ms() + run_secs * 1000
    t_last_tap  = -1000000
    t_first_tap = -1000000
    tap_count   = 0
    triggers    = 0
    cooldown_until = 0
    led_until      = 0      # 0 == LED off
    samples     = 0
    nones       = 0

    while True:
        now = sentai.rtos.ticks_ms()
        if now >= t_end:
            break

        # Non-blocking LED off.
        if led_until and now >= led_until:
            sentai.io.led_off()
            led_until = 0

        d = sentai.imu.read()
        if d is None:
            nones += 1
            sentai.rtos.sleep_ms(1)
            continue
        samples += 1

        if now < cooldown_until:
            continue
        if now - t_last_tap < refractory_ms:
            continue

        dx = _abs(d['x'] - bx)
        dy = _abs(d['y'] - by)
        dz = _abs(d['z'] - bz)
        peak = _max3(dx, dy, dz)
        if peak < tap_mg:
            continue

        t_last_tap = now
        if tap_count == 0 or (now - t_first_tap) > window_ms:
            tap_count = 1
            t_first_tap = now
        else:
            tap_count += 1

        if verbose:
            print('[tap] %d/%d  peak=%d mg  dt=%d ms'
                  % (tap_count, need_taps, int(peak), now - t_first_tap))

        if tap_count >= need_taps:
            triggers += 1
            tap_count = 0
            sentai.io.led_on()
            led_until = now + led_flash_ms
            if spin_motors:
                # NOTE: blocking — motor spin is intentional foreground.
                try:
                    sentai.crazy.test_fly(power, spin_ms)
                except Exception as e:
                    print('[tap] test_fly EXC:', e)
            cooldown_until = sentai.rtos.ticks_ms() + cooldown_ms

    sentai.io.led_off()
    print('[tap] done  triggers=%d samples=%d nones=%d'
          % (triggers, samples, nones))
    return triggers
