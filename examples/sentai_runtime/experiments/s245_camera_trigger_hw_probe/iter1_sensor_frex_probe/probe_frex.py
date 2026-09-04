#!/usr/bin/env python3
"""s245 iter1 — OV5640 FREX / snapshot block probe on BOTH sensors.

Question this answers: is a sensor-side hardware trigger (FREX frame-exposure /
snapshot mode) even reachable on this board, on both cameras?

No firmware change needed: `sentai.camera.reg_read/reg_write(cam_id, reg, val)`
already give raw SCCB access, and per `libs/camera/cam_mux.h` the cam_id → I2C
bus mapping is 1:1, so register access is independent of the MUX state.

Three things are measured:
  A. DUMP    — the FREX/strobe block 0x3B00-0x3B0C plus pad-control and system
               registers, from cam0 and cam1.
  B. WRITE   — write-readback-restore on FREX_EXP (0x3B04/0x3B05).  These are
               inert while FREX mode is off, so this is a safe liveness test
               that distinguishes a real register from a read-only shadow.
  C. COMPARE — cam0 vs cam1 must agree; a divergence would mean the two sensors
               are not configured identically, which matters for any trigger
               scheme that has to drive both.

Outputs (in this folder): frex_dump.csv, verdict.json, run_probe.log
"""
import sys, os, re, json, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 's236_model_speed_sweep_640x480'))
import board_serial as B

CSV     = os.path.join(HERE, 'frex_dump.csv')
VERDICT = os.path.join(HERE, 'verdict.json')

# (addr, name) — OV5640.  FREX = "frame exposure" = the sensor's snapshot mode.
REGS = [
    (0x3008, 'SYSTEM_CTROL0 (bit6=sw power down)'),
    (0x300E, 'MIPI_CONTROL00'),
    (0x3016, 'PAD_OUTPUT_ENABLE00'),
    (0x3017, 'PAD_OUTPUT_ENABLE01'),
    (0x3018, 'PAD_OUTPUT_ENABLE02'),
    (0x3019, 'PAD_OUTPUT_VALUE00'),
    (0x301C, 'PAD_OUTPUT_ENABLE_SEL'),
    (0x301D, 'PAD_OUTPUT_VALUE_SEL'),
    (0x3B00, 'SRB_CTRL (strobe control)'),
    (0x3B01, 'FREX_EXPOSURE_HH'),
    (0x3B02, 'FREX_SHUTTER_DLY_HI'),
    (0x3B03, 'FREX_SHUTTER_DLY_LO'),
    (0x3B04, 'FREX_EXPOSURE_HI'),
    (0x3B05, 'FREX_EXPOSURE_LO'),
    (0x3B06, 'FREX_CTRL01'),
    (0x3B07, 'FREX_MODE'),
    (0x3B08, 'FREX_REQUEST'),
    (0x3B09, 'FREX_HREF_DLY'),
    (0x3B0A, 'FREX_RST_LENGTH_HI'),
    (0x3B0B, 'FREX_RST_LENGTH_LO'),
    (0x3B0C, 'FREX_STROBE_WIDTH'),
    (0x3F00, 'ISP_CTRL_3F00'),
    (0x3F01, 'ISP_CTRL_3F01'),
    (0x3F02, 'ISP_CTRL_3F02'),
    (0x4740, 'IO_PAD_VSYNC_POLARITY'),
    (0x3035, 'SC_PLL_CTRL1 (frame timing)'),
    (0x3036, 'SC_PLL_CTRL2 (frame timing)'),
]

# Scratch registers for the liveness write test.  FREX_EXPOSURE is inert while
# FREX mode is disabled (0x3B07 == 0), so writing then restoring is harmless.
SCRATCH = [(0x3B04, 'FREX_EXPOSURE_HI'), (0x3B05, 'FREX_EXPOSURE_LO')]
PATTERNS = [0x5A, 0xA5]


def N(x):
    m = re.findall(r'-?\d+', x)
    return int(m[-1]) if m else -999


def conn():
    for _ in range(20):
        for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
            try:
                B.PORT = p
                s = B.open_repl()
                if 'OK' in B.cmd(s, 'import sentai; print("OK")', 3):
                    return s
                s.close()
            except Exception:
                pass
        time.sleep(1.5)
    return None


class BD:
    def __init__(self):
        self.ser = conn()

    def c(self, cmd, t=8):
        for _ in range(3):
            try:
                return B.cmd(self.ser, cmd, t)
            except Exception:
                time.sleep(4)
                self.ser = conn()
                if not self.ser:
                    raise
        return ''

    def n(self, cmd, t=6):
        return N(self.c(cmd, t))

    def rd(self, cam, reg):
        return self.n('print(sentai.camera.reg_read(%d,0x%04X))' % (cam, reg))

    def wr(self, cam, reg, val):
        return self.n('print(sentai.camera.reg_write(%d,0x%04X,0x%02X))'
                      % (cam, reg, val))


def main():
    log = open(os.path.join(HERE, 'run_probe.log'), 'w')

    def P(*a):
        s = ' '.join(str(x) for x in a)
        print(s)
        log.write(s + '\n')
        log.flush()

    bd = BD()
    if not bd.ser:
        P('NO REPL — is the board enumerated as 1fc9:c0a1?')
        return 1

    ver = bd.c('print(sentai.version())', 3)
    build = ver.strip().split('\n')[-2][:46] if '\n' in ver else ver.strip()[:46]
    P('build:', build)

    # reg_read/reg_write require g_cam_initialized (sentai_runtime.cc:3506).
    rc = bd.n('print(sentai.camera.init(1,45))', 20)
    P('camera.init(1,45) ->', rc, '| fps:', bd.n('print(sentai.camera.fps())'))
    if rc != 0:
        P('FAIL: camera not initialized, reg access is guarded on it')
        return 1

    # ---- A. DUMP -----------------------------------------------------
    P('\n--- A. FREX / pad / system register dump ---')
    P('%-8s %-34s %6s %6s  %s' % ('addr', 'name', 'cam0', 'cam1', 'agree'))
    rows, disagree = [], 0
    for addr, name in REGS:
        v0, v1 = bd.rd(0, addr), bd.rd(1, addr)
        ok = (v0 == v1)
        if not ok:
            disagree += 1
        s0 = ('0x%02X' % v0) if v0 >= 0 else 'ERR'
        s1 = ('0x%02X' % v1) if v1 >= 0 else 'ERR'
        P('0x%04X   %-34s %6s %6s  %s'
          % (addr, name, s0, s1, 'yes' if ok else 'NO'))
        rows.append((addr, name, v0, v1, ok))

    read_errors = sum(1 for r in rows if r[2] < 0 or r[3] < 0)

    # ---- B. WRITE-READBACK-RESTORE -----------------------------------
    P('\n--- B. FREX write-readback-restore liveness test ---')
    live = {}
    for cam in (0, 1):
        for addr, name in SCRATCH:
            orig = bd.rd(cam, addr)
            if orig < 0:
                P('cam%d 0x%04X %-20s READ ERR — skipped' % (cam, addr, name))
                live['cam%d_0x%04X' % (cam, addr)] = 'read_err'
                continue
            got = []
            for pat in PATTERNS:
                bd.wr(cam, addr, pat)
                got.append(bd.rd(cam, addr))
            bd.wr(cam, addr, orig)          # restore
            back = bd.rd(cam, addr)
            sticks = all(g == p for g, p in zip(got, PATTERNS))
            restored = (back == orig)
            verdict = 'LIVE' if sticks else 'not-writable'
            if not restored:
                verdict += ' RESTORE-FAILED'
            P('cam%d 0x%04X %-20s orig=0x%02X wrote=%s read=%s restored=%s -> %s'
              % (cam, addr, name, orig,
                 [hex(p) for p in PATTERNS], [hex(g) for g in got],
                 'ok' if restored else 'NO', verdict))
            live['cam%d_0x%04X' % (cam, addr)] = verdict

    # ---- C. VERDICT --------------------------------------------------
    frex_mode0, frex_mode1 = bd.rd(0, 0x3B07), bd.rd(1, 0x3B07)
    srb0, srb1 = bd.rd(0, 0x3B00), bd.rd(1, 0x3B00)
    all_live = all(v == 'LIVE' for v in live.values()) and bool(live)

    P('\n--- C. Verdict ---')
    P('read errors           :', read_errors)
    P('cam0/cam1 disagreements:', disagree)
    P('FREX block writable   :', 'YES' if all_live else 'NO')
    P('FREX_MODE  (0x3B07)   : cam0=0x%02X cam1=0x%02X %s'
      % (frex_mode0, frex_mode1,
         '(0 = FREX/snapshot mode OFF)' if frex_mode0 == 0 else ''))
    P('SRB_CTRL   (0x3B00)   : cam0=0x%02X cam1=0x%02X' % (srb0, srb1))
    P('\nPASS criteria: block readable on both cams (no ERR) AND writable.')
    P('RESULT:', 'PASS' if (read_errors == 0 and all_live) else 'FAIL')

    with open(CSV, 'w') as f:
        f.write('addr,name,cam0,cam1,agree\n')
        for addr, name, v0, v1, ok in rows:
            f.write('0x%04X,"%s",%d,%d,%d\n' % (addr, name, v0, v1, 1 if ok else 0))

    json.dump({
        'build': build,
        'read_errors': read_errors,
        'cam_disagreements': disagree,
        'frex_writable': all_live,
        'liveness': live,
        'frex_mode': {'cam0': frex_mode0, 'cam1': frex_mode1},
        'srb_ctrl': {'cam0': srb0, 'cam1': srb1},
        'result': 'PASS' if (read_errors == 0 and all_live) else 'FAIL',
    }, open(VERDICT, 'w'), indent=2)

    P('\nwrote %s' % CSV)
    P('wrote %s' % VERDICT)
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
