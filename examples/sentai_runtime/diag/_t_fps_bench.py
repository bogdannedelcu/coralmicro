# _t_fps_bench.py — CANONICAL camera-parity regression test.
#
# Validates the dual-camera alternation pipeline END-TO-END at the
# active sensor fps:
#   * `init(1, fps)` succeeds (or returns -11 + driver self-resets)
#   * test patterns (BARS on cam0, HBAND on cam1) are visible
#   * `peek5_b40()` 5-row sampling classifies each captured frame
#   * cam_id tag agrees with the captured pattern → 100 % at every
#     ratio (1:1, 2:1, 3:1) at every supported fps (30/45/60)
#
# Re-run this test periodically:
#   - after every change to libs/camera/, sentai_runtime.cc cam_*,
#     modsentai_camera.c, or the OV5640 driver patches
#   - after every fps-table addition (fsl_ov5640.c VGA rows,
#     csi2rxHsSettle[] entries)
#   - after every linker-script reshuffle that moves CSI ISR
#     code out of .ramfunc / m_text
# A regression here is a load-bearing failure — DO NOT ship without
# 100 % at every ratio at every supported fps.
#
# How to drive (host-side):
#   python3 diag/_host_run_with_var.py \
#       --file /lib/diag/_t_fps_bench.py \
#       --set _target_fps=30 --timeout 90
# Repeat with _target_fps=45 then 60.  When the firmware boot fps
# does not match the requested fps the driver prints
# `mismatch -> reset` and `sys.reset()`s; re-issue the same command
# after the board re-enumerates.
#
# Persistent flash REQUIRED — `sys.reset()` returns the ROM
# bootloader to flashed firmware; --ram firmware is discarded.
import sentai
sentai.verbose(1)


def _sd(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    c = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(c, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _cls(b):
    if b >= 0xA0: return "B"
    if b <= 0x80: return "H"
    return "A"


def _cls5(s):
    cs = tuple(_cls(x) for x in s)
    if all(c == "B" for c in cs): return "BARS"
    if all(c == "H" for c in cs): return "HBAND"
    if all(c == "A" for c in cs): return "AMBIG"
    return "SCR"


def _bench(label, ra, rb, N):
    sentai.camera.ratio(ra, rb)
    sentai.camera.switch_drain(1)
    sentai.camera.select(0); sentai.rtos.sleep_ms(300)
    sentai.camera.peek5_b40()
    ok = scr = wr = c0 = c1 = 0
    durs = []
    last = sentai.rtos.ticks_ms()
    for i in range(N):
        r = sentai.camera.peek5_b40()
        now = sentai.rtos.ticks_ms()
        durs.append(now - last); last = now
        cam = r[0]; cl = _cls5(r[1:6])
        if cl == "SCR": scr += 1
        elif cl == "BARS" and cam == 0: ok += 1
        elif cl == "HBAND" and cam == 1: ok += 1
        else: wr += 1
        if cam == 0: c0 += 1
        elif cam == 1: c1 += 1
        if (i % 20) == 0: sentai.diag.repl_kick()
    durs.sort()
    avg = sum(durs) // len(durs) if durs else 0
    return {"l": label, "N": N, "ok": ok, "scr": scr, "wr": wr,
            "c0": c0, "c1": c1, "a": avg,
            "p50": durs[len(durs)//2] if durs else 0,
            "p99": durs[(len(durs)*99)//100] if durs else 0,
            "f": (1000 + avg//2)//avg if avg else 0}


try: _fps = int(_target_fps)  # noqa: F821
except Exception: _fps = 30

print("=== boot ===")
print("--- target fps:", _fps, "---")
rc = sentai.camera.init(1, _fps)
print("init(1, %d) =" % _fps, rc)
if rc == -11:
    print("mismatch -> reset")
    sentai.rtos.sleep_ms(200)
    sentai.sys.reset()
elif rc != 0:
    print("FAIL init=%d" % rc)
    print("=== done ===")
else:
    sentai.rtos.sleep_ms(800)
    sentai.camera.test_pattern(0, 1)
    sentai.camera.test_pattern(1, 2)
    sentai.camera.select(0); sentai.rtos.sleep_ms(500)
    sentai.camera.select(1); sentai.rtos.sleep_ms(500)
    sentai.camera.select(0); sentai.rtos.sleep_ms(300)
    NB = 100
    res = []
    print("--- 1:1 ---"); res.append(_bench("1:1", 1, 1, NB))
    print("--- 2:1 ---"); res.append(_bench("2:1", 2, 1, NB))
    print("--- 3:1 ---"); res.append(_bench("3:1", 3, 1, NB))
    sentai.camera.ratio(0, 0)
    sentai.camera.select(0)
    sentai.camera.test_pattern(0, 0)
    sentai.camera.test_pattern(1, 0)
    print("--- summary fps=%d ---" % _fps)
    print("  mode | ok/N    | scr | wr  | c0:c1   | ms a/p50/p99 | fps")
    for r in res:
        print("  %-4s | %3d/%-3d | %3d | %3d | %2d:%-2d   | %3d/%3d/%3d  | %d" %
              (r["l"], r["ok"], r["N"], r["scr"], r["wr"],
               r["c0"], r["c1"], r["a"], r["p50"], r["p99"], r["f"]))
    sd = _sd("fps_%d" % _fps)
    L = ["fps,mode,N,ok,scr,wr,c0,c1,avg,p50,p99,floop\n"]
    for r in res:
        L.append("%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" %
                 (_fps, r["l"], r["N"], r["ok"], r["scr"], r["wr"],
                  r["c0"], r["c1"], r["a"], r["p50"], r["p99"], r["f"]))
    sentai.fs.write(sd + "/bench.csv", "".join(L))
    print("log:", sd + "/bench.csv")
    print("=== done ===")
