# _t_fps_cycle.py — verify sentai.camera.set_fps() can switch
# between 30/45/60 multiple times in one boot, with the OV5640 PLL
# config registers actually changing each time.
#
# Sequence: init → set_fps(30) → check → set_fps(45) → check →
# set_fps(60) → check → set_fps(30) → check (round-trip).  Reads
# the canonical fps-distinguishing registers (0x3036 SC_PLL_CTRL2
# and 0x4837 PCLK_PERIOD) per ov5640registers.md.
import sentai
sentai.verbose(1)


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


# Expected register values per (resolution, fps) from
# fsl_ov5640.c VGA table (see ov5640registers.md / experiment.md
# "OV5640 init-register verification").
EXPECTED = {
    30: {"pll_ctrl2": 0x38, "pclk_period": 0x14},
    45: {"pll_ctrl2": 0x54, "pclk_period": 0x0D},
    60: {"pll_ctrl2": 0x70, "pclk_period": 0x0C},
}


def _check_fps_regs(fps, results):
    p2_0 = sentai.camera.reg_read(0, 0x3036)
    pp_0 = sentai.camera.reg_read(0, 0x4837)
    p2_1 = sentai.camera.reg_read(1, 0x3036)
    pp_1 = sentai.camera.reg_read(1, 0x4837)
    exp = EXPECTED[fps]
    ok = (p2_0 == exp["pll_ctrl2"] and pp_0 == exp["pclk_period"]
          and p2_1 == exp["pll_ctrl2"] and pp_1 == exp["pclk_period"])
    print("  fps=%d  cam0[0x3036=0x%02X 0x4837=0x%02X]  "
          "cam1[0x3036=0x%02X 0x4837=0x%02X]  expect[0x%02X 0x%02X]  %s" %
          (fps, p2_0, pp_0, p2_1, pp_1,
           exp["pll_ctrl2"], exp["pclk_period"],
           "OK" if ok else "FAIL"))
    results.append({
        "fps": fps,
        "cam0_pll_ctrl2": p2_0, "cam0_pclk_period": pp_0,
        "cam1_pll_ctrl2": p2_1, "cam1_pclk_period": pp_1,
        "ok": ok,
    })
    return ok


print("=== boot ===")
print("version=", sentai.version())
sentai.camera.init(1)
sentai.rtos.sleep_ms(800)

results = []

for cycle, fps in enumerate([30, 45, 60, 30], 1):
    print("--- cycle %d: set_fps(%d) ---" % (cycle, fps))
    rc = sentai.camera.set_fps(fps)
    print("  set_fps(%d) =" % fps, rc)
    if rc != 0:
        print("  FAIL — set_fps returned %d, aborting" % rc)
        results.append({"fps": fps, "ok": False, "rc": rc})
        break
    sentai.rtos.sleep_ms(800)  # let PLL/MIPI re-lock
    sentai.diag.repl_kick()
    # (sentai.camera.fps() getter was dropped to free m_text; we
    #  read PLL/PCLK regs directly to verify the new fps applied.)
    if not _check_fps_regs(fps, results):
        print("  FAIL on register check at fps=%d" % fps)
    sentai.diag.repl_kick()

print("--- summary ---")
total = sum(1 for r in results if r.get("ok"))
print("  %d / %d cycles OK" % (total, len(results)))

sd = _session_dir("fps_cycle")
parts = ["cycle,fps,cam0_pll_ctrl2,cam0_pclk,cam1_pll_ctrl2,cam1_pclk,ok\n"]
for i, r in enumerate(results, 1):
    parts.append("%d,%d,0x%02X,0x%02X,0x%02X,0x%02X,%s\n" %
                 (i, r["fps"],
                  r.get("cam0_pll_ctrl2", 0), r.get("cam0_pclk_period", 0),
                  r.get("cam1_pll_ctrl2", 0), r.get("cam1_pclk_period", 0),
                  "1" if r.get("ok") else "0"))
sentai.fs.write(sd + "/fps_cycle.csv", "".join(parts))
print("log saved:", sd + "/fps_cycle.csv")
print("=== done ===")
