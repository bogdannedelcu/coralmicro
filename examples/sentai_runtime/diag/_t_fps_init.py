# _t_fps_init.py — verify sentai.camera.init(streaming, fps) picks
# the right OV5640 PLL config at FIRST init.
#
# REPL self-correcting idiom:
#   rc = sentai.camera.init(1, target)
#   if rc == 0:    proceed           (fresh boot OR same fps as before)
#   if rc == -11:  sys.reset()       (camera up at a DIFFERENT fps; reboot)
#
# Caller picks fps via _target_fps in REPL globals before exec.
import sentai
sentai.verbose(1)

try:
    _fps = int(_target_fps)  # noqa: F821
except Exception:
    _fps = 30

EXPECTED = {
    30: {"pll_ctrl2": 0x38, "pclk_period": 0x14},
    45: {"pll_ctrl2": 0x54, "pclk_period": 0x0D},
    60: {"pll_ctrl2": 0x70, "pclk_period": 0x0C},
}

print("=== boot ===")
print("version=", sentai.version())
print("--- target fps:", _fps, "---")

rc = sentai.camera.init(1, _fps)
print("init(1, %d) =" % _fps, rc)

if rc == -11:
    print("camera already up at different fps -> sys.reset()")
    sentai.rtos.sleep_ms(200)
    sentai.sys.reset()  # never returns
elif rc != 0:
    print("FAIL — init returned %d" % rc)
else:
    sentai.rtos.sleep_ms(800)
    p2_0 = sentai.camera.reg_read(0, 0x3036)
    pp_0 = sentai.camera.reg_read(0, 0x4837)
    p2_1 = sentai.camera.reg_read(1, 0x3036)
    pp_1 = sentai.camera.reg_read(1, 0x4837)
    exp = EXPECTED.get(_fps, {"pll_ctrl2": 0, "pclk_period": 0})
    ok = (p2_0 == exp["pll_ctrl2"] and pp_0 == exp["pclk_period"]
          and p2_1 == exp["pll_ctrl2"] and pp_1 == exp["pclk_period"])
    print("  cam0 0x3036=0x%02X 0x4837=0x%02X  cam1 0x3036=0x%02X 0x4837=0x%02X" %
          (p2_0, pp_0, p2_1, pp_1))
    print("  expected            0x%02X 0x%02X" %
          (exp["pll_ctrl2"], exp["pclk_period"]))
    print("  RESULT:", "OK" if ok else "FAIL")
print("=== done ===")
