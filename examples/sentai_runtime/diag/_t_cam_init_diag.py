# _t_cam_init_diag.py — verify OV5640 init-state registers match the
# expected fps-specific values from ov5640registers.md, on BOTH cams.
# Reads only — no select() calls (those wedge at VGA45/60 init), so
# this driver runs cleanly at any DEMO_CAMERA_FRAME_RATE.
#
# Compare values across builds:
#   - VGA30 (default):     pllCtrl1=0x11, pllCtrl2=0x46, pclkPeriod=?
#   - VGA45 (patched):     pllCtrl1=0x14, pllCtrl2=0x68, pclkPeriod=0x14
#   - VGA60 (patched):     pllCtrl1=0x14, pllCtrl2=0x70, pclkPeriod=0x0c
#
# If a register doesn't match the expected fps-specific value, the
# driver's clock-config path didn't apply correctly → sensor is
# producing the wrong line rate → CSI lock fails → wedge.
#
# Output is printed and saved under /diags/sNNN_cam_init_diag/.
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


# Registers to dump and what they mean.  See ov5640registers.md.
REGS = [
    (0x300A, "CHIP_ID_HIGH",          "should be 0x56"),
    (0x300B, "CHIP_ID_LOW",           "should be 0x40"),
    (0x3008, "SYSTEM_CTRL0",          "0x02=normal, 0x42=standby, 0x82=reset"),
    (0x3034, "SC_PLL_CTRL0",          "PLL bit-mode (0x18 = 8-bit)"),
    (0x3035, "SC_PLL_CTRL1",          "VGA30=0x11 VGA45=0x14 VGA60=0x14"),
    (0x3036, "SC_PLL_CTRL2",          "VGA30=0x46 VGA45=0x68 VGA60=0x70"),
    (0x3037, "SC_PLL_CTRL3",          "PLL pre-divider (0x13 = 3 typ.)"),
    (0x3108, "SC_PLL_CTRL5",          "PLL_root_div"),
    (0x3824, "PCLK_DIV",              "pixel clock divider"),
    (0x4837, "PCLK_PERIOD",           "VGA30=? VGA45=0x14 VGA60=0x0c"),
    (0x3800, "X_ADDR_ST_H",           "crop start X high"),
    (0x3801, "X_ADDR_ST_L",           "crop start X low"),
    (0x3802, "Y_ADDR_ST_H",           "crop start Y high"),
    (0x3803, "Y_ADDR_ST_L",           "crop start Y low"),
    (0x3804, "X_ADDR_END_H",          "crop end X high"),
    (0x3805, "X_ADDR_END_L",          "crop end X low"),
    (0x3806, "Y_ADDR_END_H",          "crop end Y high"),
    (0x3807, "Y_ADDR_END_L",          "crop end Y low"),
    (0x3808, "DVP_H_OUT_HIGH",        "VGA: 0x02 (640>>8)"),
    (0x3809, "DVP_H_OUT_LOW",         "VGA: 0x80 (640&0xFF)"),
    (0x380A, "DVP_V_OUT_HIGH",        "VGA: 0x01 (480>>8)"),
    (0x380B, "DVP_V_OUT_LOW",         "VGA: 0xE0 (480&0xFF)"),
    (0x380C, "HTS_HIGH",              "total H size incl. blanking"),
    (0x380D, "HTS_LOW",               ""),
    (0x380E, "VTS_HIGH",              "total V size incl. blanking — fps lever"),
    (0x380F, "VTS_LOW",               ""),
    (0x4814, "MIPI_CTRL14",           "0x00 = auto DT"),
    (0x501F, "FORMAT_MUX_CTRL",       "ISP output format"),
    (0x503D, "PRE_ISP_TEST_SETTING1", "0x00 = no test pattern"),
]


def _dump(cam_id):
    out = []
    for reg, name, note in REGS:
        try:
            v = sentai.camera.reg_read(cam_id, reg)
        except Exception as e:
            v = -1
        out.append((reg, name, v, note))
    return out


print("=== boot ===")
print("version=", sentai.version())
sentai.camera.init(1)
sentai.rtos.sleep_ms(1500)  # let the init dust settle
print("--- camera init done; reading registers without any select() ---")

dump0 = _dump(0)
dump1 = _dump(1)

print()
print("  reg     | name                        | cam0 | cam1 | note")
print("  --------|-----------------------------|------|------|-----")
for (r0, n, v0, note), (_r1, _n2, v1, _n3) in zip(dump0, dump1):
    print("  0x%04X | %-27s | 0x%02X | 0x%02X | %s" %
          (r0, n, v0 & 0xFF, v1 & 0xFF, note))

# Save dump to LFS for offline analysis / cross-fps comparison.
sd = _session_dir("cam_init_diag")
build_id = sentai.version().split("build")[1].split()[0] if "build" in sentai.version() else "?"
parts = ["build,reg,name,cam0,cam1,note\n"]
for (r0, n, v0, note), (_r1, _n2, v1, _n3) in zip(dump0, dump1):
    parts.append("%s,0x%04X,%s,0x%02X,0x%02X,%s\n" %
                 (build_id, r0, n, v0 & 0xFF, v1 & 0xFF, note))
sentai.fs.write(sd + "/regs.csv", "".join(parts))
print("log saved:", sd + "/regs.csv")

# Quick sanity check: chip ID + system ctrl in normal state.
def _check(label, expected, actual):
    ok = "OK" if (actual & 0xFF) == expected else "FAIL"
    print("  %-30s expected 0x%02X got 0x%02X  %s" %
          (label, expected, actual & 0xFF, ok))

print()
print("--- sanity checks ---")
_check("cam0 CHIP_ID_HIGH", 0x56, dump0[0][2])
_check("cam0 CHIP_ID_LOW",  0x40, dump0[1][2])
_check("cam0 SYSTEM_CTRL0", 0x02, dump0[2][2])
_check("cam1 CHIP_ID_HIGH", 0x56, dump1[0][2])
_check("cam1 CHIP_ID_LOW",  0x40, dump1[1][2])
_check("cam1 SYSTEM_CTRL0", 0x02, dump1[2][2])

print("=== done ===")
