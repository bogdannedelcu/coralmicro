# test_calib_ini_s188.py -- OP-S10-W21-T2 INI persistance round-trip
# + forward-compatibility smoke.
#
# Pure MicroPython; runs end-to-end inside sentai_sim via REPL `import`.
# Exercises the new sentai_calib.cc INI parser/writer (replaces the
# legacy JSON cam_calib.json -> calib.ini, schema v1 -> v2).
#
# Each test prints `[s188] T{N} {STATUS} <metrics>` and verdict.py greps.

import sentai

print("[s188] BEGIN")

I3       = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
TARGET_R = (
    0.996194720, -0.087155737, 0.000000000,
    0.087155737,  0.996194720, 0.000000000,
    0.000000000,  0.000000000, 1.000000000,
)
TARGET_OFF = (-0.04, 0.0, -0.02)

CALIB_FILE = "calib.ini"


def _write(path, content):
    sentai.fs.write(path, content)


def _read(path):
    try:
        return sentai.fs.read(path)
    except (AttributeError, OSError):
        return None


def _isclose(a, b, tol=1e-6):
    return abs(a - b) < tol


# ===================== T1 -- on-disk format ====================
# Commit known R + offset, save, then inspect calib.ini.
sentai.calib.clear()
sentai.calib.commit_R(TARGET_R, TARGET_OFF)
rc_save = sentai.calib.save()
disk    = _read(CALIB_FILE)
ok_t1   = (rc_save and
           disk is not None and
           "schema=2" in disk and
           "R_B_C=" in disk and
           "cam_offset_B=" in disk and
           "{" not in disk and        # not JSON
           "[" not in disk and        # not JSON
           '"' not in disk)           # not JSON
print("[s188] T1 {} rc_save={} len={} has_schema={} has_R={} has_off={}".format(
    "PASS" if ok_t1 else "FAIL",
    rc_save, len(disk) if disk else 0,
    "schema=2" in disk if disk else False,
    "R_B_C=" in disk if disk else False,
    "cam_offset_B=" in disk if disk else False))


# ===================== T2 -- schema mismatch rejected ==========
# Forge an INI with schema=99 (incompatible).  load() must reject and
# leave the in-RAM R untouched (still TARGET_R from T1).
sentai.calib.clear()                           # R now defaults
sentai.calib.commit_R(I3, (0.0, 0.0, 0.0))    # set a sentinel state
_write(CALIB_FILE,
       "schema=99\n"
       "R_B_C=1,0,0,0,1,0,0,0,1\n"
       "cam_offset_B=0,0,0\n")
rc_load = sentai.calib.load()
R_after = sentai.calib.get_R_cam_to_body()
# After failed load, R must still be identity (the sentinel we set).
ok_t2 = (rc_load == 0 and
         all(_isclose(R_after[i], I3[i]) for i in range(9)))
print("[s188] T2 {} rc_load={} R_unchanged={}".format(
    "PASS" if ok_t2 else "FAIL",
    rc_load,
    all(_isclose(R_after[i], I3[i]) for i in range(9))))


# ===================== T3 -- forward-compat (extra keys) =======
# Write an INI with unknown keys (kp_x, mystery_field).  Parser MUST
# ignore them and still load R + offset correctly.
sentai.calib.clear()
_write(CALIB_FILE,
       "schema=2\n"
       "kp_x=0.39\n"                            # future T3 key (ignored today)
       "mystery_field=hello world\n"             # unknown junk
       "R_B_C=0.996194720,-0.087155737,0.000000000,"
              "0.087155737,0.996194720,0.000000000,"
              "0.000000000,0.000000000,1.000000000\n"
       "cam_offset_B=-0.04,0.0,-0.02\n"
       "another_unknown=42\n")
rc_load   = sentai.calib.load()
R_loaded  = sentai.calib.get_R_cam_to_body()
off_loaded = sentai.calib.get_cam_offset_B()
err_R   = max(abs(R_loaded[i]   - TARGET_R[i])   for i in range(9))
err_off = max(abs(off_loaded[i] - TARGET_OFF[i]) for i in range(3))
ok_t3 = (rc_load == 1 and err_R < 1e-5 and err_off < 1e-5)
print("[s188] T3 {} rc_load={} err_R={:.2e} err_off={:.2e}".format(
    "PASS" if ok_t3 else "FAIL", rc_load, err_R, err_off))


# ===================== T4 -- empty / truncated file ============
sentai.calib.clear()
sentai.calib.commit_R(I3, (0.0, 0.0, 0.0))
_write(CALIB_FILE, "")                              # empty
rc_load_empty = sentai.calib.load()
_write(CALIB_FILE, "schema=2\n")                    # truncated (no R)
rc_load_trunc = sentai.calib.load()
ok_t4 = (rc_load_empty == 0 and rc_load_trunc == 0)
print("[s188] T4 {} rc_empty={} rc_trunc={}".format(
    "PASS" if ok_t4 else "FAIL", rc_load_empty, rc_load_trunc))


# ===================== T5 -- comment lines tolerated ===========
sentai.calib.clear()
_write(CALIB_FILE,
       "# sentai calib file -- generated 2026-05-21\n"
       "; legacy semicolon comment also OK\n"
       "schema=2\n"
       "\n"                                          # blank line
       "R_B_C=0.996194720,-0.087155737,0.000000000,"
              "0.087155737,0.996194720,0.000000000,"
              "0.000000000,0.000000000,1.000000000\n"
       "# offset is the cf2 SDF cam mount\n"
       "cam_offset_B=-0.04,0.0,-0.02\n")
rc_load   = sentai.calib.load()
R_loaded  = sentai.calib.get_R_cam_to_body()
err_R   = max(abs(R_loaded[i] - TARGET_R[i]) for i in range(9))
ok_t5 = (rc_load == 1 and err_R < 1e-5)
print("[s188] T5 {} rc_load={} err_R={:.2e}".format(
    "PASS" if ok_t5 else "FAIL", rc_load, err_R))


all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5
print("[s188] OVERALL", "PASS" if all_ok else "FAIL")
print("[s188] END")
