# _e21_vga_capture.py — capture a handful of JPEGs under a proper
# diag session for visual sanity of VGA 640×480 pipeline mode.
#
# Uses the standard session pattern so the files land at
# /diags/sNNN_e21_vga_capture/001_e21_vga_frames/... and the session
# index gets incremented in /diags/.counter alongside every other
# experiment — matching the convention used by E13/E14/E17/E18.
import sentai, diag, gc
from diag._session import (_session, begin, end, _save_path, _record,
                           _save_desc, _photos_dir, snapshot_scene)
from diag._util import save_csv, stats, _ticks

gc.collect()
if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.camera.ratio(0, 0)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

owned = (_session is None)
if owned:
    begin("e21_vga_capture")

# Warmup each camera with retry — first jpeg() after boot can miss if the
# CSI receiver hasn't produced a full frame yet.
def _warm(cam):
    sentai.camera.select(cam)
    for _ in range(5):
        try:
            _j = sentai.camera.jpeg(60); _j = None; gc.collect()
            return True
        except RuntimeError:
            pass
    return False

if not (_warm(0) and _warm(1)):
    print("E21 ABORT: camera warmup failed")
    if owned:
        end()
    raise SystemExit

sentai.camera.select(0)

# One scene snapshot BEFORE the measurement loop (saved by diag helper
# inside session dir).
snapshot_scene("before", name="scene_cam0")

nat = sentai.camera.native_res()
quality = 70
reps = 10

# Register the CSV path first so _photos_dir lines up with the same
# sequence number as the data file.
csv_path = _save_path("e21_vga_capture")
frames_dir = _photos_dir("e21_vga")
if frames_dir:
    try: sentai.fs.mkdir(frames_dir)
    except Exception: pass
else:
    frames_dir = "/diags/e21_vga_frames"
    try: sentai.fs.mkdir(frames_dir)
    except Exception: pass

print("E21: capturing %d JPEGs at native %dx%d → %s" % (reps, nat[0], nat[1], frames_dir))

dts = []
szs = []
saved = 0
t_run_start = _ticks()
for i in range(reps):
    t0 = _ticks()
    sentai.camera.select(0)
    jpg = sentai.camera.jpeg(quality)
    dt = _ticks() - t0
    dts.append(dt); szs.append(len(jpg))
    path = "%s/%03d_%dms_%db.jpg" % (frames_dir, i, dt, len(jpg))
    try:
        sentai.fs.write(path, jpg)
        saved += 1
    except Exception as e:
        print(" WARN", path, e)
    jpg = None
    gc.collect()
t_run_wall = _ticks() - t_run_start

# After-snapshot too, so the session has before/after pair.
snapshot_scene("after", name="scene_cam0")

st = stats(dts)
print("E21 done: saved=%d/%d wall=%dms" % (saved, reps, t_run_wall))
print("  per-frame capture ms: min=%d mean=%.1f max=%d" % (st["min"], st["mean"], st["max"]))
print("  jpeg bytes: min=%d mean=%d max=%d" % (min(szs), sum(szs)//len(szs), max(szs)))
print("  frames at:", frames_dir)

save_csv(csv_path,
    ["run_index", "elapsed_ms", "jpeg_bytes"],
    [(i, dt, sz) for i, (dt, sz) in enumerate(zip(dts, szs))])
_save_desc(csv_path,
    "E21 - VGA 640x480 visual capture (post pclkPeriod=0x14 fix).\n"
    "Purpose: confirm pixels are sane at the new native VGA mode,\n"
    "not just that throughput metrics improved.\n"
    "\nColumns:\n"
    "  run_index   : 0..N-1\n"
    "  elapsed_ms  : per-call sentai.camera.jpeg wall time\n"
    "  jpeg_bytes  : size of the encoded JPEG\n",
    params={"native_res": "%dx%d" % (nat[0], nat[1]),
            "quality": quality, "reps": reps,
            "mean_frame_ms": st["mean"],
            "jpeg_dir": frames_dir})
_record("e21_vga_capture", csv_path,
        "%dx%d reps=%d mean=%.1fms" % (nat[0], nat[1], reps, st["mean"]))

if owned:
    end()
