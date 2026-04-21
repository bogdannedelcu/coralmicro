# _e23_vga45_alt_capture.py — alternating-capture JPEG sanity at VGA@45.
#
# After E22 showed 21 fps alternating in the parallel pipeline, we want
# VISUAL proof that each camera's frames are clean — no mid-buffer seam
# (the cam0/cam1 MIPI MUX glitch E17 first documented at 720p threshold=1)
# and no inter-cam leakage at the new 74 fps effective rate.
#
# Two phases:
#   Phase 1 — pipeline with ratio(1,1) for ~80 frames (healthy run
#             signal): fps, drops, cam_stats.
#   Phase 2 — stop pipeline, loop select(cam) + jpeg(quality) for
#             10 pairs, save each JPEG with cam id in filename.
#             Incremental save to avoid heap bloat.
#
# Output: /diags/sNNN_e23_vga45_alt/{001_e23_alt_frames}/{camX_NNN...}.jpg

import sentai, diag, gc

gc.collect()
if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)

from diag._session import _session, begin, end, _save_path, _photos_dir, _record, _save_desc
from diag._util import save_csv, _ticks, stats

owned = (_session is None)
if owned:
    begin("e23_vga45_alt")

# ── Phase 1: parallel pipeline alternating ────────────────────────────
sentai.camera.ratio(1, 1)
sentai.camera.select(0)
sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)  # warmup
wall = []
t_prev = sentai.rtos.ticks_ms()
for _ in range(80):
    r = sentai.pipeline.get_ex(2000)
    t_now = sentai.rtos.ticks_ms()
    if r is None:
        continue
    wall.append(t_now - t_prev); t_prev = t_now
sentai.pipeline.stop()
sentai.camera.ratio(0, 0)

mean_wall = sum(wall) / len(wall) if wall else 0
fps_par = 1000.0 / mean_wall if mean_wall else 0
cs_before = sentai.diag.cam_stats()
print("[E23:phase1] parallel alt 1:1: %.2f fps over %d frames" % (fps_par, len(wall)))
print("             cam_stats:", cs_before)

# ── Phase 2: manual alternating JPEG capture ──────────────────────────
csv_path = _save_path("e23_vga45_alt")
frames_dir = _photos_dir("e23_alt")
if frames_dir:
    try: sentai.fs.mkdir(frames_dir)
    except Exception: pass
else:
    frames_dir = "/diags/e23_alt_frames"
    try: sentai.fs.mkdir(frames_dir)
    except Exception: pass

# Warmup each cam once (retry on first-frame miss).
def _warm(cam):
    sentai.camera.select(cam)
    for _ in range(5):
        try: sentai.camera.jpeg(60); return True
        except RuntimeError: pass
    return False
if not (_warm(0) and _warm(1)):
    print("E23 ABORT: warmup failed")
    if owned: end()
    raise SystemExit

quality = 70
pairs = 10
dts = []; szs = []; cams = []
saved = 0
t0 = _ticks()
for i in range(pairs):
    for cam in (0, 1):
        ts = _ticks()
        sentai.camera.select(cam)
        jpg = sentai.camera.jpeg(quality)
        dt = _ticks() - ts
        dts.append(dt); szs.append(len(jpg)); cams.append(cam)
        path = "%s/%03d_cam%d_%dms_%db.jpg" % (frames_dir, i*2+cam, cam, dt, len(jpg))
        try:
            sentai.fs.write(path, jpg); saved += 1
        except Exception as e:
            print(" WARN save", path, e)
        jpg = None
        gc.collect()
wall2 = _ticks() - t0

st = stats(dts)
cs_after = sentai.diag.cam_stats()
print("[E23:phase2] alt capture: saved=%d/%d wall=%dms" % (saved, pairs*2, wall2))
print("  per-capture ms: min=%d mean=%.1f max=%d" % (st["min"], st["mean"], st["max"]))
print("  jpeg bytes:     min=%d mean=%d max=%d" % (min(szs), sum(szs)//len(szs), max(szs)))
print("  cam_stats delta: ok_eof=%d fallback=%d drain_to=%d" % (
    cs_after["switch_ok_eof"] - cs_before["switch_ok_eof"],
    cs_after["switch_fallback"] - cs_before["switch_fallback"],
    cs_after["drain_timeout"] - cs_before["drain_timeout"]))
print("  frames at:", frames_dir)

save_csv(csv_path,
    ["run_index", "cam_id", "elapsed_ms", "jpeg_bytes"],
    [(i, cams[i], dts[i], szs[i]) for i in range(len(dts))])
_save_desc(csv_path,
    "E23 - VGA@45 alternating JPEG capture (after pipeline/ratio run).\n"
    "Phase 1: parallel pipeline ratio(1,1) fps recorded\n"
    "Phase 2: 10 pairs of (cam0, cam1) JPEGs saved for visual check\n",
    params={"pipeline_alt_fps": fps_par,
            "native_res": "640x480", "quality": quality,
            "pairs": pairs, "switch_drain": 1,
            "mean_capture_ms": st["mean"],
            "jpeg_dir": frames_dir})
_record("e23_vga45_alt", csv_path,
        "alt_par_fps=%.1f captures=%d mean=%.1fms" % (fps_par, saved, st["mean"]))

if owned:
    end()
