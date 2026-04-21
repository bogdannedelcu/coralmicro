# _e24_drain1_repeat.py — confirmă vizual că drain=1 nu produce artefacte.
#
# E23 a salvat 20 de JPEG-uri clean la drain=1 (Phase 2, post-stop pipeline).
# Aici repetăm capturea alternantă drain=1 în mai multe runde, cu un mic
# re-init pipeline între runde, ca să prindem eventuale artefacte rare
# (tearing, cross-cam bleed) care nu apar la prima rulare.
#
# Fiecare rundă:
#   - pornește pipeline cu ratio(1,1) timp de ~30 frame-uri (stress cu
#     flip-uri EOF-ISR back-to-back) — nu capturăm frame-urile astea
#   - oprește pipeline
#   - capturează 10 perechi (cam0, cam1) JPEG la drain=1 pentru inspecție
#
# Output: /diags/sNNN_e24_drain1/{round_N_drain1_frames}/cam{0,1}_...jpg

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
    begin("e24_drain1")

ROUNDS = 4
STRESS_FRAMES = 30
PAIRS_PER_ROUND = 10
QUALITY = 70

csv_path = _save_path("e24_drain1_repeat")
all_dts = []; all_szs = []; all_cams = []; all_rounds = []
totals = {"saved": 0, "fallback": 0, "drain_to": 0, "ok_eof": 0}

for rnd in range(ROUNDS):
    print("[E24] round %d/%d" % (rnd + 1, ROUNDS))
    # Phase A: stress — pipeline paralel ratio(1,1) drain=1 ~30 frames.
    sentai.camera.ratio(1, 1)
    sentai.camera.select(0)
    sentai.pipeline.prep_reset()
    cs0 = sentai.diag.cam_stats()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)
    stress_ok = 0
    for _ in range(STRESS_FRAMES):
        r = sentai.pipeline.get_ex(2000)
        if r is not None:
            stress_ok += 1
    sentai.pipeline.stop()
    sentai.camera.ratio(0, 0)

    cs1 = sentai.diag.cam_stats()
    d_ok = cs1["switch_ok_eof"] - cs0["switch_ok_eof"]
    d_fb = cs1["switch_fallback"] - cs0["switch_fallback"]
    d_to = cs1["drain_timeout"] - cs0["drain_timeout"]
    totals["ok_eof"] += d_ok
    totals["fallback"] += d_fb
    totals["drain_to"] += d_to
    print("  stress: %d/%d frames, switches: ok_eof=%d fb=%d to=%d" %
          (stress_ok, STRESS_FRAMES, d_ok, d_fb, d_to))

    # Phase B: manual alternant, salvăm JPEG pentru inspecție.
    frames_dir = _photos_dir("e24_r%d_drain1" % rnd)
    if frames_dir:
        try: sentai.fs.mkdir(frames_dir)
        except Exception: pass
    else:
        frames_dir = "/diags/e24_r%d" % rnd
        try: sentai.fs.mkdir(frames_dir)
        except Exception: pass

    # warmup fiecare cam o dată (retry pe miss).
    def _warm(cam):
        sentai.camera.select(cam)
        for _ in range(5):
            try: sentai.camera.jpeg(60); return True
            except RuntimeError: pass
        return False
    if not (_warm(0) and _warm(1)):
        print("E24 round %d WARN: warmup failed, skip" % rnd)
        continue

    saved_round = 0
    for i in range(PAIRS_PER_ROUND):
        for cam in (0, 1):
            ts = _ticks()
            sentai.camera.select(cam)
            jpg = sentai.camera.jpeg(QUALITY)
            dt = _ticks() - ts
            idx = i * 2 + cam
            path = "%s/r%d_%03d_cam%d_%dms_%db.jpg" % (
                frames_dir, rnd, idx, cam, dt, len(jpg))
            try:
                sentai.fs.write(path, jpg); saved_round += 1
                all_dts.append(dt); all_szs.append(len(jpg))
                all_cams.append(cam); all_rounds.append(rnd)
            except Exception as e:
                print(" WARN save", path, e)
            jpg = None
            gc.collect()
    totals["saved"] += saved_round
    print("  saved: %d/%d -> %s" % (saved_round, PAIRS_PER_ROUND * 2, frames_dir))

# Summary.
if all_dts:
    st = stats(all_dts)
    print()
    print("=== E24 drain=1 repeat SUMMARY ===")
    print(" rounds:         %d" % ROUNDS)
    print(" total saved:    %d / %d" % (totals["saved"], ROUNDS * PAIRS_PER_ROUND * 2))
    print(" capture ms:     min=%d mean=%.1f max=%d" % (st["min"], st["mean"], st["max"]))
    print(" jpeg bytes:     min=%d mean=%d max=%d" %
          (min(all_szs), sum(all_szs) // len(all_szs), max(all_szs)))
    print(" switches (cumulative across %d stress runs):" % ROUNDS)
    print("   ok_eof=%d  fallback=%d  drain_timeout=%d" %
          (totals["ok_eof"], totals["fallback"], totals["drain_to"]))

    save_csv(csv_path,
        ["round", "idx", "cam_id", "elapsed_ms", "jpeg_bytes"],
        [(all_rounds[i], i, all_cams[i], all_dts[i], all_szs[i])
         for i in range(len(all_dts))])
    _save_desc(csv_path,
        "E24 - drain=1 repeat stress + visual capture.\n"
        "Per round: %d frames pipeline ratio(1,1) stress, then %d alt JPEG pairs.\n"
        % (STRESS_FRAMES, PAIRS_PER_ROUND),
        params={"rounds": ROUNDS, "stress_frames": STRESS_FRAMES,
                "pairs_per_round": PAIRS_PER_ROUND, "quality": QUALITY,
                "switch_drain": 1,
                "total_saved": totals["saved"],
                "cum_switch_ok_eof": totals["ok_eof"],
                "cum_switch_fallback": totals["fallback"],
                "cum_drain_timeout": totals["drain_to"]})
    _record("e24_drain1_repeat", csv_path,
            "rounds=%d saved=%d fb=%d to=%d" %
            (ROUNDS, totals["saved"], totals["fallback"], totals["drain_to"]))

if owned:
    end()
