# Audit — compile-time VGA geometry in runtime-geometry code paths (2026-11-02)

**WBS**: OP-S10-W24-T20 (fixes), triggered by EXP-s266 (W24-T19).  **Status**: all class-A
findings fixed in builds 1762–1763 (2026-11-02, see "Fixes applied" at the end; smoke test
EXP-s267).  Produced by the `embeded-reviewer` agent, read-only, after two bugs of this class
were found by hand (`sentai_cam_capture_jpeg` and `sentai_cam_peek_col`, fixed in builds
1757–1760).

Since OP-S10-W24-T11 the camera geometry is a runtime property: `g_runtime_cam_w/h` (640x480 or
1280x960, `camera.set_hw()` before `init()`) and `g_sentai_cam_packed420` (CSI stores
`[C,Y1,Y2,0]` per pixel pair, pitch `(w/2)*4`).  Any code that still uses `DEMO_CAMERA_WIDTH`,
`DEMO_CAMERA_HEIGHT`, `LINE_PADDING`-based pitch or VGA-sized static buffers as the *source*
geometry is wrong at SXGA or in packed mode.

Key facts established by the audit:

- `g_runtime_cam_w/h` (camera_support.c:83-84) drive the receiver, the pitch (`w*4`, or `(w/2)*4`
  when `g_sentai_csi_wdiv == 2`) and the D-PHY; `framebuffers[]` are sized `CAM_MAX`, so
  wrong-geometry readers stay inside the 4.9 MB slot (no hard fault) but produce wrong results.
- The `NonCacheableCamera` section (`m_ncamera`, 0x82000000) is **not** non-cacheable: MPU
  region 9 maps all 64 MB of SDRAM write-back (board.c:165-166) and the non-cacheable region 10 is
  `m_ncache` at 0x20000000.  CPU readers of frame buffers therefore depend on the explicit
  `DCACHE_*ByRange` calls in `libs/camera/camera.cc`, which are still VGA-sized (bugs 1–2).
- `sentai_camera_frame_backend.c` and `modsentai_camera_sim.c` are SIM/EMU only;
  `modules/sentai/modsentai_camera.c` is a dead copy not in any CMakeLists.

## Findings (BUGs first)

| file:line | symbol | class | failure at SXGA / packed | suggested fix |
|---|---|---|---|---|
| libs/camera/camera.cc:1159-1160 | `CameraTask::HandleFrameRequest` GET → `DCACHE_InvalidateByRange(buffer, 480*640*4)` | **A** | Invalidates 1 228 800 B = rows 0..239 of an SXGA XYUV frame (or the first half of a packed SXGA frame); rows 240+ are cacheable SDRAM → every CPU reader (peek_col, peek5_*, save_raw, chase memcmp) can see stale lines of an older frame in the same slot; intermittent, undetectable by the reader | invalidate `g_runtime_cam_h * pitch` (or `CAM_MAX_HEIGHT*CAM_MAX_WIDTH*4`), or make `m_ncamera` a real MPU non-cacheable region |
| libs/camera/camera.cc:1231-1232 | RETURN → `DCACHE_CleanInvalidateByRange(buffer, VGA bytes)` | **A** | dirty CPU lines above row 239 are not written back / dropped before the slot is re-armed; a later eviction can overwrite fresh CSI DMA data | same |
| examples/sentai_runtime/detection_task.cc:813-815, 820-821, 837-838 | `publish_aux_slots_from_raw` → `sentai_pxp_xrgb_to_y8 / sentai_prep_publish_slot_rgb_64 / sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, …)` | **A** | at SXGA the PXP is told pitch 2560 for a 5120-B row: GRAY_NATIVE (aruco/whycon/markers/calib), RGB_64 (slam/places/gist), FLOW_GRAY_80x60 are built from the top-left window read as interleaved half-rows → garbage with rc 0; packed420 raw is not XRGB → garbage; reachable via pipeline start at SXGA and `camera.prep_once()` | pass `g_runtime_cam_w/h`; skip or add a packed→Y8 producer when packed |
| detection_task.cc:1271-1272 | PrepTask single-input (RGB) model path `sentai_pxp_scale(raw, DEMO_*, dst, w, h)` | **A** | RGB model + SXGA camera → tensor built with pitch 2560 → garbage detections, no error; only the dual-input YUV path is runtime-aware; no guard refuses RGB models at SXGA or packed | runtime dims; `SERR_LOG` + health fail when packed and model is RGB |
| examples/sentai_runtime/flow_task.cc:946-947 | publisher_task → `sentai_flow_publish_frame(raw, DEMO_*, …)` | **A** | standalone flow at SXGA scales the wrong-pitch window → dx/dy noise with normal confidence; packed: not XRGB | runtime dims; refuse `flow.start` when packed |
| sentai_runtime.cc:4677-4683 | `sentai_cam_capture_rgb` | **A** | `src_w/h = DEMO_*` → interleaved half-row image, rc 0; no packed guard | `g_runtime_cam_w/h`; `-7` when packed (mirror capture_jpeg) |
| sentai_runtime.cc:4821-4827 | `sentai_cam_to_tensor_ex` (`camera.to_tensor()`) | **A** | same wrong pitch → TPU tensor garbage at SXGA; no packed guard | same |
| sentai_runtime.cc:5434-5440 | `sentai_aifes_capture_camera` (libs/aifes/aifes_task.cc:1035) | **A** | same wrong pitch; the `w > src_w` clamp uses the VGA constant | same |
| sentai_runtime.cc:4736-4748 | `sentai_cam_capture_jpeg_yuv` (`camera.jpeg(q, yuv=1)`) | **A + D** | VGA constants: at SXGA JPEG of interleaved half rows returned as success; taken in packed mode because `yuv_mode` is on → garbage; planes `s_jpeg_y/cbcr/cb/cr` (4704-4711) are VGA-sized, so switching to runtime dims naively would overflow | guard `g_runtime_cam_w != 640 → -5`, packed → `-7`; or CAM_MAX planes in `m_ncamera` (+1.8 MB) with runtime dims |
| sentai_runtime.cc:2617-2627 | `sentai_cam_peek5_b40` | **A** | pitch 2560 / VGA rows: at SXGA samples rows {0,60,120,180,239.5}; in packed VGA 3 of 5 samples fall outside the live frame (stale bytes) | runtime dims + packed-aware pitch as in `peek_col_byte` |
| sentai_runtime.cc:2668-2678 | `sentai_cam_peek5_tags` | **A** | identical | same |
| sentai_runtime.cc:3578-3583 | `sentai_yuv420_extract_test` (modsentai_camera.c:347-352) | **A + D** | hard-coded 640x480 and VGA static planes: garbage dumps at SXGA with rc 0; packed not guarded | error unless `640 && !packed`, or runtime dims + CAM_MAX planes |
| sentai_runtime.cc:5417-5424 | `sentai_cam_get_native_width/height` → `CameraTask::kWidth/kHeight` | **A** (API) | `camera.native_res()` reports (640,480) at SXGA; host scripts sizing buffers/homographies from it are misled | return `g_runtime_cam_w/h` |
| detection_task.cc:935-939 | prep-only mode `s_stg_w/h/total = DEMO_*` | **A** (stats) | staging metadata reports 640x480 at SXGA | runtime dims |
| sentai_runtime.cc:3065-3069 (+ detection_task.cc:207-208, modsentai_camera.c:1100-1106, :88-92) | `sentai_cam_set_res` lets `g_cam_width/height` < sensor geometry while `sentai_cam_get_width()` is used as the *source* pitch by the YUV pipeline, `grab_gray_zerocopy` and the jpeg bindings | **A** (latent) | `set_resolution(320,240)` then any of those paths → PXP pitch 1280 on a 2560/5120-B row; at SXGA the Y_SIZE check fails loudly, at VGA it silently corrupts | source geometry from `g_runtime_cam_w/h` only; keep `g_cam_width` for output size (or drop set_res) |
| camera_support.c:1264, 1284 | `CamDumpAllRegisters / CamDumpRegisters` printf `DEMO_*` | C | log says 640x480 at SXGA | print runtime dims |
| camera.cc:808 | `CameraTask::TaskInit` printf | C | boot log before set_hw | — |
| camera.cc:833-844 `VideoConvert`, 373-401 `RotateXY`, 516-518 `GetFrame(fmts)`, 1290 | upstream helpers | C | no SentAI callers | leave; dead upstream code |
| camera_support.c:1194-1196 | `BOARD_PxpConfig` PS scaler VGA→VGA | C | every SentAI PXP helper reprograms the scaler per call | — |
| sentai_runtime.cc:3696 | `kActiveRows = 960` for the VBLANK estimate | C (verify) | VTS counts 2x-binned rows at VGA and SXGA (~984) | confirm 0x380E/F at SXGA |
| sentai_runtime.cc:4767-4775 | `sentai_cam_capture_jpeg` | B | fixed (1757+): runtime dims, -5/-6/-7; bindings halve SXGA | — |
| sentai_runtime.cc:2706-2725 | `sentai_cam_peek_col_byte` | B | fixed; still exposed to bug 1 (rows ≥ 240 not invalidated) | — |
| sentai_runtime.cc:3310-3318, 3491-3494 | `sentai_pxp_chase_test`, `sentai_cam_chase_begin` | B | runtime w/h, packed-aware pitch | — |
| detection_task.cc:1203-1267, 545-548, 216-219, 558-566 | YUV dual path, `ocram_ring_set`, `yuv_in_buf`, `staging_buf` | B | runtime, size-checked | — |
| sentai_runtime.cc:2580-2595 / 2554-2573 | `save_raw`, `peek_first_row` | C | pitch-independent | — |
| sentai_runtime.cc:4693 | `s_jpeg_rgb_buf[640*480*3]` | D (clamped) | caller `-6` + binding halving | — |
| sentai_runtime.cc:4704-4711 / 3578-3579 | `s_jpeg_y/cbcr/cb/cr`, `s_y/s_uv` | D (not clamped) | see the two rows above | — |
| modsentai_camera.c:656-658 | `col[1024]`, rows ≤ 1024 | B | clamped | — |
| detection_task.cc:172, 1043 | `kMaxStagingSize = 640*480*3` | B | tensor bound, checked | — |
| sentai_prep.cc:44-45, 101; sentai_fr.cc:347; sentai_gist.cc:87-89; aruco_bench.cc:150-151 | 640x480 / 320x240 bounds | B/C | output-slot sizes, explicit checks | — |
| sentai_virtual_camera.cc:30-31, 145-146 | `kMaxW/H = 640x480` | C | virtual frames VGA by construction | — |
| detection_task.cc:1164-1166, 1290-1292 | `sentai_flow_m4_publish_frame(raw, DEMO_*)` | C | stub ignores args | could delete |

## Summary

1. **Cache maintenance is VGA-sized** (camera.cc:1159, :1231): only rows 0..239 of an SXGA
   frame are invalidated / cleaned; every CPU reader of rows ≥ 240 can get stale data.  Highest
   impact, hardest to see.  Fix first (one-line size change, or a real MPU region).
2. **Pipeline aux slots** (detection_task.cc:813-838): aruco/whycon/markers/calib, slam/places
   and prep-slot flow receive garbage at SXGA or packed, rc 0.
3. **RGB single-input model path** (detection_task.cc:1271) still VGA-pitch; no guard.
4. **Standalone flow** (flow_task.cc:946) → noise at SXGA / packed with normal confidence.
5. **Capture family**: `capture_rgb`, `to_tensor_ex`, `aifes_capture_camera` — wrong pitch, no
   packed guard.
6. **`capture_jpeg_yuv`**: VGA constants + VGA planes + taken in packed mode.
7. **Diag oracles** `peek5_b40` / `peek5_tags`, `yuv420_extract_test`: wrong rows / hard-coded.
8. **`native_res()`** returns the compile-time 640x480 at SXGA.
9. **Latent**: `set_resolution()` changes `g_cam_width`, used as *source* pitch by several paths.
10. Already runtime-aware: `capture_jpeg`, `peek_col_byte`, chase test/begin, YUV dual-input
    prep, OCRAM ring sizing, `save_raw`, `peek_first_row`.

Plan (W24-T20): fix 1 → 2 → 3/4 → 5/6 → 7/8/9 in one build, each with a REPL check at SXGA
legacy and SXGA packed (s266 `run_sxga.sh` + `camera.prep_once()` + `flow.start()` smoke), then
re-run the s266 SXGA cadence with `frame_log(64)`.

## Fixes applied (W24-T20, builds 1762–1763, 2026-11-02)

One source of truth in `sentai_runtime.cc` (top of file): `cam_src_w/h()` (virtual camera or
`g_runtime_cam_w/h`), `cam_src_packed()`, `cam_src_pitch()` (`(w/2)*4` packed, `w*4` otherwise).
`sentai_cam_get_width/height()` now return this SOURCE geometry; the `set_resolution()` output
size moved to `sentai_cam_get_out_width/height()` (used only by `jpeg()`, `save_jpeg()`,
`resolution()`).

| # | finding | fix |
|---|---|---|
| 1 | D-cache invalidate / clean VGA-sized (camera.cc) | `LiveFrameBytes()` = `g_runtime_cam_h * (g_runtime_cam_w / g_sentai_csi_wdiv) * 4`, clamped to the CAM_MAX slot; used by both maintenance calls |
| 2 | aux prep slots with VGA pitch | runtime `cam_src_w/h()`; in packed mode the producers are skipped and `SERR_DET_CAM_FORMAT` (0x0448, val 2) logged once; `grab_gray_zerocopy` fallback refuses packed (-1) |
| 3 | RGB single-input model path | runtime geometry for the PXP; `pipeline.start` refuses packed + RGB model with -9 / 0x0448 val 1; in-loop guard rc -23 |
| 4 | standalone flow publisher | runtime geometry (`flow_cam_src_w/h()`, SIM keeps VGA); `flow.start(0/1)` refused with -4 / `SERR_FLOW_CAM_FORMAT` (0x0E21) in packed mode |
| 5 | `capture_rgb`, `to_tensor_ex`, `aifes_capture_camera` | runtime geometry, -7 when packed, -5 when the output exceeds the source |
| 6 | `capture_jpeg_yuv` | -7 packed, -5 when the sensor geometry is not 640x480 (planes are VGA) |
| 7 | `peek5_b40` / `peek5_tags`, `yuv420_extract_test` | runtime rows + pitch; the YUV test returns -5 / -7 outside VGA legacy and now returns the grabbed frame |
| 8 | `native_res()` | `cam_src_w/h()` |
| 9 | `set_resolution()` as source pitch | output-only (see above); `CamDumpRegisters` print runtime dims |

Build gotchas: the cm4 TU of `camera.cc` has no `CAM_MAX_*` (fallback literals); the
`"NonCacheableCamera,\"aw\",%nobits @"` section trick on `s_tpu_input_buf_sxga` broke the SIM
compile of `detection_task.cc` since W24-T11 (guarded).  The SIM link itself is still broken by
other W24 ARM-only symbols (TPU URB stats, chase, pack420_drop, sysfs, imu, health, and the
camera geometry symbols) — tracked as OP-S10-W24-T21, not part of this fix.

Verification: EXP-s267 `smoke.py` (VGA30 legacy, SXGA15 legacy, SXGA30 packed; results in its
README) and the s266 SXGA time-code re-run for the cache fix (#1).
