---
name: VGA fps bench — VGA30 done, VGA45/60 init blocked
description: 2026-04-26 cam_id+timing matrix at VGA30 fully validated (100/100 across 5 modes). VGA45/60 builds wedge at warm-up select(), separate from cam_id work.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
State of the VGA framerate bench matrix (build #953-#955):

**VGA30 (current default, REPRODUCIBLE):**
- All 5 modes (single_cam0, single_cam1, alt_1_1, alt_2_1, alt_3_1) hit 100/100 cam_id correct, 0 scrambled, 0 wrong-tag.
- fps: 30 single, 13 alt 1:1, 18 alt 2:1, 21 alt 3:1.  Math checks out (1 dirty buffer per MUX flip).
- Driver: `diag/_t_vga_bench.py`.  Table in `agent/experiment.md` "VGA30 timing + cam_id table".

**VGA45 / VGA60: BLOCKED on init wedge.**
- Build with `DEMO_CAMERA_FRAME_RATE = 45` (or 60), flash.  Boot.
- Diag driver's warm-up `select()` calls trigger `E:0A01` (CAM_SWITCH_FALLBACK) → `E:0A02:300` (drain timeout) within seconds.
- REPL goes silent; WDOG recovers in ~3 min.
- Removing the warm-up `select()` calls makes `peek5_b40()` itself hang (no frame flowing).
- Not a regression of the dirty-skip work — it was validated at VGA30 and is fps-independent in mechanism.

**To unblock VGA45/60:**
1. Rebuild with `DEMO_CAMERA_FRAME_RATE = 45`.
2. Verify `peek5_b40()` returns from a single camera AT INIT before any `select()` calls.  If not, the OV5640 PLL config for 45fps may need adjustment in fsl_ov5640.c.
3. If single-cam fps45 works: instrument the warm-up `select()` path with `sentai.diag.cam_stats()` between each call to find which transition stalls.
4. The `sentai.camera.set_hw(w, h, fps)` binding referenced in older drivers (`diag/alt_fps_matrix.py`) does NOT exist on this branch.  Add it cleanly if runtime fps switching is needed; don't `try/except` around AttributeError.

**Don't repeat (lessons):**
- Don't trust memory notes claiming `set_hw` is shipped — it's not.  Grep the live MP bindings before relying on an API.
- The OV5640 driver clock-tree table (`fsl_ov5640.c`) has VGA{15,30,45,60,90} entries.  Numbers outside this set fall through to defaults that may or may not produce stable frames.
- The dirty-skip / cam_id 100% mechanism is fps-independent.  Once VGA45/60 init wedges are fixed, the same `_t_vga_bench.py` should produce 100% on the upper rows of the table.
