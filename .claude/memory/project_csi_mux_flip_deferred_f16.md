---
name: csi-mux-flip-deferred-f16
description: Dual-camera MUX flip must NOT run inside the CSI FB-done ISR (races the old sensor's MIPI Frame-End packet -> CSI2RX wedge, audit F16); it is deferred 40 us on GPT1 OCR2 since build 1664. Phase handling = init pre-trim + cam_tune closed loop.
metadata:
  type: project
---

**Root cause of the CSI input wedge (audit F16, EXP-s251 iter12-20, 2026-10-31):**
a MUX flip executed at CSI DMA-done races the OLD sensor's MIPI Frame-End
short packet (a few us after the last line).  Cutting that burst leaves the
CSI2RX/D-PHY in an error state (IRQ_STATUS 0x13D: CRC, 2-bit ECC, SoT,
SoT-sync, ERRCONTROL; CSI CR3.FRMCNT frozen) that survives sensor standby
toggles, MUX switches and camera.stop()+init(); only a SoC reset recovers.
The ISR's own latency decided the race: SDK patch 0005 (F15, side-array
empty list) removed one SDRAM read and turned 1-in-8 wedges into 8-in-9;
s246's "an XOR + store in CSI_IRQHandler broke alternation" was the same
race.  The inter-sensor phase was NOT the deciding factor.

**Fix (build 1664):** CSI_IRQHandler arms GPT1 output-compare 2 at
CNT + g_cam_flip_delay_us (default 40) and `sentai_cam_deferred_flip_isr()`
(camera_support.c, ITCM, called from GPT1_IRQHandler in libs/base/timer.cc,
GPT1_IRQn at CSI priority 5) performs the flip.  `sentai.camera.flip_delay_us()`;
`stats()["flips_deferred"/"flips_executed"]`.  Validated: 3/3 cold boots clean
with the hold, 0/2 without, same build.

**Phase handling (operator goal: no initial calibration, self-align on the
fly):** `camera.init()` pre-trims cam0 by one SCCB write (kSccbStartGapUs 410
-> 18 lines at VGA45, the deterministic start-order offset, no MUX flip;
`genlock_auto` mode 1 default, mode 2 adds the measured `genlock()`); the
watchdog phase monitor reads phi from the production flips; two consecutive
|phi| > 270 us samples re-arm dirty-skip and wake `cam_tune`, which applies
one blind frame-synchronised VTS trim (edges predicted from the FB-done log,
no MUX flip), rate-limited 10 s.  Verified: injected +1776 us -> back to
+10 us in ~15 s, 44 fps.  Residual after pre-trim: -12..-35 us across boots.

**Why:** the 15-min soak showed the phase is pinned (5-6 us) because both
sensors share the 24 MHz XCLK; only register writes / restarts move it.
**How to apply:** never add work or flips inside CSI_IRQHandler without a
cold-boot alternation A/B (`experiments/s251_camera_genlock/alt_only_check.py`);
keep `flip_delay_us` > 0 in production; patch 0005 is innocent and stays.
Related: [[project_fb2_counter_fix]] (CSI ISR timing-sensitive),
[[project_todo_exhaustive_code_audit]] (W23-T9).

**T14 (build 1684):** `g_cam_ratio_unit` 1 = per sensor frame (true
cam0/cam1 interleave, 31/31 transitions at 44 fps), 2 = legacy FB2 pairs.
Armed =1 by pretrim/genlock; the drift monitor sets 2 + dirty while
protecting and restores 1.  Phase estimator: pairs -> steady intervals = T;
per-frame -> consecutive-interval sums = 2T.  T11 soaks (5 + 10 min) PASS
with mode 2 default.
