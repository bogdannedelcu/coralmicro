/*
 * cam_mux.h — single source of truth for the dual-camera analogue MUX
 * polarity on the SentAI board.
 *
 * The board multiplexes two OV5640 sensors onto one MIPI-CSI2 lane via
 * a GPIO-controlled analogue switch (Gpio::kCamMux).  Which logic level
 * selects which sensor is determined by the board schematic.  These
 * values are used from two compilation units:
 *
 *   - libs/camera/camera.cc        (task-context synchronous fallback
 *     flip inside CameraTask::HandleSwitchCameraRequest)
 *   - libs/camera/camera_support.c (ISR-context glitch-free flip inside
 *     CSI_IRQHandler, via SentaiCamMuxSetFromIsr)
 *
 * Keeping the constants in one header ensures the two paths agree.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CONVENTION (adopted 2026-04-26 — DO NOT REINTERPRET)
 * ═══════════════════════════════════════════════════════════════════
 *
 * On this board, "cam_id" is a 1-bit logical handle that maps 1:1 to
 * BOTH the sensor's I²C bus AND the analog-MUX GPIO level:
 *
 *      cam_id == 0  ≡  sensor on LPI2C1 (CameraTask::i2c_handle_)
 *                   ≡  GPIO kCamMux == 0
 *
 *      cam_id == 1  ≡  sensor on LPI2C2 (CameraTask::i2c_handle2_)
 *                   ≡  GPIO kCamMux == 1
 *
 * The GPIO level numerically equals cam_id.  This is intentional —
 * pick this convention once, here, so that any code in the firmware
 * (or any future agent reading the firmware) can rely on:
 *
 *      "writing OV5640 register R on cam_id N (via WriteToCam) and
 *       then issuing select(N) is GUARANTEED to read back from the
 *       SAME physical sensor."
 *
 * The original drop of cam_mux.h had this inverted (LEVEL_FRONT=1,
 * LEVEL_BACK=0).  That made test_pattern(0,...) inject a pattern on
 * sensor X while select(0) routed CSI from sensor (1-X) — a silent
 * routing inversion only catchable with synthetic patterns.  See the
 * empirical proof in:
 *
 *      examples/sentai_runtime/diag/_t_pattern_31.py
 *      /diags/s101_pattern_31/log.csv  (on the device LFS)
 *      /diags/s102_pattern_31/log.csv
 *
 * and the writeup in agent/agent.md §"MUX / I²C polarity convention".
 *
 * NOTE — physical orientation:
 *   The board has two OV5640 footprints labelled (somewhat informally)
 *   FRONT and BACK on the silkscreen of older revisions.  Which logical
 *   cam_id corresponds to which physical footprint is a *board-level*
 *   choice that depends on how the schematic wires LPI2C1 / LPI2C2 and
 *   the MUX inputs.  The legacy macro names CAM_MUX_LEVEL_FRONT /
 *   CAM_MUX_LEVEL_BACK kept below are *aliases for cam_id GPIO levels*,
 *   NOT statements about which physical lens you are looking out of.
 *   New code should prefer the cam_id-based names.
 *
 * Per embeded.md §J (complexity control): updating a single value here
 * flips both call sites consistently; forgetting to update one would
 * cause "switch occasionally goes to the wrong camera" — a failure
 * mode that is costly to diagnose without a unified definition.
 */

#ifndef SENTAI_CAM_MUX_H_
#define SENTAI_CAM_MUX_H_

/* ─── Primary names — prefer in new code ───────────────────────────
 *  GPIO logic level on Gpio::kCamMux that selects each cam_id. */
#define CAM_MUX_LEVEL_FOR_CAM0   0
#define CAM_MUX_LEVEL_FOR_CAM1   1

/* Canonical mapping: cam_id (0 / 1) → GPIO level. */
#define CAM_MUX_LEVEL_FOR_ID(id) ((id) == 0 ? CAM_MUX_LEVEL_FOR_CAM0 \
                                            : CAM_MUX_LEVEL_FOR_CAM1)

/* ─── Legacy aliases — DEPRECATED ──────────────────────────────────
 *  Pre-2026-04-26 names.  Kept so paper/cam_switch.md, the markdown
 *  history, and a handful of call-sites in camera.cc keep compiling.
 *
 *  WARNING for any future reader (human or LLM):
 *   • These names DO NOT mean "the front-of-board lens" / "the
 *     back-of-board lens" in any portable sense.  Treat them as
 *     spelling-equivalent to CAM_MUX_LEVEL_FOR_CAM0 / _CAM1.
 *   • Do NOT write `CAM_MUX_LEVEL_FRONT = 1` in a hardware-spin patch
 *     thinking "front camera is at HIGH level on the new board".  If
 *     the physical sensor on a new board moves between footprints,
 *     fix the schematic-side wiring, not these macros.
 *   • If for some unavoidable reason you DO need to flip polarity in
 *     firmware, change BOTH macros below in lock-step and re-run the
 *     diag/_t_pattern_31.py PHASE A baseline (the test that caught
 *     the original inversion). */
#define CAM_MUX_LEVEL_FRONT      CAM_MUX_LEVEL_FOR_CAM0
#define CAM_MUX_LEVEL_BACK       CAM_MUX_LEVEL_FOR_CAM1

#endif /* SENTAI_CAM_MUX_H_ */
