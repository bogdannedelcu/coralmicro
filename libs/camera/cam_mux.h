/*
 * cam_mux.h — single source of truth for the dual-camera analogue MUX
 * polarity on the SentAI board.
 *
 * The board multiplexes two OV5640 sensors onto one MIPI-CSI2 lane via
 * a GPIO-controlled analogue switch (Gpio::kCamMux).  Which logic level
 * selects which sensor is determined by the board schematic — front is
 * wired to drive GPIO high, back to GPIO low.  These values are used
 * from two compilation units:
 *
 *   - libs/camera/camera.cc  (task-context synchronous fallback flip
 *     inside CameraTask::HandleSwitchCameraRequest)
 *   - libs/camera/camera_support.c  (ISR-context glitch-free flip
 *     inside CSI_IRQHandler, via SentaiCamMuxSetFromIsr)
 *
 * Keeping the constants in one header ensures the two paths agree.
 * Updating a single value here flips both call sites consistently;
 * forgetting to update one would cause "switch occasionally goes to the
 * wrong camera" — a failure mode that is costly to diagnose without a
 * unified definition.  Per embeded.md §J (complexity control).
 */

#ifndef SENTAI_CAM_MUX_H_
#define SENTAI_CAM_MUX_H_

/* GPIO level that selects the front (cam0) sensor. */
#define CAM_MUX_LEVEL_FRONT  1
/* GPIO level that selects the back (cam1) sensor. */
#define CAM_MUX_LEVEL_BACK   0

/* Canonical mapping: cam_id (0=front, 1=back) → GPIO level.
 * Implemented as a macro so it works in both C and C++ without
 * introducing ABI-level functions across translation units. */
#define CAM_MUX_LEVEL_FOR_ID(id)  ((id) == 0 ? CAM_MUX_LEVEL_FRONT \
                                             : CAM_MUX_LEVEL_BACK)

#endif /* SENTAI_CAM_MUX_H_ */
