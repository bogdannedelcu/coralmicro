/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CAMERA_SUPPORT_H_
#define _CAMERA_SUPPORT_H_

#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef CPU_MIMXRT1176CVM8A_cm4
// Not really used
#define DEMO_CAMERA_HEIGHT        240
#define DEMO_CAMERA_WIDTH         320
#define DEMO_CAMERA_BUFFER_COUNT  2
#else

// Choose here your resolution and number of FBs
#define DEMO_CAMERA_HEIGHT  720
#define DEMO_CAMERA_WIDTH   1280
#define DEMO_CAMERA_BUFFER_COUNT 4

//#define DEMO_CAMERA_HEIGHT  240
//#define DEMO_CAMERA_WIDTH   320
//#define DEMO_CAMERA_BUFFER_COUNT 3

// #define DEMO_CAMERA_HEIGHT  1080
// #define DEMO_CAMERA_WIDTH   1920
// #define DEMO_CAMERA_BUFFER_COUNT 2

#endif
// 30 fps at 720p is the highest tuple in the NXP driver's csi2rxHsSettle
// lookup (fsl_csi_support.c) and well within OV5640's 720p-mode ceiling
// (datasheet: up to 45 fps in the 1280×720 subsample mode).  Halving the
// frame period from 67 ms to 33 ms cuts the post-switch drain cost
// in proportion: threshold=2 drops from ~134 ms to ~67 ms of wait,
// and the mid-buffer MUX-flip artifact window (the tearing that E17 at
// threshold=1 exposed) is also halved in wall time.
// 30 fps @ 720p — highest rate that works out-of-the-box with the NXP
// OV5640 driver on this board.  Experiments:
//   - 45 fps: not in any NXP clock table (datasheet lists 45 only at
//     1280×960, not 720p).  CAMERA_DEVICE_Init returned 4, 0 frames/s.
//   - 60 fps: OV5640 datasheet lists 720p/60 via 2×2 binning.  I added
//     a PLL entry (sys_div=1) and tHsSettle=0x09 to the CSI2RX table;
//     init accepted the PLL (ret=0) but CSI-2 never received frames
//     (fps_measured=0).  Likely needs OV5640 binning-mode register
//     programming that the NXP driver does not currently emit for this
//     resolution + a D-PHY timing re-tune — out of scope here.
#define DEMO_CAMERA_FRAME_RATE    30
#define DEMO_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)
#define DEMO_CAMERA_BUFFER_ALIGN  64
#define DEMO_CAMERA_MIPI_CSI_LANE 2
#define DEMO_CAMERA_BUFFER_BPP 4

#define LINE_PADDING              0

#define DEMO_BUFFER_WIDTH  DEMO_CAMERA_WIDTH
#define DEMO_BUFFER_HEIGHT DEMO_CAMERA_HEIGHT
#define DEMO_PXP PXP

extern camera_device_handle_t cameraDevice;
extern camera_receiver_handle_t cameraReceiver;

/* Monotonic frame counter (incremented in CSI ISR, never reset). */
extern volatile uint32_t g_camera_frame_seq;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/* This function should be called before camera pins initialization */
void BOARD_EarlyInitCamera(void);

void BOARD_InitCameraResource(void);

void BOARD_InitMipiCsi(void);

status_t BOARD_Camera_I2C_SendSCCB(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize);

status_t BOARD_Camera_I2C_ReceiveSCCB(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);

void BOARD_PullCameraResetPin(bool pullUp);

void BOARD_PullCameraPowerDownPin(bool pullUp);

void BOARD_InitPxp(void);
void BOARD_InitCamera(void);
void BOARD_PxpConfig(void);

uint8_t* IndexToFramebufferPtr(int index);
int FramebufferPtrToIndex(const uint8_t* framebuffer_ptr);

void CamDumpRegisters(void);
void CamDumpRegistersOnly(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CAMERA_SUPPORT_H_ */
