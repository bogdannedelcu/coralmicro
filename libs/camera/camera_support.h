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
// #define DEMO_CAMERA_HEIGHT  720
// #define DEMO_CAMERA_WIDTH   1280
// #define DEMO_CAMERA_BUFFER_COUNT 3

#define DEMO_CAMERA_HEIGHT  240
#define DEMO_CAMERA_WIDTH   320
#define DEMO_CAMERA_BUFFER_COUNT 3

// #define DEMO_CAMERA_HEIGHT  1080
// #define DEMO_CAMERA_WIDTH   1920
// #define DEMO_CAMERA_BUFFER_COUNT 2

#endif
#define DEMO_CAMERA_FRAME_RATE    15
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
