/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "camera_support.h"
#include "fsl_gpio.h"
#include "fsl_csi.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_ov5640.h"
#include "fsl_mipi_csi2rx.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_pxp.h"

extern volatile int g_sentai_debug;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_CSI_CLK_FREQ          (CLOCK_GetRootClockFreq(kCLOCK_Root_Bus))
#define DEMO_MIPI_CSI2_UI_CLK_FREQ (CLOCK_GetRootClockFreq(kCLOCK_Root_Csi2_Ui))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t BOARD_VerifyCameraClockSource(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Camera connect to CSI. */
static csi_resource_t csiResource = {
    .csiBase = CSI,
    .dataBus = kCSI_DataBus24Bit,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t cameraReceiver = {
    .resource    = &csiResource,
    .ops         = &csi_ops,
    .privateData = &csiPrivateData,
};

static ov5640_resource_t ov5640Resource = {
    .i2cSendFunc      = BOARD_Camera_I2C_SendSCCB,
    .i2cReceiveFunc   = BOARD_Camera_I2C_ReceiveSCCB,
    .pullResetPin     = BOARD_PullCameraResetPin,
    .pullPowerDownPin = BOARD_PullCameraPowerDownPin,
};

camera_device_handle_t cameraDevice = {
    .resource = &ov5640Resource,
    .ops      = &ov5640_ops,
};

#ifdef CPU_MIMXRT1176CVM8A_cm4
__attribute__((section(".sdram_bss,\"aw\",%nobits @")))
__attribute__((aligned(DEMO_CAMERA_BUFFER_ALIGN)))
uint8_t
    framebuffers[DEMO_CAMERA_BUFFER_COUNT][DEMO_CAMERA_HEIGHT][(DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP];
#else
__attribute__((section("NonCacheableCamera,\"aw\",%nobits @")))
__attribute__((aligned(DEMO_CAMERA_BUFFER_ALIGN)))
uint8_t
    framebuffers[DEMO_CAMERA_BUFFER_COUNT]
        [(DEMO_CAMERA_HEIGHT) * (DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP];
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void CSI_DriverIRQHandler(void);

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();
}

void BOARD_EarlyInitCamera(void)
{
    /* If the camera I2C bus should be released by sending I2C sequence,
     * add the code here.
     */
}

void BOARD_InitCameraResource(void)
{
    // BOARD_Camera_I2C_Init();

    /* CSI MCLK is connect to dedicated 24M OSC, so don't need to configure it. */
}

void BOARD_InitMipiCsi(void)
{
    csi2rx_config_t csi2rxConfig = {0};

    /* This clock should be equal or faster than the receive byte clock,
     * D0_HS_BYTE_CLKD, from the RX DPHY. For this board, there are two
     * data lanes, the MIPI CSI pixel format is 16-bit per pixel, the
     * max resolution supported is 720*1280@30Hz, so the MIPI CSI2 clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2ClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    /* ESC clock should be in the range of 60~80 MHz */
    const clock_root_config_t csi2EscClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    /* UI clock should be equal or faster than the input pixel clock.
     * The camera max resolution supported is 720*1280@30Hz, so this clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2UiClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    if (kStatus_Success != BOARD_VerifyCameraClockSource())
    {
        PRINTF("MIPI CSI clock source not valid\r\n");
        while (1)
        {
        }
    }

    /* MIPI CSI2 connect to CSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.SET = (VIDEO_MUX_VID_MUX_CTRL_CSI_SEL_MASK);

    CLOCK_SetRootClock(kCLOCK_Root_Csi2, &csi2ClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Esc, &csi2EscClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Ui, &csi2UiClockConfig);

    /* The CSI clock should be faster than MIPI CSI2 clk_ui. The CSI clock
     * is bus clock.
     */
    if (DEMO_CSI_CLK_FREQ < DEMO_MIPI_CSI2_UI_CLK_FREQ)
    {
        PRINTF("CSI clock should be faster than MIPI CSI2 ui clock.\r\n");
        while (1)
        {
        }
    }

    /* MIPI DPHY power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /*
     * Initialize the MIPI CSI2
     *
     * From D-PHY specification, the T-HSSETTLE should in the range of 85ns+6*UI to 145ns+10*UI
     * UI is Unit Interval, equal to the duration of any HS state on the Clock Lane
     *
     * T-HSSETTLE = csi2rxConfig.tHsSettle_EscClk * (Tperiod of RxClkInEsc)
     *
     * csi2rxConfig.tHsSettle_EscClk setting for camera:
     *
     *    Resolution  |  frame rate  |  T_HS_SETTLE
     *  =============================================
     *     720P       |     30       |     0x12
     *  ---------------------------------------------
     *     720P       |     15       |     0x17
     *  ---------------------------------------------
     *      VGA       |     30       |     0x1F
     *  ---------------------------------------------
     *      VGA       |     15       |     0x24
     *  ---------------------------------------------
     *     QVGA       |     30       |     0x1F
     *  ---------------------------------------------
     *     QVGA       |     15       |     0x24
     *  ---------------------------------------------
     */
    static const uint32_t csi2rxHsSettle[][3] = {
        {
            kVIDEO_Resolution1080P,
            15,
            0x12,
        },
        {
            kVIDEO_Resolution720P,
            30,
            0x12,
        },
        {
            kVIDEO_Resolution720P,
            15,
            0x17,
        },
        {
            kVIDEO_ResolutionVGA,
            30,
            0x1F,
        },
        {
            kVIDEO_ResolutionVGA,
            15,
            0x24,
        },
        {
            kVIDEO_ResolutionQVGA,
            30,
            0x1F,
        },
        {
            kVIDEO_ResolutionQVGA,
            15,
            0x24,
        },
    };

    csi2rxConfig.laneNum          = DEMO_CAMERA_MIPI_CSI_LANE;
    csi2rxConfig.tHsSettle_EscClk = 0x12;

    for (uint8_t i = 0; i < ARRAY_SIZE(csi2rxHsSettle); i++)
    {
        if ((FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT) == csi2rxHsSettle[i][0]) &&
            (csi2rxHsSettle[i][1] == DEMO_CAMERA_FRAME_RATE))
        {
            csi2rxConfig.tHsSettle_EscClk = csi2rxHsSettle[i][2];
            break;
        }
    }

    CSI2RX_Init(MIPI_CSI2RX, &csi2rxConfig);
}

static status_t BOARD_VerifyCameraClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;
    /*
     * The MIPI CSI clk_ui, clk_esc, and core_clk are all from
     * System PLL3 (PLL_480M). Verify the clock source to ensure
     * it is ready to use.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys3);

    if (480 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

void BOARD_InitPxp(void)
{
    /*
     * Configure the PXP for rotate and scale.
     */
    PXP_Init(DEMO_PXP);

    PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0U);

#if DEMO_ROTATE_FRAME
    PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_BUFFER_HEIGHT - 1U, DEMO_BUFFER_WIDTH - 1U);
#else
    PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_BUFFER_WIDTH - 1U, DEMO_BUFFER_HEIGHT - 1U);
#endif

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    PXP_EnableCsc1(DEMO_PXP, false);
}

void BOARD_InitCamera(void)
{
    status_t status;
    camera_config_t cameraConfig;

    memset(&cameraConfig, 0, sizeof(cameraConfig));

    BOARD_InitCameraResource();

    /* CSI input data bus is 24-bit, and save as XRGB8888.. */
    cameraConfig.pixelFormat                = kVIDEO_PixelFormatXRGB8888;
    cameraConfig.bytesPerPixel              = DEMO_CAMERA_BUFFER_BPP;
    cameraConfig.resolution                 = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
    cameraConfig.frameBufferLinePitch_Bytes = DEMO_CAMERA_WIDTH * DEMO_CAMERA_BUFFER_BPP;
    cameraConfig.interface                  = kCAMERA_InterfaceGatedClock;
    cameraConfig.controlFlags               = DEMO_CAMERA_CONTROL_FLAGS;
    cameraConfig.framePerSec                = DEMO_CAMERA_FRAME_RATE;

    status = CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
    if (g_sentai_debug) printf("CAMERA_RECEIVER_Init = %ld\r\n", status);

    BOARD_InitMipiCsi();

    cameraConfig.pixelFormat   = kVIDEO_PixelFormatRGB565;
    cameraConfig.bytesPerPixel = 2;
    cameraConfig.resolution    = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
    cameraConfig.interface     = kCAMERA_InterfaceMIPI;
    cameraConfig.controlFlags  = DEMO_CAMERA_CONTROL_FLAGS;
    cameraConfig.framePerSec   = DEMO_CAMERA_FRAME_RATE;
    cameraConfig.csiLanes      = DEMO_CAMERA_MIPI_CSI_LANE;

    status = CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);
    if (g_sentai_debug) printf("CAMERA_DEVICE_Init = %ld\r\n", status);

    status = CAMERA_DEVICE_Start(&cameraDevice);
    if (g_sentai_debug) printf("CAMERA_DEVICE_Start = %ld\r\n", status);

    /* Submit the empty frame buffers to buffer queue. */
    for (uint32_t i = 0; i < DEMO_CAMERA_BUFFER_COUNT; i++)
    {
        status = CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(framebuffers[i]));
        // printf("CAMERA_RECEIVER_SubmitEmptyBuffer (%08lX) = %ld\n", (uint32_t)framebuffers[i], status);
    }
}

void BOARD_PxpConfig(void)
{
    PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0);
    /* Rotate and scale the camera input to fit display output. */
#if DEMO_ROTATE_FRAME
    /* The PS rotate and scale could not work at the same time, so rotate the output. */
    PXP_SetRotateConfig(DEMO_PXP, kPXP_RotateOutputBuffer, kPXP_Rotate90, kPXP_FlipDisable);
    PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_BUFFER_HEIGHT, DEMO_BUFFER_WIDTH);
#else
    PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT);
#endif
}

uint8_t* IndexToFramebufferPtr(int index) {
  if (index < 0 || index >= DEMO_CAMERA_BUFFER_COUNT) {
    return NULL;
  }
  return (uint8_t*)(framebuffers[index]);
}

int FramebufferPtrToIndex(const uint8_t* framebuffer_ptr) {
  for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT; ++i) {
    if ((uint8_t*)(framebuffers[i]) == framebuffer_ptr) {
      return i;
    }
  }
  return -1;
}

typedef struct {
	uint16_t start;
	uint16_t end;
} reg_range_t;

void CamDumpRegistersOnly(void)
{
    uint8_t val;
    reg_range_t ov5640_regs[] = {
            {0x3000, 0x3052},
            {0x3100, 0x3108},
            {0x3200, 0x3211},
            {0x3400, 0x3406},
            {0x3500, 0x350D},
            {0x3600, 0x3606},
            {0x3800, 0x3821},
            {0x3A00, 0x3A25},
            {0x3B00, 0x3B0C},
            {0x3c00, 0x3c1e},
            {0x3d00, 0x3d21},
            {0x3f00, 0x3f02},
            {0x4000, 0x4033},
            {0x4201, 0x4202},
            {0x4300, 0x430d},
            {0x4400, 0x4431},
            {0x4600, 0x460d},
            {0x4709, 0x4745},
            {0x4800, 0x4837},
            {0x4900, 0x4902},
            {0x5000, 0x5063},
            {0x5180, 0x51d0},
            {0x5300, 0x530f},
            {0x5380, 0x538d},
            {0x5480, 0x5490},
            {0x5580, 0x558c},
            {0x5600, 0x5606},
            {0x5680, 0x56a2},
            {0x5800, 0x5849},
            {0x6000, 0x603f}
    };

    printf("Camera %dx%d@%d %d bits per pixel\r\n",
    DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_CAMERA_FRAME_RATE, DEMO_CAMERA_BUFFER_BPP * 8);

    for (int n=0; n<sizeof(ov5640_regs)/sizeof(ov5640_regs[0]); n++)
    {
        for (uint16_t reg = ov5640_regs[n].start; reg <= ov5640_regs[n].end; reg++)
        {
            status_t ret = BOARD_Camera_I2C_ReceiveSCCB(0x3c, reg, 2, &val, sizeof(val));
            printf ("0x%04X = 0x%02X (err: %ld)\r\n", reg, val, ret);
        }
    }
}

void CamDumpRegisters(void)
{
    uint16_t ov5640_regs[] = {
        0x3034, 0x3035, 0x3036, 0x3037, 0x4837, 0x3108
    };
    uint8_t val;

    printf("Camera %dx%d@%d %d bits per pixel\r\n",
    DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_CAMERA_FRAME_RATE, DEMO_CAMERA_BUFFER_BPP * 8);

    for (int n=0; n<sizeof(ov5640_regs)/sizeof(ov5640_regs[0]); n++)
    {
        status_t ret = BOARD_Camera_I2C_ReceiveSCCB(0x3c, ov5640_regs[n], 2, &val, sizeof(val));
        printf ("0x%04X = 0x%02X (err: %ld)\r\n", ov5640_regs[n], val, ret);
    }

    uint32_t base1 = 0x40CC0000;
    uint16_t offset1[] = {0x2480, 0x2580, 0x2500};

    for (int n=0; n<sizeof(offset1)/sizeof(offset1[0]); n++)
    {
        printf ("0x%08lX=%08lX\r\n", base1 + offset1[n], *(uint32_t*)(base1 + offset1[n]));
    }

    uint32_t base2 = 0x400E4000;
    uint16_t offset2[] = {0x00EC};

    for (int n=0; n<sizeof(offset2)/sizeof(offset2[0]); n++)
    {
        printf ("0x%08lX=%08lX\r\n", base2 + offset2[n], *(uint32_t*)(base2 + offset2[n]));
    }

    for (int n=0; n<14; n++)
    {
        printf ("40810%03X=%08lX\r\n",0x100 + (n * 4),
            *(uint32_t*)(0x40810000 + 0x100 + (n * 4)));
    }
}
