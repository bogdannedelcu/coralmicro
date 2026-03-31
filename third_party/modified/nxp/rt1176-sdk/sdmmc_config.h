#ifndef _LIBS_NXP_RT1176_SDK_SDMMC_CONFIG_H_
#define _LIBS_NXP_RT1176_SDK_SDMMC_CONFIG_H_

#include "third_party/nxp/rt1176-sdk/middleware/sdmmc/common/fsl_sdmmc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void BOARD_SDIO_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, sdio_int_t cardInt);
uint32_t BOARD_USDHC1ClockConfiguration(void);
uint32_t BOARD_USDHC2ClockConfiguration(void);
void BOARD_SDCardPowerResetInit(void);
void BOARD_SDCardPowerControl(bool enable);
bool BOARD_SDCardGetDetectStatus(void);
void BOARD_SDCardDetectInit(sd_cd_t cd, void *userData);
void BOARD_USDHC_Errata(void);

#ifdef __cplusplus
}
#endif

#endif  // _LIBS_NXP_RT1176_SDK_SDMMC_CONFIG_H_
