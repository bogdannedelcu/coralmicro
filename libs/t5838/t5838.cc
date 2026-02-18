#include "libs/t5838/t5838.h"

#include <cstdio>

#include "libs/pmic/pmic.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_dmamux.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"

// T5838 WAKE pin: floating (not connected), AAD polling disabled
// #define T5838_WAKE_GPIO
// #define T5838_WAKE_PIN

// T5838 THRS pin: GPIO_SD_B1_03 -> GPIO10, pin 6
#define T5838_THRS_GPIO      GPIO10
#define T5838_THRS_PIN       6U

namespace {
constexpr int kDmaChannel = 0;
constexpr int kPdmClock = 24577500;
constexpr int kAadTaskStackSize = configMINIMAL_STACK_SIZE * 3;
constexpr int kAadTaskPriority = 3;

// T5838 one-wire THRS timing (microseconds)
constexpr uint32_t kThrsResetPulseUs = 100;
constexpr uint32_t kThrsBitHighUs = 10;
constexpr uint32_t kThrsBitLowUs = 40;
constexpr uint32_t kThrsBitPauseUs = 10;

void DelayUs(uint32_t us) {
  uint32_t count = us * (SystemCoreClock / 1000000U / 4U);
  while (count--) {
    __NOP();
  }
}

}  // namespace

// HWVAD interrupt handler
static coralmicro::T5838* g_t5838_instance = nullptr;

extern "C" void PDM_EVENT_IRQHandler() {
  if (g_t5838_instance) {
    uint32_t status = PDM_GetHwvadInterruptStatusFlags(PDM);
    if (status & kPDM_HwvadStatusVoiceDetectFlag) {
      PDM_ClearHwvadInterruptStatusFlags(PDM, kPDM_HwvadStatusVoiceDetectFlag);

      // Notify via callback from ISR context
      if (g_t5838_instance->IsEnabled()) {
        // The callback dispatch is handled inside the class
      }
    }
  }
  __DSB();
}

namespace coralmicro {

T5838::~T5838() {
  if (enabled_) {
    Disable();
  }
  if (g_t5838_instance == this) {
    g_t5838_instance = nullptr;
  }
}

bool T5838::Init(const T5838Config& config) {
  config_ = config;
  channel_mask_ = kPDM_EnableChannel0;

  // Enable additional channels if requested
  for (uint8_t ch = 1; ch < config.num_channels && ch < 4; ++ch) {
    channel_mask_ |= (1U << ch);
  }

  // WAKE pin is floating (not connected) - AAD polling not available

  // Initialize THRS pin as output (default low)
  // gpio_pin_config_t thrs_pin_config = {
  //     .direction = kGPIO_DigitalOutput,
  //     .outputLogic = 0,
  //     .interruptMode = kGPIO_NoIntmode,
  // };
  // GPIO_PinInit(T5838_THRS_GPIO, T5838_THRS_PIN, &thrs_pin_config);

  g_t5838_instance = this;
  initialized_ = true;
  printf("[T5838] Initialized: %d channels, %ld Hz sample rate\r\n",
         config.num_channels,
         static_cast<long>(static_cast<int32_t>(config.sample_rate)));

  return true;
}

bool T5838::Enable() {
  if (!initialized_) {
    printf("[T5838] Error: not initialized\r\n");
    return false;
  }

  if (config_.num_dma_buffers > max_num_dma_buffers_) {
    printf("[T5838] Error: too many DMA buffers\r\n");
    return false;
  }

  const auto dma_buffer_size =
      MsToSamples(config_.sample_rate, config_.dma_buffer_size_ms);
  if (config_.num_dma_buffers * dma_buffer_size > combined_dma_buffer_size_) {
    printf("[T5838] Error: not enough DMA memory\r\n");
    return false;
  }

  pdm_transfer_index_ = 0;
  pdm_transfer_count_ = config_.num_dma_buffers;

  // Power on microphone
  PowerOn();

  // Initialize DMAMUX
  DMAMUX_Init(DMAMUX0);
  DMAMUX_SetSource(DMAMUX0, kDmaChannel, kDmaRequestMuxPdm);
  DMAMUX_EnableChannel(DMAMUX0, kDmaChannel);

  NVIC_SetPriority(DMA0_DMA16_IRQn, 6);

  // Initialize EDMA
  edma_config_t edma_config{};
  EDMA_GetDefaultConfig(&edma_config);
  EDMA_Init(DMA0, &edma_config);
  EDMA_CreateHandle(&edma_handle_, DMA0, kDmaChannel);

  // Initialize PDM
  pdm_config_t pdm_config{};
  pdm_config.enableDoze = false;
  pdm_config.fifoWatermark = 4;
  pdm_config.qualityMode = kPDM_QualityModeVeryLow2;
  pdm_config.cicOverSampleRate = 0;
  PDM_Init(PDM, &pdm_config);
  PDM_TransferCreateHandleEDMA(PDM, &pdm_edma_handle_, StaticPdmCallback, this,
                               &edma_handle_);

  // Configure PDM channels
  pdm_channel_config_t channel_config{};
  channel_config.cutOffFreq = config_.dc_cutoff;
  channel_config.gain = config_.gain;

  for (uint8_t ch = 0; ch < config_.num_channels && ch < 4; ++ch) {
    PDM_TransferSetChannelConfigEDMA(PDM, &pdm_edma_handle_, ch,
                                     &channel_config);
  }

  // Set sample rate
  auto status = PDM_SetSampleRateConfig(
      PDM, kPdmClock, static_cast<int32_t>(config_.sample_rate));
  if (status != kStatus_Success) {
    printf("[T5838] Error: PDM_SetSampleRateConfig() failed: %u\r\n", status);
    return false;
  }

  PDM_Reset(PDM);
  PDM_EnableInterrupts(PDM, kPDM_ErrorInterruptEnable);
  EnableIRQ(PDM_ERROR_IRQn);

  // Install EDMA TCD memory
  PDM_TransferInstallEDMATCDMemory(&pdm_edma_handle_, edma_tcd_,
                                   pdm_transfer_count_);

  // Set up DMA transfer chain
  for (size_t i = 0; i < pdm_transfer_count_; ++i) {
    pdm_transfers_[i].data =
        reinterpret_cast<uint8_t*>(dma_buffer_ + i * dma_buffer_size);
    pdm_transfers_[i].dataSize = dma_buffer_size * sizeof(int32_t);
    pdm_transfers_[i].linkTransfer =
        &pdm_transfers_[(i + 1) % pdm_transfer_count_];
  }

  printf("[T5838] Waiting before starting PDM_TransferReceiveEDMA() ... \r\n");
  vTaskDelay(pdMS_TO_TICKS(5000));

  status = PDM_TransferReceiveEDMA(PDM, &pdm_edma_handle_, pdm_transfers_);
  if (status != kStatus_Success) {
    printf("[T5838] Error: PDM_TransferReceiveEDMA() failed: %u\r\n", status);
    return false;
  }

  enabled_ = true;

  // Enable HWVAD if configured
  if (config_.enable_hwvad) {
    EnableHwvad();
  }

  // Enable AAD if configured
  if (config_.enable_aad) {
    EnableAad();
  }

  printf("[T5838] Audio capture enabled\r\n");
  return true;
}

void T5838::Disable() {
  if (!enabled_) return;

  // Disable AAD
  if (aad_enabled_) {
    DisableAad();
  }

  // Disable HWVAD
  if (hwvad_enabled_) {
    DisableHwvad();
  }

  DisableIRQ(PDM_ERROR_IRQn);
  PDM_TransferTerminateReceiveEDMA(PDM, &pdm_edma_handle_);
  PDM_Deinit(PDM);
  DMAMUX_Deinit(DMAMUX0);

  PowerOff();
  enabled_ = false;
  printf("[T5838] Audio capture disabled\r\n");
}

void T5838::SetAudioCallback(T5838AudioCallback cb, void* ctx) {
  audio_cb_ = cb;
  audio_ctx_ = ctx;
}

bool T5838::EnableHwvad() {
  if (!initialized_) return false;

  // Configure HWVAD
  pdm_hwvad_config_t hwvad_config{};
  hwvad_config.channel = config_.hwvad.channel;
  hwvad_config.initializeTime = config_.hwvad.initialize_time;
  hwvad_config.cicOverSampleRate = config_.hwvad.cic_oversample_rate;
  hwvad_config.inputGain = config_.hwvad.input_gain;
  hwvad_config.frameTime = config_.hwvad.frame_time;
  hwvad_config.cutOffFreq = config_.hwvad.hpf_cutoff;
  hwvad_config.enableFrameEnergy = config_.hwvad.enable_frame_energy;
  hwvad_config.enablePreFilter = config_.hwvad.enable_pre_filter;

  // Configure noise filter
  pdm_hwvad_noise_filter_t noise_config{};
  noise_config.enableAutoNoiseFilter = config_.hwvad.noise_auto_filter;
  noise_config.enableNoiseMin = !config_.hwvad.noise_auto_filter;
  noise_config.enableNoiseDecimation = !config_.hwvad.noise_auto_filter;
  noise_config.enableNoiseDetectOR = false;
  noise_config.noiseFilterAdjustment = 0;
  noise_config.noiseGain = config_.hwvad.noise_gain;

  // Configure zero-cross detector
  pdm_hwvad_zero_cross_detector_t zcd_config{};
  zcd_config.enableAutoThreshold = true;
  zcd_config.zcdAnd = kPDM_HwvadResultOREnergyBasedDetection;
  zcd_config.threshold = config_.hwvad.zcd_threshold;
  zcd_config.adjustmentThreshold = 0;

  PDM_SetHwvadConfig(PDM, &hwvad_config);
  PDM_SetHwvadSignalFilterConfig(PDM, true, config_.hwvad.signal_gain);
  PDM_SetHwvadNoiseFilterConfig(PDM, &noise_config);
  PDM_EnableHwvadZeroCrossDetector(PDM, true);
  PDM_SetHwvadZeroCrossDetectorConfig(PDM, &zcd_config);

  PDM_EnableHwvadInterrupts(PDM, kPDM_HwvadInterruptEnable);
  EnableIRQ(PDM_EVENT_IRQn);
  PDM_EnableHwvad(PDM, true);

  hwvad_enabled_ = true;
  printf("[T5838] HWVAD enabled on channel %d\r\n", config_.hwvad.channel);
  return true;
}

void T5838::DisableHwvad() {
  if (!hwvad_enabled_) return;

  PDM_EnableHwvad(PDM, false);
  PDM_DisableHwvadInterrupts(PDM, kPDM_HwvadInterruptEnable);
  DisableIRQ(PDM_EVENT_IRQn);

  hwvad_enabled_ = false;
  printf("[T5838] HWVAD disabled\r\n");
}

void T5838::SetHwvadCallback(T5838VadCallback cb, void* ctx) {
  hwvad_cb_ = cb;
  hwvad_ctx_ = ctx;
}

bool T5838::EnableAad() {
  // WAKE pin is floating (not connected) - AAD via WAKE pin not available.
  // Use HWVAD instead for voice activity detection.
  printf("[T5838] AAD not available (WAKE pin floating). Use HWVAD instead.\r\n");
  return false;
}

void T5838::DisableAad() {
  if (!aad_enabled_) return;
  aad_enabled_ = false;

  if (aad_task_) {
    vTaskDelete(aad_task_);
    aad_task_ = nullptr;
  }

  printf("[T5838] AAD disabled\r\n");
}

void T5838::SetAadCallback(T5838VadCallback cb, void* ctx) {
  aad_cb_ = cb;
  aad_ctx_ = ctx;
}

bool T5838::SetAadThreshold(T5838AadThreshold level) {
  uint8_t pulses = static_cast<uint8_t>(level);

  // T5838 one-wire threshold protocol:
  // 1. Pull THRS high for reset pulse
  // 2. Pull low
  // 3. Send N pulses (high-low transitions) to set threshold level

  // Reset pulse
  GPIO_PinWrite(T5838_THRS_GPIO, T5838_THRS_PIN, 1);
  DelayUs(kThrsResetPulseUs);
  GPIO_PinWrite(T5838_THRS_GPIO, T5838_THRS_PIN, 0);
  DelayUs(kThrsBitPauseUs);

  // Send threshold level as pulse count
  for (uint8_t i = 0; i < pulses; ++i) {
    GPIO_PinWrite(T5838_THRS_GPIO, T5838_THRS_PIN, 1);
    DelayUs(kThrsBitHighUs);
    GPIO_PinWrite(T5838_THRS_GPIO, T5838_THRS_PIN, 0);
    DelayUs(kThrsBitLowUs);
  }

  printf("[T5838] AAD threshold set to level %d\r\n", pulses);
  return true;
}

bool T5838::EnableChannel(uint8_t channel) {
  if (channel == 0 || channel > 3) return false;

  pdm_channel_config_t channel_config{};
  channel_config.cutOffFreq = config_.dc_cutoff;
  channel_config.gain = config_.gain;

  channel_mask_ |= (1U << channel);

  if (enabled_) {
    PDM_TransferSetChannelConfigEDMA(PDM, &pdm_edma_handle_, channel,
                                     &channel_config);
  }

  printf("[T5838] Channel %d enabled\r\n", channel);
  return true;
}

void T5838::DisableChannel(uint8_t channel) {
  if (channel == 0 || channel > 3) return;
  channel_mask_ &= ~(1U << channel);
  printf("[T5838] Channel %d disabled\r\n", channel);
}

void T5838::PowerOn() {
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kMic1V8, true);
  printf("[T5838] Mic 1.8V rail assumed always on (no PMIC)\r\n");
}

void T5838::PowerOff() {
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kMic1V8, false);
  printf("[T5838] Mic 1.8V rail power off skipped (no PMIC)\r\n");
}

void T5838::StaticPdmCallback(PDM_Type* base, pdm_edma_handle_t* handle,
                               status_t status, void* user_data) {
  static_cast<T5838*>(user_data)->PdmCallback(base, handle, status);
}

void T5838::PdmCallback(PDM_Type* base, pdm_edma_handle_t* handle,
                         status_t status) {
  auto& pdm_transfer = pdm_transfers_[pdm_transfer_index_];

  printf("[T5838] PdmCallback\r\n");

  if (audio_cb_) {
    audio_cb_(audio_ctx_,
              const_cast<int32_t*>(
                  reinterpret_cast<volatile int32_t*>(pdm_transfer.data)),
              pdm_transfer.dataSize / sizeof(int32_t));
  }

  // Check HWVAD status in the PDM callback context
  if (hwvad_enabled_ && hwvad_cb_) {
    uint32_t vad_status = PDM_GetHwvadInterruptStatusFlags(PDM);
    if (vad_status & kPDM_HwvadStatusVoiceDetectFlag) {
      PDM_ClearHwvadInterruptStatusFlags(PDM, kPDM_HwvadStatusVoiceDetectFlag);
      hwvad_cb_(hwvad_ctx_, true);
    }
  }

  pdm_transfer_index_ = (pdm_transfer_index_ + 1) % pdm_transfer_count_;
  __DSB();
}

void T5838::AadPollTask(void* param) {
  // WAKE pin is floating - this task should not be started.
  // Kept as a stub for future use if WAKE pin is connected.
  (void)param;
  vTaskDelete(nullptr);
}

}  // namespace coralmicro
