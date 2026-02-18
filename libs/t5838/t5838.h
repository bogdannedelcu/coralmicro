#ifndef LIBS_T5838_T5838_H_
#define LIBS_T5838_T5838_H_

#include <cstdint>

#include "libs/audio/audio_driver.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_pdm.h"

namespace coralmicro {

// T5838 AAD threshold levels (0-7).
// Lower values = more sensitive, higher values = less sensitive.
enum class T5838AadThreshold : uint8_t {
  kLevel0 = 0,
  kLevel1 = 1,
  kLevel2 = 2,
  kLevel3 = 3,
  kLevel4 = 4,
  kLevel5 = 5,
  kLevel6 = 6,
  kLevel7 = 7,
};

// Configuration for the RT1176 hardware voice activity detector (HWVAD).
struct HwvadConfig {
  // Which PDM channel to monitor (0-3).
  uint8_t channel = 0;
  // Number of frames to initialize HWVAD.
  uint8_t initialize_time = 10;
  // CIC oversample rate.
  uint8_t cic_oversample_rate = 0;
  // Input gain for HWVAD.
  uint8_t input_gain = 0;
  // Frame time.
  uint32_t frame_time = 10;
  // High-pass filter cutoff frequency.
  pdm_hwvad_hpf_config_t hpf_cutoff = kPDM_HwvadHpfCutOffFreq215Hz;
  // Enable frame energy mode.
  bool enable_frame_energy = true;
  // Enable pre-filter.
  bool enable_pre_filter = true;
  // Signal gain for noise filter.
  uint32_t signal_gain = 0;
  // Noise filter: enable auto noise filter.
  bool noise_auto_filter = true;
  // Noise filter: noise gain.
  uint32_t noise_gain = 2;
  // Zero-cross detector threshold.
  uint32_t zcd_threshold = 2;
};

// Configuration for the T5838 driver.
struct T5838Config {
  // Audio sample rate.
  AudioSampleRate sample_rate = AudioSampleRate::k16000_Hz;
  // Number of DMA buffers.
  size_t num_dma_buffers = 4;
  // DMA buffer size in milliseconds.
  size_t dma_buffer_size_ms = 50;
  // Number of PDM channels to enable (1-4).
  uint8_t num_channels = 1;
  // PDM decimation filter output gain.
  pdm_df_output_gain_t gain = kPDM_DfOutputGain5;
  // DC remover cutoff frequency.
  pdm_dc_remover_t dc_cutoff = kPDM_DcRemoverCutOff152Hz;
  // Enable RT1176 hardware voice activity detection.
  bool enable_hwvad = false;
  // HWVAD configuration (used when enable_hwvad is true).
  HwvadConfig hwvad;
  // Enable T5838 external AAD (WAKE pin interrupt).
  bool enable_aad = false;
  // AAD poll interval in milliseconds (WAKE pin is polled).
  uint32_t aad_poll_interval_ms = 10;
};

// Callback for receiving audio samples from PDM capture.
//
// @param ctx User context pointer.
// @param samples Pointer to audio sample buffer (int32_t per sample).
// @param num_samples Number of samples in the buffer.
using T5838AudioCallback = void (*)(void* ctx, const int32_t* samples,
                                    size_t num_samples);

// Callback for voice activity detection events (HWVAD or AAD).
//
// @param ctx User context pointer.
// @param detected True when voice/activity is detected.
using T5838VadCallback = void (*)(void* ctx, bool detected);

// Driver for the TDK InvenSense T5838 digital microphone IC connected
// to the RT1176 via PDM interface.
//
// Provides:
// - Multi-channel PDM audio capture with EDMA
// - RT1176 Hardware Voice Activity Detection (HWVAD)
// - T5838 Acoustic Activity Detection (AAD) via WAKE/THRS pins
// - Power management via PMIC 1.8V mic rail
//
// GPIO connections:
// - MIC_CLK: GPIO_LPSR_00 (PDM clock)
// - MIC_BITSTREAM0: GPIO_LPSR_01 (PDM data channel 0)
// - WAKE: GPIO_DISP_B2_08 / GPIO11_IO09 (AAD output)
// - THRS: GPIO_DISP_B2_09 / GPIO11_IO10 (AAD threshold config)
//
// Example usage:
// @code
// AudioDriverBuffers<4, 6 * 1024> buffers;
// T5838 mic(buffers);
// T5838Config config;
// config.sample_rate = AudioSampleRate::k16000_Hz;
// config.enable_hwvad = true;
// mic.Init(config);
// mic.SetAudioCallback(my_audio_cb, nullptr);
// mic.SetHwvadCallback(my_vad_cb, nullptr);
// mic.Enable();
// @endcode
class T5838 {
 public:
  // Constructor.
  //
  // @param buffers DMA buffer storage for PDM audio capture.
  template <size_t NumDmaBuffers, size_t CombinedDmaBufferSize>
  explicit T5838(
      AudioDriverBuffers<NumDmaBuffers, CombinedDmaBufferSize>& buffers)
      : dma_buffer_(buffers.dma_buffer),
        combined_dma_buffer_size_(CombinedDmaBufferSize),
        max_num_dma_buffers_(NumDmaBuffers),
        edma_tcd_(buffers.edma_tcd),
        pdm_transfers_(buffers.pdm_transfers) {}

  ~T5838();

  // Initialize the T5838 with the given configuration.
  // Does not start audio capture - call Enable() for that.
  //
  // @param config T5838 configuration.
  // @return True on success.
  bool Init(const T5838Config& config);

  // Start PDM audio capture and voice activity detection.
  //
  // @return True on success.
  bool Enable();

  // Stop PDM audio capture and voice activity detection.
  void Disable();

  // Set callback for receiving audio samples.
  //
  // @param cb Callback function.
  // @param ctx User context pointer passed to callback.
  void SetAudioCallback(T5838AudioCallback cb, void* ctx);

  // Enable RT1176 hardware voice activity detection (HWVAD).
  // Must be called after Init() and before or after Enable().
  //
  // @return True on success.
  bool EnableHwvad();

  // Disable RT1176 HWVAD.
  void DisableHwvad();

  // Set callback for HWVAD voice detection events.
  //
  // @param cb Callback function.
  // @param ctx User context pointer passed to callback.
  void SetHwvadCallback(T5838VadCallback cb, void* ctx);

  // Enable T5838 external AAD via WAKE pin monitoring.
  // Starts a FreeRTOS task that polls the WAKE pin.
  //
  // @return True on success.
  bool EnableAad();

  // Disable T5838 external AAD.
  void DisableAad();

  // Set callback for T5838 AAD wake events.
  //
  // @param cb Callback function.
  // @param ctx User context pointer passed to callback.
  void SetAadCallback(T5838VadCallback cb, void* ctx);

  // Set T5838 AAD threshold via THRS pin one-wire protocol.
  //
  // @param level Threshold sensitivity level (0 = most sensitive).
  // @return True on success.
  bool SetAadThreshold(T5838AadThreshold level);

  // Enable an additional PDM channel (1-3).
  // Channel 0 is always enabled. Additional channels require
  // external microphones connected via B2B connector.
  //
  // @param channel Channel number (1-3).
  // @return True on success.
  bool EnableChannel(uint8_t channel);

  // Disable a PDM channel.
  //
  // @param channel Channel number (1-3). Channel 0 cannot be disabled.
  void DisableChannel(uint8_t channel);

  // Power on the microphone 1.8V rail.
  void PowerOn();

  // Power off the microphone 1.8V rail.
  void PowerOff();

  // Check if T5838 is currently enabled (capturing audio).
  //
  // @return True if enabled.
  bool IsEnabled() const { return enabled_; }

 private:
  static void StaticPdmCallback(PDM_Type* base, pdm_edma_handle_t* handle,
                                 status_t status, void* user_data);
  void PdmCallback(PDM_Type* base, pdm_edma_handle_t* handle, status_t status);
  static void AadPollTask(void* param);

  // DMA buffers (from constructor)
  int32_t* dma_buffer_;
  size_t combined_dma_buffer_size_;
  size_t max_num_dma_buffers_;
  edma_tcd_t* edma_tcd_;
  pdm_edma_transfer_t* pdm_transfers_;

  // PDM/EDMA handles
  edma_handle_t edma_handle_{};
  pdm_edma_handle_t pdm_edma_handle_{};

  // DMA state
  volatile int pdm_transfer_index_ = 0;
  size_t pdm_transfer_count_ = 0;

  // Configuration
  T5838Config config_{};
  bool initialized_ = false;
  bool enabled_ = false;

  // Audio callback
  T5838AudioCallback audio_cb_ = nullptr;
  void* audio_ctx_ = nullptr;

  // HWVAD callback
  T5838VadCallback hwvad_cb_ = nullptr;
  void* hwvad_ctx_ = nullptr;
  bool hwvad_enabled_ = false;

  // AAD callback and task
  T5838VadCallback aad_cb_ = nullptr;
  void* aad_ctx_ = nullptr;
  TaskHandle_t aad_task_ = nullptr;
  bool aad_enabled_ = false;
  bool aad_last_state_ = false;

  // Channel enable mask
  uint32_t channel_mask_ = 0;
};

}  // namespace coralmicro

#endif  // LIBS_T5838_T5838_H_
