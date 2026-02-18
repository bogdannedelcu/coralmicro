#include <cmath>
#include <cstdio>

#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/t5838/t5838.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

namespace coralmicro {
namespace {

// DMA buffer allocation
AudioDriverBuffers</*NumDmaBuffers=*/4, /*CombinedDmaBufferSize=*/6 * 1024>
    g_audio_buffers;

// Volatile variables updated from callbacks
volatile float g_rms_level = 0.0f;
volatile uint32_t g_hwvad_count = 0;
volatile uint32_t g_aad_count = 0;
volatile bool g_aad_active = false;

// Audio callback: compute RMS level
void AudioCallback(void* ctx, const int32_t* samples, size_t num_samples) {
  (void)ctx;
  double sum = 0.0;
  for (size_t i = 0; i < num_samples; ++i) {
    double s = static_cast<double>(samples[i]);
    sum += s * s;
  }
  g_rms_level = static_cast<float>(sqrt(sum / num_samples));
}

// HWVAD callback: voice activity detected by RT1176 hardware
void HwvadCallback(void* ctx, bool detected) {
  (void)ctx;
  if (detected) {
    g_hwvad_count++;
  }
}

// AAD callback: acoustic activity detected by T5838 WAKE pin
void AadCallback(void* ctx, bool detected) {
  (void)ctx;
  g_aad_active = detected;
  if (detected) {
    g_aad_count++;
  }
}

void Main() {
  printf("\r\n");
  printf("T5838 Digital Microphone Test\r\n");
  printf("=============================\r\n\r\n");

  LedSet(Led::kStatus, true);

  // Create T5838 instance
  T5838 mic(g_audio_buffers);

  // Configure
  T5838Config config;
  config.sample_rate = AudioSampleRate::k16000_Hz;
  config.num_dma_buffers = 4;
  config.dma_buffer_size_ms = 50;
  config.num_channels = 1;
  config.enable_hwvad = true;
  config.enable_aad = false;  // WAKE pin is floating, AAD not available

  // HWVAD configuration
  config.hwvad.channel = 0;
  config.hwvad.initialize_time = 10;
  config.hwvad.input_gain = 0;
  config.hwvad.frame_time = 10;
  config.hwvad.hpf_cutoff = kPDM_HwvadHpfCutOffFreq215Hz;
  config.hwvad.enable_frame_energy = true;
  config.hwvad.enable_pre_filter = true;
  config.hwvad.signal_gain = 0;
  config.hwvad.noise_auto_filter = true;
  config.hwvad.noise_gain = 2;
  config.hwvad.zcd_threshold = 2;

  printf("Initializing T5838...\r\n");
  if (!mic.Init(config)) {
    printf("ERROR: T5838 initialization failed\r\n");
    vTaskSuspend(nullptr);
    return;
  }

  // Set callbacks
  mic.SetAudioCallback(AudioCallback, nullptr);
  mic.SetHwvadCallback(HwvadCallback, nullptr);

  printf("Enabling audio capture...\r\n");
  if (!mic.Enable()) {
    printf("ERROR: T5838 enable failed\r\n");
    vTaskSuspend(nullptr);
    return;
  }

  while(true) {
    printf("Waiting for console connection...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  printf("\r\nMonitoring audio (RMS level, HWVAD events)...\r\n");
  printf("Clap or speak near the microphone to trigger detections.\r\n\r\n");

  // Main monitoring loop
  uint32_t last_hwvad_count = 0;

  while (true) {
    float rms = g_rms_level;
    uint32_t hwvad_count = g_hwvad_count;

    printf("RMS: %10.0f | HWVAD: %lu", static_cast<double>(rms),
           static_cast<unsigned long>(hwvad_count));

    if (hwvad_count != last_hwvad_count) {
      printf(" [VOICE!]");
      last_hwvad_count = hwvad_count;
      LedSet(Led::kUser, true);
    } else {
      LedSet(Led::kUser, false);
    }

    printf("\r\n");

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::Main();
  vTaskSuspend(nullptr);
}
