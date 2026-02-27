// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstdio>
#include <vector>

#include "libs/base/http_server.h"
#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/base/strings.h"
#include "libs/base/utils.h"
#include "libs/camera/camera.h"
#include "libs/libjpeg/jpeg.h"
#include "libs/audio/audio_service.h"
#include "libs/base/console_m7.h"
#include "libs/base/network.h"
#include "libs/lis2du12/lis2du12.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/stream_buffer.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/sockets.h"
#include "libs/base/gpio.h"
#include "libs/pmic/pmic.h"

#if defined(CAMERA_STREAMING_HTTP_ETHERNET)
#include "libs/base/ethernet.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/prot/dhcp.h"
#elif defined(CAMERA_STREAMING_HTTP_WIFI)
#include "libs/base/wifi.h"
#endif  // defined(CAMERA_STREAMING_HTTP_ETHERNET)

// Hosts an RPC server on the Dev Board Micro that streams camera images
// to a connected client app.

namespace coralmicro {
namespace {

volatile uint32_t g_mic_clk_feedback_irq_count = 0;

// Audio service buffers and state
AudioDriverBuffers</*NumDmaBuffers=*/4, /*CombinedDmaBufferSize=*/6 * 1024>
    g_audio_buffers;
volatile float g_rms_level = 0.0f;

bool AudioCallback(void* ctx, const int32_t* samples, size_t num_samples) {
  (void)ctx;
  double sum = 0.0;
  for (size_t i = 0; i < num_samples; ++i) {
    double s = static_cast<double>(samples[i]);
    sum += s * s;
  }
  g_rms_level = static_cast<float>(sqrt(sum / num_samples));
  return true;
}

void AudioTask(void* param) {
  (void)param;
  printf("[AudioTask] Starting audio service task\r\n");

  AudioDriver driver(g_audio_buffers);
  const AudioDriverConfig config{AudioSampleRate::k16000_Hz,
                                 /*num_dma_buffers=*/4,
                                 /*dma_buffer_size_ms=*/50};
  if (!g_audio_buffers.CanHandle(config)) {
    printf("[AudioTask] ERROR: Not enough static memory for DMA buffers\r\n");
    vTaskSuspend(nullptr);
  }

  AudioService service(&driver, config, /*task_priority=*/3,
                        /*drop_first_samples_ms=*/200);
  service.AddCallback(nullptr, AudioCallback);
  printf("[AudioTask] Audio service running\r\n");

  while (true) {
    float rms = g_rms_level;
    printf("[AudioTask] RMS: %10.0f\r\n", static_cast<double>(rms));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

constexpr int kLogPort = 1234;
// Stream buffer used to pipe all printf/UART output to the TCP log client.
// Large enough to buffer a burst of output without blocking the UART task.
constexpr size_t kLogPipeBytes = 4096;

void TcpLogTask(void* param) {
  (void)param;

  // Allocate the pipe and register it with the console so every printf call
  // is mirrored here in addition to UART/USB-CDC.
  StreamBufferHandle_t log_pipe = xStreamBufferCreate(kLogPipeBytes, 1);
  if (!log_pipe) {
    printf("[TcpLog] ERROR: Failed to allocate log pipe\r\n");
    vTaskSuspend(nullptr);
  }
  ConsoleM7::GetSingleton()->SetLogPipe(log_pipe);

  const int server_fd = SocketServer(kLogPort, /*backlog=*/1);
  if (server_fd < 0) {
    printf("[TcpLog] ERROR: Cannot bind port %d\r\n", kLogPort);
    vTaskSuspend(nullptr);
  }
  printf("[TcpLog] Listening on port %d\r\n", kLogPort);

  char buf[256];
  while (true) {
    const int client_fd = SocketAccept(server_fd);
    if (client_fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    printf("[TcpLog] Client connected (fd=%d)\r\n", client_fd);

    // Discard any log data that accumulated while no client was connected.
    xStreamBufferReset(log_pipe);

    while (true) {
      size_t n = xStreamBufferReceive(log_pipe, buf, sizeof(buf),
                                      pdMS_TO_TICKS(200));
      if (n == 0) continue;
      if (WriteArray(client_fd, buf, n) != IOStatus::kOk) break;
    }

    printf("[TcpLog] Client disconnected\r\n");
    SocketClose(client_fd);
    xStreamBufferReset(log_pipe);
  }
}

constexpr char kIndexFileName[] = "/coral_micro_camera.html";
constexpr char kCameraStreamUrlPrefix[] = "/camera_stream";
constexpr float kRedCoefficient = .2126;
constexpr float kGreenCoefficient = .7152;
constexpr float kBlueCoefficient = .0722;
constexpr float kUint8Max = 255.0;

void AccelTask(void* param) {
  (void)param;
  Lis2du12 g_accel;

  printf("[AccelTask] Starting LIS2DU12 accelerometer task\r\n");

  if (!g_accel.Init(I2C5Handle(), 0x19)) {
    printf("[AccelTask] ERROR: LIS2DU12 init failed\r\n");
    vTaskSuspend(nullptr);
  }

  AccelData data;
  // Check if a wake-up event occurred
  lis2du12_all_sources_t sources{};
  if (lis2du12_all_sources_get(g_accel.GetDevCtx(), &sources) == 0 &&
      sources.wake_up) {
    printf("[AccelTask] Wake-up detected! x=%d y=%d z=%d\r\n",
            sources.wake_up_x, sources.wake_up_y, sources.wake_up_z);
    g_accel.ClearWakeUpInterrupt();
  }

  // Configure wake-up interrupt on INT2 for all axes.
  // Threshold ~500 mg at +/-2g (threshold=64 -> 64 * 7.8mg ~= 500mg)
  constexpr uint8_t kWakeUpThreshold = 64;
  if (!g_accel.SetInt2WakeUpThreshold(kWakeUpThreshold, true, true, true)) {
    printf("[AccelTask] ERROR: SetInt2WakeUpThreshold failed\r\n");
    vTaskSuspend(nullptr);
  }

  // Read accelerometer data after wake-up
  while (true) {
    bool ready = false;
    if (g_accel.IsDataReady(&ready) && ready) {
      if (g_accel.ReadData(&data)) {
        printf("[AccelTask] X=%.1f Y=%.1f Z=%.1f mg  T=%.1f C\r\n",
                static_cast<double>(data.x_mg),
                static_cast<double>(data.y_mg),
                static_cast<double>(data.z_mg),
                static_cast<double>(data.temp_deg_c));

      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  } // while
}

HttpServer::Content UriHandler(const char* uri) {
  // printf("HTTP url: %s\n", uri);
  if (StrEndsWith(uri, "index.shtml") ||
      StrEndsWith(uri, "coral_micro_camera.html")) {
    return std::string(kIndexFileName);
  } else if (StrEndsWith(uri, kCameraStreamUrlPrefix))
  {
    // [start-snippet:jpeg]
    std::vector<uint8_t> buf(CameraTask::kWidth * CameraTask::kHeight *
                             CameraFormatBpp(CameraFormat::kRgb));
    auto fmt = CameraFrameFormat{
        CameraFormat::kRgb,       CameraFilterMethod::kBilinear,
        CameraRotation::k0,       CameraTask::kWidth,
        CameraTask::kHeight,
        /*preserve_ratio=*/false, (uint8_t *)buf.data(),
        /*while_balance=*/true};

    // CameraTask::GetSingleton()->SetTestPattern(CameraTestPattern::kColorBar);

    if (!CameraTask::GetSingleton()->GetFrame({fmt})) {
      printf("Unable to get frame from camera\r\n");
      return {};
    }

    std::vector<uint8_t> jpeg;
    JpegCompressRgb((uint8_t *)buf.data(), fmt.width, fmt.height, /*quality=*/75, &jpeg);
    // [end-snippet:jpeg]
    return jpeg;
  }
  return {};
}

void Main() {
  printf("Camera HTTP Example!\r\n");
  
  bool value = false;

  // while(true)
  // while(false)
  {
    // value = !value;

    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, true);
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, true);
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, false);
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, false);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // value = false;

    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));

    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    
    // value = true;

    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
    // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, value);
    // vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Read Chip ID from PMIC and print it out
  uint8_t pmicChipId = coralmicro::PmicTask::GetSingleton()->GetChipId();
  printf("PMIC chip ID: %d\r\n", pmicChipId);

  // // Reset the camera power here until we fix the camera power handling
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, false);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, false);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, false);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, false);
  // vTaskDelay(pdMS_TO_TICKS(100));

  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, true);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, true);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, true);
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, true);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Turn on Status LED to show the board is on.
  LedSet(Led::kStatus, true);

  CameraTask::GetSingleton()->SetPower(true);
  CameraTask::GetSingleton()->Enable(CameraMode::kStreaming);

  // Register callback for the user button.
  GpioConfigureInterrupt(
      coralmicro::Gpio::kUserButton, coralmicro::GpioInterruptMode::kIntModeFalling,
      [handle = xTaskGetCurrentTaskHandle()]() { xTaskResumeFromISR(handle); },
      /*debounce_interval_us=*/50 * 1e3);

  // Register MIC_CLK_FEEDBACK rising edge interrupt with counter.
  GpioConfigureInterrupt(
      coralmicro::Gpio::kMicClkFeedback, coralmicro::GpioInterruptMode::kIntModeRising,
      []() {
        ++g_mic_clk_feedback_irq_count;
        printf("[MIC_CLK_FB] IRQ count: %lu\r\n",
               static_cast<unsigned long>(g_mic_clk_feedback_irq_count));
      });



#if defined(CAMERA_STREAMING_HTTP_ETHERNET)
  EthernetInit(/*default_iface=*/false);
  auto* ethernet = EthernetGetInterface();
  if (!ethernet) {
    printf("Unable to bring up ethernet...\r\n");
    vTaskSuspend(nullptr);
  }
  auto ethernet_ip = EthernetGetIp();
  if (!ethernet_ip.has_value()) {
    printf("Unable to get Ethernet IP\r\n");
    vTaskSuspend(nullptr);
  }
  printf("Serving on: %s\r\n", ethernet_ip->c_str());
#elif defined(CAMERA_STREAMING_HTTP_WIFI)
  if (!WiFiTurnOn(/*default_iface=*/false)) {
    printf("Unable to bring up WiFi...\r\n");
    vTaskSuspend(nullptr);
  }
  if (!WiFiConnect(10)) {
    printf("Unable to connect to WiFi...\r\n");
    vTaskSuspend(nullptr);
  }
  if (auto wifi_ip = WiFiGetIp()) {
    printf("Serving on: %s\r\n", wifi_ip->c_str());
  } else {
    printf("Failed to get Wifi Ip\r\n");
    vTaskSuspend(nullptr);
  }
#else   // USB
  std::string usb_ip;
  if (GetUsbIpAddress(&usb_ip)) {
    printf("Serving on---: http://%s\r\n", usb_ip.c_str());
  }
#endif  // defined(CAMERA_STREAMING_HTTP_ETHERNET)

  // Start LIS2DU12 accelerometer task
  xTaskCreate(AccelTask, "accel_task", configMINIMAL_STACK_SIZE * 4,
              nullptr, 2, nullptr);

  // Start audio service task
  xTaskCreate(AudioTask, "audio_task", configMINIMAL_STACK_SIZE * 30,
              nullptr, 3, nullptr);

  // Start TCP log server — mirrors all UART output to port 1234
  xTaskCreate(TcpLogTask, "tcp_log_task", configMINIMAL_STACK_SIZE * 4,
              nullptr, 2, nullptr);

  HttpServer http_server;
  http_server.AddUriHandler(UriHandler);
  UseHttpServer(&http_server);

  while (true) {
    // By default, front camera is selected already
    static bool front = true;

    front = !front;

    vTaskSuspend(nullptr);
    // CameraTask::GetSingleton()->ChangePattern();
    CameraTask::GetSingleton()->SwitchCamera(front ?
      SwitchCameraId::kCameraFront : SwitchCameraId::kCameraBack);
  }
}
}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::Main();
}
