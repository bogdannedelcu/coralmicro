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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "libs/audio/audio_driver.h"
#include "libs/audio/audio_service.h"
#include "libs/base/console_m7.h"
#include "libs/base/gpio.h"
#include "libs/base/http_server.h"
#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/base/network.h"
#include "libs/base/reset.h"
#include "libs/base/strings.h"
#include "libs/base/timer.h"
#include "libs/base/utils.h"
#include "libs/camera/camera.h"
#include "libs/libjpeg/jpeg.h"
#include "libs/lis2du12/lis2du12.h"
#include "libs/pmic/pmic.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/stream_buffer.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/timers.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/sockets.h"

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

// Rolling log buffer shared between the ConsoleM7 callback and /log HTTP endpoint.
// Protected by g_log_mutex; oldest bytes are dropped when the cap is reached.
SemaphoreHandle_t g_log_mutex = nullptr;
std::string g_log_buffer;
constexpr size_t kMaxLogBytes = 32 * 1024;

// LDO_1V8_INT_EN GPIO (GPIO9 pin 20) — initialized as output in main_freertos_m7.cc.
#define LDO_1V8_INT_EN_GPIO  GPIO9
#define LDO_1V8_INT_EN_PIN   20U

// Accel task handle (for start/stop commands).
TaskHandle_t g_accel_task_handle = nullptr;

// Camera side: 0 = front, 1 = back.
volatile int g_camera_side = 0;

// Audio recording.
constexpr int kRecordingSeconds = 3;
constexpr size_t kRecordingSamples = 16000u * kRecordingSeconds;
volatile bool g_recording_requested = false;
volatile bool g_audio_ready = false;
SemaphoreHandle_t g_audio_wav_mutex = nullptr;
std::vector<int16_t> g_recording_buf;
std::vector<uint8_t> g_audio_wav_data;

void OnLogMessage(const char* data, int len) {
  if (!g_log_mutex) return;
  // Non-blocking take: don't stall the console TX task if the HTTP handler
  // is momentarily holding the mutex.
  if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  if (g_log_buffer.size() + static_cast<size_t>(len) > kMaxLogBytes) {
    g_log_buffer.erase(
        0, g_log_buffer.size() + static_cast<size_t>(len) - kMaxLogBytes);
  }
  g_log_buffer.append(data, static_cast<size_t>(len));
  xSemaphoreGive(g_log_mutex);
}

// Builds a 16-bit mono PCM WAV file from int16_t samples.
static void BuildWav(const std::vector<int16_t>& samples, int sample_rate,
                     std::vector<uint8_t>& out) {
  const uint32_t data_bytes =
      static_cast<uint32_t>(samples.size()) * sizeof(int16_t);
  out.resize(44 + data_bytes);

  auto w32 = [&](uint32_t v, size_t off) {
    out[off]   =  v        & 0xFF;
    out[off+1] = (v >>  8) & 0xFF;
    out[off+2] = (v >> 16) & 0xFF;
    out[off+3] = (v >> 24) & 0xFF;
  };
  auto w16 = [&](uint16_t v, size_t off) {
    out[off]   =  v       & 0xFF;
    out[off+1] = (v >> 8) & 0xFF;
  };

  memcpy(&out[0],  "RIFF", 4);
  w32(36 + data_bytes, 4);
  memcpy(&out[8],  "WAVE", 4);
  memcpy(&out[12], "fmt ", 4);
  w32(16,                  16);  // chunk size
  w16(1,                   20);  // PCM
  w16(1,                   22);  // mono
  w32(static_cast<uint32_t>(sample_rate), 24);
  w32(static_cast<uint32_t>(sample_rate) * 2, 28);  // byte rate
  w16(2,                   32);  // block align
  w16(16,                  34);  // bits per sample
  memcpy(&out[36], "data", 4);
  w32(data_bytes,          40);
  memcpy(&out[44], samples.data(), data_bytes);
}

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

  // Accumulate int16 samples for an active recording request.
  if (g_recording_requested && !g_audio_ready) {
    for (size_t i = 0;
         i < num_samples && g_recording_buf.size() < kRecordingSamples; ++i) {
      g_recording_buf.push_back(static_cast<int16_t>(samples[i] >> 16));
    }
    if (g_recording_buf.size() >= kRecordingSamples) {
      g_recording_requested = false;
      if (g_audio_wav_mutex &&
          xSemaphoreTake(g_audio_wav_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        BuildWav(g_recording_buf, 16000, g_audio_wav_data);
        g_recording_buf.clear();
        g_audio_ready = true;
        xSemaphoreGive(g_audio_wav_mutex);
      }
    }
  }

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

// FreeRTOS one-shot timer callback: fires when the "alarm 1 minute" expires.
static void RtcAlarmCallback(TimerHandle_t) {
  struct tm now;
  TimerGetRtcTime(&now);
  printf("[Alarm] RTC alarm fired at %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
         now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
         now.tm_hour, now.tm_min, now.tm_sec);
  LedSet(Led::kStatus, true);
  vTaskDelay(pdMS_TO_TICKS(300));
  LedSet(Led::kStatus, false);
}

// Handles GET /command/<name>[/<extra>], e.g. /command/cam1_on
// or /command/set_rtc/1706000000.
// Query parameters are NOT used because lwIP strips the '?' before calling
// fs_open_custom — the command is therefore path-encoded.
static HttpServer::Content HandleCommand(const char* uri) {
  static constexpr char kPrefix[] = "/command/";
  const char* after = strstr(uri, kPrefix);
  if (!after) {
    return std::vector<uint8_t>{'E','R','R'};
  }
  after += sizeof(kPrefix) - 1;  // advance past "/command/"

  // Extract command token (everything up to the next '/' or end of string).
  const char* slash = strchr(after, '/');
  const std::string cmd = slash ? std::string(after, slash) : std::string(after);
  if (cmd.empty()) {
    return std::vector<uint8_t>{'E','R','R'};
  }
  printf("[CMD] %s\r\n", cmd.c_str());

  if (cmd == "cam1_on") {
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, true);
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, true);
    printf("[CMD] Camera 1 rails ON\r\n");

  } else if (cmd == "cam1_off") {
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_1V8, false);
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1_2V8, false);

    printf("[CMD] Camera 1 rails OFF\r\n");

  } else if (cmd == "cam2_on") {
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, true);
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, true);

    printf("[CMD] Camera 2 rails ON\r\n");

  } else if (cmd == "cam2_off") {
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_1V8, false);
    PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, false);

    printf("[CMD] Camera 2 rails OFF\r\n");

  } else if (cmd == "switch_camera") {
    g_camera_side ^= 1;
    CameraTask::GetSingleton()->SwitchCamera(
        g_camera_side == 0 ? SwitchCameraId::kCameraFront
                           : SwitchCameraId::kCameraBack);
    printf("[CMD] Switched to camera %s\r\n",
           g_camera_side == 0 ? "Front" : "Back");

  } else if (cmd == "start_accel") {
    if (g_accel_task_handle) {
      vTaskResume(g_accel_task_handle);
      printf("[CMD] Accel task resumed\r\n");
    }

  } else if (cmd == "stop_accel") {
    if (g_accel_task_handle) {
      vTaskSuspend(g_accel_task_handle);
      printf("[CMD] Accel task suspended\r\n");
    }

  } else if (cmd == "record_audio") {
    if (!g_recording_requested && !g_audio_ready) {
      if (g_audio_wav_mutex &&
          xSemaphoreTake(g_audio_wav_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_recording_buf.clear();
        g_recording_buf.reserve(kRecordingSamples);
        g_audio_wav_data.clear();
        g_audio_ready = false;
        xSemaphoreGive(g_audio_wav_mutex);
      }
      g_recording_requested = true;
      printf("[CMD] Recording %d s of audio...\r\n", kRecordingSeconds);
    } else {
      printf("[CMD] Recording in progress or previous clip not fetched yet\r\n");
    }

  } else if (cmd == "set_rtc") {
    // Timestamp is the next path segment: /command/set_rtc/<unix_ts>
    if (slash && *(slash + 1) != '\0') {
      const uint32_t ts =
          static_cast<uint32_t>(strtoul(slash + 1, nullptr, 10));
      TimerSetRtcTime(ts);
      struct tm now;
      TimerGetRtcTime(&now);
      printf("[CMD] RTC set to %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
             now.tm_hour, now.tm_min, now.tm_sec);
    }

  } else if (cmd == "set_rtc_alarm_1m") {
    TimerHandle_t alarm =
        xTimerCreate("rtc_alarm", pdMS_TO_TICKS(60000),
                     /*uxAutoReload=*/pdFALSE, nullptr, RtcAlarmCallback);
    if (alarm) {
      xTimerStart(alarm, 0);
      struct tm now;
      TimerGetRtcTime(&now);
      printf("[CMD] 1-minute alarm set (now %04d-%02d-%02d %02d:%02d:%02d)\r\n",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
             now.tm_hour, now.tm_min, now.tm_sec);
    }

  } else if (cmd == "ldo_1v8_on") {
    GPIO_PinWrite(LDO_1V8_INT_EN_GPIO, LDO_1V8_INT_EN_PIN, 1U);
    printf("[CMD] LDO_1V8_INT_EN ON\r\n");

  } else if (cmd == "ldo_1v8_off") {
    GPIO_PinWrite(LDO_1V8_INT_EN_GPIO, LDO_1V8_INT_EN_PIN, 0U);
    printf("[CMD] LDO_1V8_INT_EN OFF\r\n");

  } else if (cmd == "shutdown") {
    printf("[CMD] Shutting down...\r\n");
    vTaskDelay(pdMS_TO_TICKS(300));  // allow log to flush
    ResetToFlash();

  } else {
    printf("[CMD] Unknown command: %s\r\n", cmd.c_str());
  }

  return std::vector<uint8_t>{'O', 'K'};
}

constexpr char kIndexFileName[] = "/coral_micro_camera.html";
constexpr char kCameraStreamUrlPrefix[] = "/camera_stream";
constexpr float kRedCoefficient = .2126;
constexpr float kGreenCoefficient = .7152;
constexpr float kBlueCoefficient = .0722;
constexpr float kUint8Max = 255.0;

// Reads all PMIC status registers every 2 seconds and prints them.
static void PmicStatusTask(void*) {
  // Wait for PmicTask to be ready.
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (true) {
    PmicTask::GetSingleton()->DumpStatus();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

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
  } else if (StrEndsWith(uri, "/log")) {
    // Return accumulated log lines since last poll, then clear the buffer.
    std::vector<uint8_t> out;
    if (g_log_mutex &&
        xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      out.assign(g_log_buffer.begin(), g_log_buffer.end());
      g_log_buffer.clear();
      xSemaphoreGive(g_log_mutex);
    }
    return out;
  } else if (strncmp(uri, "/command/", 9) == 0) {
    return HandleCommand(uri);
  } else if (StrEndsWith(uri, "/audio_data")) {
    // Return recorded WAV once ready, then clear so next recording can start.
    std::vector<uint8_t> wav;
    if (g_audio_ready && g_audio_wav_mutex &&
        xSemaphoreTake(g_audio_wav_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      wav = std::move(g_audio_wav_data);
      g_audio_wav_data.clear();
      g_audio_ready = false;
      xSemaphoreGive(g_audio_wav_mutex);
    }
    return wav;
  }
  return {};
}

// I2C helpers for PMIC issue
#if 0
void CameraTaskRead(uint16_t reg, const uint8_t *val, int size) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = 0x3c;
  transfer.direction = kLPI2C_Read;
  transfer.subaddress = static_cast<uint16_t>(reg);
  transfer.subaddressSize = sizeof(reg);
  transfer.data = (void *)val;
  transfer.dataSize = size;

  status_t status1 = LPI2C_RTOS_Transfer(I2C5Handle(), &transfer);
  printf("[CAM] Rx|0x%04X=0x%02X|(s1: %ld)\n", reg, val[0], status1);
}

void CameraTaskWrite(uint16_t reg, const uint8_t *val, int size) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = 0x3c;
  transfer.direction = kLPI2C_Write;
  transfer.subaddress = static_cast<uint16_t>(reg);
  transfer.subaddressSize = sizeof(reg);
  transfer.data = (void *)val;
  transfer.dataSize = size;

  status_t status1 = LPI2C_RTOS_Transfer(I2C5Handle(), &transfer);
  printf("[CAM] Tx|0x%04X=0x%02X|(s1: %ld)\n", reg, val[0], status1);
}

bool PmicTaskWrite(uint16_t reg, uint8_t val) {

  // Send DATA then CRC as a two-byte payload.
  lpi2c_master_transfer_t transfer;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = 0x5B;
  transfer.direction      = kLPI2C_Write;
  transfer.subaddress     = reg;
  transfer.subaddressSize = 2;
  transfer.data           = &val;
  transfer.dataSize       = 1; // sizeof(payload);
  
  status_t status1 = LPI2C_RTOS_Transfer(I2C5Handle(), &transfer);
  printf("[PMIC] Tx|0x%04X=0x%02X|(s1: %ld)\n", reg, val, status1);

  return true;
}

bool PmicTaskRead(uint16_t reg, uint8_t &val) {

  // Send DATA then CRC as a two-byte payload.
  lpi2c_master_transfer_t transfer;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = 0x5B;
  transfer.direction      = kLPI2C_Read;
  transfer.subaddress     = reg;
  transfer.subaddressSize = 2;
  transfer.data           = &val;
  transfer.dataSize       = 1; // sizeof(payload);
  
  status_t status1 = LPI2C_RTOS_Transfer(I2C5Handle(), &transfer);
  printf("[PMIC] Rx|0x%04X=0x%02X|(s1: %ld)\n", reg, val, status1);

  return true;
}

void i2c_issue(void)
{
  uint8_t val = 0x00;

  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, 0);

  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamPwrDn, 0);
  vTaskDelay(pdMS_TO_TICKS(2));

  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamReset, 1);
  vTaskDelay(pdMS_TO_TICKS(40));

  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamPwrDn2, 0);
  vTaskDelay(pdMS_TO_TICKS(2));

  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamReset2, 1);
  vTaskDelay(pdMS_TO_TICKS(40));

  // Set MUX on front camera by default
  coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, 1);

  // CameraTaskRead(0x300A, &val, sizeof(val));
  // CameraTaskRead(0x300B, &val, sizeof(val));
  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, true);
  // PmicTaskWrite(0x0667, 0x39);
  // PmicTaskRead(0x0667, val);
  PmicTaskWrite(0x066E, 0x39);
  PmicTaskWrite(0x0667, 0x39);
  // PmicTaskRead(0x0667, val);

  vTaskDelay(pdMS_TO_TICKS(400));

  val = 0;
  CameraTaskWrite(0x3618, &val, sizeof(val));
  CameraTaskRead(0x300A, &val, sizeof(val));
  CameraTaskRead(0x300B, &val, sizeof(val));

  PmicTaskWrite(0x0667, 0x38);
  PmicTaskWrite(0x066E, 0x38);
  vTaskDelay(pdMS_TO_TICKS(400));

  CameraTaskWrite(0x3618, &val, sizeof(val));
  CameraTaskRead(0x300A, &val, sizeof(val));
  CameraTaskRead(0x300B, &val, sizeof(val));

  PmicTaskWrite(0x066E, 0x39);
  PmicTaskWrite(0x0667, 0x39);
  // PmicTaskRead(0x0667, val);

  vTaskDelay(pdMS_TO_TICKS(400));

  val = 0;
  CameraTaskWrite(0x3618, &val, sizeof(val));
  CameraTaskRead(0x300A, &val, sizeof(val));
  CameraTaskRead(0x300B, &val, sizeof(val));

  // PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2_2V8, true);
}
#endif // PMIC issue

void Main() {
  // Set up the HTTP log buffer before any printf so messages are captured.
  g_log_mutex = xSemaphoreCreateMutex();
  g_audio_wav_mutex = xSemaphoreCreateMutex();
  ConsoleM7::GetSingleton()->SetLogCallback(OnLogMessage);

  printf("Camera HTTP Example!\r\n");
  
  bool value = false;

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

  // Start LIS2DU12 accelerometer task (save handle for start/stop commands).
  xTaskCreate(AccelTask, "accel_task", configMINIMAL_STACK_SIZE * 4,
              nullptr, 2, &g_accel_task_handle);

  // Periodic PMIC status dump every 2 s.
  // xTaskCreate(PmicStatusTask, "pmic_status", configMINIMAL_STACK_SIZE * 4,
  //             nullptr, 2, nullptr);

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
