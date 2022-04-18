#include "apps/OOBE/oobe_json.h"
#include "libs/base/httpd.h"
#include "libs/base/ipc_m7.h"
#include "libs/base/mutex.h"
#include "libs/base/utils.h"
#include "libs/posenet/posenet.h"
#include "libs/tasks/CameraTask/camera_task.h"
#include "libs/tasks/EdgeTpuTask/edgetpu_task.h"
#include "libs/tpu/edgetpu_manager.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/mjson/src/mjson.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"

#if defined(OOBE_DEMO_ETHERNET)
#include "libs/base/ethernet.h"
#endif  // defined(OOBE_DEMO_ETHERNET)

#if defined(OOBE_DEMO_WIFI)
#include "libs/base/gpio.h"
extern "C" {
#include "libs/nxp/rt1176-sdk/rtos/freertos/libraries/abstractions/wifi/include/iot_wifi.h"
}
#endif  // defined(OOBE_DEMO_WIFI)

#include <cstdio>
#include <cstring>
#include <list>

constexpr float kThreshold = 0.4;

constexpr int kPosenetWidth = 481;
constexpr int kPosenetHeight = 353;
constexpr int kPosenetDepth = 3;
constexpr int kPosenetSize = kPosenetWidth * kPosenetHeight * kPosenetDepth;
constexpr int kCmdStart = 1;
constexpr int kCmdStop = 2;
constexpr int kCmdProcess = 3;

static SemaphoreHandle_t camera_output_mtx;
std::vector<uint8_t> camera_output;

static SemaphoreHandle_t posenet_output_mtx;
std::unique_ptr<valiant::posenet::Output> posenet_output;
TickType_t posenet_output_time;


static valiant::HttpServer::Content UriHandler(const char *name) {
  if (std::strcmp(name, "/camera") == 0) {
    valiant::MutexLock lock(camera_output_mtx);
    if (camera_output.empty()) return {};
    return valiant::HttpServer::Content{std::move(camera_output)};
  }

  if (std::strcmp(name, "/pose") == 0) {
    valiant::MutexLock lock(posenet_output_mtx);
    if (!posenet_output) return {};
    auto json = valiant::oobe::CreatePoseJSON(*posenet_output, kThreshold);
    posenet_output.reset();
    return json;
  }

  return {};
}

namespace valiant {
namespace oobe {

static void HandleAppMessage(
    const uint8_t data[valiant::ipc::kMessageBufferDataSize], void *param) {
  (void)data;
  vTaskResume(reinterpret_cast<TaskHandle_t>(param));
}

struct TaskMessage {
  int command;
  SemaphoreHandle_t completion_semaphore;
  void *data;
};

class OOBETask {
 public:
  OOBETask(const char *name) : queue_(xQueueCreate(2, sizeof(TaskMessage))) {
    xTaskCreate(StaticTaskFunc, name, configMINIMAL_STACK_SIZE * 30, this,
                APP_TASK_PRIORITY, &task_);
  }

  void Start() { SendCommandBlocking(kCmdStart); }

  void Stop() { SendCommandBlocking(kCmdStop); }

 protected:
  virtual void TaskFunc() = 0;
  QueueHandle_t queue_;

 private:
  static void StaticTaskFunc(void *param) {
    auto thiz = reinterpret_cast<OOBETask *>(param);
    thiz->TaskFunc();
  }

  void SendCommandBlocking(int command) {
    TaskMessage message = {command, xSemaphoreCreateBinary()};
    xQueueSend(queue_, &message, portMAX_DELAY);
    xSemaphoreTake(message.completion_semaphore, portMAX_DELAY);
    vSemaphoreDelete(message.completion_semaphore);
  }

  TaskHandle_t task_;
};

class PosenetTask : public OOBETask {
 public:
  PosenetTask() : OOBETask("posenet_task"){};
  void QueueFrame(std::vector<uint8_t> *frame) {
    TaskMessage message = {kCmdProcess, nullptr, frame};
    xQueueSend(queue_, &message, portMAX_DELAY);
  }

 protected:
  void TaskFunc() override {
    TaskMessage message;
    valiant::posenet::Output output;
    while (true) {
      xQueueReceive(queue_, &message, portMAX_DELAY);

      switch (message.command) {
        case kCmdStart:
          configASSERT(!started_);
          started_ = true;
          printf("Posenet: started\r\n");
          break;
        case kCmdStop:
          configASSERT(started_);
          started_ = false;
          printf("Posenet: stopped\r\n");
          break;
        case kCmdProcess: {
          configASSERT(started_);
          auto camera_output =
              reinterpret_cast<std::vector<uint8_t> *>(message.data);
          TfLiteTensor *input = valiant::posenet::input();
          memcpy(tflite::GetTensorData<uint8_t>(input), camera_output->data(),
                 camera_output->size());
          delete camera_output;

          valiant::posenet::loop(&output, false);
          int good_poses_count = 0;
          for (int i = 0; i < valiant::posenet::kPoses; ++i) {
            if (output.poses[i].score > kThreshold) {
              good_poses_count++;
            }
          }
          if (good_poses_count) {
            valiant::MutexLock lock(posenet_output_mtx);
            posenet_output = std::make_unique<valiant::posenet::Output>(output);
            posenet_output_time = xTaskGetTickCount();
          }
        } break;

        default:
          printf("Unknown command: %d\r\n", message.command);
          break;
      }

      // Signal the command completion semaphore, if present.
      if (message.completion_semaphore) {
        xSemaphoreGive(message.completion_semaphore);
      }
    }
  }

 private:
  bool started_ = false;
};

class CameraTask : public OOBETask {
 public:
  CameraTask(PosenetTask *posenet_task)
      : OOBETask("posenet_task"), posenet_task_(posenet_task){};

 protected:
  void TaskFunc() override {
    TaskMessage message;
    while (true) {
      xQueueReceive(queue_, &message, portMAX_DELAY);

      switch (message.command) {
        case kCmdStart:
          configASSERT(!started_);
          started_ = true;
          valiant::CameraTask::GetSingleton()->Enable(
              valiant::camera::Mode::STREAMING);
          printf("Camera: started\r\n");
          posenet_task_->Start();
          QueueProcess();
          break;
        case kCmdStop:
          configASSERT(started_);
          valiant::CameraTask::GetSingleton()->Disable();
          started_ = false;
          printf("Camera: stopped\r\n");
          posenet_task_->Stop();
          break;
        case kCmdProcess: {
          if (!started_) {
            continue;
          }

          std::vector<uint8_t> input(kPosenetSize);
          valiant::camera::FrameFormat fmt;
          fmt.width = kPosenetWidth;
          fmt.height = kPosenetHeight;
          fmt.fmt = valiant::camera::Format::RGB;
          fmt.preserve_ratio = false;
          fmt.buffer = input.data();
          valiant::CameraTask::GetFrame({fmt});

          {
            valiant::MutexLock lock(camera_output_mtx);
            camera_output = std::move(input);
          }

          // Signal posenet.
          posenet_task_->QueueFrame(new std::vector<uint8_t>(input));

          // Process next camera frame.
          QueueProcess();
        } break;

        default:
          printf("Unknown command: %d\r\n", message.command);
          break;
      }

      // Signal the command completion semaphore, if present.
      if (message.completion_semaphore) {
        xSemaphoreGive(message.completion_semaphore);
      }
    }
  }

 private:
  void QueueProcess() {
    TaskMessage message = {kCmdProcess};
    xQueueSend(queue_, &message, portMAX_DELAY);
  }

  PosenetTask *posenet_task_ = nullptr;
  bool started_ = false;
};

#if defined(OOBE_DEMO_WIFI)
static bool ConnectToWifi() {
  std::string wifi_ssid, wifi_psk;
  bool have_ssid = valiant::utils::GetWifiSSID(&wifi_ssid);
  bool have_psk = valiant::utils::GetWifiPSK(&wifi_psk);

  if (have_ssid) {
    WIFI_On();
    WIFIReturnCode_t xWifiStatus;
    WIFINetworkParams_t xNetworkParams;
    xNetworkParams.pcSSID = wifi_ssid.c_str();
    xNetworkParams.ucSSIDLength = wifi_ssid.length();
    if (have_psk) {
      xNetworkParams.pcPassword = wifi_psk.c_str();
      xNetworkParams.ucPasswordLength = wifi_psk.length();
      xNetworkParams.xSecurity = eWiFiSecurityWPA2;
    } else {
      xNetworkParams.pcPassword = "";
      xNetworkParams.ucPasswordLength = 0;
      xNetworkParams.xSecurity = eWiFiSecurityOpen;
    }
    xWifiStatus = WIFI_ConnectAP(&xNetworkParams);

    if (xWifiStatus != eWiFiSuccess) {
      printf("failed to connect to %s\r\n", wifi_ssid.c_str());
      return false;
    }
  } else {
    printf("No Wi-Fi SSID provided\r\n");
    return false;
  }
  return true;
}
#endif  // defined(OOBE_DEMO_WIFI)

void Main() {
  PosenetTask posenet_task;
  CameraTask camera_task(&posenet_task);
  camera_output_mtx = xSemaphoreCreateMutex();
  posenet_output_mtx = xSemaphoreCreateMutex();

// For the OOBE Demo, bring up WiFi and Ethernet. For now these are active
// but unused.
#if defined(OOBE_DEMO_ETHERNET)
  valiant::InitializeEthernet(false);
#elif defined(OOBE_DEMO_WIFI)
  if (!ConnectToWifi()) {
    // If connecting to wi-fi fails, turn our LEDs on solid, and halt.
    valiant::gpio::SetGpio(valiant::gpio::Gpio::kPowerLED, true);
    valiant::gpio::SetGpio(valiant::gpio::Gpio::kUserLED, true);
    valiant::gpio::SetGpio(valiant::gpio::Gpio::kTpuLED, true);
    vTaskSuspend(nullptr);
  }
#endif  // defined(OOBE_DEMO_ETHERNET)

  valiant::HttpServer http_server;
  http_server.AddUriHandler(UriHandler);
  http_server.AddUriHandler(valiant::FileSystemUriHandler{});
  valiant::UseHttpServer(&http_server);

  valiant::IPCM7::GetSingleton()->RegisterAppMessageHandler(
      HandleAppMessage, xTaskGetCurrentTaskHandle());
  valiant::EdgeTpuTask::GetSingleton()->SetPower(true);
  valiant::EdgeTpuManager::GetSingleton()->OpenDevice(
      valiant::PerformanceMode::kMax);
  if (!valiant::posenet::setup()) {
    printf("setup() failed\r\n");
    vTaskSuspend(nullptr);
  }

  valiant::IPCM7::GetSingleton()->StartM4();

#if defined(OOBE_DEMO)
  int count = 0;
#endif  // defined (OOBE_DEMO)

  vTaskSuspend(nullptr);
  while (true) {
    printf("CM7 awoken\r\n");

    // Start camera_task processing, which will start posenet_task.
    camera_task.Start();
    posenet_output_time = xTaskGetTickCount();

    while (true) {
// For OOBE Demo, run 20 iterations of this loop - each contains a one
// second delay. For normal OOBE, check that the posenet task hasn't
// progressed for 5 seconds (i.e. no poses detected).
#if defined(OOBE_DEMO)
      if (count >= 20) {
        count = 0;
        break;
      }
      ++count;
      printf("M7 %d\r\n", count);
#else
      TickType_t now = xTaskGetTickCount();
      if ((now - posenet_output_time) > pdMS_TO_TICKS(5000)) {
        break;
      }
#endif  // defined(OOBE_DEMO)

      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("Transition back to M4\r\n");

    // Stop camera_task processing. This will also stop posenet_task.
    camera_task.Stop();

    valiant::ipc::Message msg;
    msg.type = valiant::ipc::MessageType::APP;
    valiant::IPCM7::GetSingleton()->SendMessage(msg);
    vTaskSuspend(nullptr);
  }
}

}  // namespace oobe
}  // namespace valiant

extern "C" void app_main(void *param) {
  (void)param;
  valiant::oobe::Main();
  vTaskSuspend(nullptr);
}
