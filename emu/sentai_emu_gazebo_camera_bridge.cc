// Guest-side Gazebo camera bridge for ARM emulation.
//
// A Renode Python peripheral accepts SCM1 RGB888 frames from
// gz_to_uds_bridge and exposes "latest frame" through a tiny MMIO doorbell.
// This task copies that frame into guest SDRAM and injects it at the same
// shared runtime boundary used by SIM and virtual providers:
// sentai_camera_backend_publish_rgb888().

#include <stdint.h>
#include <stddef.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" int sentai_camera_backend_publish_rgb888(uint32_t seq,
                                                     const uint8_t* rgb,
                                                     size_t bytes);
extern "C" int sentai_cam_init_full(int streaming, int fps,
                                     int hflip, int vflip);

extern "C" {
volatile uint32_t g_sentai_emu_gazebo_cam_task_started = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_frames = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_publish_fail = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_last_seq = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_last_rc = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_bridge_seen = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_bridge_served = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_bridge_dropped = 0;
volatile uint32_t g_sentai_emu_gazebo_cam_bridge_bad = 0;
}

namespace {

constexpr uintptr_t kBridgeBase = 0x40902800u;
constexpr uint32_t kCmdReadLatest = 1;
#ifndef SENTAI_EMU_GAZEBO_CAMERA_W
#define SENTAI_EMU_GAZEBO_CAMERA_W 640
#endif
#ifndef SENTAI_EMU_GAZEBO_CAMERA_H
#define SENTAI_EMU_GAZEBO_CAMERA_H 480
#endif
constexpr int kFrameW = SENTAI_EMU_GAZEBO_CAMERA_W;
constexpr int kFrameH = SENTAI_EMU_GAZEBO_CAMERA_H;
constexpr size_t kFrameBytes = static_cast<size_t>(kFrameW) * kFrameH * 3u;
constexpr int kTaskStackWords = configMINIMAL_STACK_SIZE * 6;

enum RegOffset : uint32_t {
  kRegStatus = 0x00,
  kRegCommand = 0x04,
  kRegOutPtr = 0x08,
  kRegOutLen = 0x0C,
  kRegResult = 0x10,
  kRegSeq = 0x14,
  kRegWidth = 0x18,
  kRegHeight = 0x1C,
  kRegLatestSeq = 0x20,
  kRegFramesIn = 0x24,
  kRegDropped = 0x28,
  kRegBad = 0x2C,
  kRegFramesServed = 0x30,
};

volatile uint32_t& Reg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kBridgeBase + offset);
}

uint8_t g_frame[kFrameBytes] __attribute__((aligned(32), section(".sdram_bss")));
StaticTask_t g_task_tcb;
StackType_t g_task_stack[kTaskStackWords]
    __attribute__((aligned(8), section(".sdram_bss")));

int32_t BridgeReadLatest(uint32_t* seq) {
  Reg(kRegOutPtr) = reinterpret_cast<uintptr_t>(g_frame);
  Reg(kRegOutLen) = static_cast<uint32_t>(kFrameBytes);
  Reg(kRegCommand) = kCmdReadLatest;
  Reg(kRegResult) = 0xFFFFFFFFu;
  Reg(kRegStatus) = 1;

  for (int i = 0; i < 100; ++i) {
    if (Reg(kRegStatus) == 2) {
      const uint32_t result = Reg(kRegResult);
      if (seq) *seq = Reg(kRegSeq);
      g_sentai_emu_gazebo_cam_bridge_seen = Reg(kRegFramesIn);
      g_sentai_emu_gazebo_cam_bridge_served = Reg(kRegFramesServed);
      g_sentai_emu_gazebo_cam_bridge_dropped = Reg(kRegDropped);
      g_sentai_emu_gazebo_cam_bridge_bad = Reg(kRegBad);
      return static_cast<int32_t>(result);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return -1;
}

void GazeboCameraTask(void*) {
  g_sentai_emu_gazebo_cam_task_started = 1;
  (void)sentai_cam_init_full(1, 30, 0, 1);

  for (;;) {
    uint32_t seq = 0;
    const int32_t got = BridgeReadLatest(&seq);
    g_sentai_emu_gazebo_cam_last_rc = static_cast<uint32_t>(got);
    if (got == static_cast<int32_t>(kFrameBytes) &&
        seq != 0 && seq != g_sentai_emu_gazebo_cam_last_seq) {
      const int rc = sentai_camera_backend_publish_rgb888(seq, g_frame,
                                                          kFrameBytes);
      if (rc == 0) {
        g_sentai_emu_gazebo_cam_last_seq = seq;
        ++g_sentai_emu_gazebo_cam_frames;
      } else {
        ++g_sentai_emu_gazebo_cam_publish_fail;
      }
      vTaskDelay(pdMS_TO_TICKS(1000 / 30));
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

}  // namespace

extern "C" int sentai_emu_gazebo_camera_bridge_start(void) {
  static TaskHandle_t task = nullptr;
  if (task) return 0;
  task = xTaskCreateStatic(GazeboCameraTask, "gz_cam_bridge",
                           kTaskStackWords, nullptr,
                           tskIDLE_PRIORITY + 1,
                           g_task_stack, &g_task_tcb);
  return task ? 0 : -1;
}
