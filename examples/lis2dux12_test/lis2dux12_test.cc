// LIS2DUX12 accelerometer test (board rev 1.1, schematic sheet 16).
//
// The part sits on I2C5 next to the PMIC.  Two things learned on the first
// units (docs/board_rev_1_1/init_audit.md F18):
//   - it powers up in deep power-down and NAKs until EN_DEVICE_CONFIG.SOFT_PD
//     is written (the driver does that in Init(), then needs ~25 ms);
//   - the SA0 strap reads as 0x19 on the schematic, but the units answer at
//     0x18 -- so try both addresses, a few rounds apart.
// Then: WHO_AM_I (0x47), a few acceleration samples (Z ~ +1 g flat on the
// bench), and a single-tap event poll.

#include <cstdio>

#include "libs/base/led.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/lis2dux12/lis2dux12.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

namespace coralmicro {
namespace {

Lis2dux12 g_imu;

// Bounded: 3 rounds x 2 addresses, 30 ms apart (< 400 ms worst case).
int InitImu() {
  static const uint8_t kAddrs[2] = {0x18, 0x19};
  for (int round = 0; round < 3; ++round) {
    for (uint8_t addr : kAddrs) {
      if (g_imu.Init(I2C5Handle(), addr)) {
        printf("[LIS2DUX12] ready at 0x%02X (round %d)\r\n", addr, round);
        return addr;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return -1;
}

void Main() {
  printf("LIS2DUX12 Test Example (board rev 1.1)\r\n");
  printf("======================================\r\n\r\n");
  LedSet(Led::kStatus, true);

  int addr = InitImu();
  if (addr < 0) {
    printf("ERROR: LIS2DUX12 did not answer at 0x18 or 0x19 on I2C5\r\n");
    printf("Check VDD_1V8_INT (TCR2LN18 / R13) and the I2C5 pull-ups\r\n");
    LedSet(Led::kStatus, false);
    return;
  }

  uint8_t id = 0;
  g_imu.CheckWhoAmI(&id);
  printf("WHO_AM_I = 0x%02X (expected 0x%02X)\r\n\r\n", id, LIS2DUX12_ID);

  // 10 samples at the default ODR set by Init().
  printf("Acceleration samples (mg):\r\n");
  int got = 0;
  for (int i = 0; i < 200 && got < 10; ++i) {
    bool ready = false;
    if (!g_imu.IsDataReady(&ready)) {
      printf("ERROR: status read failed\r\n");
      break;
    }
    if (!ready) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    AccelData d{};
    if (!g_imu.ReadData(&d)) {
      printf("ERROR: data read failed\r\n");
      break;
    }
    printf("  x=%7.1f  y=%7.1f  z=%7.1f  T=%.1f C\r\n", (double)d.x_mg, (double)d.y_mg,
           (double)d.z_mg, (double)d.temp_deg_c);
    ++got;
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Single tap on Z, peak threshold 2 x 62.5 mg; poll the event sources.
  printf("\r\nTap the board (single tap, Z axis) -- polling 10 s...\r\n");
  if (!g_imu.ConfigureTap(LIS2DUX12_TAP_ON_Z, 2, /*single*/ true, /*double*/ false)) {
    printf("ERROR: ConfigureTap failed\r\n");
  } else {
    int taps = 0;
    for (int i = 0; i < 200; ++i) {
      Lis2dux12::EventSources ev{};
      if (g_imu.PollEvents(&ev) && (ev.single_tap || ev.double_tap)) {
        ++taps;
        printf("  tap #%d (%s)\r\n", taps, ev.double_tap ? "double" : "single");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("Taps seen: %d\r\n", taps);
    g_imu.DisableEvents();
  }

  printf("\r\n=== LIS2DUX12 test complete (addr 0x%02X, WHO_AM_I 0x%02X) ===\r\n", addr, id);
  for (int i = 0; i < 10; ++i) {
    LedSet(Led::kStatus, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    LedSet(Led::kStatus, false);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  LedSet(Led::kStatus, true);
}

}  // namespace
}  // namespace coralmicro

extern "C" void app_main(void* param) {
  (void)param;
  coralmicro::Main();
  vTaskSuspend(nullptr);
}
