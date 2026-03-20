#ifndef LIBS_LIS2DU12_LIS2DU12_H_
#define LIBS_LIS2DU12_LIS2DU12_H_

#include <cstdint>

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

extern "C" {
#include "lis2du12_reg.h"
}

namespace coralmicro {

struct AccelData {
  float x_mg;
  float y_mg;
  float z_mg;
  float temp_deg_c;
};

class Lis2du12 {
 public:
  // Initialize with I2C handle and 7-bit address.
  // SA0=HIGH: address = 0x19, SA0=LOW: address = 0x18
  bool Init(lpi2c_rtos_handle_t* i2c_handle, uint8_t i2c_address = 0x19);

  // Read WHO_AM_I register. Returns true if device responds with 0x45.
  bool CheckWhoAmI();

  // Configure ODR, full scale, and bandwidth.
  bool SetMode(lis2du12_odr_t odr, lis2du12_fs_t fs, lis2du12_bw_t bw);

  // Read accelerometer and temperature data.
  bool ReadData(AccelData* data);

  // Read raw status register (check DRDY bit).
  bool IsDataReady(bool* ready);

  // Enter sleep (power-down) mode. Saves current ODR for ExitSleep.
  bool EnterSleep();

  // Exit sleep mode. Restores the ODR that was active before EnterSleep.
  bool ExitSleep();

  // Configure wake-up threshold interrupt on INT2.
  // threshold: driver selects resolution based on value:
  //   0-63  → fine mode:   1 LSB = FS_XL/256 (e.g. at +/-2g: 7.8 mg/LSB)
  //   64-255 → coarse mode: 1 LSB = FS_XL/64  (e.g. at +/-2g: 31.25 mg/LSB)
  //            In coarse mode the raw register value = threshold / 4.
  //            Example: threshold=168 → wk_ths=42 → 42×31.25=1312.5mg ≈ 1.3g
  // x_en, y_en, z_en: enable detection on each axis.
  bool SetInt2WakeUpThreshold(uint8_t threshold, bool x_en, bool y_en,
                              bool z_en);

  // Configure double-tap detection and route double_tap to INT2.
  // Tap threshold ~187mg at +/-2g (3 LSB = 3 * FS/32 = 3 * 62.5mg).
  // Preserves any previously routed INT2 sources (e.g. wake_up).
  bool SetInt2DoubleTap();

  // Clear the wake-up interrupt by reading the source register.
  bool ClearWakeUpInterrupt();

  // Access the underlying ST driver context for advanced configuration.
  stmdev_ctx_t* GetDevCtx() { return &dev_ctx_; }

 private:
  static int32_t PlatformWrite(void* handle, uint8_t reg, const uint8_t* data,
                               uint16_t len);
  static int32_t PlatformRead(void* handle, uint8_t reg, uint8_t* data,
                              uint16_t len);

  lpi2c_rtos_handle_t* i2c_handle_ = nullptr;
  uint8_t i2c_address_ = 0x19;
  stmdev_ctx_t dev_ctx_{};
  lis2du12_md_t mode_{};
  lis2du12_odr_t saved_odr_ = LIS2DU12_OFF;
};

}  // namespace coralmicro

#endif  // LIBS_LIS2DU12_LIS2DU12_H_
