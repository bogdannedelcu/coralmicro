#ifndef LIBS_LIS2DUX12_LIS2DUX12_H_
#define LIBS_LIS2DUX12_LIS2DUX12_H_

// LIS2DUX12 accelerometer (board rev 2026-10, sheet 16, I2C5 @ 0x19).
// Replaces the LIS2DU12 driver (libs/lis2du12): different WHO_AM_I (0x47),
// the part powers up in DEEP POWER-DOWN and must be woken with
// EN_DEVICE_CONFIG before it answers, and the tap / wake-up engines have
// a different register model (single tap axis, no per-axis wake enables).
// Public surface mirrors Lis2du12 so modsentai_hal.cc changes stay small.
// INT1 is the pin wired on this board (-> WAKEUP via Q5); INT2 is N/C.

#include <cstdint>

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

extern "C" {
#include "lis2dux12_reg.h"
}

namespace coralmicro {

struct AccelData {
  float x_mg;
  float y_mg;
  float z_mg;
  float temp_deg_c;
};

class Lis2dux12 {
 public:
  struct EventSources {
    bool drdy;
    bool single_tap;
    bool double_tap;
    bool triple_tap;
    bool wake_up;
  };

  // Initialize with I2C handle and 7-bit address (SA0=HIGH: 0x19, LOW: 0x18).
  // Exits deep power-down, checks WHO_AM_I, soft-resets (bounded wait),
  // enables BDU + address auto-increment, sets 100 Hz / +/-2 g.
  bool Init(lpi2c_rtos_handle_t* i2c_handle, uint8_t i2c_address = 0x19);

  // Read WHO_AM_I. Returns true if the device answers 0x47; the raw value
  // is stored in `id_out` when non-null (also on mismatch, for diagnostics).
  bool CheckWhoAmI(uint8_t* id_out = nullptr);

  bool SetMode(lis2dux12_odr_t odr, lis2dux12_fs_t fs, lis2dux12_bw_t bw);
  bool ReadData(AccelData* data);
  bool IsDataReady(bool* ready);
  bool EnterSleep();
  bool ExitSleep();

  // Wake-up engine, routed to INT1 (the wired pin).  threshold 0..255:
  //   0-63   -> fine weight  (1 LSB = FS/256)
  //   64-255 -> coarse weight (1 LSB = FS/64, register = threshold/4)
  // The LIS2DUX12 has no per-axis wake enables; the axis flags are accepted
  // for API compatibility and ignored (all axes).  Also switches the global
  // interrupt mode to LATCHED.
  bool SetInt2WakeUpThreshold(uint8_t threshold, bool x_en, bool y_en,
                              bool z_en);

  // Tap engine.  `peak_ths` in 62.5 mg steps.  Raises ODR to 400 Hz (tap
  // detection needs >= 200 Hz on this part); the other timing fields come
  // from the ST application example and are a starting point for tuning.
  bool ConfigureTap(lis2dux12_axis_t axis, uint8_t peak_ths, bool single_on,
                    bool double_on);

  // Compatibility: Z axis, peak 3 (~187 mg), double tap only, routed to INT1.
  bool SetInt2DoubleTap();

  // Read-and-clear ALL_INT_SRC-derived event flags (polling pattern).
  bool PollEvents(EventSources* out);

  // Disable interrupt generation, un-route INT1/INT2, read-clear sources.
  bool DisableEvents();

  // Legacy name used by callers; same as PollEvents() without the output.
  bool ClearWakeUpInterrupt();

  stmdev_ctx_t* GetDevCtx() { return &dev_ctx_; }

 private:
  static int32_t PlatformWrite(void* handle, uint8_t reg, const uint8_t* data,
                               uint16_t len);
  static int32_t PlatformRead(void* handle, uint8_t reg, uint8_t* data,
                              uint16_t len);

  lpi2c_rtos_handle_t* i2c_handle_ = nullptr;
  uint8_t i2c_address_ = 0x19;
  stmdev_ctx_t dev_ctx_{};
  lis2dux12_priv_t priv_{};
  lis2dux12_md_t mode_{};
  lis2dux12_odr_t saved_odr_ = LIS2DUX12_OFF;
};

}  // namespace coralmicro

#endif  // LIBS_LIS2DUX12_LIS2DUX12_H_
