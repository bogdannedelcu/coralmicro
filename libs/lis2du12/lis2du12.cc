#include "libs/lis2du12/lis2du12.h"

#include <cstdio>

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

#ifdef LIS2DU12_DEBUG
#define LIS2DU12_LOG(...) printf(__VA_ARGS__)
#else
#define LIS2DU12_LOG(...) ((void)0)
#endif

namespace coralmicro {
namespace {
constexpr uint32_t kMaxTransferRetries = 10;
}  // namespace

int32_t Lis2du12::PlatformWrite(void* handle, uint8_t reg, const uint8_t* data,
                                uint16_t len) {
  auto* self = static_cast<Lis2du12*>(handle);

  LIS2DU12_LOG("[LIS2DU12] I2C Write: addr=0x%02X reg=0x%02X len=%u data=",
               self->i2c_address_, reg, len);
  for (uint16_t i = 0; i < len && i < 8; ++i) {
    LIS2DU12_LOG("0x%02X ", data[i]);
  }
  if (len > 8) LIS2DU12_LOG("...");
  LIS2DU12_LOG("\r\n");

  lpi2c_master_transfer_t transfer{};
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = self->i2c_address_;
  transfer.direction = kLPI2C_Write;
  transfer.subaddress = reg;
  transfer.subaddressSize = 1;
  transfer.data = const_cast<uint8_t*>(data);
  transfer.dataSize = len;

  status_t res = kStatus_Success;
  uint32_t attempts = 0;
  do {
    if (res == kStatus_LPI2C_Busy) {
      LIS2DU12_LOG("[LIS2DU12] I2C Write: bus busy, yielding\r\n");
      taskYIELD();
    } else if (res == kStatus_LPI2C_ArbitrationLost) {
      LIS2DU12_LOG("[LIS2DU12] I2C Write: arbitration lost, attempt %lu\r\n",
                   static_cast<unsigned long>(attempts + 1));
    }
    if (++attempts >= kMaxTransferRetries) break;

    res = LPI2C_RTOS_Transfer(self->i2c_handle_, &transfer);
  } while (res == kStatus_LPI2C_Busy || res == kStatus_LPI2C_ArbitrationLost);

  if (res != kStatus_Success) {
    LIS2DU12_LOG("[LIS2DU12] I2C Write FAILED: reg=0x%02X status=%ld\r\n",
                 reg, static_cast<long>(res));
  }

  return (res == kStatus_Success) ? 0 : -1;
}

int32_t Lis2du12::PlatformRead(void* handle, uint8_t reg, uint8_t* data,
                               uint16_t len) {
  auto* self = static_cast<Lis2du12*>(handle);

  LIS2DU12_LOG("[LIS2DU12] I2C Read: addr=0x%02X reg=0x%02X len=%u\r\n",
               self->i2c_address_, reg, len);

  lpi2c_master_transfer_t transfer{};
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = self->i2c_address_;
  transfer.direction = kLPI2C_Read;
  transfer.subaddress = reg;
  transfer.subaddressSize = 1;
  transfer.data = data;
  transfer.dataSize = len;

  status_t res = kStatus_Success;
  uint32_t attempts = 0;
  do {
    if (res == kStatus_LPI2C_Busy) {
      LIS2DU12_LOG("[LIS2DU12] I2C Read: bus busy, yielding\r\n");
      taskYIELD();
    } else if (res == kStatus_LPI2C_ArbitrationLost) {
      LIS2DU12_LOG("[LIS2DU12] I2C Read: arbitration lost, attempt %lu\r\n",
                   static_cast<unsigned long>(attempts + 1));
    }
    if (++attempts >= kMaxTransferRetries) break;

    res = LPI2C_RTOS_Transfer(self->i2c_handle_, &transfer);
  } while (res == kStatus_LPI2C_Busy || res == kStatus_LPI2C_ArbitrationLost);

  if (res == kStatus_Success) {
    LIS2DU12_LOG("[LIS2DU12] I2C Read OK: reg=0x%02X data=", reg);
    for (uint16_t i = 0; i < len && i < 8; ++i) {
      LIS2DU12_LOG("0x%02X ", data[i]);
    }
    if (len > 8) LIS2DU12_LOG("...");
    LIS2DU12_LOG("\r\n");
  } else {
    LIS2DU12_LOG("[LIS2DU12] I2C Read FAILED: reg=0x%02X status=%ld\r\n",
                 reg, static_cast<long>(res));
  }

  return (res == kStatus_Success) ? 0 : -1;
}

bool Lis2du12::Init(lpi2c_rtos_handle_t* i2c_handle, uint8_t i2c_address) {
  i2c_handle_ = i2c_handle;
  i2c_address_ = i2c_address;

  dev_ctx_.write_reg = PlatformWrite;
  dev_ctx_.read_reg = PlatformRead;
  dev_ctx_.handle = this;
  dev_ctx_.mdelay = [](uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); };

  LIS2DU12_LOG("[LIS2DU12] Init: address=0x%02X\r\n", i2c_address_);

  if (!CheckWhoAmI()) {
    return false;
  }

  // Reset device
  if (lis2du12_init_set(&dev_ctx_, LIS2DU12_RESET) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: reset failed\r\n");
    return false;
  }

  // Wait for reset to complete
  lis2du12_status_t status;
  do {
    if (lis2du12_status_get(&dev_ctx_, &status) != 0) {
      LIS2DU12_LOG("[LIS2DU12] ERROR: status read failed\r\n");
      return false;
    }
  } while (status.sw_reset);

  // Set driver ready (BDU + auto-increment)
  if (lis2du12_init_set(&dev_ctx_, LIS2DU12_DRV_RDY) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: driver ready failed\r\n");
    return false;
  }

  // Default mode: 100Hz, +/-2g, ODR/2 bandwidth
  mode_.odr = LIS2DU12_100Hz;
  mode_.fs = LIS2DU12_2g;
  mode_.bw = LIS2DU12_ODR_div_2;
  if (lis2du12_mode_set(&dev_ctx_, &mode_) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: mode set failed\r\n");
    return false;
  }

  LIS2DU12_LOG("[LIS2DU12] Init OK: 100Hz, +/-2g\r\n");
  return true;
}

bool Lis2du12::CheckWhoAmI() {
  lis2du12_id_t id;
  if (lis2du12_id_get(&dev_ctx_, &id) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: WHO_AM_I read failed\r\n");
    return false;
  }
  LIS2DU12_LOG("[LIS2DU12] WHO_AM_I=0x%02X (expected 0x%02X)\r\n", id.whoami,
         LIS2DU12_ID);
  return id.whoami == LIS2DU12_ID;
}

bool Lis2du12::SetMode(lis2du12_odr_t odr, lis2du12_fs_t fs,
                       lis2du12_bw_t bw) {
  mode_.odr = odr;
  mode_.fs = fs;
  mode_.bw = bw;
  return lis2du12_mode_set(&dev_ctx_, &mode_) == 0;
}

bool Lis2du12::ReadData(AccelData* data) {
  lis2du12_data_t raw;
  if (lis2du12_data_get(&dev_ctx_, &mode_, &raw) != 0) {
    return false;
  }
  data->x_mg = raw.xl.mg[0];
  data->y_mg = raw.xl.mg[1];
  data->z_mg = raw.xl.mg[2];
  data->temp_deg_c = raw.heat.deg_c;
  return true;
}

bool Lis2du12::IsDataReady(bool* ready) {
  lis2du12_status_t status;
  if (lis2du12_status_get(&dev_ctx_, &status) != 0) {
    return false;
  }
  *ready = status.drdy_xl;
  return true;
}

bool Lis2du12::EnterSleep() {
  LIS2DU12_LOG("[LIS2DU12] EnterSleep: saving ODR=%u\r\n",
               static_cast<unsigned>(mode_.odr));
  saved_odr_ = mode_.odr;
  mode_.odr = LIS2DU12_OFF;
  if (lis2du12_mode_set(&dev_ctx_, &mode_) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: EnterSleep failed\r\n");
    mode_.odr = saved_odr_;
    return false;
  }
  LIS2DU12_LOG("[LIS2DU12] EnterSleep OK (power-down)\r\n");
  return true;
}

bool Lis2du12::ExitSleep() {
  LIS2DU12_LOG("[LIS2DU12] ExitSleep: restoring ODR=%u\r\n",
               static_cast<unsigned>(saved_odr_));
  mode_.odr = saved_odr_;
  if (lis2du12_mode_set(&dev_ctx_, &mode_) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: ExitSleep failed\r\n");
    return false;
  }
  LIS2DU12_LOG("[LIS2DU12] ExitSleep OK\r\n");
  return true;
}

bool Lis2du12::SetInt2WakeUpThreshold(uint8_t threshold, bool x_en,
                                      bool y_en, bool z_en) {
  LIS2DU12_LOG("[LIS2DU12] SetInt2WakeUp: threshold=%u x=%d y=%d z=%d\r\n",
               threshold, x_en, y_en, z_en);

  // Configure wake-up parameters
  lis2du12_wkup_md_t wkup{};
  wkup.x_en = x_en ? 1 : 0;
  wkup.y_en = y_en ? 1 : 0;
  wkup.z_en = z_en ? 1 : 0;
  wkup.threshold = threshold;
  wkup.duration = LIS2DU12_WAKE_DUR_0_ODR;
  wkup.sleep.en = 0;
  wkup.sleep.duration = 0;
  wkup.sleep.odr = LIS2DU12_DO_NOT_CHANGE;

  if (lis2du12_wake_up_mode_set(&dev_ctx_, &wkup) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: wake_up_mode_set failed\r\n");
    return false;
  }

  // Enable interrupts globally
  lis2du12_int_mode_t int_mode{};
  int_mode.enable = 1;
  int_mode.active_low = 0;
  int_mode.drdy_latched = 0;
  int_mode.base_sig = LIS2DU12_INT_LATCHED;

  if (lis2du12_interrupt_mode_set(&dev_ctx_, &int_mode) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: interrupt_mode_set failed\r\n");
    return false;
  }

  // Route wake-up interrupt to INT2
  lis2du12_pin_int_route_t int2_route{};
  int2_route.wake_up = 1;

  if (lis2du12_pin_int2_route_set(&dev_ctx_, &int2_route) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: pin_int2_route_set failed\r\n");
    return false;
  }

  LIS2DU12_LOG("[LIS2DU12] SetInt2WakeUp OK\r\n");
  return true;
}

bool Lis2du12::SetInt2DoubleTap() {
  LIS2DU12_LOG("[LIS2DU12] SetInt2DoubleTap\r\n");

  // All axes, threshold=3 (~187mg at +/-2g, 1 LSB = FS/32 = 62.5mg).
  // shock=1 → 12 ODR times (~120ms max tap duration at 100Hz).
  // quiet=1 → 6 ODR times (~60ms dead-time after tap).
  // latency=1 → 48 ODR times (~480ms max gap between two taps at 100Hz).
  lis2du12_tap_md_t tap{};
  tap.x_en = 1;
  tap.y_en = 1;
  tap.z_en = 1;
  tap.threshold.x = 3;
  tap.threshold.y = 3;
  tap.threshold.z = 3;
  tap.shock = 1;
  tap.quiet = 1;
  tap.priority = LIS2DU12_XYZ;
  tap.tap_double.en = 1;
  tap.tap_double.latency = 1;

  if (lis2du12_tap_mode_set(&dev_ctx_, &tap) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: tap_mode_set failed\r\n");
    return false;
  }

  // Read-modify-write: add double_tap to INT2 route without clearing wake_up.
  lis2du12_pin_int_route_t route{};
  if (lis2du12_pin_int2_route_get(&dev_ctx_, &route) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: pin_int2_route_get failed\r\n");
    return false;
  }
  route.double_tap = 1;
  if (lis2du12_pin_int2_route_set(&dev_ctx_, &route) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: pin_int2_route_set failed\r\n");
    return false;
  }

  LIS2DU12_LOG("[LIS2DU12] SetInt2DoubleTap OK\r\n");
  return true;
}

bool Lis2du12::ClearWakeUpInterrupt() {
  lis2du12_all_sources_t sources;
  if (lis2du12_all_sources_get(&dev_ctx_, &sources) != 0) {
    LIS2DU12_LOG("[LIS2DU12] ERROR: ClearWakeUpInterrupt failed\r\n");
    return false;
  }
  LIS2DU12_LOG("[LIS2DU12] ClearWakeUp: wu=%d x=%d y=%d z=%d\r\n",
               sources.wake_up, sources.wake_up_x, sources.wake_up_y,
               sources.wake_up_z);
  return true;
}

}  // namespace coralmicro
