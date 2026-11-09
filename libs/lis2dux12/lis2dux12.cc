#include "libs/lis2dux12/lis2dux12.h"

#include <cstdio>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

#ifdef LIS2DUX12_DEBUG
#define LIS2DUX12_LOG(...) printf(__VA_ARGS__)
#else
#define LIS2DUX12_LOG(...) ((void)0)
#endif

namespace coralmicro {
namespace {
constexpr uint32_t kMaxTransferRetries = 10;
// Soft reset completes in well under 1 ms; 50 x 1 ms is a generous bound.
constexpr uint32_t kResetPollMax = 50;

int32_t Transfer(lpi2c_rtos_handle_t* h, uint8_t addr, lpi2c_direction_t dir,
                 uint8_t reg, uint8_t* data, uint16_t len) {
  lpi2c_master_transfer_t transfer{};
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = addr;
  transfer.direction = dir;
  transfer.subaddress = reg;
  transfer.subaddressSize = 1;
  transfer.data = data;
  transfer.dataSize = len;

  status_t res = kStatus_Success;
  uint32_t attempts = 0;
  do {
    if (res == kStatus_LPI2C_Busy) taskYIELD();
    if (++attempts > kMaxTransferRetries) break;
    res = LPI2C_RTOS_Transfer(h, &transfer);
  } while (res == kStatus_LPI2C_Busy || res == kStatus_LPI2C_ArbitrationLost);

  if (res != kStatus_Success) {
    LIS2DUX12_LOG("[LIS2DUX12] I2C %s reg=0x%02X FAILED status=%ld\r\n",
                  dir == kLPI2C_Write ? "W" : "R", reg, (long)res);
  }
  return (res == kStatus_Success) ? 0 : -1;
}
}  // namespace

int32_t Lis2dux12::PlatformWrite(void* handle, uint8_t reg,
                                 const uint8_t* data, uint16_t len) {
  auto* self = static_cast<Lis2dux12*>(handle);
  return Transfer(self->i2c_handle_, self->i2c_address_, kLPI2C_Write, reg,
                  const_cast<uint8_t*>(data), len);
}

int32_t Lis2dux12::PlatformRead(void* handle, uint8_t reg, uint8_t* data,
                                uint16_t len) {
  auto* self = static_cast<Lis2dux12*>(handle);
  return Transfer(self->i2c_handle_, self->i2c_address_, kLPI2C_Read, reg,
                  data, len);
}

bool Lis2dux12::Init(lpi2c_rtos_handle_t* i2c_handle, uint8_t i2c_address) {
  i2c_handle_ = i2c_handle;
  i2c_address_ = i2c_address;

  dev_ctx_.write_reg = PlatformWrite;
  dev_ctx_.read_reg = PlatformRead;
  dev_ctx_.handle = this;
  dev_ctx_.priv_data = &priv_;
  dev_ctx_.mdelay = [](uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); };

  // The part powers up in deep power-down: the I2C slave may NAK until
  // EN_DEVICE_CONFIG.SOFT_PD is written (AN5909 §3.1.1).  The write itself
  // may be NAKed if the device is already awake — ignore its status and
  // let WHO_AM_I decide.
  (void)lis2dux12_exit_deep_power_down(&dev_ctx_);

  uint8_t id = 0;
  if (!CheckWhoAmI(&id)) {
    printf("[LIS2DUX12] WHO_AM_I=0x%02X (expected 0x%02X) at 0x%02X\r\n", id,
           LIS2DUX12_ID, i2c_address_);
    return false;
  }

  if (lis2dux12_sw_reset(&dev_ctx_) != 0) {
    LIS2DUX12_LOG("[LIS2DUX12] ERROR: sw_reset failed\r\n");
    return false;
  }
  lis2dux12_status_t status{};
  uint32_t polls = 0;
  do {
    if (lis2dux12_status_get(&dev_ctx_, &status) != 0) {
      LIS2DUX12_LOG("[LIS2DUX12] ERROR: status read failed\r\n");
      return false;
    }
    if (!status.sw_reset) break;
    vTaskDelay(pdMS_TO_TICKS(1));
  } while (++polls < kResetPollMax);
  if (status.sw_reset) {
    LIS2DUX12_LOG("[LIS2DUX12] ERROR: sw_reset did not clear\r\n");
    return false;
  }

  if (lis2dux12_init_set(&dev_ctx_) != 0) {  // BDU + IF_ADD_INC
    LIS2DUX12_LOG("[LIS2DUX12] ERROR: init_set failed\r\n");
    return false;
  }

  mode_.odr = LIS2DUX12_100Hz_LP;
  mode_.fs = LIS2DUX12_2g;
  mode_.bw = LIS2DUX12_ODR_div_2;
  if (lis2dux12_mode_set(&dev_ctx_, &mode_) != 0) {
    LIS2DUX12_LOG("[LIS2DUX12] ERROR: mode set failed\r\n");
    return false;
  }
  LIS2DUX12_LOG("[LIS2DUX12] Init OK: 100 Hz LP, +/-2 g\r\n");
  return true;
}

bool Lis2dux12::CheckWhoAmI(uint8_t* id_out) {
  uint8_t id = 0;
  int32_t rc = lis2dux12_device_id_get(&dev_ctx_, &id);
  if (id_out) *id_out = id;
  return rc == 0 && id == LIS2DUX12_ID;
}

bool Lis2dux12::SetMode(lis2dux12_odr_t odr, lis2dux12_fs_t fs,
                        lis2dux12_bw_t bw) {
  mode_.odr = odr;
  mode_.fs = fs;
  mode_.bw = bw;
  return lis2dux12_mode_set(&dev_ctx_, &mode_) == 0;
}

bool Lis2dux12::ReadData(AccelData* data) {
  lis2dux12_xl_data_t xl{};
  if (lis2dux12_xl_data_get(&dev_ctx_, &mode_, &xl) != 0) return false;
  lis2dux12_outt_data_t t{};
  if (lis2dux12_outt_data_get(&dev_ctx_, &t) != 0) return false;
  data->x_mg = xl.mg[0];
  data->y_mg = xl.mg[1];
  data->z_mg = xl.mg[2];
  data->temp_deg_c = t.heat.deg_c;
  return true;
}

bool Lis2dux12::IsDataReady(bool* ready) {
  lis2dux12_status_t status{};
  if (lis2dux12_status_get(&dev_ctx_, &status) != 0) return false;
  *ready = status.drdy;
  return true;
}

bool Lis2dux12::EnterSleep() {
  saved_odr_ = mode_.odr;
  mode_.odr = LIS2DUX12_OFF;
  if (lis2dux12_mode_set(&dev_ctx_, &mode_) != 0) {
    mode_.odr = saved_odr_;
    return false;
  }
  return true;
}

bool Lis2dux12::ExitSleep() {
  mode_.odr = saved_odr_;
  return lis2dux12_mode_set(&dev_ctx_, &mode_) == 0;
}

bool Lis2dux12::SetInt2WakeUpThreshold(uint8_t threshold, bool x_en,
                                       bool y_en, bool z_en) {
  (void)x_en; (void)y_en; (void)z_en;  // no per-axis enables on this part
  lis2dux12_wakeup_config_t wk{};
  if (threshold < 64u) {
    wk.wake_ths_weight = 1;              // 1 LSB = FS/256
    wk.wake_ths = threshold & 0x3Fu;
  } else {
    wk.wake_ths_weight = 0;              // 1 LSB = FS/64
    wk.wake_ths = (threshold / 4u) & 0x3Fu;
  }
  wk.wake_dur = LIS2DUX12_0_ODR;
  wk.sleep_dur = 0;
  wk.wake_enable = LIS2DUX12_SLEEP_OFF;
  wk.inact_odr = LIS2DUX12_ODR_NO_CHANGE;
  if (lis2dux12_wakeup_config_set(&dev_ctx_, wk) != 0) return false;

  lis2dux12_int_config_t ic{};
  ic.int_cfg = LIS2DUX12_INT_LATCHED;
  if (lis2dux12_int_config_set(&dev_ctx_, &ic) != 0) return false;

  lis2dux12_pin_int_route_t route{};
  if (lis2dux12_pin_int1_route_get(&dev_ctx_, &route) != 0) return false;
  route.wake_up = 1;
  return lis2dux12_pin_int1_route_set(&dev_ctx_, &route) == 0;
}

bool Lis2dux12::ConfigureTap(lis2dux12_axis_t axis, uint8_t peak_ths,
                             bool single_on, bool double_on) {
  // Tap detection wants >= 200 Hz; 400 Hz low-power keeps current modest.
  if (mode_.odr != LIS2DUX12_400Hz_LP && mode_.odr != LIS2DUX12_800Hz_LP &&
      mode_.odr != LIS2DUX12_400Hz_HP && mode_.odr != LIS2DUX12_800Hz_HP) {
    if (!SetMode(LIS2DUX12_400Hz_LP, mode_.fs, mode_.bw)) return false;
  }
  // Timing fields: ST lis2dux12 tap example defaults (AN5909 §9) — tune
  // with an experiment once the event pipeline is validated on the board.
  lis2dux12_tap_config_t tap{};
  tap.axis = axis;
  tap.inverted_peak_time = 2;
  tap.pre_still_ths = 1;
  tap.post_still_ths = 1;
  tap.post_still_time = 1;
  tap.shock_wait_time = 2;
  tap.latency = 4;
  tap.wait_end_latency = 1;
  tap.peak_ths = peak_ths & 0x3Fu;
  tap.rebound = 1;
  tap.pre_still_start = 1;
  tap.pre_still_n = 1;
  tap.single_tap_on = single_on ? 1 : 0;
  tap.double_tap_on = double_on ? 1 : 0;
  tap.triple_tap_on = 0;
  if (lis2dux12_tap_config_set(&dev_ctx_, tap) != 0) return false;

  lis2dux12_pin_int_route_t route{};
  if (lis2dux12_pin_int1_route_get(&dev_ctx_, &route) != 0) return false;
  route.tap = 1;
  return lis2dux12_pin_int1_route_set(&dev_ctx_, &route) == 0;
}

bool Lis2dux12::SetInt2DoubleTap() {
  return ConfigureTap(LIS2DUX12_TAP_ON_Z, 3, false, true);
}

bool Lis2dux12::PollEvents(EventSources* out) {
  lis2dux12_all_sources_t src{};
  if (lis2dux12_all_sources_get(&dev_ctx_, &src) != 0) return false;
  if (out) {
    out->drdy = src.drdy;
    out->single_tap = src.single_tap;
    out->double_tap = src.double_tap;
    out->triple_tap = src.triple_tap;
    out->wake_up = src.wake_up;
  }
  return true;
}

bool Lis2dux12::DisableEvents() {
  bool ok = true;
  lis2dux12_int_config_t ic{};
  ic.int_cfg = LIS2DUX12_INT_DISABLED;
  ok &= lis2dux12_int_config_set(&dev_ctx_, &ic) == 0;
  lis2dux12_pin_int_route_t none{};
  ok &= lis2dux12_pin_int1_route_set(&dev_ctx_, &none) == 0;
  ok &= lis2dux12_pin_int2_route_set(&dev_ctx_, &none) == 0;
  ok &= PollEvents(nullptr);  // read-clear latched flags
  return ok;
}

bool Lis2dux12::ClearWakeUpInterrupt() { return PollEvents(nullptr); }

}  // namespace coralmicro
