// Minimal C++ stubs for B8.8 USB/EdgeTPU emulator probes.
//
// These represent board peripherals that are not modeled yet in the Renode
// platform.  They are linked only into emu probe targets.

#include "libs/base/gpio.h"

namespace coralmicro {

void GpioInit() {}

void GpioSet(Gpio gpio, bool enable) {
  (void)gpio;
  (void)enable;
}

bool GpioGet(Gpio gpio) {
  // Keep EdgeTpuTask::SetPower(true) from waiting forever if a future probe
  // exercises TPU power sequencing before we model the PMIC GPIOs.
  return gpio == Gpio::kEdgeTpuPgood;
}

void GpioSetMode(Gpio gpio, GpioMode mode) {
  (void)gpio;
  (void)mode;
}

void GpioConfigureInterrupt(Gpio gpio, GpioInterruptMode mode,
                            GpioCallback cb) {
  (void)gpio;
  (void)mode;
  (void)cb;
}

void GpioConfigureInterrupt(Gpio gpio, GpioInterruptMode mode, GpioCallback cb,
                            uint64_t debounce_interval_us) {
  (void)debounce_interval_us;
  GpioConfigureInterrupt(gpio, mode, cb);
}

}  // namespace coralmicro
