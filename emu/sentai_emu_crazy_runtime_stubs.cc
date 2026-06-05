// Emulator stubs for shared sentai.crazy runtime dependencies.
//
// These are deliberately below the sentai.crazy API boundary.  The
// MicroPython binding and the CRTP/CPX implementation stay shared; only board
// services that are unavailable in the early Renode target are substituted.

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "examples/sentai_runtime/sentai_health.h"

extern "C" void sentai_led_set(int on) {
  (void)on;
}

extern "C" int sentai_mesh_is_running(void) {
  return 0;
}

extern "C" int sentai_link_is_running(void) {
  return 0;
}

extern "C" void sentai_health_init(void) {}
extern "C" void sentai_health_success(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_fail(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_timeout(SubsystemId_t subsys) { (void)subsys; }
extern "C" void sentai_health_set_unavailable(SubsystemId_t subsys) {
  (void)subsys;
}
extern "C" void sentai_health_set_recovering(SubsystemId_t subsys) {
  (void)subsys;
}

extern "C" const HealthRecord_t* sentai_health_get(SubsystemId_t subsys) {
  (void)subsys;
  return nullptr;
}

extern "C" SystemMode_t sentai_health_system_mode(void) {
  return SYS_MODE_NORMAL;
}

extern "C" const char* sentai_health_state_name(HealthState_t state) {
  (void)state;
  return "emu";
}

extern "C" const char* sentai_health_subsys_name(SubsystemId_t subsys) {
  (void)subsys;
  return "emu";
}

extern "C" int sentai_health_summary(char* buf, int buf_size) {
  if (buf && buf_size > 0) buf[0] = '\0';
  return 0;
}

extern "C" void sentai_health_boot_complete(void) {}
extern "C" void sentai_health_set_recovery_mode(void) {}
extern "C" int sentai_health_is_safe_mode(void) { return 0; }

extern "C" __attribute__((weak)) int sentai_logf(const char* tag,
                                                 const char* fmt, ...) {
  (void)tag;
  (void)fmt;
  return 0;
}
