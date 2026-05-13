// sentai_crazy_stub_sim.c — minimal stubs for the cf2 bridge entry
// points that sentai_anchor_forward.cc and modsentai_sim.c link
// against.  On SIM there is no physical Crazyflie attached over UART
// — cf2 SITL inside Gazebo is talked to by cflib Python from the
// host, not by our firmware.  These stubs return -1 ("bridge not
// running") so the auto-forwarder cleanly skips cf2 on this target.
//
// If/when we add a sentai_crazy_sim.cc that speaks CRTP over a
// distrobox UDS to the cf2 SITL instance, replace these stubs.

#include <stdint.h>

int sentai_crazy_is_running(void) { return 0; }

int sentai_crazy_send_ext_position(float x_m, float y_m, float z_m) {
    (void)x_m; (void)y_m; (void)z_m;
    return -1;
}
