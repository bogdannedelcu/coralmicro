// sentai_flow_shared_storage_sim.c -- POSIX/SIM storage for FLOW_SHARED().
//
// The FlowTask implementation is shared with ARM in
// examples/sentai_runtime/flow_task.cc.  SIM only provides the backing
// storage symbol because it cannot dereference the RT1176 fixed address.

#include <stdint.h>
#include <string.h>

#include "examples/sentai_runtime/flow_shared.h"

volatile flow_shared_t g_sentai_flow_shared;

void sentai_flow_sim_publish_snapshot(uint32_t seq,
                                      int32_t dx_q1000,
                                      int32_t dy_q1000,
                                      uint32_t conf,
                                      uint64_t latency_us,
                                      int32_t dz_q1000,
                                      uint32_t dz_conf,
                                      const uint8_t* gray80x60) {
    (void)dz_q1000;
    (void)dz_conf;
    if (g_sentai_flow_shared.magic != FLOW_SHARED_MAGIC) {
        memset((void*)&g_sentai_flow_shared, 0, sizeof(g_sentai_flow_shared));
        g_sentai_flow_shared.magic = FLOW_SHARED_MAGIC;
        g_sentai_flow_shared.version = FLOW_SHARED_VERSION;
        g_sentai_flow_shared.m4_heartbeat = 0xFFFFFFFFu;
        g_sentai_flow_shared.m4_state = FLOW_STATE_IDLE;
    }
    if (gray80x60) {
        memcpy((void*)g_sentai_flow_shared.gray, gray80x60, FLOW_GRAY_PIXELS);
    }
    g_sentai_flow_shared.last_dx = dx_q1000;
    g_sentai_flow_shared.last_dy = dy_q1000;
    g_sentai_flow_shared.last_sad = 0;
    g_sentai_flow_shared.last_confidence = (uint8_t)(conf > 255 ? 255 : conf);
    g_sentai_flow_shared.last_frame_seq = seq;
    g_sentai_flow_shared.frames_processed = seq;
    g_sentai_flow_shared.frame_seq = seq;
    g_sentai_flow_shared.frame_valid = 1;
    g_sentai_flow_shared.last_compute_us = (uint32_t)latency_us;
}
