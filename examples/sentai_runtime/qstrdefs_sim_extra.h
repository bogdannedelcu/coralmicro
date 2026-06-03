// qstrdefs_sim_extra.h — extra MP_QSTR_* references for the SIM build.
//
// The MicroPython QSTR generator scans firmware-side modsentai_*.c files
// for MP_QSTR_xxx references and emits matching entries in
// micropython_embed/genhdr/qstrdefs.generated.h.  The SIM build
// (sim/modsentai_sim.c) uses the SAME qstrdefs.generated.h, but the
// scanner does not see the SIM file — so any MP_QSTR_xxx that exists
// only in the SIM module would be undeclared at compile time.
//
// This header solves the chicken-and-egg by listing all the SIM-only
// QSTRs as expanded references inside a static const dummy.  It's
// #include'd from modsentai_pipeline.c (which the QSTR scanner does
// process), so the scanner picks up each MP_QSTR_xxx and emits the
// corresponding QDEF.  Costs ~0 bytes on ARM (the dummy const is in
// `.rodata` but the entries themselves are tiny tags consumed by the
// QSTR system anyway).

#ifndef QSTRDEFS_SIM_EXTRA_H_
#define QSTRDEFS_SIM_EXTRA_H_

#include "py/obj.h"
#include "py/runtime.h"

static const mp_rom_obj_tuple_t _sentai_sim_qstr_keepalive __attribute__((unused)) = {
    {&mp_type_tuple}, 46, {
        MP_ROM_QSTR(MP_QSTR_detections),
        MP_ROM_QSTR(MP_QSTR_tracker_update),
        MP_ROM_QSTR(MP_QSTR_tracker_enable),
        MP_ROM_QSTR(MP_QSTR_tracker_reset),
        MP_ROM_QSTR(MP_QSTR_tracker_tracks),
        MP_ROM_QSTR(MP_QSTR_tracker_camera),
        MP_ROM_QSTR(MP_QSTR_tracker_pose),
        MP_ROM_QSTR(MP_QSTR_tracker_event),
        MP_ROM_QSTR(MP_QSTR_predict),
        MP_ROM_QSTR(MP_QSTR_once),
        MP_ROM_QSTR(MP_QSTR_on_detection),
        MP_ROM_QSTR(MP_QSTR_start_once),
        MP_ROM_QSTR(MP_QSTR_pipeline),
        MP_ROM_QSTR(MP_QSTR_flow),
        MP_ROM_QSTR(MP_QSTR_camera),
        MP_ROM_QSTR(MP_QSTR_dz_q1000),
        MP_ROM_QSTR(MP_QSTR_dz_conf),
        MP_ROM_QSTR(MP_QSTR_set_input),
        // s144: sentai.camera.grab_gray return dict keys + method
        MP_ROM_QSTR(MP_QSTR_grab_gray),
        MP_ROM_QSTR(MP_QSTR_data),
        MP_ROM_QSTR(MP_QSTR_w),
        MP_ROM_QSTR(MP_QSTR_h),
        MP_ROM_QSTR(MP_QSTR_play),
        MP_ROM_QSTR(MP_QSTR_replay),
        MP_ROM_QSTR(MP_QSTR_play_stop),
        MP_ROM_QSTR(MP_QSTR_playing),
        MP_ROM_QSTR(MP_QSTR_prep_enable),
        MP_ROM_QSTR(MP_QSTR_prep_disable),
        MP_ROM_QSTR(MP_QSTR_prep_once),
        MP_ROM_QSTR(MP_QSTR_prep_reset),
        MP_ROM_QSTR(MP_QSTR_prep_stats),
        MP_ROM_QSTR(MP_QSTR_frames_total),
        MP_ROM_QSTR(MP_QSTR_frames_with_aux),
        MP_ROM_QSTR(MP_QSTR_producer_overruns),
        MP_ROM_QSTR(MP_QSTR_slot_refcount),
        MP_ROM_QSTR(MP_QSTR_slot_seq),
        // sentai.crazy SIM bindings (Task #39 gap-fill, 2026-05-16):
        // tuple-returning methods avoid needing dict-key QSTRs.
        MP_ROM_QSTR(MP_QSTR_is_running),
        MP_ROM_QSTR(MP_QSTR_recv_crtp),
        // sentai.markers SIM dataset validation helpers.
        MP_ROM_QSTR(MP_QSTR_detect_pgm),
        MP_ROM_QSTR(MP_QSTR_get_detection),
        MP_ROM_QSTR(MP_QSTR_get_detection_tuple),
        MP_ROM_QSTR(MP_QSTR_get_observation),
        MP_ROM_QSTR(MP_QSTR_get_observation_tuple),
        // B8 emulator TPU helpers.
        MP_ROM_QSTR(MP_QSTR_load_image_mem),
        MP_ROM_QSTR(MP_QSTR_image_mem_size),
        MP_ROM_QSTR(MP_QSTR_fps_invoke),
    }
};

#endif  // QSTRDEFS_SIM_EXTRA_H_
