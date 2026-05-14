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
    {&mp_type_tuple}, 23, {
        MP_ROM_QSTR(MP_QSTR_detections),
        MP_ROM_QSTR(MP_QSTR_tracker_update),
        MP_ROM_QSTR(MP_QSTR_tracker_enable),
        MP_ROM_QSTR(MP_QSTR_tracker_reset),
        MP_ROM_QSTR(MP_QSTR_tracker_tracks),
        MP_ROM_QSTR(MP_QSTR_tracker_camera),
        MP_ROM_QSTR(MP_QSTR_tracker_pose),
        MP_ROM_QSTR(MP_QSTR_tracker_event),
        MP_ROM_QSTR(MP_QSTR_step),         // pipeline.step (also for QSTR scan)
        MP_ROM_QSTR(MP_QSTR_predict),
        MP_ROM_QSTR(MP_QSTR_pipeline),
        MP_ROM_QSTR(MP_QSTR_flow),
        MP_ROM_QSTR(MP_QSTR_camera),
        MP_ROM_QSTR(MP_QSTR_dz_q1000),
        MP_ROM_QSTR(MP_QSTR_dz_conf),
        MP_ROM_QSTR(MP_QSTR_set_input),
        // sentai.sim (SIM-only diagnostic utilities — journal API)
        MP_ROM_QSTR(MP_QSTR_sim),
        MP_ROM_QSTR(MP_QSTR_journal_open),
        MP_ROM_QSTR(MP_QSTR_journal_close),
        MP_ROM_QSTR(MP_QSTR_journal_write),
        MP_ROM_QSTR(MP_QSTR_journal_status),
        MP_ROM_QSTR(MP_QSTR_lines),
        MP_ROM_QSTR(MP_QSTR_errors),
    }
};

#endif  // QSTRDEFS_SIM_EXTRA_H_
