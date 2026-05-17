/*
 * sim/modsentai_sim.c — Phase 1.5 sentai module bindings for SIM.
 *
 * Provides hardware-independent bindings so the REPL feels like the real
 * board.  All bindings reuse QSTR table entries already present in the
 * firmware build (examples/sentai_runtime/micropython_embed/genhdr/
 * qstrdefs.generated.h).  No QSTR regen needed.
 *
 * What we expose now:
 *   sentai.version()       -> "SentAI SIM v1.0 (Phase 1.5) ..."
 *   sentai.verbose([on])   -> bool, gates [SIM] log output (set/get)
 *   sentai.io.led_on()     -> printf "[LED] ON"  (no real LED in SIM)
 *   sentai.io.led_off()    -> printf "[LED] OFF"
 *   sentai.rtos.sleep_ms(ms) -> vTaskDelay (real FreeRTOS, EINTR-safe)
 *   sentai.diag.dmesg()    -> last ~4 KB of stdout, ring-buffered
 *   sentai.sys.reset()     -> exit(0) — clean SIM exit
 *
 * Phase 2+ will add sentai.fs.* (FileX/LevelX), sentai.crazy.* (UART
 * socket → CrazySim), sentai.flow.* (camera socket → Gazebo), sentai.tpu.*
 * (libedgetpu Linux).  Same API contract as ARM firmware.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#include "py/runtime.h"
#include "py/objstr.h"
#include "py/objmodule.h"
#include "py/objtuple.h"

#include "FreeRTOS.h"
#include "task.h"

#include "build_version.h"

/* ---- sentai.version() ---- */
static mp_obj_t sentai_version(void) {
    /* Mirror the firmware format: "SentAI v1.0 build NNN (timestamp)".
     * SIM has its own build counter (sim/build_version.h), separate from
     * ARM's, so the operator can tell them apart at a glance. */
    static char vers[128];
    int n = snprintf(vers, sizeof(vers),
                     "SentAI SIM v1.0 build %d (%s) - FreeRTOS POSIX + MicroPython embed",
                     BUILD_VERSION, BUILD_TIMESTAMP);
    if (n < 0) n = 0;
    return mp_obj_new_str(vers, (size_t) n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_version_obj, sentai_version);

/* ---- sentai.verbose([on]) ---- */
static int s_verbose = 1;
static mp_obj_t sentai_verbose(size_t n_args, const mp_obj_t *args) {
    int prev = s_verbose;
    if (n_args >= 1) {
        s_verbose = mp_obj_is_true(args[0]) ? 1 : 0;
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_verbose_obj, 0, 1,
                                            sentai_verbose);


/* ============================================================
 * SIM SENTAI MODULE BINDINGS — fragment includes (refactor T0)
 *
 * Each subsystem lives in its own .c file but they are #include'd
 * here so the whole thing stays a single translation unit.  That
 * preserves the original `static` linkage between sections (helpers
 * like sim_dmesg_append + verbose flag stay file-private without
 * needing a shared header).
 *
 * Ordering matters: later sections may reference helpers/state
 * defined earlier (e.g. fs uses sim_dmesg_append from diag, journal
 * uses io for verbose printf).  Don't reorder casually — match the
 * ordering documented in Sim.md §10y.
 *
 * To add a new SIM subsystem:
 *   1. drop a `modsentai_sim_<name>.c` fragment file in `sim/`,
 *   2. add a `#include` line below,
 *   3. add a `{ MP_ROM_QSTR(MP_QSTR_<name>), MP_ROM_PTR(&sentai_<name>_module) }`
 *      entry to `sentai_globals_table` at the bottom,
 *   4. if any SIM-only QSTR is used, append it to
 *      `examples/sentai_runtime/qstrdefs_sim_extra.h` + regen QSTRs
 *      (see CLAUDE.md §QSTR regen).
 * ============================================================ */

#include "modsentai_sim_io.c"
#include "modsentai_sim_rtos.c"
#include "modsentai_sim_diag.c"
#include "modsentai_sim_sys.c"
#include "modsentai_sim_fs.c"
#include "modsentai_sim_journal.c"      /* sentai.sim.journal_*  */
#include "modsentai_sim_camera.c"
#include "modsentai_sim_flow.c"
#include "modsentai_sim_tpu.c"
#include "modsentai_sim_pipeline.c"
#include "modsentai_sim_link.c"



// ObjectsPlan L2 — sentai.objects.  Pure data-layer binding shared with
// ARM via #include of the canonical source under examples/sentai_runtime/.
// Keeps ARM and SIM exposing an identical surface; backing store + math
// live in sentai_objects.{h,cc} (also in this build via sim/CMakeLists.txt).
#include "../examples/sentai_runtime/bindings/modsentai_objects.c"
// ObjectsPlan L3 — sentai.places.  Same #include pattern as L2; the
// libh3_sim target is linked by sim/CMakeLists.txt so the H3 calls
// (cell_at, neighbors, gridDisk inside query) resolve at link time.
#include "../examples/sentai_runtime/bindings/modsentai_places.c"
// ObjectsPlan L4 — sentai.servo (action layer skeleton).  Pure FSM +
// trace ring shared with ARM via the same #include pattern.
#include "../examples/sentai_runtime/bindings/modsentai_servo.c"
// ObjectsPlan L5 — sentai.object_lifter (inverse-depth EKF landmark
// lifter). Pure float math; shared with ARM via the same #include.
#include "../examples/sentai_runtime/bindings/modsentai_object_lifter.c"
// ObjectsPlan OP-S6-W1 — sentai.calib (camera-to-body Kabsch).  Shared
// with ARM via the same #include; the .cc file branches on
// SENTAI_HAVE_FXUSER to pick FileX (ARM) vs host stdio (SIM).
#include "../examples/sentai_runtime/bindings/modsentai_calib.c"
// ObjectsPlan L6 — sentai.explore (mission FSM).  Pure FSM that wraps
// L4 servo + L5 lifter; shared with ARM via the same #include.
#include "../examples/sentai_runtime/bindings/modsentai_explore.c"

// Task #39 — sentai.crazy over CRTP-UDP to cf2 SITL (SIM-only).  ARM
// uses examples/sentai_runtime/modsentai_crazy.c (CPX-over-UART); this
// is the SIM-flavor binding with the same Python surface (arm /
// takeoff / land / go_to / hover / send_crtp / recv_crtp / stats).
#include "modsentai_sim_crazy.c"


static const mp_rom_map_elem_t sentai_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai) },
    { MP_ROM_QSTR(MP_QSTR_version),  MP_ROM_PTR(&sentai_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_verbose),  MP_ROM_PTR(&sentai_verbose_obj) },
    { MP_ROM_QSTR(MP_QSTR_io),       MP_ROM_PTR(&sentai_io_module) },
    { MP_ROM_QSTR(MP_QSTR_rtos),     MP_ROM_PTR(&sentai_rtos_module) },
    { MP_ROM_QSTR(MP_QSTR_diag),     MP_ROM_PTR(&sentai_diag_module) },
    { MP_ROM_QSTR(MP_QSTR_sys),      MP_ROM_PTR(&sentai_sys_module) },
    { MP_ROM_QSTR(MP_QSTR_fs),       MP_ROM_PTR(&sentai_fs_module) },
    { MP_ROM_QSTR(MP_QSTR_camera),   MP_ROM_PTR(&sentai_camera_module) },
    { MP_ROM_QSTR(MP_QSTR_flow),     MP_ROM_PTR(&sentai_flow_module) },
    { MP_ROM_QSTR(MP_QSTR_tpu),      MP_ROM_PTR(&sentai_tpu_module) },
    { MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&sentai_pipeline_module) },
    { MP_ROM_QSTR(MP_QSTR_link),     MP_ROM_PTR(&sentai_link_module) },
    { MP_ROM_QSTR(MP_QSTR_objects),  MP_ROM_PTR(&sentai_objects_module) },
    { MP_ROM_QSTR(MP_QSTR_places),   MP_ROM_PTR(&sentai_places_module) },
    { MP_ROM_QSTR(MP_QSTR_servo),    MP_ROM_PTR(&sentai_servo_module) },
    { MP_ROM_QSTR(MP_QSTR_object_lifter), MP_ROM_PTR(&sentai_object_lifter_module) },
    { MP_ROM_QSTR(MP_QSTR_calib),    MP_ROM_PTR(&sentai_calib_module) },
    { MP_ROM_QSTR(MP_QSTR_explore),  MP_ROM_PTR(&sentai_explore_module) },
    { MP_ROM_QSTR(MP_QSTR_crazy),    MP_ROM_PTR(&sentai_crazy_module) },
    { MP_ROM_QSTR(MP_QSTR_sim),      MP_ROM_PTR(&sentai_sim_module) },
};
static MP_DEFINE_CONST_DICT(sentai_globals, sentai_globals_table);

/* This symbol is referenced from moduledefs.h (generated for the firmware
 * build).  Replaces the empty stub previously in main_sim.c. */
const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_globals,
};
