// modsentai_flow.c — MicroPython bindings for sentai.flow
//
// Two code paths live under this namespace:
//
//   1. M7-side block-match task (flow_task.cc).  Mutually exclusive
//      with sentai.pipeline — both share the PXP + camera buffer.
//        sentai.flow.start / stop / running / read / stats
//
//   2. M4-offload heartbeat (Phase 1 — prove the M4 boots).  Reads
//      the shared-memory struct populated by flow_task_m4.cc.
//        sentai.flow.m4_heartbeat() -> dict

#include <string.h>

#include "flow_shared.h"

// The shared struct lives at the fixed OCRAM address FLOW_SHARED_ADDR.
// Accessed via the FLOW_SHARED() macro — volatile cast ensures the
// compiler always re-reads memory (the M4 writes asynchronously).

extern int  sentai_flow_start(int cam_id);
extern int  sentai_flow_stop(void);
extern int  sentai_flow_is_running(void);
extern int  sentai_flow_read(int32_t* dx, int32_t* dy,
                             uint32_t* sad, uint32_t* frame_seq,
                             uint32_t* age_ms, uint8_t* confidence,
                             uint8_t* cam_id);
extern void sentai_flow_stats(uint32_t* frames_processed,
                              uint32_t* frames_dropped,
                              uint32_t* grab_fail,
                              uint32_t* pxp_fail,
                              uint32_t* avg_fps_x10);

// sentai.flow.start([cam_id]) -> int
static mp_obj_t mod_sentai_flow_start(size_t n_args, const mp_obj_t *args) {
    int cam_id = 0;
    if (n_args >= 1) cam_id = mp_obj_get_int(args[0]);
    int rc = sentai_flow_start(cam_id);
    if (rc != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("flow.start failed (%d)"), rc);
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_start_obj,
                                            0, 1, mod_sentai_flow_start);

// sentai.flow.stop() -> int
static mp_obj_t mod_sentai_flow_stop(void) {
    return mp_obj_new_int(sentai_flow_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_stop_obj, mod_sentai_flow_stop);

// sentai.flow.running() -> bool
static mp_obj_t mod_sentai_flow_running(void) {
    return mp_obj_new_bool(sentai_flow_is_running() != 0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_running_obj,
                                  mod_sentai_flow_running);

// sentai.flow.read() -> dict
//
// When the task has not produced a sample yet (first frame after
// start()), age_ms comes back as the sentinel 0xFFFFFFFF and
// frame_seq is zero.  Callers should check frame_seq != 0 before
// acting on the delta.
static mp_obj_t mod_sentai_flow_read(void) {
    int32_t  dx = 0, dy = 0;
    uint32_t sad = 0, frame_seq = 0, age_ms = 0xFFFFFFFFu;
    uint8_t  confidence = 0, cam_id = 0;
    int rc = sentai_flow_read(&dx, &dy, &sad, &frame_seq,
                              &age_ms, &confidence, &cam_id);

    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dx),
                      mp_obj_new_int(dx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dy),
                      mp_obj_new_int(dy));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sad),
                      mp_obj_new_int_from_uint(sad));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                      mp_obj_new_int_from_uint(frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_age_ms),
                      mp_obj_new_int_from_uint(age_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_confidence),
                      mp_obj_new_int(confidence));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_id),
                      mp_obj_new_int(cam_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_status),
                      mp_obj_new_int(rc));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_read_obj,
                                  mod_sentai_flow_read);

// sentai.flow.stats() -> dict
static mp_obj_t mod_sentai_flow_stats(void) {
    uint32_t processed = 0, dropped = 0, grab_fail = 0,
             pxp_fail = 0, fps_x10 = 0;
    sentai_flow_stats(&processed, &dropped, &grab_fail, &pxp_fail, &fps_x10);

    mp_obj_t d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_processed),
                      mp_obj_new_int_from_uint(processed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_dropped),
                      mp_obj_new_int_from_uint(dropped));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_fail),
                      mp_obj_new_int_from_uint(grab_fail));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pxp_fail),
                      mp_obj_new_int_from_uint(pxp_fail));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_avg_fps_x10),
                      mp_obj_new_int_from_uint(fps_x10));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_stats_obj,
                                  mod_sentai_flow_stats);

// sentai.flow.m4_heartbeat() -> dict
//
// Reports the latest heartbeat counter published by the M4 core.
// Returns 0 for every field when the M4 is absent (shared magic not
// set).  Cheap poll — just reads four volatile uint32s out of OCRAM.
static mp_obj_t mod_sentai_flow_m4_heartbeat(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    uint32_t magic   = sh->magic;
    uint32_t version = sh->version;
    uint32_t beat    = sh->m4_heartbeat;
    uint32_t tick    = sh->m4_tick_ms;
    uint32_t boot    = sh->m4_boot_ts_ms;

    mp_obj_t d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive),
                      mp_obj_new_bool(magic == FLOW_SHARED_MAGIC));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_version),
                      mp_obj_new_int_from_uint(version));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_heartbeat),
                      mp_obj_new_int_from_uint(beat));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_tick_ms),
                      mp_obj_new_int_from_uint(tick));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_boot_ts_ms),
                      mp_obj_new_int_from_uint(boot));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_m4_heartbeat_obj,
                                  mod_sentai_flow_m4_heartbeat);

// sentai.flow.m4_enable() -> int
//   0  : M4 core started and alive
//  -1  : no M4 image linked into this firmware
//  -2  : M4 did not respond within 2s
// Irreversible in the current build — call once at startup.
extern int sentai_flow_m4_enable(void);
static mp_obj_t mod_sentai_flow_m4_enable(void) {
    int rc = sentai_flow_m4_enable();
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_m4_enable_obj,
                                  mod_sentai_flow_m4_enable);

// sentai.flow.m4_start([cam_id=0]) / m4_stop() — edge-triggered
// command channel over shared memory.  Starts/stops the M4 SAD
// worker AND toggles the PrepTask publish hook.
//
//   cam_id: 0 (default) = cam0 front, 1 = cam1 back (signs flip —
//           see paper/flow_body_frame.md).  Caller must also call
//           sentai.camera.select(cam_id) so the pipeline delivers
//           frames from the chosen sensor.
extern int sentai_flow_m4_cmd_start(int cam_id);
extern int sentai_flow_m4_cmd_stop(void);
static mp_obj_t mod_sentai_flow_m4_start(size_t n_args,
                                          const mp_obj_t *args) {
    int cam_id = 0;
    if (n_args >= 1) cam_id = mp_obj_get_int(args[0]);
    int rc = sentai_flow_m4_cmd_start(cam_id);
    if (rc != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("flow.m4_start failed (%d)"), rc);
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_m4_start_obj,
                                            0, 1, mod_sentai_flow_m4_start);
static mp_obj_t mod_sentai_flow_m4_stop(void) {
    return mp_obj_new_int(sentai_flow_m4_cmd_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_m4_stop_obj,
                                  mod_sentai_flow_m4_stop);

// sentai.flow.m4_read() -> dict with latest M4 SAD result + stats.
// Reads are cheap (handful of volatile loads) so a poll loop at
// 50 Hz is fine for an autopilot feeder.
static mp_obj_t mod_sentai_flow_m4_read(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    uint32_t magic = sh->magic;
    int alive = (magic == FLOW_SHARED_MAGIC);

    mp_obj_t d = mp_obj_new_dict(12);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive), mp_obj_new_bool(alive));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_state),
                      mp_obj_new_int_from_uint(sh->m4_state));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dx),
                      mp_obj_new_int(sh->last_dx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dy),
                      mp_obj_new_int(sh->last_dy));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sad),
                      mp_obj_new_int_from_uint(sh->last_sad));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_confidence),
                      mp_obj_new_int(sh->last_confidence));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                      mp_obj_new_int_from_uint(sh->last_frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_processed),
                      mp_obj_new_int_from_uint(sh->frames_processed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_dropped),
                      mp_obj_new_int_from_uint(sh->frames_dropped));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_avg_compute_us),
                      mp_obj_new_int_from_uint(sh->avg_compute_us));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_id),
                      mp_obj_new_int(sh->frame_cam_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_heartbeat),
                      mp_obj_new_int_from_uint(sh->m4_heartbeat));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_m4_read_obj,
                                  mod_sentai_flow_m4_read);

// sentai.flow.m4_body_read() -> dict with body-frame velocity.
//
// Converts raw (dx, dy) in image grid to drone body-frame (body_fw,
// body_left) per the E29 orientation calibration.  See
// paper/flow_body_frame.md for the convention.
//
//   cam0:  body_fw = -dx   body_left = +dy
//   cam1:  body_fw = +dx   body_left = -dy
//
// If the M4 isn't running or the frame_seq is stale (==0), the
// body-frame fields come back as 0 and alive=False.
static mp_obj_t mod_sentai_flow_m4_body_read(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    int alive = (sh->magic == FLOW_SHARED_MAGIC);
    int32_t dx = sh->last_dx;
    int32_t dy = sh->last_dy;
    int cam_id = sh->frame_cam_id;

    int32_t body_fw, body_left;
    if (cam_id == 0) {
        body_fw   = -dx;
        body_left = +dy;
    } else {
        body_fw   = +dx;
        body_left = -dy;
    }

    mp_obj_t d = mp_obj_new_dict(7);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive), mp_obj_new_bool(alive));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_body_fw),
                      mp_obj_new_int(body_fw));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_body_left),
                      mp_obj_new_int(body_left));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dx), mp_obj_new_int(dx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dy), mp_obj_new_int(dy));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                      mp_obj_new_int_from_uint(sh->last_frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_id),
                      mp_obj_new_int(cam_id));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_m4_body_read_obj,
                                  mod_sentai_flow_m4_body_read);

// sentai.flow.m4_gray_snap([into]) -> bytes / int
//
// Returns a snapshot of the 40×30 gray buffer that M7 publishes
// to M4 for SAD matching.  Intended as a debug aid.
//
//   no argument  : allocates a fresh `bytes` each call (convenient
//                  for one-shot inspection but WILL fragment the
//                  MicroPython heap at > 1 Hz — ~1.2 KB/call).
//   bytearray    : copies into the caller-owned bytearray (must be
//                  ≥ FLOW_GRAY_PIXELS bytes), returns the number
//                  of bytes copied.  Zero allocation — safe in hot
//                  loops / long diagnostics.
//
// Layout: row-major, FLOW_GRAY_W bytes per row, FLOW_GRAY_H rows.
static mp_obj_t mod_sentai_flow_m4_gray_snap(size_t n_args,
                                              const mp_obj_t *args) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    const uint8_t* src = (const uint8_t*)sh->gray;

    if (n_args == 0) {
        // One-shot allocation path (debug convenience; allocates
        // ~1.2 KB on MP heap per call — do NOT use in loops).
        return mp_obj_new_bytes(src, FLOW_GRAY_PIXELS);
    }

    // Zero-allocation path: write into caller's bytearray.
    mp_buffer_info_t bufinfo;
    if (!mp_get_buffer(args[0], &bufinfo, MP_BUFFER_WRITE)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "gray_snap target must be a writable bytearray"));
    }
    if (bufinfo.len < FLOW_GRAY_PIXELS) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "gray_snap target buffer too small (need FLOW_GRAY_PIXELS)"));
    }
    memcpy(bufinfo.buf, src, FLOW_GRAY_PIXELS);
    return mp_obj_new_int(FLOW_GRAY_PIXELS);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_m4_gray_snap_obj,
                                           0, 1, mod_sentai_flow_m4_gray_snap);

// sentai.flow.detail_score() -> int
//
// Gradient energy of the current 40×30 gray buffer in shared OCRAM,
// ×100 (basis points).  Higher = more detail = better SAD signal.
// Useful for programmatically picking the best camera ISP preset —
// apply preset, wait for AEC to settle, read score, pick max.
//
// Typical ranges (empirical):
//    0..500      : near-flat scene, SAD cannot lock reliably
//    500..1500   : low texture or dim
//   1500..4000   : healthy indoor scene
//    > 4000      : rich detail / high-contrast (outdoor, markers)
extern uint32_t sentai_flow_detail_score(void);
static mp_obj_t mod_sentai_flow_detail_score(void) {
    return mp_obj_new_int_from_uint(sentai_flow_detail_score());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_detail_score_obj,
                                  mod_sentai_flow_detail_score);

// sentai.flow.gray_stretch([enable]) -> dict
//
// Toggles / queries the auto-level (histogram min-max stretch) pass
// applied to the 80×60 gray buffer before it's published to M4.
//
//   no arg       : return current state + last (vmin,vmax) observed
//   True/False   : enable/disable, return previous state + last vmin/vmax
//
// When enabled, the publisher stretches the decimated frame so the
// darkest pixel maps to 0 and the brightest to 255 — widening the
// dynamic range after the OV5640's AEC pins the output near a fixed
// mean.  Negligible cost (~120 µs per frame).
extern int sentai_flow_gray_stretch_set(int enable);
extern int sentai_flow_gray_stretch_get(uint8_t* vmin, uint8_t* vmax);
static mp_obj_t mod_sentai_flow_gray_stretch(size_t n_args,
                                              const mp_obj_t *args) {
    uint8_t vmin = 0, vmax = 0;
    int prev;
    if (n_args == 0) {
        prev = sentai_flow_gray_stretch_get(&vmin, &vmax);
    } else {
        int want = mp_obj_is_true(args[0]) ? 1 : 0;
        prev = sentai_flow_gray_stretch_set(want);
        sentai_flow_gray_stretch_get(&vmin, &vmax);
    }
    mp_obj_t d = mp_obj_new_dict(3);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_enabled),
                      mp_obj_new_bool(prev != 0));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_vmin),
                      mp_obj_new_int(vmin));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_vmax),
                      mp_obj_new_int(vmax));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_gray_stretch_obj,
                                           0, 1, mod_sentai_flow_gray_stretch);

// ---- module table ----
static const mp_rom_map_elem_t sentai_flow_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_flow) },
    { MP_ROM_QSTR(MP_QSTR_start),        MP_ROM_PTR(&mod_sentai_flow_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),         MP_ROM_PTR(&mod_sentai_flow_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),      MP_ROM_PTR(&mod_sentai_flow_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),         MP_ROM_PTR(&mod_sentai_flow_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),        MP_ROM_PTR(&mod_sentai_flow_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_heartbeat), MP_ROM_PTR(&mod_sentai_flow_m4_heartbeat_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_enable),    MP_ROM_PTR(&mod_sentai_flow_m4_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_start),     MP_ROM_PTR(&mod_sentai_flow_m4_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_stop),      MP_ROM_PTR(&mod_sentai_flow_m4_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_read),      MP_ROM_PTR(&mod_sentai_flow_m4_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_body_read), MP_ROM_PTR(&mod_sentai_flow_m4_body_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_gray_snap), MP_ROM_PTR(&mod_sentai_flow_m4_gray_snap_obj) },
    { MP_ROM_QSTR(MP_QSTR_detail_score), MP_ROM_PTR(&mod_sentai_flow_detail_score_obj) },
    { MP_ROM_QSTR(MP_QSTR_gray_stretch), MP_ROM_PTR(&mod_sentai_flow_gray_stretch_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_flow_globals, sentai_flow_globals_table);
static const mp_obj_module_t sentai_flow_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_flow_globals,
};
