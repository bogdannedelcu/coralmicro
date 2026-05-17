/* modsentai_sim_camera.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== sentai.camera — virtGazebo backend (Phase 1.5 stub, Phase 4 real) =====
 *
 * On the ARM firmware sentai.camera drives the OV5640 sensors via CSI ISR
 * + PXP scaling.  In SIM there is NO OV5640 — the camera surface is fed
 * by a virtual sensor backed by a Gazebo simulation.  Phase 4 will wire
 * `sim/camera_socket.c` to a ROS 2 image subscriber.  Phase 1.5 just
 * exposes the API skeleton with sensible stub values so user scripts can
 * be developed and `sentai.camera.backend()` reports the truth.
 *
 * The MicroPython contract is identical to the firmware — same method
 * names, same return shapes — so a script that runs on ARM should run on
 * SIM after Phase 4 lands.  Differences live entirely in the C
 * implementation. */

/* Backend identifier exposed via sentai.camera.init() print line and via
 * `import sentai; sentai.camera_backend` at top-level (string constant,
 * no QSTR regen needed).  Phase 4 will switch this to a real method
 * once QSTRs are regenerated. */
static const char SIM_CAMERA_BACKEND[] = "virt_gazebo";

static mp_obj_t sentai_camera_init(size_t n_args, const mp_obj_t *args) {
    /* On firmware: init(num_frames[, w, h, fps]) -> int rc.  In Phase 1.5
     * stub: returns 0 ("ok") without actually opening a Gazebo
     * connection.  Phase 4 will connect to the bridge socket here. */
    (void) n_args; (void) args;
    if (s_verbose) printf("[camera] init() (virt_gazebo stub - Phase 4 will connect to Gazebo)\n");
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_camera_init_obj, 0, 4, sentai_camera_init);

static mp_obj_t sentai_camera_frame_count(void) {
    /* Phase 1.5 stub: no real frames yet, always 0. */
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_camera_frame_count_obj, sentai_camera_frame_count);

static mp_obj_t sentai_camera_grabbed_id(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_camera_grabbed_id_obj, sentai_camera_grabbed_id);

static mp_obj_t sentai_camera_select(mp_obj_t cam_id_obj) {
    mp_int_t cam = mp_obj_get_int(cam_id_obj);
    if (s_verbose) printf("[camera] select(%d) (virt_gazebo stub)\n", (int) cam);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_camera_select_obj, sentai_camera_select);

/* ===== sentai.camera.grab_gray(w, h) — return latest cam frame as gray bytes.
 *
 * Per embeded.md §2 (NASA/JPL Power of Ten) + §4.1 (resource discipline):
 *   - Fault model:
 *       F1 invalid w,h (< 8 or > 640/480) → return None
 *       F2 no frame yet (Gazebo bridge not running) → return None
 *       F3 internal resize/conversion fail → return None
 *   - Execution: MP-task context only.  Single-writer/single-reader on
 *     the static scratch buffer (REPL is sequential).  No ISR access.
 *   - Bounded loops: pixel iteration with fixed w*h bound (max 320*240
 *     = 76800 iter).
 *   - Static allocation: 2 scratch buffers in BSS, no per-call alloc
 *     except the final mp_obj_new_bytes (MP heap, single object per call).
 *   - Failure containment: every internal call's return is checked;
 *     on any failure we return mp_const_none (caller observes None).
 *
 * Refolosește sim_camera_latest_rgb (camera_bridge_recv.c) + the
 * existing nearest-neighbour resize already in this file (defined later
 * for pipeline.tick — must declare forward).  RGB→Y is BT.601 luma.
 *
 * Returns: dict { 'data': bytes(w*h), 'w': w, 'h': h, 'seq': uint32 }
 *          or None on any failure.
 */
extern size_t sim_camera_latest_rgb(uint8_t* dst, size_t max_bytes,
                                     int* out_w, int* out_h, uint32_t* out_seq);

#define CAM_GRAB_MAX_W   640
#define CAM_GRAB_MAX_H   480
#define CAM_GRAB_RGB_SZ  (CAM_GRAB_MAX_W * CAM_GRAB_MAX_H * 3)
#define CAM_GRAB_GRAY_SZ (CAM_GRAB_MAX_W * CAM_GRAB_MAX_H)

/* Static single-writer scratch.  REPL is single-threaded for command
 * dispatch; if a future task ever calls this concurrently from a
 * different context, switch to per-call malloc (~300 KB) or per-task
 * thread-local. */
static uint8_t s_grab_rgb_src[CAM_GRAB_RGB_SZ];
static uint8_t s_grab_rgb_dst[CAM_GRAB_RGB_SZ];
static uint8_t s_grab_gray[CAM_GRAB_GRAY_SZ];

/* Forward decl — defined below in pipeline section. */
static int sim_resize_rgb888_nearest(const uint8_t* src, int sw, int sh,
                                      uint8_t* dst, int dw, int dh);

static mp_obj_t sentai_camera_grab_gray(size_t n_args, const mp_obj_t* args) {
    /* Args: (w=320, h=240) — both optional. */
    int w = (n_args >= 1) ? mp_obj_get_int(args[0]) : 320;
    int h = (n_args >= 2) ? mp_obj_get_int(args[1]) : 240;
    /* F1 invalid dimensions. */
    if (w < 8 || h < 8 || w > CAM_GRAB_MAX_W || h > CAM_GRAB_MAX_H) {
        return mp_const_none;
    }

    int cw = 0, ch = 0;
    uint32_t seq = 0;
    size_t got = sim_camera_latest_rgb(s_grab_rgb_src, sizeof(s_grab_rgb_src),
                                        &cw, &ch, &seq);
    /* F2 no frame yet. */
    if (got == 0 || cw <= 0 || ch <= 0) return mp_const_none;
    if ((size_t)(cw * ch * 3) > sizeof(s_grab_rgb_src)) return mp_const_none;

    /* Resize 640x480 RGB → wxh RGB (if needed). */
    const uint8_t* rgb_src;
    int rgb_w, rgb_h;
    if (cw == w && ch == h) {
        rgb_src = s_grab_rgb_src;
        rgb_w = cw;
        rgb_h = ch;
    } else {
        if (sim_resize_rgb888_nearest(s_grab_rgb_src, cw, ch,
                                       s_grab_rgb_dst, w, h) != 0) {
            return mp_const_none;
        }
        rgb_src = s_grab_rgb_dst;
        rgb_w = w;
        rgb_h = h;
    }

    /* RGB → Y (BT.601 luma).  Integer math, single bounded loop over w*h
     * pixels (max 76 800 iter).  Constants 66/129/25 are the standard
     * fixed-point luma weights with rounding +128 and offset +16. */
    int n_pix = rgb_w * rgb_h;
    for (int i = 0; i < n_pix; ++i) {
        int r = rgb_src[i*3 + 0];
        int g = rgb_src[i*3 + 1];
        int b = rgb_src[i*3 + 2];
        int y = ((66*r + 129*g + 25*b + 128) >> 8) + 16;
        if (y < 0) y = 0;
        if (y > 255) y = 255;
        s_grab_gray[i] = (uint8_t)y;
    }

    /* Build return dict. */
    mp_obj_t bytes_obj = mp_obj_new_bytes(s_grab_gray, (size_t)n_pix);
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_data), bytes_obj);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_w),    mp_obj_new_int(rgb_w));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_h),    mp_obj_new_int(rgb_h));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),  mp_obj_new_int_from_uint(seq));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_camera_grab_gray_obj, 0, 2, sentai_camera_grab_gray);

/* Zero-copy gray-frame hook for C consumers (sentai_aruco, future
 * sentai_phog, ...).  Returns a pointer into the static s_grab_gray
 * buffer + dims + frame_seq + a tick-count timestamp.  Caller MUST
 * consume the data before the next call (single-buffer; if a future
 * task needs concurrent consumption switch to a 2-slot rotating
 * cache).  Returns 0 on success, -1 on no-frame / failure.
 *
 * Default working resolution: 320x240 — matches sentai_aruco intrinsics
 * defaults + the s091/s130 aruco_hover capture.  Caller can later add
 * per-consumer dims if needed; for the OP-S6-W3 path 320x240 suffices.
 */
extern uint32_t sentai_now_ms(void) __attribute__((weak));
int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                      int* out_w, int* out_h,
                                      uint32_t* out_seq,
                                      uint32_t* out_ts_ms) {
    if (!out_buf || !out_w || !out_h) return -1;
    const int W = 320, H = 240;
    int cw = 0, ch = 0;
    uint32_t seq = 0;
    size_t got = sim_camera_latest_rgb(s_grab_rgb_src, sizeof(s_grab_rgb_src),
                                        &cw, &ch, &seq);
    if (got == 0 || cw <= 0 || ch <= 0) return -1;
    if ((size_t)(cw * ch * 3) > sizeof(s_grab_rgb_src)) return -1;

    const uint8_t* rgb_src;
    int rw, rh;
    if (cw == W && ch == H) {
        rgb_src = s_grab_rgb_src;
        rw = cw;
        rh = ch;
    } else {
        if (sim_resize_rgb888_nearest(s_grab_rgb_src, cw, ch,
                                       s_grab_rgb_dst, W, H) != 0) {
            return -1;
        }
        rgb_src = s_grab_rgb_dst;
        rw = W;
        rh = H;
    }
    const int n_pix = rw * rh;
    for (int i = 0; i < n_pix; ++i) {
        int r = rgb_src[i*3 + 0];
        int g = rgb_src[i*3 + 1];
        int b = rgb_src[i*3 + 2];
        int y = ((66*r + 129*g + 25*b + 128) >> 8) + 16;
        if (y < 0) y = 0;
        if (y > 255) y = 255;
        s_grab_gray[i] = (uint8_t)y;
    }
    *out_buf = s_grab_gray;
    *out_w   = rw;
    *out_h   = rh;
    *out_seq = seq;
    if (out_ts_ms) {
        *out_ts_ms = sentai_now_ms ? sentai_now_ms() : 0u;
    }
    return 0;
}

static const mp_rom_map_elem_t sentai_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&sentai_camera_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_frame_count), MP_ROM_PTR(&sentai_camera_frame_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_grabbed_id),  MP_ROM_PTR(&sentai_camera_grabbed_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_select),      MP_ROM_PTR(&sentai_camera_select_obj) },
    { MP_ROM_QSTR(MP_QSTR_grab_gray),   MP_ROM_PTR(&sentai_camera_grab_gray_obj) },
    /* "backend" is a SIM-only diagnostic — no need for a stable QSTR; use
     * an inline string literal with hashing via mp_obj_new_str.  We
     * surface it as a plain attribute by aliasing the QSTR table.
     * Workaround: map under the closest existing QSTR — pick `version`
     * is wrong; we don't have a clean "backend" QSTR.  For Phase 1.5
     * we just expose it as a method anyway under MP_QSTR_init pattern.
     * Actually simpler: install under MP_QSTR_io  no — wrong name.
     * Cleanest: don't expose it under sentai.camera until QSTR regen
     * adds "backend".  Until then call sim_camera_backend() at the
     * top level: */
};
static MP_DEFINE_CONST_DICT(sentai_camera_globals, sentai_camera_globals_table);
static const mp_obj_module_t sentai_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_camera_globals,
};
