// ============== sentai.camera — Camera ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.camera.init(streaming=1, fps=<current g_runtime_fps>) -> int
// streaming=1: continuous mode, streaming=0: trigger mode
// fps: 15/30/45/60/90 (per fsl_ov5640.c VGA table).  When omitted,
// keeps the current g_runtime_fps (defaults to DEMO_CAMERA_FRAME_RATE
// at boot, but survives a sys.reset() that re-enters with a previously
// chosen rate persisted in firmware state).
// Build #980: fps argument added so the user can pick the sensor
// rate at REPL boot time without rebuilding firmware.  Calling
// init() a second time with a different fps returns -11 — the
// CSI receiver re-init path isn't re-entrant on this HAL.  To
// switch fps after a successful init, sentai.sys.reset() the
// board and call init(streaming, new_fps) again.
extern int sentai_cam_init_fps(int streaming, int fps);
extern int sentai_cam_init_full(int streaming, int fps, int hflip, int vflip);
extern int sentai_virtual_camera_select(const char* path);
extern int sentai_virtual_camera_play(const char* dir, int fps, int count);
extern int sentai_virtual_camera_replay(int fps, int count);
extern int sentai_virtual_camera_play_stop(void);
extern int sentai_virtual_camera_playing(void);
extern void sentai_virtual_camera_disable(void);
extern volatile uint32_t g_runtime_fps;
// sentai.camera.init(streaming=1, fps=<runtime>, hflip=0, vflip=1)
//   Defaults reflect the FLOW BASELINE established 2026-05-07:
//   hflip=0, vflip=1 -- with this orientation cam0's image-axis
//   convention (LEFT=fw, BOTTOM=left, TOP=right, RIGHT=back) holds
//   and matches diag/_orientation_cam0_vflip.jpg.  Pass -1 to either
//   to "leave the OV5640 init driver default" (which is H-mirror ON,
//   V-flip OFF -- the original behavior, on-screen text reads correctly
//   for visualization but flow body-frame mapping breaks).
//   Both cameras receive the same orientation; cam1's body-frame
//   mapping under this default is unverified -- run
//   diag/_t_flow_to_drone.py::verify_orientation(cam_id=1) first.
static mp_obj_t mod_sentai_cam_init(size_t n_args, const mp_obj_t *args) {
    int streaming = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    int fps       = (n_args > 1) ? mp_obj_get_int(args[1])
                                  : (int)g_runtime_fps;
    int hflip     = (n_args > 2) ? mp_obj_get_int(args[2]) : 0;
    int vflip     = (n_args > 3) ? mp_obj_get_int(args[3]) : 1;
    return mp_obj_new_int(sentai_cam_init_full(streaming, fps, hflip, vflip));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_init_obj, 0, 4, mod_sentai_cam_init);

// sentai.camera.stop() -> int
static mp_obj_t mod_sentai_cam_stop(void) {
    return mp_obj_new_int(sentai_cam_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_stop_obj, mod_sentai_cam_stop);

// sentai.camera.jpeg(quality=75) -> bytes (JPEG data)
// Captures a frame at camera native resolution and returns JPEG
static mp_obj_t mod_sentai_cam_jpeg(size_t n_args, const mp_obj_t *args) {
    int quality = (n_args > 0) ? mp_obj_get_int(args[0]) : 75;
    int w = sentai_cam_get_width();
    int h = sentai_cam_get_height();
    // Max JPEG buffer - typically much smaller than RGB
    int max_jpeg = w * h;  // generous upper bound
    uint8_t* buf = (uint8_t*)malloc(max_jpeg);
    if (!buf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("jpeg buf alloc"));
    }
    int jpeg_size = sentai_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    if (jpeg_size <= 0) {
        free(buf);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }
    mp_obj_t result = mp_obj_new_bytes(buf, jpeg_size);
    free(buf);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_jpeg_obj, 0, 1, mod_sentai_cam_jpeg);

// sentai.camera.to_tensor([path[, quality]]) -> int
// Captures camera frame into TPU input tensor.  If path is given, also saves
// the scaled RGB frame as JPEG before int8 quantization.
static mp_obj_t mod_sentai_cam_to_tensor(size_t n_args, const mp_obj_t *args) {
    const char* path = NULL;
    int quality = 75;
    if (n_args >= 1 && args[0] != mp_const_none) {
        _fs_check_usb();
        path = mp_obj_str_get_str(args[0]);
    }
    if (n_args >= 2) {
        quality = mp_obj_get_int(args[1]);
    }
    return mp_obj_new_int(sentai_cam_to_tensor_ex(path, quality));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_to_tensor_obj, 0, 2, mod_sentai_cam_to_tensor);

// sentai.camera.save_jpeg(path, quality=75) -> int (bytes written)
// Capture + JPEG + save to LFS in one call
static mp_obj_t mod_sentai_cam_save_jpeg(size_t n_args, const mp_obj_t *args) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(args[0]);
    int quality = (n_args > 1) ? mp_obj_get_int(args[1]) : 75;
    int w = sentai_cam_get_width();
    int h = sentai_cam_get_height();
    int max_jpeg = w * h;
    uint8_t* buf = (uint8_t*)malloc(max_jpeg);
    if (!buf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("jpeg buf alloc"));
    }

    uint32_t t0 = sentai_ticks_ms();
    int jpeg_size = sentai_cam_capture_jpeg(buf, max_jpeg, w, h, quality);
    uint32_t t1 = sentai_ticks_ms();

    if (jpeg_size <= 0) {
        free(buf);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("cam capture fail"));
    }

    int ok = sentai_fs_write(path, buf, jpeg_size);
    uint32_t t2 = sentai_ticks_ms();

    free(buf);
    if (!ok) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("write fail"));
    }

    printf("[save_jpeg] %s: capture=%lums save=%lums total=%lums (%d bytes, q=%d)\r\n",
           path, (unsigned long)(t1 - t0), (unsigned long)(t2 - t1),
           (unsigned long)(t2 - t0), jpeg_size, quality);

    return mp_obj_new_int(jpeg_size);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_save_jpeg_obj, 1, 2, mod_sentai_cam_save_jpeg);

// sentai.camera.resolution() -> tuple (width, height) - current capture resolution
static mp_obj_t mod_sentai_cam_resolution(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_width());
    items[1] = mp_obj_new_int(sentai_cam_get_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_resolution_obj, mod_sentai_cam_resolution);

// sentai.camera.set_resolution(w, h) -> int (0=ok, -1=invalid)
// Set capture output resolution. Max = native sensor res.
static mp_obj_t mod_sentai_cam_set_resolution(mp_obj_t w_obj, mp_obj_t h_obj) {
    int w = mp_obj_get_int(w_obj);
    int h = mp_obj_get_int(h_obj);
    return mp_obj_new_int(sentai_cam_set_res(w, h));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_set_resolution_obj, mod_sentai_cam_set_resolution);

// sentai.camera.native_res() -> tuple (width, height) - sensor native resolution
static mp_obj_t mod_sentai_cam_native_res(void) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(sentai_cam_get_native_width());
    items[1] = mp_obj_new_int(sentai_cam_get_native_height());
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_native_res_obj, mod_sentai_cam_native_res);

// sentai.camera.select(id[, path]) -> int (0=ok).
// id: 0=front, 1=back, -1=virtual static image from FS.
static mp_obj_t mod_sentai_cam_select(size_t n_args, const mp_obj_t *args) {
    int id = mp_obj_get_int(args[0]);
    if (id == -1) {
        if (n_args < 2) return mp_obj_new_int(-10);
        const char* path = mp_obj_str_get_str(args[1]);
        return mp_obj_new_int(sentai_virtual_camera_select(path));
    }
    sentai_virtual_camera_disable();
    return mp_obj_new_int(sentai_cam_switch(id));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_select_obj,
                                            1, 2, mod_sentai_cam_select);

// sentai.camera.play(dir, fps=10, count=0) -> number of BMP frames loaded.
// Starts VirtualCameraTask.  Files are read from dir in lexicographic order;
// convention for deterministic missions: frame_000.bmp, frame_001.bmp, ...
// count=0 loops forever, count>0 publishes exactly count frames and leaves
// the last frame active.
static mp_obj_t mod_sentai_cam_play(size_t n_args, const mp_obj_t *args) {
    const char* dir = mp_obj_str_get_str(args[0]);
    int fps = (n_args > 1) ? mp_obj_get_int(args[1]) : 10;
    int count = (n_args > 2) ? mp_obj_get_int(args[2]) : 0;
    return mp_obj_new_int(sentai_virtual_camera_play(dir, fps, count));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_play_obj,
                                            1, 3, mod_sentai_cam_play);

// sentai.camera.replay(fps=10, count=0) -> 1 if replay task started.
// Re-publishes the already-selected virtual frame from RAM.  This avoids
// filesystem reads during deterministic scheduler/prep diagnostics.
static mp_obj_t mod_sentai_cam_replay(size_t n_args, const mp_obj_t *args) {
    int fps = (n_args > 0) ? mp_obj_get_int(args[0]) : 10;
    int count = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    return mp_obj_new_int(sentai_virtual_camera_replay(fps, count));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_replay_obj,
                                            0, 2, mod_sentai_cam_replay);

// sentai.camera.play_stop() -> int
static mp_obj_t mod_sentai_cam_play_stop(void) {
    return mp_obj_new_int(sentai_virtual_camera_play_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_play_stop_obj,
                                  mod_sentai_cam_play_stop);

// sentai.camera.playing() -> bool
static mp_obj_t mod_sentai_cam_playing(void) {
    return mp_obj_new_bool(sentai_virtual_camera_playing() != 0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_playing_obj,
                                  mod_sentai_cam_playing);

// sentai.camera.current_id() -> 0 (front) or 1 (back).
// MUX state for the NEXT capture (post-flip).
extern int sentai_cam_current_id(void);
static mp_obj_t mod_sentai_cam_current_id(void) {
    return mp_obj_new_int(sentai_cam_current_id());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_current_id_obj, mod_sentai_cam_current_id);

// sentai.camera.last_capture_id() -> 0/1.
// Source camera that wrote the MOST RECENTLY COMPLETED buffer (pre-flip
// snapshot).  Use this to label captures correctly under alt-mode.
extern int sentai_cam_last_capture_id(void);
static mp_obj_t mod_sentai_cam_last_capture_id(void) {
    return mp_obj_new_int(sentai_cam_last_capture_id());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_last_capture_id_obj, mod_sentai_cam_last_capture_id);

// sentai.camera.stats() -> dict of camera-switch fault counters.
//
// These were historically exposed as sentai.diag.cam_stats().  They belong
// with the camera owner: persistent monotonic breadcrumbs for switch/grab
// degraded paths since boot.
extern void sentai_cam_stats_get(uint32_t* ok_eof, uint32_t* fallback,
                                 uint32_t* drain_timeout,
                                 uint32_t* grab_retry, uint32_t* grab_fatal);
static mp_obj_t mod_sentai_cam_stats(void) {
    uint32_t ok_eof = 0, fallback = 0, drain_timeout = 0,
             grab_retry = 0, grab_fatal = 0;
    sentai_cam_stats_get(&ok_eof, &fallback, &drain_timeout,
                         &grab_retry, &grab_fatal);
    mp_obj_t d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_switch_ok_eof),
                      mp_obj_new_int_from_uint(ok_eof));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_switch_fallback),
                      mp_obj_new_int_from_uint(fallback));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_drain_timeout),
                      mp_obj_new_int_from_uint(drain_timeout));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_retry),
                      mp_obj_new_int_from_uint(grab_retry));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_fatal),
                      mp_obj_new_int_from_uint(grab_fatal));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_stats_obj,
                                  mod_sentai_cam_stats);

// Build #980: sentai.camera.set_fps() removed — runtime fps switch
// is blocked by the second-init wedge in CAMERA_RECEIVER_Init.
// Use sentai.camera.init(streaming, fps) at first init instead.
// To switch fps after that, sentai.sys.reset() then init(s, new_fps).

// sentai.camera.fps() -> int (current sensor framerate).
// Reads g_runtime_fps which BOARD_InitCamera honoured at the most
// recent init.  Returns the boot default (30) if init has not yet
// run.  Cheap one-line getter; useful for the REPL idiom:
//   if sentai.camera.fps() != target: sentai.sys.reset()
extern volatile uint32_t g_runtime_fps;
static mp_obj_t mod_sentai_cam_fps(void) {
    return mp_obj_new_int_from_uint(g_runtime_fps);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_fps_obj, mod_sentai_cam_fps);

// sentai.camera.peek5_b40() -> (cam_tag, b0, b1, b2, b3, b4)
// Build #935 — single dequeue, 5 B-channel samples at col 40 from
// rows {0, H/4, H/2, 3H/4, H-1} of the SAME buffer, plus the
// per-buffer cam_tag.  Detects mid-frame-mix scrambling without the
// "5 different buffers" false positive that
// peek_row_at-in-a-loop has.
extern int sentai_cam_peek5_b40(uint8_t* dst5);
static mp_obj_t mod_sentai_cam_peek5_b40(void) {
    uint8_t b[5];
    int tag = sentai_cam_peek5_b40(b);
    if (tag < -1) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("peek5_b40 failed"));
    }
    mp_obj_t items[6] = {
        mp_obj_new_int(tag),
        mp_obj_new_int(b[0]),
        mp_obj_new_int(b[1]),
        mp_obj_new_int(b[2]),
        mp_obj_new_int(b[3]),
        mp_obj_new_int(b[4]),
    };
    return mp_obj_new_tuple(6, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_peek5_b40_obj,
                                  mod_sentai_cam_peek5_b40);

// sentai.camera.peek_row(len=64) -> bytes
// Grab the latest frame and return the first `len` bytes of its
// XRGB8888 framebuffer.  Used to inspect OV5640 embedded data
// lines (when 0x501F bit 7 is set) — the per-frame metadata
// (frame counter, AGC, exposure, etc.) lands at byte 0.
extern int sentai_cam_peek_first_row(uint8_t* dst, int len);
static mp_obj_t mod_sentai_cam_peek_row(size_t n_args, const mp_obj_t *args) {
    int len = (n_args >= 1) ? mp_obj_get_int(args[0]) : 64;
    if (len < 1)    len = 1;
    if (len > 4096) len = 4096;
    uint8_t buf[4096];
    int got = sentai_cam_peek_first_row(buf, len);
    if (got <= 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("peek_row failed"));
    }
    return mp_obj_new_bytes(buf, got);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_peek_row_obj,
                                            0, 1, mod_sentai_cam_peek_row);

// sentai.camera.buf_id_dump() -> tuple of 18 ints
//   (isr0..isr3, isr_writes, isr_idx_miss, isr_skip_both,
//    task0..task3, task_writes, task_unknown,
//    csi0..csi3, csi_hook_fires)
//
// CSI = build #893+ authoritative path: NXP fsl_csi.c calls
// coralmicro_csi_on_frame_complete(addr) at the EXACT moment a
// real frame is queued for the consumer.  csi_hook_fires increments
// only on accepted frames, so it should match sensor frame count.
extern volatile uint8_t  g_cam_buf_id[];
extern volatile uint8_t  g_cam_buf_id_task[];
extern volatile uint8_t  g_cam_buf_id_csi[];
extern volatile uint32_t g_cam_buf_tag_writes;
extern volatile uint32_t g_cam_buf_tag_idx_miss;
extern volatile uint32_t g_cam_buf_tag_skip_both;
extern volatile uint32_t g_cam_buf_task_writes;
extern volatile uint32_t g_cam_buf_task_unknown;
extern volatile uint32_t g_cam_csi_hook_fires;
static mp_obj_t mod_sentai_cam_buf_id_dump(void) {
    mp_obj_t items[18] = {
        mp_obj_new_int(g_cam_buf_id[0]),
        mp_obj_new_int(g_cam_buf_id[1]),
        mp_obj_new_int(g_cam_buf_id[2]),
        mp_obj_new_int(g_cam_buf_id[3]),
        mp_obj_new_int_from_uint(g_cam_buf_tag_writes),
        mp_obj_new_int_from_uint(g_cam_buf_tag_idx_miss),
        mp_obj_new_int_from_uint(g_cam_buf_tag_skip_both),
        mp_obj_new_int(g_cam_buf_id_task[0]),
        mp_obj_new_int(g_cam_buf_id_task[1]),
        mp_obj_new_int(g_cam_buf_id_task[2]),
        mp_obj_new_int(g_cam_buf_id_task[3]),
        mp_obj_new_int_from_uint(g_cam_buf_task_writes),
        mp_obj_new_int_from_uint(g_cam_buf_task_unknown),
        mp_obj_new_int(g_cam_buf_id_csi[0]),
        mp_obj_new_int(g_cam_buf_id_csi[1]),
        mp_obj_new_int(g_cam_buf_id_csi[2]),
        mp_obj_new_int(g_cam_buf_id_csi[3]),
        mp_obj_new_int_from_uint(g_cam_csi_hook_fires),
    };
    return mp_obj_new_tuple(18, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_buf_id_dump_obj, mod_sentai_cam_buf_id_dump);

// sentai.camera.dirty_skip_n(n) -> int (returns previous value)
// Build #952 — set the count of consecutive FB-done buffers to mark
// dirty after each MUX flip.  N=1 = only the in-flight buffer.
// Sweep N=1..6 to find the sweet spot for a given ratio.
extern volatile uint32_t g_cam_dirty_consecutive_n;
static mp_obj_t mod_sentai_cam_dirty_skip_n(mp_obj_t n_o) {
    int n = mp_obj_get_int(n_o);
    if (n < 0) n = 0;
    if (n > 8) n = 8;
    uint32_t prev = g_cam_dirty_consecutive_n;
    g_cam_dirty_consecutive_n = (uint32_t)n;
    return mp_obj_new_int_from_uint(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_dirty_skip_n_obj,
                                  mod_sentai_cam_dirty_skip_n);

// sentai.camera.flip_stats() -> tuple
//   (vblank_flips, deferred, deferred_streak, forced,
//    isr_count, isr_us_last, isr_us_max, isr_us_avg,
//    h_lt50, h_50_99, h_100_199, h_200_499, h_500_999, h_ge1000)
//
// Build #924 — measure CSI ISR latency + VBLANK gate behaviour.
// Used by diag/_t_pattern_31.py post-test to decide whether the
// 10 % residual mistag rate is bound by ISR latency (gate would
// help) or by tag-write race (gate is irrelevant — see
// agent.md §9.2).
/* Build #958: flip_stats() removed to free m_text for the new
 * sentai_cam_set_fps path.  The diagnostic counters
 * (g_cam_vblank_flips, g_cam_buf_dirty_marks, etc.) remain in DTCM
 * and can be exposed again later if needed. */

// sentai.camera.grabbed_id() -> 0/1 / -1 if not yet set.
// Per-buffer tagged source of the buffer returned by the most recent
// grab call (cam_grab_latest).  Tag was written by CSI ISR at the
// buffer's FB-done event, so it reflects which camera actually wrote
// the bytes — independent of any subsequent MUX flips.
extern int sentai_cam_grabbed_id(void);
static mp_obj_t mod_sentai_cam_grabbed_id(void) {
    return mp_obj_new_int(sentai_cam_grabbed_id());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_grabbed_id_obj, mod_sentai_cam_grabbed_id);

// sentai.camera.frame_count() -> int
// Monotonic sensor-frame counter (gated on CSI DMA_DONE flag, not raw
// ISR entries — see camera_support.c:CSI_IRQHandler for the filter).
// One tick = one actual sensor frame captured.
static mp_obj_t mod_sentai_cam_frame_count(void) {
    return mp_obj_new_int(sentai_cam_get_frame_seq());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_frame_count_obj, mod_sentai_cam_frame_count);

// sentai.camera.rotate(cam_id, degrees) -> int (0=ok)
// cam_id: 0=front, 1=back.  degrees: 0, 90, 180, 270.
static mp_obj_t mod_sentai_cam_rotate(mp_obj_t cam_obj, mp_obj_t deg_obj) {
    int cam_id = mp_obj_get_int(cam_obj);
    int degrees = mp_obj_get_int(deg_obj);
    return mp_obj_new_int(sentai_cam_rotate(cam_id, degrees));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_rotate_obj, mod_sentai_cam_rotate);

// sentai.camera.aec_set(cam_id, high, low) -> int
//
// Tune the OV5640 auto-exposure target luminance range:
//   high: stable-range target high (0x10..0xF0; ≈target/255)
//   low:  stable-range target low  (0x10..high)
// Fast-zone thresholds scaled around the same target.
//
// Reference points (write with either preset for A/B testing):
//   NXP default  : high=0x30 low=0x28  → ≈18% target (dim indoors)
//   OmniVision AN: high=0x78 low=0x68  → ≈45% target (general)
//   Bright room  : high=0x60 low=0x50  → ≈35% target (mid)
extern int sentai_cam_aec_set(int cam_id, int high, int low);
static mp_obj_t mod_sentai_cam_aec_set(mp_obj_t cam_o, mp_obj_t hi_o, mp_obj_t lo_o) {
    return mp_obj_new_int(sentai_cam_aec_set(
        mp_obj_get_int(cam_o), mp_obj_get_int(hi_o), mp_obj_get_int(lo_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_sentai_cam_aec_set_obj, mod_sentai_cam_aec_set);

// sentai.camera.gain_ceiling_set(cam_id, ceiling_u10)
// 10-bit cap for AGC (0x010..0x3FF, units of 1/16× gain).
//   0x07C = NXP default ≈ 7.75× (too low for dim indoor)
//   0x0F8 = 15.5×
//   0x1F0 = 31× (recommended for low-light; noisier)
//   0x3FF = max (62.9×)
extern int sentai_cam_gain_ceiling_set(int cam_id, int ceiling);
static mp_obj_t mod_sentai_cam_gain_ceiling_set(mp_obj_t cam_o, mp_obj_t c_o) {
    return mp_obj_new_int(sentai_cam_gain_ceiling_set(
        mp_obj_get_int(cam_o), mp_obj_get_int(c_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_gain_ceiling_set_obj,
                                 mod_sentai_cam_gain_ceiling_set);

// sentai.camera.test_pattern(cam_id, mode) — synthetic-pattern mode
//   0 = OFF  (restore AEC/AGC auto, normal capture)
//   1 = BLACK (force minimum exposure + gain → near-zero pixels)
//   2 = WHITE (force max exposure + gain → near-saturated pixels)
// Used by sentai.pipeline.calibrate(synthetic=True) to verify
// cam_id ground-truth via pixel content (cam0=BLACK, cam1=WHITE).
extern int sentai_cam_test_pattern(int cam_id, int mode);
static mp_obj_t mod_sentai_cam_test_pattern(mp_obj_t cam_o, mp_obj_t mode_o) {
    return mp_obj_new_int(sentai_cam_test_pattern(
        mp_obj_get_int(cam_o), mp_obj_get_int(mode_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_test_pattern_obj,
                                  mod_sentai_cam_test_pattern);

// sentai.camera.isp_preset(cam_id, name) — apply a bundled set of
// OV5640 ISP registers tuned for a specific scene type.
// name: "nxp_stock" | "bright_indoor" | "daylight" | "low_light"
extern int sentai_cam_isp_preset(int cam_id, const char* name);
static mp_obj_t mod_sentai_cam_isp_preset(mp_obj_t cam_o, mp_obj_t name_o) {
    const char* name = mp_obj_str_get_str(name_o);
    return mp_obj_new_int(sentai_cam_isp_preset(mp_obj_get_int(cam_o), name));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_isp_preset_obj,
                                 mod_sentai_cam_isp_preset);

// sentai.camera.reg_read(cam_id, reg) -> int value (0..255) or -1 on fail
// sentai.camera.reg_write(cam_id, reg, val) -> 0 ok, neg on fail
// Raw SCCB access for debugging the ISP — verify that writes stick,
// read current exposure/gain values, etc.
extern int sentai_cam_reg_read(int cam_id, int reg);
extern int sentai_cam_reg_write(int cam_id, int reg, int val);
static mp_obj_t mod_sentai_cam_reg_read(mp_obj_t cam_o, mp_obj_t reg_o) {
    return mp_obj_new_int(sentai_cam_reg_read(mp_obj_get_int(cam_o),
                                              mp_obj_get_int(reg_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_cam_reg_read_obj,
                                 mod_sentai_cam_reg_read);
static mp_obj_t mod_sentai_cam_reg_write(mp_obj_t cam_o, mp_obj_t reg_o,
                                          mp_obj_t val_o) {
    return mp_obj_new_int(sentai_cam_reg_write(mp_obj_get_int(cam_o),
                                               mp_obj_get_int(reg_o),
                                               mp_obj_get_int(val_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_sentai_cam_reg_write_obj,
                                 mod_sentai_cam_reg_write);

// sentai.camera.ratio([a, b]) -> (a, b)
//
// Read/write the stateless auto-alternate schedule.  When both `a` and
// `b` are > 0, the CSI ISR flips the MUX so that cam0 owns `a` frames
// per (a+b)-frame cycle and cam1 owns the other `b` frames — all with
// the glitch-free EOF-VBLANK timing.  Either zero disables the
// scheduler; manual `sentai.camera.select()` continues to work and
// always wins.  Values are clamped to [0, 1000]; out-of-range raises
// ValueError.
extern int  sentai_cam_ratio_set(uint32_t a, uint32_t b);
extern void sentai_cam_ratio_get(uint32_t* a, uint32_t* b);
static mp_obj_t mod_sentai_cam_ratio(size_t n_args, const mp_obj_t *args) {
    uint32_t a = 0, b = 0;
    sentai_cam_ratio_get(&a, &b);
    mp_obj_t prev[2] = { mp_obj_new_int(a), mp_obj_new_int(b) };
    if (n_args == 2) {
        int na = mp_obj_get_int(args[0]);
        int nb = mp_obj_get_int(args[1]);
        if (na < 0 || nb < 0 ||
            sentai_cam_ratio_set((uint32_t)na, (uint32_t)nb) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("ratio requires 0..1000 each"));
        }
    } else if (n_args != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("ratio() takes 0 or 2 args"));
    }
    return mp_obj_new_tuple(2, prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_ratio_obj,
                                            0, 2, mod_sentai_cam_ratio);

// sentai.camera.switch_drain([n]) -> int (previous value)
//
// Read/write the post-switch drain threshold: number of fresh ISR frames
// required after a MUX flip before get_raw_with_recovery returns a frame.
// Default 2 (one possibly-mixed frame + one fully-new frame).  Accepts
// n in [1,10]; raises ValueError otherwise.  A/B runtime toggle used by
// experiment E17 to inspect post-switch image artifacts at threshold=1.
extern uint32_t sentai_cam_switch_drain_get(void);
extern int      sentai_cam_switch_drain_set(uint32_t n);
static mp_obj_t mod_sentai_cam_switch_drain(size_t n_args, const mp_obj_t *args) {
    int prev = (int) sentai_cam_switch_drain_get();
    if (n_args >= 1) {
        int v = mp_obj_get_int(args[0]);
        if (sentai_cam_switch_drain_set((uint32_t) v) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("switch_drain must be 1..10"));
        }
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_cam_switch_drain_obj,
                                            0, 1, mod_sentai_cam_switch_drain);

// ───────────────────────────────────────────────────────────────────
// sentai_camera_grab_gray_zerocopy — C-side hook used by sentai.aruco
// (OP-S6-W3) so the ArUco detector runs on the latest CSI frame
// WITHOUT copying the 76 KB image buffer across the MicroPython
// binding boundary.  Per CLAUDE.md compute-in-C principle.
//
// Phase 1b dual-path (2026-05-17):
//   1. FAST: read from sentai_prep SLOT_GRAY_NATIVE if the slot is
//      enabled by a consumer (refcount > 0) AND the pipeline is
//      running.  Returns the latest slot pointer + seq — ZERO PXP
//      work on this call path (PXP already ran in PrepTask).
//   2. FALLBACK: on-demand PXP via sentai_pxp_xrgb_to_y8 if no slot
//      is published yet (pipeline not started, or first frame).
//      Same logic as the original implementation.
//
// Why dual-path: the operator architecture expects continuous slot
// publishing once pipeline starts ([[no-heavy-data-through-mp]]).
// But consumer code (sentai.aruco mission scripts) shouldn't break
// when pipeline isn't running — that's why fallback exists.
// ───────────────────────────────────────────────────────────────────
extern int sentai_cam_grab_latest(uint8_t** raw);
extern int sentai_cam_get_width(void);
extern int sentai_cam_get_height(void);
extern int sentai_camera_backend_publish_prep_once(void);
extern int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                                  uint8_t* dst, int dst_w, int dst_h);
extern volatile int g_cam_grabbed_id;

#include "sentai_prep.h"

// sentai.camera.prep_enable(slot_id) -> new refcount.
// Debug/experiment hook: enables a shared prep slot without starting the TPU
// detection pipeline.  Slot ids are sentai_prep_slot_id_t values.
static mp_obj_t mod_sentai_cam_prep_enable(mp_obj_t slot_obj) {
    int slot = mp_obj_get_int(slot_obj);
    return mp_obj_new_int(sentai_prep_slot_enable((sentai_prep_slot_id_t)slot));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_prep_enable_obj,
                                  mod_sentai_cam_prep_enable);

// sentai.camera.prep_disable(slot_id) -> new refcount.
static mp_obj_t mod_sentai_cam_prep_disable(mp_obj_t slot_obj) {
    int slot = mp_obj_get_int(slot_obj);
    return mp_obj_new_int(sentai_prep_slot_disable((sentai_prep_slot_id_t)slot));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_prep_disable_obj,
                                  mod_sentai_cam_prep_disable);

// sentai.camera.prep_once() -> int.
// Publishes enabled prep slots from the latest camera/virtual-camera frame.
// No TPU model, no InferTask, no detection queue.
static mp_obj_t mod_sentai_cam_prep_once(void) {
    return mp_obj_new_int(sentai_camera_backend_publish_prep_once());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_prep_once_obj,
                                  mod_sentai_cam_prep_once);

// sentai.camera.prep_reset() -> None.
static mp_obj_t mod_sentai_cam_prep_reset(void) {
    sentai_prep_reset_stats();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_prep_reset_obj,
                                  mod_sentai_cam_prep_reset);

// sentai.camera.prep_stats() -> dict.
static mp_obj_t mod_sentai_cam_prep_stats(void) {
    sentai_prep_stats_t st;
    sentai_prep_get_stats(&st);
    mp_obj_t d = mp_obj_new_dict(6);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_total),
                      mp_obj_new_int_from_uint(st.frames_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_with_aux),
                      mp_obj_new_int_from_uint(st.frames_with_aux));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_producer_overruns),
                      mp_obj_new_int_from_uint(st.producer_overruns));
    mp_obj_t refs[SENTAI_PREP_SLOT_COUNT];
    mp_obj_t seqs[SENTAI_PREP_SLOT_COUNT];
    for (int i = 0; i < SENTAI_PREP_SLOT_COUNT; ++i) {
        refs[i] = mp_obj_new_int(st.slot_refcount[i]);
        seqs[i] = mp_obj_new_int_from_uint(st.slot_seq[i]);
    }
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_slot_refcount),
                      mp_obj_new_tuple(SENTAI_PREP_SLOT_COUNT, refs));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_slot_seq),
                      mp_obj_new_tuple(SENTAI_PREP_SLOT_COUNT, seqs));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cam_prep_stats_obj,
                                  mod_sentai_cam_prep_stats);

#define SENTAI_ARUCO_GRAY_W 320
#define SENTAI_ARUCO_GRAY_H 240

// Scratch only used by the FALLBACK on-demand path.  When the slot
// path is active, this buffer is dormant.
static uint8_t s_aruco_gray_buf[SENTAI_ARUCO_GRAY_W * SENTAI_ARUCO_GRAY_H]
    __attribute__((section(".sdram_bss"), aligned(64)));

int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                      int* out_w, int* out_h,
                                      uint32_t* out_seq,
                                      uint32_t* out_ts_ms) {
    if (!out_buf || !out_w || !out_h) return -1;

    // Fast path: slot already populated by PrepTask.
    {
        const uint8_t* slot_buf = NULL;
        int sw = 0, sh = 0;
        uint32_t sseq = 0;
        if (sentai_prep_slot_get(SENTAI_PREP_SLOT_GRAY_NATIVE,
                                  &slot_buf, &sw, &sh, &sseq) == 0) {
            *out_buf = slot_buf;
            *out_w   = sw;
            *out_h   = sh;
            *out_seq = sseq;
            if (out_ts_ms) *out_ts_ms = (uint32_t)xTaskGetTickCount();
            return 0;
        }
    }

    // Fallback: pipeline not running OR slot not enabled.  On-demand
    // PXP into local scratch.  Cold-path for diag scripts running
    // outside of any pipeline session.
    uint8_t* xrgb = NULL;
    int idx = sentai_cam_grab_latest(&xrgb);
    if (idx < 0 || !xrgb) return -1;
    const int W = sentai_cam_get_width();
    const int H = sentai_cam_get_height();
    if (W <= 0 || H <= 0 ||
        W > SENTAI_ARUCO_GRAY_W || H > SENTAI_ARUCO_GRAY_H) {
        return -1;
    }
    if (sentai_pxp_xrgb_to_y8(xrgb, W, H,
                                s_aruco_gray_buf, W, H) != 0) {
        return -1;
    }
    *out_buf = s_aruco_gray_buf;
    *out_w   = W;
    *out_h   = H;
    *out_seq = (uint32_t)g_cam_grabbed_id;
    if (out_ts_ms) *out_ts_ms = (uint32_t)xTaskGetTickCount();
    return 0;
}

// ---- module table ----
static const mp_rom_map_elem_t sentai_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_cam_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mod_sentai_cam_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_jpeg),       MP_ROM_PTR(&mod_sentai_cam_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_tensor),  MP_ROM_PTR(&mod_sentai_cam_to_tensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_jpeg),  MP_ROM_PTR(&mod_sentai_cam_save_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_resolution),     MP_ROM_PTR(&mod_sentai_cam_resolution_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_resolution), MP_ROM_PTR(&mod_sentai_cam_set_resolution_obj) },
    { MP_ROM_QSTR(MP_QSTR_native_res), MP_ROM_PTR(&mod_sentai_cam_native_res_obj) },
    { MP_ROM_QSTR(MP_QSTR_select),     MP_ROM_PTR(&mod_sentai_cam_select_obj) },
    { MP_ROM_QSTR(MP_QSTR_play),       MP_ROM_PTR(&mod_sentai_cam_play_obj) },
    { MP_ROM_QSTR(MP_QSTR_replay),     MP_ROM_PTR(&mod_sentai_cam_replay_obj) },
    { MP_ROM_QSTR(MP_QSTR_play_stop),  MP_ROM_PTR(&mod_sentai_cam_play_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_playing),    MP_ROM_PTR(&mod_sentai_cam_playing_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_enable),  MP_ROM_PTR(&mod_sentai_cam_prep_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_disable), MP_ROM_PTR(&mod_sentai_cam_prep_disable_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_once),    MP_ROM_PTR(&mod_sentai_cam_prep_once_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_reset),   MP_ROM_PTR(&mod_sentai_cam_prep_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_stats),   MP_ROM_PTR(&mod_sentai_cam_prep_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_current_id),      MP_ROM_PTR(&mod_sentai_cam_current_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_last_capture_id), MP_ROM_PTR(&mod_sentai_cam_last_capture_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),           MP_ROM_PTR(&mod_sentai_cam_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_grabbed_id),      MP_ROM_PTR(&mod_sentai_cam_grabbed_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_buf_id_dump),     MP_ROM_PTR(&mod_sentai_cam_buf_id_dump_obj) },
    { MP_ROM_QSTR(MP_QSTR_dirty_skip_n),    MP_ROM_PTR(&mod_sentai_cam_dirty_skip_n_obj) },
    { MP_ROM_QSTR(MP_QSTR_peek_row),        MP_ROM_PTR(&mod_sentai_cam_peek_row_obj) },
    { MP_ROM_QSTR(MP_QSTR_peek5_b40),       MP_ROM_PTR(&mod_sentai_cam_peek5_b40_obj) },
    { MP_ROM_QSTR(MP_QSTR_fps),             MP_ROM_PTR(&mod_sentai_cam_fps_obj) },
    { MP_ROM_QSTR(MP_QSTR_frame_count),  MP_ROM_PTR(&mod_sentai_cam_frame_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotate),     MP_ROM_PTR(&mod_sentai_cam_rotate_obj) },
    { MP_ROM_QSTR(MP_QSTR_aec_set),    MP_ROM_PTR(&mod_sentai_cam_aec_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_gain_ceiling_set), MP_ROM_PTR(&mod_sentai_cam_gain_ceiling_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_isp_preset), MP_ROM_PTR(&mod_sentai_cam_isp_preset_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_pattern), MP_ROM_PTR(&mod_sentai_cam_test_pattern_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg_read),   MP_ROM_PTR(&mod_sentai_cam_reg_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg_write),  MP_ROM_PTR(&mod_sentai_cam_reg_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_switch_drain), MP_ROM_PTR(&mod_sentai_cam_switch_drain_obj) },
    { MP_ROM_QSTR(MP_QSTR_ratio),      MP_ROM_PTR(&mod_sentai_cam_ratio_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_camera_globals, sentai_camera_globals_table);
static const mp_obj_module_t sentai_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_camera_globals,
};
