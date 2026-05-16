/* modsentai_sim_flow.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== sentai.flow — Phase 4 real binding (was Phase 1.5 stub) =====
 *
 * Backed by the snapshot updated by sim/camera_bridge_recv.c on every
 * Gazebo frame.  read() returns a 5-tuple matching the firmware contract:
 *     (seq, dx_q1000, dy_q1000, conf, latency_us)
 *
 * Units identical to ARM:
 *   dx, dy   — milli-grid-pixels (1000 = 1 grid-px = 8 raw-px after PXP)
 *   conf     — 0..255 (peak/mean ratio of the phase-corr surface, scaled)
 *   latency  — recv-to-publish wall time, microseconds
 *
 * Body-frame mapping (cam0 vflip=1) lives at the consumer (`_t_flow_to_drone.py`
 * `body_xform`); see Sim.md §10b.  This binding stays raw-image-frame.
 */
typedef struct {
    volatile uint32_t seq;
    volatile int32_t  dx_q1000;
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t  dz_q1000;        // added 2026-05-11
    volatile uint32_t dz_conf;
} _sim_flow_snapshot_t;
extern const _sim_flow_snapshot_t* sim_camera_flow_snapshot(void);

static mp_obj_t sentai_flow_read(void) {
    const _sim_flow_snapshot_t* s = sim_camera_flow_snapshot();
    /* Snapshot stably: read seq, then payload, then re-read seq.  If the
     * second seq differs we lost the race with the writer — return the
     * later seq's data on a quick retry.  Bounded one retry. */
    uint32_t seq0 = s->seq;
    int32_t  dx   = s->dx_q1000;
    int32_t  dy   = s->dy_q1000;
    uint32_t cf   = s->conf;
    uint64_t lat  = s->latency_us;
    int32_t  dz   = s->dz_q1000;
    uint32_t dzc  = s->dz_conf;
    uint32_t seq1 = s->seq;
    if (seq1 != seq0) {
        dx  = s->dx_q1000;
        dy  = s->dy_q1000;
        cf  = s->conf;
        lat = s->latency_us;
        dz  = s->dz_q1000;
        dzc = s->dz_conf;
        seq0 = seq1;
    }
    /* Tuple: (seq, dx_q1000, dy_q1000, conf, latency_us, dz_q1000, dz_conf)
     * dz_q1000 is µ/frame (parts-per-million altitude rate); diagnostic only,
     * cf2 EKF does NOT consume it. */
    mp_obj_t items[7] = {
        mp_obj_new_int_from_uint(seq0),
        mp_obj_new_int(dx),
        mp_obj_new_int(dy),
        mp_obj_new_int_from_uint(cf),
        mp_obj_new_int_from_ull(lat),
        mp_obj_new_int(dz),
        mp_obj_new_int_from_uint(dzc),
    };
    return mp_obj_new_tuple(7, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_flow_read_obj, sentai_flow_read);

static const mp_rom_map_elem_t sentai_flow_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_flow) },
    { MP_ROM_QSTR(MP_QSTR_read),     MP_ROM_PTR(&sentai_flow_read_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_flow_globals, sentai_flow_globals_table);
static const mp_obj_module_t sentai_flow_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_flow_globals,
};
