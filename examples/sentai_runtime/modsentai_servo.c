// ============== sentai.servo — Drone action layer (Stage 4) ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Backend-agnostic action primitives the mission state-machine (sentai.explore)
// calls into.  The same MicroPython API works under cf2 (CRTP via sentai.crazy)
// and PX4 (MAVLink via sentai.link).  This skeleton stage adds the namespace,
// validation, and an internal action trace ring so we can verify in tests that
// the FSM emits the right sequence — wiring each action to the chosen backend
// is left to the next stage.
//
// Why an action layer over the existing crazy/link namespaces?  The two
// transports differ in command vocabulary (CRTP packet vs MAV_CMD_*), units
// (cf2 cm vs PX4 m), and frame (cf2 body NED vs PX4 ENU).  Code in
// sentai.explore must not be branching on the backend at every state edge;
// servo.* is the single dispatch point.

#include <stdint.h>
#include <string.h>

#define SERVO_TRACE_CAP   16    /* ring of recent actions for test introspection */

typedef enum {
    SERVO_BACKEND_NONE = 0,
    SERVO_BACKEND_SIM  = 1,
    SERVO_BACKEND_CF2  = 2,
    SERVO_BACKEND_PX4  = 3,
} servo_backend_t;

typedef enum {
    SERVO_ACT_NONE     = 0,
    SERVO_ACT_ARM      = 1,
    SERVO_ACT_DISARM   = 2,
    SERVO_ACT_TAKEOFF  = 3,
    SERVO_ACT_LAND     = 4,
    SERVO_ACT_HOVER    = 5,
    SERVO_ACT_MOVE     = 6,
    SERVO_ACT_YAW      = 7,
} servo_act_t;

typedef struct {
    uint8_t  type;       /* servo_act_t */
    uint8_t  ok;         /* 0 = pending/fail, 1 = success at time of issue */
    uint16_t seq;        /* monotonic sequence number */
    float    a, b, c, d; /* per-action args (alt, dx/dy/dz, yaw, …) */
} servo_trace_t;

static servo_backend_t g_servo_backend     = SERVO_BACKEND_NONE;
static int             g_servo_armed       = 0;
static uint16_t        g_servo_seq         = 0;
static servo_trace_t   g_servo_trace[SERVO_TRACE_CAP];
static int             g_servo_trace_head  = 0;   /* next write slot */
static int             g_servo_trace_count = 0;

static const char *servo_act_name(int t) {
    switch (t) {
        case SERVO_ACT_ARM:     return "ARM";
        case SERVO_ACT_DISARM:  return "DISARM";
        case SERVO_ACT_TAKEOFF: return "TAKEOFF";
        case SERVO_ACT_LAND:    return "LAND";
        case SERVO_ACT_HOVER:   return "HOVER";
        case SERVO_ACT_MOVE:    return "MOVE";
        case SERVO_ACT_YAW:     return "YAW";
        default:                return "NONE";
    }
}

static const char *servo_backend_name(int b) {
    switch (b) {
        case SERVO_BACKEND_SIM: return "sim";
        case SERVO_BACKEND_CF2: return "cf2";
        case SERVO_BACKEND_PX4: return "px4";
        default:                return "none";
    }
}

/* Push a trace record; oldest entries overwritten when full. */
static void servo_trace_push(uint8_t type, int ok,
                             float a, float b, float c, float d) {
    servo_trace_t *e = &g_servo_trace[g_servo_trace_head];
    e->type = type;
    e->ok   = (uint8_t)(ok ? 1 : 0);
    e->seq  = ++g_servo_seq;
    e->a = a; e->b = b; e->c = c; e->d = d;
    g_servo_trace_head = (g_servo_trace_head + 1) % SERVO_TRACE_CAP;
    if (g_servo_trace_count < SERVO_TRACE_CAP) g_servo_trace_count++;
}

// =================== MicroPython bindings ===================

// servo.init(backend) -> int   0=ok, -1=invalid backend, -2=already initialized
//   backend ∈ {"sim", "cf2", "px4"}
static mp_obj_t mod_servo_init(mp_obj_t backend_obj) {
    const char *name = mp_obj_str_get_str(backend_obj);
    servo_backend_t b = SERVO_BACKEND_NONE;
    if      (!strcmp(name, "sim")) b = SERVO_BACKEND_SIM;
    else if (!strcmp(name, "cf2")) b = SERVO_BACKEND_CF2;
    else if (!strcmp(name, "px4")) b = SERVO_BACKEND_PX4;
    else return mp_obj_new_int(-1);

    g_servo_backend = b;
    g_servo_armed = 0;
    g_servo_trace_head = 0;
    g_servo_trace_count = 0;
    g_servo_seq = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_servo_init_obj, mod_servo_init);

// servo.arm() -> int  0=ok, -1=no backend, -2=already armed
static mp_obj_t mod_servo_arm(void) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (g_servo_armed) return mp_obj_new_int(-2);
    g_servo_armed = 1;
    servo_trace_push(SERVO_ACT_ARM, 1, 0, 0, 0, 0);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_arm_obj, mod_servo_arm);

// servo.disarm() -> int  0=ok, -1=no backend, -2=not armed
static mp_obj_t mod_servo_disarm(void) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (!g_servo_armed) return mp_obj_new_int(-2);
    g_servo_armed = 0;
    servo_trace_push(SERVO_ACT_DISARM, 1, 0, 0, 0, 0);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_disarm_obj, mod_servo_disarm);

// servo.takeoff(alt_m) -> int  0=ok, -1=no backend, -2=not armed, -3=alt OOR
static mp_obj_t mod_servo_takeoff(mp_obj_t alt_obj) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (!g_servo_armed) return mp_obj_new_int(-2);
    float alt_m = mp_obj_get_float(alt_obj);
    if (alt_m <= 0.0f || alt_m > 30.0f) return mp_obj_new_int(-3);
    servo_trace_push(SERVO_ACT_TAKEOFF, 1, alt_m, 0, 0, 0);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_servo_takeoff_obj, mod_servo_takeoff);

// servo.land() -> int  0=ok, -1=no backend, -2=not armed
static mp_obj_t mod_servo_land(void) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (!g_servo_armed) return mp_obj_new_int(-2);
    servo_trace_push(SERVO_ACT_LAND, 1, 0, 0, 0, 0);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_land_obj, mod_servo_land);

// servo.hover() -> int  0=ok, -1=no backend, -2=not armed
static mp_obj_t mod_servo_hover(void) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (!g_servo_armed) return mp_obj_new_int(-2);
    servo_trace_push(SERVO_ACT_HOVER, 1, 0, 0, 0, 0);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_hover_obj, mod_servo_hover);

// servo.move(dx_m, dy_m, dz_m [, dyaw_rad=0]) -> int
//   0=ok, -1=no backend, -2=not armed, -3=delta exceeds safety bound
static mp_obj_t mod_servo_move(size_t n_args, const mp_obj_t *args) {
    if (g_servo_backend == SERVO_BACKEND_NONE) return mp_obj_new_int(-1);
    if (!g_servo_armed) return mp_obj_new_int(-2);
    float dx = mp_obj_get_float(args[0]);
    float dy = mp_obj_get_float(args[1]);
    float dz = mp_obj_get_float(args[2]);
    float dyaw = (n_args >= 4) ? mp_obj_get_float(args[3]) : 0.0f;
    /* per-axis safety bound — 5 m per step, ¼ turn yaw. */
    if (dx < -5.0f || dx > 5.0f)   return mp_obj_new_int(-3);
    if (dy < -5.0f || dy > 5.0f)   return mp_obj_new_int(-3);
    if (dz < -5.0f || dz > 5.0f)   return mp_obj_new_int(-3);
    if (dyaw < -1.5708f || dyaw > 1.5708f) return mp_obj_new_int(-3);
    servo_trace_push(SERVO_ACT_MOVE, 1, dx, dy, dz, dyaw);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_servo_move_obj, 3, 4, mod_servo_move);

// servo.status() -> dict: backend / armed / trace_count / last_action
static mp_obj_t mod_servo_status(void) {
    mp_obj_t d = mp_obj_new_dict(5);
    const char *bname = servo_backend_name(g_servo_backend);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_backend),
                      mp_obj_new_str(bname, strlen(bname)));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_armed),
                      mp_obj_new_int(g_servo_armed));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_trace_count),
                      mp_obj_new_int(g_servo_trace_count));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seq),
                      mp_obj_new_int(g_servo_seq));
    /* "last_action" = most recently pushed entry (or "" if none) */
    const char *last = "";
    if (g_servo_trace_count > 0) {
        int prev = (g_servo_trace_head + SERVO_TRACE_CAP - 1) % SERVO_TRACE_CAP;
        last = servo_act_name(g_servo_trace[prev].type);
    }
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_action),
                      mp_obj_new_str(last, strlen(last)));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_status_obj, mod_servo_status);

// servo.trace() -> list of (name, ok, a, b, c, d, seq) tuples, oldest-first
static mp_obj_t mod_servo_trace_get(void) {
    mp_obj_t out = mp_obj_new_list(0, NULL);
    int n = g_servo_trace_count;
    int start = (n < SERVO_TRACE_CAP)
              ? 0
              : g_servo_trace_head;   /* ring is full → start at head */
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % SERVO_TRACE_CAP;
        servo_trace_t *e = &g_servo_trace[idx];
        const char *nm = servo_act_name(e->type);
        mp_obj_t tup[7] = {
            mp_obj_new_str(nm, strlen(nm)),
            mp_obj_new_int(e->ok),
            mp_obj_new_float(e->a),
            mp_obj_new_float(e->b),
            mp_obj_new_float(e->c),
            mp_obj_new_float(e->d),
            mp_obj_new_int(e->seq),
        };
        mp_obj_list_append(out, mp_obj_new_tuple(7, tup));
    }
    return out;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_trace_get_obj, mod_servo_trace_get);

// servo.clear_trace() -> None
static mp_obj_t mod_servo_clear_trace(void) {
    g_servo_trace_head = 0;
    g_servo_trace_count = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_servo_clear_trace_obj, mod_servo_clear_trace);

// =================== Module table ===================

static const mp_rom_map_elem_t sentai_servo_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_servo) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mod_servo_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),         MP_ROM_PTR(&mod_servo_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),      MP_ROM_PTR(&mod_servo_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),     MP_ROM_PTR(&mod_servo_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),        MP_ROM_PTR(&mod_servo_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),       MP_ROM_PTR(&mod_servo_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_move),        MP_ROM_PTR(&mod_servo_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),      MP_ROM_PTR(&mod_servo_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_trace),       MP_ROM_PTR(&mod_servo_trace_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_trace), MP_ROM_PTR(&mod_servo_clear_trace_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_servo_globals, sentai_servo_globals_table);
static const mp_obj_module_t sentai_servo_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_servo_globals,
};
