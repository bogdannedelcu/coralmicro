// ============== sentai.rtos — FreeRTOS system ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

#include "sentai_dmesg.h"

extern void sentai_repl_activity(void);

// sentai.rtos.sleep_ms(ms)
// Chunked + drains the MicroPython scheduler queue every 10 ms so
// async callbacks (e.g. sentai.crazy.on_message dispatch trampoline)
// fire promptly even during long sleeps. Mirrors the MICROPY_EVENT_POLL_HOOK
// pattern that mainline ports use inside time.sleep / select.poll.
static mp_obj_t mod_sentai_sleep_ms(mp_obj_t ms_obj) {
    int total = mp_obj_get_int(ms_obj);
    if (total <= 0) return mp_const_none;
    while (total > 0) {
        int slice = (total > 10) ? 10 : total;
        sentai_sleep_ms(slice);
        mp_handle_pending(true);
        total -= slice;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_sleep_ms_obj, mod_sentai_sleep_ms);

// sentai.rtos.ticks_ms()
static mp_obj_t mod_sentai_ticks_ms(void) {
    return mp_obj_new_int(sentai_ticks_ms());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_ticks_ms_obj, mod_sentai_ticks_ms);

// ===================== FreeRTOS task listing =====================

// sentai.rtos.tasks() -> list of (name, state, priority, stack_hwm) tuples
// Lists all FreeRTOS tasks currently in the system.
// state: "running", "ready", "blocked", "suspended", "deleted"
// stack_hwm: minimum free stack (words) since task creation (high water mark)
static mp_obj_t mod_sentai_tasks(void) {
    #define MAX_TASKS 32
    TaskStatus_t task_buf[MAX_TASKS];
    uint32_t total_runtime;
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, &total_runtime);

    // Warn if the system has more tasks than our buffer can hold
    if (n == MAX_TASKS) {
        // Buffer may be full (actual count unknown — FreeRTOS returns at most MAX_TASKS)
        printf("[sentai.rtos] WARNING: task list truncated at %d entries\r\n", MAX_TASKS);
    }

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));

    for (UBaseType_t i = 0; i < n; i++) {
        mp_obj_t items[4];
        items[0] = mp_obj_new_str(task_buf[i].pcTaskName,
                                  strlen(task_buf[i].pcTaskName));
        const char* state_str;
        switch (task_buf[i].eCurrentState) {
            case eRunning:   state_str = "running"; break;
            case eReady:     state_str = "ready"; break;
            case eBlocked:   state_str = "blocked"; break;
            case eSuspended: state_str = "suspended"; break;
            case eDeleted:   state_str = "deleted"; break;
            default:         state_str = "?"; break;
        }
        items[1] = mp_obj_new_str(state_str, strlen(state_str));
        items[2] = mp_obj_new_int(task_buf[i].uxCurrentPriority);
        items[3] = mp_obj_new_int(task_buf[i].usStackHighWaterMark);
        mp_obj_list_append(MP_OBJ_FROM_PTR(result), mp_obj_new_tuple(4, items));
    }
    #undef MAX_TASKS
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_tasks_obj, mod_sentai_tasks);

// ===================== System info: heap =====================

// sentai.rtos.heap_info() -> dict with FreeRTOS heap stats + MicroPython GC stats
static mp_obj_t mod_sentai_heap_info(void) {
    // MicroPython GC info
    gc_info_t gc;
    gc_info(&gc);

    mp_obj_dict_t *d = MP_OBJ_TO_PTR(mp_obj_new_dict(7));

    // FreeRTOS/newlib heap (xPortGetFreeHeapSize wraps mallinfo + sbrk remainder)
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("rtos_free", 9),
        mp_obj_new_int(xPortGetFreeHeapSize()));

    // MicroPython GC heap
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_total", 8),
        mp_obj_new_int(gc.total));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_used", 7),
        mp_obj_new_int(gc.used));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_free", 7),
        mp_obj_new_int(gc.free));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_max_free", 11),
        mp_obj_new_int(gc.max_free));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_max_block", 12),
        mp_obj_new_int(gc.max_block));
    mp_obj_dict_store(MP_OBJ_FROM_PTR(d),
        mp_obj_new_str("gc_num_1block", 13),
        mp_obj_new_int(gc.num_1block));

    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_heap_info_obj, mod_sentai_heap_info);

// ===================== System info: CPU usage =====================

// sentai.rtos.cpu_usage() -> list of (name, cpu_percent) tuples, sorted by CPU% descending
// Uses FreeRTOS runtime stats (configGENERATE_RUN_TIME_STATS=1)
static mp_obj_t mod_sentai_cpu_usage(void) {
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    uint32_t total_runtime;
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, &total_runtime);

    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));

    if (total_runtime == 0) total_runtime = 1;  // avoid div by zero

    for (UBaseType_t i = 0; i < n; i++) {
        mp_obj_t items[2];
        items[0] = mp_obj_new_str(task_buf[i].pcTaskName,
                                  strlen(task_buf[i].pcTaskName));
        uint32_t pct = (task_buf[i].ulRunTimeCounter * 100) / total_runtime;
        items[1] = mp_obj_new_int(pct);
        mp_obj_list_append(MP_OBJ_FROM_PTR(result), mp_obj_new_tuple(2, items));
    }
    #undef MAX_TASKS
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_cpu_usage_obj, mod_sentai_cpu_usage);

// ===================== Task suspend/resume =====================

// Protected task names that should never be suspended
static const char* protected_tasks[] = {
    "mp_repl", "IDLE", "Tmr Svc", "ctrlc", "console", "usb_dev", NULL
};

static int is_protected_task(const char* name) {
    for (int i = 0; protected_tasks[i] != NULL; i++) {
        if (strstr(name, protected_tasks[i]) != NULL) return 1;
    }
    return 0;
}

// sentai.rtos.suspend(name) -> 1 if suspended, 0 if not found or protected
static mp_obj_t mod_sentai_task_suspend(mp_obj_t name_obj) {
    const char* name = mp_obj_str_get_str(name_obj);
    
    if (is_protected_task(name)) {
        mp_printf(&mp_plat_print, "Cannot suspend protected task: %s\r\n", name);
        return mp_obj_new_int(0);
    }
    
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, NULL);
    
    for (UBaseType_t i = 0; i < n; i++) {
        if (strcmp(task_buf[i].pcTaskName, name) == 0) {
            vTaskSuspend(task_buf[i].xHandle);
            mp_printf(&mp_plat_print, "Suspended: %s\r\n", name);
            return mp_obj_new_int(1);
        }
    }
    #undef MAX_TASKS
    mp_printf(&mp_plat_print, "Task not found: %s\r\n", name);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_task_suspend_obj, mod_sentai_task_suspend);

// sentai.rtos.resume(name) -> 1 if resumed, 0 if not found
static mp_obj_t mod_sentai_task_resume(mp_obj_t name_obj) {
    const char* name = mp_obj_str_get_str(name_obj);
    
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, NULL);
    
    for (UBaseType_t i = 0; i < n; i++) {
        if (strcmp(task_buf[i].pcTaskName, name) == 0) {
            vTaskResume(task_buf[i].xHandle);
            mp_printf(&mp_plat_print, "Resumed: %s\r\n", name);
            return mp_obj_new_int(1);
        }
    }
    #undef MAX_TASKS
    mp_printf(&mp_plat_print, "Task not found: %s\r\n", name);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_task_resume_obj, mod_sentai_task_resume);

// sentai.rtos.suspend_all() -> count of suspended tasks
// Suspends all tasks except REPL, IDLE, and other protected tasks
static mp_obj_t mod_sentai_task_suspend_all(void) {
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, NULL);
    int count = 0;
    
    for (UBaseType_t i = 0; i < n; i++) {
        if (!is_protected_task(task_buf[i].pcTaskName)) {
            vTaskSuspend(task_buf[i].xHandle);
            mp_printf(&mp_plat_print, "Suspended: %s\r\n", task_buf[i].pcTaskName);
            count++;
        }
    }
    #undef MAX_TASKS
    mp_printf(&mp_plat_print, "Total suspended: %d\r\n", count);
    return mp_obj_new_int(count);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_task_suspend_all_obj, mod_sentai_task_suspend_all);

// sentai.rtos.resume_all() -> count of resumed tasks
static mp_obj_t mod_sentai_task_resume_all(void) {
    #define MAX_TASKS 24
    TaskStatus_t task_buf[MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, NULL);
    int count = 0;
    
    for (UBaseType_t i = 0; i < n; i++) {
        if (task_buf[i].eCurrentState == eSuspended) {
            vTaskResume(task_buf[i].xHandle);
            mp_printf(&mp_plat_print, "Resumed: %s\r\n", task_buf[i].pcTaskName);
            count++;
        }
    }
    #undef MAX_TASKS
    mp_printf(&mp_plat_print, "Total resumed: %d\r\n", count);
    return mp_obj_new_int(count);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_task_resume_all_obj, mod_sentai_task_resume_all);

// ===================== System info: uptime =====================

// sentai.rtos.uptime() -> int (seconds since boot)
static mp_obj_t mod_sentai_uptime(void) {
    return mp_obj_new_int(xTaskGetTickCount() / configTICK_RATE_HZ);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uptime_obj, mod_sentai_uptime);

// ===================== Runtime log / REPL supervision =====================

// sentai.rtos.dmesg([max_bytes]) -> str
// Runtime-owned in-RAM ring log.  Kept here instead of sentai.diag so REPL
// supervision and task diagnostics stay with the RTOS namespace.
static mp_obj_t mod_sentai_rtos_dmesg(size_t n_args, const mp_obj_t *args) {
    int max_bytes = (n_args >= 1) ? mp_obj_get_int(args[0]) : 16384;
    if (max_bytes < 64)    max_bytes = 64;
    if (max_bytes > 65536) max_bytes = 65536;

    char* buf = m_new(char, max_bytes);
    size_t n = sentai_dmesg_read(buf, (size_t)max_bytes);
    mp_obj_t result = mp_obj_new_str(buf, n);
    m_del(char, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_rtos_dmesg_obj,
                                            0, 1, mod_sentai_rtos_dmesg);

// sentai.rtos.dmesg_clear() -> None
static mp_obj_t mod_sentai_rtos_dmesg_clear(void) {
    sentai_dmesg_clear();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_rtos_dmesg_clear_obj,
                                  mod_sentai_rtos_dmesg_clear);

// sentai.rtos.dmesg_stats() -> (bytes_used, bytes_dropped)
static mp_obj_t mod_sentai_rtos_dmesg_stats(void) {
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint((uint32_t)sentai_dmesg_used()),
        mp_obj_new_int_from_uint(sentai_dmesg_dropped()),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_rtos_dmesg_stats_obj,
                                  mod_sentai_rtos_dmesg_stats);

// sentai.rtos.repl_kick() -> None
static mp_obj_t mod_sentai_rtos_repl_kick(void) {
    sentai_repl_activity();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_rtos_repl_kick_obj,
                                  mod_sentai_rtos_repl_kick);

// ===================== Crash/Hang stats =====================
// Get network/HTTP health stats for debugging hangs

extern uint32_t sentai_get_http_requests(void);
extern uint32_t sentai_get_http_hangs(void);
extern int sentai_get_network_healthy(void);

// sentai.rtos.http_stats() -> (requests, hangs, healthy)
static mp_obj_t mod_sentai_http_stats(void) {
    mp_obj_t items[3] = {
        mp_obj_new_int(sentai_get_http_requests()),
        mp_obj_new_int(sentai_get_http_hangs()),
        sentai_get_network_healthy() ? mp_const_true : mp_const_false,
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_http_stats_obj, mod_sentai_http_stats);

// ---- module table ----
static const mp_rom_map_elem_t sentai_rtos_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_rtos) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms),     MP_ROM_PTR(&mod_sentai_sleep_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_ms),     MP_ROM_PTR(&mod_sentai_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_tasks),        MP_ROM_PTR(&mod_sentai_tasks_obj) },
    { MP_ROM_QSTR(MP_QSTR_heap_info),    MP_ROM_PTR(&mod_sentai_heap_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_cpu_usage),    MP_ROM_PTR(&mod_sentai_cpu_usage_obj) },
    { MP_ROM_QSTR(MP_QSTR_uptime),       MP_ROM_PTR(&mod_sentai_uptime_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg),        MP_ROM_PTR(&mod_sentai_rtos_dmesg_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg_clear),  MP_ROM_PTR(&mod_sentai_rtos_dmesg_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg_stats),  MP_ROM_PTR(&mod_sentai_rtos_dmesg_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_repl_kick),    MP_ROM_PTR(&mod_sentai_rtos_repl_kick_obj) },
    { MP_ROM_QSTR(MP_QSTR_suspend),      MP_ROM_PTR(&mod_sentai_task_suspend_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),       MP_ROM_PTR(&mod_sentai_task_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_suspend_all),  MP_ROM_PTR(&mod_sentai_task_suspend_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume_all),   MP_ROM_PTR(&mod_sentai_task_resume_all_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_rtos_globals, sentai_rtos_globals_table);
static const mp_obj_module_t sentai_rtos_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_rtos_globals,
};
