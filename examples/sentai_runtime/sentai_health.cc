/*
 * SentAI Subsystem Health Monitoring - Implementation
 */

#include "sentai_health.h"
#include "sentai_error.h"

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_soc_src.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"

#include <stdio.h>
#include <string.h>

// ===================== Static Data =====================

// Health records for all subsystems
static HealthRecord_t s_health[SUBSYS_COUNT];

// Mutex for thread-safe access
static SemaphoreHandle_t s_health_mutex = nullptr;

// System mode cache
static volatile SystemMode_t s_system_mode = SYS_MODE_BOOTING;

// Names (shortened to save flash)
static const char s_states[] = "HLTY\0DEGD\0FLTD\0RCVR\0N/A";  // 4-char state ids
static const char* const s_subsys_names[] = {
    "lk", "ms", "cz", "dt", "ht", "rp", "au", "sl", "cl"  // 2-char subsystem ids
};

// ===================== Internal Functions =====================

static void update_subsys_state(SubsystemId_t subsys) {
    HealthRecord_t* rec = &s_health[subsys];
    
    // State machine transitions based on consecutive failures
    switch (rec->state) {
        case HEALTH_HEALTHY:
            if (rec->consecutive_fails >= HEALTH_DEGRADE_THRESHOLD) {
                rec->state = HEALTH_DEGRADED;
                rec->degraded_entries++;
            }
            break;
            
        case HEALTH_DEGRADED:
            if (rec->consecutive_fails >= HEALTH_FAULT_THRESHOLD) {
                rec->state = HEALTH_FAULTED;
                rec->fault_entries++;
            } else if (rec->consecutive_fails == 0) {
                // Track successful operations needed to recover
                // Check if last_success_ms indicates recent success
                rec->state = HEALTH_HEALTHY;
            }
            break;
            
        case HEALTH_FAULTED:
            // Requires explicit recovery trigger
            break;
            
        case HEALTH_RECOVERING:
            if (rec->consecutive_fails == 0) {
                rec->state = HEALTH_HEALTHY;
            } else if (rec->consecutive_fails >= HEALTH_DEGRADE_THRESHOLD) {
                rec->state = HEALTH_FAULTED;
                rec->fault_entries++;
            }
            break;
            
        case HEALTH_UNAVAILABLE:
            // Requires explicit enable
            break;
    }
}

static void update_system_mode(void) {
    int faulted_count = 0;
    int degraded_count = 0;
    int healthy_count = 0;
    
    for (int i = 0; i < SUBSYS_COUNT; i++) {
        switch (s_health[i].state) {
            case HEALTH_HEALTHY:
                healthy_count++;
                break;
            case HEALTH_DEGRADED:
            case HEALTH_RECOVERING:
                degraded_count++;
                break;
            case HEALTH_FAULTED:
                faulted_count++;
                break;
            case HEALTH_UNAVAILABLE:
                // Don't count - not enabled
                break;
        }
    }
    
    // Critical subsystems for safe mode decision
    // If REPL and HTTP both faulted → SAFE_MODE
    int critical_faulted = 0;
    if (s_health[SUBSYS_REPL].state == HEALTH_FAULTED) critical_faulted++;
    if (s_health[SUBSYS_HTTP].state == HEALTH_FAULTED) critical_faulted++;
    
    if (critical_faulted >= 2) {
        s_system_mode = SYS_MODE_SAFE;
    } else if (faulted_count > 0 || degraded_count >= 2) {
        s_system_mode = SYS_MODE_DEGRADED;
    } else {
        s_system_mode = SYS_MODE_NORMAL;
    }
}

// ===================== Public API =====================

void sentai_health_init(void) {
    // Create mutex
    s_health_mutex = xSemaphoreCreateMutex();
    
    // Initialize all subsystems as unavailable
    for (int i = 0; i < SUBSYS_COUNT; i++) {
        memset(&s_health[i], 0, sizeof(HealthRecord_t));
        s_health[i].state = HEALTH_UNAVAILABLE;
    }
    
    s_system_mode = SYS_MODE_BOOTING;
}

void sentai_health_success(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return;
    if (!s_health_mutex) return;  // Not initialized yet

    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Health update dropped — mutex held > 10 ms (priority inversion or long CS).
        SERR_LOG(SERR_SYS_ASSERT, (uint32_t)subsys);
        return;
    }
    HealthRecord_t* rec = &s_health[subsys];

    // If was unavailable, transition to healthy
    if (rec->state == HEALTH_UNAVAILABLE) {
        rec->state = HEALTH_HEALTHY;
    }

    rec->success_count++;
    rec->consecutive_fails = 0;
    rec->last_success_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    update_subsys_state(subsys);
    update_system_mode();

    xSemaphoreGive(s_health_mutex);
}

void sentai_health_fail(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return;
    if (!s_health_mutex) return;  // Not initialized yet

    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        SERR_LOG(SERR_SYS_ASSERT, (uint32_t)subsys);
        return;
    }
    HealthRecord_t* rec = &s_health[subsys];

    // If was unavailable, transition to degraded (first fail means something is running)
    if (rec->state == HEALTH_UNAVAILABLE) {
        rec->state = HEALTH_DEGRADED;
    }

    rec->fail_count++;
    rec->consecutive_fails++;

    update_subsys_state(subsys);
    update_system_mode();

    xSemaphoreGive(s_health_mutex);
}

void sentai_health_timeout(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return;
    if (!s_health_mutex) return;  // Not initialized yet

    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        SERR_LOG(SERR_SYS_ASSERT, (uint32_t)subsys);
        return;
    }
    HealthRecord_t* rec = &s_health[subsys];

    // If was unavailable, transition to degraded
    if (rec->state == HEALTH_UNAVAILABLE) {
        rec->state = HEALTH_DEGRADED;
    }

    rec->timeout_count++;
    rec->consecutive_fails++;

    update_subsys_state(subsys);
    update_system_mode();

    xSemaphoreGive(s_health_mutex);
}

const HealthRecord_t* sentai_health_get(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return nullptr;
    return &s_health[subsys];
}

SystemMode_t sentai_health_system_mode(void) {
    return s_system_mode;
}

const char* sentai_health_state_name(HealthState_t state) {
    if (state > HEALTH_UNAVAILABLE) return "?";
    return &s_states[(int)state * 5];  // 4 chars + null each
}

const char* sentai_health_subsys_name(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return "unknown";
    return s_subsys_names[(int)subsys];
}

void sentai_health_set_unavailable(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return;
    if (!s_health_mutex) return;  // Not initialized yet
    
    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_health[subsys].state = HEALTH_UNAVAILABLE;
        update_system_mode();
        xSemaphoreGive(s_health_mutex);
    }
}

void sentai_health_set_recovering(SubsystemId_t subsys) {
    if (subsys >= SUBSYS_COUNT) return;
    if (!s_health_mutex) return;  // Not initialized yet
    
    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_health[subsys].state = HEALTH_RECOVERING;
        s_health[subsys].consecutive_fails = 0;
        update_system_mode();
        xSemaphoreGive(s_health_mutex);
    }
}

int sentai_health_summary(char* buf, int buf_size) {
    if (!buf || buf_size < 1) return 0;
    
    int pos = 0;
    int remaining = buf_size - 1; // Leave room for null terminator
    
    // System mode (compact)
    static const char mode_ids[] = "BNDS?";  // Boot/Normal/Degrad/Safe/?
    int n = snprintf(buf + pos, remaining, "M:%c\n", mode_ids[s_system_mode < 5 ? s_system_mode : 4]);
    if (n > 0 && n < remaining) { pos += n; remaining -= n; }
    
    // Per-subsystem summary (compact: name:state:ok:fail:to)
    for (int i = 0; i < SUBSYS_COUNT && remaining > 0; i++) {
        const HealthRecord_t* rec = &s_health[i];
        n = snprintf(buf + pos, remaining, "%s:%s:%lu:%lu:%lu\n",
                     s_subsys_names[i],
                     &s_states[(int)rec->state * 5],
                     (unsigned long)rec->success_count,
                     (unsigned long)rec->fail_count,
                     (unsigned long)rec->timeout_count);
        if (n > 0 && n < remaining) { pos += n; remaining -= n; }
    }
    
    buf[pos] = '\0';
    return pos;
}

void sentai_health_boot_complete(void) {
    if (!s_health_mutex) return;
    
    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Phase 8.4: Clear SRC_GPR boot-attempt counter.
        // Marks that this boot reached a safe checkpoint (REPL started or recovery active).
        // Next cold-wake starts with boot_attempts=0 so crash loops can be re-detected.
        SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister1, 0);

        // Mark REPL as healthy since it just started
        if (s_health[SUBSYS_REPL].state == HEALTH_UNAVAILABLE) {
            s_health[SUBSYS_REPL].state = HEALTH_HEALTHY;
            s_health[SUBSYS_REPL].success_count = 1;
            s_health[SUBSYS_REPL].last_success_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        
        // Transition from BOOTING to appropriate mode
        if (s_system_mode == SYS_MODE_BOOTING) {
            update_system_mode();
        }
        
        xSemaphoreGive(s_health_mutex);
    }
}

int sentai_health_is_safe_mode(void) {
    return s_system_mode == SYS_MODE_SAFE;
}

void sentai_health_set_recovery_mode(void) {
    if (!s_health_mutex) return;
    if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_system_mode = SYS_MODE_RECOVERY;
        xSemaphoreGive(s_health_mutex);
    }
}
