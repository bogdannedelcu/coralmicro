/*
 * SentAI Subsystem Health Monitoring
 * 
 * Provides AUTOSAR-style supervision with:
 * - Health states per subsystem
 * - Success/fail/timeout counters
 * - Central health aggregation
 * - Degraded mode support
 */

#ifndef SENTAI_HEALTH_H
#define SENTAI_HEALTH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== Health States =====================
// Per-subsystem health states (AUTOSAR-inspired)
typedef enum {
    HEALTH_HEALTHY      = 0,   // Normal operation
    HEALTH_DEGRADED     = 1,   // Partial failures, still functional
    HEALTH_FAULTED      = 2,   // Not functioning, needs recovery
    HEALTH_RECOVERING   = 3,   // Recovery in progress
    HEALTH_UNAVAILABLE  = 4,   // Not initialized or disabled
} HealthState_t;

// ===================== Subsystem IDs =====================
typedef enum {
    SUBSYS_LINK     = 0,   // MAVLink bridge
    SUBSYS_MESH     = 1,   // Meshtastic bridge
    SUBSYS_CRAZY    = 2,   // Crazyflie bridge
    SUBSYS_DETECT   = 3,   // Detection pipeline
    SUBSYS_HTTP     = 4,   // HTTP server
    SUBSYS_REPL     = 5,   // MicroPython REPL
    SUBSYS_AUDIO    = 6,   // Audio service
    SUBSYS_SLAM     = 7,   // SlamTask perception loop (OP-S10-W11-T3)
    SUBSYS_CALIB    = 8,   // sentai.calib bringup state (OP-S10-W21-T6).
                           // HEALTHY iff is_calibrated()==1; UNAVAILABLE
                           // pre-bringup.  Missions consult this (or the
                           // is_calibrated() shortcut) before takeoff.
    SUBSYS_COUNT            // Must be last
} SubsystemId_t;

// ===================== Health Record =====================
// Stats for each subsystem
typedef struct {
    HealthState_t state;          // Current health state
    uint32_t success_count;       // Successful operations
    uint32_t fail_count;          // Failed operations
    uint32_t timeout_count;       // Timeout events
    uint32_t last_success_ms;     // Last successful operation (ms since boot)
    uint32_t consecutive_fails;   // Consecutive failures (reset on success)
    uint32_t degraded_entries;    // Times entered degraded state
    uint32_t fault_entries;       // Times entered faulted state
} HealthRecord_t;

// ===================== System Health =====================
// Overall system health (aggregated)
typedef enum {
    SYS_MODE_BOOTING    = 0,   // Init in progress
    SYS_MODE_NORMAL     = 1,   // All critical subsystems healthy
    SYS_MODE_DEGRADED   = 2,   // Some subsystems degraded
    SYS_MODE_SAFE       = 3,   // Critical failure, minimal operation
    SYS_MODE_RECOVERY   = 4,   // Recovery/reset pending
} SystemMode_t;

// ===================== Thresholds =====================
// Configurable thresholds for state transitions
#define HEALTH_DEGRADE_THRESHOLD    3    // Consecutive fails to enter DEGRADED
#define HEALTH_FAULT_THRESHOLD      10   // Consecutive fails to enter FAULTED
#define HEALTH_RECOVER_SUCCESS      5    // Consecutive success to leave DEGRADED
#define HEALTH_TIMEOUT_WARN_MS      5000 // Warn if no success for this long

// ===================== API Functions =====================

// Initialize health monitoring (call once at startup)
void sentai_health_init(void);

// Report a successful operation
void sentai_health_success(SubsystemId_t subsys);

// Report a failed operation
void sentai_health_fail(SubsystemId_t subsys);

// Report a timeout
void sentai_health_timeout(SubsystemId_t subsys);

// Get health record for a subsystem
const HealthRecord_t* sentai_health_get(SubsystemId_t subsys);

// Get overall system mode
SystemMode_t sentai_health_system_mode(void);

// Get health state name as string (for Python/display)
const char* sentai_health_state_name(HealthState_t state);

// Get subsystem name as string
const char* sentai_health_subsys_name(SubsystemId_t subsys);

// Check if subsystem is operational (HEALTHY or DEGRADED)
static inline int sentai_health_is_operational(SubsystemId_t subsys) {
    const HealthRecord_t* rec = sentai_health_get(subsys);
    return rec && (rec->state == HEALTH_HEALTHY || rec->state == HEALTH_DEGRADED);
}

// Mark subsystem as unavailable (e.g., not started)
void sentai_health_set_unavailable(SubsystemId_t subsys);

// Mark subsystem as recovering
void sentai_health_set_recovering(SubsystemId_t subsys);

// Get system summary string (for diagnostics)
// Returns bytes written, or needed if buf is NULL
int sentai_health_summary(char* buf, int buf_size);

// Mark boot complete — transitions from BOOTING to NORMAL (if healthy)
// Also clears the SRC_GPR boot-attempt counter so the next reboot starts fresh.
void sentai_health_boot_complete(void);

// Force system mode to RECOVERY (anti-brick: 3+ consecutive boot crashes detected).
// Call AFTER sentai_health_boot_complete() in enter_recovery_mode().
void sentai_health_set_recovery_mode(void);

// Check if system is in safe mode (only essential services should run)
int sentai_health_is_safe_mode(void);

#ifdef __cplusplus
}
#endif

#endif // SENTAI_HEALTH_H
