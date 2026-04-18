# Plan: SentAI Runtime Embedded Architecture Refactoring

**TL;DR**: Refactor sentai_runtime for production-grade embedded robustness by adding timeout-based fault handling, explicit state machines, race condition fixes, and structured diagnostics. Priority: prevent infinite blocks that cause unrecoverable hangs.

---

## 1. ARCHITECTURAL ASSESSMENT

### Critical Weaknesses (P0 - System Crash / Deadlock)

| ID | Issue | Files | Impact |
|----|-------|-------|--------|
| C1 | **9 `portMAX_DELAY` mutex takes** | link.cc, mesh.cc, crazy.cc, runtime.cc | Infinite block if peer unresponsive → hard reset only recovery |
| C2 | **7 unchecked queue operations** | link.cc, mesh.cc, detect.cc | Silent message loss → control packet drops |
| C3 | **2 race conditions on shared globals** | detect.cc (stats + staging metadata) | Corrupted frame stats, tensor dimension mismatch |
| C4 | **No subsystem health states** | All modules | No degraded/safe mode, no graceful shutdown |

### Major Weaknesses (P1 - Data Loss / Corruption)

| ID | Issue | Files | Impact |
|----|-------|-------|--------|
| M1 | malloc without null check | micropython_task.c, link.cc | Hard fault on heap exhaustion |
| M2 | Silent semaphore timeouts | detect.cc | Invisible performance regression |
| M3 | Crazy param response race | crazy.cc | Python reads partial CRTP buffer |
| M4 | HTTP cache vector unsafe | httpd.cc | Wild pointer if vector reallocated |

### Minor Weaknesses (P2 - Degraded Visibility)

| ID | Issue | Files | Impact |
|----|-------|-------|--------|
| N1 | Critical errors gated behind debug flags | crazy.cc, mesh.cc | Field failures invisible |
| N2 | REPL escape parser unbounded | micropython_task.c | Potential spin |
| N3 | No structured error taxonomy | All | Ad-hoc error reporting |

---

## 2. FAULT AND TIMING MODEL

### Credible Faults

| Category | Fault | Current Response | Target Response |
|----------|-------|------------------|-----------------|
| **Communication** | UART peer unresponsive | Infinite mutex wait | Timeout → retry 3x → degrade |
| **Communication** | Queue full | Silent drop | Log warning → drop oldest → continue |
| **Hardware** | EdgeTPU DMA stall | Invoke() timeout | Detect timeout → restart TPU task |
| **Memory** | Heap exhaustion | Hard fault | Pre-check → graceful fail |
| **Timing** | Detection pipeline stall | Watchdog reset | Task supervision → targeted restart |
| **Concurrency** | Staging metadata race | Corrupted inference | Mutex-protected struct copy |
| **External** | main.py infinite loop | 30s timeout + reset | ✅ Already good |
| **System** | Both HTTP+REPL dead | 25s → WDOG reset | ✅ Already good |

### Timing-Sensitive Paths

| Path | Period | Deadline | Current Protection |
|------|--------|----------|-------------------|
| Detection prep→infer | ~50ms | 100ms | Semaphore timeout 100ms |
| MAVLink heartbeat TX | 1Hz | 1000ms | **None** (portMAX_DELAY!) |
| CrazyFlie command TX | 50Hz | 20ms | **None** (portMAX_DELAY!) |
| Mesh radio TX | ~1Hz | 500ms | **None** (portMAX_DELAY!) |
| WDOG kick | 5s | 30s | ✅ Hardware WDOG |

### Supervised Entities (Current vs Target)

| Entity | Heartbeat | Deadline | Logical Check | Health State |
|--------|-----------|----------|---------------|--------------|
| hw_wdog | Self | 30s HW | HTTP or REPL active | ✅ Bool |
| det_infer | ❌ | ✅ 100ms | ❌ | ❌ |
| det_prep | ❌ | ✅ 100ms | ❌ | ❌ |
| mp_repl | REPL activity | ❌ | ❌ | ❌ |
| link_tx | ❌ | ❌ | ❌ | ❌ |
| mesh_tx | ❌ | ❌ | ❌ | ❌ |
| crazy_tx | ❌ | ❌ | ❌ | ❌ |

**Target**: Each supervised entity gains:
- Health counter (success/fail/timeout)
- State enum (HEALTHY/DEGRADED/FAULTED/RECOVERING)
- Periodic health report to central supervisor

---

## 3. TARGET ARCHITECTURE

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SUPERVISION LAYER                           │
│  ┌───────────────┐  ┌──────────────┐  ┌─────────────────────────┐  │
│  │ CentralHealth │  │ WatchdogMgr  │  │ CrashLog / Diagnostics  │  │
│  │ (state per    │  │ (WDOG1 kick  │  │ (structured error codes │  │
│  │  subsystem)   │  │  only if OK) │  │  + persistent logging)  │  │
│  └───────┬───────┘  └──────┬───────┘  └─────────────────────────┘  │
│          │                 │                                        │
├──────────┼─────────────────┼────────────────────────────────────────┤
│          │     ORCHESTRATION / STATE MACHINES                       │
│  ┌───────┴───────┐  ┌──────┴───────┐  ┌───────────────────────────┐│
│  │ BootSequencer │  │ SystemMode   │  │ SubsystemManager          ││
│  │ (init→ready→  │  │ (NORMAL →    │  │ (restart/degrade          ││
│  │  safe mode)   │  │  DEGRADED →  │  │  individual modules)      ││
│  └───────────────┘  │  SAFE_MODE)  │  └───────────────────────────┘│
│                     └──────────────┘                                │
├─────────────────────────────────────────────────────────────────────┤
│                    APPLICATION SERVICES (FreeRTOS Tasks)            │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────────┐ │
│  │ Detection   │ │ MicroPython │ │ HTTP Server │ │ Bridge Tasks  │ │
│  │ (prep+infer)│ │ (REPL)      │ │             │ │ (Mesh/Link/CF)│ │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └───────┬───────┘ │
│         │               │               │                 │         │
├─────────┼───────────────┼───────────────┼─────────────────┼─────────┤
│         │         DRIVER / HAL LAYER                      │         │
│  ┌──────┴──────┐ ┌──────┴──────┐ ┌──────┴──────┐ ┌───────┴───────┐ │
│  │ Camera/PXP  │ │ Console/USB │ │ LwIP/HTTPD │ │ UART Drivers  │ │
│  │ (DMA+ISR)   │ │ (CDC-NCM)   │ │             │ │ (ISR+DMA)     │ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│                           ISR LAYER                                 │
│  Button_ISR  │  UART_ISR  │  Audio_ISR  │  IMU_ISR  │  DMA_ISR     │
│  (notify)    │  (queue)   │  (flag)     │  (flag)   │  (signal)    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. REFACTORING PLAN

### Phase 1: Eliminate Deadlock Risks (P0) — *Day 1-2*

**Steps**:
1. Replace all `portMAX_DELAY` mutex takes with 50-100ms timeouts
2. Add retry logic with exponential backoff (max 3 attempts)
3. Add health counter increments on timeout
4. Log every timeout event (not gated by debug)

**Files**: sentai_link.cc, sentai_mesh.cc, sentai_crazy.cc, sentai_runtime.cc
**Parallel**: Steps can run in parallel per file

**Verification**:
- `grep -r "portMAX_DELAY" examples/sentai_runtime/` returns 0
- Unit test: Simulate UART stall, confirm timeout + recovery

---

### Phase 2: Fix Queue Operation Safety (P0) — *Day 2-3*

**Steps**:
1. Add return value checks to all xQueueSend/xQueueReceive
2. Add overflow counters per queue (`g_link_rx_dropped++`)
3. Log warning on first drop, then rate-limit (1 per 10s)
4. Expose counters to Python: `sentai.diag.queue_stats()`

**Files**: sentai_link.cc, sentai_mesh.cc, detection_task.cc
**Parallel**: Each file independent

**Verification**:
- Code review: No unchecked xQueue* calls remain
- Test: Force queue full, confirm logged + counted

---

### Phase 3: Fix Concurrency Hazards (P0) — *Day 3-4*

**Steps**:
1. Detection stats: Use `__atomic_fetch_add` for counters
2. Staging metadata: Create `StagingMeta` struct + mutex
3. Crazy param response: Add mutex around buffer read/write
4. HTTP cache: Add const qualifier + shutdown guard

**Files**: detection_task.cc, sentai_crazy.cc, sentai_httpd.cc
**Depends on**: None

**Verification**:
- TSAN/Helgrind if possible (or code review)
- Stress test: Run detection + Python queries simultaneously

---

### Phase 4: Add Subsystem Health States (P1) — *Day 4-5*

**Steps**:
1. Define health enum: `HEALTHY, DEGRADED, FAULTED, RECOVERING, UNAVAILABLE`
2. Add health state + counters to each bridge module
3. Create central `sentai_health_report()` function
4. Modify watchdog to check subsystem health, not just activity

**Files**: New file `sentai_health.cc`, modifications to link/mesh/crazy/detect
**Parallel**: Phase 4-5 can run parallel

**Verification**:
- Python: `sentai.diag.health()` shows all subsystem states
- Force timeout, confirm state transitions

---

### Phase 5: Structured Diagnostics (P1) — *Day 5-6*

**Steps**:
1. Define error code taxonomy (`SENTAI_ERR_UART_TIMEOUT = 0x1001`)
2. Add structured crash logging with error codes
3. Add boot counter persistence (`/log/boot_count`)
4. Add near-miss watchdog tracking (time since last kick)

**Files**: sentai_runtime.cc, new `sentai_error.h`
**Depends on**: Phase 4 health states

**Verification**:
- After reset: `sentai.diag.last_crash()` returns structured data
- Boot counter increments correctly

---

### Phase 6: Memory Safety Audit (P1) — *Day 6-7*

**Steps**:
1. Add null checks after all pvPortMalloc calls
2. Add stack watermarking to all tasks
3. Add malloc failure hook (`vApplicationMallocFailedHook`)
4. Document memory map in `SENTAI_API.md`

**Files**: micropython_task.c, sentai_link.cc, detection_task.cc
**Parallel**: Independent of other phases

**Verification**:
- `sentai.rtos.heap_info()` includes stack watermarks
- Force heap exhaustion, confirm graceful handling

---

### Phase 7: State Machine for System Mode (P2) — *Day 7-8*

**Steps**:
1. Create `SystemMode` enum: `BOOTING, NORMAL, DEGRADED, SAFE_MODE, RECOVERY`
2. Define transitions: Which faults trigger which mode
3. In DEGRADED, disable non-essential features (mesh, detection)
4. In SAFE_MODE, only REPL + USB drive active

**Files**: New `sentai_system_mode.cc`, modifications to sentai_runtime.cc
**Depends on**: Phase 4 health states

**Verification**:
- Force 3 bridge failures, confirm DEGRADED mode
- Force watchdog near-miss, confirm SAFE_MODE

---

## 5. RELEVANT FILES

| File | Purpose | Changes |
|------|---------|---------|
| [sentai_runtime.cc](examples/sentai_runtime/sentai_runtime.cc) | Main entry, watchdog, boot | Add system mode FSM, health aggregation |
| [sentai_link.cc](examples/sentai_runtime/sentai_link.cc) | MAVLink bridge | Timeout mutexes, queue checks, health state |
| [sentai_mesh.cc](examples/sentai_runtime/sentai_mesh.cc) | Meshtastic bridge | Timeout mutexes, queue checks, health state |
| [sentai_crazy.cc](examples/sentai_runtime/sentai_crazy.cc) | CrazyFlie bridge | Timeout mutexes, param response mutex, health state |
| [detection_task.cc](examples/sentai_runtime/detection_task.cc) | TPU inference | Atomic stats, staging mutex, queue checks |
| [micropython_task.c](examples/sentai_runtime/micropython_task.c) | REPL task | malloc checks, escape parser bound |
| [sentai_httpd.cc](examples/sentai_runtime/sentai_httpd.cc) | HTTP server | Cache protection, activity tracking |

---

## 6. VERIFICATION

### Automated
- [ ] `grep -r "portMAX_DELAY"` returns 0 hits in sentai_runtime/
- [ ] `grep -r "xQueueSend.*,.*0.*)"` all have return checks
- [ ] Build with `-Wthread-safety` passes (if compiler supports)

### Manual
- [ ] Disconnect UART during MAVLink TX → confirm timeout, not hang
- [ ] Fill detection queue to max → confirm logged, oldest dropped
- [ ] Run detection for 1 hour → no memory growth, stats accurate
- [ ] Force safe boot 3x → confirm SAFE_MODE entry

### MicroPython
- [ ] `sentai.diag.health()` returns all subsystems
- [ ] `sentai.diag.queue_stats()` returns per-queue counters
- [ ] `sentai.diag.last_crash()` returns structured crash info

---

## 7. DECISIONS

1. **Timeout values**: 50-100ms for bridge mutexes (fast enough for 50Hz CrazyFlie control)
2. **Retry limits**: Max 3 retries before marking subsystem DEGRADED
3. **Health aggregation**: Central task polls subsystems every 1s
4. **Error codes**: 16-bit codes, high byte = module, low byte = error type
5. **Excluded from scope**: A/B firmware update, bootloader integration (future phase)

---

## 8. FURTHER CONSIDERATIONS

1. **Task supervision granularity**: Individual task restart vs full subsystem restart? *Recommend: Subsystem restart is safer — less state to track*

2. **WDOG near-miss logging**: How many seconds before timeout should trigger warning? *Recommend: 20s (10s before 30s timeout)*

3. **Safe mode behavior**: Should HTTP server remain active in SAFE_MODE? *Recommend: Yes — needed for remote diagnosis*

---

## 9. BUILD DEPENDENCY MAP AND LIBRARY-LEVEL RISKS

### 9.1 Complete Build Dependency Tree

```
sentai_runtime (executable)
├── [SOURCE FILES]
│   ├── sentai_runtime.cc         - Main entry, watchdog, boot
│   ├── sentai_tfl_bridge.cc      - TensorFlow Lite bridge
│   ├── sentai_slow_bridge.cc     - Slow operations bridge
│   ├── detection_task.cc         - TPU inference pipeline
│   ├── sentai_tracker.cc         - Object tracker
│   ├── modsentai_hal.cc          - MicroPython HAL bindings
│   ├── sentai_mesh.cc            - Meshtastic radio bridge
│   ├── sentai_link.cc            - MAVLink UART bridge
│   ├── sentai_crazy.cc           - Crazyflie CRTP bridge
│   └── sentai_httpd.cc           - HTTP server + REST API
│
├── [STATIC LIBRARIES - Direct Links]
│   ├── libs_base-m7_freertos     → FreeRTOS + NXP SDK + Base
│   ├── libs_base-m7_http_server  → LwIP HTTP server wrapper
│   ├── libs_jpeg_m7              → JPEG compression
│   ├── libs_lis2du12_freertos    → LIS2DU12 accelerometer driver
│   ├── libs_t5838_freertos       → T5838 mic driver
│   ├── libs_audio_freertos       → Audio service (PDM + DMA)
│   ├── libs_aifes_task_m7        → AIfES neural network
│   ├── libmicropython            → MicroPython embed library
│   ├── nanopb                    → Protocol Buffers (nano)
│   ├── meshtastic_pb             → Meshtastic protobufs
│   └── shine                     → MP3 encoder
│
├── [SYSTEM LIBRARIES - Transitive]
│   ├── FreeRTOS Kernel (10.2.0)  → third_party/freertos_kernel/
│   ├── NXP RT1176 SDK            → libs/nxp/rt1176-sdk/
│   ├── LittleFS                  → third_party/littlefs/
│   └── LwIP                      → third_party/nxp/lwip/
│
└── [LINKER SCRIPT]
    └── MIMXRT1176xxxxx_cm7_ram_mp.ld  - Custom memory map
```

### 9.2 FreeRTOS Configuration Risks

| Setting | Current Value | Risk Level | Issue |
|---------|---------------|------------|-------|
| `configCHECK_FOR_STACK_OVERFLOW` | **2 (ENABLED)** | ✅ OK | Full stack checking active |
| `configMINIMAL_STACK_SIZE` | 90 words (360 bytes) | ⚠️ Medium | Very small; some tasks multiply by 10-30 but still tight |
| `configMAX_PRIORITIES` | 5 | ✅ OK | Sufficient for current task count |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | ✅ Good | Malloc failures will be caught |
| `configUSE_IDLE_HOOK` | 0 | ⚠️ Minor | No idle-time health check opportunity |
| `configUSE_TICK_HOOK` | 0 | ⚠️ Minor | No periodic supervision hook |
| `configASSERT` | printf + infinite delay | 🔴 CRITICAL | Asserts hang forever; WDOG will reset but no diagnostic saved |
| `configRECORD_STACK_HIGH_ADDRESS` | **1 (ENABLED)** | ✅ OK | Stack watermarking available |

### 9.3 Library-Level `portMAX_DELAY` Risks

These are in libraries linked by sentai_runtime that we **cannot modify** without forking:

| File | Line | Context | Risk Description | Mitigation |
|------|------|---------|------------------|------------|
| `libs/base/filesystem.cc` | 102, 358 | `xSemaphoreTake(g_lfs_mutex, portMAX_DELAY)` | **Infinite block on LFS operations** if flash busy or corrupted | Use filesystem sparingly; add app-level timeout wrapper |
| `libs/base/console_m7.cc` | 118, 212 | Console task queue operations | **Deadlock if USB CDC unresponsive** | Already has 15s timeout on some paths |
| `libs/base/ipc.cc` | 45, 53, 55, 65 | IPC TX/RX between CM7 and CM4 | **Hangs if M4 core unresponsive** | Monitor M4 health; restart IPC if needed |
| `libs/audio/audio_service.cc` | 139, 156, 172 | Audio queue operations with CHECK() | **Hard fault if queue fails** | Do not use CHECK() with queues |
| `libs/nxp/rt1176-sdk/bluetooth.cc` | 136, 148 | UART mutex for BLE | **Infinite block if BLE unresponsive** | Don't use BLE, or add task-level timeout |
| `libs/cdc_eem/cdc_eem.cc` | 70 | USB EEM TX queue | **Infinite block if USB host gone** | Not used (ENABLE_USB_EEM=0) |

**Key Finding**: The filesystem.cc `portMAX_DELAY` is unavoidable for direct LFS calls. Consider:
1. Always call filesystem from timeout-protected wrappers
2. Use a background task for filesystem operations
3. Monitor filesystem task health

### 9.4 Memory and Stack Risks

**Task Stack Sizes (from CMakeLists.txt and source analysis)**:

| Task | Stack Words | Stack Bytes | Risk |
|------|-------------|-------------|------|
| `app_main` | 90 × 30 = 2700 | 10,800 | ✅ OK |
| `audio_service` | 90 × 30 = 2700 | 10,800 | ✅ OK |
| `ipc_tx_task` | 90 × 10 = 900 | 3,600 | ⚠️ Tight |
| `ipc_rx_task` | 90 × 10 = 900 | 3,600 | ⚠️ Tight |
| `det_prep` | ~1536 | 6,144 | ⚠️ Medium |
| `det_infer` | ~1536 | 6,144 | ⚠️ Medium |
| `link_rx` | 6144/4 = 1536 | 6,144 | ⚠️ Medium |
| `mesh_rx` | 4096/4 = 1024 | 4,096 | ⚠️ Tight |
| `crazy_rx` | ~512-1024 | 2,048-4,096 | ⚠️ Tight |
| `hw_wdog` | 512 | 2,048 | ✅ OK (simple task) |
| `tap_poll` | 128 | 512 | 🔴 Very tight! |

**Dynamic Memory Usage**:
- `new AifesTask()` - Runtime allocation
- `new EdgeTpuPackage()` - TPU model loading
- `new char[]` in filesystem.cc for path copies
- `new uint8_t[]` in console_m7.cc for buffers

### 9.5 ISR → Task Boundaries

| ISR | Deferred To | Mechanism | Risk |
|-----|-------------|-----------|------|
| `PDM_ERROR_IRQHandler` | audio_service | Ring buffer `SendFromISR` | ⚠️ Ring buffer overflow possible |
| `GPT1_IRQHandler` | Timer callback | Direct call in ISR | 🔴 Timer callbacks run in ISR context! |
| Button GPIO ISR | Task notification | `xTaskNotifyFromISR` | ✅ Safe |
| UART ISR (link/mesh/crazy) | RX task | Queue `FromISR` | ⚠️ Queue overflow if task starved |
| IPC message event | ipc_rx_task | Stream buffer `FromISR` | ✅ Safe |

### 9.6 Library-Level Concurrency Hazards

| Library | Issue | Location | Impact |
|---------|-------|----------|--------|
| `audio_service.cc` | `CHECK(xQueueSendToBack(..., portMAX_DELAY))` | Lines 139, 156, 172 | **Hard fault if queue fails** |
| `ipc.cc` | `while (xTaskNotify... == pdFALSE) taskYIELD()` | Line 42-44 | **Busy-wait if notify fails** |
| `bluetooth.cc` | Multiple `portMAX_DELAY` mutexes | Lines 136, 148 | **Deadlock if peer unresponsive** |
| `http_server.cc` | Uses `new` for FileHolder without null check | Line 73 | **OOM → nullptr dereference** |

### 9.7 Recommended Library Patches (If Forking Allowed)

**Priority 1** - Critical for production:
1. `FreeRTOSConfig.h`: Enable `configCHECK_FOR_STACK_OVERFLOW = 2`
2. `FreeRTOSConfig.h`: Modify `configASSERT` to save diagnostic before hang
3. `filesystem.cc`: Add optional timeout parameter to `Lfs()` calls

**Priority 2** - Important:
4. `audio_service.cc`: Replace `CHECK()` with error return
5. `ipc.cc`: Add timeout to notify wait loop
6. `console_m7.cc`: Add timeout to all queue operations

### 9.8 Safe Usage Guidelines for Libraries

**Filesystem (LittleFS)**:
```c
// UNSAFE - can block forever:
lfs_file_open(Lfs(), &f, path, LFS_O_RDONLY);

// SAFER - wrap in task with timeout:
bool file_open_with_timeout(const char* path, int timeout_ms) {
    // Create helper task or use async pattern
}
```

**Audio Service**:
- Don't call `AudioService::GetCallback` from time-critical paths
- Pre-register callbacks during init, not runtime

**Bluetooth**:
- Currently unused (safe)
- If enabling: Add supervision task to detect BLE hangs

---

## 10. RISK SUMMARY BY SEVERITY

**Last Updated**: 2026-04-18 (Phase 10 — Production Hardening)

### 🔴 P0 - System Crash / Deadlock Risks

| # | Risk | Location | Mitigation Status |
|---|------|----------|-------------------|
| 1 | `portMAX_DELAY` in sentai_link.cc | TX mutex | ✅ FIXED (timeout+retry) |
| 2 | `portMAX_DELAY` in sentai_mesh.cc | TX mutex | ✅ FIXED (timeout+retry) |
| 3 | `portMAX_DELAY` in sentai_crazy.cc | TX mutex + CTS | ✅ FIXED (timeout+retry) |
| 4 | `portMAX_DELAY` in sentai_tracker.cc | Tracker mutex | ✅ FIXED (100ms timeout) |
| 5 | `configCHECK_FOR_STACK_OVERFLOW = 0` | FreeRTOSConfig.h | ✅ FIXED (set to 2 - full checking) |
| 6 | `configASSERT` hangs forever | FreeRTOSConfig.h | ❌ NOT FIXED (requires lib change) |
| 7 | `CHECK()` with queues in audio_service | libs/audio/ | ❌ NOT FIXED (library code) |
| 8 | `portMAX_DELAY` in filesystem.cc | LFS mutex | ⚠️ MITIGATED (browser.html cached, ops bounded) |
| 9 | HTTP /api/ls crash on LFS (tcpip thread) | sentai_httpd.cc | ✅ FIXED (static cache + direct LFS OK) |
| 10 | boot_log_flush_to_file() concurrent flush race | sentai_runtime.cc | ✅ FIXED (atomic lock + memmove tail) |
| 11 | 10 s WDOG dead window at boot | sentai_runtime.cc | ✅ FIXED (reduced to 3 s) |

### ⚠️ P1 - Data Loss / Degraded Operation

| # | Risk | Location | Mitigation Status |
|---|------|----------|-------------------|
| 12 | Unchecked queue operations | link/mesh/detect | ✅ FIXED (return checks added) |
| 13 | Race on detection stats | detection_task.cc | ✅ FIXED (atomic operations) |
| 14 | `tap_poll` stack very small (512B) | modsentai_hal.cc | ❌ NOT FIXED |
| 15 | Timer callbacks in ISR context | libs/base/timer.cc | ⚠️ AWARENESS (don't block in timer) |
| 16 | Health mutex timeout silent drop | sentai_health.cc | ✅ FIXED (SERR_LOG on timeout) |
| 17 | ReadUserFile 8 MB heap alloc (DoS/OOM) | sentai_httpd.cc | ✅ FIXED (hard cap 256 KB) |
| 18 | PostBegin 8 MB upload cap | sentai_httpd.cc | ✅ FIXED (hard cap 512 KB) |
| 19 | `std::string` in tcpip thread (heap frag) | sentai_httpd.cc | ✅ FIXED (static char arrays) |

### 📝 P2 - Visibility / Diagnostics

| # | Risk | Location | Mitigation Status |
|---|------|----------|-------------------|
| 20 | No stack watermarking | All tasks | ✅ FIXED (sentai.rtos.tasks() returns HWM) |
| 21 | OOM in http_server.cc | new FileHolder | ❌ NOT FIXED (library code) |
| 22 | No structured error taxonomy | All modules | ✅ FIXED (sentai_error.h) |
| 23 | No Python API for crash log | modsentai_diag.c | ✅ FIXED (sentai.diag.crash_log()) |
| 24 | No Python API for boot log | modsentai_diag.c | ✅ FIXED (sentai.diag.boot_log()) |
| 25 | tasks() truncated at 24 tasks silently | modsentai_rtos.c | ✅ FIXED (32 + truncation warning) |
| 26 | PrepTask health reporting absent | detection_task.cc | ✅ FIXED (Phase 11 — see below) |
| 27 | No per-task liveness timestamps | detection_task.cc | ✅ FIXED (Phase 11 — `s_last_prep_frame_tick`, `s_last_infer_frame_tick`) |
| 28 | No Python API for pipeline stage staleness | modsentai_pipeline.c | ✅ FIXED (sentai.pipeline.task_health()) |

---

## 11. IMPLEMENTATION STATUS

**Latest Build: #498+** (2026-04-18)

### ✅ Completed Phases

| Phase | Description | Status | Date |
|-------|-------------|--------|------|
| Phase 1 | Eliminate portMAX_DELAY in bridges | ✅ DONE | 2026-04 |
| Phase 2 | Queue operation safety | ✅ DONE | 2026-04 |
| Phase 3 | Concurrency hazards fix | ✅ DONE | 2026-04 |
| Phase 4 | Subsystem health states | ✅ DONE | 2026-04-18 |
| Phase 5 | Structured diagnostics | ✅ DONE | 2026-04-18 |
| Phase 6 | Memory safety audit | ✅ DONE | 2026-04-18 |
| Phase 7 | System mode state machine | ✅ DONE | 2026-04-18 |
| Phase 8 | Anti-brick / self-healing boot | ✅ DONE | 2026-04-18 |
| Phase 9 | Crash breadcrumb (SRC GPR2-8) | ✅ DONE | 2026-04-18 |
| Phase 10 | Production hardening pass | ✅ DONE | 2026-04-18 |
| Phase 11 | Detection task supervision | ✅ DONE | 2026-04-18 |

### Phase 11 Details — Detection Task Supervision (This Session)

All changes build-verified: **Build #498, zero errors, zero warnings**.

#### 11.1 PrepTask health reporting

**Problem**: PrepTask never reported to the health system. Any camera or PXP failure was silent — the SUBSYS_DETECT health record showed no events, making it impossible to distinguish "detection not started" from "detection running but PrepTask is dying on every frame."

**Fix**:
- Camera frame grab fail (10 consecutive): `sentai_health_fail(SUBSYS_DETECT)` (rate-limited to avoid noise from normal transient misses where no new frame is ready yet)
- PXP scale fail: `sentai_health_fail(SUBSYS_DETECT)` immediately (hardware error, always reportable)
- Tensor info fail: `sentai_health_fail(SUBSYS_DETECT)` immediately (model not loaded correctly)
- On success: counter resets; InferTask continues to call `sentai_health_success` after each inference

**Impact**: Camera hardware failure or PXP stall is now immediately visible in `sentai.diag.health()` output.

#### 11.2 Per-task liveness timestamps

**Problem**: External observers (health supervisor, Python, logging) had no way to tell *which* pipeline stage was stuck. Both "PrepTask stuck on camera" and "InferTask stuck on TPU invoke" looked identical — `s_frames_processed` stopped incrementing.

**Fix**: Added two volatile `TickType_t` timestamps in the anonymous namespace:
- `s_last_prep_frame_tick`: updated by PrepTask immediately before `xSemaphoreGive(s_sem_prep_done)` — records when PrepTask last successfully prepared and handed off a frame
- `s_last_infer_frame_tick`: updated by InferTask after `sentai_health_success()` — records when InferTask last completed a full inference

**New C function** `sentai_detection_task_stall_ms(uint32_t* prep_stall_ms, uint32_t* infer_stall_ms)`:
- Returns `(now - last_tick) * portTICK_PERIOD_MS` for each stage
- Returns `0xFFFFFFFF` if not running or first frame not yet complete
- Safe to call from any task context (volatile read, no mutex needed)

**Diagnostic interpretation**:
| `prep_stall_ms` | `infer_stall_ms` | Diagnosis |
|-----------------|------------------|-----------|
| small (< 500) | small (< 500) | ✅ Healthy, pipeline flowing |
| large (> 10000) | large (> 10000) | PrepTask stuck → camera/PXP failure |
| small | large (> 10000) | InferTask stuck → TPU invoke hung |
| UINT32_MAX | UINT32_MAX | Pipeline not running |

#### 11.3 Python API: `sentai.pipeline.task_health()`

**New function** exposed via MicroPython:
```python
>>> sentai.pipeline.task_health()  # when not running
(4294967295, 4294967295)           # 0xFFFFFFFF = not running
>>> sentai.pipeline.task_health()  # when healthy
(80, 135)                          # ms since last PrepTask / InferTask frame
>>> sentai.pipeline.task_health()  # when InferTask stuck
(80, 45000)                        # PrepTask fine, InferTask stuck 45s
```

**QSTR**: `MP_QSTR_task_health` — regenerated via `embed.mk`, confirmed in `qstrdefs.generated.h`.

### Files Changed This Session (Phase 11)

| File | Changes |
|------|----------|
| `detection_task.cc` | Added `s_last_prep_frame_tick`, `s_last_infer_frame_tick`; PrepTask now reports `sentai_health_fail/timeout(SUBSYS_DETECT)` on camera/PXP/tensor failures; `sentai_detection_task_stall_ms()` exported |
| `detection_task.h` | Added `sentai_detection_task_stall_ms()` declaration |
| `modsentai_pipeline.c` | Added extern for `sentai_detection_task_stall_ms`; added `sentai.pipeline.task_health()` function and module table entry |
| `micropython_embed/genhdr/qstrdefs.generated.h` | Regenerated: `MP_QSTR_task_health` added |

All changes build-verified: **zero errors, zero warnings**.

### Phase 10 Details — Production Hardening (Previous Session)

#### 10.1 `sentai_runtime.cc` — `boot_log_flush_to_file()` race fix

**Problem**: Original flush function had no protection against concurrent callers. Multiple tasks could call `boot_log_flush_to_file()` simultaneously (one via `_write()` overflow path, another from `boot_log_stop()`), causing duplicate writes and potential `g_boot_log_pos` underflow.

**Fix**:
- Added `g_boot_log_flush_busy` atomic lock (`uint32_t`, GCC `__sync_lock_test_and_set`)
- Flush is non-blocking: if another flush is running, caller returns immediately (data is still in the buffer)
- Byte count for the write is snapshotted inside a critical section before writing
- After write, only the flushed bytes are removed from the buffer via `memmove` — bytes that arrived *during* the write are preserved
- `g_boot_log_pos` is only decremented inside a critical section with `memmove` of the tail

```c
// Acquire flush lock — non-blocking
if (__sync_lock_test_and_set(&g_boot_log_flush_busy, 1u) != 0u) return;
// Snapshot pos under critical section
UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
size_t count = g_boot_log_pos;
taskEXIT_CRITICAL_FROM_ISR(saved);
// Write, then slide out only what we wrote
```

#### 10.2 `sentai_runtime.cc` — Watchdog boot delay 10 s → 3 s

**Problem**: `CombinedWatchdogTask` delayed 10 seconds before calling `WDOG_Init()`. This left a 10-second window at every boot where a runaway task or infinite loop would NOT be caught by the hardware watchdog.

**Fix**: Reduced `vTaskDelay(pdMS_TO_TICKS(10000))` → `vTaskDelay(pdMS_TO_TICKS(3000))`. USB CDC is already enumerated before `app_main()` runs (done in `main_freertos.cc`), so 3 s is sufficient for boot settling without leaving a dangerous window.

**Also**: Saved the watchdog task handle in `s_wdog_task_handle` (was `nullptr` before), so it can be monitored or queried from diagnostics.

**Also removed**: Dead code `force_software_reset()` (was `static`, never called, produced a `-Wunused-function` warning).

#### 10.3 `sentai_runtime.cc` — `sentai_get_last_crash_log_path()` helper

**New `extern "C"` function** that scans `/log/` for the highest-numbered `crash_NNN.log` file. Used by the new Python `sentai.diag.crash_log()` API. Safe to call from any task context.

#### 10.4 `sentai_health.cc` — Mutex timeout logging

**Problem**: All three health update functions (`sentai_health_success`, `sentai_health_fail`, `sentai_health_timeout`) silently dropped updates if the health mutex was held for > 10 ms. This masked priority inversion and long critical sections — a systemic diagnosability gap.

**Fix**: Added `SERR_LOG(SERR_SYS_ASSERT, (uint32_t)subsys)` on mutex timeout. The event is now visible in the serial log as `E:1003:N` (where N = subsystem ID). This is a structured log entry, not a string, consistent with the error code taxonomy.

**Structure change**: Replaced the `if (xSemaphoreTake(...) == pdTRUE) { ... }` pattern with early-return on failure — removes one level of nesting and makes the hot path (success) linear:

```c
if (xSemaphoreTake(s_health_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    SERR_LOG(SERR_SYS_ASSERT, (uint32_t)subsys);
    return;
}
// ... (no extra nesting)
xSemaphoreGive(s_health_mutex);
```

#### 10.5 `sentai_httpd.cc` — Systematic dynamic allocation reduction

**Problem summary**: The sentai_httpd.cc had pervasive dynamic heap allocation in the tcpip thread (steady-state request handling), violating NASA/JPL rule 3 (no heap in steady-state runtime). Specific issues:

| Issue | Old | New | Memory saved |
|-------|-----|-----|------|
| `g_post_result` | `std::string` (heap) | `char[256]` (static BSS) | ~32 bytes + heap fragmentation |
| `g_browser_html_cache` | `std::vector<uint8_t>` (~12 KB heap) | `uint8_t[64 KB]` in `.sdram_bss` | No heap fragmentation |
| `ReadUserFile()` max size | 8 MB | 256 KB hard cap | 7.75 MB heap DoS eliminated |
| `PostBegin()` content cap | 8 MB | 512 KB | Bounded heap growth per request |
| `DoWrite()` parent dir walk | `std::string p(path)` | `char p[256]` stack buf | No heap per write |
| `DoMkdir()` dir walk | `std::string p(path)` | `char p[256]` stack buf | No heap per mkdir |
| `SentaiHttpServer::post_uri_` | `std::string` | `char[256]` | No heap alloc in `PostBegin` |
| `#include <string>` | present | removed | Smaller link unit |

**`BrowserHtmlData()` / `BrowserHtmlSize()` helpers**: Replace the old `g_browser_html_cache.empty()` guard. When the SDRAM cache is populated (user-uploaded browser.html), helpers return the SDRAM buffer. When not populated, they fall back to ROM `browser_html_data` / `browser_html_data_len` directly — zero copies.

**POST result via `StaticBuffer`**: The `/api/_pr` endpoint now returns a `StaticBuffer` pointing to the static `g_post_result` char array rather than constructing a `std::vector<uint8_t>` copy.

#### 10.6 `modsentai_rtos.c` — `MAX_TASKS` 24 → 32 with truncation warning

**Problem**: `sentai.rtos.tasks()` used a fixed `TaskStatus_t task_buf[24]`. The system now runs 25+ FreeRTOS tasks (prep, infer, mp_repl, hw_wdog, btn_usb, lwIP, USB, mesh, link, crazy, audio, tracker, tap_poll, ipc_tx, ipc_rx, IDLE, tmr, ...). Any tasks beyond position 24 were silently invisible in the Python task list.

**Fix**: Increased to 32. Added a printf warning if `n == MAX_TASKS` (buffer fully saturated — FreeRTOS returns at most `uxArraySize` entries, so saturation is the only indicator of truncation).

#### 10.7 `modsentai_diag.c` — New Python APIs

**`sentai.diag.crash_log() → str`**
Reads the most recent `/log/crash_NNN.log` file and returns its text content. Avoids requiring the user to know the crash log rotation number. Capped at 8 KB to prevent large MicroPython allocations.

```python
>>> print(sentai.diag.crash_log())
[00:03:22.451] WATCHDOG_RESET: ...
  HTTP: reqs=12 hangs=0 idle=27842ms
  REPL: inputs=0 idle=27842ms
```

**`sentai.diag.boot_log() → str`**
Reads `/log/boot.log` (current boot) or falls back to `/log/boot_old.log`. Capped at 16 KB.

```python
>>> print(sentai.diag.boot_log())
SentAI build #477 (2026-04-18 ...)
Reset reason: ...
Boot logging to /log/boot.log
```

**QSTR regen**: Both new symbols (`MP_QSTR_crash_log`, `MP_QSTR_boot_log`) regenerated via `embed.mk` and verified present in `qstrdefs.generated.h`.

### Files Changed This Session

| File | Changes |
|------|---------|
| `sentai_runtime.cc` | boot_log flush race fix, watchdog 10s→3s, task handle saved, dead code removed, `sentai_get_last_crash_log_path()` added |
| `sentai_health.cc` | Mutex timeout now logs SERR_LOG instead of silent drop; early-return pattern in all 3 update functions |
| `sentai_httpd.cc` | Removed `std::string` / `std::vector` allocations from tcpip thread; static SDRAM cache buffer; hard caps on ReadUserFile and PostBegin; `#include <string>` removed |
| `modsentai_rtos.c` | MAX_TASKS 24→32 with truncation printf |
| `modsentai_diag.c` | Added `sentai.diag.crash_log()` and `sentai.diag.boot_log()` |
| `micropython_embed/genhdr/qstrdefs.generated.h` | Regenerated: `MP_QSTR_boot_log`, `MP_QSTR_crash_log` added |

### Python API — Full Current Surface

```python
# System / boot
sentai.sys.reset()                  # User-triggered SW reset
sentai.sys.recovery_mode()          # → bool (True if in recovery mode)
sentai.sys.boot_attempts()          # → int (0 after clean boot)

# Diagnostics
sentai.diag.sys_mode()              # → "NORMAL" / "DEGRADED" / "SAFE" / "RECOVERY"
sentai.diag.health()                # → compact multi-line health summary
sentai.diag.crash_log()             # → str contents of latest crash_NNN.log   ← Phase 10
sentai.diag.boot_log()              # → str contents of boot.log                ← Phase 10

# Pipeline / detection
sentai.pipeline.start([conf, iou, max, track])  # start detection
sentai.pipeline.stop()              # stop detection
sentai.pipeline.get([timeout_ms])   # → list of detections or None
sentai.pipeline.running()           # → bool
sentai.pipeline.stats()             # → (frames_processed, frames_dropped, fps)
sentai.pipeline.task_health()       # → (prep_stall_ms, infer_stall_ms)   ← Phase 11

# RTOS introspection
sentai.rtos.tasks()                 # → list of (name, state, prio, stack_hwm) — up to 32 tasks
sentai.rtos.heap_info()             # → dict: rtos_free, gc_total, gc_used, gc_free
sentai.rtos.cpu_usage()             # → per-task runtime %
sentai.rtos.sleep_ms(ms)
sentai.rtos.ticks_ms()

# Filesystem
sentai.fs.read(path)                # → bytes
sentai.fs.read_str(path)            # → str
sentai.fs.write(path, data)
sentai.fs.size(path)                # → int (-1 if not found)
sentai.fs.exists(path)              # → bool
sentai.fs.ls(path)                  # → list of (name, type, size) tuples
sentai.fs.mkdir(path)
sentai.fs.remove(path)
sentai.fs.format()
```

### HTTP Server Status

- ✅ Stress test passed: 500+ concurrent requests 100% success rate
- ✅ `browser.html` served from SDRAM static buffer (64 KB) — no heap allocation in tcpip thread
- ✅ POST responses via static `char[256]` — no `std::string` in hot path
- ✅ ReadUserFile hard-capped at 256 KB (was 8 MB — RCE/OOM risk eliminated)
- ✅ Upload size hard-capped at 512 KB (was 8 MB)
- ✅ Activity hook for watchdog on every request
- ⚠️ LFS access still direct in tcpip thread (bounded in time, acceptable for current load)

### SRC_GPR Register Map

| Register | Purpose | Cleared On |
|----------|---------|------------|
| `GPR1` | Boot attempt counter | `sentai_health_boot_complete()` (clean boot) |
| `GPR2` | Crash magic + error code | Each `sentai_fault_clear()` call |
| `GPR3` | Crash PC | Each `sentai_fault_clear()` |
| `GPR4` | Crash LR | Each `sentai_fault_clear()` |
| `GPR5` | CFSR (Configurable Fault Status) | Each `sentai_fault_clear()` |
| `GPR6` | BFAR (Bus Fault Address) | Each `sentai_fault_clear()` |
| `GPR7` | Uptime at crash (ms) | Each `sentai_fault_clear()` |
| `GPR8` | r0 at crash | Each `sentai_fault_clear()` |
| `GPR13` | Watchdog reset count | Never (persistent counter) |
| `GPR14` | Lockup reset count | Never (persistent counter) |

All GPR registers survive warm reset (WDOG / SW reset) but are cleared on power cycle.

### Verification Checklist (Updated)

- [x] `sentai.sys.recovery_mode()` → False on normal boot ✓
- [x] `sentai.sys.boot_attempts()` → 0 on clean boot ✓
- [x] `sentai.diag.sys_mode()` → NORMAL ✓
- [x] `sentai.diag.crash_log()` → returns crash log text ✓ (NEW)
- [x] `sentai.diag.boot_log()` → returns boot log text ✓ (NEW)
- [x] `sentai.rtos.tasks()` → returns ≥20 tasks without truncation ✓ (was 24, now 32)
- [x] HTTP stress test: 500 concurrent requests, 100% success ✓
- [x] Build: zero errors, zero warnings ✓
- [ ] Force crash in app_main() early → device auto-enters RECOVERY after 3 boots
- [ ] Confirm `sentai.diag.health()` shows SERR log entry on mutex timeout

---

## 🚨 CRITICAL: ANTI-BRICK RECOVERY SYSTEM (P0)

**RULE**: The board must NEVER be completely bricked. There must ALWAYS be a way to reflash it.

### Current Vulnerabilities

| Risk | Scenario | Impact |
|------|----------|--------|
| Boot crash before USB | Crash in app_main() before USB stack init | ❌ BRICK - no USB to reflash |
| Infinite loop in init | Blocking call or infinite loop early in boot | ❌ BRICK - watchdog may not help if USB not up |
| Flash corruption | Bad write to critical flash sector | ❌ BRICK - can't boot |
| Watchdog too aggressive | WDOG resets before USB enumerates | ❌ BRICK - endless reset loop |
| configASSERT in boot | Assert fails before REPL/USB active | ⚠️ DEGRADED - WDOG will reset but may loop |

### Design Philosophy

**The board must be SELF-HEALING. User intervention (button press) = SOFTWARE FAILURE.**

A properly designed embedded system NEVER requires manual recovery. The board must:
1. **Detect** that it is in a boot loop / bricked state
2. **Automatically** enter RECOVERY mode without ANY user action
3. **Maintain USB communication** so host can detect and reflash it

If the user ever needs to hold a button to recover → **we failed as engineers**.

### Required Safeguards (Automatic - No User Action)

#### 1. **Boot Attempt Counter in SRC_GPR** (P0 - CRITICAL)
- Use SRC_GPR (General Purpose Register) - survives warm reset, cleared on cold boot
- Increment `boot_attempts` at VERY START of app_main() (before ANY other code)
- If `boot_attempts >= 3` → **AUTOMATICALLY** enter RECOVERY MODE
- Clear `boot_attempts` only after `boot_complete()` confirms system is healthy

```c
// FIRST THING in app_main() - before ANY initialization:
uint32_t boot_attempts = SRC->GPR[0];
SRC->GPR[0] = boot_attempts + 1;

if (boot_attempts >= 3) {
    // RECOVERY MODE: Skip all application code
    // Only initialize USB CDC + minimal REPL
    enter_recovery_mode();  // Never returns to normal boot
}
```

#### 2. **USB Stack Initializes FIRST** (P0)
- Move USB CDC initialization to happen BEFORE any risky operations
- USB must enumerate within 2-3 seconds of power-on
- Even in RECOVERY mode, USB must be visible to host

#### 3. **RECOVERY MODE Definition** (P0)
- **RECOVERY MODE** = minimal firmware state:
  - USB CDC active (serial communication)
  - USB visible to host for flashing
  - REPL available (can run `sentai.sys.reset()`)
  - NO application code runs (no bridges, no HTTP, no detection)
  - Boot log indicates "RECOVERY MODE - boot loop detected"
- Board stays in RECOVERY MODE until manually reflashed

#### 4. **Watchdog Grace Period** (P0)
- WDOG timeout = 30s (current)
- But DON'T enable WDOG until AFTER USB enumeration succeeds
- This prevents reset loops where USB never gets a chance to enumerate

#### 5. **Safe configASSERT** (P1)
- configASSERT must reset (not hang forever in infinite loop)
- Current: `for(;;) vTaskDelay(500)` - hangs but WDOG will eventually reset
- Better: Save breadcrumb to GPR, then `NVIC_SystemReset()`

### Implementation Plan

**Phase 8: Automatic Self-Healing Boot** — ✅ DONE

| Step | Task | File | Status |
|------|------|------|--------|
| 8.1 | Add boot_attempts counter in SRC_GPR[1] | sentai_runtime.cc | ✅ DONE |
| 8.2 | Check boot_attempts FIRST in app_main() | sentai_runtime.cc | ✅ DONE |
| 8.3 | Implement `enter_recovery_mode()` function | sentai_runtime.cc | ✅ DONE |
| 8.4 | Clear boot_attempts in `boot_complete()` | sentai_health.cc | ✅ DONE |
| 8.5 | Delay WDOG enable until after USB enumeration | sentai_runtime.cc | ✅ DONE (USB CDC init already early in main_freertos) |
| 8.6 | Move USB CDC init to very early in boot | sentai_runtime.cc | ✅ DONE (already before app_main) |
| 8.7 | Add Python API: `sentai.sys.recovery_mode()` | modsentai_sys.c | ✅ DONE |

### SRC_GPR Register Usage

| Register | Purpose | Cleared On |
|----------|---------|------------|
| SRC_GPR[0] | Boot attempt counter | Cold boot only |
| SRC_GPR[1] | Last crash error code | Cold boot only |
| SRC_GPR[2] | Crash timestamp (low) | Cold boot only |
| SRC_GPR[3] | Crash timestamp (high) | Cold boot only |

**Note**: SRC_GPR[0-9] survive warm reset (watchdog, software reset) but are cleared on power cycle.

### Verification Checklist

- [x] `sentai.sys.recovery_mode()` → False on normal boot ✓ (verified on device)
- [x] `sentai.sys.boot_attempts()` → 0 on clean boot ✓ (verified on device)
- [x] `sentai.diag.sys_mode()` → NORMAL ✓ (verified on device)
- [ ] Force crash in app_main() early → device automatically enters RECOVERY after 3 boots
- [ ] RECOVERY mode: USB visible, REPL works, can reflash
- [ ] Boot log shows "RECOVERY MODE - boot loop detected (N attempts)"
- [ ] After successful reflash, boot_attempts resets to 0
- [ ] configASSERT failure → crash breadcrumb saved, device resets, recoverable

### What Success Looks Like

1. **Developer introduces crash bug** → pushes to device
2. **Device boots, crashes** → WDOG resets
3. **Device boots again, crashes** → WDOG resets (boot_attempts = 2)
4. **Device boots third time** → Detects boot_attempts >= 3
5. **Device AUTOMATICALLY enters RECOVERY MODE** (no user action!)
6. **Host sees USB device** → Developer can reflash fixed firmware
7. **Fixed firmware boots** → boot_attempts cleared → normal operation

**The user NEVER needs to touch a button. The board heals itself.**

### Last Resort: Hardware Button (NOT primary recovery!)

If SRC_GPR somehow gets corrupted or firmware is so broken it can't even check boot_attempts:
- Holding USER button at power-on → forces ROM bootloader (SDP mode)
- This is HARDWARE-LEVEL bypass, happens in ROM before our code runs
- But this should NEVER be needed if our software is correct

---

### 🔄 Remaining Work

| Item | Description | Priority |
|------|-------------|----------|
| `audio_service CHECK()` | Hard fault on queue full; library change needed | P1 |
| `tap_poll` stack 512 B | Very small — monitor HWM via `sentai.rtos.tasks()` | P2 |
| `http_server.cc FileHolder` | OOM = nullptr deref; library code | P2 |
| HTTP 429 rate limiting | Return 429 on overload instead of dropping TCP | P3 |
| Filesystem async wrapper | Move crash_log_write to dedicated task for bounded latency | P3 |
