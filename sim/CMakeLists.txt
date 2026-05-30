# sim/CMakeLists.txt — SIM-only build targets.
#
# Only added to the build when SENTAI_SIM=ON.  See Sim.md and the top-level
# CMakeLists.txt for the toolchain wiring.

# Host pthread / librt for the FreeRTOS POSIX port.  Called here (not in the
# toolchain file) because find_package needs language support enabled by
# project() first.
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Auto-increment SIM build counter.
#
# Reuses the same script as the ARM firmware (examples/sentai_runtime/
# increment_version.cmake) but writes to a SEPARATE file under sim/ so the
# SIM and ARM build numbers don't collide.  ARM is at build #N, SIM is at
# its own build #M, both timestamped at last build.  Banner in main_sim.c
# prints "SentAI SIM build #M (timestamp)" so the operator always knows
# what's running.
# ---------------------------------------------------------------------------
add_custom_target(increment_sim_build_version ALL
    COMMAND ${CMAKE_COMMAND}
        -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/increment_version.cmake
    COMMENT "Incrementing SIM build version..."
)

# ---------------------------------------------------------------------------
# libfreertos_posix — FreeRTOS kernel built with the POSIX/Linux port.
# ---------------------------------------------------------------------------
set(FREERTOS_KERNEL_DIR ${CMAKE_SOURCE_DIR}/third_party/freertos_kernel)
set(FREERTOS_POSIX_PORT_DIR ${FREERTOS_KERNEL_DIR}/portable/ThirdParty/GCC/Posix)

add_library_sim(libfreertos_posix STATIC
    ${FREERTOS_KERNEL_DIR}/tasks.c
    ${FREERTOS_KERNEL_DIR}/queue.c
    ${FREERTOS_KERNEL_DIR}/list.c
    ${FREERTOS_KERNEL_DIR}/timers.c
    ${FREERTOS_KERNEL_DIR}/event_groups.c
    ${FREERTOS_KERNEL_DIR}/stream_buffer.c
    ${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_POSIX_PORT_DIR}/port.c
    ${FREERTOS_POSIX_PORT_DIR}/utils/wait_for_event.c
)

target_include_directories(libfreertos_posix PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                # sim/FreeRTOSConfig.h
    ${FREERTOS_KERNEL_DIR}/include
    ${FREERTOS_POSIX_PORT_DIR}
    ${FREERTOS_POSIX_PORT_DIR}/utils
)

target_link_libraries(libfreertos_posix PUBLIC
    Threads::Threads
    rt
)

# Suppress benign warnings from upstream FreeRTOS sources we don't own.
target_compile_options(libfreertos_posix PRIVATE
    -Wno-unused-but-set-variable
    -Wno-implicit-fallthrough
    -Wno-format
)

# ---------------------------------------------------------------------------
# sentai_sim_kernel_only — Phase 1 step 4 smoke test.
#
# Two FreeRTOS tasks that printf a heartbeat for 5 seconds, then exit.
# Validates that the POSIX port works on the dev machine before we add MP.
# ---------------------------------------------------------------------------
add_executable_sim(sentai_sim_kernel_only
    test_freertos.c
)
target_link_libraries(sentai_sim_kernel_only PRIVATE
    libfreertos_posix
)

# ---------------------------------------------------------------------------
# tpu_posix_smoke — B7 direct Coral USB transport smoke.
#
# Verifies the POSIX/libusb backend can drive the same low-level
# libs/tpu/edgetpu_driver.cc initialization path as ARM, without the Python
# pycoral helper.  This is a transport smoke only; model load/invoke parity is
# a follow-up slice after the direct USB path is stable.
# ---------------------------------------------------------------------------
add_executable_sim(tpu_posix_smoke
    tpu_posix_smoke.cc
    tpu_posix_smoke_freertos_hooks.c
    usb_host_edgetpu_posix.c
    ${CMAKE_SOURCE_DIR}/libs/tpu/edgetpu_driver.cc
    ${CMAKE_SOURCE_DIR}/libs/tpu/apex_latest_single_ep_bin.c
)
target_include_directories(tpu_posix_smoke PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(tpu_posix_smoke PRIVATE
    SENTAI_PLATFORM_SIM=1
)
target_link_libraries(tpu_posix_smoke PRIVATE
    libfreertos_posix
    usb-1.0
)

# ---------------------------------------------------------------------------
# libmicropython_embed_sim — MicroPython embed sources, host build.
#
# Reuses the SAME generated sources as the ARM build
# (examples/sentai_runtime/micropython_embed/{py,port,shared,extmod}/*.c)
# so any change to QSTR/qstrdefs is visible in both targets.  Only mpconfigport.h
# differs: SIM picks up sim/mpconfigport.h via include path priority (sim/
# is listed FIRST so the SIM config wins over the firmware one).
# ---------------------------------------------------------------------------
set(MP_EMBED_DIR ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/micropython_embed)
file(GLOB MP_EMBED_PY_SRCS     ${MP_EMBED_DIR}/py/*.c)
file(GLOB MP_EMBED_PORT_SRCS   ${MP_EMBED_DIR}/port/*.c)
file(GLOB MP_EMBED_SHARED_SRCS ${MP_EMBED_DIR}/shared/runtime/*.c)

add_library_sim(libmicropython_embed_sim STATIC
    ${MP_EMBED_PY_SRCS}
    ${MP_EMBED_PORT_SRCS}
    ${MP_EMBED_SHARED_SRCS}
)
target_include_directories(libmicropython_embed_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                # sim/mpconfigport.h FIRST
    ${MP_EMBED_DIR}                            # py/*.h, genhdr/, port/
)
# MicroPython is C99 and emits many benign warnings — silence them.
target_compile_options(libmicropython_embed_sim PRIVATE
    -std=gnu99
    -w
    -Os
)

# ---------------------------------------------------------------------------
# libh3_sim — Uber H3 hexagonal spatial index (vendored at third_party/h3).
#
# ObjectsPlan L3 (sentai.places) prerequisite.  L1 only vendored the
# submodule + standalone smoke (s119); L3 wires it into the production
# SIM build.  18 lib/*.c sources (no external deps, MIT-licensed C99) +
# generated h3api.h with version macros substituted via configure_file.
#
# Mirror this pattern in examples/sentai_runtime/CMakeLists.txt for the
# ARM build when L3 lands on hardware.
# ---------------------------------------------------------------------------
set(H3_DIR ${CMAKE_SOURCE_DIR}/third_party/h3)
file(STRINGS ${H3_DIR}/VERSION H3_VERSION_FULL LIMIT_COUNT 1)
string(REGEX REPLACE "-.*$" "" H3_VERSION_FULL "${H3_VERSION_FULL}")
string(REPLACE "." ";" H3_VERSION_LIST "${H3_VERSION_FULL}")
list(GET H3_VERSION_LIST 0 H3_VERSION_MAJOR)
list(GET H3_VERSION_LIST 1 H3_VERSION_MINOR)
list(GET H3_VERSION_LIST 2 H3_VERSION_PATCH)
configure_file(
    ${H3_DIR}/src/h3lib/include/h3api.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/h3_gen/h3api.h
    @ONLY
)
add_library_sim(libh3_sim STATIC
    ${H3_DIR}/src/h3lib/lib/algos.c
    ${H3_DIR}/src/h3lib/lib/area.c
    ${H3_DIR}/src/h3lib/lib/baseCells.c
    ${H3_DIR}/src/h3lib/lib/bbox.c
    ${H3_DIR}/src/h3lib/lib/cellsToMultiPoly.c
    ${H3_DIR}/src/h3lib/lib/directedEdge.c
    ${H3_DIR}/src/h3lib/lib/faceijk.c
    ${H3_DIR}/src/h3lib/lib/h3Assert.c
    ${H3_DIR}/src/h3lib/lib/h3Index.c
    ${H3_DIR}/src/h3lib/lib/iterators.c
    ${H3_DIR}/src/h3lib/lib/latLng.c
    ${H3_DIR}/src/h3lib/lib/linkedGeo.c
    ${H3_DIR}/src/h3lib/lib/localij.c
    ${H3_DIR}/src/h3lib/lib/mathExtensions.c
    ${H3_DIR}/src/h3lib/lib/polyfill.c
    ${H3_DIR}/src/h3lib/lib/polygon.c
    ${H3_DIR}/src/h3lib/lib/vec2d.c
    ${H3_DIR}/src/h3lib/lib/vertex.c
)
target_include_directories(libh3_sim PUBLIC
    ${H3_DIR}/src/h3lib/include
    ${CMAKE_CURRENT_BINARY_DIR}/h3_gen
)
target_compile_options(libh3_sim PRIVATE
    -std=gnu11
    -Os
    -w                          # H3 sources are clean upstream; silence host warnings
)

# ---------------------------------------------------------------------------
# sentai_sim — Phase 1 step 8-10 deliverable: REPL on stdin/stdout.
#
# Boots FreeRTOS POSIX scheduler, single REPL task drives MicroPython embed,
# user types `1+1\n`, sees `2\n`.  No SentAI hardware bindings yet
# (sentai.* module is not registered in Phase 1 — it would pull in too
# many NXP SDK dependencies).  Phase 2+ adds the bindings incrementally.
# ---------------------------------------------------------------------------
add_executable_sim(sentai_sim
    main_sim.c
    modsentai_sim.c
    camera_bridge_recv.c          # Phase 4 UDS receiver, runs flow pipeline
    sim_tpu_shim.c                # Phase 5 — pycoral helper UDS client
    fx_user_stubs.c               # W17-T8 — FxUserSize/ReadFile no-op stubs
                                  # so SIM-side PGM loaders compile.
    sentai_uart_serial_udp.c      # Phase 6 — sentai.link UART→UDP bridge for SIM
    sentai_platform_sim_backend.c # SIM backend for shared sentai_runtime bindings
    sentai_link_sim.cc            # Phase 6 — slim MAVLink bridge (heartbeat/statustext)
    sentai_crazy_sim.cc           # Task #39 — sentai.crazy CRTP-over-UDP to cf2 SITL
    # Task #44 — CRTP LOG (TOC scan + block subscribe + pose cache) in C.
    # Shared with ARM via the same SENTAI_PLATFORM_SIM pattern as L2-L5.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_crazy_log.cc
    # Phase 5b — SentAI-SORT tracker, shared with ARM (BoT-SORT adapted).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_tracker.cc
    # ObjectsPlan L2 — persistent world-frame object map (pure data + math,
    # single .cc shared with ARM via the SENTAI_PLATFORM_SIM ifdef in the
    # source).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_objects.cc
    # ObjectsPlan L3 — H3-indexed place gallery + descriptor match.
    # Pure data + math; H3 graph queries via libh3_sim (linked below).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_places.cc
    # PHOG (Bosch 2007) — Track A place descriptor component.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_phog.cc
    # Track A piece 3/4 — HSV 16h×4s histogram (OP-S10-W4).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_hsv.cc
    # Camera-frame slot infrastructure (Phase 1a — storage only).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_prep.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_virtual_camera.cc
    # OP-S10-W11-T3 — SlamTask perception loop (shared with ARM).
    # SIM compiles + runs the task; real frames land once Phase 1d
    # wires camera_bridge_recv as the SLOT_RGB_64 producer.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/slam_task.cc
    # GIST-lite (Oliva & Torralba 2001) — coarse scene envelope.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_gist.cc
    # ObjectsPlan L4 — backend-agnostic action layer (skeleton).
    # Pure FSM + 16-entry trace ring; shared with ARM via the same
    # SENTAI_PLATFORM_SIM ifdef pattern as L2/L3.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_servo.cc
    # ObjectsPlan L5 — inverse-depth EKF landmark lifter.  Pure float
    # math; SIM uses scalar fallback (no CMSIS-DSP).  Validated against
    # experiments/s131_lifter_replay/lifter_proto.py.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_object_lifter.cc
    # Shared 3x3 lin-alg + Jacobi/SVD + rigid-Kabsch aligner.  Extracted
    # from sentai_calib.cc on 2026-05-21 for OP-S10-W19-T6 (sentai.markers
    # multi-marker pose reuses it).  Pure float math, no platform deps.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_svd3.cc
    # ObjectsPlan OP-S6-W1 — sentai.calib (camera-to-body Kabsch).  Pure
    # float math; SIM build skips FxUser (HOST stdio fallback in .cc).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_calib.cc
    # ObjectsPlan OP-S10-W14 — sentai.calib autotuner extension (Flow
    # loop Kp via Åström-Hägglund relay).  Pure-compute state machine
    # + FreeRTOS worker mirroring the sentai.safety split.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_calib_autotune.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_calib_task.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_calib_orientation_task.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_servo_marker_task.cc
    # OP-S10-W21-T4 — bringup orchestrator (SAMPLE → KABSCH → AUTOTUNE
    # X/Y → HOLD → SAVE).  Mirrors ARM build.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_calib_bringup.cc
    # ObjectsPlan OP-S6-W3 — sentai.aruco fiducial detector + PnP.
    # Pure C/C++; SIM zerocopy hook lives in modsentai_sim_camera.c.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_aruco.cc
    # OP-S10-W19-T1 unified pose-emitting dispatcher (sentai.markers).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_markers.cc
    # ObjectsPlan OP-S10-W12 — sentai.safety state machine + SafetyTask
    # worker.  Pure-compute state machine + a pthread worker (FreeRTOS
    # task on ARM) that REUSES sentai_camera_grab_gray_zerocopy +
    # sentai_aruco_detect; no detection / camera intake duplicated.
    # See Safety.md for the architecture.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_safety.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_safety_task.cc
    # ObjectsPlan OP-S10-W13 — sentai.fr Flight Recorder subsystem.
    # NASA/JPL FDR-style: multi-channel bounded-queue producers + a
    # worker task → disk.  Split into state (sentai_fr.cc) + worker
    # (sentai_fr_task.cc), mirroring sentai.safety's split.  SIM-only
    # initially; ARM port deferred.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_fr.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_fr_task.cc
    # ObjectsPlan L6 — mission FSM wrapping L4 servo + L5 lifter.
    # Pure FSM, no transport bytes; same #include pattern as L2-L5.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_explore.cc
    # Phase 4 — flow algorithm shared between ARM and SIM.
    # flow_phase_corr.cc compiles unchanged on both targets thanks to
    # sentai_fft_shim.h (CMSIS on ARM, FFTW3 wrapper on SIM).
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/flow_phase_corr.cc
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_fft_shim_sim.c
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/sentai_pxp_shim_sim.c
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/generated/visionmesh.pb.c
    ${CMAKE_SOURCE_DIR}/third_party/nanopb/pb_common.c
    ${CMAKE_SOURCE_DIR}/third_party/nanopb/pb_encode.c
    ${CMAKE_SOURCE_DIR}/third_party/nanopb/pb_decode.c
)
target_link_libraries(sentai_sim PRIVATE
    libfreertos_posix
    libmicropython_embed_sim
    libh3_sim      # ObjectsPlan L3 — H3 spatial index (sentai.places)
    fftw3f         # SIM-only: FFTW3 single-precision (CMSIS shim backend)
    m              # math functions — must come AFTER libmicropython_embed_sim
)
# Need MP headers + FreeRTOS in modsentai_sim.c, plus repo root for
# flow_phase_corr's "examples/sentai_runtime/flow_shared.h" include path.
target_include_directories(sentai_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}        # build_version.h, mpconfigport.h
    ${MP_EMBED_DIR}
    ${CMAKE_SOURCE_DIR}                # for "examples/sentai_runtime/*.h"
    # Refactor T1: bindings/modsentai_*.c includes "sentai_X.h" unqualified.
    # Used to work because the includer was at examples/sentai_runtime/; now
    # it lives at .../bindings/, so we add the parent dir to the search path.
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime
    ${CMAKE_SOURCE_DIR}/examples/sentai_runtime/generated
    ${CMAKE_SOURCE_DIR}/third_party/nanopb
    ${CMAKE_SOURCE_DIR}/third_party/mavlink  # Phase 6 — sentai_link_sim.cc
)
# Force build_version.h to refresh on every build.
add_dependencies(sentai_sim increment_sim_build_version)

# SIM-only feature gates.
#   SENTAI_PLATFORM_SIM    — selects FFTW3/POSIX paths in sentai_*_shim.h
#   SENTAI_SIM_FS_ROOT_*   — see modsentai_sim.c (virtual FS root)
target_compile_definitions(sentai_sim PRIVATE
    SENTAI_PLATFORM_SIM=1
    SENTAI_SIM_FS_ROOT_DEFAULT="${CMAKE_BINARY_DIR}/sentai_fs_root"
)
# modsentai_sim.c includes FreeRTOS.h for vTaskDelay; libfreertos_posix
# already exports the include path PUBLIC, so just linking gets us those.
