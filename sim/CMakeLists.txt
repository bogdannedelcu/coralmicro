# sim/CMakeLists.txt — SIM-only build targets.
#
# Only added to the build when SENTAI_SIM=ON.  See Sim.md and the top-level
# CMakeLists.txt for the toolchain wiring.

# Host pthread / librt for the FreeRTOS POSIX port.  Called here (not in the
# toolchain file) because find_package needs language support enabled by
# project() first.
find_package(Threads REQUIRED)

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
# sentai_sim — Phase 1 step 8-10 deliverable: REPL on stdin/stdout.
#
# Boots FreeRTOS POSIX scheduler, single REPL task drives MicroPython embed,
# user types `1+1\n`, sees `2\n`.  No SentAI hardware bindings yet
# (sentai.* module is not registered in Phase 1 — it would pull in too
# many NXP SDK dependencies).  Phase 2+ adds the bindings incrementally.
# ---------------------------------------------------------------------------
add_executable_sim(sentai_sim
    main_sim.c
)
target_link_libraries(sentai_sim PRIVATE
    libfreertos_posix
    libmicropython_embed_sim
    m              # math functions — must come AFTER libmicropython_embed_sim
)
# Need MP headers in main_sim.c too.
target_include_directories(sentai_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MP_EMBED_DIR}
)
