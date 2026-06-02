# B8 ARM-emulator spike targets. These are intentionally not production
# firmware targets; they prove ARM startup/FreeRTOS in an emulator before any
# real board peripheral is initialized.

set(SENTAI_EMU_MINIMAL_SOURCES
    sentai_emu_idle.cc
    sentai_emu_freertos_hooks.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/gcc/startup_MIMXRT1176_cm7.S
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/system_MIMXRT1176_cm7.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/event_groups.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/list.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM4F/port.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/queue.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/stream_buffer.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/tasks.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/timers.c
    ${PROJECT_SOURCE_DIR}/third_party/FreeRTOS_helpers/heap_useNewlib_NXP.c
)

function(configure_sentai_emu_target target)
  target_compile_definitions(${target} PRIVATE
      SENTAI_ARM_EMU=1
      CPU_MIMXRT1176CVM8A_cm7
      XIP_BOOT_HEADER_ENABLE=1
      XIP_EXTERNAL_FLASH=1
      SDK_DELAY_USE_DWT
      DISABLE_WDOG=1
      __STARTUP_CLEAR_BSS
      __STARTUP_INITIALIZE_NONCACHEDATA
      __USE_SHMEM
      USE_SDRAM
      SDK_OS_FREE_RTOS
      USE_RTOS=1
  )

  target_include_directories(${target} PRIVATE
      ${PROJECT_SOURCE_DIR}
      ${PROJECT_SOURCE_DIR}/third_party/modified/FreeRTOS
      ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk/rtos/freertos/freertos_kernel/include
      ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/include
      ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM4F
      ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/Core/Include
      ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176
      ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers
  )
endfunction()

add_executable_m7(sentai_emu_idle
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_MINIMAL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_idle)

add_executable_m7(sentai_emu_uart
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_MINIMAL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_uart)
target_compile_definitions(sentai_emu_uart PRIVATE SENTAI_EMU_UART_SMOKE=1)

# --- B8.3 MicroPython REPL on LPUART6 ---------------------------------------
#
# Reuses the embed sources already vendored under
# examples/sentai_runtime/micropython_embed/. Excludes the embed's default
# port/mphalport.c (printf-based) and substitutes emu/sentai_emu_mphalport.c
# which talks directly to LPUART6. The shared genhdr/moduledefs.h still
# references mp_module_sentai, so sentai_emu_stub_modules.c provides an empty
# module that satisfies the linker without dragging in any binding.

set(SENTAI_EMU_MP_EMBED_DIR
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/micropython_embed)

file(GLOB SENTAI_EMU_MP_PY_SRCS ${SENTAI_EMU_MP_EMBED_DIR}/py/*.c)
file(GLOB SENTAI_EMU_MP_SHARED_SRCS
    ${SENTAI_EMU_MP_EMBED_DIR}/shared/runtime/*.c)

set(SENTAI_EMU_REPL_SOURCES
    sentai_emu_repl.cc
    sentai_emu_freertos_hooks.c
    sentai_emu_mphalport.c
    sentai_emu_stub_modules.c
    ${SENTAI_EMU_MP_EMBED_DIR}/port/embed_util.c
    ${SENTAI_EMU_MP_PY_SRCS}
    ${SENTAI_EMU_MP_SHARED_SRCS}
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/gcc/startup_MIMXRT1176_cm7.S
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/system_MIMXRT1176_cm7.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/event_groups.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/list.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM4F/port.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/queue.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/stream_buffer.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/tasks.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/timers.c
    ${PROJECT_SOURCE_DIR}/third_party/FreeRTOS_helpers/heap_useNewlib_NXP.c
)

add_executable_m7(sentai_emu_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_repl)

# emu/mp_inc must appear BEFORE the production mpconfigport so the emu build
# uses its own minimal configuration.
target_include_directories(sentai_emu_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)

target_compile_options(sentai_emu_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.4 MicroPython mission import on top of B8.3 REPL --------------------
#
# Same binary shape as sentai_emu_repl plus emu/sentai_emu_fs.c, with the
# import machinery turned on in emu/mp_inc_mission/mpconfigport.h.  A separate
# CMake target is used (rather than reusing sentai_emu_repl) so the B8.3 proof
# binary stays byte-for-byte stable as gates accumulate.

set(SENTAI_EMU_MISSION_SOURCES
    ${SENTAI_EMU_REPL_SOURCES}
    sentai_emu_fs.c
)

add_executable_m7(sentai_emu_mission
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_MISSION_SOURCES}
)
configure_sentai_emu_target(sentai_emu_mission)
target_compile_definitions(sentai_emu_mission PRIVATE SENTAI_EMU_MISSION=1)

target_include_directories(sentai_emu_mission PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)

target_compile_options(sentai_emu_mission PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.5 ARM camera frame provider via NVIC IRQ ---------------------------
#
# Mirrors the SENTAI_EMU_MINIMAL_SOURCES set (no MicroPython needed for B8.5
# — the test is on ISR + scheduler fidelity at the camera-frame-ready
# boundary, not on Python).  Adds sentai_emu_camera.cc, which provides the
# strong Reserved110_IRQHandler override and the consumer FreeRTOS task.

set(SENTAI_EMU_CAMERA_SOURCES
    sentai_emu_camera.cc
    sentai_emu_freertos_hooks.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/gcc/startup_MIMXRT1176_cm7.S
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/system_MIMXRT1176_cm7.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/event_groups.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/list.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM4F/port.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/queue.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/stream_buffer.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/tasks.c
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/timers.c
    ${PROJECT_SOURCE_DIR}/third_party/FreeRTOS_helpers/heap_useNewlib_NXP.c
)

add_executable_m7(sentai_emu_camera
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_CAMERA_SOURCES}
)
configure_sentai_emu_target(sentai_emu_camera)
target_compile_options(sentai_emu_camera PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)
