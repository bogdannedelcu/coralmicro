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

# --- B8.6 ARM two-stage pipeline-topology spike behind VCam IRQ ----------
#
# Reuses the B8.5 VCam peripheral.  Adds a two-stage pipeline inside the
# firmware so the verdict can prove the camera-frame-ready boundary feeds a
# real downstream consumer, not just an ISR-side counter.

set(SENTAI_EMU_PIPELINE_SOURCES
    sentai_emu_pipeline.cc
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

add_executable_m7(sentai_emu_pipeline
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_PIPELINE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_pipeline)
target_compile_options(sentai_emu_pipeline PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.7 ARM fan-out spike: Stage1 -> {Stage2A, Stage2B} with seqlock ----
#
# Two parallel consumers reading the same scalar slot.  Exercises the
# seqlock writer / reader primitive that production uses for multi-reader
# fan-out around the prep slot.  Stand-in stage names per B8.6 rule.

set(SENTAI_EMU_FANOUT_SOURCES
    sentai_emu_fanout.cc
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

add_executable_m7(sentai_emu_fanout
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FANOUT_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fanout)
target_compile_options(sentai_emu_fanout PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.7c ARM Flow-offset detection from host-fed cat scenes -------------
#
# Brute-force SAD block-match estimator over 32x32 grayscale frames.  The
# Renode .resc injects each scene from emu/output/scenes/scene_*.bin into
# the guest g_frame_buffer between IRQs, so the firmware never has to
# carry the image bytes.

set(SENTAI_EMU_FLOWEST_SOURCES
    sentai_emu_flowest.cc
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

add_executable_m7(sentai_emu_flowest
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FLOWEST_SOURCES}
)
configure_sentai_emu_target(sentai_emu_flowest)
target_compile_options(sentai_emu_flowest PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.8 Phase 1 USB host stack probe ------------------------------------
#
# Links the production libs/usb/usb_host_task.cc plus the minimum NXP USB
# middleware + clock_config files into an emu image so we can SEE how far
# the production stack compiles and links against the emu profile.  No
# Renode-side USB model is required to build; running comes in Phase 2.

set(SENTAI_EMU_TPU_PROBE_SOURCES
    sentai_emu_usbhost_probe.cc
    sentai_emu_freertos_hooks.c
    ${PROJECT_SOURCE_DIR}/libs/usb/usb_host_task.cc
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/clock_config.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_hci.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_ehci.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_devices.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_framework.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/class/usb_host_hub.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/class/usb_host_hub_app.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/phy/usb_phy.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/components/osa/fsl_os_abstraction_free_rtos.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/components/lists/fsl_component_generic_list.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_clock.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_anatop_ai.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_common.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_common_arm.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_pmu.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_dcdc.c
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

add_executable_m7(sentai_emu_usbhost_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_usbhost_probe)
target_compile_definitions(sentai_emu_usbhost_probe PRIVATE
    FSL_RTOS_FREE_RTOS
)
target_include_directories(sentai_emu_usbhost_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/host/class
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/include
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/phy
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/middleware/usb/output/source/host
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/utilities/debug_console
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/utilities/str
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/components/osa
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/components/lists
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/components/serial_manager
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7
)
target_compile_options(sentai_emu_usbhost_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)
