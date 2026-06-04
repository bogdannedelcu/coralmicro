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

# --- S214/B8 production FlowTask benchmark ---------------------------------
#
# Unlike sentai_emu_flowest, this target links the runtime FlowTask and
# phase-correlation matcher.  The emu harness feeds the production prep
# FLOW_GRAY_80x60 slot directly so we can measure FlowTask in isolation before
# reintroducing VirtualCameraTask/PrepTask/InferTask.

set(SENTAI_EMU_FLOW_TASK_RUNTIME_SOURCES
    sentai_emu_flow_task_runtime.cc
    sentai_emu_freertos_hooks.c
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/flow_task.cc
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/flow_phase_corr.cc
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/sentai_prep.cc
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

add_executable_m7(sentai_emu_flow_task_runtime
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FLOW_TASK_RUNTIME_SOURCES}
)
configure_sentai_emu_target(sentai_emu_flow_task_runtime)
target_include_directories(sentai_emu_flow_task_runtime PRIVATE
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Include
)
target_compile_definitions(sentai_emu_flow_task_runtime PRIVATE
    ARM_DSP_CONFIG_TABLES
    ARM_FFT_ALLOW_TABLES
    ARM_TABLE_TWIDDLECOEF_F32_64
    ARM_TABLE_BITREVIDX_FLT_64
)
target_compile_options(sentai_emu_flow_task_runtime PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)
target_link_libraries(sentai_emu_flow_task_runtime
    libs_CMSIS-m7
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
    sentai_emu_usb_stubs.c
    sentai_emu_usb_cpp_stubs.cc
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
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
)
target_compile_options(sentai_emu_usbhost_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 2: same production USB host stack probe, but also calls
# UsbHostTask::Init() and starts the ARM FreeRTOS scheduler.  A low-priority
# heartbeat task verifies that the production USB host task blocks in
# OSA_EventWait instead of monopolizing the scheduler when no USB model/device
# is present.
add_executable_m7(sentai_emu_usbhost_task
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_usbhost_task)
target_compile_definitions(sentai_emu_usbhost_task PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
)
target_include_directories(sentai_emu_usbhost_task PRIVATE
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
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
)
target_compile_options(sentai_emu_usbhost_task PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 3a: add the production EdgeTpuManager singleton above the P2
# USB-host-task scheduler smoke.  This still does not call OpenDevice() or
# start a Coral enumeration path; it proves the manager layer itself links and
# can be constructed in the emulator profile.
add_executable_m7(sentai_emu_edgetpu_manager_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
)
configure_sentai_emu_target(sentai_emu_edgetpu_manager_probe)
target_compile_definitions(sentai_emu_edgetpu_manager_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
)
target_include_directories(sentai_emu_edgetpu_manager_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_manager_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 3b: start the production EdgeTpuTask as a QueueTask and let it
# register the Coral VID/PID callback with the production UsbHostTask.  No USB
# device is attached yet; success means the task init path does not deadlock
# the ARM scheduler and the heartbeat still runs.
add_executable_m7(sentai_emu_edgetpu_task_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_task_probe)
target_compile_definitions(sentai_emu_edgetpu_task_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
)
target_include_directories(sentai_emu_edgetpu_task_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_task_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 4a: call EdgeTpuManager::OpenDevice() from a FreeRTOS task after
# the scheduler starts, with the manager pre-marked in an error/no-device
# state.  This proves the high-level OpenDevice error path returns instead of
# wedging the ARM scheduler while real USB attach modeling is still absent.
add_executable_m7(sentai_emu_edgetpu_opendevice_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_opendevice_probe)
target_compile_definitions(sentai_emu_edgetpu_opendevice_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
    SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE=1
)
target_include_directories(sentai_emu_edgetpu_opendevice_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_opendevice_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 4b: synthesize a Coral attach + enumeration-done event from an
# emu FreeRTOS task, using NXP usb_host_* descriptor structs and the real
# UsbHostTask -> EdgeTpuTask callback path.  This is not an EHCI model yet; it
# proves the registered callback chain and minimal EdgeTPU descriptors.
add_executable_m7(sentai_emu_edgetpu_synth_enum_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_synth_enum_probe)
target_compile_definitions(sentai_emu_edgetpu_synth_enum_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
    SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE=1
)
target_include_directories(sentai_emu_edgetpu_synth_enum_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_synth_enum_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 5a: Renode-side Coral MMIO provider.  The device descriptor is
# now exposed by emu/renode/sentai_rt1176.repl (`coral_usb`) and delivered via
# IRQ 95; firmware consumes that provider and routes through the real
# UsbHostTask -> EdgeTpuTask callback chain.
add_executable_m7(sentai_emu_edgetpu_mmio_enum_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_mmio_enum_probe)
target_compile_definitions(sentai_emu_edgetpu_mmio_enum_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE=1
)
target_include_directories(sentai_emu_edgetpu_mmio_enum_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_mmio_enum_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 5b: after the Renode-side MMIO descriptor block is consumed and
# routed through the real UsbHostTask -> EdgeTpuTask callback chain, call the
# production EdgeTpuManager::OpenDevice() from a FreeRTOS task.  The emu-scoped
# TpuDriver stub returns success so this gate proves manager/task sequencing,
# not real TPU transfers.
add_executable_m7(sentai_emu_edgetpu_mmio_opendevice_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_mmio_opendevice_probe)
target_compile_definitions(sentai_emu_edgetpu_mmio_opendevice_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
    SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_OPENDEVICE_PROBE=1
    SENTAI_EMU_TPU_DRIVER_INIT_OK=1
)
target_include_directories(sentai_emu_edgetpu_mmio_opendevice_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_mmio_opendevice_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.8 Phase 6a: keep the emulator injection exactly at the production
# TpuDriver transfer boundary.  After P5b OpenDevice succeeds, firmware calls
# SendParameters/SendInputs/SendInstructions/GetOutputs/ReadEvent.  Renode's
# `coral_tpu_bridge` MMIO peripheral acks those calls and writes a deterministic
# output pattern into guest memory.  This is the transport seam for the later
# physical host-Coral bridge.
add_executable_m7(sentai_emu_edgetpu_mmio_send_bridge_probe
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PROBE_SOURCES}
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_task.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/usb_host_edgetpu.c
)
configure_sentai_emu_target(sentai_emu_edgetpu_mmio_send_bridge_probe)
target_compile_definitions(sentai_emu_edgetpu_mmio_send_bridge_probe PRIVATE
    FSL_RTOS_FREE_RTOS
    SENTAI_PLATFORM_SIM
    SENTAI_EMU_USBHOST_PROBE_START_TASK=1
    SENTAI_EMU_EDGETPU_MANAGER_PROBE=1
    SENTAI_EMU_EDGETPU_TASK_PROBE=1
    SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_OPENDEVICE_PROBE=1
    SENTAI_EMU_EDGETPU_MMIO_SEND_BRIDGE_PROBE=1
    SENTAI_EMU_TPU_DRIVER_MMIO_SEND_BRIDGE=1
)
target_include_directories(sentai_emu_edgetpu_mmio_send_bridge_probe PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_compile_options(sentai_emu_edgetpu_mmio_send_bridge_probe PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.9 ARM FileX/LevelX user storage over emulated raw NAND ----------
#
# This is not a RAM filesystem.  The guest firmware runs the production
# FxUser* stack and fx_nand_driver callbacks.  Only the physical NAND
# read/program/erase operations are redirected to Renode's `nand_bridge`
# host-backed raw NAND image.

set(SENTAI_EMU_FX_STORAGE_SOURCES
    smoke/sentai_emu_fx_storage_smoke.cc
    sentai_emu_freertos_hooks.c
    sentai_emu_lfs_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_user_fs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_nand_driver.cc
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

add_executable_m7(sentai_emu_fx_storage
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FX_STORAGE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fx_storage)
target_compile_definitions(sentai_emu_fx_storage PRIVATE
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_link_libraries(sentai_emu_fx_storage
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_fx_storage PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.9c: stage host-side experiment assets into the same raw NAND image
# before booting a REPL/inference target.  The host bridge only supplies
# bytes; all file creation/truncation/appending is done by FxUser* in guest.
set(SENTAI_EMU_FS_STAGE_ASSETS_SOURCES
    sentai_emu_fs_stage_assets.cc
    sentai_emu_freertos_hooks.c
    sentai_emu_lfs_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_user_fs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_nand_driver.cc
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

add_executable_m7(sentai_emu_fs_stage_assets
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FS_STAGE_ASSETS_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fs_stage_assets)
target_compile_definitions(sentai_emu_fs_stage_assets PRIVATE
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_link_libraries(sentai_emu_fs_stage_assets
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_fs_stage_assets PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.9b MicroPython sentai.fs over FileX/LevelX raw NAND -------------
#
# Same VM/REPL shape as B8.3, but the `sentai` module exposes `sentai.fs`
# directly over FxUser*.  This validates the path we need for uploading and
# loading models/images in the ARM-emulator runtime without inventing a RAM FS.

set(SENTAI_EMU_FS_REPL_SOURCES ${SENTAI_EMU_REPL_SOURCES})
list(REMOVE_ITEM SENTAI_EMU_FS_REPL_SOURCES sentai_emu_stub_modules.c)
list(APPEND SENTAI_EMU_FS_REPL_SOURCES
    sentai_emu_fs_module.c
    sentai_emu_lfs_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_user_fs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_nand_driver.cc
)

add_executable_m7(sentai_emu_fs_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FS_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fs_repl)
target_compile_definitions(sentai_emu_fs_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_fs_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_fs_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_fs_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_fs_repl_smoke
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FS_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fs_repl_smoke)
target_compile_definitions(sentai_emu_fs_repl_smoke PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_FS_REPL_SMOKE=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_fs_repl_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_fs_repl_smoke
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_fs_repl_smoke PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_fs_asset_check
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FS_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_fs_asset_check)
target_compile_definitions(sentai_emu_fs_asset_check PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_FS_ASSET_CHECK=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_fs_asset_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_fs_asset_check
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_fs_asset_check PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# --- B8.10 MicroPython mission from FileX + physical Coral host bridge ---
#
# This target boots the staged NAND image, imports /mission.py from FxUser, and
# exposes emu-scoped sentai.tpu/pipeline modules.  sentai.tpu.load/load_image
# stream the guest FS bytes to Renode; invoke delegates to the host C++ Coral
# smoke that talks to the physical USB TPU.
set(SENTAI_EMU_TPU_CAT_REPL_SOURCES ${SENTAI_EMU_FS_REPL_SOURCES})
list(APPEND SENTAI_EMU_TPU_CAT_REPL_SOURCES
    sentai_emu_fx_import.c
    sentai_emu_tpu_bridge_module.c
)

add_executable_m7(sentai_emu_tpu_cat_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_cat_repl)
target_compile_definitions(sentai_emu_tpu_cat_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_CAT_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_cat_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_cat_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_cat_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_fps_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_fps_repl)
target_compile_definitions(sentai_emu_tpu_fps_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_FPS_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_fps_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_fps_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_fps_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_fps_mem_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_fps_mem_repl)
target_compile_definitions(sentai_emu_tpu_fps_mem_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_FPS_MEM_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_fps_mem_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_fps_mem_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_fps_mem_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_fps_mem_invoke_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_fps_mem_invoke_repl)
target_compile_definitions(sentai_emu_tpu_fps_mem_invoke_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_FPS_MEM_INVOKE_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_fps_mem_invoke_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_fps_mem_invoke_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_fps_mem_invoke_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_fps_mem_session_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_fps_mem_session_repl)
target_compile_definitions(sentai_emu_tpu_fps_mem_session_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_FPS_MEM_SESSION_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_fps_mem_session_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_fps_mem_session_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_fps_mem_session_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_timing_repl
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_CAT_REPL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_timing_repl)
target_compile_definitions(sentai_emu_tpu_timing_repl PRIVATE
    SENTAI_EMU_FX_FS=1
    SENTAI_EMU_MISSION=1
    SENTAI_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_TIMING_AUTORUN=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
)
target_include_directories(sentai_emu_tpu_timing_repl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_inc_mission
    ${SENTAI_EMU_MP_EMBED_DIR}
    ${SENTAI_EMU_MP_EMBED_DIR}/port
)
target_link_libraries(sentai_emu_tpu_timing_repl
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_timing_repl PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# B8.11 low-level physical Coral bridge smoke.  This is the next step below
# B8.10's high-level sentai.tpu host-invoke bridge: the guest owns
# EdgeTpuManager/EdgeTpuExecutable and Renode forwards only the real
# TpuDriver SendParameters/SendInputs/SendInstructions/GetOutputs/ReadEvent
# calls to a host POSIX/libusb server.
set(SENTAI_EMU_TPU_PHYSICAL_SEND_SMOKE_SOURCES
    smoke/sentai_emu_tpu_physical_send_smoke.cc
    sentai_emu_freertos_hooks.c
    sentai_emu_lfs_stubs.cc
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_user_fs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_nand_driver.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_executable.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/c/common.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/kernels/kernel_util.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/memory_helpers.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/micro_utils.cc
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

add_executable_m7(sentai_emu_tpu_physical_send_smoke
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PHYSICAL_SEND_SMOKE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_physical_send_smoke)
target_compile_definitions(sentai_emu_tpu_physical_send_smoke PRIVATE
    SENTAI_PLATFORM_SIM=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE=1
    FLATBUFFERS_LOCALE_INDEPENDENT=0
)
target_include_directories(sentai_emu_tpu_physical_send_smoke PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/gemmlowp
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_link_libraries(sentai_emu_tpu_physical_send_smoke
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_physical_send_smoke PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

add_executable_m7(sentai_emu_tpu_physical_send_fps
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_TPU_PHYSICAL_SEND_SMOKE_SOURCES}
)
configure_sentai_emu_target(sentai_emu_tpu_physical_send_fps)
target_compile_definitions(sentai_emu_tpu_physical_send_fps PRIVATE
    SENTAI_PLATFORM_SIM=1
    SENTAI_ARM_EMU_NAND_BRIDGE=1
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE=1
    SENTAI_EMU_TPU_PHYSICAL_SEND_RUNS=10
    FLATBUFFERS_LOCALE_INDEPENDENT=0
)
target_include_directories(sentai_emu_tpu_physical_send_fps PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/gemmlowp
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
target_link_libraries(sentai_emu_tpu_physical_send_fps
    libs_filex
    libs_levelx
)
target_compile_options(sentai_emu_tpu_physical_send_fps PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# S215/B8: run production FlowTask in parallel with the guest-owned
# EdgeTpuManager -> TpuDriver::Send* -> physical USB Coral path.  This target
# intentionally does NOT define SENTAI_PLATFORM_SIM so FlowTask keeps the ARM
# phase-correlation/CMSIS algorithm.
set(SENTAI_EMU_FLOW_TPU_PARALLEL_SOURCES
    sentai_emu_flow_tpu_parallel.cc
    sentai_emu_freertos_hooks.c
    sentai_emu_lfs_stubs.cc
    sentai_emu_tpu_driver_stubs.cc
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/flow_task.cc
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/flow_phase_corr.cc
    ${PROJECT_SOURCE_DIR}/examples/sentai_runtime/sentai_prep.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_user_fs.cc
    ${PROJECT_SOURCE_DIR}/libs/base/fx_nand_driver.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_executable.cc
    ${PROJECT_SOURCE_DIR}/libs/tpu/edgetpu_manager.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/c/common.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/kernels/kernel_util.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/memory_helpers.cc
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro/tensorflow/lite/micro/micro_utils.cc
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

add_executable_m7(sentai_emu_flow_tpu_parallel
    LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld
    ${SENTAI_EMU_FLOW_TPU_PARALLEL_SOURCES}
)
configure_sentai_emu_target(sentai_emu_flow_tpu_parallel)
target_compile_definitions(sentai_emu_flow_tpu_parallel PRIVATE
    SENTAI_ARM_EMU_NAND_BRIDGE=1
    SENTAI_ARM_EMU_TPU_HOST_BRIDGE=1
    SENTAI_EMU_TPU_DRIVER_PHYSICAL_SEND_BRIDGE=1
    SENTAI_EMU_TPU_PHYSICAL_SEND_RUNS=5
    FLATBUFFERS_LOCALE_INDEPENDENT=0
    ARM_DSP_CONFIG_TABLES
    ARM_FFT_ALLOW_TABLES
    ARM_TABLE_TWIDDLECOEF_F32_64
    ARM_TABLE_BITREVIDX_FLT_64
)
target_include_directories(sentai_emu_flow_tpu_parallel PRIVATE
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/modified/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/third_party/flatbuffers/include
    ${PROJECT_SOURCE_DIR}/third_party/gemmlowp
    ${PROJECT_SOURCE_DIR}/third_party/tflite-micro
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
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Include
)
target_link_libraries(sentai_emu_flow_tpu_parallel
    libs_filex
    libs_levelx
    libs_CMSIS-m7
)
target_compile_options(sentai_emu_flow_tpu_parallel PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)
