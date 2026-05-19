# Copyright 2022 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# --- Auto-increment build version ---
add_custom_target(increment_build_version ALL
    COMMAND ${CMAKE_COMMAND}
        -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/increment_version.cmake
    COMMENT "Incrementing build version..."
)

# --- Collect MicroPython embed sources ---
set(MP_EMBED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/micropython_embed)
file(GLOB MP_PY_SRCS ${MP_EMBED_DIR}/py/*.c)
file(GLOB MP_PORT_SRCS ${MP_EMBED_DIR}/port/*.c)
file(GLOB MP_SHARED_SRCS ${MP_EMBED_DIR}/shared/runtime/*.c)

# Build MicroPython as a static library so the linker script can place it in OCRAM
add_library(libmicropython STATIC
    ${MP_PY_SRCS}
    ${MP_PORT_SRCS}
    ${MP_SHARED_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/modsentai.c
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_port_import.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mp_embed_safe.c
)

target_include_directories(libmicropython PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                # mpconfigport.h
    ${MP_EMBED_DIR}                            # py/*.h, genhdr/
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_embed  # for #include "port/..."
    ${PROJECT_SOURCE_DIR}/third_party/nanopb   # pb.h (for sentai_mesh.h -> visionmesh.pb.h)
    ${CMAKE_CURRENT_SOURCE_DIR}/generated      # meshtastic/*.pb.h, visionmesh.pb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/h3_gen         # h3api.h pre-generated (matches QSTR pre-pass)
    ${PROJECT_SOURCE_DIR}/third_party/h3/src/h3lib/include  # H3 internals headers
)

# MicroPython is C99; compile with -Os; suppress warnings; must match CM7 target flags
target_compile_options(libmicropython PRIVATE
    -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
    -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
    -std=gnu99 -Os
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# FreeRTOS includes needed by micropython_task.c and modsentai_hal
target_include_directories(libmicropython PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third_party/modified/FreeRTOS
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/include
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM7/r0p1
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/CMSIS
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/devices/MIMXRT1176
)

# nanopb runtime library
add_library(nanopb STATIC
    ${PROJECT_SOURCE_DIR}/third_party/nanopb/pb_common.c
    ${PROJECT_SOURCE_DIR}/third_party/nanopb/pb_encode.c
    ${PROJECT_SOURCE_DIR}/third_party/nanopb/pb_decode.c
)
target_include_directories(nanopb PUBLIC
    ${PROJECT_SOURCE_DIR}/third_party/nanopb
)
# Silence nanopb warnings under arm-none-eabi-gcc; must match CM7 VFP ABI
target_compile_options(nanopb PRIVATE -w
    -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
    -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
)

# Shine fixed-point MP3 encoder (for sentai.mic.save_l3)
add_library(shine STATIC
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/bitstream.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/huffman.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/l3bitstream.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/l3loop.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/l3mdct.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/l3subband.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/layer3.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/reservoir.c
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib/tables.c
)
target_include_directories(shine PUBLIC
    ${PROJECT_SOURCE_DIR}/third_party/shine/src/lib
)
target_compile_options(shine PRIVATE -w
    -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
    -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
)

# Generated nanopb protobuf sources (meshtastic + visionmesh)
add_library(meshtastic_pb STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/mesh.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/portnums.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/channel.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/config.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/module_config.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/telemetry.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/xmodem.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/device_ui.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/meshtastic/atak.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/visionmesh.pb.c
)
target_include_directories(meshtastic_pb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/generated
    ${PROJECT_SOURCE_DIR}/third_party/nanopb
)
target_link_libraries(meshtastic_pb PUBLIC nanopb)
target_compile_options(meshtastic_pb PRIVATE -w
    -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
    -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
)

# ---------------------------------------------------------------------------
# libh3_arm — Uber H3 hexagonal spatial index, ARM build.
#
# ObjectsPlan L3 (sentai.places) prerequisite.  L1 vendored the submodule
# at third_party/h3 v4.4.1; L3 wires it into both SIM (sim/CMakeLists.txt)
# and ARM here.  Pure C99, 18 lib/*.c sources, no external deps.  The 4
# version macros in h3api.h.in are substituted via configure_file (mirror
# of the SIM-side recipe; same generated header layout).
#
# Per [[itcm-budget]]: H3 is general code, NOT a hot path — defaults to
# .text via the M7 flags and lands in SDRAM .text via the linker script's
# wildcard rules.  No .ramfunc placement.
# ---------------------------------------------------------------------------
set(H3_DIR ${PROJECT_SOURCE_DIR}/third_party/h3)
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
add_library_m7(libh3_arm STATIC
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
target_include_directories(libh3_arm PUBLIC
    ${H3_DIR}/src/h3lib/include
    ${CMAKE_CURRENT_BINARY_DIR}/h3_gen
)
target_compile_options(libh3_arm PRIVATE
    -std=gnu11
    -Os
    -w                          # H3 sources are clean upstream; silence ARM-host warnings
)

add_executable_m7(sentai_runtime
    LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/MIMXRT1176xxxxx_cm7_ram_mp.ld
    sentai_runtime.cc
    sentai_fault.cc
    sentai_tfl_bridge.cc
    sentai_slow_bridge.cc
    detection_task.cc
    flow_task.cc
    flow_phase_corr.cc
    aruco_bench.cc
    sentai_tracker.cc
    sentai_objects.cc
    sentai_places.cc
    sentai_phog.cc
    sentai_gist.cc
    # Track A piece 3/4 — HSV 16h×4s histogram (OP-S10-W4).
    sentai_hsv.cc
    # Camera-frame slot infrastructure (sentai_prep) — Phase 1a foundation.
    # Producer (PrepTask) populates slots; consumers (SlamTask, future
    # AnchorTask) read via zerocopy.  No PXP work here — pure storage.
    sentai_prep.cc
    # OP-S10-W11-T3 — InferTask-style perception loop: wakes on PrepTask
    # SLOT_RGB_64 signal, runs sentai_hsv_compute + sentai_places_query,
    # publishes match struct atomically.  MP sees only small scalars.
    slam_task.cc
    sentai_servo.cc
    sentai_object_lifter.cc
    # ObjectsPlan OP-S6-W1 — sentai.calib Kabsch.  ARM build enables
    # FxUser persistence via SENTAI_HAVE_FXUSER (set below); SIM build
    # falls back to host stdio in the cwd.  See Sim.md §2 rule 2 — we
    # feature-detect, never test for the ARM target directly.
    sentai_calib.cc
    # ObjectsPlan OP-S10-W14 — `sentai.calib` autotuner extension.
    # Adds a long-running C++ task that drives an Åström-Hägglund
    # relay on the velocity-feedback loop to learn Kp_flow without
    # manual tuning.  Pure-compute state machine + dedicated worker
    # mirror the sentai.safety split.  See objects_plan/16_*.
    sentai_calib_autotune.cc
    sentai_calib_task.cc
    # ObjectsPlan OP-S6-W3 — sentai.aruco fiducial detector + PnP.
    # Compute-heavy work (adaptive threshold, contour finder, PnP)
    # stays in C/C++; MP binding returns small scalars only.
    sentai_aruco.cc
    # ObjectsPlan OP-S10-W12 — sentai.safety state machine + SafetyTask
    # worker.  Pure state machine + FreeRTOS task consuming the
    # EXISTING sentai_aruco_detect + sentai_camera_grab_gray_zerocopy.
    # No detection / camera intake duplicated.  See Safety.md for the
    # architecture.  Routed to .sdram_text per [[itcm-budget]].
    sentai_safety.cc
    sentai_safety_task.cc
    # ObjectsPlan OP-S10-W13 — sentai.fr Flight Recorder.  Independent
    # NASA/JPL-style FDR subsystem; producers push items, worker task
    # writes to disk.  Split into state (sentai_fr.cc) + worker
    # (sentai_fr_task.cc), mirroring sentai.safety's split.  See
    # objects_plan/15_sentai_fr.md.
    sentai_fr.cc
    sentai_fr_task.cc
    sentai_explore.cc
    modsentai_hal.cc
    sentai_mesh.cc
    sentai_link.cc
    sentai_crazy.cc
    sentai_crazy_log.cc          # Task #44 — CRTP LOG (pose subscribe) in C
    sentai_httpd.cc
    sentai_fs_task.cc
    sentai_health.cc
    sentai_dmesg.cc
    sentai_fs_cache.cc
    m4_bench_host.cc           # OP-S10-W16-T3: M7 driver for the M4 ArUco bench
    M4_EXECUTABLE
    sentai_m4_aruco_bench
    DATA
)

# M4 build target removed 2026-05-05 (build #1130+).  Flow SAD now
# runs on M7 in flow_task.cc (M4 had unreliable SysTick + freeze
# under load).  flow_task_m4.cc is kept on disk as historical
# reference but no longer compiled.

# OP-S10-W16-T3 (2026-05-19): M4 ArUco-threshold cycle bench, built
# as a SEPARATE ELF that we can test in isolation (no link into
# sentai_runtime yet — that's T4+).  Bare-metal entry, no FreeRTOS
# scheduler, no SysTick — sidesteps the build #1130 freeze mode and
# gives a clean cycle count for the threshold kernel on M4.  Boot
# integration lands in a follow-up commit.
add_executable_m4(sentai_m4_aruco_bench
    m4_aruco_bench.cc
)
target_link_libraries(sentai_m4_aruco_bench
    libs_base-m4_freertos
)
# OP-S10-W16 fix 2026-05-19: the global M4 linker
# (libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm4_ram.ld) has been modified
# to mirror the sentai M7 rpmsg_sh_mem layout (0x2033E000, 8 KB).
# Other coralmicro multi-core examples are disabled in the build
# (examples/CMakeLists.txt:41-43), so the global change is safe for
# this repo's current scope.  See OP-S10-W16_multicore.md §4.

# Embed browser.html as a C byte array (avoids LFS system partition dependency)
set(BROWSER_HTML_SRC "${CMAKE_CURRENT_SOURCE_DIR}/web/browser.html")
set(BROWSER_HTML_HDR "${CMAKE_CURRENT_BINARY_DIR}/browser_html_data.h")
add_custom_command(
    OUTPUT "${BROWSER_HTML_HDR}"
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/web/gen_embed.py"
            "${BROWSER_HTML_SRC}" "${BROWSER_HTML_HDR}" "browser_html_data"
    DEPENDS "${BROWSER_HTML_SRC}" "${CMAKE_CURRENT_SOURCE_DIR}/web/gen_embed.py"
    COMMENT "Embedding browser.html as C array"
)
add_custom_target(sentai_browser_html DEPENDS "${BROWSER_HTML_HDR}")
add_dependencies(sentai_runtime sentai_browser_html)

# Embed help.txt directly in the SDRAM .help_data section (VMA=LMA, no ITCM overflow).
set(HELP_TXT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/help.txt")
set(HELP_TXT_CC  "${CMAKE_CURRENT_BINARY_DIR}/help_txt_data.cc")
add_custom_command(
    OUTPUT "${HELP_TXT_CC}"
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/gen_help_embed.py"
            "${HELP_TXT_SRC}" "${HELP_TXT_CC}"
    DEPENDS "${HELP_TXT_SRC}" "${CMAKE_CURRENT_SOURCE_DIR}/gen_help_embed.py"
    COMMENT "Embedding help.txt in .help_data SDRAM section"
)
target_sources(sentai_runtime PRIVATE "${HELP_TXT_CC}")

# Disable networking stack (DHCP, TCP/IP, Ethernet over USB) to save power
target_compile_definitions(sentai_runtime PRIVATE
    ENABLE_NETWORK_STACK=0
    ENABLE_USB_EEM=0
    ENABLE_RANDOM=0
    # Feature flag for sentai_calib.cc — pulls in libs/base/fx_user_fs.h
    # for /system/cam_calib.json persistence.  SIM build omits this and
    # falls back to host stdio (Sim.md §2 rule 2: feature-detect, never
    # branch on SENTAI_SIM directly).
    SENTAI_HAVE_FXUSER=1
)

target_link_libraries(sentai_runtime
    libs_base-m7_freertos
    libs_base-m7_http_server
    libs_jpeg_m7
    libs_lis2du12_freertos
    libs_t5838_freertos
    libs_audio_freertos
    shine
    libmicropython
    libs_aifes_task_m7
    libs_CMSIS-m7              # CMSIS-DSP arm_cfft_f32 (flow_phase_corr.cc)
    libh3_arm                  # ObjectsPlan L3 — H3 spatial index (sentai.places)
    nanopb
    meshtastic_pb
)

# Include dirs for MicroPython headers in main sources
target_include_directories(sentai_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                # mpconfigport.h, micropython_task.h
    ${CMAKE_CURRENT_BINARY_DIR}                # browser_html_data.h (generated)
    ${MP_EMBED_DIR}                            # py/*.h, genhdr/
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_embed  # for #include "port/..."
    ${PROJECT_SOURCE_DIR}/third_party/nanopb   # pb.h, pb_encode.h, pb_decode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/generated      # meshtastic/*.pb.h, visionmesh.pb.h
    ${PROJECT_SOURCE_DIR}/third_party/mavlink    # MAVLink c_library_v2 headers
)

target_compile_options(sentai_runtime PRIVATE -Os)

# Ensure version header is generated before compiling
add_dependencies(libmicropython increment_build_version)
add_dependencies(sentai_runtime increment_build_version)
