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

cmake_minimum_required(VERSION 3.18)

# Toolchain must be set before project() call.
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain-arm-none-eabi-gcc.cmake)
endif()

project(CoralMicroProject)

include(third_party/github/cristianadam/bundle_static_library.cmake)

if (NOT DEFINED CMAKE_BUILD_TYPE OR NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type" FORCE)
endif()

if (NOT DEFINED WIFI_SSID)
    set(WIFI_SSID "MyAccessPoint")
endif()
add_definitions(-DWIFI_SSID="${WIFI_SSID}")

# Leave as is for an open AP.
# If a password is specified, we try to connect as WPA2.
if (NOT DEFINED WIFI_PSK)
    set(WIFI_PSK "")
endif()
add_definitions(-DWIFI_PSK="${WIFI_PSK}")

# sentai: apply local patches to the coralmicro-rt1176-sdk submodule
# at configure time.  The patches live under patches/coralmicro-rt1176-sdk/
# and are applied by scripts/apply_sdk_patches.sh (idempotent: reruns
# are no-ops if a patch is already present).  This ensures the OV5640
# VGA/30 pclkPeriod fix + EHCI queue-depth bump are present in any
# build even after a `git submodule update` pulls upstream HEAD.
# Set SENTAI_SKIP_SDK_PATCHES=ON to bypass (for CI that wants a
# known-pristine SDK).
option(SENTAI_SKIP_SDK_PATCHES "Skip auto-applying patches in patches/coralmicro-rt1176-sdk/" OFF)
if(NOT SENTAI_SKIP_SDK_PATCHES)
    execute_process(
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/apply_sdk_patches.sh
        RESULT_VARIABLE _sentai_patch_rc
        OUTPUT_VARIABLE _sentai_patch_out
        ERROR_VARIABLE  _sentai_patch_out
    )
    message(STATUS "sentai sdk-patches:\n${_sentai_patch_out}")
    if(NOT _sentai_patch_rc EQUAL 0)
        message(FATAL_ERROR
            "sentai: apply_sdk_patches.sh failed (rc=${_sentai_patch_rc}). "
            "Inspect output above or rerun manually: "
            "bash scripts/apply_sdk_patches.sh")
    endif()
endif()

# sentai: EdgeTPU firmware blob selector.  Top-level so every target that
# includes libs/tpu/apex_firmware.h (including main_freertos_m7.cc which
# pulls it via edgetpu_dfu_task.h) sees the same SENTAI_TPU_MULTI_EP
# value.  Default 0 (single_ep); set to 1 at configure time with
# `cmake -DSENTAI_TPU_MULTI_EP=ON -S . -B build` to select the
# experimental multi-endpoint blob.
option(SENTAI_TPU_MULTI_EP "Use libedgetpu multi_ep EdgeTPU firmware blob (experimental)" OFF)
if(SENTAI_TPU_MULTI_EP)
    add_compile_definitions(SENTAI_TPU_MULTI_EP=1)
endif()

if (NOT DEFINED CORAL_MICRO_ARDUINO)
    set(CORAL_MICRO_ARDUINO 0)
endif()
add_definitions(-DCORAL_MICRO_ARDUINO=${CORAL_MICRO_ARDUINO})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(.)

if (${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
    add_subdirectory(apps)
    add_subdirectory(examples)
else()
    add_subdirectory(apps/elf_loader)
endif()

add_subdirectory(libs)
