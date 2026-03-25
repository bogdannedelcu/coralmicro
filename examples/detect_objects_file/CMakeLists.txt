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
    ${CMAKE_CURRENT_SOURCE_DIR}/modcoral.c
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_task.c
)

target_include_directories(libmicropython PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                # mpconfigport.h
    ${MP_EMBED_DIR}                            # py/*.h, genhdr/
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_embed  # for #include "port/..."
)

# MicroPython is C99; compile with -Os; suppress warnings; must match CM7 target flags
target_compile_options(libmicropython PRIVATE
    -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
    -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin
    -std=gnu99 -Os
    -Wno-error -Wno-unused-parameter -Wno-sign-compare
    -Wno-old-style-definition -Wno-missing-prototypes
)

# FreeRTOS includes needed by micropython_task.c and modcoral_hal
target_include_directories(libmicropython PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third_party/modified/FreeRTOS
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/include
    ${PROJECT_SOURCE_DIR}/third_party/freertos_kernel/portable/GCC/ARM_CM7/r0p1
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/CMSIS
    ${PROJECT_SOURCE_DIR}/libs/nxp/rt1176-sdk/devices/MIMXRT1176
)

add_executable_m7(detect_objects_file
    LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/MIMXRT1176xxxxx_cm7_ram_mp.ld
    detect_objects_file.cc
    modcoral_hal.cc
    DATA
    #cat_300x300.rgb
    test_512x512.rgb
    #test_1024x1024.rgb
    ${PROJECT_SOURCE_DIR}/models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite
    #${PROJECT_SOURCE_DIR}/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite
)

target_link_libraries(detect_objects_file
    libs_base-m7_freertos
    libs_jpeg_m7
    libmicropython
)

# Include dirs for MicroPython headers in main sources
target_include_directories(detect_objects_file PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                # mpconfigport.h, micropython_task.h
    ${MP_EMBED_DIR}                            # py/*.h, genhdr/
    ${CMAKE_CURRENT_SOURCE_DIR}/micropython_embed  # for #include "port/..."
)

target_compile_options(detect_objects_file PRIVATE -O2)
