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

add_library_m7(libs_usb_host_edgetpu_bm STATIC
    usb_host_edgetpu.c
    sentai_edgetpu_ep_log.c
)
target_link_libraries(libs_usb_host_edgetpu_bm
    libs_nxp_rt1176-sdk_bm
)

add_library_m7(libs_usb_host_edgetpu_freertos STATIC
    usb_host_edgetpu.c
    sentai_edgetpu_ep_log.c
)
target_link_libraries(libs_usb_host_edgetpu_freertos
    libs_nxp_rt1176-sdk_freertos
)

# sentai: SENTAI_TPU_MULTI_EP is declared at top-level CMakeLists.txt so it
# is visible to every target that includes libs/tpu/apex_firmware.h (not
# just this subtree).  Here we only pick which C file compiles the blob
# bytes into the DFU task library.
if(SENTAI_TPU_MULTI_EP)
    set(_SENTAI_TPU_APEX_BIN_C apex_latest_multi_ep_bin.c)
    message(STATUS "sentai: EdgeTPU firmware = multi_ep (experimental)")
else()
    set(_SENTAI_TPU_APEX_BIN_C apex_latest_single_ep_bin.c)
    message(STATUS "sentai: EdgeTPU firmware = single_ep (default)")
endif()
add_library_m7(libs_tpu_dfu_task_freertos STATIC
    ${_SENTAI_TPU_APEX_BIN_C}
    edgetpu_dfu_task.cc
)
target_link_libraries(libs_tpu_dfu_task_freertos
    libs_base-m7_freertos
)

add_library_m7(libs_tpu_task_freertos STATIC
    edgetpu_task.cc
)
target_link_libraries(libs_tpu_task_freertos
    libs_base-m7_freertos
    libs_tpu_freertos
    libs_usb_host_edgetpu_freertos
)

add_library_m7(libs_tpu_freertos STATIC
    edgetpu_executable.cc
    edgetpu_manager.cc
    edgetpu_op.cc
    edgetpu_driver.cc
)
target_link_libraries(libs_tpu_freertos
    libs_base-m7_freertos
    libs_usb_host_edgetpu_freertos
    libs_tpu_task_freertos
    libs_tensorflow-m7
    libs_FreeRTOS
    libs_flatbuffers
)
