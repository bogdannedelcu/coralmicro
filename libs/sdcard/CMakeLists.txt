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

add_library_m7(libs_sdcard_freertos STATIC
    sdcard.cc
)

# Block-level only.  libs_base-m7_freertos links THIS library (the user FS
# backend lives in libs/base/fx_sd_driver.cc), so no dependency back on
# libs_base here, and no FatFs: the filesystem on the card is FileX.
target_link_libraries(libs_sdcard_freertos
    libs_nxp_rt1176-sdk_freertos
    libs_nxp_rt1176-sdk_sdmmc
    libs_nxp_rt1176-sdk_middleware_sdmmc_sd
)
