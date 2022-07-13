/*
 * Copyright 2022 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "third_party/nxp/rt1176-sdk/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/43455/resources.h"

const resource_hnd_t wifi_firmware_image = {
    .location = RESOURCE_IN_EXTERNAL_STORAGE,
    .size = 580041,
    .val =
        {
            .fs =
                {
                    .filename =
                        "/third_party/firmware/cypress/43455C0-mfgtest.bin",
                    .offset = 0,
                },
        },
};

const resource_hnd_t wifi_firmware_clm_blob = {
    .location = RESOURCE_IN_EXTERNAL_STORAGE,
    .size = 5290,
    .val =
        {
            .fs =
                {
                    .filename = "/third_party/firmware/cypress/"
                                "43455C0-mfgtest.clm_blob",
                    .offset = 0,
                },
        },
};
