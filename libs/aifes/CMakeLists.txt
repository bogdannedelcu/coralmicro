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

# AIfES - Artificial Intelligence for Embedded Systems
# https://github.com/Fraunhofer-IMS/AIfES_for_Arduino
# License: AGPL-3.0 (see third_party/aifes/LICENSE)

set(AIFES_DIR ${PROJECT_SOURCE_DIR}/third_party/aifes/src)

# Basic AIfES sources - FNN only for size (CNN can be added later when we have more space)
set(AIFES_BASIC_SOURCES
    # Base implementations
    ${AIFES_DIR}/basic/base/aialgo/aialgo_sequential_inference.c
    ${AIFES_DIR}/basic/base/aialgo/aialgo_sequential_training.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_dense.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_elu.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_input.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_leaky_relu.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_relu.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_sigmoid.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_softmax.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_softsign.c
    ${AIFES_DIR}/basic/base/ailayer/ailayer_tanh.c
    ${AIFES_DIR}/basic/base/ailoss/ailoss_crossentropy.c
    ${AIFES_DIR}/basic/base/ailoss/ailoss_mse.c
    ${AIFES_DIR}/basic/base/aimath/aimath_basic.c
    ${AIFES_DIR}/basic/base/aimath/aimath_f32.c
    ${AIFES_DIR}/basic/base/aiopti/aiopti_adam.c
    ${AIFES_DIR}/basic/base/aiopti/aiopti_sgd.c
    # Default implementations (F32 only)
    ${AIFES_DIR}/basic/default/ailayer/ailayer_dense_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_elu_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_input_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_leaky_relu_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_relu_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_sigmoid_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_softmax_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_softsign_default.c
    ${AIFES_DIR}/basic/default/ailayer/ailayer_tanh_default.c
    ${AIFES_DIR}/basic/default/ailoss/ailoss_crossentropy_default.c
    ${AIFES_DIR}/basic/default/ailoss/ailoss_mse_default.c
    ${AIFES_DIR}/basic/default/aimath/aimath_f32_default.c
    ${AIFES_DIR}/basic/default/aiopti/aiopti_adam_default.c
    ${AIFES_DIR}/basic/default/aiopti/aiopti_sgd_default.c
    # Express API (high-level) - F32 only
    ${AIFES_DIR}/basic/express/aifes_express_f32_fnn.c
)

# Build AIfES as static library for M7
add_library_m7(libs_aifes_m7 STATIC
    ${AIFES_BASIC_SOURCES}
)

# Aggressive size optimization
target_compile_options(libs_aifes_m7 PRIVATE -Os -ffunction-sections -fdata-sections)

target_include_directories(libs_aifes_m7 PUBLIC
    ${AIFES_DIR}
    ${AIFES_DIR}/core
    ${AIFES_DIR}/basic
    ${AIFES_DIR}/cnn
)

# AifesTask wrapper
add_library_m7(libs_aifes_task_m7 STATIC
    aifes_task.cc
)

target_compile_options(libs_aifes_task_m7 PRIVATE -Os -ffunction-sections -fdata-sections)

target_include_directories(libs_aifes_task_m7 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${AIFES_DIR}
)

target_link_libraries(libs_aifes_task_m7
    libs_aifes_m7
    libs_base-m7_freertos
)
