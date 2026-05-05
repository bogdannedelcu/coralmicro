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

set(libs_CMSIS_SOURCES
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1_x_n_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_3x3_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_wrapper_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_padded_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_1x_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_4x_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_svdf_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SVDFunctions/arm_svdf_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SoftmaxFunctions/arm_nn_softmax_common_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.c

    # Sources for s16 support
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_fast_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_kernel_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SVDFunctions/arm_svdf_state_s16_s8.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s16.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8_s16.c

    # CMSIS-DSP TransformFunctions: float32 complex FFT (lengths 16-4096).
    # Required by flow_phase_corr_task for 64-point and 128-point 2D FFTs
    # used in phase-correlation optical flow.  Twiddle tables live in
    # CommonTables.  No s16/q31/f64 variants -- we only use f32 path.
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f32.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_f32.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_f32.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Source/CommonTables/arm_const_structs.c
)

set(libs_CMSIS_INCLUDE_DIRECTORIES
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/Core/Include
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/DSP/Include
    ${PROJECT_SOURCE_DIR}/third_party/CMSIS/CMSIS/NN/Include
)

add_library_m7(libs_CMSIS-m7 STATIC
    ${libs_CMSIS_SOURCES}
)

target_include_directories(libs_CMSIS-m7 PUBLIC
    ${libs_CMSIS_INCLUDE_DIRECTORIES}
)

# Trim CMSIS-DSP twiddle/bitrev tables to ONLY what flow_phase_corr.cc
# uses (length-64 complex f32 FFT).  Default build includes ~900 KB of
# tables for every length and every numeric type -- way too large for
# our linker layout.  ARM_DSP_CONFIG_TABLES disables all by default;
# we then re-enable just the entries we reference.
target_compile_definitions(libs_CMSIS-m7 PRIVATE
    ARM_DSP_CONFIG_TABLES
    ARM_FFT_ALLOW_TABLES
    ARM_TABLE_TWIDDLECOEF_F32_64
    ARM_TABLE_BITREVIDX_FLT_64
)

add_library_m4(libs_CMSIS-m4 STATIC
    ${libs_CMSIS_SOURCES}
)

target_include_directories(libs_CMSIS-m4 PUBLIC
    ${libs_CMSIS_INCLUDE_DIRECTORIES}
)
