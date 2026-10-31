# LIS2DUX12 accelerometer (board rev 2026-10).  ST platform-independent
# driver vendored under third_party/st/lis2dux12-pid (BSD-3-Clause).
add_library_m7(libs_lis2dux12_freertos STATIC
    lis2dux12.cc
    ${PROJECT_SOURCE_DIR}/third_party/st/lis2dux12-pid/lis2dux12_reg.c
)

target_include_directories(libs_lis2dux12_freertos PUBLIC
    ${PROJECT_SOURCE_DIR}/third_party/st/lis2dux12-pid
)

target_link_libraries(libs_lis2dux12_freertos
    libs_base-m7_freertos
    libs_nxp_rt1176-sdk_freertos
    libs_FreeRTOS
)
