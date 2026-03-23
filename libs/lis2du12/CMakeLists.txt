add_library_m7(libs_lis2du12_freertos STATIC
    lis2du12.cc
    ${PROJECT_SOURCE_DIR}/third_party/st/lis2du12-pid/lis2du12_reg.c
)

target_include_directories(libs_lis2du12_freertos PUBLIC
    ${PROJECT_SOURCE_DIR}/third_party/st/lis2du12-pid
)

target_compile_definitions(libs_lis2du12_freertos PUBLIC
    # LIS2DU12_DEBUG
)

target_link_libraries(libs_lis2du12_freertos
    libs_base-m7_freertos
    libs_nxp_rt1176-sdk_freertos
    libs_FreeRTOS
)
