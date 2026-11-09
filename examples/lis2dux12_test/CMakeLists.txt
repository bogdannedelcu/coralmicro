# LIS2DUX12 accelerometer test (board rev 1.1).
add_executable_m7(lis2dux12_test
    lis2dux12_test.cc
)

target_link_libraries(lis2dux12_test
    libs_base-m7_freertos
    libs_lis2dux12_freertos
)
