add_executable_m7(t5838_test
    t5838_test.cc
)

target_link_libraries(t5838_test
    libs_base-m7_freertos
    libs_t5838_freertos
)
