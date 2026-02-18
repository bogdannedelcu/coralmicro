add_library_m7(libs_t5838_freertos STATIC
    t5838.cc
)
target_link_libraries(libs_t5838_freertos
    libs_nxp_rt1176-sdk_freertos
    libs_FreeRTOS
    libs_audio_freertos
    libs_pmic_freertos
)
