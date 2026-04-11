add_library_m7(libs_cdc_ecm_freertos STATIC
    cdc_ecm.cc
)
target_link_libraries(libs_cdc_ecm_freertos
    libs_base-m7_freertos
)
