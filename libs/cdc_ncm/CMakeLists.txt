add_library_m7(libs_cdc_ncm_freertos STATIC
    cdc_ncm.cc
)
target_link_libraries(libs_cdc_ncm_freertos
    libs_base-m7_freertos
)
