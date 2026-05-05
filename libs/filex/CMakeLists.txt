# Copyright 2026 Bogdan Nedelcu / SentAI
#
# Static library for FileX (Eclipse ThreadX project, MIT licensed).
# Built in STANDALONE mode (no ThreadX kernel needed) so it co-exists with
# the existing FreeRTOS task model.

set(FILEX_ROOT ${PROJECT_SOURCE_DIR}/third_party/eclipse-threadx/filex)

# Use ALL FileX common sources except simulator/demo bridges.  Unused
# sources are stripped at link time by --gc-sections.
file(GLOB FILEX_SOURCES
    ${FILEX_ROOT}/common/src/fx_*.c
    ${FILEX_ROOT}/common/src/fxe_*.c
)

# Drop simulator-only sources (RAM-backed demo NAND/NOR drivers).
list(FILTER FILEX_SOURCES EXCLUDE REGEX ".*fx_nand_flash_simulated_driver\\.c$")
list(FILTER FILEX_SOURCES EXCLUDE REGEX ".*fx_nor_flash_simulator_driver\\.c$")

add_library_m7(libs_filex STATIC
    ${FILEX_SOURCES}
)

target_include_directories(libs_filex PUBLIC
    ${FILEX_ROOT}/common/inc
    ${FILEX_ROOT}/ports/cortex_m7/gnu/inc
    ${PROJECT_SOURCE_DIR}/libs/filex
)

target_compile_definitions(libs_filex PUBLIC
    FX_STANDALONE_ENABLE
    FX_INCLUDE_USER_DEFINE_FILE
)

target_compile_options(libs_filex PRIVATE
    -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-implicit-fallthrough
    -Wno-sign-compare
    -Wno-strict-aliasing
    -Wno-pointer-sign
)
