# Copyright 2026 Bogdan Nedelcu / SentAI
#
# Static library for LevelX (NAND/NOR wear-leveling layer) — Eclipse ThreadX
# project, MIT licensed.  Built in STANDALONE mode (no ThreadX kernel needed)
# so it co-exists with the existing FreeRTOS task model.
#
# All sources are compiled; -ffunction-sections / -Wl,--gc-sections in the
# top-level toolchain prune anything that the firmware does not actually
# call (e.g. all NOR-flash code paths when only NAND is used).

set(LEVELX_ROOT ${PROJECT_SOURCE_DIR}/third_party/eclipse-threadx/levelx)

file(GLOB LEVELX_SOURCES ${LEVELX_ROOT}/common/src/lx_*.c)

# Drop sources we explicitly do not want pulled in:
#  * lx_nand_flash_simulator.c — RAM-backed simulator demo, not for embedded.
#  * fx_nand_flash_simulated_driver.c, fx_nor_flash_simulator_driver.c —
#    bridge layers for the simulators above.
list(FILTER LEVELX_SOURCES EXCLUDE REGEX ".*lx_nand_flash_simulator\\.c$")
list(FILTER LEVELX_SOURCES EXCLUDE REGEX ".*fx_nand_flash_simulated_driver\\.c$")
list(FILTER LEVELX_SOURCES EXCLUDE REGEX ".*fx_nor_flash_simulator_driver\\.c$")
list(FILTER LEVELX_SOURCES EXCLUDE REGEX ".*lx_nor_flash_simulator\\.c$")

add_library_m7(libs_levelx STATIC
    ${LEVELX_SOURCES}
)

target_include_directories(libs_levelx PUBLIC
    ${LEVELX_ROOT}/common/inc
    ${PROJECT_SOURCE_DIR}/libs/levelx
)

target_compile_definitions(libs_levelx PUBLIC
    LX_STANDALONE_ENABLE
    LX_INCLUDE_USER_DEFINE_FILE
)

# FileX/LevelX upstream code uses many K&R-style return-type warnings; mute
# them for the third-party compile only.  Our own glue stays under -Wall.
target_compile_options(libs_levelx PRIVATE
    -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-implicit-fallthrough
    -Wno-sign-compare
    -Wno-strict-aliasing
)
