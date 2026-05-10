# Toolchain for the x86_64 Linux SIM build of SentAI.
# Counterpart to cmake/toolchain-arm-none-eabi-gcc.cmake.
#
# Usage:
#   cmake -B build-sim -S . -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake -DSENTAI_SIM=ON
#   cmake --build build-sim --target sentai_sim_kernel_only
#
# See Sim.md for the broader plan.

# Use host compilers (no cross-compile).
# Allow override via standard CMake env vars (CC, CXX) or -D flags.
if(NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER gcc)
endif()
if(NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER g++)
endif()

# We're targeting the host system (Linux x86_64) — DON'T set CMAKE_SYSTEM_NAME
# to "Generic" like the ARM toolchain does.  We want full host-libc support.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# NOTE: find_package(Threads) MUST be called from a CMakeLists.txt after
# project() is evaluated — not here in the toolchain file (which runs
# before language detection).  See sim/CMakeLists.txt where it's called.

# ---------------------------------------------------------------------------
# add_executable_sim / add_library_sim
#
# Mirror of add_executable_m7 / add_library_m7 from the ARM toolchain, but for
# x86 host build.  Targets get linked against pthread + rt + m by default.
# ---------------------------------------------------------------------------

function(add_executable_sim TARGET_NAME)
    add_executable(${TARGET_NAME} ${ARGN})
    target_link_libraries(${TARGET_NAME} PRIVATE
        Threads::Threads
        rt
        m
    )
    # Sanitizer-friendly defaults; can be overridden per-target.
    target_compile_options(${TARGET_NAME} PRIVATE
        -Wall -Wextra -Wno-unused-parameter -Wno-unused-function
        -fno-strict-aliasing
    )
endfunction()

function(add_library_sim TARGET_NAME)
    # First arg may be STATIC/SHARED/INTERFACE, mirror standard add_library.
    add_library(${TARGET_NAME} ${ARGN})
    target_compile_options(${TARGET_NAME} PRIVATE
        -Wall -Wextra -Wno-unused-parameter -Wno-unused-function
        -fno-strict-aliasing
    )
endfunction()
