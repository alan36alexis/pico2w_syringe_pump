# tools/fsm_test/CMakeLists.txt
# Host-only FSM golden table test — does NOT require the Pico SDK or ARM toolchain.
#
# Usage (from project root, requires a native C compiler in PATH):
#   mkdir build_host && cd build_host
#   cmake .. -DCMAKE_BUILD_TYPE=Debug -DFSM_TEST_HOST=ON
#   make fsm_golden_test
#   ./tools/fsm_test/fsm_golden_test
#
# Or compile directly without CMake (Linux/macOS/WSL/Git Bash):
#   cd tools/fsm_test
#   gcc -Wall -Wextra -o test_golden test_fsm_golden.c && ./test_golden
#
# The target is excluded from the default build so it never interferes
# with the Pico cross-compilation.

cmake_minimum_required(VERSION 3.13)

if(DEFINED FSM_TEST_HOST AND FSM_TEST_HOST)
    project(fsm_golden_test C)

    add_executable(fsm_golden_test
        test_fsm_golden.c
    )

    target_include_directories(fsm_golden_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_compile_options(fsm_golden_test PRIVATE
        -Wall -Wextra -Wpedantic
    )

    set_target_properties(fsm_golden_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()
