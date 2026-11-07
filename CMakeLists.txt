cmake_minimum_required(VERSION 3.20)
project(rallysim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single-config generators need a default; multi-config (Visual Studio) choose at build time.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Strict floating point for deterministic results. Do not enable fast-math.
if(MSVC)
  add_compile_options(/W4 /fp:precise)
else()
  add_compile_options(-Wall -Wextra -ffp-contract=off)
endif()

# Header-only core library.
add_library(rallysim_core INTERFACE)
target_include_directories(rallysim_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(rallysim apps/main.cpp)
target_link_libraries(rallysim PRIVATE rallysim_core)

# Tests
enable_testing()

add_executable(test_vec3 tests/test_vec3.cpp)
target_link_libraries(test_vec3 PRIVATE rallysim_core)
add_test(NAME vec3 COMMAND test_vec3)

add_executable(test_quat tests/test_quat.cpp)
target_link_libraries(test_quat PRIVATE rallysim_core)
add_test(NAME quat COMMAND test_quat)

add_executable(test_rigid_body tests/test_rigid_body.cpp)
target_link_libraries(test_rigid_body PRIVATE rallysim_core)
add_test(NAME rigid_body COMMAND test_rigid_body)

add_executable(test_telemetry tests/test_telemetry.cpp)
target_link_libraries(test_telemetry PRIVATE rallysim_core)
add_test(NAME telemetry COMMAND test_telemetry)

add_executable(test_rigid_body_telemetry tests/test_rigid_body_telemetry.cpp)
target_link_libraries(test_rigid_body_telemetry PRIVATE rallysim_core)
add_test(NAME rigid_body_telemetry COMMAND test_rigid_body_telemetry)

add_executable(test_golden tests/test_golden.cpp)
target_link_libraries(test_golden PRIVATE rallysim_core)
add_test(NAME golden COMMAND test_golden)

add_executable(test_wrench_script tests/test_wrench_script.cpp)
target_link_libraries(test_wrench_script PRIVATE rallysim_core)
add_test(NAME wrench_script COMMAND test_wrench_script)

add_executable(test_wrench tests/test_wrench.cpp)
target_link_libraries(test_wrench PRIVATE rallysim_core)
add_test(NAME wrench COMMAND test_wrench)