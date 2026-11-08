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

# Tests: one executable per tests/test_<name>.cpp, registered with CTest as <name>.
enable_testing()

function(rallysim_test name)
  add_executable(test_${name} tests/test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE rallysim_core)
  add_test(NAME ${name} COMMAND test_${name})
endfunction()

rallysim_test(vec3)
rallysim_test(quat)
rallysim_test(rigid_body)
rallysim_test(telemetry)
rallysim_test(rigid_body_telemetry)
rallysim_test(golden)
rallysim_test(wrench)
rallysim_test(bicycle)
rallysim_test(keyframe_script)
rallysim_test(bicycle_telemetry)
rallysim_test(golden_bicycle)
rallysim_test(single_track)
rallysim_test(single_track_telemetry)
rallysim_test(single_track_behaviour)
rallysim_test(fixed_step)
rallysim_test(golden_single_track)
rallysim_test(tyre)
rallysim_test(wheel)
rallysim_test(four_wheel)
rallysim_test(four_wheel_behaviour)
rallysim_test(four_wheel_driving)
rallysim_test(four_wheel_springs)
rallysim_test(golden_four_wheel)
rallysim_test(tyre_gate)
rallysim_test(finite)
