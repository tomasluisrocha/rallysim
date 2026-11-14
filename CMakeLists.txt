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
# The scenario files (scenarios/*.ini), for rallysim <name> and four_wheel_scenario() in the tests.
target_compile_definitions(rallysim_core INTERFACE RALLYSIM_SCENARIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenarios")

add_executable(rallysim apps/main.cpp)
target_link_libraries(rallysim PRIVATE rallysim_core)

# Compiles every header alone, included twice, to catch missing includes and broken include guards.
file(GLOB_RECURSE RALLYSIM_HEADERS CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     core/*.hpp vehicle/*.hpp driver/*.hpp runner/*.hpp)
set(RALLYSIM_HEADER_CHECKS)
set(header_index 0)
foreach(header ${RALLYSIM_HEADERS})
  set(check_cpp ${CMAKE_BINARY_DIR}/header_check/${header_index}.cpp)
  file(GENERATE OUTPUT ${check_cpp} CONTENT "#include \"${header}\"\n#include \"${header}\"\n")
  list(APPEND RALLYSIM_HEADER_CHECKS ${check_cpp})
  math(EXPR header_index "${header_index} + 1")
endforeach()
add_library(header_check OBJECT ${RALLYSIM_HEADER_CHECKS})
target_link_libraries(header_check PRIVATE rallysim_core)

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
rallysim_test(unsprung_corner)
rallysim_test(four_wheel_unsprung)
rallysim_test(golden_four_wheel)
rallysim_test(golden_unsprung)
rallysim_test(tyre_gate)
rallysim_test(finite)
rallysim_test(energy)
rallysim_test(landing)
rallysim_test(registry)
rallysim_test(ini)
rallysim_test(ini_values)
rallysim_test(scenario_file)
rallysim_test(quarter_car)
target_compile_definitions(test_ini PRIVATE RALLYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}") # to read scenarios/*.ini
target_compile_definitions(test_scenario_file PRIVATE RALLYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# The command line: --set reaches a four-wheel car; a bad name, a missing value, a model without settings or a value
# outside its range fails.
add_test(NAME cli_set COMMAND rallysim fw_brake ${CMAKE_BINARY_DIR}/cli_set.csv
         --set spring_front=60000 --set mu_rear=0.8)
set_tests_properties(cli_set PROPERTIES PASS_REGULAR_EXPRESSION "with --set")
add_test(NAME cli_set_unknown COMMAND rallysim fw_brake ${CMAKE_BINARY_DIR}/cli_bad.csv --set sprng_front=1)
add_test(NAME cli_set_no_value COMMAND rallysim fw_brake ${CMAKE_BINARY_DIR}/cli_bad.csv --set spring_front)
add_test(NAME cli_set_not_four_wheel COMMAND rallysim st_circle ${CMAKE_BINARY_DIR}/cli_bad.csv --set mass=1)
add_test(NAME cli_set_out_of_range COMMAND rallysim fw_brake ${CMAKE_BINARY_DIR}/cli_bad.csv --set mass=0)
set_tests_properties(cli_set_unknown cli_set_no_value cli_set_not_four_wheel cli_set_out_of_range
                     PROPERTIES WILL_FAIL TRUE)

# params takes --set too: the listing is of the car with the settings (tyre_rate_front is used with vertical = unsprung,
# and its line then has no "not used" note), and a bad setting fails.
add_test(NAME cli_params_set COMMAND rallysim params fw_lane --set vertical=unsprung)
set_tests_properties(cli_params_set PROPERTIES PASS_REGULAR_EXPRESSION "tyre_rate_front +[0-9]+ +N/m +\n")
add_test(NAME cli_params_set_bad COMMAND rallysim params fw_lane --set vertical=unsprun)
set_tests_properties(cli_params_set_bad PROPERTIES WILL_FAIL TRUE)
