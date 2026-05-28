cmake_minimum_required(VERSION 3.20)
project(battery-thermal-mpc
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "Predictive thermal control for immersion-cooled 24s13p Li-ion module")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE tooling (clangd, VS Code, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings (treat most as errors for portfolio quality)
if (MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Dependencies
find_package(yaml-cpp REQUIRED)
# yaml-cpp < 0.8.0 (Ubuntu 22.04 ships 0.7.0) exports target "yaml-cpp" without a namespace;
# 0.8.0+ (Homebrew) exports "yaml-cpp::yaml-cpp".  Provide the alias so the rest of this file
# can always use the namespaced form.
if(NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

find_package(Eigen3 REQUIRED)  # header-only; version relaxed for Homebrew packaging
find_package(GTest REQUIRED)

# Core library (types + config + model + solver + control + scenario + sim)
add_library(btm_core
    src/config/loader.cpp
    src/model/thermal_state.cpp
    src/model/thermal_model.cpp
    src/control/pid_controller.cpp
    src/control/mpc_controller.cpp
    src/solver/gradient_descent.cpp
    src/scenario/scenario.cpp
    src/sim/csv_logger.cpp
)
target_compile_features(btm_core PUBLIC cxx_std_20)
target_include_directories(btm_core PUBLIC include)
target_link_libraries(btm_core PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
)

# Main executable
add_executable(btm src/main.cpp)
target_link_libraries(btm PRIVATE btm_core)

# Tests
#
# We deliberately use a plain add_test instead of gtest_discover_tests.
# gtest_discover_tests executes the test binary at configure time to parse
# the test list. On macOS (especially with GTest from Conda), this frequently
# fails because the shared dylib cannot be found via rpath/DYLD_LIBRARY_PATH
# during the discovery step.
#
# A plain add_test is the simplest, most reliable approach across machines.
enable_testing()

add_executable(btm_tests
    test/test_core_types.cpp
    test/test_config_validation.cpp
    test/test_thermal_model.cpp
    test/test_pid_controller.cpp
    test/test_mpc_solver.cpp
    test/test_sensor_model.cpp
    test/test_integration.cpp
)
target_link_libraries(btm_tests PRIVATE btm_core GTest::gtest_main)

# Simple, reliable test registration (no auto-discovery at configure time)
add_test(NAME btm_unit_tests COMMAND btm_tests)
# Run tests from the repo root so relative paths (e.g. config/default.yaml) resolve correctly
set_tests_properties(btm_unit_tests PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Convenience target:  cmake --build . --target run_tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS btm_tests
    COMMENT "Running all unit tests with ctest"
)

# Install rules (optional for now)
install(TARGETS btm DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/btm/config)
