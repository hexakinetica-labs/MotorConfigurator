cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(motion_core LANGUAGES CXX)
endif()

add_library(motion_core STATIC
    runtime_loop.cpp
    runtime_factory_registry.cpp
    hal_runtime.cpp
    axis_config_json.cpp
    hal_runtime_config_json.cpp
    motion_orchestrator.cpp
    telemetry_distributor.cpp
    parameter_manager.cpp
    config_manager.cpp
    trajectory_generator.cpp
)

target_compile_definitions(motion_core PRIVATE MOTION_CORE_WITH_DRIVERS)

target_include_directories(motion_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/interfaces
)

target_compile_features(motion_core PUBLIC cxx_std_20)

# Variant without HW - for unit tests
add_library(motion_core_no_hw STATIC
    runtime_loop.cpp
    runtime_factory_registry.cpp
    hal_runtime.cpp
    axis_config_json.cpp
    hal_runtime_config_json.cpp
    motion_orchestrator.cpp
    telemetry_distributor.cpp
    parameter_manager.cpp
    config_manager.cpp
    trajectory_generator.cpp
)
target_include_directories(motion_core_no_hw
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
)
target_compile_features(motion_core_no_hw PUBLIC cxx_std_20)

if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()