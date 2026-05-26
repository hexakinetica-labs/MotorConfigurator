cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(motion_core LANGUAGES CXX)
endif()

add_library(motion_core STATIC
    src/runtime_loop.cpp
    src/runtime_factory_registry.cpp
    src/hal_runtime.cpp
    src/config/axis_config_json.cpp
    src/config/hal_runtime_config_json.cpp
)

target_compile_definitions(motion_core PRIVATE MOTION_CORE_WITH_DRIVERS)

target_include_directories(motion_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/interfaces
)

target_compile_features(motion_core PUBLIC cxx_std_20)

# Вариант без драйверов — только для unit-тестов (нет MOTION_CORE_WITH_DRIVERS).
# Исключает open_runtime/discover, которые тянут MKS/EtherCAT символы.
add_library(motion_core_no_hw STATIC
    src/runtime_loop.cpp
    src/runtime_factory_registry.cpp
    src/hal_runtime.cpp
    src/config/axis_config_json.cpp
    src/config/hal_runtime_config_json.cpp
)
target_include_directories(motion_core_no_hw
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
)
target_compile_features(motion_core_no_hw PUBLIC cxx_std_20)

if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
