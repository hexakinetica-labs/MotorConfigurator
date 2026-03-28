cmake_minimum_required(VERSION 3.16)
project(MotorTester_v1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "This project must be built with GCC/G++ via CMake (CMAKE_CXX_COMPILER_ID=GNU).")
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_path(LIBUSB_INCLUDE_DIR libusb.h PATH_SUFFIXES libusb-1.0)
find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0)

if (NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(WARNING "libusb-1.0 not found. Install libusb dev package and/or set LIBUSB_INCLUDE_DIR and LIBUSB_LIBRARY.")
endif()

# Using system libethercat (ecrt) instead of legacy wrapper

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/motion_core motion_core_build)

add_executable(mks_can_cli
    apps/cli/main.cpp
    drivers/interfaces/mks_can/internal/port/gs_usb_can_port.cpp
    drivers/interfaces/mks_can/internal/port/sim_can_port.h
    drivers/interfaces/mks_can/internal/port/sim_can_port.cpp
    drivers/interfaces/mks_can/internal/protocol/mks_protocol.cpp
    drivers/interfaces/mks_can/manager/mks_can_bus_manager.h
    drivers/interfaces/mks_can/manager/mks_can_bus_manager.cpp
    drivers/interfaces/mks_can/manager/mks_topology_scanner.h
    drivers/interfaces/mks_can/manager/mks_topology_scanner.cpp
    drivers/interfaces/mks_can/dictionary/mks_dictionary.h
    drivers/interfaces/mks_can/dictionary/mks_dictionary.cpp
    drivers/interfaces/mks_can/motion/mks_motion_mode.h
    drivers/interfaces/mks_can/motion/mks_absolute_position_motion.cpp
    drivers/interfaces/mks_can/motion/mks_velocity_motion.cpp
    drivers/interfaces/mks_can/axis/mks_axis_worker.h
    drivers/interfaces/mks_can/axis/mks_axis_worker.cpp
    drivers/interfaces/mks_can/axis/mks_axis_state_cache.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.cpp
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.h
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_config.h
    drivers/interfaces/mks_can/adapter/mks_runtime_config.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.h
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.cpp
    # Added EtherCAT driver to CLI for unified runtime factory flow
    drivers/interfaces/ethercat/manager/ethercat_bus_manager.cpp
    drivers/interfaces/ethercat/manager/ethercat_runtime_factory.cpp
    drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp
    drivers/interfaces/init_all_drivers.cpp
)

target_include_directories(mks_can_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/interfaces
    ${LIBUSB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

target_link_libraries(mks_can_cli PRIVATE ${LIBUSB_LIBRARY} motion_core ethercat atomic)
target_compile_definitions(mks_can_cli PRIVATE MOTION_CORE_WITH_DRIVERS)

find_package(Qt6 REQUIRED COMPONENTS Widgets Core)

add_executable(motor_tester_gui
    ui/qt/mks/ScopeWidget.h
    ui/qt/mks/axis_manager.h
    ui/qt/mks/axis_workspace.h
    ui/qt/mks/main_window.h
    drivers/interfaces/mks_can/manager/mks_can_bus_manager.h
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.h
    drivers/interfaces/mks_can/adapter/mks_runtime_config.h
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.h
    ui/qt/mks/sequencer_widget.h
    ui/qt/mks/axis_manager.cpp
    ui/qt/mks/axis_workspace.cpp
    ui/qt/mks/ScopeWidget.cpp
    ui/qt/mks/sequencer_widget.cpp
    apps/qt/qt_main.cpp
    ui/qt/mks/main_window.cpp
    drivers/interfaces/mks_can/internal/port/gs_usb_can_port.cpp
    drivers/interfaces/mks_can/internal/port/sim_can_port.h
    drivers/interfaces/mks_can/internal/port/sim_can_port.cpp
    drivers/interfaces/mks_can/internal/protocol/mks_protocol.cpp
    drivers/interfaces/mks_can/manager/mks_can_bus_manager.cpp
    drivers/interfaces/mks_can/manager/mks_topology_scanner.h
    drivers/interfaces/mks_can/manager/mks_topology_scanner.cpp
    drivers/interfaces/mks_can/dictionary/mks_dictionary.cpp
    drivers/interfaces/mks_can/motion/mks_motion_mode.h
    drivers/interfaces/mks_can/motion/mks_absolute_position_motion.cpp
    drivers/interfaces/mks_can/motion/mks_velocity_motion.cpp
    drivers/interfaces/mks_can/axis/mks_axis_worker.h
    drivers/interfaces/mks_can/axis/mks_axis_worker.cpp
    drivers/interfaces/mks_can/axis/mks_axis_state_cache.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.cpp
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_config.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.cpp
    # EtherCAT backend integration (Qt-independent)
    drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.h
    drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp
    drivers/interfaces/ethercat/manager/ethercat_bus_manager.h
    drivers/interfaces/ethercat/manager/ethercat_bus_manager.cpp
    drivers/interfaces/ethercat/manager/ethercat_runtime_factory.h
    drivers/interfaces/ethercat/manager/ethercat_runtime_factory.cpp
    drivers/interfaces/init_all_drivers.cpp
)

target_include_directories(motor_tester_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/interfaces
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt
    ${LIBUSB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

target_link_libraries(motor_tester_gui PRIVATE Qt6::Widgets ${LIBUSB_LIBRARY} motion_core ethercat atomic)
target_compile_definitions(motor_tester_gui PRIVATE MOTION_CORE_WITH_DRIVERS)
