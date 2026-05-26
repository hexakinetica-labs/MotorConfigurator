cmake_minimum_required(VERSION 3.16)
project(MotorTester_v1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

option(MOTORCONF_ENABLE_MKS_CAN "Enable MKS CAN backend" ON)
option(MOTORCONF_ENABLE_ETHERCAT "Enable Linux IgH EtherCAT backend" ON)

if (NOT MOTORCONF_ENABLE_MKS_CAN)
    message(FATAL_ERROR "MOTORCONF_ENABLE_MKS_CAN=OFF is not supported by the current target layout. MKS CAN sources are part of the CLI/GUI builds in this revision.")
endif()

if (WIN32 AND MOTORCONF_ENABLE_ETHERCAT)
    message(STATUS "MOTORCONF_ENABLE_ETHERCAT is forced OFF on Windows because the current backend depends on Linux IgH ecrt APIs.")
    set(MOTORCONF_ENABLE_ETHERCAT OFF CACHE BOOL "Enable Linux IgH EtherCAT backend" FORCE)
endif()

if (WIN32 AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "Windows builds are supported only with MinGW/GCC. Use a Qt MinGW kit and CMAKE_CXX_COMPILER_ID=GNU.")
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

if (MOTORCONF_ENABLE_MKS_CAN)
    if (WIN32)
        set(MOTORCONF_WIN_LIBUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/win/libusb")
        set(LIBUSB_INCLUDE_DIR "${MOTORCONF_WIN_LIBUSB_ROOT}/include/libusb-1.0")
        set(LIBUSB_LIBRARY "${MOTORCONF_WIN_LIBUSB_ROOT}/lib/x64/libusb-1.0.dll.a")
        set(LIBUSB_RUNTIME_DLL "${MOTORCONF_WIN_LIBUSB_ROOT}/bin/x64/libusb-1.0.dll")

        if (NOT EXISTS "${LIBUSB_INCLUDE_DIR}/libusb.h" OR NOT EXISTS "${LIBUSB_LIBRARY}" OR NOT EXISTS "${LIBUSB_RUNTIME_DLL}")
            message(FATAL_ERROR
                "MKS GS-USB support is enabled, but required MinGW/GCC libusb files are missing.\n"
                "Expected layout:\n"
                "  3rdparty/win/libusb/include/libusb-1.0/libusb.h\n"
                "  3rdparty/win/libusb/lib/x64/libusb-1.0.dll.a\n"
                "  3rdparty/win/libusb/bin/x64/libusb-1.0.dll\n"
                "The current CLI/GUI targets require the MKS CAN backend in this revision.")
        endif()
    else()
        find_path(LIBUSB_INCLUDE_DIR libusb.h PATH_SUFFIXES libusb-1.0)
        find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0)

        if (NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
            message(FATAL_ERROR "MKS GS-USB support is required by the current CLI/GUI targets, but libusb-1.0 was not found. Install libusb dev package and/or set LIBUSB_INCLUDE_DIR and LIBUSB_LIBRARY.")
        endif()
    endif()
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/motion_core motion_core_build)

add_library(hal_ipc STATIC
    hal_ipc/src/protocol.cpp
    hal_ipc/src/client.cpp
    hal_ipc/src/server.cpp
)

if (WIN32)
    target_link_libraries(hal_ipc PRIVATE ws2_32)
endif()

target_include_directories(hal_ipc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/hal_ipc/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/motion_core/include
)

target_link_libraries(hal_ipc PRIVATE motion_core)
target_compile_features(hal_ipc PUBLIC cxx_std_20)


set(MOTORCONF_MKS_CAN_SOURCES
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

    drivers/interfaces/mks_can/axis/mks_axis_worker.h
    drivers/interfaces/mks_can/axis/mks_axis_worker.cpp
    drivers/interfaces/mks_can/axis/mks_axis_state_cache.h
    drivers/interfaces/mks_can/axis/mks_homing_state_machine.h
    drivers/interfaces/mks_can/axis/mks_homing_state_machine.cpp
    drivers/interfaces/mks_can/axis/mks_axis_configurator.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.cpp
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.h
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_config.h
    drivers/interfaces/mks_can/adapter/mks_runtime_config.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.h
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.cpp
)

set(MOTORCONF_ETHERCAT_SOURCES
    drivers/interfaces/ethercat/manager/ethercat_bus_manager.cpp
    drivers/interfaces/ethercat/manager/ethercat_runtime_factory.cpp
    drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp
    drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.cpp
)

add_executable(mks_can_cli
    ${MOTORCONF_MKS_CAN_SOURCES}
    drivers/interfaces/init_all_drivers.cpp
)

if (MOTORCONF_ENABLE_ETHERCAT)
    target_sources(mks_can_cli PRIVATE ${MOTORCONF_ETHERCAT_SOURCES})
endif()

target_include_directories(mks_can_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/interfaces
    ${LIBUSB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

target_link_libraries(mks_can_cli PRIVATE ${LIBUSB_LIBRARY} motion_core)
target_compile_definitions(mks_can_cli PRIVATE MOTION_CORE_WITH_DRIVERS MOTORCONF_ENABLE_MKS_CAN=$<BOOL:${MOTORCONF_ENABLE_MKS_CAN}> MOTORCONF_ENABLE_ETHERCAT=$<BOOL:${MOTORCONF_ENABLE_ETHERCAT}>)
if (MOTORCONF_ENABLE_ETHERCAT)
    target_link_libraries(mks_can_cli PRIVATE ethercat)
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(mks_can_cli PRIVATE atomic)
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets Core)

add_executable(motor_tester_gui
    ui/qt/mks/ScopeWidget.h
    ui/qt/mks/axis_manager/axis_manager.h
    ui/qt/mks/axis_workspace/axis_workspace.h
    ui/qt/mks/axis_workspace/axis_workspace_config_panel.h
    ui/qt/mks/axis_workspace/axis_workspace_control_panel.h
    ui/qt/mks/mks_all_axes_control_workspace.h
    ui/qt/mks/main_window.h
    drivers/interfaces/mks_can/manager/mks_can_bus_manager.h
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.h
    drivers/interfaces/mks_can/adapter/mks_runtime_config.h
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.h
    ui/qt/mks/sequencer_widget.h
    ui/qt/mks/axis_manager/axis_manager_core.cpp
    ui/qt/mks/axis_manager/axis_manager_params.cpp
    ui/qt/mks/axis_manager/axis_manager_motion.cpp
    ui/qt/mks/axis_manager/axis_manager_config.cpp
    ui/qt/mks/axis_manager/axis_manager_hal_service.cpp
    ui/qt/mks/axis_manager/axis_manager_homing_sequence.cpp
    ui/qt/mks/axis_workspace/axis_workspace.cpp
    ui/qt/mks/axis_workspace/axis_workspace_config_panel.cpp
    ui/qt/mks/axis_workspace/axis_workspace_control_panel.cpp
    ui/qt/mks/mks_all_axes_control_workspace.cpp
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

    drivers/interfaces/mks_can/axis/mks_axis_worker.h
    drivers/interfaces/mks_can/axis/mks_axis_worker.cpp
    drivers/interfaces/mks_can/axis/mks_axis_state_cache.h
    drivers/interfaces/mks_can/axis/mks_homing_state_machine.h
    drivers/interfaces/mks_can/axis/mks_homing_state_machine.cpp
    drivers/interfaces/mks_can/axis/mks_axis_configurator.h
    drivers/interfaces/mks_can/axis/mks_axis_configurator.cpp
    drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_config.cpp
    drivers/interfaces/mks_can/adapter/mks_runtime_factory.cpp
    drivers/interfaces/init_all_drivers.cpp
)

if (MOTORCONF_ENABLE_ETHERCAT)
    target_sources(motor_tester_gui PRIVATE ${MOTORCONF_ETHERCAT_SOURCES})
endif()

target_include_directories(motor_tester_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/interfaces
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt
    ${LIBUSB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

target_link_libraries(motor_tester_gui PRIVATE Qt6::Widgets ${LIBUSB_LIBRARY} motion_core hal_ipc)
target_compile_definitions(motor_tester_gui PRIVATE MOTION_CORE_WITH_DRIVERS MOTORCONF_ENABLE_MKS_CAN=$<BOOL:${MOTORCONF_ENABLE_MKS_CAN}> MOTORCONF_ENABLE_ETHERCAT=$<BOOL:${MOTORCONF_ENABLE_ETHERCAT}>)
if (MOTORCONF_ENABLE_ETHERCAT)
    target_link_libraries(motor_tester_gui PRIVATE ethercat)
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(motor_tester_gui PRIVATE atomic)
endif()
