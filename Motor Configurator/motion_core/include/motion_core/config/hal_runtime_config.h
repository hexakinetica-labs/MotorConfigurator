#pragma once

#include "motion_core/axis_data.h"
#include <string>
#include <vector>
#include <cstdint>

namespace motion_core {

/**
 * @brief Конфигурация шины MKS CAN (Bootstrap).
 */
struct HalBusConfigMks {
    std::string interface_id;
    std::string device_path;
    uint32_t baud_rate{1000000};
};

/**
 * @brief Описание привязки оси в рантайме (Topology entry).
 */
struct HalAxisRuntimeEntry {
    AxisId axis_id{};
    AxisName axis_name{};
    AxisTransportKind transport{AxisTransportKind::Unknown};
    std::string bus_ref;           // Идентификатор шины (interface_id или interface_name)
    uint16_t transport_address{0}; // CAN ID или EtherCAT Position
    std::string config_file;       // Путь к AxisConfig файлу
    bool enable_on_start{false};
};

/**
 * @brief Конфигурация шины EtherCAT (Bootstrap).
 */
struct HalBusConfigEthercat {
    std::string interface_name{};
    std::vector<HalAxisRuntimeEntry> axes{};
};

/**
 * @brief "Царь-конфиг" HAL (Master Bootstrap Config).
 */
struct HalRuntimeConfig {
    uint32_t version{1};
    
    struct {
        uint32_t dispatch_period_ms{4};
        uint32_t telemetry_period_ms{100};
    } runtime;

    std::vector<HalBusConfigMks> mks_buses;
    std::vector<HalBusConfigEthercat> ethercat_buses;
    std::vector<HalAxisRuntimeEntry> axes;
};

} // namespace motion_core
