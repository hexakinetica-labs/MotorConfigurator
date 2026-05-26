#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/parameter_types.h"
#include "motion_core/types.h"

#include <string>
#include <vector>
#include <cstdint>

namespace motion_core {

struct AxisConfigParameterMetadata {
    std::string domain_name{};
    std::string name{};
    std::string group{};
    std::string unit{};
    bool read_only{false};
    bool persistable{false};
};

struct AxisConfigParameterRecord {
    ParameterEntry entry{};
    AxisConfigParameterMetadata meta{};
};

/**
 * @brief Полный снимок конфигурации оси для применения при старте (Runtime Axis Config).
 * 
 * Включает в себя полный parameter snapshot оси вместе с базовой metadata,
 * чтобы файл был понятен человеку и мог использоваться для diff-based apply.
 */
struct AxisConfig {
    uint32_t version{2};
    AxisId axis_id{};
    AxisName axis_name{};
    AxisTransportKind transport{AxisTransportKind::Unknown};
    
    // Параметры, которые должны быть применены к приводу
    ParameterSet parameters;
    std::vector<AxisConfigParameterRecord> parameter_records;
    
    // Метаданные для пересчета единиц (если не входят в ParameterSet привода)
    double gear_ratio{1.0};
    uint32_t encoder_resolution_bits{0};
};

} // namespace motion_core
