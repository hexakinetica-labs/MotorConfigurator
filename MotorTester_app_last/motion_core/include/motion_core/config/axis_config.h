#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/parameter_types.h"
#include "motion_core/types.h"

#include <string>
#include <vector>
#include <cstdint>

namespace motion_core {

/**
 * @brief Полный снимок конфигурации оси для применения при старте (Runtime Axis Config).
 * 
 * Включает в себя PID, gear ratio, пределы скоростей/ускорений, настройки энкодера и т.д.
 * НЕ включает в себя bootstrap identity (CAN ID, Baudrate), так как они нужны до загрузки этого конфига.
 */
struct AxisConfig {
    uint32_t version{1};
    AxisId axis_id{};
    AxisName axis_name{};
    AxisTransportKind transport{AxisTransportKind::Unknown};
    
    // Параметры, которые должны быть применены к приводу
    ParameterSet parameters;
    
    // Метаданные для пересчета единиц (если не входят в ParameterSet привода)
    double gear_ratio{1.0};
    uint32_t encoder_resolution_bits{0};
};

} // namespace motion_core
