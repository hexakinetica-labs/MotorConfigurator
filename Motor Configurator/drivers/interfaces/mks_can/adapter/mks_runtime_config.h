#pragma once

#include "motion_core/result.h"
#include "motion_core/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace mks {

struct MksAxisRuntimeConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    std::uint16_t can_id{1};
    double axis_units_per_degree{(16384.0 * 100.0) / 360.0};
    std::uint16_t default_speed{1800};
    std::uint8_t default_accel{255};
};

struct MksBusRuntimeConfig {
    std::string interface_id{};
    std::string device_path{};
    unsigned int baud_rate{1'000'000U};
    int cycle_time_us{2'500};
    std::vector<MksAxisRuntimeConfig> axes{};
};

struct MksRuntimeConfig {
    std::vector<MksBusRuntimeConfig> buses{};
};

[[nodiscard]] motion_core::Result<MksRuntimeConfig> load_mks_runtime_config_from_file(const std::string& path);

} // namespace mks
