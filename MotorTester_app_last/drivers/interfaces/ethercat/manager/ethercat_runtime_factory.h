#pragma once

#include "ethercat/manager/ethercat_bus_manager.h"
#include "motion_core/axis_interface.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/runtime_factory_registry.h"

#include <memory>
#include <string>
#include <vector>

namespace ethercat_driver {

struct EthercatAxisRuntimeConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    std::string bus_ref{};
    std::uint16_t transport_address{0};
};

[[nodiscard]] motion_core::Result<motion_core::RuntimeBuildResult> build_ethercat_runtime(
    const motion_core::HalRuntimeConfig& config);

} // namespace ethercat_driver
