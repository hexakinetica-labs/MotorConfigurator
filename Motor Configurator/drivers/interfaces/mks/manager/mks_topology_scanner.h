#pragma once

#include "motion_core/result.h"
#include "motion_core/config/hal_runtime_config.h"

#include <string>

namespace mks {

struct MksTopologyDiscoveryResult {
    motion_core::HalBusConfigMks bus_config;
    std::vector<motion_core::HalAxisRuntimeEntry> axes;
};

/// Scans the specified CAN bus for MKS axes without starting a full motion runtime.
/// \param device_path e.g. "can0", "sim0"
/// \param baud_rate e.g. 1000000
/// \param max_id maximum CAN ID to scan for (default: 127)
/// \return A populated HalBusConfigMks containing the discovered axes.
[[nodiscard]] motion_core::Result<MksTopologyDiscoveryResult> discover_mks_topology(
    const std::string& device_path,
    unsigned int baud_rate,
    int max_id = 127);

} // namespace mks
