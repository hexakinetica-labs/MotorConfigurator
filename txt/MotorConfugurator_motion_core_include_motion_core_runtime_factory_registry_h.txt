#pragma once

#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/bus_manager_interface.h"
#include "motion_core/axis_interface.h"
#include "motion_core/result.h"

#include <vector>
#include <memory>
#include <functional>
#include <cstdint>
#include <string>

namespace motion_core {

struct RuntimeBuildResult {
    std::vector<std::shared_ptr<IBusManager>> bus_managers;
    std::vector<std::shared_ptr<IAxis>> axes;
};

using RuntimeFactoryFn = std::function<Result<RuntimeBuildResult>(const HalRuntimeConfig&)>;

struct MksScanRequest {
    std::string device_path;
    std::uint32_t baud_rate{1000000};
    int max_id{127};
};

struct EthercatScanRequest {
    std::string interface_name;
};

using MksTopologyScannerFn = std::function<Result<HalRuntimeConfig>(const MksScanRequest&)>;
using EthercatTopologyScannerFn = std::function<Result<HalRuntimeConfig>(const EthercatScanRequest&)>;

class RuntimeFactoryRegistry {
public:
    static void register_factory(AxisTransportKind kind, RuntimeFactoryFn factory);
    static RuntimeFactoryFn get_factory(AxisTransportKind kind);

    static void register_mks_topology_scanner(MksTopologyScannerFn scanner);
    static MksTopologyScannerFn get_mks_topology_scanner();

    static void register_ethercat_topology_scanner(EthercatTopologyScannerFn scanner);
    static EthercatTopologyScannerFn get_ethercat_topology_scanner();
};

} // namespace motion_core
