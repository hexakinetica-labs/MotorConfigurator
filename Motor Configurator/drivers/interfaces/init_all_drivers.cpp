#include "init_all_drivers.h"

#include "mks/adapter/mks_runtime_factory.h"
#include "mks/manager/mks_topology_scanner.h"
#include "ethercat/manager/ethercat_runtime_factory.h"
#include "ethercat/manager/ethercat_bus_manager.h"
#include "motion_core/runtime_factory_registry.h"
#include "motion_core/config/hal_runtime_config.h"

namespace drivers {

void init_all_drivers() {
    motion_core::RuntimeFactoryRegistry::register_factory(
        motion_core::AxisTransportKind::CanBus,
        [](const motion_core::HalRuntimeConfig& cfg) { return mks::build_mks_runtime(cfg); }
    );
    motion_core::RuntimeFactoryRegistry::register_factory(
        motion_core::AxisTransportKind::Ethercat,
        [](const motion_core::HalRuntimeConfig& cfg) { return ethercat_driver::build_ethercat_runtime(cfg); }
    );
    motion_core::RuntimeFactoryRegistry::register_mks_topology_scanner(
        [](const motion_core::MksScanRequest& request) -> motion_core::Result<motion_core::HalRuntimeConfig> {
            const auto discovered = mks::discover_mks_topology(
                request.device_path,
                request.baud_rate,
                request.max_id);
            if (!discovered.ok()) {
                return motion_core::Result<motion_core::HalRuntimeConfig>::failure(discovered.error());
            }
            motion_core::HalRuntimeConfig config{};
            config.mks_buses.push_back(discovered.value().bus_config);
            config.axes = discovered.value().axes;
            return motion_core::Result<motion_core::HalRuntimeConfig>::success(std::move(config));
        }
    );
    motion_core::RuntimeFactoryRegistry::register_ethercat_topology_scanner(
        [](const motion_core::EthercatScanRequest& request) -> motion_core::Result<motion_core::HalRuntimeConfig> {
            const auto discovered = ethercat_driver::EthercatBusManager::discover_ethercat_topology(
                request.interface_name);
            if (!discovered.ok()) {
                return motion_core::Result<motion_core::HalRuntimeConfig>::failure(discovered.error());
            }
            motion_core::HalRuntimeConfig config{};
            auto bus_cfg = discovered.value();
            config.axes.insert(config.axes.end(), bus_cfg.axes.begin(), bus_cfg.axes.end());
            bus_cfg.axes.clear();
            config.ethercat_buses.push_back(std::move(bus_cfg));
            return motion_core::Result<motion_core::HalRuntimeConfig>::success(std::move(config));
        }
    );
}

} // namespace drivers
