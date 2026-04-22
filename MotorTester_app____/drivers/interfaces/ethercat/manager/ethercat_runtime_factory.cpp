#include "ethercat/manager/ethercat_runtime_factory.h"

#include "ethercat/adapter/ethercat_axis_adapter.h"

#include <unordered_map>
#include <unordered_set>

namespace ethercat_driver {

namespace {

motion_core::Result<motion_core::RuntimeBuildResult> build_axes_on_bus(
    const std::shared_ptr<EthercatBusManager>& bus_manager,
    const std::vector<EthercatAxisRuntimeConfig>& axes,
    const bool close_bus_on_failure) {
    if (!bus_manager) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime has null bus manager"});
    }
    if (axes.empty()) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime config has no axes"});
    }

    motion_core::RuntimeBuildResult out{};
    out.bus_managers.push_back(bus_manager);

    auto fail_and_close = [bus_manager, close_bus_on_failure](const motion_core::Error& error)
        -> motion_core::Result<motion_core::RuntimeBuildResult> {
        if (close_bus_on_failure && bus_manager) {
            (void)bus_manager->close();
        }
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(error);
    };

    std::unordered_set<std::uint16_t> axis_ids_seen;
    out.axes.reserve(axes.size());

    for (const auto& axis_cfg : axes) {
        if (!axis_cfg.axis_id.valid()) {
            return fail_and_close(
                {motion_core::ErrorCode::InvalidArgument, "axis_id must be > 0"});
        }
        if (axis_cfg.axis_name.value.empty()) {
            return fail_and_close(
                {motion_core::ErrorCode::InvalidArgument, "axis_name cannot be empty"});
        }

        const auto axis_id_raw = axis_cfg.axis_id.value;
        if (!axis_ids_seen.insert(axis_id_raw).second) {
            return fail_and_close(
                {motion_core::ErrorCode::AlreadyExists, "duplicate axis_id in ethercat runtime config"});
        }

        const auto axis_info_result = bus_manager->get_slave_bus_info_by_position(axis_cfg.transport_address);
        if (!axis_info_result.ok()) {
            return fail_and_close(
                {motion_core::ErrorCode::NotFound, "transport_address from config is not present on scanned EtherCAT bus"});
        }

        EthercatAxisAdapterConfig adapter_cfg{};
        adapter_cfg.axis_id = axis_cfg.axis_id;
        adapter_cfg.axis_name = axis_cfg.axis_name;
        adapter_cfg.ecat_axis_index = axis_info_result.value().runtime_index;
        adapter_cfg.ecat_bus_position = axis_info_result.value().bus_position;
        adapter_cfg.bus_manager = bus_manager;

        auto adapter_result = make_configured_ethercat_axis_adapter(std::move(adapter_cfg));
        if (!adapter_result.ok()) {
            return fail_and_close(adapter_result.error());
        }
        
        out.axes.push_back(std::move(adapter_result.value()));
    }

    return motion_core::Result<motion_core::RuntimeBuildResult>::success(std::move(out));
}

} // namespace

motion_core::Result<motion_core::RuntimeBuildResult> build_ethercat_runtime(const motion_core::HalRuntimeConfig& hal_config) {
    std::unordered_map<std::string, EthercatBusConfig> bus_configs;
    for (const auto& bus : hal_config.ethercat_buses) {
        if (bus.interface_name.empty()) {
            return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                {motion_core::ErrorCode::InvalidArgument, "ethercat bus interface_name cannot be empty"});
        }
        bus_configs.emplace(bus.interface_name, EthercatBusConfig{bus.interface_name});
    }

    if (bus_configs.size() > 1U) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::Unsupported,
             "current EtherCAT runtime supports only one bus (master 0); multi-bus config is not supported"});
    }

    std::unordered_map<std::string, std::vector<EthercatAxisRuntimeConfig>> axes_by_bus_ref;
    std::unordered_set<std::uint16_t> global_axis_ids;

    for (const auto& axis : hal_config.axes) {
        if (axis.transport == motion_core::AxisTransportKind::Ethercat) {
            if (axis.bus_ref.empty()) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "ethercat axis bus_ref cannot be empty"});
            }
            if (bus_configs.find(axis.bus_ref) == bus_configs.end()) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::NotFound, "ethercat axis bus_ref does not match any ethercat bus"});
            }
            if (!global_axis_ids.insert(axis.axis_id.value).second) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::AlreadyExists, "duplicate axis_id across ethercat runtime config"});
            }
            EthercatAxisRuntimeConfig a;
            a.axis_id = axis.axis_id;
            a.axis_name = axis.axis_name;
            a.bus_ref = axis.bus_ref;
            a.transport_address = axis.transport_address;
            axes_by_bus_ref[axis.bus_ref].push_back(a);
        }
    }

    if (axes_by_bus_ref.empty()) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime config has no axes"});
    }

    if (axes_by_bus_ref.size() > 1U) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::Unsupported,
             "current EtherCAT runtime supports only one bus (master 0); axes must belong to a single bus_ref"});
    }

    motion_core::RuntimeBuildResult merged_result{};

    for (const auto& [bus_ref, axes] : axes_by_bus_ref) {
        const auto bus_cfg_it = bus_configs.find(bus_ref);
        if (bus_cfg_it == bus_configs.end()) {
            return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                {motion_core::ErrorCode::NotFound, "ethercat bus_ref is missing in bus configs"});
        }

        auto bus_manager = make_ethercat_bus_manager(bus_cfg_it->second);
        const auto open_result = bus_manager->open();
        if (!open_result.ok()) {
            return motion_core::Result<motion_core::RuntimeBuildResult>::failure(open_result.error());
        }

        const auto discovered = bus_manager->scan_axes();
        if (!discovered.ok()) {
            (void)bus_manager->close();
            return motion_core::Result<motion_core::RuntimeBuildResult>::failure(discovered.error());
        }

        const auto built = build_axes_on_bus(bus_manager, axes, true);
        if (!built.ok()) {
            return built;
        }

        for (const auto& bus : built.value().bus_managers) {
            merged_result.bus_managers.push_back(bus);
        }
        for (const auto& axis : built.value().axes) {
            merged_result.axes.push_back(axis);
        }
    }

    if (merged_result.axes.empty()) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime build produced no axes"});
    }

    return motion_core::Result<motion_core::RuntimeBuildResult>::success(std::move(merged_result));
}

} // namespace ethercat_driver
