#include "mks/adapter/mks_runtime_factory.h"

#include "mks/adapter/mks_axis_adapter.h"

#include <chrono>
#include <unordered_set>

namespace mks {

motion_core::Result<motion_core::RuntimeBuildResult> build_mks_runtime(const motion_core::HalRuntimeConfig& hal_config) {
    if (hal_config.mks_buses.empty()) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "runtime config has no MKS buses"});
    }

    MksRuntimeConfig config;
    for (const auto& bus : hal_config.mks_buses) {
        MksBusRuntimeConfig b;
        b.interface_id = bus.interface_id;
        b.device_path = bus.device_path;
        b.baud_rate = bus.baud_rate;
        config.buses.push_back(std::move(b));
    }
    for (const auto& axis : hal_config.axes) {
        if (axis.transport == motion_core::AxisTransportKind::CanBus) {
            if (axis.bus_ref.empty()) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "mks axis bus_ref cannot be empty"});
            }
            MksAxisRuntimeConfig a;
            a.axis_id = axis.axis_id;
            a.axis_name = axis.axis_name;
            a.can_id = axis.transport_address;
            bool bus_found = false;
            for (auto& bus : config.buses) {
                if (bus.interface_id == axis.bus_ref) {
                    bus.axes.push_back(a);
                    bus_found = true;
                    break;
                }
            }
            if (!bus_found) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::NotFound, "mks axis bus_ref does not match any configured MKS bus"});
            }
        }
    }

    motion_core::RuntimeBuildResult out{};
    std::unordered_set<std::uint16_t> axis_ids_seen;

    for (const auto& bus_cfg : config.buses) {
        MksCanBusConfig low_bus_cfg{};
        low_bus_cfg.device_path = bus_cfg.device_path;
        low_bus_cfg.baud_rate = bus_cfg.baud_rate;
        low_bus_cfg.cycle_time = std::chrono::milliseconds(bus_cfg.cycle_time_ms);
        low_bus_cfg.simulated = (bus_cfg.interface_id.rfind("sim", 0) == 0) || (bus_cfg.device_path.rfind("sim", 0) == 0);

        auto bus_manager = make_mks_can_bus_manager(std::move(low_bus_cfg));
        out.bus_managers.push_back(bus_manager);

        std::unordered_set<std::uint16_t> can_ids_seen;

        for (const auto& axis_cfg : bus_cfg.axes) {
            if (!axis_cfg.axis_id.valid()) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "axis_id must be > 0"});
            }
            if (axis_cfg.axis_name.value.empty()) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "axis_name cannot be empty"});
            }

            const auto axis_id_raw = axis_cfg.axis_id.value;
            if (!axis_ids_seen.insert(axis_id_raw).second) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::AlreadyExists, "duplicate axis_id in runtime config"});
            }
            if (!can_ids_seen.insert(axis_cfg.can_id).second) {
                return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
                    {motion_core::ErrorCode::AlreadyExists, "duplicate can_id within one bus config"});
            }

            MksAxisAdapterConfig adapter_cfg{};
            adapter_cfg.axis_id = axis_cfg.axis_id;
            adapter_cfg.axis_name = axis_cfg.axis_name;
            adapter_cfg.can_id = axis_cfg.can_id;
            adapter_cfg.bus_manager = bus_manager;
            adapter_cfg.auto_start_bus_manager = true;
            adapter_cfg.axis_units_per_degree = axis_cfg.axis_units_per_degree;
            adapter_cfg.default_speed = axis_cfg.default_speed;
            adapter_cfg.default_accel = axis_cfg.default_accel;

            out.axes.push_back(make_mks_axis_adapter(std::move(adapter_cfg)));
        }
    }

    if (out.axes.empty()) {
        return motion_core::Result<motion_core::RuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "runtime config has no axes"});
    }

    return motion_core::Result<motion_core::RuntimeBuildResult>::success(std::move(out));
}

} // namespace mks
