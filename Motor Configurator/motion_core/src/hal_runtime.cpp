#include "motion_core/hal_runtime.h"

#include "motion_core/config/axis_config_json.h"
#include "motion_core/runtime_factory_registry.h"

#include <algorithm>

namespace motion_core {

Result<void> HalRuntime::merge_runtime_build(const RuntimeBuildResult& build,
                                             std::vector<std::shared_ptr<IBusManager>>& buses,
                                             std::unordered_map<std::uint16_t, std::shared_ptr<IAxis>>& axes) const {
    for (const auto& bus : build.bus_managers) {
        buses.push_back(bus);
    }
    for (const auto& axis : build.axes) {
        if (!axis) {
            return Result<void>::failure(
                {ErrorCode::InvalidArgument, "runtime factory returned null axis"});
        }
        const auto id = axis->info().id.value;
        if (axes.find(id) != axes.end()) {
            return Result<void>::failure(
                {ErrorCode::AlreadyExists, "duplicate axis id in runtime"});
        }
        axes.emplace(id, axis);
    }
    return Result<void>::success();
}

Result<void> HalRuntime::open_from_config(const HalRuntimeConfig& config) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (runtime_open_) {
            return Result<void>::failure(
                {ErrorCode::AlreadyExists, "runtime is already open; close it before re-open"});
        }
    }

    std::vector<std::shared_ptr<IBusManager>> built_buses;
    std::unordered_map<std::uint16_t, std::shared_ptr<IAxis>> built_axes;

    const auto has_transport = [&](const AxisTransportKind kind) {
        return std::any_of(config.axes.begin(), config.axes.end(), [kind](const auto& axis) {
            return axis.transport == kind;
        });
    };

    const auto build_for = [&](const AxisTransportKind kind) -> Result<void> {
        if (!has_transport(kind)) {
            return Result<void>::success();
        }
        auto factory = RuntimeFactoryRegistry::get_factory(kind);
        if (!factory) {
            return Result<void>::failure(
                {ErrorCode::NotFound, "runtime factory is not registered"});
        }
        const auto built = factory(config);
        if (!built.ok()) {
            return Result<void>::failure(built.error());
        }
        return merge_runtime_build(built.value(), built_buses, built_axes);
    };

    if (auto r = build_for(AxisTransportKind::CanBus); !r.ok()) {
        return r;
    }
    if (auto r = build_for(AxisTransportKind::Ethercat); !r.ok()) {
        return r;
    }
    if (built_axes.empty()) {
        return Result<void>::failure(
            {ErrorCode::InvalidArgument, "runtime config has no axes"});
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (runtime_open_) {
            return Result<void>::failure(
                {ErrorCode::AlreadyExists, "runtime is already open; close it before re-open"});
        }
        axes_ = std::move(built_axes);
        buses_ = std::move(built_buses);
        runtime_config_ = config;
        runtime_open_ = true;
        runtime_active_ = false;
    }
    return Result<void>::success();
}

Result<HalRuntimeConfig> HalRuntime::scan_mks_topology(const MksScanRequest& request) const {
    if (request.device_path.empty()) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "mks scan requires non-empty device_path"});
    }
    if (request.baud_rate == 0U) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "mks scan requires baud_rate > 0"});
    }
    if (request.max_id <= 0) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "mks scan requires max_id > 0"});
    }

    auto scanner = RuntimeFactoryRegistry::get_mks_topology_scanner();
    if (!scanner) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::NotFound, "mks topology scanner is not registered"});
    }
    return scanner(request);
}

Result<HalRuntimeConfig> HalRuntime::scan_ethercat_topology(const EthercatScanRequest& request) const {
    if (request.interface_name.empty()) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "ethercat scan requires non-empty interface_name"});
    }

    auto scanner = RuntimeFactoryRegistry::get_ethercat_topology_scanner();
    if (!scanner) {
        return Result<HalRuntimeConfig>::failure(
            {ErrorCode::NotFound, "ethercat topology scanner is not registered"});
    }
    return scanner(request);
}

Result<void> HalRuntime::start() {
    std::vector<std::shared_ptr<IBusManager>> buses_snapshot;
    std::vector<std::pair<std::uint16_t, std::shared_ptr<IAxis>>> axes_snapshot;
    HalRuntimeConfig runtime_config_snapshot{};

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!runtime_open_) {
            return Result<void>::failure({ErrorCode::NotConnected, "runtime is not open"});
        }
        if (runtime_active_) {
            return Result<void>::success();
        }

        buses_snapshot = buses_;
        axes_snapshot.reserve(axes_.size());
        for (const auto& [id, axis] : axes_) {
            axes_snapshot.emplace_back(id, axis);
        }
        runtime_config_snapshot = runtime_config_;
    }

    std::unordered_map<std::uint16_t, bool> enable_on_start_policy;
    for (const auto& axis_cfg : runtime_config_snapshot.axes) {
        enable_on_start_policy[axis_cfg.axis_id.value] = axis_cfg.enable_on_start;
    }

    std::vector<std::shared_ptr<IBusManager>> started_buses;
    for (const auto& bus : buses_snapshot) {
        if (!bus) {
            continue;
        }
        const auto started = bus->start();
        if (!started.ok()) {
            for (auto it = started_buses.rbegin(); it != started_buses.rend(); ++it) {
                (void)(*it)->stop();
            }
            return started;
        }
        started_buses.push_back(bus);
    }

    std::vector<std::shared_ptr<IAxis>> started_axes;
    for (const auto& [id, axis] : axes_snapshot) {
        if (!axis) {
            continue;
        }
        const auto started = axis->start();
        if (!started.ok()) {
            for (auto it = started_axes.rbegin(); it != started_axes.rend(); ++it) {
                (void)(*it)->stop();
            }
            for (auto it = started_buses.rbegin(); it != started_buses.rend(); ++it) {
                (void)(*it)->stop();
            }
            return started;
        }
        started_axes.push_back(axis);

        const auto policy_it = enable_on_start_policy.find(id);
        if (policy_it != enable_on_start_policy.end()) {
            const auto enable_res = axis->set_enabled(policy_it->second);
            if (!enable_res.ok()) {
                for (auto it = started_axes.rbegin(); it != started_axes.rend(); ++it) {
                    (void)(*it)->stop();
                }
                for (auto it = started_buses.rbegin(); it != started_buses.rend(); ++it) {
                    (void)(*it)->stop();
                }
                return enable_res;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!runtime_open_) {
            for (auto it = started_axes.rbegin(); it != started_axes.rend(); ++it) {
                (void)(*it)->stop();
            }
            for (auto it = started_buses.rbegin(); it != started_buses.rend(); ++it) {
                (void)(*it)->stop();
            }
            return Result<void>::failure({ErrorCode::NotConnected, "runtime was closed during start"});
        }
        runtime_active_ = true;
    }

    return Result<void>::success();
}

Result<void> HalRuntime::stop() {
    std::vector<std::shared_ptr<IAxis>> axes_snapshot;
    std::vector<std::shared_ptr<IBusManager>> buses_snapshot;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        axes_snapshot.reserve(axes_.size());
        for (const auto& [id, axis] : axes_) {
            (void)id;
            if (axis) {
                axes_snapshot.push_back(axis);
            }
        }
        buses_snapshot = buses_;
        runtime_active_ = false;
    }

    bool has_error = false;
    Error first_error = Error::success();

    for (const auto& axis : axes_snapshot) {
        const auto stop_res = axis->stop();
        if (!stop_res.ok() && !has_error) {
            has_error = true;
            first_error = stop_res.error();
        }
    }

    for (const auto& bus : buses_snapshot) {
        if (!bus) {
            continue;
        }
        const auto stop_res = bus->stop();
        if (!stop_res.ok() && !has_error) {
            has_error = true;
            first_error = stop_res.error();
        }
    }

    if (has_error) {
        return Result<void>::failure(first_error);
    }
    return Result<void>::success();
}

Result<void> HalRuntime::close() {
    (void)stop();
    std::lock_guard<std::mutex> lock(mutex_);
    axes_.clear();
    buses_.clear();
    runtime_config_ = {};
    runtime_open_ = false;
    runtime_active_ = false;
    return Result<void>::success();
}

Result<std::shared_ptr<IAxis>> HalRuntime::find_axis(const std::uint16_t axis_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = axes_.find(axis_id);
    if (it == axes_.end() || !it->second) {
        return Result<std::shared_ptr<IAxis>>::failure({ErrorCode::NotFound, "axis not found"});
    }
    return Result<std::shared_ptr<IAxis>>::success(it->second);
}

Result<std::vector<AxisInfo>> HalRuntime::list_axes() const {
    std::vector<AxisInfo> out;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        out.reserve(axes_.size());
        for (const auto& [id, axis] : axes_) {
            (void)id;
            if (axis) {
                out.push_back(axis->info());
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.id.value < rhs.id.value;
    });
    return Result<std::vector<AxisInfo>>::success(std::move(out));
}

std::vector<std::shared_ptr<IBusManager>> HalRuntime::bus_managers_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buses_;
}

Result<void> HalRuntime::export_axis_config_to_file(const std::uint16_t axis_id, const std::string& path) const {
    const auto axis_res = find_axis(axis_id);
    if (!axis_res.ok()) {
        return Result<void>::failure(axis_res.error());
    }

    const auto desc_res = axis_res.value()->list_parameters();
    if (!desc_res.ok()) {
        return Result<void>::failure(desc_res.error());
    }

    const auto read_res = axis_res.value()->read_parameters();
    if (!read_res.ok()) {
        return Result<void>::failure(read_res.error());
    }

    AxisConfig cfg{};
    const auto info = axis_res.value()->info();
    cfg.axis_id = info.id;
    cfg.axis_name = info.name;
    cfg.transport = info.transport;

    for (const auto& entry : read_res.value().entries) {
        bool persistable = false;
        for (const auto& d : desc_res.value()) {
            if (d.id.domain == entry.id.domain && d.id.value == entry.id.value) {
                persistable = d.persistable;
                break;
            }
        }
        if (persistable) {
            cfg.parameters.entries.push_back(entry);
        }
    }

    return save_axis_config_to_file(path, cfg);
}

Result<ParameterPatch> HalRuntime::build_axis_config_patch(const std::uint16_t axis_id, const std::string& path) const {
    const auto axis_res = find_axis(axis_id);
    if (!axis_res.ok()) {
        return Result<ParameterPatch>::failure(axis_res.error());
    }

    const auto load_res = load_axis_config_from_file(path);
    if (!load_res.ok()) {
        return Result<ParameterPatch>::failure(load_res.error());
    }

    const auto desc_res = axis_res.value()->list_parameters();
    if (!desc_res.ok()) {
        return Result<ParameterPatch>::failure(desc_res.error());
    }

    ParameterPatch patch{};
    for (const auto& entry : load_res.value().parameters.entries) {
        bool writable_persistable = false;
        for (const auto& d : desc_res.value()) {
            if (d.id.domain == entry.id.domain && d.id.value == entry.id.value && d.persistable && !d.read_only) {
                writable_persistable = true;
                break;
            }
        }
        if (writable_persistable) {
            patch.entries.push_back(entry);
        }
    }

    return Result<ParameterPatch>::success(std::move(patch));
}

Result<void> HalRuntime::apply_axis_config_patch(const std::uint16_t axis_id, const ParameterPatch& patch) {
    const auto axis_res = find_axis(axis_id);
    if (!axis_res.ok()) {
        return Result<void>::failure(axis_res.error());
    }

    if (patch.entries.empty()) {
        return Result<void>::success();
    }
    return axis_res.value()->apply_parameter_patch(patch);
}

Result<void> HalRuntime::apply_axis_config_file(const std::uint16_t axis_id, const std::string& path) {
    const auto patch_res = build_axis_config_patch(axis_id, path);
    if (!patch_res.ok()) {
        return Result<void>::failure(patch_res.error());
    }
    return apply_axis_config_patch(axis_id, patch_res.value());
}

bool HalRuntime::is_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_open_;
}

bool HalRuntime::is_active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_active_;
}

} // namespace motion_core
