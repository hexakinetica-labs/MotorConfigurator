#include "motion_core/hal_runtime.h"

#include "motion_core/config/axis_config_json.h"
#include "motion_core/runtime_factory_registry.h"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <unordered_map>

namespace motion_core {

namespace {

[[nodiscard]] std::uint64_t make_parameter_key(const ParameterId id) {
    return (static_cast<std::uint64_t>(id.domain) << 32U) | static_cast<std::uint64_t>(id.value);
}

[[nodiscard]] const char* parameter_domain_name(const ParameterDomain domain) {
    switch (domain) {
        case ParameterDomain::Common: return "Common";
        case ParameterDomain::Ethercat: return "EtherCAT";
        case ParameterDomain::Mks: return "MKS";
    }
    return "Unknown";
}

[[nodiscard]] bool starts_with(const std::string& value, const std::string_view prefix) {
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix.data(), prefix.size()) == 0;
}

[[nodiscard]] std::string strip_group_root(const char* raw_group) {
    std::string group = raw_group ? std::string(raw_group) : std::string{};
    if (starts_with(group, "Common/")) {
        return group.substr(7);
    }
    if (starts_with(group, "Software Defined HAL/")) {
        return group.substr(21);
    }
    if (starts_with(group, "Drive/EtherCAT/")) {
        return group.substr(15);
    }
    if (starts_with(group, "Drive/MKS/")) {
        return group.substr(10);
    }
    if (starts_with(group, "Drive/")) {
        return group.substr(6);
    }
    return group;
}

[[nodiscard]] bool is_ethercat_software_defined_parameter(const ParameterId id) {
    if (id.domain != ParameterDomain::Common) {
        return false;
    }

    return id.value == static_cast<std::uint32_t>(CommonParameter::HardwareGearRatio)
        || id.value == static_cast<std::uint32_t>(CommonParameter::HomeSwitchToZeroShiftDeg);
}

[[nodiscard]] std::string build_parameter_group_for_display(const AxisTransportKind transport,
                                                            const ParameterDescriptor& descriptor) {
    const std::string suffix = strip_group_root(descriptor.group);
    const auto append_suffix = [&suffix](const std::string& root) {
        return suffix.empty() ? root : root + "/" + suffix;
    };

    if (transport == AxisTransportKind::CanBus) {
        if (descriptor.id.domain == ParameterDomain::Common) {
            return append_suffix("Software Defined HAL");
        }
        if (descriptor.group && descriptor.group[0] != '\0') {
            return descriptor.group;
        }
        return append_suffix("Drive/MKS");
    }

    if (transport == AxisTransportKind::Ethercat) {
        if (is_ethercat_software_defined_parameter(descriptor.id)) {
            return append_suffix("Software Defined HAL");
        }
        return append_suffix("Drive/EtherCAT");
    }

    if (descriptor.group && descriptor.group[0] != '\0') {
        return descriptor.group;
    }
    return "Ungrouped";
}

[[nodiscard]] bool is_transport_breaking_parameter(const ParameterDescriptor& descriptor) {
    const std::string_view name = descriptor.name ? descriptor.name : "";
    return name == "CAN ID" || name == "CAN Bitrate Index" || name == "Fixed address";
}

[[nodiscard]] bool should_apply_loaded_parameter(const ParameterDescriptor& descriptor) {
    if (descriptor.read_only) {
        return false;
    }
    if (descriptor.persistable) {
        return true;
    }
    return is_transport_breaking_parameter(descriptor);
}

[[nodiscard]] bool parameter_values_equal(const ParameterValue& lhs, const ParameterValue& rhs) {
    if (lhs.type == rhs.type) {
        switch (lhs.type) {
            case ParameterValueType::SignedInteger: return lhs.signed_value == rhs.signed_value;
            case ParameterValueType::UnsignedInteger: return lhs.unsigned_value == rhs.unsigned_value;
            case ParameterValueType::FloatingPoint: return std::abs(lhs.floating_value - rhs.floating_value) <= 1e-9;
            case ParameterValueType::Boolean: return lhs.bool_value == rhs.bool_value;
        }
    }

    const auto to_numeric = [](const ParameterValue& value) -> double {
        switch (value.type) {
            case ParameterValueType::SignedInteger: return static_cast<double>(value.signed_value);
            case ParameterValueType::UnsignedInteger: return static_cast<double>(value.unsigned_value);
            case ParameterValueType::FloatingPoint: return value.floating_value;
            case ParameterValueType::Boolean: return value.bool_value ? 1.0 : 0.0;
        }
        return 0.0;
    };

    return std::abs(to_numeric(lhs) - to_numeric(rhs)) <= 1e-9;
}

} // namespace

HalRuntime::HalRuntime() = default;

HalRuntime::~HalRuntime() = default;

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
        std::scoped_lock lock(mutex_);
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
        std::scoped_lock lock(mutex_);
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
        std::scoped_lock lock(mutex_);
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
    std::vector<std::shared_ptr<IAxis>> started_axes;

    const auto rollback_started = [&started_buses, &started_axes]() {
        for (auto it = started_axes.rbegin(); it != started_axes.rend(); ++it) {
            ServiceCommandPoint cmd{};
            cmd.type = ServiceCommandType::Disable;
            cmd.axis_id = (*it)->info().id.value;
            (void)(*it)->enqueueServicePoint(cmd);
        }
        for (auto it = started_buses.rbegin(); it != started_buses.rend(); ++it) {
            (void)(*it)->stop();
        }
    };

    for (const auto& bus : buses_snapshot) {
        if (!bus) {
            continue;
        }
        const auto started = bus->start();
        if (!started.ok()) {
            rollback_started();
            return started;
        }
        started_buses.push_back(bus);
    }

    for (const auto& [id, axis] : axes_snapshot) {
        if (!axis) {
            continue;
        }
        started_axes.push_back(axis);

        const auto policy_it = enable_on_start_policy.find(id);
        if (policy_it != enable_on_start_policy.end() && policy_it->second) {
            ServiceCommandPoint cmd{};
            cmd.type = ServiceCommandType::Enable;
            cmd.axis_id = id;
            if (!axis->enqueueServicePoint(cmd)) {
                rollback_started();
                return Result<void>::failure({ErrorCode::Busy, "Failed to enqueue initial enable command"});
            }
        }
    }

    {
        std::scoped_lock lock(mutex_);
        if (!runtime_open_) {
            rollback_started();
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
        std::scoped_lock lock(mutex_);
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
        // IAxis no longer has stop(); could enqueue Disable but axes are shut down when bus stops.
        ServiceCommandPoint cmd{};
        cmd.type = ServiceCommandType::Disable;
        cmd.axis_id = axis->info().id.value;
        (void)axis->enqueueServicePoint(cmd);
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
    std::scoped_lock lock(mutex_);
    axes_.clear();
    buses_.clear();
    runtime_config_ = {};
    runtime_open_ = false;
    runtime_active_ = false;
    return Result<void>::success();
}

Result<std::shared_ptr<IAxis>> HalRuntime::find_axis(const std::uint16_t axis_id) const {
    std::scoped_lock lock(mutex_);
    const auto it = axes_.find(axis_id);
    if (it == axes_.end() || !it->second) {
        return Result<std::shared_ptr<IAxis>>::failure({ErrorCode::NotFound, "axis not found"});
    }
    return Result<std::shared_ptr<IAxis>>::success(it->second);
}

Result<std::vector<AxisInfo>> HalRuntime::list_axes() const {
    std::vector<AxisInfo> out;
    {
        std::scoped_lock lock(mutex_);
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
    std::scoped_lock lock(mutex_);
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
    cfg.version = 2;

    std::unordered_map<std::uint64_t, ParameterDescriptor> descriptors_by_key;
    descriptors_by_key.reserve(desc_res.value().size());
    for (const auto& descriptor : desc_res.value()) {
        descriptors_by_key.emplace(make_parameter_key(descriptor.id), descriptor);
    }

    for (const auto& entry : read_res.value().entries) {
        cfg.parameters.entries.push_back(entry);

        AxisConfigParameterRecord record{};
        record.entry = entry;
        if (const auto it = descriptors_by_key.find(make_parameter_key(entry.id)); it != descriptors_by_key.end()) {
            const auto& descriptor = it->second;
            record.meta.domain_name = parameter_domain_name(entry.id.domain);
            record.meta.name = descriptor.name ? descriptor.name : "";
            record.meta.group = build_parameter_group_for_display(info.transport, descriptor);
            record.meta.unit = descriptor.unit ? descriptor.unit : "";
            record.meta.read_only = descriptor.read_only;
            record.meta.persistable = descriptor.persistable;
        } else {
            record.meta.domain_name = parameter_domain_name(entry.id.domain);
        }
        cfg.parameter_records.push_back(std::move(record));
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

    const auto current_res = axis_res.value()->read_parameters();
    if (!current_res.ok()) {
        return Result<ParameterPatch>::failure(current_res.error());
    }

    ParameterPatch patch{};

    std::unordered_map<std::uint64_t, ParameterDescriptor> descriptors_by_key;
    descriptors_by_key.reserve(desc_res.value().size());
    for (const auto& descriptor : desc_res.value()) {
        descriptors_by_key.emplace(make_parameter_key(descriptor.id), descriptor);
    }

    std::unordered_map<std::uint64_t, ParameterValue> current_by_key;
    current_by_key.reserve(current_res.value().entries.size());
    for (const auto& entry : current_res.value().entries) {
        current_by_key.emplace(make_parameter_key(entry.id), entry.value);
    }

    std::vector<ParameterEntry> regular_entries;
    std::vector<ParameterEntry> late_entries;
    regular_entries.reserve(load_res.value().parameters.entries.size());

    for (const auto& entry : load_res.value().parameters.entries) {
        const auto descriptor_it = descriptors_by_key.find(make_parameter_key(entry.id));
        if (descriptor_it == descriptors_by_key.end()) {
            continue;
        }

        const auto& descriptor = descriptor_it->second;
        if (!should_apply_loaded_parameter(descriptor)) {
            continue;
        }

        const auto current_it = current_by_key.find(make_parameter_key(entry.id));
        if (current_it != current_by_key.end() && parameter_values_equal(current_it->second, entry.value)) {
            continue;
        }

        if (is_transport_breaking_parameter(descriptor)) {
            late_entries.push_back(entry);
        } else {
            regular_entries.push_back(entry);
        }
    }

    patch.entries.reserve(regular_entries.size() + late_entries.size());
    patch.entries.insert(patch.entries.end(), regular_entries.begin(), regular_entries.end());
    patch.entries.insert(patch.entries.end(), late_entries.begin(), late_entries.end());

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
    std::scoped_lock lock(mutex_);
    return runtime_open_;
}

bool HalRuntime::is_active() const {
    std::scoped_lock lock(mutex_);
    return runtime_active_;
}

} // namespace motion_core
