#include "motion_core/config/hal_runtime_config_json.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <limits>
#include <string>

using json = nlohmann::json;

namespace motion_core {

namespace {

[[nodiscard]] Result<void> require_object_field(const json& j, const char* field_name, const char* error_message) {
    if (!j.contains(field_name)) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    if (!j.at(field_name).is_object()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> require_array_field(const json& j, const char* field_name, const char* error_message) {
    if (!j.contains(field_name)) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    if (!j.at(field_name).is_array()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> require_string_field(const json& j, const char* field_name, const char* error_message) {
    if (!j.contains(field_name)) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    if (!j.at(field_name).is_string()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> require_uint_field(const json& j, const char* field_name, const char* error_message) {
    if (!j.contains(field_name)) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    if (!j.at(field_name).is_number_unsigned()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> require_uint_field_max(const json& j,
                                                  const char* field_name,
                                                  const char* error_message,
                                                  const std::uint64_t max_value) {
    const auto req = require_uint_field(j, field_name, error_message);
    if (!req.ok()) {
        return req;
    }

    const auto value = j.at(field_name).get<std::uint64_t>();
    if (value > max_value) {
        return Result<void>::failure({ErrorCode::InvalidArgument, error_message});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<AxisTransportKind> parse_transport_kind(const json& transport) {
    if (!transport.is_string()) {
        return Result<AxisTransportKind>::failure(
            {ErrorCode::InvalidArgument, "axis transport must be string"});
    }

    const auto value = transport.get<std::string>();
    if (value == "mks_can") {
        return Result<AxisTransportKind>::success(AxisTransportKind::CanBus);
    }
    if (value == "ethercat") {
        return Result<AxisTransportKind>::success(AxisTransportKind::Ethercat);
    }
    if (value == "unknown") {
        return Result<AxisTransportKind>::success(AxisTransportKind::Unknown);
    }
    return Result<AxisTransportKind>::failure(
        {ErrorCode::InvalidArgument, "unknown axis transport string"});
}

[[nodiscard]] const char* transport_kind_to_string(const AxisTransportKind kind) {
    switch (kind) {
        case AxisTransportKind::CanBus:
            return "mks_can";
        case AxisTransportKind::Ethercat:
            return "ethercat";
        case AxisTransportKind::Unknown:
            return "unknown";
    }
    return "unknown";
}

} // namespace

void to_json(json& j, const HalBusConfigMks& b) {
    j = json{
        {"interface_id", b.interface_id},
        {"device_path", b.device_path},
        {"baud_rate", b.baud_rate},
    };
}
void from_json(const json& j, HalBusConfigMks& b) {
    b.interface_id = j.at("interface_id").get<std::string>();
    b.device_path = j.at("device_path").get<std::string>();
    b.baud_rate = static_cast<std::uint32_t>(j.at("baud_rate").get<std::uint64_t>());
}

void to_json(json& j, const HalAxisRuntimeEntry& e) {
    j = json{
        {"axis_id", e.axis_id.value},
        {"axis_name", e.axis_name.value},
        {"transport", transport_kind_to_string(e.transport)},
        {"bus_ref", e.bus_ref},
        {"transport_address", e.transport_address},
        {"config_file", e.config_file},
        {"enable_on_start", e.enable_on_start},
    };
}
void from_json(const json& j, HalAxisRuntimeEntry& e) {
    const auto transport_res = parse_transport_kind(j.at("transport"));
    if (!transport_res.ok()) {
        e.transport = AxisTransportKind::Unknown;
    } else {
        e.transport = transport_res.value();
    }
    e.axis_id.value = static_cast<std::uint16_t>(j.at("axis_id").get<std::uint64_t>());
    e.axis_name.value = j.at("axis_name").get<std::string>();
    e.bus_ref = j.at("bus_ref").get<std::string>();
    e.transport_address = static_cast<std::uint16_t>(j.at("transport_address").get<std::uint64_t>());
    e.config_file = j.at("config_file").get<std::string>();
    e.enable_on_start = j.at("enable_on_start").get<bool>();
}

void to_json(json& j, const HalBusConfigEthercat& b) {
    j = json{{"interface_name", b.interface_name}};
    if (!b.axes.empty()) {
        j["axes"] = b.axes;
    }
}
void from_json(const json& j, HalBusConfigEthercat& b) {
    b.interface_name = j.at("interface_name").get<std::string>();
    if (j.contains("axes")) {
        b.axes = j.at("axes").get<std::vector<HalAxisRuntimeEntry>>();
    }
}


Result<HalRuntimeConfig> load_hal_runtime_config_from_file(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        return Result<HalRuntimeConfig>::failure({ErrorCode::NotFound, "failed to open hal config file"});
    }

    const json j = json::parse(f, nullptr, false);
    if (j.is_discarded()) {
        return Result<HalRuntimeConfig>::failure({ErrorCode::ProtocolFailure, "HAL config JSON syntax error"});
    }
    if (!j.is_object()) {
        return Result<HalRuntimeConfig>::failure({ErrorCode::InvalidArgument, "HAL config root must be object"});
    }

    if (const auto req = require_uint_field_max(
            j,
            "version",
            "HAL config: missing/invalid 'version'",
            static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()));
        !req.ok()) {
        return Result<HalRuntimeConfig>::failure(req.error());
    }
    if (const auto req = require_object_field(j, "runtime", "HAL config: missing/invalid 'runtime' object"); !req.ok()) {
        return Result<HalRuntimeConfig>::failure(req.error());
    }

    const auto& runtime = j.at("runtime");
    if (const auto req = require_uint_field_max(
            runtime,
            "dispatch_period_ms",
            "HAL config: missing/invalid 'runtime.dispatch_period_ms'",
            static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()));
        !req.ok()) {
        return Result<HalRuntimeConfig>::failure(req.error());
    }
    if (const auto req = require_uint_field_max(
            runtime,
            "telemetry_period_ms",
            "HAL config: missing/invalid 'runtime.telemetry_period_ms'",
            static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()));
        !req.ok()) {
        return Result<HalRuntimeConfig>::failure(req.error());
    }

    if (j.contains("mks_buses")) {
        if (!j.at("mks_buses").is_array()) {
            return Result<HalRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "mks_buses must be array"});
        }
        for (const auto& bus : j.at("mks_buses")) {
            if (!bus.is_object()) {
                return Result<HalRuntimeConfig>::failure(
                    {ErrorCode::InvalidArgument, "mks bus entry must be object"});
            }
            if (const auto req = require_string_field(bus, "interface_id", "HAL config: missing/invalid 'mks_buses[].interface_id'"); !req.ok()) {
                return Result<HalRuntimeConfig>::failure(req.error());
            }
            if (const auto req = require_string_field(bus, "device_path", "HAL config: missing/invalid 'mks_buses[].device_path'"); !req.ok()) {
                return Result<HalRuntimeConfig>::failure(req.error());
            }
            if (const auto req = require_uint_field_max(
                    bus,
                    "baud_rate",
                    "HAL config: missing/invalid 'mks_buses[].baud_rate'",
                    static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()));
                !req.ok()) {
                return Result<HalRuntimeConfig>::failure(req.error());
            }
        }
    }

    if (j.contains("ethercat_buses")) {
        if (!j.at("ethercat_buses").is_array()) {
            return Result<HalRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "ethercat_buses must be array"});
        }
        for (const auto& bus : j.at("ethercat_buses")) {
            if (!bus.is_object()) {
                return Result<HalRuntimeConfig>::failure(
                    {ErrorCode::InvalidArgument, "ethercat bus entry must be object"});
            }
            if (const auto req = require_string_field(bus, "interface_name", "HAL config: missing/invalid 'ethercat_buses[].interface_name'"); !req.ok()) {
                return Result<HalRuntimeConfig>::failure(req.error());
            }
        }
    }

    if (const auto req = require_array_field(j, "axes", "HAL config: missing/invalid 'axes' array"); !req.ok()) {
        return Result<HalRuntimeConfig>::failure(req.error());
    }
    for (const auto& axis : j.at("axes")) {
        if (!axis.is_object()) {
            return Result<HalRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "axis entry must be object"});
        }
        if (const auto req = require_uint_field_max(
                axis,
                "axis_id",
                "HAL config: missing/invalid 'axes[].axis_id'",
                static_cast<std::uint64_t>(std::numeric_limits<std::uint16_t>::max()));
            !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (const auto req = require_string_field(axis, "axis_name", "HAL config: missing/invalid 'axes[].axis_name'"); !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (const auto req = require_string_field(axis, "transport", "HAL config: missing/invalid 'axes[].transport'"); !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (!parse_transport_kind(axis.at("transport")).ok()) {
            return Result<HalRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "HAL config: unknown 'axes[].transport' value"});
        }
        if (const auto req = require_string_field(axis, "bus_ref", "HAL config: missing/invalid 'axes[].bus_ref'"); !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (const auto req = require_uint_field_max(
                axis,
                "transport_address",
                "HAL config: missing/invalid 'axes[].transport_address'",
                static_cast<std::uint64_t>(std::numeric_limits<std::uint16_t>::max()));
            !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (const auto req = require_string_field(axis, "config_file", "HAL config: missing/invalid 'axes[].config_file'"); !req.ok()) {
            return Result<HalRuntimeConfig>::failure(req.error());
        }
        if (!axis.contains("enable_on_start") || !axis.at("enable_on_start").is_boolean()) {
            return Result<HalRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "HAL config: missing/invalid 'axes[].enable_on_start'"});
        }
    }

    HalRuntimeConfig cfg;
    cfg.version = static_cast<std::uint32_t>(j.at("version").get<std::uint64_t>());
    cfg.runtime.dispatch_period_ms = static_cast<std::uint32_t>(runtime.at("dispatch_period_ms").get<std::uint64_t>());
    cfg.runtime.telemetry_period_ms = static_cast<std::uint32_t>(runtime.at("telemetry_period_ms").get<std::uint64_t>());
    if (j.contains("mks_buses")) {
        cfg.mks_buses = j.at("mks_buses").get<std::vector<HalBusConfigMks>>();
    }
    if (j.contains("ethercat_buses")) {
        cfg.ethercat_buses = j.at("ethercat_buses").get<std::vector<HalBusConfigEthercat>>();
    }
    cfg.axes = j.at("axes").get<std::vector<HalAxisRuntimeEntry>>();

    return Result<HalRuntimeConfig>::success(std::move(cfg));
}

Result<void> save_hal_runtime_config_to_file(const std::string& path, const HalRuntimeConfig& config) {
    for (const auto& axis : config.axes) {
        if (axis.axis_name.value.empty()) {
            return Result<void>::failure({ErrorCode::InvalidArgument, "axis_name cannot be empty"});
        }
        if (axis.bus_ref.empty()) {
            return Result<void>::failure({ErrorCode::InvalidArgument, "axis bus_ref cannot be empty"});
        }
        if (axis.transport == AxisTransportKind::Unknown) {
            return Result<void>::failure({ErrorCode::InvalidArgument, "axis transport cannot be unknown"});
        }
    }

    json j;
    j["version"] = config.version;
    j["runtime"]["dispatch_period_ms"] = config.runtime.dispatch_period_ms;
    j["runtime"]["telemetry_period_ms"] = config.runtime.telemetry_period_ms;
    j["mks_buses"] = config.mks_buses;
    j["ethercat_buses"] = config.ethercat_buses;
    j["axes"] = config.axes;

    std::ofstream f(path);
    if (!f.is_open()) {
        return Result<void>::failure({ErrorCode::InternalError, "failed to write hal config file"});
    }
    f << j.dump(4);
    return Result<void>::success();
}

} // namespace motion_core
