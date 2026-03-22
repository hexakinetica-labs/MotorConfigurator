#include "mks/adapter/mks_runtime_config.h"

#include <fstream>
#include <vector>
#include <limits>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

using motion_core::Error;
using motion_core::ErrorCode;
using motion_core::Result;

[[nodiscard]] Result<mks::MksAxisRuntimeConfig> parse_axis_config(const json& axis_obj,
                                                                   const int bus_index,
                                                                   const int axis_index) {
    if (!axis_obj.contains("axis_id") || !axis_obj["axis_id"].is_number()) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axis_id is required and must be numeric"});
    }
    if (!axis_obj.contains("can_id") || !axis_obj["can_id"].is_number()) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "can_id is required and must be numeric"});
    }

    const int axis_id_int = axis_obj["axis_id"].get<int>();
    const int can_id_int = axis_obj["can_id"].get<int>();
    if (axis_id_int <= 0 || axis_id_int > std::numeric_limits<std::uint16_t>::max()) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axis_id is out of range"});
    }
    if (can_id_int <= 0 || can_id_int > std::numeric_limits<std::uint16_t>::max()) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "can_id is out of range"});
    }

    mks::MksAxisRuntimeConfig axis_cfg{};
    axis_cfg.axis_id = motion_core::AxisId{static_cast<std::uint16_t>(axis_id_int)};
    axis_cfg.can_id = static_cast<std::uint16_t>(can_id_int);

    if (axis_obj.contains("axis_name") && axis_obj["axis_name"].is_string()) {
        axis_cfg.axis_name = motion_core::AxisName{axis_obj["axis_name"].get<std::string>()};
    } else {
        axis_cfg.axis_name = motion_core::AxisName{"axis_" + std::to_string(axis_cfg.axis_id.value)};
    }

    if (axis_obj.contains("axis_units_per_degree") && axis_obj["axis_units_per_degree"].is_number()) {
        axis_cfg.axis_units_per_degree = axis_obj["axis_units_per_degree"].get<double>();
    }
    if (axis_obj.contains("default_speed") && axis_obj["default_speed"].is_number()) {
        const int speed = axis_obj["default_speed"].get<int>();
        if (speed < 0 || speed > std::numeric_limits<std::uint16_t>::max()) {
            return Result<mks::MksAxisRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "default_speed is out of range"});
        }
        axis_cfg.default_speed = static_cast<std::uint16_t>(speed);
    }
    if (axis_obj.contains("default_accel") && axis_obj["default_accel"].is_number()) {
        const int accel = axis_obj["default_accel"].get<int>();
        if (accel < 0 || accel > std::numeric_limits<std::uint8_t>::max()) {
            return Result<mks::MksAxisRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "default_accel is out of range"});
        }
        axis_cfg.default_accel = static_cast<std::uint8_t>(accel);
    }

    if (axis_cfg.axis_name.value.empty()) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axis_name resolved to empty string"});
    }

    if (axis_cfg.axis_units_per_degree > -1e-9 && axis_cfg.axis_units_per_degree < 1e-9) {
        return Result<mks::MksAxisRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axis_units_per_degree must be non-zero"});
    }

    (void)bus_index;
    (void)axis_index;
    return Result<mks::MksAxisRuntimeConfig>::success(axis_cfg);
}

[[nodiscard]] Result<mks::MksBusRuntimeConfig> parse_bus_config(const json& bus_obj, const int bus_index) {
    if (!bus_obj.contains("device_path") || !bus_obj["device_path"].is_string()) {
        return Result<mks::MksBusRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "device_path is required and must be string"});
    }
    if (!bus_obj.contains("axes") || !bus_obj["axes"].is_array()) {
        return Result<mks::MksBusRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axes is required and must be array"});
    }

    mks::MksBusRuntimeConfig bus_cfg{};
    bus_cfg.device_path = bus_obj["device_path"].get<std::string>();
    if (bus_cfg.device_path.empty()) {
        return Result<mks::MksBusRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "device_path cannot be empty"});
    }

    if (bus_obj.contains("interface_id") && bus_obj["interface_id"].is_string()) {
        bus_cfg.interface_id = bus_obj["interface_id"].get<std::string>();
    } else {
        bus_cfg.interface_id = "mks_bus_" + std::to_string(bus_index);
    }

    if (bus_obj.contains("baud_rate") && bus_obj["baud_rate"].is_number()) {
        const auto baud = bus_obj["baud_rate"].get<int>();
        if (baud <= 0) {
            return Result<mks::MksBusRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "baud_rate must be > 0"});
        }
        bus_cfg.baud_rate = static_cast<unsigned int>(baud);
    }

    if (bus_obj.contains("cycle_time_ms") && bus_obj["cycle_time_ms"].is_number()) {
        const int cycle = bus_obj["cycle_time_ms"].get<int>();
        if (cycle <= 0) {
            return Result<mks::MksBusRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "cycle_time_ms must be > 0"});
        }
        bus_cfg.cycle_time_ms = cycle;
    }

    const auto& axes_array = bus_obj["axes"];
    if (axes_array.empty()) {
        return Result<mks::MksBusRuntimeConfig>::failure(
            {ErrorCode::InvalidArgument, "axes array cannot be empty"});
    }

    bus_cfg.axes.reserve(axes_array.size());
    int axis_index = 0;
    for (const auto& axis_value : axes_array) {
        if (!axis_value.is_object()) {
            return Result<mks::MksBusRuntimeConfig>::failure(
                {ErrorCode::InvalidArgument, "axis entry must be object"});
        }

        const auto axis_result = parse_axis_config(axis_value, bus_index, axis_index);
        if (!axis_result.ok()) {
            return Result<mks::MksBusRuntimeConfig>::failure(axis_result.error());
        }

        bus_cfg.axes.push_back(axis_result.value());
        ++axis_index;
    }

    return Result<mks::MksBusRuntimeConfig>::success(std::move(bus_cfg));
}

} // namespace

namespace mks {

Result<MksRuntimeConfig> load_mks_runtime_config_from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::NotFound, "failed to open runtime config file"});
    }

    const json root = json::parse(file, nullptr, false);
    if (root.is_discarded()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "json parse error"});
    }

    if (!root.is_object()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "runtime config root must be object"});
    }

    if (!root.contains("buses") || !root["buses"].is_array()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "buses array is required"});
    }

    const auto& buses_array = root["buses"];
    if (buses_array.empty()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "buses array cannot be empty"});
    }

    MksRuntimeConfig config{};
    config.buses.reserve(buses_array.size());

    int bus_index = 0;
    for (const auto& bus_value : buses_array) {
        if (!bus_value.is_object()) {
            return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "bus entry must be object"});
        }

        const auto bus_result = parse_bus_config(bus_value, bus_index);
        if (!bus_result.ok()) {
            return Result<MksRuntimeConfig>::failure(bus_result.error());
        }
        config.buses.push_back(bus_result.value());
        ++bus_index;
    }

    return Result<MksRuntimeConfig>::success(std::move(config));
}

} // namespace mks
