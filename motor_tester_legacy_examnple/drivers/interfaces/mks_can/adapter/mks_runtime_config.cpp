#include "mks_can/adapter/mks_runtime_config.h"

#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

using motion_core::ErrorCode;
using motion_core::Result;

[[nodiscard]] Result<mks::MksAxisRuntimeConfig> parse_axis_config(const json& axis_obj) {
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
    if (can_id_int <= 0 || can_id_int > 0x7FF) {
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
    return Result<mks::MksAxisRuntimeConfig>::success(axis_cfg);
}

} // namespace

namespace mks {

Result<MksRuntimeConfig> load_mks_runtime_config_from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::NotFound, "failed to open runtime config file"});
    }

    const json root = json::parse(file, nullptr, false);
    if (root.is_discarded() || !root.is_object()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "json parse error"});
    }
    if (!root.contains("buses") || !root["buses"].is_array() || root["buses"].empty()) {
        return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "buses array is required"});
    }

    MksRuntimeConfig config{};
    for (const auto& bus_obj : root["buses"]) {
        if (!bus_obj.is_object()) {
            return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "bus entry must be object"});
        }
        if (!bus_obj.contains("device_path") || !bus_obj["device_path"].is_string()) {
            return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "device_path is required"});
        }

        MksBusRuntimeConfig bus_cfg{};
        bus_cfg.device_path = bus_obj["device_path"].get<std::string>();
        bus_cfg.interface_id = bus_obj.contains("interface_id") && bus_obj["interface_id"].is_string()
            ? bus_obj["interface_id"].get<std::string>()
            : bus_cfg.device_path;

        if (bus_obj.contains("baud_rate") && bus_obj["baud_rate"].is_number()) {
            const int baud = bus_obj["baud_rate"].get<int>();
            if (baud <= 0) {
                return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "baud_rate must be > 0"});
            }
            bus_cfg.baud_rate = static_cast<unsigned int>(baud);
        }
        if (bus_obj.contains("cycle_time_us") && bus_obj["cycle_time_us"].is_number()) {
            const int cycle_us = bus_obj["cycle_time_us"].get<int>();
            if (cycle_us <= 0) {
                return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "cycle_time_us must be > 0"});
            }
            bus_cfg.cycle_time_us = cycle_us;
        } else if (bus_obj.contains("cycle_time_ms") && bus_obj["cycle_time_ms"].is_number()) {
            const int cycle_ms = bus_obj["cycle_time_ms"].get<int>();
            if (cycle_ms <= 0) {
                return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "cycle_time_ms must be > 0"});
            }
            bus_cfg.cycle_time_us = cycle_ms * 1000;
        }

        if (!bus_obj.contains("axes") || !bus_obj["axes"].is_array() || bus_obj["axes"].empty()) {
            return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "axes array is required"});
        }

        for (const auto& axis_obj : bus_obj["axes"]) {
            if (!axis_obj.is_object()) {
                return Result<MksRuntimeConfig>::failure({ErrorCode::InvalidArgument, "axis entry must be object"});
            }
            const auto axis_result = parse_axis_config(axis_obj);
            if (!axis_result.ok()) {
                return Result<MksRuntimeConfig>::failure(axis_result.error());
            }
            bus_cfg.axes.push_back(axis_result.value());
        }

        config.buses.push_back(std::move(bus_cfg));
    }

    return Result<MksRuntimeConfig>::success(std::move(config));
}

} // namespace mks
