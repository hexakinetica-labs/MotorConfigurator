#include "init_all_drivers.h"
#include "mks_can/internal/port/gs_usb_can_port.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

void printUsage() {
    std::cout << "mks_can_cli usage:\n"
                 "  mks_can_cli --list\n"
                 "  mks_can_cli --runtime-config <path.json> --control-status [--id <axis_id>]\n"
                 "  mks_can_cli --runtime-config <path.json> --control-status --set-target <axis_id> <deg>\n"
                 "  mks_can_cli --runtime-config <path.json> --control-status --set-persistent <axis_id> <motor_type|can_id|can_bitrate> <value>\n"
                 "  mks_can_cli --device usb:BUS:ADDR --id <can_id> --control-status\n"
                 "\n"
                 "Note: direct IAxis + BusManager runtime path is used.\n";
}

[[nodiscard]] bool parse_int_in_range(const std::string& text,
                                      const int min_value,
                                      const int max_value,
                                      int& out_value) {
    long long parsed_value = 0;
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto parse_result = std::from_chars(begin, end, parsed_value);
    if (parse_result.ec != std::errc() || parse_result.ptr != end) {
        return false;
    }
    if (parsed_value < static_cast<long long>(min_value)
        || parsed_value > static_cast<long long>(max_value)) {
        return false;
    }
    out_value = static_cast<int>(parsed_value);
    return true;
}

[[nodiscard]] bool parse_double_strict(const std::string& text, double& out_value) {
    std::istringstream stream(text);
    stream >> std::noskipws >> out_value;
    if (stream.fail() || !stream.eof()) {
        return false;
    }
    return std::isfinite(out_value);
}

[[nodiscard]] bool parse_int64_strict(const std::string& text, std::int64_t& out_value) {
    long long parsed_value = 0;
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto parse_result = std::from_chars(begin, end, parsed_value);
    if (parse_result.ec != std::errc() || parse_result.ptr != end) {
        return false;
    }
    out_value = static_cast<std::int64_t>(parsed_value);
    return true;
}

[[nodiscard]] bool parse_persistent_command(const std::string& text,
                                            motion_core::PersistentCommand& out_command) {
    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (normalized == "motor_type" || normalized == "motortype") {
        out_command = motion_core::PersistentCommand::MotorType;
        return true;
    }
    if (normalized == "can_id" || normalized == "canid") {
        out_command = motion_core::PersistentCommand::CanId;
        return true;
    }
    if (normalized == "can_bitrate" || normalized == "canbitrate") {
        out_command = motion_core::PersistentCommand::CanBitrate;
        return true;
    }
    return false;
}

[[nodiscard]] const char* to_string(const motion_core::PersistentCommand command) {
    switch (command) {
        case motion_core::PersistentCommand::MotorType:
            return "motor_type";
        case motion_core::PersistentCommand::CanId:
            return "can_id";
        case motion_core::PersistentCommand::CanBitrate:
            return "can_bitrate";
        default:
            return "unknown";
    }
}

} // namespace

int main(int argc, char* argv[]) {
    drivers::init_all_drivers();

    if (argc < 2) {
        printUsage();
        return 0;
    }

    std::vector<std::string> args;
    args.reserve(static_cast<std::size_t>(argc));
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    bool list_only = false;
    bool do_control_status = false;
    bool has_explicit_id = false;
    bool has_set_target = false;
    bool has_set_persistent = false;

    int can_id = 1;
    int baud_rate = 1000000;
    int target_axis_id = 0;
    double target_position_deg = 0.0;
    int persistent_axis_id = 0;
    motion_core::PersistentCommand persistent_command = motion_core::PersistentCommand::MotorType;
    std::int64_t persistent_raw_value = 0;
    std::string device_path;
    std::string runtime_config_path;
    std::string parse_error;

    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--list") {
            list_only = true;
        } else if (args[i] == "--runtime-config" || args[i] == "--config") {
            if (i + 1 >= args.size()) {
                parse_error = "Missing value for --runtime-config";
                break;
            }
            runtime_config_path = args[++i];
        } else if (args[i] == "--device") {
            if (i + 1 >= args.size()) {
                parse_error = "Missing value for --device";
                break;
            }
            device_path = args[++i];
        } else if (args[i] == "--id") {
            if (i + 1 >= args.size()) {
                parse_error = "Missing value for --id";
                break;
            }
            has_explicit_id = true;
            const auto& value = args[++i];
            if (!parse_int_in_range(value, 1, std::numeric_limits<std::uint16_t>::max(), can_id)) {
                parse_error = "Invalid --id: expected integer in range [1..65535]";
                break;
            }
        } else if (args[i] == "--baud" || args[i] == "--bitrate") {
            if (i + 1 >= args.size()) {
                parse_error = "Missing value for --baud/--bitrate";
                break;
            }
            const auto& value = args[++i];
            if (!parse_int_in_range(value, 1, std::numeric_limits<int>::max(), baud_rate)) {
                parse_error = "Invalid --baud/--bitrate: expected positive integer";
                break;
            }
        } else if (args[i] == "--control-status") {
            do_control_status = true;
        } else if (args[i] == "--set-target") {
            if (i + 2 >= args.size()) {
                parse_error = "Missing values for --set-target <axis_id> <deg>";
                break;
            }
            has_set_target = true;
            const auto& axis_id_text = args[++i];
            if (!parse_int_in_range(axis_id_text, 1, std::numeric_limits<std::uint16_t>::max(), target_axis_id)) {
                parse_error = "Invalid --set-target axis id: expected integer in range [1..65535]";
                break;
            }
            const auto& target_text = args[++i];
            if (!parse_double_strict(target_text, target_position_deg)) {
                parse_error = "Invalid --set-target position: expected finite floating-point value";
                break;
            }
        } else if (args[i] == "--set-persistent") {
            if (i + 3 >= args.size()) {
                parse_error = "Missing values for --set-persistent <axis_id> <command> <value>";
                break;
            }
            has_set_persistent = true;

            const auto& axis_id_text = args[++i];
            if (!parse_int_in_range(axis_id_text, 1, std::numeric_limits<std::uint16_t>::max(), persistent_axis_id)) {
                parse_error = "Invalid --set-persistent axis id: expected integer in range [1..65535]";
                break;
            }

            const auto& command_text = args[++i];
            if (!parse_persistent_command(command_text, persistent_command)) {
                parse_error = "Invalid --set-persistent command: use motor_type|can_id|can_bitrate";
                break;
            }

            const auto& value_text = args[++i];
            if (!parse_int64_strict(value_text, persistent_raw_value)) {
                parse_error = "Invalid --set-persistent value: expected integer";
                break;
            }
        }
    }

    if (has_set_target && has_set_persistent) {
        parse_error = "--set-target and --set-persistent cannot be used together";
    }

    if (!parse_error.empty()) {
        std::cerr << parse_error << "\n";
        printUsage();
        return 2;
    }

    if (list_only) {
        std::cout << "sim:default : Virtual Simulator CAN Device\n";
        const auto devs = mks::GsUsbCanPort::enumerateDevices();
        for (const auto& d : devs) {
            std::cout << d.path << " : " << d.description << "\n";
        }
        return 0;
    }

    if (!do_control_status) {
        std::cerr << "Only --control-status mode is supported\n";
        printUsage();
        return 2;
    }

    if (device_path.empty() && runtime_config_path.empty()) {
        std::cerr << "--device or --runtime-config is required\n";
        printUsage();
        return 2;
    }

    motion_core::HalRuntime runtime{};

    if (!runtime_config_path.empty()) {
        const auto config_result = motion_core::load_hal_runtime_config_from_file(runtime_config_path);
        if (!config_result.ok()) {
            std::cerr << "Runtime config load failed: " << config_result.error().message << "\n";
            return 12;
        }

        const auto runtime_result = runtime.open_from_config(config_result.value());
        if (!runtime_result.ok()) {
            std::cerr << "Runtime open failed: " << runtime_result.error().message << "\n";
            return 13;
        }
    } else {
        if (can_id > 2047) {
            std::cerr << "Invalid --id for direct MKS mode: expected CAN ID in range [1..2047]\n";
            return 21;
        }

        motion_core::HalRuntimeConfig cfg{};
        motion_core::HalBusConfigMks bus{};
        bus.interface_id = "cli_mks_bus_0";
        bus.device_path = device_path;
        bus.baud_rate = static_cast<std::uint32_t>(baud_rate);
        cfg.mks_buses.push_back(bus);

        motion_core::HalAxisRuntimeEntry axis{};
        axis.axis_id = motion_core::AxisId{static_cast<std::uint16_t>(can_id)};
        axis.axis_name = motion_core::AxisName{"mks_axis_" + std::to_string(can_id)};
        axis.transport = motion_core::AxisTransportKind::CanBus;
        axis.bus_ref = bus.interface_id;
        axis.transport_address = static_cast<std::uint16_t>(can_id);
        axis.enable_on_start = true;
        cfg.axes.push_back(axis);

        const auto runtime_result = runtime.open_from_config(cfg);
        if (!runtime_result.ok()) {
            std::cerr << "Runtime open failed: " << runtime_result.error().message << "\n";
            return 14;
        }
    }

    std::vector<motion_core::AxisId> target_axes;
    if (has_explicit_id) {
        target_axes.push_back(motion_core::AxisId{static_cast<std::uint16_t>(can_id)});
    } else {
        const auto listed = runtime.list_axes();
        if (!listed.ok()) {
            std::cerr << "List axes failed: " << listed.error().message << "\n";
            return 15;
        }
        for (const auto& info : listed.value()) {
            target_axes.push_back(info.id);
        }
        std::sort(target_axes.begin(), target_axes.end(), [](const motion_core::AxisId lhs, const motion_core::AxisId rhs) {
            return lhs.value < rhs.value;
        });
    }

    const auto start_runtime = runtime.start();
    if (!start_runtime.ok()) {
        std::cerr << "Runtime start failed: " << start_runtime.error().message << "\n";
        return 16;
    }

    if (has_set_target) {
        const auto axis_res = runtime.find_axis(static_cast<std::uint16_t>(target_axis_id));
        if (!axis_res.ok()) {
            std::cerr << "Set target failed: axis not found\n";
            (void)runtime.stop();
            (void)runtime.close();
            return 19;
        }

        motion_core::AxisCommand target_cmd{};
        target_cmd.has_target_position = true;
        target_cmd.target_position_deg = target_position_deg;
        const auto apply_cmd_res = axis_res.value()->apply_command(target_cmd);
        if (!apply_cmd_res.ok()) {
            std::cerr << "Set target failed: " << apply_cmd_res.error().message << "\n";
            (void)runtime.stop();
            (void)runtime.close();
            return 20;
        }
    }

    if (has_set_persistent) {
        const auto axis_res = runtime.find_axis(static_cast<std::uint16_t>(persistent_axis_id));
        if (!axis_res.ok()) {
            std::cerr << "Set persistent failed: axis not found\n";
            (void)runtime.stop();
            (void)runtime.close();
            return 22;
        }

        const auto persistent_result = axis_res.value()->set_persistent(
            persistent_command,
            motion_core::ParameterValue::from_signed(persistent_raw_value));
        if (!persistent_result.ok()) {
            std::cerr << "Set persistent failed: " << persistent_result.error().message << "\n";
            (void)runtime.stop();
            (void)runtime.close();
            return 23;
        }

        const auto& report = persistent_result.value();
        std::cout << "persistent axis=" << persistent_axis_id
                  << " command=" << to_string(persistent_command)
                  << " write_completed=" << (report.write_completed ? "true" : "false")
                  << " save_completed=" << (report.persistent_save_completed ? "true" : "false")
                  << " readback_verified=" << (report.readback_verified ? "true" : "false")
                  << " reconnect_required=" << (report.reconnect_required ? "true" : "false")
                  << " power_cycle_required=" << (report.power_cycle_required ? "true" : "false")
                  << "\n";
    }

    std::vector<motion_core::AxisId> started_axes;
    for (const auto axis_id : target_axes) {
        const auto axis_res = runtime.find_axis(axis_id.value);
        if (!axis_res.ok()) {
            continue;
        }
        started_axes.push_back(axis_id);
    }

    if (started_axes.empty()) {
        std::cerr << "No axis was started via control service\n";
        (void)runtime.stop();
        (void)runtime.close();
        return 10;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    for (int i = 0; i < 5; ++i) {
        for (const auto axis_id : started_axes) {
            const auto axis_res = runtime.find_axis(axis_id.value);
            if (!axis_res.ok()) {
                continue;
            }
            const auto telemetry_result = axis_res.value()->read_telemetry();
            if (!telemetry_result.ok()) {
                std::cerr << "Telemetry failed for axis " << axis_id.value << ": "
                          << telemetry_result.error().message << "\n";
                continue;
            }

            const auto& telemetry = telemetry_result.value();
            std::cout << "telemetry axis=" << axis_id.value
                      << " pos_deg=" << telemetry.actual_position_deg
                      << " vel_deg_s=" << telemetry.actual_velocity_deg_per_sec
                      << " status_word=" << telemetry.status_word
                      << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    (void)runtime.stop();
    (void)runtime.close();

    return 0;
}
