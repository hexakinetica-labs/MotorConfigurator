#include "mks_can/manager/mks_topology_scanner.h"

#include "mks_can/internal/port/can_port_interface.h"
#include "mks_can/internal/port/gs_usb_can_port.h"
#include "mks_can/internal/port/sim_can_port.h"
#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace mks {

motion_core::Result<MksTopologyDiscoveryResult> discover_mks_topology(
    const std::string& device_path,
    unsigned int baud_rate,
    int max_id) {

    if (device_path.empty()) {
        return motion_core::Result<MksTopologyDiscoveryResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "device_path is empty"});
    }
    if (max_id < 1 || max_id > 2047) {
        return motion_core::Result<MksTopologyDiscoveryResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "scan max id must be in [1, 2047]"});
    }

    std::unique_ptr<ICanPort> can_port;
    // Simple check for "sim" prefix
    if (device_path.size() >= 3 && 
        (device_path.substr(0, 3) == "sim" || device_path.substr(0, 3) == "SIM")) {
        can_port = std::make_unique<SimCanPort>();
    } else {
        can_port = std::make_unique<GsUsbCanPort>();
    }

    if (!can_port->open(device_path.c_str(), baud_rate)) {
        return motion_core::Result<MksTopologyDiscoveryResult>::failure(
            {motion_core::ErrorCode::TransportFailure,
             "failed to open CAN device for scan (device busy, disconnected, or insufficient permissions)"});
    }

    MksProtocol protocol(*can_port);

    // Safety feature: disable all motors on the bus before scanning
    std::vector<std::uint8_t> disable_payload = {0x00};
    std::vector<std::uint8_t> dummy_resp;
    (void)protocol.sendCommand(0x00, MksCommand::EnableMotor, disable_payload, dummy_resp, 0xFF, 0, false);

    auto scan_once = [&](const int timeout_ms) {
        auto ids = protocol.scanBus(1, static_cast<std::uint16_t>(max_id), static_cast<unsigned int>(timeout_ms));
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    };

    auto found = scan_once(25);

    // Robustness on noisy CAN: confirm candidates with stricter timeout and keep stable intersection.
    if (found.size() > 1) {
        const auto confirm = scan_once(40);
        std::vector<std::uint16_t> stable;
        std::set_intersection(found.begin(),
                               found.end(),
                               confirm.begin(),
                               confirm.end(),
                               std::back_inserter(stable));

        if (!stable.empty()) {
            found = std::move(stable);
        }
    }

    // Final pass: query status directly to filter out IDs that only intermittently echoed.
    std::vector<std::uint16_t> verified;
    verified.reserve(found.size());
    for (const auto id : found) {
        std::vector<std::uint8_t> resp;
        if (protocol.sendCommand(id, MksCommand::QueryMotorStatus, {}, resp, 0xFF, 50, true)) {
            verified.push_back(id);
        }
    }
    found = std::move(verified);

    can_port->close();

    std::sort(found.begin(), found.end());
    found.erase(std::unique(found.begin(), found.end()), found.end());

    motion_core::HalBusConfigMks bus_cfg{};
    bus_cfg.interface_id = device_path;
    bus_cfg.device_path = device_path;
    bus_cfg.baud_rate = baud_rate;

    MksTopologyDiscoveryResult result{};
    result.bus_config = std::move(bus_cfg);

    for (const auto can_id : found) {
        motion_core::HalAxisRuntimeEntry entry{};
        entry.axis_id = motion_core::AxisId{can_id}; // Default map 1:1 for discovery
        entry.axis_name.value = "Axis " + std::to_string(can_id);
        entry.transport = motion_core::AxisTransportKind::CanBus;
        entry.bus_ref = device_path;
        entry.transport_address = can_id;
        entry.enable_on_start = false;
        result.axes.push_back(std::move(entry));
    }

    return motion_core::Result<MksTopologyDiscoveryResult>::success(std::move(result));
}

} // namespace mks
