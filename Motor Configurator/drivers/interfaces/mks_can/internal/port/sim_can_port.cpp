#include "mks_can/internal/port/sim_can_port.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

namespace mks {

namespace {

[[nodiscard]] CanFrame make_frame(const std::uint16_t can_id, const std::vector<std::uint8_t>& payload) {
    CanFrame out{};
    out.id = can_id;
    out.dlc = static_cast<std::uint8_t>(std::min<std::size_t>(payload.size(), 8));
    std::fill(std::begin(out.data), std::end(out.data), 0);
    std::memcpy(out.data, payload.data(), out.dlc);
    return out;
}

} // namespace

SimCanPort::SimCanPort() = default;

bool SimCanPort::open(const char* channel, const unsigned int baud_rate) {
    std::lock_guard<std::mutex> lock(mutex_);

    channel_ = channel ? channel : "sim";
    baud_rate_ = baud_rate;
    opened_ = true;
    rx_queue_.clear();
    axes_.clear();

    for (std::uint16_t id = 1; id <= 4; ++id) {
        SimAxisState axis{};
        axis.can_id = id;
        axis.axis_position = static_cast<std::int64_t>(id) * 1000;
        axis.speed_rpm = 0;
        axis.status = 0x01;
        axis.speed_mode_active = false;
        axis.speed_mode_last_update = std::chrono::steady_clock::now();
        axes_.emplace(id, axis);
    }

    return true;
}

void SimCanPort::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    opened_ = false;
    rx_queue_.clear();
}

bool SimCanPort::isOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return opened_;
}

bool SimCanPort::write(const CanFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!opened_) {
        return false;
    }
    return handle_frame(frame);
}

bool SimCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    const auto start = std::chrono::steady_clock::now();

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!opened_) {
                return false;
            }
            if (!rx_queue_.empty()) {
                frame = rx_queue_.front();
                rx_queue_.pop_front();
                return true;
            }
        }

        if (timeout_ms == 0) {
            return false;
        }
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (elapsed_ms >= timeout_ms) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

std::uint8_t SimCanPort::compute_crc(const std::uint16_t can_id, const std::vector<std::uint8_t>& bytes_without_crc) {
    return MksProtocol::computeCrc(can_id, bytes_without_crc);
}

void SimCanPort::push_response(const CanFrame& frame) {
    rx_queue_.push_back(frame);
}

bool SimCanPort::handle_frame(const CanFrame& frame) {
    if (frame.id > 0x7FF || frame.dlc < 2 || frame.dlc > 8) {
        return false;
    }

    const std::uint16_t can_id = static_cast<std::uint16_t>(frame.id);
    auto axis_it = axes_.find(can_id);
    if (axis_it == axes_.end()) {
        return true; // emulate no response (device absent)
    }

    std::vector<std::uint8_t> req(frame.data, frame.data + frame.dlc);
    const std::uint8_t got_crc = req.back();
    req.pop_back();
    const std::uint8_t expected_crc = compute_crc(can_id, req);
    if (got_crc != expected_crc || req.empty()) {
        return true;
    }

    const std::uint8_t cmd = req[0];
    auto& axis = axis_it->second;

    update_motion(axis);

    auto reply_with_cmd = [&](const std::uint8_t response_cmd,
                              const std::vector<std::uint8_t>& data_after_cmd) {
        std::vector<std::uint8_t> payload;
        payload.reserve(1 + data_after_cmd.size() + 1);
        payload.push_back(response_cmd);
        payload.insert(payload.end(), data_after_cmd.begin(), data_after_cmd.end());
        payload.push_back(compute_crc(can_id, payload));
        push_response(make_frame(can_id, payload));
    };

    auto reply = [&](const std::vector<std::uint8_t>& data_after_cmd) {
        reply_with_cmd(cmd, data_after_cmd);
    };

    switch (cmd) {
        case 0xF1: { // QueryMotorStatus
            reply({axis.status});
            break;
        }
        case 0x31: { // ReadEncoderAddition (48 bit signed)
            update_motion(axis);
            const std::uint64_t raw = static_cast<std::uint64_t>(axis.axis_position) & 0x0000FFFFFFFFFFFFull;
            reply({static_cast<std::uint8_t>((raw >> 40) & 0xFF),
                   static_cast<std::uint8_t>((raw >> 32) & 0xFF),
                   static_cast<std::uint8_t>((raw >> 24) & 0xFF),
                   static_cast<std::uint8_t>((raw >> 16) & 0xFF),
                   static_cast<std::uint8_t>((raw >> 8) & 0xFF),
                   static_cast<std::uint8_t>(raw & 0xFF)});
            break;
        }
        case 0x32: { // ReadMotorSpeed
            update_motion(axis);
            const auto speed = static_cast<std::uint16_t>(axis.speed_rpm);
            reply({static_cast<std::uint8_t>((speed >> 8) & 0xFF), static_cast<std::uint8_t>(speed & 0xFF)});
            break;
        }
        case 0x3E: { // ReadProtectionState
            reply({axis.protection});
            break;
        }
        case 0xF3: { // EnableMotor
            if (req.size() >= 2) {
                const bool en = req[1] != 0;
                axis.status = en ? 0x02 : 0x01;
                if (!en) {
                    axis.speed_rpm = 0;
                    axis.motion_active = false;
                    axis.speed_mode_active = false;
                }
            }
            reply({axis.status});
            break;
        }
        case 0xF7: { // EmergencyStop
            axis.speed_rpm = 0;
            axis.status = 0x00;
            axis.motion_active = false;
            axis.speed_mode_active = false;
            reply({axis.status});
            break;
        }
        case 0xF5: { // RunPositionAbsoluteAxis
            if (req.size() >= 7) {
                const auto speed_cmd = static_cast<int>(MksProtocol::readBe16(req.data() + 1));
                const std::int32_t axis24 = MksProtocol::readBe24s(req.data() + 4);
                axis.motion_start_pos = axis.axis_position;
                axis.motion_target_pos = axis24;
                axis.motion_start = std::chrono::steady_clock::now();
                const std::int64_t distance = std::llabs(axis.motion_target_pos - axis.motion_start_pos);
                const int speed = std::max<int>(1, speed_cmd);
                const auto duration_ms = std::max<std::int64_t>(50, (distance * 5) / speed);
                axis.motion_duration = std::chrono::milliseconds(duration_ms);
                axis.motion_speed_rpm = static_cast<std::int16_t>(speed);
                axis.motion_active = true;
                axis.speed_mode_active = false;
                axis.status = 0x02;
            }
            reply({0x01});
            break;
        }
        case 0xF4: { // RunPositionRelativeAxis
            if (req.size() >= 7) {
                const auto speed_cmd = static_cast<int>(MksProtocol::readBe16(req.data() + 1));
                const std::int32_t axis24 = MksProtocol::readBe24s(req.data() + 4);
                axis.motion_start_pos = axis.axis_position;
                axis.motion_target_pos = axis.axis_position + axis24;
                axis.motion_start = std::chrono::steady_clock::now();
                const std::int64_t distance = std::llabs(axis24);
                const int speed = std::max<int>(1, speed_cmd);
                const auto duration_ms = std::max<std::int64_t>(50, (distance * 5) / speed);
                axis.motion_duration = std::chrono::milliseconds(duration_ms);
                axis.motion_speed_rpm = static_cast<std::int16_t>(speed);
                axis.motion_active = true;
                axis.speed_mode_active = false;
                axis.status = 0x02;
            }
            reply({0x01});
            break;
        }
        case 0xF6: { // RunSpeedMode
            if (req.size() >= 4) {
                const std::uint16_t speed = static_cast<std::uint16_t>(((req[1] & 0x0F) << 8) | req[2]);
                // Protocol-aligned direction for F6:
                // bit7=0 -> CCW (positive RPM), bit7=1 -> CW (negative RPM)
                const bool clockwise = (req[1] & 0x80) != 0;
                axis.speed_rpm = static_cast<std::int16_t>(clockwise ? -static_cast<int>(speed) : static_cast<int>(speed));
                axis.motion_active = false;
                axis.speed_mode_active = (axis.speed_rpm != 0);
                axis.speed_mode_last_update = std::chrono::steady_clock::now();
                axis.status = 0x02;
            }
            reply({0x01});
            break;
        }
        case 0x82: { // SetWorkMode
            if (req.size() >= 2) {
                axis.work_mode = req[1];
            }
            reply({axis.work_mode});
            break;
        }
        case 0x83: { // SetWorkingCurrent
            if (req.size() >= 3) {
                axis.current_ma = MksProtocol::readBe16(req.data() + 1);
            }
            reply({static_cast<std::uint8_t>((axis.current_ma >> 8) & 0xFF),
                   static_cast<std::uint8_t>(axis.current_ma & 0xFF)});
            break;
        }
        case 0x84: { // SetSubdivision
            if (req.size() >= 2) {
                axis.subdivision = req[1];
            }
            reply({axis.subdivision});
            break;
        }
        case 0x85: { // SetEnPinActiveLevel
            if (req.size() >= 2) {
                axis.en_pin_active_level = req[1];
            }
            reply({axis.en_pin_active_level});
            break;
        }
        case 0x86: { // SetMotorDirection
            if (req.size() >= 2) {
                axis.motor_direction = req[1];
            }
            reply({axis.motor_direction});
            break;
        }
        case 0x87: { // SetAutoTurnOffScreen
            if (req.size() >= 2) {
                axis.auto_screen_off = req[1];
            }
            reply({axis.auto_screen_off});
            break;
        }
        case 0x88: { // SetLockedRotorProtection
            if (req.size() >= 2) {
                axis.locked_rotor_protection = req[1];
            }
            reply({axis.locked_rotor_protection});
            break;
        }
        case 0x89: { // SetSubdivisionInterpolation
            if (req.size() >= 2) {
                axis.subdivision_interpolation = req[1];
            }
            reply({axis.subdivision_interpolation});
            break;
        }
        case 0x8A: { // SetCanBitrate
            if (req.size() >= 2) {
                axis.can_bitrate_index = req[1];
            }
            reply({axis.can_bitrate_index});
            break;
        }
        case 0x8C: { // SetSlaveRespondActive
            if (req.size() >= 3) {
                axis.slave_respond = req[1] ? 1 : 0;
                axis.slave_active = req[2] ? 1 : 0;
            }
            reply({axis.slave_respond, axis.slave_active});
            break;
        }
        case 0x8D: { // SetGroupId
            if (req.size() >= 3) {
                axis.group_id = MksProtocol::readBe16(req.data() + 1);
            }
            reply({static_cast<std::uint8_t>((axis.group_id >> 8) & 0xFF),
                   static_cast<std::uint8_t>(axis.group_id & 0xFF)});
            break;
        }
        case 0x8F: { // SetKeyLock
            if (req.size() >= 2) {
                axis.key_lock = req[1];
            }
            reply({axis.key_lock});
            break;
        }
        case 0x9B: { // SetHoldingCurrent
            if (req.size() >= 2) {
                axis.holding_current_idx = req[1];
            }
            reply({axis.holding_current_idx});
            break;
        }
        case 0x9E: { // SetLimitPortRemap
            if (req.size() >= 2) {
                axis.limit_port_remap = req[1];
            }
            reply({axis.limit_port_remap});
            break;
        }
        case 0x92: { // SetCurrentAxisToZero
            axis.axis_position = 0;
            reply({0x01});
            break;
        }
        case 0x00: { // ReadParameter (prefix)
            if (req.size() >= 2) {
                const std::uint8_t p = req[1];
                switch (p) {
                    case 0x82:
                        reply_with_cmd(0x82, {axis.work_mode});
                        break;
                    case 0x83:
                        reply_with_cmd(0x83,
                                       {static_cast<std::uint8_t>((axis.current_ma >> 8) & 0xFF),
                                        static_cast<std::uint8_t>(axis.current_ma & 0xFF)});
                        break;
                    case 0x84:
                        reply_with_cmd(0x84, {axis.subdivision});
                        break;
                    case 0x85:
                        reply_with_cmd(0x85, {axis.en_pin_active_level});
                        break;
                    case 0x86:
                        reply_with_cmd(0x86, {axis.motor_direction});
                        break;
                    case 0x87:
                        reply_with_cmd(0x87, {axis.auto_screen_off});
                        break;
                    case 0x88:
                        reply_with_cmd(0x88, {axis.locked_rotor_protection});
                        break;
                    case 0x89:
                        reply_with_cmd(0x89, {axis.subdivision_interpolation});
                        break;
                    case 0x8A:
                        reply_with_cmd(0x8A, {axis.can_bitrate_index});
                        break;
                    case 0x8B:
                        reply_with_cmd(0x8B,
                                       {static_cast<std::uint8_t>((axis.can_id >> 8) & 0xFF),
                                        static_cast<std::uint8_t>(axis.can_id & 0xFF)});
                        break;
                    case 0x8C:
                        reply_with_cmd(0x8C, {axis.slave_respond, axis.slave_active});
                        break;
                    case 0x8D:
                        reply_with_cmd(0x8D,
                                       {static_cast<std::uint8_t>((axis.group_id >> 8) & 0xFF),
                                        static_cast<std::uint8_t>(axis.group_id & 0xFF)});
                        break;
                    case 0x8F:
                        reply_with_cmd(0x8F, {axis.key_lock});
                        break;
                    case 0x9B:
                        reply_with_cmd(0x9B, {axis.holding_current_idx});
                        break;
                    case 0x9E:
                        reply_with_cmd(0x9E, {axis.limit_port_remap});
                        break;
                    default:
                        break; // no response for unsupported parameter
                }
            }
            break;
        }
        case 0x8B: { // SetCanId
            if (req.size() >= 3) {
                const auto new_id = MksProtocol::readBe16(req.data() + 1);
                if (new_id > 0 && new_id <= 0x7FF && axes_.find(new_id) == axes_.end()) {
                    SimAxisState moved = axis;
                    moved.can_id = new_id;
                    axes_.erase(axis_it);
                    axes_.emplace(new_id, moved);
                    reply({static_cast<std::uint8_t>((new_id >> 8) & 0xFF),
                           static_cast<std::uint8_t>(new_id & 0xFF)});
                }
            }
            break;
        }
        default:
            break; // unsupported command -> no response
    }

    return true;
}

void SimCanPort::update_motion(SimAxisState& axis) {
    const auto now = std::chrono::steady_clock::now();

    if (axis.speed_mode_active) {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - axis.speed_mode_last_update).count();
        if (elapsed_ms > 0) {
            const double delta_axis_units =
                (static_cast<double>(axis.speed_rpm) * static_cast<double>(elapsed_ms)) / 5.0;
            axis.axis_position += static_cast<std::int64_t>(std::llround(delta_axis_units));
            axis.speed_mode_last_update = now;
        }
        axis.status = 0x02;
        return;
    }

    if (!axis.motion_active) {
        return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - axis.motion_start);
    if (axis.motion_duration.count() <= 0) {
        axis.axis_position = axis.motion_target_pos;
        axis.speed_rpm = 0;
        axis.motion_active = false;
        axis.status = 0x02;
        return;
    }

    const double ratio = std::clamp(elapsed.count() / static_cast<double>(axis.motion_duration.count()), 0.0, 1.0);
    const double pos = static_cast<double>(axis.motion_start_pos) +
                       (static_cast<double>(axis.motion_target_pos - axis.motion_start_pos) * ratio);
    axis.axis_position = static_cast<std::int64_t>(std::llround(pos));

    if (ratio >= 1.0) {
        axis.speed_rpm = 0;
        axis.motion_active = false;
    } else {
        const auto direction = (axis.motion_target_pos >= axis.motion_start_pos) ? 1 : -1;
        axis.speed_rpm = static_cast<std::int16_t>(direction * axis.motion_speed_rpm);
    }
    axis.status = 0x02;
}

} // namespace mks
