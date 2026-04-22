#pragma once

#include "mks_can/internal/port/can_port_interface.h"

#include <chrono>
#include <deque>
#include <mutex>
#include <unordered_map>

namespace mks {

class SimCanPort : public ICanPort {
public:
    SimCanPort();
    ~SimCanPort() override = default;

    bool open(const char* channel, unsigned int baud_rate) override;
    void close() override;
    bool isOpen() const override;
    bool write(const CanFrame& frame) override;
    bool read(CanFrame& frame, unsigned int timeout_ms) override;

private:
    struct SimAxisState {
        std::uint16_t can_id{1};
        std::int64_t axis_position{0};
        std::int16_t speed_rpm{0};
        std::uint8_t protection{0};
        std::uint8_t status{0x01};

        std::uint8_t work_mode{5};
        std::uint16_t current_ma{1600};
        std::uint8_t subdivision{16};
        std::uint8_t en_pin_active_level{2};
        std::uint8_t motor_direction{0};
        std::uint8_t auto_screen_off{0};
        std::uint8_t locked_rotor_protection{1};
        std::uint8_t subdivision_interpolation{1};
        std::uint8_t can_bitrate_index{2};
        std::uint8_t slave_respond{1};
        std::uint8_t slave_active{1};
        std::uint16_t group_id{1};
        std::uint8_t key_lock{0};
        std::uint8_t holding_current_idx{4};
        std::uint8_t limit_port_remap{0};

        bool motion_active{false};
        std::int64_t motion_start_pos{0};
        std::int64_t motion_target_pos{0};
        std::chrono::steady_clock::time_point motion_start{};
        std::chrono::milliseconds motion_duration{0};
        std::int16_t motion_speed_rpm{0};

        bool speed_mode_active{false};
        std::chrono::steady_clock::time_point speed_mode_last_update{};
    };

    bool handle_frame(const CanFrame& frame);
    static std::uint8_t compute_crc(std::uint16_t can_id, const std::vector<std::uint8_t>& bytes_without_crc);
    void push_response(const CanFrame& frame);
    static void update_motion(SimAxisState& axis);

    mutable std::mutex mutex_;
    bool opened_{false};
    std::string channel_{};
    unsigned int baud_rate_{0};

    std::unordered_map<std::uint16_t, SimAxisState> axes_;
    std::deque<CanFrame> rx_queue_;
};

} // namespace mks
