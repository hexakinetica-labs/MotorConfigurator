#pragma once

#include "motion_core/types.h"

#include <cstddef>
#include <cstdint>

namespace motion_core {

struct AxisTelemetry {
    AxisState state{AxisState::Unknown};
    AxisMode mode{AxisMode::ProfilePosition};
    double actual_position_deg{0.0};
    std::int64_t raw_axis_position{0};
    double target_position_deg{0.0};
    double actual_velocity_deg_per_sec{0.0};
    double actual_torque_percent{0.0};
    std::uint32_t status_word{0};
    std::uint32_t protection_code{0};
    std::uint32_t motion_status_code{0};
    std::uint64_t timestamp_ns{0};
};

struct AxisCommand {
    bool has_target_position{false};
    double target_position_deg{0.0};
    bool has_profile_speed_rpm{false};
    std::uint16_t profile_speed_rpm{0};
    bool has_profile_accel_percent{false};
    double profile_accel_percent{0.0};
    bool has_target_velocity{false};
    double target_velocity_deg_per_sec{0.0};
    bool is_relative{false};
    bool emergency_stop{false};
    bool clear_errors{false};
    bool go_home{false};
    bool set_zero{false};
};

struct QueuedSetpoint {
    double target_position_deg{0.0};
    bool has_profile_speed_rpm{false};
    std::uint16_t profile_speed_rpm{0};
    bool has_profile_accel_percent{false};
    double profile_accel_percent{0.0};
    bool has_target_velocity{false};
    double target_velocity_deg_per_sec{0.0};
    // Producer-side sample period for this setpoint (seconds).
    // Used by buffered streaming drivers to keep a time-based horizon.
    double sample_period_sec{0.004};
};

struct MotionQueueStats {
    std::size_t size{0U};
    std::size_t capacity{0U};
    std::uint64_t pushed{0U};
    std::uint64_t dropped{0U};
    std::uint64_t underruns{0U};
    std::uint64_t short_starts{0U};
};

} // namespace motion_core
