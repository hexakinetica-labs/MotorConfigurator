#include "mks_can/motion/mks_motion_mode.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace mks {

namespace {

} // namespace

void MksAbsolutePositionMotionMode::on_mode_enter(const std::chrono::steady_clock::time_point now) {
    has_pending_point_ = false;
    next_emit_time_ = now;
}

void MksAbsolutePositionMotionMode::on_mode_exit() {
    has_pending_point_ = false;
    next_emit_time_ = {};
}

void MksAbsolutePositionMotionMode::on_queue_cleared() {
    has_pending_point_ = false;
    next_emit_time_ = {};
}

motion_core::Result<MksBusCommand> MksAbsolutePositionMotionMode::build_absolute_command(
    const MksMotionBuildContext& context,
    const motion_core::QueuedSetpoint& point) {
    if (!(std::abs(context.axis_units_per_degree) > 1e-9) || !std::isfinite(context.axis_units_per_degree)) {
        return motion_core::Result<MksBusCommand>::failure(
            {motion_core::ErrorCode::InvalidArgument, "axis_units_per_degree must be finite and non-zero"});
    }

    const double signed_target_deg = context.invert_direction
        ? -point.target_position_deg
        : point.target_position_deg;
    const double raw_axis = signed_target_deg * context.axis_units_per_degree;
    const auto axis_value = std::llround(raw_axis);
    const auto clamped_axis = static_cast<std::int32_t>(
        std::clamp(axis_value, -8388608LL, 8388607LL));

    std::uint16_t speed_rpm = context.fallback_speed_rpm;
    if (point.has_profile_speed_rpm) {
        speed_rpm = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(point.profile_speed_rpm, 1U, 3000U));
    }

    std::uint8_t accel_byte = context.fallback_accel_byte;
    if (point.has_profile_accel_percent) {
        const auto accel = std::clamp(point.profile_accel_percent, 0.0, 100.0);
        accel_byte = static_cast<std::uint8_t>(std::llround((accel / 100.0) * 255.0));
    }

    MksBusCommand command{};
    command.can_id = context.can_id;
    command.command = static_cast<std::uint8_t>(MksCommand::RunPositionAbsoluteAxis);
    command.append_be16(speed_rpm);
    command.push_back(accel_byte);
    command.append_be24(clamped_axis);
    command.requires_sync_execute = true;
    return motion_core::Result<MksBusCommand>::success(std::move(command));
}

motion_core::Result<MksMotionStepResult> MksAbsolutePositionMotionMode::step(
    const std::chrono::steady_clock::time_point now,
    const MksMotionBuildContext& context) {
    if (!has_pending_point_) {
        motion_core::QueuedSetpoint next_point{};
        if (!try_pop_setpoint(next_point)) {
            return motion_core::Result<MksMotionStepResult>::success(MksMotionStepResult{});
        }
        pending_point_ = next_point;
        has_pending_point_ = true;
        // Emit immediately on the first pop — no hold-off for new setpoints.
        next_emit_time_ = now;
    }

    if (now < next_emit_time_) {
        return motion_core::Result<MksMotionStepResult>::success(MksMotionStepResult{});
    }

    const auto command_result = build_absolute_command(context, pending_point_);
    if (!command_result.ok()) {
        return motion_core::Result<MksMotionStepResult>::failure(command_result.error());
    }

    // Advance the deadline by sample_period_sec from the current deadline (not from now),
    // so accumulated overruns don't cause burst catch-up.
    double sample_period_sec = pending_point_.sample_period_sec;
    if (!(std::isfinite(sample_period_sec) && sample_period_sec > 0.0)) {
        sample_period_sec = 0.005;
    }
    const auto period_ns = static_cast<long long>(sample_period_sec * 1e9);
    next_emit_time_ += std::chrono::nanoseconds(period_ns);
    // Safety: if we're already far behind (e.g. mode was idle), reset deadline to now.
    if (next_emit_time_ < now) {
        next_emit_time_ = now;
    }
    has_pending_point_ = false;

    MksMotionStepResult out{};
    out.has_command = true;
    out.command = command_result.value();
    return motion_core::Result<MksMotionStepResult>::success(std::move(out));
}

} // namespace mks
