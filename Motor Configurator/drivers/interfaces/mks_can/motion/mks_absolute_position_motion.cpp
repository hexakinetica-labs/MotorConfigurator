#include "mks_can/motion/mks_motion_mode.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace mks {

motion_core::Result<MksBusCommand> MksAbsolutePositionMotionMode::build_command(
    const MksMotionBuildContext& context,
    const motion_core::QueuedSetpoint& point) const {
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
    MksProtocol::appendBe16(command.payload, speed_rpm);
    command.payload.push_back(accel_byte);
    MksProtocol::appendBe24(command.payload, clamped_axis);
    command.requires_sync_execute = true;
    return motion_core::Result<MksBusCommand>::success(std::move(command));
}

} // namespace mks
