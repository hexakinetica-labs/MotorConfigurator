#include "mks_can/motion/mks_motion_mode.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace mks {

motion_core::Result<MksBusCommand> MksVelocityMotionMode::build_command(
    const MksMotionBuildContext& context,
    const motion_core::QueuedSetpoint& point) const {
    if (!(context.software_gear_ratio > 0.0) || !std::isfinite(context.software_gear_ratio)) {
        return motion_core::Result<MksBusCommand>::failure(
            {motion_core::ErrorCode::InvalidArgument, "software_gear_ratio must be finite and > 0"});
    }

    double velocity_deg_s = point.has_target_velocity
        ? point.target_velocity_deg_per_sec
        : 0.0;
    if (context.invert_direction) {
        velocity_deg_s = -velocity_deg_s;
    }

    const double motor_deg_s = velocity_deg_s * context.software_gear_ratio;
    const double rpm = std::abs(motor_deg_s) / 6.0;
    const auto speed = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(std::llround(rpm)),
        0U,
        3000U));
    const bool clockwise = motor_deg_s < 0.0;

    std::uint8_t accel_byte = context.fallback_accel_byte;
    if (point.has_profile_accel_percent) {
        const auto accel = std::clamp(point.profile_accel_percent, 0.0, 100.0);
        accel_byte = static_cast<std::uint8_t>(std::llround((accel / 100.0) * 255.0));
    }

    MksBusCommand command{};
    command.can_id = context.can_id;
    command.command = static_cast<std::uint8_t>(MksCommand::RunSpeedMode);
    command.payload.push_back((clockwise ? 0x80u : 0x00u) | static_cast<std::uint8_t>((speed >> 8) & 0x0Fu));
    command.payload.push_back(static_cast<std::uint8_t>(speed & 0xFFu));
    command.payload.push_back(accel_byte);
    command.requires_sync_execute = true;
    return motion_core::Result<MksBusCommand>::success(std::move(command));
}

} // namespace mks
