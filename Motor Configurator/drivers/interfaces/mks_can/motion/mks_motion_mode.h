#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/result.h"

#include <cstdint>
#include <vector>

namespace mks {

struct MksBusCommand {
    std::uint16_t can_id{0};
    std::uint8_t command{0};
    std::vector<std::uint8_t> payload{};
    bool requires_sync_execute{false};
};

struct MksMotionBuildContext {
    std::uint16_t can_id{0};
    double axis_units_per_degree{1.0};
    double software_gear_ratio{1.0};
    bool invert_direction{false};
    std::uint16_t fallback_speed_rpm{300};
    std::uint8_t fallback_accel_byte{16};
};

class MksMotionModeBase {
public:
    virtual ~MksMotionModeBase() = default;

    [[nodiscard]] virtual motion_core::Result<MksBusCommand> build_command(
        const MksMotionBuildContext& context,
        const motion_core::QueuedSetpoint& point) const = 0;
};

class MksAbsolutePositionMotionMode final : public MksMotionModeBase {
public:
    [[nodiscard]] motion_core::Result<MksBusCommand> build_command(
        const MksMotionBuildContext& context,
        const motion_core::QueuedSetpoint& point) const override;
};

class MksVelocityMotionMode final : public MksMotionModeBase {
public:
    [[nodiscard]] motion_core::Result<MksBusCommand> build_command(
        const MksMotionBuildContext& context,
        const motion_core::QueuedSetpoint& point) const override;
};

} // namespace mks
