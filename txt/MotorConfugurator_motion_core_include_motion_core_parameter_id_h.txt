#pragma once

#include <cstdint>

namespace motion_core {

enum class ParameterDomain {
    Common = 0,
    Ethercat,
    Mks
};

enum class CommonParameter : std::uint16_t {
    HardwareGearRatio = 1,
    HardwareEncoderResolutionBits = 2,
    HardwareInvertDirection = 3,
    LimitsMaxVelocityDegPerSec = 4,
    LimitsMaxAccelerationDegPerSec2 = 5,
    LimitsSoftwareMinDeg = 6,
    LimitsSoftwareMaxDeg = 7,
    HomingMethod = 8,
    HomingSpeedSwitchDegPerSec = 9,
    HomingSpeedZeroDegPerSec = 10,
    HomingOffsetDeg = 11,
    LimitsCurrentLimitPositivePct = 12,
    LimitsCurrentLimitNegativePct = 13,
    HomingAccelerationDegPerSec2 = 14,
    MotorSelectionCode = 15,
    HomeSwitchToZeroShiftDeg = 16,
    TelemetryInvertPositionSign = 17
};

struct ParameterId {
    ParameterDomain domain{ParameterDomain::Common};
    std::uint32_t value{0};

    [[nodiscard]] bool valid() const noexcept { return value != 0; }
};

constexpr ParameterId make_parameter_id(CommonParameter parameter) {
    return ParameterId{ParameterDomain::Common, static_cast<std::uint32_t>(parameter)};
}

constexpr ParameterId make_parameter_id(const ParameterDomain domain, const std::uint32_t value) {
    return ParameterId{domain, value};
}

} // namespace motion_core
