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
    HardwareEncoderResolutionBits,
    HardwareInvertDirection,
    LimitsMaxVelocityDegPerSec,
    LimitsMaxAccelerationDegPerSec2,
    LimitsSoftwareMinDeg,
    LimitsSoftwareMaxDeg,
    HomingMethod,
    HomingSpeedSwitchDegPerSec,
    HomingSpeedZeroDegPerSec,
    HomingOffsetDeg,
    LimitsCurrentLimitPositivePct,  // = 12
    LimitsCurrentLimitNegativePct,  // = 13
    HomingAccelerationDegPerSec2,   // = 14
    MotorSelectionCode              // = 15
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
