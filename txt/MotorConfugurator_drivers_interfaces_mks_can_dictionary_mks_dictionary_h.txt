#pragma once

#include "motion_core/axis_interface.h"
#include <vector>

namespace mks {

enum class MksParameter : std::uint32_t {
    WorkMode = 1,
    WorkingCurrentMilliAmp,
    Subdivision,
    EnPinActiveLevel,
    MotorDirection,
    AutoScreenOff,
    LockedRotorProtection,
    SubdivisionInterpolation,
    CanBitrateIndex,
    CanId,
    SlaveRespondMode,
    SlaveActiveReport,
    GroupId,
    KeyLock,
    HoldingCurrentIndex,
    LimitPortRemap,
    AxisPositionRaw,
    MotorSpeedRpm,
    ProtectionState,
    MotorStatus,
    EnableMotor,
};

constexpr motion_core::ParameterId parameter_id(const MksParameter parameter) {
    return motion_core::make_parameter_id(
        motion_core::ParameterDomain::Mks,
        static_cast<std::uint32_t>(parameter));
}

[[nodiscard]] const motion_core::ParameterDescriptor* find_mks_descriptor(motion_core::ParameterId id);
[[nodiscard]] std::vector<motion_core::ParameterDescriptor> get_mks_parameter_descriptors();

// Helper functions for parameter conversion
[[nodiscard]] motion_core::Result<std::uint64_t> require_unsigned_value(const motion_core::ParameterValue& value, const char* field_name);
[[nodiscard]] motion_core::Result<double> require_floating_value(const motion_core::ParameterValue& value, const char* field_name);
[[nodiscard]] motion_core::Result<bool> require_bool_value(const motion_core::ParameterValue& value, const char* field_name);

} // namespace mks

namespace motion_core {

using MksParameter = ::mks::MksParameter;

constexpr ParameterId make_parameter_id(const ::mks::MksParameter parameter) {
    return ::mks::parameter_id(parameter);
}

} // namespace motion_core
