#include "mks_dictionary.h"
#include <cmath>
#include <iterator>

namespace mks {

using motion_core::MksParameter;
using motion_core::ParameterDescriptor;
using motion_core::ParameterDomain;
using motion_core::ParameterValue;

const ParameterDescriptor kMksParameterDescriptors[] = {
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio),
     "Gear Ratio", "Common/Hardware", "motor_turns_per_output_turn", false, true, true, true,
     ParameterValue::from_floating(0.001), ParameterValue::from_floating(1000.0)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareEncoderResolutionBits),
     "Encoder Resolution (bits)", "Common/Hardware", "bits", true, false, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(31)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareInvertDirection),
     "Invert Direction", "Common/Hardware", "", false, true, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxVelocityDegPerSec),
     "Max Velocity", "Common/Limits", "deg/s", false, true, true, true,
     ParameterValue::from_floating(0.1), ParameterValue::from_floating(18000.0)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2),
     "Max Acceleration (%)", "Common/Limits", "%", false, true, true, true,
     ParameterValue::from_floating(0.0), ParameterValue::from_floating(100.0)},
    {motion_core::make_parameter_id(MksParameter::WorkMode),
     "Work Mode", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(5)},
    {motion_core::make_parameter_id(MksParameter::WorkingCurrentMilliAmp),
     "Working Current", "Drive/MKS/Config", "mA", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(5200)},
    {motion_core::make_parameter_id(MksParameter::Subdivision),
     "Subdivision", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(255)},
    {motion_core::make_parameter_id(MksParameter::EnPinActiveLevel),
     "EN Pin Active Level", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(2)},
    {motion_core::make_parameter_id(MksParameter::MotorDirection),
     "Motor Direction", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::AutoScreenOff),
     "Auto Screen Off", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::LockedRotorProtection),
     "Locked Rotor Protection", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::SubdivisionInterpolation),
     "Subdivision Interpolation", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::CanBitrateIndex),
     "CAN Bitrate Index", "Drive/MKS/Config", "", true, false, true, true, // Not persistable in axis config (bootstrap)
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(3)},
    {motion_core::make_parameter_id(MksParameter::CanId),
     "CAN ID", "Drive/MKS/Config", "", false, false, true, true, // writable runtime command 0x8B; keep non-persistable for AxisConfig bootstrap split
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(0x7FF)},
    {motion_core::make_parameter_id(MksParameter::SlaveRespondMode),
     "Slave Respond Mode", "Drive/MKS/Config", "", true, false, false, false,
     // LOCKED_BY_POLICY: modification forbidden by system policy
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::SlaveActiveReport),
     "Slave Active Report", "Drive/MKS/Config", "", true, false, false, false,
     // LOCKED_BY_POLICY: modification forbidden by system policy
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::GroupId),
     "Group ID", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(0x7FF)},
    {motion_core::make_parameter_id(MksParameter::KeyLock),
     "Key Lock", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::HoldingCurrentIndex),
     "Holding Current Index", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(8)},
    {motion_core::make_parameter_id(MksParameter::LimitPortRemap),
     "Limit Port Remap", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::AxisPositionRaw),
     "Axis Position Raw", "Drive/MKS/Telemetry", "axis", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::MotorSpeedRpm),
     "Motor Speed", "Drive/MKS/Telemetry", "rpm", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::ProtectionState),
     "Protection State", "Drive/MKS/Telemetry", "", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::MotorStatus),
     "Motor Status", "Drive/MKS/Telemetry", "", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::EnableMotor),
     "Enable Motor", "Drive/MKS", "", false, false, true, true, // Runtime-only (don't save to config file)
     ParameterValue::from_bool(false), ParameterValue::from_bool(true)},
};

const motion_core::ParameterDescriptor* find_mks_descriptor(const motion_core::ParameterId id) {
    for (const auto& descriptor : kMksParameterDescriptors) {
        if (descriptor.id.domain == id.domain && descriptor.id.value == id.value) {
            return &descriptor;
        }
    }
    return nullptr;
}

std::vector<motion_core::ParameterDescriptor> get_mks_parameter_descriptors() {
    std::vector<motion_core::ParameterDescriptor> out;
    out.assign(std::begin(kMksParameterDescriptors), std::end(kMksParameterDescriptors));
    return out;
}

motion_core::Result<std::uint64_t> require_unsigned_value(const motion_core::ParameterValue& value,
                                                          const char* field_name) {
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<std::uint64_t>::success(value.unsigned_value);
    if (value.type == motion_core::ParameterValueType::SignedInteger) {
        if (value.signed_value < 0) return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
        return motion_core::Result<std::uint64_t>::success(static_cast<std::uint64_t>(value.signed_value));
    }
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<std::uint64_t>::success(value.bool_value ? 1U : 0U);
    if (value.type == motion_core::ParameterValueType::FloatingPoint) {
        if (value.floating_value < 0.0) return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
        return motion_core::Result<std::uint64_t>::success(static_cast<std::uint64_t>(std::llround(value.floating_value)));
    }
    return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

motion_core::Result<double> require_floating_value(const motion_core::ParameterValue& value,
                                                   const char* field_name) {
    if (value.type == motion_core::ParameterValueType::FloatingPoint) return motion_core::Result<double>::success(value.floating_value);
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<double>::success(static_cast<double>(value.unsigned_value));
    if (value.type == motion_core::ParameterValueType::SignedInteger) return motion_core::Result<double>::success(static_cast<double>(value.signed_value));
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<double>::success(value.bool_value ? 1.0 : 0.0);
    return motion_core::Result<double>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

motion_core::Result<bool> require_bool_value(const motion_core::ParameterValue& value,
                                             const char* field_name) {
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<bool>::success(value.bool_value);
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<bool>::success(value.unsigned_value != 0U);
    if (value.type == motion_core::ParameterValueType::SignedInteger) return motion_core::Result<bool>::success(value.signed_value != 0);
    if (value.type == motion_core::ParameterValueType::FloatingPoint) return motion_core::Result<bool>::success(std::abs(value.floating_value) >= 0.5);
    return motion_core::Result<bool>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

} // namespace mks
