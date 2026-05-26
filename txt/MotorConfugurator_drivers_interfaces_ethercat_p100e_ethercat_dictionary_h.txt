#pragma once

#include "motion_core/parameter_id.h"
#include "motion_core/parameter_types.h"
#include "motion_core/result.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>

namespace ethercat_driver {

enum class EthercatParameter : std::uint32_t {
    // CiA402 cyclic (PDO) — managed by process_cycle(), not in SDO dictionary
    Controlword = 1,
    Statusword,
    OperationMode,
    ActualPositionCounts,
    ActualVelocityCountsPerSec,
    TargetPositionCounts,

    // CiA402 SDO — motion profile & limits
    MaxProfileVelocityCountsPerSec = 7,
    MaxTorqueTenthsPercent,
    SoftLimitMinCounts,
    SoftLimitMaxCounts,
    ProfileVelocity,
    ProfileAcceleration,
    ProfileDeceleration,
    QuickStopDecel,
    PositionArrivalThreshold,
    TorqueLimitForward,
    TorqueLimitReverse,
    HomingAcceleration,
    AxialResolution,

    // Vendor specific PID (0x2001 "PID Adjust Parameter", DT2001)
    PidPosKp,
    PidVelKp,
    PidVelKi,
    PidPosFF,
    PidTorqueFilter,
    PidSpdDetectionFilter,
    PidAccelTime,
    PidDecelTime,

    // Vendor specific current control (0x2008 "Step Mode Parameter I", DT2008)
    PidCurrentKp,
    PidCurrentKi,

    // Vendor specific hardware config (0x2000 "Basic Control Parameter", DT2000)
    EncoderSelection,
    EncoderResolutionBits,

    PidModelKp = 32,

    // CiA402 SDO — additional profile/status objects from ESI/PDF
    ModeDisplay,
    TargetVelocity,
    ActualVelocity,
    TargetTorque,
    HomeOffset,
    TorqueSlope,
    QuickStopOptionCode,
    FollowingErrorActualValue,
    DigitalInputs,
    PhysicalOutputs,

    // Probe function objects
    ProbeFunction,
    ProbeStatus,
    Probe1RisePosition,
    Probe1FallPosition,
    Probe2RisePosition,
    Probe2FallPosition,

    // Vendor-specific communication / basic control objects
    CommunicationFixedAddress,
    BasicInitialStatusDisplay,
    BasicActionOnStop,
    BasicActuatorAction,
    BasicRunningBrakeSpeed,
    BasicTorqueControlSpeedLimit,
    BasicServoOnDelayMs,
    BasicIoInputActiveLevelWord,
    InputPort1Function,
    InputPort2Function,
    InputPort3Function,
    InputPort4Function,
    InputPort5Function,
    InputPort6Function,
    InputPort7Function,
    InputPort8Function,
    BasicIoOutputActiveLevelWord,
    BasicIoDebounceMs,
    BasicMotorPoles,
    BasicPwmDutyCycle,

    // Vendor step-mode objects
    StepLockCurrent,
    StepRunCurrent,

    // Vendor servo model objects
    MotorType,

    // Internal service object used only as durable-save trigger.
    SaveParametersToEeprom,
};

constexpr motion_core::ParameterId parameter_id(const EthercatParameter parameter) {
    return motion_core::make_parameter_id(
        motion_core::ParameterDomain::Ethercat,
        static_cast<std::uint32_t>(parameter));
}

struct ParameterDefinition {
    // Technical data
    motion_core::ParameterId id;
    const char* rdt_key;
    uint16_t index;
    uint8_t sub_index;
    uint8_t data_size;
    motion_core::ParameterValueType type;
    bool is_read_only;
    bool persistable_runtime;

    // UI and validation data
    const char* name;
    const char* group;
    const char* unit;
    const char* description;
    motion_core::ParameterValue min_value;
    motion_core::ParameterValue max_value;

    // Conversion/semantic metadata (dictionary = single source of truth)
    const char* raw_unit{""};
    const char* display_unit{""};
    const char* conversion_id{"NoConversion"};
    const char* semantic_scope{"DriveRaw"};
    const char* conversion_formula{""};
    bool conversion_depends_on_gear_ratio{false};
    bool conversion_depends_on_axial_resolution{false};
};

enum class EthercatConversionKind : std::uint8_t {
    NoConversion = 0,
    RawInstructionPosition_OutputDegrees,
    RawInstructionVelocity_OutputDegPerSec,
    RawInstructionAcceleration_OutputDegPerSec2,
    RawTenthsPercent_Percent,
    RawPercentPerMs_PercentPerMs,
};

struct EthercatConversionContext {
    double gear_ratio{1.0};
    std::uint32_t axial_resolution_instr_per_motor_rev{10000};
};

[[nodiscard]] motion_core::Result<EthercatConversionKind> parse_conversion_kind(const char* conversion_id);
[[nodiscard]] motion_core::Result<motion_core::ParameterValue> convert_from_raw_to_display(
    const ParameterDefinition& definition,
    const motion_core::ParameterValue& raw_value,
    const EthercatConversionContext& context);
[[nodiscard]] motion_core::Result<motion_core::ParameterValue> convert_from_display_to_raw(
    const ParameterDefinition& definition,
    const motion_core::ParameterValue& display_value,
    const EthercatConversionContext& context);
[[nodiscard]] motion_core::Result<motion_core::ParameterValue> convert_display_min_to_raw(
    const ParameterDefinition& definition,
    const EthercatConversionContext& context);
[[nodiscard]] motion_core::Result<motion_core::ParameterValue> convert_display_max_to_raw(
    const ParameterDefinition& definition,
    const EthercatConversionContext& context);

[[nodiscard]] const ParameterDefinition* find_ethercat_parameter_definition(motion_core::ParameterId id);

constexpr std::array<ParameterDefinition, 68> p100e_dictionary = {{

    // ── Common/Homing ──────────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingMethod),
     "homing.method", 0x6098, 0, 1, motion_core::ParameterValueType::SignedInteger, false, true,
     "Homing method", "Common/Homing", "", "Homing method (17=fwd limit, 18=rev limit).",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(35)},

    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedSwitchDegPerSec),
     "homing.speed_switch", 0x6099, 1, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing speed (switch)", "Common/Homing", "instr/s", "Speed during search for switch.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedZeroDegPerSec),
     "homing.speed_zero", 0x6099, 2, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing speed (zero)", "Common/Homing", "instr/s", "Speed during search for zero.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::HomingAcceleration),
     "homing.accel", 0x609A, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing acceleration", "Common/Homing", "instr/s2", "Acceleration/deceleration for homing.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s2", "deg/s2", "RawInstructionAcceleration_OutputDegPerSec2", "AxisOutput",
     "deg_per_sec2 = raw_instr_per_sec2 * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::HomeOffset),
     "homing.offset", 0x607C, 0, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Home offset", "Common/Homing", "instr", "Home offset applied after homing sequence.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr", "deg", "RawInstructionPosition_OutputDegrees", "AxisOutput",
     "deg = raw_instr * 360 / (axial_resolution * gear_ratio)", true, true},

    // ── Common/Limits ──────────────────────────────────────────────────────
    {parameter_id(EthercatParameter::SoftLimitMinCounts),
     "limits.soft_min", 0x607D, 1, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Software position limit (min)", "Common/Limits", "instr", "Minimum software position limit in instruction units.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr", "deg", "RawInstructionPosition_OutputDegrees", "AxisOutput",
     "deg = raw_instr * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::SoftLimitMaxCounts),
     "limits.soft_max", 0x607D, 2, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Software position limit (max)", "Common/Limits", "instr", "Maximum software position limit in instruction units.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr", "deg", "RawInstructionPosition_OutputDegrees", "AxisOutput",
     "deg = raw_instr * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::MaxTorqueTenthsPercent),
     "limits.max_torque", 0x6072, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Max torque", "Common/Limits", "0.1%", "Maximum commanded torque in torque modes.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000),
     "0.1%", "%", "RawTenthsPercent_Percent", "AxisOutput",
     "percent = raw_tenths_percent / 10", false, false},

    {parameter_id(EthercatParameter::TorqueLimitForward),
     "limits.torque_fwd", 0x60E0, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Forward torque limit", "Common/Limits", "0.1%", "Forward torque limit (3000 = 300%).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000),
     "0.1%", "%", "RawTenthsPercent_Percent", "AxisOutput",
     "percent = raw_tenths_percent / 10", false, false},

    {parameter_id(EthercatParameter::TorqueLimitReverse),
     "limits.torque_rev", 0x60E1, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Reverse torque limit", "Common/Limits", "0.1%", "Reverse torque limit (3000 = 300%).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000),
     "0.1%", "%", "RawTenthsPercent_Percent", "AxisOutput",
     "percent = raw_tenths_percent / 10", false, false},

    // ── Common/Motion ──────────────────────────────────────────────────────
    {parameter_id(EthercatParameter::MaxProfileVelocityCountsPerSec),
     "motion.max_speed", 0x607F, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Max speed", "Common/Motion", "instr/s", "Maximum running speed.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::ProfileVelocity),
     "motion.profile_vel", 0x6081, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile velocity", "Common/Motion", "instr/s", "Speed in profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::ProfileAcceleration),
     "motion.profile_accel", 0x6083, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile acceleration", "Common/Motion", "instr/s2", "Acceleration for profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s2", "deg/s2", "RawInstructionAcceleration_OutputDegPerSec2", "AxisOutput",
     "deg_per_sec2 = raw_instr_per_sec2 * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::ProfileDeceleration),
     "motion.profile_decel", 0x6084, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile deceleration", "Common/Motion", "instr/s2", "Deceleration for profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s2", "deg/s2", "RawInstructionAcceleration_OutputDegPerSec2", "AxisOutput",
     "deg_per_sec2 = raw_instr_per_sec2 * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::QuickStopDecel),
     "motion.quickstop_decel", 0x6085, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Quick stop deceleration", "Common/Motion", "instr/s2", "Deceleration for emergency stop.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "instr/s2", "deg/s2", "RawInstructionAcceleration_OutputDegPerSec2", "AxisOutput",
     "deg_per_sec2 = raw_instr_per_sec2 * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::QuickStopOptionCode),
     "motion.quickstop_option", 0x605A, 0, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Quick stop option code", "Common/Motion", "", "Quick stop behavior option code.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(6)},

    {parameter_id(EthercatParameter::PositionArrivalThreshold),
     "motion.arrival_threshold", 0x6067, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Position arrival threshold", "Common/Motion", "instr", "Window for target-reached flag (status bit 10).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(65535),
     "instr", "deg", "RawInstructionPosition_OutputDegrees", "AxisOutput",
     "deg = raw_instr * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::TargetVelocity),
     "motion.target_velocity", 0x60FF, 0, 4, motion_core::ParameterValueType::SignedInteger, false, false,
     "Target velocity", "Common/Motion", "instr/s", "Runtime velocity target in PV/CSV modes.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::ActualVelocity),
     "motion.actual_velocity", 0x606C, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Actual velocity", "Common/Motion", "instr/s", "Actual velocity feedback.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr/s", "deg/s", "RawInstructionVelocity_OutputDegPerSec", "AxisOutput",
     "deg_per_sec = raw_instr_per_sec * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::TargetTorque),
     "motion.target_torque", 0x6071, 0, 2, motion_core::ParameterValueType::SignedInteger, false, false,
     "Target torque", "Common/Motion", "0.1%", "Runtime torque target in PT/CST modes.",
     motion_core::ParameterValue::from_signed(-3000), motion_core::ParameterValue::from_signed(3000),
     "0.1%", "%", "RawTenthsPercent_Percent", "AxisOutput",
     "percent = raw_tenths_percent / 10", false, false},

    {parameter_id(EthercatParameter::TorqueSlope),
     "motion.torque_slope", 0x6087, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Torque slope", "Common/Motion", "0.1%/ms", "Torque slope in profile torque mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u),
     "0.1%/ms", "%/ms", "RawPercentPerMs_PercentPerMs", "AxisOutput",
     "percent_per_ms = raw_tenths_percent_per_ms / 10", false, false},

    {parameter_id(EthercatParameter::ModeDisplay),
     "motion.mode_display", 0x6061, 0, 1, motion_core::ParameterValueType::SignedInteger, true, false,
     "Mode display", "Common/Motion", "", "Current active operation mode reported by drive.",
     motion_core::ParameterValue::from_signed(-128), motion_core::ParameterValue::from_signed(127)},

    // ── Common/Mechanics ───────────────────────────────────────────────────
    {parameter_id(EthercatParameter::AxialResolution),
     "mechanics.axial_resolution", 0x6091, 2, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Axial resolution", "Common/Mechanics", "instr/rev",
     "Instruction units per motor revolution. All PDO position values are in these units. Default 10000.",
     motion_core::ParameterValue::from_unsigned(1), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {parameter_id(EthercatParameter::FollowingErrorActualValue),
     "mechanics.following_error", 0x60F4, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Following error actual value", "Common/Mechanics", "instr", "Actual following error.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647),
     "instr", "deg", "RawInstructionPosition_OutputDegrees", "AxisOutput",
     "deg = raw_instr * 360 / (axial_resolution * gear_ratio)", true, true},

    {parameter_id(EthercatParameter::DigitalInputs),
     "io.digital_inputs", 0x60FD, 0, 4, motion_core::ParameterValueType::UnsignedInteger, true, false,
     "Digital inputs", "Common/IO", "bitmask", "Digital input status bitmask.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {parameter_id(EthercatParameter::PhysicalOutputs),
     "io.physical_outputs", 0x60FE, 1, 4, motion_core::ParameterValueType::UnsignedInteger, false, false,
     "Physical outputs", "Common/IO", "bitmask", "Digital output command bitmask.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    // ── Vendor Specific/PID ────────────────────────────────────────────────
    // 0x2001 "PID Adjust Parameter" (DT2001 from ESI XML)
    {parameter_id(EthercatParameter::PidPosKp),
     "pid.pos_kp", 0x2001, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Position Kp (PA9)", "Vendor Specific/PID", "", "Position proportional gain.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    {parameter_id(EthercatParameter::PidVelKp),
     "pid.vel_kp", 0x2001, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed Kp (PA5)", "Vendor Specific/PID", "Hz", "Speed loop proportional gain.",
     motion_core::ParameterValue::from_signed(5), motion_core::ParameterValue::from_signed(2000)},

    {parameter_id(EthercatParameter::PidVelKi),
     "pid.vel_ki", 0x2001, 3, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed Ki (PA6)", "Vendor Specific/PID", "ms", "Speed loop integral time constant.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    // ── Vendor Specific/Filters ────────────────────────────────────────────
    {parameter_id(EthercatParameter::PidPosFF),
     "pid.pos_smooth", 0x2001, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Position smooth filter (PA19)", "Vendor Specific/Filters", "x0.1ms",
     "Position command exponential smoothing filter time constant.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(1000)},

    {parameter_id(EthercatParameter::PidTorqueFilter),
     "pid.torque_filter", 0x2001, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Torque filter (PA7)", "Vendor Specific/Filters", "%", "Torque command low-pass filter cutoff.",
     motion_core::ParameterValue::from_signed(20), motion_core::ParameterValue::from_signed(500)},

    {parameter_id(EthercatParameter::PidSpdDetectionFilter),
     "pid.spd_filter", 0x2001, 6, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed detection filter (PA8)", "Vendor Specific/Filters", "%", "Speed feedback low-pass filter cutoff.",
     motion_core::ParameterValue::from_signed(20), motion_core::ParameterValue::from_signed(500)},

    {parameter_id(EthercatParameter::PidAccelTime),
     "pid.accel_time", 0x2001, 7, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Acceleration time (PA40)", "Vendor Specific/Filters", "ms", "Time to ramp from 0 to 1000 rpm.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(10000)},

    {parameter_id(EthercatParameter::PidDecelTime),
     "pid.decel_time", 0x2001, 8, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Deceleration time (PA41)", "Vendor Specific/Filters", "ms", "Time to ramp from 1000 rpm to 0.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(10000)},

    // ── Vendor Specific/Current Control ────────────────────────────────────
    // 0x2008 "Step Mode Parameter I" (DT2008 from ESI XML)
    {parameter_id(EthercatParameter::PidCurrentKp),
     "pid.cur_kp", 0x2008, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Current KP", "Vendor Specific/Current Control", "", "Current loop proportional gain.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::PidCurrentKi),
     "pid.cur_ki", 0x2008, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Current KI", "Vendor Specific/Current Control", "", "Current loop integral gain.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::StepLockCurrent),
     "step.lock_current", 0x2008, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Locking current", "Vendor Specific/Current Control", "", "Step mode locking current.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::StepRunCurrent),
     "step.run_current", 0x2008, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Running current", "Vendor Specific/Current Control", "", "Step mode running current.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::MotorType),
     "motor.type", 0x2007, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Motor type (PA1)", "Vendor Specific/Hardware", "", "Servo motor model selection code.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(180)},

    {parameter_id(EthercatParameter::SaveParametersToEeprom),
     "internal.save_to_eeprom", 0x2005, 3, 2, motion_core::ParameterValueType::SignedInteger, false, false,
     "Save parameters to EEPROM", "Internal/EtherCAT", "", "Internal trigger for persisting current EtherCAT parameters.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(1)},

    // ── Vendor Specific/Hardware ───────────────────────────────────────────
    // 0x2000 "Basic Control Parameter" (DT2000 from ESI XML)
    {parameter_id(EthercatParameter::EncoderSelection),
     "hw.encoder_sel", 0x2000, 10, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Encoder selection (PA62)", "Vendor Specific/Hardware", "",
     "4 = single-turn absolute encoder, 5 = multi-turn absolute encoder.",
     motion_core::ParameterValue::from_signed(4), motion_core::ParameterValue::from_signed(5)},

    {parameter_id(EthercatParameter::EncoderResolutionBits),
     "hw.encoder_bits", 0x2000, 11, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Encoder resolution (PA95)", "Vendor Specific/Hardware", "bits",
     "Motor encoder resolution in bits. Default 17 (2^17=131072 counts/rev). Modify carefully.",
     motion_core::ParameterValue::from_signed(10), motion_core::ParameterValue::from_signed(32)},

    {parameter_id(EthercatParameter::BasicInitialStatusDisplay),
     "basic.initial_status_display", 0x2000, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Initial status display (PA3)", "Vendor Specific/Basic", "", "Initial panel status display code.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(23)},

    {parameter_id(EthercatParameter::BasicActionOnStop),
     "basic.action_on_stop", 0x2000, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake delay on stop (PA47)", "Vendor Specific/Basic", "x10ms", "Delay to release brake after motor stops.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(200)},

    {parameter_id(EthercatParameter::BasicActuatorAction),
     "basic.actuator_action", 0x2000, 3, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake delay on run (PA48)", "Vendor Specific/Basic", "x10ms", "Delay to engage brake after motor runs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(1000)},

    {parameter_id(EthercatParameter::BasicRunningBrakeSpeed),
     "basic.running_brake_speed", 0x2000, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake speed (PA49)", "Vendor Specific/Basic", "rpm", "Speed threshold for brake logic.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(3000)},

    {parameter_id(EthercatParameter::BasicTorqueControlSpeedLimit),
     "basic.torque_speed_limit", 0x2000, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Torque mode speed (PA50)", "Vendor Specific/Basic", "rpm", "Speed limit in torque control mode.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(5000)},

    {parameter_id(EthercatParameter::BasicServoOnDelayMs),
     "basic.servo_on_delay", 0x2000, 6, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Servo OFF delay (PA54)", "Vendor Specific/Basic", "ms", "Delay to cut current after servo off signal.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(30000)},

    {parameter_id(EthercatParameter::BasicIoInputActiveLevelWord),
     "basic.io_input_active_level", 0x2000, 7, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input logic word (PA55)", "Vendor Specific/Basic", "", "Bitmask to invert digital inputs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(31)},

    // 0x2003 "Input Port Function Definition" (DT2003 from ESI XML)
    {parameter_id(EthercatParameter::InputPort1Function),
     "io.input_port_1_function", 0x2003, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 1 Function", "Vendor Specific/Input Ports", "", "Input port 1 function code (P3-0).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort2Function),
     "io.input_port_2_function", 0x2003, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 2 Function", "Vendor Specific/Input Ports", "", "Input port 2 function code (P3-1).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort3Function),
     "io.input_port_3_function", 0x2003, 3, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 3 Function", "Vendor Specific/Input Ports", "", "Input port 3 function code (P3-2).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort4Function),
     "io.input_port_4_function", 0x2003, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 4 Function", "Vendor Specific/Input Ports", "", "Input port 4 function code (P3-3).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort5Function),
     "io.input_port_5_function", 0x2003, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 5 Function", "Vendor Specific/Input Ports", "", "Input port 5 function code (P3-4).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort6Function),
     "io.input_port_6_function", 0x2003, 6, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 6 Function", "Vendor Specific/Input Ports", "", "Input port 6 function code (P3-5).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort7Function),
     "io.input_port_7_function", 0x2003, 7, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 7 Function", "Vendor Specific/Input Ports", "", "Input port 7 function code (P3-6).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::InputPort8Function),
     "io.input_port_8_function", 0x2003, 8, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input Port 8 Function", "Vendor Specific/Input Ports", "", "Input port 8 function code (P3-7).",
     motion_core::ParameterValue::from_signed(-32768), motion_core::ParameterValue::from_signed(32767)},

    {parameter_id(EthercatParameter::BasicIoOutputActiveLevelWord),
     "basic.io_output_active_level", 0x2000, 8, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Output logic word (PA57)", "Vendor Specific/Basic", "", "Bitmask to invert digital outputs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(31)},

    {parameter_id(EthercatParameter::BasicIoDebounceMs),
     "basic.io_debounce", 0x2000, 9, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input filter / debounce (PA58)", "Vendor Specific/Basic", "ms", "IO jitter suppression time constant.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    {parameter_id(EthercatParameter::BasicMotorPoles),
     "basic.motor_poles", 0x2000, 12, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Motor poles (PA96)", "Vendor Specific/Basic", "", "Configured motor pole count.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(360)},

    {parameter_id(EthercatParameter::BasicPwmDutyCycle),
     "basic.pwm_duty", 0x2000, 13, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "PWM duty cycle (PA99)", "Vendor Specific/Basic", "%", "PWM duty cycle limit.",
     motion_core::ParameterValue::from_signed(5), motion_core::ParameterValue::from_signed(90)},

    // ── Vendor Specific/Communication ──────────────────────────────────────
    {parameter_id(EthercatParameter::CommunicationFixedAddress),
     "comm.fixed_address", 0x2002, 3, 2, motion_core::ParameterValueType::SignedInteger, false, false,
     "Fixed address", "Vendor Specific/Communication", "", "Communication fixed address (bootstrap).",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    // ── Probe ───────────────────────────────────────────────────────────────
    {parameter_id(EthercatParameter::ProbeFunction),
     "probe.function", 0x60B8, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Probe function", "Common/Probe", "", "Probe function configuration.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(65535)},

    {parameter_id(EthercatParameter::ProbeStatus),
     "probe.status", 0x60B9, 0, 4, motion_core::ParameterValueType::UnsignedInteger, true, false,
     "Probe status", "Common/Probe", "", "Probe status bitfield.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {parameter_id(EthercatParameter::Probe1RisePosition),
     "probe.p1_rise", 0x60BA, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 1 rise position", "Common/Probe", "instr", "Probe 1 rising edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {parameter_id(EthercatParameter::Probe1FallPosition),
     "probe.p1_fall", 0x60BB, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 1 fall position", "Common/Probe", "instr", "Probe 1 falling edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {parameter_id(EthercatParameter::Probe2RisePosition),
     "probe.p2_rise", 0x60BC, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 2 rise position", "Common/Probe", "instr", "Probe 2 rising edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {parameter_id(EthercatParameter::Probe2FallPosition),
     "probe.p2_fall", 0x60BD, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 2 fall position", "Common/Probe", "instr", "Probe 2 falling edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},
}};

namespace detail {

[[nodiscard]] inline motion_core::Result<double> parameter_value_to_double(
    const motion_core::ParameterValue& value) {
    switch (value.type) {
        case motion_core::ParameterValueType::SignedInteger:
            return motion_core::Result<double>::success(static_cast<double>(value.signed_value));
        case motion_core::ParameterValueType::UnsignedInteger:
            return motion_core::Result<double>::success(static_cast<double>(value.unsigned_value));
        case motion_core::ParameterValueType::FloatingPoint:
            return motion_core::Result<double>::success(value.floating_value);
        case motion_core::ParameterValueType::Boolean:
            return motion_core::Result<double>::success(value.bool_value ? 1.0 : 0.0);
    }
    return motion_core::Result<double>::failure(
        {motion_core::ErrorCode::InvalidArgument, "Unsupported ParameterValueType"});
}

[[nodiscard]] inline motion_core::Result<double> compute_degrees_per_instruction(
    const EthercatConversionContext& context) {
    if (context.axial_resolution_instr_per_motor_rev == 0U) {
        return motion_core::Result<double>::failure(
            {motion_core::ErrorCode::InvalidArgument, "Invalid conversion context: axial_resolution must be > 0"});
    }

    const double denominator = static_cast<double>(context.axial_resolution_instr_per_motor_rev);
    if (!(denominator > 0.0) || !std::isfinite(denominator)) {
        return motion_core::Result<double>::failure(
            {motion_core::ErrorCode::InvalidArgument, "Invalid conversion context denominator"});
    }
    return motion_core::Result<double>::success(360.0 / denominator);
}

[[nodiscard]] inline motion_core::Result<motion_core::ParameterValue> double_to_raw_typed_value(
    const double value,
    const motion_core::ParameterValueType type,
    const std::uint8_t data_size) {
    if (!std::isfinite(value)) {
        return motion_core::Result<motion_core::ParameterValue>::failure(
            {motion_core::ErrorCode::InvalidArgument, "Conversion produced non-finite value"});
    }

    switch (type) {
        case motion_core::ParameterValueType::UnsignedInteger: {
            if (value < 0.0) {
                return motion_core::Result<motion_core::ParameterValue>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "Negative value cannot be converted to unsigned integer"});
            }
            const double rounded = std::llround(value);
            std::uint64_t max_u = std::numeric_limits<std::uint32_t>::max();
            if (data_size == 1U) max_u = std::numeric_limits<std::uint8_t>::max();
            else if (data_size == 2U) max_u = std::numeric_limits<std::uint16_t>::max();
            else if (data_size >= 8U) max_u = std::numeric_limits<std::uint64_t>::max();
            const auto clamped = static_cast<std::uint64_t>(
                std::clamp(rounded, 0.0, static_cast<double>(max_u)));
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_unsigned(clamped));
        }
        case motion_core::ParameterValueType::SignedInteger: {
            const double rounded = std::llround(value);
            std::int64_t min_s = std::numeric_limits<std::int32_t>::min();
            std::int64_t max_s = std::numeric_limits<std::int32_t>::max();
            if (data_size == 1U) {
                min_s = std::numeric_limits<std::int8_t>::min();
                max_s = std::numeric_limits<std::int8_t>::max();
            } else if (data_size == 2U) {
                min_s = std::numeric_limits<std::int16_t>::min();
                max_s = std::numeric_limits<std::int16_t>::max();
            }
            const auto clamped = static_cast<std::int64_t>(
                std::clamp(rounded, static_cast<double>(min_s), static_cast<double>(max_s)));
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_signed(clamped));
        }
        case motion_core::ParameterValueType::FloatingPoint:
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_floating(value));
        case motion_core::ParameterValueType::Boolean:
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_bool(value >= 0.5));
    }

    return motion_core::Result<motion_core::ParameterValue>::failure(
        {motion_core::ErrorCode::InvalidArgument, "Unsupported ParameterValueType for raw conversion"});
}

} // namespace detail

[[nodiscard]] inline motion_core::Result<EthercatConversionKind> parse_conversion_kind(
    const char* conversion_id) {
    if (!conversion_id || conversion_id[0] == '\0' ||
        std::strcmp(conversion_id, "NoConversion") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(EthercatConversionKind::NoConversion);
    }
    if (std::strcmp(conversion_id, "RawInstructionPosition_OutputDegrees") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(
            EthercatConversionKind::RawInstructionPosition_OutputDegrees);
    }
    if (std::strcmp(conversion_id, "RawInstructionVelocity_OutputDegPerSec") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(
            EthercatConversionKind::RawInstructionVelocity_OutputDegPerSec);
    }
    if (std::strcmp(conversion_id, "RawInstructionAcceleration_OutputDegPerSec2") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(
            EthercatConversionKind::RawInstructionAcceleration_OutputDegPerSec2);
    }
    if (std::strcmp(conversion_id, "RawTenthsPercent_Percent") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(
            EthercatConversionKind::RawTenthsPercent_Percent);
    }
    if (std::strcmp(conversion_id, "RawPercentPerMs_PercentPerMs") == 0) {
        return motion_core::Result<EthercatConversionKind>::success(
            EthercatConversionKind::RawPercentPerMs_PercentPerMs);
    }
    return motion_core::Result<EthercatConversionKind>::failure(
        {motion_core::ErrorCode::NotFound, "Unknown EtherCAT conversion_id"});
}

[[nodiscard]] inline motion_core::Result<motion_core::ParameterValue> convert_from_raw_to_display(
    const ParameterDefinition& definition,
    const motion_core::ParameterValue& raw_value,
    const EthercatConversionContext& context) {
    const auto conversion_kind = parse_conversion_kind(definition.conversion_id);
    if (!conversion_kind.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(conversion_kind.error());
    }

    if (conversion_kind.value() == EthercatConversionKind::NoConversion) {
        return motion_core::Result<motion_core::ParameterValue>::success(raw_value);
    }

    const auto raw_double = detail::parameter_value_to_double(raw_value);
    if (!raw_double.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(raw_double.error());
    }

    if (conversion_kind.value() == EthercatConversionKind::RawTenthsPercent_Percent ||
        conversion_kind.value() == EthercatConversionKind::RawPercentPerMs_PercentPerMs) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_floating(raw_double.value() / 10.0));
    }

    const auto deg_per_instruction = detail::compute_degrees_per_instruction(context);
    if (!deg_per_instruction.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(deg_per_instruction.error());
    }

    return motion_core::Result<motion_core::ParameterValue>::success(
        motion_core::ParameterValue::from_floating(raw_double.value() * deg_per_instruction.value()));
}

[[nodiscard]] inline motion_core::Result<motion_core::ParameterValue> convert_from_display_to_raw(
    const ParameterDefinition& definition,
    const motion_core::ParameterValue& display_value,
    const EthercatConversionContext& context) {
    const auto conversion_kind = parse_conversion_kind(definition.conversion_id);
    if (!conversion_kind.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(conversion_kind.error());
    }

    if (conversion_kind.value() == EthercatConversionKind::NoConversion) {
        return motion_core::Result<motion_core::ParameterValue>::success(display_value);
    }

    const auto display_double = detail::parameter_value_to_double(display_value);
    if (!display_double.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(display_double.error());
    }

    if (conversion_kind.value() == EthercatConversionKind::RawTenthsPercent_Percent ||
        conversion_kind.value() == EthercatConversionKind::RawPercentPerMs_PercentPerMs) {
        return detail::double_to_raw_typed_value(
            display_double.value() * 10.0,
            definition.type,
            definition.data_size);
    }

    const auto deg_per_instruction = detail::compute_degrees_per_instruction(context);
    if (!deg_per_instruction.ok()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(deg_per_instruction.error());
    }
    return detail::double_to_raw_typed_value(
        display_double.value() / deg_per_instruction.value(),
        definition.type,
        definition.data_size);
}

[[nodiscard]] inline motion_core::Result<motion_core::ParameterValue> convert_display_min_to_raw(
    const ParameterDefinition& definition,
    const EthercatConversionContext& context) {
    return convert_from_display_to_raw(definition, definition.min_value, context);
}

[[nodiscard]] inline motion_core::Result<motion_core::ParameterValue> convert_display_max_to_raw(
    const ParameterDefinition& definition,
    const EthercatConversionContext& context) {
    return convert_from_display_to_raw(definition, definition.max_value, context);
}

[[nodiscard]] inline const ParameterDefinition* find_ethercat_parameter_definition(
    const motion_core::ParameterId id) {
    for (const auto& definition : p100e_dictionary) {
        if (definition.id.domain == id.domain && definition.id.value == id.value) {
            return &definition;
        }
    }
    return nullptr;
}

} // namespace ethercat_driver

namespace motion_core {

using EthercatParameter = ::ethercat_driver::EthercatParameter;

constexpr ParameterId make_parameter_id(const ::ethercat_driver::EthercatParameter parameter) {
    return ::ethercat_driver::parameter_id(parameter);
}

} // namespace motion_core
