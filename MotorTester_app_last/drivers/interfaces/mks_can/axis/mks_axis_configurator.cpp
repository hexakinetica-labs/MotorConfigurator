#include "mks_can/axis/mks_axis_configurator.h"

#include "mks_can/dictionary/mks_dictionary.h"
#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace {

using motion_core::ErrorCode;
using motion_core::ParameterDomain;
using motion_core::ParameterValue;
using motion_core::Result;

[[nodiscard]] double compute_axis_units_per_degree(const double software_gear_ratio,
                                                   const std::uint32_t encoder_resolution_bits) {
    if (!(std::isfinite(software_gear_ratio) && software_gear_ratio > 0.0)
        || encoder_resolution_bits == 0U
        || encoder_resolution_bits > 31U) {
        return 0.0;
    }
    const auto pulses_per_rev = static_cast<double>(1ULL << encoder_resolution_bits);
    return (pulses_per_rev * software_gear_ratio) / 360.0;
}

[[nodiscard]] Result<void> ensure_status_ok(const Result<std::vector<std::uint8_t>>& write_result) {
    if (!write_result.ok()) {
        return Result<void>::failure(write_result.error());
    }
    if (write_result.value().empty()) {
        return Result<void>::failure({ErrorCode::ProtocolFailure, "MKS write response is empty"});
    }
    if (write_result.value().front() != 0x01U) {
        return Result<void>::failure({ErrorCode::ProtocolFailure, "MKS write rejected by drive"});
    }
    return Result<void>::success();
}

[[nodiscard]] Result<void> ensure_can_id_status_ok(const Result<std::vector<std::uint8_t>>& write_result,
                                                   const std::uint16_t requested_can_id) {
    if (!write_result.ok()) {
        return Result<void>::failure(write_result.error());
    }
    if (write_result.value().empty()) {
        return Result<void>::failure({ErrorCode::ProtocolFailure, "MKS SetCanId response is empty"});
    }
    if (write_result.value().front() == 0x01U) {
        return Result<void>::success();
    }
    std::uint16_t echoed_can_id = 0U;
    if (write_result.value().size() >= 2U) {
        echoed_can_id = static_cast<std::uint16_t>(
            mks::MksProtocol::readBe16(write_result.value().data() + write_result.value().size() - 2U) & 0x07FFU);
    } else {
        echoed_can_id = static_cast<std::uint16_t>(write_result.value().back() & 0x07FFU);
    }
    if (echoed_can_id == requested_can_id) {
        return Result<void>::success();
    }
    return Result<void>::failure({ErrorCode::ProtocolFailure, "MKS SetCanId rejected"});
}

class BusyFlagScope final {
public:
    explicit BusyFlagScope(mks::MksAxisStateCache* state_cache)
        : state_cache_(state_cache) {
        if (state_cache_) {
            state_cache_->set_config_busy(true);
        }
    }

    ~BusyFlagScope() {
        if (state_cache_) {
            state_cache_->set_config_busy(false);
        }
    }

    BusyFlagScope(const BusyFlagScope&) = delete;
    BusyFlagScope& operator=(const BusyFlagScope&) = delete;

private:
    mks::MksAxisStateCache* state_cache_{nullptr};
};

} // namespace

namespace mks {

MksAxisConfigurator::MksAxisConfigurator(Config config)
    : config_(std::move(config)) {
    runtime_can_id_.store(config_.can_id, std::memory_order_release);
    software_gear_ratio_.store(config_.software_gear_ratio, std::memory_order_release);
    axis_units_per_degree_.store(config_.axis_units_per_degree, std::memory_order_release);
    software_invert_direction_.store(config_.invert_direction, std::memory_order_release);
    const double default_velocity = (static_cast<double>(config_.default_speed_rpm) * 6.0)
        / std::max(0.001, config_.software_gear_ratio);
    limits_max_velocity_deg_per_sec_.store(default_velocity, std::memory_order_release);
    limits_max_accel_percent_.store((static_cast<double>(config_.default_accel_byte) / 255.0) * 100.0,
                                    std::memory_order_release);
}

Result<void> MksAxisConfigurator::start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return Result<void>::success();
    }
    if (!config_.bus_manager) {
        started_.store(false, std::memory_order_release);
        return Result<void>::failure({ErrorCode::InvalidArgument, "bus_manager is null"});
    }
    return Result<void>::success();
}

Result<void> MksAxisConfigurator::stop() {
    started_.store(false, std::memory_order_release);
    return Result<void>::success();
}

Result<std::vector<motion_core::ParameterDescriptor>> MksAxisConfigurator::list_parameters() const {
    return Result<std::vector<motion_core::ParameterDescriptor>>::success(get_mks_parameter_descriptors());
}

Result<motion_core::ParameterSet> MksAxisConfigurator::read_parameters() const {
    const auto status = ensure_started();
    if (!status.ok()) {
        return Result<motion_core::ParameterSet>::failure(status.error());
    }

    if (config_.state_cache) {
        config_.state_cache->set_config_busy(true);
    }

    motion_core::ParameterSet set{};
    auto add_value = [&set](const motion_core::ParameterId id, const ParameterValue& value) {
        if (find_mks_descriptor(id)) {
            set.entries.push_back({id, value});
        }
    };

    add_value(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio),
              ParameterValue::from_floating(software_gear_ratio_.load(std::memory_order_acquire)));
    add_value(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareEncoderResolutionBits),
              ParameterValue::from_unsigned(software_encoder_resolution_bits_.load(std::memory_order_acquire)));
    add_value(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareInvertDirection),
              ParameterValue::from_bool(software_invert_direction_.load(std::memory_order_acquire)));
    add_value(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxVelocityDegPerSec),
              ParameterValue::from_floating(limits_max_velocity_deg_per_sec_.load(std::memory_order_acquire)));
    add_value(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2),
              ParameterValue::from_floating(limits_max_accel_percent_.load(std::memory_order_acquire)));

    const auto can_id = runtime_can_id();
    auto read_param = [this, can_id](const MksCommand command) {
        return config_.bus_manager->read_system_parameter(can_id, static_cast<std::uint8_t>(command));
    };

    if (const auto res = read_param(MksCommand::SetWorkMode); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::WorkMode),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetWorkingCurrent); res.ok() && res.value().size() >= 2U) {
        add_value(motion_core::make_parameter_id(MksParameter::WorkingCurrentMilliAmp),
                  ParameterValue::from_unsigned(MksProtocol::readBe16(res.value().data() + res.value().size() - 2U)));
    }
    if (const auto res = read_param(MksCommand::SetSubdivision); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::Subdivision),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetEnPinActiveLevel); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::EnPinActiveLevel),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetMotorDirection); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::MotorDirection),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetAutoTurnOffScreen); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::AutoScreenOff),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetLockedRotorProtection); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::LockedRotorProtection),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetSubdivisionInterpolation); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::SubdivisionInterpolation),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetCanBitrate); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::CanBitrateIndex),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetCanId); res.ok() && !res.value().empty()) {
        std::uint64_t current_can_id = res.value().back();
        if (res.value().size() >= 2U) {
            current_can_id = MksProtocol::readBe16(res.value().data() + res.value().size() - 2U) & 0x07FFu;
        }
        add_value(motion_core::make_parameter_id(MksParameter::CanId),
                  ParameterValue::from_unsigned(current_can_id));
    }
    if (const auto res = read_param(MksCommand::SetGroupId); res.ok() && !res.value().empty()) {
        std::uint64_t group_id = res.value().back();
        if (res.value().size() >= 2U) {
            group_id = MksProtocol::readBe16(res.value().data() + res.value().size() - 2U) & 0x07FFu;
        }
        add_value(motion_core::make_parameter_id(MksParameter::GroupId),
                  ParameterValue::from_unsigned(group_id));
    }
    if (const auto res = read_param(MksCommand::SetKeyLock); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::KeyLock),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetHoldingCurrent); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::HoldingCurrentIndex),
                  ParameterValue::from_unsigned(res.value().back()));
    }
    if (const auto res = read_param(MksCommand::SetLimitPortRemap); res.ok() && !res.value().empty()) {
        add_value(motion_core::make_parameter_id(MksParameter::LimitPortRemap),
                  ParameterValue::from_unsigned(res.value().back()));
    }

    if (config_.state_cache) {
        const auto telem = config_.state_cache->latest_telemetry();
        add_value(motion_core::make_parameter_id(MksParameter::AxisPositionRaw),
                  ParameterValue::from_signed(telem.raw_axis_position));

        const auto gear_ratio = std::max(0.001, software_gear_ratio_.load(std::memory_order_acquire));
        const auto rpm = std::llround((telem.actual_velocity_deg_per_sec * gear_ratio) / 6.0);
        add_value(motion_core::make_parameter_id(MksParameter::MotorSpeedRpm),
                  ParameterValue::from_signed(static_cast<std::int64_t>(rpm)));
        add_value(motion_core::make_parameter_id(MksParameter::ProtectionState),
                  ParameterValue::from_unsigned(telem.protection_code));
        add_value(motion_core::make_parameter_id(MksParameter::MotorStatus),
                  ParameterValue::from_unsigned(telem.motion_status_code));
    }

    if (config_.state_cache) {
        config_.state_cache->set_config_busy(false);
    }
    return Result<motion_core::ParameterSet>::success(std::move(set));
}

Result<void> MksAxisConfigurator::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }

    BusyFlagScope busy_guard(config_.state_cache);

    for (const auto& entry : patch.entries) {
        const auto* descriptor = find_mks_descriptor(entry.id);
        if (!descriptor) {
            return Result<void>::failure({ErrorCode::NotFound, "parameter not found"});
        }
        if (descriptor->read_only) {
            return Result<void>::failure({ErrorCode::Unsupported, "parameter is read-only"});
        }

        if (entry.id.domain == ParameterDomain::Common) {
            const auto common_parameter = static_cast<motion_core::CommonParameter>(entry.id.value);
            if (common_parameter == motion_core::CommonParameter::HardwareGearRatio) {
                const auto ratio = require_floating_value(entry.value, "HardwareGearRatio requires floating point");
                if (!ratio.ok() || !(ratio.value() > 0.0) || !std::isfinite(ratio.value())) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "invalid HardwareGearRatio"});
                }
                software_gear_ratio_.store(ratio.value(), std::memory_order_release);
                const auto bits = software_encoder_resolution_bits_.load(std::memory_order_acquire);
                const auto axis_units = compute_axis_units_per_degree(ratio.value(), bits);
                if (!(std::isfinite(axis_units) && std::abs(axis_units) > 1e-9)) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "invalid axis units after HardwareGearRatio change"});
                }
                axis_units_per_degree_.store(axis_units, std::memory_order_release);
                continue;
            }
            if (common_parameter == motion_core::CommonParameter::HardwareEncoderResolutionBits) {
                const auto bits = require_unsigned_value(entry.value, "HardwareEncoderResolutionBits requires uint");
                if (!bits.ok() || bits.value() < 1U || bits.value() > 31U) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "invalid HardwareEncoderResolutionBits"});
                }
                const auto bits_u32 = static_cast<std::uint32_t>(bits.value());
                software_encoder_resolution_bits_.store(bits_u32, std::memory_order_release);
                const auto ratio = software_gear_ratio_.load(std::memory_order_acquire);
                const auto axis_units = compute_axis_units_per_degree(ratio, bits_u32);
                if (!(std::isfinite(axis_units) && std::abs(axis_units) > 1e-9)) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "invalid axis units after HardwareEncoderResolutionBits change"});
                }
                axis_units_per_degree_.store(axis_units, std::memory_order_release);
                continue;
            }
            if (common_parameter == motion_core::CommonParameter::HardwareInvertDirection) {
                const auto invert = require_bool_value(entry.value, "HardwareInvertDirection requires bool");
                if (!invert.ok()) {
                    return Result<void>::failure(invert.error());
                }
                software_invert_direction_.store(invert.value(), std::memory_order_release);
                continue;
            }
            if (common_parameter == motion_core::CommonParameter::LimitsMaxVelocityDegPerSec) {
                const auto v = require_floating_value(entry.value, "LimitsMaxVelocityDegPerSec requires float");
                if (!v.ok() || !(v.value() > 0.0)) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "invalid LimitsMaxVelocityDegPerSec"});
                }
                limits_max_velocity_deg_per_sec_.store(v.value(), std::memory_order_release);
                continue;
            }
            if (common_parameter == motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2) {
                const auto a = require_floating_value(entry.value, "LimitsMaxAccelerationDegPerSec2 requires float");
                if (!a.ok()) {
                    return Result<void>::failure(a.error());
                }
                if (!(a.value() >= 0.0 && a.value() <= 100.0) || !std::isfinite(a.value())) {
                    return Result<void>::failure(
                        {ErrorCode::InvalidArgument,
                         "LimitsMaxAccelerationDegPerSec2 is interpreted as percent in [0..100]"});
                }
                limits_max_accel_percent_.store(a.value(), std::memory_order_release);
                continue;
            }
            continue;
        }

        if (entry.id.domain != ParameterDomain::Mks) {
            continue;
        }

        const auto can_id = runtime_can_id();
        std::vector<std::uint8_t> payload;
        const auto parameter = static_cast<MksParameter>(entry.id.value);
        switch (parameter) {
            case MksParameter::WorkMode: {
                const auto value = require_unsigned_value(entry.value, "WorkMode requires uint");
                if (!value.ok() || value.value() > 5U) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "WorkMode out of range"});
                }
                payload.push_back(static_cast<std::uint8_t>(value.value()));
                const auto write_result = config_.bus_manager->execute_raw_command_sync(
                    can_id, static_cast<std::uint8_t>(MksCommand::SetWorkMode), payload);
                const auto check = ensure_status_ok(write_result);
                if (!check.ok()) {
                    return check;
                }
                break;
            }
            case MksParameter::WorkingCurrentMilliAmp: {
                const auto value = require_unsigned_value(entry.value, "WorkingCurrent requires uint");
                if (!value.ok() || value.value() > 5200U) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "WorkingCurrent out of range"});
                }
                MksProtocol::appendBe16(payload, static_cast<std::uint16_t>(value.value()));
                const auto write_result = config_.bus_manager->execute_raw_command_sync(
                    can_id, static_cast<std::uint8_t>(MksCommand::SetWorkingCurrent), payload);
                const auto check = ensure_status_ok(write_result);
                if (!check.ok()) {
                    return check;
                }
                break;
            }
            case MksParameter::Subdivision:
            case MksParameter::EnPinActiveLevel:
            case MksParameter::MotorDirection:
            case MksParameter::AutoScreenOff:
            case MksParameter::LockedRotorProtection:
            case MksParameter::SubdivisionInterpolation:
            case MksParameter::CanBitrateIndex:
            case MksParameter::KeyLock:
            case MksParameter::HoldingCurrentIndex:
            case MksParameter::LimitPortRemap: {
                const auto value = require_unsigned_value(entry.value, "parameter requires uint");
                if (!value.ok() || value.value() > 255U) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "parameter out of range"});
                }
                payload.push_back(static_cast<std::uint8_t>(value.value()));
                MksCommand cmd = MksCommand::SetSubdivision;
                if (parameter == MksParameter::EnPinActiveLevel) cmd = MksCommand::SetEnPinActiveLevel;
                else if (parameter == MksParameter::MotorDirection) cmd = MksCommand::SetMotorDirection;
                else if (parameter == MksParameter::AutoScreenOff) cmd = MksCommand::SetAutoTurnOffScreen;
                else if (parameter == MksParameter::LockedRotorProtection) cmd = MksCommand::SetLockedRotorProtection;
                else if (parameter == MksParameter::SubdivisionInterpolation) cmd = MksCommand::SetSubdivisionInterpolation;
                else if (parameter == MksParameter::CanBitrateIndex) cmd = MksCommand::SetCanBitrate;
                else if (parameter == MksParameter::KeyLock) cmd = MksCommand::SetKeyLock;
                else if (parameter == MksParameter::HoldingCurrentIndex) cmd = MksCommand::SetHoldingCurrent;
                else if (parameter == MksParameter::LimitPortRemap) cmd = MksCommand::SetLimitPortRemap;
                const auto write_result = config_.bus_manager->execute_raw_command_sync(
                    can_id, static_cast<std::uint8_t>(cmd), payload);
                const auto check = ensure_status_ok(write_result);
                if (!check.ok()) {
                    return check;
                }
                break;
            }
            case MksParameter::CanId: {
                const auto value = require_unsigned_value(entry.value, "CanId requires uint");
                if (!value.ok() || value.value() < 1U || value.value() > 0x7FFU) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "CanId out of range"});
                }
                const auto new_can_id = static_cast<std::uint16_t>(value.value());
                MksProtocol::appendBe16(payload, new_can_id);
                const auto write_result = config_.bus_manager->execute_raw_command_sync(
                    can_id, static_cast<std::uint8_t>(MksCommand::SetCanId), payload);
                const auto check = ensure_can_id_status_ok(write_result, new_can_id);
                if (!check.ok()) {
                    return check;
                }
                if (config_.on_runtime_can_id_changed) {
                    const auto remap_result = config_.on_runtime_can_id_changed(new_can_id);
                    if (!remap_result.ok()) {
                        return Result<void>::failure(remap_result.error());
                    }
                }
                runtime_can_id_.store(new_can_id, std::memory_order_release);
                break;
            }
            case MksParameter::GroupId: {
                const auto value = require_unsigned_value(entry.value, "GroupId requires uint");
                if (!value.ok() || value.value() < 1U || value.value() > 0x7FFU) {
                    return Result<void>::failure({ErrorCode::InvalidArgument, "GroupId out of range"});
                }
                MksProtocol::appendBe16(payload, static_cast<std::uint16_t>(value.value()));
                const auto write_result = config_.bus_manager->execute_raw_command_sync(
                    can_id, static_cast<std::uint8_t>(MksCommand::SetGroupId), payload);
                const auto check = ensure_status_ok(write_result);
                if (!check.ok()) {
                    return check;
                }
                break;
            }
            case MksParameter::EnableMotor:
            case MksParameter::SlaveRespondMode:
            case MksParameter::SlaveActiveReport:
            case MksParameter::AxisPositionRaw:
            case MksParameter::MotorSpeedRpm:
            case MksParameter::ProtectionState:
            case MksParameter::MotorStatus:
                return Result<void>::failure({ErrorCode::Unsupported, "parameter is runtime/service-owned"});
        }
    }

    return Result<void>::success();
}

Result<motion_core::PersistentWriteReport> MksAxisConfigurator::set_persistent(
    const motion_core::PersistentCommand command,
    const motion_core::ParameterValue& value) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return Result<motion_core::PersistentWriteReport>::failure(status.error());
    }

    BusyFlagScope busy_guard(config_.state_cache);

    motion_core::PersistentWriteReport report{};
    report.command_supported = true;

    if (command == motion_core::PersistentCommand::CanId) {
        const auto can_id_value = require_unsigned_value(value, "CanId requires uint");
        if (!can_id_value.ok() || can_id_value.value() < 1U || can_id_value.value() > 0x7FFU) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::InvalidArgument, "CanId out of range"});
        }
        motion_core::ParameterPatch patch{};
        patch.entries.push_back({motion_core::make_parameter_id(MksParameter::CanId),
                                 ParameterValue::from_unsigned(can_id_value.value())});
        const auto apply_result = apply_parameter_patch(patch);
        if (!apply_result.ok()) {
            return Result<motion_core::PersistentWriteReport>::failure(apply_result.error());
        }

        report.write_completed = true;
        report.persistent_save_completed = true;
        report.reconnect_required = true;

        const auto readback = config_.bus_manager->read_system_parameter(
            runtime_can_id(), static_cast<std::uint8_t>(MksCommand::SetCanId));
        if (!readback.ok() || readback.value().empty()) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::ProtocolFailure, "CanId readback failed"});
        }
        std::uint64_t readback_can_id = readback.value().back();
        if (readback.value().size() >= 2U) {
            readback_can_id = MksProtocol::readBe16(readback.value().data() + readback.value().size() - 2U) & 0x07FFU;
        }
        report.readback_value = ParameterValue::from_unsigned(readback_can_id);
        report.readback_verified = (readback_can_id == can_id_value.value());
        if (!report.readback_verified) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::ProtocolFailure, "CanId readback mismatch"});
        }
        return Result<motion_core::PersistentWriteReport>::success(report);
    }

    if (command == motion_core::PersistentCommand::CanBitrate) {
        const auto bitrate_value = require_unsigned_value(value, "CanBitrate requires uint");
        if (!bitrate_value.ok() || bitrate_value.value() > 3U) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::InvalidArgument, "CanBitrate index out of range"});
        }

        motion_core::ParameterPatch patch{};
        patch.entries.push_back({motion_core::make_parameter_id(MksParameter::CanBitrateIndex),
                                 ParameterValue::from_unsigned(bitrate_value.value())});
        const auto apply_result = apply_parameter_patch(patch);
        if (!apply_result.ok()) {
            return Result<motion_core::PersistentWriteReport>::failure(apply_result.error());
        }

        report.write_completed = true;
        report.persistent_save_completed = true;
        report.reconnect_required = true;

        const auto readback = config_.bus_manager->read_system_parameter(
            runtime_can_id(), static_cast<std::uint8_t>(MksCommand::SetCanBitrate));
        if (!readback.ok() || readback.value().empty()) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::ProtocolFailure, "CanBitrate readback failed"});
        }

        const auto readback_idx = static_cast<std::uint64_t>(readback.value().back());
        report.readback_value = ParameterValue::from_unsigned(readback_idx);
        report.readback_verified = (readback_idx == bitrate_value.value());
        if (!report.readback_verified) {
            return Result<motion_core::PersistentWriteReport>::failure(
                {ErrorCode::ProtocolFailure, "CanBitrate readback mismatch"});
        }
        return Result<motion_core::PersistentWriteReport>::success(report);
    }

    return Result<motion_core::PersistentWriteReport>::failure(
        {ErrorCode::Unsupported, "Persistent command is not supported by MKS configurator"});
}

double MksAxisConfigurator::software_gear_ratio() const {
    return software_gear_ratio_.load(std::memory_order_acquire);
}

std::uint32_t MksAxisConfigurator::software_encoder_resolution_bits() const {
    return software_encoder_resolution_bits_.load(std::memory_order_acquire);
}

double MksAxisConfigurator::axis_units_per_degree() const {
    return axis_units_per_degree_.load(std::memory_order_acquire);
}

bool MksAxisConfigurator::software_invert_direction() const {
    return software_invert_direction_.load(std::memory_order_acquire);
}

double MksAxisConfigurator::limits_max_velocity_deg_per_sec() const {
    return limits_max_velocity_deg_per_sec_.load(std::memory_order_acquire);
}

double MksAxisConfigurator::limits_max_accel_percent() const {
    return limits_max_accel_percent_.load(std::memory_order_acquire);
}

Result<void> MksAxisConfigurator::ensure_started() const {
    if (!started_.load(std::memory_order_acquire)) {
        return Result<void>::failure({ErrorCode::NotConnected, "configurator is not started"});
    }
    if (!config_.bus_manager) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "bus_manager is null"});
    }
    return Result<void>::success();
}

std::uint16_t MksAxisConfigurator::runtime_can_id() const {
    return runtime_can_id_.load(std::memory_order_acquire);
}

} // namespace mks
