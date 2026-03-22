#include "mks/adapter/mks_axis_adapter.h"
#include "mks/internal/protocol/mks_protocol.h"
#include "mks/dictionary/mks_dictionary.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <cstring>

namespace {

using motion_core::MksParameter;
using motion_core::ParameterDescriptor;
using motion_core::ParameterDomain;
using motion_core::ParameterValue;

constexpr double kDegreesPerRevolution = 360.0;
constexpr std::uint32_t kDefaultEncoderResolutionBits = 14;

[[nodiscard]] double compute_axis_units_per_degree(const std::uint32_t encoder_bits, const double gear_ratio) {
    const auto bits = std::min<std::uint32_t>(encoder_bits, 31U);
    const double ticks_per_rev = std::ldexp(1.0, static_cast<int>(bits));
    return (ticks_per_rev * gear_ratio) / kDegreesPerRevolution;
}

[[nodiscard]] motion_core::Result<void> ensure_write_status_success(
    const motion_core::Result<std::vector<std::uint8_t>>& write_result) {
    if (!write_result.ok()) {
        return motion_core::Result<void>::failure(write_result.error());
    }
    if (write_result.value().empty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::ProtocolFailure,
             "MKS write response payload is empty"});
    }
    if (write_result.value().front() != 0x01U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::ProtocolFailure,
             "MKS write command rejected by drive (status != 1)"});
    }
    return motion_core::Result<void>::success();
}

[[nodiscard]] motion_core::Result<void> ensure_can_id_write_success(
    const motion_core::Result<std::vector<std::uint8_t>>& write_result,
    const std::uint16_t requested_can_id) {
    if (!write_result.ok()) {
        return motion_core::Result<void>::failure(write_result.error());
    }
    if (write_result.value().empty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::ProtocolFailure,
             "MKS SetCanId response payload is empty"});
    }

    const auto& payload = write_result.value();
    if (payload.front() == 0x01U) {
        return motion_core::Result<void>::success();
    }

    std::uint16_t echoed_can_id = 0U;
    if (payload.size() >= 2U) {
        echoed_can_id = static_cast<std::uint16_t>(
            mks::MksProtocol::readBe16(payload.data() + (payload.size() - 2U)) & 0x07FFU);
    } else {
        echoed_can_id = static_cast<std::uint16_t>(payload.back() & 0x07FFU);
    }

    if (echoed_can_id == requested_can_id) {
        return motion_core::Result<void>::success();
    }

    return motion_core::Result<void>::failure(
        {motion_core::ErrorCode::ProtocolFailure,
         "MKS SetCanId command rejected or malformed response"});
}

} // namespace

namespace mks {

MksAxisAdapter::MksAxisAdapter(MksAxisAdapterConfig config)
    : config_(std::move(config)) {
    axis_units_per_degree_.store(config_.axis_units_per_degree, std::memory_order_relaxed);
    runtime_can_id_.store(config_.can_id, std::memory_order_relaxed);
    software_gear_ratio_ = config_.axis_units_per_degree / compute_axis_units_per_degree(kDefaultEncoderResolutionBits, 1.0);
    if (!(software_gear_ratio_ > 0.0) || std::isnan(software_gear_ratio_) || std::isinf(software_gear_ratio_)) {
        software_gear_ratio_ = 1.0;
    }
    software_encoder_resolution_bits_ = kDefaultEncoderResolutionBits;
    recalculate_axis_units_per_degree_from_software_params();

    pending_speed_.store(config_.default_speed, std::memory_order_relaxed);
    pending_accel_.store(config_.default_accel, std::memory_order_relaxed);
}

MksAxisAdapter::~MksAxisAdapter() {
    (void)stop();
}

motion_core::AxisInfo MksAxisAdapter::info() const {
    motion_core::AxisInfo info{};
    info.id = config_.axis_id;
    info.name = config_.axis_name;
    info.transport = motion_core::AxisTransportKind::CanBus;
    info.capabilities = {
        motion_core::Capability::ReadTelemetry,
        motion_core::Capability::SetTargetPosition,
        motion_core::Capability::SetTargetVelocity,
        motion_core::Capability::EnableDisable,
        motion_core::Capability::EmergencyStop,
        motion_core::Capability::ReadParameters,
        motion_core::Capability::WriteParameters,
        motion_core::Capability::Homing,
    };
    return info;
}

motion_core::Result<void> MksAxisAdapter::start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true)) {
        return motion_core::Result<void>::success();
    }

    if (!config_.bus_manager) {
        started_ = false;
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "bus_manager is null"});
    }
    if (std::abs(axis_units_per_degree_.load(std::memory_order_acquire)) < 1e-9) {
        started_ = false;
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "axis_units_per_degree must be non-zero"});
    }

    if (config_.auto_start_bus_manager) {
        const auto bus_start = config_.bus_manager->start();
        if (!bus_start.ok()) {
            started_ = false;
            return bus_start;
        }
    }

    const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
    const auto reg_result = config_.bus_manager->register_adapter(runtime_can_id, this);
    if (!reg_result.ok()) {
        started_ = false;
        return reg_result;
    }

    // Initialize state
    auto pos_res = config_.bus_manager->execute_raw_command_sync(
        runtime_can_id, static_cast<uint8_t>(MksCommand::ReadEncoderAddition), {});
    if (pos_res.ok()) {
        const auto axis_units_per_degree = axis_units_per_degree_.load(std::memory_order_acquire);
        auto payload = pos_res.value();
        if (payload.size() >= 6) {
            target_position_deg_.store(
                static_cast<double>(MksProtocol::readBe48s(payload.data())) / axis_units_per_degree,
                std::memory_order_relaxed);
        } else if (payload.size() == 4) {
            target_position_deg_.store(
                static_cast<double>(static_cast<std::int32_t>(MksProtocol::readBe32s(payload.data()))) / axis_units_per_degree,
                std::memory_order_relaxed);
        }
    }

    // Set initial target velocity
    target_velocity_deg_s_.store(0.0, std::memory_order_relaxed);

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::stop() {
    bool expected = true;
    if (!started_.compare_exchange_strong(expected, false)) {
        return motion_core::Result<void>::success();
    }

    if (config_.bus_manager) {
        const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
        (void)config_.bus_manager->unregister_adapter(runtime_can_id, this);
    }

    enabled_.store(false, std::memory_order_relaxed);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::set_enabled(const bool enabled) {
    const auto status = ensure_started();
    if (!status.ok()) return status;

    if (enabled && !enabled_.load(std::memory_order_relaxed)) {
        // Only sync target to actual position if telemetry has been populated at least once.
        // If telem_.timestamp_ns == 0, no CAN response has arrived yet and
        // actual_position_deg is 0.0 (stale default) — storing it would overwrite
        // the initial position set by start() and cause a jump-to-zero command.
        std::lock_guard<std::mutex> lock(telem_mutex_);
        if (telem_.timestamp_ns != 0) {
            target_position_deg_.store(telem_.actual_position_deg, std::memory_order_relaxed);
        }
    }

    // We send command immediately to ensure responsive disable, but state is also managed in poll
    std::vector<uint8_t> payload = { static_cast<uint8_t>(enabled ? 1 : 0) };
    const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
    const auto send_res = config_.bus_manager->send_raw_command(
        runtime_can_id, static_cast<uint8_t>(MksCommand::EnableMotor), payload);
    if (!send_res.ok()) {
        return send_res;
    }

    enabled_.store(enabled, std::memory_order_relaxed);

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::set_enabled_locked(const bool enabled) {
    return set_enabled(enabled);
}

motion_core::Result<void> MksAxisAdapter::set_mode(const motion_core::AxisMode mode) {
    const auto status = ensure_started();
    if (!status.ok()) return status;

    const int work_mode = mode_to_work_mode(mode);
    if (work_mode < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "axis mode not supported"});
    }

    std::vector<uint8_t> payload = { static_cast<uint8_t>(work_mode) };
    const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
    const auto send_res = config_.bus_manager->send_raw_command(
        runtime_can_id, static_cast<uint8_t>(MksCommand::SetWorkMode), payload);
    if (!send_res.ok()) {
        return send_res;
    }

    mode_.store(mode, std::memory_order_relaxed);
    
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::apply_command(const motion_core::AxisCommand& command) {
    const auto status = ensure_started();
    if (!status.ok()) return status;

    if (command.emergency_stop) {
        emergency_stop_req_.store(true, std::memory_order_relaxed);
        enabled_.store(false, std::memory_order_relaxed);
        return motion_core::Result<void>::success();
    }

    if (command.clear_errors) {
        clear_errors_req_.store(true, std::memory_order_relaxed);
        double actual_pos = 0.0;
        {
            std::lock_guard<std::mutex> telem_lock(telem_mutex_);
            actual_pos = telem_.actual_position_deg;
        }
        target_position_deg_.store(actual_pos, std::memory_order_relaxed);
    }

    if (command.set_zero) {
        set_zero_req_.store(true, std::memory_order_relaxed);
        has_new_target_pos_.store(false, std::memory_order_release);
        has_new_target_vel_.store(false, std::memory_order_release);
        target_velocity_deg_s_.store(0.0, std::memory_order_relaxed);
        target_position_deg_.store(0.0, std::memory_order_relaxed);
    }

    if (command.go_home) {
        go_home_req_.store(true, std::memory_order_relaxed);
    }

    if (command.has_target_position) {
        if (command.has_profile_speed_rpm) {
            pending_speed_.store(command.profile_speed_rpm, std::memory_order_relaxed);
        }
        if (command.has_profile_accel_percent) {
            const auto accel_byte = static_cast<std::uint8_t>(
                std::llround(std::clamp(command.profile_accel_percent, 0.0, 100.0) / 100.0 * 255.0));
            pending_accel_.store(accel_byte, std::memory_order_relaxed);
        }

        if (command.is_relative) {
            double current_target = target_position_deg_.load(std::memory_order_relaxed);
            while (!target_position_deg_.compare_exchange_weak(
                current_target,
                current_target + command.target_position_deg,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
            }
        } else {
            target_position_deg_.store(command.target_position_deg, std::memory_order_relaxed);
        }
        has_new_target_pos_.store(true, std::memory_order_release);
    }

    if (command.has_target_velocity) {
        target_velocity_deg_s_.store(command.target_velocity_deg_per_sec, std::memory_order_relaxed);
        has_new_target_vel_.store(true, std::memory_order_release);
    }

    return motion_core::Result<void>::success();
}


void MksAxisAdapter::process_cycle(MksProtocol& protocol) {
    if (!started_.load(std::memory_order_relaxed)) return;
    const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);

    // --- Write commands ---
    if (emergency_stop_req_.exchange(false, std::memory_order_relaxed)) {
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, MksCommand::EmergencyStop, {}, r, 0xFF, 0, false);
    }
    if (clear_errors_req_.exchange(false, std::memory_order_relaxed)) {
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, MksCommand::ReleaseStallProtection, {}, r, 0xFF, 0, false);
    }
    const bool set_zero_executed = set_zero_req_.exchange(false, std::memory_order_relaxed);
    if (set_zero_executed) {
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, static_cast<MksCommand>(0x92), {}, r, 0xFF, 0, false);
    }
    if (go_home_req_.exchange(false, std::memory_order_relaxed)) {
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, static_cast<MksCommand>(0x91), {}, r, 0xFF, 0, false);
    }

    if (!set_zero_executed && has_new_target_pos_.exchange(false, std::memory_order_acquire)) {
        const double target_deg = target_position_deg_.load(std::memory_order_relaxed);
        bool invert_direction = false;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            invert_direction = software_invert_direction_;
        }
        const double signed_target_deg = invert_direction ? -target_deg : target_deg;
        const double axis_units_per_degree = axis_units_per_degree_.load(std::memory_order_acquire);
        const double raw_axis = signed_target_deg * axis_units_per_degree;
        const auto axis_value = std::llround(raw_axis);
        const auto clamped_axis = static_cast<std::int32_t>(
            std::clamp(axis_value, -8'388'608LL, 8'388'607LL));

        const auto speed = pending_speed_.load(std::memory_order_relaxed);
        const auto accel = pending_accel_.load(std::memory_order_relaxed);

        std::vector<uint8_t> p;
        MksProtocol::appendBe16(p, speed);
        p.push_back(accel);
        MksProtocol::appendBe24(p, clamped_axis);
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, static_cast<MksCommand>(0xF5), p, r, 0xFF, 0, false);
    }

    if (!set_zero_executed && has_new_target_vel_.exchange(false, std::memory_order_acquire)) {
        const double target_vel = target_velocity_deg_s_.load(std::memory_order_relaxed);
        
        double gear_ratio = 1.0;
        bool invert_direction = false;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            gear_ratio = software_gear_ratio_;
            invert_direction = software_invert_direction_;
        }

        const double signed_target_vel = invert_direction ? -target_vel : target_vel;
        
        const double motor_deg_per_sec = signed_target_vel * gear_ratio;
        const double rpm = std::abs(motor_deg_per_sec) / 6.0;
        const bool clockwise = motor_deg_per_sec < 0.0;
        const auto speed = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(static_cast<std::uint32_t>(std::llround(rpm)), 0U, 3000U));

        std::vector<uint8_t> p;
        // F6 frame format (manual §6.4): byte2 = dir[b7] | Rev[b6:4]=0 | speed[b11:8], byte3 = speed[b7:0]
        // dir=0 → CCW, dir=1 (bit7 set) → CW
        p.push_back((clockwise ? 0x80u : 0x00u) | static_cast<uint8_t>((speed >> 8) & 0x0Fu));
        p.push_back(static_cast<uint8_t>(speed & 0xFFu));
        p.push_back(pending_accel_.load(std::memory_order_relaxed));
        std::vector<uint8_t> r;
        (void)protocol.sendCommand(runtime_can_id, MksCommand::RunSpeedMode, p, r, 0xFF, 0, false);
    }

    // --- Read Telemetry (round-robin: one CAN query per cycle) ---
    // Each query has its own timeout budget, so all 4 telemetry values
    // are refreshed in 4 consecutive cycles instead of competing for 2ms.
    constexpr unsigned int kTelemetryTimeoutMs = 10;

    SharedTelemetry t_update;
    {
        std::lock_guard<std::mutex> lock(telem_mutex_);
        t_update = telem_;
    }

    double gear_ratio = 1.0;
    bool invert_direction = false;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        gear_ratio = software_gear_ratio_;
        invert_direction = software_invert_direction_;
    }

    bool t_ok = false;
    std::vector<std::uint8_t> payload;

    switch (telem_phase_.fetch_add(1, std::memory_order_relaxed) % 4) {
        case 0: // Motor status
            if (protocol.sendCommand(runtime_can_id, MksCommand::QueryMotorStatus, {}, payload,
                                     static_cast<uint8_t>(MksCommand::QueryMotorStatus),
                                     kTelemetryTimeoutMs, true)
                && !payload.empty()) {
                t_update.motor_status = payload.back();
                t_ok = true;
            }
            break;

        case 1: // Motor speed (RPM → deg/s)
            if (protocol.sendCommand(runtime_can_id, MksCommand::ReadMotorSpeed, {}, payload,
                                     static_cast<uint8_t>(MksCommand::ReadMotorSpeed),
                                     kTelemetryTimeoutMs, true)
                && payload.size() >= 2) {
                const std::int16_t rpm = MksProtocol::readBe16s(payload.data());
                t_update.motor_speed_rpm = rpm;
                const double velocity = (static_cast<double>(rpm) * 6.0) / gear_ratio;
                t_update.actual_velocity_deg_s = invert_direction ? -velocity : velocity;
                t_ok = true;
            }
            break;

        case 2: // Encoder position
            if (protocol.sendCommand(runtime_can_id, MksCommand::ReadEncoderAddition, {}, payload,
                                     static_cast<uint8_t>(MksCommand::ReadEncoderAddition),
                                     kTelemetryTimeoutMs, true)) {
                const double axis_units_per_degree = axis_units_per_degree_.load(std::memory_order_acquire);
                if (payload.size() >= 6) {
                    t_update.raw_axis_position = MksProtocol::readBe48s(payload.data());
                    const double position_deg = static_cast<double>(t_update.raw_axis_position) / axis_units_per_degree;
                    t_update.actual_position_deg = invert_direction ? -position_deg : position_deg;
                    t_ok = true;
                } else if (payload.size() >= 4) {
                    t_update.raw_axis_position = MksProtocol::readBe32s(payload.data());
                    const double position_deg = static_cast<double>(t_update.raw_axis_position) / axis_units_per_degree;
                    t_update.actual_position_deg = invert_direction ? -position_deg : position_deg;
                    t_ok = true;
                }
            }
            break;

        case 3: // Protection / fault state
            if (protocol.sendCommand(runtime_can_id, MksCommand::ReadProtectionState, {}, payload,
                                     static_cast<uint8_t>(MksCommand::ReadProtectionState),
                                     kTelemetryTimeoutMs, true)
                && !payload.empty()) {
                t_update.status_word = payload.back();
                t_ok = true;
            }
            break;

        default:
            break;
    }

    if (t_ok) {
        t_update.timestamp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        std::lock_guard<std::mutex> lock(telem_mutex_);
        telem_ = t_update;
    }
}

motion_core::Result<motion_core::AxisTelemetry> MksAxisAdapter::read_telemetry() const {
    const auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::AxisTelemetry>::failure(status.error());

    motion_core::AxisTelemetry out{};
    SharedTelemetry current_telem;
    {
        std::lock_guard<std::mutex> lock(telem_mutex_);
        current_telem = telem_;
    }

    out.actual_position_deg = current_telem.actual_position_deg;
    out.actual_velocity_deg_per_sec = current_telem.actual_velocity_deg_s;
    out.status_word = current_telem.status_word;
    out.protection_code = current_telem.status_word;
    out.motion_status_code = current_telem.motor_status;
    out.target_position_deg = target_position_deg_.load(std::memory_order_relaxed);
    out.mode = mode_.load(std::memory_order_relaxed);
    out.timestamp_ns = current_telem.timestamp_ns;

    bool is_enabled = enabled_.load(std::memory_order_relaxed);

    if (current_telem.status_word != 0) {
        out.state = motion_core::AxisState::Fault;
    } else {
        switch (current_telem.motor_status) {
            case 0: out.state = motion_core::AxisState::Fault; break;
            case 1: out.state = motion_core::AxisState::Ready; break;
            case 2: case 3: case 4: case 5: case 6:
                out.state = motion_core::AxisState::OperationEnabled; break;
            default: 
                out.state = is_enabled ? motion_core::AxisState::OperationEnabled : motion_core::AxisState::Ready;
                break;
        }
    }

    return motion_core::Result<motion_core::AxisTelemetry>::success(std::move(out));
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> MksAxisAdapter::list_parameters() const {
    return motion_core::Result<std::vector<motion_core::ParameterDescriptor>>::success(get_mks_parameter_descriptors());
}

motion_core::Result<motion_core::ParameterSet> MksAxisAdapter::read_parameters() const {
    const auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::ParameterSet>::failure(status.error());

    motion_core::ParameterSet set{};
    
    auto add_if_known = [&set](const motion_core::ParameterId id, ParameterValue val) {
        const auto* desc = find_mks_descriptor(id);
        if (desc) {
            set.entries.push_back({id, std::move(val)});
        }
    };

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        add_if_known(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio), ParameterValue::from_floating(software_gear_ratio_));
        add_if_known(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareEncoderResolutionBits), ParameterValue::from_unsigned(software_encoder_resolution_bits_));
        add_if_known(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareInvertDirection), ParameterValue::from_bool(software_invert_direction_));
        add_if_known(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxVelocityDegPerSec), ParameterValue::from_floating(static_cast<double>(config_.default_speed) * 6.0 / software_gear_ratio_));
        add_if_known(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2), ParameterValue::from_floating((static_cast<double>(config_.default_accel) / 255.0) * 100.0));
    }

    auto read_prm = [this](std::uint8_t cmd) {
        const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
        return config_.bus_manager->read_system_parameter(runtime_can_id, cmd);
    };

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetWorkMode)); res.ok() && res.value().size() >= 1) {
        add_if_known(motion_core::make_parameter_id(MksParameter::WorkMode), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetWorkingCurrent)); res.ok() && res.value().size() >= 2) {
        add_if_known(motion_core::make_parameter_id(MksParameter::WorkingCurrentMilliAmp), ParameterValue::from_unsigned(MksProtocol::readBe16(res.value().data() + res.value().size() - 2)));
    }
    
    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetSubdivision)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::Subdivision), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetEnPinActiveLevel)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::EnPinActiveLevel), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetMotorDirection)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::MotorDirection), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetAutoTurnOffScreen)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::AutoScreenOff), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetLockedRotorProtection)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::LockedRotorProtection), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetSubdivisionInterpolation)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::SubdivisionInterpolation), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetCanBitrate)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::CanBitrateIndex), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetCanId)); res.ok() && !res.value().empty()) {
        std::uint64_t can_id_value = res.value().back();
        if (res.value().size() >= 2) {
            const auto* raw = res.value().data() + (res.value().size() - 2);
            can_id_value = MksProtocol::readBe16(raw) & 0x07FFu;
        }
        add_if_known(motion_core::make_parameter_id(MksParameter::CanId), ParameterValue::from_unsigned(can_id_value));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetGroupId)); res.ok() && !res.value().empty()) {
        std::uint64_t group_id_value = res.value().back();
        if (res.value().size() >= 2) {
            const auto* raw = res.value().data() + (res.value().size() - 2);
            group_id_value = MksProtocol::readBe16(raw) & 0x07FFu;
        }
        add_if_known(motion_core::make_parameter_id(MksParameter::GroupId), ParameterValue::from_unsigned(group_id_value));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetKeyLock)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::KeyLock), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetHoldingCurrent)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::HoldingCurrentIndex), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetLimitPortRemap)); res.ok() && !res.value().empty()) {
        add_if_known(motion_core::make_parameter_id(MksParameter::LimitPortRemap), ParameterValue::from_unsigned(res.value().back()));
    }

    SharedTelemetry t;
    {
        std::lock_guard<std::mutex> lock(telem_mutex_);
        t = telem_;
    }

    add_if_known(motion_core::make_parameter_id(MksParameter::AxisPositionRaw), ParameterValue::from_signed(t.raw_axis_position));
    add_if_known(motion_core::make_parameter_id(MksParameter::MotorSpeedRpm), ParameterValue::from_signed(t.motor_speed_rpm));
    add_if_known(motion_core::make_parameter_id(MksParameter::ProtectionState), ParameterValue::from_unsigned(t.status_word));
    add_if_known(motion_core::make_parameter_id(MksParameter::MotorStatus), ParameterValue::from_unsigned(t.motor_status));

    add_if_known(motion_core::make_parameter_id(MksParameter::EnableMotor), ParameterValue::from_bool(enabled_.load(std::memory_order_relaxed)));

    return motion_core::Result<motion_core::ParameterSet>::success(std::move(set));
}

motion_core::Result<void> MksAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    const auto status = ensure_started();
    if (!status.ok()) return status;

    for (const auto& entry : patch.entries) {
        const auto* descriptor = find_mks_descriptor(entry.id);
        if (!descriptor) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::NotFound, "parameter not found"});
        }
        if (descriptor->read_only) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "parameter is read-only"});
        }

        if (entry.id.domain == ParameterDomain::Common) {
            const auto common_parameter = static_cast<motion_core::CommonParameter>(entry.id.value);
            if (common_parameter == motion_core::CommonParameter::HardwareGearRatio) {
                const auto ratio_result = require_floating_value(entry.value, "HardwareGearRatio requires floating point");
                if (!ratio_result.ok()) {
                    return motion_core::Result<void>::failure(ratio_result.error());
                }
                if (!(ratio_result.value() > 0.0) || !std::isfinite(ratio_result.value())) {
                    return motion_core::Result<void>::failure(
                        {motion_core::ErrorCode::InvalidArgument,
                         "HardwareGearRatio must be finite and > 0"});
                }
                std::lock_guard<std::mutex> lock(config_mutex_);
                software_gear_ratio_ = ratio_result.value();
                recalculate_axis_units_per_degree_from_software_params();
            } else if (common_parameter == motion_core::CommonParameter::HardwareEncoderResolutionBits) {
                const auto bits_result = mks::require_unsigned_value(entry.value, "HardwareEncoderResolutionBits requires unsigned integer");
                if (!bits_result.ok()) {
                    return motion_core::Result<void>::failure(bits_result.error());
                }
                if (bits_result.value() < 1 || bits_result.value() > 31) {
                    return motion_core::Result<void>::failure(
                        {motion_core::ErrorCode::InvalidArgument,
                         "HardwareEncoderResolutionBits must be in range [1..31]"});
                }
                std::lock_guard<std::mutex> lock(config_mutex_);
                software_encoder_resolution_bits_ = static_cast<std::uint32_t>(bits_result.value());
                recalculate_axis_units_per_degree_from_software_params();
            } else if (common_parameter == motion_core::CommonParameter::HardwareInvertDirection) {
                const auto invert_result = require_bool_value(entry.value, "HardwareInvertDirection requires bool");
                if (!invert_result.ok()) {
                    return motion_core::Result<void>::failure(invert_result.error());
                }
                std::lock_guard<std::mutex> lock(config_mutex_);
                software_invert_direction_ = invert_result.value();
            } else if (common_parameter == motion_core::CommonParameter::LimitsMaxVelocityDegPerSec) {
                const auto r = require_floating_value(entry.value, "LimitsMaxVelocityDegPerSec requires floating point");
                if (!r.ok()) {
                    return motion_core::Result<void>::failure(r.error());
                }
                double gear_ratio = 1.0;
                {
                    std::lock_guard<std::mutex> lock(config_mutex_);
                    gear_ratio = software_gear_ratio_;
                }
                // RPM = (deg/sec / 6) * gear_ratio
                const double rpm = (r.value() * gear_ratio) / 6.0;
                const auto new_speed = static_cast<std::uint16_t>(std::clamp(rpm, 1.0, 3000.0));
                {
                    std::lock_guard<std::mutex> lock(config_mutex_);
                    config_.default_speed = new_speed;
                }
                pending_speed_.store(new_speed, std::memory_order_relaxed);
            } else if (common_parameter == motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2) {
                const auto r = require_floating_value(entry.value, "LimitsMaxAccelerationDegPerSec2 requires floating point");
                if (!r.ok()) {
                    return motion_core::Result<void>::failure(r.error());
                }
                // UI semantic: acceleration in percent, no gear-ratio scaling.
                const double accel_percent = std::clamp(r.value(), 0.0, 100.0);
                const auto new_accel = static_cast<std::uint8_t>(std::llround((accel_percent / 100.0) * 255.0));
                {
                    std::lock_guard<std::mutex> lock(config_mutex_);
                    config_.default_accel = new_accel;
                }
                pending_accel_.store(new_accel, std::memory_order_relaxed);
            }
            continue;
        }

        if (entry.id.domain != ParameterDomain::Mks) continue;

        const auto parameter = static_cast<MksParameter>(entry.id.value);
        const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
        std::vector<uint8_t> payload;

        // Centralized packing directly to raw commands
        switch (parameter) {
            case MksParameter::EnableMotor:
                if (auto b = require_bool_value(entry.value, "bool"); b.ok()) {
                    const auto enable_result = set_enabled_locked(b.value());
                    if (!enable_result.ok()) {
                        return enable_result;
                    }
                }
                break;
            case MksParameter::WorkMode:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetWorkMode), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::WorkingCurrentMilliAmp:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    MksProtocol::appendBe16(payload, static_cast<uint16_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetWorkingCurrent), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::Subdivision:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetSubdivision), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::EnPinActiveLevel:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetEnPinActiveLevel), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::MotorDirection:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetMotorDirection), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::AutoScreenOff:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetAutoTurnOffScreen), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::LockedRotorProtection:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetLockedRotorProtection), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::SubdivisionInterpolation:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetSubdivisionInterpolation), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::CanBitrateIndex:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetCanBitrate), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::CanId:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    if (r.value() < 1U || r.value() > 0x7FFU) {
                        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "CanId must be in range 1..0x7FF"});
                    }
                    const auto new_can_id = static_cast<std::uint16_t>(r.value() & 0x07FFu);
                    MksProtocol::appendBe16(payload, new_can_id);
                    const auto old_can_id = runtime_can_id_.load(std::memory_order_relaxed);
                    if (new_can_id != old_can_id && config_.bus_manager->is_can_id_registered(new_can_id)) {
                        return motion_core::Result<void>::failure(
                            {motion_core::ErrorCode::AlreadyExists, "CanId is already used by another registered axis"});
                    }
                    const auto status_result = ensure_can_id_write_success(
                        config_.bus_manager->execute_raw_command_sync(
                            old_can_id,
                            static_cast<uint8_t>(MksCommand::SetCanId),
                            payload),
                        new_can_id);
                    if (!status_result.ok()) return status_result;

                    if (new_can_id != old_can_id) {
                        const auto remap_result = config_.bus_manager->remap_adapter_can_id(old_can_id, new_can_id, this);
                        if (!remap_result.ok()) {
                            return remap_result;
                        }
                        runtime_can_id_.store(new_can_id, std::memory_order_relaxed);
                    }
                }
                break;
            case MksParameter::SlaveRespondMode:
                return motion_core::Result<void>::failure(
                    {motion_core::ErrorCode::Unsupported,
                     "SlaveRespondMode is policy-locked and cannot be modified"});
            case MksParameter::SlaveActiveReport:
                return motion_core::Result<void>::failure(
                    {motion_core::ErrorCode::Unsupported,
                     "SlaveActiveReport is policy-locked and cannot be modified"});
            case MksParameter::GroupId:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    if (r.value() < 1U || r.value() > 0x7FFU) {
                        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "GroupId must be in range 1..0x7FF"});
                    }
                    MksProtocol::appendBe16(payload, static_cast<std::uint16_t>(r.value() & 0x07FFu));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetGroupId), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::KeyLock:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetKeyLock), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::HoldingCurrentIndex:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetHoldingCurrent), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            case MksParameter::LimitPortRemap:
                if (auto r = mks::require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto status_result = ensure_write_status_success(config_.bus_manager->execute_raw_command_sync(
                        runtime_can_id, static_cast<uint8_t>(MksCommand::SetLimitPortRemap), payload));
                    if (!status_result.ok()) return status_result;
                }
                break;
            default:
                break;
        }
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::PersistentWriteReport> MksAxisAdapter::set_persistent(
    const motion_core::PersistentCommand command,
    const motion_core::ParameterValue& value) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(status.error());
    }

    motion_core::PersistentWriteReport report{};

    if (command == motion_core::PersistentCommand::CanId) {
        report.command_supported = true;

        const auto can_id_value = mks::require_unsigned_value(value, "CanId requires unsigned integer");
        if (!can_id_value.ok()) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(can_id_value.error());
        }
        if (can_id_value.value() < 1U || can_id_value.value() > 0x7FFU) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(
                {motion_core::ErrorCode::InvalidArgument, "CanId must be in range 1..0x7FF"});
        }

        const auto old_can_id = runtime_can_id_.load(std::memory_order_relaxed);
        const auto new_can_id = static_cast<std::uint16_t>(can_id_value.value() & 0x07FFu);
        if (new_can_id != old_can_id && config_.bus_manager->is_can_id_registered(new_can_id)) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(
                {motion_core::ErrorCode::AlreadyExists, "CanId is already used by another registered axis"});
        }

        std::vector<std::uint8_t> payload;
        MksProtocol::appendBe16(payload, new_can_id);

        const auto write_status = ensure_can_id_write_success(
            config_.bus_manager->execute_raw_command_sync(
                old_can_id,
                static_cast<std::uint8_t>(MksCommand::SetCanId),
                payload),
            new_can_id);
        if (!write_status.ok()) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(write_status.error());
        }

        report.write_completed = true;
        report.persistent_save_completed = true; // Device command persists CAN ID itself.

        if (new_can_id != old_can_id) {
            const auto remap_result = config_.bus_manager->remap_adapter_can_id(old_can_id, new_can_id, this);
            if (!remap_result.ok()) {
                return motion_core::Result<motion_core::PersistentWriteReport>::failure(remap_result.error());
            }
            runtime_can_id_.store(new_can_id, std::memory_order_relaxed);
            report.reconnect_required = true;
        }

        // Readback with runtime id after possible remap.
        const auto runtime_can_id = runtime_can_id_.load(std::memory_order_relaxed);
        const auto readback = config_.bus_manager->read_system_parameter(
            runtime_can_id, static_cast<std::uint8_t>(MksCommand::SetCanId));
        if (!readback.ok()) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(readback.error());
        }
        if (readback.value().empty()) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(
                {motion_core::ErrorCode::ProtocolFailure, "CanId readback payload is empty"});
        }

        std::uint64_t readback_can_id = readback.value().back();
        if (readback.value().size() >= 2) {
            const auto* raw = readback.value().data() + (readback.value().size() - 2);
            readback_can_id = MksProtocol::readBe16(raw) & 0x07FFu;
        }
        report.readback_value = motion_core::ParameterValue::from_unsigned(readback_can_id);
        report.readback_verified = (readback_can_id == static_cast<std::uint64_t>(new_can_id));
        if (!report.readback_verified) {
            return motion_core::Result<motion_core::PersistentWriteReport>::failure(
                {motion_core::ErrorCode::ProtocolFailure, "CanId readback does not match requested value"});
        }

        return motion_core::Result<motion_core::PersistentWriteReport>::success(report);
    }

    return motion_core::Result<motion_core::PersistentWriteReport>::failure(
        {motion_core::ErrorCode::Unsupported, "Persistent command is not supported by MKS adapter"});
}

motion_core::Result<void> MksAxisAdapter::configure_motion_queue(
    const std::size_t capacity,
    const bool drop_oldest) {
    (void)capacity;
    (void)drop_oldest;
    return motion_core::Result<void>::failure(
        {motion_core::ErrorCode::Unsupported,
         "MKS adapter does not expose queue configuration in current runtime"});
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisAdapter::enqueue_motion_batch(
    const std::vector<motion_core::QueuedSetpoint>& points) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(status.error());
    }
    if (points.empty()) {
        return motion_core::Result<motion_core::MotionQueueStats>::success({});
    }

    const auto& last = points.back();
    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.target_position_deg = last.target_position_deg;
    cmd.has_profile_speed_rpm = last.has_profile_speed_rpm;
    cmd.profile_speed_rpm = last.profile_speed_rpm;
    cmd.has_profile_accel_percent = last.has_profile_accel_percent;
    cmd.profile_accel_percent = last.profile_accel_percent;
    cmd.has_target_velocity = last.has_target_velocity;
    cmd.target_velocity_deg_per_sec = last.target_velocity_deg_per_sec;

    const auto apply = apply_command(cmd);
    if (!apply.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(apply.error());
    }

    motion_core::MotionQueueStats stats{};
    stats.size = 0U;
    stats.capacity = 1U;
    stats.pushed = static_cast<std::uint64_t>(points.size());
    stats.dropped = points.size() > 1U ? static_cast<std::uint64_t>(points.size() - 1U) : 0U;
    return motion_core::Result<motion_core::MotionQueueStats>::success(stats);
}

motion_core::Result<void> MksAxisAdapter::clear_motion_queue() {
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisAdapter::query_motion_queue_stats() const {
    motion_core::MotionQueueStats stats{};
    stats.size = 0U;
    stats.capacity = 1U;
    stats.pushed = 0U;
    stats.dropped = 0U;
    return motion_core::Result<motion_core::MotionQueueStats>::success(stats);
}

motion_core::Result<void> MksAxisAdapter::ensure_started() const {
    if (!started_.load(std::memory_order_relaxed)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "axis adapter is not started"});
    }
    return motion_core::Result<void>::success();
}

void MksAxisAdapter::recalculate_axis_units_per_degree_from_software_params() {
    const auto new_axis_units_per_degree = compute_axis_units_per_degree(
        software_encoder_resolution_bits_, software_gear_ratio_);
    config_.axis_units_per_degree = new_axis_units_per_degree;
    axis_units_per_degree_.store(new_axis_units_per_degree, std::memory_order_release);
}

int MksAxisAdapter::mode_to_work_mode(const motion_core::AxisMode mode) const {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return 4; // SR_CLOSE (mode=4 per manual §4.2; SR_vFOC would be mode=5)
        default:
            return -1;
    }
}

std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config) {
    return std::make_shared<MksAxisAdapter>(std::move(config));
}

} // namespace mks
