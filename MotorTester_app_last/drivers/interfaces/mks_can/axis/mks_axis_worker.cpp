#include "mks_can/axis/mks_axis_worker.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace mks {

namespace {

[[nodiscard]] MksBusCommand make_simple_command(const std::uint16_t can_id,
                                                const std::uint8_t command,
                                                const std::vector<std::uint8_t>& payload) {
    MksBusCommand out{};
    out.can_id = can_id;
    out.command = command;
    for (auto b : payload) {
        out.push_back(b);
    }
    return out;
}

[[nodiscard]] int axis_mode_to_mks_work_mode(const motion_core::AxisMode mode) {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
            return 5; // SR_vFOC
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return 3; // SR_OPEN
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return -1;
    }
    return -1;
}

} // namespace

MksAxisWorker::MksAxisWorker(Config config)
    : config_(std::move(config)) {
    runtime_can_id_.store(config_.can_id, std::memory_order_release);
    axis_units_per_degree_runtime_.store(config_.axis_units_per_degree, std::memory_order_release);
    software_gear_ratio_runtime_.store(config_.software_gear_ratio, std::memory_order_release);
    invert_direction_runtime_.store(config_.invert_direction, std::memory_order_release);
    default_speed_rpm_runtime_.store(config_.default_speed_rpm, std::memory_order_release);
    default_accel_byte_runtime_.store(config_.default_accel_byte, std::memory_order_release);

    motion_core::AxisTelemetry initial{};
    initial.mode = motion_core::AxisMode::ProfilePosition;
    initial.state = motion_core::AxisState::Disabled;
    latest_telemetry_.store(initial, std::memory_order_release);

    pending_work_mode_.store(axis_mode_to_mks_work_mode(motion_core::AxisMode::ProfilePosition),
                             std::memory_order_release);
    active_mode_controller_ = &absolute_mode_;
}

MksAxisWorker::~MksAxisWorker() {
    (void)stop();
}

motion_core::Result<void> MksAxisWorker::start() {
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::stop() {
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_enable(const bool enabled) {
    pending_enable_state_.store(enabled ? 1 : 0, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_emergency_stop() {
    MksBusCommand ignored;
    while (tx_queue_.try_pop(ignored)) {}
    service_flags_.fetch_or(kServiceFlagEmergencyStop, std::memory_order_acq_rel);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_clear_errors() {
    service_flags_.fetch_or(kServiceFlagClearErrors, std::memory_order_acq_rel);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_home() {
    service_flags_.fetch_or(kServiceFlagHome, std::memory_order_acq_rel);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_set_zero() {
    service_flags_.fetch_or(kServiceFlagSetZero, std::memory_order_acq_rel);
    return motion_core::Result<void>::success();
}

void MksAxisWorker::set_runtime_can_id(const std::uint16_t can_id) {
    if (can_id == 0U || can_id > 0x07FFU) {
        return;
    }
    runtime_can_id_.store(can_id, std::memory_order_release);
}

motion_core::Result<void> MksAxisWorker::set_axis_units_per_degree(const double axis_units_per_degree) {
    if (!(std::isfinite(axis_units_per_degree) && std::abs(axis_units_per_degree) > 1e-9)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "axis_units_per_degree must be finite and non-zero"});
    }
    axis_units_per_degree_runtime_.store(axis_units_per_degree, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_software_gear_ratio(const double software_gear_ratio) {
    if (!(std::isfinite(software_gear_ratio) && software_gear_ratio > 0.0)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "software_gear_ratio must be finite and > 0"});
    }
    software_gear_ratio_runtime_.store(software_gear_ratio, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_invert_direction(const bool invert_direction) {
    invert_direction_runtime_.store(invert_direction, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_default_speed_rpm(const std::uint16_t default_speed_rpm) {
    if (default_speed_rpm == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "default_speed_rpm must be > 0"});
    }
    default_speed_rpm_runtime_.store(default_speed_rpm, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_default_accel_byte(const std::uint8_t default_accel_byte) {
    default_accel_byte_runtime_.store(default_accel_byte, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_mode(const motion_core::AxisMode mode) {
    const auto previous_mode = mode_.load(std::memory_order_acquire);
    if (previous_mode == mode) {
        return motion_core::Result<void>::success();
    }

    const auto work_mode = axis_mode_to_mks_work_mode(mode);
    if (work_mode >= 0) {
        pending_work_mode_.store(work_mode, std::memory_order_release);
    }

    mode_.store(mode, std::memory_order_release);

    auto current = latest_telemetry_.load(std::memory_order_acquire);
    current.mode = mode;
    latest_telemetry_.store(current, std::memory_order_release);

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_motion_queue_policy(const std::size_t capacity, const bool drop_oldest) {
    const auto abs_result = absolute_mode_.set_motion_queue_policy(capacity, drop_oldest);
    if (!abs_result.ok()) {
        return abs_result;
    }

    const auto vel_result = velocity_mode_.set_motion_queue_policy(capacity, drop_oldest);
    if (!vel_result.ok()) {
        return vel_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisWorker::enqueue_motion_batch(
    const std::vector<motion_core::QueuedSetpoint>& points) {
    auto* mode_controller = active_motion_mode();
    if (!mode_controller) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(
            {motion_core::ErrorCode::Unsupported, "motion mode does not accept queued setpoints"});
    }
    return mode_controller->enqueue_motion_batch(points);
}

motion_core::Result<void> MksAxisWorker::clear_motion_queue() {
    const auto abs_result = absolute_mode_.clear_motion_queue();
    if (!abs_result.ok()) {
        return abs_result;
    }

    const auto vel_result = velocity_mode_.clear_motion_queue();
    if (!vel_result.ok()) {
        return vel_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisWorker::query_motion_queue_stats() const {
    const auto mode = mode_.load(std::memory_order_acquire);
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return absolute_mode_.query_motion_queue_stats();
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return velocity_mode_.query_motion_queue_stats();
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return motion_core::Result<motion_core::MotionQueueStats>::failure(
                {motion_core::ErrorCode::Unsupported, "motion mode does not support queued setpoints"});
    }
    return motion_core::Result<motion_core::MotionQueueStats>::failure(
        {motion_core::ErrorCode::Unsupported, "unknown motion mode"});
}

motion_core::Result<motion_core::AxisTelemetry> MksAxisWorker::read_telemetry() const {
    return motion_core::Result<motion_core::AxisTelemetry>::success(
        latest_telemetry_.load(std::memory_order_acquire));
}



bool MksAxisWorker::consume_tx_command(MksBusCommand& command) {
    return tx_queue_.try_pop(command);
}

void MksAxisWorker::publish_telemetry(const motion_core::AxisTelemetry& telemetry) {
    auto updated = telemetry;
    updated.mode = mode_.load(std::memory_order_acquire);
    latest_telemetry_.store(updated, std::memory_order_release);
}

void MksAxisWorker::step(const std::chrono::steady_clock::time_point now) {
    if (handle_service_request()) {
        return;
    }
    handle_motion_request(now);
}

bool MksAxisWorker::handle_service_request() {
    auto flags = service_flags_.exchange(0U, std::memory_order_acq_rel);
    bool pushed_any = false;

    auto try_send_flag = [&](std::uint32_t flag, std::uint8_t cmd, const std::vector<std::uint8_t>& payload) {
        if ((flags & flag) != 0U) {
            auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire), cmd, payload);
            if (tx_queue_.try_push(bus_cmd)) {
                flags &= ~flag;
                pushed_any = true;
            }
        }
    };

    try_send_flag(kServiceFlagEmergencyStop, static_cast<std::uint8_t>(MksCommand::EmergencyStop), {});
    try_send_flag(kServiceFlagClearErrors, static_cast<std::uint8_t>(MksCommand::ReleaseStallProtection), {});
    try_send_flag(kServiceFlagHome, static_cast<std::uint8_t>(MksCommand::GoHome), {});
    try_send_flag(kServiceFlagSetZero, static_cast<std::uint8_t>(MksCommand::SetCurrentAxisToZero), {});

    if (flags != 0) {
        service_flags_.fetch_or(flags, std::memory_order_acq_rel);
    }

    const auto work_mode = pending_work_mode_.exchange(-1, std::memory_order_acq_rel);
    if (work_mode >= 0) {
        auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                           static_cast<std::uint8_t>(MksCommand::SetWorkMode),
                                           {static_cast<std::uint8_t>(work_mode)});
        if (tx_queue_.try_push(bus_cmd)) {
            pushed_any = true;
        } else {
            pending_work_mode_.store(work_mode, std::memory_order_release);
        }
    }

    const auto enable_state = pending_enable_state_.exchange(-1, std::memory_order_acq_rel);
    if (enable_state >= 0) {
        auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                           static_cast<std::uint8_t>(MksCommand::EnableMotor),
                                           {static_cast<std::uint8_t>(enable_state > 0 ? 1U : 0U)});
        if (tx_queue_.try_push(bus_cmd)) {
            pushed_any = true;
        } else {
            pending_enable_state_.store(enable_state, std::memory_order_release);
        }
    }

    return pushed_any;
}

void MksAxisWorker::handle_motion_request(const std::chrono::steady_clock::time_point now) {
    auto* mode_controller = active_motion_mode();
    if (!mode_controller) {
        if (active_mode_controller_) {
            active_mode_controller_->on_mode_exit();
            active_mode_controller_ = nullptr;
        }
        return;
    }

    if (active_mode_controller_ != mode_controller) {
        if (active_mode_controller_) {
            active_mode_controller_->on_mode_exit();
        }
        mode_controller->on_mode_enter(now);
        active_mode_controller_ = mode_controller;
    }

    const auto context = build_motion_context();
    const auto step_result = mode_controller->step(now, context);
    if (!step_result.ok()) {
        return;
    }

    if (!step_result.value().has_command) {
        return;
    }

    push_tx_command(step_result.value().command);
}

void MksAxisWorker::push_tx_command(const MksBusCommand& command) {
    (void)tx_queue_.try_push(command);
}



MksMotionModeBase* MksAxisWorker::active_motion_mode() {
    const auto mode = mode_.load(std::memory_order_acquire);
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return &absolute_mode_;
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return &velocity_mode_;
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return nullptr;
    }
    return nullptr;
}

MksMotionBuildContext MksAxisWorker::build_motion_context() const {
    MksMotionBuildContext context{};
    context.can_id = runtime_can_id_.load(std::memory_order_acquire);
    context.axis_units_per_degree = axis_units_per_degree_runtime_.load(std::memory_order_acquire);
    context.software_gear_ratio = software_gear_ratio_runtime_.load(std::memory_order_acquire);
    context.bus_cycle_period_sec =
        static_cast<double>(config_.cycle_period.count()) / 1'000'000.0;
    context.invert_direction = invert_direction_runtime_.load(std::memory_order_acquire);
    context.fallback_speed_rpm = default_speed_rpm_runtime_.load(std::memory_order_acquire);
    context.fallback_accel_byte = default_accel_byte_runtime_.load(std::memory_order_acquire);
    return context;
}

} // namespace mks
