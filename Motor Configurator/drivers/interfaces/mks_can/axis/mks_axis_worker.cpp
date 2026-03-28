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
    out.payload = payload;
    return out;
}

[[nodiscard]] int axis_mode_to_mks_work_mode(const motion_core::AxisMode mode) {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
        case motion_core::AxisMode::Homing:
            return 5; // SR_vFOC
        case motion_core::AxisMode::CyclicSyncPosition:
            return -1; // Disabled in current MKS runtime (reserved for future rework)
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
}

MksAxisWorker::~MksAxisWorker() {
    (void)stop();
}

motion_core::Result<void> MksAxisWorker::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return motion_core::Result<void>::success();
    }
    worker_thread_ = std::thread(&MksAxisWorker::run_loop, this);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        return motion_core::Result<void>::success();
    }
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_enable(const bool enabled) {
    pending_enable_state_.store(enabled ? 1 : 0, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::request_emergency_stop() {
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
    if (mode == motion_core::AxisMode::CyclicSyncPosition) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::Unsupported, "MKS CyclicSyncPosition is disabled by design"});
    }

    const auto work_mode = axis_mode_to_mks_work_mode(mode);
    pending_work_mode_.store(work_mode, std::memory_order_release);

    mode_.store(mode, std::memory_order_release);
    auto current = latest_telemetry_.load(std::memory_order_acquire);
    current.mode = mode;
    latest_telemetry_.store(current, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_motion_queue_policy(const std::size_t capacity, const bool drop_oldest) {
    if (capacity == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "motion queue capacity must be > 0"});
    }
    motion_queue_capacity_limit_.store(std::min(capacity, kMotionQueuePhysicalCapacity), std::memory_order_release);
    motion_queue_drop_oldest_policy_.store(drop_oldest, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisWorker::enqueue_motion_batch(
    const std::vector<motion_core::QueuedSetpoint>& points) {
    if (points.empty()) {
        return query_motion_queue_stats();
    }

    std::uint64_t pushed = 0U;
    std::uint64_t dropped = 0U;
    const auto capacity_limit = motion_queue_capacity_limit_.load(std::memory_order_acquire);
    const bool drop_oldest = motion_queue_drop_oldest_policy_.load(std::memory_order_acquire);

    for (const auto& point : points) {
        const auto current_size = motion_queue_.size_approx();
        if (current_size >= capacity_limit) {
            if (!drop_oldest) {
                ++dropped;
                continue;
            }
            motion_core::QueuedSetpoint ignored{};
            if (motion_queue_.try_pop(ignored)) {
                ++dropped;
            } else {
                ++dropped;
                continue;
            }
        }
        if (motion_queue_.try_push(point)) {
            ++pushed;
        } else {
            ++dropped;
        }
    }

    motion_points_pushed_.fetch_add(pushed, std::memory_order_relaxed);
    motion_points_dropped_.fetch_add(dropped, std::memory_order_relaxed);
    return query_motion_queue_stats();
}

motion_core::Result<void> MksAxisWorker::clear_motion_queue() {
    motion_core::QueuedSetpoint point{};
    while (motion_queue_.try_pop(point)) {
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisWorker::query_motion_queue_stats() const {
    motion_core::MotionQueueStats stats{};
    stats.size = motion_queue_.size_approx();
    stats.capacity = motion_queue_capacity_limit_.load(std::memory_order_acquire);
    stats.pushed = motion_points_pushed_.load(std::memory_order_acquire);
    stats.dropped = motion_points_dropped_.load(std::memory_order_acquire);
    stats.underruns = motion_queue_underruns_.load(std::memory_order_acquire);
    stats.short_starts = motion_queue_short_starts_.load(std::memory_order_acquire);
    return motion_core::Result<motion_core::MotionQueueStats>::success(stats);
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
    push_telemetry_sample(updated);
}

void MksAxisWorker::run_loop() {
    auto next_tp = std::chrono::steady_clock::now();
    while (running_.load(std::memory_order_acquire)) {
        if (!handle_service_request()) {
            handle_motion_request();
        }

        next_tp += config_.cycle_period;
        std::this_thread::sleep_until(next_tp);
    }
}

bool MksAxisWorker::handle_service_request() {
    const auto flags = service_flags_.exchange(0U, std::memory_order_acq_rel);
    if ((flags & kServiceFlagEmergencyStop) != 0U) {
        const auto remaining = flags & ~kServiceFlagEmergencyStop;
        if (remaining != 0U) {
            service_flags_.fetch_or(remaining, std::memory_order_acq_rel);
        }
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::EmergencyStop),
                                            {}));
        return true;
    }
    if ((flags & kServiceFlagClearErrors) != 0U) {
        const auto remaining = flags & ~kServiceFlagClearErrors;
        if (remaining != 0U) {
            service_flags_.fetch_or(remaining, std::memory_order_acq_rel);
        }
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::ReleaseStallProtection),
                                            {}));
        return true;
    }
    if ((flags & kServiceFlagHome) != 0U) {
        const auto remaining = flags & ~kServiceFlagHome;
        if (remaining != 0U) {
            service_flags_.fetch_or(remaining, std::memory_order_acq_rel);
        }
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::GoHome),
                                            {}));
        return true;
    }
    if ((flags & kServiceFlagSetZero) != 0U) {
        const auto remaining = flags & ~kServiceFlagSetZero;
        if (remaining != 0U) {
            service_flags_.fetch_or(remaining, std::memory_order_acq_rel);
        }
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::SetCurrentAxisToZero),
                                            {}));
        return true;
    }

    const auto work_mode = pending_work_mode_.exchange(-1, std::memory_order_acq_rel);
    if (work_mode >= 0) {
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::SetWorkMode),
                                            {static_cast<std::uint8_t>(work_mode)}));
        return true;
    }

    const auto enable_state = pending_enable_state_.exchange(-1, std::memory_order_acq_rel);
    if (enable_state >= 0) {
        push_tx_command(make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                            static_cast<std::uint8_t>(MksCommand::EnableMotor),
                                            {static_cast<std::uint8_t>(enable_state > 0 ? 1U : 0U)}));
        return true;
    }
    return false;
}

void MksAxisWorker::handle_motion_request() {
    const auto mode = mode_.load(std::memory_order_acquire);

    motion_core::QueuedSetpoint point{};
    if (!motion_queue_.try_pop(point)) {
        return;
    }

    const auto context = build_motion_context();

    motion_core::Result<MksBusCommand> command_result =
        motion_core::Result<MksBusCommand>::failure({motion_core::ErrorCode::Unsupported, "unsupported mode"});
    switch (mode) {
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            command_result = velocity_mode_.build_command(context, point);
            break;
        case motion_core::AxisMode::CyclicSyncPosition:
            command_result = motion_core::Result<MksBusCommand>::failure(
                {motion_core::ErrorCode::Unsupported, "MKS CyclicSyncPosition is disabled by design"});
            break;
        case motion_core::AxisMode::ProfilePosition:
            command_result = absolute_mode_.build_command(context, point);
            break;
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            command_result = motion_core::Result<MksBusCommand>::failure(
                {motion_core::ErrorCode::Unsupported, "motion mode is service-only or unsupported"});
            break;
    }

    if (!command_result.ok()) {
        return;
    }
    push_tx_command(command_result.value());
}

void MksAxisWorker::push_tx_command(const MksBusCommand& command) {
    MksBusCommand dropped{};
    while (!tx_queue_.try_push(command)) {
        if (!tx_queue_.try_pop(dropped)) {
            break;
        }
    }
}

void MksAxisWorker::push_telemetry_sample(const motion_core::AxisTelemetry& telemetry) {
    motion_core::AxisTelemetry dropped{};
    while (!telemetry_queue_.try_push(telemetry)) {
        if (!telemetry_queue_.try_pop(dropped)) {
            break;
        }
        telemetry_points_dropped_.fetch_add(1U, std::memory_order_relaxed);
    }
    telemetry_points_pushed_.fetch_add(1U, std::memory_order_relaxed);
}

MksMotionBuildContext MksAxisWorker::build_motion_context() const {
    MksMotionBuildContext context{};
    context.can_id = runtime_can_id_.load(std::memory_order_acquire);
    context.axis_units_per_degree = axis_units_per_degree_runtime_.load(std::memory_order_acquire);
    context.software_gear_ratio = software_gear_ratio_runtime_.load(std::memory_order_acquire);
    context.invert_direction = invert_direction_runtime_.load(std::memory_order_acquire);
    context.fallback_speed_rpm = default_speed_rpm_runtime_.load(std::memory_order_acquire);
    context.fallback_accel_byte = default_accel_byte_runtime_.load(std::memory_order_acquire);
    return context;
}

} // namespace mks
