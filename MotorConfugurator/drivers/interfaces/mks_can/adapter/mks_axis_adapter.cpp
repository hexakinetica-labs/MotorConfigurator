#include "mks_can/adapter/mks_axis_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mks {

MksAxisAdapter::MksAxisAdapter(MksAxisAdapterConfig config)
    : config_(std::move(config)) {
    runtime_can_id_.store(config_.can_id, std::memory_order_release);
    axis_units_per_degree_runtime_.store(config_.axis_units_per_degree, std::memory_order_release);

    MksAxisWorker::Config worker_config{};
    worker_config.axis_id = config_.axis_id;
    worker_config.can_id = config_.can_id;
    worker_config.axis_units_per_degree = config_.axis_units_per_degree;
    worker_config.software_gear_ratio = config_.software_gear_ratio;
    worker_config.invert_direction = config_.invert_direction;
    worker_config.telemetry_invert_position_sign = config_.telemetry_invert_position_sign;
    worker_config.default_speed_rpm = config_.default_speed_rpm;
    worker_config.default_accel_byte = config_.default_accel_byte;
    if (config_.bus_manager) {
        worker_config.cycle_period = config_.bus_manager->cycle_time();
    }
    worker_ = std::make_unique<MksAxisWorker>(worker_config);

    MksAxisConfigurator::Config configurator_config{};
    configurator_config.axis_id = config_.axis_id;
    configurator_config.can_id = config_.can_id;
    configurator_config.bus_manager = config_.bus_manager;
    configurator_config.state_cache = &state_cache_;
    configurator_config.axis_units_per_degree = config_.axis_units_per_degree;
    configurator_config.software_gear_ratio = config_.software_gear_ratio;
    configurator_config.invert_direction = config_.invert_direction;
    configurator_config.telemetry_invert_position_sign = config_.telemetry_invert_position_sign;
    configurator_config.default_speed_rpm = config_.default_speed_rpm;
    configurator_config.default_accel_byte = config_.default_accel_byte;
    configurator_config.on_runtime_can_id_changed = [this](const std::uint16_t new_can_id) {
        const auto old_can_id = runtime_can_id_.load(std::memory_order_acquire);
        if (new_can_id == old_can_id) {
            return motion_core::Result<void>::success();
        }
        if (!config_.bus_manager) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::InvalidArgument, "bus_manager is null"});
        }
        const auto remap_result = config_.bus_manager->remap_adapter_can_id(old_can_id, new_can_id, this);
        if (!remap_result.ok()) {
            return remap_result;
        }
        runtime_can_id_.store(new_can_id, std::memory_order_release);
        if (worker_) {
            worker_->set_runtime_can_id(new_can_id);
        }
        return motion_core::Result<void>::success();
    };
    configurator_ = std::make_unique<MksAxisConfigurator>(std::move(configurator_config));

    // Register with bus manager so poll_cycle() dispatches our commands and
    // collects telemetry.  Pre-start registration is allowed — start() will
    // pick up already-registered adapters.
    if (config_.bus_manager) {
        (void)config_.bus_manager->register_adapter(config_.can_id, this);
    }

    // Arm the configurator so parameter reads/writes succeed.
    (void)configurator_->start();
}

MksAxisAdapter::~MksAxisAdapter() {
    if (configurator_) {
        (void)configurator_->stop();
    }
    if (config_.bus_manager) {
        (void)config_.bus_manager->unregister_adapter(
            runtime_can_id_.load(std::memory_order_acquire), this);
    }
}

motion_core::AxisInfo MksAxisAdapter::info() const {
    motion_core::AxisInfo out{};
    out.id = config_.axis_id;
    out.name = config_.axis_name;
    out.transport = motion_core::AxisTransportKind::CanBus;
    return out;
}

bool MksAxisAdapter::enqueueServicePoint(const motion_core::ServiceCommandPoint& p) {    
    return enqueue_service_point_impl(p, false);
}

bool MksAxisAdapter::enqueueCommandPoint(const motion_core::MotionCommandPoint& point) {
    return enqueue_command_point_impl(point, false);
}

bool MksAxisAdapter::enqueue_service_point_impl(const motion_core::ServiceCommandPoint& p,
                                                const bool internal_request) {
    if (!internal_request) {
        if (p.type == motion_core::ServiceCommandType::Home) {
            if (homing_active_.load(std::memory_order_acquire)
                || homing_start_requested_.load(std::memory_order_acquire)) {
                return false;
            }
            has_requested_target_position_.store(false, std::memory_order_release);
            requested_target_position_deg_.store(0.0, std::memory_order_release);
            homing_start_requested_.store(true, std::memory_order_release);
            // BUG-5 fix: do NOT set homing_status_ here — let the SM step produce
            // the authoritative status after it processes the start request.
            return true;
        }

        if ((homing_active_.load(std::memory_order_acquire)
             || homing_start_requested_.load(std::memory_order_acquire))
            && p.type != motion_core::ServiceCommandType::Disable
            && p.type != motion_core::ServiceCommandType::ClearErrors
            && p.type != motion_core::ServiceCommandType::ResetDrive) {
            return false;
        }

        if (p.type == motion_core::ServiceCommandType::Disable) {
            homing_cancel_requested_.store(true, std::memory_order_release);
            set_homing_status(HomingStatus::Idle);
            // BUG-3 fix (adapter level): clear motion queue when homing is cancelled
            // via Disable to prevent stale move-to-offset commands.
            if (worker_) {
                worker_->clear_motion_queue();
            }
        }
    }

    if (p.type == motion_core::ServiceCommandType::Disable
        || p.type == motion_core::ServiceCommandType::Home
        || p.type == motion_core::ServiceCommandType::SetZero
        || p.type == motion_core::ServiceCommandType::ClearMotionQueue
        || p.type == motion_core::ServiceCommandType::SetOperatingMode) {
        has_requested_target_position_.store(false, std::memory_order_release);
    }
    if (p.type == motion_core::ServiceCommandType::SetZero) {
        requested_target_position_deg_.store(0.0, std::memory_order_release);
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    return worker_ && worker_->enqueue_service_point(p);
}

bool MksAxisAdapter::enqueue_command_point_impl(const motion_core::MotionCommandPoint& point,
                                                const bool internal_request) {
    if (!internal_request
        && (homing_active_.load(std::memory_order_acquire)
            || homing_start_requested_.load(std::memory_order_acquire))) {
        return false;
    }

    motion_core::MotionCommandPoint normalized_point = point;
    const bool updates_position_target = (normalized_point.type != motion_core::MotionCommandType::Velocity);

    if (updates_position_target && normalized_point.is_relative) {
        double base_position_deg = 0.0;
        if (has_requested_target_position_.load(std::memory_order_acquire)) {
            base_position_deg = requested_target_position_deg_.load(std::memory_order_acquire);
        } else {
            base_position_deg = state_cache_.latest_telemetry().position;
        }

        normalized_point.value = base_position_deg + normalized_point.value;
        normalized_point.is_relative = false;
    }

    if (updates_position_target && configurator_) {
        double gear_ratio = configurator_->software_gear_ratio();
        if (!(std::isfinite(gear_ratio) && gear_ratio > 0.0)) {
            gear_ratio = config_.software_gear_ratio;
        }

        double motor_target_deg = normalized_point.value * gear_ratio;
        if (invert_direction()) {
            motor_target_deg = -motor_target_deg;
        }

        const double min_motor_deg = configurator_->limits_software_min_motor_deg();
        const double max_motor_deg = configurator_->limits_software_max_motor_deg();
        if (std::isfinite(min_motor_deg) && motor_target_deg < min_motor_deg) {
            return false;
        }
        if (std::isfinite(max_motor_deg) && motor_target_deg > max_motor_deg) {
            return false;
        }
    }

    bool ok = false;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        ok = worker_ && worker_->enqueue_command_batch({normalized_point});
    }
    if (!ok) {
        return false;
    }

    if (updates_position_target) {
        has_requested_target_position_.store(true, std::memory_order_release);
        requested_target_position_deg_.store(normalized_point.value, std::memory_order_release);
    }
    state_cache_.set_motion_active(true);
    return true;
}

void MksAxisAdapter::estop() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    has_requested_target_position_.store(false, std::memory_order_release);
    homing_cancel_requested_.store(true, std::memory_order_release);
    homing_active_.store(false, std::memory_order_release);
    set_homing_status(HomingStatus::Idle);
    worker_->request_emergency_stop();
}

void MksAxisAdapter::setControlOwner(motion_core::ControlOwner owner) {
    // Adapter technically doesn't own this state in a complex way for now, UI layer controls mapping.
    // The Boss specifically allowed it to exist on IAxis, but it's typically used inside RuntimeQueueIngress.
    (void)owner;
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> MksAxisAdapter::list_parameters() const {
    return configurator_->list_parameters();
}

motion_core::Result<motion_core::ParameterSet> MksAxisAdapter::read_parameters() const {
    return configurator_->read_parameters();
}

motion_core::Result<void> MksAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    const auto apply_result = configurator_->apply_parameter_patch(patch);
    const auto sync_result = sync_motion_runtime_from_configurator();

    if (!apply_result.ok()) {
        return apply_result;
    }

    return sync_result;
}

motion_core::Result<motion_core::PersistentWriteReport> MksAxisAdapter::set_persistent(
    motion_core::PersistentCommand command,
    const motion_core::ParameterValue& value) {
    const auto result = configurator_->set_persistent(command, value);
    if (!result.ok()) {
        return result;
    }
    const auto sync_result = sync_motion_runtime_from_configurator();
    if (!sync_result.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(sync_result.error());
    }
    return result;
}

// Queues are now hidden from public IAxis

bool MksAxisAdapter::consume_tx_command(MksBusCommand& command) {
    return worker_ && worker_->consume_tx_command(command);
}

void MksAxisAdapter::step_worker(const std::chrono::steady_clock::time_point now) {
    process_homing_state_machine(now);
    if (worker_) {
        worker_->step(now);
    }
}

void MksAxisAdapter::process_homing_state_machine(const std::chrono::steady_clock::time_point now) {
    const auto phase_to_status = [](const MksHomingStateMachine::Phase phase) {
        switch (phase) {
            case MksHomingStateMachine::Phase::Idle:
                return HomingStatus::Idle;
            case MksHomingStateMachine::Phase::StartHardwareHome:
                return HomingStatus::StartingHardwareHome;
            case MksHomingStateMachine::Phase::WaitHomingStatusActive:
                return HomingStatus::WaitingHardwareHomeActive;
            case MksHomingStateMachine::Phase::WaitHomingStatusInactiveAtZero:
                return HomingStatus::WaitingHardwareHomeDone;
            case MksHomingStateMachine::Phase::CommandMoveToOffset:
            case MksHomingStateMachine::Phase::WaitMoveToOffset:
                return HomingStatus::MovingToOffset;
        }
        return HomingStatus::Idle;
    };

    if (homing_cancel_requested_.exchange(false, std::memory_order_acq_rel)) {
        homing_state_machine_.request_cancel();
        set_homing_status(HomingStatus::Idle);
    }
    if (homing_start_requested_.exchange(false, std::memory_order_acq_rel)) {
        homing_state_machine_.request_start();
        set_homing_status(HomingStatus::StartingHardwareHome);
    }

    const auto telemetry = state_cache_.latest_telemetry();
    MksHomingInputs inputs{};
    inputs.faulted = telemetry.fault
        || telemetry.state == motion_core::AxisState::Fault
        || telemetry.protection_code != 0U;
    inputs.home_status_active = telemetry.motion_status_code == 5U;
    inputs.position_deg = telemetry.position;
    inputs.velocity_deg_per_sec = telemetry.velocity;
    inputs.homing_offset_deg = configurator_ ? configurator_->homing_offset_deg() : 0.0;

    const auto output = homing_state_machine_.step(inputs, now);
    homing_active_.store(output.active || homing_state_machine_.active(), std::memory_order_release);

    if (output.failed) {
        set_homing_status(HomingStatus::Failed);
    } else if (output.request_set_zero) {
        set_homing_status(HomingStatus::ApplyingSetZero);
    } else if (output.request_move_to_offset) {
        set_homing_status(HomingStatus::MovingToOffset);
    } else {
        const auto sm_status = phase_to_status(homing_state_machine_.current_phase());
        // Completed and Failed are terminal statuses that must stay until the
        // orchestrator starts a new Home (which sets StartingHardwareHome) or
        // an explicit Disable (which sets Idle in enqueue_service_point_impl).
        // Without this, the SM step overwrites Completed→Idle within 2.5ms,
        // and the 100ms orchestrator tick never sees it → sequence gets stuck.
        const auto current = static_cast<HomingStatus>(homing_status_.load(std::memory_order_acquire));
        if (sm_status == HomingStatus::Idle
            && (current == HomingStatus::Completed || current == HomingStatus::Failed)) {
            // Keep the terminal status — don't overwrite.
        } else {
            set_homing_status(sm_status);
        }
    }

    if (output.request_start_hardware_home) {
        motion_core::ServiceCommandPoint clear_queue{};
        clear_queue.type = motion_core::ServiceCommandType::ClearMotionQueue;
        clear_queue.axis_id = config_.axis_id.value;
        motion_core::ServiceCommandPoint home{};
        home.type = motion_core::ServiceCommandType::Home;
        home.axis_id = config_.axis_id.value;
        if (!enqueue_service_point_impl(clear_queue, true) || !enqueue_service_point_impl(home, true)) {
            std::fprintf(stderr,
                         "[MKS][HOMING] axis=%u failed to queue hardware home sequence\n",
                         static_cast<unsigned int>(config_.axis_id.value));
        }
    }

    if (output.request_move_to_offset) {
        motion_core::MotionCommandPoint move{};
        move.type = motion_core::MotionCommandType::Position;
        move.axis_id = config_.axis_id.value;
        move.value = output.move_to_offset_deg;
        move.velocity = 0.0;
        move.acceleration = 0.0;
        move.timestamp_us = telemetry.timestamp_us;
        move.sample_period_sec = 0.004;
        move.is_relative = false;
        if (!enqueue_command_point_impl(move, true)) {
            std::fprintf(stderr,
                         "[MKS][HOMING] axis=%u failed to queue move-to-offset command\n",
                         static_cast<unsigned int>(config_.axis_id.value));
            homing_state_machine_.request_cancel();
            homing_active_.store(false, std::memory_order_release);
        }
    }

    if (output.request_set_zero) {
        motion_core::ServiceCommandPoint set_zero{};
        set_zero.type = motion_core::ServiceCommandType::SetZero;
        set_zero.axis_id = config_.axis_id.value;
        if (!enqueue_service_point_impl(set_zero, true)) {
            std::fprintf(stderr,
                         "[MKS][HOMING] axis=%u failed to queue final set-zero command\n",
                         static_cast<unsigned int>(config_.axis_id.value));
        }
        // BUG-4 fix: set Completed status. It will be reset to Idle on the next
        // SM step when the SM is in Phase::Idle and no special outputs are requested.
        set_homing_status(HomingStatus::Completed);
    }

    if (output.failed) {
        homing_active_.store(false, std::memory_order_release);
        std::fprintf(stderr,
                     "[MKS][HOMING] axis=%u homing sequence failed or timed out\n",
                     static_cast<unsigned int>(config_.axis_id.value));
    }
}

void MksAxisAdapter::set_homing_status(const HomingStatus status) noexcept {
    homing_status_.store(static_cast<std::uint8_t>(status), std::memory_order_release);
}

const char* MksAxisAdapter::homing_status_to_cstr(const HomingStatus status) noexcept {
    switch (status) {
        case HomingStatus::Idle:
            return "Idle";
        case HomingStatus::StartingHardwareHome:
            return "HardwareHomeStart";
        case HomingStatus::WaitingHardwareHomeActive:
            return "WaitingHardwareHomeActive";
        case HomingStatus::WaitingHardwareHomeDone:
            return "WaitingHardwareHomeDone";
        case HomingStatus::MovingToOffset:
            return "MovingToOffset";
        case HomingStatus::ApplyingSetZero:
            return "ApplyingSetZero";
        case HomingStatus::Completed:
            return "Completed";
        case HomingStatus::Failed:
            return "Failed";
    }
    return "Unknown";
}

const char* MksAxisAdapter::homing_status_text() const {
    const auto status = static_cast<HomingStatus>(homing_status_.load(std::memory_order_acquire));
    return homing_status_to_cstr(status);
}

motion_core::TelemetrySnapshot MksAxisAdapter::telemetry() const {
    auto telemetry = state_cache_.latest_telemetry();
    telemetry.fault = telemetry.protection_code != 0U || telemetry.state == motion_core::AxisState::Fault;
    telemetry.enabled = telemetry.state == motion_core::AxisState::OperationEnabled;
    return telemetry;
}

void MksAxisAdapter::publish_polled_telemetry(const motion_core::TelemetrySnapshot& telemetry) {
    AxisPositionSample sample{};
    sample.timestamp_ns = telemetry.timestamp_us * 1000ULL;
    sample.position_deg = telemetry.position;
    (void)position_samples_.try_push(sample);

    state_cache_.set_latest_telemetry(telemetry);
    const bool is_moving = (telemetry.motion_status_code >= 2U && telemetry.motion_status_code <= 6U);
    state_cache_.set_motion_active(is_moving);
    if (worker_) {
        worker_->publish_telemetry(telemetry);
    }
}

void MksAxisAdapter::mark_command_tx() {
    cycle_metrics_.mark_command_tx();
}

void MksAxisAdapter::mark_telemetry_publish() {
    cycle_metrics_.mark_telemetry_publish();
}

void MksAxisAdapter::mark_position_rx() {
    cycle_metrics_.mark_position_rx();
}

void MksAxisAdapter::mark_speed_rx() {
    cycle_metrics_.mark_speed_rx();
}

void MksAxisAdapter::mark_status_rx() {
    cycle_metrics_.mark_status_rx();
}

void MksAxisAdapter::mark_protection_rx() {
    cycle_metrics_.mark_protection_rx();
}

AxisCycleMetricsSnapshot MksAxisAdapter::cycle_metrics_snapshot() const {
    return cycle_metrics_.snapshot();
}

motion_core::MotionQueueStats MksAxisAdapter::query_motion_queue_stats() const {
    const auto stats = worker_->query_motion_queue_stats();
    return stats.ok() ? stats.value() : motion_core::MotionQueueStats{};
}

std::size_t MksAxisAdapter::drain_position_samples(std::vector<AxisPositionSample>& out_samples,
                                                   const std::size_t max_samples) const {
    if (max_samples == 0U) {
        return 0U;
    }

    std::size_t drained = 0U;
    AxisPositionSample sample{};
    while (drained < max_samples && position_samples_.try_pop(sample)) {
        out_samples.push_back(sample);
        drained += 1U;
    }
    return drained;
}

std::uint16_t MksAxisAdapter::runtime_can_id() const {
    return runtime_can_id_.load(std::memory_order_acquire);
}

double MksAxisAdapter::axis_units_per_degree() const {
    return axis_units_per_degree_runtime_.load(std::memory_order_acquire);
}

double MksAxisAdapter::requested_target_position_deg() const {
    return requested_target_position_deg_.load(std::memory_order_acquire);
}

bool MksAxisAdapter::has_requested_target_position() const {
    return has_requested_target_position_.load(std::memory_order_acquire);
}

double MksAxisAdapter::software_gear_ratio() const {
    if (!configurator_) {
        return config_.software_gear_ratio;
    }
    return configurator_->software_gear_ratio();
}

bool MksAxisAdapter::invert_direction() const {
    if (!configurator_) {
        return config_.invert_direction;
    }
    return configurator_->software_invert_direction();
}

bool MksAxisAdapter::telemetry_invert_position_sign() const {
    if (!configurator_) {
        return config_.telemetry_invert_position_sign;
    }
    return configurator_->telemetry_invert_position_sign();
}

motion_core::AxisMode MksAxisAdapter::current_mode() const {
    return state_cache_.mode();
}

motion_core::Result<void> MksAxisAdapter::sync_motion_runtime_from_configurator() {
    if (!worker_ || !configurator_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InternalError, "worker/configurator is not initialized"});
    }

    const auto gear_ratio_result = worker_->set_software_gear_ratio(configurator_->software_gear_ratio());
    if (!gear_ratio_result.ok()) {
        return gear_ratio_result;
    }
    const auto axis_units_result = worker_->set_axis_units_per_degree(configurator_->axis_units_per_degree());
    if (!axis_units_result.ok()) {
        return axis_units_result;
    }
    axis_units_per_degree_runtime_.store(configurator_->axis_units_per_degree(), std::memory_order_release);
    const auto invert_result = worker_->set_invert_direction(configurator_->software_invert_direction());
    if (!invert_result.ok()) {
        return invert_result;
    }
    worker_->set_telemetry_invert_position_sign(configurator_->telemetry_invert_position_sign());

    const auto velocity_limit = configurator_->limits_max_velocity_deg_per_sec();
    const auto gear_ratio = configurator_->software_gear_ratio();
    if (!(std::isfinite(velocity_limit) && velocity_limit > 0.0 && std::isfinite(gear_ratio) && gear_ratio > 0.0)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "invalid velocity limit or gear ratio for runtime sync"});
    }
    const auto motor_rpm_double = (velocity_limit * gear_ratio) / 6.0;
    if (!(std::isfinite(motor_rpm_double) && motor_rpm_double >= 1.0)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "computed motor rpm fallback is invalid"});
    }
    const auto motor_rpm = static_cast<std::uint16_t>(
        std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::llround(motor_rpm_double)),
            1U,
            3000U));
    const auto speed_result = worker_->set_default_speed_rpm(motor_rpm);
    if (!speed_result.ok()) {
        return speed_result;
    }

    const auto accel_percent = configurator_->limits_max_accel_percent();
    if (!std::isfinite(accel_percent)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "invalid acceleration percent for runtime sync"});
    }
    const auto accel_clamped = std::clamp(accel_percent, 0.0, 100.0);
    const auto accel_byte = static_cast<std::uint8_t>(std::llround((accel_clamped / 100.0) * 255.0));
    const auto accel_result = worker_->set_default_accel_byte(accel_byte);
    if (!accel_result.ok()) {
        return accel_result;
    }

    return motion_core::Result<void>::success();
}


std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config) {
    return std::make_shared<MksAxisAdapter>(std::move(config));
}

} // namespace mks
