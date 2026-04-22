#include "mks_can/adapter/mks_axis_adapter.h"

#include <algorithm>
#include <cmath>

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
}

MksAxisAdapter::~MksAxisAdapter() {
    (void)stop();
}

motion_core::AxisInfo MksAxisAdapter::info() const {
    motion_core::AxisInfo out{};
    out.id = config_.axis_id;
    out.name = config_.axis_name;
    out.transport = motion_core::AxisTransportKind::CanBus;
    out.capabilities = {
        motion_core::Capability::ReadTelemetry,
        motion_core::Capability::SetTargetPosition,
        motion_core::Capability::SetTargetVelocity,
        motion_core::Capability::EnableDisable,
        motion_core::Capability::EmergencyStop,
        motion_core::Capability::ReadParameters,
        motion_core::Capability::WriteParameters,
        motion_core::Capability::Homing,
    };
    return out;
}

motion_core::Result<void> MksAxisAdapter::start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return motion_core::Result<void>::success();
    }

    if (!config_.bus_manager) {
        started_.store(false, std::memory_order_release);
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "bus_manager is null"});
    }

    if (config_.auto_start_bus_manager) {
        const auto start_bus = config_.bus_manager->start();
        if (!start_bus.ok()) {
            started_.store(false, std::memory_order_release);
            return start_bus;
        }
    }

    const auto reg = config_.bus_manager->register_adapter(runtime_can_id_.load(std::memory_order_acquire), this);
    if (!reg.ok()) {
        started_.store(false, std::memory_order_release);
        return reg;
    }

    const auto worker_started = worker_->start();
    if (!worker_started.ok()) {
        (void)config_.bus_manager->unregister_adapter(runtime_can_id_.load(std::memory_order_acquire), this);
        started_.store(false, std::memory_order_release);
        return worker_started;
    }

    const auto configurator_started = configurator_->start();
    if (!configurator_started.ok()) {
        (void)worker_->stop();
        (void)config_.bus_manager->unregister_adapter(runtime_can_id_.load(std::memory_order_acquire), this);
        started_.store(false, std::memory_order_release);
        return configurator_started;
    }

    const auto sync_result = sync_motion_runtime_from_configurator();
    if (!sync_result.ok()) {
        (void)configurator_->stop();
        (void)worker_->stop();
        (void)config_.bus_manager->unregister_adapter(runtime_can_id_.load(std::memory_order_acquire), this);
        started_.store(false, std::memory_order_release);
        return sync_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::stop() {
    bool expected = true;
    if (!started_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        return motion_core::Result<void>::success();
    }
    if (configurator_) {
        (void)configurator_->stop();
    }
    (void)worker_->stop();
    if (config_.bus_manager) {
        (void)config_.bus_manager->unregister_adapter(runtime_can_id_.load(std::memory_order_acquire), this);
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::set_enabled(const bool enabled) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }
    if (!enabled) {
        has_requested_target_position_.store(false, std::memory_order_release);
    }
    return worker_->request_enable(enabled);
}

motion_core::Result<void> MksAxisAdapter::set_mode(const motion_core::AxisMode mode) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }
    const auto result = worker_->set_mode(mode);
    if (result.ok()) {
        state_cache_.set_mode(mode);
    }
    return result;
}

motion_core::Result<void> MksAxisAdapter::apply_command(const motion_core::AxisCommand& command) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);

    if (command.emergency_stop) {
        has_requested_target_position_.store(false, std::memory_order_release);
        return worker_->request_emergency_stop();
    }
    if (command.clear_errors) {
        return worker_->request_clear_errors();
    }
    if (command.go_home) {
        has_requested_target_position_.store(false, std::memory_order_release);
        return worker_->request_home();
    }
    if (command.set_zero) {
        has_requested_target_position_.store(true, std::memory_order_release);
        requested_target_position_deg_.store(0.0, std::memory_order_release);
        return worker_->request_set_zero();
    }

    if (!command.has_target_position && !command.has_target_velocity) {
        return motion_core::Result<void>::success();
    }

    motion_core::QueuedSetpoint point{};
    bool updates_position_target = false;
    if (command.has_target_position) {
        point.target_position_deg = command.target_position_deg;
        updates_position_target = true;
    }
    if (command.has_profile_speed_rpm) {
        point.has_profile_speed_rpm = true;
        point.profile_speed_rpm = command.profile_speed_rpm;
    }
    if (command.has_profile_accel_percent) {
        point.has_profile_accel_percent = true;
        point.profile_accel_percent = command.profile_accel_percent;
    }
    if (command.has_target_velocity) {
        point.has_target_velocity = true;
        point.target_velocity_deg_per_sec = command.target_velocity_deg_per_sec;
    }
    if (command.is_relative) {
        double base_pos = 0.0;
        if (has_requested_target_position_.load(std::memory_order_acquire)) {
            base_pos = requested_target_position_deg_.load(std::memory_order_acquire);
        } else {
            auto telem_result = worker_->read_telemetry();
            if (!telem_result.ok()) {
                return motion_core::Result<void>::failure(telem_result.error());
            }
            base_pos = telem_result.value().actual_position_deg;
        }
        point.target_position_deg = base_pos + command.target_position_deg;
        updates_position_target = true;
    }
    if (updates_position_target) {
        has_requested_target_position_.store(true, std::memory_order_release);
        requested_target_position_deg_.store(point.target_position_deg, std::memory_order_release);
    }

    std::vector<motion_core::QueuedSetpoint> single_point;
    single_point.push_back(point);
    const auto enqueue_result = worker_->enqueue_motion_batch(single_point);
    if (!enqueue_result.ok()) {
        return motion_core::Result<void>::failure(enqueue_result.error());
    }
    state_cache_.set_motion_active(true);
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::AxisTelemetry> MksAxisAdapter::read_telemetry() const {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::AxisTelemetry>::failure(status.error());
    }
    return motion_core::Result<motion_core::AxisTelemetry>::success(state_cache_.latest_telemetry());
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> MksAxisAdapter::list_parameters() const {
    return configurator_->list_parameters();
}

motion_core::Result<motion_core::ParameterSet> MksAxisAdapter::read_parameters() const {
    return configurator_->read_parameters();
}

motion_core::Result<void> MksAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    const auto result = configurator_->apply_parameter_patch(patch);
    if (!result.ok()) {
        return result;
    }
    return sync_motion_runtime_from_configurator();
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

motion_core::Result<void> MksAxisAdapter::configure_motion_queue(const std::size_t capacity,
                                                                 const bool drop_oldest) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }
    return worker_->set_motion_queue_policy(capacity, drop_oldest);
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisAdapter::enqueue_motion_batch(
    const std::vector<motion_core::QueuedSetpoint>& points) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(status.error());
    }
    const auto enqueue_result = worker_->enqueue_motion_batch(points);
    if (!enqueue_result.ok()) {
        return enqueue_result;
    }

    if (!points.empty()) {
        const auto& last_point = points.back();
        has_requested_target_position_.store(true, std::memory_order_release);
        requested_target_position_deg_.store(last_point.target_position_deg, std::memory_order_release);
        state_cache_.set_motion_active(true);
    }

    return enqueue_result;
}

motion_core::Result<void> MksAxisAdapter::clear_motion_queue() {
    const auto status = ensure_started();
    if (!status.ok()) {
        return status;
    }
    return worker_->clear_motion_queue();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisAdapter::query_motion_queue_stats() const {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(status.error());
    }
    return worker_->query_motion_queue_stats();
}

bool MksAxisAdapter::consume_tx_command(MksBusCommand& command) {
    return worker_ && worker_->consume_tx_command(command);
}

void MksAxisAdapter::step_worker(const std::chrono::steady_clock::time_point now) {
    if (worker_) {
        worker_->step(now);
    }
}

void MksAxisAdapter::publish_polled_telemetry(const motion_core::AxisTelemetry& telemetry) {
    AxisPositionSample sample{};
    sample.timestamp_ns = telemetry.timestamp_ns;
    sample.position_deg = telemetry.actual_position_deg;
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

motion_core::Result<void> MksAxisAdapter::ensure_started() const {
    if (!started_.load(std::memory_order_acquire)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "axis adapter is not started"});
    }
    return motion_core::Result<void>::success();
}

std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config) {
    return std::make_shared<MksAxisAdapter>(std::move(config));
}

} // namespace mks
