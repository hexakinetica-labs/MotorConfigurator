#pragma once

#include "mks_can/axis/mks_axis_configurator.h"
#include "mks_can/axis/axis_cycle_metrics.h"
#include "mks_can/axis/mks_axis_state_cache.h"
#include "mks_can/axis/mks_axis_worker.h"
#include "mks_can/manager/mks_can_bus_manager.h"
#include "motion_core/axis_interface.h"
#include "motion_core/spsc_queue.h"

#include <atomic>
#include <memory>

namespace mks {

struct MksAxisAdapterConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    std::uint16_t can_id{1};
    std::shared_ptr<MksCanBusManager> bus_manager{};
    bool auto_start_bus_manager{true};

    double axis_units_per_degree{(16384.0 * 100.0) / 360.0};
    double software_gear_ratio{100.0};
    bool invert_direction{false};
    std::uint16_t default_speed_rpm{1800};
    std::uint8_t default_accel_byte{255};
};

struct AxisPositionSample final {
    std::uint64_t timestamp_ns{0U};
    double position_deg{0.0};
};

class MksAxisAdapter final : public motion_core::IAxis {
public:
    explicit MksAxisAdapter(MksAxisAdapterConfig config);
    ~MksAxisAdapter() override;

    [[nodiscard]] motion_core::AxisInfo info() const override;
    motion_core::Result<void> start() override;
    motion_core::Result<void> stop() override;

    motion_core::Result<void> set_enabled(bool enabled) override;
    motion_core::Result<void> set_mode(motion_core::AxisMode mode) override;
    motion_core::Result<void> apply_command(const motion_core::AxisCommand& command) override;
    [[nodiscard]] motion_core::Result<motion_core::AxisTelemetry> read_telemetry() const override;

    [[nodiscard]] motion_core::Result<std::vector<motion_core::ParameterDescriptor>> list_parameters() const override;
    [[nodiscard]] motion_core::Result<motion_core::ParameterSet> read_parameters() const override;
    motion_core::Result<void> apply_parameter_patch(const motion_core::ParameterPatch& patch) override;
    [[nodiscard]] motion_core::Result<motion_core::PersistentWriteReport> set_persistent(
        motion_core::PersistentCommand command,
        const motion_core::ParameterValue& value) override;

    motion_core::Result<void> configure_motion_queue(std::size_t capacity, bool drop_oldest) override;
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> enqueue_motion_batch(
        const std::vector<motion_core::QueuedSetpoint>& points) override;
    motion_core::Result<void> clear_motion_queue() override;
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats() const override;

    bool consume_tx_command(MksBusCommand& command);
    void publish_polled_telemetry(const motion_core::AxisTelemetry& telemetry);
    void mark_command_tx();
    void mark_telemetry_publish();
    void mark_position_rx();
    void mark_speed_rx();
    void mark_status_rx();
    void mark_protection_rx();
    [[nodiscard]] AxisCycleMetricsSnapshot cycle_metrics_snapshot() const;
    [[nodiscard]] std::size_t drain_position_samples(std::vector<AxisPositionSample>& out_samples,
                                                     std::size_t max_samples) const;
    [[nodiscard]] std::uint16_t runtime_can_id() const;
    [[nodiscard]] double axis_units_per_degree() const;
    [[nodiscard]] double requested_target_position_deg() const;
    [[nodiscard]] bool has_requested_target_position() const;
    [[nodiscard]] double software_gear_ratio() const;
    [[nodiscard]] bool invert_direction() const;
    [[nodiscard]] motion_core::AxisMode current_mode() const;
    [[nodiscard]] motion_core::Result<void> sync_motion_runtime_from_configurator();

private:
    [[nodiscard]] motion_core::Result<void> ensure_started() const;

    MksAxisAdapterConfig config_{};
    MksAxisStateCache state_cache_{};
    std::unique_ptr<MksAxisWorker> worker_{};
    std::unique_ptr<MksAxisConfigurator> configurator_{};
    std::atomic<bool> started_{false};
    std::atomic<std::uint16_t> runtime_can_id_{1};
    std::atomic<double> axis_units_per_degree_runtime_{(16384.0 * 100.0) / 360.0};
    std::atomic<double> requested_target_position_deg_{0.0};
    std::atomic<bool> has_requested_target_position_{false};
    AxisCycleMetrics cycle_metrics_{};
    mutable motion_core::SpscQueue<AxisPositionSample, 2048U> position_samples_{};
};

[[nodiscard]] std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config);

} // namespace mks


