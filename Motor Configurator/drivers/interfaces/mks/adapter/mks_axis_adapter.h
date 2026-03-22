#pragma once

#include "mks/manager/mks_can_bus_manager.h"
#include "motion_core/axis_interface.h"

#include <memory>
#include <mutex>
#include <string>
#include <atomic>

namespace mks {

struct MksAxisAdapterConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    std::uint16_t can_id{1};
    std::shared_ptr<MksCanBusManager> bus_manager{};
    bool auto_start_bus_manager{true};

    // Conversion between human angle and motor axis units.
    double axis_units_per_degree{16384.0 / 360.0};

    // Default motion profile used for absolute position commands.
    std::uint16_t default_speed{300};
    std::uint8_t default_accel{10};
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

    void process_cycle(MksProtocol& protocol);

private:
    motion_core::Result<void> ensure_started() const;
    motion_core::Result<void> set_enabled_locked(bool enabled);
    void recalculate_axis_units_per_degree_from_software_params();

    [[nodiscard]] int mode_to_work_mode(motion_core::AxisMode mode) const;

    MksAxisAdapterConfig config_{};

    std::atomic<bool> started_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<motion_core::AxisMode> mode_{motion_core::AxisMode::ProfilePosition};
    std::atomic<std::uint16_t> runtime_can_id_{1};
    
    // Command requests from UI to bus
    std::atomic<bool> emergency_stop_req_{false};
    std::atomic<bool> clear_errors_req_{false};
    std::atomic<bool> go_home_req_{false};
    std::atomic<bool> set_zero_req_{false};
    
    std::atomic<double> target_position_deg_{0.0};
    std::atomic<double> target_velocity_deg_s_{0.0};
    
    std::atomic<bool> has_new_target_pos_{false};
    std::atomic<bool> has_new_target_vel_{false};

    // Motion profile applied to the next position command
    std::atomic<std::uint16_t> pending_speed_{300};
    std::atomic<std::uint8_t>  pending_accel_{10};

    // Telemetry round-robin phase (0..3)
    std::atomic<int> telem_phase_{0};

    // Telemetry from bus to UI
    struct SharedTelemetry {
        double actual_position_deg{0.0};
        double actual_velocity_deg_s{0.0};
        std::int16_t motor_speed_rpm{0};
        std::uint8_t status_word{0};
        std::uint8_t motor_status{0};
        std::int64_t raw_axis_position{0};
        std::uint64_t timestamp_ns{0};
    };
    
    mutable std::mutex telem_mutex_;
    SharedTelemetry telem_{};

    // Config
    mutable std::mutex config_mutex_;
    double software_gear_ratio_{1.0};
    std::uint32_t software_encoder_resolution_bits_{14};
    bool software_invert_direction_{false};
    std::atomic<double> axis_units_per_degree_{16384.0 / 360.0};
};

[[nodiscard]] std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config);

} // namespace mks
