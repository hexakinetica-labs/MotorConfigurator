#pragma once

#include "motion_core/axis_interface.h"
#include "motion_core/spsc_queue.h"
#include "ethercat/manager/ethercat_bus_manager.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>

namespace ethercat_driver {

struct EthercatAxisAdapterConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    int ecat_axis_index{0};
    uint16_t ecat_bus_position{0};
    std::shared_ptr<EthercatBusManager> bus_manager{};
};

class EthercatAxisAdapter final : public motion_core::IAxis {
public:
    explicit EthercatAxisAdapter(EthercatAxisAdapterConfig config);
    ~EthercatAxisAdapter() override;

    [[nodiscard]] motion_core::AxisInfo info() const override;

    motion_core::Result<void> configure_hardware();

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

    // Called by EthercatBusManager's RT loop
    void process_cycle(uint8_t* domain_pd, double dt_s);

private:
    motion_core::Result<void> ensure_started() const;
    void recalculate_counts_per_radian();
    [[nodiscard]] int8_t mode_to_work_mode(motion_core::AxisMode mode) const;
    [[nodiscard]] std::vector<motion_core::ParameterDescriptor> make_parameter_descriptors() const;
    [[nodiscard]] motion_core::Result<std::vector<std::uint8_t>> request_sdo_read(
        std::uint16_t index,
        std::uint8_t sub_index,
        std::size_t expected_size) const;
    [[nodiscard]] motion_core::Result<void> request_sdo_write(
        std::uint16_t index,
        std::uint8_t sub_index,
        const std::vector<std::uint8_t>& payload);
    [[nodiscard]] ec_sdo_request_t* write_request_for_size(std::size_t payload_size) const;
    
    // Command atomic state from NRT to RT
    struct AtomicCommand {
        std::atomic<bool> enable_req{false};
        std::atomic<bool> reset_req{false};
        std::atomic<int8_t> mode_req{8}; // CSP
        std::atomic<double> target_pos_deg{0.0};
        std::atomic<std::int32_t> target_pos_counts{0};
        std::atomic<std::uint16_t> controlword_last_sent{0};
        std::atomic<bool> homing_req{false};
        std::atomic<bool> homing_active{false};
        std::atomic<bool> manual_homing_active{false};
        std::atomic<bool> pp_new_setpoint_req{false};
        std::atomic<bool> pp_setpoint_bit_state{false};
        std::atomic<bool> set_zero_req{false};
        std::atomic<bool> has_target_vel{false};
        std::atomic<double> target_vel_deg_s{0.0};
        std::atomic<bool> has_profile_vel{false};
        std::atomic<std::uint16_t> profile_vel_rpm{0};
    };
    
    // Telemetry atomic state from RT to NRT
    struct AtomicTelemetry {
        std::atomic<uint64_t> timestamp_ns{0};
        std::atomic<std::int32_t> actual_position_counts{0};
        std::atomic<double> actual_position_deg{0.0};
        std::atomic<double> actual_velocity_deg_s{0.0};
        std::atomic<double> actual_torque_pct{0.0};
        std::atomic<std::uint32_t> digital_inputs{0U};
        std::atomic<std::int8_t> mode_display{0};
        std::atomic<uint16_t> statusword{0};
        std::atomic<uint16_t> error_code{0};
        std::atomic<bool> is_ready{false};
        std::atomic<bool> is_faulted{false};
        std::atomic<bool> is_enabled{false};
    };

    EthercatAxisAdapterConfig config_{};
    
    AtomicCommand cmd_;
    AtomicTelemetry telem_;

    std::atomic<bool> started_{false};
    std::atomic<motion_core::AxisMode> mode_{motion_core::AxisMode::ProfilePosition};
    
    // PDO offsets assigned by ecrt
    unsigned int off_ctrl_{0}, off_target_pos_{0}, off_modes_op_{0}, off_max_vel_{0};
    unsigned int off_status_{0}, off_modes_display_{0}, off_act_pos_{0}, off_error_{0}, off_act_torque_{0};
    unsigned int off_target_vel_{0}, off_digital_inputs_{0};
    
    // Hardware params
    std::atomic<double> gear_ratio_{1.0};
    // Instruction units per motor revolution (0x6091:02, default 10000).
    // PDO 0x6064/0x607A use these units — NOT raw encoder counts.
    std::atomic<std::uint32_t> counts_per_revolution_{10000};
    std::atomic<std::uint32_t> max_profile_velocity_instr_s_{0};
    std::atomic<bool> has_max_profile_velocity_instr_s_{false};
    std::atomic<double> counts_per_radian_{1.0};
    std::atomic<double> home_switch_to_zero_shift_deg_{0.0};
    std::atomic<std::int32_t> zero_offset_counts_{0};
    std::atomic<bool> has_last_position_sample_{false};
    double last_position_deg_{0.0};
    std::uint64_t last_timestamp_ns_{0};
    ec_slave_config_t* slave_config_{nullptr};
    ec_sdo_request_t* read_sdo_request_{nullptr};
    std::array<ec_sdo_request_t*, 4> write_sdo_requests_{{nullptr, nullptr, nullptr, nullptr}};
    mutable std::mutex sdo_request_mutex_;

    static constexpr std::size_t kMotionQueuePhysicalCapacity = 2048U;
    motion_core::SpscQueue<motion_core::QueuedSetpoint, kMotionQueuePhysicalCapacity + 1U> motion_queue_{};
    std::atomic<std::size_t> motion_queue_capacity_limit_{kMotionQueuePhysicalCapacity};
    std::atomic<bool> motion_queue_drop_oldest_policy_{true};
    std::atomic<bool> clear_motion_queue_req_{false};
    std::atomic<std::uint64_t> motion_points_pushed_{0U};
    std::atomic<std::uint64_t> motion_points_dropped_{0U};
    std::atomic<std::uint64_t> motion_underruns_{0U};
    std::atomic<std::uint64_t> motion_short_starts_{0U};
    std::atomic<bool> motion_queue_was_populated_{false};
    std::atomic<bool> has_queued_target_{false};
    std::atomic<double> queued_target_position_deg_{0.0};

    bool interpolator_initialized_{false};
    double interpolated_target_deg_{0.0};
};

[[nodiscard]] std::shared_ptr<motion_core::IAxis> make_ethercat_axis_adapter(EthercatAxisAdapterConfig config);
[[nodiscard]] motion_core::Result<std::shared_ptr<motion_core::IAxis>>
make_configured_ethercat_axis_adapter(EthercatAxisAdapterConfig config);

} // namespace ethercat_driver
