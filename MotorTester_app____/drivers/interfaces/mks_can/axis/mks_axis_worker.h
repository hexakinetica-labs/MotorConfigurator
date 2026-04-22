#pragma once

#include "mks_can/motion/mks_motion_mode.h"
#include "motion_core/axis_data.h"
#include "motion_core/result.h"
#include "motion_core/spsc_queue.h"
#include "motion_core/types.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mks {

class MksAxisWorker final {
public:
    struct Config {
        motion_core::AxisId axis_id{};
        std::uint16_t can_id{1};
        double axis_units_per_degree{16384.0 / 360.0};
        double software_gear_ratio{1.0};
        bool invert_direction{false};
        std::uint16_t default_speed_rpm{300};
        std::uint8_t default_accel_byte{16};
        std::chrono::microseconds cycle_period{5'000};
    };

    explicit MksAxisWorker(Config config);
    ~MksAxisWorker();

    MksAxisWorker(const MksAxisWorker&) = delete;
    MksAxisWorker& operator=(const MksAxisWorker&) = delete;

    motion_core::Result<void> start();
    motion_core::Result<void> stop();

    motion_core::Result<void> request_enable(bool enabled);
    motion_core::Result<void> request_emergency_stop();
    motion_core::Result<void> request_clear_errors();
    motion_core::Result<void> request_home();
    motion_core::Result<void> request_set_zero();
    void set_runtime_can_id(std::uint16_t can_id);
    motion_core::Result<void> set_axis_units_per_degree(double axis_units_per_degree);
    motion_core::Result<void> set_software_gear_ratio(double software_gear_ratio);
    motion_core::Result<void> set_invert_direction(bool invert_direction);
    motion_core::Result<void> set_default_speed_rpm(std::uint16_t default_speed_rpm);
    motion_core::Result<void> set_default_accel_byte(std::uint8_t default_accel_byte);

    motion_core::Result<void> set_mode(motion_core::AxisMode mode);
    motion_core::Result<void> set_motion_queue_policy(std::size_t capacity, bool drop_oldest);
    motion_core::Result<motion_core::MotionQueueStats> enqueue_motion_batch(
        const std::vector<motion_core::QueuedSetpoint>& points);
    motion_core::Result<void> clear_motion_queue();
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats() const;

    [[nodiscard]] motion_core::Result<motion_core::AxisTelemetry> read_telemetry() const;
    void drain_telemetry_queue() const;
    bool consume_tx_command(MksBusCommand& command);
    void publish_telemetry(const motion_core::AxisTelemetry& telemetry);
    void step(std::chrono::steady_clock::time_point now);

private:
    static constexpr std::size_t kTelemetryQueuePhysicalCapacity = 1024U;
    static constexpr std::size_t kTxQueuePhysicalCapacity = 256U;

    static constexpr std::uint32_t kServiceFlagEmergencyStop = 1U << 0U;
    static constexpr std::uint32_t kServiceFlagClearErrors = 1U << 1U;
    static constexpr std::uint32_t kServiceFlagHome = 1U << 2U;
    static constexpr std::uint32_t kServiceFlagSetZero = 1U << 3U;

    bool handle_service_request();
    void handle_motion_request(std::chrono::steady_clock::time_point now);
    void push_tx_command(const MksBusCommand& command);
    void push_telemetry_sample(const motion_core::AxisTelemetry& telemetry);
    [[nodiscard]] MksMotionModeBase* active_motion_mode();
    [[nodiscard]] MksMotionBuildContext build_motion_context() const;

    Config config_{};

    std::atomic<motion_core::AxisMode> mode_{motion_core::AxisMode::ProfilePosition};
    std::atomic<int> pending_enable_state_{-1}; // -1=no request, 0=disable, 1=enable
    std::atomic<int> pending_work_mode_{-1}; // -1=no request, [0..5]=SetWorkMode payload
    std::atomic<std::uint32_t> service_flags_{0U};
    std::atomic<std::uint16_t> runtime_can_id_{1};
    std::atomic<double> axis_units_per_degree_runtime_{16384.0 / 360.0};
    std::atomic<double> software_gear_ratio_runtime_{1.0};
    std::atomic<bool> invert_direction_runtime_{false};
    std::atomic<std::uint16_t> default_speed_rpm_runtime_{300};
    std::atomic<std::uint8_t> default_accel_byte_runtime_{16};
    mutable motion_core::SpscQueue<motion_core::AxisTelemetry, kTelemetryQueuePhysicalCapacity + 1U> telemetry_queue_{};
    motion_core::SpscQueue<MksBusCommand, kTxQueuePhysicalCapacity + 1U> tx_queue_{};
    std::atomic<std::uint64_t> telemetry_points_pushed_{0U};
    std::atomic<std::uint64_t> telemetry_points_dropped_{0U};

    std::atomic<motion_core::AxisTelemetry> latest_telemetry_{};

    MksAbsolutePositionMotionMode absolute_mode_{};
    MksVelocityMotionMode velocity_mode_{};
    MksMotionModeBase* active_mode_controller_{nullptr};
};

} // namespace mks
