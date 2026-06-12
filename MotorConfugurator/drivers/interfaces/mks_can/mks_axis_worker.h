#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/result.h"
#include "motion_core/spsc_queue.h"
#include "motion_core/types.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mks {

struct MksBusCommand {
    std::uint16_t can_id{0};
    std::uint8_t command{0};
    std::array<std::uint8_t, 8> payload{};
    std::uint8_t payload_size{0};
    bool requires_sync_execute{false};

    void push_back(std::uint8_t byte) {
        if (payload_size < payload.size()) {
            payload[payload_size++] = byte;
        }
    }

    void append_be16(std::uint16_t value) {
        push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
        push_back(static_cast<std::uint8_t>(value & 0xFF));
    }

    void append_be24(std::int32_t value) {
        const std::uint32_t v = static_cast<std::uint32_t>(value) & 0x00FFFFFFu;
        push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
        push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
        push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
};

struct MksMotionBuildContext {
    std::uint16_t can_id{0};
    double axis_units_per_degree{1.0};
    double software_gear_ratio{1.0};
    double bus_cycle_period_sec{0.0025};
    bool invert_direction{false};
    bool telemetry_invert_position_sign{false};
    std::uint16_t fallback_speed_rpm{300};
    std::uint8_t fallback_accel_byte{204};
};
class MksAxisWorker final {
public:
    struct Config {
        motion_core::AxisId axis_id{};
        std::uint16_t can_id{1};
        double axis_units_per_degree{16384.0 / 360.0};
        double software_gear_ratio{1.0};
        bool invert_direction{false};
        bool telemetry_invert_position_sign{false};
        std::uint16_t default_speed_rpm{300};
        std::uint8_t default_accel_byte{204};
        std::chrono::microseconds cycle_period{5'000};
    };

    explicit MksAxisWorker(Config config);
    ~MksAxisWorker();

    MksAxisWorker(const MksAxisWorker&) = delete;
    MksAxisWorker& operator=(const MksAxisWorker&) = delete;



    void set_runtime_can_id(std::uint16_t can_id);
    motion_core::Result<void> set_axis_units_per_degree(double axis_units_per_degree);
    motion_core::Result<void> set_software_gear_ratio(double software_gear_ratio);
    motion_core::Result<void> set_invert_direction(bool invert_direction);
    void set_telemetry_invert_position_sign(bool invert);
    motion_core::Result<void> set_default_speed_rpm(std::uint16_t default_speed_rpm);
    motion_core::Result<void> set_default_accel_byte(std::uint8_t default_accel_byte);

    motion_core::Result<void> set_mode(motion_core::AxisMode mode);

    bool enqueue_service_point(const motion_core::ServiceCommandPoint& point);
    bool enqueue_command_batch(const std::vector<motion_core::MotionCommandPoint>& points);
    void request_emergency_stop();
    void clear_motion_queue();
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats() const;

    [[nodiscard]] motion_core::Result<motion_core::TelemetrySnapshot> read_telemetry() const;
    void drain_telemetry_queue() const;
    bool consume_tx_command(MksBusCommand& command);
    void publish_telemetry(const motion_core::TelemetrySnapshot& telemetry);
    void step(std::chrono::steady_clock::time_point now);

private:
    static constexpr std::size_t kTelemetryQueuePhysicalCapacity = 1024U;
    static constexpr std::size_t kTxQueuePhysicalCapacity = 256U;
    static constexpr std::size_t kServiceQueuePhysicalCapacity = 256U;

    static constexpr std::uint32_t kServiceFlagEmergencyStop = 1U << 0U;
    static constexpr std::uint32_t kServiceFlagClearErrors = 1U << 1U;
    static constexpr std::uint32_t kServiceFlagHome = 1U << 2U;
    static constexpr std::uint32_t kServiceFlagSetZero = 1U << 3U;

    bool handle_service_request();
    void handle_service_queue_command(const motion_core::ServiceCommandPoint& command);
    void handle_motion_request(std::chrono::steady_clock::time_point now);
    void push_tx_command(const MksBusCommand& command);
    void push_telemetry_sample(const motion_core::TelemetrySnapshot& telemetry);
    [[nodiscard]] static motion_core::Result<MksBusCommand> build_absolute_command(
        const MksMotionBuildContext& context,
        const motion_core::MotionCommandPoint& point);

    [[nodiscard]] static motion_core::Result<MksBusCommand> build_velocity_command(
        const MksMotionBuildContext& context,
        const motion_core::MotionCommandPoint& point);

    static constexpr std::size_t kMotionQueuePhysicalCapacity = 2048U;

    Config config_{};

    std::atomic<bool> estop_latched_{false};
    std::atomic<motion_core::AxisMode> mode_{motion_core::AxisMode::ProfilePosition};
    std::atomic<int> pending_work_mode_{-1}; // -1=no request, [0..5]=SetWorkMode payload
    std::atomic<int> current_hardware_mode_{-1};
    std::atomic<std::uint16_t> runtime_can_id_{1};
    std::atomic<double> axis_units_per_degree_runtime_{16384.0 / 360.0};
    std::atomic<double> software_gear_ratio_runtime_{1.0};
    std::atomic<bool> invert_direction_runtime_{false};
    std::atomic<bool> telemetry_invert_position_sign_runtime_{false};
    std::atomic<std::uint16_t> default_speed_rpm_runtime_{300};
    std::atomic<std::uint8_t> default_accel_byte_runtime_{204};
    mutable motion_core::SpscQueue<motion_core::TelemetrySnapshot, kTelemetryQueuePhysicalCapacity + 1U> telemetry_queue_{};
    motion_core::SpscQueue<motion_core::ServiceCommandPoint, kServiceQueuePhysicalCapacity + 1U> service_queue_{};
    motion_core::SpscQueue<MksBusCommand, kTxQueuePhysicalCapacity + 1U> tx_queue_{};
    std::atomic<std::uint64_t> telemetry_points_pushed_{0U};
    std::atomic<std::uint64_t> telemetry_points_dropped_{0U};

    std::atomic<motion_core::TelemetrySnapshot> latest_telemetry_{};

    motion_core::SpscQueue<motion_core::MotionCommandPoint, kMotionQueuePhysicalCapacity + 1U> motion_queue_{};
    std::atomic<std::size_t> motion_queue_capacity_limit_{kMotionQueuePhysicalCapacity};
    std::atomic<bool> motion_queue_drop_oldest_policy_{true};
    std::atomic<std::uint64_t> motion_points_pushed_{0U};
    std::atomic<std::uint64_t> motion_points_dropped_{0U};
    std::atomic<std::uint64_t> motion_queue_underruns_{0U};
    std::atomic<std::uint64_t> motion_queue_short_starts_{0U};
};

} // namespace mks
