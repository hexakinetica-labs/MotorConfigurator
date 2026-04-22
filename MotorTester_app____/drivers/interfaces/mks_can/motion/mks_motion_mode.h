#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/result.h"
#include "motion_core/spsc_queue.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
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
    std::uint16_t fallback_speed_rpm{300};
    std::uint8_t fallback_accel_byte{16};
};

struct MksMotionStepResult final {
    bool has_command{false};
    MksBusCommand command{};
};

class MksMotionModeBase {
public:
    static constexpr std::size_t kMotionQueuePhysicalCapacity = 2048U;

    virtual ~MksMotionModeBase() = default;

    motion_core::Result<void> set_motion_queue_policy(std::size_t capacity, bool drop_oldest);
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> enqueue_motion_batch(
        const std::vector<motion_core::QueuedSetpoint>& points);
    motion_core::Result<void> clear_motion_queue();
    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats() const;

    virtual void on_mode_enter(std::chrono::steady_clock::time_point now);
    virtual void on_mode_exit();

    [[nodiscard]] virtual motion_core::Result<MksMotionStepResult> step(
        std::chrono::steady_clock::time_point now,
        const MksMotionBuildContext& context) = 0;

protected:
    [[nodiscard]] bool try_pop_setpoint(motion_core::QueuedSetpoint& point);
    [[nodiscard]] std::size_t queue_size_approx() const;
    void mark_underrun();
    void mark_short_start();

    virtual void on_queue_cleared();

private:
    motion_core::SpscQueue<motion_core::QueuedSetpoint, kMotionQueuePhysicalCapacity + 1U> motion_queue_{};
    std::atomic<std::size_t> motion_queue_capacity_limit_{kMotionQueuePhysicalCapacity};
    std::atomic<bool> motion_queue_drop_oldest_policy_{true};

    std::atomic<std::uint64_t> motion_points_pushed_{0U};
    std::atomic<std::uint64_t> motion_points_dropped_{0U};
    std::atomic<std::uint64_t> motion_queue_underruns_{0U};
    std::atomic<std::uint64_t> motion_queue_short_starts_{0U};
};

class MksAbsolutePositionMotionMode final : public MksMotionModeBase {
public:
    void on_mode_enter(std::chrono::steady_clock::time_point now) override;
    void on_mode_exit() override;

    [[nodiscard]] motion_core::Result<MksMotionStepResult> step(
        std::chrono::steady_clock::time_point now,
        const MksMotionBuildContext& context) override;

private:
    void on_queue_cleared() override;

    [[nodiscard]] static motion_core::Result<MksBusCommand> build_absolute_command(
        const MksMotionBuildContext& context,
        const motion_core::QueuedSetpoint& point);

    bool has_pending_point_{false};
    motion_core::QueuedSetpoint pending_point_{};
    std::chrono::steady_clock::time_point next_emit_time_{};
};

class MksVelocityMotionMode final : public MksMotionModeBase {
public:
    void on_mode_enter(std::chrono::steady_clock::time_point now) override;
    void on_mode_exit() override;

    [[nodiscard]] motion_core::Result<MksMotionStepResult> step(
        std::chrono::steady_clock::time_point now,
        const MksMotionBuildContext& context) override;

private:
    void on_queue_cleared() override;

    [[nodiscard]] static motion_core::Result<MksBusCommand> build_velocity_command(
        const MksMotionBuildContext& context,
        const motion_core::QueuedSetpoint& point);

    bool has_pending_point_{false};
    motion_core::QueuedSetpoint pending_point_{};
    std::chrono::steady_clock::time_point next_emit_time_{};
};

} // namespace mks
