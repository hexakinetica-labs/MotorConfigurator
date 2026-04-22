#include "mks_can/motion/mks_motion_mode.h"

#include <algorithm>

namespace mks {

motion_core::Result<void> MksMotionModeBase::set_motion_queue_policy(const std::size_t capacity,
                                                                     const bool drop_oldest) {
    if (capacity == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "motion queue capacity must be > 0"});
    }
    if (capacity > kMotionQueuePhysicalCapacity) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "motion queue capacity exceeds physical limit"});
    }

    motion_queue_capacity_limit_.store(capacity, std::memory_order_release);
    motion_queue_drop_oldest_policy_.store(drop_oldest, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksMotionModeBase::enqueue_motion_batch(
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

motion_core::Result<void> MksMotionModeBase::clear_motion_queue() {
    motion_core::QueuedSetpoint point{};
    while (motion_queue_.try_pop(point)) {
    }
    on_queue_cleared();
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> MksMotionModeBase::query_motion_queue_stats() const {
    motion_core::MotionQueueStats stats{};
    stats.size = motion_queue_.size_approx();
    stats.capacity = motion_queue_capacity_limit_.load(std::memory_order_acquire);
    stats.pushed = motion_points_pushed_.load(std::memory_order_acquire);
    stats.dropped = motion_points_dropped_.load(std::memory_order_acquire);
    stats.underruns = motion_queue_underruns_.load(std::memory_order_acquire);
    stats.short_starts = motion_queue_short_starts_.load(std::memory_order_acquire);
    return motion_core::Result<motion_core::MotionQueueStats>::success(stats);
}

void MksMotionModeBase::on_mode_enter(std::chrono::steady_clock::time_point) {
}

void MksMotionModeBase::on_mode_exit() {
}

bool MksMotionModeBase::try_pop_setpoint(motion_core::QueuedSetpoint& point) {
    return motion_queue_.try_pop(point);
}

std::size_t MksMotionModeBase::queue_size_approx() const {
    return motion_queue_.size_approx();
}

void MksMotionModeBase::mark_underrun() {
    motion_queue_underruns_.fetch_add(1U, std::memory_order_relaxed);
}

void MksMotionModeBase::mark_short_start() {
    motion_queue_short_starts_.fetch_add(1U, std::memory_order_relaxed);
}

void MksMotionModeBase::on_queue_cleared() {
}

} // namespace mks
