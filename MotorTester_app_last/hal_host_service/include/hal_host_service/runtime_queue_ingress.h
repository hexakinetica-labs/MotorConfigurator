#pragma once

#include "motion_core/hal_runtime.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace hal_host_service {

enum class MotionControlSource {
    Ui = 0,
    HexaMotion = 1,
};

struct RuntimeQueueIngressState {
    MotionControlSource control_source{MotionControlSource::Ui};
    bool hexamotion_connected{false};
    bool estop_active{false};
};

class RuntimeQueueIngress final {
public:
    using StateProvider = std::function<RuntimeQueueIngressState()>;

    RuntimeQueueIngress(motion_core::HalRuntime& runtime, StateProvider state_provider)
        : runtime_(runtime), state_provider_(std::move(state_provider)) {
    }

    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> submit_motion_batch(
        const MotionControlSource source,
        const std::uint16_t axis_id,
        const std::vector<motion_core::QueuedSetpoint>& batch) const {
        const auto access_result = authorize(source);
        if (!access_result.ok()) {
            return motion_core::Result<motion_core::MotionQueueStats>::failure(access_result.error());
        }

        if (batch.empty()) {
            return query_motion_queue_stats(source, axis_id);
        }

        const auto axis_result = runtime_.find_axis(axis_id);
        if (!axis_result.ok()) {
            return motion_core::Result<motion_core::MotionQueueStats>::failure(axis_result.error());
        }
        return axis_result.value()->enqueue_motion_batch(batch);
    }

    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats(
        const MotionControlSource source,
        const std::uint16_t axis_id) const {
        const auto access_result = authorize(source);
        if (!access_result.ok()) {
            return motion_core::Result<motion_core::MotionQueueStats>::failure(access_result.error());
        }

        return query_motion_queue_stats_observed(axis_id);
    }

    [[nodiscard]] motion_core::Result<motion_core::MotionQueueStats> query_motion_queue_stats_observed(
        const std::uint16_t axis_id) const {
        const auto axis_result = runtime_.find_axis(axis_id);
        if (!axis_result.ok()) {
            return motion_core::Result<motion_core::MotionQueueStats>::failure(axis_result.error());
        }
        return axis_result.value()->query_motion_queue_stats();
    }

private:
    [[nodiscard]] motion_core::Result<void> authorize(const MotionControlSource source) const {
        if (!state_provider_) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::InternalError, "runtime queue ingress state provider is not configured"});
        }

        const RuntimeQueueIngressState state = state_provider_();
        if (state.estop_active) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "global estop is active"});
        }

        if (source == MotionControlSource::Ui) {
            if (state.control_source != MotionControlSource::Ui) {
                return motion_core::Result<void>::failure(
                    {motion_core::ErrorCode::Busy, "ui motion source is not active"});
            }
            return motion_core::Result<void>::success();
        }

        if (state.control_source != MotionControlSource::HexaMotion) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "hexamotion motion source is not active"});
        }
        if (!state.hexamotion_connected) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::NotConnected, "hexamotion is not connected"});
        }
        return motion_core::Result<void>::success();
    }

    motion_core::HalRuntime& runtime_;
    StateProvider state_provider_{};
};

} // namespace hal_host_service