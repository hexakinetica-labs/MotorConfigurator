#pragma once

#include "motion_core/axis_data.h"

#include <atomic>

namespace mks {

class MksAxisStateCache final {
public:
    MksAxisStateCache() = default;

    void set_latest_telemetry(const motion_core::AxisTelemetry& telemetry) {
        latest_telemetry_.store(telemetry, std::memory_order_release);
    }

    [[nodiscard]] motion_core::AxisTelemetry latest_telemetry() const {
        return latest_telemetry_.load(std::memory_order_acquire);
    }

    void set_mode(const motion_core::AxisMode mode) {
        mode_.store(mode, std::memory_order_release);
    }

    [[nodiscard]] motion_core::AxisMode mode() const {
        return mode_.load(std::memory_order_acquire);
    }

    void set_motion_active(const bool active) {
        motion_active_.store(active, std::memory_order_release);
    }

    [[nodiscard]] bool motion_active() const {
        return motion_active_.load(std::memory_order_acquire);
    }

    void set_config_busy(const bool busy) {
        config_busy_.store(busy, std::memory_order_release);
    }

    [[nodiscard]] bool config_busy() const {
        return config_busy_.load(std::memory_order_acquire);
    }

private:
    std::atomic<motion_core::AxisTelemetry> latest_telemetry_{};
    std::atomic<motion_core::AxisMode> mode_{motion_core::AxisMode::ProfilePosition};
    std::atomic<bool> motion_active_{false};
    std::atomic<bool> config_busy_{false};
};

} // namespace mks
