#pragma once

#include "motion_core/motion_orchestrator.h"
#include <cstdint>
#include <unordered_map>
#include <atomic>
#include <mutex>

namespace motion_core {

struct SineConfig {
    double amplitude_deg{10.0};
    double frequency_hz{1.0};
    double center_deg{0.0};
    double sample_period_sec{0.004};
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(MotionOrchestrator& orchestrator);

    void start_sine(uint16_t axis_id, const SineConfig& config);
    void stop_sine(uint16_t axis_id);
    [[nodiscard]] bool is_sine_active(uint16_t axis_id) const;

    // Called from polling loop to generate and enqueue trajectory points for all active axes
    void update_all_queues();

private:
    struct AxisSineState {
        SineConfig config;
        double phase_accum_rad{0.0};
        bool active{false};
    };

    MotionOrchestrator& orchestrator_;
    mutable std::mutex mutex_;
    std::unordered_map<uint16_t, AxisSineState> sine_states_;
};

} // namespace motion_core
