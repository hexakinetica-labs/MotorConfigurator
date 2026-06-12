#include "motion_core/trajectory_generator.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_core {

TrajectoryGenerator::TrajectoryGenerator(MotionOrchestrator& orchestrator)
    : orchestrator_(orchestrator) {}

void TrajectoryGenerator::start_sine(uint16_t axis_id, const SineConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = sine_states_[axis_id];
    state.config = config;
    state.phase_accum_rad = 0.0;
    state.active = true;
}

void TrajectoryGenerator::stop_sine(uint16_t axis_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sine_states_.find(axis_id);
    if (it != sine_states_.end()) {
        it->second.active = false;
    }
}

bool TrajectoryGenerator::is_sine_active(uint16_t axis_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sine_states_.find(axis_id);
    if (it != sine_states_.end()) {
        return it->second.active;
    }
    return false;
}

void TrajectoryGenerator::update_all_queues() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [axis_id, state] : sine_states_) {
        if (!state.active) {
            continue;
        }

        const double sp = state.config.sample_period_sec;
        const double freq = state.config.frequency_hz;
        const double amp = state.config.amplitude_deg;
        const double center = state.config.center_deg;

        // Determine how many points we can enqueue. Try to fill up to 50 points.
        size_t points_to_generate = 50; // Just an arbitrary chunk size
        
        for (size_t i = 0; i < points_to_generate; ++i) {
            state.phase_accum_rad += 2.0 * M_PI * freq * sp;
            if (state.phase_accum_rad > 2.0 * M_PI) {
                state.phase_accum_rad = std::fmod(state.phase_accum_rad, 2.0 * M_PI);
            }

            const double pos_deg = center + amp * std::sin(state.phase_accum_rad);
            const double vel_deg_s = amp * 2.0 * M_PI * freq * std::cos(state.phase_accum_rad);

            MotionCommandPoint sp_cmd{};
            sp_cmd.type = MotionCommandType::Stream;
            sp_cmd.axis_id = axis_id;
            sp_cmd.source = ControlOwner::UI;
            sp_cmd.value = pos_deg;
            sp_cmd.velocity = vel_deg_s;
            sp_cmd.acceleration = 0.0;
            sp_cmd.sample_period_sec = sp;
            sp_cmd.timestamp_us = 0;

            auto res = orchestrator_.enqueue_motion(axis_id, sp_cmd);
            if (!res.ok()) {
                // Queue full, step back phase accum and break
                state.phase_accum_rad -= 2.0 * M_PI * freq * sp;
                if (state.phase_accum_rad < 0.0) {
                    state.phase_accum_rad += 2.0 * M_PI;
                }
                break;
            }
        }
    }
}

} // namespace motion_core
