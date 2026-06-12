#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/axis_interface.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/result.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace motion_core {

// ---------------------------------------------------------------------------
// HomingSequenceProgress — status payload for multi-axis homing callbacks
// ---------------------------------------------------------------------------
struct HomingSequenceProgress {
    bool running{false};
    std::string status_text;
    int total_axes{0};
    int current_index{-1};
    int current_axis_id{-1};
};

// ---------------------------------------------------------------------------
// MotionOrchestrator
//
// Pure C++ module that owns:
//   - Motion command routing to axes
//   - Service command routing to axes
//   - E-stop state management
//   - Control ownership (UI vs HexaMotion)
//   - Multi-axis MKS homing sequence orchestration
//
// This class has NO Qt dependency. UI integration is via callbacks.
// ---------------------------------------------------------------------------
class MotionOrchestrator {
public:
    using LogCallback = std::function<void(const std::string& tag, const std::string& message)>;
    using HomingProgressCallback = std::function<void(const HomingSequenceProgress& progress)>;

    explicit MotionOrchestrator(HalRuntime& runtime);
    ~MotionOrchestrator();

    MotionOrchestrator(const MotionOrchestrator&) = delete;
    MotionOrchestrator& operator=(const MotionOrchestrator&) = delete;

    // -----------------------------------------------------------------------
    // Motion dispatch
    // -----------------------------------------------------------------------
    [[nodiscard]] Result<void> enqueue_motion(std::uint16_t axis_id,
                                               const MotionCommandPoint& point);
    [[nodiscard]] Result<void> enqueue_service(std::uint16_t axis_id,
                                                const ServiceCommandPoint& cmd);

    // -----------------------------------------------------------------------
    // E-stop
    // -----------------------------------------------------------------------
    [[nodiscard]] Result<void> emergency_stop_all();
    [[nodiscard]] Result<void> emergency_stop(std::uint16_t axis_id);
    [[nodiscard]] bool estop_active() const;
    void clear_estop();

    // -----------------------------------------------------------------------
    // Control ownership
    // -----------------------------------------------------------------------
    void set_control_owner(ControlOwner owner);
    [[nodiscard]] ControlOwner control_owner() const;

    // -----------------------------------------------------------------------
    // Safety baseline
    // -----------------------------------------------------------------------
    void apply_safety_baseline(std::uint16_t axis_id, bool force_disable = true);

    // -----------------------------------------------------------------------
    // Multi-axis MKS homing sequence
    // -----------------------------------------------------------------------
    [[nodiscard]] Result<void> start_homing_sequence(const std::vector<std::uint16_t>& axis_ids);
    void stop_homing_sequence();
    [[nodiscard]] bool homing_sequence_running() const;

    /// Must be called periodically (~100ms) to drive the homing state machine.
    void tick_homing_sequence();

    // -----------------------------------------------------------------------
    // Callbacks
    // -----------------------------------------------------------------------
    void set_log_callback(LogCallback cb);
    void set_homing_progress_callback(HomingProgressCallback cb);

private:
    // Homing internal helpers
    void homing_start_current_axis();
    void homing_emit_progress(const std::string& status_text);

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    HalRuntime& runtime_;

    mutable std::recursive_mutex control_mutex_;
    std::atomic<ControlOwner> active_source_{ControlOwner::UI};
    std::atomic<bool> estop_active_{false};

    // Multi-axis homing sequence state
    static constexpr int kHomingSequencePauseMs = 1000;
    static constexpr int kHomingSequenceAxisTimeoutMs = 120000;

    std::vector<std::uint16_t> homing_axis_ids_;
    int homing_index_{-1};
    enum class HomingPhase { Idle, WaitingAxisComplete, WaitingAxisIdle, WaitingPause };
    HomingPhase homing_phase_{HomingPhase::Idle};
    std::chrono::steady_clock::time_point homing_phase_started_{};
    bool homing_running_{false};

    // Callbacks
    LogCallback log_callback_;
    HomingProgressCallback homing_progress_callback_;

    void log(const std::string& tag, const std::string& message) const;
};

} // namespace motion_core
