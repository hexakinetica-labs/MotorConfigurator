#include "motion_core/motion_orchestrator.h"

#include "mks_can/mks_axis_adapter.h"
#include <chrono>

#include <algorithm>
#include <cstdio>

namespace motion_core {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MotionOrchestrator::MotionOrchestrator(HalRuntime& runtime)
    : runtime_(runtime) {}

MotionOrchestrator::~MotionOrchestrator() {
    stop_homing_sequence();
}

// ---------------------------------------------------------------------------
// Logging helper
// ---------------------------------------------------------------------------

void MotionOrchestrator::log(const std::string& tag, const std::string& message) const {
    if (log_callback_) {
        log_callback_(tag, message);
    }
}

void MotionOrchestrator::set_log_callback(LogCallback cb) {
    log_callback_ = std::move(cb);
}

void MotionOrchestrator::set_homing_progress_callback(HomingProgressCallback cb) {
    homing_progress_callback_ = std::move(cb);
}

// ---------------------------------------------------------------------------
// Motion dispatch
// ---------------------------------------------------------------------------

Result<void> MotionOrchestrator::enqueue_motion(const std::uint16_t axis_id,
                                                 const MotionCommandPoint& point) {
    {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        if (estop_active_.load(std::memory_order_acquire)) {
            return Result<void>::failure(
                {ErrorCode::Busy, "Cannot enqueue motion: global E-Stop is active"});
        }
    }

    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) return Result<void>::failure(axis_res.error());

    auto pt = point;
    pt.source = active_source_.load(std::memory_order_acquire);

    if (!axis_res.value()->enqueueCommandPoint(pt)) {
        return Result<void>::failure(
            {ErrorCode::PermissionDenied,
             "Enqueue rejected by adapter (active source mismatch or estop)"});
    }
    return Result<void>::success();
}

Result<void> MotionOrchestrator::enqueue_service(const std::uint16_t axis_id,
                                                  const ServiceCommandPoint& cmd) {
    {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        if (estop_active_.load(std::memory_order_acquire)) {
            // Block unsafe service commands during E-STOP
            if (cmd.type != ServiceCommandType::Disable &&
                cmd.type != ServiceCommandType::ClearMotionQueue &&
                cmd.type != ServiceCommandType::ClearErrors) {
                return Result<void>::failure(
                    {ErrorCode::Busy, "Cannot enqueue service command: global E-Stop is active"});
            }
        }
    }

    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) return Result<void>::failure(axis_res.error());

    auto p = cmd;
    p.source = active_source_.load(std::memory_order_acquire);

    if (!axis_res.value()->enqueueServicePoint(p)) {
        return Result<void>::failure(
            {ErrorCode::PermissionDenied,
             "Enqueue rejected by adapter (active source mismatch or estop)"});
    }
    return Result<void>::success();
}

// ---------------------------------------------------------------------------
// E-stop
// ---------------------------------------------------------------------------

Result<void> MotionOrchestrator::emergency_stop_all() {
    const auto listed = runtime_.list_axes();
    if (!listed.ok())
        return Result<void>::failure(listed.error());

    for (const auto& info : listed.value()) {
        const auto axis_res = runtime_.find_axis(info.id.value);
        if (!axis_res.ok()) continue;
        axis_res.value()->estop();
    }

    estop_active_.store(true, std::memory_order_release);
    log("hal", "Global E-STOP activated — all axes stopped");
    return Result<void>::success();
}

Result<void> MotionOrchestrator::emergency_stop(const std::uint16_t axis_id) {
    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) return Result<void>::failure(axis_res.error());
    axis_res.value()->estop();
    return Result<void>::success();
}

bool MotionOrchestrator::estop_active() const {
    return estop_active_.load(std::memory_order_acquire);
}

void MotionOrchestrator::clear_estop() {
    estop_active_.store(false, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Control ownership
// ---------------------------------------------------------------------------

void MotionOrchestrator::set_control_owner(const ControlOwner owner) {
    const auto prev = active_source_.exchange(owner, std::memory_order_acq_rel);
    if (prev == owner) return;

    // Broadcast to all axes.
    const auto listed = runtime_.list_axes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            auto axis_res = runtime_.find_axis(info.id.value);
            if (axis_res.ok()) {
                axis_res.value()->setControlOwner(owner);
            }
        }
    }
}

ControlOwner MotionOrchestrator::control_owner() const {
    return active_source_.load(std::memory_order_acquire);
}

// ---------------------------------------------------------------------------
// Safety baseline
// ---------------------------------------------------------------------------

void MotionOrchestrator::apply_safety_baseline(const std::uint16_t axis_id, const bool force_disable) {
    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) return;
    const auto& axis = axis_res.value();

    if (force_disable) {
        ServiceCommandPoint cmd{};
        cmd.type = ServiceCommandType::Disable;
        cmd.axis_id = static_cast<int>(axis_id);
        if (!axis->enqueueServicePoint(cmd)) {
            log("hal", "Axis " + std::to_string(axis_id) + " disable in safety baseline failed");
        }
    }

    ServiceCommandPoint set_zero{};
    set_zero.type = ServiceCommandType::SetZero;
    set_zero.axis_id = static_cast<int>(axis_id);
    if (!axis->enqueueServicePoint(set_zero)) {
        log("hal", "Axis " + std::to_string(axis_id) + " set-zero in safety baseline failed");
    }
}

// ---------------------------------------------------------------------------
// Multi-axis MKS homing sequence
// ---------------------------------------------------------------------------

Result<void> MotionOrchestrator::start_homing_sequence(
    const std::vector<std::uint16_t>& axis_ids) {
    std::lock_guard<std::recursive_mutex> lock(control_mutex_);
    if (homing_running_ || axis_ids.empty()) {
        return Result<void>::failure(
            {ErrorCode::Busy,
             homing_running_ ? "homing sequence already running" : "no axes provided"});
    }

    if (estop_active_.load(std::memory_order_acquire)) {
        return Result<void>::failure(
            {ErrorCode::Busy, "cannot start homing: E-STOP active"});
    }

    // Normalize and deduplicate, sorted ascending.
    homing_axis_ids_ = axis_ids;
    std::sort(homing_axis_ids_.begin(), homing_axis_ids_.end());
    homing_axis_ids_.erase(
        std::unique(homing_axis_ids_.begin(), homing_axis_ids_.end()),
        homing_axis_ids_.end());

    homing_running_ = true;
    homing_index_ = 0;
    homing_phase_ = HomingPhase::WaitingAxisComplete;
    homing_phase_started_ = {};

    log("mks", "MKS homing sequence started: "
        + std::to_string(homing_axis_ids_.size()) + " axis(es)");

    homing_start_current_axis();
    homing_emit_progress("Sequence started");

    return Result<void>::success();
}

void MotionOrchestrator::stop_homing_sequence() {
    std::lock_guard<std::recursive_mutex> lock(control_mutex_);
    if (!homing_running_) {
        homing_emit_progress("Idle");
        return;
    }

    // Abort current axis: clear motion queue + disable.
    if (homing_index_ >= 0
        && homing_index_ < static_cast<int>(homing_axis_ids_.size())) {
        const auto active_axis_id = homing_axis_ids_[static_cast<std::size_t>(homing_index_)];
        const auto axis_res = runtime_.find_axis(active_axis_id);
        if (axis_res.ok()) {
            ServiceCommandPoint clear_q{};
            clear_q.type = ServiceCommandType::ClearMotionQueue;
            clear_q.axis_id = static_cast<int>(active_axis_id);
            (void)axis_res.value()->enqueueServicePoint(clear_q);

            ServiceCommandPoint disable{};
            disable.type = ServiceCommandType::Disable;
            disable.axis_id = static_cast<int>(active_axis_id);
            (void)axis_res.value()->enqueueServicePoint(disable);
        }
    }

    homing_running_ = false;
    homing_index_ = -1;
    homing_phase_ = HomingPhase::Idle;
    homing_phase_started_ = {};

    log("mks", "MKS homing sequence stopped");
    homing_emit_progress("Sequence stopped");
}

bool MotionOrchestrator::homing_sequence_running() const {
    std::lock_guard<std::recursive_mutex> lock(control_mutex_);
    return homing_running_;
}

void MotionOrchestrator::homing_start_current_axis() {
    if (!homing_running_
        || homing_index_ < 0
        || homing_index_ >= static_cast<int>(homing_axis_ids_.size())) {
        return;
    }

    homing_phase_ = HomingPhase::WaitingAxisComplete;
    homing_phase_started_ = std::chrono::steady_clock::now();
    const auto axis_id = homing_axis_ids_[static_cast<std::size_t>(homing_index_)];

    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) {
        log("mks", "MKS homing sequence: axis "
            + std::to_string(axis_id) + " not found, stopping");
        stop_homing_sequence();
        return;
    }

    // Send Home service command.
    ServiceCommandPoint home{};
    home.type = ServiceCommandType::Home;
    home.axis_id = static_cast<int>(axis_id);
    if (!axis_res.value()->enqueueServicePoint(home)) {
        log("mks", "MKS homing sequence: failed to enqueue Home on axis "
            + std::to_string(axis_id));
        stop_homing_sequence();
        return;
    }

    log("mks", "MKS homing sequence: starting homing on axis " + std::to_string(axis_id));
    homing_emit_progress("Homing axis " + std::to_string(axis_id) + "...");
}

void MotionOrchestrator::tick_homing_sequence() {
    std::lock_guard<std::recursive_mutex> lock(control_mutex_);
    if (!homing_running_
        || homing_index_ < 0
        || homing_index_ >= static_cast<int>(homing_axis_ids_.size())) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto axis_id = homing_axis_ids_[static_cast<std::size_t>(homing_index_)];

    // Timeout check.
    if (homing_phase_ == HomingPhase::WaitingAxisComplete
        || homing_phase_ == HomingPhase::WaitingAxisIdle) {
        if (homing_phase_started_ != std::chrono::steady_clock::time_point{}
            && std::chrono::duration_cast<std::chrono::milliseconds>(now - homing_phase_started_).count()
                   > kHomingSequenceAxisTimeoutMs) {
            log("mks", "MKS homing sequence: axis " + std::to_string(axis_id) + " timed out");
            stop_homing_sequence();
            return;
        }
    }

    // Read telemetry for the current axis.
    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) {
        stop_homing_sequence();
        return;
    }

    // Check for E-STOP.
    if (estop_active_.load(std::memory_order_acquire)) {
        log("mks", "MKS homing sequence: E-STOP detected, stopping");
        stop_homing_sequence();
        return;
    }

    // Get homing status from the MKS adapter (transport-specific).
    std::string homing_status_text = "Idle";
    int motion_status_code = 0;
    auto* mks_adapter = dynamic_cast<mks::MksAxisAdapter*>(axis_res.value().get());
    if (mks_adapter) {
        homing_status_text = std::string(mks_adapter->homing_status_text());
        const auto telem = mks_adapter->telemetry();
        motion_status_code = static_cast<int>(telem.motion_status_code);
    } else {
        const auto telem = axis_res.value()->telemetry();
        motion_status_code = static_cast<int>(telem.motion_status_code);
    }

    // Check for failure.
    if (homing_status_text == "Failed") {
        log("mks", "MKS homing sequence: axis " + std::to_string(axis_id) + " homing failed");
        stop_homing_sequence();
        return;
    }

    switch (homing_phase_) {
        case HomingPhase::Idle:
            return;

        case HomingPhase::WaitingAxisComplete:
            if (homing_status_text == "Completed") {
                homing_phase_ = HomingPhase::WaitingAxisIdle;
                homing_phase_started_ = now;
                homing_emit_progress("Axis " + std::to_string(axis_id)
                                     + " homing completed, waiting for motor to settle...");
            }
            return;

        case HomingPhase::WaitingAxisIdle: {
            const bool homing_is_idle =
                (homing_status_text == "Idle" || homing_status_text == "Completed");
            const bool motor_is_stopped = (motion_status_code == 1);
            if (homing_is_idle && motor_is_stopped) {
                homing_phase_ = HomingPhase::WaitingPause;
                homing_phase_started_ = now;
                homing_emit_progress("Axis " + std::to_string(axis_id)
                                     + " settled. Pausing before next axis...");
            }
            return;
        }

        case HomingPhase::WaitingPause: {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - homing_phase_started_).count();
            if (elapsed_ms < kHomingSequencePauseMs) {
                return;
            }
            // Advance to next axis.
            ++homing_index_;
            if (homing_index_ >= static_cast<int>(homing_axis_ids_.size())) {
                homing_running_ = false;
                homing_index_ = -1;
                homing_phase_ = HomingPhase::Idle;
                homing_phase_started_ = {};
                log("mks", "MKS homing sequence completed successfully");
                homing_emit_progress("Sequence completed");
                return;
            }
            homing_start_current_axis();
            return;
        }
    }
}

void MotionOrchestrator::homing_emit_progress(const std::string& status_text) {
    if (!homing_progress_callback_) return;

    HomingSequenceProgress progress{};
    progress.running = homing_running_;
    progress.status_text = status_text;
    progress.total_axes = static_cast<int>(homing_axis_ids_.size());
    progress.current_index = homing_running_ ? homing_index_ : -1;
    if (homing_running_
        && homing_index_ >= 0
        && homing_index_ < static_cast<int>(homing_axis_ids_.size())) {
        progress.current_axis_id = static_cast<int>(
            homing_axis_ids_[static_cast<std::size_t>(homing_index_)]);
    }
    homing_progress_callback_(progress);
}

} // namespace motion_core
