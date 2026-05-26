#include "mks_can/axis/mks_homing_state_machine.h"

#include <cmath>

namespace mks {

namespace {

constexpr auto kWaitHomingStatusActiveTimeout = std::chrono::seconds(5);
constexpr auto kWaitHomingDoneTimeout = std::chrono::seconds(90);
constexpr auto kWaitMoveToOffsetTimeout = std::chrono::seconds(30);
constexpr double kZeroReachedToleranceDeg = 0.5;
constexpr double kOffsetReachedToleranceDeg = 0.5;
constexpr double kVelocitySettledToleranceDegPerSec = 1.0;

[[nodiscard]] bool is_position_near(const double actual_deg,
                                    const double target_deg,
                                    const double tolerance_deg) {
    return std::isfinite(actual_deg)
        && std::isfinite(target_deg)
        && std::abs(actual_deg - target_deg) <= tolerance_deg;
}

[[nodiscard]] bool is_velocity_settled(const double velocity_deg_per_sec) {
    return std::isfinite(velocity_deg_per_sec)
        && std::abs(velocity_deg_per_sec) <= kVelocitySettledToleranceDegPerSec;
}

} // namespace

void MksHomingStateMachine::request_start() noexcept {
    start_requested_ = true;
}

void MksHomingStateMachine::request_cancel() noexcept {
    cancel_requested_ = true;
}

void MksHomingStateMachine::reset() noexcept {
    phase_ = Phase::Idle;
    start_requested_ = false;
    cancel_requested_ = false;
    seen_homing_status_active_ = false;
    phase_deadline_ = {};
}

bool MksHomingStateMachine::active() const noexcept {
    return phase_ != Phase::Idle;
}

MksHomingStateMachine::Phase MksHomingStateMachine::current_phase() const noexcept {
    return phase_;
}

MksHomingOutputs MksHomingStateMachine::step(const MksHomingInputs& in,
                                            const std::chrono::steady_clock::time_point now) noexcept {
    MksHomingOutputs out{};

    if (cancel_requested_) {
        reset();
        cancel_requested_ = false;
        return out;
    }

    if (start_requested_ && phase_ == Phase::Idle) {
        start_requested_ = false;
        seen_homing_status_active_ = false;
        phase_ = Phase::StartHardwareHome;
    } else {
        start_requested_ = false;
    }

    if (phase_ != Phase::Idle && in.faulted) {
        out.failed = true;
        reset();
        return out;
    }

    switch (phase_) {
        case Phase::Idle:
            return out;

        case Phase::StartHardwareHome:
            out.active = true;
            out.request_start_hardware_home = true;
            phase_ = Phase::WaitHomingStatusActive;
            phase_deadline_ = now + kWaitHomingStatusActiveTimeout;
            return out;

        case Phase::WaitHomingStatusActive:
            out.active = true;
            if (in.home_status_active) {
                seen_homing_status_active_ = true;
                phase_ = Phase::WaitHomingStatusInactiveAtZero;
                phase_deadline_ = now + kWaitHomingDoneTimeout;
                return out;
            }
            if (now >= phase_deadline_) {
                out.failed = true;
                reset();
            }
            return out;

        case Phase::WaitHomingStatusInactiveAtZero:
            out.active = true;
            if (in.home_status_active) {
                seen_homing_status_active_ = true;
                return out;
            }
            if (seen_homing_status_active_
                && is_position_near(in.position_deg, 0.0, kZeroReachedToleranceDeg)
                && is_velocity_settled(in.velocity_deg_per_sec)) {
                phase_ = Phase::CommandMoveToOffset;
                return out;
            }
            if (now >= phase_deadline_) {
                out.failed = true;
                reset();
            }
            return out;

        case Phase::CommandMoveToOffset:
            out.active = true;
            out.request_move_to_offset = true;
            out.move_to_offset_deg = in.homing_offset_deg;
            phase_ = Phase::WaitMoveToOffset;
            phase_deadline_ = now + kWaitMoveToOffsetTimeout;
            return out;

        case Phase::WaitMoveToOffset:
            out.active = true;
            if (is_position_near(in.position_deg, in.homing_offset_deg, kOffsetReachedToleranceDeg)
                && is_velocity_settled(in.velocity_deg_per_sec)) {
                out.request_set_zero = true;
                reset();
                return out;
            }
            if (now >= phase_deadline_) {
                out.failed = true;
                reset();
            }
            return out;
    }

    return out;
}

} // namespace mks