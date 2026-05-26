#include "ethercat/manual_homing/ethercat_manual_homing_state_machine.h"

#include <cmath>

namespace ethercat_driver {

namespace {

constexpr double kDefaultCycleDtSec = 0.004;
constexpr double kOffsetReachedToleranceDeg = 0.2;
constexpr double kSwitchPauseSec = 1.0;

} // namespace

void EthercatManualHomingStateMachine::request_start() noexcept {
    start_requested_ = true;
}

void EthercatManualHomingStateMachine::request_cancel() noexcept {
    cancel_requested_ = true;
}

void EthercatManualHomingStateMachine::reset() noexcept {
    phase_ = Phase::Idle;
    start_requested_ = false;
    cancel_requested_ = false;
    search_target_deg_ = 0.0;
    switch_position_deg_ = 0.0;
    offset_target_deg_ = 0.0;
    pause_remaining_sec_ = 0.0;
    offset_pp_setpoint_issued_ = false;
}

bool EthercatManualHomingStateMachine::active() const noexcept {
    return phase_ != Phase::Idle;
}

EthercatManualHomingOutputs EthercatManualHomingStateMachine::step(const EthercatManualHomingInputs& in) noexcept {
    EthercatManualHomingOutputs out{};
    const bool di3_now = (in.digital_inputs & kDi3Mask) != 0U;
    const double search_velocity_deg_s =
        (std::isfinite(in.search_velocity_deg_per_sec) && in.search_velocity_deg_per_sec > 0.0)
            ? in.search_velocity_deg_per_sec
            : kDefaultSearchVelocityDegPerSec;
    const double dt_sec = (std::isfinite(in.cycle_dt_sec) && in.cycle_dt_sec > 0.0)
        ? in.cycle_dt_sec
        : kDefaultCycleDtSec;

    if (cancel_requested_) {
        phase_ = Phase::Idle;
        cancel_requested_ = false;
        start_requested_ = false;
        offset_pp_setpoint_issued_ = false;
    }

    if (start_requested_) {
        start_requested_ = false;
        if (in.enabled && !in.faulted) {
            search_target_deg_ = in.position_deg;
            switch_position_deg_ = in.position_deg;
            offset_target_deg_ = in.position_deg + in.home_offset_deg;
            offset_pp_setpoint_issued_ = false;
            if (di3_now) {
                pause_remaining_sec_ = kSwitchPauseSec;
                phase_ = Phase::SwitchDetectedPause;
            } else {
                phase_ = Phase::SearchForSwitch;
            }
        } else {
            phase_ = Phase::Idle;
        }
    }

    if (phase_ != Phase::Idle && (!in.enabled || in.faulted)) {
        phase_ = Phase::Idle;
        offset_pp_setpoint_issued_ = false;
    }

    if (phase_ == Phase::SearchForSwitch) {
        if (di3_now) {
            switch_position_deg_ = in.position_deg;
            offset_target_deg_ = switch_position_deg_ + in.home_offset_deg;
            pause_remaining_sec_ = kSwitchPauseSec;
            phase_ = Phase::SwitchDetectedPause;
        } else {
            search_target_deg_ += search_velocity_deg_s * dt_sec;
            out.active = true;
            out.force_csp_mode = true;
            out.target_position_override = true;
            out.target_position_deg = search_target_deg_;
            return out;
        }
    }

    if (phase_ == Phase::SwitchDetectedPause) {
        pause_remaining_sec_ -= dt_sec;
        if (pause_remaining_sec_ <= 0.0) {
            pause_remaining_sec_ = 0.0;
            phase_ = Phase::MoveToOffset;
        }

        out.active = true;
        out.force_csp_mode = true;
        out.target_position_override = true;
        out.target_position_deg = switch_position_deg_;
        return out;
    }

    if (phase_ == Phase::MoveToOffset) {
        out.active = true;
        out.force_pp_mode = true;
        out.target_position_override = true;
        out.target_position_deg = offset_target_deg_;
        if (!offset_pp_setpoint_issued_) {
            out.request_pp_setpoint = true;
            offset_pp_setpoint_issued_ = true;
        }

        if (std::abs(offset_target_deg_ - in.position_deg) <= kOffsetReachedToleranceDeg) {
            out.apply_set_zero = true;
            out.active = false;
            phase_ = Phase::Idle;
            offset_pp_setpoint_issued_ = false;
        }
        return out;
    }

    out.active = false;
    return out;
}

} // namespace ethercat_driver
