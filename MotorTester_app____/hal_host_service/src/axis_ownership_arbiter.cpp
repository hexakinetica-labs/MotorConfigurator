#include "hal_host_service/axis_ownership_arbiter.h"

namespace hal_host_service {

namespace {

[[nodiscard]] bool is_takeover_request(const hal_ipc::ControlOp op) {
    return op == hal_ipc::ControlOp::RequestManualLease;
}

[[nodiscard]] bool is_release_request(const hal_ipc::ControlOp op) {
    return op == hal_ipc::ControlOp::ReleaseManualLease;
}

} // namespace

motion_core::Result<void> AxisOwnershipArbiter::handle_lease_op(const hal_ipc::OwnerRole caller,
                                                                 const hal_ipc::ControlOp op) {
    std::scoped_lock lock(mutex_);

    if (is_takeover_request(op)) {
        if (state_.motion_owner != caller) {
            state_.motion_owner = caller;
            ++state_.motion_epoch;
        }
        state_.manual_override_active = (state_.motion_owner == hal_ipc::OwnerRole::MotorTesterUi);
        state_.service_mode_active = (state_.service_owner != hal_ipc::OwnerRole::None);
        return motion_core::Result<void>::success();
    }

    if (is_release_request(op)) {
        if (state_.motion_owner == caller) {
            state_.motion_owner = hal_ipc::OwnerRole::None;
            ++state_.motion_epoch;
        }
        state_.manual_override_active = (state_.motion_owner == hal_ipc::OwnerRole::MotorTesterUi);
        state_.service_mode_active = (state_.service_owner != hal_ipc::OwnerRole::None);
        return motion_core::Result<void>::success();
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> AxisOwnershipArbiter::authorize_operation(const hal_ipc::OwnerRole caller,
                                                                     const hal_ipc::ControlOp op,
                                                                     const std::uint32_t expected_epoch) {
    std::scoped_lock lock(mutex_);

    if (op == hal_ipc::ControlOp::None || op == hal_ipc::ControlOp::Stop
        || is_takeover_request(op) || is_release_request(op)
        || is_readonly_query(op)) {
        return motion_core::Result<void>::success();
    }

    if (is_motion_command(op)) {
        if (state_.motion_owner == hal_ipc::OwnerRole::None) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::PermissionDenied, "motion lease is not acquired"});
        }
        if (state_.motion_owner != caller) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "motion owner is held by another client"});
        }
        if (expected_epoch != 0 && expected_epoch != state_.motion_epoch) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::PermissionDenied, "motion epoch mismatch"});
        }
    }

    if (is_service_command(op)) {
        if (state_.service_owner == hal_ipc::OwnerRole::None
            && caller == hal_ipc::OwnerRole::MotorTesterUi) {
            state_.service_owner = caller;
            ++state_.service_epoch;
        }
        if (state_.service_owner == hal_ipc::OwnerRole::None) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::PermissionDenied, "service owner is not assigned"});
        }
        if (state_.service_owner != caller) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "service owner is held by another client"});
        }
        if (expected_epoch != 0 && expected_epoch != state_.service_epoch) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::PermissionDenied, "service epoch mismatch"});
        }
    }

    state_.manual_override_active = (state_.motion_owner == hal_ipc::OwnerRole::MotorTesterUi);
    state_.service_mode_active = (state_.service_owner != hal_ipc::OwnerRole::None);
    return motion_core::Result<void>::success();
}

OwnershipSnapshot AxisOwnershipArbiter::snapshot() const {
    std::scoped_lock lock(mutex_);
    return state_;
}

void AxisOwnershipArbiter::reset() {
    std::scoped_lock lock(mutex_);
    state_ = {};
}

bool AxisOwnershipArbiter::is_motion_command(const hal_ipc::ControlOp op) {
    return op == hal_ipc::ControlOp::StreamPoint 
        || op == hal_ipc::ControlOp::Hold
        || op == hal_ipc::ControlOp::EnqueueMotionBatch;
}

bool AxisOwnershipArbiter::is_service_command(const hal_ipc::ControlOp op) {
    return op == hal_ipc::ControlOp::EnableAxis
        || op == hal_ipc::ControlOp::DisableAxis
        || op == hal_ipc::ControlOp::SetZero
        || op == hal_ipc::ControlOp::ClearFault
        || op == hal_ipc::ControlOp::Home
        || op == hal_ipc::ControlOp::SetAxisMode
        || op == hal_ipc::ControlOp::ConfigureMotionQueue
        || op == hal_ipc::ControlOp::ClearMotionQueue
        || op == hal_ipc::ControlOp::ApplyParameterPatch
        || op == hal_ipc::ControlOp::SetPersistent
        || op == hal_ipc::ControlOp::ImportAxisConfig
        || op == hal_ipc::ControlOp::ExportAxisConfig;
}

bool AxisOwnershipArbiter::is_readonly_query(const hal_ipc::ControlOp op) {
    return op == hal_ipc::ControlOp::QueryMotionQueueStats
        || op == hal_ipc::ControlOp::ReadParameters
        || op == hal_ipc::ControlOp::ListParameters
        || op == hal_ipc::ControlOp::ImportAxisConfigPreview;
}

} // namespace hal_host_service
