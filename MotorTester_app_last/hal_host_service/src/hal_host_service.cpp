#include "hal_host_service/hal_host_service.h"

namespace hal_host_service {

namespace {

[[nodiscard]] motion_core::Result<hal_ipc::OwnerRole> owner_from_client_id(const std::uint32_t client_id) {
    if (client_id == hal_ipc::kClientIdHexaMotion) {
        return motion_core::Result<hal_ipc::OwnerRole>::success(hal_ipc::OwnerRole::HexaMotion);
    }
    if (client_id == hal_ipc::kClientIdMotorTesterUi) {
        return motion_core::Result<hal_ipc::OwnerRole>::success(hal_ipc::OwnerRole::MotorTesterUi);
    }
    return motion_core::Result<hal_ipc::OwnerRole>::failure(
        {motion_core::ErrorCode::PermissionDenied, "unknown client_id"});
}

} // namespace

HalHostService::~HalHostService() {
    (void)stop();
}

motion_core::Result<void> HalHostService::start(const HalHostServiceConfig& config) {
    std::scoped_lock lock(mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return motion_core::Result<void>::success();
    }

    const auto server_res = server_.start(
        config.bind_host,
        config.port,
        [this](const hal_ipc::HalControlFrameDto& frame) {
            return this->handle_control_frame(frame);
        },
        []() {});
    if (!server_res.ok()) {
        return server_res;
    }

    state_seq_ = 0;
    running_.store(true, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> HalHostService::stop() {
    std::scoped_lock lock(mutex_);

    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    if (!was_running) {
        return motion_core::Result<void>::success();
    }

    (void)server_.stop();
    return motion_core::Result<void>::success();
}

bool HalHostService::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

void HalHostService::set_axis_operation_handler(AxisOperationHandler handler) {
    std::scoped_lock lock(mutex_);
    axis_operation_handler_ = std::make_shared<AxisOperationHandler>(std::move(handler));
}

void HalHostService::set_state_provider(std::function<HostStateSnapshot()> provider) {
    std::scoped_lock lock(mutex_);
    state_provider_ = std::make_shared<std::function<HostStateSnapshot()>>(std::move(provider));
}

hal_ipc::HalStateFrameDto HalHostService::handle_control_frame(const hal_ipc::HalControlFrameDto& frame) {
    hal_ipc::HalStateFrameDto state{};
    state.seq = state_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
    state.ack_control_seq = frame.seq;

    const bool allow_local_inproc = (frame.client_id == hal_ipc::kClientIdMotorTesterUi);
    if (!running_.load(std::memory_order_acquire) && !allow_local_inproc) {
        state.service_status_code = static_cast<std::int32_t>(motion_core::ErrorCode::NotConnected);
        return state;
    }

    const auto apply_res = apply_control_frame(frame);
    if (!apply_res.ok()) {
        state.service_status_code = static_cast<std::int32_t>(apply_res.error().code);
    } else {
        state.service_string_response = apply_res.value();
    }

    const HostStateSnapshot snapshot = state_snapshot();
    state.motion_owner = snapshot.motion_owner;
    state.manual_override_active = snapshot.manual_override_active;
    state.estop_active = snapshot.estop_active;
    refresh_state_from_runtime(state);
    return state;
}

HalHostService::HostStateSnapshot HalHostService::state_snapshot() const {
    std::shared_ptr<std::function<HostStateSnapshot()>> state_provider;
    int connected_client_count = 0;
    int connected_hexamotion_client_count = 0;
    bool ipc_server_running = false;
    {
        std::scoped_lock lock(mutex_);
        state_provider = state_provider_;
        connected_client_count = server_.connected_client_count();
        connected_hexamotion_client_count = server_.connected_hexamotion_client_count();
        ipc_server_running = running_.load(std::memory_order_acquire);
    }

    HostStateSnapshot snapshot{};
    if (state_provider && *state_provider) {
        snapshot = (*state_provider)();
    }

    snapshot.connected_client_count = connected_client_count;
    snapshot.connected_hexamotion_client_count = connected_hexamotion_client_count;
    snapshot.ipc_server_running = ipc_server_running;
    return snapshot;
}

motion_core::Result<std::string> HalHostService::apply_control_frame(const hal_ipc::HalControlFrameDto& frame) {
    if (frame.op == hal_ipc::ControlOp::None && frame.client_id == 0U) {
        return motion_core::Result<std::string>::success("");
    }

    const auto caller_res = owner_from_client_id(frame.client_id);
    if (!caller_res.ok()) {
        return motion_core::Result<std::string>::failure(caller_res.error());
    }

    if (!frame.axes.empty()) {
        std::string last_response;
        for (const auto& axis : frame.axes) {
            const auto op_res = apply_axis_op(caller_res.value(), frame.op, axis.axis_id, &axis, frame);
            if (!op_res.ok()) {
                return op_res;
            }
            last_response = op_res.value();
        }
        return motion_core::Result<std::string>::success(last_response);
    }

    if (frame.op == hal_ipc::ControlOp::Stop && frame.service_axis_id <= 0) {
        return apply_axis_op(caller_res.value(), frame.op, 0U, nullptr, frame);
    }

    if (frame.service_axis_id > 0) {
        return apply_axis_op(caller_res.value(),
                             frame.op,
                             static_cast<std::uint16_t>(frame.service_axis_id),
                             nullptr,
                             frame);
    }

    return motion_core::Result<std::string>::success("");
}

motion_core::Result<std::string> HalHostService::apply_axis_op(const hal_ipc::OwnerRole caller,
                                                               const hal_ipc::ControlOp op,
                                                               const std::uint16_t axis_id,
                                                               const hal_ipc::AxisPointDto* point,
                                                               const hal_ipc::HalControlFrameDto& frame) {
    std::shared_ptr<AxisOperationHandler> external_handler{};
    {
        std::scoped_lock lock(mutex_);
        external_handler = axis_operation_handler_;
    }
    if (external_handler && *external_handler) {
        return (*external_handler)(caller, op, axis_id, point, frame);
    }

    return motion_core::Result<std::string>::failure(
        {motion_core::ErrorCode::InternalError, "axis operation handler is not configured"});
}

void HalHostService::refresh_state_from_runtime(hal_ipc::HalStateFrameDto& state) const {
    const auto axes_res = runtime_.list_axes();
    if (!axes_res.ok()) {
        if (state.service_status_code == 0) {
            state.service_status_code = static_cast<std::int32_t>(axes_res.error().code);
        }
        return;
    }

    state.axes.clear();
    state.axes.reserve(axes_res.value().size());
    for (const auto& axis_info : axes_res.value()) {
        const auto axis_res = runtime_.find_axis(axis_info.id.value);
        if (!axis_res.ok()) {
            if (state.service_status_code == 0) {
                state.service_status_code = static_cast<std::int32_t>(axis_res.error().code);
            }
            continue;
        }

        const auto telemetry_res = axis_res.value()->read_telemetry();
        if (!telemetry_res.ok()) {
            if (state.service_status_code == 0) {
                state.service_status_code = static_cast<std::int32_t>(telemetry_res.error().code);
            }
            continue;
        }

        hal_ipc::AxisPointDto dto{};
        dto.axis_id = axis_info.id.value;
        dto.actual_position_deg = telemetry_res.value().actual_position_deg;
        dto.actual_velocity_deg_s = telemetry_res.value().actual_velocity_deg_per_sec;
        dto.interpolated_position_deg = telemetry_res.value().target_position_deg;
        dto.has_interpolated_position = true;
        state.axes.push_back(dto);
    }
}

} // namespace hal_host_service
