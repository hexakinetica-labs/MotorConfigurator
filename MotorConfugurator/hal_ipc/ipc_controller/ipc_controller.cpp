#include "hal_ipc/ipc_controller/ipc_controller.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

using json = nlohmann::json;

namespace hal_ipc {

HalIpcController::HalIpcController(motion_core::HalRuntime& runtime, motion_core::MotionOrchestrator& orchestrator)
    : runtime_(runtime), orchestrator_(orchestrator) {
    // Handler is set in start()
}

HalIpcController::~HalIpcController() {
    stop();
}

motion_core::Result<void> HalIpcController::start(const std::string& bind_host, std::uint16_t port, 
                                                  HalIpcServer::DisconnectHandler on_disconnect) {
    return server_.start(bind_host, port, [this](const HalControlFrameDto& frame) {
        return handle_control_frame(frame);
    }, on_disconnect);
}

motion_core::Result<void> HalIpcController::stop() {
    return server_.stop();
}

bool HalIpcController::is_running() const {
    return server_.is_running();
}

int HalIpcController::connected_client_count() const {
    return server_.connected_client_count();
}

HalStateFrameDto HalIpcController::handle_control_frame(const HalControlFrameDto& frame) {
    HalStateFrameDto resp{};
    resp.ack_control_seq = frame.seq;

    OwnerRole motion_owner = OwnerRole::None;
    motion_core::ControlOwner control_source = orchestrator_.control_owner();
    motion_owner = (control_source == motion_core::ControlOwner::HexaMotion)
                       ? OwnerRole::HexaMotion
                       : OwnerRole::MotorTesterUi;
    bool manual_override = (control_source == motion_core::ControlOwner::UI);
    bool estop = orchestrator_.estop_active();

    resp.motion_owner = motion_owner;
    resp.manual_override_active = manual_override;
    resp.estop_active = estop;

    const OwnerRole caller = (frame.client_id == kClientIdHexaMotion)
                                 ? OwnerRole::HexaMotion
                                 : OwnerRole::MotorTesterUi;

    if (frame.op != ControlOp::None) {
        if (!frame.axes.empty()) {
            for (const auto& point : frame.axes) {
                auto res = execute_axis_operation(caller, frame.op, point.axis_id, &point, frame);
                if (!res.ok()) {
                    resp.service_status_code = static_cast<std::int32_t>(res.error().code);
                    resp.service_string_response = res.error().message;
                }
            }
        } else if (frame.service_axis_id >= 0) {
            auto res = execute_axis_operation(caller, frame.op, static_cast<std::uint16_t>(frame.service_axis_id), nullptr, frame);
            if (!res.ok()) {
                resp.service_status_code = static_cast<std::int32_t>(res.error().code);
                resp.service_string_response = res.error().message;
            }
        } else {
            auto res = execute_axis_operation(caller, frame.op, 0, nullptr, frame);
            if (!res.ok()) {
                resp.service_status_code = static_cast<std::int32_t>(res.error().code);
                resp.service_string_response = res.error().message;
            }
        }
    }

    const auto listed = runtime_.list_axes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            auto axis_res = runtime_.find_axis(info.id.value);
            if (!axis_res.ok()) continue;
            auto telem = axis_res.value()->telemetry();
            AxisPointDto pt{};
            pt.axis_id = info.id.value;
            pt.actual_position_deg = telem.position;
            pt.actual_velocity_deg_s = telem.velocity;
            resp.axes.push_back(pt);
        }
    }
    return resp;
}

motion_core::Result<std::string> HalIpcController::execute_axis_operation(
    OwnerRole caller,
    ControlOp op,
    std::uint16_t axis_id,
    const AxisPointDto* point,
    const HalControlFrameDto& frame) {

    const auto source = (caller == OwnerRole::HexaMotion)
                            ? motion_core::ControlOwner::HexaMotion
                            : motion_core::ControlOwner::UI;

    const auto submit_motion = [this, axis_id, source](const motion_core::MotionCommandPoint& pt)
        -> motion_core::Result<std::string> {
        auto p = pt;
        p.source = source;
        const auto res = orchestrator_.enqueue_motion(axis_id, p);
        if (!res.ok()) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::PermissionDenied, res.error().message});
        }
        return motion_core::Result<std::string>::success("");
    };

    const auto submit_service = [this, axis_id, source](const motion_core::ServiceCommandPoint& pt)
        -> motion_core::Result<std::string> {
        auto p = pt;
        p.source = source;
        const auto res = orchestrator_.enqueue_service(axis_id, p);
        if (!res.ok()) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::PermissionDenied, res.error().message});
        }
        return motion_core::Result<std::string>::success("");
    };

    if (op == ControlOp::Stop && axis_id == 0U) {
        auto res = orchestrator_.emergency_stop_all();
        if (!res.ok()) return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InternalError, res.error().message});
        return motion_core::Result<std::string>::success("");
    }

    if (op == ControlOp::StartMksHomingSequence) {
        std::vector<std::uint16_t> axis_ids;
        if (!frame.service_string_value.empty()) {
            try {
                auto doc = json::parse(frame.service_string_value);
                if (doc.is_array()) {
                    for (const auto& item : doc) {
                        if (item.is_number()) {
                            axis_ids.push_back(item.get<std::uint16_t>());
                        }
                    }
                }
            } catch (const std::exception& e) {
                // fall back or ignore
            }
        }
        if (axis_ids.empty()) {
            auto listed = runtime_.list_axes();
            if (listed.ok()) {
                for (const auto& info : listed.value()) {
                    if (info.transport == motion_core::AxisTransportKind::CanBus) {
                        axis_ids.push_back(info.id.value);
                    }
                }
            }
        }
        if (axis_ids.empty()) {
            return motion_core::Result<std::string>::failure(
                {motion_core::ErrorCode::InvalidArgument, "no MKS axes available for homing sequence"});
        }
        auto res = orchestrator_.start_homing_sequence(axis_ids);
        if (!res.ok()) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InternalError, res.error().message});
        }
        return motion_core::Result<std::string>::success("");
    }

    if (op == ControlOp::StopMksHomingSequence) {
        orchestrator_.stop_homing_sequence();
        return motion_core::Result<std::string>::success("");
    }

    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok())
        return motion_core::Result<std::string>::failure(axis_res.error());

    auto& axis = axis_res.value();

    switch (op) {
        case ControlOp::EnableAxis: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Enable;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::DisableAxis: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Disable;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::Stop: {
            axis->estop();
            return motion_core::Result<std::string>::success("");
        }

        case ControlOp::Hold: {
            const auto telem = axis->telemetry();
            motion_core::MotionCommandPoint point{};
            point.type = motion_core::MotionCommandType::Position;
            point.axis_id = static_cast<int>(axis_id);
            point.value = telem.position;
            point.velocity = 0.0;
            point.acceleration = 0.0;
            point.timestamp_us = telem.timestamp_us;
            point.sample_period_sec = 0.004;
            return submit_motion(point);
        }

        case ControlOp::SetZero: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::SetZero;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::ClearFault: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::ClearErrors;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::Home: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Home;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::StartManualHoming: {
            if (axis->info().transport != motion_core::AxisTransportKind::Ethercat) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::Unsupported, "manual DI3 homing is supported only for EtherCAT axes"});
            }
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Home;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case ControlOp::StreamPoint: {
            if (!point)
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "StreamPoint requires axis point"});
            
            const double target_pos = point->has_interpolated_position
                ? point->interpolated_position_deg
                : point->segment_target_deg;
            const double target_vel = point->has_interpolated_velocity ? point->interpolated_velocity_deg_s : 0.0;
            const double sample_period = (point->segment_duration_sec > 0.0) ? point->segment_duration_sec : 0.004;

            const auto sp = motion_core::build_motion_command_point(
                axis_id,
                motion_core::MotionCommandType::Stream,
                target_pos,
                target_vel,
                0.0,
                sample_period,
                false
            );
            return submit_motion(sp);
        }

        case ControlOp::EnqueueMotionBatch: {
            if (frame.service_string_value.empty())
                return motion_core::Result<std::string>::success("");
            
            try {
                auto doc = json::parse(frame.service_string_value);
                if (!doc.is_array()) {
                    return motion_core::Result<std::string>::failure(
                        {motion_core::ErrorCode::InvalidArgument, "EnqueueMotionBatch expects JSON array"});
                }
                for (const auto& obj : doc) {
                    if (!obj.is_object()) continue;

                    const double target_pos = obj.value("target_position_deg", 0.0);
                    const double target_vel = obj.value("target_velocity_deg_per_sec", 0.0);
                    const double profile_accel = obj.value("profile_accel_percent", 0.0);
                    const double sample_period = obj.value("sample_period_sec", 0.005);
                    const int profile_speed = obj.value("profile_speed_rpm", 0);
                    const bool is_relative = obj.value("is_relative", false);

                    const auto sp = motion_core::build_motion_command_point(
                        axis_id,
                        motion_core::MotionCommandType::Stream,
                        target_pos,
                        target_vel,
                        profile_accel,
                        sample_period,
                        is_relative,
                        profile_speed
                    );

                    const auto submit_res = submit_motion(sp);
                    if (!submit_res.ok()) {
                        return submit_res;
                    }
                }
            } catch (const std::exception& e) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "Invalid JSON in EnqueueMotionBatch"});
            }
            return motion_core::Result<std::string>::success("");
        }

        case ControlOp::ClearMotionQueue: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::ClearMotionQueue;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        default:
            return motion_core::Result<std::string>::success("");
    }
}

} // namespace hal_ipc
