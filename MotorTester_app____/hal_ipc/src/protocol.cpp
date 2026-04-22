#include "hal_ipc/protocol.h"

#include <nlohmann/json.hpp>

#include <string>

using json = nlohmann::json;

namespace hal_ipc {

namespace {

template <typename T>
motion_core::Result<T> protocol_failure(const char* message) {
    return motion_core::Result<T>::failure({motion_core::ErrorCode::ProtocolFailure, message});
}

const char* control_op_to_string(const ControlOp op) {
    switch (op) {
        case ControlOp::None: return "None";
        case ControlOp::StreamPoint: return "StreamPoint";
        case ControlOp::Hold: return "Hold";
        case ControlOp::Stop: return "Stop";
        case ControlOp::RequestManualLease: return "RequestManualLease";
        case ControlOp::ReleaseManualLease: return "ReleaseManualLease";
        case ControlOp::EnableAxis: return "EnableAxis";
        case ControlOp::DisableAxis: return "DisableAxis";
        case ControlOp::SetZero: return "SetZero";
        case ControlOp::ClearFault: return "ClearFault";
        case ControlOp::Home: return "Home";
        case ControlOp::EnqueueMotionBatch: return "EnqueueMotionBatch";
        case ControlOp::SetAxisMode: return "SetAxisMode";
        case ControlOp::ConfigureMotionQueue: return "ConfigureMotionQueue";
        case ControlOp::ClearMotionQueue: return "ClearMotionQueue";
        case ControlOp::QueryMotionQueueStats: return "QueryMotionQueueStats";
        case ControlOp::ReadParameters: return "ReadParameters";
        case ControlOp::ListParameters: return "ListParameters";
        case ControlOp::ApplyParameterPatch: return "ApplyParameterPatch";
        case ControlOp::SetPersistent: return "SetPersistent";
        case ControlOp::ImportAxisConfig: return "ImportAxisConfig";
        case ControlOp::ImportAxisConfigPreview: return "ImportAxisConfigPreview";
        case ControlOp::ExportAxisConfig: return "ExportAxisConfig";
    }
    return "None";
}

motion_core::Result<ControlOp> control_op_from_string(const std::string& value) {
    if (value == "None") return motion_core::Result<ControlOp>::success(ControlOp::None);
    if (value == "StreamPoint") return motion_core::Result<ControlOp>::success(ControlOp::StreamPoint);
    if (value == "Hold") return motion_core::Result<ControlOp>::success(ControlOp::Hold);
    if (value == "Stop") return motion_core::Result<ControlOp>::success(ControlOp::Stop);
    if (value == "RequestManualLease") return motion_core::Result<ControlOp>::success(ControlOp::RequestManualLease);
    if (value == "ReleaseManualLease") return motion_core::Result<ControlOp>::success(ControlOp::ReleaseManualLease);
    if (value == "EnableAxis") return motion_core::Result<ControlOp>::success(ControlOp::EnableAxis);
    if (value == "DisableAxis") return motion_core::Result<ControlOp>::success(ControlOp::DisableAxis);
    if (value == "SetZero") return motion_core::Result<ControlOp>::success(ControlOp::SetZero);
    if (value == "ClearFault") return motion_core::Result<ControlOp>::success(ControlOp::ClearFault);
    if (value == "Home") return motion_core::Result<ControlOp>::success(ControlOp::Home);
    if (value == "EnqueueMotionBatch") return motion_core::Result<ControlOp>::success(ControlOp::EnqueueMotionBatch);
    if (value == "SetAxisMode") return motion_core::Result<ControlOp>::success(ControlOp::SetAxisMode);
    if (value == "ConfigureMotionQueue") return motion_core::Result<ControlOp>::success(ControlOp::ConfigureMotionQueue);
    if (value == "ClearMotionQueue") return motion_core::Result<ControlOp>::success(ControlOp::ClearMotionQueue);
    if (value == "QueryMotionQueueStats") return motion_core::Result<ControlOp>::success(ControlOp::QueryMotionQueueStats);
    if (value == "ReadParameters") return motion_core::Result<ControlOp>::success(ControlOp::ReadParameters);
    if (value == "ListParameters") return motion_core::Result<ControlOp>::success(ControlOp::ListParameters);
    if (value == "ApplyParameterPatch") return motion_core::Result<ControlOp>::success(ControlOp::ApplyParameterPatch);
    if (value == "SetPersistent") return motion_core::Result<ControlOp>::success(ControlOp::SetPersistent);
    if (value == "ImportAxisConfig") return motion_core::Result<ControlOp>::success(ControlOp::ImportAxisConfig);
    if (value == "ImportAxisConfigPreview") return motion_core::Result<ControlOp>::success(ControlOp::ImportAxisConfigPreview);
    if (value == "ExportAxisConfig") return motion_core::Result<ControlOp>::success(ControlOp::ExportAxisConfig);
    return protocol_failure<ControlOp>("unknown control op");
}

const char* owner_role_to_string(const OwnerRole role) {
    switch (role) {
        case OwnerRole::None: return "None";
        case OwnerRole::HexaMotion: return "HexaMotion";
        case OwnerRole::MotorTesterUi: return "MotorTesterUi";
        case OwnerRole::Service: return "Service";
    }
    return "None";
}

motion_core::Result<OwnerRole> owner_role_from_string(const std::string& value) {
    if (value == "None") return motion_core::Result<OwnerRole>::success(OwnerRole::None);
    if (value == "HexaMotion") return motion_core::Result<OwnerRole>::success(OwnerRole::HexaMotion);
    if (value == "MotorTesterUi") return motion_core::Result<OwnerRole>::success(OwnerRole::MotorTesterUi);
    if (value == "Service") return motion_core::Result<OwnerRole>::success(OwnerRole::Service);
    return protocol_failure<OwnerRole>("unknown owner role");
}

void axis_to_json(json& out, const AxisPointDto& axis) {
    out = json{
        {"axis_id", axis.axis_id},
        {"has_interpolated_position", axis.has_interpolated_position},
        {"interpolated_position_deg", axis.interpolated_position_deg},
        {"has_interpolated_velocity", axis.has_interpolated_velocity},
        {"interpolated_velocity_deg_s", axis.interpolated_velocity_deg_s},
        {"segment_start_deg", axis.segment_start_deg},
        {"segment_target_deg", axis.segment_target_deg},
        {"segment_duration_sec", axis.segment_duration_sec},
        {"actual_position_deg", axis.actual_position_deg},
        {"actual_velocity_deg_s", axis.actual_velocity_deg_s},
    };
}

motion_core::Result<AxisPointDto> axis_from_json(const json& in) {
    if (!in.is_object()) {
        return protocol_failure<AxisPointDto>("axis entry must be object");
    }
    AxisPointDto axis{};
    axis.axis_id = in.value("axis_id", 0U);
    axis.has_interpolated_position = in.value("has_interpolated_position", false);
    axis.interpolated_position_deg = in.value("interpolated_position_deg", 0.0);
    axis.has_interpolated_velocity = in.value("has_interpolated_velocity", false);
    axis.interpolated_velocity_deg_s = in.value("interpolated_velocity_deg_s", 0.0);
    axis.segment_start_deg = in.value("segment_start_deg", 0.0);
    axis.segment_target_deg = in.value("segment_target_deg", 0.0);
    axis.segment_duration_sec = in.value("segment_duration_sec", 0.0);
    axis.actual_position_deg = in.value("actual_position_deg", 0.0);
    axis.actual_velocity_deg_s = in.value("actual_velocity_deg_s", 0.0);
    return motion_core::Result<AxisPointDto>::success(axis);
}

} // namespace

motion_core::Result<std::string> serialize_control_frame(const HalControlFrameDto& frame) {
    json j;
    j["seq"] = frame.seq;
    j["client_id"] = frame.client_id;
    j["lease_epoch"] = frame.lease_epoch;
    j["op"] = control_op_to_string(frame.op);
    j["service_axis_id"] = frame.service_axis_id;
    j["service_bool_value"] = frame.service_bool_value;
    j["service_int_value"] = frame.service_int_value;
    j["service_string_value"] = frame.service_string_value;

    j["axes"] = json::array();
    for (const auto& axis : frame.axes) {
        json a;
        axis_to_json(a, axis);
        j["axes"].push_back(std::move(a));
    }
    return motion_core::Result<std::string>::success(j.dump());
}

motion_core::Result<std::string> serialize_state_frame(const HalStateFrameDto& frame) {
    json j;
    j["seq"] = frame.seq;
    j["ack_control_seq"] = frame.ack_control_seq;
    j["motion_owner"] = owner_role_to_string(frame.motion_owner);
    j["service_owner"] = owner_role_to_string(frame.service_owner);
    j["motion_epoch"] = frame.motion_epoch;
    j["service_epoch"] = frame.service_epoch;
    j["manual_override_active"] = frame.manual_override_active;
    j["service_mode_active"] = frame.service_mode_active;
    j["estop_active"] = frame.estop_active;
    j["service_status_code"] = frame.service_status_code;
    j["service_axis_id"] = frame.service_axis_id;
    j["service_string_response"] = frame.service_string_response;

    j["axes"] = json::array();
    for (const auto& axis : frame.axes) {
        json a;
        axis_to_json(a, axis);
        j["axes"].push_back(std::move(a));
    }
    return motion_core::Result<std::string>::success(j.dump());
}

motion_core::Result<HalControlFrameDto> deserialize_control_frame(const std::string& payload) {
    const auto parsed = json::parse(payload, nullptr, false);
    if (parsed.is_discarded() || !parsed.is_object()) {
        return protocol_failure<HalControlFrameDto>("invalid control json payload");
    }

    HalControlFrameDto out{};
    out.seq = parsed.value("seq", static_cast<std::uint64_t>(0));
    out.client_id = parsed.value("client_id", static_cast<std::uint32_t>(0));
    out.lease_epoch = parsed.value("lease_epoch", static_cast<std::uint32_t>(0));
    out.service_axis_id = parsed.value("service_axis_id", -1);
    out.service_bool_value = parsed.value("service_bool_value", false);
    out.service_int_value = parsed.value("service_int_value", 0);
    out.service_string_value = parsed.value("service_string_value", std::string(""));

    const auto op_string = parsed.value("op", std::string("None"));
    const auto op_res = control_op_from_string(op_string);
    if (!op_res.ok()) {
        return motion_core::Result<HalControlFrameDto>::failure(op_res.error());
    }
    out.op = op_res.value();

    if (parsed.contains("axes")) {
        if (!parsed.at("axes").is_array()) {
            return protocol_failure<HalControlFrameDto>("control axes must be array");
        }
        for (const auto& axis_json : parsed.at("axes")) {
            const auto axis_res = axis_from_json(axis_json);
            if (!axis_res.ok()) {
                return motion_core::Result<HalControlFrameDto>::failure(axis_res.error());
            }
            out.axes.push_back(axis_res.value());
        }
    }

    return motion_core::Result<HalControlFrameDto>::success(std::move(out));
}

motion_core::Result<HalStateFrameDto> deserialize_state_frame(const std::string& payload) {
    const auto parsed = json::parse(payload, nullptr, false);
    if (parsed.is_discarded() || !parsed.is_object()) {
        return protocol_failure<HalStateFrameDto>("invalid state json payload");
    }

    HalStateFrameDto out{};
    out.seq = parsed.value("seq", static_cast<std::uint64_t>(0));
    out.ack_control_seq = parsed.value("ack_control_seq", static_cast<std::uint64_t>(0));
    out.manual_override_active = parsed.value("manual_override_active", false);
    out.service_mode_active = parsed.value("service_mode_active", false);
    out.estop_active = parsed.value("estop_active", false);
    out.service_status_code = parsed.value("service_status_code", 0);
    out.service_axis_id = parsed.value("service_axis_id", -1);
    out.service_string_response = parsed.value("service_string_response", std::string(""));

    const auto motion_owner_string = parsed.value("motion_owner", std::string("None"));
    const auto motion_owner_res = owner_role_from_string(motion_owner_string);
    if (!motion_owner_res.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(motion_owner_res.error());
    }
    out.motion_owner = motion_owner_res.value();

    const auto service_owner_string = parsed.value("service_owner", std::string("None"));
    const auto service_owner_res = owner_role_from_string(service_owner_string);
    if (!service_owner_res.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(service_owner_res.error());
    }
    out.service_owner = service_owner_res.value();

    out.motion_epoch = parsed.value("motion_epoch", static_cast<std::uint32_t>(0));
    out.service_epoch = parsed.value("service_epoch", static_cast<std::uint32_t>(0));

    if (parsed.contains("axes")) {
        if (!parsed.at("axes").is_array()) {
            return protocol_failure<HalStateFrameDto>("state axes must be array");
        }
        for (const auto& axis_json : parsed.at("axes")) {
            const auto axis_res = axis_from_json(axis_json);
            if (!axis_res.ok()) {
                return motion_core::Result<HalStateFrameDto>::failure(axis_res.error());
            }
            out.axes.push_back(axis_res.value());
        }
    }

    return motion_core::Result<HalStateFrameDto>::success(std::move(out));
}

} // namespace hal_ipc
