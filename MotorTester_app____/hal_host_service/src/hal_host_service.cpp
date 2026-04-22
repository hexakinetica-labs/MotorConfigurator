#include "hal_host_service/hal_host_service.h"

#include "drivers/interfaces/init_all_drivers.h"
#include "motion_core/config/hal_runtime_config_json.h"
#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <vector>

namespace hal_host_service {


using json = nlohmann::json;

namespace {
void param_value_to_json(json& out, const motion_core::ParameterValue& val) {
    out["type"] = static_cast<int>(val.type);
    switch (val.type) {
        case motion_core::ParameterValueType::SignedInteger: out["val"] = val.signed_value; break;
        case motion_core::ParameterValueType::UnsignedInteger: out["val"] = val.unsigned_value; break;
        case motion_core::ParameterValueType::FloatingPoint: out["val"] = val.floating_value; break;
        case motion_core::ParameterValueType::Boolean: out["val"] = val.bool_value; break;
    }
}

motion_core::Result<motion_core::ParameterValue> param_value_from_json(const json& in) {
    if (!in.contains("type") || !in.contains("val")) return motion_core::Result<motion_core::ParameterValue>::failure({motion_core::ErrorCode::InvalidArgument, "Invalid JSON ParameterValue"});
    auto type = static_cast<motion_core::ParameterValueType>(in["type"].get<int>());
    switch (type) {
        case motion_core::ParameterValueType::SignedInteger: return motion_core::Result<motion_core::ParameterValue>::success(motion_core::ParameterValue::from_signed(in["val"].get<std::int64_t>()));
        case motion_core::ParameterValueType::UnsignedInteger: return motion_core::Result<motion_core::ParameterValue>::success(motion_core::ParameterValue::from_unsigned(in["val"].get<std::uint64_t>()));
        case motion_core::ParameterValueType::FloatingPoint: return motion_core::Result<motion_core::ParameterValue>::success(motion_core::ParameterValue::from_floating(in["val"].get<double>()));
        case motion_core::ParameterValueType::Boolean: return motion_core::Result<motion_core::ParameterValue>::success(motion_core::ParameterValue::from_bool(in["val"].get<bool>()));
    }
    return motion_core::Result<motion_core::ParameterValue>::failure({motion_core::ErrorCode::InvalidArgument, "Unknown type"});
}

[[nodiscard]] json motion_queue_stats_to_json(const motion_core::MotionQueueStats& s) {
    json stats;
    stats["size"]               = s.size;
    stats["capacity"]           = s.capacity;
    stats["pushed"]             = s.pushed;
    stats["dropped"]            = s.dropped;
    stats["underruns"]          = s.underruns;
    stats["short_starts"]       = s.short_starts;
    stats["available_capacity"] = s.capacity - s.size;
    stats["configured"]         = s.capacity > 0;
    return stats;
}
}

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
        [this]() {
            (void)ownership_.handle_lease_op(hal_ipc::OwnerRole::HexaMotion,
                                             hal_ipc::ControlOp::ReleaseManualLease);
        });
    if (!server_res.ok()) {
        return server_res;
    }

    state_seq_ = 0;
    ownership_.reset();
    estop_latched_ = false;
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
    ownership_.reset();
    estop_latched_ = false;
    return motion_core::Result<void>::success();
}

bool HalHostService::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

hal_ipc::HalStateFrameDto HalHostService::handle_control_frame(const hal_ipc::HalControlFrameDto& frame) {
    std::unique_lock<std::mutex> lock(mutex_);

    hal_ipc::HalStateFrameDto state{};
    state.seq = ++state_seq_;
    state.ack_control_seq = frame.seq;

    const auto initial_ownership = ownership_.snapshot();
    state.motion_owner = initial_ownership.motion_owner;
    state.service_owner = initial_ownership.service_owner;
    state.motion_epoch = initial_ownership.motion_epoch;
    state.service_epoch = initial_ownership.service_epoch;
    state.manual_override_active = initial_ownership.manual_override_active;
    state.service_mode_active = initial_ownership.service_mode_active;

    const bool allow_local_inproc = (frame.client_id == hal_ipc::kClientIdMotorTesterUi);
    if (!running_.load(std::memory_order_acquire) && !allow_local_inproc) {
        state.service_status_code = static_cast<std::int32_t>(motion_core::ErrorCode::NotConnected);
        return state;
    }

    lock.unlock();
    const auto apply_res = apply_control_frame(frame);
    lock.lock();

    if (!apply_res.ok()) {
        state.service_status_code = static_cast<std::int32_t>(apply_res.error().code);
    } else {
        state.service_string_response = apply_res.value();
    }

    const auto final_ownership = ownership_.snapshot();
    state.motion_owner = final_ownership.motion_owner;
    state.service_owner = final_ownership.service_owner;
    state.motion_epoch = final_ownership.motion_epoch;
    state.service_epoch = final_ownership.service_epoch;
    state.manual_override_active = final_ownership.manual_override_active;
    state.service_mode_active = final_ownership.service_mode_active;
    refresh_state_from_runtime(state);
    return state;
}

hal_ipc::HalStateFrameDto HalHostService::execute_local_command(const hal_ipc::HalControlFrameDto& frame) {
    return handle_control_frame(frame);
}

HalHostService::HostStateSnapshot HalHostService::state_snapshot() const {
    std::scoped_lock lock(mutex_);
    HostStateSnapshot snapshot{};
    const auto ownership_state = ownership_.snapshot();
    snapshot.motion_owner = ownership_state.motion_owner;
    snapshot.service_owner = ownership_state.service_owner;
    snapshot.motion_epoch = ownership_state.motion_epoch;
    snapshot.service_epoch = ownership_state.service_epoch;
    snapshot.manual_override_active = ownership_state.manual_override_active;
    snapshot.service_mode_active = ownership_state.service_mode_active;
    snapshot.estop_active = estop_latched_;
    snapshot.connected_client_count = server_.connected_client_count();
    snapshot.ipc_server_running = running_.load(std::memory_order_acquire);
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
    const auto caller = caller_res.value();

    const auto lease_res = ownership_.handle_lease_op(caller, frame.op);
    if (!lease_res.ok()) {
        return motion_core::Result<std::string>::failure(lease_res.error());
    }
    const auto auth_res = ownership_.authorize_operation(caller, frame.op, frame.lease_epoch);
    if (!auth_res.ok()) {
        return motion_core::Result<std::string>::failure(auth_res.error());
    }

    if (frame.op == hal_ipc::ControlOp::Stop) {
        estop_latched_ = true;
    }

    if (estop_latched_ && frame.op == hal_ipc::ControlOp::ClearFault) {
        estop_latched_ = false;
    }

    if (estop_latched_
        && frame.op != hal_ipc::ControlOp::None
        && frame.op != hal_ipc::ControlOp::Stop
        && frame.op != hal_ipc::ControlOp::ClearFault
        && frame.op != hal_ipc::ControlOp::RequestManualLease
        && frame.op != hal_ipc::ControlOp::ReleaseManualLease
        && frame.op != hal_ipc::ControlOp::QueryMotionQueueStats
        && frame.op != hal_ipc::ControlOp::ReadParameters
        && frame.op != hal_ipc::ControlOp::ListParameters
        && frame.op != hal_ipc::ControlOp::ImportAxisConfigPreview) {
        return motion_core::Result<std::string>::failure(
            {motion_core::ErrorCode::Busy, "global estop is active"});
    }

    if (!frame.axes.empty()) {
        std::string last_response;
        for (const auto& axis : frame.axes) {
            const auto op_res = apply_axis_op(frame.op, axis.axis_id, &axis, frame);
            if (!op_res.ok()) {
                return op_res;
            }
            last_response = op_res.value();
        }
        return motion_core::Result<std::string>::success(last_response);
    }

    if (frame.op == hal_ipc::ControlOp::Stop && frame.service_axis_id <= 0) {
        const auto axes_res = runtime_.list_axes();
        if (!axes_res.ok()) {
            return motion_core::Result<std::string>::failure(axes_res.error());
        }
        for (const auto& axis_info : axes_res.value()) {
            const auto stop_res = apply_axis_op(frame.op, axis_info.id.value, nullptr, frame);
            if (!stop_res.ok()) {
                return stop_res;
            }
            const auto clear_queue_res = apply_axis_op(hal_ipc::ControlOp::ClearMotionQueue,
                                                       axis_info.id.value,
                                                       nullptr,
                                                       frame);
            if (!clear_queue_res.ok()) {
                return clear_queue_res;
            }
        }
        return motion_core::Result<std::string>::success("");
    }

    if (frame.service_axis_id > 0) {
        return apply_axis_op(frame.op, static_cast<std::uint16_t>(frame.service_axis_id), nullptr, frame);
    }
    
    // Some configs might not target axis explicitly or default to empty response
    return motion_core::Result<std::string>::success("");
}
motion_core::Result<std::string> HalHostService::apply_axis_op(const hal_ipc::ControlOp op,
                                                        const std::uint16_t axis_id,
                                                        const hal_ipc::AxisPointDto* point,
                                                        const hal_ipc::HalControlFrameDto& frame) {
    if (axis_id == 0U) {
        return motion_core::Result<std::string>::failure(
            {motion_core::ErrorCode::InvalidArgument, "axis_id must be > 0"});
    }

    const auto axis_res = runtime_.find_axis(axis_id);
    if (!axis_res.ok()) {
        return motion_core::Result<std::string>::failure(axis_res.error());
    }

    auto axis = axis_res.value();
    const auto transport = axis->info().transport;
    switch (op) {
        case hal_ipc::ControlOp::EnableAxis: {
            auto r = axis->set_enabled(true);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::DisableAxis: {
            auto r = axis->set_enabled(false);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::StreamPoint:
        case hal_ipc::ControlOp::Hold:
        case hal_ipc::ControlOp::SetZero:
        case hal_ipc::ControlOp::ClearFault:
        case hal_ipc::ControlOp::Home: {
            const auto cmd = build_axis_command(op, point);
            auto r = axis->apply_command(cmd);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::Stop: {
            const auto cmd = build_axis_command(op, point);
            auto r = axis->apply_command(cmd);
            auto disable_r = axis->set_enabled(false);
            if (!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            if (!disable_r.ok()) return motion_core::Result<std::string>::failure(disable_r.error());
            return motion_core::Result<std::string>::success("");
        }
        
        case hal_ipc::ControlOp::SetAxisMode: {
            auto r = axis->set_mode(static_cast<motion_core::AxisMode>(frame.service_int_value));
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }
        
        case hal_ipc::ControlOp::ConfigureMotionQueue: {
            auto r = axis->configure_motion_queue(frame.service_int_value, frame.service_bool_value);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }
        
        case hal_ipc::ControlOp::ClearMotionQueue: {
            auto r = axis->clear_motion_queue();
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }
        
        case hal_ipc::ControlOp::QueryMotionQueueStats: {
            auto r = axis->query_motion_queue_stats();
            if(!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            return motion_core::Result<std::string>::success(motion_queue_stats_to_json(r.value()).dump());
        }
        
        case hal_ipc::ControlOp::ReadParameters: {
            auto r = axis->read_parameters();
            if(!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            json j = json::array();
            for (const auto& entry : r.value().entries) {
                json e; 
                e["domain"] = entry.id.domain;
                e["value"] = entry.id.value;
                json val; param_value_to_json(val, entry.value); 
                e["data"] = val; 
                j.push_back(std::move(e));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }
        
        case hal_ipc::ControlOp::ListParameters: {
            auto r = axis->list_parameters();
            if(!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            json j = json::array();
            for (const auto& desc : r.value()) {
                json d;
                d["domain"] = desc.id.domain;
                d["value"] = desc.id.value;
                d["name"] = desc.name;
                d["group"] = desc.group;
                d["unit"] = desc.unit;
                d["read_only"] = desc.read_only;
                d["persistable"] = desc.persistable;
                d["has_min"] = desc.has_min;
                d["has_max"] = desc.has_max;
                if (desc.has_min) { json min_val; param_value_to_json(min_val, desc.min_value); d["min_value"] = min_val; }
                if (desc.has_max) { json max_val; param_value_to_json(max_val, desc.max_value); d["max_value"] = max_val; }
                j.push_back(std::move(d));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }
        
        case hal_ipc::ControlOp::EnqueueMotionBatch: {
            if (frame.service_string_value.empty()) return motion_core::Result<std::string>::success("");
            const auto parsed = json::parse(frame.service_string_value, nullptr, false);
            if (parsed.is_discarded() || !parsed.is_array()) return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InvalidArgument, "batch must be array"});
            std::vector<motion_core::QueuedSetpoint> batch;
            for (const auto& pt : parsed) {
                motion_core::QueuedSetpoint point{};
                point.target_position_deg = pt.value("target_position_deg", 0.0);
                if (pt.contains("has_profile_speed_rpm") && pt.value("has_profile_speed_rpm", false)) {
                    point.has_profile_speed_rpm = true;
                    point.profile_speed_rpm = pt.value("profile_speed_rpm", 0);
                }
                if (pt.contains("has_profile_accel_percent") && pt.value("has_profile_accel_percent", false)) {
                    point.has_profile_accel_percent = true;
                    point.profile_accel_percent = pt.value("profile_accel_percent", 0.0);
                }
                if (pt.contains("has_target_velocity") && pt.value("has_target_velocity", false)) {
                    point.has_target_velocity = true;
                    point.target_velocity_deg_per_sec = pt.value("target_velocity_deg_per_sec", 0.0);
                }
                if (pt.contains("sample_period_sec")) {
                    point.sample_period_sec = pt.value("sample_period_sec", 0.001);
                }
                batch.push_back(point);
            }
            auto res = axis->enqueue_motion_batch(batch);
            if (!res.ok()) return motion_core::Result<std::string>::failure(res.error());
            return motion_core::Result<std::string>::success(motion_queue_stats_to_json(res.value()).dump());
        }
        
        case hal_ipc::ControlOp::ApplyParameterPatch: {
            if (frame.service_string_value.empty()) return motion_core::Result<std::string>::success("");
            const auto parsed = json::parse(frame.service_string_value, nullptr, false);
            if (parsed.is_discarded() || !parsed.is_array()) return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InvalidArgument, "patch must be array"});
            motion_core::ParameterPatch patch;
            for (const auto& e : parsed) {
                motion_core::ParameterEntry entry{};
                entry.id.domain = static_cast<motion_core::ParameterDomain>(e.value("domain", 0U));
                entry.id.value = e.value("value", 0U);
                if (e.contains("data")) {
                    auto pv = param_value_from_json(e["data"]);
                    if (pv.ok()) entry.value = pv.value();
                }
                patch.entries.push_back(entry);
            }
            auto r = axis->apply_parameter_patch(patch);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }
        
        case hal_ipc::ControlOp::SetPersistent: {
            if (frame.service_string_value.empty()) return motion_core::Result<std::string>::success("");
            const auto parsed = json::parse(frame.service_string_value, nullptr, false);
            if (parsed.is_discarded() || !parsed.is_object()) return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InvalidArgument, "persistent req must be object"});
            auto cmd = static_cast<motion_core::PersistentCommand>(parsed.value("command", 0));
            motion_core::ParameterValue val{};
            if (parsed.contains("data")) {
                auto pv = param_value_from_json(parsed["data"]);
                if (pv.ok()) val = pv.value();
            }
            auto r = axis->set_persistent(cmd, val);
            if (!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            json out;
            out["command_supported"] = r.value().command_supported;
            out["unlock_performed"] = r.value().unlock_performed;
            out["write_completed"] = r.value().write_completed;
            out["persistent_save_completed"] = r.value().persistent_save_completed;
            out["readback_verified"] = r.value().readback_verified;
            out["reconnect_required"] = r.value().reconnect_required;
            out["power_cycle_required"] = r.value().power_cycle_required;
            return motion_core::Result<std::string>::success(out.dump());
        }
        
        case hal_ipc::ControlOp::ImportAxisConfigPreview: {
            auto r = runtime_.build_axis_config_patch(axis_id, frame.service_string_value);
            if (!r.ok()) return motion_core::Result<std::string>::failure(r.error());
            json j = json::array();
            for (const auto& entry : r.value().entries) {
                json e; 
                e["domain"] = entry.id.domain;
                e["value"] = entry.id.value;
                json val; param_value_to_json(val, entry.value); 
                e["data"] = val; 
                j.push_back(std::move(e));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }

        case hal_ipc::ControlOp::ImportAxisConfig: {
            auto r = runtime_.apply_axis_config_file(axis_id, frame.service_string_value);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }
        
        case hal_ipc::ControlOp::ExportAxisConfig: {
            auto r = runtime_.export_axis_config_to_file(axis_id, frame.service_string_value);
            return r.ok() ? motion_core::Result<std::string>::success("") : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::RequestManualLease:
        case hal_ipc::ControlOp::ReleaseManualLease:
        case hal_ipc::ControlOp::None:
            return motion_core::Result<std::string>::success("");
    }

    return motion_core::Result<std::string>::success("");
}

void HalHostService::refresh_state_from_runtime(hal_ipc::HalStateFrameDto& state) const {
    state.estop_active = estop_latched_;
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

motion_core::AxisCommand HalHostService::build_axis_command(const hal_ipc::ControlOp op,
                                                            const hal_ipc::AxisPointDto* point) {
    motion_core::AxisCommand cmd{};

    switch (op) {
        case hal_ipc::ControlOp::Stop:
            cmd.emergency_stop = true;
            break;

        case hal_ipc::ControlOp::SetZero:
            cmd.set_zero = true;
            break;

        case hal_ipc::ControlOp::ClearFault:
            cmd.clear_errors = true;
            break;

        case hal_ipc::ControlOp::Home:
            cmd.go_home = true;
            break;

        case hal_ipc::ControlOp::Hold:
            if (point != nullptr) {
                cmd.has_target_position = true;
                cmd.target_position_deg = point->actual_position_deg;
            }
            break;

        case hal_ipc::ControlOp::StreamPoint:
            if (point != nullptr) {
                if (point->has_interpolated_position) {
                    cmd.has_target_position = true;
                    cmd.target_position_deg = point->interpolated_position_deg;
                } else {
                    cmd.has_target_position = true;
                    cmd.target_position_deg = point->segment_target_deg;
                }
                if (point->has_interpolated_velocity) {
                    cmd.has_target_velocity = true;
                    cmd.target_velocity_deg_per_sec = point->interpolated_velocity_deg_s;
                }
            }
            break;

        case hal_ipc::ControlOp::EnableAxis:
        case hal_ipc::ControlOp::DisableAxis:
        case hal_ipc::ControlOp::RequestManualLease:
        case hal_ipc::ControlOp::ReleaseManualLease:
        case hal_ipc::ControlOp::EnqueueMotionBatch:
        case hal_ipc::ControlOp::SetAxisMode:
        case hal_ipc::ControlOp::ConfigureMotionQueue:
        case hal_ipc::ControlOp::ClearMotionQueue:
        case hal_ipc::ControlOp::QueryMotionQueueStats:
        case hal_ipc::ControlOp::ReadParameters:
        case hal_ipc::ControlOp::ListParameters:
        case hal_ipc::ControlOp::ApplyParameterPatch:
        case hal_ipc::ControlOp::SetPersistent:
        case hal_ipc::ControlOp::ImportAxisConfig:
        case hal_ipc::ControlOp::ImportAxisConfigPreview:
        case hal_ipc::ControlOp::ExportAxisConfig:
        case hal_ipc::ControlOp::None:
            break;
    }

    return cmd;
}

} // namespace hal_host_service
