#include "mks/axis_manager.h"

#include "ethercat/p100e_ethercat_dictionary.h"
#include "mks_can/adapter/mks_axis_adapter.h"
#include <QCoreApplication>
#include <QThreadPool>
#include <QRunnable>
#include "mks_can/dictionary/mks_dictionary.h"
#include "motion_core/config/hal_runtime_config_json.h"
#include <nlohmann/json.hpp>

#include <QMetaObject>
#include <QPointer>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {

using json = nlohmann::json;

constexpr std::size_t kMaxUiPositionSamplesPerTick = 512U;
constexpr int kSharedMotionQueueCapacity = 50;

// TODO: transport tag should be passed from UI, or derived from axis
[[nodiscard]] static QString transport_tag_for_axis(const std::shared_ptr<motion_core::IAxis>& axis) {
    if (!axis) {
        return "hal";
    }
    switch (axis->info().transport) {
        case motion_core::AxisTransportKind::CanBus:   return "mks";
        case motion_core::AxisTransportKind::Ethercat: return "ethercat";
        default:                                       return "hal";
    }
}

[[nodiscard]] static QVariant parameter_value_to_qvariant(const motion_core::ParameterValue& value) {
    switch (value.type) {
        case motion_core::ParameterValueType::SignedInteger:
            return QVariant::fromValue<qlonglong>(static_cast<qlonglong>(value.signed_value));
        case motion_core::ParameterValueType::UnsignedInteger:
            return QVariant::fromValue<qulonglong>(static_cast<qulonglong>(value.unsigned_value));
        case motion_core::ParameterValueType::FloatingPoint:
            return QVariant::fromValue<double>(value.floating_value);
        case motion_core::ParameterValueType::Boolean:
            return QVariant::fromValue<bool>(value.bool_value);
    }
    return {};
}

[[nodiscard]] static QString parameter_value_to_string(const motion_core::ParameterValue& value) {
    const QVariant v = parameter_value_to_qvariant(value);
    if (!v.isValid()) {
        return {};
    }
    if (v.typeId() == QMetaType::Bool) {
        return v.toBool() ? QStringLiteral("true") : QStringLiteral("false");
    }
    return v.toString();
}

[[nodiscard]] static motion_core::Result<motion_core::ParameterValue> parameter_value_from_variant(
    const QVariant& value) {
    switch (value.typeId()) {
        case QMetaType::Bool:
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_bool(value.toBool()));
        case QMetaType::Float:
        case QMetaType::Double:
            return motion_core::Result<motion_core::ParameterValue>::success(
                motion_core::ParameterValue::from_floating(value.toDouble()));
        default:
            break;
    }

    bool signed_ok = false;
    const qlonglong signed_value = value.toLongLong(&signed_ok);
    if (signed_ok) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_signed(static_cast<std::int64_t>(signed_value)));
    }

    bool unsigned_ok = false;
    const qulonglong unsigned_value = value.toULongLong(&unsigned_ok);
    if (unsigned_ok) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_unsigned(static_cast<std::uint64_t>(unsigned_value)));
    }

    bool floating_ok = false;
    const double floating_value = value.toDouble(&floating_ok);
    if (floating_ok) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_floating(floating_value));
    }

    return motion_core::Result<motion_core::ParameterValue>::failure(
        {motion_core::ErrorCode::InvalidArgument, "parameter value must be bool/numeric"});
}



[[nodiscard]] static QString owner_role_to_string(const hal_ipc::OwnerRole role) {
    switch (role) {
        case hal_ipc::OwnerRole::HexaMotion:
            return QStringLiteral("hexamotion");
        case hal_ipc::OwnerRole::MotorTesterUi:
            return QStringLiteral("ui");
        case hal_ipc::OwnerRole::None:
        default:
            return QStringLiteral("none");
    }
}

[[nodiscard]] static hal_host_service::MotionControlSource motion_source_from_owner_role(
    const hal_ipc::OwnerRole role) {
    switch (role) {
        case hal_ipc::OwnerRole::HexaMotion:
            return hal_host_service::MotionControlSource::HexaMotion;
        case hal_ipc::OwnerRole::MotorTesterUi:
            return hal_host_service::MotionControlSource::Ui;
        default:
            // Unknown caller must not silently get UI privileges
            return hal_host_service::MotionControlSource::Ui;
    }
}

[[nodiscard]] static bool is_hexamotion_stream_client_op(const hal_ipc::ControlOp op) {
    switch (op) {
        case hal_ipc::ControlOp::StreamPoint:
        case hal_ipc::ControlOp::EnqueueMotionBatch:
        case hal_ipc::ControlOp::QueryMotionQueueStats:
        case hal_ipc::ControlOp::None:
            return true;

        case hal_ipc::ControlOp::EnableAxis:
        case hal_ipc::ControlOp::DisableAxis:
        case hal_ipc::ControlOp::Stop:
        case hal_ipc::ControlOp::Hold:
        case hal_ipc::ControlOp::SetZero:
        case hal_ipc::ControlOp::ClearFault:
        case hal_ipc::ControlOp::Home:
        case hal_ipc::ControlOp::SetAxisMode:
        case hal_ipc::ControlOp::ConfigureMotionQueue:
        case hal_ipc::ControlOp::ClearMotionQueue:
        case hal_ipc::ControlOp::ReadParameters:
        case hal_ipc::ControlOp::ListParameters:
        case hal_ipc::ControlOp::ApplyParameterPatch:
        case hal_ipc::ControlOp::SetPersistent:
        case hal_ipc::ControlOp::ImportAxisConfig:
        case hal_ipc::ControlOp::ImportAxisConfigPreview:
        case hal_ipc::ControlOp::ExportAxisConfig:
            return false;
    }

    return false;
}

[[nodiscard]] static motion_core::QueuedSetpoint queued_setpoint_from_variant_map(const QVariantMap& map) {
    motion_core::QueuedSetpoint point{};
    point.target_position_deg = map.value(QStringLiteral("target_position_deg")).toDouble();
    point.has_profile_speed_rpm = map.value(QStringLiteral("has_profile_speed_rpm")).toBool();
    point.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(map.value(QStringLiteral("profile_speed_rpm")).toInt(), 0, 3000));
    point.has_profile_accel_percent = map.value(QStringLiteral("has_profile_accel_percent")).toBool();
    point.profile_accel_percent = std::clamp(map.value(QStringLiteral("profile_accel_percent")).toDouble(), 0.0, 100.0);
    point.has_target_velocity = map.value(QStringLiteral("has_target_velocity")).toBool();
    point.target_velocity_deg_per_sec = map.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
    const double sample_period_sec = map.value(QStringLiteral("sample_period_sec")).toDouble();
    point.sample_period_sec = (std::isfinite(sample_period_sec) && sample_period_sec > 0.0)
        ? sample_period_sec
        : 0.005;
    return point;
}

[[nodiscard]] static json motion_queue_stats_to_json(const motion_core::MotionQueueStats& s) {
    json stats;
    stats["size"] = s.size;
    stats["capacity"] = s.capacity;
    stats["pushed"] = s.pushed;
    stats["dropped"] = s.dropped;
    stats["underruns"] = s.underruns;
    stats["short_starts"] = s.short_starts;
    stats["available_capacity"] = s.capacity - s.size;
    stats["configured"] = s.capacity > 0;
    return stats;
}

[[nodiscard]] static motion_core::QueuedSetpoint queued_setpoint_from_axis_point(const hal_ipc::AxisPointDto& axis_point) {
    motion_core::QueuedSetpoint point{};
    point.target_position_deg = axis_point.has_interpolated_position
        ? axis_point.interpolated_position_deg
        : axis_point.segment_target_deg;
    point.has_target_velocity = axis_point.has_interpolated_velocity;
    point.target_velocity_deg_per_sec = axis_point.interpolated_velocity_deg_s;
    point.sample_period_sec = (axis_point.segment_duration_sec > 0.0)
        ? axis_point.segment_duration_sec
        : 0.004;
    return point;
}

void param_value_to_json(json& out, const motion_core::ParameterValue& value) {
    switch (value.type) {
        case motion_core::ParameterValueType::SignedInteger:
            out = json{{"type", "signed"}, {"signed_value", value.signed_value}};
            break;
        case motion_core::ParameterValueType::UnsignedInteger:
            out = json{{"type", "unsigned"}, {"unsigned_value", value.unsigned_value}};
            break;
        case motion_core::ParameterValueType::FloatingPoint:
            out = json{{"type", "floating"}, {"floating_value", value.floating_value}};
            break;
        case motion_core::ParameterValueType::Boolean:
            out = json{{"type", "bool"}, {"bool_value", value.bool_value}};
            break;
    }
}

[[nodiscard]] motion_core::Result<motion_core::ParameterValue> param_value_from_json(const json& in) {
    if (!in.is_object()) {
        return motion_core::Result<motion_core::ParameterValue>::failure(
            {motion_core::ErrorCode::InvalidArgument, "parameter value must be object"});
    }

    const std::string type = in.value("type", std::string{});
    if (type == "signed") {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_signed(in.value("signed_value", static_cast<std::int64_t>(0))));
    }
    if (type == "unsigned") {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_unsigned(in.value("unsigned_value", static_cast<std::uint64_t>(0))));
    }
    if (type == "floating") {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_floating(in.value("floating_value", 0.0)));
    }
    if (type == "bool") {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_bool(in.value("bool_value", false)));
    }

    if (in.contains("signed_value")) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_signed(in.value("signed_value", static_cast<std::int64_t>(0))));
    }
    if (in.contains("unsigned_value")) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_unsigned(in.value("unsigned_value", static_cast<std::uint64_t>(0))));
    }
    if (in.contains("floating_value")) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_floating(in.value("floating_value", 0.0)));
    }
    if (in.contains("bool_value")) {
        return motion_core::Result<motion_core::ParameterValue>::success(
            motion_core::ParameterValue::from_bool(in.value("bool_value", false)));
    }

    return motion_core::Result<motion_core::ParameterValue>::failure(
        {motion_core::ErrorCode::InvalidArgument, "parameter value object is missing typed field"});
}

[[nodiscard]] static motion_core::AxisCommand build_axis_command(const hal_ipc::ControlOp op,
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
                cmd.has_target_position = true;
                cmd.target_position_deg = point->has_interpolated_position
                    ? point->interpolated_position_deg
                    : point->segment_target_deg;
                if (point->has_interpolated_velocity) {
                    cmd.has_target_velocity = true;
                    cmd.target_velocity_deg_per_sec = point->interpolated_velocity_deg_s;
                }
            }
            break;

        case hal_ipc::ControlOp::EnableAxis:
        case hal_ipc::ControlOp::DisableAxis:
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

} // namespace

namespace mks {

AxisManager::AxisManager(QObject* parent) : QObject(parent) {
    fast_timer_ = new QTimer(this);
    fast_timer_->setTimerType(Qt::PreciseTimer);
    fast_timer_->setInterval(4);
    connect(fast_timer_, &QTimer::timeout, this, &AxisManager::onFastTick);

    slow_timer_ = new QTimer(this);
    slow_timer_->setInterval(1000);
    connect(slow_timer_, &QTimer::timeout, this, [this]() {
        QVariantMap stats_map;
        double first_bus_hz = 0.0;
        for (const auto& bus : unified_runtime_.bus_managers_snapshot()) {
            if (!bus) {
                continue;
            }
            const auto stat_res = bus->get_statistics();
            if (!stat_res.ok()) {
                continue;
            }
            QVariantMap bus_map;
            bus_map["cycle_rate_hz"] = stat_res.value().cycle_rate_hz;
            bus_map["bus_load_percent"] = stat_res.value().bus_load_percent;
            stats_map[QString::fromStdString(bus->get_name())] = bus_map;
            if (first_bus_hz == 0.0 && stat_res.value().cycle_rate_hz > 0.0) {
                first_bus_hz = stat_res.value().cycle_rate_hz;
            }
        }
        cached_cycle_hz_ = first_bus_hz;
        if (!stats_map.isEmpty()) {
            emit busStatisticsUpdated(stats_map);
        }
        publishHostState();
    });

    host_service_ = std::make_unique<hal_host_service::HalHostService>(unified_runtime_);
    runtime_queue_ingress_ = std::make_shared<hal_host_service::RuntimeQueueIngress>(
        unified_runtime_,
        [this]() {
            hal_host_service::RuntimeQueueIngressState state{};
            hal_host_service::MotionControlSource control_source = hal_host_service::MotionControlSource::Ui;
            bool estop_active = false;
            bool hexamotion_connected = false;

            {
                std::lock_guard<std::mutex> lock(control_state_mutex_);
                control_source = control_source_;
                estop_active = estop_active_;
            }

            if (host_service_) {
                const auto snapshot = host_service_->state_snapshot();
                hexamotion_connected = snapshot.connected_hexamotion_client_count > 0;
            }

            state.control_source = control_source;
            state.hexamotion_connected = hexamotion_connected;
            state.estop_active = estop_active;
            return state;
        });
    host_service_->set_axis_operation_handler(
        [this](const hal_ipc::OwnerRole caller,
               const hal_ipc::ControlOp op,
               const std::uint16_t axis_id,
               const hal_ipc::AxisPointDto* point,
               const hal_ipc::HalControlFrameDto& frame) {
            return executeAxisOperation(caller, op, axis_id, point, frame);
        });
    host_service_->set_state_provider([this]() {
        hal_host_service::HalHostService::HostStateSnapshot snapshot{};
        {
            std::lock_guard<std::mutex> lock(control_state_mutex_);
            snapshot.motion_owner = (control_source_ == hal_host_service::MotionControlSource::HexaMotion)
                ? hal_ipc::OwnerRole::HexaMotion
                : hal_ipc::OwnerRole::MotorTesterUi;
            snapshot.manual_override_active = (control_source_ == hal_host_service::MotionControlSource::Ui);
            snapshot.estop_active = estop_active_;
        }
        return snapshot;
    });
}

AxisManager::~AxisManager() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

motion_core::Result<void> AxisManager::ensureManualMotionQueueReady(const int axis_id) {
    const auto stats_res = queryMotionQueueStatsDirect(axis_id);
    if (!stats_res.ok()) {
        return motion_core::Result<void>::failure(stats_res.error());
    }

    if (stats_res.value().capacity > 0U) {
        return motion_core::Result<void>::success();
    }

    hal_ipc::HalControlFrameDto frame{};
    frame.service_int_value = kSharedMotionQueueCapacity;
    frame.service_bool_value = false;
    const auto configure_res = executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::ConfigureMotionQueue, static_cast<std::uint16_t>(axis_id), nullptr, frame);
    if (!configure_res.ok()) {
        return motion_core::Result<void>::failure(configure_res.error());
    }

    const auto verify_res = queryMotionQueueStatsDirect(axis_id);
    if (!verify_res.ok()) {
        return motion_core::Result<void>::failure(verify_res.error());
    }
    if (verify_res.value().capacity == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InternalError, "motion queue configuration was not applied"});
    }

    return motion_core::Result<void>::success();
}

void AxisManager::publishHostState() {
    hal_ipc::OwnerRole motion_owner = hal_ipc::OwnerRole::None;
    bool manual_override_active = true;
    bool estop_active = false;
    hal_host_service::MotionControlSource control_source = hal_host_service::MotionControlSource::Ui;
    bool ipc_server_running = false;
    int connected_hexamotion_client_count = 0;

    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        control_source = control_source_;
        motion_owner = (control_source_ == hal_host_service::MotionControlSource::HexaMotion)
            ? hal_ipc::OwnerRole::HexaMotion
            : hal_ipc::OwnerRole::MotorTesterUi;
        manual_override_active = (control_source_ == hal_host_service::MotionControlSource::Ui);
        estop_active = estop_active_;
    }

    if (host_service_) {
        const auto snapshot = host_service_->state_snapshot();
        ipc_server_running = snapshot.ipc_server_running;
        connected_hexamotion_client_count = snapshot.connected_hexamotion_client_count;
    }

    QVariantMap state;
    state.insert(QStringLiteral("motion_owner"), owner_role_to_string(motion_owner));
    state.insert(QStringLiteral("manual_override_active"), manual_override_active);
    state.insert(QStringLiteral("estop_active"), estop_active);
    const bool hexamotion_owns = (control_source == hal_host_service::MotionControlSource::HexaMotion);
    state.insert(QStringLiteral("control_source"),
                 hexamotion_owns ? QStringLiteral("hexamotion") : QStringLiteral("ui"));
    state.insert(QStringLiteral("hexamotion_ipc_running"), ipc_server_running);
    state.insert(QStringLiteral("hexamotion_connected_clients"), connected_hexamotion_client_count);

    emit hostStateUpdated(state);
}

bool AxisManager::isReady() const {
    const auto listed = listAxes();
    return listed.ok() && !listed.value().empty();
}

motion_core::Result<std::shared_ptr<motion_core::IAxis>> AxisManager::findAxis(const std::uint16_t axis_id) const {
    return unified_runtime_.find_axis(axis_id);
}

motion_core::AxisTransportKind AxisManager::getAxisTransport(const int axis_id) const {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return motion_core::AxisTransportKind::Unknown;
    }
    return axis_res.value()->info().transport;
}

motion_core::Result<std::vector<motion_core::AxisInfo>> AxisManager::listAxes() const {
    return unified_runtime_.list_axes();
}

motion_core::Result<std::string> AxisManager::executeStopAllAxes(const hal_ipc::OwnerRole caller) {
    const auto axes_res = listAxes();
    if (!axes_res.ok()) {
        return motion_core::Result<std::string>::failure(axes_res.error());
    }

    const hal_ipc::HalControlFrameDto empty_frame{};
    for (const auto& axis_info : axes_res.value()) {
        const auto stop_res = executeAxisOperation(
            caller,
            hal_ipc::ControlOp::Stop,
            axis_info.id.value,
            nullptr,
            empty_frame);
        if (!stop_res.ok()) {
            return stop_res;
        }

        const auto clear_res = executeAxisOperation(
            caller,
            hal_ipc::ControlOp::ClearMotionQueue,
            axis_info.id.value,
            nullptr,
            empty_frame);
        if (!clear_res.ok()) {
            return clear_res;
        }
    }

    return motion_core::Result<std::string>::success("");
}

motion_core::Result<std::string> AxisManager::executeAxisOperation(const hal_ipc::OwnerRole caller,
                                                                   const hal_ipc::ControlOp op,
                                                                   const std::uint16_t axis_id,
                                                                   const hal_ipc::AxisPointDto* point,
                                                                   const hal_ipc::HalControlFrameDto& frame) {
    bool estop_active = false;
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        estop_active = estop_active_;
    }

    if (estop_active) {
        const bool op_allowed_during_estop =
            (op == hal_ipc::ControlOp::Stop)
            || (op == hal_ipc::ControlOp::ClearFault)
            || (op == hal_ipc::ControlOp::ClearMotionQueue)
            || (op == hal_ipc::ControlOp::QueryMotionQueueStats)
            || (op == hal_ipc::ControlOp::None);
        if (!op_allowed_during_estop) {
            return motion_core::Result<std::string>::failure(
                {motion_core::ErrorCode::Busy,
                 "operation is blocked while global estop is active"});
        }
    }

    if (caller == hal_ipc::OwnerRole::HexaMotion && !is_hexamotion_stream_client_op(op)) {
        return motion_core::Result<std::string>::failure(
            {motion_core::ErrorCode::PermissionDenied,
             "operation is blocked for HexaMotion stream/read client"});
    }

    if (op == hal_ipc::ControlOp::Stop && axis_id == 0U) {
        return executeStopAllAxes(caller);
    }

    if (axis_id == 0U) {
        return motion_core::Result<std::string>::failure(
            {motion_core::ErrorCode::InvalidArgument, "axis_id must be > 0"});
    }

    const auto axis_res = findAxis(axis_id);
    if (!axis_res.ok()) {
        return motion_core::Result<std::string>::failure(axis_res.error());
    }

    const auto axis = axis_res.value();
    switch (op) {
        case hal_ipc::ControlOp::EnableAxis: {
            const auto r = axis->set_enabled(true);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::DisableAxis: {
            const auto r = axis->set_enabled(false);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::StreamPoint: {
            if (point == nullptr) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "stream point payload is required"});
            }
            if (!runtime_queue_ingress_) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
            }

            std::vector<motion_core::QueuedSetpoint> batch;
            batch.push_back(queued_setpoint_from_axis_point(*point));
            const auto submit_res = runtime_queue_ingress_->submit_motion_batch(
                motion_source_from_owner_role(caller),
                axis_id,
                batch);
            if (!submit_res.ok()) {
                return motion_core::Result<std::string>::failure(submit_res.error());
            }
            return motion_core::Result<std::string>::success(
                motion_queue_stats_to_json(submit_res.value()).dump());
        }

        case hal_ipc::ControlOp::Hold:
        case hal_ipc::ControlOp::SetZero:
        case hal_ipc::ControlOp::ClearFault:
        case hal_ipc::ControlOp::Home: {
            const auto cmd = build_axis_command(op, point);
            const auto r = axis->apply_command(cmd);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::Stop: {
            const auto cmd = build_axis_command(op, point);
            const auto stop_r = axis->apply_command(cmd);
            if (!stop_r.ok()) {
                return motion_core::Result<std::string>::failure(stop_r.error());
            }
            const auto disable_r = axis->set_enabled(false);
            return disable_r.ok() ? motion_core::Result<std::string>::success("")
                                  : motion_core::Result<std::string>::failure(disable_r.error());
        }

        case hal_ipc::ControlOp::SetAxisMode: {
            const auto r = axis->set_mode(static_cast<motion_core::AxisMode>(frame.service_int_value));
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::ConfigureMotionQueue: {
            if (frame.service_int_value < 0) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "queue capacity must be >= 0"});
            }
            const auto r = axis->configure_motion_queue(
                static_cast<std::size_t>(frame.service_int_value),
                frame.service_bool_value);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::ClearMotionQueue: {
            const auto r = axis->clear_motion_queue();
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::QueryMotionQueueStats: {
            if (!runtime_queue_ingress_) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
            }

            const auto r = runtime_queue_ingress_->query_motion_queue_stats_observed(axis_id);
            if (!r.ok()) {
                return motion_core::Result<std::string>::failure(r.error());
            }
            return motion_core::Result<std::string>::success(motion_queue_stats_to_json(r.value()).dump());
        }

        case hal_ipc::ControlOp::ReadParameters: {
            const auto r = axis->read_parameters();
            if (!r.ok()) {
                return motion_core::Result<std::string>::failure(r.error());
            }
            json j = json::array();
            for (const auto& entry : r.value().entries) {
                json e;
                e["domain"] = entry.id.domain;
                e["value"] = entry.id.value;
                json val;
                param_value_to_json(val, entry.value);
                e["data"] = val;
                j.push_back(std::move(e));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }

        case hal_ipc::ControlOp::ListParameters: {
            const auto r = axis->list_parameters();
            if (!r.ok()) {
                return motion_core::Result<std::string>::failure(r.error());
            }
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
                if (desc.has_min) {
                    json min_val;
                    param_value_to_json(min_val, desc.min_value);
                    d["min_value"] = min_val;
                }
                if (desc.has_max) {
                    json max_val;
                    param_value_to_json(max_val, desc.max_value);
                    d["max_value"] = max_val;
                }
                j.push_back(std::move(d));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }

        case hal_ipc::ControlOp::EnqueueMotionBatch: {
            if (!runtime_queue_ingress_) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
            }

            std::vector<motion_core::QueuedSetpoint> batch;
            if (!frame.service_string_value.empty()) {
                const auto parsed = json::parse(frame.service_string_value, nullptr, false);
                if (parsed.is_discarded() || !parsed.is_array()) {
                    return motion_core::Result<std::string>::failure(
                        {motion_core::ErrorCode::InvalidArgument, "batch must be array"});
                }

                for (const auto& pt : parsed) {
                    motion_core::QueuedSetpoint q{};
                    q.target_position_deg = pt.value("target_position_deg", 0.0);
                    if (pt.contains("has_profile_speed_rpm") && pt.value("has_profile_speed_rpm", false)) {
                        q.has_profile_speed_rpm = true;
                        q.profile_speed_rpm = pt.value("profile_speed_rpm", 0);
                    }
                    if (pt.contains("has_profile_accel_percent") && pt.value("has_profile_accel_percent", false)) {
                        q.has_profile_accel_percent = true;
                        q.profile_accel_percent = pt.value("profile_accel_percent", 0.0);
                    }
                    if (pt.contains("has_target_velocity") && pt.value("has_target_velocity", false)) {
                        q.has_target_velocity = true;
                        q.target_velocity_deg_per_sec = pt.value("target_velocity_deg_per_sec", 0.0);
                    }
                    if (pt.contains("sample_period_sec")) {
                        q.sample_period_sec = pt.value("sample_period_sec", 0.001);
                    }
                    batch.push_back(q);
                }
            }

            const auto submit_res = runtime_queue_ingress_->submit_motion_batch(
                motion_source_from_owner_role(caller),
                axis_id,
                batch);
            if (!submit_res.ok()) {
                return motion_core::Result<std::string>::failure(submit_res.error());
            }
            return motion_core::Result<std::string>::success(
                motion_queue_stats_to_json(submit_res.value()).dump());
        }

        case hal_ipc::ControlOp::ApplyParameterPatch: {
            if (frame.service_string_value.empty()) {
                return motion_core::Result<std::string>::success("");
            }
            const auto parsed = json::parse(frame.service_string_value, nullptr, false);
            if (parsed.is_discarded() || !parsed.is_array()) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "patch must be array"});
            }

            motion_core::ParameterPatch patch{};
            for (const auto& e : parsed) {
                motion_core::ParameterEntry entry{};
                entry.id.domain = static_cast<motion_core::ParameterDomain>(e.value("domain", 0U));
                entry.id.value = e.value("value", 0U);
                if (e.contains("data")) {
                    const auto pv = param_value_from_json(e["data"]);
                    if (!pv.ok()) {
                        return motion_core::Result<std::string>::failure(pv.error());
                    }
                    entry.value = pv.value();
                }
                patch.entries.push_back(entry);
            }

            const auto r = axis->apply_parameter_patch(patch);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::SetPersistent: {
            if (frame.service_string_value.empty()) {
                return motion_core::Result<std::string>::success("");
            }
            const auto parsed = json::parse(frame.service_string_value, nullptr, false);
            if (parsed.is_discarded() || !parsed.is_object()) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "persistent req must be object"});
            }

            motion_core::ParameterValue value{};
            if (parsed.contains("data")) {
                const auto pv = param_value_from_json(parsed["data"]);
                if (!pv.ok()) {
                    return motion_core::Result<std::string>::failure(pv.error());
                }
                value = pv.value();
            }

            const auto command = static_cast<motion_core::PersistentCommand>(parsed.value("command", 0));
            const auto r = axis->set_persistent(command, value);
            if (!r.ok()) {
                return motion_core::Result<std::string>::failure(r.error());
            }

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
            const auto r = unified_runtime_.build_axis_config_patch(axis_id, frame.service_string_value);
            if (!r.ok()) {
                return motion_core::Result<std::string>::failure(r.error());
            }

            json j = json::array();
            for (const auto& entry : r.value().entries) {
                json e;
                e["domain"] = entry.id.domain;
                e["value"] = entry.id.value;
                json data;
                param_value_to_json(data, entry.value);
                e["data"] = data;
                j.push_back(std::move(e));
            }
            return motion_core::Result<std::string>::success(j.dump());
        }

        case hal_ipc::ControlOp::ImportAxisConfig: {
            const auto r = unified_runtime_.apply_axis_config_file(axis_id, frame.service_string_value);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::ExportAxisConfig: {
            const auto r = unified_runtime_.export_axis_config_to_file(axis_id, frame.service_string_value);
            return r.ok() ? motion_core::Result<std::string>::success("")
                          : motion_core::Result<std::string>::failure(r.error());
        }

        case hal_ipc::ControlOp::None:
            return motion_core::Result<std::string>::success("");
    }

    return motion_core::Result<std::string>::success("");
}

void AxisManager::removeTransportConfig(const motion_core::AxisTransportKind transport) {
    if (transport == motion_core::AxisTransportKind::CanBus) {
        current_hal_config_.mks_buses.clear();
    } else if (transport == motion_core::AxisTransportKind::Ethercat) {
        current_hal_config_.ethercat_buses.clear();
    }

    current_hal_config_.axes.erase(
        std::remove_if(current_hal_config_.axes.begin(),
                       current_hal_config_.axes.end(),
                       [transport](const motion_core::HalAxisRuntimeEntry& axis) {
                           return axis.transport == transport;
                       }),
        current_hal_config_.axes.end());
}

void AxisManager::publishTopologySnapshot() {
    QVariantList mks_out;
    QVariantList ecat_out;

    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            if (info.transport == motion_core::AxisTransportKind::CanBus) {
                mks_out.push_back(static_cast<int>(info.id.value));
            } else if (info.transport == motion_core::AxisTransportKind::Ethercat) {
                ecat_out.push_back(static_cast<int>(info.id.value));
            }
        }
    }

    emit scanFinished(QStringLiteral("mks"), mks_out);
    emit scanFinished(QStringLiteral("ecat"), ecat_out);
}

void AxisManager::publishTransportOpenStates() {
    emit transportOpenStateChanged(QStringLiteral("mks"), mks_device_opened_);
    emit transportOpenStateChanged(QStringLiteral("ecat"), ethercat_device_opened_);
}

void AxisManager::syncOpenStateFromCurrentConfig() {
    mks_device_opened_ = !current_hal_config_.mks_buses.empty();
    ethercat_device_opened_ = !current_hal_config_.ethercat_buses.empty();

    if (mks_device_opened_) {
        opened_mks_device_path_ = QString::fromStdString(current_hal_config_.mks_buses.front().device_path);
        opened_mks_baud_rate_ = static_cast<int>(current_hal_config_.mks_buses.front().baud_rate);
    } else {
        opened_mks_device_path_.clear();
        opened_mks_baud_rate_ = 0;
    }

    if (ethercat_device_opened_) {
        opened_ethercat_interface_ =
            QString::fromStdString(current_hal_config_.ethercat_buses.front().interface_name);
    } else {
        opened_ethercat_interface_.clear();
    }
}

void AxisManager::rebuildRuntimeFromCurrentConfig() {
    if (fast_timer_) {
        fast_timer_->stop();
    }
    if (slow_timer_) {
        slow_timer_->stop();
    }
    watched_axes_.clear();
    ui_priority_axis_id_ = -1;
    rr_index_ = 0;
    runtime_started_axes_.clear();
    mks_axes_.clear();
    ethercat_axes_.clear();

    if (unified_runtime_.is_active()) {
        (void)unified_runtime_.stop();
    }
    if (unified_runtime_.is_open()) {
        (void)unified_runtime_.close();
    }

    if (current_hal_config_.axes.empty()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    const auto open_res = unified_runtime_.open_from_config(current_hal_config_);
    if (!open_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Unified runtime open failed: %1").arg(open_res.error().message));
    }

    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            const int id = static_cast<int>(info.id.value);
            if (info.transport == motion_core::AxisTransportKind::CanBus) {
                mks_axes_.insert(id);
            } else if (info.transport == motion_core::AxisTransportKind::Ethercat) {
                ethercat_axes_.insert(id);
            }
        }
    }

    if (mks_axes_.isEmpty() && ethercat_axes_.isEmpty()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    const auto start_res = startRuntimeHeadless();
    if (!start_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Runtime start failed after rebuild: %1").arg(start_res.error().message));
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    emit connectionChanged(!runtime_started_axes_.isEmpty());
    publishTopologySnapshot();
}

void AxisManager::applySafetyBaselineForAxis(const int axis_id, const QString& reason, const bool force_disable) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }
    auto axis = axis_res.value();
    const auto transport = axis->info().transport;
    const bool should_force_disable = force_disable
        && transport == motion_core::AxisTransportKind::Ethercat;
    if (should_force_disable) {
        hal_ipc::HalControlFrameDto frame{};
        const auto disable_res = executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::DisableAxis, static_cast<std::uint16_t>(axis_id), nullptr, frame);
        if (!disable_res.ok()) {
            emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 disable in safety baseline failed (%2): %3")
                                .arg(axis_id)
                                .arg(reason)
                                .arg(QString::fromStdString(disable_res.error().message)));
        }
    }

    const auto telem = axis->read_telemetry();
    if (telem.ok()) {
        hal_ipc::AxisPointDto point{};
        point.axis_id = static_cast<std::uint16_t>(axis_id);
        point.actual_position_deg = telem.value().actual_position_deg;
        hal_ipc::HalControlFrameDto frame{};
        (void)executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::Hold, static_cast<std::uint16_t>(axis_id), &point, frame);
    }
}

void AxisManager::rebuildTransportRuntime(const motion_core::AxisTransportKind transport) {
    Q_UNUSED(transport);
    // Since we now use a unified runtime, rebuilding a transport runtime is equivalent to rebuilding
    // the entire runtime with the updated master config.
    
    // Remember which axes were being watched so we can restore after rebuild.
    const QSet<int> prev_watched = watched_axes_;
    
    rebuildRuntimeFromCurrentConfig();
    
    // Restore watched axes
    for (int id : prev_watched) {
        if (mks_axes_.contains(id) || ethercat_axes_.contains(id)) {
            watched_axes_.insert(id);
            if (ui_priority_axis_id_ == -1) {
                ui_priority_axis_id_ = id;
            }
        }
    }
}



void AxisManager::reset_runtime_state() {
    runtime_started_axes_.clear();
    mks_axes_.clear();
    ethercat_axes_.clear();
    ui_priority_axis_id_ = -1;
    if (unified_runtime_.is_open()) (void)unified_runtime_.close();
}

motion_core::Result<void> AxisManager::startRuntimeHeadless() {
    if (unified_runtime_.is_open()) {
        const auto start_result = unified_runtime_.start();
        if (!start_result.ok()) {
            return start_result;
        }
    }

    runtime_started_axes_.clear();
    for (const int axis_id : mks_axes_) {
        runtime_started_axes_.insert(axis_id);
        applySafetyBaselineForAxis(axis_id, "startRuntimeHeadless", true);
    }
    for (const int axis_id : ethercat_axes_) {
        runtime_started_axes_.insert(axis_id);
        applySafetyBaselineForAxis(axis_id, "startRuntimeHeadless", true);
    }
    watched_axes_.clear();
    rr_index_ = 0;
    if (!runtime_started_axes_.isEmpty()) {
        fast_timer_->start();
        slow_timer_->start();

        // Bug 2 fix: start IPC server so HexaMotion can connect
        if (host_service_ && !host_service_->is_running()) {
            hal_host_service::HalHostServiceConfig ipc_cfg{};
            const auto ipc_res = host_service_->start(ipc_cfg);
            if (ipc_res.ok()) {
                emit logMessage(QStringLiteral("hal"),
                                QStringLiteral("IPC server listening on %1:%2")
                                    .arg(QString::fromStdString(ipc_cfg.bind_host))
                                    .arg(ipc_cfg.port));
            } else {
                emit logMessage(QStringLiteral("hal"),
                                QStringLiteral("IPC server start failed: %1")
                                    .arg(QString::fromStdString(ipc_res.error().message)));
            }
        }

    }
    return motion_core::Result<void>::success();
}

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    if (device_path.isEmpty() || baud_rate <= 0) {
        emit logMessage("mks", "Open failed: invalid device path or baud rate");
        emit connectionChanged(false);
        return;
    }
    opened_mks_device_path_ = device_path;
    opened_mks_baud_rate_ = baud_rate;
    mks_device_opened_ = true;
    emit logMessage("mks",QString("Device opened: %1 @ %2").arg(device_path).arg(baud_rate));
    emit transportOpenStateChanged(QStringLiteral("mks"), true);
}

void AxisManager::openEthercatDevice(const QString& interface_name) {
    if (interface_name.isEmpty()) {
        emit logMessage("ecat", "EtherCAT open failed: interface name is empty");
        emit connectionChanged(false);
        return;
    }
    opened_ethercat_interface_ = interface_name;
    ethercat_device_opened_ = true;
    emit logMessage("ecat",QString("EtherCAT interface opened: %1").arg(interface_name));
    emit transportOpenStateChanged(QStringLiteral("ecat"), true);
}

void AxisManager::closeMksDevice() {
    mks_device_opened_ = false;
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    removeTransportConfig(motion_core::AxisTransportKind::CanBus);
    emit transportOpenStateChanged(QStringLiteral("mks"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
}

void AxisManager::closeEthercatDevice() {
    ethercat_device_opened_ = false;
    opened_ethercat_interface_.clear();
    removeTransportConfig(motion_core::AxisTransportKind::Ethercat);
    emit transportOpenStateChanged(QStringLiteral("ecat"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
}

void AxisManager::closeDevice() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    // Stop IPC server on shutdown
    if (host_service_ && host_service_->is_running()) {
        (void)host_service_->stop();
    }
    watched_axes_.clear();
    ui_priority_axis_id_ = -1;
    rr_index_ = 0;
    reset_runtime_state();
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    mks_device_opened_ = false;
    opened_ethercat_interface_.clear();
    ethercat_device_opened_ = false;
    current_hal_config_ = {};
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        control_source_ = hal_host_service::MotionControlSource::Ui;
        estop_active_ = false;
    }
    publishHostState();
    emit connectionChanged(false);
    publishTransportOpenStates();
    publishTopologySnapshot();
    emit logMessage("hal","Device/runtime closed");
}

void AxisManager::loadHalConfig(const QString& config_path) {
    const auto hal_cfg = motion_core::load_hal_runtime_config_from_file(config_path.toStdString());
    if (!hal_cfg.ok()) {
        emit logMessage("hal",QString("HAL config load failed: %1").arg(hal_cfg.error().message));
        return;
    }
    reset_runtime_state();
    current_hal_config_ = hal_cfg.value();
    syncOpenStateFromCurrentConfig();
    publishTransportOpenStates();
    rebuildRuntimeFromCurrentConfig();
    emit logMessage("hal",QString("HAL config loaded: total=%1 (MKS=%2, EtherCAT=%3)")
                        .arg(current_hal_config_.axes.size())
                        .arg(current_hal_config_.mks_buses.size())
                        .arg(current_hal_config_.ethercat_buses.size()));
}

void AxisManager::saveHalConfig(const QString& config_path) {
    current_hal_config_ = unified_runtime_.current_config();
    const auto res = motion_core::save_hal_runtime_config_to_file(config_path.toStdString(), current_hal_config_);
    if (!res.ok()) {
        emit logMessage("hal",QString("Configuration save failed: %1").arg(res.error().message));
    } else {
        emit logMessage("hal",QString("Saved master configuration to %1").arg(config_path));
    }
}

void AxisManager::startRuntime() {
    if (!isReady()) {
        emit logMessage("hal","Runtime start failed: runtime is not open");
        emit connectionChanged(false);
        return;
    }
    const auto start_result = startRuntimeHeadless();
    if (!start_result.ok()) {
        emit logMessage("hal",QString("Runtime start failed: %1").arg(start_result.error().message));
        emit connectionChanged(false);
        return;
    }
    emit connectionChanged(!runtime_started_axes_.isEmpty());
    emit logMessage("hal",QString("Runtime started: %1 axis(es)").arg(runtime_started_axes_.size()));
}

void AxisManager::stopRuntime() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    if (host_service_ && host_service_->is_running()) {
        (void)host_service_->stop();
    }
    watched_axes_.clear();
    rr_index_ = 0;
    if (unified_runtime_.is_active()) (void)unified_runtime_.stop();
    runtime_started_axes_.clear();
    publishHostState();
    emit connectionChanged(false);
    emit logMessage("hal","Runtime stopped");
}

void AxisManager::scanMotors(int max_id) {
    if (!mks_device_opened_) {
        emit logMessage("mks","Scan failed: open device first");
        return;
    }

    motion_core::MksScanRequest request{};
    request.device_path = opened_mks_device_path_.toStdString();
    request.baud_rate = static_cast<std::uint32_t>(opened_mks_baud_rate_);
    request.max_id = max_id;

    const auto discovered = unified_runtime_.scan_mks_topology(request);
    if (!discovered.ok()) {
        emit logMessage("mks",QString("Scan failed via HalRuntime: %1").arg(discovered.error().message));
        return;
    }

    motion_core::HalRuntimeConfig mks_hal_cfg = discovered.value();
    removeTransportConfig(motion_core::AxisTransportKind::CanBus);
    current_hal_config_.mks_buses.insert(current_hal_config_.mks_buses.end(), mks_hal_cfg.mks_buses.begin(), mks_hal_cfg.mks_buses.end());
    current_hal_config_.axes.insert(current_hal_config_.axes.end(), mks_hal_cfg.axes.begin(), mks_hal_cfg.axes.end());
    rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
}

void AxisManager::scanEthercatMotors() {
    if (!ethercat_device_opened_) {
        emit logMessage("ecat","EtherCAT scan failed: open EtherCAT interface first");
        return;
    }

    motion_core::EthercatScanRequest request{};
    request.interface_name = opened_ethercat_interface_.toStdString();
    const auto discovered = unified_runtime_.scan_ethercat_topology(request);
    if (!discovered.ok()) {
        emit logMessage("ecat",QString("EtherCAT scan failed via HalRuntime: %1").arg(discovered.error().message));
        return;
    }

    motion_core::HalRuntimeConfig ecat_hal_cfg = discovered.value();
    removeTransportConfig(motion_core::AxisTransportKind::Ethercat);
    current_hal_config_.ethercat_buses.insert(current_hal_config_.ethercat_buses.end(), ecat_hal_cfg.ethercat_buses.begin(), ecat_hal_cfg.ethercat_buses.end());
    current_hal_config_.axes.insert(current_hal_config_.axes.end(), ecat_hal_cfg.axes.begin(), ecat_hal_cfg.axes.end());

    rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
}

void AxisManager::watchAxis(int axis_id, bool enabled) {
    if (axis_id < 1 || axis_id > 2047) return;
    if (enabled) {
        watched_axes_.insert(axis_id);
        ui_priority_axis_id_ = axis_id;
        return;
    }

    watched_axes_.remove(axis_id);
    if (ui_priority_axis_id_ == axis_id) {
        if (watched_axes_.isEmpty()) {
            ui_priority_axis_id_ = -1;
        } else {
            ui_priority_axis_id_ = *watched_axes_.cbegin();
        }
    }
}

void AxisManager::enableMotor(int axis_id, bool enabled) {
    const auto op = enabled ? hal_ipc::ControlOp::EnableAxis : hal_ipc::ControlOp::DisableAxis;
    hal_ipc::HalControlFrameDto frame{};
    const auto result = executeAxisOperation(
        hal_ipc::OwnerRole::MotorTesterUi,
        op,
        static_cast<std::uint16_t>(axis_id),
        nullptr,
        frame);
    if (!result.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 enable=%2 failed: %3")
                            .arg(axis_id)
                            .arg(enabled ? 1 : 0)
                            .arg(QString::fromStdString(result.error().message)));
    }
}

void AxisManager::emergencyStop(int axis_id) {
    Q_UNUSED(axis_id);
    const auto stop_res = executeStopAllAxes(hal_ipc::OwnerRole::MotorTesterUi);
    if (!stop_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Global E-STOP failed: %1")
                            .arg(QString::fromStdString(stop_res.error().message)));
        return;
    }
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        estop_active_ = true;
    }
    publishHostState();
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP activated — all axes stopped"));
}

void AxisManager::clearErrors(int axis_id) {
    hal_ipc::HalControlFrameDto frame{};
    const auto result = executeAxisOperation(
        hal_ipc::OwnerRole::MotorTesterUi,
        hal_ipc::ControlOp::ClearFault,
        static_cast<std::uint16_t>(axis_id),
        nullptr,
        frame);
    if (!result.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 clear errors failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
        return;
    }
    // Global E-STOP latch must not be implicitly reset by per-axis clear.
    // Reset policy is global and should be handled explicitly by operator flow.
    applySafetyBaselineForAxis(axis_id, "clear_errors", false);
}

void AxisManager::moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage("hal", QString("Axis %1 absolute move failed: axis not found").arg(axis_id));
        return;
    }

    const auto queue_ready_res = ensureManualMotionQueueReady(axis_id);
    if (!queue_ready_res.ok()) {
        const auto axis = axis_res.value();
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 absolute move failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(queue_ready_res.error().message)));
        return;
    }

    motion_core::QueuedSetpoint point{};
    point.target_position_deg = axis_deg;
    point.has_profile_speed_rpm = true;
    point.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    point.has_profile_accel_percent = axis_res.value()->info().transport == motion_core::AxisTransportKind::CanBus;
    point.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    point.sample_period_sec = 0.005;

    const auto res = runtime_queue_ingress_
        ? runtime_queue_ingress_->submit_motion_batch(
              hal_host_service::MotionControlSource::Ui,
              static_cast<std::uint16_t>(axis_id),
              std::vector<motion_core::QueuedSetpoint>{point})
        : motion_core::Result<motion_core::MotionQueueStats>::failure(
              {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
    if (!res.ok()) {
        const auto axis_res_inner = findAxis(axis_id);
        const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 absolute move failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(res.error().message)));
    }
}

void AxisManager::moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;

    const auto queue_ready_res = ensureManualMotionQueueReady(axis_id);
    if (!queue_ready_res.ok()) {
        const auto axis = axis_res.value();
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 relative move failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(queue_ready_res.error().message)));
        return;
    }

    const auto telem_res = axis_res.value()->read_telemetry();
    if (!telem_res.ok()) {
        const auto axis_res_inner = findAxis(axis_id);
        const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 relative move failed: %2")
                            .arg(axis_id)
                            .arg(telem_res.error().message));
        return;
    }

    const double absolute_target = telem_res.value().actual_position_deg + delta_deg;

    motion_core::QueuedSetpoint point{};
    point.target_position_deg = absolute_target;
    point.has_profile_speed_rpm = true;
    point.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    point.has_profile_accel_percent = axis_res.value()->info().transport == motion_core::AxisTransportKind::CanBus;
    point.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    point.sample_period_sec = 0.005;

    const auto res = runtime_queue_ingress_
        ? runtime_queue_ingress_->submit_motion_batch(
              hal_host_service::MotionControlSource::Ui,
              static_cast<std::uint16_t>(axis_id),
              std::vector<motion_core::QueuedSetpoint>{point})
        : motion_core::Result<motion_core::MotionQueueStats>::failure(
              {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
    if (!res.ok()) {
        const auto axis_res_inner = findAxis(axis_id);
        const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 relative move failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(res.error().message)));
    }
}
void AxisManager::configureMotionQueue(int axis_id, int capacity, bool drop_oldest) {
    hal_ipc::HalControlFrameDto frame{};
    frame.service_int_value = capacity;
    frame.service_bool_value = drop_oldest;
    const auto result = executeAxisOperation(
        hal_ipc::OwnerRole::MotorTesterUi,
        hal_ipc::ControlOp::ConfigureMotionQueue,
        static_cast<std::uint16_t>(axis_id),
        nullptr,
        frame);
    if (!result.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 configure motion queue failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
    }
}

void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    std::vector<motion_core::QueuedSetpoint> batch;
    batch.reserve(static_cast<std::size_t>(points.size()));
    for (const auto& v : points) {
        batch.push_back(queued_setpoint_from_variant_map(v.toMap()));
    }
    if (!runtime_queue_ingress_) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 enqueue batch failed: runtime queue ingress is not configured")
                            .arg(axis_id));
        return;
    }
    const auto submit_res = runtime_queue_ingress_->submit_motion_batch(
        hal_host_service::MotionControlSource::Ui,
        static_cast<std::uint16_t>(axis_id),
        batch);
    if (!submit_res.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 enqueue batch failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(submit_res.error().message)));
    }
}

motion_core::Result<motion_core::MotionQueueStats> AxisManager::enqueueMotionBatchDirect(
    int axis_id,
    const std::vector<motion_core::QueuedSetpoint>& points) {
    if (!runtime_queue_ingress_) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(
            {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
    }
    return runtime_queue_ingress_->submit_motion_batch(
        hal_host_service::MotionControlSource::Ui,
        static_cast<std::uint16_t>(axis_id),
        points);
}

motion_core::Result<motion_core::MotionQueueStats> AxisManager::queryMotionQueueStatsDirect(
    int axis_id) const {
    if (!runtime_queue_ingress_) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(
            {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
    }
    return runtime_queue_ingress_->query_motion_queue_stats_observed(
        static_cast<std::uint16_t>(axis_id));
}

void AxisManager::clearMotionQueue(int axis_id) {
    hal_ipc::HalControlFrameDto frame{};
    const auto result = executeAxisOperation(
        hal_ipc::OwnerRole::MotorTesterUi,
        hal_ipc::ControlOp::ClearMotionQueue,
        static_cast<std::uint16_t>(axis_id),
        nullptr,
        frame);
    if (!result.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 clear motion queue failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
    }
}

void AxisManager::requestMotionQueueStats(int axis_id) {
    const auto result = queryMotionQueueStatsDirect(axis_id);
    if (!result.ok()) {
        return;
    }
    QVariantMap stats;
    stats[QStringLiteral("size")] = static_cast<qulonglong>(result.value().size);
    stats[QStringLiteral("capacity")] = static_cast<qulonglong>(result.value().capacity);
    stats[QStringLiteral("pushed")] = static_cast<qulonglong>(result.value().pushed);
    stats[QStringLiteral("dropped")] = static_cast<qulonglong>(result.value().dropped);
    stats[QStringLiteral("underruns")] = static_cast<qulonglong>(result.value().underruns);
    stats[QStringLiteral("short_starts")] = static_cast<qulonglong>(result.value().short_starts);
    emit motionQueueStatsUpdated(axis_id, stats);
}

void AxisManager::setAxisMode(int axis_id, int mode_code) {
    hal_ipc::HalControlFrameDto frame{};
    frame.service_int_value = mode_code;
    (void)executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::SetAxisMode, static_cast<std::uint16_t>(axis_id), nullptr, frame);
}

void AxisManager::setZeroPosition(int axis_id) {
    hal_ipc::HalControlFrameDto frame{};
    (void)executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::SetZero, static_cast<std::uint16_t>(axis_id), nullptr, frame);
}

void AxisManager::goHome(int axis_id) {
    hal_ipc::HalControlFrameDto frame{};
    (void)executeAxisOperation(hal_ipc::OwnerRole::MotorTesterUi, hal_ipc::ControlOp::Home, static_cast<std::uint16_t>(axis_id), nullptr, frame);
}

void AxisManager::requestListParameters(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit parameterListReady(axis_id, {});
        return;
    }

    const auto descriptors = axis_res.value()->list_parameters();
    if (!descriptors.ok()) {
        emit parameterListReady(axis_id, {});
        return;
    }

    QVariantList out;
    out.reserve(static_cast<int>(descriptors.value().size()));
    for (const auto& descriptor : descriptors.value()) {
        QVariantMap item;
        item.insert(QStringLiteral("domain"), static_cast<int>(descriptor.id.domain));
        item.insert(QStringLiteral("value"), static_cast<int>(descriptor.id.value));
        item.insert(QStringLiteral("name"), QString::fromUtf8(descriptor.name ? descriptor.name : ""));
        item.insert(QStringLiteral("group"), QString::fromUtf8(descriptor.group ? descriptor.group : ""));
        item.insert(QStringLiteral("unit"), QString::fromUtf8(descriptor.unit ? descriptor.unit : ""));
        item.insert(QStringLiteral("read_only"), descriptor.read_only);
        item.insert(QStringLiteral("persistable"), descriptor.persistable);
        item.insert(QStringLiteral("has_min"), descriptor.has_min);
        item.insert(QStringLiteral("has_max"), descriptor.has_max);
        item.insert(QStringLiteral("min_value"), parameter_value_to_qvariant(descriptor.min_value));
        item.insert(QStringLiteral("max_value"), parameter_value_to_qvariant(descriptor.max_value));
        out.push_back(item);
    }
    emit parameterListReady(axis_id, out);
}

void AxisManager::requestReadParameters(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit parametersRead(axis_id, {});
        return;
    }

    const auto read_result = axis_res.value()->read_parameters();
    if (!read_result.ok()) {
        emit parametersRead(axis_id, {});
        return;
    }

    QVariantList out;
    out.reserve(static_cast<int>(read_result.value().entries.size()));
    for (const auto& entry : read_result.value().entries) {
        QVariantMap item;
        item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
        item.insert(QStringLiteral("value"), static_cast<int>(entry.id.value));
        item.insert(QStringLiteral("data"), parameter_value_to_string(entry.value));
        out.push_back(item);
    }
    emit parametersRead(axis_id, out);
}

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }

    motion_core::ParameterPatch typed_patch{};
    typed_patch.entries.reserve(static_cast<std::size_t>(patch.size()));
    for (const auto& v : patch) {
        const QVariantMap map = v.toMap();
        motion_core::ParameterEntry entry{};
        entry.id.domain = static_cast<motion_core::ParameterDomain>(map.value(QStringLiteral("domain")).toInt());
        entry.id.value = static_cast<std::uint32_t>(map.value(QStringLiteral("value")).toUInt());
        const auto value_res = parameter_value_from_variant(map.value(QStringLiteral("data")));
        if (!value_res.ok()) {
            emit logMessage(QStringLiteral("hal"),
                            QStringLiteral("Axis %1 apply patch failed: %2")
                                .arg(axis_id)
                                .arg(QString::fromStdString(value_res.error().message)));
            return;
        }
        entry.value = value_res.value();
        typed_patch.entries.push_back(entry);
    }

    const auto apply_result = axis_res.value()->apply_parameter_patch(typed_patch);
    if (!apply_result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 apply patch failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(apply_result.error().message)));
    }
}

void AxisManager::setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data) {
    Q_UNUSED(name);

    int command_code = -1;
    if (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
        && value == static_cast<int>(mks::MksParameter::CanId)) {
        command_code = static_cast<int>(motion_core::PersistentCommand::CanId);
    } else if (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
               && value == static_cast<int>(mks::MksParameter::CanBitrateIndex)) {
        command_code = static_cast<int>(motion_core::PersistentCommand::CanBitrate);
    } else if (domain == static_cast<int>(motion_core::ParameterDomain::Common)
               && value == static_cast<int>(motion_core::CommonParameter::MotorSelectionCode)) {
        command_code = static_cast<int>(motion_core::PersistentCommand::MotorType);
    }

    if (command_code < 0) {
        return;
    }

    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }

    const auto value_res = parameter_value_from_variant(data);
    if (!value_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 set persistent failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(value_res.error().message)));
        return;
    }

    const auto command = static_cast<motion_core::PersistentCommand>(command_code);
    const auto persistent_result = axis_res.value()->set_persistent(command, value_res.value());
    if (!persistent_result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 set persistent failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(persistent_result.error().message)));
    }
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    const auto result = unified_runtime_.export_axis_config_to_file(
        static_cast<std::uint16_t>(axis_id),
        path.toStdString());
    if (!result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 export config failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
    }
}

void AxisManager::importAxisConfigPreview(int axis_id, const QString& path) {
    const auto patch_result = unified_runtime_.build_axis_config_patch(
        static_cast<std::uint16_t>(axis_id),
        path.toStdString());
    if (!patch_result.ok()) {
        emit axisConfigPreviewReady(axis_id, {});
        return;
    }

    QVariantList out;
    out.reserve(static_cast<int>(patch_result.value().entries.size()));
    for (const auto& entry : patch_result.value().entries) {
        QVariantMap item;
        item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
        item.insert(QStringLiteral("value"), static_cast<int>(entry.id.value));
        item.insert(QStringLiteral("data"), parameter_value_to_string(entry.value));
        out.push_back(item);
    }
    emit axisConfigPreviewReady(axis_id, out);
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    const auto result = unified_runtime_.apply_axis_config_file(
        static_cast<std::uint16_t>(axis_id),
        path.toStdString());
    if (!result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis %1 import config failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
    }
}

void AxisManager::resetEmergencyStop() {
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        estop_active_ = false;
    }
    publishHostState();
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP reset by operator"));
}

void AxisManager::setControlSource(const hal_host_service::MotionControlSource source) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (control_source_ != source) {
            control_source_ = source;
            changed = true;
        }
    }
    if (!changed) {
        return;
    }

    emit manualTakeoverChanged(source == hal_host_service::MotionControlSource::Ui);
    publishHostState();
}

void AxisManager::onFastTick() {
    if (runtime_started_axes_.isEmpty()) return;

    // cycle_hz is cached in slow_timer_ (1 Hz) to avoid mutex + vector copy on every fast tick.
    const double cycle_hz = cached_cycle_hz_;
    // Bug 5 fix: publishHostState() moved to slow_timer_ (1 Hz) to avoid 250 Hz widget thrashing.

    for (int axis_id : watched_axes_) {
        auto axis_res = findAxis(axis_id);
        if (!axis_res.ok()) {
            continue;
        }

        const auto& axis = axis_res.value();

        // Emit motion queue stats through the shared ingress so UI polling and HAL IPC
        // observe the same guard + runtime access path.
        const auto stats_res = queryMotionQueueStatsDirect(axis_id);
        if (stats_res.ok()) {
            const auto& qs = stats_res.value();
            QVariantMap stats_map;
            stats_map[QStringLiteral("size")]         = static_cast<qulonglong>(qs.size);
            stats_map[QStringLiteral("capacity")]     = static_cast<qulonglong>(qs.capacity);
            stats_map[QStringLiteral("pushed")]       = static_cast<qulonglong>(qs.pushed);
            stats_map[QStringLiteral("dropped")]      = static_cast<qulonglong>(qs.dropped);
            stats_map[QStringLiteral("underruns")]    = static_cast<qulonglong>(qs.underruns);
            stats_map[QStringLiteral("short_starts")] = static_cast<qulonglong>(qs.short_starts);
            emit motionQueueStatsUpdated(axis_id, stats_map);
        }

        auto telem = axis->read_telemetry();
        if (telem.ok()) {
            QVariantMap t_map;
            t_map["transport"] = transport_tag_for_axis(axis);
            t_map["actual_position_deg"] = telem.value().actual_position_deg;
            t_map["raw_axis_position"] = static_cast<qlonglong>(telem.value().raw_axis_position);
            t_map["actual_velocity_deg_per_sec"] = telem.value().actual_velocity_deg_per_sec;
            t_map["target_position_deg"] = telem.value().target_position_deg;
            t_map["actual_torque_percent"] = telem.value().actual_torque_percent;
            t_map["digital_inputs"] = static_cast<qulonglong>(telem.value().digital_inputs);
            t_map["mode"] = static_cast<int>(telem.value().mode);
            t_map["state"] = static_cast<int>(telem.value().state);
            t_map["status_word"] = static_cast<int>(telem.value().status_word);
            t_map["protection_code"] = static_cast<int>(telem.value().protection_code);
            t_map["motion_status"] = static_cast<int>(telem.value().motion_status_code);
            // NOTE: AxisTelemetry has no separate error_code field.
            // For EtherCAT, protection_code holds the PDO error_code (0x603F).
            // For MKS, protection_code is the MKS protection state.
            t_map["error_code"] = static_cast<int>(telem.value().protection_code);
            t_map["timestamp_ns"] = static_cast<qulonglong>(telem.value().timestamp_ns);

            // Inject real per-axis cycle metrics for MKS, bus-level stats for EtherCAT.
            const auto transport = axis->info().transport;
            if (transport == motion_core::AxisTransportKind::CanBus) {
                // MKS: read real per-axis CycleMetrics from the adapter
                auto* mks_adapter = dynamic_cast<mks::MksAxisAdapter*>(axis.get());
                if (mks_adapter) {
                    const auto metrics = mks_adapter->cycle_metrics_snapshot();
                    t_map["cmd_tx_hz"]            = metrics.command_tx.rate_hz;
                    t_map["cmd_tx_period_ms"]     = metrics.command_tx.last_period_ms;
                    t_map["telemetry_publish_hz"]        = metrics.telemetry_publish.rate_hz;
                    t_map["telemetry_publish_period_ms"] = metrics.telemetry_publish.last_period_ms;
                    t_map["position_rx_hz"]        = metrics.position_rx.rate_hz;
                    t_map["position_rx_period_ms"] = metrics.position_rx.last_period_ms;
                    t_map["speed_rx_hz"]           = metrics.speed_rx.rate_hz;
                    t_map["status_rx_hz"]          = metrics.status_rx.rate_hz;
                    t_map["protection_rx_hz"]      = metrics.protection_rx.rate_hz;
                }
            } else if (cycle_hz > 0.0) {
                // EtherCAT: bus-level cycle rate applies uniformly to all axes on the bus
                const double cycle_period_ms = 1000.0 / cycle_hz;
                t_map["cmd_tx_hz"]            = cycle_hz;
                t_map["cmd_tx_period_ms"]     = cycle_period_ms;
                t_map["telemetry_publish_hz"]        = cycle_hz;
                t_map["telemetry_publish_period_ms"] = cycle_period_ms;
                t_map["position_rx_hz"]        = cycle_hz;
                t_map["position_rx_period_ms"] = cycle_period_ms;
                t_map["speed_rx_hz"]           = cycle_hz;
                t_map["status_rx_hz"]          = cycle_hz;
                t_map["protection_rx_hz"]      = cycle_hz;
            }

            emit telemetryUpdated(axis_id, t_map);
        }
    }
}


} // namespace mks
