#include "mks/axis_manager/axis_motion_controller.h"
#include "mks/axis_manager/parameter_ui_utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

namespace {
[[nodiscard]] bool should_emit_throttled_axis_log(const int axis_id, const std::chrono::milliseconds period) {
    static std::unordered_map<int, std::chrono::steady_clock::time_point> next_allowed_log_time{};
    const auto now = std::chrono::steady_clock::now();
    const auto it = next_allowed_log_time.find(axis_id);
    if (it != next_allowed_log_time.end() && now < it->second) {
        return false;
    }
    next_allowed_log_time[axis_id] = now + period;
    return true;
}
} // namespace

namespace mks {

AxisMotionController::AxisMotionController(
    motion_core::HalRuntime& runtime,
    motion_core::MotionOrchestrator& orchestrator,
    motion_core::TrajectoryGenerator& trajectory_generator,
    QObject* parent)
    : QObject(parent)
    , unified_runtime_(runtime)
    , orchestrator_(orchestrator)
    , trajectory_generator_(trajectory_generator)
{}

void AxisMotionController::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    if (points.isEmpty()) return;

    const auto axis_result = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_result.ok()) {
        emit logMessage(QStringLiteral("hal"), QString("Axis %1 motion enqueue failed: axis not found").arg(axis_id));
        return;
    }

    for (const QVariant& v : points) {
        const QVariantMap map = v.toMap();
        const int kc = map.value(QStringLiteral("kind"), 0).toInt();
        motion_core::MotionCommandType command_type;
        if (kc == static_cast<int>(MotionPointKind::Velocity)) {
            command_type = motion_core::MotionCommandType::Velocity;
        } else if (kc == static_cast<int>(MotionPointKind::Stream)) {
            command_type = motion_core::MotionCommandType::Stream;
        } else {
            command_type = motion_core::MotionCommandType::Position;
        }

        const double target_pos = map.value(QStringLiteral("target_position_deg")).toDouble();
        const double target_vel = map.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
        const double profile_accel = map.value(QStringLiteral("profile_accel_percent")).toDouble();
        const double sample_period = map.value(QStringLiteral("sample_period_sec")).toDouble();
        const int profile_speed = map.value(QStringLiteral("profile_speed_rpm")).toInt();
        const bool is_relative = map.value(QStringLiteral("is_relative")).toBool();

        const auto pt = motion_core::build_motion_command_point(
            axis_id,
            command_type,
            target_pos,
            target_vel,
            profile_accel,
            sample_period,
            is_relative,
            profile_speed
        );

        const auto enqueue_res = orchestrator_.enqueue_motion(axis_id, pt);
        if (!enqueue_res.ok()) {
            if (should_emit_throttled_axis_log(axis_id, std::chrono::milliseconds(500))) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 motion enqueue failed: %2")
                                    .arg(axis_id)
                                    .arg(QString::fromStdString(enqueue_res.error().message)));
            }
            return;
        }
    }
}

void AxisMotionController::enqueueServiceBatch(int axis_id, const QVariantList& commands) {
    if (commands.isEmpty()) return;

    const auto axis_result = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_result.ok()) {
        emit logMessage(QStringLiteral("hal"), QString("Axis %1 service enqueue failed: axis not found").arg(axis_id));
        return;
    }

    for (const QVariant& v : commands) {
        const QVariantMap map = v.toMap();
        motion_core::ServiceCommandPoint command{};
        command.axis_id = axis_id;

        const int kind_value = map.value(QStringLiteral("kind"), 0).toInt();
        switch (static_cast<ServiceCommandKind>(kind_value)) {
            case ServiceCommandKind::Enable:           command.type = motion_core::ServiceCommandType::Enable; break;
            case ServiceCommandKind::Disable:          command.type = motion_core::ServiceCommandType::Disable; break;
            case ServiceCommandKind::ClearErrors:      command.type = motion_core::ServiceCommandType::ClearErrors; break;
            case ServiceCommandKind::Home:             command.type = motion_core::ServiceCommandType::Home; break;
            case ServiceCommandKind::SetZero:          command.type = motion_core::ServiceCommandType::SetZero; break;
            case ServiceCommandKind::ClearMotionQueue: command.type = motion_core::ServiceCommandType::ClearMotionQueue; break;
            default:                                   command.type = motion_core::ServiceCommandType::ResetDrive; break;
        }

        if (command.type == motion_core::ServiceCommandType::Home) {
            const double target_velocity_deg_s = map.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
            if (std::isfinite(target_velocity_deg_s) && target_velocity_deg_s > 0.0) {
                command.homing_speed_deg_per_sec = target_velocity_deg_s;
            } else {
                const int profile_speed_rpm = std::clamp(map.value(QStringLiteral("profile_speed_rpm")).toInt(), 0, 3000);
                if (profile_speed_rpm > 0) {
                    command.homing_speed_deg_per_sec = static_cast<double>(profile_speed_rpm) * 6.0;
                }
            }
        }

        const auto enqueue_res = orchestrator_.enqueue_service(axis_id, command);
        if (!enqueue_res.ok()) {
            emit logMessage(QStringLiteral("hal"),
                            QString("Axis %1 service enqueue failed: %2")
                                .arg(axis_id)
                                .arg(QString::fromStdString(enqueue_res.error().message)));
            return;
        }
    }
}

void AxisMotionController::emergencyStop(int axis_id) {
    Q_UNUSED(axis_id);
    const auto stop_res = orchestrator_.emergency_stop_all();
    if (!stop_res.ok()) {
        emit logMessage(QStringLiteral("hal"), QString("Global E-STOP failed: %1").arg(QString::fromStdString(stop_res.error().message)));
        return;
    }
    emit hostStateUpdated();
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP activated — all axes stopped"));
}

void AxisMotionController::startMksHomingSequence(const QList<int>& axis_ids) {
    std::vector<std::uint16_t> ids;
    ids.reserve(axis_ids.size());
    for (int id : axis_ids) {
        ids.push_back(static_cast<std::uint16_t>(id));
    }
    const auto res = orchestrator_.start_homing_sequence(ids);
    if (!res.ok()) {
        emit logMessage(QStringLiteral("mks"), QString("MKS homing sequence start rejected: %1").arg(QString::fromStdString(res.error().message)));
    }
}

void AxisMotionController::stopMksHomingSequence() {
    orchestrator_.stop_homing_sequence();
}

void AxisMotionController::startSineMode(int axis_id, const motion_core::SineConfig& config) {
    trajectory_generator_.start_sine(static_cast<std::uint16_t>(axis_id), config);
}

void AxisMotionController::stopSineMode(int axis_id) {
    trajectory_generator_.stop_sine(static_cast<std::uint16_t>(axis_id));
}

bool AxisMotionController::isSineModeActive(int axis_id) const {
    return trajectory_generator_.is_sine_active(static_cast<std::uint16_t>(axis_id));
}

} // namespace mks