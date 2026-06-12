#include "mks/axis_manager/axis_telemetry_controller.h"
#include "mks_can/mks_axis_adapter.h"
#include "mks/axis_manager/parameter_ui_utils.h"
#include <QMetaObject>

namespace {

[[nodiscard]] static QVariantMap motion_queue_stats_to_qvariant_map(const motion_core::MotionQueueStats& s) {
    QVariantMap stats;
    stats[QStringLiteral("size")]          = static_cast<qulonglong>(s.size);
    stats[QStringLiteral("capacity")]      = static_cast<qulonglong>(s.capacity);
    stats[QStringLiteral("pushed")]        = static_cast<qulonglong>(s.pushed);
    stats[QStringLiteral("dropped")]       = static_cast<qulonglong>(s.dropped);
    stats[QStringLiteral("underruns")]     = static_cast<qulonglong>(s.underruns);
    stats[QStringLiteral("short_starts")]  = static_cast<qulonglong>(s.short_starts);
    return stats;
}
} // namespace

namespace mks {

AxisTelemetryController::AxisTelemetryController(
    motion_core::HalRuntime& runtime,
    motion_core::TelemetryDistributor& telemetry,
    motion_core::ParameterManager& parameter_manager,
    QObject* parent)
    : QObject(parent)
    , unified_runtime_(runtime)
    , telemetry_(telemetry)
    , parameter_manager_(parameter_manager)
{
    telemetry_token_ = telemetry_.register_callback([this](const motion_core::TelemetryFrame& frame) {
        QVariantMap t;
        const auto& telem = frame.snapshot;
        const double target_position_deg = telem.has_target_position ? telem.target_position : telem.position;
        t[QStringLiteral("actual_position_deg")]       = telem.position;
        t[QStringLiteral("raw_axis_position")]         = static_cast<qlonglong>(0);
        t[QStringLiteral("actual_velocity_deg_per_sec")] = telem.velocity;
        t[QStringLiteral("target_position_deg")]       = target_position_deg;
        t[QStringLiteral("actual_torque_percent")]     = telem.current;
        t[QStringLiteral("mode")]                      = static_cast<int>(telem.mode);
        t[QStringLiteral("state")]                     = static_cast<int>(telem.state);
        t[QStringLiteral("status_word")]               = static_cast<int>(telem.status_word);
        t[QStringLiteral("protection_code")]           = static_cast<int>(telem.protection_code);
        t[QStringLiteral("motion_status")]             = static_cast<int>(telem.motion_status_code);
        t[QStringLiteral("error_code")]                = static_cast<int>(telem.protection_code);
        t[QStringLiteral("timestamp_ns")]              = static_cast<qulonglong>(telem.timestamp_us * 1000ULL);

        if (frame.transport == motion_core::AxisTransportKind::CanBus) {
            t[QStringLiteral("transport")] = QStringLiteral("MKS CAN");
            auto axis_res = unified_runtime_.find_axis(frame.axis_id);
            if (axis_res.ok()) {
                auto* mks_adapter = dynamic_cast<mks::MksAxisAdapter*>(axis_res.value().get());
                if (mks_adapter) {
                    const auto m = mks_adapter->cycle_metrics_snapshot();
                    t[QStringLiteral("mks_homing_sequence_status")] = QString::fromUtf8(mks_adapter->homing_status_text());
                    t[QStringLiteral("cmd_tx_hz")]                 = m.command_tx.rate_hz;
                    t[QStringLiteral("cmd_tx_period_ms")]          = m.command_tx.last_period_ms;
                    t[QStringLiteral("telemetry_publish_hz")]      = m.telemetry_publish.rate_hz;
                    t[QStringLiteral("telemetry_publish_period_ms")] = m.telemetry_publish.last_period_ms;
                    t[QStringLiteral("position_rx_hz")]            = m.position_rx.rate_hz;
                    t[QStringLiteral("position_rx_period_ms")]     = m.position_rx.last_period_ms;
                    t[QStringLiteral("speed_rx_hz")]               = m.speed_rx.rate_hz;
                    t[QStringLiteral("status_rx_hz")]              = m.status_rx.rate_hz;
                    t[QStringLiteral("protection_rx_hz")]          = m.protection_rx.rate_hz;

                    std::vector<mks::AxisPositionSample> position_samples;
                    position_samples.reserve(256U);
                    (void)mks_adapter->drain_position_samples(position_samples, 256U);
                    if (!position_samples.empty()) {
                        QVariantList sample_list;
                        sample_list.reserve(static_cast<int>(position_samples.size()));
                        for (const auto& sample : position_samples) {
                            QVariantMap sample_map;
                            sample_map[QStringLiteral("timestamp_ns")] = static_cast<qulonglong>(sample.timestamp_ns);
                            sample_map[QStringLiteral("position_deg")] = sample.position_deg;
                            sample_list.push_back(sample_map);
                        }
                        t[QStringLiteral("position_samples")] = sample_list;
                    }
                }
            }
        } else if (frame.transport == motion_core::AxisTransportKind::Ethercat) {
            t[QStringLiteral("transport")] = QStringLiteral("EtherCAT");
            t[QStringLiteral("digital_inputs")] = static_cast<qulonglong>(telem.digital_inputs);
            if (frame.cycle_hz > 0.0) {
                const double period_ms = 1000.0 / frame.cycle_hz;
                t[QStringLiteral("cmd_tx_hz")]           = frame.cycle_hz;
                t[QStringLiteral("cmd_tx_period_ms")]    = period_ms;
                t[QStringLiteral("telemetry_publish_hz")]        = frame.cycle_hz;
                t[QStringLiteral("telemetry_publish_period_ms")] = period_ms;
                t[QStringLiteral("position_rx_hz")]      = frame.cycle_hz;
                t[QStringLiteral("position_rx_period_ms")] = period_ms;
                t[QStringLiteral("speed_rx_hz")]         = frame.cycle_hz;
                t[QStringLiteral("status_rx_hz")]        = frame.cycle_hz;
                t[QStringLiteral("protection_rx_hz")]    = frame.cycle_hz;
            }
        } else {
            t[QStringLiteral("transport")] = QStringLiteral("Unknown");
        }

        emit telemetryUpdated(frame.axis_id, t);
        emit motionQueueStatsUpdated(frame.axis_id, motion_queue_stats_to_qvariant_map(frame.queue_stats));
    });
}

AxisTelemetryController::~AxisTelemetryController() {
    if (telemetry_token_ != 0) {
        telemetry_.unregister_callback(telemetry_token_);
    }
}

void AxisTelemetryController::requestListParameters(int axis_id) {
    const auto request_result = parameter_manager_.request_list_parameters(axis_id, [this, axis_id](motion_core::Result<std::vector<motion_core::ParameterDescriptor>> result) {
        QMetaObject::invokeMethod(this, [this, axis_id, result = std::move(result)]() mutable {
            if (!result.ok()) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 list parameters failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(result.error().message)));
                emit parameterListReady(axis_id, QVariantList{});
                return;
            }

            auto axis_ptr_res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
            const auto transport = axis_ptr_res.ok() ? axis_ptr_res.value()->info().transport : motion_core::AxisTransportKind::CanBus;

            QVariantList out;
            for (const auto& pd : result.value()) {
                QVariantMap item;
                item.insert(QStringLiteral("domain"),      static_cast<int>(pd.id.domain));
                item.insert(QStringLiteral("value"),       static_cast<int>(pd.id.value));
                item.insert(QStringLiteral("name"),        QString::fromLatin1(pd.name));
                item.insert(QStringLiteral("group"),       QString::fromStdString(motion_core::build_parameter_group_for_display(transport, pd)));
                item.insert(QStringLiteral("unit"),        QString::fromLatin1(pd.unit));
                item.insert(QStringLiteral("read_only"),   pd.read_only);
                item.insert(QStringLiteral("persistable"), pd.persistable);
                item.insert(QStringLiteral("has_min"),     pd.has_min);
                item.insert(QStringLiteral("has_max"),     pd.has_max);
                if (pd.has_min) item.insert(QStringLiteral("min_value"), param_value_to_string(pd.min_value));
                if (pd.has_max) item.insert(QStringLiteral("max_value"), param_value_to_string(pd.max_value));
                out.push_back(item);
            }
            emit parameterListReady(axis_id, out);
        }, Qt::QueuedConnection);
    });
    if (!request_result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 list parameters request rejected: %2")
                            .arg(axis_id).arg(QString::fromStdString(request_result.error().message)));
        emit parameterListReady(axis_id, QVariantList{});
    }
}

void AxisTelemetryController::requestReadParameters(int axis_id) {
    const auto request_result = parameter_manager_.request_read_parameters(axis_id, [this, axis_id](motion_core::Result<motion_core::ParameterSet> read_res) {
        QMetaObject::invokeMethod(this, [this, axis_id, read_res = std::move(read_res)]() mutable {
            if (!read_res.ok()) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 read parameters failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(read_res.error().message)));
                emit parametersRead(axis_id, QVariantList{});
                return;
            }

            QVariantList out;
            for (const auto& entry : read_res.value().entries) {
                QVariantMap item;
                item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
                item.insert(QStringLiteral("value"),  static_cast<int>(entry.id.value));
                item.insert(QStringLiteral("data"),   param_value_to_string(entry.value));
                out.push_back(item);
            }
            emit parametersRead(axis_id, out);
        }, Qt::QueuedConnection);
    });
    if (!request_result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 read parameters request rejected: %2")
                            .arg(axis_id).arg(QString::fromStdString(request_result.error().message)));
        emit parametersRead(axis_id, QVariantList{});
    }
}

void AxisTelemetryController::applyParameterPatch(int axis_id, const QVariantList& patch) {
    motion_core::ParameterPatch param_patch{};
    for (const QVariant& entry_v : patch) {
        const QVariantMap entry = entry_v.toMap();
        motion_core::ParameterEntry pe{};
        pe.id.domain = static_cast<motion_core::ParameterDomain>(entry.value(QStringLiteral("domain")).toInt());
        pe.id.value  = static_cast<std::uint32_t>(entry.value(QStringLiteral("value")).toInt());

        pe.value = qvariant_to_param_value(entry.value(QStringLiteral("data")));
        param_patch.entries.push_back(pe);
    }

    const auto param_patch_size = param_patch.entries.size();
    parameter_manager_.request_apply_patch(axis_id, std::move(param_patch), [this, axis_id, param_patch_size](motion_core::Result<void> write_res) {
        QMetaObject::invokeMethod(this, [this, axis_id, param_patch_size, write_res = std::move(write_res)]() mutable {
            if (!write_res.ok()) {
                const QString message = QString("Axis %1 parameter patch failed: %2")
                    .arg(axis_id).arg(QString::fromStdString(write_res.error().message));
                emit logMessage(QStringLiteral("hal"),
                                message);
                emit parameterPatchCompleted(axis_id, false, message);
            } else {
                const QString message = QString("Axis %1 parameter patch applied (%2 entries)")
                    .arg(axis_id).arg(param_patch_size);
                emit logMessage(QStringLiteral("hal"),
                                message);
                emit parameterPatchCompleted(axis_id, true, message);
            }
        }, Qt::QueuedConnection);
    });
}

void AxisTelemetryController::setPersistentParameter(int axis_id, int domain, int value, const QString&, const QVariant& data) {
    motion_core::ParameterEntry pe{};
    pe.id.domain = static_cast<motion_core::ParameterDomain>(domain);
    pe.id.value  = static_cast<std::uint32_t>(value);

    pe.value = qvariant_to_param_value(data);

    motion_core::ParameterPatch param_patch{};
    param_patch.entries.push_back(pe);

    parameter_manager_.request_apply_patch(axis_id, std::move(param_patch), [this, axis_id, domain, value](motion_core::Result<void> write_res) {
        QMetaObject::invokeMethod(this, [this, axis_id, domain, value, write_res = std::move(write_res)]() mutable {
            if (!write_res.ok()) {
                const QString message = QString("Axis %1 set persistent %2/%3 failed: %4")
                    .arg(axis_id).arg(domain).arg(value)
                    .arg(QString::fromStdString(write_res.error().message));
                emit logMessage(QStringLiteral("hal"),
                                message);
                emit persistentParameterCompleted(axis_id, false, message);
            } else {
                const QString message = QString("Axis %1 persistent parameter %2/%3 written")
                    .arg(axis_id).arg(domain).arg(value);
                emit logMessage(QStringLiteral("hal"), message);
                emit persistentParameterCompleted(axis_id, true, message);
            }
        }, Qt::QueuedConnection);
    });
}

void AxisTelemetryController::setWatchedAxes(const std::vector<uint16_t>& axis_ids) {
    telemetry_.set_watched_axes(axis_ids);
}

} // namespace mks