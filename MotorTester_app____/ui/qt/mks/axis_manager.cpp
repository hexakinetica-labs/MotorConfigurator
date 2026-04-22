#include "mks/axis_manager.h"

#include "ethercat/p100e_ethercat_dictionary.h"
#include "mks_can/adapter/mks_axis_adapter.h"
#include <QCoreApplication>
#include <QThreadPool>
#include <QRunnable>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "mks_can/dictionary/mks_dictionary.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <QMetaObject>
#include <QPointer>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {

constexpr std::size_t kMaxUiPositionSamplesPerTick = 512U;

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

[[nodiscard]] static QVariant decode_parameter_value_json(const QJsonValue& value_json) {
    if (!value_json.isObject()) {
        return {};
    }

    const QJsonObject object = value_json.toObject();
    const int type = object.value(QStringLiteral("type")).toInt();
    const QJsonValue raw_value = object.value(QStringLiteral("val"));

    switch (type) {
        case 0:
            return raw_value.toVariant().toLongLong();
        case 1:
            return raw_value.toVariant().toULongLong();
        case 2:
            return raw_value.toDouble();
        case 3:
            return raw_value.toBool();
        default:
            return raw_value.toVariant();
    }
}

[[nodiscard]] static QJsonObject encode_parameter_value_json(const QVariant& value) {
    QJsonObject object;
    switch (value.typeId()) {
        case QMetaType::Bool:
            object.insert(QStringLiteral("type"), 3);
            object.insert(QStringLiteral("val"), value.toBool());
            return object;
        case QMetaType::Float:
        case QMetaType::Double:
            object.insert(QStringLiteral("type"), 2);
            object.insert(QStringLiteral("val"), value.toDouble());
            return object;
        default:
            break;
    }

    bool integer_ok = false;
    const qlonglong integer_value = value.toLongLong(&integer_ok);
    if (integer_ok) {
        object.insert(QStringLiteral("type"), 0);
        object.insert(QStringLiteral("val"), integer_value);
        return object;
    }

    bool unsigned_ok = false;
    const qulonglong unsigned_value = value.toULongLong(&unsigned_ok);
    if (unsigned_ok) {
        object.insert(QStringLiteral("type"), 1);
        object.insert(QStringLiteral("val"), static_cast<qint64>(unsigned_value));
        return object;
    }

    object.insert(QStringLiteral("type"), 2);
    object.insert(QStringLiteral("val"), value.toDouble());
    return object;
}

[[nodiscard]] static QVariantList parse_parameter_list_payload(const std::string& payload) {
    QVariantList output;
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(payload));
    if (!document.isArray()) {
        return output;
    }

    for (const QJsonValue& value : document.array()) {
        if (!value.isObject()) {
            continue;
        }
        const QJsonObject object = value.toObject();
        QVariantMap item;
        item.insert(QStringLiteral("domain"), object.value(QStringLiteral("domain")).toInt());
        item.insert(QStringLiteral("value"), object.value(QStringLiteral("value")).toInt());
        item.insert(QStringLiteral("name"), object.value(QStringLiteral("name")).toString());
        item.insert(QStringLiteral("group"), object.value(QStringLiteral("group")).toString());
        item.insert(QStringLiteral("unit"), object.value(QStringLiteral("unit")).toString());
        item.insert(QStringLiteral("read_only"), object.value(QStringLiteral("read_only")).toBool());
        item.insert(QStringLiteral("persistable"), object.value(QStringLiteral("persistable")).toBool(true));
        item.insert(QStringLiteral("has_min"), object.value(QStringLiteral("has_min")).toBool());
        item.insert(QStringLiteral("has_max"), object.value(QStringLiteral("has_max")).toBool());
        item.insert(QStringLiteral("min_value"), decode_parameter_value_json(object.value(QStringLiteral("min_value"))));
        item.insert(QStringLiteral("max_value"), decode_parameter_value_json(object.value(QStringLiteral("max_value"))));
        output.push_back(item);
    }
    return output;
}

[[nodiscard]] static QVariantList parse_parameter_read_payload(const std::string& payload) {
    QVariantList output;
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(payload));
    if (!document.isArray()) {
        return output;
    }

    for (const QJsonValue& value : document.array()) {
        if (!value.isObject()) {
            continue;
        }
        const QJsonObject object = value.toObject();
        QVariantMap item;
        item.insert(QStringLiteral("domain"), object.value(QStringLiteral("domain")).toInt());
        item.insert(QStringLiteral("value"), object.value(QStringLiteral("value")).toInt());
        item.insert(QStringLiteral("data"), decode_parameter_value_json(object.value(QStringLiteral("data"))).toString());
        output.push_back(item);
    }
    return output;
}



[[nodiscard]] static QVariantMap parse_motion_queue_stats_payload(const std::string& payload) {
    QVariantMap output;
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(payload));
    if (!document.isObject()) {
        return output;
    }

    const QJsonObject object = document.object();
    for (auto it = object.begin(); it != object.end(); ++it) {
        output.insert(it.key(), it.value().toVariant());
    }
    return output;
}

[[nodiscard]] static QString owner_role_to_string(const hal_ipc::OwnerRole role) {
    switch (role) {
        case hal_ipc::OwnerRole::HexaMotion:
            return QStringLiteral("hexamotion");
        case hal_ipc::OwnerRole::MotorTesterUi:
            return QStringLiteral("ui");
        case hal_ipc::OwnerRole::Service:
            return QStringLiteral("service");
        case hal_ipc::OwnerRole::None:
        default:
            return QStringLiteral("none");
    }
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
}

AxisManager::~AxisManager() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

void AxisManager::publishHostState() {
    QVariantMap state;
    state.insert(QStringLiteral("motion_owner"), owner_role_to_string(motion_owner_));
    state.insert(QStringLiteral("service_owner"), owner_role_to_string(service_owner_));
    state.insert(QStringLiteral("motion_epoch"), static_cast<quint32>(motion_epoch_));
    state.insert(QStringLiteral("service_epoch"), static_cast<quint32>(service_epoch_));
    state.insert(QStringLiteral("manual_override_active"), manual_override_active_);
    state.insert(QStringLiteral("service_mode_active"), service_mode_active_);
    state.insert(QStringLiteral("estop_active"), estop_active_);
    // BUG-1 fix: default owner is UI.  Only explicit HexaMotion ownership
    // switches control_source to "hexamotion".  None == UI (operator default).
    const bool hexamotion_owns = (motion_owner_ == hal_ipc::OwnerRole::HexaMotion);
    state.insert(QStringLiteral("control_source"),
                 hexamotion_owns ? QStringLiteral("hexamotion") : QStringLiteral("ui"));

    // BUG-3 fix: publish IPC server state so HexaMotion tab shows real status.
    if (host_service_) {
        const auto snapshot = host_service_->state_snapshot();
        state.insert(QStringLiteral("hexamotion_ipc_running"), snapshot.ipc_server_running);
        state.insert(QStringLiteral("hexamotion_connected_clients"), snapshot.connected_client_count);
    } else {
        state.insert(QStringLiteral("hexamotion_ipc_running"), false);
        state.insert(QStringLiteral("hexamotion_connected_clients"), 0);
    }

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
    const auto service = getHostServiceForAxis(axis_id);
        const auto disable_res = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::DisableAxis);
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
        const auto service = getHostServiceForAxis(axis_id);
        (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::Hold, &point);
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


hal_host_service::HalHostService* AxisManager::getHostServiceForAxis(int axis_id) {
    Q_UNUSED(axis_id);
    return host_service_.get();
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
    motion_owner_ = hal_ipc::OwnerRole::None;
    service_owner_ = hal_ipc::OwnerRole::None;
    manual_override_active_ = false;
    service_mode_active_ = false;
    estop_active_ = false;
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
    watched_axes_.clear();
    rr_index_ = 0;
    if (unified_runtime_.is_active()) (void)unified_runtime_.stop();
    runtime_started_axes_.clear();
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
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, op);
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
    // Bug 3 fix: global E-Stop — stop ALL axes, not just the requested one.
    // Send a global Stop (axis_id <= 0) to HalHostService which iterates all axes.
    Q_UNUSED(axis_id);
    hal_ipc::HalControlFrameDto frame{};
    frame.seq = ++local_control_seq_;
    frame.client_id = hal_ipc::kClientIdMotorTesterUi;
    frame.op = hal_ipc::ControlOp::Stop;
    frame.service_axis_id = -1; // global stop

    if (!host_service_) {
        emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP failed: no HAL service"));
        return;
    }
    const auto state = host_service_->execute_local_command(frame);
    motion_owner_ = state.motion_owner;
    service_owner_ = state.service_owner;
    manual_override_active_ = state.manual_override_active;
    service_mode_active_ = state.service_mode_active;
    estop_active_ = state.estop_active;
    publishHostState();
    if (state.service_status_code != 0) {
        emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP returned error code %1").arg(state.service_status_code));
    } else {
        emit logMessage(QStringLiteral("hal"), QStringLiteral("Global E-STOP activated — all axes stopped"));
    }
}

void AxisManager::clearErrors(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ClearFault);
    if (!result.ok()) {
        const auto axis_res = findAxis(axis_id);
        const auto axis = axis_res.ok() ? axis_res.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 clear errors failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(result.error().message)));
        return;
    }
    applySafetyBaselineForAxis(axis_id, "clear_errors", false);
}

void AxisManager::moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage("hal", QString("Axis %1 absolute move failed: axis not found").arg(axis_id));
        return;
    }

    const auto transport = axis_res.value()->info().transport;
    if (transport == motion_core::AxisTransportKind::Ethercat) {
        const auto telem_res = axis_res.value()->read_telemetry();
        const auto mode = telem_res.ok() ? telem_res.value().mode : motion_core::AxisMode::ProfilePosition;
        if (mode == motion_core::AxisMode::ProfilePosition || mode == motion_core::AxisMode::Homing) {
            hal_ipc::AxisPointDto point{};
            point.axis_id = static_cast<std::uint16_t>(axis_id);
            point.has_interpolated_position = true;
            point.interpolated_position_deg = axis_deg;
            const auto service = getHostServiceForAxis(axis_id);
            const auto direct_res = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::StreamPoint, &point);
            if (!direct_res.ok()) {
                const auto axis_res_inner = findAxis(axis_id);
                const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
                emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 absolute move failed: %2")
                                    .arg(axis_id)
                                    .arg(QString::fromStdString(direct_res.error().message)));
            }
            return;
        }
    }

    QJsonObject point;
    point["target_position_deg"] = axis_deg;
    point["has_profile_speed_rpm"] = true;
    point["profile_speed_rpm"] = std::clamp(speed, 0, 3000);
    point["has_profile_accel_percent"] = true;
    point["profile_accel_percent"] = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    point["has_target_velocity"] = false;
    point["target_velocity_deg_per_sec"] = 0.0;
    point["sample_period_sec"] = 0.005;

    QJsonArray payload;
    payload.push_back(point);
    const QJsonDocument payload_doc(payload);
    const auto service = getHostServiceForAxis(axis_id);
    const auto res = applyCommandLocally(
        service,
        axis_id,
        hal_ipc::ControlOp::EnqueueMotionBatch,
        nullptr,
        payload_doc.toJson(QJsonDocument::Compact).toStdString());
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
    const auto transport = axis_res.value()->info().transport;
    if (transport == motion_core::AxisTransportKind::Ethercat) {
        const auto mode = telem_res.value().mode;
        if (mode == motion_core::AxisMode::ProfilePosition || mode == motion_core::AxisMode::Homing) {
            hal_ipc::AxisPointDto point{};
            point.axis_id = static_cast<std::uint16_t>(axis_id);
            point.has_interpolated_position = true;
            point.interpolated_position_deg = absolute_target;
            const auto service = getHostServiceForAxis(axis_id);
            const auto direct_res = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::StreamPoint, &point);
            if (!direct_res.ok()) {
                const auto axis_res_inner = findAxis(axis_id);
                const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
                emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 relative move failed: %2")
                                    .arg(axis_id)
                                    .arg(QString::fromStdString(direct_res.error().message)));
            }
            return;
        }
    }

    QJsonObject point;
    point["target_position_deg"] = absolute_target;
    point["has_profile_speed_rpm"] = true;
    point["profile_speed_rpm"] = std::clamp(speed, 0, 3000);
    point["has_profile_accel_percent"] = true;
    point["profile_accel_percent"] = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    point["has_target_velocity"] = false;
    point["target_velocity_deg_per_sec"] = 0.0;
    point["sample_period_sec"] = 0.005;

    QJsonArray payload;
    payload.push_back(point);
    const QJsonDocument payload_doc(payload);
    const auto service = getHostServiceForAxis(axis_id);
    const auto res = applyCommandLocally(
        service,
        axis_id,
        hal_ipc::ControlOp::EnqueueMotionBatch,
        nullptr,
        payload_doc.toJson(QJsonDocument::Compact).toStdString());
    if (!res.ok()) {
        const auto axis_res_inner = findAxis(axis_id);
        const auto axis = axis_res_inner.ok() ? axis_res_inner.value() : nullptr;
        emit logMessage(transport_tag_for_axis(axis),QString("Axis %1 relative move failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(res.error().message)));
    }
}
void AxisManager::configureMotionQueue(int axis_id, int capacity, bool drop_oldest) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ConfigureMotionQueue, nullptr, "", capacity, drop_oldest);
}

void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    QJsonArray payload;
    for (const auto& v : points) {
        payload.push_back(QJsonObject::fromVariantMap(v.toMap()));
    }
    const QJsonDocument doc(payload);
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::EnqueueMotionBatch, nullptr, doc.toJson(QJsonDocument::Compact).toStdString());
}

motion_core::Result<motion_core::MotionQueueStats> AxisManager::enqueueMotionBatchDirect(
    int axis_id,
    const std::vector<motion_core::QueuedSetpoint>& points) {
    auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(axis_res.error());
    }
    return axis_res.value()->enqueue_motion_batch(points);
}

motion_core::Result<motion_core::MotionQueueStats> AxisManager::queryMotionQueueStatsDirect(
    int axis_id) const {
    auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(axis_res.error());
    }
    return axis_res.value()->query_motion_queue_stats();
}

void AxisManager::clearMotionQueue(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ClearMotionQueue);
}

void AxisManager::requestMotionQueueStats(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::QueryMotionQueueStats);
    if (!result.ok()) {
        return;
    }
    emit motionQueueStatsUpdated(axis_id, parse_motion_queue_stats_payload(result.value()));
}

void AxisManager::setAxisMode(int axis_id, int mode_code) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::SetAxisMode, nullptr, "", mode_code);
}

void AxisManager::setZeroPosition(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::SetZero);
}

void AxisManager::goHome(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::Home);
}

void AxisManager::requestListParameters(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ListParameters);
    if (!result.ok()) {
        emit parameterListReady(axis_id, {});
        return;
    }
    emit parameterListReady(axis_id, parse_parameter_list_payload(result.value()));
}

void AxisManager::requestReadParameters(int axis_id) {
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ReadParameters);
    if (!result.ok()) {
        emit parametersRead(axis_id, {});
        return;
    }
    emit parametersRead(axis_id, parse_parameter_read_payload(result.value()));
}

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch) {
    QJsonArray payload;
    for (const auto& v : patch) {
        const QVariantMap map = v.toMap();
        QJsonObject entry;
        entry.insert(QStringLiteral("domain"), map.value(QStringLiteral("domain")).toInt());
        entry.insert(QStringLiteral("value"), map.value(QStringLiteral("value")).toInt());
        entry.insert(QStringLiteral("data"), encode_parameter_value_json(map.value(QStringLiteral("data"))));
        payload.push_back(entry);
    }
    const QJsonDocument doc(payload);
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ApplyParameterPatch, nullptr, doc.toJson(QJsonDocument::Compact).toStdString());
}

void AxisManager::setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data) {
    QJsonObject req;
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

    req.insert(QStringLiteral("command"), command_code);
    req.insert(QStringLiteral("data"), encode_parameter_value_json(data));

    const QJsonDocument doc(req);
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::SetPersistent, nullptr, doc.toJson(QJsonDocument::Compact).toStdString());
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ExportAxisConfig, nullptr, path.toStdString());
}

void AxisManager::importAxisConfigPreview(int axis_id, const QString& path) {
    const auto service = getHostServiceForAxis(axis_id);
    const auto result = applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ImportAxisConfigPreview, nullptr, path.toStdString());
    if (!result.ok()) {
        emit axisConfigPreviewReady(axis_id, {});
        return;
    }
    emit axisConfigPreviewReady(axis_id, parse_parameter_read_payload(result.value()));
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    const auto service = getHostServiceForAxis(axis_id);
    (void)applyCommandLocally(service, axis_id, hal_ipc::ControlOp::ImportAxisConfig, nullptr, path.toStdString());
}

void AxisManager::requestManualTakeover(bool enable) {
    const auto op = enable ? hal_ipc::ControlOp::RequestManualLease : hal_ipc::ControlOp::ReleaseManualLease;
    // Manual lease is global, so we can use any service
    bool manual_override_active = false;
    const auto res = applyCommandLocally(host_service_.get(), 0, op, nullptr, "", 0, false, &manual_override_active);
    if (!res.ok()) {
        emit logMessage("hal", QString("Manual takeover request failed"));
        return;
    }
    emit manualTakeoverChanged(manual_override_active);
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

        // Emit motion queue stats directly from the axis (lock-free atomic read, no HAL round-trip).
        // This drives low-watermark sine generation in workspace callbacks at full 250 Hz cadence,
        // decoupled from the unstable UI timer.
        const auto stats_res = axis->query_motion_queue_stats();
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

motion_core::Result<std::string> AxisManager::applyCommandLocally(hal_host_service::HalHostService* service,
                                                                   int axis_id,
                                                                   hal_ipc::ControlOp op,
                                                                   const hal_ipc::AxisPointDto* point,
                                                                   const std::string& input_json,
                                                                   int service_int,
                                                                   bool service_bool,
                                                                   bool* manual_override_active) {
    if (!service) {
        return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InternalError, "No HAL service available"});
    }

    hal_ipc::HalControlFrameDto frame{};
    frame.seq = ++local_control_seq_;
    frame.client_id = hal_ipc::kClientIdMotorTesterUi;
    const bool is_service_op =
        op == hal_ipc::ControlOp::EnableAxis
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
    frame.lease_epoch = is_service_op ? service_epoch_ : motion_epoch_;
    frame.op = op;
    frame.service_axis_id = axis_id;
    frame.service_int_value = service_int;
    frame.service_bool_value = service_bool;
    frame.service_string_value = input_json;
    if (point) {
        frame.axes.push_back(*point);
    }

    const auto state = service->execute_local_command(frame);
    if (state.service_status_code != 0) {
        motion_owner_ = state.motion_owner;
        service_owner_ = state.service_owner;
        manual_override_active_ = state.manual_override_active;
        service_mode_active_ = state.service_mode_active;
        estop_active_ = state.estop_active;
        publishHostState();
        return motion_core::Result<std::string>::failure({motion_core::ErrorCode::InternalError, "HAL command failed"});
    }
    motion_epoch_  = state.motion_epoch;
    service_epoch_ = state.service_epoch;
    motion_owner_ = state.motion_owner;
    service_owner_ = state.service_owner;
    manual_override_active_ = state.manual_override_active;
    service_mode_active_ = state.service_mode_active;
    estop_active_ = state.estop_active;
    publishHostState();
    if (manual_override_active != nullptr) {
        *manual_override_active = state.manual_override_active;
    }
    return motion_core::Result<std::string>::success(state.service_string_response);
}

} // namespace mks
