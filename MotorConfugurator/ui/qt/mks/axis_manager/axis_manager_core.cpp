#include "mks/axis_manager/axis_manager.h"

#include "ethercat/p100e_ethercat_dictionary.h"
#include "mks_can/adapter/mks_axis_adapter.h"
#include "mks_can/internal/port/gs_usb_can_port.h"
#include "mks_can/internal/port/sim_can_port.h"
#include <QCoreApplication>
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
#include <unordered_set>
#include <thread>
#include <chrono>

namespace {

constexpr const char* kMksTransportBusRef = "mks_can";
constexpr const char* kEthercatTransportBusRef = "ethercat";

// ---------------------------------------------------------------------------
// Shared helpers (used across TUs via static linkage per TU — acceptable for
// small helper functions that are only called within their own compilation unit)
// ---------------------------------------------------------------------------

[[nodiscard]] static QString transport_tag_for_axis(const std::shared_ptr<motion_core::IAxis>& axis) {
    if (!axis) return QStringLiteral("hal");
    switch (axis->info().transport) {
        case motion_core::AxisTransportKind::CanBus:   return QStringLiteral("mks");
        case motion_core::AxisTransportKind::Ethercat: return QStringLiteral("ethercat");
        default:                                        return QStringLiteral("hal");
    }
}

[[nodiscard]] static QString owner_role_to_string(const hal_ipc::OwnerRole role) {
    switch (role) {
        case hal_ipc::OwnerRole::HexaMotion:   return QStringLiteral("hexamotion");
        case hal_ipc::OwnerRole::MotorTesterUi: return QStringLiteral("ui");
        default:                                return QStringLiteral("none");
    }
}

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

[[nodiscard]] static std::uint16_t next_free_axis_id(
    const std::unordered_set<std::uint16_t>& used_ids) {
    for (std::uint32_t candidate = 1U;
         candidate <= static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max());
         ++candidate) {
        const auto id = static_cast<std::uint16_t>(candidate);
        if (used_ids.find(id) == used_ids.end()) {
            return id;
        }
    }
    return 0U;
}

[[nodiscard]] static bool assign_discovered_axis_ids(
    const std::vector<motion_core::HalAxisRuntimeEntry>& existing_axes,
    std::vector<motion_core::HalAxisRuntimeEntry>& discovered_axes,
    QString* error_out = nullptr) {
    std::unordered_set<std::uint16_t> used_ids;
    used_ids.reserve(existing_axes.size() + discovered_axes.size());
    for (const auto& axis : existing_axes) {
        if (axis.axis_id.valid()) {
            used_ids.insert(axis.axis_id.value);
        }
    }

    for (auto& axis : discovered_axes) {
        const auto assigned = next_free_axis_id(used_ids);
        if (assigned == 0U) {
            if (error_out) {
                *error_out = QStringLiteral("No free global axis_id available");
            }
            return false;
        }
        axis.axis_id.value = assigned;
        used_ids.insert(assigned);
    }

    return true;
}

[[nodiscard]] static bool assign_mks_axis_ids_from_can_ids(
    const std::vector<motion_core::HalAxisRuntimeEntry>& existing_axes,
    std::vector<motion_core::HalAxisRuntimeEntry>& discovered_axes,
    QString* error_out = nullptr) {
    std::unordered_set<std::uint16_t> used_ids;
    used_ids.reserve(existing_axes.size() + discovered_axes.size());
    for (const auto& axis : existing_axes) {
        if (axis.axis_id.valid()) {
            used_ids.insert(axis.axis_id.value);
        }
    }

    for (auto& axis : discovered_axes) {
        const auto can_id = axis.transport_address;
        if (can_id == 0U || can_id > 2047U) {
            if (error_out) {
                *error_out = QStringLiteral("MKS CAN_ID must be in range [1, 2047]");
            }
            return false;
        }
        if (used_ids.find(can_id) != used_ids.end()) {
            if (error_out) {
                *error_out = QStringLiteral("MKS CAN_ID %1 conflicts with an existing axis_id").arg(can_id);
            }
            return false;
        }
        axis.axis_id.value = can_id;
        used_ids.insert(can_id);
    }

    return true;
}

[[nodiscard]] static QString make_mks_open_probe_failure_message(const QString& device_path,
                                                                 const int baud_rate) {
    QString message = QStringLiteral("Failed to access MKS device %1 @ %2. "
                                     "The device may be busy, disconnected, or not accessible.")
                          .arg(device_path)
                          .arg(baud_rate);
#ifdef _WIN32
    message += QStringLiteral(" On Windows, also verify that the GS-USB adapter is bound to a "
                              "WinUSB/libusb-compatible driver.");
#endif
    return message;
}

[[nodiscard]] static bool probe_mks_device_access(const QString& device_path,
                                                  const int baud_rate,
                                                  QString* error_out) {
    if (device_path.isEmpty() || baud_rate <= 0) {
        if (error_out) {
            *error_out = QStringLiteral("invalid device path or baud rate");
        }
        return false;
    }

    std::unique_ptr<mks::ICanPort> can_port{};
    const std::string device_path_std = device_path.toStdString();
    if (device_path_std.rfind("sim", 0) == 0 || device_path_std.rfind("SIM", 0) == 0) {
        can_port = std::make_unique<mks::SimCanPort>();
    } else {
        can_port = std::make_unique<mks::GsUsbCanPort>();
    }

    if (auto* gs_usb_port = dynamic_cast<mks::GsUsbCanPort*>(can_port.get())) {
        if (!gs_usb_port->probeAccess(device_path_std.c_str())) {
            if (error_out) {
                const QString low_level = QString::fromStdString(gs_usb_port->lastError());
                *error_out = make_mks_open_probe_failure_message(device_path, baud_rate)
                             + (low_level.isEmpty() ? QString() : QStringLiteral(" Details: %1").arg(low_level));
            }
            return false;
        }
        return true;
    }

    if (!can_port->open(device_path_std.c_str(), static_cast<unsigned int>(baud_rate))) {
        if (error_out) {
            *error_out = make_mks_open_probe_failure_message(device_path, baud_rate);
        }
        return false;
    }

    can_port->close();
    return true;
}

} // namespace

namespace mks {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

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
            if (!bus) continue;
            const auto stat_res = bus->get_statistics();
            if (!stat_res.ok()) continue;
            QVariantMap bus_map;
            bus_map[QStringLiteral("cycle_rate_hz")]     = stat_res.value().cycle_rate_hz;
            bus_map[QStringLiteral("bus_load_percent")]  = stat_res.value().bus_load_percent;
            stats_map[QString::fromStdString(bus->get_name())] = bus_map;
            if (first_bus_hz == 0.0 && stat_res.value().cycle_rate_hz > 0.0)
                first_bus_hz = stat_res.value().cycle_rate_hz;
        }
        cached_cycle_hz_ = first_bus_hz;
        if (!stats_map.isEmpty()) emit busStatisticsUpdated(stats_map);
        publishHostState();
    });
}

AxisManager::~AxisManager() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

bool AxisManager::isReady() const {
    const auto listed = unified_runtime_.list_axes();
    return listed.ok() && !listed.value().empty();
}

std::shared_ptr<motion_core::IAxis> AxisManager::getAxis(int axis_id) const {
    auto res = unified_runtime_.find_axis(axis_id);
    if (res.ok()) {
        return res.value();
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// State & topology helpers
// ---------------------------------------------------------------------------

void AxisManager::publishHostState() {
    hal_ipc::OwnerRole motion_owner  = hal_ipc::OwnerRole::None;
    bool manual_override_active = true;
    bool estop_active = false;
    bool mks_homing_sequence_ui_lock_active = false;
    motion_core::ControlOwner control_source = motion_core::ControlOwner::UI;
    bool ipc_server_running = false;
    int connected_hexamotion_client_count = 0;

    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        control_source = active_source_.load();
        motion_owner = (control_source == motion_core::ControlOwner::HexaMotion)
            ? hal_ipc::OwnerRole::HexaMotion
            : hal_ipc::OwnerRole::MotorTesterUi;
        manual_override_active = (control_source == motion_core::ControlOwner::UI);
        estop_active = estop_active_;
        mks_homing_sequence_ui_lock_active = mks_homing_sequence_ui_lock_active_;
    }

    ipc_server_running = ipc_server_.is_running();
    connected_hexamotion_client_count = ipc_server_.connected_client_count();

    QVariantMap state;
    state.insert(QStringLiteral("motion_owner"),             owner_role_to_string(motion_owner));
    state.insert(QStringLiteral("manual_override_active"),   manual_override_active);
    state.insert(QStringLiteral("estop_active"),             estop_active);
    const bool hexamotion_owns = (control_source == motion_core::ControlOwner::HexaMotion);
    state.insert(QStringLiteral("control_source"),
                 hexamotion_owns ? QStringLiteral("hexamotion") : QStringLiteral("ui"));
    state.insert(QStringLiteral("hexamotion_ipc_running"),        ipc_server_running);
    state.insert(QStringLiteral("hexamotion_connected_clients"),  connected_hexamotion_client_count);
    state.insert(QStringLiteral("mks_homing_sequence_active"),    mks_homing_sequence_ui_lock_active);
    emit hostStateUpdated(state);
}



void AxisManager::removeTransportConfig(const motion_core::AxisTransportKind transport) {
    if (transport == motion_core::AxisTransportKind::CanBus)
        current_hal_config_.mks_buses.clear();
    else if (transport == motion_core::AxisTransportKind::Ethercat)
        current_hal_config_.ethercat_buses.clear();

    current_hal_config_.axes.erase(
        std::remove_if(current_hal_config_.axes.begin(), current_hal_config_.axes.end(),
                       [transport](const motion_core::HalAxisRuntimeEntry& a){ return a.transport == transport; }),
        current_hal_config_.axes.end());
}

void AxisManager::publishTopologySnapshot() {
    QVariantList mks_out, ecat_out;
    const auto listed = unified_runtime_.list_axes();
    if (listed.ok() && !listed.value().empty()) {
        for (const auto& info : listed.value()) {
            if (info.transport == motion_core::AxisTransportKind::CanBus)
                mks_out.push_back(static_cast<int>(info.id.value));
            else if (info.transport == motion_core::AxisTransportKind::Ethercat)
                ecat_out.push_back(static_cast<int>(info.id.value));
        }
    } else {
        for (const auto& axis : current_hal_config_.axes) {
            if (!axis.axis_id.valid()) {
                continue;
            }
            if (axis.transport == motion_core::AxisTransportKind::CanBus)
                mks_out.push_back(static_cast<int>(axis.axis_id.value));
            else if (axis.transport == motion_core::AxisTransportKind::Ethercat)
                ecat_out.push_back(static_cast<int>(axis.axis_id.value));
        }
    }
    emit scanFinished(QStringLiteral("mks"),  mks_out);
    emit scanFinished(QStringLiteral("ecat"), ecat_out);
}

void AxisManager::publishTransportOpenStates() {
    emit transportOpenStateChanged(QStringLiteral("mks"),  mks_device_opened_);
    emit transportOpenStateChanged(QStringLiteral("ecat"), ethercat_device_opened_);
}

// ---------------------------------------------------------------------------
// Runtime lifecycle
// ---------------------------------------------------------------------------

void AxisManager::rebuildRuntimeFromCurrentConfig() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    ui_priority_axis_id_ = -1;
    rr_index_ = 0;
    runtime_started_axes_.clear();
    mks_axes_.clear();
    ethercat_axes_.clear();


    if (unified_runtime_.is_active()) (void)unified_runtime_.stop();
    if (unified_runtime_.is_open())   (void)unified_runtime_.close();

    if (current_hal_config_.axes.empty()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    motion_core::HalRuntimeConfig effective_config = current_hal_config_;

    if (mks_device_opened_) {
        for (auto& bus : effective_config.mks_buses) {
            bus.interface_id = kMksTransportBusRef;
            bus.device_path = opened_mks_device_path_.toStdString();
            bus.baud_rate = static_cast<std::uint32_t>(opened_mks_baud_rate_);
        }
        if (!has_mks_scan_snapshot_) {
            effective_config.mks_buses.clear();
            effective_config.axes.erase(
                std::remove_if(effective_config.axes.begin(), effective_config.axes.end(),
                               [](const motion_core::HalAxisRuntimeEntry& axis) {
                                   return axis.transport == motion_core::AxisTransportKind::CanBus;
                               }),
                effective_config.axes.end());

            if (!current_hal_config_.mks_buses.empty() || std::any_of(current_hal_config_.axes.begin(),
                                                                       current_hal_config_.axes.end(),
                                                                       [](const motion_core::HalAxisRuntimeEntry& axis) {
                                                                           return axis.transport == motion_core::AxisTransportKind::CanBus;
                                                                       })) {
                emit logMessage(QStringLiteral("mks"),
                                QStringLiteral("MKS config is loaded, but no live scan snapshot exists. Scan the bus before activating MKS axes from config."));
            }
        } else {
            std::size_t skipped_mks_axes = 0U;
            effective_config.axes.erase(
                std::remove_if(effective_config.axes.begin(), effective_config.axes.end(),
                               [this, &skipped_mks_axes](const motion_core::HalAxisRuntimeEntry& axis) {
                                   if (axis.transport != motion_core::AxisTransportKind::CanBus) {
                                       return false;
                                   }

                                   const bool live = last_scanned_mks_can_ids_.contains(static_cast<int>(axis.transport_address));
                                   if (!live) {
                                       ++skipped_mks_axes;
                                   }
                                   return !live;
                               }),
                effective_config.axes.end());

            for (auto& axis : effective_config.axes) {
                if (axis.transport == motion_core::AxisTransportKind::CanBus) {
                    axis.bus_ref = kMksTransportBusRef;
                }
            }

            if (skipped_mks_axes > 0U) {
                emit logMessage(QStringLiteral("mks"),
                                QStringLiteral("MKS runtime filtered by live scan: skipped %1 configured axis(es) not present in the latest bus scan.")
                                    .arg(static_cast<qulonglong>(skipped_mks_axes)));
            }
        }
    } else {
        effective_config.mks_buses.clear();
        effective_config.axes.erase(
            std::remove_if(effective_config.axes.begin(), effective_config.axes.end(),
                           [](const motion_core::HalAxisRuntimeEntry& axis) {
                               return axis.transport == motion_core::AxisTransportKind::CanBus;
                           }),
            effective_config.axes.end());
    }

    if (ethercat_device_opened_) {
        for (auto& bus : effective_config.ethercat_buses) {
            bus.interface_name = opened_ethercat_interface_.toStdString();
        }
        for (auto& axis : effective_config.axes) {
            if (axis.transport == motion_core::AxisTransportKind::Ethercat) {
                axis.bus_ref = kEthercatTransportBusRef;
            }
        }
    } else {
        effective_config.ethercat_buses.clear();
        effective_config.axes.erase(
            std::remove_if(effective_config.axes.begin(), effective_config.axes.end(),
                           [](const motion_core::HalAxisRuntimeEntry& axis) {
                               return axis.transport == motion_core::AxisTransportKind::Ethercat;
                           }),
            effective_config.axes.end());
    }

    if (effective_config.axes.empty()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    const auto open_res = unified_runtime_.open_from_config(effective_config);
    if (!open_res.ok())
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Unified runtime open failed: %1").arg(open_res.error().message));

    const auto listed = unified_runtime_.list_axes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            const int id = static_cast<int>(info.id.value);
            if (info.transport == motion_core::AxisTransportKind::CanBus)
                mks_axes_.insert(id);
            else if (info.transport == motion_core::AxisTransportKind::Ethercat)
                ethercat_axes_.insert(id);
        }
    }

    if (mks_axes_.isEmpty() && ethercat_axes_.isEmpty()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    const auto start_res = startRuntimeHeadless();
    if (!start_res.ok()) {
        QString details;
        if (!opened_mks_device_path_.isEmpty()) {
            details += QStringLiteral(" [MKS=%1 @ %2]")
                           .arg(opened_mks_device_path_)
                           .arg(opened_mks_baud_rate_);
        }
        if (!opened_ethercat_interface_.isEmpty()) {
            details += QStringLiteral(" [EtherCAT=%1]").arg(opened_ethercat_interface_);
        }
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Runtime start failed after rebuild: %1%2")
                            .arg(start_res.error().message)
                            .arg(details));
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    std::size_t axis_configs_applied = 0U;
    std::size_t axis_config_failures = 0U;
    for (const auto& axis : current_hal_config_.axes) {
        if (!axis.axis_id.valid() || axis.config_file.empty()) {
            continue;
        }
        const auto axis_res = unified_runtime_.find_axis(axis.axis_id.value);
        if (!axis_res.ok()) {
            continue;
        }

        const auto apply_res = unified_runtime_.apply_axis_config_file(axis.axis_id.value, axis.config_file);
        if (!apply_res.ok()) {
            ++axis_config_failures;
            emit logMessage(QStringLiteral("hal"),
                            QStringLiteral("Axis %1 config apply failed from %2: %3")
                                .arg(static_cast<int>(axis.axis_id.value))
                                .arg(QString::fromStdString(axis.config_file))
                                .arg(QString::fromStdString(apply_res.error().message)));
            continue;
        }

        ++axis_configs_applied;
    }

    if (axis_configs_applied > 0U || axis_config_failures > 0U) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Axis configs applied after rebuild: success=%1 failed=%2")
                            .arg(axis_configs_applied)
                            .arg(axis_config_failures));
    }

    emit connectionChanged(!runtime_started_axes_.isEmpty());
    publishTopologySnapshot();
}

void AxisManager::applySafetyBaselineForAxis(const int axis_id, const QString& reason, const bool force_disable) {
    const auto axis_res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    auto axis = axis_res.value();

    if (force_disable) {
        motion_core::ServiceCommandPoint cmd{};
        cmd.type = motion_core::ServiceCommandType::Disable;
        cmd.axis_id = static_cast<std::uint16_t>(axis_id);
        if (!axis->enqueueServicePoint(cmd)) {
            emit logMessage(transport_tag_for_axis(axis),
                            QString("Axis %1 disable in safety baseline failed (%2)")
                                .arg(axis_id).arg(reason));
        }
    }

    motion_core::ServiceCommandPoint set_zero{};
    set_zero.type = motion_core::ServiceCommandType::SetZero;
    set_zero.axis_id = static_cast<std::uint16_t>(axis_id);
    if (!axis->enqueueServicePoint(set_zero)) {
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 set-zero in safety baseline failed (%2)")
                            .arg(axis_id).arg(reason));
    } else {
        emit logMessage(transport_tag_for_axis(axis),
                        QString("Axis %1 baseline requested: disable + set-zero (%2)")
                            .arg(axis_id).arg(reason));
    }
}

void AxisManager::rebuildTransportRuntime(const motion_core::AxisTransportKind transport) {
    Q_UNUSED(transport);
    const QSet<int> prev_watched = watched_axes_;
    rebuildRuntimeFromCurrentConfig();
    for (int id : prev_watched) {
        if (mks_axes_.contains(id) || ethercat_axes_.contains(id)) {
            watched_axes_.insert(id);
            if (ui_priority_axis_id_ == -1) ui_priority_axis_id_ = id;
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
        const auto r = unified_runtime_.start();
        if (!r.ok()) return r;
    }

    runtime_started_axes_.clear();
    for (const int id : mks_axes_) {
        runtime_started_axes_.insert(id);
        applySafetyBaselineForAxis(id, QStringLiteral("startRuntimeHeadless"), true);
    }
    for (const int id : ethercat_axes_) {
        runtime_started_axes_.insert(id);
        applySafetyBaselineForAxis(id, QStringLiteral("startRuntimeHeadless"), true);
    }
    watched_axes_.clear();
    rr_index_ = 0;

    if (!runtime_started_axes_.isEmpty()) {
        fast_timer_->start();
        slow_timer_->start();

        if (!ipc_server_.is_running()) {
            const std::string bind_host = "127.0.0.1";
            const int port = 5555;
            const auto ipc_res = ipc_server_.start(
                bind_host, port,
                [this](const hal_ipc::HalControlFrameDto& frame) { return handleControlFrame(frame); },
                []() {});
            if (ipc_res.ok())
                emit logMessage(QStringLiteral("hal"),
                                QStringLiteral("IPC server listening on %1:%2")
                                    .arg(QString::fromStdString(bind_host))
                                    .arg(port));
            else
                emit logMessage(QStringLiteral("hal"),
                                QStringLiteral("IPC server start failed: %1")
                                    .arg(QString::fromStdString(ipc_res.error().message)));
        }
    }
    // Broadcast initial owner to newly initialized axes
    setMotionSource(active_source_.load());

    return motion_core::Result<void>::success();
}

// ---------------------------------------------------------------------------
// Device open / close
// ---------------------------------------------------------------------------

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    if (device_path.isEmpty() || baud_rate <= 0) {
        emit logMessage(QStringLiteral("mks"), QStringLiteral("Open failed: invalid device path or baud rate"));
        emit connectionChanged(false);
        return;
    }

    QString probe_error;
    if (!probe_mks_device_access(device_path, baud_rate, &probe_error)) {
        emit logMessage(QStringLiteral("mks"), probe_error);
        emit connectionChanged(false);
        return;
    }

    opened_mks_device_path_ = device_path;
    opened_mks_baud_rate_   = baud_rate;
    mks_device_opened_      = true;
    emit logMessage(QStringLiteral("mks"), QString("Device opened: %1 @ %2").arg(device_path).arg(baud_rate));
    emit transportOpenStateChanged(QStringLiteral("mks"), true);

    if (!current_hal_config_.mks_buses.empty() || std::any_of(current_hal_config_.axes.begin(),
                                                               current_hal_config_.axes.end(),
                                                               [](const motion_core::HalAxisRuntimeEntry& axis) {
                                                                   return axis.transport == motion_core::AxisTransportKind::CanBus;
                                                               })) {
        rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
    }
}

void AxisManager::openEthercatDevice(const QString& interface_name) {
    if (interface_name.isEmpty()) {
        emit logMessage(QStringLiteral("ecat"), QStringLiteral("EtherCAT open failed: interface name is empty"));
        emit connectionChanged(false);
        return;
    }
    opened_ethercat_interface_ = interface_name;
    ethercat_device_opened_    = true;
    emit logMessage(QStringLiteral("ecat"), QString("EtherCAT interface opened: %1").arg(interface_name));
    emit transportOpenStateChanged(QStringLiteral("ecat"), true);

    if (!current_hal_config_.ethercat_buses.empty() || std::any_of(current_hal_config_.axes.begin(),
                                                                    current_hal_config_.axes.end(),
                                                                    [](const motion_core::HalAxisRuntimeEntry& axis) {
                                                                        return axis.transport == motion_core::AxisTransportKind::Ethercat;
                                                                    })) {
        rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
    }
}

void AxisManager::setMksHomingSequenceUiLock(const bool active) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (mks_homing_sequence_ui_lock_active_ != active) {
            mks_homing_sequence_ui_lock_active_ = active;
            changed = true;
        }
    }
    if (changed) {
        publishHostState();
    }
}

void AxisManager::closeMksDevice() {
    mks_device_opened_ = false;
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    emit transportOpenStateChanged(QStringLiteral("mks"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
}

void AxisManager::closeEthercatDevice() {
    ethercat_device_opened_ = false;
    opened_ethercat_interface_.clear();
    emit transportOpenStateChanged(QStringLiteral("ecat"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
}

void AxisManager::closeDevice() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    if (ipc_server_.is_running()) ipc_server_.stop();
    watched_axes_.clear();
    ui_priority_axis_id_ = -1;
    rr_index_ = 0;
    reset_runtime_state();
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    mks_device_opened_ = false;
    last_scanned_mks_can_ids_.clear();
    has_mks_scan_snapshot_ = false;
    opened_ethercat_interface_.clear();
    ethercat_device_opened_ = false;
    current_hal_config_ = {};
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        active_source_ = motion_core::ControlOwner::UI;
        estop_active_   = false;
        mks_homing_sequence_ui_lock_active_ = false;
    }
    publishHostState();
    emit connectionChanged(false);
    publishTransportOpenStates();
    publishTopologySnapshot();
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Device/runtime closed"));
}

// ---------------------------------------------------------------------------
// Runtime start / stop / scan
// ---------------------------------------------------------------------------

void AxisManager::startRuntime() {
    if (!isReady()) {
        emit logMessage(QStringLiteral("hal"), QStringLiteral("Runtime start failed: runtime is not open"));
        emit connectionChanged(false);
        return;
    }
    const auto r = startRuntimeHeadless();
    if (!r.ok()) {
        emit logMessage(QStringLiteral("hal"), QString("Runtime start failed: %1").arg(r.error().message));
        emit connectionChanged(false);
        return;
    }
    emit connectionChanged(!runtime_started_axes_.isEmpty());
    emit logMessage(QStringLiteral("hal"), QString("Runtime started: %1 axis(es)").arg(runtime_started_axes_.size()));
}

void AxisManager::stopRuntime() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    if (ipc_server_.is_running()) ipc_server_.stop();
    watched_axes_.clear();
    rr_index_ = 0;
    if (unified_runtime_.is_active()) (void)unified_runtime_.stop();
    runtime_started_axes_.clear();
    publishHostState();
    emit connectionChanged(false);
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Runtime stopped"));
}

void AxisManager::scanMotors(int max_id) {
    if (!mks_device_opened_) {
        emit logMessage(QStringLiteral("mks"), QStringLiteral("Scan failed: open device first"));
        return;
    }
    
    if (unified_runtime_.is_open()) {
        (void)unified_runtime_.close();
    }

    const std::string device_path = opened_mks_device_path_.toStdString();
    const std::uint32_t baud_rate = static_cast<std::uint32_t>(opened_mks_baud_rate_);

    std::thread([this, max_id, device_path, baud_rate]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        motion_core::MksScanRequest request{};
        request.device_path = device_path;
        request.baud_rate   = baud_rate;
        request.max_id      = max_id;
        auto discovered = unified_runtime_.scan_mks_topology(request);

        QMetaObject::invokeMethod(this, [this, discovered]() mutable {
            if (!discovered.ok()) {
                QString message = QString("Scan failed via HalRuntime: %1")
                                      .arg(QString::fromStdString(discovered.error().message));
#ifdef _WIN32
                message += QStringLiteral(" If the device was just selected/opened, verify that the adapter "
                                          "is not claimed by another process and that WinUSB/libusb driver "
                                          "binding is correct.");
#endif
                emit logMessage(QStringLiteral("mks"), message);
                return;
            }
            motion_core::HalRuntimeConfig cfg = discovered.value();
            last_scanned_mks_can_ids_.clear();
            for (const auto& axis : cfg.axes) {
                last_scanned_mks_can_ids_.insert(static_cast<int>(axis.transport_address));
            }
            has_mks_scan_snapshot_ = true;

            QString assign_error;
            if (!assign_mks_axis_ids_from_can_ids(current_hal_config_.axes, cfg.axes, &assign_error)) {
                emit logMessage(QStringLiteral("mks"),
                                QStringLiteral("Scan failed: %1").arg(assign_error));
                return;
            }

            const auto previous_axes = current_hal_config_.axes;
            for (auto& axis : cfg.axes) {
                axis.bus_ref = kMksTransportBusRef;
                for (const auto& previous_axis : previous_axes) {
                    if (previous_axis.transport != motion_core::AxisTransportKind::CanBus) {
                        continue;
                    }
                    if (previous_axis.transport_address != axis.transport_address) {
                        continue;
                    }
                    axis.config_file = previous_axis.config_file;
                    axis.enable_on_start = previous_axis.enable_on_start;
                    if (!previous_axis.axis_name.value.empty()) {
                        axis.axis_name = previous_axis.axis_name;
                    }
                    break;
                }
            }

            removeTransportConfig(motion_core::AxisTransportKind::CanBus);
            current_hal_config_.mks_buses.insert(current_hal_config_.mks_buses.end(), cfg.mks_buses.begin(), cfg.mks_buses.end());
            current_hal_config_.axes.insert(current_hal_config_.axes.end(), cfg.axes.begin(), cfg.axes.end());
            rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
        }, Qt::QueuedConnection);
    }).detach();
}

void AxisManager::scanEthercatMotors() {
    if (!ethercat_device_opened_) {
        emit logMessage(QStringLiteral("ecat"), QStringLiteral("EtherCAT scan failed: open EtherCAT interface first"));
        return;
    }
    
    if (unified_runtime_.is_open()) {
        (void)unified_runtime_.close();
    }

    const std::string interface_name = opened_ethercat_interface_.toStdString();

    std::thread([this, interface_name]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        motion_core::EthercatScanRequest request{};
        request.interface_name = interface_name;
        auto discovered = unified_runtime_.scan_ethercat_topology(request);

        QMetaObject::invokeMethod(this, [this, discovered]() mutable {
            if (!discovered.ok()) {
                emit logMessage(QStringLiteral("ecat"), QString("EtherCAT scan failed via HalRuntime: %1").arg(QString::fromStdString(discovered.error().message)));
                return;
            }
            motion_core::HalRuntimeConfig cfg = discovered.value();
            const auto previous_axes = current_hal_config_.axes;

            std::unordered_set<std::uint16_t> used_axis_ids;
            for (const auto& axis : current_hal_config_.axes) {
                if (axis.transport != motion_core::AxisTransportKind::Ethercat || !axis.axis_id.valid()) {
                    continue;
                }
                used_axis_ids.insert(axis.axis_id.value);
            }

            for (auto& axis : cfg.axes) {
                axis.bus_ref = kEthercatTransportBusRef;
                for (const auto& previous_axis : previous_axes) {
                    if (previous_axis.transport != motion_core::AxisTransportKind::Ethercat) {
                        continue;
                    }
                    if (previous_axis.transport_address != axis.transport_address) {
                        continue;
                    }
                    axis.axis_id = previous_axis.axis_id;
                    axis.config_file = previous_axis.config_file;
                    axis.enable_on_start = previous_axis.enable_on_start;
                    if (!previous_axis.axis_name.value.empty()) {
                        axis.axis_name = previous_axis.axis_name;
                    }
                    break;
                }
            }

            QString assign_error;
            if (!assign_discovered_axis_ids(current_hal_config_.axes, cfg.axes, &assign_error)) {
                emit logMessage(QStringLiteral("ecat"),
                                QStringLiteral("EtherCAT scan failed: %1").arg(assign_error));
                return;
            }

            removeTransportConfig(motion_core::AxisTransportKind::Ethercat);
            current_hal_config_.ethercat_buses.insert(current_hal_config_.ethercat_buses.end(), cfg.ethercat_buses.begin(), cfg.ethercat_buses.end());
            current_hal_config_.axes.insert(current_hal_config_.axes.end(), cfg.axes.begin(), cfg.axes.end());
            rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
        }, Qt::QueuedConnection);
    }).detach();
}

// ---------------------------------------------------------------------------
// Axis control
// ---------------------------------------------------------------------------

void AxisManager::watchAxis(int axis_id, bool enabled) {
    if (axis_id < 1 || axis_id > 2047) return;
    if (enabled) {
        watched_axes_.insert(axis_id);
        ui_priority_axis_id_ = axis_id;
        return;
    }
    watched_axes_.remove(axis_id);
    if (ui_priority_axis_id_ == axis_id) {
        ui_priority_axis_id_ = watched_axes_.isEmpty() ? -1 : *watched_axes_.cbegin();
    }
}

void AxisManager::setMotionSource(const motion_core::ControlOwner source) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (active_source_ != source) {
            active_source_ = source;
            changed = true;
        }
    }
    if (!changed) return;
    
    // Broadcast active source to all axes
    const auto listed = unified_runtime_.list_axes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            if (auto axis = getAxis(info.id.value)) {
                axis->setControlOwner(source);
            }
        }
    }
    
    emit manualTakeoverChanged(source == motion_core::ControlOwner::UI);
    publishHostState();
}

motion_core::ControlOwner AxisManager::activeSource() const {
    return active_source_.load();
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



// ---------------------------------------------------------------------------
// Manual control source takeover
// ---------------------------------------------------------------------------

void AxisManager::requestManualTakeover(bool enable) {
    setMotionSource(enable ? motion_core::ControlOwner::UI
                           : motion_core::ControlOwner::HexaMotion);
}

// ---------------------------------------------------------------------------
// Fast tick (telemetry loop)
// ---------------------------------------------------------------------------

void AxisManager::onFastTick() {
    if (runtime_started_axes_.isEmpty()) return;
    const double cycle_hz = cached_cycle_hz_;

    for (int axis_id : watched_axes_) {
        auto axis_res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
        if (!axis_res.ok()) continue;
        const auto& axis = axis_res.value();

        {
            const auto stats = axis->query_motion_queue_stats();
            emit motionQueueStatsUpdated(axis_id, motion_queue_stats_to_qvariant_map(stats));
        }

        const auto telem = axis->telemetry();
        {
            QVariantMap t;
            const double target_position_deg = telem.has_target_position ? telem.target_position : telem.position;
            t[QStringLiteral("transport")]                 = transport_tag_for_axis(axis);
            t[QStringLiteral("actual_position_deg")]       = telem.position;
            t[QStringLiteral("raw_axis_position")]         = static_cast<qlonglong>(0);
            t[QStringLiteral("actual_velocity_deg_per_sec")] = telem.velocity;
            t[QStringLiteral("target_position_deg")]       = target_position_deg;
            t[QStringLiteral("actual_torque_percent")]     = telem.current;
            t[QStringLiteral("digital_inputs")]            = static_cast<qulonglong>(telem.digital_inputs);
            t[QStringLiteral("mode")]                      = static_cast<int>(telem.mode);
            t[QStringLiteral("state")]                     = static_cast<int>(telem.state);
            t[QStringLiteral("status_word")]               = static_cast<int>(telem.status_word);
            t[QStringLiteral("protection_code")]           = static_cast<int>(telem.protection_code);
            t[QStringLiteral("motion_status")]             = static_cast<int>(telem.motion_status_code);
            t[QStringLiteral("error_code")]                = static_cast<int>(telem.protection_code);
            t[QStringLiteral("timestamp_ns")]              = static_cast<qulonglong>(telem.timestamp_us * 1000ULL);

            const auto transport = axis->info().transport;
            if (transport == motion_core::AxisTransportKind::CanBus) {
                auto* mks_adapter = dynamic_cast<mks::MksAxisAdapter*>(axis.get());
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
            } else if (cycle_hz > 0.0) {
                const double period_ms = 1000.0 / cycle_hz;
                t[QStringLiteral("cmd_tx_hz")]           = cycle_hz;
                t[QStringLiteral("cmd_tx_period_ms")]    = period_ms;
                t[QStringLiteral("telemetry_publish_hz")]        = cycle_hz;
                t[QStringLiteral("telemetry_publish_period_ms")] = period_ms;
                t[QStringLiteral("position_rx_hz")]      = cycle_hz;
                t[QStringLiteral("position_rx_period_ms")] = period_ms;
                t[QStringLiteral("speed_rx_hz")]         = cycle_hz;
                t[QStringLiteral("status_rx_hz")]        = cycle_hz;
                t[QStringLiteral("protection_rx_hz")]    = cycle_hz;
            }

            emit telemetryUpdated(axis_id, t);
        }
    }
}

} // namespace mks
