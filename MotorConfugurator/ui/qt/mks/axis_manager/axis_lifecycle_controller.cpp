#include "mks/axis_manager/axis_lifecycle_controller.h"
#include "hal_host_service/hal_host_service.h"
#include "mks_can/gs_usb_can_port.h"
#include "mks_can/sim_can_port.h"
#include "mks_can/mks_axis_adapter.h"

#include <QTimer>
#include <QMetaObject>
#include <QPointer>
#include <thread>
#include <chrono>
#include <algorithm>
#include <limits>

namespace {
constexpr const char* kMksTransportBusRef = motion_core::kMksTransportBusRef;
constexpr const char* kEthercatTransportBusRef = motion_core::kEthercatTransportBusRef;

[[nodiscard]] static QString make_mks_open_probe_failure_message(const QString& device_path, const int baud_rate) {
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

[[nodiscard]] static bool probe_mks_device_access(const QString& device_path, const int baud_rate, QString* error_out) {
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

[[nodiscard]] static std::uint16_t next_free_axis_id(const std::unordered_set<std::uint16_t>& used_ids) {
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
} // namespace

namespace mks {

AxisLifecycleController::AxisLifecycleController(
    hal_host::HalHostService& host_service,
    QObject* parent)
    : QObject(parent)
    , host_service_(host_service)
    , unified_runtime_(host_service.runtime())
    , orchestrator_(host_service.orchestrator())
    , ipc_controller_(host_service.ipc())
{
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
        emit hostStateUpdated();
    });
}

AxisLifecycleController::~AxisLifecycleController() {
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

motion_core::ControlOwner AxisLifecycleController::activeSource() const {
    return orchestrator_.control_owner();
}

std::shared_ptr<motion_core::IAxis> AxisLifecycleController::getAxis(int axis_id) const {
    auto res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (res.ok()) {
        return res.value();
    }
    return nullptr;
}

void AxisLifecycleController::setHalConfig(const motion_core::HalRuntimeConfig& config) {
    current_hal_config_ = config;
}

void AxisLifecycleController::openDevice(const QString& device_path, int baud_rate) {
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

void AxisLifecycleController::openEthercatDevice(const QString& interface_name) {
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

void AxisLifecycleController::closeMksDevice() {
    mks_device_opened_ = false;
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    emit transportOpenStateChanged(QStringLiteral("mks"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
}

void AxisLifecycleController::closeEthercatDevice() {
    ethercat_device_opened_ = false;
    opened_ethercat_interface_.clear();
    emit transportOpenStateChanged(QStringLiteral("ecat"), false);
    rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
}

void AxisLifecycleController::closeDevice() {
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    updateTelemetryDistributorAxes();
    ui_priority_axis_id_ = -1;
    resetRuntimeState();
    opened_mks_device_path_.clear();
    opened_mks_baud_rate_ = 0;
    mks_device_opened_ = false;
    last_scanned_mks_can_ids_.clear();
    has_mks_scan_snapshot_ = false;
    opened_ethercat_interface_.clear();
    ethercat_device_opened_ = false;
    current_hal_config_ = {};
    orchestrator_.set_control_owner(motion_core::ControlOwner::UI);
    orchestrator_.clear_estop();

    emit hostStateUpdated();
    emit connectionChanged(false);
    publishTransportOpenStates();
    publishTopologySnapshot();
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Device/runtime closed"));
}

void AxisLifecycleController::startRuntime() {
    rebuildRuntimeFromCurrentConfig();
}

void AxisLifecycleController::stopRuntime() {
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    updateTelemetryDistributorAxes();
    (void)host_service_.stop();
    runtime_started_axes_.clear();
    emit hostStateUpdated();
    emit connectionChanged(false);
    emit logMessage(QStringLiteral("hal"), QStringLiteral("Runtime stopped"));
}

void AxisLifecycleController::scanMotors(int max_id) {
    if (!mks_device_opened_) {
        emit logMessage(QStringLiteral("mks"), QStringLiteral("Scan failed: open device first"));
        return;
    }
    
    if (unified_runtime_.is_open()) {
        (void)unified_runtime_.close();
    }

    const std::string device_path = opened_mks_device_path_.toStdString();
    const std::uint32_t baud_rate = static_cast<std::uint32_t>(opened_mks_baud_rate_);

    QPointer<AxisLifecycleController> safeThis(this);
    std::thread([safeThis, max_id, device_path, baud_rate, &unified_runtime = unified_runtime_]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        motion_core::MksScanRequest request{};
        request.device_path = device_path;
        request.baud_rate   = baud_rate;
        request.max_id      = max_id;
        auto discovered = unified_runtime.scan_mks_topology(request);

        QMetaObject::invokeMethod(safeThis, [safeThis, discovered = std::move(discovered)]() mutable {
            if (!safeThis) return;
            if (!discovered.ok()) {
                QString message = QString("Scan failed via HalRuntime: %1")
                                      .arg(QString::fromStdString(discovered.error().message));
#ifdef _WIN32
                message += QStringLiteral(" If the device was just selected/opened, verify that the adapter "
                                          "is not claimed by another process and that WinUSB/libusb driver "
                                          "binding is correct.");
#endif
                emit safeThis->logMessage(QStringLiteral("mks"), message);
                return;
            }
            motion_core::HalRuntimeConfig cfg = discovered.value();
            safeThis->last_scanned_mks_can_ids_.clear();
            for (const auto& axis : cfg.axes) {
                safeThis->last_scanned_mks_can_ids_.insert(static_cast<int>(axis.transport_address));
            }
            safeThis->has_mks_scan_snapshot_ = true;

            QString assign_error;
            if (!assign_mks_axis_ids_from_can_ids(safeThis->current_hal_config_.axes, cfg.axes, &assign_error)) {
                emit safeThis->logMessage(QStringLiteral("mks"),
                                QStringLiteral("Scan failed: %1").arg(assign_error));
                return;
            }

            const auto previous_axes = safeThis->current_hal_config_.axes;
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

            safeThis->removeTransportConfig(motion_core::AxisTransportKind::CanBus);
            safeThis->current_hal_config_.mks_buses.insert(safeThis->current_hal_config_.mks_buses.end(), cfg.mks_buses.begin(), cfg.mks_buses.end());
            safeThis->current_hal_config_.axes.insert(safeThis->current_hal_config_.axes.end(), cfg.axes.begin(), cfg.axes.end());
            safeThis->rebuildTransportRuntime(motion_core::AxisTransportKind::CanBus);
        }, Qt::QueuedConnection);
    }).detach();
}

void AxisLifecycleController::scanEthercatMotors() {
    if (!ethercat_device_opened_) {
        emit logMessage(QStringLiteral("ecat"), QStringLiteral("EtherCAT scan failed: open EtherCAT interface first"));
        return;
    }
    
    if (unified_runtime_.is_open()) {
        (void)unified_runtime_.close();
    }

    const std::string interface_name = opened_ethercat_interface_.toStdString();

    QPointer<AxisLifecycleController> safeThis(this);
    std::thread([safeThis, interface_name, &unified_runtime = unified_runtime_]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        motion_core::EthercatScanRequest request{};
        request.interface_name = interface_name;
        auto discovered = unified_runtime.scan_ethercat_topology(request);

        QMetaObject::invokeMethod(safeThis, [safeThis, discovered = std::move(discovered)]() mutable {
            if (!safeThis) return;
            if (!discovered.ok()) {
                emit safeThis->logMessage(QStringLiteral("ecat"), QString("EtherCAT scan failed via HalRuntime: %1").arg(QString::fromStdString(discovered.error().message)));
                return;
            }
            motion_core::HalRuntimeConfig cfg = discovered.value();
            const auto previous_axes = safeThis->current_hal_config_.axes;

            std::unordered_set<std::uint16_t> used_axis_ids;
            for (const auto& axis : safeThis->current_hal_config_.axes) {
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
            if (!assign_discovered_axis_ids(safeThis->current_hal_config_.axes, cfg.axes, &assign_error)) {
                emit safeThis->logMessage(QStringLiteral("ecat"),
                                QStringLiteral("EtherCAT scan failed: %1").arg(assign_error));
                return;
            }

            safeThis->removeTransportConfig(motion_core::AxisTransportKind::Ethercat);
            safeThis->current_hal_config_.ethercat_buses.insert(safeThis->current_hal_config_.ethercat_buses.end(), cfg.ethercat_buses.begin(), cfg.ethercat_buses.end());
            safeThis->current_hal_config_.axes.insert(safeThis->current_hal_config_.axes.end(), cfg.axes.begin(), cfg.axes.end());
            safeThis->rebuildTransportRuntime(motion_core::AxisTransportKind::Ethercat);
        }, Qt::QueuedConnection);
    }).detach();
}

void AxisLifecycleController::watchAxis(int axis_id, bool enabled) {
    if (axis_id < 1 || axis_id > 2047) return;
    if (enabled) {
        if (!watched_axes_.contains(axis_id)) {
            watched_axes_.insert(axis_id);
            updateTelemetryDistributorAxes();
        }
        ui_priority_axis_id_ = axis_id;
        emit watchedAxesChanged();
        return;
    }
    watched_axes_.remove(axis_id);
    updateTelemetryDistributorAxes();
    if (ui_priority_axis_id_ == axis_id) {
        ui_priority_axis_id_ = watched_axes_.isEmpty() ? -1 : *watched_axes_.cbegin();
    }
    emit watchedAxesChanged();
}

void AxisLifecycleController::setMotionSource(const motion_core::ControlOwner source) {
    if (orchestrator_.control_owner() == source) return;
    orchestrator_.set_control_owner(source);
    emit manualTakeoverChanged(source == motion_core::ControlOwner::UI);
    emit hostStateUpdated();
}

void AxisLifecycleController::requestManualTakeover(bool enable) {
    setMotionSource(enable ? motion_core::ControlOwner::UI
                           : motion_core::ControlOwner::HexaMotion);
}

void AxisLifecycleController::rebuildRuntimeFromCurrentConfig() {
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    updateTelemetryDistributorAxes();
    ui_priority_axis_id_ = -1;
    runtime_started_axes_.clear();
    mks_axes_.clear();
    ethercat_axes_.clear();

    (void)host_service_.stop();

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

    const auto start_res = host_service_.start(effective_config);
    if (!start_res.ok()) {
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

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
        (void)host_service_.stop();
        emit connectionChanged(false);
        publishTopologySnapshot();
        return;
    }

    // Bug 8: Load config files BEFORE applying the safety baseline
    for (const auto& axis : current_hal_config_.axes) {
        if (!axis.axis_id.valid() || axis.config_file.empty()) {
            continue;
        }
        const auto axis_res = unified_runtime_.find_axis(axis.axis_id.value);
        if (!axis_res.ok()) {
            continue;
        }
        (void)unified_runtime_.apply_axis_config_file(axis.axis_id.value, axis.config_file);
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

    if (!runtime_started_axes_.isEmpty()) {
        slow_timer_->start();
    }
    setMotionSource(orchestrator_.control_owner());

    emit connectionChanged(!runtime_started_axes_.isEmpty());
    publishTopologySnapshot();
}

void AxisLifecycleController::applySafetyBaselineForAxis(const int axis_id, const QString& reason, const bool force_disable) {
    Q_UNUSED(reason);
    // Bug 2.2: Fix direct driver bypass from UI. Use orchestrator_.apply_safety_baseline(...)
    orchestrator_.apply_safety_baseline(static_cast<std::uint16_t>(axis_id), force_disable);
}

void AxisLifecycleController::rebuildTransportRuntime(const motion_core::AxisTransportKind transport) {
    Q_UNUSED(transport);
    const QSet<int> prev_watched = watched_axes_;
    rebuildRuntimeFromCurrentConfig();
    for (int id : prev_watched) {
        if (mks_axes_.contains(id) || ethercat_axes_.contains(id)) {
            watched_axes_.insert(id);
            if (ui_priority_axis_id_ == -1) ui_priority_axis_id_ = id;
        }
    }
    updateTelemetryDistributorAxes();
}

void AxisLifecycleController::resetRuntimeState() {
    runtime_started_axes_.clear();
    mks_axes_.clear();
    ethercat_axes_.clear();
    ui_priority_axis_id_ = -1;

    if (unified_runtime_.is_open()) (void)unified_runtime_.close();
}

void AxisLifecycleController::publishTopologySnapshot() {
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

void AxisLifecycleController::publishTransportOpenStates() {
    emit transportOpenStateChanged(QStringLiteral("mks"),  mks_device_opened_);
    emit transportOpenStateChanged(QStringLiteral("ecat"), ethercat_device_opened_);
}

void AxisLifecycleController::updateTelemetryDistributorAxes() {
    emit watchedAxesChanged();
}

void AxisLifecycleController::removeTransportConfig(const motion_core::AxisTransportKind transport) {
    if (transport == motion_core::AxisTransportKind::CanBus)
        current_hal_config_.mks_buses.clear();
    else if (transport == motion_core::AxisTransportKind::Ethercat)
        current_hal_config_.ethercat_buses.clear();

    current_hal_config_.axes.erase(
        std::remove_if(current_hal_config_.axes.begin(), current_hal_config_.axes.end(),
                       [transport](const motion_core::HalAxisRuntimeEntry& a){ return a.transport == transport; }),
        current_hal_config_.axes.end());
}

} // namespace mks