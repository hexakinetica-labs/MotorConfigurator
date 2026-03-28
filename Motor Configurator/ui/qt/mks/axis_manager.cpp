#include "mks/axis_manager.h"

#include "ethercat/p100e_ethercat_dictionary.h"
#include "mks_can/adapter/mks_axis_adapter.h"
#include "mks_can/dictionary/mks_dictionary.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <QMetaObject>
#include <QPointer>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include <unordered_map>

namespace {

constexpr std::size_t kMaxUiPositionSamplesPerTick = 512U;

[[nodiscard]] motion_core::Result<motion_core::PersistentCommand> persistent_command_for_parameter(
    const int domain,
    const int value) {
    if (domain == static_cast<int>(motion_core::ParameterDomain::Ethercat)
        && value == static_cast<int>(ethercat_driver::EthercatParameter::MotorType)) {
        return motion_core::Result<motion_core::PersistentCommand>::success(
            motion_core::PersistentCommand::MotorType);
    }

    if (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
        && value == static_cast<int>(mks::MksParameter::CanId)) {
        return motion_core::Result<motion_core::PersistentCommand>::success(
            motion_core::PersistentCommand::CanId);
    }

    if (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
        && value == static_cast<int>(mks::MksParameter::CanBitrateIndex)) {
        return motion_core::Result<motion_core::PersistentCommand>::success(
            motion_core::PersistentCommand::CanBitrate);
    }

    return motion_core::Result<motion_core::PersistentCommand>::failure(
        {motion_core::ErrorCode::Unsupported, "persistent write is not mapped for selected parameter"});
}

[[nodiscard]] QString persistent_command_name(const motion_core::PersistentCommand command) {
    switch (command) {
        case motion_core::PersistentCommand::MotorType:
            return QStringLiteral("MotorType");
        case motion_core::PersistentCommand::CanId:
            return QStringLiteral("CanId");
        case motion_core::PersistentCommand::CanBitrate:
            return QStringLiteral("CanBitrate");
    }
    return QStringLiteral("Unknown");
}

[[nodiscard]] QString persistent_write_report_message(
    const int axis_id,
    const QString& label,
    const motion_core::PersistentWriteReport& report) {
    return QString("Axis %1 persistent %2 write report: write=%3 save=%4 readback=%5 power_cycle=%6 reconnect=%7")
        .arg(axis_id)
        .arg(label)
        .arg(report.write_completed ? "ok" : "no")
        .arg(report.persistent_save_completed ? "ok" : "no")
        .arg(report.readback_verified ? "ok" : "no")
        .arg(report.power_cycle_required ? "yes" : "no")
        .arg(report.reconnect_required ? "yes" : "no");
}

[[nodiscard]] QString persistent_write_report_message(
    const int axis_id,
    const motion_core::PersistentCommand command,
    const motion_core::PersistentWriteReport& report) {
    return persistent_write_report_message(axis_id, persistent_command_name(command), report);
}

[[nodiscard]] bool is_internal_parameter(const motion_core::ParameterId id) {
    return id.domain == motion_core::ParameterDomain::Ethercat
        && id.value == static_cast<std::uint32_t>(ethercat_driver::EthercatParameter::SaveParametersToEeprom);
}

[[nodiscard]] bool parameter_value_matches_requested(
    const motion_core::ParameterValue& requested,
    const motion_core::ParameterValue& actual) {
    const auto to_double = [](const motion_core::ParameterValue& value) {
        switch (value.type) {
            case motion_core::ParameterValueType::SignedInteger:
                return static_cast<double>(value.signed_value);
            case motion_core::ParameterValueType::UnsignedInteger:
                return static_cast<double>(value.unsigned_value);
            case motion_core::ParameterValueType::FloatingPoint:
                return value.floating_value;
            case motion_core::ParameterValueType::Boolean:
                return value.bool_value ? 1.0 : 0.0;
        }
        return 0.0;
    };

    if (requested.type == motion_core::ParameterValueType::Boolean
        || actual.type == motion_core::ParameterValueType::Boolean) {
        return (to_double(requested) >= 0.5) == (to_double(actual) >= 0.5);
    }

    return std::abs(to_double(requested) - to_double(actual)) <= 1e-6;
}

QVariant paramValueToVariant(const motion_core::ParameterValue& val) {
    switch (val.type) {
        case motion_core::ParameterValueType::SignedInteger: return static_cast<qlonglong>(val.signed_value);
        case motion_core::ParameterValueType::UnsignedInteger: return static_cast<qulonglong>(val.unsigned_value);
        case motion_core::ParameterValueType::FloatingPoint: return val.floating_value;
        case motion_core::ParameterValueType::Boolean: return val.bool_value;
    }
    return {};
}

QVariantMap motionQueueStatsToVariantMap(const motion_core::MotionQueueStats& stats) {
    QVariantMap out;
    out["size"] = static_cast<qulonglong>(stats.size);
    out["capacity"] = static_cast<qulonglong>(stats.capacity);
    out["pushed"] = static_cast<qulonglong>(stats.pushed);
    out["dropped"] = static_cast<qulonglong>(stats.dropped);
    out["underruns"] = static_cast<qulonglong>(stats.underruns);
    out["short_starts"] = static_cast<qulonglong>(stats.short_starts);
    return out;
}

motion_core::ParameterValue variantToParamValue(const QVariant& var, const motion_core::ParameterDescriptor* desc) {
    const bool prefer_unsigned = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::UnsignedInteger;
    const bool prefer_float = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::FloatingPoint;

    if (var.typeId() == QMetaType::Bool) return motion_core::ParameterValue::from_bool(var.toBool());
    if (var.typeId() == QMetaType::Double || var.typeId() == QMetaType::Float || prefer_float) {
        return motion_core::ParameterValue::from_floating(var.toDouble());
    }
    if (var.typeId() == QMetaType::ULongLong) return motion_core::ParameterValue::from_unsigned(var.toULongLong());
    if (var.typeId() == QMetaType::UInt) return motion_core::ParameterValue::from_unsigned(var.toUInt());

    const auto ll = var.toLongLong();
    if (prefer_unsigned && ll >= 0) {
        return motion_core::ParameterValue::from_unsigned(static_cast<std::uint64_t>(ll));
    }
    return motion_core::ParameterValue::from_signed(ll);
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
        for (const auto& bus : hal_runtime_.bus_managers_snapshot()) {
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
        }
        if (!stats_map.isEmpty()) {
            emit busStatisticsUpdated(stats_map);
        }
    });
}

AxisManager::~AxisManager() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

bool AxisManager::isReady() const {
    if (!hal_runtime_.is_open()) {
        return false;
    }
    const auto listed = hal_runtime_.list_axes();
    return listed.ok() && !listed.value().empty();
}

QString AxisManager::current_transport_tag() const {
    switch (active_transport_) {
        case ActiveTransport::Mks:      return "mks";
        case ActiveTransport::Ethercat: return "ecat";
        default:                        return "";
    }
}

motion_core::Result<std::shared_ptr<motion_core::IAxis>> AxisManager::findAxis(const std::uint16_t axis_id) const {
    return hal_runtime_.find_axis(axis_id);
}

motion_core::Result<std::vector<motion_core::AxisInfo>> AxisManager::listAxes() const {
    return hal_runtime_.list_axes();
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
        const auto disable_res = axis->set_enabled(false);
        if (!disable_res.ok()) {
            emit logMessage(current_transport_tag(),QString("Axis %1 disable in safety baseline failed (%2): %3")
                                .arg(axis_id)
                                .arg(reason)
                                .arg(disable_res.error().message));
        }
    }

    const auto telem = axis->read_telemetry();
    if (telem.ok()) {
        motion_core::AxisCommand cmd{};
        cmd.has_target_position = true;
        cmd.target_position_deg = telem.value().actual_position_deg;
        (void)axis->apply_command(cmd);
    }
}

motion_core::Result<void> AxisManager::openRuntimeFromConfig(const motion_core::HalRuntimeConfig& config) {
    return hal_runtime_.open_from_config(config);
}

motion_core::Result<void> AxisManager::startRuntimeInternal() {
    return hal_runtime_.start();
}

motion_core::Result<void> AxisManager::stopRuntimeInternal() {
    return hal_runtime_.stop();
}

motion_core::Result<void> AxisManager::closeRuntimeInternal() {
    return hal_runtime_.close();
}

void AxisManager::reset_runtime_state() {
    runtime_started_axes_.clear();
    runtime_known_axes_.clear();
    ui_priority_axis_id_ = -1;
    (void)closeRuntimeInternal();
}

motion_core::Result<void> AxisManager::startRuntimeHeadless() {
    const auto start_result = startRuntimeInternal();
    if (!start_result.ok()) {
        return start_result;
    }

    runtime_started_axes_.clear();
    for (const int axis_id : runtime_known_axes_) {
        runtime_started_axes_.insert(axis_id);
        applySafetyBaselineForAxis(axis_id, "startRuntimeHeadless", true);
    }
    watched_axes_.clear();
    rr_index_ = 0;
    if (!runtime_started_axes_.isEmpty()) {
        fast_timer_->start();
        slow_timer_->start();
    }
    return motion_core::Result<void>::success();
}

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    closeDevice();
    if (device_path.isEmpty() || baud_rate <= 0) {
        emit logMessage("mks", "Open failed: invalid device path or baud rate");
        emit connectionChanged(false);
        return;
    }
    opened_device_path_ = device_path;
    opened_baud_rate_ = baud_rate;
    device_opened_ = true;
    active_transport_ = ActiveTransport::Mks;
    emit logMessage(current_transport_tag(),QString("Device opened: %1 @ %2").arg(device_path).arg(baud_rate));
    emit scanFinished("mks", {});
    emit connectionChanged(false);
}

void AxisManager::openEthercatDevice(const QString& interface_name) {
    closeDevice();
    if (interface_name.isEmpty()) {
        emit logMessage("ecat", "EtherCAT open failed: interface name is empty");
        emit connectionChanged(false);
        return;
    }
    opened_device_path_ = interface_name;
    device_opened_ = true;
    active_transport_ = ActiveTransport::Ethercat;
    emit logMessage(current_transport_tag(),QString("EtherCAT interface opened: %1").arg(interface_name));
    emit scanFinished("ecat", {});
    emit connectionChanged(false);
}

void AxisManager::closeDevice() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    ui_priority_axis_id_ = -1;
    rr_index_ = 0;
    reset_runtime_state();
    opened_device_path_.clear();
    opened_baud_rate_ = 0;
    device_opened_ = false;
    active_transport_ = ActiveTransport::None;
    current_hal_config_ = {};
    emit connectionChanged(false);
    emit logMessage(current_transport_tag(),"Device/runtime closed");
}

void AxisManager::loadHalConfig(const QString& config_path) {
    const auto hal_cfg = motion_core::load_hal_runtime_config_from_file(config_path.toStdString());
    if (!hal_cfg.ok()) {
        emit logMessage(current_transport_tag(),QString("HAL config load failed: %1").arg(hal_cfg.error().message));
        return;
    }
    reset_runtime_state();
    current_hal_config_ = hal_cfg.value();
    const auto open_res = openRuntimeFromConfig(current_hal_config_);
    if (!open_res.ok()) {
        emit logMessage(current_transport_tag(),QString("HAL open runtime failed: %1").arg(open_res.error().message));
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            runtime_known_axes_.insert(static_cast<int>(info.id.value));
        }
    }
    QVariantList mks_out;
    QVariantList ecat_out;
    if (listed.ok()) {
        std::vector<motion_core::AxisInfo> infos = listed.value();
        std::sort(infos.begin(), infos.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.id.value < rhs.id.value;
        });
        for (const auto& info : infos) {
            if (info.transport == motion_core::AxisTransportKind::CanBus) {
                mks_out.push_back(static_cast<int>(info.id.value));
            } else if (info.transport == motion_core::AxisTransportKind::Ethercat) {
                ecat_out.push_back(static_cast<int>(info.id.value));
            }
        }
    }

    emit scanFinished("mks", mks_out);
    emit scanFinished("ecat", ecat_out);
    emit logMessage(current_transport_tag(),QString("HAL config loaded: total=%1 (MKS=%2, EtherCAT=%3)")
                        .arg(runtime_known_axes_.size())
                        .arg(mks_out.size())
                        .arg(ecat_out.size()));
}

void AxisManager::saveHalConfig(const QString& config_path) {
    const auto res = motion_core::save_hal_runtime_config_to_file(config_path.toStdString(), current_hal_config_);
    if (!res.ok()) {
        emit logMessage(current_transport_tag(),QString("Configuration save failed: %1").arg(res.error().message));
    } else {
        emit logMessage(current_transport_tag(),QString("Saved master configuration to %1").arg(config_path));
    }
}

void AxisManager::startRuntime() {
    if (!isReady()) {
        emit logMessage(current_transport_tag(),"Runtime start failed: runtime is not open");
        emit connectionChanged(false);
        return;
    }
    const auto start_result = startRuntimeHeadless();
    if (!start_result.ok()) {
        emit logMessage(current_transport_tag(),QString("Runtime start failed: %1").arg(start_result.error().message));
        emit connectionChanged(false);
        return;
    }
    emit connectionChanged(!runtime_started_axes_.isEmpty());
    emit logMessage(current_transport_tag(),QString("Runtime started: %1 axis(es)").arg(runtime_started_axes_.size()));
}

void AxisManager::stopRuntime() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    rr_index_ = 0;
    (void)stopRuntimeInternal();
    runtime_started_axes_.clear();
    emit connectionChanged(false);
    emit logMessage(current_transport_tag(),"Runtime stopped");
}

void AxisManager::scanMotors(int max_id) {
    if (active_transport_ == ActiveTransport::Ethercat) {
        Q_UNUSED(max_id);
        scanEthercatMotors();
        return;
    }
    if (!device_opened_) {
        emit logMessage(current_transport_tag(),"Scan failed: open device first");
        emit scanFinished("mks", {});
        return;
    }
    reset_runtime_state();
    motion_core::MksScanRequest request{};
    request.device_path = opened_device_path_.toStdString();
    request.baud_rate = static_cast<std::uint32_t>(opened_baud_rate_);
    request.max_id = max_id;
    const auto discovered = hal_runtime_.scan_mks_topology(request);
    if (!discovered.ok()) {
        emit logMessage(current_transport_tag(),QString("Scan failed via HalRuntime: %1").arg(discovered.error().message));
        emit scanFinished("mks", {});
        return;
    }
    motion_core::HalRuntimeConfig hal_cfg = discovered.value();
    current_hal_config_ = hal_cfg;
    const auto open_res = openRuntimeFromConfig(hal_cfg);
    if (!open_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Runtime build failed after scan: %1").arg(open_res.error().message));
        emit scanFinished("mks", {});
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) runtime_known_axes_.insert(static_cast<int>(info.id.value));
    }
    const auto start_res = startRuntimeHeadless();
    if (!start_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Runtime start failed after scan: %1").arg(start_res.error().message));
        emit scanFinished("mks", {});
        emit connectionChanged(false);
        return;
    }
    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) out.push_back(id);
    emit scanFinished("mks", out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
}

void AxisManager::scanEthercatMotors() {
    if (active_transport_ != ActiveTransport::Ethercat || !device_opened_) {
        emit logMessage(current_transport_tag(),"EtherCAT scan failed: open EtherCAT interface first");
        emit scanFinished("ecat", {});
        return;
    }
    reset_runtime_state();
    motion_core::EthercatScanRequest request{};
    request.interface_name = opened_device_path_.toStdString();
    const auto discovered = hal_runtime_.scan_ethercat_topology(request);
    if (!discovered.ok()) {
        emit logMessage(current_transport_tag(),QString("EtherCAT scan failed via HalRuntime: %1").arg(discovered.error().message));
        emit scanFinished("ecat", {});
        return;
    }
    motion_core::HalRuntimeConfig hal_cfg = discovered.value();
    current_hal_config_ = hal_cfg;
    const auto open_res = openRuntimeFromConfig(hal_cfg);
    if (!open_res.ok()) {
        emit logMessage(current_transport_tag(),QString("EtherCAT runtime rebuild failed after scan: %1").arg(open_res.error().message));
        emit scanFinished("ecat", {});
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) runtime_known_axes_.insert(static_cast<int>(info.id.value));
    }
    const auto start_res = startRuntimeHeadless();
    if (!start_res.ok()) {
        emit logMessage(current_transport_tag(),QString("EtherCAT runtime start failed after scan: %1").arg(start_res.error().message));
        emit scanFinished("ecat", {});
        emit connectionChanged(false);
        return;
    }
    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) out.push_back(id);
    emit scanFinished("ecat", out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
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
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Enable failed for axis %1: %2").arg(axis_id).arg(axis_res.error().message));
        return;
    }
    const auto result = axis_res.value()->set_enabled(enabled);
    if (!result.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 enable=%2 failed: %3").arg(axis_id).arg(enabled ? 1 : 0).arg(result.error().message));
    }
}

void AxisManager::emergencyStop(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.emergency_stop = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 E-STOP failed: %2").arg(axis_id).arg(result.error().message));
}

void AxisManager::clearErrors(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.clear_errors = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 clear errors failed: %2").arg(axis_id).arg(result.error().message));
        return;
    }
    applySafetyBaselineForAxis(axis_id, "clear_errors", false);
}

void AxisManager::moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.target_position_deg = axis_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    const auto res = axis_res.value()->apply_command(cmd);
    if (!res.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 absolute move failed: %2").arg(axis_id).arg(res.error().message));
}

void AxisManager::moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.is_relative = true;
    cmd.target_position_deg = delta_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    const auto res = axis_res.value()->apply_command(cmd);
    if (!res.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 relative move failed: %2").arg(axis_id).arg(res.error().message));
}

void AxisManager::configureMotionQueue(const int axis_id, const int capacity, const bool drop_oldest) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 configure queue failed: %2")
                            .arg(axis_id)
                            .arg(axis_res.error().message));
        return;
    }
    const auto result = axis_res.value()->configure_motion_queue(
        static_cast<std::size_t>(std::max(1, capacity)),
        drop_oldest);
    if (!result.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 configure queue failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
        return;
    }
    requestMotionQueueStats(axis_id);
}

void AxisManager::enqueueMotionBatch(const int axis_id, const QVariantList& points) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 enqueue batch failed: %2")
                            .arg(axis_id)
                            .arg(axis_res.error().message));
        return;
    }

    std::vector<motion_core::QueuedSetpoint> batch;
    batch.reserve(static_cast<std::size_t>(points.size()));
    for (const auto& point_variant : points) {
        const QVariantMap map = point_variant.toMap();
        if (!map.contains("target_position_deg")) {
            continue;
        }

        motion_core::QueuedSetpoint point{};
        point.target_position_deg = map.value("target_position_deg").toDouble();
        if (map.contains("has_profile_speed_rpm") && map.value("has_profile_speed_rpm").toBool()) {
            point.has_profile_speed_rpm = true;
            point.profile_speed_rpm = static_cast<std::uint16_t>(
                std::clamp(map.value("profile_speed_rpm").toInt(), 0, 3000));
        }
        if (map.contains("has_profile_accel_percent") && map.value("has_profile_accel_percent").toBool()) {
            point.has_profile_accel_percent = true;
            point.profile_accel_percent = std::clamp(map.value("profile_accel_percent").toDouble(), 0.0, 100.0);
        }
        if (map.contains("has_target_velocity") && map.value("has_target_velocity").toBool()) {
            point.has_target_velocity = true;
            point.target_velocity_deg_per_sec = map.value("target_velocity_deg_per_sec").toDouble();
        }
        if (map.contains("sample_period_sec")) {
            point.sample_period_sec = std::clamp(map.value("sample_period_sec").toDouble(), 0.001, 0.100);
        }
        batch.push_back(point);
    }

    if (batch.empty()) {
        return;
    }

    const auto result = axis_res.value()->enqueue_motion_batch(batch);
    if (!result.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 enqueue batch failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
        return;
    }

    emit motionQueueStatsUpdated(axis_id, motionQueueStatsToVariantMap(result.value()));
}

void AxisManager::clearMotionQueue(const int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }
    const auto result = axis_res.value()->clear_motion_queue();
    if (!result.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 clear queue failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
    }
    requestMotionQueueStats(axis_id);
}

void AxisManager::requestMotionQueueStats(const int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }
    const auto stats_result = axis_res.value()->query_motion_queue_stats();
    if (!stats_result.ok()) {
        emit logMessage(current_transport_tag(), QString("Axis %1 queue stats failed: %2")
                            .arg(axis_id)
                            .arg(stats_result.error().message));
        return;
    }
    emit motionQueueStatsUpdated(axis_id, motionQueueStatsToVariantMap(stats_result.value()));
}

void AxisManager::setAxisMode(int axis_id, int mode_code) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisMode mode = motion_core::AxisMode::ProfilePosition;
    switch (mode_code) {
        case 9: mode = motion_core::AxisMode::CyclicSyncVelocity; break;
        case 8: mode = motion_core::AxisMode::CyclicSyncPosition; break;
        case 6: mode = motion_core::AxisMode::Homing; break;
        case 3: mode = motion_core::AxisMode::ProfileVelocity; break;
        default: mode = motion_core::AxisMode::ProfilePosition; break;
    }
    const auto result = axis_res.value()->set_mode(mode);
    if (!result.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 set mode %2 failed: %3").arg(axis_id).arg(mode_code).arg(result.error().message));
}

void AxisManager::setZeroPosition(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.set_zero = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 set zero failed: %2").arg(axis_id).arg(result.error().message));
}

void AxisManager::goHome(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.go_home = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(current_transport_tag(),QString("Axis %1 go home failed: %2").arg(axis_id).arg(result.error().message));
}

QVariantList AxisManager::listParameters(int axis_id) const {
    QVariantList out;
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return out;
    const auto res = axis_res.value()->list_parameters();
    if (!res.ok()) return out;
    for (const auto& desc : res.value()) {
        const auto group = QString::fromUtf8(desc.group);
        if (group.startsWith(QStringLiteral("Internal/"))) {
            continue;
        }
        QVariantMap map;
        map["domain"] = static_cast<int>(desc.id.domain);
        map["value"] = static_cast<int>(desc.id.value);
        map["name"] = QString::fromUtf8(desc.name);
        map["group"] = group;
        map["unit"] = QString::fromUtf8(desc.unit);
        map["read_only"] = desc.read_only;
        map["persistable"] = desc.persistable;
        map["has_min"] = desc.has_min;
        map["has_max"] = desc.has_max;
        if (desc.has_min) map["min_value"] = paramValueToVariant(desc.min_value);
        if (desc.has_max) map["max_value"] = paramValueToVariant(desc.max_value);
        out.push_back(map);
    }
    return out;
}

QVariantList AxisManager::readParameters(int axis_id) {
    QVariantList out;
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return out;

    const auto set_res = axis_res.value()->read_parameters();

    if (!set_res.ok()) return out;
    for (const auto& entry : set_res.value().entries) {
        if (is_internal_parameter(entry.id)) {
            continue;
        }
        QVariantMap map;
        map["domain"] = static_cast<int>(entry.id.domain);
        map["value"] = static_cast<int>(entry.id.value);
        map["data"] = paramValueToVariant(entry.value);
        out.push_back(map);
    }
    return out;
}

void AxisManager::requestListParameters(int axis_id) { emit parameterListReady(axis_id, listParameters(axis_id)); }
void AxisManager::requestReadParameters(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit parametersRead(axis_id, {});
        return;
    }

    const auto transport = axis_res.value()->info().transport;
    if (transport != motion_core::AxisTransportKind::Ethercat) {
        emit parametersRead(axis_id, readParameters(axis_id));
        return;
    }

    if (parameter_reads_in_progress_.contains(axis_id)) {
        emit logMessage(current_transport_tag(),QString("Axis %1 parameter read already in progress").arg(axis_id));
        return;
    }

    parameter_reads_in_progress_.insert(axis_id);
    emit logMessage(current_transport_tag(),QString("Axis %1 EtherCAT parameter read started").arg(axis_id));

    QPointer<AxisManager> self(this);
    auto axis = axis_res.value();
    std::thread([self, axis_id, axis]() {
        QVariantList out;
        const auto set_res = axis->read_parameters();
        if (set_res.ok()) {
            for (const auto& entry : set_res.value().entries) {
                QVariantMap map;
                map["domain"] = static_cast<int>(entry.id.domain);
                map["value"] = static_cast<int>(entry.id.value);
                map["data"] = paramValueToVariant(entry.value);
                out.push_back(map);
            }
        }

        if (!self) {
            return;
        }

        if (set_res.ok()) {
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, out]() {
                    if (!self) return;
                    self->completeParameterRead(axis_id, out);
                },
                Qt::QueuedConnection);
            return;
        }

        const QString error_message = QString::fromStdString(set_res.error().message);
        QMetaObject::invokeMethod(
            self,
            [self, axis_id, error_message]() {
                if (!self) return;
                self->completeParameterReadFailure(axis_id, error_message);
            },
            Qt::QueuedConnection);
    }).detach();
}

void AxisManager::completeParameterRead(int axis_id, QVariantList params) {
    parameter_reads_in_progress_.remove(axis_id);
    emit logMessage(current_transport_tag(),QString("Axis %1 EtherCAT parameter read completed: %2 entries")
                        .arg(axis_id)
                        .arg(params.size()));
    emit parametersRead(axis_id, params);
}

void AxisManager::completeParameterReadFailure(int axis_id, QString message) {
    parameter_reads_in_progress_.remove(axis_id);
    emit logMessage(current_transport_tag(),QString("Axis %1 EtherCAT parameter read failed: %2").arg(axis_id).arg(message));
    emit parametersRead(axis_id, {});
}

void AxisManager::setPersistentParameter(int axis_id, int domain, int value, const QVariant& data) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: axis not found").arg(axis_id));
        return;
    }

    const auto command_result = persistent_command_for_parameter(domain, value);
    if (command_result.ok()) {
        const auto persistent_command = command_result.value();

        bool parse_ok = false;
        const std::int64_t raw_value = data.toLongLong(&parse_ok);
        if (!parse_ok) {
            emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: integer value expected").arg(axis_id));
            return;
        }

        if (parameter_writes_in_progress_.contains(axis_id)) {
            emit logMessage(current_transport_tag(),QString("Axis %1 parameter write already in progress").arg(axis_id));
            return;
        }

        parameter_writes_in_progress_.insert(axis_id);
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent %2 write started")
                            .arg(axis_id)
                            .arg(persistent_command_name(persistent_command)));

        QPointer<AxisManager> self(this);
        auto axis = axis_res.value();
        std::thread([self, axis_id, axis, persistent_command, raw_value]() {
            const auto persistent_res = axis->set_persistent(
                persistent_command,
                motion_core::ParameterValue::from_signed(raw_value));
            if (!self) {
                return;
            }

            if (!persistent_res.ok()) {
                const QString message = QString("Axis %1 persistent %2 write failed: %3")
                                            .arg(axis_id)
                                            .arg(persistent_command_name(persistent_command))
                                            .arg(QString::fromStdString(persistent_res.error().message));
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            const QString message = persistent_write_report_message(
                axis_id,
                persistent_command,
                persistent_res.value());
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteSuccess(axis_id, message, true);
                },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    if (domain == static_cast<int>(motion_core::ParameterDomain::Mks)) {
        const auto parameter_id = motion_core::make_parameter_id(
            motion_core::ParameterDomain::Mks,
            static_cast<std::uint32_t>(value));

        const auto descriptors_result = axis_res.value()->list_parameters();
        if (!descriptors_result.ok()) {
            emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: %2")
                                .arg(axis_id)
                                .arg(QString::fromStdString(descriptors_result.error().message)));
            return;
        }

        const motion_core::ParameterDescriptor* selected_descriptor = nullptr;
        for (const auto& descriptor : descriptors_result.value()) {
            if (descriptor.id.domain == parameter_id.domain && descriptor.id.value == parameter_id.value) {
                selected_descriptor = &descriptor;
                break;
            }
        }
        if (!selected_descriptor || selected_descriptor->read_only || !selected_descriptor->persistable) {
            emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: descriptor is read-only or missing").arg(axis_id));
            return;
        }

        motion_core::ParameterPatch patch{};
        patch.entries.push_back({parameter_id, variantToParamValue(data, selected_descriptor)});

        if (parameter_writes_in_progress_.contains(axis_id)) {
            emit logMessage(current_transport_tag(),QString("Axis %1 parameter write already in progress").arg(axis_id));
            return;
        }

        parameter_writes_in_progress_.insert(axis_id);
        const auto parameter_label = QString::fromUtf8(selected_descriptor->name);
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent %2 write started")
                            .arg(axis_id)
                            .arg(parameter_label));

        QPointer<AxisManager> self(this);
        auto axis = axis_res.value();
        const auto requested_value = patch.entries.front().value;
        std::thread([self, axis_id, axis, patch, parameter_id, requested_value, parameter_label]() {
            const auto apply_result = axis->apply_parameter_patch(patch);
            if (!self) {
                return;
            }

            if (!apply_result.ok()) {
                const QString message = QString("Axis %1 persistent %2 write failed: %3")
                                            .arg(axis_id)
                                            .arg(parameter_label)
                                            .arg(QString::fromStdString(apply_result.error().message));
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            const auto readback_result = axis->read_parameters();
            if (!readback_result.ok()) {
                const QString message = QString("Axis %1 persistent %2 write failed: %3")
                                            .arg(axis_id)
                                            .arg(parameter_label)
                                            .arg(QString::fromStdString(readback_result.error().message));
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            motion_core::PersistentWriteReport report{};
            report.command_supported = true;
            report.write_completed = true;
            report.persistent_save_completed = true; // MKS intrinsically saves to flash on write
            report.power_cycle_required = false;

            bool found_readback = false;
            for (const auto& entry : readback_result.value().entries) {
                if (entry.id.domain == parameter_id.domain && entry.id.value == parameter_id.value) {
                    report.readback_value = entry.value;
                    report.readback_verified = parameter_value_matches_requested(requested_value, entry.value);
                    found_readback = true;
                    break;
                }
            }

            if (!found_readback) {
                const QString message = QString("Axis %1 persistent %2 write failed: readback parameter not found")
                                            .arg(axis_id)
                                            .arg(parameter_label);
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            if (!report.readback_verified) {
                const QString message = QString("Axis %1 persistent %2 write failed: readback does not match requested value")
                                            .arg(axis_id)
                                            .arg(parameter_label);
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            const QString message = persistent_write_report_message(
                axis_id,
                parameter_label,
                report);
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteSuccess(axis_id, message, true);
                },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    if (domain != static_cast<int>(motion_core::ParameterDomain::Ethercat)) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: unsupported parameter").arg(axis_id));
        return;
    }

    const auto parameter_id = motion_core::make_parameter_id(
        motion_core::ParameterDomain::Ethercat,
        static_cast<std::uint32_t>(value));
    if (is_internal_parameter(parameter_id)) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: internal parameter cannot be persisted directly").arg(axis_id));
        return;
    }

    const auto* definition = ethercat_driver::find_ethercat_parameter_definition(parameter_id);
    if (!definition || definition->is_read_only || !definition->persistable_runtime) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: parameter is not writable/persistable").arg(axis_id));
        return;
    }

    const auto descriptors_result = axis_res.value()->list_parameters();
    if (!descriptors_result.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: %2")
                            .arg(axis_id)
                            .arg(QString::fromStdString(descriptors_result.error().message)));
        return;
    }

    const motion_core::ParameterDescriptor* selected_descriptor = nullptr;
    for (const auto& descriptor : descriptors_result.value()) {
        if (descriptor.id.domain == parameter_id.domain && descriptor.id.value == parameter_id.value) {
            selected_descriptor = &descriptor;
            break;
        }
    }
    if (!selected_descriptor || selected_descriptor->read_only || !selected_descriptor->persistable) {
        emit logMessage(current_transport_tag(),QString("Axis %1 persistent write failed: descriptor is read-only or missing").arg(axis_id));
        return;
    }

    motion_core::ParameterPatch patch{};
    patch.entries.push_back({parameter_id, variantToParamValue(data, selected_descriptor)});
    patch.entries.push_back({
        motion_core::make_parameter_id(ethercat_driver::EthercatParameter::SaveParametersToEeprom),
        motion_core::ParameterValue::from_signed(1)});

    if (parameter_writes_in_progress_.contains(axis_id)) {
        emit logMessage(current_transport_tag(),QString("Axis %1 parameter write already in progress").arg(axis_id));
        return;
    }

    parameter_writes_in_progress_.insert(axis_id);
    const auto parameter_label = QString::fromUtf8(selected_descriptor->name);
    emit logMessage(current_transport_tag(),QString("Axis %1 persistent %2 write started")
                        .arg(axis_id)
                        .arg(parameter_label));

    QPointer<AxisManager> self(this);
    auto axis = axis_res.value();
    const auto requested_value = patch.entries.front().value;
    std::thread([self, axis_id, axis, patch, parameter_id, requested_value, parameter_label]() {
        const auto apply_result = axis->apply_parameter_patch(patch);
        if (!self) {
            return;
        }

        if (!apply_result.ok()) {
            const QString message = QString("Axis %1 persistent %2 write failed: %3")
                                        .arg(axis_id)
                                        .arg(parameter_label)
                                        .arg(QString::fromStdString(apply_result.error().message));
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteFailure(axis_id, message);
                },
                Qt::QueuedConnection);
            return;
        }

        const auto readback_result = axis->read_parameters();
        if (!readback_result.ok()) {
            const QString message = QString("Axis %1 persistent %2 write failed: %3")
                                        .arg(axis_id)
                                        .arg(parameter_label)
                                        .arg(QString::fromStdString(readback_result.error().message));
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteFailure(axis_id, message);
                },
                Qt::QueuedConnection);
            return;
        }

        motion_core::PersistentWriteReport report{};
        report.command_supported = true;
        report.write_completed = true;
        report.persistent_save_completed = true;
        report.power_cycle_required = true;

        bool found_readback = false;
        for (const auto& entry : readback_result.value().entries) {
            if (entry.id.domain == parameter_id.domain && entry.id.value == parameter_id.value) {
                report.readback_value = entry.value;
                report.readback_verified = parameter_value_matches_requested(requested_value, entry.value);
                found_readback = true;
                break;
            }
        }

        if (!found_readback) {
            const QString message = QString("Axis %1 persistent %2 write failed: readback parameter not found")
                                        .arg(axis_id)
                                        .arg(parameter_label);
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteFailure(axis_id, message);
                },
                Qt::QueuedConnection);
            return;
        }

        if (!report.readback_verified) {
            const QString message = QString("Axis %1 persistent %2 write failed: readback does not match requested value")
                                        .arg(axis_id)
                                        .arg(parameter_label);
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteFailure(axis_id, message);
                },
                Qt::QueuedConnection);
            return;
        }

        const QString message = persistent_write_report_message(
            axis_id,
            parameter_label,
            report);
        QMetaObject::invokeMethod(
            self,
            [self, axis_id, message]() {
                if (!self) return;
                self->completeParameterWriteSuccess(axis_id, message, true);
            },
            Qt::QueuedConnection);
    }).detach();
}

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch_entries) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(current_transport_tag(),"applyParameterPatch: axis not found");
        return;
    }
    std::unordered_map<std::uint32_t, motion_core::ParameterDescriptor> descriptors;
    const auto list_res = axis_res.value()->list_parameters();
    if (list_res.ok()) {
        for (const auto& d : list_res.value()) {
            const auto key = (static_cast<std::uint32_t>(d.id.domain) << 24) | (d.id.value & 0x00FFFFFFu);
            descriptors[key] = d;
        }
    }
    motion_core::ParameterPatch patch{};
    for (const auto& v : patch_entries) {
        const QVariantMap map = v.toMap();
        if (!map.contains("domain") || !map.contains("value") || !map.contains("data")) continue;
        motion_core::ParameterEntry entry{};
        entry.id.domain = static_cast<motion_core::ParameterDomain>(map["domain"].toInt());
        entry.id.value = static_cast<std::uint32_t>(map["value"].toInt());
        const auto key = (static_cast<std::uint32_t>(entry.id.domain) << 24) | (entry.id.value & 0x00FFFFFFu);
        const auto it = descriptors.find(key);
        const motion_core::ParameterDescriptor* desc = (it != descriptors.end()) ? &it->second : nullptr;
        entry.value = variantToParamValue(map["data"], desc);
        patch.entries.push_back(entry);
    }
    if (patch.entries.empty()) return;
    const auto transport = axis_res.value()->info().transport;
    if (transport == motion_core::AxisTransportKind::Ethercat) {
        if (parameter_writes_in_progress_.contains(axis_id)) {
            emit logMessage(current_transport_tag(),QString("Axis %1 parameter write already in progress").arg(axis_id));
            return;
        }
        parameter_writes_in_progress_.insert(axis_id);
        emit logMessage(current_transport_tag(),QString("Axis %1 EtherCAT parameter patch started (%2 entries)")
                            .arg(axis_id)
                            .arg(patch.entries.size()));

        QPointer<AxisManager> self(this);
        auto axis = axis_res.value();
        std::thread([self, axis_id, axis, patch]() {
            const auto result = axis->apply_parameter_patch(patch);
            if (!self) {
                return;
            }
            if (!result.ok()) {
                const QString message = QString("Axis %1 apply patch error: %2")
                                            .arg(axis_id)
                                            .arg(QString::fromStdString(result.error().message));
                QMetaObject::invokeMethod(
                    self,
                    [self, axis_id, message]() {
                        if (!self) return;
                        self->completeParameterWriteFailure(axis_id, message);
                    },
                    Qt::QueuedConnection);
                return;
            }

            const QString message = QString("Axis %1 applied patch with %2 entries")
                                        .arg(axis_id)
                                        .arg(patch.entries.size());
            QMetaObject::invokeMethod(
                self,
                [self, axis_id, message]() {
                    if (!self) return;
                    self->completeParameterWriteSuccess(axis_id, message, false);
                },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    const auto result = axis_res.value()->apply_parameter_patch(patch);
    if (!result.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 apply patch error: %2").arg(axis_id).arg(result.error().message));
    } else {
        emit logMessage(current_transport_tag(),QString("Axis %1 applied patch with %2 entries").arg(axis_id).arg(patch.entries.size()));
        applySafetyBaselineForAxis(axis_id, "apply_parameter_patch", false);
    }
}

void AxisManager::completeParameterWriteSuccess(int axis_id, QString message, bool refresh_after_write) {
    parameter_writes_in_progress_.remove(axis_id);
    emit logMessage(current_transport_tag(),message);
    applySafetyBaselineForAxis(axis_id, "ethercat_async_parameter_write", false);
    if (refresh_after_write) {
        requestReadParameters(axis_id);
    }
}

void AxisManager::completeParameterWriteFailure(int axis_id, QString message) {
    parameter_writes_in_progress_.remove(axis_id);
    emit logMessage(current_transport_tag(),message);
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    const auto save_res = hal_runtime_.export_axis_config_to_file(static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!save_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 save config failed: %2").arg(axis_id).arg(save_res.error().message));
    } else {
        emit logMessage(current_transport_tag(),QString("Axis %1 config exported to %2").arg(axis_id).arg(path));
    }
}

void AxisManager::importAxisConfigPreview(int axis_id, const QString& path) {
    const auto patch_res = hal_runtime_.build_axis_config_patch(static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!patch_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 preview config failed: %2")
                            .arg(axis_id)
                            .arg(patch_res.error().message));
        emit axisConfigPreviewReady(axis_id, {});
        return;
    }

    QVariantList preview_entries;
    for (const auto& entry : patch_res.value().entries) {
        QVariantMap map;
        map["domain"] = static_cast<int>(entry.id.domain);
        map["value"] = static_cast<int>(entry.id.value);
        map["data"] = paramValueToVariant(entry.value);
        preview_entries.push_back(map);
    }

    emit axisConfigPreviewReady(axis_id, preview_entries);
    emit logMessage(current_transport_tag(),QString("Axis %1 config preview loaded from %2 (%3 entries)")
                        .arg(axis_id)
                        .arg(path)
                        .arg(preview_entries.size()));
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    const auto patch_res = hal_runtime_.build_axis_config_patch(static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!patch_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 apply config failed: %2")
                            .arg(axis_id)
                            .arg(patch_res.error().message));
        return;
    }

    const auto apply_res = hal_runtime_.apply_axis_config_patch(static_cast<std::uint16_t>(axis_id), patch_res.value());
    if (!apply_res.ok()) {
        emit logMessage(current_transport_tag(),QString("Axis %1 apply config failed: %2").arg(axis_id).arg(apply_res.error().message));
    } else {
        emit logMessage(current_transport_tag(),QString("Axis %1 config imported from %2").arg(axis_id).arg(path));
        applySafetyBaselineForAxis(axis_id, "import_axis_config", false);
    }
}

void AxisManager::onFastTick() {
    if (watched_axes_.isEmpty()) return;
    const QList<int> ids = watched_axes_.values();
    if (ids.isEmpty()) return;

    QVector<int> poll_order;
    poll_order.reserve(2);

    const bool has_priority = ui_priority_axis_id_ > 0 && watched_axes_.contains(ui_priority_axis_id_);
    if (has_priority) {
        poll_order.push_back(ui_priority_axis_id_);
    }

    rr_index_ %= ids.size();
    int rr_axis_id = ids.at(rr_index_);
    rr_index_ = (rr_index_ + 1) % ids.size();
    if (!poll_order.contains(rr_axis_id)) {
        poll_order.push_back(rr_axis_id);
    }

    for (const int axis_id : poll_order) {
        const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
        if (!axis_res.ok()) {
            continue;
        }

        const auto telemetry_res = axis_res.value()->read_telemetry();
        if (!telemetry_res.ok()) {
            continue;
        }

        QVariantMap t;
        const auto& telemetry = telemetry_res.value();
        t["status"] = static_cast<int>(telemetry.status_word);
        t["state"] = static_cast<int>(telemetry.state);
        t["axis"] = static_cast<double>(telemetry.actual_position_deg);
        t["target"] = static_cast<double>(telemetry.target_position_deg);
        t["speed"] = static_cast<double>(telemetry.actual_velocity_deg_per_sec);
        t["torque"] = static_cast<double>(telemetry.actual_torque_percent);
        t["protection"] = static_cast<int>(telemetry.protection_code);
        t["motion_status"] = static_cast<int>(telemetry.motion_status_code);
        t["error_code"] = static_cast<int>(telemetry.protection_code);
        t["timestamp_ns"] = static_cast<qulonglong>(telemetry.timestamp_ns);
        const auto transport = axis_res.value()->info().transport;
        t["transport"] = (transport == motion_core::AxisTransportKind::CanBus)
            ? QStringLiteral("mks")
            : (transport == motion_core::AxisTransportKind::Ethercat
                ? QStringLiteral("ethercat")
                : QStringLiteral("unknown"));

        if (transport == motion_core::AxisTransportKind::CanBus) {
            const auto mks_axis = std::dynamic_pointer_cast<mks::MksAxisAdapter>(axis_res.value());
            if (mks_axis) {
                const auto metrics = mks_axis->cycle_metrics_snapshot();
                t["cmd_tx_hz"] = metrics.command_tx.rate_hz;
                t["cmd_tx_period_ms"] = metrics.command_tx.last_period_ms;
                t["telemetry_publish_hz"] = metrics.telemetry_publish.rate_hz;
                t["telemetry_publish_period_ms"] = metrics.telemetry_publish.last_period_ms;
                t["position_rx_hz"] = metrics.position_rx.rate_hz;
                t["position_rx_period_ms"] = metrics.position_rx.last_period_ms;
                t["speed_rx_hz"] = metrics.speed_rx.rate_hz;
                t["status_rx_hz"] = metrics.status_rx.rate_hz;
                t["protection_rx_hz"] = metrics.protection_rx.rate_hz;

                std::vector<mks::AxisPositionSample> drained_samples;
                drained_samples.reserve(kMaxUiPositionSamplesPerTick);
                const auto drained = mks_axis->drain_position_samples(drained_samples, kMaxUiPositionSamplesPerTick);
                if (drained > 0U) {
                    QVariantList samples_list;
                    samples_list.reserve(static_cast<int>(drained_samples.size()));
                    for (const auto& sample : drained_samples) {
                        QVariantMap sample_map;
                        sample_map["timestamp_ns"] = static_cast<qulonglong>(sample.timestamp_ns);
                        sample_map["position_deg"] = sample.position_deg;
                        samples_list.push_back(sample_map);
                    }
                    t["position_samples"] = samples_list;
                }
            }
        }

        emit telemetryUpdated(axis_id, t);
    }
}

} // namespace mks
