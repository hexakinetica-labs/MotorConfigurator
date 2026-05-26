#include "mks/axis_manager/axis_manager.h"

#include "motion_core/config/axis_config_json.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <filesystem>
#include <unordered_set>

namespace {

namespace fs = std::filesystem;

constexpr const char* kHalProjectMasterFileName = "hal_config.json";
constexpr const char* kHalProjectAxisDirectoryName = "axes";
constexpr const char* kMksTransportBusRef = "mks_can";
constexpr const char* kEthercatTransportBusRef = "ethercat";

[[nodiscard]] static fs::path hal_project_master_path(const fs::path& project_dir) {
    return project_dir / kHalProjectMasterFileName;
}

[[nodiscard]] static fs::path hal_project_axis_relative_path(const std::uint16_t axis_id) {
    return fs::path(kHalProjectAxisDirectoryName) / (std::string("axis_") + std::to_string(axis_id) + ".json");
}

[[nodiscard]] static fs::path resolve_config_reference(const fs::path& master_config_path,
                                                       const std::string& config_reference) {
    const fs::path config_path(config_reference);
    if (config_path.is_absolute()) {
        return config_path;
    }
    return master_config_path.parent_path() / config_path;
}

[[nodiscard]] static motion_core::Result<void> export_axis_config_for_hal_project(
    const motion_core::HalRuntime& runtime,
    const std::uint16_t axis_id,
    const std::string& path) {
    return runtime.export_axis_config_to_file(axis_id, path);
}

[[nodiscard]] static bool has_duplicate_axis_ids(const motion_core::HalRuntimeConfig& cfg,
                                                 std::uint16_t* duplicate_id = nullptr) {
    std::unordered_set<std::uint16_t> ids;
    ids.reserve(cfg.axes.size());
    for (const auto& axis : cfg.axes) {
        if (!axis.axis_id.valid()) {
            continue;
        }
        if (!ids.insert(axis.axis_id.value).second) {
            if (duplicate_id) {
                *duplicate_id = axis.axis_id.value;
            }
            return true;
        }
    }
    return false;
}

} // namespace

namespace mks {

void AxisManager::loadHalConfig(const QString& config_path) {
    const auto hal_cfg = motion_core::load_hal_runtime_config_from_file(config_path.toStdString());
    if (!hal_cfg.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("HAL config load failed: %1").arg(hal_cfg.error().message));
        return;
    }

    std::uint16_t duplicate_id = 0U;
    if (has_duplicate_axis_ids(hal_cfg.value(), &duplicate_id)) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("HAL config load failed: duplicate global axis_id=%1")
                            .arg(static_cast<int>(duplicate_id)));
        return;
    }

    reset_runtime_state();
    current_hal_config_ = hal_cfg.value();
    const fs::path master_config_path(config_path.toStdString());

    for (auto& bus : current_hal_config_.mks_buses) {
        bus.interface_id = kMksTransportBusRef;
    }
    for (auto& axis : current_hal_config_.axes) {
        if (axis.transport == motion_core::AxisTransportKind::CanBus) {
            axis.bus_ref = kMksTransportBusRef;
        } else if (axis.transport == motion_core::AxisTransportKind::Ethercat) {
            axis.bus_ref = kEthercatTransportBusRef;
        }

        if (!axis.config_file.empty()) {
            const fs::path resolved = resolve_config_reference(master_config_path, axis.config_file);
            axis.config_file = resolved.string();
        }
    }

    publishTransportOpenStates();
    rebuildRuntimeFromCurrentConfig();

    emit logMessage(QStringLiteral("hal"),
                    QString("HAL config loaded: total=%1 (MKS=%2, EtherCAT=%3), live transports: MKS=%4, EtherCAT=%5")
                        .arg(current_hal_config_.axes.size())
                        .arg(current_hal_config_.mks_buses.size())
                        .arg(current_hal_config_.ethercat_buses.size())
                        .arg(mks_device_opened_ ? QStringLiteral("open") : QStringLiteral("closed"))
                        .arg(ethercat_device_opened_ ? QStringLiteral("open") : QStringLiteral("closed")));
}

void AxisManager::saveHalConfig(const QString& config_path) {
    if (!unified_runtime_.is_active()) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Configuration save failed: runtime must be active to export per-axis configs"));
        return;
    }

    current_hal_config_ = unified_runtime_.current_config();
    const fs::path project_dir(config_path.toStdString());
    std::error_code ec;
    fs::create_directories(project_dir / kHalProjectAxisDirectoryName, ec);
    if (ec) {
        emit logMessage(QStringLiteral("hal"),
                        QStringLiteral("Configuration save failed: cannot create project directory %1")
                            .arg(QString::fromStdString((project_dir / kHalProjectAxisDirectoryName).string())));
        return;
    }

    for (auto& axis : current_hal_config_.axes) {
        if (!axis.axis_id.valid()) {
            continue;
        }

        const auto relative_axis_config_path = hal_project_axis_relative_path(axis.axis_id.value);
        const auto absolute_axis_config_path = project_dir / relative_axis_config_path;
        const auto export_res = export_axis_config_for_hal_project(unified_runtime_,
                                                                   axis.axis_id.value,
                                                                   absolute_axis_config_path.string());
        if (!export_res.ok()) {
            emit logMessage(QStringLiteral("hal"),
                            QStringLiteral("Axis %1 config export failed: %2")
                                .arg(static_cast<int>(axis.axis_id.value))
                                .arg(QString::fromStdString(export_res.error().message)));
            return;
        }

        axis.config_file = relative_axis_config_path.generic_string();
    }

    const auto master_config_path = hal_project_master_path(project_dir);
    const auto res = motion_core::save_hal_runtime_config_to_file(master_config_path.string(), current_hal_config_);
    if (!res.ok())
        emit logMessage(QStringLiteral("hal"),
                        QString("Configuration save failed: %1").arg(res.error().message));
    else
        emit logMessage(QStringLiteral("hal"),
                        QString("Saved HAL config project to %1 (master=%2)")
                            .arg(config_path)
                            .arg(QString::fromStdString(master_config_path.filename().string())));
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    const auto res = unified_runtime_.export_axis_config_to_file(
        static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!res.ok())
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 export config failed: %2")
                            .arg(axis_id).arg(QString::fromStdString(res.error().message)));
    else
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 config exported to %2").arg(axis_id).arg(path));
}

void AxisManager::importAxisConfigPreview(int axis_id, const QString& path) {
    const auto patch_res = unified_runtime_.build_axis_config_patch(
        static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!patch_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 import config preview failed: %2")
                            .arg(axis_id).arg(QString::fromStdString(patch_res.error().message)));
        return;
    }

    QVariantList out;
    for (const auto& entry : patch_res.value().entries) {
        QVariantMap item;
        item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
        item.insert(QStringLiteral("value"),  static_cast<int>(entry.id.value));
        switch (entry.value.type) {
            case motion_core::ParameterValueType::SignedInteger:
                item.insert(QStringLiteral("data"), QString::number(entry.value.signed_value)); break;
            case motion_core::ParameterValueType::UnsignedInteger:
                item.insert(QStringLiteral("data"), QString::number(entry.value.unsigned_value)); break;
            case motion_core::ParameterValueType::FloatingPoint:
                item.insert(QStringLiteral("data"), QString::number(entry.value.floating_value)); break;
            case motion_core::ParameterValueType::Boolean:
                item.insert(QStringLiteral("data"),
                            entry.value.bool_value ? QStringLiteral("true") : QStringLiteral("false")); break;
        }
        out.push_back(item);
    }
    emit axisConfigPreviewReady(axis_id, out);
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    const auto res = unified_runtime_.apply_axis_config_file(
        static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!res.ok())
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 import config failed: %2")
                            .arg(axis_id).arg(QString::fromStdString(res.error().message)));
    else
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 config imported from %2").arg(axis_id).arg(path));
}

} // namespace mks
