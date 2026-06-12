#include "mks/axis_manager/axis_config_controller.h"
#include "mks/axis_manager/parameter_ui_utils.h"
#include <QVariantList>
#include <QVariantMap>
#include <QPointer>
#include <QMetaObject>
#include <thread>

namespace mks {

AxisConfigController::AxisConfigController(
    motion_core::HalRuntime& runtime,
    motion_core::ConfigManager& config_manager,
    QObject* parent)
    : QObject(parent)
    , unified_runtime_(runtime)
    , config_manager_(config_manager)
{}

bool AxisConfigController::tryStartConfigOperation(const QString& operation_name) {
    bool expected = false;
    if (!config_operation_active_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Configuration operation '%1' rejected: another config operation is already running")
                            .arg(operation_name));
        return false;
    }
    return true;
}

void AxisConfigController::finishConfigOperation() {
    config_operation_active_.store(false, std::memory_order_release);
}

void AxisConfigController::loadHalConfig(const QString& config_path) {
    auto hal_cfg_res = config_manager_.load_master_config(config_path.toStdString());
    if (!hal_cfg_res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("HAL config load failed: %1").arg(QString::fromStdString(hal_cfg_res.error().message)));
        return;
    }

    emit halConfigLoaded(hal_cfg_res.value());
}

void AxisConfigController::saveHalConfig(const QString& config_path) {
    auto res = config_manager_.save_master_config(config_path.toStdString());
    if (!res.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Configuration save failed: %1").arg(QString::fromStdString(res.error().message)));
    } else {
        emit logMessage(QStringLiteral("hal"), QString("Saved HAL config project to %1").arg(config_path));
    }
}

void AxisConfigController::exportAxisConfig(int axis_id, const QString& path) {
    if (!tryStartConfigOperation(QStringLiteral("export axis config"))) {
        return;
    }
    QPointer<AxisConfigController> safeThis(this);
    std::thread([safeThis, axis_id, path, &config_manager = config_manager_]() {
        auto res = config_manager.export_axis_config(static_cast<std::uint16_t>(axis_id), path.toStdString());
        if (!safeThis) return;
        QMetaObject::invokeMethod(safeThis, [safeThis, axis_id, path, res = std::move(res)]() {
            if (!safeThis) return;
            safeThis->finishConfigOperation();
            if (!res.ok()) {
                emit safeThis->logMessage(QStringLiteral("hal"),
                                QString("Axis %1 export config failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(res.error().message)));
            } else {
                emit safeThis->logMessage(QStringLiteral("hal"),
                                QString("Axis %1 config exported to %2").arg(axis_id).arg(path));
            }
        }, Qt::QueuedConnection);
    }).detach();
}

void AxisConfigController::importAxisConfigPreview(int axis_id, const QString& path) {
    if (!tryStartConfigOperation(QStringLiteral("import axis config preview"))) {
        return;
    }
    QPointer<AxisConfigController> safeThis(this);
    std::thread([safeThis, axis_id, path, &config_manager = config_manager_]() {
        auto patch_res = config_manager.preview_axis_config(static_cast<std::uint16_t>(axis_id), path.toStdString());
        if (!safeThis) return;
        QMetaObject::invokeMethod(safeThis, [safeThis, axis_id, path, patch_res = std::move(patch_res)]() mutable {
            if (!safeThis) return;
            safeThis->finishConfigOperation();
            if (!patch_res.ok()) {
                emit safeThis->logMessage(QStringLiteral("hal"),
                                QString("Axis %1 import config preview failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(patch_res.error().message)));
                return;
            }

            QVariantList out;
            for (const auto& entry : patch_res.value().entries) {
                QVariantMap item;
                item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
                item.insert(QStringLiteral("value"),  static_cast<int>(entry.id.value));
                item.insert(QStringLiteral("data"), param_value_to_string(entry.value));
                out.push_back(item);
            }
            emit safeThis->axisConfigPreviewReady(axis_id, out);
        }, Qt::QueuedConnection);
    }).detach();
}

void AxisConfigController::importAxisConfig(int axis_id, const QString& path) {
    if (!tryStartConfigOperation(QStringLiteral("import axis config"))) {
        return;
    }
    QPointer<AxisConfigController> safeThis(this);
    std::thread([safeThis, axis_id, path, &config_manager = config_manager_]() {
        auto res = config_manager.apply_axis_config(static_cast<std::uint16_t>(axis_id), path.toStdString());
        if (!safeThis) return;
        QMetaObject::invokeMethod(safeThis, [safeThis, axis_id, path, res = std::move(res)]() {
            if (!safeThis) return;
            safeThis->finishConfigOperation();
            if (!res.ok()) {
                emit safeThis->logMessage(QStringLiteral("hal"),
                                QString("Axis %1 import config failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(res.error().message)));
            } else {
                emit safeThis->logMessage(QStringLiteral("hal"),
                                QString("Axis %1 config imported from %2").arg(axis_id).arg(path));
            }
        }, Qt::QueuedConnection);
    }).detach();
}

} // namespace mks