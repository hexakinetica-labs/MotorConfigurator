#pragma once

#include <QObject>
#include <QString>
#include <QVariant>
#include <QVariantList>
#include <atomic>
#include "motion_core/config_manager.h"
#include "motion_core/hal_runtime.h"

namespace mks {

class AxisConfigController final : public QObject {
    Q_OBJECT

public:
    explicit AxisConfigController(
        motion_core::HalRuntime& runtime,
        motion_core::ConfigManager& config_manager,
        QObject* parent = nullptr);
    ~AxisConfigController() override = default;

    void loadHalConfig(const QString& config_path);
    void saveHalConfig(const QString& config_path);
    void exportAxisConfig(int axis_id, const QString& path);
    void importAxisConfigPreview(int axis_id, const QString& path);
    void importAxisConfig(int axis_id, const QString& path);

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void halConfigLoaded(const motion_core::HalRuntimeConfig& config);
    void axisConfigPreviewReady(int axis_id, const QVariantList& patch_entries);

private:
    bool tryStartConfigOperation(const QString& operation_name);
    void finishConfigOperation();

    motion_core::HalRuntime& unified_runtime_;
    motion_core::ConfigManager& config_manager_;
    std::atomic<bool> config_operation_active_{false};
};

} // namespace mks