#pragma once

#include <QObject>
#include <QVariantList>
#include <QVariantMap>
#include "motion_core/telemetry_distributor.h"
#include "motion_core/parameter_manager.h"
#include "motion_core/hal_runtime.h"

namespace mks {

class AxisTelemetryController final : public QObject {
    Q_OBJECT

public:
    explicit AxisTelemetryController(
        motion_core::HalRuntime& runtime,
        motion_core::TelemetryDistributor& telemetry,
        motion_core::ParameterManager& parameter_manager,
        QObject* parent = nullptr);
    ~AxisTelemetryController() override;

    void requestListParameters(int axis_id);
    void requestReadParameters(int axis_id);
    void applyParameterPatch(int axis_id, const QVariantList& patch);
    void setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data);
    void setWatchedAxes(const std::vector<uint16_t>& axis_ids);

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void telemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void motionQueueStatsUpdated(int axis_id, const QVariantMap& stats);
    void parameterListReady(int axis_id, const QVariantList& params);
    void parametersRead(int axis_id, const QVariantList& params);
    void parameterPatchCompleted(int axis_id, bool success, const QString& message);
    void persistentParameterCompleted(int axis_id, bool success, const QString& message);

private:
    motion_core::HalRuntime& unified_runtime_;
    motion_core::TelemetryDistributor& telemetry_;
    motion_core::ParameterManager& parameter_manager_;
    int telemetry_token_{0};
};

} // namespace mks