#pragma once

#include "mks/axis_workspace/axis_workspace.h"

class MksAxisWorkspace final : public AxisWorkspace {
public:
    explicit MksAxisWorkspace(int axis_id,
                              mks::AxisManager* manager,
                              QWidget* parent = nullptr);
    ~MksAxisWorkspace() override;

private:
    void configureTransportUi() override;
    void onTransportSineToggled(bool enabled) override;
    void onTransportMotionQueueStatsUpdated(const QVariantMap& stats) override;
    void onTransportTelemetryUpdated(const QVariantMap& telemetry,
                                     const QString& transport,
                                     double t_sec) override;
    bool transportOwnsTargetUi() const override;
    bool transportProvidesTargetTrace() const override;
    void onBeforeDisableAxis() override;
    void onBeforeHomeAxis() override;
    void onBeforeSetZeroAxis() override;

    double samplePeriodSec() const override;
    QString transportTag() const override;

    static constexpr int kDefaultMksSpeedRpm = 1800;
    static constexpr int kDefaultMksAccelPercent = 100;
};
