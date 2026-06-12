#pragma once

#include <QObject>
#include <QVariantList>
#include <memory>

#include "motion_core/axis_interface.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/motion_orchestrator.h"
#include "motion_core/trajectory_generator.h"

namespace mks {

class AxisMotionController final : public QObject {
    Q_OBJECT

public:
    explicit AxisMotionController(
        motion_core::HalRuntime& runtime,
        motion_core::MotionOrchestrator& orchestrator,
        motion_core::TrajectoryGenerator& trajectory_generator,
        QObject* parent = nullptr);
    ~AxisMotionController() override = default;

    void enqueueMotionBatch(int axis_id, const QVariantList& points);
    void enqueueServiceBatch(int axis_id, const QVariantList& commands);
    void emergencyStop(int axis_id);
    void startMksHomingSequence(const QList<int>& axis_ids);
    void stopMksHomingSequence();
    void startSineMode(int axis_id, const motion_core::SineConfig& config);
    void stopSineMode(int axis_id);
    [[nodiscard]] bool isSineModeActive(int axis_id) const;

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void hostStateUpdated();

private:
    motion_core::HalRuntime& unified_runtime_;
    motion_core::MotionOrchestrator& orchestrator_;
    motion_core::TrajectoryGenerator& trajectory_generator_;
};

} // namespace mks