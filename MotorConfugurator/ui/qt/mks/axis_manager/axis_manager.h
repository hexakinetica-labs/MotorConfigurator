#pragma once

#include <QObject>
#include <QVariantList>
#include <QVariantMap>
#include <memory>

#include "motion_core/types.h"
#include "motion_core/axis_interface.h"
#include "motion_core/axis_config.h"
#include "motion_core/trajectory_generator.h"

// Forward declarations of sub-controllers
namespace mks {
class AxisLifecycleController;
class AxisMotionController;
class AxisConfigController;
class AxisTelemetryController;
}

namespace hal_host {
class HalHostService;
}

namespace mks {

class AxisManager final : public QObject {
    Q_OBJECT

public:
    explicit AxisManager(
        hal_host::HalHostService& host_service,
        QObject* parent = nullptr);
    ~AxisManager() override;

public slots:
    // Lifecycle API
    void openDevice(const QString& device_path, int baud_rate);
    void openEthercatDevice(const QString& interface_name);
    void closeMksDevice();
    void closeEthercatDevice();
    void closeDevice();
    void scanMotors(int max_id);
    void scanEthercatMotors();
    void watchAxis(int axis_id, bool enabled);
    void setMotionSource(motion_core::ControlOwner source);
    motion_core::ControlOwner activeSource() const;
    void requestManualTakeover(bool enable);
    void startRuntime();
    void stopRuntime();
    std::shared_ptr<motion_core::IAxis> getAxis(int axis_id) const;

    // Motion API
    void enqueueMotionBatch(int axis_id, const QVariantList& points);
    void enqueueServiceBatch(int axis_id, const QVariantList& commands);
    void emergencyStop(int axis_id);
    void startMksHomingSequence(const QList<int>& axis_ids);
    void stopMksHomingSequence();
    void startSineMode(int axis_id, const motion_core::SineConfig& config);
    void stopSineMode(int axis_id);
    bool isSineModeActive(int axis_id) const;

    // Config API
    void loadHalConfig(const QString& config_path);
    void saveHalConfig(const QString& config_path);
    void exportAxisConfig(int axis_id, const QString& path);
    void importAxisConfigPreview(int axis_id, const QString& path);
    void importAxisConfig(int axis_id, const QString& path);

    // Telemetry & Parameters API
    void requestListParameters(int axis_id);
    void requestReadParameters(int axis_id);
    void applyParameterPatch(int axis_id, const QVariantList& patch);
    void setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data);

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void connectionChanged(bool connected);
    void transportOpenStateChanged(const QString& transport_tag, bool opened);
    void scanFinished(const QString& transport_tag, const QVariantList& axis_ids);
    void telemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void busStatisticsUpdated(const QVariantMap& bus_stats);
    void motionQueueStatsUpdated(int axis_id, const QVariantMap& stats);
    void parameterListReady(int axis_id, const QVariantList& params);
    void parametersRead(int axis_id, const QVariantList& params);
    void parameterPatchCompleted(int axis_id, bool success, const QString& message);
    void persistentParameterCompleted(int axis_id, bool success, const QString& message);
    void axisConfigPreviewReady(int axis_id, const QVariantList& patch_entries);
    void manualTakeoverChanged(bool active);
    void hostStateUpdated(const QVariantMap& state);
    void mksHomingSequenceProgress(const QVariantMap& progress);

private:
    void publishHostState();

    hal_host::HalHostService& host_service_;
    std::unique_ptr<AxisLifecycleController> lifecycle_;
    std::unique_ptr<AxisMotionController> motion_;
    std::unique_ptr<AxisConfigController> config_;
    std::unique_ptr<AxisTelemetryController> telemetry_ctrl_;
};

} // namespace mks

Q_DECLARE_METATYPE(motion_core::SineConfig)