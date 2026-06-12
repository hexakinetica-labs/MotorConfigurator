#pragma once

#include <QObject>
#include <QSet>
#include <QString>
#include <QVariantList>
#include <QVariantMap>
#include <memory>
#include <unordered_set>
#include <vector>

#include "motion_core/axis_interface.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/motion_orchestrator.h"
#include "hal_ipc/ipc_controller/ipc_controller.h"

class QTimer;

namespace hal_host {
class HalHostService;
}

namespace mks {

class AxisLifecycleController final : public QObject {
    Q_OBJECT

public:
    explicit AxisLifecycleController(
        hal_host::HalHostService& host_service,
        QObject* parent = nullptr);
    ~AxisLifecycleController() override;

    // Getters for state
    [[nodiscard]] bool mksDeviceOpened() const { return mks_device_opened_; }
    [[nodiscard]] QString openedMksDevicePath() const { return opened_mks_device_path_; }
    [[nodiscard]] int openedMksBaudRate() const { return opened_mks_baud_rate_; }
    [[nodiscard]] bool ethercatDeviceOpened() const { return ethercat_device_opened_; }
    [[nodiscard]] QString openedEthercatInterface() const { return opened_ethercat_interface_; }
    [[nodiscard]] const QSet<int>& mksAxes() const { return mks_axes_; }
    [[nodiscard]] const QSet<int>& ethercatAxes() const { return ethercat_axes_; }
    [[nodiscard]] const QSet<int>& runtimeStartedAxes() const { return runtime_started_axes_; }
    [[nodiscard]] const QSet<int>& lastScannedMksCanIds() const { return last_scanned_mks_can_ids_; }
    [[nodiscard]] bool hasMksScanSnapshot() const { return has_mks_scan_snapshot_; }
    [[nodiscard]] const QSet<int>& watchedAxes() const { return watched_axes_; }
    [[nodiscard]] int uiPriorityAxisId() const { return ui_priority_axis_id_; }
    [[nodiscard]] double cachedCycleHz() const { return cached_cycle_hz_; }

    [[nodiscard]] motion_core::ControlOwner activeSource() const;
    [[nodiscard]] std::shared_ptr<motion_core::IAxis> getAxis(int axis_id) const;

    // Setters / API
    void setHalConfig(const motion_core::HalRuntimeConfig& config);
    const motion_core::HalRuntimeConfig& halConfig() const { return current_hal_config_; }

    void openDevice(const QString& device_path, int baud_rate);
    void openEthercatDevice(const QString& interface_name);
    void closeMksDevice();
    void closeEthercatDevice();
    void closeDevice();
    void scanMotors(int max_id);
    void scanEthercatMotors();
    void watchAxis(int axis_id, bool enabled);
    void setMotionSource(motion_core::ControlOwner source);
    void requestManualTakeover(bool enable);
    void startRuntime();
    void stopRuntime();

    void rebuildRuntimeFromCurrentConfig();
    void rebuildTransportRuntime(motion_core::AxisTransportKind transport);
    void resetRuntimeState();

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void connectionChanged(bool connected);
    void transportOpenStateChanged(const QString& transport_tag, bool opened);
    void scanFinished(const QString& transport_tag, const QVariantList& axis_ids);
    void watchedAxesChanged();
    void manualTakeoverChanged(bool active);
    void hostStateUpdated();
    void busStatisticsUpdated(const QVariantMap& bus_stats);

private:
    void publishTopologySnapshot();
    void publishTransportOpenStates();
    void applySafetyBaselineForAxis(int axis_id, const QString& reason, bool force_disable = true);
    void updateTelemetryDistributorAxes();
    void removeTransportConfig(motion_core::AxisTransportKind transport);

    hal_host::HalHostService& host_service_;
    motion_core::HalRuntime& unified_runtime_;
    motion_core::MotionOrchestrator& orchestrator_;
    hal_ipc::HalIpcController& ipc_controller_;

    QSet<int> mks_axes_;
    QSet<int> ethercat_axes_;
    QSet<int> runtime_started_axes_;
    QSet<int> last_scanned_mks_can_ids_;
    bool has_mks_scan_snapshot_{false};
    QString opened_mks_device_path_;
    int opened_mks_baud_rate_{0};
    bool mks_device_opened_{false};
    QString opened_ethercat_interface_;
    bool ethercat_device_opened_{false};
    motion_core::HalRuntimeConfig current_hal_config_{};

    QTimer* slow_timer_{nullptr};
    QSet<int> watched_axes_;
    int ui_priority_axis_id_{-1};
    double cached_cycle_hz_{0.0};
};

} // namespace mks