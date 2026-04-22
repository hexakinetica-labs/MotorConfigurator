#pragma once

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QSet>
#include <QString>
#include <QVariantList>
#include <QVariantMap>

#include "motion_core/axis_interface.h"
#include "motion_core/bus_manager_interface.h"
#include "motion_core/hal_runtime.h"
#include "hal_host_service/hal_host_service.h"
#include "motion_core/result.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/config/axis_config.h"

#include <cstdint>
#include <memory>
#include <vector>

class QTimer;

namespace mks {

class AxisManager final : public QObject {
    Q_OBJECT

public:
    explicit AxisManager(QObject* parent = nullptr);
    ~AxisManager() override;

    motion_core::AxisTransportKind getAxisTransport(int axis_id) const;

public slots:
    void openDevice(const QString& device_path, int baud_rate);
    void openEthercatDevice(const QString& interface_name);
    void closeMksDevice();
    void closeEthercatDevice();
    void closeDevice();
    void scanMotors(int max_id);
    void scanEthercatMotors();
    void watchAxis(int axis_id, bool enabled);
    void requestManualTakeover(bool enable);

    void enableMotor(int axis_id, bool enabled);
    void emergencyStop(int axis_id);
    void clearErrors(int axis_id);
    void moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg);
    void moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg);
    void configureMotionQueue(int axis_id, int capacity, bool drop_oldest);
    void enqueueMotionBatch(int axis_id, const QVariantList& points);
    void clearMotionQueue(int axis_id);
    void requestMotionQueueStats(int axis_id);
    void setAxisMode(int axis_id, int mode_code);
    void setZeroPosition(int axis_id);
    void goHome(int axis_id);

    void requestListParameters(int axis_id);
    void requestReadParameters(int axis_id);
    void applyParameterPatch(int axis_id, const QVariantList& patch);
    void setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data);

    void exportAxisConfig(int axis_id, const QString& path);
    void importAxisConfigPreview(int axis_id, const QString& path);
    void importAxisConfig(int axis_id, const QString& path);

    void loadHalConfig(const QString& config_path);
    void saveHalConfig(const QString& config_path);
    void startRuntime();
    void stopRuntime();

    // Direct enqueue for real-time streaming — bypasses HalHostService/JSON overhead.
    // Callable from any thread (IAxis::enqueue_motion_batch is internally synchronized).
    motion_core::Result<motion_core::MotionQueueStats> enqueueMotionBatchDirect(
        int axis_id,
        const std::vector<motion_core::QueuedSetpoint>& points);

    // Direct stats read from runtime axis queue — bypasses HalHostService/JSON path.
    motion_core::Result<motion_core::MotionQueueStats> queryMotionQueueStatsDirect(int axis_id) const;

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
    void axisConfigPreviewReady(int axis_id, const QVariantList& patch_entries);
    void manualTakeoverChanged(bool active);
    void hostStateUpdated(const QVariantMap& state);

private slots:
    void onFastTick();

private:
    bool isReady() const;
    void removeTransportConfig(motion_core::AxisTransportKind transport);
    void publishTopologySnapshot();
    void publishTransportOpenStates();
    void syncOpenStateFromCurrentConfig();
    void rebuildRuntimeFromCurrentConfig();
    QVariantList listParameters(int axis_id) const;
    QVariantList readParameters(int axis_id);
    void completeParameterRead(int axis_id, QVariantList params);
    void completeParameterReadFailure(int axis_id, QString message);
    void completeParameterWriteSuccess(int axis_id, QString message, bool refresh_after_write);
    void completeParameterWriteFailure(int axis_id, QString message);
    void applySafetyBaselineForAxis(int axis_id, const QString& reason, bool force_disable = true);
    void publishHostState();

    void reset_runtime_state();
    motion_core::Result<void> startRuntimeHeadless();
    void rebuildTransportRuntime(motion_core::AxisTransportKind transport);
    motion_core::Result<std::shared_ptr<motion_core::IAxis>> findAxis(std::uint16_t axis_id) const;
    motion_core::Result<std::vector<motion_core::AxisInfo>> listAxes() const;
    motion_core::Result<std::string> applyCommandLocally(hal_host_service::HalHostService* service,
                                                     int axis_id,
                                                     hal_ipc::ControlOp op,
                                                     const hal_ipc::AxisPointDto* point = nullptr,
                                                     const std::string& input_json = "",
                                                     int service_int = 0,
                                                     bool service_bool = false,
                                                     bool* manual_override_active = nullptr);
    hal_host_service::HalHostService* getHostServiceForAxis(int axis_id);

    motion_core::HalRuntime unified_runtime_;
    std::unique_ptr<hal_host_service::HalHostService> host_service_;
    
    QSet<int> mks_axes_;
    QSet<int> ethercat_axes_;
    QSet<int> runtime_started_axes_;
    QString opened_mks_device_path_;
    int opened_mks_baud_rate_{0};
    bool mks_device_opened_{false};
    QString opened_ethercat_interface_;
    bool ethercat_device_opened_{false};
    motion_core::HalRuntimeConfig current_hal_config_{};

    QTimer* fast_timer_{nullptr};
    QTimer* slow_timer_{nullptr};
    QSet<int> watched_axes_;
    QSet<int> parameter_reads_in_progress_;
    QSet<int> parameter_writes_in_progress_;
    int rr_index_{0};
    int ui_priority_axis_id_{-1};
    std::uint64_t local_control_seq_{0};
    std::uint32_t motion_epoch_{0};
    std::uint32_t service_epoch_{0};
    hal_ipc::OwnerRole motion_owner_{hal_ipc::OwnerRole::None};
    hal_ipc::OwnerRole service_owner_{hal_ipc::OwnerRole::None};
    bool manual_override_active_{false};
    bool service_mode_active_{false};
    bool estop_active_{false};
    double cached_cycle_hz_{0.0};
};

} // namespace mks
