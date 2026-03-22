#pragma once

#include <QObject>
#include <QPointer>
#include <QSet>
#include <QString>
#include <QVariantList>
#include <QVariantMap>

#include "motion_core/axis_interface.h"
#include "motion_core/bus_manager_interface.h"
#include "motion_core/hal_runtime.h"
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

public slots:
    void openDevice(const QString& device_path, int baud_rate);
    void openEthercatDevice(const QString& interface_name);
    void closeDevice();
    void scanMotors(int max_id);
    void scanEthercatMotors();
    void watchAxis(int axis_id, bool enabled);

    void enableMotor(int axis_id, bool enabled);
    void emergencyStop(int axis_id);
    void clearErrors(int axis_id);
    void moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg);
    void moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg);
    void setAxisMode(int axis_id, int mode_code);
    void setZeroPosition(int axis_id);
    void goHome(int axis_id);

    void requestListParameters(int axis_id);
    void requestReadParameters(int axis_id);
    void applyParameterPatch(int axis_id, const QVariantList& patch);
    void setPersistentParameter(int axis_id, int domain, int value, const QVariant& data);

    void exportAxisConfig(int axis_id, const QString& path);
    void importAxisConfigPreview(int axis_id, const QString& path);
    void importAxisConfig(int axis_id, const QString& path);

    void loadHalConfig(const QString& config_path);
    void saveHalConfig(const QString& config_path);
    void startRuntime();
    void stopRuntime();

signals:
    void logMessage(const QString& transport_tag, const QString& line);
    void connectionChanged(bool connected);
    void scanFinished(const QString& transport_tag, const QVariantList& axis_ids);
    void telemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void busStatisticsUpdated(const QVariantMap& bus_stats);
    void parameterListReady(int axis_id, const QVariantList& params);
    void parametersRead(int axis_id, const QVariantList& params);
    void axisConfigPreviewReady(int axis_id, const QVariantList& patch_entries);

private slots:
    void onFastTick();

private:
    enum class ActiveTransport {
        None = 0,
        Mks,
        Ethercat,
    };

    bool isReady() const;
    [[nodiscard]] QString current_transport_tag() const;
    QVariantList listParameters(int axis_id) const;
    QVariantList readParameters(int axis_id);
    void completeParameterRead(int axis_id, QVariantList params);
    void completeParameterReadFailure(int axis_id, QString message);
    void completeParameterWriteSuccess(int axis_id, QString message, bool refresh_after_write);
    void completeParameterWriteFailure(int axis_id, QString message);
    void applySafetyBaselineForAxis(int axis_id, const QString& reason, bool force_disable = true);

    void reset_runtime_state();
    motion_core::Result<void> startRuntimeHeadless();

    motion_core::Result<void> openRuntimeFromConfig(const motion_core::HalRuntimeConfig& config);
    motion_core::Result<void> closeRuntimeInternal();
    motion_core::Result<void> startRuntimeInternal();
    motion_core::Result<void> stopRuntimeInternal();
    motion_core::Result<std::shared_ptr<motion_core::IAxis>> findAxis(std::uint16_t axis_id) const;
    motion_core::Result<std::vector<motion_core::AxisInfo>> listAxes() const;

    motion_core::HalRuntime hal_runtime_;
    QSet<int> runtime_known_axes_;
    QSet<int> runtime_started_axes_;
    QString opened_device_path_;
    int opened_baud_rate_{0};
    bool device_opened_{false};
    ActiveTransport active_transport_{ActiveTransport::None};
    motion_core::HalRuntimeConfig current_hal_config_{};

    QTimer* fast_timer_{nullptr};
    QTimer* slow_timer_{nullptr};
    QSet<int> watched_axes_;
    QSet<int> parameter_reads_in_progress_;
    QSet<int> parameter_writes_in_progress_;
    int rr_index_{0};
};

} // namespace mks
