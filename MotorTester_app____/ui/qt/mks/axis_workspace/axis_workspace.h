#pragma once

#include <QPointer>
#include <QWidget>
#include <QVariantMap>
#include <atomic>
#include <mutex>
#include <cstdint>

class QTabWidget;
class QTimer;

namespace mks {
class SequencerWidget;
class AxisManager;
}

class AxisWorkspaceControlPanel;
class AxisWorkspaceConfigPanel;

class AxisWorkspace : public QWidget {
    Q_OBJECT

public:
    ~AxisWorkspace() override;

protected:
    explicit AxisWorkspace(int axis_id,
                           mks::AxisManager* manager,
                           QWidget* parent = nullptr);

    void applyCurrentModeSelection();
    void setModeOptions(const QList<QPair<QString, int>>& options, int current_index = 0);
    void setSineControlsEnabled(bool enabled, const QString& tool_tip = {});
    void setMotionQueueStatsPlaceholder(const QString& text, const QString& style_sheet = {});
    void resetUiAfterSetZero();

    virtual void configureTransportUi() = 0;
    virtual void onTransportSineToggled(bool enabled);
    virtual void onTransportMotionQueueStatsUpdated(const QVariantMap& stats);
    virtual void onTransportTelemetryUpdated(const QVariantMap& telemetry,
                                             const QString& transport,
                                             double t_sec);
    virtual bool transportOwnsTargetUi() const;
    virtual bool transportProvidesTargetTrace() const;
    virtual void onBeforeDisableAxis();
    virtual void onBeforeHomeAxis();
    virtual void onBeforeSetZeroAxis();

    // --- Virtual hooks for sine trajectory pipeline (overridden by subclasses) ---
    virtual double samplePeriodSec() const = 0;
    virtual QString transportTag() const = 0;
    virtual bool supportsSineMode() const;
    virtual void disableSineMode();

    // --- Sine trajectory pipeline (shared implementation) ---
    void ensureMotionQueueConfigured();
    void fillTrajectoryQueue();
    void flushTrajectoryBatchToRuntime();
    void clearMotionBuffersForServiceCommand(bool reset_ui_to_zero);
    void stopSineModeForDisable();

private slots:
    void onTelemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void onHostStateUpdated(const QVariantMap& state);
    void refreshParameterList();
    void readParametersFromDrive();
    void applyParametersPatch();
    void saveSelectedParameterToDriveFlash();
    void exportAxisConfig();
    void importAxisConfig();
    void onAxisConfigPreviewReady(int axis_id, const QVariantList& patch_entries);
    void onParameterListReady(int axis_id, const QVariantList& params);
    void onParametersRead(int axis_id, const QVariantList& params);
    void onSineToggled(bool enabled);
    void onMotionQueueStatsUpdated(int axis_id, const QVariantMap& stats);

private:
    void setupUi();
    void scheduleWatchAxis(bool enabled);
    void setParameterReadInProgress(bool in_progress);
    void applyManualMotionControls(bool enabled);

    friend class mks::SequencerWidget;

    void setTargetPosition(double pos_deg);
    void triggerAbsoluteMove();
    bool isTargetReached(double tolerance_deg) const;

protected:
    int axis_id_{1};
    QPointer<mks::AxisManager> manager_;

    QTabWidget* tabs_{nullptr};
    AxisWorkspaceControlPanel* control_panel_{nullptr};
    AxisWorkspaceConfigPanel* config_panel_{nullptr};
    mks::SequencerWidget* sequencer_{nullptr};

    qint64 telemetry_t0_ms_{0};
    std::uint64_t telemetry_t0_ns_{0U};
    double scope_target_time_cursor_sec_{0.0};
    bool scope_target_time_cursor_initialized_{false};

    std::atomic<int> current_speed_{400};
    std::atomic<int> current_accel_{50};

    std::atomic<double> desired_target_deg_{0.0};
    std::atomic<double> commanded_target_deg_{0.0};
    double displayed_actual_deg_{0.0};
    bool target_seeded_from_telemetry_{false};
    bool have_prev_target_sample_{false};
    double prev_target_sample_deg_{0.0};
    double prev_target_sample_time_sec_{0.0};
    qint64 manual_target_hold_until_ms_{0};
    bool parameter_read_in_progress_{false};
    bool manual_motion_controls_enabled_{true};

    // --- Sine trajectory pipeline state (shared by MKS and EtherCAT) ---
    static constexpr int kUiTrajectoryLoopPeriodUs = 4'000;
    static constexpr std::size_t kUiTrajectorySourceBufferSize =
        static_cast<std::size_t>(1'000'000 / kUiTrajectoryLoopPeriodUs);
    static constexpr int kMotionQueueCapacity = 50;
    static constexpr std::size_t kMotionQueuePrefillSamples = 40U;
    static constexpr std::size_t kMotionQueueLowWatermarkSamples = 20U;
    static constexpr std::size_t kMotionQueueTopUpBatchSamples = 20U;

    std::atomic<std::size_t> driver_queue_size_{0};
    std::uint64_t last_driver_underruns_{0U};
    std::uint64_t last_driver_short_starts_{0U};
    bool motion_queue_configured_{false};
    bool motion_queue_prefilled_{false};
    std::size_t pending_refill_points_{0U};
    std::atomic<bool> sine_enabled_{false};
    std::atomic<double> sine_center_deg_{0.0};
    double sine_phase_accum_rad_{0.0};
};
