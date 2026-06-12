#pragma once

#include <QPointer>
#include <QWidget>
#include <QVariantMap>
#include "motion_core/axis_interface.h"
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

#include "mks/axis_manager/parameter_ui_utils.h"

class AxisWorkspace : public QWidget {
    Q_OBJECT

public:
    ~AxisWorkspace() override;

    explicit AxisWorkspace(int axis_id,
                           mks::AxisManager* manager,
                           QWidget* parent = nullptr);

protected:

    void setSineControlsEnabled(bool enabled, const QString& tool_tip = {});
    void setMotionQueueStatsPlaceholder(const QString& text, const QString& style_sheet = {});
    void resetUiAfterSetZero();

    void configureTransportUi();
    void onTransportSineToggled(bool enabled);
    void onTransportMotionQueueStatsUpdated(const QVariantMap& stats);
    void onTransportTelemetryUpdated(const QVariantMap& telemetry,
                                     const QString& transport,
                                     double t_sec);
    bool transportOwnsTargetUi() const;
    bool transportProvidesTargetTrace() const;
    void onBeforeDisableAxis();
    void onBeforeHomeAxis();
    void onBeforeSetZeroAxis();

    // --- Transport-specific helpers ---
    motion_core::AxisTransportKind getTransportKind() const;
    double samplePeriodSec() const;
    QString transportTag() const;
    bool supportsSineMode() const;
    void disableSineMode();
    void updateSineControlsAvailability();

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
    void onParameterPatchCompleted(int axis_id, bool success, const QString& message);
    void onPersistentParameterCompleted(int axis_id, bool success, const QString& message);
    void onSineToggled(bool enabled);
    void onMotionQueueStatsUpdated(int axis_id, const QVariantMap& stats);

private:
    void setupUi();
    void scheduleWatchAxis(bool enabled);
    void setParameterReadInProgress(bool in_progress);
    void setParameterWriteInProgress(bool in_progress, const QString& operation_text);
    void applyManualMotionControls(bool enabled);

    friend class mks::SequencerWidget;

    void setTargetPosition(double pos_deg);
    void triggerAbsoluteMove();
    bool isTargetReached(double tolerance_deg) const;

    void resetScopeDeadband();
    void resetDropTracking();
    void sendServiceCommand(ServiceCommandKind kind, int profile_speed_rpm = 0);
    QVariantMap buildMotionPoint(MotionPointKind kind, double target_deg, bool is_relative = false) const;

protected:
    int axis_id_{1};
    QString transport_tag_{QStringLiteral("unknown")};
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
    bool has_last_scope_actual_position_{false};
    double last_scope_actual_position_deg_{0.0};
    qint64 manual_target_hold_until_ms_{0};
    bool parameter_read_in_progress_{false};
    bool parameter_write_in_progress_{false};
    bool manual_motion_controls_enabled_{true};

    // --- Sine trajectory pipeline state (shared by MKS and EtherCAT) ---
    static constexpr int kMotionQueueCapacity = 50;
    static constexpr std::size_t kMotionQueuePrefillSamples = 40U;
    static constexpr std::size_t kMotionQueueLowWatermarkSamples = 20U;
    static constexpr std::size_t kUiTrajectorySourceBufferSize = 250U;

    std::atomic<std::size_t> driver_queue_size_{0};
    std::uint64_t last_driver_underruns_{0U};
    std::uint64_t last_driver_short_starts_{0U};
    bool motion_queue_configured_{false};
    bool motion_queue_prefilled_{false};
    std::size_t pending_refill_points_{0U};
    std::atomic<bool> sine_enabled_{false};
    std::atomic<double> sine_center_deg_{0.0};
    double sine_phase_accum_rad_{0.0};
    bool sine_stopped_due_to_drops_{false};
    bool drop_baseline_initialized_{false};
    std::uint64_t last_driver_dropped_{0U};
};



class QTreeWidget;
class QTextEdit;
class QPushButton;

class AxisWorkspaceConfigPanel final : public QWidget {
public:
    struct Handles {
        QTreeWidget* config_tree{nullptr};
        QTextEdit* txt_description{nullptr};
        QPushButton* btn_refresh_list{nullptr};
        QPushButton* btn_read_params{nullptr};
        QPushButton* btn_apply_params{nullptr};
        QPushButton* btn_save_drive_flash{nullptr};
        QPushButton* btn_export_full{nullptr};
        QPushButton* btn_import_full{nullptr};
    };

    explicit AxisWorkspaceConfigPanel(QWidget* parent = nullptr);

    const Handles& handles() const noexcept;

private:
    Handles handles_{};
};



class QLabel;
class QSpinBox;
class QPushButton;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QSlider;
class QRadioButton;

namespace RDT {
class ScopeWidget;
}

class AxisWorkspaceControlPanel final : public QWidget {
public:
    struct Handles {

        QSpinBox* speed_spin{nullptr};
        QSpinBox* accel_spin{nullptr};
        QDoubleSpinBox* target_pos_spin{nullptr};
        QSlider* target_slider{nullptr};
        QDoubleSpinBox* jog_step_spin{nullptr};
        QPushButton* btn_jog_neg{nullptr};
        QPushButton* btn_jog_pos{nullptr};
        QRadioButton* radio_move_abs{nullptr};
        QRadioButton* radio_move_rel{nullptr};
        QCheckBox* chk_plot_actual_pos{nullptr};
        QCheckBox* chk_plot_target_pos{nullptr};
        QCheckBox* chk_plot_actual_vel{nullptr};
        QCheckBox* chk_plot_target_vel{nullptr};
        QCheckBox* chk_plot_pos_error{nullptr};
        QComboBox* cmb_scope_signal{nullptr};
        QSlider* sld_scope_time{nullptr};
        QLabel* lbl_scope_time{nullptr};
        QCheckBox* chk_auto_scale{nullptr};
        QLabel* lbl_motion_queue_stats{nullptr};
        QLabel* lbl_cmd_tx_rate_title{nullptr};
        QLabel* lbl_telemetry_rate_title{nullptr};
        QLabel* lbl_position_rx_rate_title{nullptr};
        QLabel* lbl_cmd_tx_rate{nullptr};
        QLabel* lbl_telemetry_rate{nullptr};
        QLabel* lbl_position_rx_rate{nullptr};
        QLabel* lbl_speed_rx_rate{nullptr};
        QLabel* lbl_status_rx_rate{nullptr};
        QLabel* lbl_protection_rx_rate{nullptr};
        QLabel* lbl_state{nullptr};
        QLabel* lbl_homing_sequence{nullptr};
        QLabel* lbl_sys_state{nullptr};
        QLabel* lbl_axis{nullptr};
        QLabel* lbl_target{nullptr};
        RDT::ScopeWidget* scope{nullptr};
        QLabel* lbl_speed{nullptr};
        QLabel* lbl_torque{nullptr};
        QLabel* lbl_protection{nullptr};
        QLabel* lbl_error_code{nullptr};
        QLabel* lbl_digital_inputs{nullptr};
        QCheckBox* chk_sine_enable{nullptr};
        QDoubleSpinBox* spin_sine_amp{nullptr};
        QDoubleSpinBox* spin_sine_freq{nullptr};
        QPushButton* btn_enable{nullptr};
        QPushButton* btn_disable{nullptr};
        QPushButton* btn_clear_err{nullptr};
        QPushButton* btn_set_zero{nullptr};
        QPushButton* btn_home{nullptr};
        QPushButton* btn_estop{nullptr};
        QPushButton* btn_move{nullptr};
    };

    explicit AxisWorkspaceControlPanel(QWidget* parent = nullptr);

    const Handles& handles() const noexcept;

private:
    Handles handles_{};
};



