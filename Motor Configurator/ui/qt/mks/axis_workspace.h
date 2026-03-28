#pragma once

#include <QPointer>
#include <QWidget>
#include <QVariantMap>
#include <atomic>
#include <deque>
#include <mutex>
#include <cstdint>

class QTabWidget;
class QLabel;
class QSpinBox;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QTextEdit;
class QTimer;
class QSlider;

namespace RDT {
class ScopeWidget;
}

namespace mks {
class SequencerWidget;
class AxisManager;
}

class QRadioButton;

class AxisWorkspace final : public QWidget {
    Q_OBJECT

public:
    explicit AxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent = nullptr);
    ~AxisWorkspace() override;

private slots:
    void onTelemetryUpdated(int axis_id, const QVariantMap& telemetry);
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
    void fillTrajectoryQueue();
    void onMotionQueueStatsUpdated(int axis_id, const QVariantMap& stats);

private:
    void setupUi();
    void setupControlTab();
    void setupConfigTab();
    void scheduleWatchAxis(bool enabled);
    void ensureMotionQueueConfigured();
    void flushTrajectoryBatchToRuntime();
    void setParameterReadInProgress(bool in_progress);
    void stopSineModeForDisable();
    void clearMotionBuffersForServiceCommand(bool reset_ui_to_zero);
    void resetUiAfterSetZero();

    friend class mks::SequencerWidget;

    void setTargetPosition(double pos_deg);
    void triggerAbsoluteMove();
    bool isTargetReached(double tolerance_deg) const;

    int axis_id_{1};
    QPointer<mks::AxisManager> manager_;

    QTabWidget* tabs_{nullptr};

    // Control tab
    QLabel* lbl_state_{nullptr};
    QLabel* lbl_sys_state_{nullptr}; // Added
    QLabel* lbl_axis_{nullptr};
    QLabel* lbl_target_{nullptr};
    RDT::ScopeWidget* scope_{nullptr};
    QLabel* lbl_speed_{nullptr};
    QLabel* lbl_torque_{nullptr};
    QLabel* lbl_protection_{nullptr};
    QLabel* lbl_error_code_{nullptr};
    
    // New Advanced Control Widgets
    QComboBox* mode_combo_{nullptr};
    QSpinBox* speed_spin_{nullptr};
    QSpinBox* accel_spin_{nullptr};
    QDoubleSpinBox* target_pos_spin_{nullptr}; // Replaced axis_spin_ with float
    QSlider* target_slider_{nullptr};
    QDoubleSpinBox* jog_step_spin_{nullptr};
    QPushButton* btn_jog_neg_{nullptr};
    QPushButton* btn_jog_pos_{nullptr};

    QRadioButton* radio_move_abs_{nullptr};
    QRadioButton* radio_move_rel_{nullptr};
    QCheckBox* chk_plot_actual_pos_{nullptr};
    QCheckBox* chk_plot_target_pos_{nullptr};
    QCheckBox* chk_plot_actual_vel_{nullptr};
    QCheckBox* chk_plot_target_vel_{nullptr};
    QCheckBox* chk_plot_pos_error_{nullptr};
    QComboBox* cmb_scope_signal_{nullptr};
    QSlider* sld_scope_time_{nullptr};
    QLabel* lbl_scope_time_{nullptr};
    QCheckBox* chk_auto_scale_{nullptr};
    QLabel* lbl_motion_queue_stats_{nullptr};
    QLabel* lbl_cmd_tx_rate_{nullptr};
    QLabel* lbl_telemetry_rate_{nullptr};
    QLabel* lbl_position_rx_rate_{nullptr};
    QLabel* lbl_speed_rx_rate_{nullptr};
    QLabel* lbl_status_rx_rate_{nullptr};
    QLabel* lbl_protection_rx_rate_{nullptr};

    qint64 telemetry_t0_ms_{0};
    std::uint64_t telemetry_t0_ns_{0U};
    double scope_target_time_cursor_sec_{0.0};
    bool scope_target_time_cursor_initialized_{false};

    
    // Config tab
    QTreeWidget* config_tree_{nullptr};
    QTextEdit* txt_description_{nullptr}; // Added
    QPushButton* btn_refresh_list_{nullptr};
    QPushButton* btn_read_params_{nullptr};
    QPushButton* btn_apply_params_{nullptr};
    QPushButton* btn_save_drive_flash_{nullptr};
    bool parameter_read_in_progress_{false};
    
    // Sequencer Tab
    mks::SequencerWidget* sequencer_{nullptr};
    
    // Sine Generator
    QCheckBox* chk_sine_enable_{nullptr};
    QDoubleSpinBox* spin_sine_amp_{nullptr};
    QDoubleSpinBox* spin_sine_freq_{nullptr};
    
    std::atomic<std::size_t> driver_queue_size_{0};
    std::uint64_t last_driver_underruns_{0U};
    std::uint64_t last_driver_short_starts_{0U};
    bool motion_queue_configured_{false};
    bool motion_queue_prefilled_{false};

    std::deque<double> trajectory_queue_;
    std::size_t streamed_trajectory_points_{0U};
    std::mutex trajectory_mutex_;
    QTimer* queue_fill_timer_{nullptr};
    
    std::atomic<int> current_speed_{1800};
    std::atomic<int> current_accel_{100};
    
    std::atomic<double> desired_target_deg_{0.0};
    std::atomic<double> commanded_target_deg_{0.0};
    std::atomic<bool> sine_enabled_{false};
    std::atomic<double> sine_center_deg_{0.0};
    double sine_phase_accum_rad_{0.0};
    double displayed_actual_deg_{0.0};
    bool target_seeded_from_telemetry_{false};
    bool have_prev_target_sample_{false};
    double prev_target_sample_deg_{0.0};
    double prev_target_sample_time_sec_{0.0};
    bool accel_marked_inactive_for_ethercat_{false};
    qint64 manual_target_hold_until_ms_{0};
    std::atomic<qint64> last_sine_ui_sync_ms_{0};
};
