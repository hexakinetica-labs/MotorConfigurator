#pragma once

#include <QPointer>
#include <QWidget>
#include <QVariantMap>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>

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

private:
    void setupUi();
    void setupControlTab();
    void setupConfigTab();
    void scheduleWatchAxis(bool enabled);
    void trajectoryLoop();
    void setParameterReadInProgress(bool in_progress);
    void stopSineModeForDisable();

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

    qint64 telemetry_t0_ms_{0};

    
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
    
    std::thread trajectory_thread_;
    std::atomic<bool> trajectory_thread_active_{false};
    
    std::queue<double> trajectory_queue_;
    std::mutex trajectory_mutex_;
    QTimer* queue_fill_timer_{nullptr};
    
    std::atomic<int> current_speed_{400};
    std::atomic<int> current_accel_{50};
    
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
