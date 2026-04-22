#pragma once

#include <QWidget>

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
        QComboBox* mode_combo{nullptr};
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
        QLabel* lbl_cmd_tx_rate{nullptr};
        QLabel* lbl_telemetry_rate{nullptr};
        QLabel* lbl_position_rx_rate{nullptr};
        QLabel* lbl_speed_rx_rate{nullptr};
        QLabel* lbl_status_rx_rate{nullptr};
        QLabel* lbl_protection_rx_rate{nullptr};
        QLabel* lbl_state{nullptr};
        QLabel* lbl_sys_state{nullptr};
        QLabel* lbl_axis{nullptr};
        QLabel* lbl_target{nullptr};
        RDT::ScopeWidget* scope{nullptr};
        QLabel* lbl_speed{nullptr};
        QLabel* lbl_torque{nullptr};
        QLabel* lbl_protection{nullptr};
        QLabel* lbl_error_code{nullptr};
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
