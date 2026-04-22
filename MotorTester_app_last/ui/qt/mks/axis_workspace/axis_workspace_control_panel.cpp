#include "mks/axis_workspace/axis_workspace_control_panel.h"

#include "mks/ScopeWidget.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QEvent>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

class QEvent;

namespace {

QLabel* make_value_label(QWidget* parent, const QString& text = QStringLiteral("---")) {
    auto* label = new QLabel(text, parent);
    label->setMinimumWidth(96);
    label->setWordWrap(true);
    return label;
}

class WheelEventFilter final : public QObject {
public:
    explicit WheelEventFilter(QObject* parent = nullptr)
        : QObject(parent) {}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
};

bool WheelEventFilter::eventFilter(QObject* watched, QEvent* event) {
    if (!event) {
        return QObject::eventFilter(watched, event);
    }

    if (event->type() == QEvent::Wheel) {
        auto* widget = qobject_cast<QWidget*>(watched);
        if (widget && !widget->hasFocus()) {
            return true;
        }
        if (qobject_cast<QAbstractSpinBox*>(watched)) {
            return true;
        }
    }

    return QObject::eventFilter(watched, event);
}

} // namespace

AxisWorkspaceControlPanel::AxisWorkspaceControlPanel(QWidget* parent)
    : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    root->addWidget(scroll);

    auto* content = new QWidget(scroll);
    auto* layout = new QVBoxLayout(content);
    auto* wheel_filter = new WheelEventFilter(this);

    auto* actions_group = new QGroupBox(QStringLiteral("Actions"), content);
    auto* actions_row = new QHBoxLayout(actions_group);
    handles_.btn_enable = new QPushButton(QStringLiteral("ENABLE"), actions_group);
    handles_.btn_disable = new QPushButton(QStringLiteral("DISABLE"), actions_group);
    handles_.btn_clear_err = new QPushButton(QStringLiteral("CLEAR ERRORS"), actions_group);
    handles_.btn_set_zero = new QPushButton(QStringLiteral("SET ZERO"), actions_group);
    handles_.btn_home = new QPushButton(QStringLiteral("GO HOME"), actions_group);
    handles_.btn_estop = new QPushButton(QStringLiteral("E-STOP"), actions_group);
    handles_.btn_enable->setStyleSheet(QStringLiteral("background-color: #2ea043; color: white; font-weight: bold;"));
    handles_.btn_disable->setStyleSheet(QStringLiteral("background-color: #da3633; color: white; font-weight: bold;"));
    handles_.btn_clear_err->setStyleSheet(QStringLiteral("background-color: #d29922; color: black;"));
    handles_.btn_estop->setStyleSheet(QStringLiteral("background-color: #da3633; color: white; font-weight: bold; padding: 8px;"));
    actions_row->addWidget(handles_.btn_enable);
    actions_row->addWidget(handles_.btn_disable);
    actions_row->addWidget(handles_.btn_clear_err);
    actions_row->addWidget(handles_.btn_set_zero);
    actions_row->addWidget(handles_.btn_home);
    actions_row->addSpacing(12);
    actions_row->addWidget(handles_.btn_estop);
    actions_row->addStretch();
    layout->addWidget(actions_group);

    auto* top_row = new QHBoxLayout();

    auto* motion_group = new QGroupBox(QStringLiteral("Basic Motion"), content);
    auto* motion_form = new QFormLayout(motion_group);
    handles_.mode_combo = new QComboBox(motion_group);
    handles_.mode_combo->addItem(QStringLiteral("Profile Position"), 0);
    handles_.mode_combo->setFixedWidth(170);

    handles_.speed_spin = new QSpinBox(motion_group);
    handles_.speed_spin->setRange(0, 3000);
    handles_.speed_spin->setValue(1800);
    handles_.speed_spin->setSuffix(QStringLiteral(" RPM"));
    handles_.speed_spin->setFixedWidth(170);
    handles_.speed_spin->installEventFilter(wheel_filter);

    handles_.accel_spin = new QSpinBox(motion_group);
    handles_.accel_spin->setRange(0, 100);
    handles_.accel_spin->setValue(100);
    handles_.accel_spin->setSuffix(QStringLiteral(" %"));
    handles_.accel_spin->setFixedWidth(170);
    handles_.accel_spin->installEventFilter(wheel_filter);

    handles_.target_pos_spin = new QDoubleSpinBox(motion_group);
    handles_.target_pos_spin->setRange(-1'000'000.0, 1'000'000.0);
    handles_.target_pos_spin->setDecimals(2);
    handles_.target_pos_spin->setValue(0.0);
    handles_.target_pos_spin->setAlignment(Qt::AlignRight);
    handles_.target_pos_spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    handles_.target_pos_spin->setKeyboardTracking(false);
    handles_.target_pos_spin->setFixedWidth(170);
    handles_.target_pos_spin->installEventFilter(wheel_filter);

    handles_.target_slider = new QSlider(Qt::Horizontal, motion_group);
    handles_.target_slider->setRange(-360000, 360000);
    handles_.target_slider->setSingleStep(1);
    handles_.target_slider->setPageStep(100);

    auto* move_type_row = new QWidget(motion_group);
    auto* move_type_layout = new QHBoxLayout(move_type_row);
    move_type_layout->setContentsMargins(0, 0, 0, 0);
    handles_.radio_move_abs = new QRadioButton(QStringLiteral("Abs"), move_type_row);
    handles_.radio_move_rel = new QRadioButton(QStringLiteral("Rel"), move_type_row);
    handles_.radio_move_abs->setChecked(true);
    move_type_layout->addWidget(handles_.radio_move_abs);
    move_type_layout->addWidget(handles_.radio_move_rel);
    move_type_layout->addStretch();

    handles_.btn_move = new QPushButton(QStringLiteral("Move"), motion_group);
    handles_.chk_sine_enable = new QCheckBox(QStringLiteral("Enable Sine Mode"), motion_group);

    handles_.spin_sine_amp = new QDoubleSpinBox(motion_group);
    handles_.spin_sine_amp->setRange(0.1, 360000.0);
    handles_.spin_sine_amp->setValue(20.0);
    handles_.spin_sine_amp->setSuffix(QStringLiteral(" deg"));
    handles_.spin_sine_amp->setFixedWidth(170);
    handles_.spin_sine_amp->installEventFilter(wheel_filter);

    handles_.spin_sine_freq = new QDoubleSpinBox(motion_group);
    handles_.spin_sine_freq->setRange(0.01, 50.0);
    handles_.spin_sine_freq->setDecimals(3);
    handles_.spin_sine_freq->setValue(0.5);
    handles_.spin_sine_freq->setSuffix(QStringLiteral(" Hz"));
    handles_.spin_sine_freq->setFixedWidth(170);
    handles_.spin_sine_freq->installEventFilter(wheel_filter);

    handles_.jog_step_spin = new QDoubleSpinBox(motion_group);
    handles_.jog_step_spin->setRange(0.01, 360.0);
    handles_.jog_step_spin->setValue(1.0);
    handles_.jog_step_spin->setSuffix(QStringLiteral("°"));
    handles_.jog_step_spin->installEventFilter(wheel_filter);

    auto* jog_row = new QWidget(motion_group);
    auto* jog_layout = new QHBoxLayout(jog_row);
    jog_layout->setContentsMargins(0, 0, 0, 0);
    handles_.btn_jog_neg = new QPushButton(QStringLiteral("Jog -"), jog_row);
    handles_.btn_jog_pos = new QPushButton(QStringLiteral("Jog +"), jog_row);
    jog_layout->addWidget(handles_.btn_jog_neg);
    jog_layout->addWidget(handles_.btn_jog_pos);
    jog_layout->addStretch();

    motion_form->addRow(QStringLiteral("Op Mode:"), handles_.mode_combo);
    motion_form->addRow(QStringLiteral("Speed:"), handles_.speed_spin);
    motion_form->addRow(QStringLiteral("Accel:"), handles_.accel_spin);
    motion_form->addRow(QStringLiteral("Target Position:"), handles_.target_pos_spin);
    motion_form->addRow(QStringLiteral("Live Target:"), handles_.target_slider);
    motion_form->addRow(QStringLiteral("Move Type:"), move_type_row);
    motion_form->addRow(QString(), handles_.btn_move);
    motion_form->addRow(QString(), handles_.chk_sine_enable);
    motion_form->addRow(QStringLiteral("Sine Amplitude:"), handles_.spin_sine_amp);
    motion_form->addRow(QStringLiteral("Sine Frequency:"), handles_.spin_sine_freq);
    motion_form->addRow(QStringLiteral("Jog step:"), handles_.jog_step_spin);
    motion_form->addRow(QString(), jog_row);
    top_row->addWidget(motion_group, 3);

    auto* telemetry_group = new QGroupBox(QStringLiteral("Telemetry (Live)"), content);
    auto* telemetry_form = new QFormLayout(telemetry_group);
    handles_.lbl_sys_state = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_state = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_protection = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_error_code = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_digital_inputs = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_axis = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_target = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_speed = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_torque = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_cmd_tx_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_telemetry_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_position_rx_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_speed_rx_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_status_rx_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_protection_rx_rate = make_value_label(telemetry_group, QStringLiteral("N/A"));
    handles_.lbl_motion_queue_stats = make_value_label(
        telemetry_group,
        QStringLiteral("queue: size=0 / 0, pushed=0, dropped=0, underruns=0, short_starts=0"));

    telemetry_form->addRow(QStringLiteral("System State:"), handles_.lbl_sys_state);
    telemetry_form->addRow(QStringLiteral("Motor Status:"), handles_.lbl_state);
    telemetry_form->addRow(QStringLiteral("Protection:"), handles_.lbl_protection);
    telemetry_form->addRow(QStringLiteral("Error Code:"), handles_.lbl_error_code);
    telemetry_form->addRow(QStringLiteral("Digital Inputs:"), handles_.lbl_digital_inputs);
    telemetry_form->addRow(QStringLiteral("Actual Pos:"), handles_.lbl_axis);
    telemetry_form->addRow(QStringLiteral("Target Pos:"), handles_.lbl_target);
    telemetry_form->addRow(QStringLiteral("Speed:"), handles_.lbl_speed);
    telemetry_form->addRow(QStringLiteral("Torque:"), handles_.lbl_torque);
    telemetry_form->addRow(QStringLiteral("Cmd TX Rate:"), handles_.lbl_cmd_tx_rate);
    telemetry_form->addRow(QStringLiteral("Telemetry Rate:"), handles_.lbl_telemetry_rate);
    telemetry_form->addRow(QStringLiteral("Position RX Rate:"), handles_.lbl_position_rx_rate);
    telemetry_form->addRow(QStringLiteral("Speed RX Rate:"), handles_.lbl_speed_rx_rate);
    telemetry_form->addRow(QStringLiteral("Status RX Rate:"), handles_.lbl_status_rx_rate);
    telemetry_form->addRow(QStringLiteral("Protection RX Rate:"), handles_.lbl_protection_rx_rate);
    telemetry_form->addRow(QStringLiteral("Queue Stats:"), handles_.lbl_motion_queue_stats);
    top_row->addWidget(telemetry_group, 2);

    layout->addLayout(top_row);

    auto* plot_group = new QGroupBox(QStringLiteral("Real-time Monitor"), content);
    auto* plot_layout = new QVBoxLayout(plot_group);
    auto* scope_controls = new QHBoxLayout();
    scope_controls->addWidget(new QLabel(QStringLiteral("Signal:"), plot_group));
    handles_.cmb_scope_signal = new QComboBox(plot_group);
    handles_.cmb_scope_signal->addItems({QStringLiteral("Position (deg)"),
                                         QStringLiteral("Velocity (deg/s)"),
                                         QStringLiteral("Torque (%)")});
    scope_controls->addWidget(handles_.cmb_scope_signal);

    scope_controls->addWidget(new QLabel(QStringLiteral("Time Window:"), plot_group));
    handles_.sld_scope_time = new QSlider(Qt::Horizontal, plot_group);
    handles_.sld_scope_time->setRange(2, 60);
    handles_.sld_scope_time->setValue(10);
    handles_.sld_scope_time->setFixedWidth(140);
    handles_.lbl_scope_time = new QLabel(QStringLiteral("10 s"), plot_group);
    scope_controls->addWidget(handles_.sld_scope_time);
    scope_controls->addWidget(handles_.lbl_scope_time);

    handles_.chk_plot_actual_pos = new QCheckBox(QStringLiteral("Actual"), plot_group);
    handles_.chk_plot_actual_pos->setChecked(true);
    handles_.chk_plot_target_pos = new QCheckBox(QStringLiteral("Target"), plot_group);
    handles_.chk_plot_target_pos->setChecked(true);
    handles_.chk_plot_actual_vel = new QCheckBox(QStringLiteral("Actual Vel"), plot_group);
    handles_.chk_plot_target_vel = new QCheckBox(QStringLiteral("Target Vel"), plot_group);
    handles_.chk_plot_pos_error = new QCheckBox(QStringLiteral("Pos Error"), plot_group);
    handles_.chk_auto_scale = new QCheckBox(QStringLiteral("Auto Scale"), plot_group);
    handles_.chk_auto_scale->setChecked(true);
    scope_controls->addWidget(handles_.chk_plot_actual_pos);
    scope_controls->addWidget(handles_.chk_plot_target_pos);
    scope_controls->addWidget(handles_.chk_plot_actual_vel);
    scope_controls->addWidget(handles_.chk_plot_target_vel);
    scope_controls->addWidget(handles_.chk_plot_pos_error);
    scope_controls->addWidget(handles_.chk_auto_scale);
    scope_controls->addStretch();
    plot_layout->addLayout(scope_controls);

    handles_.scope = new RDT::ScopeWidget(plot_group);
    handles_.scope->setMinimumHeight(240);
    handles_.scope->setXRange(10.0);
    handles_.scope->setAutoRange(true);
    handles_.scope->addChannel(QStringLiteral("actual"), QColor(80, 200, 255));
    handles_.scope->addChannel(QStringLiteral("target"), QColor(255, 80, 80));
    handles_.scope->addChannel(QStringLiteral("speed"), QColor(255, 180, 80));
    handles_.scope->addChannel(QStringLiteral("target_speed"), QColor(180, 255, 120));
    handles_.scope->addChannel(QStringLiteral("pos_error"), QColor(255, 120, 255));
    handles_.scope->addChannel(QStringLiteral("torque"), QColor(180, 140, 255));
    plot_layout->addWidget(handles_.scope, 1);

    layout->addStretch(1);
    layout->addWidget(plot_group, 0, Qt::AlignBottom);

    scroll->setWidget(content);
}

const AxisWorkspaceControlPanel::Handles& AxisWorkspaceControlPanel::handles() const noexcept {
    return handles_;
}
