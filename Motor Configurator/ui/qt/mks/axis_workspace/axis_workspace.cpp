#include "mks/axis_workspace.h"

#include "ethercat/p100e_ethercat_dictionary.h"
#include "mks/ScopeWidget.h"
#include "mks/axis_manager.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QLabel>
#include <QDateTime>
#include <QMessageBox>
#include <QEvent>
#include <QMetaObject>
#include <QMetaType>
#include <QPushButton>
#include <QStringList>
#include <QThread>
#include <QSpinBox>
#include <QStyledItemDelegate>
#include <QTabWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTextEdit>
#include <QFileDialog>
#include <QScrollArea>
#include <QFile>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QSlider>
#include <QTimer>
#include <QHash>
#include <QRadioButton>
#include <QAbstractSpinBox>
#include <cmath>
#include "mks/dictionary/mks_dictionary.h"
#include "mks/sequencer_widget.h"

namespace {

constexpr int kNameColumn = 0;
constexpr int kGroupColumn = 1;
constexpr int kUnitColumn = 2;
constexpr int kReadOnlyColumn = 3;
constexpr int kCurrentValueColumn = 4;
constexpr int kNewValueColumn = 5;

class NewValueColumnDelegate final : public QStyledItemDelegate {
public:
    explicit NewValueColumnDelegate(QObject* parent = nullptr)
        : QStyledItemDelegate(parent) {}

    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option,
                          const QModelIndex& index) const override {
        if (index.column() != kNewValueColumn) {
            return nullptr;
        }

        const QString read_only = index.sibling(index.row(), kReadOnlyColumn).data(Qt::DisplayRole).toString();
        if (read_only.compare("Yes", Qt::CaseInsensitive) == 0 ||
            read_only.compare("N/A", Qt::CaseInsensitive) == 0) {
            return nullptr;
        }

        return QStyledItemDelegate::createEditor(parent, option, index);
    }
};

class WheelEventFilter : public QObject {
public:
    explicit WheelEventFilter(QObject* parent = nullptr) : QObject(parent) {}

protected:
    bool eventFilter(QObject* obj, QEvent* event) override {
        if (event->type() == QEvent::Wheel) {
            auto* widget = qobject_cast<QWidget*>(obj);
            if (widget && !widget->hasFocus()) {
                return true; // Ignore wheel event if not focused
            }
            // If focused, some users prefer it to still not scroll, but let's just completely disable wheel changes on spinboxes.
            if (qobject_cast<QAbstractSpinBox*>(obj)) {
                return true; // Ignore completely
            }
        }
        return QObject::eventFilter(obj, event);
    }
};

QString protocolDetailsForParameter(const int domain, const int value) {
    using motion_core::CommonParameter;
    using motion_core::ParameterDomain;
    using mks::MksParameter;

    if (domain == static_cast<int>(motion_core::ParameterDomain::Common)) {
        if (static_cast<motion_core::CommonParameter>(value) == motion_core::CommonParameter::HardwareGearRatio) {
            return QStringLiteral(
                "<b>Gear Ratio</b><br>"
                "<b>Purpose:</b> Mechanical transmission ratio used by runtime kinematics.<br>"
                "<b>Meaning:</b> Number of motor revolutions per one output-shaft revolution.<br>"
                "<b>Example:</b> 1.0 = direct drive, 50.0 = 50:1 reducer.<br>"
                "<b>Operational note:</b> Keep this value consistent with real mechanics to avoid scale mismatch.");
        }
        if (static_cast<motion_core::CommonParameter>(value) ==
            motion_core::CommonParameter::HardwareEncoderResolutionBits) {
            return QStringLiteral(
                "<b>Encoder Resolution (bits)</b><br>"
                "<b>Purpose:</b> Software-side encoder CPR used for ticks↔degrees conversion.<br>"
                "<b>Formula:</b> ticks_per_degree = (2^bits * gear_ratio) / 360.<br>"
                "<b>Operational note:</b> Not auto-read from drive firmware; changed only by explicit UI/API patch.");
        }
        return {};
    }

    if (domain != static_cast<int>(motion_core::ParameterDomain::Mks)) {
        return {};
    }

    switch (static_cast<MksParameter>(value)) {
        case MksParameter::WorkMode:
            return QStringLiteral(
                "<b>Command:</b> 0x82 SetWorkMode<br>"
                "<b>Purpose:</b> Selects drive control source and control profile.<br>"
                "<b>Values:</b><br>"
                "0 = Pulse Open (CR_OPEN)<br>"
                "1 = Pulse Close (CR_CLOSE)<br>"
                "2 = Pulse vFOC (CR_vFOC)<br>"
                "3 = Serial Open (SR_OPEN)<br>"
                "4 = Serial Close (SR_CLOSE)<br>"
                "5 = Serial vFOC (SR_vFOC)<br>"
                "<b>Runtime requirement:</b> CAN motion commands require serial modes 3/4/5.");
        case MksParameter::WorkingCurrentMilliAmp:
            return QStringLiteral(
                "<b>Command:</b> 0x83 SetWorkingCurrent<br>"
                "<b>Range:</b> 0..5200 mA.<br>"
                "<b>Effect:</b> Higher current increases torque and thermal load.");
        case MksParameter::Subdivision:
            return QStringLiteral(
                "<b>Command:</b> 0x84 SetSubdivision<br>"
                "<b>Purpose:</b> Microstep subdivision setting.<br>"
                "<b>Range:</b> 0..255 (byte range; some firmware maps 0x00 to 256 microsteps).<br>"
                "<b>Effect:</b> Higher subdivision improves position smoothness and granularity.");
        case MksParameter::EnPinActiveLevel:
            return QStringLiteral(
                "<b>Command:</b> 0x85 SetEnPinActiveLevel<br>"
                "<b>Values:</b> 0=Active Low, 1=Active High, 2=Always Enabled.");
        case MksParameter::MotorDirection:
            return QStringLiteral(
                "<b>Command:</b> 0x86 SetMotorDirection<br>"
                "<b>Values:</b> 0=CW, 1=CCW.<br>"
                "<b>Operational note:</b> Vendor documentation states this mainly affects pulse mode.");
        case MksParameter::AutoScreenOff:
            return QStringLiteral(
                "<b>Command:</b> 0x87 SetAutoTurnOffScreen<br>"
                "<b>Values:</b> 0=Off, 1=On.");
        case MksParameter::LockedRotorProtection:
            return QStringLiteral(
                "<b>Command:</b> 0x88 SetLockedRotorProtection<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.<br>"
                "<b>Effect:</b> Triggers protection when rotor movement is blocked.");
        case MksParameter::SubdivisionInterpolation:
            return QStringLiteral(
                "<b>Command:</b> 0x89 SetSubdivisionInterpolation<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.");
        case MksParameter::CanBitrateIndex:
            return QStringLiteral(
                "<b>Command:</b> 0x8A SetCanBitrate (read-only here)<br>"
                "<b>Values:</b> 0=125k, 1=250k, 2=500k, 3=1M.");
        case MksParameter::CanId:
            return QStringLiteral(
                "<b>Command:</b> 0x8B SetCanId<br>"
                "<b>Range:</b> 1..2047 (0x7FF).<br>"
                "<b>Addressing note:</b> 0 (broadcast) must not be used as a device ID.<br>"
                "<b>Operational note:</b> After write, the axis is reachable only at the new CAN ID.");
        case MksParameter::SlaveRespondMode:
            return QStringLiteral(
                "<b>Command:</b> 0x8C SetSlaveRespondActive (Respond flag)<br>"
                "<b>Policy:</b> LOCKED (read-only).<br>"
                "<b>Required value:</b> 1 for current runtime contract.<br>"
                "<b>Operational note:</b> Editing disabled by system safety policy.");
        case MksParameter::SlaveActiveReport:
            return QStringLiteral(
                "<b>Command:</b> 0x8C SetSlaveRespondActive (Active flag)<br>"
                "<b>Policy:</b> LOCKED (read-only).<br>"
                "<b>Values:</b><br>"
                "0 = Passive response (acceptance acknowledgment only)<br>"
                "1 = Active response (acknowledgment + motion completion report).<br>"
                "<b>Operational note:</b> Editing disabled by system safety policy.");
        case MksParameter::GroupId:
            return QStringLiteral(
                "<b>Command:</b> 0x8D SetGroupId<br>"
                "<b>Range:</b> 1..2047 (0x7FF).<br>"
                "<b>Use:</b> Group addressing on CAN bus.");
        case MksParameter::KeyLock:
            return QStringLiteral(
                "<b>Command:</b> 0x8F SetKeyLock<br>"
                "<b>Values:</b> 0=Unlock keys, 1=Lock keys.");
        case MksParameter::HoldingCurrentIndex:
            return QStringLiteral(
                "<b>Command:</b> 0x9B SetHoldingCurrent<br>"
                "<b>Range:</b> 1..8 (approximately 10%..90% of working current).<br>"
                "<b>Purpose:</b> Holding current while idle to balance torque and heat.");
        case MksParameter::LimitPortRemap:
            return QStringLiteral(
                "<b>Command:</b> 0x9E SetLimitPortRemap<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.");
        case MksParameter::AxisPositionRaw:
            return QStringLiteral(
                "<b>Source:</b> 0x31 ReadEncoderAddition.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Description:</b> Raw accumulated encoder position.");
        case MksParameter::MotorSpeedRpm:
            return QStringLiteral(
                "<b>Source:</b> 0x32 ReadMotorSpeed.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Description:</b> Instant motor shaft speed in RPM.");
        case MksParameter::ProtectionState:
            return QStringLiteral(
                "<b>Source:</b> 0x3E ReadProtectionState.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Description:</b> Internal protection state (0 = no active protection fault).");
        case MksParameter::MotorStatus:
            return QStringLiteral(
                "<b>Source:</b> 0xF1 QueryMotorStatus.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Values:</b> 0=Fault, 1=Stopped, 2=Accelerating, 3=Decelerating, 4=At Max Speed, 5=Homing, 6=Calibrating.");
        case MksParameter::EnableMotor:
            return QStringLiteral(
                "<b>Command:</b> 0xF3 EnableMotor<br>"
                "<b>Purpose:</b> Drive power stage enable/disable control.<br>"
                "<b>Values:</b> false/0 = Disable (release), true/1 = Enable (hold).<br>"
                "<b>Verification:</b> Runtime state confirmed by 0xF1 polling.");
        default:
            break;
    }

    return {};
}

QList<QTreeWidgetItem*> parameterItems(QTreeWidget* tree) {
    QList<QTreeWidgetItem*> out;
    if (!tree) {
        return out;
    }

    std::function<void(QTreeWidgetItem*)> collect = [&](QTreeWidgetItem* item) {
        if (!item) {
            return;
        }
        if (item->data(kNameColumn, Qt::UserRole).isValid()) {
            out.push_back(item);
        }
        for (int i = 0; i < item->childCount(); ++i) {
            collect(item->child(i));
        }
    };

    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        collect(tree->topLevelItem(i));
    }

    return out;
}

QString decodeSystemState(const int state) {
    switch (state) {
        case 0: return QStringLiteral("Unknown");
        case 1: return QStringLiteral("Disabled");
        case 2: return QStringLiteral("Ready");
        case 3: return QStringLiteral("Moving/Enabled");
        case 4: return QStringLiteral("Fault / E-Stop");
        default: break;
    }
    return QStringLiteral("N/A");
}

QString decodeMksMotionStatus(const int status_code) {
    switch (status_code) {
        case 0: return QStringLiteral("Fault");
        case 1: return QStringLiteral("Stopped");
        case 2: return QStringLiteral("Accelerating");
        case 3: return QStringLiteral("Decelerating");
        case 4: return QStringLiteral("At max speed");
        case 5: return QStringLiteral("Homing");
        case 6: return QStringLiteral("Calibrating");
        default: break;
    }
    return QStringLiteral("Unknown");
}

QString decodeMksProtectionText(const int code) {
    switch (code) {
        case 0: return QStringLiteral("No active protection fault");
        case 1: return QStringLiteral("Overcurrent protection");
        case 2: return QStringLiteral("Overvoltage protection");
        case 3: return QStringLiteral("Undervoltage protection");
        case 4: return QStringLiteral("Overtemperature protection");
        case 10: return QStringLiteral("Rotor lock / obstruction protection");
        default: break;
    }
    return QStringLiteral("Drive protection fault");
}

QString decodeEthercatFaultText(const int code) {
    if (code == 0) {
        return QStringLiteral("No active drive fault");
    }
    switch (code) {
        case 0x2310: return QStringLiteral("Overcurrent fault");
        case 0x3210: return QStringLiteral("Overvoltage fault");
        case 0x3220: return QStringLiteral("Undervoltage fault");
        case 0x4310: return QStringLiteral("Motor overtemperature fault");
        case 0x6320: return QStringLiteral("Encoder / feedback fault");
        case 0x7121: return QStringLiteral("Position following error fault");
        case 0x8180: return QStringLiteral("Communication / fieldbus fault");
        default: break;
    }
    return QStringLiteral("Drive fault");
}

QString decodeMotorStatusText(const QString& transport,
                             const int state_code,
                             const int motion_status,
                             const int status_word) {
    if (transport.compare(QStringLiteral("mks"), Qt::CaseInsensitive) == 0) {
        Q_UNUSED(state_code);
        Q_UNUSED(status_word);
        return decodeMksMotionStatus(motion_status);
    }

    if (transport.compare(QStringLiteral("ethercat"), Qt::CaseInsensitive) == 0) {
        Q_UNUSED(motion_status);
        Q_UNUSED(status_word);
        return decodeSystemState(state_code);
    }

    Q_UNUSED(state_code);
    Q_UNUSED(motion_status);
    Q_UNUSED(status_word);
    return QStringLiteral("Unknown status");
}

QString decodeProtectionText(const QString& transport, const int protection_code) {
    if (transport.compare(QStringLiteral("mks"), Qt::CaseInsensitive) == 0) {
        return decodeMksProtectionText(protection_code);
    }

    if (transport.compare(QStringLiteral("ethercat"), Qt::CaseInsensitive) == 0) {
        return decodeEthercatFaultText(protection_code);
    }

    return QStringLiteral("Unknown protection state");
}

QString decodeErrorCodeText(const QString& transport, const int error_code) {
    if (transport.compare(QStringLiteral("mks"), Qt::CaseInsensitive) == 0) {
        return decodeMksProtectionText(error_code);
    }
    if (transport.compare(QStringLiteral("ethercat"), Qt::CaseInsensitive) == 0) {
        return decodeEthercatFaultText(error_code);
    }
    return QStringLiteral("No decoder for current transport");
}

} // namespace

AxisWorkspace::AxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent)
    : QWidget(parent), axis_id_(axis_id), manager_(manager) {
    setupUi();

    connect(manager_, SIGNAL(telemetryUpdated(int,QVariantMap)),
            this, SLOT(onTelemetryUpdated(int,QVariantMap)));
            
    // Parameter Async Signals
    connect(manager_, SIGNAL(parameterListReady(int,QVariantList)),
            this, SLOT(onParameterListReady(int,QVariantList)));
    connect(manager_, SIGNAL(parametersRead(int,QVariantList)),
            this, SLOT(onParametersRead(int,QVariantList)));
    connect(manager_, SIGNAL(axisConfigPreviewReady(int,QVariantList)),
            this, SLOT(onAxisConfigPreviewReady(int,QVariantList)));

    queue_fill_timer_ = new QTimer(this);
    connect(queue_fill_timer_, &QTimer::timeout, this, &AxisWorkspace::fillTrajectoryQueue);
    queue_fill_timer_->start(100); // Check every 100ms

    trajectory_thread_active_ = true;
    trajectory_thread_ = std::thread(&AxisWorkspace::trajectoryLoop, this);

    scheduleWatchAxis(true);
}

AxisWorkspace::~AxisWorkspace() {
    trajectory_thread_active_ = false;
    if (trajectory_thread_.joinable()) {
        trajectory_thread_.join();
    }
    scheduleWatchAxis(false);
}

void AxisWorkspace::scheduleWatchAxis(const bool enabled) {
    auto* manager = manager_.data();
    if (!manager) {
        return;
    }

    const auto ok = QMetaObject::invokeMethod(
        manager,
        [manager, axis_id = axis_id_, enabled]() {
            manager->watchAxis(axis_id, enabled);
        },
        Qt::QueuedConnection);

    if (!ok) {
        qWarning("AxisWorkspace: failed to schedule watchAxis(%d, %d)", axis_id_, enabled ? 1 : 0);
    }
}

void AxisWorkspace::setupUi() {
    auto* root = new QVBoxLayout(this);
    tabs_ = new QTabWidget(this);
    root->addWidget(tabs_);

    setupControlTab();
    setupConfigTab();
    sequencer_ = new mks::SequencerWidget(this);
    tabs_->addTab(sequencer_, "Sequencer (Trapezoidal)");
}

void AxisWorkspace::setupControlTab() {
    auto* tab_root = new QWidget(this);
    auto* root_layout = new QVBoxLayout(tab_root);
    root_layout->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea(tab_root);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* scroll_content = new QWidget(scroll);
    auto* layout = new QVBoxLayout(scroll_content);

    auto* filter = new WheelEventFilter(this);

    // Top actions row (requested: controls on top)
    auto* actions_box = new QGroupBox("Actions", scroll_content);
    auto* actions_layout = new QHBoxLayout(actions_box);

    auto* btn_enable = new QPushButton("ENABLE", actions_box);
    auto* btn_disable = new QPushButton("DISABLE", actions_box);
    auto* btn_clear_err = new QPushButton("CLEAR ERRORS", actions_box);
    auto* btn_set_zero = new QPushButton("SET ZERO", actions_box);
    auto* btn_home = new QPushButton("GO HOME", actions_box);
    auto* btn_estop = new QPushButton("E-STOP", actions_box);

    btn_enable->setStyleSheet("background-color: #2ea043; color: white; font-weight: bold;");
    btn_disable->setStyleSheet("background-color: #da3633; color: white; font-weight: bold;");
    btn_clear_err->setStyleSheet("background-color: #d29922; color: black;");
    btn_estop->setStyleSheet("background-color: #da3633; color: white; font-weight: bold; padding: 8px;");

    actions_layout->addWidget(btn_enable);
    actions_layout->addWidget(btn_disable);
    actions_layout->addWidget(btn_clear_err);
    actions_layout->addWidget(btn_set_zero);
    actions_layout->addWidget(btn_home);
    actions_layout->addSpacing(12);
    actions_layout->addWidget(btn_estop);
    actions_layout->addStretch();

    layout->addWidget(actions_box);

    // Middle split: Basic Motion (left) + Telemetry one-column (right)
    auto* top_row = new QHBoxLayout();

    auto* motion_grp = new QGroupBox("Basic Motion", scroll_content);
    auto* motion_form = new QFormLayout(motion_grp);

    mode_combo_ = new QComboBox(motion_grp);
    mode_combo_->addItem("Position (Generic/PP)", 1);
    mode_combo_->addItem("CSP (EtherCAT mode 8)", 8);
    mode_combo_->addItem("CSV (EtherCAT mode 9)", 9);
    mode_combo_->addItem("Homing (EtherCAT mode 6)", 6);
    mode_combo_->setFixedWidth(160);
    motion_form->addRow("Op Mode:", mode_combo_);

    speed_spin_ = new QSpinBox(motion_grp);
    speed_spin_->setRange(0, 3000);
    speed_spin_->setValue(400);
    speed_spin_->setSuffix(" RPM");
    speed_spin_->setFixedWidth(160);
    speed_spin_->installEventFilter(filter);
    motion_form->addRow("Speed:", speed_spin_);

    accel_spin_ = new QSpinBox(motion_grp);
    accel_spin_->setRange(0, 100);
    accel_spin_->setValue(50);
    accel_spin_->setSuffix(" %");
    accel_spin_->setFixedWidth(160);
    accel_spin_->installEventFilter(filter);
    motion_form->addRow("Accel:", accel_spin_);

    target_pos_spin_ = new QDoubleSpinBox(motion_grp);
    target_pos_spin_->setRange(-1e6, 1e6);
    target_pos_spin_->setDecimals(2);
    target_pos_spin_->setValue(0.0);
    target_pos_spin_->setFixedWidth(160);
    target_pos_spin_->setAlignment(Qt::AlignRight);
    target_pos_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    target_pos_spin_->setKeyboardTracking(false);
    target_pos_spin_->installEventFilter(filter);
    motion_form->addRow("Target Position:", target_pos_spin_);

    target_slider_ = new QSlider(Qt::Horizontal, motion_grp);
    target_slider_->setRange(-360000, 360000); // ±3600° (10 full turns) at x100 scale
    target_slider_->setSingleStep(1);          // 0.01° per step
    target_slider_->setPageStep(100);          // 1° per page
    motion_form->addRow("Live Target:", target_slider_);

    auto* type_row = new QWidget(motion_grp);
    auto* type_layout = new QHBoxLayout(type_row);
    type_layout->setContentsMargins(0, 0, 0, 0);
    radio_move_abs_ = new QRadioButton("Abs", type_row);
    radio_move_rel_ = new QRadioButton("Rel", type_row);
    radio_move_abs_->setChecked(true);
    type_layout->addWidget(radio_move_abs_);
    type_layout->addWidget(radio_move_rel_);
    type_layout->addStretch();
    motion_form->addRow("Move Type:", type_row);

    auto* btn_move = new QPushButton("Move", motion_grp);
    motion_form->addRow(QString(), btn_move);

    chk_sine_enable_ = new QCheckBox("Enable Sine Mode", motion_grp);
    motion_form->addRow(QString(), chk_sine_enable_);

    spin_sine_amp_ = new QDoubleSpinBox(motion_grp);
    spin_sine_amp_->setRange(0.1, 360000.0);
    spin_sine_amp_->setValue(20.0);
    spin_sine_amp_->setSuffix(" deg");
    spin_sine_amp_->setFixedWidth(160);
    spin_sine_amp_->installEventFilter(filter);
    motion_form->addRow("Sine Amplitude:", spin_sine_amp_);

    spin_sine_freq_ = new QDoubleSpinBox(motion_grp);
    spin_sine_freq_->setRange(0.01, 50.0);
    spin_sine_freq_->setDecimals(3);
    spin_sine_freq_->setValue(0.5);
    spin_sine_freq_->setSuffix(" Hz");
    spin_sine_freq_->setFixedWidth(160);
    spin_sine_freq_->installEventFilter(filter);
    motion_form->addRow("Sine Frequency:", spin_sine_freq_);

    jog_step_spin_ = new QDoubleSpinBox(motion_grp);
    jog_step_spin_->setRange(0.01, 360.0);
    jog_step_spin_->setValue(1.0);
    jog_step_spin_->setSuffix("°");
    auto* jog_row = new QWidget(motion_grp);
    auto* jog_layout = new QHBoxLayout(jog_row);
    jog_layout->setContentsMargins(0, 0, 0, 0);
    btn_jog_neg_ = new QPushButton("Jog -", jog_row);
    btn_jog_pos_ = new QPushButton("Jog +", jog_row);
    jog_layout->addWidget(btn_jog_neg_);
    jog_layout->addWidget(btn_jog_pos_);
    jog_layout->addStretch();
    motion_form->addRow("Jog step:", jog_step_spin_);
    motion_form->addRow(QString(), jog_row);

    top_row->addWidget(motion_grp, 3);

    auto* status = new QGroupBox("Telemetry (Live)", scroll_content);
    auto* status_form = new QFormLayout(status);
    lbl_sys_state_ = new QLabel("N/A", status);
    lbl_state_ = new QLabel("N/A", status);
    lbl_protection_ = new QLabel("N/A", status);
    lbl_axis_ = new QLabel("N/A", status);
    lbl_target_ = new QLabel("N/A", status);
    lbl_speed_ = new QLabel("N/A", status);
    lbl_torque_ = new QLabel("N/A", status);
    lbl_error_code_ = new QLabel("N/A", status);

    status_form->addRow("System State:", lbl_sys_state_);
    status_form->addRow("Motor Status:", lbl_state_);
    status_form->addRow("Protection:", lbl_protection_);
    status_form->addRow("Error Code:", lbl_error_code_);
    status_form->addRow("Actual Pos:", lbl_axis_);
    status_form->addRow("Target Pos:", lbl_target_);
    status_form->addRow("Speed:", lbl_speed_);
    status_form->addRow("Torque:", lbl_torque_);

    top_row->addWidget(status, 2);
    layout->addLayout(top_row, 1);

    auto* scope_group = new QGroupBox("Real-time Monitor", scroll_content);
    auto* scope_layout = new QVBoxLayout(scope_group);

    auto* scope_ctrl = new QHBoxLayout();
    scope_ctrl->addWidget(new QLabel("Signal:"));
    cmb_scope_signal_ = new QComboBox(scope_group);
    cmb_scope_signal_->addItems({"Position (deg)", "Velocity (deg/s)", "Torque (%)"});
    scope_ctrl->addWidget(cmb_scope_signal_);

    scope_ctrl->addWidget(new QLabel("Time Window:"));
    sld_scope_time_ = new QSlider(Qt::Horizontal, scope_group);
    sld_scope_time_->setRange(2, 60);
    sld_scope_time_->setValue(10);
    sld_scope_time_->setFixedWidth(140);
    lbl_scope_time_ = new QLabel("10 s", scope_group);
    scope_ctrl->addWidget(sld_scope_time_);
    scope_ctrl->addWidget(lbl_scope_time_);

    chk_plot_actual_pos_ = new QCheckBox("Actual", scope_group);
    chk_plot_actual_pos_->setChecked(true);
    chk_plot_target_pos_ = new QCheckBox("Target", scope_group);
    chk_plot_target_pos_->setChecked(true);
    chk_plot_actual_vel_ = new QCheckBox("Actual Vel", scope_group);
    chk_plot_actual_vel_->setChecked(true);
    chk_plot_target_vel_ = new QCheckBox("Target Vel", scope_group);
    chk_plot_target_vel_->setChecked(true);
    chk_plot_pos_error_ = new QCheckBox("Pos Error", scope_group);
    chk_plot_pos_error_->setChecked(true);

    scope_ctrl->addWidget(chk_plot_actual_pos_);
    scope_ctrl->addWidget(chk_plot_target_pos_);
    scope_ctrl->addWidget(chk_plot_actual_vel_);
    scope_ctrl->addWidget(chk_plot_target_vel_);
    scope_ctrl->addWidget(chk_plot_pos_error_);

    chk_auto_scale_ = new QCheckBox("Auto Scale", scope_group);
    chk_auto_scale_->setChecked(true);
    scope_ctrl->addWidget(chk_auto_scale_);
    scope_ctrl->addStretch();

    scope_ = new RDT::ScopeWidget(scope_group);
    scope_->setMinimumHeight(240);
    scope_->setXRange(10.0);
    scope_->setAutoRange(true);
    scope_->addChannel("actual", QColor(80, 200, 255));
    scope_->addChannel("target", QColor(255, 80, 80));
    scope_->addChannel("speed", QColor(255, 180, 80));
    scope_->addChannel("target_speed", QColor(180, 255, 120));
    scope_->addChannel("pos_error", QColor(255, 120, 255));
    scope_->addChannel("torque", QColor(180, 140, 255));

    scope_->setChannelVisibility("actual", chk_plot_actual_pos_->isChecked());
    scope_->setChannelVisibility("target", chk_plot_target_pos_->isChecked());
    scope_->setChannelVisibility("speed", chk_plot_actual_vel_->isChecked());
    scope_->setChannelVisibility("target_speed", chk_plot_target_vel_->isChecked());
    scope_->setChannelVisibility("pos_error", chk_plot_pos_error_->isChecked());

    connect(chk_plot_actual_pos_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("actual", checked);
    });
    connect(chk_plot_target_pos_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("target", checked);
    });
    connect(chk_plot_actual_vel_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("speed", checked);
    });
    connect(chk_plot_target_vel_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("target_speed", checked);
    });
    connect(chk_plot_pos_error_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("pos_error", checked);
    });
    connect(chk_auto_scale_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setAutoRange(checked);
    });
    connect(sld_scope_time_, &QSlider::valueChanged, this, [this](int sec) {
        if (scope_) scope_->setXRange(static_cast<double>(sec));
        if (lbl_scope_time_) lbl_scope_time_->setText(QString::number(sec) + " s");
    });
    connect(cmb_scope_signal_, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (scope_) scope_->clear();
    });

    scope_layout->addLayout(scope_ctrl);
    scope_layout->addWidget(scope_);

    layout->addStretch(1);
    layout->addWidget(scope_group, 0, Qt::AlignBottom);

    connect(chk_sine_enable_, &QCheckBox::toggled, this, &AxisWorkspace::onSineToggled);

    connect(btn_enable, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "enableMotor", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(bool, true));
    });
    connect(btn_disable, &QPushButton::clicked, this, [this]() {
        stopSineModeForDisable();
        QMetaObject::invokeMethod(manager_, "enableMotor", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(bool, false));
    });
    connect(btn_estop, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "emergencyStop", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_clear_err, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "clearErrors", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_home, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "goHome", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_set_zero, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "setZeroPosition", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });

    connect(btn_move, &QPushButton::clicked, this, &AxisWorkspace::triggerAbsoluteMove);

    connect(mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int idx) {
        if (!mode_combo_ || !manager_) {
            return;
        }
        const int mode_code = mode_combo_->itemData(idx).toInt();
        QMetaObject::invokeMethod(manager_, "setAxisMode", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(int, mode_code));
    });

    connect(target_slider_, &QSlider::valueChanged, this, [this](int raw) {
        if (!target_pos_spin_) {
            return;
        }
        const double v = static_cast<double>(raw) / 100.0;
        target_pos_spin_->blockSignals(true);
        target_pos_spin_->setValue(v);
        target_pos_spin_->blockSignals(false);
        desired_target_deg_.store(v, std::memory_order_relaxed);
        commanded_target_deg_.store(v, std::memory_order_relaxed);
        manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;
        QMetaObject::invokeMethod(manager_, "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, v));
    });

    connect(target_pos_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double v) {
        if (target_pos_spin_) {
            desired_target_deg_.store(v, std::memory_order_relaxed);
            commanded_target_deg_.store(v, std::memory_order_relaxed);
            manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1500;
            if (target_slider_) {
                target_slider_->blockSignals(true);
                target_slider_->setValue(static_cast<int>(std::llround(v * 100.0)));
                target_slider_->blockSignals(false);
            }
        }
    });

    connect(btn_jog_neg_, &QPushButton::clicked, this, [this]() {
        if (!manager_) return;
        const double step = jog_step_spin_ ? jog_step_spin_->value() : 1.0;
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, -step));
    });
    connect(btn_jog_pos_, &QPushButton::clicked, this, [this]() {
        if (!manager_) return;
        const double step = jog_step_spin_ ? jog_step_spin_->value() : 1.0;
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, step));
    });
    
    // Fast path for background thread speeds
    connect(speed_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v){ current_speed_.store(v, std::memory_order_relaxed); });
    connect(accel_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v){ current_accel_.store(v, std::memory_order_relaxed); });

    scroll->setWidget(scroll_content);
    root_layout->addWidget(scroll);
    tabs_->addTab(tab_root, "Control");
}

void AxisWorkspace::stopSineModeForDisable() {
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        std::queue<double> empty;
        std::swap(trajectory_queue_, empty);
    }

    if (chk_sine_enable_ && chk_sine_enable_->isChecked()) {
        chk_sine_enable_->blockSignals(true);
        chk_sine_enable_->setChecked(false);
        chk_sine_enable_->blockSignals(false);
        onSineToggled(false);
    }

    desired_target_deg_.store(commanded_target_deg_.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

void AxisWorkspace::setupConfigTab() {
    auto* tab_root = new QWidget(this);
    auto* root_layout = new QVBoxLayout(tab_root);
    root_layout->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea(tab_root);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* scroll_content = new QWidget(scroll);
    auto* layout = new QVBoxLayout(scroll_content);

    auto* btn_row = new QHBoxLayout();
    btn_refresh_list_ = new QPushButton("Refresh List", scroll_content);
    btn_read_params_ = new QPushButton("Read Values", scroll_content);
    btn_apply_params_ = new QPushButton("Apply Changes", scroll_content);
    btn_save_drive_flash_ = new QPushButton("Save Selected Persistently", scroll_content);
    auto* btn_export_full = new QPushButton("Export Config (AxisConfig)", scroll_content);
    auto* btn_import_full = new QPushButton("Import Config (AxisConfig)", scroll_content);
    btn_row->addWidget(btn_refresh_list_);
    btn_row->addWidget(btn_read_params_);
    btn_row->addWidget(btn_apply_params_);
    btn_row->addWidget(btn_save_drive_flash_);
    btn_row->addWidget(btn_export_full);
    btn_row->addWidget(btn_import_full);
    btn_row->addStretch();
    layout->addLayout(btn_row);

    auto* cfg_note = new QLabel(
        "<b>Config model:</b> AxisConfig import/export is the canonical configuration flow.",
        scroll_content);
    cfg_note->setWordWrap(true);
    layout->addWidget(cfg_note);

    auto* split = new QSplitter(Qt::Horizontal, scroll_content);
    
    config_tree_ = new QTreeWidget(split);
    config_tree_->setColumnCount(6);
    config_tree_->setHeaderLabels({"Name", "Group", "Unit", "Read Only", "Current Value", "New Value"});
    config_tree_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    config_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    config_tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    config_tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    config_tree_->setEditTriggers(QAbstractItemView::DoubleClicked |
                                  QAbstractItemView::SelectedClicked |
                                  QAbstractItemView::EditKeyPressed);
    config_tree_->setItemDelegateForColumn(kNewValueColumn, new NewValueColumnDelegate(config_tree_));
    
    txt_description_ = new QTextEdit(split);
    txt_description_->setReadOnly(true);
    txt_description_->setPlaceholderText("Select a parameter to view its details...");
    
    split->setSizes({600, 200});
    layout->addWidget(split, 1);
    
    connect(config_tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
        if (!current) {
            txt_description_->clear();
            return;
        }

        if (!current->data(kNameColumn, Qt::UserRole).isValid()) {
            txt_description_->setHtml(QString("<b>Category:</b> %1").arg(current->text(kNameColumn)));
            return;
        }
        
        // Build description text
        QString desc;
        desc += "<b>Name:</b> " + current->text(kNameColumn) + "<br>";
        desc += "<b>Group:</b> " + current->text(kGroupColumn) + "<br>";
        if (!current->text(kUnitColumn).isEmpty()) {
            desc += "<b>Unit:</b> " + current->text(kUnitColumn) + "<br>";
        }
        const int domain = current->data(kNameColumn, Qt::UserRole).toInt();
        const int value_id = current->data(kGroupColumn, Qt::UserRole).toInt();
        desc += "<b>Domain:</b> " + QString::number(domain) + "<br>";
        desc += "<b>Value ID:</b> " + QString::number(value_id) + "<br>";
        desc += "<b>Current:</b> " + current->text(kCurrentValueColumn) + "<br>";
        desc += "<b>Pending:</b> " + (current->text(kNewValueColumn).isEmpty() ? QString("-") : current->text(kNewValueColumn)) + "<br>";
        
        bool has_min = current->data(kUnitColumn, Qt::UserRole).toBool();
        bool has_max = current->data(kReadOnlyColumn, Qt::UserRole).toBool();
        QString min_val = current->data(kCurrentValueColumn, Qt::UserRole).toString();
        QString max_val = current->data(kNewValueColumn, Qt::UserRole).toString();
        
        if (has_min || has_max) {
            desc += "<br><b>Bounds:</b><br>";
            if (has_min) desc += "- Min: " + min_val + "<br>";
            if (has_max) desc += "- Max: " + max_val + "<br>";
        }

        const bool persistable = current->data(kNameColumn, Qt::UserRole + 1).toBool();
        desc += QString("<br><b>Persistable (AxisConfig):</b> %1<br>")
                    .arg(persistable ? "Yes" : "No");

        const QString protocol_details = protocolDetailsForParameter(domain, value_id);
        if (!protocol_details.isEmpty()) {
            desc += "<br><b>Protocol details:</b><br>" + protocol_details;
        }
        
        txt_description_->setHtml(desc);
    });

    connect(btn_refresh_list_, &QPushButton::clicked, this, &AxisWorkspace::refreshParameterList);
    connect(btn_read_params_, &QPushButton::clicked, this, &AxisWorkspace::readParametersFromDrive);
    connect(btn_apply_params_, &QPushButton::clicked, this, &AxisWorkspace::applyParametersPatch);
    connect(btn_save_drive_flash_, &QPushButton::clicked, this, &AxisWorkspace::saveSelectedParameterToDriveFlash);
    connect(btn_export_full, &QPushButton::clicked, this, &AxisWorkspace::exportAxisConfig);
    connect(btn_import_full, &QPushButton::clicked, this, &AxisWorkspace::importAxisConfig);

    scroll->setWidget(scroll_content);
    root_layout->addWidget(scroll);
    tabs_->addTab(tab_root, "Configuration");
}

void AxisWorkspace::refreshParameterList() {
    config_tree_->clear();
    txt_description_->clear();
    if (!manager_) return;

    QMetaObject::invokeMethod(manager_.data(), "requestListParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
}

void AxisWorkspace::onParameterListReady(int axis_id, const QVariantList& p_list) {
    if (axis_id != axis_id_) return;
    config_tree_->clear();
    txt_description_->clear();

    QHash<QString, QTreeWidgetItem*> group_roots;
    
    for (const auto& v : p_list) {
        QVariantMap map = v.toMap();

        const QString raw_group = map["group"].toString().trimmed();
        const QString group_name = raw_group.isEmpty() ? QStringLiteral("Ungrouped") : raw_group;
        QTreeWidgetItem* group_item = group_roots.value(group_name, nullptr);
        if (!group_item) {
            group_item = new QTreeWidgetItem(config_tree_);
            group_item->setText(kNameColumn, group_name);
            group_item->setFirstColumnSpanned(true);
            group_item->setExpanded(true);
            group_roots.insert(group_name, group_item);
        }

        auto* item = new QTreeWidgetItem(group_item);
        item->setText(0, map["name"].toString());
        item->setText(1, group_name);
        item->setText(2, map["unit"].toString());
        item->setText(3, map["read_only"].toBool() ? "Yes" : "No");
        item->setText(4, "Unknown");
        
        item->setData(kNameColumn, Qt::UserRole, map["domain"]);
        item->setData(kGroupColumn, Qt::UserRole, map["value"]);
        item->setData(kUnitColumn, Qt::UserRole, map["has_min"]);
        item->setData(kReadOnlyColumn, Qt::UserRole, map["has_max"]);
        item->setData(kCurrentValueColumn, Qt::UserRole, map["min_value"]);
        item->setData(kNewValueColumn, Qt::UserRole, map["max_value"]);
        item->setData(kNameColumn, Qt::UserRole + 1, map.value("persistable", true));
        
        if (!map["read_only"].toBool()) {
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item->setText(kNewValueColumn, "");
        } else {
            item->setText(kNewValueColumn, "N/A");
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        }
    }

    config_tree_->expandAll();
}

void AxisWorkspace::readParametersFromDrive() {
    if (!manager_) return;
    if (parameter_read_in_progress_) return;
    setParameterReadInProgress(true);
    QMetaObject::invokeMethod(manager_.data(), "requestReadParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
}

void AxisWorkspace::onParametersRead(int axis_id, const QVariantList& vals) {
    if (axis_id != axis_id_) return;
    setParameterReadInProgress(false);

    for (const auto& v : vals) {
        QVariantMap map = v.toMap();
        int domain = map["domain"].toInt();
        int val_id = map["value"].toInt();
        QString data = map["data"].toString();

        for (auto* item : parameterItems(config_tree_)) {
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain &&
                item->data(kGroupColumn, Qt::UserRole).toInt() == val_id) {
                item->setText(kCurrentValueColumn, data);
                break;
            }
        }
    }
}

void AxisWorkspace::setParameterReadInProgress(const bool in_progress) {
    parameter_read_in_progress_ = in_progress;
    if (btn_read_params_) {
        btn_read_params_->setEnabled(!in_progress);
        btn_read_params_->setText(in_progress ? "Reading..." : "Read Values");
    }
    if (btn_apply_params_) {
        btn_apply_params_->setEnabled(!in_progress);
    }
    if (btn_save_drive_flash_) {
        btn_save_drive_flash_->setEnabled(!in_progress);
    }
}

void AxisWorkspace::applyParametersPatch() {
    if (!manager_) return;

    auto parse_bool_token = [](const QString& token, bool* ok_out) -> bool {
        const QString normalized = token.trimmed().toLower();
        if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
            if (ok_out) *ok_out = true;
            return true;
        }
        if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
            if (ok_out) *ok_out = true;
            return false;
        }
        if (ok_out) *ok_out = false;
        return false;
    };

    auto variant_to_double = [](const QVariant& value, bool* ok_out) -> double {
        if (value.userType() == QMetaType::Bool) {
            if (ok_out) *ok_out = true;
            return value.toBool() ? 1.0 : 0.0;
        }
        bool ok = false;
        const double out = value.toDouble(&ok);
        if (ok_out) *ok_out = ok;
        return out;
    };

    QVariantList patch;
    QList<QTreeWidgetItem*> patched_items;
    QStringList validation_errors;
    bool can_id_change_requested = false;
    QStringList can_id_targets;

    for (auto* item : parameterItems(config_tree_)) {
        const QString new_val = item->text(kNewValueColumn).trimmed();
        if (new_val.isEmpty() || new_val == "N/A") {
            continue;
        }

        const bool has_min = item->data(kUnitColumn, Qt::UserRole).toBool();
        const bool has_max = item->data(kReadOnlyColumn, Qt::UserRole).toBool();
        const QVariant min_value = item->data(kCurrentValueColumn, Qt::UserRole);
        const QVariant max_value = item->data(kNewValueColumn, Qt::UserRole);
        const bool bool_like_bounds = has_min && has_max &&
                                      min_value.userType() == QMetaType::Bool &&
                                      max_value.userType() == QMetaType::Bool;

        QVariant parsed_value;
        bool parsed_ok = false;
        if (bool_like_bounds) {
            parsed_value = parse_bool_token(new_val, &parsed_ok);
        } else {
            bool bool_ok = false;
            const bool bool_value = parse_bool_token(new_val, &bool_ok);
            if (bool_ok) {
                parsed_value = bool_value;
                parsed_ok = true;
            } else {
                bool int_ok = false;
                const qlonglong int_value = new_val.toLongLong(&int_ok);
                if (int_ok && !new_val.contains('.') && !new_val.contains('e', Qt::CaseInsensitive)) {
                    parsed_value = int_value;
                    parsed_ok = true;
                } else {
                    bool double_ok = false;
                    const double double_value = new_val.toDouble(&double_ok);
                    if (double_ok) {
                        parsed_value = double_value;
                        parsed_ok = true;
                    } else {
                        parsed_value = new_val;
                        parsed_ok = !(has_min || has_max);
                    }
                }
            }
        }

        if (!parsed_ok) {
            validation_errors.push_back(QString("%1: invalid value '%2'")
                                            .arg(item->text(kNameColumn), new_val));
            continue;
        }

        if (has_min || has_max) {
            bool parsed_numeric_ok = false;
            const double parsed_numeric = variant_to_double(parsed_value, &parsed_numeric_ok);
            if (!parsed_numeric_ok) {
                validation_errors.push_back(QString("%1: value must be numeric/boolean")
                                                .arg(item->text(kNameColumn)));
                continue;
            }

            if (has_min) {
                bool min_ok = false;
                const double min_numeric = variant_to_double(min_value, &min_ok);
                if (min_ok && parsed_numeric < min_numeric) {
                    validation_errors.push_back(QString("%1: value %2 is below min %3")
                                                    .arg(item->text(kNameColumn))
                                                    .arg(new_val)
                                                    .arg(QString::number(min_numeric, 'g', 12)));
                    continue;
                }
            }
            if (has_max) {
                bool max_ok = false;
                const double max_numeric = variant_to_double(max_value, &max_ok);
                if (max_ok && parsed_numeric > max_numeric) {
                    validation_errors.push_back(QString("%1: value %2 is above max %3")
                                                    .arg(item->text(kNameColumn))
                                                    .arg(new_val)
                                                    .arg(QString::number(max_numeric, 'g', 12)));
                    continue;
                }
            }
        }

        QVariantMap map;
        map["domain"] = item->data(kNameColumn, Qt::UserRole);
        map["value"] = item->data(kGroupColumn, Qt::UserRole);
        map["data"] = parsed_value;

        const int domain_value = map["domain"].toInt();
        const int parameter_value = map["value"].toInt();
        if (domain_value == static_cast<int>(motion_core::ParameterDomain::Mks) &&
            parameter_value == static_cast<int>(mks::MksParameter::CanId)) {
            can_id_change_requested = true;
            can_id_targets.push_back(parsed_value.toString());
        }

        patch.push_back(map);
        patched_items.push_back(item);
    }

    if (!validation_errors.isEmpty()) {
        QMessageBox::warning(this,
                             "Invalid parameter values",
                             "Please fix invalid values before apply:\n- " + validation_errors.join("\n- "));
        return;
    }

    if (!patch.isEmpty()) {
        if (can_id_change_requested) {
            const auto answer = QMessageBox::warning(
                this,
                "CAN ID change",
                QString("Applying CAN ID change (%1) will switch the drive address immediately.\n"
                        "After apply, this workspace may stop receiving telemetry until re-scan/reconnect.\n\n"
                        "Continue?")
                    .arg(can_id_targets.join(", ")),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No);
            if (answer != QMessageBox::Yes) {
                return;
            }
        }

        auto* manager = manager_.data();
        QMetaObject::invokeMethod(manager, [manager, id = axis_id_, p = patch]() {
            manager->applyParameterPatch(id, p);
        }, Qt::QueuedConnection);

        for (auto* item : patched_items) {
            if (item) {
                item->setText(kNewValueColumn, "");
            }
        }

        QTimer::singleShot(120, this, [this]() { readParametersFromDrive(); });

        if (can_id_change_requested) {
            QMessageBox::information(
                this,
                "CAN ID changed",
                "CAN ID write requested. If telemetry freezes, perform Scan/Connect using the new CAN ID.");
        }
    }
}

void AxisWorkspace::saveSelectedParameterToDriveFlash() {
    if (!manager_ || !config_tree_) {
        return;
    }

    auto parse_bool_token = [](const QString& token, bool* ok_out) -> bool {
        const QString normalized = token.trimmed().toLower();
        if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
            if (ok_out) *ok_out = true;
            return true;
        }
        if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
            if (ok_out) *ok_out = true;
            return false;
        }
        if (ok_out) *ok_out = false;
        return false;
    };

    auto* selected = config_tree_->currentItem();
    if (!selected) {
        QMessageBox::warning(this,
                             "Persistent write",
                             "Select a parameter row first.");
        return;
    }

    const int domain = selected->data(kNameColumn, Qt::UserRole).toInt();
    const int value_id = selected->data(kGroupColumn, Qt::UserRole).toInt();
    const QString parameter_name = selected->text(kNameColumn).trimmed();
    const bool is_persistable = selected->data(kNameColumn, Qt::UserRole + 1).toBool();
    const bool is_read_only = selected->text(kReadOnlyColumn).compare("Yes", Qt::CaseInsensitive) == 0;
    const QString pending_value_text = selected->text(kNewValueColumn).trimmed();
    const QString current_value_text = selected->text(kCurrentValueColumn).trimmed();
    const bool has_pending_value = !pending_value_text.isEmpty() && pending_value_text != "N/A";
    const QString value_to_use = has_pending_value ? pending_value_text : current_value_text;

    if (value_to_use.isEmpty() || value_to_use == "Unknown" || value_to_use == "N/A") {
        QMessageBox::warning(this,
                             "Persistent write",
                             "Selected row has no valid value to persist.");
        return;
    }

    if (is_read_only) {
        QMessageBox::information(this,
                                 "Persistent write",
                                 "Selected parameter is read-only and cannot be saved persistently.");
        return;
    }

    if (!is_persistable) {
        QMessageBox::information(this,
                                 "Persistent write",
                                 "Selected parameter is marked as non-persistable for runtime EEPROM save.");
        return;
    }

    const bool is_supported_parameter =
        (domain == static_cast<int>(motion_core::ParameterDomain::Ethercat)
         && value_id != static_cast<int>(motion_core::EthercatParameter::SaveParametersToEeprom))
        || (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
            && value_id == static_cast<int>(mks::MksParameter::CanId));

    if (!is_supported_parameter) {
        QMessageBox::information(this,
                                 "Persistent write",
                                 "For now persistent write is supported for ordinary EtherCAT writable parameters and MKS CAN ID.");
        return;
    }

    QVariant parsed_value;
    bool parsed_ok = false;

    bool bool_ok = false;
    const bool bool_value = parse_bool_token(value_to_use, &bool_ok);
    if (bool_ok) {
        parsed_value = bool_value;
        parsed_ok = true;
    } else {
        bool int_ok = false;
        const qlonglong int_value = value_to_use.toLongLong(&int_ok);
        if (int_ok && !value_to_use.contains('.') && !value_to_use.contains('e', Qt::CaseInsensitive)) {
            parsed_value = int_value;
            parsed_ok = true;
        } else {
            bool double_ok = false;
            const double double_value = value_to_use.toDouble(&double_ok);
            if (double_ok) {
                parsed_value = double_value;
                parsed_ok = true;
            }
        }
    }

    if (!parsed_ok) {
        QMessageBox::warning(this,
                             "Persistent write",
                             "Selected persistent value must be numeric or boolean.");
        return;
    }

    const auto answer = QMessageBox::warning(
        this,
        "Persistent write",
        QString("Write %1=%2 and store it persistently?\n"
                "Use this only if you are sure the selected value is correct.")
            .arg(parameter_name.isEmpty() ? QStringLiteral("parameter") : parameter_name)
            .arg(parsed_value.toString()),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes) {
        return;
    }

    auto* manager = manager_.data();
    QMetaObject::invokeMethod(
        manager,
        [manager, axis_id = axis_id_, domain, value_id, parsed_value]() {
            manager->setPersistentParameter(axis_id,
                                            domain,
                                            value_id,
                                            parsed_value);
        },
        Qt::QueuedConnection);

    QTimer::singleShot(150, this, [this]() { readParametersFromDrive(); });
}

void AxisWorkspace::onSineToggled(bool enabled) {
    sine_enabled_.store(enabled, std::memory_order_release);
    if (target_pos_spin_) target_pos_spin_->setEnabled(!enabled);
    
    if (manager_ && enabled) {
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            std::queue<double> empty;
            std::swap(trajectory_queue_, empty);
        }
        
        const double start_center_deg = displayed_actual_deg_;
        sine_center_deg_.store(start_center_deg, std::memory_order_relaxed);
        desired_target_deg_.store(start_center_deg, std::memory_order_relaxed);
        commanded_target_deg_.store(start_center_deg, std::memory_order_relaxed);
        sine_phase_accum_rad_ = 0.0;
        last_sine_ui_sync_ms_.store(0, std::memory_order_release);
        
        QFile::remove("sine_dump.csv");
        fillTrajectoryQueue(); // Pre-calculate the first segment immediately
    }

    if (!enabled) {
        desired_target_deg_.store(commanded_target_deg_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
}

void AxisWorkspace::fillTrajectoryQueue() {
    if (!sine_enabled_.load(std::memory_order_acquire)) return;

    constexpr int kTargetDtMs = 4;
    constexpr double kTargetDtSec = kTargetDtMs / 1000.0;
    constexpr size_t kTargetBufferSize = 250; // 1 second buffer (250 * 4ms)

    size_t current_size = 0;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_size = trajectory_queue_.size();
    }

    if (current_size >= kTargetBufferSize) return;

    const double target_amp = spin_sine_amp_ ? spin_sine_amp_->value() : 0.0;
    const double target_freq = spin_sine_freq_ ? spin_sine_freq_->value() : 0.0;
    const double center = sine_center_deg_.load(std::memory_order_relaxed);

    std::vector<double> new_points;
    new_points.reserve(kTargetBufferSize - current_size);

    size_t start_index = current_size;
    if (start_index == 0U && std::abs(sine_phase_accum_rad_) < 1e-12) {
        // First point must start exactly at current center (no initial phase jump).
        new_points.push_back(center);
        start_index = 1U;
    }

    for (size_t i = start_index; i < kTargetBufferSize; ++i) {
        sine_phase_accum_rad_ += 2.0 * M_PI * target_freq * kTargetDtSec;
        if (sine_phase_accum_rad_ > 2.0 * M_PI) {
            sine_phase_accum_rad_ = std::fmod(sine_phase_accum_rad_, 2.0 * M_PI);
        }
        new_points.push_back(center + target_amp * std::sin(sine_phase_accum_rad_));
    }

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        for (double p : new_points) {
            trajectory_queue_.push(p);
        }
    }
}

void AxisWorkspace::trajectoryLoop() {
    constexpr int kTargetDtMs = 4; // 250Hz target loop
    
    auto next_wakeup = std::chrono::steady_clock::now();

    while (trajectory_thread_active_.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now > next_wakeup + std::chrono::milliseconds(kTargetDtMs * 4)) {
            // If scheduler/OS caused a large delay, re-sync and skip catch-up burst.
            next_wakeup = now;
        }
        next_wakeup += std::chrono::milliseconds(kTargetDtMs);
        
        if (!manager_) {
            std::this_thread::sleep_until(next_wakeup);
            continue;
        }

        bool has_target = false;
        double new_target = 0.0;

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (!trajectory_queue_.empty()) {
                new_target = trajectory_queue_.front();
                trajectory_queue_.pop();
                has_target = true;
            }
        }

        if (has_target) {
            desired_target_deg_.store(new_target, std::memory_order_relaxed);
            
            QMetaObject::invokeMethod(manager_.data(), "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, current_speed_.load(std::memory_order_relaxed)),
                                  Q_ARG(int, current_accel_.load(std::memory_order_relaxed)),
                                  Q_ARG(double, new_target));

            const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
            const bool sine_active = sine_enabled_.load(std::memory_order_acquire);
            const qint64 last_sync_ms = last_sine_ui_sync_ms_.load(std::memory_order_acquire);
            const bool should_sync_ui = !sine_active || (now_ms - last_sync_ms >= 80);
            if (should_sync_ui) {
                last_sine_ui_sync_ms_.store(now_ms, std::memory_order_release);
                QMetaObject::invokeMethod(this, [this, new_target](){
                    if (target_pos_spin_) {
                        target_pos_spin_->blockSignals(true);
                        target_pos_spin_->setValue(new_target);
                        target_pos_spin_->blockSignals(false);
                    }
                    if (target_slider_) {
                        target_slider_->blockSignals(true);
                        target_slider_->setValue(static_cast<int>(std::llround(new_target * 100.0)));
                        target_slider_->blockSignals(false);
                    }
                }, Qt::QueuedConnection);
            }
            
        } else {
            // Keep last desired target from atomic state; avoid blocking UI calls from worker thread.
            desired_target_deg_.store(desired_target_deg_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        
        commanded_target_deg_.store(desired_target_deg_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        double cmd_tgt = commanded_target_deg_.load(std::memory_order_relaxed);
        
        double t_sec = 0.0;
        if (telemetry_t0_ms_ > 0) {
            t_sec = static_cast<double>(QDateTime::currentMSecsSinceEpoch() - telemetry_t0_ms_) / 1000.0;
        }

        if (has_target && telemetry_t0_ms_ > 0) {
            QMetaObject::invokeMethod(this, [this, t_sec, cmd_tgt]() {
                if (!scope_) return;
                if (chk_plot_target_pos_ && chk_plot_target_pos_->isChecked()) {
                    scope_->addData("target", t_sec, cmd_tgt);
                }
                if (chk_plot_target_vel_ && chk_plot_target_vel_->isChecked()) {
                    if (have_prev_target_sample_ && (t_sec - prev_target_sample_time_sec_) > 1e-6) {
                        const double target_vel = (cmd_tgt - prev_target_sample_deg_) / (t_sec - prev_target_sample_time_sec_);
                        scope_->addData("target_speed", t_sec, target_vel);
                    }
                    prev_target_sample_deg_ = cmd_tgt;
                    prev_target_sample_time_sec_ = t_sec;
                    have_prev_target_sample_ = true;
                }
            }, Qt::QueuedConnection);
        }
        
        std::this_thread::sleep_until(next_wakeup);
    }
}

void AxisWorkspace::setTargetPosition(double pos_deg) {
    if (target_pos_spin_) {
        manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 800;
        desired_target_deg_.store(pos_deg, std::memory_order_relaxed);
        commanded_target_deg_.store(pos_deg, std::memory_order_relaxed);
        target_pos_spin_->setValue(pos_deg);
    }
    if (target_slider_) {
        target_slider_->blockSignals(true);
        target_slider_->setValue(static_cast<int>(std::llround(pos_deg * 100.0)));
        target_slider_->blockSignals(false);
    }
}

void AxisWorkspace::triggerAbsoluteMove() {
    if (!manager_) return;

    if (target_pos_spin_) {
        // Commit in-progress typed text before reading value (important when user types and directly clicks Move).
        target_pos_spin_->interpretText();
    }

    const double manual_target = target_pos_spin_ ? target_pos_spin_->value() : 0.0;
    desired_target_deg_.store(manual_target, std::memory_order_relaxed);
    commanded_target_deg_.store(manual_target, std::memory_order_relaxed);
    manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;
    
    if (radio_move_rel_ && radio_move_rel_->isChecked()) {
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, manual_target));
    } else {
        QMetaObject::invokeMethod(manager_, "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, manual_target));
    }
}

bool AxisWorkspace::isTargetReached(double tolerance_deg) const {
    return std::abs(displayed_actual_deg_ - (target_pos_spin_ ? target_pos_spin_->value() : 0)) <= tolerance_deg;
}

void AxisWorkspace::onTelemetryUpdated(int axis_id, const QVariantMap& telemetry) {
    if (axis_id != axis_id_) return;
    const QString transport = telemetry.value("transport").toString();
    const bool is_ethercat =
        transport.compare(QStringLiteral("ethercat"), Qt::CaseInsensitive) == 0;
    if (accel_spin_ && accel_marked_inactive_for_ethercat_ != is_ethercat) {
        accel_marked_inactive_for_ethercat_ = is_ethercat;
        accel_spin_->setEnabled(!is_ethercat);
        if (is_ethercat) {
            accel_spin_->setToolTip(QStringLiteral("Inactive for EtherCAT: accel command is currently not applied in runtime."));
            accel_spin_->setStyleSheet(QStringLiteral("color: #808080;"));
        } else {
            accel_spin_->setToolTip({});
            accel_spin_->setStyleSheet({});
        }
    }

    const int state_code = telemetry.value("state").toInt();
    const int status_word = telemetry.value("status").toInt();
    const int motion_status = telemetry.value("motion_status").toInt();
    const int protection_code = telemetry.value("protection").toInt();
    const int error_code = telemetry.value("error_code").toInt();

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (telemetry_t0_ms_ == 0) {
        telemetry_t0_ms_ = now_ms;
    }
    const double t_sec = static_cast<double>(now_ms - telemetry_t0_ms_) / 1000.0;

    if (scope_) {
        const QString signal = cmb_scope_signal_ ? cmb_scope_signal_->currentText() : QStringLiteral("Position (deg)");
        const bool show_position = signal.startsWith("Position");
        const bool show_velocity = signal.startsWith("Velocity");
        const bool show_torque = signal.startsWith("Torque");

        scope_->setChannelVisibility("actual", show_position && chk_plot_actual_pos_ && chk_plot_actual_pos_->isChecked());
        scope_->setChannelVisibility("target", show_position && chk_plot_target_pos_ && chk_plot_target_pos_->isChecked());
        scope_->setChannelVisibility("pos_error", show_position && chk_plot_pos_error_ && chk_plot_pos_error_->isChecked());
        scope_->setChannelVisibility("speed", show_velocity && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked());
        scope_->setChannelVisibility("target_speed", show_velocity && chk_plot_target_vel_ && chk_plot_target_vel_->isChecked());
        scope_->setChannelVisibility("torque", show_torque && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked());

        if (show_position && chk_plot_actual_pos_ && chk_plot_actual_pos_->isChecked() && telemetry.contains("axis")) {
            scope_->addData("actual", t_sec, telemetry.value("axis").toDouble());
        }
        if (show_position && chk_plot_target_pos_ && chk_plot_target_pos_->isChecked() && telemetry.contains("target") && !chk_sine_enable_->isChecked()) {
            scope_->addData("target", t_sec, telemetry.value("target").toDouble());
        }
        if (show_position && chk_plot_pos_error_ && chk_plot_pos_error_->isChecked() && telemetry.contains("axis") && telemetry.contains("target")) {
            const double err = telemetry.value("target").toDouble() - telemetry.value("axis").toDouble();
            scope_->addData("pos_error", t_sec, err);
        }
        if (show_velocity && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked() && telemetry.contains("speed")) {
            scope_->addData("speed", t_sec, telemetry.value("speed").toDouble());
        }
        if (show_torque && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked() && telemetry.contains("torque")) {
            scope_->addData("torque", t_sec, telemetry.value("torque").toDouble());
        }
    }

    if (telemetry.contains("state")) {
        if (lbl_sys_state_) lbl_sys_state_->setText(decodeSystemState(state_code));
    }
    if (telemetry.contains("status")) {
        lbl_state_->setText(decodeMotorStatusText(transport, state_code, motion_status, status_word));
    }
    if (telemetry.contains("axis")) {
        const double axis_val = telemetry.value("axis").toDouble();
        displayed_actual_deg_ = axis_val; // Save for sequencer checking
        lbl_axis_->setText(QString::number(axis_val, 'f', 2) + " °");
        if (!target_seeded_from_telemetry_ && target_pos_spin_) {
            target_seeded_from_telemetry_ = true;
            target_pos_spin_->blockSignals(true);
            target_pos_spin_->setValue(axis_val);
            target_pos_spin_->blockSignals(false);
            desired_target_deg_.store(axis_val, std::memory_order_relaxed);
            commanded_target_deg_.store(axis_val, std::memory_order_relaxed);
        }
    }
    if (telemetry.contains("target")) {
        const double target_val = telemetry.value("target").toDouble();
        if (lbl_target_) lbl_target_->setText(QString::number(target_val, 'f', 2) + " °");
        const qint64 now_ms_guard = QDateTime::currentMSecsSinceEpoch();
        const bool manual_hold_active = now_ms_guard < manual_target_hold_until_ms_;
        if (target_pos_spin_ && (!chk_sine_enable_ || !chk_sine_enable_->isChecked()) && !target_pos_spin_->hasFocus() && !manual_hold_active) {
            target_pos_spin_->blockSignals(true);
            target_pos_spin_->setValue(target_val);
            target_pos_spin_->blockSignals(false);
            if (target_slider_) {
                target_slider_->blockSignals(true);
                target_slider_->setValue(static_cast<int>(std::llround(target_val * 100.0)));
                target_slider_->blockSignals(false);
            }
        }
    } else if (lbl_target_) {
        lbl_target_->setText(QString::number(target_pos_spin_->value(), 'f', 2) + " °");
    }

    if (telemetry.contains("speed")) {
        lbl_speed_->setText(QString::number(telemetry.value("speed").toDouble(), 'f', 1) + " °/s");
    }
    if (telemetry.contains("torque")) {
        lbl_torque_->setText(QString::number(telemetry.value("torque").toDouble(), 'f', 1) + " %");
    }
    if (telemetry.contains("protection")) {
        lbl_protection_->setText(decodeProtectionText(transport, protection_code));
    }
    if (telemetry.contains("error_code") && lbl_error_code_) {
        lbl_error_code_->setText(decodeErrorCodeText(transport, error_code));
    }
}

// Removed config readback slot

void AxisWorkspace::exportAxisConfig() {
    if (!manager_) return;

    const QString path = QFileDialog::getSaveFileName(
        this,
        "Export AxisConfig",
        QString("axis_%1.axis_config.json").arg(axis_id_),
        "JSON Files (*.json)");
    if (path.isEmpty()) {
        return;
    }

    auto* manager = manager_.data();
    QMetaObject::invokeMethod(
        manager,
        [manager, axis_id = axis_id_, path]() {
            manager->exportAxisConfig(axis_id, path);
        },
        Qt::QueuedConnection);
}

void AxisWorkspace::importAxisConfig() {
    if (!manager_) return;

    const QString path = QFileDialog::getOpenFileName(
        this,
        "Import AxisConfig",
        QString(),
        "JSON Files (*.json)");
    if (path.isEmpty()) {
        return;
    }

    auto* manager = manager_.data();
    QMetaObject::invokeMethod(
        manager,
        [manager, axis_id = axis_id_, path]() {
            manager->importAxisConfigPreview(axis_id, path);
        },
        Qt::QueuedConnection);
}

void AxisWorkspace::onAxisConfigPreviewReady(int axis_id, const QVariantList& patch_entries) {
    if (axis_id != axis_id_ || !config_tree_) {
        return;
    }

    for (auto* item : parameterItems(config_tree_)) {
        if (!item) {
            continue;
        }
        if (item->text(kReadOnlyColumn).compare("Yes", Qt::CaseInsensitive) == 0) {
            continue;
        }
        item->setText(kNewValueColumn, "");
    }

    for (const auto& entry_variant : patch_entries) {
        const auto map = entry_variant.toMap();
        const int domain = map.value("domain").toInt();
        const int value_id = map.value("value").toInt();
        const QString data = map.value("data").toString();

        for (auto* item : parameterItems(config_tree_)) {
            if (!item) {
                continue;
            }
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain &&
                item->data(kGroupColumn, Qt::UserRole).toInt() == value_id) {
                if (item->text(kReadOnlyColumn).compare("Yes", Qt::CaseInsensitive) != 0) {
                    item->setText(kNewValueColumn, data);
                }
                break;
            }
        }
    }
}
