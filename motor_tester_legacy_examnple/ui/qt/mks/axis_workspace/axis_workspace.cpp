#include "mks/axis_workspace/axis_workspace.h"

#include "mks/axis_workspace/axis_workspace_config_panel.h"
#include "mks/axis_workspace/axis_workspace_control_panel.h"
#include "mks/axis_manager.h"
#include "mks/sequencer_widget.h"
#include "mks_can/dictionary/mks_dictionary.h"
#include "motion_core/parameter_id.h"

#include <QDateTime>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "mks/ScopeWidget.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace {

constexpr int kNameColumn = 0;
constexpr int kGroupColumn = 1;
constexpr int kUnitColumn = 2;
constexpr int kReadOnlyColumn = 3;
constexpr int kCurrentValueColumn = 4;
constexpr int kNewValueColumn = 5;
constexpr int kDefaultSpeedRpm = 1800;
constexpr int kDefaultAccelPercent = 100;

QString protocolDetailsForParameter(const int domain, const int value) {
    using motion_core::CommonParameter;
    using motion_core::ParameterDomain;

    if (domain == static_cast<int>(ParameterDomain::Common)) {
        if (value == static_cast<int>(CommonParameter::HardwareGearRatio)) {
            return QStringLiteral("<b>Gear Ratio</b><br>Mechanical transmission ratio used for runtime conversions.");
        }
        if (value == static_cast<int>(CommonParameter::HardwareEncoderResolutionBits)) {
            return QStringLiteral("<b>Encoder Resolution</b><br>Software-side encoder resolution used for ticks-to-degrees conversion.");
        }
        if (value == static_cast<int>(CommonParameter::MotorSelectionCode)) {
            return QStringLiteral("<b>Motor Selection Code</b><br>Persistent motor type selection used by runtime/drive integration.");
        }
        return {};
    }

    if (domain != static_cast<int>(ParameterDomain::Mks)) {
        return {};
    }

    switch (static_cast<mks::MksParameter>(value)) {
        case mks::MksParameter::CanId:
            return QStringLiteral("<b>CAN ID</b><br>After write, the axis becomes reachable only via the new CAN ID.");
        case mks::MksParameter::CanBitrateIndex:
            return QStringLiteral("<b>CAN Bitrate</b><br>After write, manual reconnect at the new bitrate is required.");
        case mks::MksParameter::WorkMode:
            return QStringLiteral("<b>Work Mode</b><br>Selects drive control source/profile.");
        case mks::MksParameter::EnableMotor:
            return QStringLiteral("<b>Enable Motor</b><br>Drive power stage enable/disable control.");
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

QString valueToDisplayString(const QVariant& value) {
    if (!value.isValid()) {
        return QStringLiteral("Unknown");
    }
    if (value.userType() == QMetaType::Bool) {
        return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
    }
    return value.toString();
}

QString format_digital_inputs_first_8(const std::uint32_t digital_inputs) {
    QStringList parts;
    parts.reserve(8);
    for (int i = 0; i < 8; ++i) {
        const int bit_value = ((digital_inputs >> i) & 0x1U) != 0U ? 1 : 0;
        parts.push_back(QStringLiteral("DI%1=%2").arg(i + 1).arg(bit_value));
    }

    QString raw_bits;
    raw_bits.reserve(8);
    for (int bit = 7; bit >= 0; --bit) {
        raw_bits.append(((digital_inputs >> bit) & 0x1U) != 0U ? QChar('1') : QChar('0'));
    }

    return parts.join(QStringLiteral(", ")) + QStringLiteral(" | raw[7:0]=") + raw_bits;
}

} // namespace

AxisWorkspace::AxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent)
    : QWidget(parent), axis_id_(axis_id), manager_(manager) {
    setupUi();

    connect(manager_, &mks::AxisManager::telemetryUpdated, this, &AxisWorkspace::onTelemetryUpdated);
    connect(manager_, &mks::AxisManager::axisConfigPreviewReady, this, &AxisWorkspace::onAxisConfigPreviewReady);
    connect(manager_, &mks::AxisManager::parameterListReady, this, &AxisWorkspace::onParameterListReady);
    connect(manager_, &mks::AxisManager::parametersRead, this, &AxisWorkspace::onParametersRead);
    connect(manager_, &mks::AxisManager::motionQueueStatsUpdated, this, &AxisWorkspace::onMotionQueueStatsUpdated);
    connect(manager_, &mks::AxisManager::hostStateUpdated, this, &AxisWorkspace::onHostStateUpdated);
}

AxisWorkspace::~AxisWorkspace() {
    scheduleWatchAxis(false);
}

void AxisWorkspace::applyCurrentModeSelection() {
    if (!manager_ || !control_panel_ || !control_panel_->handles().mode_combo) {
        return;
    }

    QMetaObject::invokeMethod(manager_.data(),
                              "setAxisMode",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id_),
                              Q_ARG(int, control_panel_->handles().mode_combo->currentData().toInt()));
}

void AxisWorkspace::setModeOptions(const QList<QPair<QString, int>>& options, int current_index) {
    if (!control_panel_ || !control_panel_->handles().mode_combo) {
        return;
    }

    auto* combo = control_panel_->handles().mode_combo;
    combo->blockSignals(true);
    combo->clear();
    for (const auto& option : options) {
        combo->addItem(option.first, option.second);
    }
    if (!options.isEmpty()) {
        const int last_index = static_cast<int>(options.size()) - 1;
        combo->setCurrentIndex(std::clamp(current_index, 0, last_index));
    }
    combo->blockSignals(false);
}

void AxisWorkspace::setSineControlsEnabled(bool enabled, const QString& tool_tip) {
    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    if (h.chk_sine_enable) {
        h.chk_sine_enable->setEnabled(enabled);
        h.chk_sine_enable->setToolTip(tool_tip);
        if (!enabled && h.chk_sine_enable->isChecked()) {
            h.chk_sine_enable->setChecked(false);
        }
    }
    if (h.spin_sine_amp) {
        h.spin_sine_amp->setEnabled(enabled);
        h.spin_sine_amp->setToolTip(tool_tip);
    }
    if (h.spin_sine_freq) {
        h.spin_sine_freq->setEnabled(enabled);
        h.spin_sine_freq->setToolTip(tool_tip);
    }
}

void AxisWorkspace::setMotionQueueStatsPlaceholder(const QString& text, const QString& style_sheet) {
    if (!control_panel_ || !control_panel_->handles().lbl_motion_queue_stats) {
        return;
    }

    control_panel_->handles().lbl_motion_queue_stats->setText(text);
    control_panel_->handles().lbl_motion_queue_stats->setStyleSheet(style_sheet);
}

void AxisWorkspace::resetUiAfterSetZero() {
    desired_target_deg_.store(0.0, std::memory_order_relaxed);
    commanded_target_deg_.store(0.0, std::memory_order_relaxed);
    displayed_actual_deg_ = 0.0;
    target_seeded_from_telemetry_ = true;
    have_prev_target_sample_ = false;
    prev_target_sample_deg_ = 0.0;
    prev_target_sample_time_sec_ = 0.0;
    telemetry_t0_ms_ = 0;
    telemetry_t0_ns_ = 0U;
    scope_target_time_cursor_sec_ = 0.0;
    scope_target_time_cursor_initialized_ = false;
    manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1000;

    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    if (h.target_pos_spin) {
        h.target_pos_spin->blockSignals(true);
        h.target_pos_spin->setValue(0.0);
        h.target_pos_spin->blockSignals(false);
    }
    if (h.target_slider) {
        h.target_slider->blockSignals(true);
        h.target_slider->setValue(0);
        h.target_slider->blockSignals(false);
    }
    if (h.lbl_axis) {
        h.lbl_axis->setText(QStringLiteral("0.00 °"));
    }
    if (h.lbl_target) {
        h.lbl_target->setText(QStringLiteral("0.00 °"));
    }
    if (h.lbl_motion_queue_stats) {
        h.lbl_motion_queue_stats->setText(
            QStringLiteral("queue: size=0 / 0, pushed=0, dropped=0, underruns=0, short_starts=0"));
        h.lbl_motion_queue_stats->setStyleSheet({});
    }
    if (h.scope) {
        h.scope->clear();
    }
}

void AxisWorkspace::onTransportSineToggled(bool enabled) {
    Q_UNUSED(enabled);
}

void AxisWorkspace::onTransportMotionQueueStatsUpdated(const QVariantMap& stats) {
    Q_UNUSED(stats);
}

void AxisWorkspace::onTransportTelemetryUpdated(const QVariantMap& telemetry,
                                                const QString& transport,
                                                double t_sec) {
    Q_UNUSED(telemetry);
    Q_UNUSED(transport);
    Q_UNUSED(t_sec);
}

bool AxisWorkspace::transportOwnsTargetUi() const {
    return false;
}

bool AxisWorkspace::transportProvidesTargetTrace() const {
    return false;
}

void AxisWorkspace::onBeforeDisableAxis() {
}

void AxisWorkspace::onBeforeHomeAxis() {
}

void AxisWorkspace::onBeforeSetZeroAxis() {
}

bool AxisWorkspace::supportsSineMode() const {
    return true;
}

void AxisWorkspace::disableSineMode() {
    if (!control_panel_ || !control_panel_->handles().chk_sine_enable) {
        return;
    }
    auto* checkbox = control_panel_->handles().chk_sine_enable;
    if (!checkbox->isChecked()) {
        sine_enabled_.store(false, std::memory_order_release);
        return;
    }
    checkbox->blockSignals(true);
    checkbox->setChecked(false);
    checkbox->blockSignals(false);
    onTransportSineToggled(false);
}

void AxisWorkspace::ensureMotionQueueConfigured() {
    if (!manager_ || motion_queue_configured_) {
        return;
    }
    QMetaObject::invokeMethod(manager_.data(),
                              "configureMotionQueue",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id_),
                              Q_ARG(int, kMotionQueueCapacity),
                              Q_ARG(bool, false));
    motion_queue_configured_ = true;
}

void AxisWorkspace::fillTrajectoryQueue() {
    if (!sine_enabled_.load(std::memory_order_acquire) || !control_panel_ || !supportsSineMode()) {
        pending_refill_points_ = 0U;
        return;
    }

    std::size_t runtime_queue_size = driver_queue_size_.load(std::memory_order_acquire);
    if (manager_) {
        const auto direct_stats = manager_->queryMotionQueueStatsDirect(axis_id_);
        if (direct_stats.ok()) {
            runtime_queue_size = direct_stats.value().size;
            driver_queue_size_.store(runtime_queue_size, std::memory_order_release);
        }
    }
    const bool is_prefill_phase = !motion_queue_prefilled_;

    if (runtime_queue_size >= static_cast<std::size_t>(kMotionQueueCapacity)) {
        pending_refill_points_ = 0U;
        return;
    }

    std::size_t points_to_generate = 0U;
    if (is_prefill_phase) {
        if (runtime_queue_size < kMotionQueuePrefillSamples) {
            points_to_generate = kMotionQueuePrefillSamples - runtime_queue_size;
        }
    } else {
        if (runtime_queue_size >= kMotionQueueLowWatermarkSamples) {
            pending_refill_points_ = 0U;
            return;
        }
        points_to_generate = kMotionQueueLowWatermarkSamples - runtime_queue_size;
    }

    if (points_to_generate == 0U) {
        pending_refill_points_ = 0U;
        return;
    }

    const std::size_t available_capacity = static_cast<std::size_t>(kMotionQueueCapacity) - runtime_queue_size;
    if (available_capacity == 0U) {
        pending_refill_points_ = 0U;
        return;
    }
    points_to_generate = std::min(points_to_generate, available_capacity);
    points_to_generate = std::min(points_to_generate, kUiTrajectorySourceBufferSize);
    pending_refill_points_ = points_to_generate;
}

void AxisWorkspace::flushTrajectoryBatchToRuntime() {
    if (!manager_ || !control_panel_ || !sine_enabled_.load(std::memory_order_acquire) || !supportsSineMode()) {
        return;
    }

    ensureMotionQueueConfigured();

    const std::size_t batch_size = std::max<std::size_t>(pending_refill_points_, 0U);
    if (batch_size == 0U) {
        return;
    }
    pending_refill_points_ = 0U;

    const double sample_period = samplePeriodSec();
    const int speed = current_speed_.load(std::memory_order_relaxed);
    const int accel = current_accel_.load(std::memory_order_relaxed);
    const auto& handles = control_panel_->handles();
    const double target_amp = handles.spin_sine_amp ? handles.spin_sine_amp->value() : 0.0;
    const double target_freq = handles.spin_sine_freq ? handles.spin_sine_freq->value() : 0.0;
    const double center = sine_center_deg_.load(std::memory_order_relaxed);

    std::vector<double> batch_points;
    batch_points.reserve(batch_size);
    std::vector<motion_core::QueuedSetpoint> direct_points;
    direct_points.reserve(batch_size);
    for (std::size_t i = 0U; i < batch_size; ++i) {
        sine_phase_accum_rad_ += 2.0 * M_PI * target_freq * sample_period;
        if (sine_phase_accum_rad_ > 2.0 * M_PI) {
            sine_phase_accum_rad_ = std::fmod(sine_phase_accum_rad_, 2.0 * M_PI);
        }
        const double pos_deg = center + target_amp * std::sin(sine_phase_accum_rad_);
        batch_points.push_back(pos_deg);

        motion_core::QueuedSetpoint point{};
        point.target_position_deg = pos_deg;
        point.has_profile_speed_rpm = true;
        point.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
        // MKS uses accel; EtherCAT does not.
        const bool use_accel = (transportTag() == QStringLiteral("mks"));
        point.has_profile_accel_percent = use_accel;
        point.profile_accel_percent = use_accel ? static_cast<double>(std::clamp(accel, 0, 100)) : 0.0;
        point.sample_period_sec = sample_period;
        direct_points.push_back(point);
    }

    const auto enqueue_result = manager_->enqueueMotionBatchDirect(axis_id_, direct_points);
    std::size_t accepted_points = 0U;
    if (enqueue_result.ok()) {
        const auto& queue_stats = enqueue_result.value();
        driver_queue_size_.store(queue_stats.size, std::memory_order_release);
        if (queue_stats.pushed >= static_cast<std::uint64_t>(batch_points.size())) {
            accepted_points = batch_points.size();
        }
    }

    if (accepted_points == 0U) {
        return;
    }

    batch_points.resize(accepted_points);

    const double last_target = batch_points.back();
    desired_target_deg_.store(last_target, std::memory_order_relaxed);
    commanded_target_deg_.store(last_target, std::memory_order_relaxed);

    auto& h = control_panel_->handles();
    if (h.scope && h.cmb_scope_signal && h.cmb_scope_signal->currentText().startsWith(QStringLiteral("Position"))) {
        if (telemetry_t0_ns_ == 0U) {
            return;
        }
        const auto now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        const double now_t_sec = static_cast<double>(now_ns - telemetry_t0_ns_) / 1e9;

        if (!scope_target_time_cursor_initialized_) {
            scope_target_time_cursor_sec_ = now_t_sec;
            scope_target_time_cursor_initialized_ = true;
        }

        double target_sample_t_sec = scope_target_time_cursor_sec_;
        if (h.chk_plot_target_pos && h.chk_plot_target_pos->isChecked()) {
            QVector<QPointF> target_batch;
            target_batch.reserve(static_cast<int>(accepted_points));
            for (const double target_point : batch_points) {
                target_batch.append(QPointF(target_sample_t_sec, target_point));
                target_sample_t_sec += sample_period;
            }
            h.scope->addDataBatch(QStringLiteral("target"), target_batch);
        } else {
            target_sample_t_sec += sample_period * static_cast<double>(accepted_points);
        }
        scope_target_time_cursor_sec_ = target_sample_t_sec;

        if (h.chk_plot_pos_error && h.chk_plot_pos_error->isChecked()) {
            h.scope->addData(QStringLiteral("pos_error"), now_t_sec, last_target - displayed_actual_deg_);
        }
    }

    if (!motion_queue_prefilled_) {
        motion_queue_prefilled_ = true;
    }
}

void AxisWorkspace::clearMotionBuffersForServiceCommand(bool reset_ui_to_zero) {
    pending_refill_points_ = 0U;
    motion_queue_prefilled_ = false;
    driver_queue_size_.store(0U, std::memory_order_release);
    have_prev_target_sample_ = false;
    prev_target_sample_deg_ = 0.0;
    prev_target_sample_time_sec_ = 0.0;

    if (reset_ui_to_zero) {
        resetUiAfterSetZero();
    }

    disableSineMode();

    if (manager_) {
        QMetaObject::invokeMethod(manager_.data(),
                                  "clearMotionQueue",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    }
}

void AxisWorkspace::stopSineModeForDisable() {
    clearMotionBuffersForServiceCommand(false);
    desired_target_deg_.store(commanded_target_deg_.load(std::memory_order_relaxed),
                              std::memory_order_relaxed);
}

void AxisWorkspace::applyManualMotionControls(const bool enabled) {
    manual_motion_controls_enabled_ = enabled;
    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    if (h.btn_enable) h.btn_enable->setEnabled(enabled);
    if (h.btn_disable) h.btn_disable->setEnabled(enabled);
    if (h.btn_clear_err) h.btn_clear_err->setEnabled(enabled);
    if (h.btn_set_zero) h.btn_set_zero->setEnabled(enabled);
    if (h.btn_home) h.btn_home->setEnabled(enabled);
    if (h.btn_move) h.btn_move->setEnabled(enabled);
    if (h.btn_jog_neg) h.btn_jog_neg->setEnabled(enabled);
    if (h.btn_jog_pos) h.btn_jog_pos->setEnabled(enabled);
    if (h.target_pos_spin) h.target_pos_spin->setEnabled(enabled);
    if (h.target_slider) h.target_slider->setEnabled(enabled);
    if (h.speed_spin) h.speed_spin->setEnabled(enabled);
    if (h.accel_spin) h.accel_spin->setEnabled(enabled);
    if (h.mode_combo) h.mode_combo->setEnabled(enabled);
    if (h.radio_move_abs) h.radio_move_abs->setEnabled(enabled);
    if (h.radio_move_rel) h.radio_move_rel->setEnabled(enabled);
    if (h.chk_sine_enable) h.chk_sine_enable->setEnabled(enabled);
    if (h.spin_sine_amp) h.spin_sine_amp->setEnabled(enabled);
    if (h.spin_sine_freq) h.spin_sine_freq->setEnabled(enabled);
    // Keep E-Stop available in all modes.
    if (h.btn_estop) h.btn_estop->setEnabled(true);
}

void AxisWorkspace::onHostStateUpdated(const QVariantMap& state) {
    const QString control_source = state.value(QStringLiteral("control_source")).toString();
    const bool estop_active = state.value(QStringLiteral("estop_active")).toBool();

    const bool manual_mode = (control_source == QStringLiteral("ui"));
    applyManualMotionControls(manual_mode && !estop_active);

    // Hard stop for UI sine producer when UI loses control source or global E-Stop is active.
    // This prevents further enqueue attempts from AxisWorkspace while HexaMotion owns motion,
    // and immediately cuts local trajectory generation on E-Stop.
    if (!manual_mode || estop_active) {
        disableSineMode();
    }

    // Bug 6 fix: disable config panel mutating buttons when HexaMotion owns or E-Stop is active.
    // Read-only operations (refresh list, read params, export) stay enabled.
    const bool config_write_allowed = manual_mode && !estop_active;
    if (config_panel_) {
        auto& cfg = config_panel_->handles();
        if (cfg.btn_apply_params) cfg.btn_apply_params->setEnabled(config_write_allowed);
        if (cfg.btn_save_drive_flash) cfg.btn_save_drive_flash->setEnabled(config_write_allowed);
        if (cfg.btn_import_full) cfg.btn_import_full->setEnabled(config_write_allowed);
    }
}

void AxisWorkspace::onTelemetryUpdated(int axis_id, const QVariantMap& telemetry) {
    if (axis_id != axis_id_ || !control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (telemetry_t0_ms_ == 0) {
        telemetry_t0_ms_ = now_ms;
    }
    const auto telemetry_now_ns = telemetry.value(QStringLiteral("timestamp_ns")).toULongLong();
    if (telemetry_t0_ns_ == 0U && telemetry_now_ns != 0U) {
        telemetry_t0_ns_ = telemetry_now_ns;
    }
    
    // Use hardware timestamp for plotting if available, fallback to OS time otherwise
    double t_sec = static_cast<double>(now_ms - telemetry_t0_ms_) / 1000.0;
    if (telemetry_now_ns != 0U && telemetry_t0_ns_ != 0U) {
        t_sec = static_cast<double>(telemetry_now_ns - telemetry_t0_ns_) / 1e9;
    }
    
    const QString transport = telemetry.value(QStringLiteral("transport")).toString();

    const int state_code = telemetry.value(QStringLiteral("state")).toInt();
    const int status_word = telemetry.value(QStringLiteral("status_word")).toInt();
    const int motion_status = telemetry.value(QStringLiteral("motion_status")).toInt();
    const int protection_code = telemetry.value(QStringLiteral("protection_code")).toInt();
    const int error_code = telemetry.value(QStringLiteral("error_code")).toInt();

    if (h.lbl_sys_state) {
        h.lbl_sys_state->setText(decodeSystemState(state_code));
    }
    if (h.lbl_state) {
        h.lbl_state->setText(decodeMotorStatusText(transport, state_code, motion_status, status_word));
    }
    if (h.lbl_protection) {
        h.lbl_protection->setText(decodeProtectionText(transport, protection_code));
    }
    if (h.lbl_error_code) {
        h.lbl_error_code->setText(decodeErrorCodeText(transport, error_code));
    }
    if (h.lbl_digital_inputs) {
        if (transport.compare(QStringLiteral("ethercat"), Qt::CaseInsensitive) == 0
            && telemetry.contains(QStringLiteral("digital_inputs"))) {
            const auto di_mask = static_cast<std::uint32_t>(
                telemetry.value(QStringLiteral("digital_inputs")).toULongLong());
            h.lbl_digital_inputs->setText(format_digital_inputs_first_8(di_mask));
        } else {
            h.lbl_digital_inputs->setText(QStringLiteral("N/A"));
        }
    }

    if (telemetry.contains(QStringLiteral("actual_position_deg"))) {
        const double actual_deg = telemetry.value(QStringLiteral("actual_position_deg")).toDouble();
        displayed_actual_deg_ = actual_deg;
        if (h.lbl_axis) {
            h.lbl_axis->setText(QString::number(actual_deg, 'f', 2) + QStringLiteral(" °"));
        }
        if (!target_seeded_from_telemetry_) {
            setTargetPosition(actual_deg);
            target_seeded_from_telemetry_ = true;
        }
    }

    const bool has_target = telemetry.contains(QStringLiteral("target_position_deg"));
    if (has_target) {
        const double target_deg = telemetry.value(QStringLiteral("target_position_deg")).toDouble();
        if (h.lbl_target) {
            h.lbl_target->setText(QString::number(target_deg, 'f', 2) + QStringLiteral(" °"));
        }
        const bool manual_hold_active = now_ms < manual_target_hold_until_ms_;
        if (h.target_pos_spin && !h.target_pos_spin->hasFocus() && !manual_hold_active) {
            h.target_pos_spin->blockSignals(true);
            h.target_pos_spin->setValue(target_deg);
            h.target_pos_spin->blockSignals(false);
        }
        if (h.target_slider && !manual_hold_active) {
            h.target_slider->blockSignals(true);
            h.target_slider->setValue(static_cast<int>(std::llround(target_deg * 100.0)));
            h.target_slider->blockSignals(false);
        }
    } else if (h.lbl_target && h.target_pos_spin) {
        h.lbl_target->setText(QString::number(h.target_pos_spin->value(), 'f', 2) + QStringLiteral(" °"));
    }

    if (telemetry.contains(QStringLiteral("actual_velocity_deg_per_sec")) && h.lbl_speed) {
        h.lbl_speed->setText(QString::number(telemetry.value(QStringLiteral("actual_velocity_deg_per_sec")).toDouble(), 'f', 1) + QStringLiteral(" °/s"));
    }
    if (telemetry.contains(QStringLiteral("actual_torque_percent")) && h.lbl_torque) {
        h.lbl_torque->setText(QString::number(telemetry.value(QStringLiteral("actual_torque_percent")).toDouble(), 'f', 1) + QStringLiteral(" %"));
    }

    auto format_rate_period = [](double hz, double period_ms) {
        return QStringLiteral("%1 Hz (%2 ms)").arg(hz, 0, 'f', 1).arg(period_ms, 0, 'f', 1);
    };

    if (h.lbl_cmd_tx_rate) {
        if (telemetry.contains(QStringLiteral("cmd_tx_hz")) && telemetry.contains(QStringLiteral("cmd_tx_period_ms"))) {
            h.lbl_cmd_tx_rate->setText(format_rate_period(telemetry.value(QStringLiteral("cmd_tx_hz")).toDouble(),
                                                          telemetry.value(QStringLiteral("cmd_tx_period_ms")).toDouble()));
        } else {
            h.lbl_cmd_tx_rate->setText(QStringLiteral("N/A"));
        }
    }
    if (h.lbl_telemetry_rate) {
        if (telemetry.contains(QStringLiteral("telemetry_publish_hz")) && telemetry.contains(QStringLiteral("telemetry_publish_period_ms"))) {
            h.lbl_telemetry_rate->setText(format_rate_period(telemetry.value(QStringLiteral("telemetry_publish_hz")).toDouble(),
                                                             telemetry.value(QStringLiteral("telemetry_publish_period_ms")).toDouble()));
        } else {
            h.lbl_telemetry_rate->setText(QStringLiteral("N/A"));
        }
    }
    if (h.lbl_position_rx_rate) {
        if (telemetry.contains(QStringLiteral("position_rx_hz")) && telemetry.contains(QStringLiteral("position_rx_period_ms"))) {
            h.lbl_position_rx_rate->setText(format_rate_period(telemetry.value(QStringLiteral("position_rx_hz")).toDouble(),
                                                               telemetry.value(QStringLiteral("position_rx_period_ms")).toDouble()));
        } else {
            h.lbl_position_rx_rate->setText(QStringLiteral("N/A"));
        }
    }
    if (h.lbl_speed_rx_rate) {
        if (telemetry.contains(QStringLiteral("speed_rx_hz"))) {
            h.lbl_speed_rx_rate->setText(QStringLiteral("%1 Hz").arg(telemetry.value(QStringLiteral("speed_rx_hz")).toDouble(), 0, 'f', 1));
        } else {
            h.lbl_speed_rx_rate->setText(QStringLiteral("N/A"));
        }
    }
    if (h.lbl_status_rx_rate) {
        if (telemetry.contains(QStringLiteral("status_rx_hz"))) {
            h.lbl_status_rx_rate->setText(QStringLiteral("%1 Hz").arg(telemetry.value(QStringLiteral("status_rx_hz")).toDouble(), 0, 'f', 1));
        } else {
            h.lbl_status_rx_rate->setText(QStringLiteral("N/A"));
        }
    }
    if (h.lbl_protection_rx_rate) {
        if (telemetry.contains(QStringLiteral("protection_rx_hz"))) {
            h.lbl_protection_rx_rate->setText(QStringLiteral("%1 Hz").arg(telemetry.value(QStringLiteral("protection_rx_hz")).toDouble(), 0, 'f', 1));
        } else {
            h.lbl_protection_rx_rate->setText(QStringLiteral("N/A"));
        }
    }

    if (h.scope && h.cmb_scope_signal) {
        const QString signal = h.cmb_scope_signal->currentText();
        const bool show_position = signal.startsWith(QStringLiteral("Position"));
        const bool show_velocity = signal.startsWith(QStringLiteral("Velocity"));
        const bool show_torque = signal.startsWith(QStringLiteral("Torque"));

        h.scope->setChannelVisibility(QStringLiteral("actual"), show_position && h.chk_plot_actual_pos && h.chk_plot_actual_pos->isChecked());
        h.scope->setChannelVisibility(QStringLiteral("target"), show_position && h.chk_plot_target_pos && h.chk_plot_target_pos->isChecked());
        h.scope->setChannelVisibility(QStringLiteral("pos_error"), show_position && h.chk_plot_pos_error && h.chk_plot_pos_error->isChecked());
        h.scope->setChannelVisibility(QStringLiteral("speed"), show_velocity && h.chk_plot_actual_vel && h.chk_plot_actual_vel->isChecked());
        h.scope->setChannelVisibility(QStringLiteral("target_speed"), show_velocity && h.chk_plot_target_vel && h.chk_plot_target_vel->isChecked());
        h.scope->setChannelVisibility(QStringLiteral("torque"), show_torque && h.chk_plot_actual_vel && h.chk_plot_actual_vel->isChecked());

        // Skip single-point actual plotting when the transport provides batched position_samples
        // (e.g., MKS). The transport override will plot the high-resolution batch instead.
        if (show_position && h.chk_plot_actual_pos && h.chk_plot_actual_pos->isChecked()
            && telemetry.contains(QStringLiteral("actual_position_deg"))
            && !telemetry.contains(QStringLiteral("position_samples"))) {
            h.scope->addData(QStringLiteral("actual"), t_sec, telemetry.value(QStringLiteral("actual_position_deg")).toDouble());
        }
        if (show_position
            && h.chk_plot_target_pos
            && h.chk_plot_target_pos->isChecked()
            && !transportProvidesTargetTrace()) {
            h.scope->addData(QStringLiteral("target"),
                             t_sec,
                             has_target ? telemetry.value(QStringLiteral("target_position_deg")).toDouble()
                                         : desired_target_deg_.load(std::memory_order_relaxed));
        }
        if (show_position
            && h.chk_plot_pos_error
            && h.chk_plot_pos_error->isChecked()
            && telemetry.contains(QStringLiteral("actual_position_deg"))) {
            const double target = has_target ? telemetry.value(QStringLiteral("target_position_deg")).toDouble()
                                             : desired_target_deg_.load(std::memory_order_relaxed);
            const double actual = telemetry.value(QStringLiteral("actual_position_deg")).toDouble();
            h.scope->addData(QStringLiteral("pos_error"), t_sec, target - actual);
        }
        if (show_velocity && h.chk_plot_actual_vel && h.chk_plot_actual_vel->isChecked() && telemetry.contains(QStringLiteral("actual_velocity_deg_per_sec"))) {
            h.scope->addData(QStringLiteral("speed"), t_sec, telemetry.value(QStringLiteral("actual_velocity_deg_per_sec")).toDouble());
        }
        if (show_torque && h.chk_plot_actual_vel && h.chk_plot_actual_vel->isChecked() && telemetry.contains(QStringLiteral("actual_torque_percent"))) {
            h.scope->addData(QStringLiteral("torque"), t_sec, telemetry.value(QStringLiteral("actual_torque_percent")).toDouble());
        }
    }

    onTransportTelemetryUpdated(telemetry, transport, t_sec);
}

void AxisWorkspace::refreshParameterList() {
    if (!config_panel_) {
        return;
    }
    if (config_panel_->handles().config_tree) {
        config_panel_->handles().config_tree->clear();
    }
    if (config_panel_->handles().txt_description) {
        config_panel_->handles().txt_description->clear();
    }
    if (manager_) {
        QMetaObject::invokeMethod(manager_.data(), "requestListParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
    }
}

void AxisWorkspace::readParametersFromDrive() {
    if (!manager_ || parameter_read_in_progress_) {
        return;
    }
    setParameterReadInProgress(true);
    QMetaObject::invokeMethod(manager_.data(), "requestReadParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
}

void AxisWorkspace::applyParametersPatch() {
    if (!manager_ || !config_panel_ || !config_panel_->handles().config_tree) {
        return;
    }

    auto parse_bool_token = [](const QString& token, bool* ok_out) -> bool {
        const QString normalized = token.trimmed().toLower();
        if (normalized == QStringLiteral("1") || normalized == QStringLiteral("true")
            || normalized == QStringLiteral("yes") || normalized == QStringLiteral("on")) {
            if (ok_out) {
                *ok_out = true;
            }
            return true;
        }
        if (normalized == QStringLiteral("0") || normalized == QStringLiteral("false")
            || normalized == QStringLiteral("no") || normalized == QStringLiteral("off")) {
            if (ok_out) {
                *ok_out = true;
            }
            return false;
        }
        if (ok_out) {
            *ok_out = false;
        }
        return false;
    };

    auto variant_to_double = [](const QVariant& value, bool* ok_out) -> double {
        if (value.userType() == QMetaType::Bool) {
            if (ok_out) {
                *ok_out = true;
            }
            return value.toBool() ? 1.0 : 0.0;
        }
        bool ok = false;
        const double out = value.toDouble(&ok);
        if (ok_out) {
            *ok_out = ok;
        }
        return out;
    };

    QVariantList patch;
    QList<QTreeWidgetItem*> patched_items;
    QStringList validation_errors;
    bool can_id_change_requested = false;
    bool can_bitrate_change_requested = false;

    for (auto* item : parameterItems(config_panel_->handles().config_tree)) {
        const QString new_val = item->text(kNewValueColumn).trimmed();
        if (new_val.isEmpty() || new_val == QStringLiteral("N/A")) {
            continue;
        }

        const bool has_min = item->data(kUnitColumn, Qt::UserRole).toBool();
        const bool has_max = item->data(kReadOnlyColumn, Qt::UserRole).toBool();
        const QVariant min_value = item->data(kCurrentValueColumn, Qt::UserRole);
        const QVariant max_value = item->data(kNewValueColumn, Qt::UserRole);
        const bool bool_like_bounds = has_min && has_max
            && min_value.userType() == QMetaType::Bool
            && max_value.userType() == QMetaType::Bool;

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
                if (int_ok && !new_val.contains('.')) {
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
            validation_errors.push_back(QStringLiteral("%1: invalid value '%2'")
                                            .arg(item->text(kNameColumn), new_val));
            continue;
        }

        if (has_min || has_max) {
            bool numeric_ok = false;
            const double numeric = variant_to_double(parsed_value, &numeric_ok);
            if (!numeric_ok) {
                validation_errors.push_back(QStringLiteral("%1: value must be numeric/boolean")
                                                 .arg(item->text(kNameColumn)));
                continue;
            }
            if (has_min) {
                bool min_ok = false;
                const double min_numeric = variant_to_double(min_value, &min_ok);
                if (min_ok && numeric < min_numeric) {
                    validation_errors.push_back(QStringLiteral("%1: value %2 is below min %3")
                                                    .arg(item->text(kNameColumn),
                                                         new_val,
                                                         QString::number(min_numeric, 'g', 12)));
                    continue;
                }
            }
            if (has_max) {
                bool max_ok = false;
                const double max_numeric = variant_to_double(max_value, &max_ok);
                if (max_ok && numeric > max_numeric) {
                    validation_errors.push_back(QStringLiteral("%1: value %2 is above max %3")
                                                    .arg(item->text(kNameColumn),
                                                         new_val,
                                                         QString::number(max_numeric, 'g', 12)));
                    continue;
                }
            }
        }

        QVariantMap map;
        map.insert(QStringLiteral("domain"), item->data(kNameColumn, Qt::UserRole));
        map.insert(QStringLiteral("value"), item->data(kGroupColumn, Qt::UserRole));
        map.insert(QStringLiteral("data"), parsed_value);
        patch.push_back(map);
        patched_items.push_back(item);

        const int domain_value = map.value(QStringLiteral("domain")).toInt();
        const int parameter_value = map.value(QStringLiteral("value")).toInt();
        if (domain_value == static_cast<int>(motion_core::ParameterDomain::Mks)
            && parameter_value == static_cast<int>(mks::MksParameter::CanId)) {
            can_id_change_requested = true;
        }
        if (domain_value == static_cast<int>(motion_core::ParameterDomain::Mks)
            && parameter_value == static_cast<int>(mks::MksParameter::CanBitrateIndex)) {
            can_bitrate_change_requested = true;
        }
    }

    if (!validation_errors.isEmpty()) {
        QMessageBox::warning(this,
                             QStringLiteral("Invalid parameter values"),
                             QStringLiteral("Please fix invalid values before apply:\n- ")
                                   + validation_errors.join(QStringLiteral("\n- ")));
        return;
    }

    if (patch.isEmpty()) {
        return;
    }

    if (can_id_change_requested) {
        const auto answer = QMessageBox::warning(
            this,
            QStringLiteral("CAN ID change"),
            QStringLiteral("Applying CAN ID change switches the drive address immediately. Continue?"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (answer != QMessageBox::Yes) {
            return;
        }
    }
    if (can_bitrate_change_requested) {
        const auto answer = QMessageBox::warning(
            this,
            QStringLiteral("CAN bitrate change"),
            QStringLiteral("Applying CAN bitrate change requires manual reconnect at the new bitrate. Continue?"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (answer != QMessageBox::Yes) {
            return;
        }
    }

    auto* manager = manager_.data();
    QMetaObject::invokeMethod(manager,
                              [manager, id = axis_id_, patch]() { manager->applyParameterPatch(id, patch); },
                              Qt::QueuedConnection);

    for (auto* item : patched_items) {
        item->setText(kNewValueColumn, QString());
    }

    readParametersFromDrive();
}

void AxisWorkspace::saveSelectedParameterToDriveFlash() {
    if (!manager_ || !config_panel_ || !config_panel_->handles().config_tree) {
        return;
    }

    auto* selected = config_panel_->handles().config_tree->currentItem();
    if (!selected) {
        QMessageBox::warning(this, QStringLiteral("Persistent write"), QStringLiteral("Select a parameter row first."));
        return;
    }

    const int domain = selected->data(kNameColumn, Qt::UserRole).toInt();
    const int value_id = selected->data(kGroupColumn, Qt::UserRole).toInt();
    const bool is_read_only = selected->text(kReadOnlyColumn).compare(QStringLiteral("Yes"), Qt::CaseInsensitive) == 0;
    const bool is_persistable = selected->data(kNameColumn, Qt::UserRole + 1).toBool();
    if (is_read_only) {
        QMessageBox::information(this, QStringLiteral("Persistent write"), QStringLiteral("Selected parameter is read-only."));
        return;
    }

    const QString pending_value = selected->text(kNewValueColumn).trimmed();
    const QString current_value = selected->text(kCurrentValueColumn).trimmed();
    const QString value_to_use = (!pending_value.isEmpty() && pending_value != QStringLiteral("N/A")) ? pending_value : current_value;
    if (value_to_use.isEmpty() || value_to_use == QStringLiteral("Unknown") || value_to_use == QStringLiteral("N/A")) {
        QMessageBox::warning(this, QStringLiteral("Persistent write"), QStringLiteral("Selected row has no valid value to persist."));
        return;
    }

    const bool supported = (domain == static_cast<int>(motion_core::ParameterDomain::Mks)
                            && (value_id == static_cast<int>(mks::MksParameter::CanId)
                                || value_id == static_cast<int>(mks::MksParameter::CanBitrateIndex)))
        || (domain == static_cast<int>(motion_core::ParameterDomain::Common)
            && value_id == static_cast<int>(motion_core::CommonParameter::MotorSelectionCode));
    if (!is_persistable && !supported) {
        QMessageBox::information(this,
                                 QStringLiteral("Persistent write"),
                                 QStringLiteral("Selected parameter is marked as non-persistable."));
        return;
    }
    if (!supported) {
        QMessageBox::information(this,
                                 QStringLiteral("Persistent write"),
                                 QStringLiteral("For now persistent write is supported for CAN ID, CAN bitrate and motor selection code."));
        return;
    }

    QVariant parsed_value;
    bool bool_ok = false;
    const QString lower = value_to_use.trimmed().toLower();
    if (lower == QStringLiteral("true") || lower == QStringLiteral("false")
        || lower == QStringLiteral("1") || lower == QStringLiteral("0")) {
        parsed_value = (lower == QStringLiteral("true") || lower == QStringLiteral("1"));
        bool_ok = true;
    }
    if (!bool_ok) {
        bool int_ok = false;
        const qlonglong int_value = value_to_use.toLongLong(&int_ok);
        if (int_ok && !value_to_use.contains('.')) {
            parsed_value = int_value;
        } else {
            bool double_ok = false;
            const double double_value = value_to_use.toDouble(&double_ok);
            if (double_ok) {
                parsed_value = double_value;
            } else {
                QMessageBox::warning(this,
                                     QStringLiteral("Persistent write"),
                                     QStringLiteral("Selected persistent value must be numeric or boolean."));
                return;
            }
        }
    }

    const auto answer = QMessageBox::warning(
        this,
        QStringLiteral("Persistent write"),
        QStringLiteral("Write %1=%2 and store it persistently?")
            .arg(selected->text(kNameColumn), value_to_use),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes) {
        return;
    }

    auto* manager = manager_.data();
    QMetaObject::invokeMethod(manager,
                              [manager,
                               axis_id = axis_id_,
                               domain,
                               value_id,
                               name = selected->text(kNameColumn),
                               parsed_value]() {
                                  manager->setPersistentParameter(axis_id, domain, value_id, name, parsed_value);
                              },
                              Qt::QueuedConnection);
}

void AxisWorkspace::exportAxisConfig() {
    if (!manager_) {
        return;
    }

    const QString path = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("Export AxisConfig"),
        QStringLiteral("axis_%1.axis_config.json").arg(axis_id_),
        QStringLiteral("JSON Files (*.json)"));
    if (path.isEmpty()) {
        return;
    }

    QMetaObject::invokeMethod(manager_.data(), "exportAxisConfig", Qt::QueuedConnection, Q_ARG(int, axis_id_), Q_ARG(QString, path));
}

void AxisWorkspace::importAxisConfig() {
    if (!manager_) {
        return;
    }

    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Import AxisConfig"),
        QString(),
        QStringLiteral("JSON Files (*.json)"));
    if (path.isEmpty()) {
        return;
    }

    QMetaObject::invokeMethod(manager_.data(), "importAxisConfigPreview", Qt::QueuedConnection, Q_ARG(int, axis_id_), Q_ARG(QString, path));
}

void AxisWorkspace::onAxisConfigPreviewReady(int axis_id, const QVariantList& patch_entries) {
    if (axis_id != axis_id_ || !config_panel_ || !config_panel_->handles().config_tree) {
        return;
    }

    for (auto* item : parameterItems(config_panel_->handles().config_tree)) {
        if (item->text(kReadOnlyColumn).compare(QStringLiteral("Yes"), Qt::CaseInsensitive) != 0) {
            item->setText(kNewValueColumn, QString());
        }
    }

    for (const auto& entry_variant : patch_entries) {
        const QVariantMap map = entry_variant.toMap();
        const int domain = map.value(QStringLiteral("domain")).toInt();
        const int value_id = map.value(QStringLiteral("value")).toInt();
        const QString data = map.value(QStringLiteral("data")).toString();

        for (auto* item : parameterItems(config_panel_->handles().config_tree)) {
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain
                && item->data(kGroupColumn, Qt::UserRole).toInt() == value_id
                && item->text(kReadOnlyColumn).compare(QStringLiteral("Yes"), Qt::CaseInsensitive) != 0) {
                item->setText(kNewValueColumn, data);
                break;
            }
        }
    }
}

void AxisWorkspace::onParameterListReady(int axis_id, const QVariantList& params) {
    if (axis_id != axis_id_ || !config_panel_ || !config_panel_->handles().config_tree) {
        return;
    }

    auto* tree = config_panel_->handles().config_tree;
    auto* desc = config_panel_->handles().txt_description;
    tree->clear();
    if (desc) {
        desc->clear();
    }

    QHash<QString, QTreeWidgetItem*> group_roots;
    for (const auto& param_variant : params) {
        const QVariantMap map = param_variant.toMap();
        const QString raw_group = map.value(QStringLiteral("group")).toString().trimmed();
        const QString group_name = raw_group.isEmpty() ? QStringLiteral("Ungrouped") : raw_group;
        QTreeWidgetItem* group_item = group_roots.value(group_name, nullptr);
        if (!group_item) {
            group_item = new QTreeWidgetItem(tree);
            group_item->setText(kNameColumn, group_name);
            group_item->setFirstColumnSpanned(true);
            group_item->setExpanded(true);
            group_roots.insert(group_name, group_item);
        }

        auto* item = new QTreeWidgetItem(group_item);
        item->setText(kNameColumn, map.value(QStringLiteral("name")).toString());
        item->setText(kGroupColumn, group_name);
        item->setText(kUnitColumn, map.value(QStringLiteral("unit")).toString());
        item->setText(kReadOnlyColumn, map.value(QStringLiteral("read_only")).toBool() ? QStringLiteral("Yes") : QStringLiteral("No"));
        item->setText(kCurrentValueColumn, QStringLiteral("Unknown"));
        item->setText(kNewValueColumn, map.value(QStringLiteral("read_only")).toBool() ? QStringLiteral("N/A") : QString());

        item->setData(kNameColumn, Qt::UserRole, map.value(QStringLiteral("domain")));
        item->setData(kGroupColumn, Qt::UserRole, map.value(QStringLiteral("value")));
        item->setData(kUnitColumn, Qt::UserRole, map.value(QStringLiteral("has_min")));
        item->setData(kReadOnlyColumn, Qt::UserRole, map.value(QStringLiteral("has_max")));
        item->setData(kCurrentValueColumn, Qt::UserRole, map.value(QStringLiteral("min_value")));
        item->setData(kNewValueColumn, Qt::UserRole, map.value(QStringLiteral("max_value")));
        item->setData(kNameColumn, Qt::UserRole + 1, map.value(QStringLiteral("persistable"), true));

        if (!map.value(QStringLiteral("read_only")).toBool()) {
            item->setFlags(item->flags() | Qt::ItemIsEditable);
        } else {
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        }
    }

    tree->expandAll();
}

void AxisWorkspace::onParametersRead(int axis_id, const QVariantList& params) {
    if (axis_id != axis_id_ || !config_panel_ || !config_panel_->handles().config_tree) {
        return;
    }

    setParameterReadInProgress(false);

    for (const auto& value_variant : params) {
        const QVariantMap map = value_variant.toMap();
        const int domain = map.value(QStringLiteral("domain")).toInt();
        const int value_id = map.value(QStringLiteral("value")).toInt();
        const QString data = map.value(QStringLiteral("data")).toString();

        for (auto* item : parameterItems(config_panel_->handles().config_tree)) {
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain
                && item->data(kGroupColumn, Qt::UserRole).toInt() == value_id) {
                item->setText(kCurrentValueColumn, data);
                break;
            }
        }
    }
}

void AxisWorkspace::onSineToggled(bool enabled) {
    if (control_panel_) {
        auto& h = control_panel_->handles();
        if (h.target_pos_spin) {
            h.target_pos_spin->setEnabled(!enabled);
        }
        if (h.target_slider) {
            h.target_slider->setEnabled(!enabled);
        }
    }
    onTransportSineToggled(enabled);
}

void AxisWorkspace::onMotionQueueStatsUpdated(int axis_id, const QVariantMap& stats) {
    if (axis_id != axis_id_ || !control_panel_ || !control_panel_->handles().lbl_motion_queue_stats) {
        return;
    }

    const auto size = stats.value(QStringLiteral("size")).toULongLong();
    const auto capacity = stats.value(QStringLiteral("capacity")).toULongLong();
    const auto pushed = stats.value(QStringLiteral("pushed")).toULongLong();
    const auto dropped = stats.value(QStringLiteral("dropped")).toULongLong();
    const auto underruns = stats.value(QStringLiteral("underruns")).toULongLong();
    const auto short_starts = stats.value(QStringLiteral("short_starts")).toULongLong();

    auto* label = control_panel_->handles().lbl_motion_queue_stats;
    label->setText(QStringLiteral("queue: size=%1 / %2, pushed=%3, dropped=%4, underruns=%5, short_starts=%6")
                       .arg(size)
                       .arg(capacity)
                       .arg(pushed)
                       .arg(dropped)
                       .arg(underruns)
                       .arg(short_starts));
    if (size == 0U) {
        label->setStyleSheet(QStringLiteral("color: #da3633; font-weight: bold;"));
    } else if (size < 30U) {
        label->setStyleSheet(QStringLiteral("color: #d29922; font-weight: bold;"));
    } else {
        label->setStyleSheet({});
    }

    onTransportMotionQueueStatsUpdated(stats);
}

void AxisWorkspace::setupUi() {
    auto* root = new QVBoxLayout(this);
    tabs_ = new QTabWidget(this);
    root->addWidget(tabs_);

    control_panel_ = new AxisWorkspaceControlPanel(this);
    tabs_->addTab(control_panel_, QStringLiteral("Control"));

    config_panel_ = new AxisWorkspaceConfigPanel(this);
    tabs_->addTab(config_panel_, QStringLiteral("Configuration"));

    sequencer_ = new mks::SequencerWidget(this);
    tabs_->addTab(sequencer_, QStringLiteral("Sequencer (Trapezoidal)"));

    auto& h = control_panel_->handles();
    connect(h.btn_enable, &QPushButton::clicked, this, [this]() {
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "enableMotor", Qt::QueuedConnection, Q_ARG(int, axis_id_), Q_ARG(bool, true));
        }
    });
    connect(h.btn_disable, &QPushButton::clicked, this, [this]() {
        onBeforeDisableAxis();
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "enableMotor", Qt::QueuedConnection, Q_ARG(int, axis_id_), Q_ARG(bool, false));
        }
    });
    connect(h.btn_estop, &QPushButton::clicked, this, [this]() {
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "emergencyStop", Qt::QueuedConnection, Q_ARG(int, axis_id_));
        }
    });
    connect(h.btn_clear_err, &QPushButton::clicked, this, [this]() {
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "clearErrors", Qt::QueuedConnection, Q_ARG(int, axis_id_));
        }
    });
    connect(h.btn_home, &QPushButton::clicked, this, [this]() {
        onBeforeHomeAxis();
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "goHome", Qt::QueuedConnection, Q_ARG(int, axis_id_));
        }
    });
    connect(h.btn_set_zero, &QPushButton::clicked, this, [this]() {
        onBeforeSetZeroAxis();
        resetUiAfterSetZero();
        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "setZeroPosition", Qt::QueuedConnection, Q_ARG(int, axis_id_));
        }
    });
    connect(h.btn_move, &QPushButton::clicked, this, &AxisWorkspace::triggerAbsoluteMove);
    connect(h.chk_sine_enable, &QCheckBox::toggled, this, &AxisWorkspace::onSineToggled);
    connect(h.mode_combo,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int) { applyCurrentModeSelection(); });
    connect(h.speed_spin,
            qOverload<int>(&QSpinBox::valueChanged),
            this,
            [this](int value) { current_speed_.store(value, std::memory_order_relaxed); });
    connect(h.accel_spin,
            qOverload<int>(&QSpinBox::valueChanged),
            this,
            [this](int value) { current_accel_.store(value, std::memory_order_relaxed); });

    connect(h.target_slider, &QSlider::valueChanged, this, [this](int raw) {
        if (!control_panel_ || transportOwnsTargetUi()) {
            return;
        }
        auto& handles = control_panel_->handles();
        if (!handles.target_pos_spin || !manager_) {
            return;
        }

        const double value = static_cast<double>(raw) / 100.0;
        handles.target_pos_spin->blockSignals(true);
        handles.target_pos_spin->setValue(value);
        handles.target_pos_spin->blockSignals(false);
        desired_target_deg_.store(value, std::memory_order_relaxed);
        commanded_target_deg_.store(value, std::memory_order_relaxed);
        manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;
        QMetaObject::invokeMethod(manager_.data(),
                                  "moveAbsoluteAxis",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, handles.speed_spin ? handles.speed_spin->value() : kDefaultSpeedRpm),
                                  Q_ARG(int, handles.accel_spin ? handles.accel_spin->value() : kDefaultAccelPercent),
                                  Q_ARG(double, value));
    });

    connect(h.target_pos_spin,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            [this](double value) {
                if (!control_panel_) {
                    return;
                }
                desired_target_deg_.store(value, std::memory_order_relaxed);
                commanded_target_deg_.store(value, std::memory_order_relaxed);
                manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1500;
                if (control_panel_->handles().target_slider) {
                    control_panel_->handles().target_slider->blockSignals(true);
                    control_panel_->handles().target_slider->setValue(static_cast<int>(std::llround(value * 100.0)));
                    control_panel_->handles().target_slider->blockSignals(false);
                }
            });

    connect(h.btn_jog_neg, &QPushButton::clicked, this, [this]() {
        if (!manager_ || !control_panel_) {
            return;
        }
        auto& handles = control_panel_->handles();
        const double step = handles.jog_step_spin ? handles.jog_step_spin->value() : 1.0;
        QMetaObject::invokeMethod(manager_.data(),
                                  "moveRelativeAxis",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, handles.speed_spin ? handles.speed_spin->value() : kDefaultSpeedRpm),
                                  Q_ARG(int, handles.accel_spin ? handles.accel_spin->value() : kDefaultAccelPercent),
                                  Q_ARG(double, -step));
    });
    connect(h.btn_jog_pos, &QPushButton::clicked, this, [this]() {
        if (!manager_ || !control_panel_) {
            return;
        }
        auto& handles = control_panel_->handles();
        const double step = handles.jog_step_spin ? handles.jog_step_spin->value() : 1.0;
        QMetaObject::invokeMethod(manager_.data(),
                                  "moveRelativeAxis",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, handles.speed_spin ? handles.speed_spin->value() : kDefaultSpeedRpm),
                                  Q_ARG(int, handles.accel_spin ? handles.accel_spin->value() : kDefaultAccelPercent),
                                  Q_ARG(double, step));
    });

    connect(h.chk_plot_actual_pos, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("actual"), checked);
        }
    });
    connect(h.chk_plot_target_pos, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("target"), checked);
        }
    });
    connect(h.chk_plot_actual_vel, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("speed"), checked);
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("torque"), checked);
        }
    });
    connect(h.chk_plot_target_vel, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("target_speed"), checked);
        }
    });
    connect(h.chk_plot_pos_error, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setChannelVisibility(QStringLiteral("pos_error"), checked);
        }
    });
    connect(h.chk_auto_scale, &QCheckBox::toggled, this, [this](bool checked) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->setAutoRange(checked);
        }
    });
    connect(h.sld_scope_time, &QSlider::valueChanged, this, [this](int value) {
        if (!control_panel_ || !control_panel_->handles().scope) {
            return;
        }
        control_panel_->handles().scope->setXRange(static_cast<double>(value));
        if (control_panel_->handles().lbl_scope_time) {
            control_panel_->handles().lbl_scope_time->setText(QString::number(value) + QStringLiteral(" s"));
        }
    });
    connect(h.cmb_scope_signal, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (control_panel_ && control_panel_->handles().scope) {
            control_panel_->handles().scope->clear();
        }
    });

    auto& cfg = config_panel_->handles();
    connect(cfg.btn_refresh_list, &QPushButton::clicked, this, &AxisWorkspace::refreshParameterList);
    connect(cfg.btn_read_params, &QPushButton::clicked, this, &AxisWorkspace::readParametersFromDrive);
    connect(cfg.btn_apply_params, &QPushButton::clicked, this, &AxisWorkspace::applyParametersPatch);
    connect(cfg.btn_save_drive_flash, &QPushButton::clicked, this, &AxisWorkspace::saveSelectedParameterToDriveFlash);
    connect(cfg.btn_export_full, &QPushButton::clicked, this, &AxisWorkspace::exportAxisConfig);
    connect(cfg.btn_import_full, &QPushButton::clicked, this, &AxisWorkspace::importAxisConfig);
    connect(cfg.config_tree, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
        if (!config_panel_ || !config_panel_->handles().txt_description) {
            return;
        }
        auto* desc = config_panel_->handles().txt_description;
        if (!current) {
            desc->clear();
            return;
        }
        if (!current->data(kNameColumn, Qt::UserRole).isValid()) {
            desc->setHtml(QStringLiteral("<b>Category:</b> %1").arg(current->text(kNameColumn)));
            return;
        }

        QString html;
        html  = QStringLiteral("<b>Name:</b> ") + current->text(kNameColumn) + QStringLiteral("<br>");
        html += QStringLiteral("<b>Group:</b> ") + current->text(kGroupColumn) + QStringLiteral("<br>");
        if (!current->text(kUnitColumn).isEmpty()) {
            html += QStringLiteral("<b>Unit:</b> ") + current->text(kUnitColumn) + QStringLiteral("<br>");
        }
        const int domain = current->data(kNameColumn, Qt::UserRole).toInt();
        const int value_id = current->data(kGroupColumn, Qt::UserRole).toInt();
        html += QStringLiteral("<b>Domain:</b> ") + QString::number(domain) + QStringLiteral("<br>");
        html += QStringLiteral("<b>Value ID:</b> ") + QString::number(value_id) + QStringLiteral("<br>");
        html += QStringLiteral("<b>Current:</b> ") + current->text(kCurrentValueColumn) + QStringLiteral("<br>");
        html += QStringLiteral("<b>Pending:</b> ")
               + (current->text(kNewValueColumn).isEmpty() ? QStringLiteral("-") : current->text(kNewValueColumn))
               + QStringLiteral("<br>");

        const bool has_min = current->data(kUnitColumn, Qt::UserRole).toBool();
        const bool has_max = current->data(kReadOnlyColumn, Qt::UserRole).toBool();
        if (has_min || has_max) {
            html += QStringLiteral("<br><b>Bounds:</b><br>");
            if (has_min) {
                html += QStringLiteral("- Min: ") + valueToDisplayString(current->data(kCurrentValueColumn, Qt::UserRole)) + QStringLiteral("<br>");
            }
            if (has_max) {
                html += QStringLiteral("- Max: ") + valueToDisplayString(current->data(kNewValueColumn, Qt::UserRole)) + QStringLiteral("<br>");
            }
        }

        html += QStringLiteral("<br><b>Persistable (AxisConfig):</b> %1<br>")
                    .arg(current->data(kNameColumn, Qt::UserRole + 1).toBool() ? QStringLiteral("Yes") : QStringLiteral("No"));

        const QString protocol_details = protocolDetailsForParameter(domain, value_id);
        if (!protocol_details.isEmpty()) {
            html += QStringLiteral("<br><b>Protocol details:</b><br>") + protocol_details;
        }
        desc->setHtml(html);
    });

    scheduleWatchAxis(true);
}

void AxisWorkspace::scheduleWatchAxis(bool enabled) {
    if (manager_) {
        QMetaObject::invokeMethod(manager_.data(), "watchAxis", Qt::QueuedConnection, Q_ARG(int, axis_id_), Q_ARG(bool, enabled));
    }
}

void AxisWorkspace::setParameterReadInProgress(bool in_progress) {
    parameter_read_in_progress_ = in_progress;
    if (!config_panel_) {
        return;
    }

    auto& cfg = config_panel_->handles();
    if (cfg.btn_read_params) {
        cfg.btn_read_params->setEnabled(!in_progress);
        cfg.btn_read_params->setText(in_progress ? QStringLiteral("Reading...") : QStringLiteral("Read Values"));
    }
    if (cfg.btn_apply_params) {
        cfg.btn_apply_params->setEnabled(!in_progress);
    }
    if (cfg.btn_save_drive_flash) {
        cfg.btn_save_drive_flash->setEnabled(!in_progress);
    }
}

void AxisWorkspace::setTargetPosition(double pos_deg) {
    desired_target_deg_.store(pos_deg, std::memory_order_relaxed);
    commanded_target_deg_.store(pos_deg, std::memory_order_relaxed);

    if (!control_panel_ || transportOwnsTargetUi()) {
        return;
    }

    auto& h = control_panel_->handles();
    if (h.target_pos_spin) {
        h.target_pos_spin->blockSignals(true);
        h.target_pos_spin->setValue(pos_deg);
        h.target_pos_spin->blockSignals(false);
    }
    if (h.target_slider) {
        h.target_slider->blockSignals(true);
        h.target_slider->setValue(static_cast<int>(std::llround(pos_deg * 100.0)));
        h.target_slider->blockSignals(false);
    }
}

void AxisWorkspace::triggerAbsoluteMove() {
    if (!manager_ || !control_panel_ || transportOwnsTargetUi() || !manual_motion_controls_enabled_) {
        return;
    }

    auto& h = control_panel_->handles();
    const double target = h.target_pos_spin ? h.target_pos_spin->value() : 0.0;
    desired_target_deg_.store(target, std::memory_order_relaxed);
    commanded_target_deg_.store(target, std::memory_order_relaxed);
    manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;

    if (h.radio_move_rel && h.radio_move_rel->isChecked()) {
        QMetaObject::invokeMethod(manager_.data(),
                                  "moveRelativeAxis",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, h.speed_spin ? h.speed_spin->value() : kDefaultSpeedRpm),
                                  Q_ARG(int, h.accel_spin ? h.accel_spin->value() : kDefaultAccelPercent),
                                  Q_ARG(double, target));
        return;
    }

    QMetaObject::invokeMethod(manager_.data(),
                              "moveAbsoluteAxis",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id_),
                              Q_ARG(int, h.speed_spin ? h.speed_spin->value() : kDefaultSpeedRpm),
                              Q_ARG(int, h.accel_spin ? h.accel_spin->value() : kDefaultAccelPercent),
                              Q_ARG(double, target));
}

bool AxisWorkspace::isTargetReached(double tolerance_deg) const {
    return std::abs(desired_target_deg_.load(std::memory_order_relaxed) - displayed_actual_deg_) <= tolerance_deg;
}