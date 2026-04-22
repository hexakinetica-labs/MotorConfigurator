#include "mks/axis_workspace/ethercat_axis_workspace.h"

#include "mks/axis_workspace/axis_workspace_control_panel.h"
#include "mks/axis_manager.h"
#include "mks/ScopeWidget.h"

#include "motion_core/axis_data.h"
#include "motion_core/types.h"

#include <QCheckBox>
#include <QComboBox>

#include <QDoubleSpinBox>
#include <QMetaObject>
#include <QSlider>
#include <QSpinBox>

#include <QVariantList>
#include <QVector>

#include <algorithm>
#include <chrono>
#include <cmath>


EthercatAxisWorkspace::EthercatAxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent)
    : AxisWorkspace(axis_id, manager, parent) {
    configureTransportUi();
}

EthercatAxisWorkspace::~EthercatAxisWorkspace() = default;

void EthercatAxisWorkspace::configureTransportUi() {
    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    setModeOptions({
                       {QStringLiteral("Profile Position"), static_cast<int>(motion_core::AxisMode::ProfilePosition)},
                       {QStringLiteral("Cyclic Sync Position"), static_cast<int>(motion_core::AxisMode::CyclicSyncPosition)},
                       {QStringLiteral("Cyclic Sync Velocity"), static_cast<int>(motion_core::AxisMode::CyclicSyncVelocity)},
                       {QStringLiteral("Homing"), static_cast<int>(motion_core::AxisMode::Homing)},
                       {QStringLiteral("Manual Homing (DI3)"), static_cast<int>(motion_core::AxisMode::ManualHoming)},
                   },
                   0);
    if (h.speed_spin) {
        h.speed_spin->setValue(kDefaultEthercatSpeedRpm);
    }
    if (h.accel_spin) {
        h.accel_spin->setToolTip(QStringLiteral("Inactive for EtherCAT: accel command is currently not applied in runtime."));
        h.accel_spin->setStyleSheet(QStringLiteral("color: #808080;"));
    }

    current_speed_.store(kDefaultEthercatSpeedRpm, std::memory_order_relaxed);
    setMotionQueueStatsPlaceholder(
        QStringLiteral("queue: size=0 / 0, pushed=0, dropped=0, underruns=0, short_starts=0"));

    // Stats are emitted from AxisManager::onFastTick at 250 Hz (lock-free queue read),
    // which drives sine generation via onTransportMotionQueueStatsUpdated at the low-watermark.
    // No separate timer needed.

    if (h.mode_combo) {
        connect(h.mode_combo,
                qOverload<int>(&QComboBox::currentIndexChanged),
                this,
                [this](int) { updateSineControlsAvailability(); });
    }
    updateSineControlsAvailability();
    applyCurrentModeSelection();
}

void EthercatAxisWorkspace::onTransportSineToggled(bool enabled) {
    if (enabled && !supportsSineMode()) {
        disableSineUiState();
        return;
    }

    sine_enabled_.store(enabled, std::memory_order_release);
    if (!manager_) {
        return;
    }

    if (enabled) {
        motion_queue_prefilled_ = false;
        pending_refill_points_ = 0U;
        driver_queue_size_.store(0U, std::memory_order_release);
        const double start_center_deg = displayed_actual_deg_;
        sine_center_deg_.store(start_center_deg, std::memory_order_relaxed);
        desired_target_deg_.store(start_center_deg, std::memory_order_relaxed);
        commanded_target_deg_.store(start_center_deg, std::memory_order_relaxed);
        sine_phase_accum_rad_ = 0.0;
        scope_target_time_cursor_sec_ = 0.0;
        scope_target_time_cursor_initialized_ = false;
        ensureMotionQueueConfigured();
        QMetaObject::invokeMethod(manager_.data(),
                                  "clearMotionQueue",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
        // Don't fill/flush here — clearMotionQueue is async (QueuedConnection).
        // The next onFastTick (~4ms) will see size=0 < watermark and trigger
        // fillTrajectoryQueue + flushTrajectoryBatchToRuntime after the clear has executed.
        return;
    }

    motion_queue_prefilled_ = false;
    pending_refill_points_ = 0U;
    QMetaObject::invokeMethod(manager_.data(),
                              "clearMotionQueue",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id_));
    QMetaObject::invokeMethod(manager_.data(),
                              "requestMotionQueueStats",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id_));
}

void EthercatAxisWorkspace::onTransportMotionQueueStatsUpdated(const QVariantMap& stats) {
    const auto size         = stats.value(QStringLiteral("size")).toULongLong();
    const auto underruns    = stats.value(QStringLiteral("underruns")).toULongLong();
    const auto short_starts = stats.value(QStringLiteral("short_starts")).toULongLong();

    driver_queue_size_.store(size, std::memory_order_release);
    if (!motion_queue_prefilled_ && size >= kMotionQueuePrefillSamples) {
        motion_queue_prefilled_ = true;
    }

    // Generate a new batch if the hardware RT-cycle has drained the queue below the low-watermark.
    // This is the mirror of MksAxisWorkspace logic: generation is driven by the hardware consumer
    // (RT-cycle at 250 Hz), not by the UI timer (unstable OS scheduler).
    if (sine_enabled_.load(std::memory_order_acquire)) {
        if (size < kMotionQueueLowWatermarkSamples) {
            fillTrajectoryQueue();
            flushTrajectoryBatchToRuntime();
        }
    }

    // Log underruns for diagnostics
    if (underruns > last_driver_underruns_ && manager_) {
        const auto new_underruns = underruns - last_driver_underruns_;
        const auto capacity = stats.value(QStringLiteral("capacity")).toULongLong();
        const QString message = QStringLiteral(
                                    "EtherCAT Axis %1 streaming underrun: queue emptied (%2 new, size=%3 / %4)")
                                    .arg(axis_id_)
                                    .arg(new_underruns)
                                    .arg(size)
                                    .arg(capacity);
        auto* manager = manager_.data();
        QMetaObject::invokeMethod(
            manager,
            [manager, message]() {
                if (manager) {
                    emit manager->logMessage(QStringLiteral("ecat"), message);
                }
            },
            Qt::QueuedConnection);
    }
    last_driver_underruns_ = underruns;

    if (short_starts > last_driver_short_starts_ && manager_) {
        const auto new_short_starts = short_starts - last_driver_short_starts_;
        const auto capacity = stats.value(QStringLiteral("capacity")).toULongLong();
        const QString message = QStringLiteral(
                                    "EtherCAT Axis %1 streaming short-start: started before buffer ready (%2 new, size=%3 / %4)")
                                    .arg(axis_id_)
                                    .arg(new_short_starts)
                                    .arg(size)
                                    .arg(capacity);
        auto* manager = manager_.data();
        QMetaObject::invokeMethod(
            manager,
            [manager, message]() {
                if (manager) {
                    emit manager->logMessage(QStringLiteral("ecat"), message);
                }
            },
            Qt::QueuedConnection);
    }
    last_driver_short_starts_ = short_starts;
}

void EthercatAxisWorkspace::onTransportTelemetryUpdated(const QVariantMap& telemetry, const QString& transport, double t_sec) {
    Q_UNUSED(transport);

    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    if (h.accel_spin) {
        h.accel_spin->setEnabled(false);
    }

    if (!h.scope || !h.cmb_scope_signal) {
        return;
    }

    updateSineControlsAvailability();

    const QString signal = h.cmb_scope_signal->currentText();
    const bool show_velocity = signal.startsWith(QStringLiteral("Velocity"));
    if (show_velocity && h.chk_plot_target_vel && h.chk_plot_target_vel->isChecked()) {
        const double target = desired_target_deg_.load(std::memory_order_relaxed);
        if (have_prev_target_sample_) {
            const double dt = t_sec - prev_target_sample_time_sec_;
            if (dt > 1e-6) {
                const double target_vel = (target - prev_target_sample_deg_) / dt;
                h.scope->addData(QStringLiteral("target_speed"), t_sec, target_vel);
            }
        }
        prev_target_sample_deg_ = target;
        prev_target_sample_time_sec_ = t_sec;
        have_prev_target_sample_ = true;
    }

    if (telemetry.contains(QStringLiteral("mode")) && h.mode_combo && !h.mode_combo->hasFocus()) {
        const int mode = telemetry.value(QStringLiteral("mode")).toInt();
        const int combo_index = h.mode_combo->findData(mode);
        if (combo_index >= 0 && combo_index != h.mode_combo->currentIndex()) {
            h.mode_combo->blockSignals(true);
            h.mode_combo->setCurrentIndex(combo_index);
            h.mode_combo->blockSignals(false);
        }
    }
}

bool EthercatAxisWorkspace::transportOwnsTargetUi() const { return false; }

void EthercatAxisWorkspace::onBeforeDisableAxis() {
    clearMotionBuffersForServiceCommand(false);
}

void EthercatAxisWorkspace::onBeforeHomeAxis() {
    clearMotionBuffersForServiceCommand(false);
}

void EthercatAxisWorkspace::onBeforeSetZeroAxis() {
    clearMotionBuffersForServiceCommand(true);
}

bool EthercatAxisWorkspace::transportProvidesTargetTrace() const { return true; }

double EthercatAxisWorkspace::samplePeriodSec() const {
    return static_cast<double>(AxisWorkspace::kUiTrajectoryLoopPeriodUs) / 1'000'000.0;
}

QString EthercatAxisWorkspace::transportTag() const {
    return "EtherCAT";
}

bool EthercatAxisWorkspace::supportsSineMode() const {
    if (!control_panel_ || !control_panel_->handles().mode_combo) {
        return false;
    }
    const int mode = control_panel_->handles().mode_combo->currentData().toInt();
    return mode == static_cast<int>(motion_core::AxisMode::CyclicSyncPosition)
        || mode == static_cast<int>(motion_core::AxisMode::CyclicSyncVelocity);
}

void EthercatAxisWorkspace::updateSineControlsAvailability() {
    const bool enabled = supportsSineMode();
    const QString tool_tip = enabled
        ? QString()
        : QStringLiteral("Sine mode is available only in Cyclic Sync Position / Cyclic Sync Velocity modes.");
    setSineControlsEnabled(enabled, tool_tip);
    if (!enabled) {
        disableSineUiState();
    }
}

void EthercatAxisWorkspace::disableSineUiState() {
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
