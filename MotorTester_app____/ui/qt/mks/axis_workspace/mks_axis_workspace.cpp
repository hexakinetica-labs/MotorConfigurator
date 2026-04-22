#include "mks/axis_workspace/mks_axis_workspace.h"

#include "mks/axis_workspace/axis_workspace_control_panel.h"
#include "mks/axis_manager.h"
#include "mks/ScopeWidget.h"

#include "motion_core/axis_data.h"
#include "motion_core/types.h"

#include <QCheckBox>
#include <QComboBox>

#include <QMetaObject>
#include <QSlider>
#include <QSpinBox>

#include <QVariantList>
#include <QVector>

#include <algorithm>
#include <cmath>

namespace {

constexpr int kUiTrajectoryLoopPeriodUs = 4'000;
constexpr double kMksQueueSamplePeriodSec =
    static_cast<double>(kUiTrajectoryLoopPeriodUs) / 1'000'000.0;

}

MksAxisWorkspace::MksAxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent)
    : AxisWorkspace(axis_id, manager, parent) {
    configureTransportUi();
}

MksAxisWorkspace::~MksAxisWorkspace() = default;

void MksAxisWorkspace::configureTransportUi() {
    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    setModeOptions({{QStringLiteral("Position (MKS)"), static_cast<int>(motion_core::AxisMode::ProfilePosition)}}, 0);
    if (h.speed_spin) {
        h.speed_spin->setValue(kDefaultMksSpeedRpm);
    }
    if (h.accel_spin) {
        h.accel_spin->setValue(kDefaultMksAccelPercent);
    }
    current_speed_.store(kDefaultMksSpeedRpm, std::memory_order_relaxed);
    current_accel_.store(kDefaultMksAccelPercent, std::memory_order_relaxed);
    setSineControlsEnabled(true);
    setMotionQueueStatsPlaceholder(
        QStringLiteral("queue: size=0 / 0, pushed=0, dropped=0, underruns=0, short_starts=0"));

    applyCurrentModeSelection();
}

void MksAxisWorkspace::onTransportSineToggled(bool enabled) {
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
    scope_target_time_cursor_sec_ = 0.0;
    scope_target_time_cursor_initialized_ = false;
    desired_target_deg_.store(commanded_target_deg_.load(std::memory_order_relaxed),
                              std::memory_order_relaxed);
}

void MksAxisWorkspace::onTransportMotionQueueStatsUpdated(const QVariantMap& stats) {
    const auto size = stats.value(QStringLiteral("size")).toULongLong();
    const auto capacity = stats.value(QStringLiteral("capacity")).toULongLong();
    const auto underruns = stats.value(QStringLiteral("underruns")).toULongLong();
    const auto short_starts = stats.value(QStringLiteral("short_starts")).toULongLong();

    driver_queue_size_.store(size, std::memory_order_release);
    if (!motion_queue_prefilled_ && size >= kMotionQueuePrefillSamples) {
        motion_queue_prefilled_ = true;
    }

    if (sine_enabled_.load(std::memory_order_acquire)) {
        if (size < kMotionQueueLowWatermarkSamples) {
            fillTrajectoryQueue();
            flushTrajectoryBatchToRuntime();
        }
    }

    if (underruns > last_driver_underruns_ && manager_) {
        const auto new_underruns = underruns - last_driver_underruns_;
        const QString message = QStringLiteral(
                                    "Axis %1 streaming underrun detected: queue emptied (%2 new, size=%3 / %4)")
                                    .arg(axis_id_)
                                    .arg(new_underruns)
                                    .arg(size)
                                    .arg(capacity);
        auto* manager = manager_.data();
        QMetaObject::invokeMethod(
            manager,
            [manager, message]() {
                if (manager) {
                    emit manager->logMessage(QStringLiteral("mks"), message);
                }
            },
            Qt::QueuedConnection);
    }
    last_driver_underruns_ = underruns;

    if (short_starts > last_driver_short_starts_ && manager_) {
        const auto new_short_starts = short_starts - last_driver_short_starts_;
        const QString message = QStringLiteral(
                                    "Axis %1 streaming short-start detected: started before full pre-buffer was ready (%2 new, size=%3 / %4)")
                                    .arg(axis_id_)
                                    .arg(new_short_starts)
                                    .arg(size)
                                    .arg(capacity);
        auto* manager = manager_.data();
        QMetaObject::invokeMethod(
            manager,
            [manager, message]() {
                if (manager) {
                    emit manager->logMessage(QStringLiteral("mks"), message);
                }
            },
            Qt::QueuedConnection);
    }
    last_driver_short_starts_ = short_starts;
}

void MksAxisWorkspace::onTransportTelemetryUpdated(const QVariantMap& telemetry, const QString& transport, double t_sec) {
    Q_UNUSED(transport);

    if (!control_panel_) {
        return;
    }

    auto& h = control_panel_->handles();
    if (!h.scope || !h.cmb_scope_signal) {
        return;
    }

    const QString signal = h.cmb_scope_signal->currentText();
    const bool show_position = signal.startsWith(QStringLiteral("Position"));
    const bool show_velocity = signal.startsWith(QStringLiteral("Velocity"));
    const bool show_torque = signal.startsWith(QStringLiteral("Torque"));

    if (show_position && h.chk_plot_actual_pos && h.chk_plot_actual_pos->isChecked() && telemetry.contains(QStringLiteral("position_samples")) && telemetry_t0_ns_ != 0U) {
        const QVariantList samples = telemetry.value(QStringLiteral("position_samples")).toList();
        QVector<QPointF> actual_batch;
        actual_batch.reserve(samples.size());
        for (const auto& sample_variant : samples) {
            const QVariantMap sample = sample_variant.toMap();
            const auto sample_ns = sample.value(QStringLiteral("timestamp_ns")).toULongLong();
            if (sample_ns == 0U || sample_ns < telemetry_t0_ns_) {
                continue;
            }
            const double sample_t_sec = static_cast<double>(sample_ns - telemetry_t0_ns_) / 1'000'000'000.0;
            actual_batch.append(QPointF(sample_t_sec, sample.value(QStringLiteral("position_deg")).toDouble()));
        }
        if (!actual_batch.isEmpty()) {
            h.scope->addDataBatch(QStringLiteral("actual"), actual_batch);
        }
    }

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

    if (show_torque && h.chk_plot_actual_vel && h.chk_plot_actual_vel->isChecked() && telemetry.contains(QStringLiteral("actual_torque_percent"))) {
        h.scope->addData(QStringLiteral("torque"), t_sec, telemetry.value(QStringLiteral("actual_torque_percent")).toDouble());
    }
}

bool MksAxisWorkspace::transportOwnsTargetUi() const { return false; }

void MksAxisWorkspace::onBeforeDisableAxis() {
    stopSineModeForDisable();
}

void MksAxisWorkspace::onBeforeHomeAxis() {
    clearMotionBuffersForServiceCommand(false);
}

void MksAxisWorkspace::onBeforeSetZeroAxis() {
    clearMotionBuffersForServiceCommand(true);
}

bool MksAxisWorkspace::transportProvidesTargetTrace() const { return true; }

double MksAxisWorkspace::samplePeriodSec() const {
    return kMksQueueSamplePeriodSec;
}

QString MksAxisWorkspace::transportTag() const {
    return QStringLiteral("mks");
}
