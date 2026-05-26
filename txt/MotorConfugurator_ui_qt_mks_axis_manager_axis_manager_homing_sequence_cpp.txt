#include "mks/axis_manager/axis_manager.h"

#include "mks_can/adapter/mks_axis_adapter.h"

#include <QDateTime>
#include <QTimer>

#include <algorithm>

namespace {

constexpr int kHomingSequencePauseMs = 1000;
constexpr int kHomingSequenceAxisTimeoutMs = 120000;
constexpr int kHomingSequenceTickIntervalMs = 100;

} // namespace

namespace mks {

// ---------------------------------------------------------------------------
// startMksHomingSequence
//
// Entry point for multi-axis MKS homing. Can be called from:
// - UI: MksAllAxesControlWorkspace delegates here
// - IPC: HexaMotion sends ControlOp::StartMksHomingSequence
// ---------------------------------------------------------------------------
void AxisManager::startMksHomingSequence(const QList<int>& axis_ids) {
    if (homing_seq_running_ || axis_ids.isEmpty()) {
        emit logMessage(QStringLiteral("mks"),
                        QStringLiteral("MKS homing sequence start rejected: %1")
                            .arg(homing_seq_running_ ? QStringLiteral("already running")
                                                     : QStringLiteral("no axes")));
        return;
    }

    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (estop_active_) {
            emit logMessage(QStringLiteral("mks"),
                            QStringLiteral("MKS homing sequence start rejected: E-STOP active"));
            return;
        }
    }

    // Normalize and deduplicate axis list, sorted ascending.
    homing_sequence_axis_ids_ = axis_ids;
    std::sort(homing_sequence_axis_ids_.begin(), homing_sequence_axis_ids_.end());
    homing_sequence_axis_ids_.erase(
        std::unique(homing_sequence_axis_ids_.begin(), homing_sequence_axis_ids_.end()),
        homing_sequence_axis_ids_.end());

    homing_seq_running_ = true;
    homing_sequence_index_ = 0;
    homing_seq_phase_ = HomingSeqPhase::WaitingAxisComplete;
    homing_seq_phase_started_ms_ = 0;

    // Set the UI lock so other workspaces know a sequence is active.
    setMksHomingSequenceUiLock(true);

    // Create timer if needed.
    if (!homing_sequence_timer_) {
        homing_sequence_timer_ = new QTimer(this);
        homing_sequence_timer_->setInterval(kHomingSequenceTickIntervalMs);
        connect(homing_sequence_timer_, &QTimer::timeout,
                this, &AxisManager::onHomingSequenceTick);
    }
    homing_sequence_timer_->start();

    emit logMessage(QStringLiteral("mks"),
                    QStringLiteral("MKS homing sequence started: %1 axis(es)")
                        .arg(homing_sequence_axis_ids_.size()));

    homingSequenceStartCurrentAxis();
    homingSequenceEmitProgress(QStringLiteral("Sequence started"));
}

// ---------------------------------------------------------------------------
// stopMksHomingSequence
// ---------------------------------------------------------------------------
void AxisManager::stopMksHomingSequence() {
    if (!homing_seq_running_) {
        setMksHomingSequenceUiLock(false);
        homingSequenceEmitProgress(QStringLiteral("Idle"));
        return;
    }

    // Abort current axis: clear motion queue + disable.
    if (homing_sequence_index_ >= 0
        && homing_sequence_index_ < homing_sequence_axis_ids_.size()) {
        const int active_axis_id = homing_sequence_axis_ids_.at(homing_sequence_index_);
        const auto axis_res = unified_runtime_.find_axis(
            static_cast<std::uint16_t>(active_axis_id));
        if (axis_res.ok()) {
            motion_core::ServiceCommandPoint clear_q{};
            clear_q.type = motion_core::ServiceCommandType::ClearMotionQueue;
            clear_q.axis_id = active_axis_id;
            (void)axis_res.value()->enqueueServicePoint(clear_q);

            motion_core::ServiceCommandPoint disable{};
            disable.type = motion_core::ServiceCommandType::Disable;
            disable.axis_id = active_axis_id;
            (void)axis_res.value()->enqueueServicePoint(disable);
        }
    }

    homing_seq_running_ = false;
    homing_sequence_index_ = -1;
    homing_seq_phase_ = HomingSeqPhase::Idle;
    homing_seq_phase_started_ms_ = 0;
    if (homing_sequence_timer_) {
        homing_sequence_timer_->stop();
    }

    setMksHomingSequenceUiLock(false);

    emit logMessage(QStringLiteral("mks"),
                    QStringLiteral("MKS homing sequence stopped"));
    homingSequenceEmitProgress(QStringLiteral("Sequence stopped"));
}

// ---------------------------------------------------------------------------
// homingSequenceStartCurrentAxis
// ---------------------------------------------------------------------------
void AxisManager::homingSequenceStartCurrentAxis() {
    if (!homing_seq_running_
        || homing_sequence_index_ < 0
        || homing_sequence_index_ >= homing_sequence_axis_ids_.size()) {
        return;
    }

    homing_seq_phase_ = HomingSeqPhase::WaitingAxisComplete;
    homing_seq_phase_started_ms_ = QDateTime::currentMSecsSinceEpoch();
    const int axis_id = homing_sequence_axis_ids_.at(homing_sequence_index_);

    const auto axis_res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(QStringLiteral("mks"),
                        QStringLiteral("MKS homing sequence: axis %1 not found, stopping")
                            .arg(axis_id));
        stopMksHomingSequence();
        return;
    }

    // Send Home service command.
    motion_core::ServiceCommandPoint home{};
    home.type = motion_core::ServiceCommandType::Home;
    home.axis_id = axis_id;
    if (!axis_res.value()->enqueueServicePoint(home)) {
        emit logMessage(QStringLiteral("mks"),
                        QStringLiteral("MKS homing sequence: failed to enqueue Home on axis %1")
                            .arg(axis_id));
        stopMksHomingSequence();
        return;
    }

    emit logMessage(QStringLiteral("mks"),
                    QStringLiteral("MKS homing sequence: starting homing on axis %1")
                        .arg(axis_id));
    homingSequenceEmitProgress(
        QStringLiteral("Homing axis %1...").arg(axis_id));
}

// ---------------------------------------------------------------------------
// onHomingSequenceTick
//
// Runs every 100ms during a homing sequence. Monitors per-axis progress via
// telemetry and drives the inter-axis state machine.
// ---------------------------------------------------------------------------
void AxisManager::onHomingSequenceTick() {
    if (!homing_seq_running_
        || homing_sequence_index_ < 0
        || homing_sequence_index_ >= homing_sequence_axis_ids_.size()) {
        return;
    }

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    const int axis_id = homing_sequence_axis_ids_.at(homing_sequence_index_);

    // Timeout check for WaitingAxisComplete and WaitingAxisIdle.
    if (homing_seq_phase_ == HomingSeqPhase::WaitingAxisComplete
        || homing_seq_phase_ == HomingSeqPhase::WaitingAxisIdle) {
        if (homing_seq_phase_started_ms_ > 0
            && (now_ms - homing_seq_phase_started_ms_) > kHomingSequenceAxisTimeoutMs) {
            emit logMessage(QStringLiteral("mks"),
                            QStringLiteral("MKS homing sequence: axis %1 timed out").arg(axis_id));
            stopMksHomingSequence();
            return;
        }
    }

    // Read telemetry for the current axis.
    const auto axis_res = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        stopMksHomingSequence();
        return;
    }

    // Check for E-STOP.
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (estop_active_) {
            emit logMessage(QStringLiteral("mks"),
                            QStringLiteral("MKS homing sequence: E-STOP detected, stopping"));
            stopMksHomingSequence();
            return;
        }
    }

    // Get homing status from the MKS adapter (transport-specific).
    QString homing_status_text;
    int motion_status_code = 0;
    auto* mks_adapter = dynamic_cast<MksAxisAdapter*>(axis_res.value().get());
    if (mks_adapter) {
        homing_status_text = QString::fromUtf8(mks_adapter->homing_status_text());
        const auto telem = mks_adapter->telemetry();
        motion_status_code = static_cast<int>(telem.motion_status_code);
    } else {
        const auto telem = axis_res.value()->telemetry();
        motion_status_code = static_cast<int>(telem.motion_status_code);
        homing_status_text = QStringLiteral("Idle");
    }

    // Check for failure.
    if (homing_status_text == QStringLiteral("Failed")) {
        emit logMessage(QStringLiteral("mks"),
                        QStringLiteral("MKS homing sequence: axis %1 homing failed").arg(axis_id));
        stopMksHomingSequence();
        return;
    }

    switch (homing_seq_phase_) {
        case HomingSeqPhase::Idle:
            return;

        case HomingSeqPhase::WaitingAxisComplete:
            if (homing_status_text == QStringLiteral("Completed")) {
                homing_seq_phase_ = HomingSeqPhase::WaitingAxisIdle;
                homing_seq_phase_started_ms_ = now_ms;
                homingSequenceEmitProgress(
                    QStringLiteral("Axis %1 homing completed, waiting for motor to settle...")
                        .arg(axis_id));
            }
            return;

        case HomingSeqPhase::WaitingAxisIdle: {
            const bool homing_is_idle =
                (homing_status_text.compare(QStringLiteral("Idle"), Qt::CaseInsensitive) == 0
                 || homing_status_text == QStringLiteral("Completed"));
            const bool motor_is_stopped = (motion_status_code == 1); // 1 = Stopped
            if (homing_is_idle && motor_is_stopped) {
                homing_seq_phase_ = HomingSeqPhase::WaitingPause;
                homing_seq_phase_started_ms_ = now_ms;
                homingSequenceEmitProgress(
                    QStringLiteral("Axis %1 settled. Pausing before next axis...")
                        .arg(axis_id));
            }
            return;
        }

        case HomingSeqPhase::WaitingPause:
            if (now_ms - homing_seq_phase_started_ms_ < kHomingSequencePauseMs) {
                return;
            }
            // Advance to next axis.
            ++homing_sequence_index_;
            if (homing_sequence_index_ >= homing_sequence_axis_ids_.size()) {
                homing_seq_running_ = false;
                homing_sequence_index_ = -1;
                homing_seq_phase_ = HomingSeqPhase::Idle;
                homing_seq_phase_started_ms_ = 0;
                if (homing_sequence_timer_) {
                    homing_sequence_timer_->stop();
                }
                setMksHomingSequenceUiLock(false);
                emit logMessage(QStringLiteral("mks"),
                                QStringLiteral("MKS homing sequence completed successfully"));
                homingSequenceEmitProgress(QStringLiteral("Sequence completed"));
                return;
            }
            homingSequenceStartCurrentAxis();
            return;
    }
}

// ---------------------------------------------------------------------------
// homingSequenceEmitProgress
// ---------------------------------------------------------------------------
void AxisManager::homingSequenceEmitProgress(const QString& status_text) {
    QVariantMap progress;
    progress.insert(QStringLiteral("running"), homing_seq_running_);
    progress.insert(QStringLiteral("status_text"), status_text);
    progress.insert(QStringLiteral("total_axes"),
                    homing_sequence_axis_ids_.size());
    progress.insert(QStringLiteral("current_index"),
                    homing_seq_running_ ? homing_sequence_index_ : -1);
    if (homing_seq_running_
        && homing_sequence_index_ >= 0
        && homing_sequence_index_ < homing_sequence_axis_ids_.size()) {
        progress.insert(QStringLiteral("current_axis_id"),
                        homing_sequence_axis_ids_.at(homing_sequence_index_));
    }
    emit mksHomingSequenceProgress(progress);
}

} // namespace mks
