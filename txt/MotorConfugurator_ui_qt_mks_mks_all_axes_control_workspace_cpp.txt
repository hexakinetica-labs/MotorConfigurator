#include "mks/mks_all_axes_control_workspace.h"

#include "mks/axis_manager/axis_manager.h"

#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QVBoxLayout>

#include <algorithm>

namespace {

constexpr int kPositionSliderScale = 100;
constexpr int kSliderMinRaw = -2000;   // -20°
constexpr int kSliderMaxRaw = 2000;    // +20°
constexpr int kGlobalEstopAxisId = 0;

QLabel* make_value_label(QWidget* parent, const QString& text = QStringLiteral("---")) {
    auto* label = new QLabel(text, parent);
    label->setMinimumWidth(110);
    return label;
}

QPushButton* make_action_button(const QString& text, QWidget* parent, const QString& style = {}) {
    auto* button = new QPushButton(text, parent);
    if (!style.isEmpty()) {
        button->setStyleSheet(style);
    }
    return button;
}

} // namespace

MksAllAxesControlWorkspace::MksAllAxesControlWorkspace(mks::AxisManager* manager, QWidget* parent)
    : QWidget(parent)
    , manager_(manager) {
    setupUi();

    if (manager_) {
        connect(manager_, &mks::AxisManager::telemetryUpdated,
                this, &MksAllAxesControlWorkspace::onTelemetryUpdated);
        connect(manager_, &mks::AxisManager::hostStateUpdated,
                this, &MksAllAxesControlWorkspace::onHostStateUpdated);
        connect(manager_, &mks::AxisManager::scanFinished,
                this, &MksAllAxesControlWorkspace::onScanFinished);

        // Listen to homing sequence progress from AxisManager (the canonical orchestrator).
        connect(manager_, &mks::AxisManager::mksHomingSequenceProgress,
                this, [this](const QVariantMap& progress) {
            const bool running = progress.value(QStringLiteral("running")).toBool();
            const QString status = progress.value(QStringLiteral("status_text")).toString();
            const int total = progress.value(QStringLiteral("total_axes")).toInt();
            const int index = progress.value(QStringLiteral("current_index")).toInt();
            const int current_axis = progress.value(QStringLiteral("current_axis_id"), -1).toInt();

            sequence_running_ = running;
            sequence_current_axis_id_ = running ? current_axis : -1;

            // Build rich status text.
            if (running && total > 0 && index >= 0) {
                updateSequenceStatusLabel(
                    QStringLiteral("[%1/%2] %3").arg(index + 1).arg(total).arg(status));
            } else {
                updateSequenceStatusLabel(status);
            }

            // Update progress bar.
            if (progress_sequence_) {
                if (running && total > 0) {
                    progress_sequence_->setRange(0, total);
                    progress_sequence_->setValue(index);
                    progress_sequence_->setFormat(
                        QStringLiteral("Axis %1 — %2 / %3")
                            .arg(current_axis).arg(index + 1).arg(total));
                    progress_sequence_->setVisible(true);
                } else {
                    progress_sequence_->setVisible(false);
                    progress_sequence_->setValue(0);
                }
            }

            // Highlight the row being homed.
            applySequenceHighlight(sequence_current_axis_id_);
            updateUiState();
        });
    }

    updateUiState();
}

MksAllAxesControlWorkspace::~MksAllAxesControlWorkspace() {
    stopHomingSequence();
    for (const int axis_id : axis_ids_) {
        watchAxis(axis_id, false);
    }
}

void MksAllAxesControlWorkspace::setAxisIds(const QList<int>& axis_ids) {
    QList<int> normalized = axis_ids;
    std::sort(normalized.begin(), normalized.end());
    normalized.erase(std::unique(normalized.begin(), normalized.end()), normalized.end());

    for (const int axis_id : axis_ids_) {
        if (!normalized.contains(axis_id)) {
            watchAxis(axis_id, false);
        }
    }
    for (const int axis_id : normalized) {
        if (!axis_ids_.contains(axis_id)) {
            watchAxis(axis_id, true);
        }
    }

    axis_ids_ = normalized;

    if (sequence_running_) {
        stopHomingSequence();
    }

    rebuildAxisRows();
    updateUiState();
}

// ---------------------------------------------------------------------------
// Telemetry — pure display, no orchestration logic
// ---------------------------------------------------------------------------
void MksAllAxesControlWorkspace::onTelemetryUpdated(const int axis_id, const QVariantMap& telemetry) {
    auto row_it = axis_rows_.find(axis_id);
    if (row_it == axis_rows_.end()) {
        return;
    }

    auto& row = row_it.value();
    const double actual_position_deg = telemetry.value(QStringLiteral("actual_position_deg")).toDouble();
    if (row.position) {
        row.position->setText(QString::number(actual_position_deg, 'f', 2) + QStringLiteral(" °"));
    }

    const QString homing_status = telemetry.value(QStringLiteral("mks_homing_sequence_status"),
                                                  QStringLiteral("Idle")).toString();
    row.homing_status = homing_status;
    row.motion_status_code = telemetry.value(QStringLiteral("motion_status")).toInt();
    QString status_text = homing_status;
    if (status_text.compare(QStringLiteral("Idle"), Qt::CaseInsensitive) == 0) {
        status_text = decodeMksMotionStatus(row.motion_status_code);
    }
    if (row.status) {
        row.status->setText(status_text);
    }

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (row.slider && !row.slider->isSliderDown() && now_ms >= row.manual_slider_hold_until_ms) {
        row.slider->blockSignals(true);
        row.slider->setValue(static_cast<int>(std::llround(actual_position_deg * kPositionSliderScale)));
        row.slider->blockSignals(false);
        if (row.slider_value) {
            row.slider_value->setText(QString::number(actual_position_deg, 'f', 2) + QStringLiteral(" °"));
        }
    }
}

void MksAllAxesControlWorkspace::onHostStateUpdated(const QVariantMap& state) {
    ui_manual_control_available_ =
        state.value(QStringLiteral("control_source")).toString() == QStringLiteral("ui");
    estop_active_ = state.value(QStringLiteral("estop_active")).toBool();
    mks_homing_sequence_ui_lock_active_ = state.value(QStringLiteral("mks_homing_sequence_active")).toBool();

    if ((!ui_manual_control_available_ || estop_active_) && sequence_running_) {
        stopHomingSequence();
    }

    updateUiState();
}

void MksAllAxesControlWorkspace::onScanFinished(const QString& transport_tag, const QVariantList& axis_ids) {
    if (transport_tag != QStringLiteral("mks")) {
        return;
    }

    QList<int> ids;
    ids.reserve(axis_ids.size());
    for (const auto& id_value : axis_ids) {
        ids.push_back(id_value.toInt());
    }
    setAxisIds(ids);
}

// ---------------------------------------------------------------------------
// UI setup
// ---------------------------------------------------------------------------
void MksAllAxesControlWorkspace::setupUi() {
    auto* root = new QVBoxLayout(this);

    auto* actions_group = new QGroupBox(QStringLiteral("MKS All Axes Control"), this);
    auto* actions_layout = new QVBoxLayout(actions_group);

    auto* buttons_row = new QHBoxLayout();
    btn_enable_all_ = make_action_button(QStringLiteral("ENABLE ALL"), actions_group,
                                         QStringLiteral("background-color:#2ea043; color:white; font-weight:bold;"));
    btn_disable_all_ = make_action_button(QStringLiteral("DISABLE ALL"), actions_group,
                                          QStringLiteral("background-color:#da3633; color:white; font-weight:bold;"));
    btn_set_zero_all_ = make_action_button(QStringLiteral("SET ZERO ALL"), actions_group,
                                           QStringLiteral("font-weight:bold;"));
    btn_estop_all_ = make_action_button(QStringLiteral("E-STOP ALL"), actions_group,
                                        QStringLiteral("background-color:#da3633; color:white; font-weight:bold; padding:8px;"));
    btn_start_sequence_ = make_action_button(QStringLiteral("START HOMING SEQUENCE"), actions_group,
                                             QStringLiteral("font-weight:bold;"));
    btn_stop_sequence_ = make_action_button(QStringLiteral("STOP SEQUENCE"), actions_group,
                                            QStringLiteral("background-color:#d29922; color:black; font-weight:bold;"));
    buttons_row->addWidget(btn_enable_all_);
    buttons_row->addWidget(btn_disable_all_);
    buttons_row->addWidget(btn_set_zero_all_);
    buttons_row->addSpacing(12);
    buttons_row->addWidget(btn_estop_all_);
    buttons_row->addSpacing(16);
    buttons_row->addWidget(btn_start_sequence_);
    buttons_row->addWidget(btn_stop_sequence_);
    buttons_row->addStretch();
    actions_layout->addLayout(buttons_row);

    lbl_sequence_status_ = new QLabel(QStringLiteral("Idle"), actions_group);
    lbl_sequence_status_->setStyleSheet(QStringLiteral("font-weight:bold; font-size:11pt;"));
    actions_layout->addWidget(lbl_sequence_status_);

    progress_sequence_ = new QProgressBar(actions_group);
    progress_sequence_->setRange(0, 1);
    progress_sequence_->setValue(0);
    progress_sequence_->setTextVisible(true);
    progress_sequence_->setVisible(false);
    progress_sequence_->setMinimumHeight(22);
    progress_sequence_->setStyleSheet(QStringLiteral(
        "QProgressBar { border: 1px solid #555; border-radius: 3px; background: #2b2b2b; text-align: center; color: white; }"
        "QProgressBar::chunk { background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #2ea043, stop:1 #4ecdc4); }"
    ));
    actions_layout->addWidget(progress_sequence_);

    root->addWidget(actions_group);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    axis_list_container_ = new QWidget(scroll_area_);
    axis_list_layout_ = new QVBoxLayout(axis_list_container_);
    axis_list_layout_->setContentsMargins(0, 0, 0, 0);
    axis_list_layout_->setSpacing(8);
    axis_list_layout_->addStretch();
    scroll_area_->setWidget(axis_list_container_);
    root->addWidget(scroll_area_, 1);

    connect(btn_enable_all_, &QPushButton::clicked, this, [this]() {
        for (const int axis_id : axis_ids_) {
            sendServiceCommand(axis_id, 1);
        }
    });
    connect(btn_disable_all_, &QPushButton::clicked, this, [this]() {
        for (const int axis_id : axis_ids_) {
            sendServiceCommand(axis_id, 2);
        }
    });
    connect(btn_set_zero_all_, &QPushButton::clicked, this, [this]() {
        for (const int axis_id : axis_ids_) {
            sendServiceCommand(axis_id, 5);
        }
    });
    connect(btn_estop_all_, &QPushButton::clicked, this, [this]() {
        if (!manager_) {
            return;
        }
        QMetaObject::invokeMethod(manager_.data(),
                                  "emergencyStop",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, kGlobalEstopAxisId));
    });
    connect(btn_start_sequence_, &QPushButton::clicked,
            this, &MksAllAxesControlWorkspace::startHomingSequence);
    connect(btn_stop_sequence_, &QPushButton::clicked, this, [this]() {
        stopHomingSequence();
    });
}

void MksAllAxesControlWorkspace::rebuildAxisRows() {
    while (axis_list_layout_->count() > 1) {
        auto* item = axis_list_layout_->takeAt(0);
        if (!item) {
            continue;
        }
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }
    axis_rows_.clear();

    for (const int axis_id : axis_ids_) {
        AxisRowWidgets row{};
        row.container = new QWidget(axis_list_container_);
        auto* layout = new QVBoxLayout(row.container);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->setSpacing(4);

        row.title = new QLabel(QStringLiteral("Axis %1").arg(axis_id), row.container);
        row.title->setStyleSheet(QStringLiteral("font-weight:bold; font-size:12pt;"));

        auto* position_block = new QHBoxLayout();
        position_block->addWidget(new QLabel(QStringLiteral("Position:"), row.container));
        row.position = make_value_label(row.container, QStringLiteral("---"));
        position_block->addWidget(row.position);
        position_block->addStretch();

        auto* status_block = new QHBoxLayout();
        status_block->addWidget(new QLabel(QStringLiteral("Status:"), row.container));
        row.status = make_value_label(row.container, QStringLiteral("Idle"));
        status_block->addWidget(row.status);
        status_block->addStretch();

        row.slider_value = make_value_label(row.container, QStringLiteral("0.00 °"));
        row.slider = new QSlider(Qt::Horizontal, row.container);
        row.slider->setRange(kSliderMinRaw, kSliderMaxRaw);
        row.slider->setValue(0);
        row.slider->setTracking(true);

        row.btn_enable = make_action_button(QStringLiteral("Enable"), row.container);
        row.btn_disable = make_action_button(QStringLiteral("Disable"), row.container);
        row.btn_home = make_action_button(QStringLiteral("Home"), row.container);
        row.btn_set_zero = make_action_button(QStringLiteral("Set Zero"), row.container);

        auto* slider_block = new QHBoxLayout();
        slider_block->addWidget(row.slider);
        slider_block->addWidget(row.slider_value);

        auto* buttons_block = new QHBoxLayout();
        buttons_block->addWidget(row.btn_enable);
        buttons_block->addWidget(row.btn_disable);
        buttons_block->addWidget(row.btn_home);
        buttons_block->addWidget(row.btn_set_zero);

        layout->addWidget(row.title);
        layout->addLayout(position_block);
        layout->addLayout(status_block);
        layout->addLayout(slider_block, 1);
        layout->addLayout(buttons_block);

        connect(row.btn_enable, &QPushButton::clicked, this, [this, axis_id]() {
            sendServiceCommand(axis_id, 1);
        });
        connect(row.btn_disable, &QPushButton::clicked, this, [this, axis_id]() {
            sendServiceCommand(axis_id, 2);
        });
        connect(row.btn_home, &QPushButton::clicked, this, [this, axis_id]() {
            sendServiceCommand(axis_id, 4);
        });
        connect(row.btn_set_zero, &QPushButton::clicked, this, [this, axis_id]() {
            sendServiceCommand(axis_id, 5);
        });
        connect(row.slider, &QSlider::valueChanged, this, [this, axis_id](const int raw_value) {
            auto row_it = axis_rows_.find(axis_id);
            if (row_it == axis_rows_.end()) {
                return;
            }
            if (row_it->slider_value) {
                row_it->slider_value->setText(
                    QString::number(static_cast<double>(raw_value) / kPositionSliderScale, 'f', 2)
                    + QStringLiteral(" °"));
            }
        });
        connect(row.slider, &QSlider::sliderReleased, this, [this, axis_id]() {
            auto row_it = axis_rows_.find(axis_id);
            if (row_it == axis_rows_.end() || !row_it->slider) {
                return;
            }
            row_it->manual_slider_hold_until_ms = QDateTime::currentMSecsSinceEpoch() + 1500;
            const double target_deg = static_cast<double>(row_it->slider->value()) / kPositionSliderScale;
            sendAbsoluteMove(axis_id, target_deg);
        });

        axis_rows_.insert(axis_id, row);
        axis_list_layout_->insertWidget(axis_list_layout_->count() - 1, row.container);
    }
}

void MksAllAxesControlWorkspace::updateUiState() {
    const bool manual_allowed = ui_manual_control_available_ && !estop_active_;
    const bool external_lock_active = mks_homing_sequence_ui_lock_active_ && !sequence_running_;
    const bool per_axis_controls_enabled = manual_allowed && !mks_homing_sequence_ui_lock_active_;
    const bool global_buttons_enabled = manual_allowed && !sequence_running_ && !external_lock_active;

    for (auto it = axis_rows_.begin(); it != axis_rows_.end(); ++it) {
        auto& row = it.value();
        if (row.slider) row.slider->setEnabled(per_axis_controls_enabled);
        if (row.btn_enable) row.btn_enable->setEnabled(per_axis_controls_enabled);
        if (row.btn_disable) row.btn_disable->setEnabled(per_axis_controls_enabled);
        if (row.btn_home) row.btn_home->setEnabled(per_axis_controls_enabled);
        if (row.btn_set_zero) row.btn_set_zero->setEnabled(per_axis_controls_enabled);
    }

    if (btn_enable_all_) btn_enable_all_->setEnabled(global_buttons_enabled);
    if (btn_disable_all_) btn_disable_all_->setEnabled(global_buttons_enabled);
    if (btn_set_zero_all_) btn_set_zero_all_->setEnabled(global_buttons_enabled);
    if (btn_estop_all_) btn_estop_all_->setEnabled(true);
    if (btn_start_sequence_) {
        btn_start_sequence_->setEnabled(global_buttons_enabled && !axis_ids_.isEmpty());
    }
    if (btn_stop_sequence_) {
        btn_stop_sequence_->setEnabled(sequence_running_);
    }

    if (!manual_allowed && !sequence_running_) {
        updateSequenceStatusLabel(QStringLiteral("UI control is unavailable"));
    } else if (external_lock_active) {
        updateSequenceStatusLabel(QStringLiteral("MKS homing sequence is active in another workspace"));
    } else if (!sequence_running_ && axis_ids_.isEmpty()) {
        updateSequenceStatusLabel(QStringLiteral("No MKS axes available"));
    } else if (!sequence_running_ && lbl_sequence_status_ && lbl_sequence_status_->text().isEmpty()) {
        updateSequenceStatusLabel(QStringLiteral("Idle"));
    }
}

// ---------------------------------------------------------------------------
// Command helpers
// ---------------------------------------------------------------------------
void MksAllAxesControlWorkspace::watchAxis(const int axis_id, const bool enabled) {
    if (!manager_) {
        return;
    }
    QMetaObject::invokeMethod(manager_.data(),
                              "watchAxis",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id),
                              Q_ARG(bool, enabled));
}

void MksAllAxesControlWorkspace::sendServiceCommand(const int axis_id, const int kind) {
    if (!manager_) {
        return;
    }

    QVariantMap command;
    command.insert(QStringLiteral("kind"), kind);
    QVariantList batch;
    batch.push_back(command);
    QMetaObject::invokeMethod(manager_.data(),
                              "enqueueServiceBatch",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id),
                              Q_ARG(QVariantList, batch));
}

void MksAllAxesControlWorkspace::sendAbsoluteMove(const int axis_id, const double target_deg) {
    if (!manager_) {
        return;
    }

    QVariantMap point;
    point.insert(QStringLiteral("kind"), 1);
    point.insert(QStringLiteral("target_position_deg"), target_deg);
    point.insert(QStringLiteral("profile_speed_rpm"), 300);
    point.insert(QStringLiteral("profile_accel_percent"), 80);
    point.insert(QStringLiteral("has_profile_speed_rpm"), true);
    point.insert(QStringLiteral("has_profile_accel_percent"), true);
    point.insert(QStringLiteral("is_relative"), false);
    QVariantList batch;
    batch.push_back(point);
    QMetaObject::invokeMethod(manager_.data(),
                              "enqueueMotionBatch",
                              Qt::QueuedConnection,
                              Q_ARG(int, axis_id),
                              Q_ARG(QVariantList, batch));
}

// ---------------------------------------------------------------------------
// Homing sequence — thin delegates to AxisManager
// ---------------------------------------------------------------------------
void MksAllAxesControlWorkspace::startHomingSequence() {
    if (!manager_ || axis_ids_.isEmpty() || sequence_running_ || mks_homing_sequence_ui_lock_active_
        || !ui_manual_control_available_ || estop_active_) {
        return;
    }

    sequence_running_ = true;
    updateUiState();

    QMetaObject::invokeMethod(manager_.data(),
                              [mgr = QPointer<mks::AxisManager>(manager_), ids = axis_ids_]() {
                                  if (mgr) {
                                      mgr->startMksHomingSequence(ids);
                                  }
                              },
                              Qt::QueuedConnection);
}

void MksAllAxesControlWorkspace::stopHomingSequence() {
    sequence_running_ = false;
    sequence_current_axis_id_ = -1;
    applySequenceHighlight(-1);
    updateUiState();

    if (manager_) {
        QMetaObject::invokeMethod(manager_.data(),
                                  [mgr = QPointer<mks::AxisManager>(manager_)]() {
                                      if (mgr) mgr->stopMksHomingSequence();
                                  },
                                  Qt::QueuedConnection);
    }
}

void MksAllAxesControlWorkspace::updateSequenceStatusLabel(const QString& text) {
    if (lbl_sequence_status_) {
        lbl_sequence_status_->setText(text);
    }
}

void MksAllAxesControlWorkspace::applySequenceHighlight(const int active_axis_id) {
    const QString active_style = QStringLiteral(
        "border: 2px solid #4ecdc4; border-radius: 4px; background-color: rgba(78, 205, 196, 30);");
    const QString normal_style{};

    for (auto it = axis_rows_.begin(); it != axis_rows_.end(); ++it) {
        if (it.value().container) {
            it.value().container->setStyleSheet(
                it.key() == active_axis_id ? active_style : normal_style);
        }
    }
}

QString MksAllAxesControlWorkspace::decodeMksMotionStatus(const int status_code) {
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