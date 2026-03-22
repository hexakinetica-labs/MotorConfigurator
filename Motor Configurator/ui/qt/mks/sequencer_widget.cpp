#include "mks/sequencer_widget.h"
#include "mks/axis_workspace.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QHeaderView>
#include <QDateTime>
#include <QMessageBox>
#include <QTableWidget>
#include <QPushButton>
#include <QTimer>

namespace mks {

SequencerWidget::SequencerWidget(AxisWorkspace* workspace, QWidget* parent)
    : QWidget(parent)
    , workspace_(workspace)
{
    auto* layout = new QVBoxLayout(this);
    
    // Toolbar
    auto* toolbar = new QHBoxLayout();
    auto* btn_add_move = new QPushButton("Add Move");
    auto* btn_add_wait = new QPushButton("Add Wait");
    auto* btn_add_loop = new QPushButton("Add Loop");
    auto* btn_remove = new QPushButton("Remove");
    auto* btn_clear = new QPushButton("Clear");
    
    toolbar->addWidget(btn_add_move);
    toolbar->addWidget(btn_add_wait);
    toolbar->addWidget(btn_add_loop);
    toolbar->addWidget(btn_remove);
    toolbar->addWidget(btn_clear);
    toolbar->addStretch();
    
    layout->addLayout(toolbar);
    
    // Table
    table_ = new QTableWidget();
    table_->setColumnCount(2);
    table_->setHorizontalHeaderLabels({"Command", "Parameters"});
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    layout->addWidget(table_);
    
    // Control
    auto* ctrl_layout = new QHBoxLayout();
    btn_run_ = new QPushButton("RUN SEQUENCE");
    btn_stop_ = new QPushButton("STOP");
    btn_stop_->setEnabled(false);
    
    btn_run_->setStyleSheet("background-color: #2ea043; color: white; font-weight: bold; padding: 10px;");
    btn_stop_->setStyleSheet("background-color: #da3633; color: white; font-weight: bold; padding: 10px;");
    
    ctrl_layout->addWidget(btn_run_);
    ctrl_layout->addWidget(btn_stop_);
    layout->addLayout(ctrl_layout);
    
    // Connections
    connect(btn_add_move, &QPushButton::clicked, this, &SequencerWidget::onAddMoveClicked);
    connect(btn_add_wait, &QPushButton::clicked, this, &SequencerWidget::onAddWaitClicked);
    connect(btn_add_loop, &QPushButton::clicked, this, &SequencerWidget::onAddLoopClicked);
    connect(btn_remove, &QPushButton::clicked, this, &SequencerWidget::onRemoveClicked);
    connect(btn_clear, &QPushButton::clicked, this, &SequencerWidget::onClearClicked);
    
    connect(btn_run_, &QPushButton::clicked, this, &SequencerWidget::onRunClicked);
    connect(btn_stop_, &QPushButton::clicked, this, &SequencerWidget::onStopClicked);
    
    step_timer_ = new QTimer(this);
    connect(step_timer_, &QTimer::timeout, this, &SequencerWidget::onStepTimer);
}

SequencerWidget::~SequencerWidget() {
    if (step_timer_) step_timer_->stop();
}

void SequencerWidget::onAddMoveClicked() {
    bool ok;
    double pos = QInputDialog::getDouble(this, "Add Move", "Target Position (deg):", 0, -360000, 360000, 1, &ok);
    if (ok) {
        commands_.append({SeqCmdType::MoveAbs, pos, 0.0});
        updateTable();
    }
}

void SequencerWidget::onAddWaitClicked() {
    bool ok;
    int ms = QInputDialog::getInt(this, "Add Wait", "Wait Time (ms):", 1000, 0, 60000, 100, &ok);
    if (ok) {
        commands_.append({SeqCmdType::Wait, static_cast<double>(ms), 0.0});
        updateTable();
    }
}

void SequencerWidget::onAddLoopClicked() {
    bool ok;
    int count = QInputDialog::getInt(this, "Add Loop", "Loop Count (0=Infinite):", 1, 0, 1000, 1, &ok);
    if (ok) {
        commands_.append({SeqCmdType::Loop, static_cast<double>(count), 0.0});
        updateTable();
    }
}

void SequencerWidget::onRemoveClicked() {
    int row = table_->currentRow();
    if (row >= 0 && row < commands_.size()) {
        commands_.removeAt(row);
        updateTable();
    }
}

void SequencerWidget::onClearClicked() {
    commands_.clear();
    updateTable();
}

void SequencerWidget::updateTable() {
    table_->setRowCount(commands_.size());
    for (int i = 0; i < commands_.size(); ++i) {
        const auto& cmd = commands_[i];
        QString typeStr;
        QString paramStr;
        
        switch (cmd.type) {
            case SeqCmdType::MoveAbs:
                typeStr = "MOVE ABS";
                paramStr = QString("Pos: %1 deg").arg(cmd.param1);
                break;
            case SeqCmdType::Wait:
                typeStr = "WAIT";
                paramStr = QString("Time: %1 ms").arg(cmd.param1);
                break;
            case SeqCmdType::Loop:
                typeStr = "LOOP";
                paramStr = QString("Restart Count: %1").arg(static_cast<int>(cmd.param1));
                break;
        }
        
        table_->setItem(i, 0, new QTableWidgetItem(typeStr));
        table_->setItem(i, 1, new QTableWidgetItem(paramStr));
    }
}

void SequencerWidget::onRunClicked() {
    if (commands_.isEmpty()) return;
    
    current_step_ = 0;
    loop_counter_ = 0;
    is_running_ = true;
    wait_start_time_ = 0;
    
    btn_run_->setEnabled(false);
    btn_stop_->setEnabled(true);
    table_->setEnabled(false);
    
    step_timer_->start(50); // Logic tick every 50ms
}

void SequencerWidget::onStopClicked() {
    is_running_ = false;
    step_timer_->stop();
    
    btn_run_->setEnabled(true);
    btn_stop_->setEnabled(false);
    table_->setEnabled(true);
    
    table_->clearSelection();
}

void SequencerWidget::onStepTimer() {
    if (!is_running_ || !workspace_) return;
    
    if (current_step_ >= commands_.size()) {
        onStopClicked();
        QMessageBox::information(this, "Sequencer", "Sequence Completed.");
        return;
    }
    
    const auto& cmd = commands_[current_step_];
    table_->selectRow(current_step_);
    
    switch (cmd.type) {
        case SeqCmdType::MoveAbs: {
            if (wait_start_time_ == 0) { 
                workspace_->setTargetPosition(cmd.param1);
                workspace_->triggerAbsoluteMove();
                wait_start_time_ = QDateTime::currentMSecsSinceEpoch(); 
            }
            
            // Checking if target is reached. Wait at least 100ms before checking to allow trajectory to start.
            if ((QDateTime::currentMSecsSinceEpoch() - wait_start_time_) > 100) {
                if (workspace_->isTargetReached(1.0)) { // 1 degree tolerance
                    wait_start_time_ = 0;
                    current_step_++;
                }
            }
            break;
        }
        case SeqCmdType::Wait: {
            if (wait_start_time_ == 0) {
                wait_start_time_ = QDateTime::currentMSecsSinceEpoch();
            }
            
            if (QDateTime::currentMSecsSinceEpoch() - wait_start_time_ >= cmd.param1) {
                wait_start_time_ = 0;
                current_step_++;
            }
            break;
        }
        case SeqCmdType::Loop: {
            int max_loops = static_cast<int>(cmd.param1);
            if (max_loops == 0 || loop_counter_ < max_loops) {
                current_step_ = 0;
                loop_counter_++;
                wait_start_time_ = 0;
            } else {
                current_step_++; // Exit loop
            }
            break;
        }
    }
}

} // namespace mks
