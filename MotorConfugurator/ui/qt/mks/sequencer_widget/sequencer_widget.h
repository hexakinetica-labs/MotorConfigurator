#pragma once

#include <QWidget>
#include <QList>
#include <QPointer>

class QTableWidget;
class QPushButton;
class QTimer;
class AxisWorkspace;

namespace mks {

enum class SeqCmdType {
    MoveAbs,
    Wait,
    Loop
};

struct SeqCmd {
    SeqCmdType type;
    double param1;
    double param2; // Unused for now
};

class SequencerWidget : public QWidget {
    Q_OBJECT
public:
    explicit SequencerWidget(AxisWorkspace* workspace, QWidget* parent = nullptr);
    ~SequencerWidget() override;

private slots:
    void onAddMoveClicked();
    void onAddWaitClicked();
    void onAddLoopClicked();
    void onRemoveClicked();
    void onClearClicked();
    
    void onRunClicked();
    void onStopClicked();
    void onStepTimer();

private:
    void updateTable();

    QPointer<AxisWorkspace> workspace_;
    QTableWidget* table_{nullptr};
    
    QPushButton* btn_run_{nullptr};
    QPushButton* btn_stop_{nullptr};
    QTimer* step_timer_{nullptr};

    QList<SeqCmd> commands_;
    
    bool is_running_{false};
    int current_step_{0};
    int loop_counter_{0};
    
    int64_t wait_start_time_{0};
};

} // namespace mks
