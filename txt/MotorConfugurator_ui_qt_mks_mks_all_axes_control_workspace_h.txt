#pragma once

#include <QHash>
#include <QPointer>
#include <QWidget>

#include <cstdint>

class QLabel;
class QProgressBar;
class QPushButton;
class QScrollArea;
class QSlider;
class QVBoxLayout;

namespace mks {
class AxisManager;
}

class MksAllAxesControlWorkspace final : public QWidget {
    Q_OBJECT

public:
    explicit MksAllAxesControlWorkspace(mks::AxisManager* manager, QWidget* parent = nullptr);
    ~MksAllAxesControlWorkspace() override;

    void setAxisIds(const QList<int>& axis_ids);

private slots:
    void onTelemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void onHostStateUpdated(const QVariantMap& state);
    void onScanFinished(const QString& transport_tag, const QVariantList& axis_ids);

private:
    struct AxisRowWidgets {
        QWidget* container{nullptr};
        QLabel* title{nullptr};
        QLabel* position{nullptr};
        QLabel* status{nullptr};
        QLabel* slider_value{nullptr};
        QSlider* slider{nullptr};
        QPushButton* btn_enable{nullptr};
        QPushButton* btn_disable{nullptr};
        QPushButton* btn_home{nullptr};
        QPushButton* btn_set_zero{nullptr};
        qint64 manual_slider_hold_until_ms{0};
        QString homing_status{QStringLiteral("Idle")};
        int motion_status_code{0};
    };

    void setupUi();
    void rebuildAxisRows();
    void updateUiState();
    void watchAxis(int axis_id, bool enabled);
    void sendServiceCommand(int axis_id, int kind);
    void sendAbsoluteMove(int axis_id, double target_deg);
    void startHomingSequence();
    void stopHomingSequence();
    void updateSequenceStatusLabel(const QString& text);
    void applySequenceHighlight(int active_axis_id);
    static QString decodeMksMotionStatus(int status_code);

    QPointer<mks::AxisManager> manager_;
    QScrollArea* scroll_area_{nullptr};
    QWidget* axis_list_container_{nullptr};
    QVBoxLayout* axis_list_layout_{nullptr};
    QPushButton* btn_enable_all_{nullptr};
    QPushButton* btn_disable_all_{nullptr};
    QPushButton* btn_set_zero_all_{nullptr};
    QPushButton* btn_estop_all_{nullptr};
    QPushButton* btn_start_sequence_{nullptr};
    QPushButton* btn_stop_sequence_{nullptr};
    QLabel* lbl_sequence_status_{nullptr};
    QProgressBar* progress_sequence_{nullptr};

    QList<int> axis_ids_{};
    QHash<int, AxisRowWidgets> axis_rows_{};

    bool ui_manual_control_available_{true};
    bool estop_active_{false};
    bool mks_homing_sequence_ui_lock_active_{false};
    bool sequence_running_{false};
    int sequence_current_axis_id_{-1};
};