#pragma once

#include <QWidget>

class QTreeWidget;
class QTextEdit;
class QPushButton;

class AxisWorkspaceConfigPanel final : public QWidget {
public:
    struct Handles {
        QTreeWidget* config_tree{nullptr};
        QTextEdit* txt_description{nullptr};
        QPushButton* btn_refresh_list{nullptr};
        QPushButton* btn_read_params{nullptr};
        QPushButton* btn_apply_params{nullptr};
        QPushButton* btn_save_drive_flash{nullptr};
        QPushButton* btn_export_full{nullptr};
        QPushButton* btn_import_full{nullptr};
    };

    explicit AxisWorkspaceConfigPanel(QWidget* parent = nullptr);

    const Handles& handles() const noexcept;

private:
    Handles handles_{};
};