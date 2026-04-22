#include "mks/axis_workspace/axis_workspace_config_panel.h"

#include <QAbstractItemView>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStyledItemDelegate>
#include <QTextEdit>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace {

constexpr int kReadOnlyColumn = 3;
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
        if (read_only.compare(QStringLiteral("Yes"), Qt::CaseInsensitive) == 0 ||
            read_only.compare(QStringLiteral("N/A"), Qt::CaseInsensitive) == 0) {
            return nullptr;
        }

        return QStyledItemDelegate::createEditor(parent, option, index);
    }
};

} // namespace

AxisWorkspaceConfigPanel::AxisWorkspaceConfigPanel(QWidget* parent)
    : QWidget(parent) {
    auto* root_layout = new QVBoxLayout(this);
    root_layout->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* scroll_content = new QWidget(scroll);
    auto* layout = new QVBoxLayout(scroll_content);

    auto* btn_row = new QHBoxLayout();
    handles_.btn_refresh_list = new QPushButton(QStringLiteral("Refresh List"), scroll_content);
    handles_.btn_read_params = new QPushButton(QStringLiteral("Read Values"), scroll_content);
    handles_.btn_apply_params = new QPushButton(QStringLiteral("Apply Changes"), scroll_content);
    handles_.btn_save_drive_flash = new QPushButton(QStringLiteral("Save Selected Persistently"), scroll_content);
    handles_.btn_export_full = new QPushButton(QStringLiteral("Export Config (AxisConfig)"), scroll_content);
    handles_.btn_import_full = new QPushButton(QStringLiteral("Import Config (AxisConfig)"), scroll_content);
    btn_row->addWidget(handles_.btn_refresh_list);
    btn_row->addWidget(handles_.btn_read_params);
    btn_row->addWidget(handles_.btn_apply_params);
    btn_row->addWidget(handles_.btn_save_drive_flash);
    btn_row->addWidget(handles_.btn_export_full);
    btn_row->addWidget(handles_.btn_import_full);
    btn_row->addStretch();
    layout->addLayout(btn_row);

    auto* cfg_note = new QLabel(
        QStringLiteral("<b>Config model:</b> AxisConfig import/export is the canonical configuration flow."),
        scroll_content);
    cfg_note->setWordWrap(true);
    layout->addWidget(cfg_note);

    auto* split = new QSplitter(Qt::Horizontal, scroll_content);

    handles_.config_tree = new QTreeWidget(split);
    handles_.config_tree->setColumnCount(6);
    handles_.config_tree->setHeaderLabels({QStringLiteral("Name"),
                                           QStringLiteral("Group"),
                                           QStringLiteral("Unit"),
                                           QStringLiteral("Read Only"),
                                           QStringLiteral("Current Value"),
                                           QStringLiteral("New Value")});
    handles_.config_tree->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    handles_.config_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    handles_.config_tree->setSelectionBehavior(QAbstractItemView::SelectRows);
    handles_.config_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    handles_.config_tree->setEditTriggers(QAbstractItemView::DoubleClicked |
                                          QAbstractItemView::SelectedClicked |
                                          QAbstractItemView::EditKeyPressed);
    handles_.config_tree->setItemDelegateForColumn(kNewValueColumn,
                                                   new NewValueColumnDelegate(handles_.config_tree));

    handles_.txt_description = new QTextEdit(split);
    handles_.txt_description->setReadOnly(true);
    handles_.txt_description->setPlaceholderText(QStringLiteral("Select a parameter to view its details..."));

    split->setSizes({600, 200});
    layout->addWidget(split, 1);

    scroll->setWidget(scroll_content);
    root_layout->addWidget(scroll);
}

const AxisWorkspaceConfigPanel::Handles& AxisWorkspaceConfigPanel::handles() const noexcept {
    return handles_;
}