#pragma once

#include <QMainWindow>

#include <QHash>

#include <cstdint>

class QComboBox;
class QSpinBox;
class QLineEdit;
class QTextEdit;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QCheckBox;
class QMdiArea;
class QMdiSubWindow;
class QDockWidget;
class QThread;
class QWidget;
class QLabel;

namespace mks { class AxisManager; }

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void setupUi();
    void appendLog(const QString& line);
    void openAxisWorkspace(uint16_t axis_id, const QString& axis_key);
    void openEthercatWorkspace(uint16_t axis_id);
    QTreeWidgetItem* ensureTopologyRoot();
    QTreeWidgetItem* findTransportBusNode(const QString& bus_key) const;
    void setTransportBusOpenState(const QString& transport_tag, bool opened);
    void updateTopologyTree(const QString& transport_tag, const QVariantList& ids);

    QComboBox* device_combo_{nullptr};
    QComboBox* baud_combo_{nullptr};
    QSpinBox* scan_max_id_spin_{nullptr};
    QPushButton* btn_open_{nullptr};
    QPushButton* btn_close_{nullptr};
    QPushButton* btn_load_hal_{nullptr};
    QPushButton* btn_save_hal_{nullptr};
    
    // EtherCAT Controls
    QLineEdit* ethercat_iface_edit_{nullptr};
    QPushButton* btn_ecat_open_{nullptr};
    QPushButton* btn_ecat_close_{nullptr};
    QPushButton* btn_ecat_scan_{nullptr};
    QCheckBox* chk_ecat_auto_connect_{nullptr};
    QPushButton* btn_ecat_start_runtime_{nullptr};
    
    QCheckBox* chk_client_mode_{nullptr};
    QComboBox* control_source_combo_{nullptr};
    QLabel* host_state_label_{nullptr};

    QTextEdit* log_view_{nullptr};
    QTreeWidget* topology_tree_{nullptr};
    QMdiArea* mdi_area_{nullptr};
    QDockWidget* dock_log_{nullptr};
    QLabel* status_stats_label_{nullptr};
    QLabel* mks_bus_load_label_{nullptr};
    QLabel* mks_bus_rate_label_{nullptr};
    QLabel* ecat_bus_load_label_{nullptr};
    QLabel* ecat_bus_rate_label_{nullptr};
    QLabel* hexamotion_status_label_{nullptr};
    QLabel* hexamotion_clients_label_{nullptr};
    QLabel* hexamotion_endpoint_label_{nullptr};

    QThread* manager_thread_{nullptr};
    mks::AxisManager* manager_{nullptr};

    QHash<int, QWidget*> opened_axes_;
    QHash<int, QWidget*> opened_ecat_axes_;
};
