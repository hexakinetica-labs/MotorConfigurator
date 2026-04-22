#include "mks/main_window.h"

#include "mks/axis_manager.h"
#include "mks/axis_workspace/ethercat_axis_workspace.h"
#include "mks/axis_workspace/mks_axis_workspace.h"
#include "mks_can/internal/port/gs_usb_can_port.h"

#include <QDateTime>
#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QToolBox>
#include <QTextEdit>
#include <QThread>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QCheckBox>
#include <QStatusBar>
#include <QFileDialog>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUi();
    if (scan_max_id_spin_) {
        scan_max_id_spin_->setValue(10);
    }

    manager_thread_ = new QThread(this);
    manager_ = new mks::AxisManager();
    manager_->moveToThread(manager_thread_);
    connect(manager_thread_, &QThread::finished, manager_, &QObject::deleteLater);
    manager_thread_->start();

    connect(manager_, &mks::AxisManager::logMessage, this,
            [this](const QString& tag, const QString& s) {
                const QString prefix = tag.isEmpty() ? QString{} : "[" + tag.toUpper() + "] ";
                appendLog(prefix + s);
            });
    
    connect(manager_, &mks::AxisManager::busStatisticsUpdated, this, [this](const QVariantMap& bus_stats) {
        bool has_mks = false;
        bool has_ecat = false;
        
        for (auto it = bus_stats.constBegin(); it != bus_stats.constEnd(); ++it) {
            const QString& bus_name = it.key();
            const QVariantMap& stats = it.value().toMap();
            const double cycle_rate_hz = stats["cycle_rate_hz"].toDouble();
            const double bus_load_percent = stats["bus_load_percent"].toDouble();

            if (bus_name.contains("MKS")) {
                has_mks = true;
                if (mks_bus_load_label_) mks_bus_load_label_->setText(QString::number(bus_load_percent, 'f', 1) + "%");
                if (mks_bus_rate_label_) mks_bus_rate_label_->setText(QString::number(cycle_rate_hz, 'f', 1) + " Hz");
            } else if (bus_name.contains("EtherCAT")) {
                has_ecat = true;
                if (ecat_bus_load_label_) ecat_bus_load_label_->setText(QString::number(bus_load_percent, 'f', 1) + "%");
                if (ecat_bus_rate_label_) ecat_bus_rate_label_->setText(QString::number(cycle_rate_hz, 'f', 1) + " Hz");
            }
        }
        
        if (!has_mks) {
            if (mks_bus_load_label_) mks_bus_load_label_->setText("--- %");
            if (mks_bus_rate_label_) mks_bus_rate_label_->setText("--- Hz");
        }
        if (!has_ecat) {
            if (ecat_bus_load_label_) ecat_bus_load_label_->setText("--- %");
            if (ecat_bus_rate_label_) ecat_bus_rate_label_->setText("--- Hz");
        }
        if (status_stats_label_) {
            status_stats_label_->setText(QString("Active Buses: %1").arg(bus_stats.size()));
        }
    });

    connect(manager_, &mks::AxisManager::scanFinished, this,
            [this](const QString& tag, const QVariantList& ids) {
                updateTopologyTree(tag, ids);
            });
    connect(manager_, &mks::AxisManager::transportOpenStateChanged, this,
            [this](const QString& tag, const bool opened) {
                setTransportBusOpenState(tag, opened);
            });
            
    connect(manager_, &mks::AxisManager::manualTakeoverChanged, this, [this](const bool active) {
        if (!control_source_combo_) {
            return;
        }
        const int target_index = active ? 0 : 1;
        if (control_source_combo_->currentIndex() != target_index) {
            control_source_combo_->blockSignals(true);
            control_source_combo_->setCurrentIndex(target_index);
            control_source_combo_->blockSignals(false);
        }
    });
    connect(manager_, &mks::AxisManager::hostStateUpdated, this, [this](const QVariantMap& state) {
        const QString control_source = state.value(QStringLiteral("control_source")).toString();
        const bool estop_active = state.value(QStringLiteral("estop_active")).toBool();
        const QString motion_owner = state.value(QStringLiteral("motion_owner")).toString();

        if (control_source_combo_) {
            const int target_index = (control_source == QStringLiteral("ui")) ? 0 : 1;
            if (control_source_combo_->currentIndex() != target_index) {
                control_source_combo_->blockSignals(true);
                control_source_combo_->setCurrentIndex(target_index);
                control_source_combo_->blockSignals(false);
            }
        }

        if (host_state_label_) {
            host_state_label_->setText(
                QStringLiteral("Owner=%1 | E-Stop=%2")
                    .arg(motion_owner.isEmpty() ? QStringLiteral("none") : motion_owner)
                    .arg(estop_active ? QStringLiteral("ACTIVE") : QStringLiteral("off")));
            host_state_label_->setStyleSheet(estop_active
                ? QStringLiteral("color:#da3633; font-weight:700;")
                : QString());
        }

        // Feature 7: Update HexaMotion status labels and topology tree
        const bool ipc_running = state.value(QStringLiteral("hexamotion_ipc_running")).toBool();
        const int client_count = state.value(QStringLiteral("hexamotion_connected_clients")).toInt();

        if (hexamotion_status_label_) {
            hexamotion_status_label_->setText(ipc_running ? QStringLiteral("Listening") : QStringLiteral("Stopped"));
            hexamotion_status_label_->setStyleSheet(ipc_running
                ? QStringLiteral("color:#3fb950; font-weight:700;")
                : QStringLiteral("color:#8b949e;"));
        }
        if (hexamotion_clients_label_) {
            hexamotion_clients_label_->setText(QString::number(client_count));
            hexamotion_clients_label_->setStyleSheet(client_count > 0
                ? QStringLiteral("color:#3fb950; font-weight:700;")
                : QString());
        }

        // Topology tree: show/hide HexaMotion node
        if (topology_tree_) {
            auto* bus = findTransportBusNode(QStringLiteral("hexamotion_bus"));
            if (ipc_running && client_count > 0) {
                if (!bus) {
                    auto* root = ensureTopologyRoot();
                    if (root) {
                        bus = new QTreeWidgetItem(root);
                        bus->setData(0, Qt::UserRole, QStringLiteral("hexamotion_bus"));
                        bus->setExpanded(true);
                    }
                }
                if (bus) {
                    bus->setText(0, QStringLiteral("HexaMotion Client (%1 connected)").arg(client_count));
                }
            } else if (bus) {
                auto* root = bus->parent();
                if (root) {
                    root->removeChild(bus);
                    delete bus;
                    if (root->childCount() == 0) {
                        delete root;
                    }
                }
            }
        }
    });

    connect(btn_open_, &QPushButton::clicked, this, [this]() {
        const QString dev = device_combo_->currentData().toString();
        const int baud = baud_combo_->currentData().toInt();
        QMetaObject::invokeMethod(manager_, "openDevice", Qt::QueuedConnection,
                                  Q_ARG(QString, dev), Q_ARG(int, baud));
    });

    connect(btn_close_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "closeMksDevice", Qt::QueuedConnection);
    });

    connect(btn_load_hal_, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, "Load Master HAL Config", "", "JSON Files (*.json)");
        if (!path.isEmpty()) {
            QMetaObject::invokeMethod(manager_, "loadHalConfig", Qt::QueuedConnection, Q_ARG(QString, path));
        }
    });

    connect(btn_save_hal_, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getSaveFileName(this, "Save Master HAL Config", "master_config.json", "JSON Files (*.json)");
        if (!path.isEmpty()) {
            QMetaObject::invokeMethod(manager_, "saveHalConfig", Qt::QueuedConnection, Q_ARG(QString, path));
        }
    });

    connect(findChild<QPushButton*>("btn_refresh"), &QPushButton::clicked, this, [this]() {
        const QString prev_selected = device_combo_->currentData().toString();
        device_combo_->clear();

        // Strict operator list: only Simulator + detected GS-USB devices.
        device_combo_->setEditable(false);
        device_combo_->addItem("Simulator (virtual MKS bus)", QString("sim:default"));

        int gs_default_index = -1;
        const auto gs_usb_ports = mks::GsUsbCanPort::enumerateDevices();
        for (const auto& port : gs_usb_ports) {
            device_combo_->addItem(QString::fromStdString(port.description), QString::fromStdString(port.path));
            if (gs_default_index < 0) {
                gs_default_index = device_combo_->count() - 1;
            }
        }

        int selected_index = (gs_default_index >= 0) ? gs_default_index : 0;
        if (!prev_selected.isEmpty()) {
            int idx = device_combo_->findData(prev_selected);
            if (idx >= 0) {
                selected_index = idx;
            }
        }

        device_combo_->setCurrentIndex(selected_index);

        appendLog(QString("Devices refreshed. (selected: %1)")
                      .arg(device_combo_->currentText()));
    });

    // MKS Scan Button connection
    connect(findChild<QPushButton*>("btn_scan"), &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "scanMotors", Qt::QueuedConnection,
                                  Q_ARG(int, scan_max_id_spin_->value()));
    });

    // EtherCAT Panel Connections
    connect(btn_ecat_open_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "openEthercatDevice", Qt::QueuedConnection,
                                  Q_ARG(QString, ethercat_iface_edit_->text()));
    });
    connect(btn_ecat_scan_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "scanEthercatMotors", Qt::QueuedConnection);
    });
    connect(btn_ecat_close_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "closeEthercatDevice", Qt::QueuedConnection);
    });
    connect(btn_ecat_start_runtime_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "startRuntime", Qt::QueuedConnection);
    });

    findChild<QPushButton*>("btn_refresh")->click();
}

MainWindow::~MainWindow() {
    if (manager_thread_) {
        manager_thread_->quit();
        manager_thread_->wait();
    }
}

void MainWindow::setupUi() {
    setWindowTitle("HexaLabs Motor Configurator");
    resize(1300, 850);

    mdi_area_ = new QMdiArea(this);
    mdi_area_->setViewMode(QMdiArea::TabbedView);
    mdi_area_->setTabsClosable(true);
    setCentralWidget(mdi_area_);

    auto* dock_network = new QDockWidget("Network", this);
    dock_network->setMinimumWidth(250);
    dock_network->setMaximumWidth(350);
    auto* network_host = new QWidget(dock_network);
    auto* network_layout = new QVBoxLayout(network_host);

    chk_client_mode_ = new QCheckBox("Canonical Single-App HAL Path", network_host);
    chk_client_mode_->setChecked(true);
    chk_client_mode_->setEnabled(false);
    chk_client_mode_->setToolTip("HAL is embedded into MotorTester GUI by default. External host process is not required for normal operation.");
    network_layout->addWidget(chk_client_mode_);

    auto* control_source_row = new QHBoxLayout();
    auto* control_source_label = new QLabel("Control Source:", network_host);
    control_source_combo_ = new QComboBox(network_host);
    control_source_combo_->addItem(QStringLiteral("UI (Manual)"));
    control_source_combo_->addItem(QStringLiteral("HexaMotion"));
    control_source_combo_->setCurrentIndex(0);
    control_source_combo_->setToolTip("Switch global motion owner between local UI and HexaMotion client.");
    control_source_row->addWidget(control_source_label);
    control_source_row->addWidget(control_source_combo_, 1);
    network_layout->addLayout(control_source_row);

    host_state_label_ = new QLabel("Owner=none | E-Stop=off", network_host);
    network_layout->addWidget(host_state_label_);

    connect(control_source_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](const int index) {
        QMetaObject::invokeMethod(manager_,
                                  "requestManualTakeover",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, index == 0));
    });

    auto* conn_tabs = new QToolBox(network_host);
    network_layout->addWidget(conn_tabs);

    // MKS CAN Tab
    auto* mks_tab = new QWidget(conn_tabs);
    auto* mks_layout = new QVBoxLayout(mks_tab);
    
    auto* mks_form = new QFormLayout();
    device_combo_ = new QComboBox(mks_tab);
    baud_combo_ = new QComboBox(mks_tab);
    baud_combo_->addItem("125000", 125000);
    baud_combo_->addItem("250000", 250000);
    baud_combo_->addItem("500000", 500000);
    baud_combo_->addItem("1000000", 1000000);
    baud_combo_->setCurrentText("500000");
    scan_max_id_spin_ = new QSpinBox(mks_tab);
    scan_max_id_spin_->setRange(1, 2047);
    scan_max_id_spin_->setValue(10);
    
    mks_form->addRow("Interface:", device_combo_);
    mks_form->addRow("Baudrate:", baud_combo_);
    mks_form->addRow("Max Node ID:", scan_max_id_spin_);
    mks_layout->addLayout(mks_form);

    auto* btn_mks_row1 = new QHBoxLayout();
    btn_open_ = new QPushButton("Open Master", mks_tab);
    auto* btn_refresh = new QPushButton("Refresh List", mks_tab);
    btn_refresh->setObjectName("btn_refresh");
    btn_mks_row1->addWidget(btn_open_);
    btn_mks_row1->addWidget(btn_refresh);
    mks_layout->addLayout(btn_mks_row1);

    auto* btn_mks_row2 = new QHBoxLayout();
    auto* btn_scan = new QPushButton("Scan Network", mks_tab);
    btn_scan->setObjectName("btn_scan");
    btn_close_ = new QPushButton("Close Master", mks_tab);
    btn_mks_row2->addWidget(btn_scan);
    btn_mks_row2->addWidget(btn_close_);
    mks_layout->addLayout(btn_mks_row2);

    auto* mks_stats_group = new QGroupBox("Bus Statistics", mks_tab);
    auto* mks_stats_form = new QFormLayout(mks_stats_group);
    mks_bus_load_label_ = new QLabel("--- %", mks_stats_group);
    mks_bus_rate_label_ = new QLabel("--- Hz", mks_stats_group);
    mks_stats_form->addRow("Load:", mks_bus_load_label_);
    mks_stats_form->addRow("Cycle rate:", mks_bus_rate_label_);
    mks_layout->addWidget(mks_stats_group);

    mks_layout->addStretch();
    conn_tabs->addItem(mks_tab, "MKS CAN Interface");

    // EtherCAT Tab
    auto* ecat_tab = new QWidget(conn_tabs);
    auto* ecat_layout = new QVBoxLayout(ecat_tab);
    
    auto* ecat_form = new QFormLayout();
    ethercat_iface_edit_ = new QLineEdit("enp2s0", ecat_tab);
    ecat_form->addRow("Interface:", ethercat_iface_edit_);
    
    chk_ecat_auto_connect_ = new QCheckBox("Auto-Connect Axes", ecat_tab);
    chk_ecat_auto_connect_->setChecked(true);
    ecat_form->addRow("", chk_ecat_auto_connect_);
    ecat_layout->addLayout(ecat_form);
    
    auto* ecat_btn_row1 = new QHBoxLayout();
    btn_ecat_open_ = new QPushButton("Open Master", ecat_tab);
    btn_ecat_start_runtime_ = new QPushButton("Connect Axes", ecat_tab);
    ecat_btn_row1->addWidget(btn_ecat_open_);
    ecat_btn_row1->addWidget(btn_ecat_start_runtime_);
    ecat_layout->addLayout(ecat_btn_row1);

    auto* ecat_btn_row2 = new QHBoxLayout();
    btn_ecat_scan_ = new QPushButton("Scan Network", ecat_tab);
    btn_ecat_close_ = new QPushButton("Close Master", ecat_tab);
    ecat_btn_row2->addWidget(btn_ecat_scan_);
    ecat_btn_row2->addWidget(btn_ecat_close_);
    ecat_layout->addLayout(ecat_btn_row2);

    auto* ecat_stats_group = new QGroupBox("Bus Statistics", ecat_tab);
    auto* ecat_stats_form = new QFormLayout(ecat_stats_group);
    ecat_bus_load_label_ = new QLabel("--- %", ecat_stats_group);
    ecat_bus_rate_label_ = new QLabel("--- Hz", ecat_stats_group);
    ecat_stats_form->addRow("Load:", ecat_bus_load_label_);
    ecat_stats_form->addRow("Cycle rate:", ecat_bus_rate_label_);
    ecat_layout->addWidget(ecat_stats_group);

    ecat_layout->addStretch();
    conn_tabs->addItem(ecat_tab, "EtherCAT Interface");

    // HexaMotion Client Tab (Feature 7)
    auto* hexamotion_tab = new QWidget(conn_tabs);
    auto* hexamotion_layout = new QVBoxLayout(hexamotion_tab);

    auto* hexamotion_form = new QFormLayout();
    hexamotion_endpoint_label_ = new QLabel("127.0.0.1:30110", hexamotion_tab);
    hexamotion_endpoint_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    hexamotion_form->addRow("Endpoint:", hexamotion_endpoint_label_);
    hexamotion_status_label_ = new QLabel("Stopped", hexamotion_tab);
    hexamotion_form->addRow("IPC Server:", hexamotion_status_label_);
    hexamotion_clients_label_ = new QLabel("0", hexamotion_tab);
    hexamotion_form->addRow("Connected Clients:", hexamotion_clients_label_);
    hexamotion_layout->addLayout(hexamotion_form);

    hexamotion_layout->addStretch();
    conn_tabs->addItem(hexamotion_tab, "HexaMotion Client");

    auto* btn_hal_row = new QHBoxLayout();
    btn_load_hal_ = new QPushButton("Load Config...", network_host);
    btn_save_hal_ = new QPushButton("Save Config...", network_host);
    btn_hal_row->addWidget(btn_load_hal_);
    btn_hal_row->addWidget(btn_save_hal_);
    network_layout->addLayout(btn_hal_row);

    topology_tree_ = new QTreeWidget(this);
    topology_tree_->setHeaderLabel("Deployment Topology");
    topology_tree_->setContextMenuPolicy(Qt::DefaultContextMenu);
    network_layout->addWidget(topology_tree_, 1);

    connect(topology_tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int) {
        if (!item) return;
        const QString axis_key = item->data(0, Qt::UserRole).toString();
        if (axis_key == "mks_axis" || axis_key == "ecat_axis") {
            bool ok = false;
            const int id = item->data(0, Qt::UserRole + 1).toInt(&ok);
            if (ok) openAxisWorkspace(static_cast<uint16_t>(id), axis_key);
        }
    });

    dock_network->setWidget(network_host);
    addDockWidget(Qt::LeftDockWidgetArea, dock_network);

    dock_log_ = new QDockWidget("Log", this);
    log_view_ = new QTextEdit(this);
    log_view_->setReadOnly(true);
    dock_log_->setWidget(log_view_);
    addDockWidget(Qt::BottomDockWidgetArea, dock_log_);

    status_stats_label_ = new QLabel("Bus Load: ---% | Cycle Rate: --- Hz", this);
    statusBar()->addPermanentWidget(status_stats_label_);
}

void MainWindow::appendLog(const QString& line) {
    if (!log_view_) return;
    log_view_->append(QDateTime::currentDateTime().toString("HH:mm:ss.zzz") + " | " + line);
}

void MainWindow::openAxisWorkspace(uint16_t axis_id, const QString& axis_key) {
    if (opened_axes_.contains(axis_id) && opened_axes_[axis_id]) {
        if (auto* sub = qobject_cast<QMdiSubWindow*>(opened_axes_[axis_id]->parentWidget())) {
            mdi_area_->setActiveSubWindow(sub);
            return;
        }
    }

    AxisWorkspace* ws = nullptr;
    if (axis_key == "mks_axis") {
        ws = new MksAxisWorkspace(static_cast<int>(axis_id), manager_);
    } else if (axis_key == "ecat_axis") {
        ws = new EthercatAxisWorkspace(static_cast<int>(axis_id), manager_);
    } else {
        appendLog(QString("Unknown axis type for axis %1: %2").arg(axis_id).arg(axis_key));
        return;
    }

    connect(ws, &QObject::destroyed, this, [this, axis_id]() { opened_axes_.remove(axis_id); });
    auto* sub = mdi_area_->addSubWindow(ws);
    sub->setAttribute(Qt::WA_DeleteOnClose, true);
    sub->setWindowTitle(QString("Axis %1").arg(axis_id));
    sub->resize(860, 620);
    sub->show();
    opened_axes_.insert(axis_id, ws);
}

QTreeWidgetItem* MainWindow::ensureTopologyRoot() {
    if (!topology_tree_) {
        return nullptr;
    }

    QTreeWidgetItem* root = topology_tree_->topLevelItemCount() == 0
        ? new QTreeWidgetItem(topology_tree_)
        : topology_tree_->topLevelItem(0);
    root->setText(0, QStringLiteral("HexaLabs Runtime"));
    root->setExpanded(true);
    return root;
}

QTreeWidgetItem* MainWindow::findTransportBusNode(const QString& bus_key) const {
    if (!topology_tree_ || topology_tree_->topLevelItemCount() == 0) {
        return nullptr;
    }

    auto* root = topology_tree_->topLevelItem(0);
    if (!root) {
        return nullptr;
    }

    for (int i = 0; i < root->childCount(); ++i) {
        auto* child = root->child(i);
        if (child && child->data(0, Qt::UserRole).toString() == bus_key) {
            return child;
        }
    }

    return nullptr;
}

void MainWindow::setTransportBusOpenState(const QString& transport_tag, const bool opened) {
    if (!topology_tree_) {
        return;
    }

    const bool is_ecat = (transport_tag == QStringLiteral("ecat"));
    const QString bus_key = is_ecat ? QStringLiteral("ecat_bus") : QStringLiteral("mks_bus");
    const QString bus_label = is_ecat ? QStringLiteral("EtherCAT Bus") : QStringLiteral("MKS CAN Bus");

    auto* bus = findTransportBusNode(bus_key);
    if (opened) {
        if (!bus) {
            auto* root = ensureTopologyRoot();
            if (!root) {
                return;
            }
            bus = new QTreeWidgetItem(root);
            bus->setData(0, Qt::UserRole, bus_key);
            bus->setExpanded(true);
        }
        bus->setText(0, bus_label);
        return;
    }

    if (!bus) {
        return;
    }

    auto* root = bus->parent();
    if (!root) {
        return;
    }

    root->removeChild(bus);
    delete bus;

    if (root->childCount() == 0) {
        delete root;
    }
}

void MainWindow::updateTopologyTree(const QString& transport_tag, const QVariantList& ids) {
    if (!topology_tree_) return;
    const bool is_ecat = (transport_tag == "ecat");
    const QString bus_key        = is_ecat ? "ecat_bus"     : "mks_bus";
    const QString bus_label      = is_ecat ? "EtherCAT Bus" : "MKS CAN Bus";
    const QString axis_key       = is_ecat ? "ecat_axis"    : "mks_axis";
    const QString axis_label_pfx = is_ecat ? "ECAT Axis"    : "MKS Axis";

    QTreeWidgetItem* bus = findTransportBusNode(bus_key);
    if (!bus) {
        if (ids.isEmpty()) {
            return;
        }
        auto* root = ensureTopologyRoot();
        if (!root) {
            return;
        }
        bus = new QTreeWidgetItem(root);
        bus->setText(0, bus_label);
        bus->setData(0, Qt::UserRole, bus_key);
        bus->setExpanded(true);
    } else {
        qDeleteAll(bus->takeChildren());
    }
    for (const auto& id_var : ids) {
        const int id = id_var.toInt();
        auto* item = new QTreeWidgetItem(bus);
        item->setText(0, QString("%1 %2 (Online)").arg(axis_label_pfx).arg(id));
        item->setData(0, Qt::UserRole, axis_key);
        item->setData(0, Qt::UserRole + 1, id);
    }
    if (is_ecat && chk_ecat_auto_connect_ && chk_ecat_auto_connect_->isChecked()) {
        if (btn_ecat_start_runtime_) btn_ecat_start_runtime_->click();
    }
}

