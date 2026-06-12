#include "mks/main_window/main_window.h"
#include "mks/styles/DarkStyle.h"
#include "init_all_drivers.h"

#include "hal_host_service/hal_host_service.h"

#include <QApplication>
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "Starting MotorConfigurator Version 1.2.5" << std::endl;
    drivers::init_all_drivers();
    QApplication app(argc, argv);
    DarkStyle::apply();

    // Instantiate the application facade
    hal_host::HalHostService host_service;

    MainWindow w(host_service);
    w.show();
    return app.exec();
}
