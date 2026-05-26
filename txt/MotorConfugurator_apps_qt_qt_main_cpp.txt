#include "mks/main_window.h"
#include "mks/DarkStyle.h"
#include "init_all_drivers.h"

#include <QApplication>

int main(int argc, char* argv[]) {
    drivers::init_all_drivers();
    QApplication app(argc, argv);
    DarkStyle::apply();
    MainWindow w;
    w.show();
    return app.exec();
}
