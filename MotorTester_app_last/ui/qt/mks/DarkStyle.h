#ifndef DARKSTYLE_H
#define DARKSTYLE_H

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>
#include <QFont>

class DarkStyle {
public:
    static void apply() {
        qApp->setStyle(QStyleFactory::create("Fusion"));

        QPalette darkPalette;
        QColor darkGray(45, 45, 48);
        QColor gray(90, 90, 90);
        QColor lightGray(240, 240, 240);
        QColor blueAccent(0, 122, 204);

        darkPalette.setColor(QPalette::Window, darkGray);
        darkPalette.setColor(QPalette::WindowText, lightGray);
        darkPalette.setColor(QPalette::Base, QColor(30, 30, 30));
        darkPalette.setColor(QPalette::AlternateBase, darkGray);
        darkPalette.setColor(QPalette::ToolTipBase, blueAccent);
        darkPalette.setColor(QPalette::ToolTipText, lightGray);
        darkPalette.setColor(QPalette::Text, lightGray);
        darkPalette.setColor(QPalette::Button, darkGray);
        darkPalette.setColor(QPalette::ButtonText, lightGray);
        darkPalette.setColor(QPalette::BrightText, Qt::red);
        darkPalette.setColor(QPalette::Link, blueAccent);
        darkPalette.setColor(QPalette::Highlight, blueAccent);
        darkPalette.setColor(QPalette::HighlightedText, Qt::white);

        qApp->setPalette(darkPalette);

        qApp->setStyleSheet(
            "QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }"
            "QDockWidget { titlebar-close-icon: url(:/close.png); titlebar-normal-icon: url(:/undock.png); }"
            "QDockWidget::title { text-align: left; background: #007ACC; padding-left: 5px; }"
            "QTreeWidget { background-color: #1E1E1E; color: #F0F0F0; border: 1px solid #3F3F46; }"
            "QHeaderView::section { background-color: #2D2D30; color: #F0F0F0; border: 1px solid #3F3F46; }"
            "QMainWindow::separator { background-color: #3F3F46; width: 4px; height: 4px; }"
            "QPushButton { background-color: #3E3E42; border: 1px solid #3F3F46; color: #F0F0F0; padding: 5px; }"
            "QPushButton:hover { background-color: #505050; }"
            "QPushButton:pressed { background-color: #007ACC; }"
            "QLineEdit { background-color: #1E1E1E; color: #F0F0F0; border: 1px solid #3F3F46; }"
            "QComboBox { background-color: #3E3E42; color: #F0F0F0; border: 1px solid #3F3F46; }"
            "QGroupBox { border: 1px solid #3F3F46; margin-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 3px; }"
        );
        
        // Optimize font for industrial readability
        QFont font = qApp->font();
        font.setPointSize(10);
        qApp->setFont(font);
    }
};

#endif // DARKSTYLE_H
