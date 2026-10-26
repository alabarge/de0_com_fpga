#include <QApplication>

#include "MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    //qApp->setStyleSheet("Fusion");
    //QApplication::setStyle("Fusion");
    QApplication::setOrganizationName(QStringLiteral("Omniware"));
    QApplication::setApplicationName(QStringLiteral("jack-qt"));

    MainWindow window;
    window.show();

    return app.exec();
}
