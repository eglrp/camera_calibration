#include "mainwindow.h"
#include <QApplication>

int QtDisplayThread()
{
	int argc =1;
	char **argv = NULL;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
