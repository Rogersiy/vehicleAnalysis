#include <iostream>
#pragma execution_character_set("utf-8")
#include "app.h"
#include <QApplication>
#include "monitor.h"

using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    App::ConfigFile = qApp->applicationDirPath() + "/config.ini";
    App::readConfig();

    monitor monitor_t;
    monitor_t.startApp();

    return a.exec();
}
