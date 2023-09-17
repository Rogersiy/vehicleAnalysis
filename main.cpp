#include <QCoreApplication>
#include <QApplication>
#include <QDir>
#include "global.h"
#include "analysis.h"
#include "applive.h"
#include <QMutex>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include "tools.h"

void outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    mutex.lock();
    QString text;
    switch(type)
    {
        case QtDebugMsg:
            text = QString("Debug:");
            break;
        case QtWarningMsg:
            text = QString("Warning:");
            break;
        case QtCriticalMsg:
            text = QString("Critical:");
            break;
        case QtFatalMsg:
            text = QString("Fatal:");
    }
    QString context_info = QString("File:(%1) Line:(%2)").arg(QString(context.file)).arg(context.line);
    QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss ddd");
    QString current_date = QString("(%1)").arg(current_date_time);
    QString message;
    message = QString("%1 %2 %3 %4").arg(text).arg(current_date).arg(msg).arg(context_info);
    QFile file(QApplication::applicationDirPath() + "/log/log_" +  QDateTime::currentDateTime().toString("yyyy_MM_dd") + ".txt");
    if(file.exists()){
        if(msg == "程序启动中..."){
            message = QString("\n\n%1 %2 %3 %4").arg(text).arg(current_date).arg(msg).arg(context_info);
        }
    }
    file.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream text_stream(&file);
    text_stream << message << "\r\n";
    file.flush();
    file.close();
    mutex.unlock();
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QString folderPath = "log";
    QDir dir;
    if (dir.mkpath(folderPath)) 
    {
        qDebug() << "Log folder created successfully.";
    } else 
    {
        qDebug() << "Failed to create log folder.";
    }
    
    //注册MessageHandler->log
    qInstallMessageHandler(outputMessage);
    qDebug() << "程序启动中...";

    //读取配置文件
    App::readConfig();

    //注册信号槽参数类型
    qRegisterMetaType< QString >("QString");
    qRegisterMetaType< DroneInfo >("DroneInfo");
    qRegisterMetaType< CameraInfo >("CameraInfo");
    qRegisterMetaType< WayPointStateInfo >("WayPointStateInfo");
    qRegisterMetaType< LogInfo >("LogInfo");
    qRegisterMetaType< DownloadFileInfo >("DownloadFileInfo");
    qRegisterMetaType< vector<CameraFileInfo> >("vector<CameraFileInfo>");
    qRegisterMetaType< string >("string");
    qRegisterMetaType< CameraTapZoomPoint >("CameraTapZoomPoint");
    qRegisterMetaType< CameraFocusPoint >("CameraFocusPoint");
    qRegisterMetaType< ErrorInfo >("ErrorInfo");
    qRegisterMetaType< WayPointMissionInfo >("WayPointMissionInfo");
    qRegisterMetaType< uint32_t >("uint32_t");
    qRegisterMetaType< DetectionInfo >("DetectionInfo");
    qRegisterMetaType< EventInfo >("EventInfo");
    qRegisterMetaType< ControlReturnInfo >("ControlReturnInfo");
    qRegisterMetaType< WayPointExecuteInfo >("WayPointExecuteInfo");
    qRegisterMetaType< WayPointMissionReportInfo >("WayPointMissionReportInfo");

    //启动守护程序服务类
    AppLive::Instance()->start(6666);

    //网络状态检查
    Tools::checkNetworkConnect();

    //启动主程序,Analysis为交互类
    qDebug() << "主程序启动中...";
    Analysis* analysisProcess = new Analysis();
    analysisProcess->beginVehicle();

    return a.exec();
}
