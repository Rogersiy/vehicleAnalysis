#include <QDebug>
#include <QCoreApplication>
#include "analysisstream.h"
#include "global.h"

int main(int argc, char *argv[])
{
    qDebug() << "程序启动中...";
    QCoreApplication a(argc, argv);
    g_application_path = QCoreApplication::applicationDirPath().toStdString();

    //参数校验
    string analysisType;
    string outPut;
    string rtmpAddress;
    if(argc == 4 && string(argv[3]) != "rtmp"){
        analysisType = argv[1];
        g_platform_event_address = argv[2];
        outPut = argv[3];
    }
    else if(argc == 5 && string(argv[3]) == "rtmp"){
        analysisType = argv[1];
        g_platform_event_address = argv[2];
        outPut = argv[3];
        rtmpAddress = argv[4];
    }
    else{
        qDebug() << "参数不合法";
        return 0;
    }
    if(!(outPut == "rtmp" || outPut == "rtsp" || outPut == "display" || outPut == "none")){
        qDebug() << "参数不合法";
        return 0;
    }


    AnalysisStream* analysisStream_t = new AnalysisStream(analysisType, outPut, rtmpAddress);
    analysisStream_t->initPipeline();
    analysisStream_t->startAnalysis();

    return 0;
}
