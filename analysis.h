#ifndef ANALYSIS_H
#define ANALYSIS_H
/*
 * all communcations to main ui thread must pass through this class, like image show, start/stop control...
 * analysis include analysisStream and postprocess, these three classes communicate with each other through out the signal/slot mechanism
*/
#include <QThread>
#include <QObject>
#include <QTimer>
#include <QProcess>

#include "global.h"
#include "vehiclecontrol.h"
#include "mqttclient.h"
#include "tools.h"
#include "httpclient.h"

using namespace std;

class Analysis: public QObject
{
    Q_OBJECT //The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and slots or that uses other services provided by Qt's meta-object system
public:
    Analysis();
    ~Analysis();
    //无人机相关
    void beginVehicle();

public Q_SLOTS:
    void slotBeginAnalysis();
    void slotStartAnalysis();
    void slotStopAnalysis();
    void slotBeginRtmp();
    void slotStartRtmp();
    void slotStopRtmp();
    void slotStopVehicle();

Q_SIGNALS:
    void sendInitVehicle();
    void sendDeInitVehicle();

private:
    void analysis_pipeline_thread();
    void rtmp_client_thread();
    QThread thread_;
    VehicleControl* vehicleControl_t;
    MqttClient* mqttClient_t;
    HttpClient* httpClient_t;               //用于图片视频的数据推送
    bool rtmpAddressGetFlag;                //rtmp流地址获取标志
    bool manuStopRtmp;                      //主动停止rtmp推流标志
    bool manuStopAnalysis;                  //主动停止算法分析标志
};

#endif // ANALYSIS_H
