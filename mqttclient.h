#ifndef MQTTCLIENT_H
#define MQTTCLIENT_H

/*
 *
日志信息以及事件信息需要进行离线存储,防止网络异常以及程序异常情况下数据无法回传至平台
实时信息：无人机状态信息，无人机相机信息，无人机航线状态信息，无人机控制异常信息,此类信息无需保证数据必达
*/

#include <QThread>
#include <QObject>
#include <QApplication>

#include "global.h"
#include "tools.h"
#include "MQTTAsync.h"
#include <QtSql>

using namespace std;

class MqttClient: public QObject
{
    Q_OBJECT //The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and slots or that uses other services provided by Qt's meta-object system
public:
    MqttClient();
    ~MqttClient();

public Q_SLOTS:
    void slotInitMqtt();
    void slotDeInitMqtt();
    void slotPublishDroneInfo(DroneInfo droneInfo);
    void slotPublishCameraInfo(CameraInfo cameraInfo);
    void slotPublishWayPointStateInfo(WayPointStateInfo wayPointStateInfo);
    void slotPublishCameraFileList(vector<CameraFileInfo> cameraFileList);
    void slotPublishCameraFileData(DownloadFileInfo downloadFileInfo);
    void slotPublishLogInfo(LogInfo logInfo);                                           //无人机任务日志信息
    void slotPublishEventInfo(EventInfo eventInfo);                                     //无人机事件信息
    void slotPublishControlReturnInfo(ControlReturnInfo controlReturnInfo);
    void slotPublishWayPointExecuteInfo(WayPointExecuteInfo wayPointExecuteInfo);
    void slotPublishWayPointMissionReportInfo(WayPointMissionReportInfo wayPointMissionReportInfo);
    void slotPublishDeviceAbnormal(ErrorInfo errorInfo);

Q_SIGNALS:
    void sendResetGimbal();
    void sendRotateGimbalPitchPositive(int);
    void sendRotateGimbalPitchNegative(int);
    void sendRotateGimbalYawPositive(int);
    void sendRotateGimbalYawNegative(int);
    void sendChangeLiveViewCameraSource(string);
    void sendChangeCameraWorkMode(int);
    void sendShootSinglePhoto();
    void sendStartRecordVideo();
    void sendStopRecordVideo();
    void sendSetOpticalZoomParam(float);
    void sendSetTapZoomPoint(CameraTapZoomPoint);
    void sendSetFoucsPoint(CameraFocusPoint);
    void sendGetDownloadFileList();
    void sendDownloadFile(uint32_t);
    void sendDeleteFile(uint32_t);
    void sendUploadWayPointMission(WayPointMissionInfo);
    void sendStartWayPointMission();
    void sendStopWayPointMission();
    void sendPauseWayPointMission();
    void sendResumeWayPointMission();
    void sendFlightControlGoHomeAndConfirmLanding();

private:
    QThread thread_;
    MQTTAsync client;
    static MqttClient *static_mqttClient;
    bool connected;
    QSqlDatabase db;
    map<string, vector<string>> logDuplicationMap;          //日志去重map
    map<int, vector<string>> wayPointDuplicationMap;        //航点去重map
    static void onConnectFailure(void* context, MQTTAsync_failureData5* response);
    static void onConnect(void* context, MQTTAsync_successData5* response);
    static void onDisconnectFailure(void* context, MQTTAsync_failureData5* response);
    static void onDisconnect(void* context, MQTTAsync_successData5* response);
    static void onSendFailure(void* context, MQTTAsync_failureData5* response);
    static void onSend(void* context, MQTTAsync_successData5* response);
    static void onSubscribe(void* context, MQTTAsync_successData5* response);
    static void onSubscribeFailure(void* context, MQTTAsync_failureData5* response);
    static void connlost(void *context, char *cause);                                                   //心跳时间达到，如果没有收到服务器的reponse，断开连接,客户端会自动重连
    static void onReconnected(void* context, char* cause);                                              //连接成功后的回调函数,进行信息订阅
    static int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message);      //消息订阅到达
    void publish(char *topic, char *payload, int qos, int retained);                                    //发布
    void subscribe(const char* TOPIC, const char* CLIENTID, int QOS);                                   //订阅
    void droneControl(string topic, string content);                                                    //无人机控制
    void publishOfflineData();
};

#endif // ANALYSIS_H
