#include "analysis.h"

/*
 following functions will be executed in main thread
*/
Analysis::Analysis()
{
    //init rtmp
    this->rtmpAddressGetFlag = false;

    //init mqtt
    mqttClient_t = new MqttClient();

    //init httpclient
    httpClient_t = new HttpClient();

    moveToThread(&thread_);
    thread_.start(); 
}

Analysis::~Analysis()
{

    //deinit mqtt
    qDebug() << "mqtt反初始化";
    delete mqttClient_t;

    //deinit httpClient
    qDebug() << "HttpClient反初始化";
    delete httpClient_t;

    //deinit rtmp
    this->rtmpAddressGetFlag = false;

    thread_.quit();
    thread_.wait();
}

/*
 * vehicle
*/
void Analysis::beginVehicle(){
    qDebug() << "无人机控制模块启动中...";
    //init vehicleControl
    vehicleControl_t = new VehicleControl();
    connect(this, SIGNAL(sendInitVehicle()), vehicleControl_t, SLOT(slotInitVehicle()));
    connect(this, SIGNAL(sendDeInitVehicle()), vehicleControl_t, SLOT(slotDeInitVehicle()));
    connect(vehicleControl_t, SIGNAL(sendStreamInitSuccess()), this, SLOT(slotBeginRtmp()));
    connect(vehicleControl_t, SIGNAL(sendStreamInitSuccess()), this, SLOT(slotBeginAnalysis()));
    connect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), mqttClient_t, SLOT(slotDeInitMqtt()));
    connect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), this, SLOT(slotStopRtmp()));
    connect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), this, SLOT(slotStopAnalysis()));
    //数据发布
    connect(vehicleControl_t, SIGNAL(sendDroneInfo(DroneInfo)), mqttClient_t, SLOT(slotPublishDroneInfo(DroneInfo)));
    connect(vehicleControl_t, SIGNAL(sendCameraInfo(CameraInfo)), mqttClient_t, SLOT(slotPublishCameraInfo(CameraInfo)));
    connect(vehicleControl_t, SIGNAL(sendWayPointStateInfo(WayPointStateInfo)), mqttClient_t, SLOT(slotPublishWayPointStateInfo(WayPointStateInfo)));
    connect(vehicleControl_t, SIGNAL(sendLogInfo(LogInfo)), mqttClient_t, SLOT(slotPublishLogInfo(LogInfo)));
    connect(vehicleControl_t, SIGNAL(sendCameraFileList(vector<CameraFileInfo>)), mqttClient_t, SLOT(slotPublishCameraFileList(vector<CameraFileInfo>)));
    connect(vehicleControl_t, SIGNAL(sendControlReturnInfo(ControlReturnInfo)), mqttClient_t, SLOT(slotPublishControlReturnInfo(ControlReturnInfo)));
    connect(vehicleControl_t, SIGNAL(sendWayPointExecuteInfo(WayPointExecuteInfo)), mqttClient_t, SLOT(slotPublishWayPointExecuteInfo(WayPointExecuteInfo)));
    connect(vehicleControl_t, SIGNAL(sendWayPointMissionReportInfo(WayPointMissionReportInfo)), mqttClient_t, SLOT(slotPublishWayPointMissionReportInfo(WayPointMissionReportInfo)));
    connect(vehicleControl_t, SIGNAL(sendDeviceAbnormal(ErrorInfo)), mqttClient_t, SLOT(slotPublishDeviceAbnormal(ErrorInfo)));
    connect(vehicleControl_t, SIGNAL(sendCameraFileData(DownloadFileInfo)), httpClient_t, SLOT(slotPushDownloadFileInfo(DownloadFileInfo)));            //http推送
    //云台控制
    connect(mqttClient_t, SIGNAL(sendResetGimbal()), vehicleControl_t, SLOT(slotResetGimbal()));
    connect(mqttClient_t, SIGNAL(sendRotateGimbalPitchPositive(int)), vehicleControl_t, SLOT(slotRotateGimbalPitchPositive(int)));
    connect(mqttClient_t, SIGNAL(sendRotateGimbalPitchNegative(int)), vehicleControl_t, SLOT(slotRotateGimbalPitchNegative(int)));
    connect(mqttClient_t, SIGNAL(sendRotateGimbalYawPositive(int)), vehicleControl_t, SLOT(slotRotateGimbalYawPositive(int)));
    connect(mqttClient_t, SIGNAL(sendRotateGimbalYawNegative(int)), vehicleControl_t, SLOT(slotRotateGimbalYawNegative(int)));
    //相机控制
    connect(mqttClient_t, SIGNAL(sendChangeLiveViewCameraSource(string)), vehicleControl_t, SLOT(slotChangeLiveViewCameraSource(string)));
    connect(mqttClient_t, SIGNAL(sendChangeCameraWorkMode(int)), vehicleControl_t, SLOT(slotChangeCameraWorkMode(int)));
    connect(mqttClient_t, SIGNAL(sendShootSinglePhoto()), vehicleControl_t, SLOT(slotShootSinglePhoto()));
    connect(mqttClient_t, SIGNAL(sendStartRecordVideo()), vehicleControl_t, SLOT(slotStartRecordVideo()));
    connect(mqttClient_t, SIGNAL(sendStopRecordVideo()), vehicleControl_t, SLOT(slotStopRecordVideo()));
    connect(mqttClient_t, SIGNAL(sendSetOpticalZoomParam(float)), vehicleControl_t, SLOT(slotSetOpticalZoomParam(float)));
    connect(mqttClient_t, SIGNAL(sendSetTapZoomPoint(CameraTapZoomPoint)), vehicleControl_t, SLOT(slotSetTapZoomPoint(CameraTapZoomPoint)));
    connect(mqttClient_t, SIGNAL(sendSetFoucsPoint(CameraFocusPoint)), vehicleControl_t, SLOT(slotSetFoucsPoint(CameraFocusPoint)));
    connect(mqttClient_t, SIGNAL(sendGetDownloadFileList()), vehicleControl_t, SLOT(slotGetDownloadFileList()));
    connect(mqttClient_t, SIGNAL(sendDownloadFile(uint32_t)), vehicleControl_t, SLOT(slotDownloadFile(uint32_t)));
    connect(mqttClient_t, SIGNAL(sendDeleteFile(uint32_t)), vehicleControl_t, SLOT(slotDeleteFile(uint32_t)));
    //航点任务控制
    connect(mqttClient_t, SIGNAL(sendUploadWayPointMission(WayPointMissionInfo)), vehicleControl_t, SLOT(slotUploadWayPointMission(WayPointMissionInfo)));
    connect(mqttClient_t, SIGNAL(sendStartWayPointMission()), vehicleControl_t, SLOT(slotStartWayPointMission()));
    connect(mqttClient_t, SIGNAL(sendStopWayPointMission()), vehicleControl_t, SLOT(slotStopWayPointMission()));
    connect(mqttClient_t, SIGNAL(sendPauseWayPointMission()), vehicleControl_t, SLOT(slotPauseWayPointMission()));
    connect(mqttClient_t, SIGNAL(sendResumeWayPointMission()), vehicleControl_t, SLOT(slotResumeWayPointMission()));
    connect(mqttClient_t, SIGNAL(sendFlightControlGoHomeAndConfirmLanding()), vehicleControl_t, SLOT(slotFlightControlGoHomeAndConfirmLanding()));

    Q_EMIT sendInitVehicle();
}

void Analysis::slotStopVehicle(){
    qDebug() << "无人机控制模块停止中...";
    Q_EMIT sendDeInitVehicle();
    disconnect(this, SIGNAL(sendInitVehicle()), vehicleControl_t, SLOT(slotInitVehicle()));
    disconnect(this, SIGNAL(sendDeInitVehicle()), vehicleControl_t, SLOT(slotDeInitVehicle()));
    disconnect(vehicleControl_t, SIGNAL(sendStreamInitSuccess()), this, SLOT(slotBeginRtmp()));
    disconnect(vehicleControl_t, SIGNAL(sendStreamInitSuccess()), this, SLOT(slotBeginAnalysis()));
    disconnect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), mqttClient_t, SLOT(slotDeInitMqtt()));
    disconnect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), this, SLOT(slotStopRtmp()));
    disconnect(vehicleControl_t, SIGNAL(sendAnalysisDeInit()), this, SLOT(slotStopAnalysis()));
    //数据发布
    disconnect(vehicleControl_t, SIGNAL(sendDroneInfo(DroneInfo)), mqttClient_t, SLOT(slotPublishDroneInfo(DroneInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendCameraInfo(CameraInfo)), mqttClient_t, SLOT(slotPublishCameraInfo(CameraInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendWayPointStateInfo(WayPointStateInfo)), mqttClient_t, SLOT(slotPublishWayPointStateInfo(WayPointStateInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendLogInfo(LogInfo)), mqttClient_t, SLOT(slotPublishLogInfo(LogInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendCameraFileList(vector<CameraFileInfo>)), mqttClient_t, SLOT(slotPublishCameraFileList(vector<CameraFileInfo>)));
    disconnect(vehicleControl_t, SIGNAL(sendControlReturnInfo(ControlReturnInfo)), mqttClient_t, SLOT(slotPublishControlReturnInfo(ControlReturnInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendWayPointExecuteInfo(WayPointExecuteInfo)), mqttClient_t, SLOT(slotPublishWayPointExecuteInfo(WayPointExecuteInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendWayPointMissionReportInfo(WayPointMissionReportInfo)), mqttClient_t, SLOT(slotPublishWayPointMissionReportInfo(WayPointMissionReportInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendDeviceAbnormal(ErrorInfo)), mqttClient_t, SLOT(slotPublishDeviceAbnormal(ErrorInfo)));
    disconnect(vehicleControl_t, SIGNAL(sendCameraFileData(DownloadFileInfo)), httpClient_t, SLOT(slotPushDownloadFileInfo(DownloadFileInfo)));            //http推送
    //云台控制
    disconnect(mqttClient_t, SIGNAL(sendResetGimbal()), vehicleControl_t, SLOT(slotResetGimbal()));
    disconnect(mqttClient_t, SIGNAL(sendRotateGimbalPitchNegative(int)), vehicleControl_t, SLOT(slotRotateGimbalPitchNegative(int)));
    disconnect(mqttClient_t, SIGNAL(sendRotateGimbalYawPositive(int)), vehicleControl_t, SLOT(slotRotateGimbalYawPositive(int)));
    disconnect(mqttClient_t, SIGNAL(sendRotateGimbalYawNegative(int)), vehicleControl_t, SLOT(slotRotateGimbalYawNegative(int)));
    //相机控制
    disconnect(mqttClient_t, SIGNAL(sendChangeLiveViewCameraSource(string)), vehicleControl_t, SLOT(slotChangeLiveViewCameraSource(string)));
    disconnect(mqttClient_t, SIGNAL(sendChangeCameraWorkMode(int)), vehicleControl_t, SLOT(slotChangeCameraWorkMode(int)));
    disconnect(mqttClient_t, SIGNAL(sendShootSinglePhoto()), vehicleControl_t, SLOT(slotShootSinglePhoto()));
    disconnect(mqttClient_t, SIGNAL(sendStartRecordVideo()), vehicleControl_t, SLOT(slotStartRecordVideo()));
    disconnect(mqttClient_t, SIGNAL(sendStopRecordVideo()), vehicleControl_t, SLOT(slotStopRecordVideo()));
    disconnect(mqttClient_t, SIGNAL(sendSetOpticalZoomParam(float)), vehicleControl_t, SLOT(slotSetOpticalZoomParam(float)));
    disconnect(mqttClient_t, SIGNAL(sendSetTapZoomPoint(CameraTapZoomPoint)), vehicleControl_t, SLOT(slotSetTapZoomPoint(CameraTapZoomPoint)));
    disconnect(mqttClient_t, SIGNAL(sendSetFoucsPoint(CameraFocusPoint)), vehicleControl_t, SLOT(slotSetFoucsPoint(CameraFocusPoint)));
    disconnect(mqttClient_t, SIGNAL(sendGetDownloadFileList()), vehicleControl_t, SLOT(slotGetDownloadFileList()));
    disconnect(mqttClient_t, SIGNAL(sendDownloadFile(uint32_t)), vehicleControl_t, SLOT(slotDownloadFile(uint32_t)));
    disconnect(mqttClient_t, SIGNAL(sendDeleteFile(uint32_t)), vehicleControl_t, SLOT(slotDeleteFile(uint32_t)));
    //航点任务控制
    disconnect(mqttClient_t, SIGNAL(sendUploadWayPointMission(WayPointMissionInfo)), vehicleControl_t, SLOT(slotUploadWayPointMission(WayPointMissionInfo)));
    disconnect(mqttClient_t, SIGNAL(sendStartWayPointMission()), vehicleControl_t, SLOT(slotStartWayPointMission()));
    disconnect(mqttClient_t, SIGNAL(sendStopWayPointMission()), vehicleControl_t, SLOT(slotStopWayPointMission()));
    disconnect(mqttClient_t, SIGNAL(sendPauseWayPointMission()), vehicleControl_t, SLOT(slotPauseWayPointMission()));
    disconnect(mqttClient_t, SIGNAL(sendResumeWayPointMission()), vehicleControl_t, SLOT(slotResumeWayPointMission()));
    disconnect(mqttClient_t, SIGNAL(sendFlightControlGoHomeAndConfirmLanding()), vehicleControl_t, SLOT(slotFlightControlGoHomeAndConfirmLanding()));

    delete vehicleControl_t;
}

/*
 * analysis
*/
void Analysis::slotBeginAnalysis(){
    if(g_analysis_enable.toStdString() == "true"){
        qDebug("已启用算法分析功能!");
        this->slotStartAnalysis();
    }
    else{
        qWarning("未启用算法分析功能!");
    }
}

void Analysis::slotStopAnalysis(){
    qDebug("关闭分析程序!");
    this->manuStopAnalysis = true;
    QProcess *p = new QProcess;
    p->start(QString("killall analysisClient"));
}

void Analysis::slotStartAnalysis(){
    this->manuStopAnalysis = false;
    std::thread analysisThread(&Analysis::analysis_pipeline_thread, this);
    analysisThread.detach();
}
/*
 * rtmp
*/
void Analysis::slotBeginRtmp(){
    if(g_rtmp_client_output.toStdString() == "rtmp" || g_rtmp_client_output.toStdString() == "display"){
        qDebug("已启用RTMP推流/显示功能!");
        this->slotStartRtmp();
    }
    else{
        qWarning("未启用RTMP推流/显示功能!");
    }
}

void Analysis::slotStopRtmp(){
    qDebug("关闭RTMP推流/显示程序!");
    this->manuStopRtmp = true;
    QProcess *p = new QProcess;
    p->start(QString("killall rtmpClient"));
}

void Analysis::slotStartRtmp(){
    this->manuStopRtmp = false;
    std::thread rtmpThread(&Analysis::rtmp_client_thread, this);
    rtmpThread.detach();
}


/*
 following function will be executed in another thread
*/
void Analysis::analysis_pipeline_thread(){
    //关闭所有算法分析进程
    QProcess *p = new QProcess;
    p->start(QString("killall analysisClient"));
    p->waitForFinished(-1);
    //启动算法分析进程
    qDebug() << "算法分析进程启动中, 算法类型:" << g_analysis_algorithm_type << " 输出类型:" << g_analysis_client_output << " rtmpAddress(输出类型为RTMP有效):" << g_analysis_client_address;
    QStringList args;
    args.append(QString("%1").arg(g_analysis_algorithm_type));
    args.append(QString("%1").arg(g_platform_data_push_address));
    args.append(QString("%1").arg(g_analysis_client_output));
    if(g_analysis_client_output.toStdString() == "rtmp"){
        args.append(QString("%1").arg(g_analysis_client_address));
    }
    QProcess *analysisClient_p = new QProcess;
    //绑定process进程完成信号，垃圾回收
    connect(analysisClient_p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [analysisClient_p, this](int exitCode, QProcess::ExitStatus exitStatus){
        qWarning() << "算法分析进程异常";
        Q_UNUSED(exitCode);
        Q_UNUSED(exitStatus);
        analysisClient_p->close();
        analysisClient_p->kill();
        analysisClient_p->deleteLater();
    });
    //开始执行
    analysisClient_p->start(g_analysis_client_process_path, args);
    //等待系统调度
    if(!analysisClient_p->waitForStarted(-1)){
       qCritical() << "算法分析进程启动失败";
    }
    analysisClient_p->waitForFinished(-1);
    if(!this->manuStopAnalysis){
        qWarning() << "算法分析进程退出,程序重启中...";
        this->slotStartAnalysis();
    }
}

void Analysis::rtmp_client_thread(){
    if(g_rtmp_client_output.toStdString() == "rtmp"){
        //检查网络状态
        Tools::checkNetworkConnect();
        //获取推流地址
        if(g_rtmp_client_address_source == "platform"){
            if(!this->rtmpAddressGetFlag){
                while( Tools::getPushAddressInfo(g_rtmp_client_address) == -1){
                    qWarning("从平台获取推流地址失败,3秒后重试...");
                    sleep(3);
                }
                qDebug("成功从平台获取推流地址!");
                this->rtmpAddressGetFlag = true;
            }
        }
        else{
            qDebug("使用配置文件中推流地址!");
        }
    }
    //关闭所有推流/显示进程
    QProcess *p = new QProcess;
    p->start(QString("killall rtmpClient"));
    p->waitForFinished(-1);
    //启动rtmp推流/显示进程
    qDebug() << "rtmp推流/显示进程启动中, 输出类型:" << g_rtmp_client_output << " rtmpAddress(输出类型为RTMP有效):" << g_rtmp_client_address;
    QStringList args;
    args.append(QString("%1").arg(g_rtmp_client_output));
    if(g_rtmp_client_output.toStdString() == "rtmp"){
        args.append(QString("%1").arg(g_rtmp_client_address));
    }
    QProcess *rtmpClient_p = new QProcess;
    //绑定process进程完成信号，垃圾回收
    connect(rtmpClient_p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [rtmpClient_p, this](int exitCode, QProcess::ExitStatus exitStatus){
        qWarning() << "rtmp推流进程异常";
        Q_UNUSED(exitCode);
        Q_UNUSED(exitStatus);
        rtmpClient_p->close();
        rtmpClient_p->kill();
        rtmpClient_p->deleteLater();
    });
    //开始执行
    rtmpClient_p->start(g_rtmp_client_process_path, args);
    //等待系统调度
    if(!rtmpClient_p->waitForStarted(-1)){
       qCritical() << "rtmp推流进程启动失败";
    }
    rtmpClient_p->waitForFinished(-1);
    if(!this->manuStopRtmp){
        qWarning() << "rtmp推流/显示进程退出,程序重启中...";
        this->slotStartRtmp();
    }
}
