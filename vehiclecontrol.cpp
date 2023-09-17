#include "vehiclecontrol.h"
VehicleControl* VehicleControl::static_vehicleControl = NULL;
/*
 following functions will be executed in main thread
*/
VehicleControl::VehicleControl()
{
    static_vehicleControl = this;

    // 初始化互斥锁
    vehicleInfoMutex = PTHREAD_MUTEX_INITIALIZER;
    cameraInfoMutex = PTHREAD_MUTEX_INITIALIZER;

    //获得或创建一个共享内存标识符
    semid = shmget((key_t)1122, sizeof(SemMsg), 0666|IPC_CREAT);
    if(semid == -1)        //获取或创建一个共享内存标识符失败
    {
        qCritical() << "共享内存创建失败,程序退出";
        exit(EXIT_FAILURE);
    }
    qDebug() << "共享内存信号量初始化,semid:" << semid;
    sem = shmat(semid, (void*)0, 0);        //返回共享存储段连接的实际地址
    if(sem == (void*)-1)
    {
        qCritical() << "共享存储段实际地址获取失败,程序退出";
        exit(EXIT_FAILURE);
    }
    g_shared_sem = (SemMsg*)sem;                  //缓冲区为共享存储段连接地址
    sem_init(&(g_shared_sem->streamSem),1,1);           //信号量初始化，且信号量初始值为第二个1
    sem_init(&(g_shared_sem->droneInfoSem),1,1);        //信号量初始化，且信号量初始值为第二个1

    //获得或创建一个共享内存标识符
    shmid = shmget((key_t)1121, sizeof(StreamShmMsg), 0666|IPC_CREAT);
    if(shmid == -1)        //获取或创建一个共享内存标识符失败
    {
        qCritical() << "共享内存创建失败,程序退出";
        exit(EXIT_FAILURE);
    }
    qDebug() << "共享内存视频流初始化,shmid:" << shmid;
    shm = shmat(shmid, (void*)0, 0);        //返回共享存储段连接的实际地址
    if(shm == (void*)-1)
    {
        qCritical() << "共享存储段实际地址获取失败,程序退出";
        exit(EXIT_FAILURE);
    }
    g_shared_stream = (StreamShmMsg*)shm;                  //缓冲区为共享存储段连接地址
    g_shared_stream->rtmpUse = 1;
    g_shared_stream->analysisUse = 1;
    g_shared_stream->len = 0;

    //获得或创建一个共享内存标识符
    shmdroneid = shmget((key_t)1120, sizeof(DroneInfo), 0666|IPC_CREAT);
    if(shmdroneid == -1)        //获取或创建一个共享内存标识符失败
    {
        qCritical() << "共享内存创建失败,程序退出";
        exit(EXIT_FAILURE);
    }
    qDebug() << "共享内存无人机状态信息初始化,shmdroneid:" << shmdroneid;
    shmdrone = shmat(shmdroneid, (void*)0, 0);        //返回共享存储段连接的实际地址
    if(shmdrone == (void*)-1)
    {
        qCritical() << "共享存储段实际地址获取失败,程序退出";
        exit(EXIT_FAILURE);
    }
    g_shared_drone = (DroneInfo*)shmdrone;                  //缓冲区为共享存储段连接地址

    //无人机信息相关变量
    this->psdkInitFlag = false;
    this->streamFlag = false;
    this->infoFlag = false;
    this->cameraFlag = false;
    this->flightControlTid = 0;
    this->cameraDownloadTid = 0;
    //航线任务相关变量
    this->missionEndTime = QDateTime::currentDateTime();
    this->missionEntryFlag = false;
    this->missionObstacleAvoidanceFlag = false;
    this->missionManuPauseFlag = false;
    this->wayPointTaskUploadStatus = 0;
    this->wayPointTaskExecuteStatus = 0;
    this->lastMissionState.state = 0;
    this->lastMissionState.curWaypointIndex = -1;
    this->lastMissionState.velocity = 0;
    this->currentMissionState.state = 0;
    this->currentMissionState.curWaypointIndex = -1;
    this->currentMissionState.velocity = 0;

    moveToThread(&thread_);
    thread_.start();
}

VehicleControl::~VehicleControl()
{
    static_vehicleControl = NULL;

    //将共享内存从当前进程中分离
    if(shmdt(sem) == -1)        //失败
    {
        qCritical() << "共享内存信号量分离失败";
        exit(EXIT_FAILURE);
    }
    if(shmctl(semid, IPC_RMID, 0) == -1)        //失败
    {
        qCritical() << "删除该信号量结构以及相连的共享存储段标识";
        exit(EXIT_FAILURE);
    }

    //将共享内存从当前进程中分离
    if(shmdt(shm) == -1)        //失败
    {
        qCritical() << "共享内存分离失败";
        exit(EXIT_FAILURE);
    }
    if(shmctl(shmid, IPC_RMID, 0) == -1)        //失败
    {
        qCritical() << "删除视频流结构以及相连的共享存储段标识";
        exit(EXIT_FAILURE);
    }

    //将共享内存从当前进程中分离
    if(shmdt(shmdrone) == -1)        //失败
    {
        qCritical() << "共享内存分离失败";
        exit(EXIT_FAILURE);
    }
    if(shmctl(shmdroneid, IPC_RMID, 0) == -1)        //失败
    {
        qCritical() << "删除无人机状态信息结构以及相连的共享存储段标识";
        exit(EXIT_FAILURE);
    }

    thread_.quit();
    thread_.wait();
}

////初始化
void VehicleControl::slotInitVehicle(){
    qDebug() << "无人机控制模块初始化中...";
    //1. Setup PSDK
    this->psdkInitFlag = false;
    std::thread initCheckThread([this]{
        sleep(5);
        while(!this->psdkInitFlag){
            qCritical() << "psdk初始化超时,请检查设备连线,或联系维护人员检查设备状态";
            sleep(3);
        }
    });
    initCheckThread.detach();
    Application application(NULL, NULL);
    T_DjiReturnCode returnCode;
    qDebug() << "psdk初始化成功";
    this->psdkInitFlag = true;
    this->mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;

    //2. get aircraft Info
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(0, "无人机类型获取失败");
        qCritical() << QString().sprintf("无人机类型获取失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    else{
        this->droneInfo.aircraftType = aircraftInfoBaseInfo.aircraftType;
        this->droneInfo.networkOperator = g_network_operator.toStdString();             //与硬件设备相关，功能待完善
    }

    //3. 订阅无人机信息
    std::thread vehicleInfoThread(&VehicleControl::getDroneInfo, this);
    vehicleInfoThread.detach();

    //4. init power management
    qDebug() << "Init power management";
    USER_LOG_INFO("Init power management");
    returnCode = DjiPowerManagement_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(0, "电源管理模块初始化失败");
        qCritical() << QString().sprintf("电源管理模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    returnCode = DjiPowerManagement_RegPowerOffNotificationCallback(powerOffNotificationCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(0, "电源管理模块回调订阅失败");
        qCritical() << QString().sprintf("电源管理模块回调订阅失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }

    //5. init camera manager
    /*
     * 不挂载相机，相机管理模块，云台管理模块，视频流liveView管理模块都无法正常初始化，日后优化无挂载情况下的使用
    */
    qDebug() << "Init camera manager module";
    USER_LOG_INFO("Init camera manager module");
    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(1, "相机管理模块初始化失败");
        qCritical() << QString().sprintf("相机管理模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    else{
        //获取相机信息
        std::thread cameraInfoThread(&VehicleControl::getCameraInfo, this);
        cameraInfoThread.detach();
    }
    this->cameraRecordStatus = 0;
    this->cameraZoomStatus = 0;
    this->cameraDownloadStatus = 0;

    //6. 打开视频流
    qDebug() << "Init liveview module";
    USER_LOG_INFO("Init liveview module");
    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(6, "视频流模块初始化失败");
        qCritical() << QString().sprintf("视频流模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    else{
        std::thread vehicleStreamThread(&VehicleControl::startCameraStream, this);
        vehicleStreamThread.detach();
    }

    //7. init gimbal manager
    qDebug() << "Init gimbal manager module";
    USER_LOG_INFO("Init gimbal manager module");
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(2, "云台管理模块初始化失败");
        qCritical() << QString().sprintf("云台管理模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    else{
        qDebug() << "set gimbal mode free";
        USER_LOG_INFO("set gimbal mode free");
        returnCode = DjiGimbalManager_SetMode(this->mountPosition, DJI_GIMBAL_MODE_FREE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            this->sendDeviceAbnormal(2, "云台模式设置失败");
            qCritical() << QString().sprintf("云台模式设置失败,错误码:0x%08lX,程序尝试重启...", returnCode);
            exit(0);
        }
        else{
            this->droneInfo.gimbalMode = DJI_GIMBAL_MODE_FREE;
            //reset gimbal
            this->slotResetGimbal();
        }
    }

    //8. init flight control
    qDebug() << "Init flight Control";
    USER_LOG_INFO("Init flight Control");
    returnCode = DjiFlightController_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "飞行控制模块初始化失败");
        qCritical() << QString().sprintf("飞行控制模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    qDebug() << "Register joystick Ctrl Authority Event Callback";
    USER_LOG_INFO("Register joystick Ctrl Authority Event Callback\r");
    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(flightControlJoystickCtrlAuthSwitchEventCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        this->sendDeviceAbnormal(4, "飞行控制事件回调注册失败");
        qCritical() << QString().sprintf("飞行控制模块事件回调注册失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }

    //9.init waypoint V2 control
    qDebug() << "Init Waypoint V2";
    USER_LOG_INFO("Init Waypoint V2");
    returnCode = DjiWaypointV2_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(5, "航点飞行模块初始化失败");
        qCritical() << QString().sprintf("航点飞行模块初始化失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    qDebug() << "Register waypoint V2 event and state callback";
    USER_LOG_INFO("Register waypoint V2 event and state callback\r");
    returnCode = DjiWaypointV2_RegisterMissionEventCallback(waypointV2EventCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(5, "航点飞行模块事件回调注册失败");
        qCritical() << QString().sprintf("航点飞行模块事件回调注册失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    returnCode = DjiWaypointV2_RegisterMissionStateCallback(waypointV2StateCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(5, "航点飞行模块状态回调注册失败");
        qCritical() << QString().sprintf("航点飞行模块状态回调注册失败,错误码:0x%08lX,程序尝试重启...", returnCode);
        exit(0);
    }
    sleep(1);

    //10.set flight Control params
    qDebug() << "set flight Control params...";
    USER_LOG_INFO("set flight control params...");
    //->1.启用RTK--等待连接
    qDebug() << "Get rtk enable status";
    E_DjiFlightControllerRtkPositionEnableStatus flightControllerRtkPositionEnableStatus;
    returnCode = DjiFlightController_GetRtkPositionEnableStatus(&flightControllerRtkPositionEnableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       qWarning() << QString().sprintf("Get rtk enable failed, error code: 0x%08X", returnCode);
    }
    qDebug() << QString().sprintf("Current rtk enable status is %d\r\n", flightControllerRtkPositionEnableStatus);
    if(!flightControllerRtkPositionEnableStatus){
        USER_LOG_INFO("Set rtk enable status");
        qDebug() << "Set rtk enable status";
        returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << QString().sprintf("Set rtk enable failed, error code: 0x%08X", returnCode);
        }
        returnCode = DjiFlightController_GetRtkPositionEnableStatus(&flightControllerRtkPositionEnableStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
           qWarning() << QString().sprintf("Get rtk enable failed, error code: 0x%08X", returnCode);
        }
        qDebug() << QString().sprintf("Current rtk enable status is %d\r\n", flightControllerRtkPositionEnableStatus);
    }
    //->2.起飞，降落，返航为飞控固有动作，不需要获取飞机控制权
    qDebug() << "Obtain joystick control authority.";
    USER_LOG_INFO("Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "飞行控制权获取失败");
        qCritical() << "Obtain joystick authority failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    }
    //->3.启用避障
    this->flightControlAvoidParam();

    //11. 获取激光测距信息
//    std::thread laserRangThread(&VehicleControl::getLaserRangingInfo, this);
//    laserRangThread.detach();
}

void VehicleControl::slotDeInitVehicle(){
    //停止视频推流
    this->streamFlag = false;

    //停止获取相机信息
    this->cameraFlag = false;

    //停止无人机信息获取
    this->infoFlag = false;

    //deinit PSDK
    T_DjiReturnCode returnCode = DjiGimbalManager_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       qCritical() << "Deinit gimbal manager failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
       USER_LOG_ERROR("Deinit gimbal manager failed, error code: 0x%08X", returnCode);
    }

    returnCode = DjiCameraManager_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Camera manager deinit failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Camera manager deinit failed ,error code :0x%08X", returnCode);
    }

    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Release joystick authority failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
    }

    returnCode = DjiFlightController_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Deinit Flight Control failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Deinit Flight Control module failed,error code:0x%08llX", returnCode);
    }

    returnCode = DjiWaypointV2_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Deinit waypoint V2 module failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
       USER_LOG_ERROR("Deinit waypoint V2 module failed, error code: 0x%08X", returnCode);
    }

    returnCode = DjiLiveview_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Liveview deinit failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Liveview deinit failed, error code: 0x%08X", returnCode);
    }
}

////视频流相关
bool VehicleControl::startCameraStream(){
    //h264 stream
    this->streamFlag = true;
    T_DjiReturnCode returnCode;
    if(this->cameraType == DJI_CAMERA_TYPE_H20 || this->cameraType == DJI_CAMERA_TYPE_H20T){
        //camera
        this->cameraSource = (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE;
        returnCode = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition)this->mountPosition,
        this->cameraSource,
        liveViewH264Cb);
    }
    else{
        //fpv
        this->cameraSource = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;
        returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV,
                                                 this->cameraSource,
                                                 liveViewH264Cb);
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(6, "码流获取失败");
        qCritical() << "Request h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Request h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
    }
    E_DjiLiveViewCameraSource currentCameraSource = this->cameraSource;
    qDebug() << "Start MainCamera h264 Stream!";
    USER_LOG_INFO("Start MainCamera h264 Stream!");
    Q_EMIT sendStreamInitSuccess();         //发送视频流初始化成功信号
    while(this->streamFlag){
        g_usleep(500000);
        if(currentCameraSource != this->cameraSource){
            if(currentCameraSource == DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT){
                //fpv
                returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    this->sendDeviceAbnormal(6, "码流停止获取失败");
                    qCritical() << "Request h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
                    USER_LOG_ERROR("Request to stop h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
                }
            }
            else{
                //camera
                returnCode = DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition)this->mountPosition);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    this->sendDeviceAbnormal(6, "码流停止获取失败");
                    qCritical() << "Request h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
                    USER_LOG_ERROR("Request to stop h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
                }
            }
            if(this->cameraSource == DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT){
                //fpv
                T_DjiReturnCode returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV,
                                                         this->cameraSource,
                                                         liveViewH264Cb);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    this->sendDeviceAbnormal(6, "码流获取失败");
                    qCritical() << "Request h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
                    USER_LOG_ERROR("Request h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
                }
            }
            else{
                //camera
                T_DjiReturnCode returnCode = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition) this->mountPosition,
                                                         this->cameraSource,
                                                         liveViewH264Cb);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    this->sendDeviceAbnormal(6, "码流获取失败");
                    qCritical() << "Request h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
                    USER_LOG_ERROR("Request h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
                }
            }
            currentCameraSource = this->cameraSource;
        }
    }

    returnCode = DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition)this->mountPosition);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(6, "码流停止获取失败");
        qCritical() << "Request to stop h264 failed, error code:0x"<< QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Request to stop h264 of payload %d failed, error code: 0x%08X", this->mountPosition, returnCode);
    }
    qDebug() << "liveview StopMainCameraStream!";
    USER_LOG_INFO("liveview StopMainCameraStream!");

    return true;
}

bool VehicleControl::stopCameraStream() {
    qDebug() << "无人机视频流获取停止中...";
    this->streamFlag = false;
    return true;
}

void VehicleControl::liveViewH264Cb(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen){
    //全局变量
//    msg *mp = new msg;
//    mp->len = bufLen;
//    mp->data = new uint8_t[bufLen];
//    memcpy(mp->data, buf, bufLen);
//    // 加锁
//    pthread_mutex_lock(&myMutex);
//    // 访问公共区
//    if(!loopqueue.inLoopQueue(mp)){
//        delete mp;
//    }
//    // 解锁
//    pthread_mutex_unlock(&myMutex);
//    // 唤醒阻塞在条件变量上的消费者
//    pthread_cond_signal(&cond);
//    cout << "loopqueue size:" << loopqueue.getSize() << endl;

    //共享内存
    if(sem_wait(&(g_shared_sem->streamSem)) == -1)        //sem_wait为P操作，减少信号量的值
    {
        qCritical() << "信号量P操作 ERROR!";
        return;
    }
    if(bufLen > 0 && bufLen < 60000){
        cout << "len: " << g_shared_stream->len << "   rtmpUse:" << g_shared_stream->rtmpUse << "   analysisUse:" << g_shared_stream->analysisUse << "   time:" << time(NULL) << endl;
        g_shared_stream->rtmpUse = 0;
        g_shared_stream->analysisUse = 0;
        g_shared_stream->len = bufLen;
        memcpy(g_shared_stream->data, buf, bufLen);
    }
    else{
        cout << "数据异常,未更新数据,len: " << g_shared_stream->len << "   rtmpUse:" << g_shared_stream->rtmpUse << "   analysisUse:" << g_shared_stream->analysisUse << "   time:" << time(NULL) << endl;
        qWarning() << "数据异常,未更新数据,len: " << g_shared_stream->len << "   rtmpUse:" << g_shared_stream->rtmpUse << "   analysisUse:" << g_shared_stream->analysisUse << "   time:" << time(NULL);
    }
    sem_post(&g_shared_sem->streamSem);                           //V 操作增加信号量

    return;
}

////无人机相关信息获取
bool VehicleControl::getDroneInfo(){
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionFlightStatus flight = {0};
    T_DjiFcSubscriptionDisplaymode displayMode;
    T_DjiFcSubscriptionThreeGimbalData threeGimbalData = {0};
    T_DjiFcSubscriptionPositionFused position = {0};
    T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
    T_DjiFcSubscriptionHeightFusion relativeHeight;
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomePoint;
    T_DjiFcSubscriptionAltitudeFused altitudeFused;                     //barometer data
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};
    T_DjiDataTimestamp timestamp = {0};
    string netWorkQualityCommand = "ping " + g_platform_ip.toStdString() + " -c 1 |grep -E 'time='| cut -f4 -d= | cut -f1 -dm | sed 's/ //g'";

    qDebug() << "Init fc subscription module";
    USER_LOG_INFO("Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "消息订阅模块初始化失败");
        qCritical() << QString().sprintf("消息订阅模块初始化失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "飞行状态订阅失败");
        qCritical() << QString().sprintf("消息订阅模块飞行状态订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "飞行模式订阅失败");
        qCritical() << QString().sprintf("消息订阅模块飞行模式订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "云台数据订阅失败");
        qCritical() << QString().sprintf("消息订阅模块云台数据订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "飞行速度订阅失败");
        qCritical() << QString().sprintf("消息订阅模块飞行速度订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    //传感器获取实时的对地高度，10米以内较为准确，10米以上不可靠,DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION也是
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "对地高度订阅失败");
        qCritical() << QString().sprintf("消息订阅模块对地高度订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "融合海拔订阅失败");
        qCritical() << QString().sprintf("消息订阅模块融合海拔订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "融合数据订阅失败");
        qCritical() << QString().sprintf("消息订阅模块融合数据订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "RTK数据订阅失败");
        qCritical() << QString().sprintf("消息订阅模块RTK数据订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "RTK位置定位状态订阅失败");
        qCritical() << QString().sprintf("消息订阅模块位置定位状态订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                      NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "返航点海拔高度订阅失败");
        qCritical() << QString().sprintf("消息订阅模块返航点海拔高度订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    //以下订阅信息暂未放到无人机信息中
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                                                     DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                     NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(3, "避障数据订阅失败");
        qCritical() << QString().sprintf("消息订阅模块避障数据订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                      NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "姿态四元数订阅失败");
        qCritical() << QString().sprintf("消息订阅模块姿态四元数订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                      NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(3, "控制权订阅失败");
        qCritical() << QString().sprintf("消息订阅模块控制权订阅失败,错误码:0x%08lX,程序尝试重启...", djiStat);
        exit(0);
    }

    this->infoFlag = true;
    while (this->infoFlag)
    {
        // 加锁
        pthread_mutex_lock(&vehicleInfoMutex);

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                          (uint8_t *) &flight,
                                                          sizeof(T_DjiFcSubscriptionFlightStatus),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic flight status error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic flight status error, error code: 0x%08X", djiStat);
            this->droneInfo.flightStatus = -1;
        } else {
            USER_LOG_DEBUG("flight status : %d.", flight);
            this->droneInfo.flightStatus = (int)flight;
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                          (uint8_t *) &displayMode,
                                                          sizeof(T_DjiFcSubscriptionDisplaymode),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            this->droneInfo.displayMode = 0;
            qWarning() << "Get value of topic display mode error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("Get value of topic display mode error, error code: 0x%08X", djiStat);
            displayMode = 0;
        } else {
            this->droneInfo.displayMode = displayMode;
            USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                           timestamp.microsecond);
            USER_LOG_DEBUG("Display mode : %d.", displayMode);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA,
                                                          (uint8_t *) &threeGimbalData,
                                                          sizeof(T_DjiFcSubscriptionThreeGimbalData),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic gimbal data error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic gimbal data error, error code: 0x%08X", djiStat);
            this->droneInfo.gimbalMode = -1;
            this->droneInfo.gimbalStatus = -1;
            this->droneInfo.gimbalYaw = 0;
            this->droneInfo.gimbalPitch = 0;
            this->droneInfo.gimbalRoll = 0;
        } else {
            USER_LOG_DEBUG("threeGimbalData  Gimbal pitch = %f, roll = %f, yaw = %f.", threeGimbalData.gbData[0].pitch, threeGimbalData.gbData[0].roll, threeGimbalData.gbData[0].yaw);
            this->droneInfo.gimbalMode = int(threeGimbalData.gbData[0].mode);
            this->droneInfo.gimbalStatus = int(threeGimbalData.gbData[0].status);
            this->droneInfo.gimbalYaw = threeGimbalData.gbData[0].yaw;
            this->droneInfo.gimbalPitch = threeGimbalData.gbData[0].pitch;
            this->droneInfo.gimbalRoll = threeGimbalData.gbData[0].roll;
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic velocity error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic velocity error, error code: 0x%08X", djiStat);
            this->droneInfo.velocityX = 0.0;
            this->droneInfo.velocityY = 0.0;
            this->droneInfo.velocityZ = 0.0;
        } else {
            USER_LOG_DEBUG("velocity: x = %f y = %f z = %f healthFlag = %d.", velocity.data.x, velocity.data.y,
                          velocity.data.z, velocity.health);
            this->droneInfo.velocityX = (float)velocity.data.x;
            this->droneInfo.velocityY = (float)velocity.data.y;
            this->droneInfo.velocityZ = (float)velocity.data.z;
        }

        //相对高度，10m内有效
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                          (uint8_t *) &relativeHeight,
                                                          sizeof(T_DjiFcSubscriptionHeightFusion),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic relativeHeight error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic relativeHeight error, error code: 0x%08X", djiStat);
            this->droneInfo.relativeHeight = 0;
        } else {
            USER_LOG_DEBUG("relativeHeight: &d.", relativeHeight);
            this->droneInfo.relativeHeight = qAbs((float)relativeHeight);
        }

        //融合海拔高度获取(气压计)-----计算相对起飞点高度
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                          (uint8_t *) &altitudeFused,
                                                          sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic fused altitude error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic fused altitude error, error code: 0x%08X", djiStat);
            altitudeFused = 0;
            this->droneInfo.relativeAltitude = 0;
        } else {
            USER_LOG_DEBUG("fused altitude: &d.", altitudeFused);
            //返航点海拔高度
            djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                              (uint8_t *) &altitudeOfHomePoint,
                                                              sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                              &timestamp);
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                qWarning() << "Get value of topic fused altitude of home Point error, error code: 0x" << QString().sprintf("%08lX", djiStat);
                USER_LOG_ERROR("get value of topic fused altitude of home Point error, error code: 0x%08X", djiStat);
                altitudeOfHomePoint = 0;
                this->droneInfo.relativeAltitude = 0;
            } else {
                USER_LOG_DEBUG("fused altitude of home Point: &d.", altitudeOfHomePoint);
                this->droneInfo.relativeAltitude = qAbs((float)altitudeFused - (float)altitudeOfHomePoint);
            }
        }

        //融合数据获取
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                          (uint8_t *) &position,
                                                          sizeof(T_DjiFcSubscriptionPositionFused),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic position error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic position error, error code: 0x%08X", djiStat);
            this->droneInfo.latitude = 0.0;
            this->droneInfo.longitude = 0.0;
            this->droneInfo.fusedAltitude = 0.0;
            this->droneInfo.visibleSatelliteNumber = 0;
        } else {
            USER_LOG_DEBUG("position: longitude = %f, latitude = %f, fusedAltitude = %f.", position.longitude, position.latitude, position.altitude);
            //qDebug() << QString().sprintf("position: longitude = %f, latitude = %f, fusedAltitude = %f, altitudeOfHomePoint = %f.", position.longitude, position.latitude, position.altitude, altitudeOfHomePoint);
            this->droneInfo.latitude = (float)position.latitude;
            this->droneInfo.longitude = (float)position.longitude;
            this->droneInfo.fusedAltitude = (float)position.altitude;
            this->droneInfo.visibleSatelliteNumber = (int)position.visibleSatelliteNumber;
        }

        //RTK > 融合数据 > GPS  ----- (任务执行中，位置及海拔采用RTK数据)
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkPositionInfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "Get value of topic RTK position info error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic RTK position info error, error code: 0x%08X", djiStat);
        } else {
            USER_LOG_DEBUG("RTK position info : %d.", rtkPositionInfo);
        }
        if(rtkPositionInfo == 50){
            //RTK status
            this->droneInfo.RTKConnectStatus = 1;
            //获取RTK定位信息
            djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                             (uint8_t *) &rtkPosition,
                                                             sizeof(T_DjiFcSubscriptionRtkPosition),
                                                             &timestamp);
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
               qWarning() << "Get value of topic rtk position error, error code: 0x" << QString().sprintf("%08lX", djiStat);
               USER_LOG_ERROR("get value of topic rtk position error, error code: 0x%08X", djiStat);
            } else {
               USER_LOG_DEBUG("rtk position: longitude = %f, latitude = %f, hfsl = %f.", rtkPosition.longitude, rtkPosition.latitude, rtkPosition.hfsl);
               //qDebug() << QString().sprintf("rtk position: longitude = %f, latitude = %f, hfsl = %f, altitudeOfHomePoint = %f.", rtkPosition.longitude, rtkPosition.latitude, rtkPosition.hfsl, altitudeOfHomePoint);
                this->droneInfo.latitude = (float)rtkPosition.latitude * M_PI / 180;
                this->droneInfo.longitude = (float)rtkPosition.longitude * M_PI / 180;
                this->droneInfo.fusedAltitude = (float)rtkPosition.hfsl;
            }
        }
        else{
            this->droneInfo.RTKConnectStatus = 0;
        }


        //Attention: if you want to subscribe the single battery info on M300 RTK, you need connect USB cable to
        //OSDK device or use topic DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO instead.
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "et value of topic battery single info index1 error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic battery single info index1 error, error code: 0x%08X", djiStat);
            this->droneInfo.firstBatteryCapacityPercent = 0;
            this->droneInfo.firstBatteryTemperature = 0;
            this->droneInfo.firstBatteryVoltage = 0;
        } else {
            USER_LOG_DEBUG(
                "battery single info index1: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
            this->droneInfo.firstBatteryCapacityPercent = (int)(singleBatteryInfo.batteryCapacityPercent);
            this->droneInfo.firstBatteryTemperature = (int)(singleBatteryInfo.batteryTemperature/10);
            this->droneInfo.firstBatteryVoltage = (int)(singleBatteryInfo.currentVoltage/1000);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qWarning() << "et value of topic battery single info index2 error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            USER_LOG_ERROR("get value of topic battery single info index2 error, error code: 0x%08X", djiStat);
            this->droneInfo.firstBatteryCapacityPercent = 0;
            this->droneInfo.firstBatteryTemperature = 0;
            this->droneInfo.firstBatteryVoltage = 0;
        } else {
            USER_LOG_DEBUG(
                "battery single info index2: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\r\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
            this->droneInfo.secondBatteryCapacityPercent = (int)(singleBatteryInfo.batteryCapacityPercent);
            this->droneInfo.secondBatteryTemperature = (int)(singleBatteryInfo.batteryTemperature/10);
            this->droneInfo.secondBatteryVoltage = (int)(singleBatteryInfo.currentVoltage/1000);
        }
        //获取网络延迟
        string result = Tools::getCmdResult(netWorkQualityCommand);
        this->droneInfo.linkQuality = (result == "" ? -1 : stof(result));
        //采集时间
        this->droneInfo.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();

        Q_EMIT sendDroneInfo(this->droneInfo);

        //更新共享内存
        if(sem_wait(&(g_shared_sem->droneInfoSem)) == -1)        //sem_wait为P操作，减少信号量的值
        {
            qCritical() << "信号量P操作 ERROR!";
        }
        else{
            memcpy(g_shared_drone, &this->droneInfo, sizeof(this->droneInfo));
            sem_post(&g_shared_sem->droneInfoSem);                           //V 操作增加信号量
        }

        // 解锁
        pthread_mutex_unlock(&vehicleInfoMutex);

        osalHandler->TaskSleepMs(500 / FC_SUBSCRIPTION_TASK_FREQ);
    }

    qDebug() << "Deinit fc subscription module";
    USER_LOG_INFO("Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return false;
    }

    return true;
}

bool VehicleControl::getCameraInfo(){
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiCameraManagerFirmwareVersion cameraFirmwareVersion;    //相机固件版本
    E_DjiCameraManagerWorkMode cameraWorkMode;                  //相机工作模式
    E_DjiCameraManagerShootPhotoMode cameraShootPhotoMode;      //相机拍照模式
    E_DjiCameraManagerFocusMode cameraFocusMode;                //相机聚焦模式
    bool tapZoomEnabled;                                        //相机指点变焦功能是否可用
    uint8_t tapZoomMultiplier;                                  //相机指点变焦
    T_DjiCameraManagerOpticalZoomParam opticalZoomParam;        //相机当前变焦信息(currentOpticalZoomFactor/maxOpticalZoomFactor)

    qDebug() << "Get camera type";
    USER_LOG_INFO("Get camera type");
    T_DjiReturnCode returnCode = DjiCameraManager_GetCameraType(this->mountPosition, &this->cameraType);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qWarning() << QString().sprintf("相机类型获取失败,错误码:0x%08lX,2s后重新获取!", returnCode);
        //等待两秒后重新获取相机类型
        osalHandler->TaskSleepMs(2000);
        T_DjiReturnCode returnCode = DjiCameraManager_GetCameraType(this->mountPosition, &this->cameraType);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            this->sendDeviceAbnormal(1, "相机类型获取失败");
            qCritical() << QString().sprintf("相机类型获取失败,错误码:0x%08lX,程序尝试重启...", returnCode);
            exit(0);
        }
    }
    this->cameraInfo.cameraType = this->cameraType;
    USER_LOG_INFO("camera's type is %d", this->cameraType);

    qDebug() << "Get camera firmware version";
    USER_LOG_INFO("Get camera firmware version");
    returnCode = DjiCameraManager_GetFirmwareVersion(this->mountPosition, &cameraFirmwareVersion);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(1, "相机固件版本获取失败");
        qCritical() << "Get camera's firmware version failed,error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Get mounted position %d camera's firmware version failed, error code: 0x%08X\r",
                       this->mountPosition, returnCode);
    }
    this->cameraInfo.cameraFirmwareVersion = to_string(cameraFirmwareVersion.firmware_version[0]) + "." + to_string(cameraFirmwareVersion.firmware_version[1]) + "." +
            to_string(cameraFirmwareVersion.firmware_version[2]) + "." + to_string(cameraFirmwareVersion.firmware_version[3]);
    USER_LOG_INFO("Mounted position %d camera's firmware is V%d.%d.%d.%d\r", this->mountPosition,
                  cameraFirmwareVersion.firmware_version[0], cameraFirmwareVersion.firmware_version[1],
                  cameraFirmwareVersion.firmware_version[2], cameraFirmwareVersion.firmware_version[3]);

    qDebug() << "Get camera's tap zoom param";
    USER_LOG_INFO("Get camera's tap zoom param");
    returnCode = DjiCameraManager_SetTapZoomEnabled(this->mountPosition, true);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(1, "启用指点变焦失败");
        qCritical() << "enable camera's tap zoom param failed,error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("enable camera's tap zoom param failed, error code: 0x%08X\r",
                       this->mountPosition, returnCode);
    }
    returnCode = DjiCameraManager_GetTapZoomEnabled(this->mountPosition, &tapZoomEnabled);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(1, "指点变焦可用性获取失败");
        qCritical() << "Get camera's tap zoom param failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Get camera's tap zoom param failed, error code: 0x%08X\r",
                       this->mountPosition, returnCode);
        tapZoomEnabled = false;
    }
    this->cameraInfo.tapZoomEnabled = tapZoomEnabled;
    USER_LOG_INFO("camera's tap zoom enabled is %d", tapZoomEnabled);

    if(tapZoomEnabled){
        qDebug() << "Get camera's tap zoom multiplier";
        USER_LOG_INFO("Get camera's tap zoom multiplier");
        returnCode = DjiCameraManager_GetTapZoomMultiplier(this->mountPosition, &tapZoomMultiplier);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            this->sendDeviceAbnormal(1, "指点变焦倍数获取失败");
            qCritical() << "Get camera's tap zoom multiplier failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get camera's tap zoom multiplier failed, error code: 0x%08X\r",
                           this->mountPosition, returnCode);
            tapZoomMultiplier = 1;
        }
        this->cameraInfo.tapZoomMultiplier = tapZoomMultiplier;
        USER_LOG_INFO("camera's tap zoom multiplier is %d", tapZoomMultiplier);
    }

    this->cameraFlag = true;
    while (this->cameraFlag)
    {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        // 加锁
        pthread_mutex_lock(&cameraInfoMutex);

        USER_LOG_DEBUG("Get camera work mode");
        returnCode = DjiCameraManager_GetMode(this->mountPosition, &cameraWorkMode);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Get camera's WorkMode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get mounted position %d camera's WorkMode failed, error code: 0x%08X\r",
                           this->mountPosition, returnCode);
            cameraWorkMode = DJI_CAMERA_MANAGER_WORK_MODE_WORK_MODE_UNKNOWN;
        }
        this->cameraInfo.cameraWorkMode = cameraWorkMode;
        USER_LOG_DEBUG("camera's WorkMode is %d", cameraWorkMode);

        USER_LOG_DEBUG("Get camera shoot photo mode");
        returnCode = DjiCameraManager_GetShootPhotoMode(this->mountPosition, &cameraShootPhotoMode);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Get camera's shoot photo mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get mounted position %d camera's shoot photo mode failed, error code: 0x%08X\r",
                           this->mountPosition, returnCode);
            cameraShootPhotoMode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_UNKNOWN;
        }
        this->cameraInfo.cameraShootPhotoMode = cameraShootPhotoMode;
        USER_LOG_DEBUG("camera's shoot photo mode is %d", cameraShootPhotoMode);

        if(this->cameraSource == ((this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_ZOOM : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM)){
            USER_LOG_DEBUG("Get camera focus mode");
            returnCode = DjiCameraManager_GetFocusMode(this->mountPosition, &cameraFocusMode);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                qCritical() << "Get camera's focus mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
                USER_LOG_ERROR("Get mounted position %d camera's focus mode failed, error code: 0x%08X\r",
                               this->mountPosition, returnCode);
                cameraFocusMode = DJI_CAMERA_MANAGER_FOCUS_MODE_UNKNOWN;
            }
            this->cameraInfo.cameraFocusMode = cameraFocusMode;
            USER_LOG_DEBUG("camera's focus mode is %d", cameraFocusMode);
        }

        USER_LOG_DEBUG("Get camera's zoom param");
        returnCode = DjiCameraManager_GetOpticalZoomParam(this->mountPosition, &opticalZoomParam);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Get camera's zoom param failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get mounted position %d camera's zoom param failed, error code: 0x%08X\r",
                           this->mountPosition, returnCode);
            opticalZoomParam.currentOpticalZoomFactor = 0;
            opticalZoomParam.maxOpticalZoomFactor = 0;
        }
        this->cameraInfo.currentOpticalZoomFactor = opticalZoomParam.currentOpticalZoomFactor;
        this->cameraInfo.maxOpticalZoomFactor = opticalZoomParam.maxOpticalZoomFactor;
        USER_LOG_DEBUG("camera's currentOpticalZoomFactor is %f, maxOpticalZoomFactor is %f", opticalZoomParam.currentOpticalZoomFactor, opticalZoomParam.maxOpticalZoomFactor);

        this->cameraInfo.cameraSource = this->cameraSource;
        this->cameraInfo.cameraRecordStatus = this->cameraRecordStatus;
        this->cameraInfo.cameraDownloadStatus = this->cameraDownloadStatus;
        this->cameraInfo.cameraZoomStatus = this->cameraZoomStatus;

        //发送数据
        Q_EMIT sendCameraInfo(this->cameraInfo);

        // 解锁
        pthread_mutex_unlock(&cameraInfoMutex);
    }
}

void VehicleControl::getLaserRangingInfo(){
    T_DjiCameraManagerLaserRangingInfo laserRangingInfo;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    while(true){
        returnCode = DjiCameraManager_GetLaserRangingInfo(this->mountPosition, &laserRangingInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("GetLaserRangingInfo error");
        }
        cout << "longitude:" << laserRangingInfo.longitude << ",latitude:" << laserRangingInfo.latitude << ",altitude:" << laserRangingInfo.altitude <<",distance:" << laserRangingInfo.distance
             <<",screenX:" << laserRangingInfo.screenX << ",screenY:" << laserRangingInfo.screenY << ",enable_lidar:" << laserRangingInfo.enable_lidar << ",exception:" <<laserRangingInfo.exception << endl;
        osalHandler->TaskSleepMs(1000);
    }
}

////电源管理
T_DjiReturnCode VehicleControl::powerOffNotificationCallback(bool *powerOffPreparationFlag)
{
    USER_LOG_INFO("无人机即将关机");
    qWarning() << "无人机即将关机";

    *powerOffPreparationFlag = true;

    static_vehicleControl->sendPowerOff();
    static_vehicleControl->slotDeInitVehicle();
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void VehicleControl::sendPowerOff(){
    Q_EMIT sendAnalysisDeInit();
}

////无人机控制相关
///云台控制
void VehicleControl::slotResetGimbal(){
    T_DjiReturnCode returnCode = DjiGimbalManager_Reset(this->mountPosition);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendReturn(0, 0, 0, "云台控制失败");
        qCritical() << "reset gimbal manager failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("reset gimbal manager failed, error code: 0x%08X", returnCode);
    }
    this->sendReturn(0, 0, 1, "");
    g_usleep(1000000);
}

void VehicleControl::slotRotateGimbalPitchPositive(int angle){
    T_DjiGimbalManagerRotation rotation;
    rotation.pitch = angle;
    rotation.yaw = 0;
    rotation.roll = 0;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.time = 1.0 + 0.1 * angle;
    T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(this->mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendReturn(0, 1, 0, "云台控制失败");
        qCritical() << "gimbal rotate failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("gimbal rotate failed, error code: 0x%08X", returnCode);
    }
    this->sendReturn(0, 1, 1, "");
}

void VehicleControl::slotRotateGimbalPitchNegative(int angle){
    T_DjiGimbalManagerRotation rotation;
    rotation.pitch = -angle;
    rotation.yaw = 0;
    rotation.roll = 0;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.time = 1.0 + 0.1 * angle;
    T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(this->mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendReturn(0, 2, 0, "云台控制失败");
        qCritical() << "gimbal rotate failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("gimbal rotate failed, error code: 0x%08X", returnCode);
    }
    this->sendReturn(0, 2, 1, "");
}

void VehicleControl::slotRotateGimbalYawPositive(int angle){
    T_DjiGimbalManagerRotation rotation;
    rotation.pitch = 0;
    rotation.yaw = angle;
    rotation.roll = 0;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.time = 1.0 + 0.1 * angle;
    T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(this->mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendReturn(0, 4, 0, "云台控制失败");
        qCritical() << "gimbal rotate failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("gimbal rotate failed, error code: 0x%08X", returnCode);
    }
    this->sendReturn(0, 4, 1, "");
}

void VehicleControl::slotRotateGimbalYawNegative(int angle){
    T_DjiGimbalManagerRotation rotation;
    rotation.pitch = 0;
    rotation.yaw = -angle;
    rotation.roll = 0;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.time = 1.0 + 0.1 * angle;
    T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(this->mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendReturn(0, 3, 0, "云台控制失败");
        qCritical() << "gimbal rotate failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("gimbal rotate failed, error code: 0x%08X", returnCode);
    }
    this->sendReturn(0, 3, 1, "");
}

///相机控制
void VehicleControl::slotChangeLiveViewCameraSource(string cameraSourceName){
    if(this->cameraType == DJI_CAMERA_TYPE_H20 || this->cameraType == DJI_CAMERA_TYPE_H20T){
        if(cameraSourceName == "WIDE"){
            this->cameraSource = (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE;
            this->sendReturn(1, 0, 1, "");
        }
        else if(cameraSourceName == "ZOOM"){
            this->cameraSource = (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_ZOOM : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM;
            this->sendReturn(1, 0, 1, "");
        }
        else if(cameraSourceName == "IR"){
            if(this->cameraType == DJI_CAMERA_TYPE_H20T){
                this->cameraSource = DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR;
                this->sendReturn(1, 0, 1, "");
            }
            else{
                this->sendReturn(1, 0, 0, "当前相机不支持红外");
                qCritical() << "camera not support IR";
                USER_LOG_ERROR("camera not support IR");
            }
        }
        else if(cameraSourceName == "FPV"){
            this->cameraSource = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;
            this->sendReturn(1, 0, 1, "");
        }
        else{
            this->sendReturn(1, 0, 0, "参数错误");
            qCritical() << "parerameter not supprted";
            USER_LOG_ERROR("parerameter not supprted");
        }
    }
    else{
        this->sendReturn(1, 0, 0, "相机不支持");
        qCritical() << "camera not supprted";
        USER_LOG_ERROR("camera not supprted");
    }
}

void VehicleControl::slotShootSinglePhoto(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if(this->cameraRecordStatus != 1){
        /*!< set camera work mode as shoot photo */
        USER_LOG_INFO("Set mounted position %d camera's work mode as shoot-photo mode", this->mountPosition);
        returnCode = DjiCameraManager_SetMode(this->mountPosition, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
            returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
            qCritical() << "set camera's work mode as shoot-photo mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("set mounted position %d camera's work mode as shoot-photo mode failed,"
                           " error code :0x%08X", this->mountPosition, returnCode);
            this->sendReturn(1, 2, 0, "相机拍照控制失败");
            return;
        }

        /*!< set shoot-photo mode */
        USER_LOG_INFO("Set mounted position %d camera's shoot photo mode as single-photo mode", this->mountPosition);
        returnCode = DjiCameraManager_SetShootPhotoMode(this->mountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
            returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
            qCritical() << "set camera's hoot photo mode as single-photo mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("set mounted position %d camera's shoot photo mode as single-photo mode failed,"
                           " error code :0x%08X", this->mountPosition, returnCode);
            this->sendReturn(1, 2, 0, "相机拍照控制失败");
            return;
        }

        /*! wait the APP change the shoot-photo mode display */
        USER_LOG_INFO("Sleep 0.5s...");
        osalHandler->TaskSleepMs(500);

        /*!< start to shoot single photo */
        USER_LOG_INFO("Mounted position %d camera start to shoot photo", this->mountPosition);
        returnCode = DjiCameraManager_StartShootPhoto(this->mountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "camera shoot photo failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera shoot photo failed, "
                           "error code :0x%08X", this->mountPosition, returnCode);
            this->sendReturn(1, 2, 0, "相机拍照控制失败");
            return;
        }
    }
    else{
        //录像状态中可直接进行拍照
        /*!< start to shoot single photo */
        USER_LOG_INFO("Mounted position %d camera start to shoot photo", this->mountPosition);
        returnCode = DjiCameraManager_StartShootPhoto(this->mountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "camera shoot photo failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera shoot photo failed, "
                           "error code :0x%08X", this->mountPosition, returnCode);
            this->sendReturn(1, 2, 0, "相机拍照控制失败");
            return;
        }
    }
    this->sendReturn(1, 2, 1, "");
}

void VehicleControl::slotShootAEBPhoto(int count){
    if(this->cameraType == DJI_CAMERA_TYPE_H20T){
        USER_LOG_ERROR("H20T camera not support AEB Photo.");
        return;
    }
    if(this->cameraRecordStatus == 1){
        USER_LOG_ERROR("the camera is recording now, can not shoot AEB photo.");
        return;
    }
    if(!(count == 3 || count == 5 || count == 7)){
        USER_LOG_ERROR("AEB count not correct.");
        return;
    }
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiCameraManagerPhotoAEBCount aebCount = (E_DjiCameraManagerPhotoAEBCount)count;

    /*!< set camera work mode as shoot photo */
    USER_LOG_INFO("set mounted position %d camera's work mode as shoot photo mode.", this->mountPosition);
    returnCode = DjiCameraManager_SetMode(this->mountPosition, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("set mounted position %d camera's work mode as shoot photo mode failed,"
                       " error code :0x%08X.", this->mountPosition, returnCode);
        return;
    }

    /*!< set shoot-photo mode */
    USER_LOG_INFO("Set mounted position %d camera's shoot photo mode as AEB-photo mode", this->mountPosition);
    returnCode = DjiCameraManager_SetShootPhotoMode(this->mountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
    if (returnCode == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        return;
    }

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("set mounted position %d camera's shoot photo mode as AEB-photo mode failed,"
                       " error code :0x%08X.", this->mountPosition, returnCode);
        return;
    }

    /*! wait the APP change the shoot-photo mode display */
    USER_LOG_INFO("Sleep 0.5s...");
    osalHandler->TaskSleepMs(500);

    /*!< set shoot-photo mode parameter */
    USER_LOG_INFO("Set mounted position %d camera's AEB count to %d", this->mountPosition, aebCount);
    returnCode = DjiCameraManager_SetPhotoAEBCount(this->mountPosition, aebCount);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("set mounted position %d camera's AEB count(%d) failed,"
                       " error code :0x%08X.", this->mountPosition, aebCount, returnCode);
        return;
    }
    /*!< start to shoot single photo */
    USER_LOG_INFO("Mounted position %d camera start to shoot photo.", this->mountPosition);
    returnCode = DjiCameraManager_StartShootPhoto(this->mountPosition, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mounted position %d camera shoot photo failed, "
                       "error code :0x%08X.", this->mountPosition, returnCode);
    }

    return;
}

void VehicleControl::slotChangeCameraWorkMode(int workMode){
    T_DjiReturnCode returnCode;
    USER_LOG_INFO("set mounted position %d camera's work mode as %d", this->mountPosition, (E_DjiCameraManagerWorkMode)workMode);
    returnCode = DjiCameraManager_SetMode(this->mountPosition, (E_DjiCameraManagerWorkMode)workMode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "change camera's work mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("set mounted position %d camera's work mode as %d mode failed,"
                       " error code :0x%08X", this->mountPosition, workMode, returnCode);
        this->sendReturn(1, 1, 0, "相机工作模式切换失败");
        return;
    }
    this->sendReturn(1, 1, 1, "");
}

void VehicleControl::slotStartRecordVideo(){
   T_DjiReturnCode returnCode;
   /*!< set camera work mode as record video */
   USER_LOG_INFO("set mounted position %d camera's work mode as record-video mode", this->mountPosition);
   returnCode = DjiCameraManager_SetMode(this->mountPosition, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
   if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
       returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
       qCritical() << "set camera's work mode as record-video mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
       USER_LOG_ERROR("set mounted position %d camera's work mode as record-video mode failed,"
                      " error code :0x%08X", this->mountPosition, returnCode);
       this->sendReturn(1, 3, 0, "相机开始录像控制失败");
       return;
   }

   /*!< start to take video */
   USER_LOG_INFO("Mounted position %d camera start to record video.", this->mountPosition);
   returnCode = DjiCameraManager_StartRecordVideo(this->mountPosition);
   if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
       returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
       qCritical() << "camera start to record video failed, error code:0x" << QString().sprintf("%08lX", returnCode);
       USER_LOG_ERROR("Mounted position %d camera start to record video failed,"
                      " error code:0x%08X.", this->mountPosition, returnCode);
       this->sendReturn(1, 3, 0, "相机开始录像控制失败");
       return;
   }
   this->cameraRecordStatus = 1;
   this->sendReturn(1, 3, 1, "");
}

void VehicleControl::slotStopRecordVideo(){
    T_DjiReturnCode returnCode;
    USER_LOG_INFO("Mounted position %d camera stop to record video.", this->mountPosition);
    returnCode = DjiCameraManager_StopRecordVideo(this->mountPosition);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "camera stop to record video failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Mounted position %d camera stop to record video failed,"
                       " error code:0x%08X.", this->mountPosition, returnCode);
        this->sendReturn(1, 4, 0, "相机停止录像控制失败");
        return;
    }
    this->cameraRecordStatus = 0;
    this->sendReturn(1, 4, 1, "");
}

void VehicleControl::slotStartContinuousZoom(int direction){
    if(this->cameraSource != (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE){
        this->slotChangeLiveViewCameraSource("ZOOM");
    }
    if(this->cameraZoomStatus == 1){
        this->slotStopContinuousZoom();
    }
    USER_LOG_INFO("Mounted position %d camera start continuous zoom with zoom-out direction and normal zoom speed.", this->mountPosition);
    T_DjiReturnCode returnCode = DjiCameraManager_StartContinuousOpticalZoom(this->mountPosition, (E_DjiCameraZoomDirection)direction, DJI_CAMERA_ZOOM_SPEED_NORMAL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mounted position %d camera start continuous zoom failed,"
                       "error code: 0x%08X\r\n", this->mountPosition, returnCode);
        return;
    }
    this->cameraZoomStatus = 1;
}

void VehicleControl::slotStopContinuousZoom(){
    if(this->cameraSource != (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE){
        this->slotChangeLiveViewCameraSource("ZOOM");
    }
    USER_LOG_INFO("Mounted position %d camera stop continuous zoom.", this->mountPosition);
    T_DjiReturnCode returnCode = DjiCameraManager_StopContinuousOpticalZoom(this->mountPosition);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mounted position %d camera stop continuous zoom failed,"
                       "error code: 0x%08X\r\n", this->mountPosition, returnCode);
        return;
    }
    this->cameraZoomStatus = 0;
}

void VehicleControl::slotSetOpticalZoomParam(float factor){
    if(this->cameraSource != (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE){
        this->slotChangeLiveViewCameraSource("ZOOM");
    }
    if(this->cameraZoomStatus){
        this->slotStopContinuousZoom();
    }
    USER_LOG_INFO("Mounted position %d camera set zoom param.", this->mountPosition);
    E_DjiCameraZoomDirection direction = factor > this->cameraInfo.currentOpticalZoomFactor ? DJI_CAMERA_ZOOM_DIRECTION_OUT : DJI_CAMERA_ZOOM_DIRECTION_IN;
    T_DjiReturnCode returnCode = DjiCameraManager_SetOpticalZoomParam(this->mountPosition, direction, factor);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "camera set zoom param failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Mounted position %d camera set zoom param failed,"
                       "error code: 0x%08X\r\n", this->mountPosition, returnCode);
        this->sendReturn(1, 5, 0, "相机变焦执行失败");
        return;
    }
    this->sendReturn(1, 5, 1, "");
}

void VehicleControl::slotSetTapZoomPoint(CameraTapZoomPoint tapZoomPoint){
    if(this->cameraSource != (this->cameraType == DJI_CAMERA_TYPE_H20) ? DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE : DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE){
        this->slotChangeLiveViewCameraSource("ZOOM");
    }
    if(this->cameraZoomStatus){
        this->slotStopContinuousZoom();
    }
    T_DjiReturnCode returnCode;
    T_DjiCameraManagerTapZoomPosData tapZoomPosData;
    tapZoomPosData.focusX = tapZoomPoint.x;
    tapZoomPosData.focusY = tapZoomPoint.y;

    /*!< set camera tap zoom enable parameter to be enable */
    USER_LOG_INFO("Enable mounted position %d camera's tap zoom status.",
                  this->mountPosition);
    returnCode = DjiCameraManager_SetTapZoomEnabled(this->mountPosition, true);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "Enable camera's tap zoom function failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Enable mounted position %d camera's tap zoom function failed,"
                       " error code :0x%08X.", this->mountPosition, returnCode);
        this->sendReturn(1, 6, 0, "相机变焦执行失败");
        return;
    }

    /*!< set camera tap zoom multiplier parameter */
    USER_LOG_INFO("Set mounted position %d camera's tap zoom multiplier to %d x.",
                  this->mountPosition, tapZoomPoint.multiplier);
    returnCode = DjiCameraManager_SetTapZoomMultiplier(this->mountPosition, tapZoomPoint.multiplier);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "Set camera's tap zoom multiplier failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Set mounted position %d camera's tap zoom multiplier(%d) failed,"
                       " error code :0x%08X.", this->mountPosition, tapZoomPoint.multiplier, returnCode);
        this->sendReturn(1, 6, 0, "相机变焦执行失败");
        return;
    }

    USER_LOG_INFO("Set mounted position %d camera's tap zoom point to (%f, %f).",
                  this->mountPosition, tapZoomPosData.focusX, tapZoomPosData.focusY);
    returnCode = DjiCameraManager_TapZoomAtTarget(this->mountPosition, tapZoomPosData);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "Set camera's tap zoom target failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Set mounted position %d camera's tap zoom target point(%f ,%f) failed,"
                       " error code :0x%08X.", this->mountPosition, tapZoomPosData.focusX, tapZoomPosData.focusY,
                       returnCode);
        this->sendReturn(1, 6, 0, "相机变焦执行失败");
        return;
    }
    this->sendReturn(1, 6, 1, "");
}

void VehicleControl::slotSetFoucsPoint(CameraFocusPoint focusPoint){
    T_DjiReturnCode returnCode;
    T_DjiCameraManagerFocusPosData focusPosData;
    focusPosData.focusX = focusPoint.x;
    focusPosData.focusY = focusPoint.y;

    /*!< set camera focus mode to be CameraModule::FocusMode::AUTO */
    USER_LOG_INFO("Set mounted position %d camera's focus mode to auto mode.",
                  this->mountPosition);
    returnCode = DjiCameraManager_SetFocusMode(this->mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "Set camera's focus mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Set mounted position %d camera's focus mode(%d) failed,"
                       " error code :0x%08X.", this->mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
                       returnCode);
        this->sendReturn(1, 7, 0, "相机焦点设置失败");
        return;
    }

    USER_LOG_INFO("Set mounted position %d camera's focus point to (%0.1f, %0.1f).",
                  this->mountPosition, focusPosData.focusX, focusPosData.focusY);
    returnCode = DjiCameraManager_SetFocusTarget(this->mountPosition, focusPosData);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        qCritical() << "Set camera's focus point failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        USER_LOG_ERROR("Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
                       " error code :0x%08X.", this->mountPosition, focusPosData.focusX, focusPosData.focusY,
                       returnCode);
        this->sendReturn(1, 7, 0, "相机焦点设置失败");
        return;
    }
    this->sendReturn(1, 7, 1, "");
}

void VehicleControl::slotSetFoucsAuto(){
    USER_LOG_INFO("Mounted position %d camera set focus auto.", this->mountPosition);
    T_DjiReturnCode returnCode = DjiCameraManager_SetFocusMode(this->mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mounted position %d camera set focus auto failed,"
                       "error code: 0x%08X\r\n", this->mountPosition, returnCode);
        return;
    }
}

void VehicleControl::slotGetDownloadFileList(){
    if(this->cameraDownloadTid > 0){
        pthread_cancel(this->cameraDownloadTid);
    }
    this->cameraDownloadThread = std::thread([this]{
        USER_LOG_INFO("Mounted position %d camera Get DownloadFile List.", this->mountPosition);
        T_DjiCameraManagerFileList fileList;
        T_DjiReturnCode returnCode = DjiCameraManager_DownloadFileList(this->mountPosition, &fileList);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "get camera's DownloadFile List failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera Get DownloadFile List failed,"
                           "error code: 0x%08X\r\n", this->mountPosition, returnCode);
            this->sendReturn(1, 8, 0, "相机文件列表获取失败:接口异常");
            this->cameraDownloadTid = -1;
            return;
        }
        vector<CameraFileInfo> cameraFileListInfo;
        for(int i = 0; i < fileList.totalCount; i++){
            if(fileList.fileListInfo[i].type == 255){
                continue;
            }
//            cout << i << " " <<  fileList.fileListInfo[i].fileIndex << " " << fileList.fileListInfo[i].fileName << " " << fileList.fileListInfo[i].fileSize <<  " " << fileList.fileListInfo[i].type <<  endl;
//            if(i == 7){
//                //this->slotDeleteFile(fileList.fileListInfo[i].fileIndex);
//                this->slotDownloadFile(fileList.fileListInfo[i].fileIndex);
//            }
            CameraFileInfo cameraFileInfo;
            cameraFileInfo.fileIndex = to_string(fileList.fileListInfo[i].fileIndex);
            cameraFileInfo.fileName = fileList.fileListInfo[i].fileName;
            cameraFileInfo.fileType = fileList.fileListInfo[i].type;
            cameraFileInfo.fileSize = fileList.fileListInfo[i].fileSize;
            cameraFileInfo.createTime = to_string(fileList.fileListInfo[i].createTime.year) + "-" +
                    (fileList.fileListInfo[i].createTime.month <= 9 ? ("0" + to_string(fileList.fileListInfo[i].createTime.month)) : to_string(fileList.fileListInfo[i].createTime.month)) + "-" +
                    (fileList.fileListInfo[i].createTime.day <= 9 ? ("0" + to_string(fileList.fileListInfo[i].createTime.day)) : to_string(fileList.fileListInfo[i].createTime.day)) + " " +
                    (fileList.fileListInfo[i].createTime.hour <= 9 ? ("0" + to_string(fileList.fileListInfo[i].createTime.hour)) : to_string(fileList.fileListInfo[i].createTime.hour)) + ":" +
                    (fileList.fileListInfo[i].createTime.minute <= 9 ? ("0" + to_string(fileList.fileListInfo[i].createTime.minute)) : to_string(fileList.fileListInfo[i].createTime.minute)) + ":" +
                    (fileList.fileListInfo[i].createTime.second <= 9 ? ("0" + to_string(fileList.fileListInfo[i].createTime.second)) : to_string(fileList.fileListInfo[i].createTime.second));
            if(cameraFileInfo.fileType == 2 || cameraFileInfo.fileType == 3){
                cameraFileInfo.videoDuration = fileList.fileListInfo[i].attributeData.videoAttribute.attributeVideoDuration;
            }
            cameraFileListInfo.push_back(cameraFileInfo);
        }
        Q_EMIT sendCameraFileList(cameraFileListInfo);
        this->sendReturn(1, 8, 1, "");
        this->cameraDownloadTid = -1;
    });
    this->cameraDownloadTid = this->cameraDownloadThread.native_handle();
    this->cameraDownloadThread.detach();
}

void VehicleControl::slotDeleteFile(uint32_t fileIndex){
    if(this->cameraDownloadTid > 0){
        pthread_cancel(this->cameraDownloadTid);
    }
    this->cameraDownloadThread = std::thread([this, fileIndex]{
        T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
        T_DjiReturnCode returnCode;

        qDebug() << "媒体文件index:" + QString::number(fileIndex)  + "开始删除";

        /*!< set camera work mode as MEDIA DOWNLOAD */
//        USER_LOG_INFO("set mounted position %d camera's work mode as MEDIA DOWNLOAD mode.", this->mountPosition);
//        returnCode = DjiCameraManager_SetMode(this->mountPosition, DJI_CAMERA_MANAGER_WORK_MODE_MEDIA_DOWNLOAD);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
//            returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
//            qCritical() << "set camera's work mode as MEDIA DOWNLOAD mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
//            USER_LOG_ERROR("set mounted position %d camera's work mode as MEDIA DOWNLOAD mode failed,"
//                           " error code :0x%08X.", this->mountPosition, returnCode);
//            this->sendReturn(1, 10, 0, "数据删除失败:接口异常");
//            this->cameraDownloadTid = -1;
//            return;
//        }

        /*! < delete files*/
        USER_LOG_INFO("Mounted position %d camera delete file: %d.", this->mountPosition, fileIndex);
        returnCode = DjiCameraManager_DeleteFileByIndex(this->mountPosition, fileIndex);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "camera delete file failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera delete file: %d failed,"
                           "error code: 0x%08X\r\n", this->mountPosition, fileIndex, returnCode);
            this->sendReturn(1, 10, 0, "数据删除失败:接口异常");
            this->cameraDownloadTid = -1;
            return;
        }
        osalHandler->TaskSleepMs(1000);
        this->sendReturn(1, 10, 1, "");
        this->cameraDownloadTid = -1;
        return;
    });
    this->cameraDownloadTid = this->cameraDownloadThread.native_handle();
    this->cameraDownloadThread.detach();
}

void VehicleControl::slotDownloadFile(uint32_t fileIndex){
    if(this->cameraDownloadTid > 0){
        pthread_cancel(this->cameraDownloadTid);
    }
    this->cameraDownloadThread = std::thread([this, fileIndex]{
        T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
        T_DjiReturnCode returnCode;
        qDebug() << "媒体文件index:" + QString::number(fileIndex)  + "开始下载";
//        writeFile.open("/home/bx/projects/vehicleAnalysis/download/" + to_string(fileIndex) + ".mp4");
//        if (!writeFile)
//        {
//            qDebug() << "存储文件打开失败:" + QString::number(fileIndex);  //打开文件失败
//        }

        /*!< set camera work mode as MEDIA DOWNLOAD */
//        USER_LOG_INFO("set mounted position %d camera's work mode as MEDIA DOWNLOAD mode.", this->mountPosition);
//        returnCode = DjiCameraManager_SetMode(this->mountPosition, DJI_CAMERA_MANAGER_WORK_MODE_MEDIA_DOWNLOAD);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
//            returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
//            qCritical() << "数据下载失败,set camera's work mode as MEDIA DOWNLOAD mode failed, error code:0x" << QString().sprintf("%08lX", returnCode);
//            USER_LOG_ERROR("set mounted position %d camera's work mode as MEDIA DOWNLOAD mode failed,"
//                           " error code :0x%08lX.", this->mountPosition, returnCode);
//            this->sendReturn(1, 9, 0, "数据下载失败:接口异常");
//            static_vehicleControl->writeFile.close();
//            this->cameraDownloadTid = -1;
//            return;
//        }

        /*!< register download file callback */
        USER_LOG_INFO("Mounted position %d camera register download file callback.", this->mountPosition);
        returnCode = DjiCameraManager_RegDownloadFileDataCallback(this->mountPosition, cameraDownloadFileCallback);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "数据下载失败,camera register download file callback failed failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera register download file callback failed,"
                           "error code: 0x%08X\r\n", this->mountPosition, returnCode);
            this->sendReturn(1, 9, 0, "数据下载失败:接口异常");
            static_vehicleControl->writeFile.close();
            this->cameraDownloadTid = -1;
            return;
        }

        /*!< download files */
        USER_LOG_INFO("Mounted position %d camera download file: %d.", this->mountPosition, fileIndex);
        returnCode = DjiCameraManager_DownloadFileByIndex(this->mountPosition, fileIndex);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "数据下载失败, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Mounted position %d camera download file: %d failed,"
                           "error code: 0x%08lX\r\n", this->mountPosition, fileIndex, returnCode);
            this->sendReturn(1, 9, 0, "数据下载失败:接口异常");
            static_vehicleControl->writeFile.close();
            this->cameraDownloadTid = -1;
            return;
        }
        osalHandler->TaskSleepMs(1000);
        qDebug() << "媒体文件index:" + QString::number(fileIndex)  + "下载成功";
        //static_vehicleControl->writeFile.close();
        this->sendReturn(1, 9, 1, "");
        this->cameraDownloadTid = -1;
        return;
    });
    this->cameraDownloadTid = this->cameraDownloadThread.native_handle();
    this->cameraDownloadThread.detach();
}

void VehicleControl::sendCameraFile(DownloadFileInfo downloadFileInfo){
    Q_EMIT sendCameraFileData(downloadFileInfo);
}

T_DjiReturnCode VehicleControl::cameraDownloadFileCallback(T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t dataLen){
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if(packetInfo.downloadFileEvent == 0){
        g_data_integrity_abnormal_count = 0;
        //开始下载
        static_vehicleControl->downloadFileInfoCache = new DownloadFileInfo();       //在堆上分配文件下载缓存
        static_vehicleControl->cameraDownloadStatus = 1;
        static_vehicleControl->downloadFileInfoCacheCount = 0;
        static_vehicleControl->downloadFileInfoFirstCacheStatus = 1;
    }
    if(g_data_integrity_abnormal_count > 20){
        //当失败次数大于20次时，不再推送数据
        static_vehicleControl->downloadFileInfoCacheCount = 0;
        static_vehicleControl->downloadFileInfoFirstCacheStatus = 0;
        if(static_vehicleControl->downloadFileInfoCache != NULL){
            delete static_vehicleControl->downloadFileInfoCache;
            static_vehicleControl->downloadFileInfoCache = NULL;
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    //对数据进行缓存
    //static_vehicleControl->downloadFileInfoCache->data += string((const char*)data, dataLen);         //本地写入数据
    static_vehicleControl->downloadFileInfoCache->data += Tools::base64Encode((const unsigned char*)data, dataLen);
    static_vehicleControl->downloadFileInfoCache->dataLen += dataLen;
    if(packetInfo.downloadFileEvent == 2 && g_data_integrity_abnormal_count > 0){
        static_vehicleControl->downloadFileInfoCache->downloadFileEvent = 3;
    }
    else{
        static_vehicleControl->downloadFileInfoCache->downloadFileEvent = packetInfo.downloadFileEvent;
    }
    static_vehicleControl->downloadFileInfoCache->fileIndex = to_string(packetInfo.fileIndex);
    static_vehicleControl->downloadFileInfoCache->progressInPercent = packetInfo.progressInPercent;
    static_vehicleControl->downloadFileInfoCacheCount += 1;
    if(static_vehicleControl->downloadFileInfoCacheCount <= g_cache_size){
        if(packetInfo.downloadFileEvent == 2){
            //收到最后一个数据
            static_vehicleControl->sendCameraFile(*static_vehicleControl->downloadFileInfoCache);
            //static_vehicleControl->writeFile << static_vehicleControl->downloadFileInfoCache->data;       //本地写入数据
            static_vehicleControl->downloadFileInfoCacheCount = 0;
        }
    }
    else{
        if(static_vehicleControl->downloadFileInfoFirstCacheStatus == 1 && static_vehicleControl->downloadFileInfoCache->downloadFileEvent != 2){
            //第一个缓存片段需将下载事件置0
            static_vehicleControl->downloadFileInfoCache->downloadFileEvent = 0;
            static_vehicleControl->downloadFileInfoFirstCacheStatus = 0;
        }
        static_vehicleControl->sendCameraFile(*static_vehicleControl->downloadFileInfoCache);
        //static_vehicleControl->writeFile << static_vehicleControl->downloadFileInfoCache->data;           //本地写入数据
        static_vehicleControl->downloadFileInfoCacheCount = 0;
        static_vehicleControl->downloadFileInfoCache->data = "";
        static_vehicleControl->downloadFileInfoCache->dataLen = 0;
        static_vehicleControl->downloadFileInfoCache->downloadFileEvent = -1;
        static_vehicleControl->downloadFileInfoCache->fileIndex = "";
        static_vehicleControl->downloadFileInfoCache->progressInPercent = -1;
    }
    if(packetInfo.downloadFileEvent == 2){
        g_data_integrity_abnormal_count = 0;
        //结束下载
        delete static_vehicleControl->downloadFileInfoCache;
        static_vehicleControl->downloadFileInfoCache = NULL;
        static_vehicleControl->cameraDownloadStatus = 0;
        static_vehicleControl->downloadFileInfoCacheCount = 0;
        static_vehicleControl->downloadFileInfoFirstCacheStatus = 0;
    }
}

///飞行控制
void VehicleControl::slotFlightControlTakeoff(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlTakeoff();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlLanding(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlLanding();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlConfirmLanding(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlConfirmLanding();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlForceLanding(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlForceLanding();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightCancelLanding(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightCancelLanding();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlGoHome(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlGoHome();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlGoHomeAndConfirmLanding(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlGoHomeAndConfirmLanding();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlGoHomeAndConfirmLandingNative(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightControlGoHomeAndConfirmLandingNative();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightCancelGoHome(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightCancelGoHome();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlMoveByPositionOffset(DronePositionOffset droneOffset){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this, droneOffset]{
        flightControlMoveByPositionOffset(droneOffset);
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightControlVelocityAndYawRateCtrl(DroneVelocityTimeOffset droneVelocityTimeOffset){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this, droneVelocityTimeOffset]{
        flightControlVelocityAndYawRateCtrl(droneVelocityTimeOffset);
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightPositionTest(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightPositionTest();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotFlightVelocityTest(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        flightVelocityTest();
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::flightControlTakeoff(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    //控制执行前检查
    if(droneInfo.flightStatus == 2){
        USER_LOG_ERROR("drone alreadly takeoff, it is in air.");
        goto out;
    }
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS){
        USER_LOG_ERROR("drone can not takeoff.");
        goto out;
    }
    //控制开始
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    //使用当前飞机 GPS（不是 RTK）位置设置返航位置
    USER_LOG_INFO("--> Step 2: set home location.");
    returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set aircraft current position as new home location failed, error code: 0x%08X", returnCode);
        goto out;
    }
    //设置返航高度
    USER_LOG_INFO("--> Step 3: set go home altitude.");
    returnCode = DjiFlightController_SetGoHomeAltitude(100);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    //起飞
    USER_LOG_INFO("--> Step 4: Start takeoff.");
    returnCode = DjiFlightController_StartTakeoff();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to take off failed, error code: 0x%08X", returnCode);
        goto out;
    }
    //控制检查
    USER_LOG_INFO("--> Step 5: Motors start check.");
    if (!DjiTest_FlightControlMotorStartedCheck()) {
        USER_LOG_ERROR("Takeoff failed. Motors are not spinning.");
        goto out;
    } else {
        USER_LOG_INFO("Motors spinning...");
    }
    USER_LOG_INFO("--> Step 6: In air check.");
    if (!DjiTest_FlightControlTakeOffInAirCheck()) {
        USER_LOG_ERROR("Takeoff failed. Aircraft is still on the ground, but the "
                       "motors are spinning");
        goto out;
    } else {
        USER_LOG_INFO("Ascending...");
    }
    USER_LOG_INFO("--> Step 7: Finished takeoff check.");
    if (!takeoffFinishedCheck()) {
        USER_LOG_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.");
        goto out;
    }
    //控制结束
    out:
        USER_LOG_INFO("take off control end.");
}

void VehicleControl::flightControlLanding(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    //控制执行前检查
    if(droneInfo.flightStatus != 2){
        USER_LOG_ERROR("drone alreadly land, it is on the ground.");
        goto out;
    }
    //控制开始
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    USER_LOG_INFO("--> Step 2: Start landing.");
    returnCode = DjiFlightController_StartLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to landing failed, error code: 0x%08X", returnCode);

        goto out;
    }
    //控制检查
    USER_LOG_INFO("--> Step 3: check Landing start.");
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action!");
        goto out;
    } else {
        USER_LOG_INFO("--> Step 4: check Landing finished.");
        if (!DjiTest_FlightControlLandFinishedCheck()) {
            USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                           "Please connect DJI Assistant.");
            goto out;
        }
    }
    //控制结束
    out:
        USER_LOG_INFO("landing control end.");
}

void VehicleControl::flightControlConfirmLanding(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if(droneInfo.flightStatus != 2){
        USER_LOG_ERROR("drone alreadly land, it is on the ground.");
        goto out;
    }
    if(!(DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)){
        USER_LOG_ERROR("drone is not on the auto landing status, execute failed.");
        goto out;
    }
    /*! Step 1: Confirm Landing */
    USER_LOG_INFO("--> Step 1: Start confirm Landing and avoid ground action");
    returnCode = DjiFlightController_StartConfirmLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", returnCode);
        goto out;
    }
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        goto out;
    } else {

        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
            osalHandler->TaskSleepMs(1000);
        }

    }
    /*! Step 2: Landing finished check*/
    USER_LOG_INFO("--> Step 2: Landing finished check");
    if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        goto out;
    }
    out:
        USER_LOG_INFO("confirm landing control end.");
}

void VehicleControl::flightControlForceLanding(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    //控制执行前检查
    if(droneInfo.flightStatus != 2){
        USER_LOG_ERROR("drone alreadly land, it is on the ground.");
        goto out;
    }
    //控制开始
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    USER_LOG_INFO("--> Step 2: Start force landing.");
    returnCode = DjiFlightController_StartForceLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to force landing failed, error code: 0x%08X", returnCode);
        goto out;
    }
    //控制检查
    USER_LOG_INFO("--> Step 3: check force Landing start.");
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute force Landing action!");
        goto out;
    } else {
        USER_LOG_INFO("--> Step 4: check force Landing finished.");
        if (!DjiTest_FlightControlLandFinishedCheck()) {
            USER_LOG_ERROR("force Landing finished, but the aircraft is in an unexpected mode. "
                           "Please connect DJI Assistant.");
            goto out;
        }
    }
    //控制结束
    out:
        USER_LOG_INFO("force landing control end.");
}

void VehicleControl::flightCancelLanding(){
    T_DjiReturnCode returnCode;
    if(!(DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING || DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)){
        USER_LOG_ERROR("drone is not on the landing status, execute failed.");
        goto out;
    }
    USER_LOG_INFO("Start cancel landing.");
    returnCode = DjiFlightController_CancelLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to cancael landing failed, error code: 0x%08X", returnCode);
        goto out;
    }
    out:
        USER_LOG_INFO("cancel landing end.");
}

void VehicleControl::flightControlGoHome(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    //控制执行前检查
    if(droneInfo.flightStatus != 2){
        USER_LOG_ERROR("drone alreadly land, it is on the ground.");
        goto out;
    }
    //控制开始
    /*! Step 1: Obtain joystick control authority */
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    /*! Step 2: Start go home */
    USER_LOG_INFO("--> Step 2: Start go home action");
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start to go home failed, error code: 0x%08X", returnCode);
        goto out;
    }
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)) {
        goto out;
    } else {
        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }
    //控制结束
    out:
        USER_LOG_INFO("go home control end.");
}

void VehicleControl::flightCancelGoHome(){
    T_DjiReturnCode returnCode;
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME){
        USER_LOG_ERROR("drone is not on the go home status, execute failed.");
        goto out;
    }
    USER_LOG_INFO("Start cancel go home.");
    returnCode = DjiFlightController_CancelGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to cancel go home failed, error code: 0x%08X", returnCode);
        goto out;
    }
    out:
        USER_LOG_INFO("cancel go home end.");
}

void VehicleControl::flightControlGoHomeAndConfirmLanding(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    this->sendLog(1, "无人机收到返航指令");
    //控制执行前检查
    if(droneInfo.flightStatus != 2){
        qCritical() << "返航失败:无人机已在地面";
        this->sendLog(3, "无人机已在地面,无法返航");
        this->sendReturn(2, 0, 0, "返航失败");
        return;
    }
    //控制开始
//    /*! Step 1: Obtain joystick control authority */
//    qDebug() << "--> Step 1: Obtain joystick control authority.";
//    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        qCritical() << QString().sprintf("返航失败:无法获得控制权,错误码 0x%08X",returnCode) ;
//        this->sendLog(3, QString().sprintf("无法获得控制权,错误码 0x%08X", returnCode).toStdString());
//        this->sendReturn(2, 0, 0, "返航失败");
//        return;
//    }
//    osalHandler->TaskSleepMs(1000);
    /*! Step 2: Start go home */
    qDebug() << "--> Step 2: Start go home action";
    this->sendLog(1, "无人机开始返航");
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << QString().sprintf("返航失败:返航接口调用失败,错误码:%08lX", returnCode);
        this->sendLog(3, QString().sprintf("返航接口调用失败,错误码:%08lX", returnCode).toStdString());
        this->sendReturn(2, 0, 0, "返航失败");
        return;
    }
    int checkCount = 0;
    while(!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)){
        if(checkCount > 10){
            qCritical() << "返航失败:返航状态确认失败";
            this->sendLog(3, "返航状态确认失败");
            this->sendReturn(2, 0, 0, "返航失败");
            return;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    qDebug() << "返航状态确认成功,等待降落";
    this->sendLog(1, "返航状态确认成功,等待降落");
    this->sendReturn(2, 0, 1, "");                //返航状态确认成功后即代表一键返航接口调用成功，用于平台对接
    while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
        osalHandler->TaskSleepMs(1000);// waiting for this action finished
    }
    /*! Step 3: Start landing */
    qDebug() << "--> Step 3: Start landing action";
    this->sendLog(1, "开始自动降落");
    checkCount = 0;
    while(!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)){
        if(checkCount > 10){
            qCritical() << "自动降落执行失败,开始强制降落";
            this->sendLog(3, "自动降落执行失败,开始强制降落");
            break;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
           DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        T_DjiFcSubscriptionAvoidData avoidData = DjiTest_FlightControlGetValueOfAvoidData();
        osalHandler->TaskSleepMs(1000);
        //等待降落至0.65至0.75距离时，且可正常降落的情况下进行降落确认
        if (((dji_f64_t) 0.65 < avoidData.down && avoidData.down < (dji_f64_t) 0.75) &&
            avoidData.downHealth == 1) {
            break;
        }
    }
    /*! Step 4: Confirm Landing */
    qDebug() << "--> Step 4: Start confirm Landing and avoid ground action";
    this->sendLog(1, "确认降落");
    returnCode = DjiFlightController_StartConfirmLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << QString().sprintf("确认降落执行失败,错误码:0x%08lX", returnCode);
        this->sendLog(3, QString().sprintf("确认降落执行失败,错误码:0x%08lX", returnCode).toStdString());
        return;
    }
    osalHandler->TaskSleepMs(1000);
    checkCount = 0;
    while(!(DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) ||
            DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING))){
        if(checkCount > 10){
            qCritical() << QString().sprintf("确认降落状态检查失败, 当前飞行模式:%d", this->droneInfo.displayMode);
            this->sendLog(3, QString().sprintf("确认降落状态检查失败, 当前飞行模式:%d", this->droneInfo.displayMode).toStdString());
            return;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    qDebug() << "确认降落执行成功";
    while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
        osalHandler->TaskSleepMs(1000);
    }
    /*! Step 5: Landing finished check*/
    qDebug() << "--> Step 5: Landing finished check";
    if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        qDebug() << "降落结束";
        this->sendLog(1, "降落结束");
    } else {
        qCritical() << "降落结束，但无人机处于异常状态，请联系维护人员.";
        this->sendLog(2, "降落结束，但无人机处于异常状态，请联系维护人员");
    }
    this->sendLog(1, "一键返航成功");
}

void VehicleControl::flightControlGoHomeAndConfirmLandingNative() {
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    //控制执行前检查
    if(droneInfo.flightStatus != 2){
        qCritical() << "无人机自动返航失败:已在地面";
        this->sendLog(1, "无人机已在地面,未执行返航");
        return;
    }
    //控制开始
//    /*! Step 1: Obtain joystick control authority */
//    qDebug() << "--> Step 1: Obtain joystick control authority.";
//    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        qCritical() << QString().sprintf("返航失败:无法获得控制权,错误码 0x%08X",returnCode) ;
//        this->sendLog(3, QString().sprintf("无法获得控制权,错误码 0x%08X", returnCode).toStdString());
//        return;
//    }
//    osalHandler->TaskSleepMs(1000);
    /*! Step 2: Start go home */
    qDebug() << "--> Step 2: Start go home action";
    this->sendLog(1, "无人机自动返航中");
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << QString().sprintf("返航失败:返航接口调用失败,错误码:%08lX", returnCode);
        this->sendLog(3, QString().sprintf("返航接口调用失败,错误码:%08lX", returnCode).toStdString());
        return;
    }
    int checkCount = 0;
    while(!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)){
        if(checkCount > 10){
            qCritical() << "返航失败:返航状态确认失败";
            this->sendLog(3, "返航状态确认失败");
            return;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    qDebug() << "返航状态确认成功,等待降落";
    this->sendLog(1, "返航状态确认成功,等待降落");
    while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
        osalHandler->TaskSleepMs(1000);// waiting for this action finished
    }
    /*! Step 3: Start landing */
    qDebug() << "--> Step 3: Start landing action";
    this->sendLog(1, "开始自动降落");
    checkCount = 0;
    while(!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)){
        if(checkCount > 10){
            qCritical() << "自动降落执行失败,开始强制降落";
            this->sendLog(3, "自动降落执行失败,开始强制降落");
            break;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
           DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        T_DjiFcSubscriptionAvoidData avoidData = DjiTest_FlightControlGetValueOfAvoidData();
        osalHandler->TaskSleepMs(1000);
        //等待降落至0.65至0.75距离时，且可正常降落的情况下进行降落确认
        if (((dji_f64_t) 0.65 < avoidData.down && avoidData.down < (dji_f64_t) 0.75) &&
            avoidData.downHealth == 1) {
            break;
        }
    }
    /*! Step 4: Confirm Landing */
    qDebug() << "--> Step 4: Start confirm Landing and avoid ground action";
    this->sendLog(1, "确认降落");
    returnCode = DjiFlightController_StartConfirmLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << QString().sprintf("确认降落执行失败,错误码:0x%08lX", returnCode);
        this->sendLog(3, QString().sprintf("确认降落执行失败,错误码:0x%08lX", returnCode).toStdString());
        return;
    }
    osalHandler->TaskSleepMs(1000);
    checkCount = 0;
    while(!(DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) ||
            DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING))){
        if(checkCount > 10){
            qCritical() << QString().sprintf("确认降落状态检查失败, 当前飞行模式:%d", this->droneInfo.displayMode);
            this->sendLog(3, QString().sprintf("确认降落状态检查失败, 当前飞行模式:%d", this->droneInfo.displayMode).toStdString());
            return;
        }
        osalHandler->TaskSleepMs(500);
        checkCount++;
    }
    qDebug() << "确认降落执行成功";
    while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
        osalHandler->TaskSleepMs(1000);
    }
    /*! Step 5: Landing finished check*/
    qDebug() << "--> Step 5: Landing finished check";
    if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        qDebug() << "降落结束";
        this->sendLog(1, "降落结束");
    } else {
        qCritical() << "降落结束，但无人机处于异常状态，请联系维护人员.";
        this->sendLog(2, "降落结束，但无人机处于异常状态，请联系维护人员");
    }
    this->sendLog(1, "返航成功");
}

/*启用飞行避障*/
void VehicleControl::flightControlAvoidParam(){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downloadsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    E_DjiFlightControllerRCLostAction rcLostAction;

    qDebug() << "启用自动避障功能";

    /*! Turn on horizontal vision avoid enable */
    qDebug("--> Step 1: Turn on horizontal visual obstacle avoidance");
    returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
       DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(4, "启用水平视觉避障失败");
       qWarning("Turn on horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    qDebug("--> Step 2: Get horizontal horizontal visual obstacle status\r\n");
    returnCode = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
       &horizontalVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(4, "水平视觉避障状态获取失败");
       qWarning("Get horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current horizontal visual obstacle avoidance status is %d\r\n",
                 horizontalVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on horizontal radar avoid enable */
    qDebug("--> Step 3: Turn on horizontal radar obstacle avoidance");
    returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
       DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(4, "启用水平雷达避障失败");
       qWarning("Turn on horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    qDebug("--> Step 4: Get horizontal radar obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
       &horizontalRadarObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(4, "水平雷达避障状态获取失败");
       qWarning("Get horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current horizontal radar obstacle avoidance status is %d\r\n",
                 horizontalRadarObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards vision avoid enable */
    qDebug("--> Step 5: Turn on upwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
       DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "启用向上视觉避障失败");
        qWarning("Turn on upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    qDebug("--> Step 6: Get upwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
       &upwardsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       this->sendDeviceAbnormal(4, "向上视觉避障状态获取失败");
       qWarning("Get upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current upwards visual obstacle avoidance status is %d\r\n", upwardsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards radar avoid enable */
    qDebug("--> Step 7: Turn on upwards radar obstacle avoidance.");
    returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
       DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "启用向上雷达避障失败");
       qWarning("Turn on upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    s_osalHandler->TaskSleepMs(1000);

    qDebug("--> Step 8: Get upwards radar obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(&upwardsRadarObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "向上雷达避障状态获取失败");
       qWarning("Get upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current upwards radar obstacle avoidance status is %d\r\n", upwardsRadarObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on downwards vision avoid enable */
    qDebug("--> Step 9: Turn on downwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
       DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "启用向下视觉避障失败");
       qWarning("Turn on downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    s_osalHandler->TaskSleepMs(1000);

    qDebug("--> Step 10: Get downwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
       &downloadsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        this->sendDeviceAbnormal(4, "向下视觉避障状态获取失败");
       qWarning("Get downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current downwards visual obstacle avoidance status is %d\r\n",
                 downloadsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    qDebug() << "自动避障功能设置结束";
}

/*启用RTK数据*/
void VehicleControl::flightControlRtkParam(){
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionRtkPositionInfo RtkPositionInfo = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiReturnCode returnCode;
    E_DjiFlightControllerRtkPositionEnableStatus flightControllerRtkPositionEnableStatus;
    //get rtk enable status
    qDebug() << "Get rtk enable status";
    returnCode = DjiFlightController_GetRtkPositionEnableStatus(&flightControllerRtkPositionEnableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       qWarning() << QString().sprintf("Get rtk enable failed, error code: 0x%08X", returnCode);
    }
    qDebug() << QString().sprintf("Current rtk enable status is %d\r\n", flightControllerRtkPositionEnableStatus);
    if(flightControllerRtkPositionEnableStatus == DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION){
        //Set rtk enable
        this->sendLog(1, "RTK定位功能启用中");
        qDebug() << "RTK定位功能启用中";
        returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
           USER_LOG_ERROR("Set rtk enable failed, error code: 0x%08lX", returnCode);
           this->sendLog(3, QString().sprintf("RTK定位接口异常,错误码:0x%08lX", returnCode).toStdString());
           this->sendReturn(3, 1, 0, "航线任务执行失败");
           this->wayPointTaskExecuteStatus == 0;
           return;
        }
        s_osalHandler->TaskSleepMs(5000);
    }
    //校验RTK状态
    qDebug() << "RTK定位功能状态校验中";
    this->sendLog(1, "RTK定位功能状态校验中");
    int count = 0;
    while(true){
        //Get rtk connect status
        returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &RtkPositionInfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qDebug() << "get value of topic RTK Position info error.";
        } else {
            qDebug() << QString().sprintf("RtkPositionInfo status : %d.", RtkPositionInfo);
            //this->sendLog(2, QString().sprintf("RtkPositionInfo status : %d.", RtkPositionInfo).toStdString());
            if(RtkPositionInfo == 50){
                qDebug() << "RTK数据连接成功";
                this->sendLog(1, "RTK数据连接成功");
                break;
            }
        }
        count++;
        if(count >= 10){
            qDebug() << "RTK定位功能不可用";
            this->sendLog(2, "RTK定位功能不可用");
            returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
               qDebug() << QString().sprintf("RTK定位接口异常,错误码:0x%08lX", returnCode);
               this->sendLog(3, QString().sprintf("RTK定位接口异常,错误码:0x%08lX", returnCode).toStdString());
            }
            break;
        }
        s_osalHandler->TaskSleepMs(1000);
    }
}

void VehicleControl::flightControlMoveByPositionOffset(DronePositionOffset droneOffset){
    int timeoutInMilSec = 20000;
    int controlFreqInHz = 50;  // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;
    T_DjiFcSubscriptionPositionFused originGPSPosition;
    dji_f32_t originHeightBaseHomePoint;
    T_DjiFcSubscriptionPositionFused currentGPSPosition;
    T_DjiFcSubscriptionQuaternion currentQuaternion;
    dji_f32_t currentHeight;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    //data process
    double s_degToRad = 0.01745329252;
    float posThresholdInM = 0.8;
    float yawThresholdInDeg = 1;
    T_DjiTestFlightControlVector3f offsetDesired;
    offsetDesired.x = droneOffset.x;
    offsetDesired.y = droneOffset.y;
    offsetDesired.z = droneOffset.z;
    float yawDesiredInDeg = droneOffset.yawDesiredInDeg;

    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    //控制执行前检查
    if(this->droneInfo.flightStatus != 2){
        USER_LOG_ERROR("trhe drone is on the ground, exectute failed.");
        goto out;
    }
    //控制开始
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    USER_LOG_INFO("--> Step 2: begin drone position control.");

    //! get origin position and relative height(from home point)of aircraft.
    originGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
    originHeightBaseHomePoint = DjiTest_FlightControlGetValueOfRelativeHeight();
    if (originHeightBaseHomePoint == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        goto out;
    }

    DjiFlightController_SetJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec) {
        currentGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
        currentQuaternion = DjiTest_FlightControlGetValueOfQuaternion();
        currentHeight = DjiTest_FlightControlGetValueOfRelativeHeight();
        if (originHeightBaseHomePoint == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            goto out;
        }

        float yawInRad = DjiTest_FlightControlQuaternionToEulerAngle(currentQuaternion).z;
        //! get the vector between aircraft and origin point.

        T_DjiTestFlightControlVector3f localOffset = DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(
            currentGPSPosition,
            originGPSPosition,
            currentHeight,
            originHeightBaseHomePoint);
        //! get the vector between aircraft and target point.
        T_DjiTestFlightControlVector3f offsetRemaining = DjiTest_FlightControlVector3FSub(offsetDesired, localOffset);

        T_DjiTestFlightControlVector3f positionCommand = offsetRemaining;
        DjiTest_FlightControlHorizCommandLimit(speedFactor, &positionCommand.x, &positionCommand.y);

        T_DjiFlightControllerJoystickCommand joystickCommand = {positionCommand.x, positionCommand.y,
                                                                offsetDesired.z + originHeightBaseHomePoint,
                                                                yawDesiredInDeg};
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        if (DjiTest_FlightControlVectorNorm(offsetRemaining) < posThresholdInM &&
            fabs(yawInRad / s_degToRad - yawDesiredInDeg) < yawThresholdInDeg) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        } else {
            if (withinBoundsCounter != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit) {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
            break;
        }
        osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt) {
        osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
        goto out;
    }

    //控制结束
    out:
        USER_LOG_INFO("MoveByPositionOffset control end");
}

void VehicleControl::flightControlVelocityAndYawRateCtrl(DroneVelocityTimeOffset droneVelocityTimeOffset){
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    osalHandler->GetTimeMs(&originTime);
    osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    //data process
    T_DjiTestFlightControlVector3f offsetDesired;
    offsetDesired.x = droneVelocityTimeOffset.x;
    offsetDesired.y = droneVelocityTimeOffset.y;
    offsetDesired.z = droneVelocityTimeOffset.z;
    float yawRate = droneVelocityTimeOffset.yawRate;
    uint32_t timeMs = droneVelocityTimeOffset.timeMs;
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};
    //控制执行前检查
    if(this->droneInfo.flightStatus != 2){
        USER_LOG_ERROR("trhe drone is on the ground, exectute failed.");
        goto out;
    }
    //控制开始
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);
    USER_LOG_INFO("--> Step 2: begin drone Velocity control.");

    DjiFlightController_SetJoystickMode(joystickMode);

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(2);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }

    //控制结束
    out:
        USER_LOG_INFO("VelocityAndYawRate control end");
}

void VehicleControl::flightPositionTest(){
    DronePositionOffset offset;
    //控制执行前检查
    if(this->droneInfo.flightStatus != 2){
        USER_LOG_ERROR("trhe drone is on the ground, exectute failed.");
        return;
    }

    offset.x = 5;
    offset.y = 0;
    offset.z = 0;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move forward 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = -5;
    offset.y = 0;
    offset.z = 0;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move back 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = -5;
    offset.z = 0;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move left 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = 5;
    offset.z = 0;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move right 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = 0;
    offset.z = 5;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move up 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = 0;
    offset.z = -5;
    offset.yawDesiredInDeg = 0;
    USER_LOG_INFO("move down 5m.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = 0;
    offset.z = 0;
    offset.yawDesiredInDeg = 60;
    USER_LOG_INFO("Clockwise rotation 60.");
    this->flightControlMoveByPositionOffset(offset);
    sleep(2);
    if(DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL){
        USER_LOG_ERROR("flightPositionTest error, end.");
        return;
    }

    offset.x = 0;
    offset.y = 0;
    offset.z = 0;
    offset.yawDesiredInDeg = -60;
    USER_LOG_INFO("Counterclockwise rotation 60.");
    this->flightControlMoveByPositionOffset(offset);

}

void VehicleControl::flightVelocityTest(){
    //控制执行前检查
    if(this->droneInfo.flightStatus != 2){
        USER_LOG_ERROR("trhe drone is on the ground, exectute failed.");
        return;
    }

    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    DroneVelocityTimeOffset offset;
    offset.x = 5;
    offset.y = 0;
    offset.z = 0;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move forward 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = -5;
    offset.y = 0;
    offset.z = 0;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move back 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = -5;
    offset.z = 0;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move left 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = 5;
    offset.z = 0;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move right 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = 0;
    offset.z = 5;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move up 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = 0;
    offset.z = -5;
    offset.yawRate = 0;
    offset.timeMs = 5000;
    USER_LOG_INFO("move down 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = 0;
    offset.z = 0;
    offset.yawRate = 5;
    offset.timeMs = 5000;
    USER_LOG_INFO("Clockwise rotation 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
    sleep(5);

    offset.x = 0;
    offset.y = 0;
    offset.z = 0;
    offset.yawRate = -5;
    offset.timeMs = 5000;
    USER_LOG_INFO("Counterclockwise rotation 5s.");
    this->flightControlVelocityAndYawRateCtrl(offset);
}

T_DjiReturnCode VehicleControl::flightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    qDebug() << "1111111111";
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                USER_LOG_INFO("[Event]Msdk request to obtain joystick ctrl authority\r\n");
                qWarning() << "[Event]Msdk request to obtain joystick ctrl authority\r\n";
            } else {
                USER_LOG_INFO("[Event]Msdk request to release joystick ctrl authority\r\n");
                qWarning() << "[Event]Msdk request to release joystick ctrl authority\r\n";
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                USER_LOG_INFO("[Event]Internal request to obtain joystick ctrl authority\r\n");
                qWarning() <<"[Event]Internal request to obtain joystick ctrl authority\r\n";
            } else {
                USER_LOG_INFO("[Event]Internal request to release joystick ctrl authority\r\n");
                qWarning() << "[Event]Internal request to release joystick ctrl authority\r\n";
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                USER_LOG_INFO("[Event]Osdk request to obtain joystick ctrl authority\r\n");
                qWarning() << "[Event]Osdk request to obtain joystick ctrl authority\r\n";
            } else {
                USER_LOG_INFO("[Event]Osdk request to release joystick ctrl authority\r\n");
                qWarning() << "[Event]Osdk request to release joystick ctrl authority\r\n";
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to osdk lost\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to osdk lost\r\n";
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            qWarning() << "[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n";
            break;
        default:
            USER_LOG_INFO("[Event]Unknown joystick ctrl authority event\r\n");
            qWarning() << "[Event]Unknown joystick ctrl authority event\r\n";
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

///航点任务控制
void VehicleControl::slotWayPointMissionTest(){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    this->flightControlThread = std::thread([this]{
        //航线设置
        WayPointMissionInfo wayPointMission;
        wayPointMission.finishAction = 1;
        wayPointMission.idleVelocity = 15;
        wayPointMission.repeatTimes = 1;
        wayPointMission.goHomeAltitude = 120;
        //航点设置
        vector<WayPointInfo> wayPoints;
        WayPointInfo wayPoint;
        pthread_mutex_lock(&vehicleInfoMutex);
        wayPoint.index = 0;
        wayPoint.relativeHeight = 20;
        wayPoint.longitude = this->droneInfo.longitude;
        wayPoint.latitude = this->droneInfo.latitude;
        wayPoints.push_back(wayPoint);

        //模拟器测试
        wayPoint.index = 1;
        wayPoint.relativeHeight = 20;
        wayPoint.longitude = this->droneInfo.longitude + 0.000052;
        wayPoint.latitude = this->droneInfo.latitude + 0.000052;
        wayPoints.push_back(wayPoint);

        wayPoint.index = 2;
        wayPoint.relativeHeight = 20;
        wayPoint.longitude = this->droneInfo.longitude + 0.000058;
        wayPoint.latitude = this->droneInfo.latitude + 0.000058;
        wayPoints.push_back(wayPoint);

        wayPoint.index = 3;
        wayPoint.relativeHeight = 20;
        wayPoint.longitude = this->droneInfo.longitude + 0.000087;
        wayPoint.latitude = this->droneInfo.latitude + 0.000087;
        wayPoints.push_back(wayPoint);

        //海大东操测试
//         wayPoint.index = 0;
//        wayPoint.relativeHeight = 150;
//        wayPoint.longitude = 2.103023;
//        wayPoint.latitude = 0.631181;
//        wayPoints.push_back(wayPoint);

//        wayPoint.index = 1;
//        wayPoint.relativeHeight = 150;
//        wayPoint.longitude = 2.10295575;
//        wayPoint.latitude = 0.63114758;
//        wayPoints.push_back(wayPoint);

//        wayPoint.index = 2;
//        wayPoint.relativeHeight = 200;
//        wayPoint.longitude = 2.1029645;
//        wayPoint.latitude = 0.63108701;
//        wayPoints.push_back(wayPoint);

//        wayPoint.index = 3;
//        wayPoint.relativeHeight = 200;
//        wayPoint.longitude = 2.1030279;
//        wayPoint.latitude = 0.63106636;
//        wayPoints.push_back(wayPoint);
        pthread_mutex_unlock(&vehicleInfoMutex);
        wayPointMission.wayPoint = wayPoints;

        //动作设置
        vector<WayPointActionInfo> wayPointActions;
        WayPointActionInfo wayPointAction;

        wayPointAction.wayPointIndex = 0;
        wayPointAction.actionActuatorType = 5;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 0;
        wayPointAction.actionActuatorType = 1;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 1;
        wayPointAction.actionActuatorType = 2;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 2;
        wayPointAction.actionActuatorType = 6;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 2;
        wayPointAction.actionActuatorType = 3;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 3;
        wayPointAction.actionActuatorType = 4;
        wayPointAction.hoverTime = 10;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 3;
        wayPointAction.actionActuatorType = 5;
        wayPointActions.push_back(wayPointAction);

        wayPointAction.wayPointIndex = 3;
        wayPointAction.actionActuatorType = 1;
        wayPointActions.push_back(wayPointAction);

        wayPointMission.wayPointAction = wayPointActions;
        flightRunWaypoints(wayPointMission);
        this->flightControlTid = -1;
    });
    this->flightControlTid = this->flightControlThread.native_handle();
    this->flightControlThread.detach();
}

void VehicleControl::slotRunWayPointMission(WayPointMissionInfo wayPointMission){
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    std::thread wayPointThread([this, wayPointMission]{
        flightRunWaypoints(wayPointMission);
    });
    wayPointThread.detach();
}

void VehicleControl::slotUploadWayPointMission(WayPointMissionInfo wayPointMission){
    if(this->wayPointTaskUploadStatus == 1){
        qCritical() << "航线任务上传失败:正在上传中...";
        this->sendLog(3, "航线任务上传失败:正在上传中...");
        this->sendReturn(3, 0, 0, "正在上传中...");
    }
    std::thread wayPointThread([this, wayPointMission]{
        //无人机状态校验
        if(DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING || DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING || DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME){
            USER_LOG_ERROR("drone is on the landing or go home status,way point execute failed.");
            qCritical() << "drone is on the landing or go home status,way point execute failed.";
            this->sendLog(3, "航线任务上传失败:无人机处于降落或者返航状态");
            this->sendReturn(3, 0, 0, "无人机处于降落或者返航状态");
            return;
        }
        //开始上传航线
        this->wayPointTaskUploadStatus = 1;
        T_DjiReturnCode returnCode;
        T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
        T_DjiWayPointV2MissionSettings missionInitSettings = {0};
        T_DjiWaypointV2GlobalCruiseSpeed getGlobalCruiseSpeed;
        //Init waypoint settings
        this->sendLog(1, "航线信息初始化中");
        qDebug() << "航线信息初始化中";
        qDebug() << "--> Step 1: Init waypoint settings";
        USER_LOG_INFO("--> Step 1: Init waypoint settings");
        missionInitSettings.missionID = 12345;            //一般PSDK端设置同一个值就可以，上传时直接覆盖前一条航线
        missionInitSettings.repeatTimes = wayPointMission.repeatTimes;                                                  //配置项
        missionInitSettings.finishedAction = (E_DJIWaypointV2MissionFinishedAction)(wayPointMission.finishAction);      //配置项
        missionInitSettings.maxFlightSpeed = 15;
        missionInitSettings.autoFlightSpeed = 8;
        missionInitSettings.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
        missionInitSettings.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
        //waypoints init
        qDebug() << "--> Step 2: Init waypoints";
        USER_LOG_INFO("--> Step 2: Init waypoints");
        //航点index排序并校验
        vector<WayPointInfo> currentWayPointList = wayPointMission.wayPoint;
        sort(currentWayPointList.begin(), currentWayPointList.end(), Tools::wayPointCompare);
        for (unsigned int i = 0; i < currentWayPointList.size(); i++)
        {
            if(currentWayPointList[i].index != i){
                qCritical() << "航线任务上传失败:航点信息错误";
                this->sendLog(3, "航线任务上传失败:航点信息错误");
                this->sendReturn(3, 0, 0, "航点信息错误");
                return;
            }
        }
        T_DjiWaypointV2 waypointV2;
        T_DjiWaypointV2 *waypointV2List = (T_DjiWaypointV2 *) osalHandler->Malloc(currentWayPointList.size() * sizeof(T_DjiWaypointV2));
        for (unsigned int i = 0; i < currentWayPointList.size(); i++)
        {
            DjiTest_WaypointV2SetDefaultSetting(&waypointV2);
            waypointV2.latitude = currentWayPointList[i].latitude;                   //配置项
            waypointV2.longitude = currentWayPointList[i].longitude;                 //配置项
            waypointV2.relativeHeight = currentWayPointList[i].relativeHeight;       //配置项
            //航点配置可更加详细**********************************************************************************
            qDebug() << QString().sprintf("waypointV2 longitude: %f, latitude: %f", waypointV2.longitude, waypointV2.latitude);
            waypointV2List[i] = waypointV2;
        }
        missionInitSettings.mission = waypointV2List;
        missionInitSettings.missTotalLen = currentWayPointList.size();
        //actions init(最后一个航点只能执行第一个动作，由于飞机固件BUG导致)
        qDebug() << "--> Step 3: Init reach wayPoint actions";
        USER_LOG_INFO("--> Step 3: Init reach wayPoint actions");
        //先遍历查找需要悬停的动作数
        int hoverPoint = 0;
        for (unsigned int i = 0; i < wayPointMission.wayPointAction.size(); i++)
        {
            if(wayPointMission.wayPointAction[i].actionActuatorType == 4){
                hoverPoint++;
            }
        }
        T_DJIWaypointV2Action *actions = (T_DJIWaypointV2Action *) osalHandler->Malloc((wayPointMission.wayPointAction.size() + hoverPoint) * sizeof(T_DJIWaypointV2Action));
        T_DJIWaypointV2ActionList actionList = {NULL, 0};
        int actionIndex = 0;
        int lastActionWayPointIndex = -1;
        for (unsigned int i = 0; i < wayPointMission.wayPointAction.size(); i++)
        {
//            qDebug() << "actionIndex:" << actionIndex << " lastActionWayPointIndex:" << lastActionWayPointIndex << " wayPointIndex:" << wayPointMission.wayPointAction[i].wayPointIndex
//                     << " actionActuatorType:" << wayPointMission.wayPointAction[i].actionActuatorType << " hoverTime:" << wayPointMission.wayPointAction[i].hoverTime;
            T_DJIWaypointV2Trigger trigger = {0};
            T_DJIWaypointV2Actuator actuator = {0};
            T_DJIWaypointV2Action action = {0};
            action.actionId = actionIndex;                                                                              //配置项
            //trigger
            if(wayPointMission.wayPointAction[i].wayPointIndex == lastActionWayPointIndex){
                //同一航点的后续动作设置关联触发
                trigger.actionTriggerType = DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
                trigger.associateTriggerParam.actionAssociatedType = DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISHED;
                trigger.associateTriggerParam.waitingTime = 1;
                trigger.associateTriggerParam.waitTimeUint = 1;
                trigger.associateTriggerParam.actionIdAssociated = actionIndex - 1;
            }
            else{
                //到点触发配置项
                trigger.actionTriggerType = DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT;
                trigger.sampleReachPointTriggerParam.terminateNum = 0;
                trigger.sampleReachPointTriggerParam.waypointIndex = wayPointMission.wayPointAction[i].wayPointIndex;
            }
            //actuator
            switch(wayPointMission.wayPointAction[i].actionActuatorType){
                case 1:
                    //拍照
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                    actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO;
                    break;
                case 2:
                    //开始录像
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                    actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_START_RECORD_VIDEO;
                    break;
                case 3:
                    //停止录像
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                    actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_STOP_RECORD_VIDEO;
                    break;
                case 4:
                    //悬停
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
                    actuator.aircraftControlActuatorParam.operationType = DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
                    actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 0;
                    actuator.aircraftControlActuatorParam.flyControlParam.reserved = 7;
                    break;
                case 5:
                    //云台向下
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
                    actuator.gimbalActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;
                    actuator.gimbalActuatorParam.rotation.absYawModeRef = 0;
                    actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 1;
                    actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 1;
                    actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 0;
                    actuator.gimbalActuatorParam.rotation.reserved = 0;
                    actuator.gimbalActuatorParam.rotation.ctrl_mode = 0;
                    actuator.gimbalActuatorParam.rotation.durationTime = 1.0;
                    actuator.gimbalActuatorParam.rotation.x = 0;
                    actuator.gimbalActuatorParam.rotation.y = -900;
                    actuator.gimbalActuatorParam.rotation.z = 0;         //负数向左，正数向右
                    break;
                case 6:
                    //云台回中
                    actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
                    actuator.gimbalActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;
                    actuator.gimbalActuatorParam.rotation.absYawModeRef = 0;
                    actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 0;
                    actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 1;
                    actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 0;
                    actuator.gimbalActuatorParam.rotation.reserved = 0;
                    actuator.gimbalActuatorParam.rotation.ctrl_mode = 0;
                    actuator.gimbalActuatorParam.rotation.durationTime = 1.0;
                    actuator.gimbalActuatorParam.rotation.x = 0;
                    actuator.gimbalActuatorParam.rotation.y = 0;
                    actuator.gimbalActuatorParam.rotation.z = 0;         //负数向左，正数向右
                    break;
            }
            memcpy(&action.actuator, &actuator, sizeof(actuator));
            memcpy(&action.trigger, &trigger, sizeof(trigger));
            actions[actionIndex] = action;
            actionIndex++;
            //悬停增加动作
            if(wayPointMission.wayPointAction[i].actionActuatorType == 4){
                T_DJIWaypointV2Trigger trigger = {0};
                T_DJIWaypointV2Actuator actuator = {0};
                T_DJIWaypointV2Action action = {0};
                action.actionId = actionIndex;
                trigger.actionTriggerType = DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
                trigger.associateTriggerParam.actionAssociatedType = DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISHED;
                trigger.associateTriggerParam.waitingTime = wayPointMission.wayPointAction[i].hoverTime;
                trigger.associateTriggerParam.waitTimeUint = 1;
                trigger.associateTriggerParam.actionIdAssociated = actionIndex - 1;
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
                actuator.aircraftControlActuatorParam.operationType = DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
                actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 1;
                actuator.aircraftControlActuatorParam.flyControlParam.reserved = 7;
                memcpy(&action.actuator, &actuator, sizeof(actuator));
                memcpy(&action.trigger, &trigger, sizeof(trigger));
                actions[actionIndex] = action;
                actionIndex++;
            }
            lastActionWayPointIndex = wayPointMission.wayPointAction[i].wayPointIndex;
        }
        actionList.actions = actions;
        actionList.actionNum = wayPointMission.wayPointAction.size() + hoverPoint;
        missionInitSettings.actionList = actionList;
        //upload waypoints
        this->sendLog(1, "航线任务上传中");
        qDebug() << "航线任务上传中";
        qDebug() << "--> Step 4: upload waypoints";
        USER_LOG_INFO("--> Step 4: upload waypoints");
        returnCode = DjiWaypointV2_UploadMission(&missionInitSettings);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "upload waypoint V2 mission setting failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("upload waypoint V2 mission setting failed, ErrorCode:0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("航线任务上传失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 0, 0, (QString().sprintf("航线任务上传失败,错误码:0x%08lX", returnCode)).toStdString());
            goto out;
        }
        osalHandler->TaskSleepMs(1000);
        qDebug() << "--> Step 5: Set global cruise speed";
        USER_LOG_INFO("--> Step 5: Set global cruise speed\r");
        returnCode = DjiWaypointV2_SetGlobalCruiseSpeed((T_DjiWaypointV2GlobalCruiseSpeed)(wayPointMission.idleVelocity));
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Set global cruise speed failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Set global cruise speed failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("全局巡航速度设置失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 0, 0, (QString().sprintf("全局巡航速度设置失败,错误码:0x%08lX", returnCode)).toStdString());
            goto out;
        }
        osalHandler->TaskSleepMs(1000);
        qDebug() << "--> Step 6: Get global cruise speed";
        USER_LOG_INFO("--> Step 6: Get global cruise speed\r");
        getGlobalCruiseSpeed = 0;
        returnCode = DjiWaypointV2_GetGlobalCruiseSpeed(&getGlobalCruiseSpeed);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Get global cruise speed failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get global cruise speed failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("全局巡航速度获取失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 0, 0, (QString().sprintf("全局巡航速度获取失败,错误码:0x%08lX", returnCode)).toStdString());
            goto out;
        }
        USER_LOG_INFO("Current global cruise speed is %f m/s", getGlobalCruiseSpeed);
        osalHandler->TaskSleepMs(1000);
        //设置返航高度
        qDebug() << "--> Step 7: set go home altitude.";
        USER_LOG_INFO("--> Step 7: set go home altitude.");
        returnCode = DjiFlightController_SetGoHomeAltitude(wayPointMission.goHomeAltitude);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Set go home altitude failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Set go home altitude failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("返航高度设置失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 0, 0, (QString().sprintf("返航高度设置失败,错误码:0x%08lX", returnCode)).toStdString());
            goto out;
        }
        E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
        returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Get go home altitude failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("返航高度获取失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 0, 0, (QString().sprintf("返航高度获取失败,错误码:0x%08lX", returnCode)).toStdString());
            goto out;
        }
        USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
        //设置航线任务信息
        this->uploadWayPointMission = wayPointMission;
        //任务上传成功
        this->sendReturn(3, 0, 1, "");
        this->sendLog(1, "航线任务上传成功");
        out:
            USER_LOG_INFO("flightRunWaypoints end.");
            this->wayPointTaskUploadStatus = 0;
            osalHandler->Free(waypointV2List);
            osalHandler->Free(actions);
    });
    wayPointThread.detach();
}

/*
 * 0x10036报错是上传航点任务数小于初始化总航点任务数，0x10037是上传航点任务数大于初始化总航点任务数,
 * 上述异常均由航线上传异常造成，该异常很有可能由于未开遥控器解除限高限远导致
*/
void VehicleControl::slotStartWayPointMission(){
    if(this->wayPointTaskExecuteStatus == 1){
        qCritical() << "航线任务开始失败:任务进行中";
        this->sendLog(3, "航线任务开始失败:任务进行中");
        this->sendReturn(3, 1, 0, "航线任务开始失败");
    }
    if(this->flightControlTid > 0){
        pthread_cancel(this->flightControlTid);
    }
    std::thread wayPointThread([this]{
        this->wayPointTaskExecuteStatus == 1;
        T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
        T_DjiFcSubscriptionRTKConnectStatus RTKConnectStatus = {0};
        T_DjiDataTimestamp timestamp = {0};
        T_DjiReturnCode returnCode;
        //RTK related
        this->flightControlRtkParam();

        //返航点设置
        qDebug() << "返航点设置中";
        this->sendLog(1, "返航点设置中");
        returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "返航点设置接口异常,错误码:0x" << QString().sprintf("%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
            this->wayPointTaskExecuteStatus == 0;
            return;
        }

        qDebug() << "航线任务开始";
        returnCode = DjiWaypointV2_Start();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            QString code = QString().sprintf("%08lX", returnCode);
            if(code == "2300010036" || code == "2300010037"){
                //提示可能遥控器未开，未认证
                qCritical() << "任务开始接口异常,错误码:0x" << code << ",航线校验异常,可能未开启遥控器完成飞行认证导致";
                this->sendLog(3, (QString().sprintf("任务开始接口异常,错误码:0x%08X,错误原因:航线校验异常,可能未开启遥控器完成飞行认证导致", returnCode)).toStdString());
                this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                this->wayPointTaskExecuteStatus == 0;
                return;
            }
            else if(code == "2300020005"){
                //RTK数据异常,将关闭RTK后重新开始任务
                this->sendLog(2, "RTK数据异常，将关闭此功能，并重新开始任务");
                returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                   qWarning() << QString().sprintf("Set rtk disable failed, error code: 0x%08lX", returnCode);
                   this->sendLog(3, QString().sprintf("RTK定位接口异常,错误码:0x%08lX", returnCode).toStdString());
                   this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                   this->wayPointTaskExecuteStatus == 0;
                   return;
                }
                qDebug() << "返航点设置中";
                this->sendLog(1, "返航点设置中");
                returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    qCritical() << "返航点设置接口异常,错误码:0x" << QString().sprintf("%08lX", returnCode);
                    this->sendLog(3, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                    this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                    this->wayPointTaskExecuteStatus == 0;
                    return;
                }
                qDebug() << "航线任务开始";
                returnCode = DjiWaypointV2_Start();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    QString code = QString().sprintf("%08lX", returnCode);
                    if(code == "2300010036" || code == "2300010037"){
                        //提示可能遥控器未开，未认证
                        qCritical() << "任务开始接口异常,错误码:0x" << code << ",航线校验异常,可能未开启遥控器完成飞行认证导致";
                        this->sendLog(3, (QString().sprintf("任务开始接口异常,错误码:0x%08X,错误原因:航线校验异常,可能未开启遥控器完成飞行认证导致", returnCode)).toStdString());
                        this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                        this->wayPointTaskExecuteStatus == 0;
                        return;
                    }
                    else{
                        qCritical() << "任务开始接口异常,错误码:0x" << code;
                        this->sendLog(3, (QString().sprintf("任务开始接口异常,错误码:0x%08lX", returnCode)).toStdString());
                        this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                        this->wayPointTaskExecuteStatus == 0;
                        return;
                    }
                }
            }
            else{
                qCritical() << "任务开始接口异常,错误码:0x" << code;
                this->sendLog(3, (QString().sprintf("任务开始接口异常,错误码:0x%08lX", returnCode)).toStdString());
                this->sendReturn(3, 1, 0, (QString().sprintf("返航点设置接口异常,错误码:0x%08lX", returnCode)).toStdString());
                this->wayPointTaskExecuteStatus == 0;
                return;
            }
        }
        this->sendReturn(3, 1, 1, "");
        this->sendLog(1, "任务开始");
        //初始化本次任务中异常暂停次数变量
        this->missionAutoPauseCount = 0;
        //初始化进入航线的标志
        this->missionEntryFlag = false;
        //初始化避障标志
        this->missionObstacleAvoidanceFlag = false;
        //设置全局航线信息及执行信息
        g_wp_mission_info = this->uploadWayPointMission;
        g_wp_mission_exe_info.currentRepeatTimes = 0;
        g_wp_mission_exe_info.currentWayPointIndex = -1;
        g_wp_mission_exe_info.wayPointTotalCount = g_wp_mission_info.wayPoint.size();
        g_wp_mission_exe_info.repeatTimes = g_wp_mission_info.repeatTimes;
        g_wp_mission_exe_info.startTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
        //清空当前上传航线记录信息
        this->uploadWayPointMission.taskID = -1;
        this->uploadWayPointMission.finishAction = -1;
        this->uploadWayPointMission.goHomeAltitude = -1;
        this->uploadWayPointMission.idleVelocity = -1;
        this->uploadWayPointMission.repeatTimes = -1;
        this->uploadWayPointMission.wayPoint.clear();
        this->uploadWayPointMission.wayPointAction.clear();
    });
    wayPointThread.detach();
}

void VehicleControl::slotStopWayPointMission(){
    std::thread wayPointThread([this]{
        qDebug() << "stop waypoint V2 mission";
        USER_LOG_INFO("stop waypoint V2.\n");
        T_DjiReturnCode returnCode = DjiWaypointV2_Stop();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "stop waypoint V2 mission failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("stop waypoint V2 mission failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("任务停止失败,错误码:0x%08lX,强制返航中", returnCode)).toStdString());
            this->sendReturn(3, 2, 0, (QString().sprintf("任务停止失败,错误码:0x%08lX,强制返航中", returnCode)).toStdString());
            this->slotFlightControlGoHomeAndConfirmLandingNative();
            return;
        }
        this->sendReturn(3, 2, 1, "");
        //go home
        this->slotFlightControlGoHomeAndConfirmLandingNative();
    });
    wayPointThread.detach();
}

void VehicleControl::slotPauseWayPointMission(){
    std::thread wayPointThread([this]{
        qDebug() << "Pause waypoint V2 mission";
        USER_LOG_INFO("Pause waypoint V2.\n");
        this->missionManuPauseFlag = true;
        T_DjiReturnCode returnCode = DjiWaypointV2_Pause();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "pause waypoint V2 mission failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("pause waypoint V2 mission failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("任务暂停失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 3, 0, (QString().sprintf("任务暂停失败,错误码:0x%08lX", returnCode)).toStdString());
            this->missionManuPauseFlag = false;
            return;
        }
        this->sendReturn(3, 3, 1, "");
        this->sendLog(1, "任务暂停");
    });
    wayPointThread.detach();
}

void VehicleControl::slotResumeWayPointMission(){
    std::thread wayPointThread([this]{
        qDebug() << "Resume waypoint V2 mission";
        USER_LOG_INFO("Resume waypoint V2.\n");
        T_DjiReturnCode returnCode = DjiWaypointV2_Resume();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            qCritical() << "Resume waypoint V2 mission failed, error code:0x" << QString().sprintf("%08lX", returnCode);
            USER_LOG_ERROR("Resume waypoint V2 mission failed, error code: 0x%08lX", returnCode);
            this->sendLog(3, (QString().sprintf("任务恢复失败,错误码:0x%08lX", returnCode)).toStdString());
            this->sendReturn(3, 4, 0, (QString().sprintf("任务恢复失败,错误码:0x%08lX", returnCode)).toStdString());
            return;
        }
        this->sendReturn(3, 4, 1, "");
        this->sendLog(1, "任务恢复");
    });
    wayPointThread.detach();
}

void VehicleControl::flightRunWaypoints(WayPointMissionInfo wayPointMission){
    if(DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING || DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING || DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME){
        USER_LOG_ERROR("drone is on the landing or go home status,way point execute failed.");
        return;
    }
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiWayPointV2MissionSettings missionInitSettings = {0};
    T_DjiWaypointV2GlobalCruiseSpeed getGlobalCruiseSpeed;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionRTKConnectStatus RTKConnectStatus = {0};
    T_DjiDataTimestamp timestamp = {0};
    //Init waypoint settings
    USER_LOG_INFO("--> Step 1: Init waypoint settings");
    missionInitSettings.missionID = 12345;            //一般PSDK端设置同一个值就可以，上传时直接覆盖前一条航线
    USER_LOG_DEBUG("Generate mission id:%d", missionInitSettings.missionID);
    missionInitSettings.repeatTimes = wayPointMission.repeatTimes;                                                  //配置项
    missionInitSettings.finishedAction = (E_DJIWaypointV2MissionFinishedAction)(wayPointMission.finishAction);      //配置项
    missionInitSettings.maxFlightSpeed = 15;
    missionInitSettings.autoFlightSpeed = 5;
    missionInitSettings.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
    missionInitSettings.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
    //waypoints init
    USER_LOG_INFO("--> Step 2: Init waypoints");
    T_DjiWaypointV2 waypointV2;
    T_DjiWaypointV2 *waypointV2List = (T_DjiWaypointV2 *) osalHandler->Malloc((wayPointMission.wayPoint.size() + 1) * sizeof(T_DjiWaypointV2));
    for (unsigned int i = 0; i < wayPointMission.wayPoint.size(); i++)
    {
        DjiTest_WaypointV2SetDefaultSetting(&waypointV2);
        waypointV2.latitude = wayPointMission.wayPoint[i].latitude;                   //配置项
        waypointV2.longitude = wayPointMission.wayPoint[i].longitude;                 //配置项
        waypointV2.relativeHeight = wayPointMission.wayPoint[i].relativeHeight;       //配置项
        USER_LOG_DEBUG("waypointV2 longitude: %f, latitude: %f", waypointV2.longitude, waypointV2.latitude);
        waypointV2List[i] = waypointV2;
    }
    waypointV2List[wayPointMission.wayPoint.size()] = waypointV2;
    missionInitSettings.mission = waypointV2List;
    missionInitSettings.missTotalLen = wayPointMission.wayPoint.size() + 1;
    //actions init
    USER_LOG_INFO("--> Step 3: Init reach wayPoint actions");
    //先遍历查找需要悬停的动作数
    int hoverPoint = 0;
    for (unsigned int i = 0; i < wayPointMission.wayPointAction.size(); i++)
    {
        if(wayPointMission.wayPointAction[i].actionActuatorType == 4){
            hoverPoint++;
        }
    }
    T_DJIWaypointV2Action *actions = (T_DJIWaypointV2Action *) osalHandler->Malloc((wayPointMission.wayPointAction.size() + hoverPoint) * sizeof(T_DJIWaypointV2Action));
    T_DJIWaypointV2ActionList actionList = {NULL, 0};
    int actionIndex = 0;
    for (unsigned int i = 0; i < wayPointMission.wayPointAction.size(); i++)
    {
        T_DJIWaypointV2Trigger trigger = {0};
        T_DJIWaypointV2Actuator actuator = {0};
        T_DJIWaypointV2Action action = {0};
        action.actionId = actionIndex;                                                                              //配置项
        //trigger
        trigger.actionTriggerType = DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT;                         //trigger配置项
        trigger.sampleReachPointTriggerParam.terminateNum = 0;                                                      //到点触发配置项
        trigger.sampleReachPointTriggerParam.waypointIndex = wayPointMission.wayPointAction[i].wayPointIndex;       //到点触发配置项
        //actuator
        switch(wayPointMission.wayPointAction[i].actionActuatorType){
            case 1:
                //拍照
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO;
                break;
            case 2:
                //开始录像
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_START_RECORD_VIDEO;
                break;
            case 3:
                //停止录像
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
                actuator.cameraActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_STOP_RECORD_VIDEO;
                break;
            case 4:
                //悬停
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
                actuator.aircraftControlActuatorParam.operationType = DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
                actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 0;
                actuator.aircraftControlActuatorParam.flyControlParam.reserved = 7;
                break;
            case 5:
                //云台向下
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
                actuator.gimbalActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;
                actuator.gimbalActuatorParam.rotation.absYawModeRef = 0;
                actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 1;
                actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 1;
                actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 0;
                actuator.gimbalActuatorParam.rotation.reserved = 0;
                actuator.gimbalActuatorParam.rotation.ctrl_mode = 0;
                actuator.gimbalActuatorParam.rotation.durationTime = 1.0;
                actuator.gimbalActuatorParam.rotation.x = 0;
                actuator.gimbalActuatorParam.rotation.y = -900;
                actuator.gimbalActuatorParam.rotation.z = 0;         //负数向左，正数向右
                break;
            case 6:
                //云台回中
                actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
                actuator.gimbalActuatorParam.operationType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;
                actuator.gimbalActuatorParam.rotation.absYawModeRef = 0;
                actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 0;
                actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 1;
                actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 0;
                actuator.gimbalActuatorParam.rotation.reserved = 0;
                actuator.gimbalActuatorParam.rotation.ctrl_mode = 0;
                actuator.gimbalActuatorParam.rotation.durationTime = 1.0;
                actuator.gimbalActuatorParam.rotation.x = 0;
                actuator.gimbalActuatorParam.rotation.y = 0;
                actuator.gimbalActuatorParam.rotation.z = 0;         //负数向左，正数向右
                break;
        }
        memcpy(&action.actuator, &actuator, sizeof(actuator));
        memcpy(&action.trigger, &trigger, sizeof(trigger));
        actions[actionIndex] = action;
        actionIndex++;
        //悬停增加动作
        if(wayPointMission.wayPointAction[i].actionActuatorType == 4){
            T_DJIWaypointV2Trigger trigger = {0};
            T_DJIWaypointV2Actuator actuator = {0};
            T_DJIWaypointV2Action action = {0};
            action.actionId = actionIndex;
            trigger.actionTriggerType = DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
            trigger.associateTriggerParam.actionAssociatedType = DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISHED;
            trigger.associateTriggerParam.waitingTime = wayPointMission.wayPointAction[i].hoverTime;
            trigger.associateTriggerParam.waitTimeUint = 1;
            trigger.associateTriggerParam.actionIdAssociated = actionIndex - 1;
            actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
            actuator.aircraftControlActuatorParam.operationType = DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
            actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 1;
            actuator.aircraftControlActuatorParam.flyControlParam.reserved = 7;
            memcpy(&action.actuator, &actuator, sizeof(actuator));
            memcpy(&action.trigger, &trigger, sizeof(trigger));
            actions[actionIndex] = action;
            actionIndex++;
        }
    }
    actionList.actions = actions;
    actionList.actionNum = wayPointMission.wayPointAction.size() + hoverPoint;
    missionInitSettings.actionList = actionList;

    //upload waypoints
    USER_LOG_INFO("--> Step 4: upload waypoints");
    returnCode = DjiWaypointV2_UploadMission(&missionInitSettings);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("upload waypoint V2 mission setting failed, ErrorCode:0x%lX", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 5: Set global cruise speed\r");
    returnCode = DjiWaypointV2_SetGlobalCruiseSpeed((T_DjiWaypointV2GlobalCruiseSpeed)(wayPointMission.idleVelocity));
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set global cruise speed failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 6: Get global cruise speed\r");
    getGlobalCruiseSpeed = 0;
    returnCode = DjiWaypointV2_GetGlobalCruiseSpeed(&getGlobalCruiseSpeed);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get global cruise speed failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current global cruise speed is %f m/s", getGlobalCruiseSpeed);
    osalHandler->TaskSleepMs(1000);

    //设置返航高度
    USER_LOG_INFO("--> Step 7: set go home altitude.");
    returnCode = DjiFlightController_SetGoHomeAltitude(wayPointMission.goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);

    //Set rtk enable
    USER_LOG_INFO("--> Step 8: Set rtk enable status");
    returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
       USER_LOG_ERROR("Set rtk enable failed, error code: 0x%08X", returnCode);
       goto out;
    }
    s_osalHandler->TaskSleepMs(3000);
    //Get rtk connect status
    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &RTKConnectStatus,
                                                      sizeof(T_DjiFcSubscriptionRTKConnectStatus),
                                                      &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic RTKConnect status error.");
        returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
           USER_LOG_ERROR("Set rtk disable failed, error code: 0x%08X", returnCode);
           goto out;
        }
    } else {
        USER_LOG_DEBUG("RTKConnect status : %d.", RTKConnectStatus.rtkConnected);
        if(!RTKConnectStatus.rtkConnected){
            returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
               USER_LOG_ERROR("Set rtk disable failed, error code: 0x%08X", returnCode);
               goto out;
            }
        }
    }

    USER_LOG_INFO("--> Step 9: Start waypoint V2 mission\r");
    returnCode = DjiWaypointV2_Start();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start waypoint V2 mission failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(20000);
    out:
        USER_LOG_INFO("flightRunWaypoints end.");
        osalHandler->Free(waypointV2List);
        osalHandler->Free(actions);
}

/*事件回调，wayPoint V3中没有此接口*/
T_DjiReturnCode VehicleControl::waypointV2EventCallback(T_DjiWaypointV2MissionEventPush eventData)
{
    if (eventData.event == 0x01) {
//        static_vehicleControl->sendLog(2, (QString().sprintf("任务暂停, 状态码:0x%x", eventData.data.interruptReason)).toStdString());
//        qWarning() << QString().sprintf("[%s]: Mission interrupted reason is 0x%x", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
//                eventData.data.interruptReason);
//        USER_LOG_INFO("[%s]: Mission interrupted reason is 0x%x",
//                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
//                      eventData.data.interruptReason);
//        static_vehicleControl->wayPointExceptionPauseProcess();                   //任务暂停处理
    } else if (eventData.event == 0x02) {
//        static_vehicleControl->sendLog(1, (QString().sprintf("任务恢复, 状态码:0x%x", eventData.data.interruptReason)).toStdString());
//        qWarning() << QString().sprintf("[%s]: Mission recover reason is 0x%x", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
//                eventData.data.recoverProcess);
//        USER_LOG_INFO("[%s]: Mission recover reason is 0x%x",
//                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
//                      eventData.data.recoverProcess);
    } else if (eventData.event == 0x03) {
        static_vehicleControl->sendLog(2, (QString().sprintf("航线任务退出, 状态码:0x%x", eventData.data.exitReason)).toStdString());
        static_vehicleControl->wayPointMissionEnd();
        qWarning() << QString().sprintf("[%s]: 航线任务退出, 状态码: 0x%x", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                eventData.data.exitReason);
        USER_LOG_INFO("[%s]: Mission exit reason is 0x%x",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.exitReason);
    } else if (eventData.event == 0x10) {
        g_wp_mission_exe_info.currentWayPointIndex = eventData.data.waypointIndex;
        static_vehicleControl->sendWayPoint(eventData.data.waypointIndex);
        static_vehicleControl->sendLog(1, "当前所在航点:" + to_string(eventData.data.waypointIndex + 1));
        qDebug() << QString().sprintf("[%s]: Current waypoint index is %d", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                eventData.data.waypointIndex);
        USER_LOG_INFO("[%s]: Current waypoint index is %d",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.waypointIndex);
    } else if (eventData.event == 0x11) {
        g_wp_mission_exe_info.currentRepeatTimes = eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes;
        static_vehicleControl->sendLog(1, "当前任务执行次数:" + to_string(eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes));
        qDebug() << QString().sprintf("[%s]: Current mission execute times is %d", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes);
        USER_LOG_INFO("[%s]: Current mission execute times is %d",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes);
    } else if (eventData.event == 0x12) {
        T_DjiFcSubscriptionAvoidData avoidData = DjiTest_FlightControlGetValueOfAvoidData();
        static_vehicleControl->missionObstacleAvoidanceFlag = true;
        static_vehicleControl->sendLog(2, QString().sprintf("无人机触发自动避障,下方:%fm,上方:%fm,前方:%fm,后方:%fm,左方:%fm,右方:%fm,", avoidData.downHealth?avoidData.down:0,
                avoidData.upHealth?avoidData.up:0,avoidData.frontHealth?avoidData.front:0,avoidData.backHealth?avoidData.back:0,avoidData.leftHealth?avoidData.left:0,
                avoidData.rightHealth?avoidData.right:0).toStdString());
        qWarning() << QString().sprintf("[%s]: avoid obstacle state: %d. downHealth: %d, down: %f; frontHealth: %d, front: %f; rightHealth: %d, right: %f; backHealth: %d, back: %f; "
                "leftHealth: %d, left: %f; upHealth: %d, up: %f", s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr, eventData.data.avoidState,
                avoidData.downHealth, avoidData.down, avoidData.frontHealth, avoidData.front, avoidData.rightHealth, avoidData.right, avoidData.backHealth, avoidData.back, avoidData.leftHealth,
                avoidData.left, avoidData.upHealth, avoidData.up);
    } else if (eventData.event == 0x30) {
        static_vehicleControl->sendLog(1, "当前执行动作id:" + to_string(eventData.data.T_DjiWaypointV2ActionExecEvent.actionId) + " 执行结果:" +
                                           to_string(eventData.data.T_DjiWaypointV2ActionExecEvent.result));
        qDebug() << QString().sprintf("[%s]: action id:%d, pre actuator state:%d, current actuator state:%d, result:%d",
                                        s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                eventData.data.T_DjiWaypointV2ActionExecEvent.actionId,
                eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState,
                eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState,
                eventData.data.T_DjiWaypointV2ActionExecEvent.result);
        USER_LOG_INFO(
            "[%s]: action id:%d, pre actuator state:%d, current actuator state:%d, result:0x%08llX",
            s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
            eventData.data.T_DjiWaypointV2ActionExecEvent.actionId,
            eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState,
            eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState,
            eventData.data.T_DjiWaypointV2ActionExecEvent.result
        );
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/*该接口调用频率在100ms左右*/
T_DjiReturnCode VehicleControl::waypointV2StateCallback(T_DjiWaypointV2MissionStatePush stateData)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    WayPointStateInfo wayPointStateInfo;
    static uint32_t curMs = 0;
    static uint32_t preMs = 0;
    osalHandler->GetTimeMs(&curMs);
    //更新航线状态
    wayPointStateInfo.state = DjiTest_WaypointV2GetMissionStateIndex(stateData.state);
    if(static_vehicleControl->missionEntryFlag == false && wayPointStateInfo.state == 3){
        //更新航线进入标志
        static_vehicleControl->missionEntryFlag = true;
    }
    if(static_vehicleControl->missionEntryFlag == true && wayPointStateInfo.state == 6){
        //更新航线进入标志
        static_vehicleControl->missionEntryFlag = false;
    }
    wayPointStateInfo.curWaypointIndex = static_vehicleControl->missionEntryFlag == true ? stateData.curWaypointIndex : -1;
    wayPointStateInfo.velocity = stateData.velocity;
    //速度
    static_vehicleControl->lastMissionState.velocity = static_vehicleControl->currentMissionState.velocity;
    static_vehicleControl->currentMissionState.velocity = wayPointStateInfo.velocity;
    //状态
    if(static_vehicleControl->currentMissionState.state != wayPointStateInfo.state){
        qWarning() << "航线状态改变!";
        if(wayPointStateInfo.state == 0x00){
            qDebug() << "航线未开始!";
            if(static_vehicleControl->currentMissionState.state >= 0x01 && static_vehicleControl->currentMissionState.state <= 0x06){
                //exist error sometimes  状态变化太快时，该方案无法检测
                qWarning() << "航线状态变化检测到航线任务退出!";
//                static_vehicleControl->sendLog(2, "航线任务退出");
//                static_vehicleControl->wayPointMissionEnd();
            }
        }
        else if(wayPointStateInfo.state == 0x01){
            qDebug() << "航线准备完成!";
            static_vehicleControl->sendLog(1, "航线准备完成");
        }
        else if(wayPointStateInfo.state == 0x02){
            qDebug() << "航线进入!";
            static_vehicleControl->sendLog(1, "航线进入");
        }
        else if(wayPointStateInfo.state == 0x03){
            qDebug() << "航线执行中!";
            static_vehicleControl->sendLog(1, "航线执行中");
        }
        else if(wayPointStateInfo.state == 0x04){
            if(static_vehicleControl->missionManuPauseFlag == false){
                qWarning() << "航线自动暂停,触发异常处理机制!";
                static_vehicleControl->sendLog(2, "航线异常暂停,自动恢复中");
                static_vehicleControl->wayPointExceptionPauseProcess();                   //任务暂停处理
            }
            else{
                static_vehicleControl->sendLog(1, "航线暂停");
                qWarning() << "航线手动暂停!";
            }
        }
        else if(wayPointStateInfo.state == 0x05){
            qWarning() << "航线恢复!";
            static_vehicleControl->sendLog(1, "航线恢复");
            static_vehicleControl->missionManuPauseFlag = false;
        }
        else if(wayPointStateInfo.state == 0x06){
            qWarning() << "航线退出!";
            static_vehicleControl->sendLog(2, "航线退出");
        }
        else if(wayPointStateInfo.state == 0xFF){
            qDebug() << "航线状态未知!";
        }
        static_vehicleControl->lastMissionState.state = static_vehicleControl->currentMissionState.state;
        static_vehicleControl->currentMissionState.state = wayPointStateInfo.state;
    }
    //航点
    if(static_vehicleControl->currentMissionState.curWaypointIndex != wayPointStateInfo.curWaypointIndex){
        qDebug() << "当前所在航点index：" << wayPointStateInfo.curWaypointIndex;
        static_vehicleControl->lastMissionState.curWaypointIndex = static_vehicleControl->currentMissionState.curWaypointIndex;
        static_vehicleControl->currentMissionState.curWaypointIndex = wayPointStateInfo.curWaypointIndex;
    }
    //每隔一秒更新航线状态信息
    if (curMs - preMs >= 1000) {
        preMs = curMs;
        USER_LOG_DEBUG("[Waypoint Index:%d]: State: %s, velocity:%.2f m/s",
                      stateData.curWaypointIndex,
                      s_waypointV2StateStr[DjiTest_WaypointV2GetMissionStateIndex(stateData.state)].stateStr,
                      (dji_f32_t) stateData.velocity / 100);
        //航线速度
        wayPointStateInfo.velocity = stateData.velocity;
        //航线位置
        if(static_vehicleControl->missionEntryFlag){
            wayPointStateInfo.curWaypointIndex = stateData.curWaypointIndex;
        }
        else{
            wayPointStateInfo.curWaypointIndex = -1;
        }
        //发送航线状态信息
        static_vehicleControl->sendWayPointState(wayPointStateInfo);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/*
 * 航线任务异常暂停处理
 *waypointV2通过事件回调反应的都是异常暂停的情况，waypointV3没有事件回调函数，需要判断是手动暂停还是异常暂停
*/
void VehicleControl::wayPointExceptionPauseProcess(){
    this->missionAutoPauseCount++;
    //异常导致的自动暂停,启动线程进行异常处理
    std::thread wayPointExceptionPauseProcessThread([this]{
        if(this->missionAutoPauseCount > 30){
            qCritical() << "任务自动恢复次数达上限，航线执行异常,强制返航中";
            this->sendLog(3, "任务自动恢复次数达上限，航线执行异常,强制返航中");
            this->slotFlightControlGoHomeAndConfirmLanding();
            return;
        }
        if(this->missionObstacleAvoidanceFlag){
            qWarning() << "无人机姿态调整中";
            this->sendLog(2, "无人机姿态调整中");
            //更改姿态,等待实际测试效果
            T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
            osalHandler->TaskSleepMs(5000);
            this->sendLog(2, "无人机姿态调整完成");
        }
        qWarning() << "航线任务自动恢复中...";
        this->sendLog(2, "航线任务自动恢复中...");
        int resumeCount = 0;
        this->wayPointResumeProcess(resumeCount);
        qDebug() << "resumeCount:" << resumeCount;
        if(resumeCount > 5){
            this->sendLog(3, "航线任务无法恢复，强制返航中");
            this->slotFlightControlGoHomeAndConfirmLanding();
        }
    });
    wayPointExceptionPauseProcessThread.detach();
}

void VehicleControl::wayPointResumeProcess(int& resumeCount){
    if(resumeCount > 5){
        //达到恢复上限次数
        return;
    }
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode = DjiWaypointV2_Resume();
    resumeCount++;
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        qCritical() << "Resume waypoint V2 mission failed, error code:0x" << QString().sprintf("%08lX", returnCode);
        this->sendLog(3, (QString().sprintf("任务恢复失败,错误码:0x%08lX", returnCode)).toStdString());
        osalHandler->TaskSleepMs(3000);     //3秒后重试
        qWarning() << "任务恢复失败,3秒后重试";
        this->wayPointResumeProcess(resumeCount);
    }
    //恢复校验
    int checkCount = 0;
    while(true){
        if(this->currentMissionState.state == 5){
            //任务恢复成功
            break;
        }
        if(checkCount > 30){
            qWarning() << "任务恢复失败,重试";
            this->wayPointResumeProcess(resumeCount);
            break;
        }
        checkCount++;
        osalHandler->TaskSleepMs(100);
    }
 }

//航线任务结束
void VehicleControl::wayPointMissionEnd(){
    std::thread wayPointMissionEndThread([this]{
        T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
        T_DjiReturnCode returnCode;
        //去重
        QDateTime endTime = QDateTime::currentDateTime();
        if(qAbs(endTime.msecsTo(this->missionEndTime)) < 3000){
            qWarning() << "航线任务结束重复";
            return;
        }
        this->missionEndTime = endTime;
        //重置标志位
        this->missionEntryFlag = false;
        this->missionObstacleAvoidanceFlag = false;
        this->wayPointTaskExecuteStatus == 0;
        this->missionAutoPauseCount = 0;
        this->missionManuPauseFlag = false;
        //生成飞行报告
        g_wp_mission_exe_info.endTime = endTime.toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
        WayPointMissionReportInfo wayPointMissionReportInfo;
        wayPointMissionReportInfo.taskID = g_wp_mission_info.taskID;
        wayPointMissionReportInfo.startTime = g_wp_mission_exe_info.startTime;
        wayPointMissionReportInfo.endTime = g_wp_mission_exe_info.endTime;
        wayPointMissionReportInfo.createTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
        wayPointMissionReportInfo.currentWayPointIndex = g_wp_mission_exe_info.currentWayPointIndex;
        wayPointMissionReportInfo.currentRepeatTimes = g_wp_mission_exe_info.currentRepeatTimes;
        if(g_wp_mission_exe_info.currentWayPointIndex + 1 < g_wp_mission_exe_info.wayPointTotalCount || g_wp_mission_exe_info.currentRepeatTimes < g_wp_mission_exe_info.repeatTimes){
            wayPointMissionReportInfo.executeResult = 0;
            wayPointMissionReportInfo.exceptionReason = "航线未完成执行";
        }
        else{
            wayPointMissionReportInfo.executeResult = 1;
            wayPointMissionReportInfo.exceptionReason = "";
        }
        //发送飞行报告
        Q_EMIT sendWayPointMissionReportInfo(wayPointMissionReportInfo);
        qDebug() << "飞行报告已发送" ;
        //重置全局航线执行信息
        g_wp_mission_exe_info.currentRepeatTimes = -1;
        g_wp_mission_exe_info.currentWayPointIndex = -1;
        g_wp_mission_exe_info.endTime = "";
        g_wp_mission_exe_info.startTime = "";
        g_wp_mission_exe_info.repeatTimes = -1;
        g_wp_mission_exe_info.wayPointTotalCount = -1;
        //处理航线退出, 无人机未起飞，电机转动,需主动关闭电机
        if(this->droneInfo.flightStatus == 1){
            qWarning() << "任务异常退出，无人机在地面且电机转动，主动关闭电机中";
            this->sendLog(1, "航线退出，无人机在地面且电机转动，主动关闭电机");
            returnCode = DjiFlightController_TurnOffMotors();
            if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                qCritical() << "无人机电机关闭失败!";
                this->sendLog(2, "电机关闭失败");
            }
        }
        //处理航线退出，但无人机未正常返航或降落的异常情况,循环判断直到降落(排除遥控器或者msdk获取控制权的情况)
        T_DjiReturnCode djiStat;
        T_DjiFcSubscriptionControlDevice  controlDevice = {0};
        T_DjiDataTimestamp timestamp = {0};
        while(true){
            osalHandler->TaskSleepMs(3000);
            //获取控制权
            djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                              (uint8_t *) &controlDevice,
                                                              sizeof(T_DjiFcSubscriptionControlDevice),
                                                              &timestamp);
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                qWarning() << "Get value of topic CONTROL DEVICE error, error code: 0x" << QString().sprintf("%08lX", djiStat);
            } else {
                if(controlDevice.deviceStatus < 2){
                    qWarning() << "psdk控制权被抢夺,将不执行返航校验策略";
                    break;
                }
            }
            if(g_wp_mission_info.finishAction == 0 || g_wp_mission_info.finishAction == 3){
                //完成动作为悬停或者返回起始点，即仍处于空中，此时不做异常处理
                break;
            }
            if(this->droneInfo.flightStatus == 2){
                //在空中
                if(!(this->droneInfo.displayMode == 12 || this->droneInfo.displayMode == 15 || this->droneInfo.displayMode == 33)){
                    //且飞行模式不处于返航，自动降落，强制降落状态时，需要强制其返航
                    qWarning() << "航线退出，无人机未正常返航，触发自动返航机制";
                    this->sendLog(2, "航线退出，无人机未正常返航，触发自动返航机制");
                    this->slotFlightControlGoHomeAndConfirmLandingNative();
                }
            }
            else{
                break;
            }
        }
        //重置全局航线信息
        g_wp_mission_info.taskID = -1;
        g_wp_mission_info.finishAction = -1;
        g_wp_mission_info.goHomeAltitude = -1;
        g_wp_mission_info.idleVelocity = -1;
        g_wp_mission_info.repeatTimes = -1;
        g_wp_mission_info.wayPoint.clear();
        g_wp_mission_info.wayPointAction.clear();
    });
    wayPointMissionEndThread.detach();
}

//航线任务航点发送
void VehicleControl::sendWayPoint(int curWaypointIndex){
    std::thread wayPointSendThread([this, curWaypointIndex]{
        WayPointExecuteInfo wayPointExecuteInfo;
        wayPointExecuteInfo.curWaypointIndex = curWaypointIndex;
        wayPointExecuteInfo.taskID = g_wp_mission_info.taskID;
        wayPointExecuteInfo.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
        pthread_mutex_lock(&vehicleInfoMutex);
        wayPointExecuteInfo.latitude = this->droneInfo.latitude;
        wayPointExecuteInfo.longitude = this->droneInfo.longitude;
        wayPointExecuteInfo.fusedAltitude = this->droneInfo.fusedAltitude;
        wayPointExecuteInfo.relativeAltitude = this->droneInfo.relativeAltitude;
        wayPointExecuteInfo.velocityX = this->droneInfo.velocityX;
        wayPointExecuteInfo.velocityY = this->droneInfo.velocityY;
        wayPointExecuteInfo.velocityZ = this->droneInfo.velocityZ;
        wayPointExecuteInfo.linkQuality = this->droneInfo.linkQuality;
        wayPointExecuteInfo.networkOperator = this->droneInfo.networkOperator;
        pthread_mutex_unlock(&vehicleInfoMutex);
        Q_EMIT sendWayPointExecuteInfo(wayPointExecuteInfo);
    });
    wayPointSendThread.detach();
}

//航线任务状态发送
void VehicleControl::sendWayPointState(WayPointStateInfo wayPointStateInfo){
    Q_EMIT sendWayPointStateInfo(wayPointStateInfo);
}

//航线任务日志发送
void VehicleControl::sendLog(int logLevel, string logDetail){
    LogInfo logInfo;
    logInfo.logDetail = logDetail;
    logInfo.logLevel = logLevel;
    logInfo.createTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
    Q_EMIT sendLogInfo(logInfo);
}

////异常及反馈发送
//设备异常发送
void VehicleControl::sendDeviceAbnormal(int module, string content){
    ErrorInfo errorInfo;
    errorInfo.content = content;
    errorInfo.module = module;
    errorInfo.createTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString();
    Q_EMIT sendDeviceAbnormal(errorInfo);
}

//控制返回发送
void VehicleControl::sendReturn(int module, int type, int status, string reason){
    ControlReturnInfo controlReturnInfo;
    controlReturnInfo.module = module;
    controlReturnInfo.type = type;
    controlReturnInfo.status = status;
    controlReturnInfo.reason = reason;
    Q_EMIT sendControlReturnInfo(controlReturnInfo);
}
