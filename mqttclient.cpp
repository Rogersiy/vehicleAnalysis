#include "mqttclient.h"

MqttClient* MqttClient::static_mqttClient = NULL;

MqttClient::MqttClient()
{

    static_mqttClient = this;

    if(QSqlDatabase::contains("qt_sql_default_connection"))  //查看是否存在默认连接
    {
        db = QSqlDatabase::database("qt_sql_default_connection"); //存在
    }
    else
    {
        db = QSqlDatabase::addDatabase("QSQLITE");  //不存在
    }
    db.setDatabaseName(QApplication::applicationDirPath() + "/../database/drone.db");
    if( ! db.open())
    {
       qCritical() << db.lastError().text();  //失败时输出错误信息
    }

    this->slotInitMqtt();

    moveToThread(&thread_);
    thread_.start();
}

MqttClient::~MqttClient()
{
    this->slotDeInitMqtt();

    db.removeDatabase(QApplication::applicationDirPath() + "/../database/drone.db");
    if(db.isOpen()){
        db.close();
    }

    static_mqttClient = NULL;

    thread_.quit();
    thread_.wait();
}

void MqttClient::slotInitMqtt(){
    qDebug() << "mqtt初始化中...";
    this->connected = false;

    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer5;
    MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer5;
    MQTTProperties props = MQTTProperties_initializer;
    MQTTProperty property;
    create_opts.MQTTVersion = MQTTVERSION_5;
    create_opts.sendWhileDisconnected = 1;
    create_opts.maxBufferedMessages = 1000;
    create_opts.deleteOldestMessages = 1;
    int rc;
    int ch;

    if ((rc = MQTTAsync_createWithOptions(&this->client, g_mqtt_address.toStdString().c_str(), ("drone_client_" + g_serial_number.toStdString()).c_str(),
        MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to create client, return code %d\n", rc);
        rc = EXIT_FAILURE;
    }

    if ((rc = MQTTAsync_setCallbacks(this->client, this->client, connlost, msgarrvd, NULL)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to set callbacks, return code %d\n", rc);
        rc = EXIT_FAILURE;
        MQTTAsync_destroy(&client);
    }

    if ((rc = MQTTAsync_setConnected(client, client, onReconnected)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to MQTTAsync_setConnected, return code %d\n", rc);
        rc = EXIT_FAILURE;
        MQTTAsync_destroy(&client);
    }

//    conn_opts.username = UserName;            // 用户名
//    conn_opts.password = PassWord;            // 用户名对应的密码
    conn_opts.keepAliveInterval = 3;
    conn_opts.cleanstart = 0;
    conn_opts.onSuccess5 = onConnect;
    conn_opts.onFailure5 = onConnectFailure;
    conn_opts.context = client;
    conn_opts.MQTTVersion = MQTTVERSION_5;      //使用5.0版本协议
    //断开重连设置
    conn_opts.automaticReconnect = 1;           //设置非零，断开自动重连
    conn_opts.minRetryInterval = 1;             //单位秒，重连间隔次数，每次重新连接失败时，重试间隔都会加倍，直到最大间隔
    conn_opts.maxRetryInterval = 1;             //单位秒，最大重连尝试间隔
    //设置会话超时时间
    property.identifier = MQTTPROPERTY_CODE_SESSION_EXPIRY_INTERVAL;
    property.value.integer4 = 60*60;            //一小时
    MQTTProperties_add(&props, &property);
    conn_opts.connectProperties = &props;
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
        qCritical() << "mqtt初始化失败";
        printf("Failed to s tart connect, return code %d\n", rc);
        rc = EXIT_FAILURE;
        MQTTAsync_destroy(&client);
    }
    else{
        qDebug() << "mqtt初始化成功!";
    }
}

void MqttClient::slotDeInitMqtt(){
    qDebug() << "mqtt反初始化中...";
    int rc;
    MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer5;
    MQTTProperties props = MQTTProperties_initializer;
    MQTTProperty property;
    property.identifier = MQTTPROPERTY_CODE_SESSION_EXPIRY_INTERVAL;
    property.value.integer4 = 0;                    //正常退出。立即清除回话
    MQTTProperties_add(&props, &property);
    disc_opts.properties = props;
    disc_opts.onSuccess5 = onDisconnect;
    disc_opts.onFailure5 = onDisconnectFailure;
    if ((rc = MQTTAsync_disconnect(client, &disc_opts)) != MQTTASYNC_SUCCESS)
    {
        qWarning() << "mqtt反初始化失败";
        printf("Failed to start disconnect, return code %d\n", rc);
        rc = EXIT_FAILURE;
    }

    while(this->connected){
        usleep(100000);
    }

    MQTTAsync_destroy(&client);
}

/*
 following functions will be executed in main thread
*/
void MqttClient::onConnectFailure(void* context, MQTTAsync_failureData5* response){
    printf("Connect failed, rc %d\n", response ? response->code : 0);
}

void MqttClient::onConnect(void* context, MQTTAsync_successData5* response){
    if(response->alt.connect.sessionPresent){
        qDebug() << "MQTT建立连接,会话已存在!";
    }else{
        qDebug() << "MQTT建立连接,新会话，订阅topic!";
        //会话不存在时需要订阅信息
        static_mqttClient->subscribe(("drone/" + g_serial_number.toStdString() + "/control/#").c_str(), ("drone_client_" + g_serial_number.toStdString()).c_str(), 2);
    }
}

void MqttClient::onDisconnectFailure(void* context, MQTTAsync_failureData5* response){
    printf("Disconnect failed %d, %s\n",response->code, response->message);
}

void MqttClient::onDisconnect(void* context, MQTTAsync_successData5* response){
    printf("Successful disconnection\n");
    static_mqttClient->connected = false;
}

void MqttClient::onSendFailure(void* context, MQTTAsync_failureData5* response){
    printf("Message send failed token %d error code %d\n", response->token, response->code);
}

void MqttClient::onSend(void* context, MQTTAsync_successData5* response){
    //printf("Message with token value %d delivery confirmed\n", response->token);
}

void MqttClient::onSubscribe(void* context, MQTTAsync_successData5* response){
    printf("Subscribe succeeded\n");
}

void MqttClient::onSubscribeFailure(void* context, MQTTAsync_failureData5* response){
    printf("Subscribe failed, rc %d\n", response->code);
}

//心跳时间达到，如果没有收到服务器的reponse，客户端会自动重连
void MqttClient::connlost(void *context, char *cause){
    int rc;
    printf("\nConnection lost\n");
    if (cause)
       printf("     cause: %s\n", cause);
    printf("Reconnecting\n");
    static_mqttClient->connected = false;
}

//断开重连成功
void MqttClient::onReconnected(void* context, char* cause){
    printf("Successful connected: %s!\n", cause);
    qDebug() << QString().sprintf("MQTT成功建立连接: %s!", cause);
    static_mqttClient->connected = true; 
    //service restart , no subscribe topic save, need resubscribe
    //订阅信息
    //static_mqttClient->subscribe(("drone/" + g_serial_number.toStdString() + "/control/#").c_str(), ("drone_client_" + g_serial_number.toStdString()).c_str(
    //对离线数据进行发送
    static_mqttClient->publishOfflineData();
}

//消息订阅到达
int MqttClient::msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message){
    printf("Message arrived\n");
//    printf("     topic: %s\n", topicName);
//    printf("   message: %.*s\n", message->payloadlen, (char*)message->payload);
    string content = ((char*)message->payload);
    content.resize(message->payloadlen);
    static_mqttClient->droneControl(topicName, content);
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}

//信息发布
void MqttClient::publish(char *topic, char *payload, int qos, int retained){
    if(this->connected){
        //printf("publish topic %s\n", topic);
        MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
        MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
        int rc;
        opts.onSuccess5 = this->onSend;
        opts.onFailure5 = this->onSendFailure;
        opts.context = client;
        pubmsg.payload = payload;
        pubmsg.payloadlen = (int)strlen(payload);
        pubmsg.qos = qos;
        pubmsg.retained = retained;
        if ((rc = MQTTAsync_sendMessage(client, topic, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
        {
            printf("Failed to start sendMessage, return code %d\n", rc);
        }
    }
}

//信息订阅
void MqttClient::subscribe(const char* TOPIC, const char* CLIENTID, int QOS){
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    int rc;

    printf("Subscribing to topic %s for client %s using QoS%d\n", TOPIC, CLIENTID, QOS);
    opts.onSuccess5 = onSubscribe;
    opts.onFailure5 = onSubscribeFailure;
    opts.context = client;
    if ((rc = MQTTAsync_subscribe(client, TOPIC, QOS, &opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start subscribe, return code %d\n", rc);
    }
}

//业务接口
void MqttClient::slotPublishDroneInfo(DroneInfo droneInfo){
    QJsonObject jsonobj;
    jsonobj.insert("aircraftType", droneInfo.aircraftType);
    jsonobj.insert("networkOperator", droneInfo.networkOperator.c_str());
    jsonobj.insert("signalQuality", droneInfo.signalQuality);
    jsonobj.insert("linkQuality", droneInfo.linkQuality);
    jsonobj.insert("algorithmStatus", int(g_analysis));
    jsonobj.insert("flightStatus", droneInfo.flightStatus);
    jsonobj.insert("displayMode", droneInfo.displayMode);
    jsonobj.insert("latitude", droneInfo.latitude);
    jsonobj.insert("longitude", droneInfo.longitude);
    jsonobj.insert("RTKConnectStatus", droneInfo.RTKConnectStatus);
    jsonobj.insert("relativeHeight", droneInfo.relativeHeight);
    jsonobj.insert("relativeAltitude", droneInfo.relativeAltitude);
    jsonobj.insert("fusedAltitude", droneInfo.fusedAltitude);
    jsonobj.insert("visibleSatelliteNumber", droneInfo.visibleSatelliteNumber);
    jsonobj.insert("velocityX", droneInfo.velocityX);
    jsonobj.insert("velocityY", droneInfo.velocityY);
    jsonobj.insert("velocityZ", droneInfo.velocityZ);
    jsonobj.insert("firstBatteryCapacityPercent", droneInfo.firstBatteryCapacityPercent);
    jsonobj.insert("firstBatteryTemperature", droneInfo.firstBatteryTemperature);
    jsonobj.insert("firstBatteryVoltage", droneInfo.firstBatteryVoltage);
    jsonobj.insert("secondBatteryCapacityPercent", droneInfo.secondBatteryCapacityPercent);
    jsonobj.insert("secondBatteryTemperature", droneInfo.secondBatteryTemperature);
    jsonobj.insert("secondBatteryVoltage", droneInfo.secondBatteryVoltage);
    jsonobj.insert("gimbalStatus", droneInfo.gimbalStatus);
    jsonobj.insert("gimbalMode", droneInfo.gimbalMode);
    jsonobj.insert("gimbalYaw", droneInfo.gimbalYaw);
    jsonobj.insert("gimbalPitch", droneInfo.gimbalPitch);
    jsonobj.insert("gimbalRoll", droneInfo.gimbalRoll);
    jsonobj.insert("time", droneInfo.time.c_str());
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/state").c_str(), (char*)msg.c_str(), 0, 0);
}

void MqttClient::slotPublishCameraInfo(CameraInfo cameraInfo){
    QJsonObject jsonobj;
    jsonobj.insert("cameraType", cameraInfo.cameraType);
    jsonobj.insert("cameraFirmwareVersion", cameraInfo.cameraFirmwareVersion.c_str());
    jsonobj.insert("tapZoomEnabled", cameraInfo.tapZoomEnabled);
    jsonobj.insert("tapZoomMultiplier", cameraInfo.tapZoomMultiplier);
    jsonobj.insert("cameraSource", cameraInfo.cameraSource);
    jsonobj.insert("cameraWorkMode", cameraInfo.cameraWorkMode);
    jsonobj.insert("cameraShootPhotoMode", cameraInfo.cameraShootPhotoMode);
    jsonobj.insert("cameraFocusMode", cameraInfo.cameraFocusMode);
    jsonobj.insert("currentOpticalZoomFactor", cameraInfo.currentOpticalZoomFactor);
    jsonobj.insert("maxOpticalZoomFactor", cameraInfo.maxOpticalZoomFactor);
    jsonobj.insert("cameraRecordStatus", cameraInfo.cameraRecordStatus);
    jsonobj.insert("cameraZoomStatus", cameraInfo.cameraZoomStatus);
    jsonobj.insert("cameraDownloadStatus", cameraInfo.cameraDownloadStatus);
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/camera").c_str(), (char*)msg.c_str(), 0, 0);
}

void MqttClient::slotPublishWayPointStateInfo(WayPointStateInfo wayPointStateInfo){
    QJsonObject jsonobj;
    jsonobj.insert("curWaypointIndex", wayPointStateInfo.curWaypointIndex);
    jsonobj.insert("velocity", wayPointStateInfo.velocity);
    jsonobj.insert("state", wayPointStateInfo.state);
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/wayPointState").c_str(), (char*)msg.c_str(), 0, 0);
}

void MqttClient::slotPublishCameraFileList(vector<CameraFileInfo> cameraFileList){
    QJsonArray jsonArray;
    for(unsigned int i = 0; i < cameraFileList.size(); i++){
        QJsonObject jsonobj;
        jsonobj.insert("fileName", cameraFileList[i].fileName.c_str());
        jsonobj.insert("fileSize", int(cameraFileList[i].fileSize));
        jsonobj.insert("fileIndex", cameraFileList[i].fileIndex.c_str());
        jsonobj.insert("createTime", cameraFileList[i].createTime.c_str());
        jsonobj.insert("fileType", int(cameraFileList[i].fileType));
        jsonobj.insert("videoDuration", int(cameraFileList[i].videoDuration));
        jsonArray.append(jsonobj);
    }
    QJsonDocument doc(jsonArray);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/cameraFileList").c_str(), (char*)msg.c_str(), 0, 0);
}

void MqttClient::slotPublishCameraFileData(DownloadFileInfo downloadFileInfo){
    QJsonObject jsonobj;
    jsonobj.insert("fileIndex", downloadFileInfo.fileIndex.c_str());
    jsonobj.insert("progressInPercent", downloadFileInfo.progressInPercent);
    jsonobj.insert("downloadFileEvent", downloadFileInfo.downloadFileEvent);
    jsonobj.insert("dataLen", downloadFileInfo.dataLen);
    jsonobj.insert("data", downloadFileInfo.data.c_str());
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/cameraFileData").c_str(), (char*)msg.c_str(), 0, 0);
}

void MqttClient::slotPublishControlReturnInfo(ControlReturnInfo controlReturnInfo){
    QJsonObject jsonobj;
    jsonobj.insert("type", controlReturnInfo.type);
    jsonobj.insert("status", controlReturnInfo.status);
    jsonobj.insert("reason", controlReturnInfo.reason.c_str());
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    switch (controlReturnInfo.module) {
    case 0:
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/return/gimbal").c_str(), (char*)msg.c_str(), 2, 0);
        break;
    case 1:
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/return/camera").c_str(), (char*)msg.c_str(), 2, 0);
        break;
    case 2:
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/return/flightControl").c_str(), (char*)msg.c_str(), 2, 0);
        break;
    case 3:
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/return/wayPoint").c_str(), (char*)msg.c_str(), 2, 0);
        break;
    default:
        break;
    }
}

////以下数据需要离线存储
void MqttClient::slotPublishLogInfo(LogInfo logInfo){
    //根据网络情况进行日志的发送与离线存储,需要去重
    if(this->connected){
        //去重
        if(this->logDuplicationMap.count(logInfo.logDetail)){
            if(this->logDuplicationMap[logInfo.logDetail].size() > 0){
                QDateTime time = QDateTime::fromString(QString::fromStdString(logInfo.createTime), "yyyy-MM-dd hh:mm:ss.zzz");
                for(unsigned int i = 0; i < this->logDuplicationMap[logInfo.logDetail].size(); i++){
                    //判断其时间是否在3s以内
                    if(qAbs(time.msecsTo(QDateTime::fromString(QString::fromStdString(this->logDuplicationMap[logInfo.logDetail][i]), "yyyy-MM-dd hh:mm:ss.zzz"))) < 3000){
                        return;
                    }
                }
            }
        }
        else{
            vector<string> logVector;
            this->logDuplicationMap[logInfo.logDetail] = logVector;
        }
        this->logDuplicationMap[logInfo.logDetail].push_back(logInfo.createTime);
        //发送
        QJsonObject jsonobj;
        jsonobj.insert("createTime", logInfo.createTime.c_str());
        jsonobj.insert("logLevel", logInfo.logLevel);
        jsonobj.insert("logDetail", logInfo.logDetail.c_str());
        QJsonDocument doc(jsonobj);
        QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
        std::string msg(byte_array.data(),byte_array.length());
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/log").c_str(), (char*)msg.c_str(), 2, 0);
    }
    else{
        //离线存储
        if(! db.isOpen()){
            if( ! db.open())
            {
               qDebug() << db.lastError().text();  //失败时输出错误信息
               return;
            }
        }
        QSqlQuery query;
        QString sql = "insert into log_info(logLevel, logDetail, createTime) values (" + QString::number(logInfo.logLevel) + ", '" + QString::fromStdString(logInfo.logDetail) + "', '"  +
                QString::fromStdString(logInfo.createTime) + "');";
        if(! query.exec(sql)){
            cout << "insert log info error" << db.lastError().text().toStdString();
            qCritical() << "insert log info error" << db.lastError();
        }
    }
}

void MqttClient::slotPublishWayPointExecuteInfo(WayPointExecuteInfo wayPointExecuteInfo){
    //根据网络情况进行日志的发送与离线存储,需要去重
    if(this->connected){
        //去重
        if(this->wayPointDuplicationMap.count(wayPointExecuteInfo.curWaypointIndex)){
            if(this->wayPointDuplicationMap[wayPointExecuteInfo.curWaypointIndex].size() > 0){
                QDateTime time = QDateTime::fromString(QString::fromStdString(wayPointExecuteInfo.time), "yyyy-MM-dd hh:mm:ss.zzz");
                for(unsigned int i = 0; i < this->wayPointDuplicationMap[wayPointExecuteInfo.curWaypointIndex].size(); i++){
                    //判断其时间是否在3s以内
                    if(qAbs(time.msecsTo(QDateTime::fromString(QString::fromStdString(this->wayPointDuplicationMap[wayPointExecuteInfo.curWaypointIndex][i]), "yyyy-MM-dd hh:mm:ss.zzz"))) < 3000){
                        return;
                    }
                }
            }
        }
        else{
            vector<string> logVector;
            this->wayPointDuplicationMap[wayPointExecuteInfo.curWaypointIndex] = logVector;
        }
        this->wayPointDuplicationMap[wayPointExecuteInfo.curWaypointIndex].push_back(wayPointExecuteInfo.time);
        //发送
        QJsonObject jsonobj;
        jsonobj.insert("taskID", wayPointExecuteInfo.taskID);
        jsonobj.insert("curWaypointIndex", wayPointExecuteInfo.curWaypointIndex);
        jsonobj.insert("latitude", wayPointExecuteInfo.latitude);
        jsonobj.insert("longitude", wayPointExecuteInfo.longitude);
        jsonobj.insert("relativeAltitude", wayPointExecuteInfo.relativeAltitude);
        jsonobj.insert("fusedAltitude", wayPointExecuteInfo.fusedAltitude);
        jsonobj.insert("velocityX", wayPointExecuteInfo.velocityX);
        jsonobj.insert("velocityY", wayPointExecuteInfo.velocityY);
        jsonobj.insert("velocityZ", wayPointExecuteInfo.velocityZ);
        jsonobj.insert("linkQuality", wayPointExecuteInfo.linkQuality);
        jsonobj.insert("networkOperator", wayPointExecuteInfo.networkOperator.c_str());
        jsonobj.insert("time", wayPointExecuteInfo.time.c_str());
        QJsonDocument doc(jsonobj);
        QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
        std::string msg(byte_array.data(),byte_array.length());
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/wayPoint").c_str(), (char*)msg.c_str(), 2, 0);
    }
    else{
        if(! db.isOpen()){
            if( ! db.open())
            {
               qDebug() << db.lastError().text();  //失败时输出错误信息
               return;
            }
        }
        QSqlQuery query;
        QString sql = "insert into wp_info(taskID, curWaypointIndex, latitude, longitude, relativeAltitude, fusedAltitude, velocityX, velocityY, velocityZ, linkQuality, networkOperator, time) values (" +
                QString::number(wayPointExecuteInfo.taskID) + ", " + QString::number(wayPointExecuteInfo.curWaypointIndex) + ", "  + QString::number(wayPointExecuteInfo.latitude) + ", "
                + QString::number(wayPointExecuteInfo.longitude) + ", " + QString::number(wayPointExecuteInfo.relativeAltitude) + ", " + QString::number(wayPointExecuteInfo.fusedAltitude) + ", "
                + QString::number(wayPointExecuteInfo.velocityX) + ", " + QString::number(wayPointExecuteInfo.velocityY) + ", " + QString::number(wayPointExecuteInfo.velocityZ) + ", " +
                QString::number(wayPointExecuteInfo.linkQuality) + ", '" + QString::fromStdString(wayPointExecuteInfo.networkOperator) + "', '" + QString::fromStdString(wayPointExecuteInfo.time) + "');";
        if(! query.exec(sql)){
            cout << "insert wp info error" << db.lastError().text().toStdString();
            qCritical() << "insert wp info error" << db.lastError();
        }
    }
}

void MqttClient::slotPublishWayPointMissionReportInfo(WayPointMissionReportInfo wayPointMissionReportInfo){
    //根据网络情况进行报告的发送与离线存储
    if(this->connected){
        QJsonObject jsonobj;
        jsonobj.insert("taskID", wayPointMissionReportInfo.taskID);
        jsonobj.insert("executeResult", wayPointMissionReportInfo.executeResult);
        jsonobj.insert("exceptionReason", wayPointMissionReportInfo.exceptionReason.c_str());
        jsonobj.insert("createTime", wayPointMissionReportInfo.createTime.c_str());
        jsonobj.insert("startTime", wayPointMissionReportInfo.startTime.c_str());
        jsonobj.insert("endTime", wayPointMissionReportInfo.endTime.c_str());
        jsonobj.insert("currentWayPointIndex", wayPointMissionReportInfo.currentWayPointIndex);
        jsonobj.insert("currentRepeatTimes", wayPointMissionReportInfo.currentRepeatTimes);
        QJsonDocument doc(jsonobj);
        QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
        std::string msg(byte_array.data(),byte_array.length());
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/report/wayPointMission").c_str(), (char*)msg.c_str(), 2, 0);
    }
    else{
        if(! db.isOpen()){
            if( ! db.open())
            {
               qDebug() << db.lastError().text();  //失败时输出错误信息
               return;
            }
        }
        QSqlQuery query;
        QString sql = "insert into wp_report_info(taskID, executeResult, exceptionReason, createTime, startTime, endTime, currentWayPointIndex, currentRepeatTimes) values (" +
                QString::number(wayPointMissionReportInfo.taskID) + ", " + QString::number(wayPointMissionReportInfo.executeResult) + ", '"  +
                QString::fromStdString(wayPointMissionReportInfo.exceptionReason) + "', '" + QString::fromStdString(wayPointMissionReportInfo.createTime) + "', '" +
                QString::fromStdString(wayPointMissionReportInfo.startTime) + "', '" + QString::fromStdString(wayPointMissionReportInfo.endTime) + "', " +
                QString::number(wayPointMissionReportInfo.currentWayPointIndex) + ", " + QString::number(wayPointMissionReportInfo.currentRepeatTimes) + ");";
        if(! query.exec(sql)){
            cout << "insert wp_report info error" << db.lastError().text().toStdString();
            qCritical() << "insert wp_report info error" << db.lastError();
        }
    }
}

void MqttClient::slotPublishEventInfo(EventInfo eventInfo){
    //根据网络情况进行事件的发送与离线存储
    if(this->connected){
        QJsonObject jsonobj;
        jsonobj.insert("UUID", eventInfo.UUID.c_str());
        jsonobj.insert("taskID", eventInfo.taskID);
        jsonobj.insert("eventType", eventInfo.eventType);
        jsonobj.insert("createTime", eventInfo.createTime.c_str());
        jsonobj.insert("eventDescribe", eventInfo.eventDescribe.c_str());
        jsonobj.insert("longitude", eventInfo.longitude);
        jsonobj.insert("latitude", eventInfo.latitude);
        jsonobj.insert("picture", Tools::getBase64(eventInfo.picture, eventInfo.pictureCode).c_str());
        jsonobj.insert("pictureCode", eventInfo.pictureCode.c_str());
        QJsonDocument doc(jsonobj);
        QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
        std::string msg(byte_array.data(),byte_array.length());
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/info/event").c_str(), (char*)msg.c_str(), 2, 0);
    }
    else{
        if(! db.isOpen()){
            if( ! db.open())
            {
               qDebug() << db.lastError().text();  //失败时输出错误信息
               return;
            }
        }
        string pictureAddress = QApplication::applicationDirPath().toStdString() + "/../images/event/event_" + to_string(time(NULL)) + "." + eventInfo.pictureCode;
        cv::imwrite(pictureAddress, eventInfo.picture);
        QSqlQuery query;
        QString sql = "insert into event_info(eventType, createTime, eventDescribe, pictureAddress, pictureCode, taskID, longitude, latitude, UUID) values (" + QString::number(eventInfo.eventType) +
                ", '" + QString::fromStdString(eventInfo.createTime) + "', '"  + QString::fromStdString(eventInfo.eventDescribe) + "', '" + QString::fromStdString(pictureAddress) + "', '" +
                QString::fromStdString(eventInfo.pictureCode) + "', " + QString::number(eventInfo.taskID) + ", " + QString::number(eventInfo.longitude) + ", " +
                QString::number(eventInfo.latitude) + ", '" + QString::fromStdString(eventInfo.UUID) + "');";
        if(! query.exec(sql)){
            cout << "insert event info error" << db.lastError().text().toStdString();
            qCritical() << "insert event info error" << db.lastError();
        }
    }
}

void MqttClient::slotPublishDeviceAbnormal(ErrorInfo errorInfo){
    if(this->connected){
        QJsonObject jsonobj;
        jsonobj.insert("module", errorInfo.module);
        jsonobj.insert("content", errorInfo.content.c_str());
        jsonobj.insert("createTime", errorInfo.createTime.c_str());
        QJsonDocument doc(jsonobj);
        QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
        std::string msg(byte_array.data(),byte_array.length());
        this->publish((char*)("drone/" + g_serial_number.toStdString() + "/abnormal").c_str(), (char*)msg.c_str(), 2, 0);
    }else{
        //离线存储
        if(! db.isOpen()){
            if( ! db.open())
            {
               qDebug() << db.lastError().text();  //失败时输出错误信息
               return;
            }
        }
        QSqlQuery query;
        QString sql = "insert into device_abnormal_info(module, content, createTime) values (" + QString::number(errorInfo.module) + ", '" + QString::fromStdString(errorInfo.content)
                + "', '"  + QString::fromStdString(errorInfo.createTime) + "');";
        if(! query.exec(sql)){
            cout << "insert device_abnormal_info error" << db.lastError().text().toStdString();
            qCritical() << "insert device_abnormal_info error" << db.lastError();
        }
    }
}

void MqttClient::publishOfflineData(){
    if(! db.isOpen()){
        if( ! db.open())
        {
           qCritical() << db.lastError().text();  //失败时输出错误信息
           return;
        }
    }
    set<int> deleteIdSet;
    QSqlQuery query;
    //log*********
    if(!query.exec("select * from log_info order by id asc;")){
        cout << "select log info error" << db.lastError().text().toStdString();
        qCritical() << "select log info error" << db.lastError();
    }
    while(query.next())
    {
        int id = query.value(0).toInt();
        deleteIdSet.insert(id);
        LogInfo logInfo;
        logInfo.logLevel = query.value(1).toInt();
        logInfo.logDetail = query.value(2).toString().toStdString();
        logInfo.createTime = query.value(3).toString().toStdString();
        this->slotPublishLogInfo(logInfo);
    }
    for(auto it = deleteIdSet.begin(); it != deleteIdSet.end(); it++){
        if(!query.exec("delete from log_info where id = " + QString::number(*it) + ";")){
            cout << "delete log info error" << db.lastError().text().toStdString();
            qCritical() << "delete log info error" << db.lastError();
        }
    }
    deleteIdSet.clear();
    //event*******
    if(! query.exec("select * from event_info order by id asc;")){
        cout << "select event info error" << db.lastError().text().toStdString();
        qCritical() << "select event info error" << db.lastError();
    }
    while(query.next())
    {
        int id = query.value(0).toInt();
        string pictureAddress = query.value(4).toString().toStdString();
        deleteIdSet.insert(id);
        EventInfo eventInfo;
        eventInfo.eventType = query.value(1).toInt();
        eventInfo.createTime = query.value(2).toString().toStdString();
        eventInfo.eventDescribe = query.value(3).toString().toStdString();
        eventInfo.picture = cv::imread(pictureAddress);
        eventInfo.pictureCode = query.value(5).toString().toStdString();
        eventInfo.taskID = query.value(6).toInt();
        eventInfo.longitude = query.value(7).toDouble();
        eventInfo.latitude = query.value(8).toDouble();
        eventInfo.UUID = query.value(9).toString().toStdString();
        this->slotPublishEventInfo(eventInfo);
        //删除本地文件
        if(remove(pictureAddress.c_str())==0){
            qDebug() << "文件删除成功!";
        }
        else{
            qCritical() << "文件删除失败!";
        }
    }
    for(auto it = deleteIdSet.begin(); it != deleteIdSet.end(); it++){
        if(!query.exec("delete from event_info where id = " + QString::number(*it) + ";")){
            cout << "delete event info error" << db.lastError().text().toStdString();
            qCritical() << "delete event info error" << db.lastError();
        }
    }
    deleteIdSet.clear();
    //wp*******
    if(! query.exec("select * from wp_info order by id asc;")){
        cout << "select wp info error" << db.lastError().text().toStdString();
        qCritical() << "select wp info error" << db.lastError();
    }
    while(query.next())
    {
        int id = query.value(0).toInt();
        deleteIdSet.insert(id);
        WayPointExecuteInfo wayPointExecuteInfo;
        wayPointExecuteInfo.taskID = query.value(1).toInt();
        wayPointExecuteInfo.curWaypointIndex = query.value(2).toInt();
        wayPointExecuteInfo.latitude = query.value(3).toDouble();
        wayPointExecuteInfo.longitude = query.value(4).toDouble();
        wayPointExecuteInfo.relativeAltitude = query.value(5).toDouble();
        wayPointExecuteInfo.fusedAltitude = query.value(6).toDouble();
        wayPointExecuteInfo.velocityX = query.value(7).toDouble();
        wayPointExecuteInfo.velocityY = query.value(8).toDouble();
        wayPointExecuteInfo.velocityZ = query.value(9).toDouble();
        wayPointExecuteInfo.linkQuality = query.value(10).toDouble();
        wayPointExecuteInfo.networkOperator = query.value(11).toString().toStdString();
        wayPointExecuteInfo.time = query.value(12).toString().toStdString();
        this->slotPublishWayPointExecuteInfo(wayPointExecuteInfo);
    }
    for(auto it = deleteIdSet.begin(); it != deleteIdSet.end(); it++){
        if(!query.exec("delete from wp_info where id = " + QString::number(*it) + ";")){
            cout << "delete wp info error" << db.lastError().text().toStdString();
            qCritical() << "delete wp info error" << db.lastError();
        }
    }
    deleteIdSet.clear();
    //wp report*******
    if(! query.exec("select * from wp_report_info order by id asc;")){
        cout << "select wp_report info error" << db.lastError().text().toStdString();
        qCritical() << "select wp_report info error" << db.lastError();
    }
    while(query.next())
    {
        int id = query.value(0).toInt();
        deleteIdSet.insert(id);
        WayPointMissionReportInfo wayPointMissionReportInfo;
        wayPointMissionReportInfo.taskID = query.value(1).toInt();
        wayPointMissionReportInfo.executeResult = query.value(2).toInt();
        wayPointMissionReportInfo.exceptionReason = query.value(3).toString().toStdString();
        wayPointMissionReportInfo.createTime = query.value(4).toString().toStdString();
        wayPointMissionReportInfo.startTime = query.value(5).toString().toStdString();
        wayPointMissionReportInfo.endTime = query.value(6).toString().toStdString();
        wayPointMissionReportInfo.currentWayPointIndex = query.value(7).toInt();
        wayPointMissionReportInfo.currentRepeatTimes = query.value(8).toInt();
        this->slotPublishWayPointMissionReportInfo(wayPointMissionReportInfo);
    }
    for(auto it = deleteIdSet.begin(); it != deleteIdSet.end(); it++){
        if(!query.exec("delete from wp_report_info where id = " + QString::number(*it) + ";")){
            cout << "delete wp_report_info error" << db.lastError().text().toStdString();
            qCritical() << "delete wp_report_info error" << db.lastError();
        }
    }
    deleteIdSet.clear();
    //device abnormal*********
    if(!query.exec("select * from device_abnormal_info order by id asc;")){
        cout << "select device_abnormal_info error" << db.lastError().text().toStdString();
        qCritical() << "select device_abnormal_info error" << db.lastError();
    }
    while(query.next())
    {
        int id = query.value(0).toInt();
        deleteIdSet.insert(id);
        ErrorInfo errorInfo;
        errorInfo.module = query.value(1).toInt();
        errorInfo.content = query.value(2).toString().toStdString();
        errorInfo.createTime = query.value(3).toString().toStdString();
        this->slotPublishDeviceAbnormal(errorInfo);
    }
    for(auto it = deleteIdSet.begin(); it != deleteIdSet.end(); it++){
        if(!query.exec("delete from device_abnormal_info where id = " + QString::number(*it) + ";")){
            cout << "delete device_abnormal_info error" << db.lastError().text().toStdString();
            qCritical() << "delete device_abnormal_info error" << db.lastError();
        }
    }
    deleteIdSet.clear();
}

////无人机控制
void MqttClient::droneControl(string topic, string content){
    QStringList topicList = QString::fromStdString(topic).split("/");
    if(topicList[2] != "control"){
        return;
    }
    if(topicList[3] == "gimbal"){
        QJsonParseError json_error;
        QJsonDocument json_doc = QJsonDocument::fromJson(content.c_str(), &json_error);
        if (json_error.error != QJsonParseError::NoError){
            qCritical() << "control gimbal json parse error!";
            return;
        }
        QJsonObject json_object = json_doc.object();
        switch (json_object["code"].toInt()) {
        case 0:
            {
                qDebug() << "收到云台回中指令";
                Q_EMIT sendResetGimbal();
                break;
            }
        case 1:
            {
                qDebug() << "收到云台Pitch角调整指令:" << json_object["rodQuantity"].toInt();
                Q_EMIT sendRotateGimbalPitchPositive(json_object["rodQuantity"].toInt());
                break;
            }
        case 2:
            {
                qDebug() << "收到云台Pitch角调整指令:" << json_object["rodQuantity"].toInt();
                Q_EMIT sendRotateGimbalPitchNegative(json_object["rodQuantity"].toInt());
                break;
            }
        case 3:
            {
                qDebug() << "收到云台Yaw角调整指令:" << json_object["rodQuantity"].toInt();
                Q_EMIT sendRotateGimbalYawNegative(json_object["rodQuantity"].toInt());
                break;
            }
        case 4:
            {
                qDebug() << "收到云台Yaw角调整指令:" << json_object["rodQuantity"].toInt();
                Q_EMIT sendRotateGimbalYawPositive(json_object["rodQuantity"].toInt());
                break;
            }
        default:
            break;
        }
    }
    else if(topicList[3] == "camera"){
        QJsonParseError json_error;
        QJsonDocument json_doc = QJsonDocument::fromJson(content.c_str(), &json_error);
        if (json_error.error != QJsonParseError::NoError){
            qCritical() << "control camera json parse error!";
            return;
        }
        QJsonObject json_object = json_doc.object();
        switch (json_object["code"].toInt()) {
        case 0:
            {

                string source = json_object["cameraSource"].toString().toStdString();
                qDebug() << "收到相机镜头切换指令:" << QString::fromStdString(source);
                if(source == "FPV" || source == "WIDE" || source == "ZOOM" || source == "IR"){
                    Q_EMIT sendChangeLiveViewCameraSource(source);
                }
            }
            break;
        case 1:
            {
                qDebug() << "收到相机工作模式切换指令:" << json_object["workMode"].toInt();
                Q_EMIT sendChangeCameraWorkMode(json_object["workMode"].toInt());
                break;
            }
        case 2:
            {
                qDebug() << "收到相机拍照指令";
                Q_EMIT sendShootSinglePhoto();
                break;
            }
        case 3:
            {
                qDebug() << "收到相机开始录像指令";
                Q_EMIT sendStartRecordVideo();
                break;
            }
        case 4:
            {
                qDebug() << "收到相机停止录像指令";
                Q_EMIT sendStopRecordVideo();
                break;
            }
        case 5:
            {
                qDebug() << "收到相机变焦指令,倍数:" << json_object["zoomFactor"].toInt();
                Q_EMIT sendSetOpticalZoomParam(json_object["zoomFactor"].toInt());
                break;
            }
        case 6:
            {
                QJsonValue value = json_object.value("tapZoomPoint");
                QJsonObject object = value.toObject();
                qDebug() << "收到相机指点变焦指令,位置:" << object["x"].toDouble() << "," << object["y"].toDouble() << " 倍数:" << object["multiplier"].toInt();
                CameraTapZoomPoint tapZoomPoint;
                tapZoomPoint.x = object["x"].toDouble();
                tapZoomPoint.y = object["y"].toDouble();
                tapZoomPoint.multiplier = object["multiplier"].toInt();
                Q_EMIT sendSetTapZoomPoint(tapZoomPoint);
            }
            break;
        case 7:
            {
                QJsonValue value = json_object.value("focusPoint");
                QJsonObject object = value.toObject();
                qDebug() << "收到相机指点对焦指令,位置:" << object["x"].toDouble() << "," << object["y"].toDouble();
                CameraFocusPoint focusPoint;
                focusPoint.x = object["x"].toDouble();
                focusPoint.y = object["y"].toDouble();
                Q_EMIT sendSetFoucsPoint(focusPoint);
            }
            break;
        case 8:
            {
                qDebug() << "收到获取相机文件列表指令";
                Q_EMIT sendGetDownloadFileList();
                break;
            }
        case 9:
            {
                qDebug() << "收到下载相机文件指令,文件index：" << json_object["fileIndex"].toString().toInt();
                Q_EMIT sendDownloadFile(json_object["fileIndex"].toString().toInt());
                break;
            }
        case 10:
            {
                qDebug() << "收到删除相机文件指令,文件index：" << json_object["fileIndex"].toString().toInt();
                Q_EMIT sendDeleteFile(json_object["fileIndex"].toString().toInt());
                break;
            }
        default:
            break;
        }
    }
    else if(topicList[3] == "wayPoint"){
        QJsonParseError json_error;
        QJsonDocument json_doc = QJsonDocument::fromJson(content.c_str(), &json_error);
        if (json_error.error != QJsonParseError::NoError){
            qCritical() << "control wayPoint json parse error!";
            return;
        }
        QJsonObject json_object = json_doc.object();
        switch (json_object["code"].toInt()) {
        case 0:
            {
                qDebug() << "收到航线上传指令:" << QString::fromStdString(content);
                QJsonValue missionValue = json_object.value("wayPointMissionInfo");
                QJsonObject missionObject = missionValue.toObject();
                WayPointMissionInfo wayPointMissionInfo;
                wayPointMissionInfo.taskID = missionObject["taskID"].toInt();
                wayPointMissionInfo.finishAction = missionObject["finishAction"].toInt();
                wayPointMissionInfo.repeatTimes = missionObject["repeatTimes"].toInt();
                wayPointMissionInfo.idleVelocity = missionObject["idleVelocity"].toDouble();
                wayPointMissionInfo.goHomeAltitude = missionObject["goHomeAltitude"].toInt();
                QJsonValue pointValue = missionObject.value("wayPoint");
                QJsonArray pointArray = pointValue.toArray();
                for(int i = 0; i < pointArray.count(); i++){
                    WayPointInfo wayPointInfo;
                    wayPointInfo.index = pointArray.at(i).toObject()["index"].toInt();
                    wayPointInfo.latitude = pointArray.at(i).toObject()["latitude"].toDouble();
                    wayPointInfo.longitude = pointArray.at(i).toObject()["longitude"].toDouble();
                    wayPointInfo.relativeHeight = pointArray.at(i).toObject()["relativeHeight"].toDouble();
                    wayPointMissionInfo.wayPoint.push_back(wayPointInfo);
                }
                QJsonValue actionValue = missionObject.value("wayPointAction");
                QJsonArray actionArray = actionValue.toArray();
                for(int i = 0; i < actionArray.count(); i++){
                    WayPointActionInfo wayPointActionInfo;
                    wayPointActionInfo.wayPointIndex = actionArray.at(i).toObject()["wayPointIndex"].toInt();
                    wayPointActionInfo.actionActuatorType = actionArray.at(i).toObject()["actionActuatorType"].toInt();
                    wayPointActionInfo.hoverTime = actionArray.at(i).toObject()["hoverTime"].toInt();
                    wayPointMissionInfo.wayPointAction.push_back(wayPointActionInfo);
                }
                Q_EMIT sendUploadWayPointMission(wayPointMissionInfo);
            }
            break;
        case 1:
            {
                qDebug() << "收到航线开始指令";
                Q_EMIT sendStartWayPointMission();
                break;
            }
        case 2:
            {
                qDebug() << "收到航线停止指令";
                Q_EMIT sendStopWayPointMission();
                break;
            }
        case 3:
            {
                qDebug() << "收到航线暂停指令";
                Q_EMIT sendPauseWayPointMission();
                break;
            }
        case 4:
            {
                qDebug() << "收到航线恢复指令";
                Q_EMIT sendResumeWayPointMission();
                break;
            }
        default:
            break;
        }
    }
    else if(topicList[3] == "flightControl"){
        QJsonParseError json_error;
        QJsonDocument json_doc = QJsonDocument::fromJson(content.c_str(), &json_error);
        if (json_error.error != QJsonParseError::NoError){
            qCritical() << "control wayPoint json parse error!";
            return;
        }
        QJsonObject json_object = json_doc.object();
        switch (json_object["code"].toInt()) {
        case 0:
            {
                qDebug() << "收到一键返航指令";
                Q_EMIT sendFlightControlGoHomeAndConfirmLanding();
                break;
            }
        default:
            break;
        }
    }
    else{
        qWarning() << "控制指令不支持:" << QString::fromStdString(content);
        printf("not supported");
    }
}
