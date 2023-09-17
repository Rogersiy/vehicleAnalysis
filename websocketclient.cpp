#include "websocketclient.h"

/*
 following functions will be executed in main thread
*/
WebSocketClient::WebSocketClient()
{
    moveToThread(&thread_);
    thread_.start();
}

WebSocketClient::~WebSocketClient()
{
    thread_.quit();
    thread_.wait();
}

void WebSocketClient::OnConnected(std::string const& nsp)
{
    qDebug() << "Socket (RE)Connected...";
}

void WebSocketClient::OnClosed(const sio::client::close_reason &reason)
{
    qDebug() << "Socket Closed.";
}

void WebSocketClient::OnFailed()
{
    qDebug() << "Socket Connect failed.";
}

void WebSocketClient::OnNewMessage(std::string const& name, sio::message::ptr const& data,bool hasAck, sio::message::list &ack_resp){ //处理接收消息 name为事件名称，data为事件内容
    qDebug() << "Socket received:" << data->get_string().data();
}

// 连接websocket服务器的URL
void WebSocketClient::slotConnectUrl(QString url)
{
    qDebug() << "webSocket init...";
    socket_t = new sio::client();
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    sio::socket::ptr sock = socket_t->socket();
    socket_t->set_socket_open_listener(std::bind(&WebSocketClient::OnConnected,this, std::placeholders::_1));
    socket_t->set_close_listener(std::bind(&WebSocketClient::OnClosed,this, _1));
    socket_t->set_fail_listener(std::bind(&WebSocketClient::OnFailed,this));
    BIND_EVENT(sock, "operate", std::bind(&WebSocketClient::OnNewMessage, this, _1, _2, _3, _4)); //绑定事件，当接收到chat message事件时，执行OnNewMessage函数
    map<string,string> query;
    query.insert(pair<string, string>("type", "web"));
    query.insert(pair<string, string>("serialNo", g_serial_number.toStdString()));

    qDebug() << "webSocket connect...";
    socket_t->connect(url.toStdString(), query);
}

// 关闭websocket
void WebSocketClient::slotClose()
{
    qDebug() << "webSocket close...";
    socket_t->close();
    delete socket_t;
}


void WebSocketClient::slotsendDroneInfo(DroneInfo drone){
    QJsonObject jsonobj;
    jsonobj.insert("algorithmStatus", int(g_analysis));
    jsonobj.insert("flightStatus", drone.flightStatus);
    jsonobj.insert("latitude", drone.latitude);
    jsonobj.insert("longitude", drone.longitude);
    jsonobj.insert("altitude", drone.fusedAltitude);
    jsonobj.insert("velocityX", drone.velocityX);
    jsonobj.insert("velocityY", drone.velocityY);
    jsonobj.insert("velocityZ", drone.velocityZ);
    jsonobj.insert("firstBatteryCapacityPercent", drone.firstBatteryCapacityPercent);
    jsonobj.insert("firstBatteryTemperature", drone.firstBatteryTemperature);
    jsonobj.insert("firstBatteryVoltage", drone.firstBatteryVoltage);
    jsonobj.insert("secondBatteryCapacityPercent", drone.secondBatteryCapacityPercent);
    jsonobj.insert("secondBatteryTemperature", drone.secondBatteryTemperature);
    jsonobj.insert("secondBatteryVoltage", drone.secondBatteryVoltage);
    jsonobj.insert("gimbalStatus", drone.gimbalStatus);
    jsonobj.insert("gimbalMode", drone.gimbalMode);
    jsonobj.insert("gimbalYaw", drone.gimbalYaw);
    jsonobj.insert("gimbalPitch", drone.gimbalPitch);
    jsonobj.insert("gimbalRoll", drone.gimbalRoll);
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string msg(byte_array.data(),byte_array.length());
    qDebug() << "send drone Info: " << QString::fromStdString(msg);
    socket_t->socket()->emit("state", msg);
}




