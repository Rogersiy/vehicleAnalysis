#ifndef WEBSOCKETCLIENT_H
#define WEBSOCKETCLIENT_H

#include <QThread>
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <sio_client.h>
#include <sio_socket.h>
#include <sio_message.h>

#include "global.h"

#define BIND_EVENT(IO,EV,FN) \
    IO->on(EV,FN)


using namespace std;
using namespace sio;

class WebSocketClient: public QObject
{
    Q_OBJECT //The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and slots or that uses other services provided by Qt's meta-object system
public:
    WebSocketClient();
    ~WebSocketClient();
    void OnNewMessage(std::string const& name,sio::message::ptr const& data,bool hasAck, sio::message::list &ack_resp); //接收数据
    void OnConnected(std::string const& nsp);
    void OnClosed(sio::client::close_reason const& reason);
    void OnFailed();


public Q_SLOTS:
    void slotConnectUrl(QString url);           // 连接websocket服务器的URL
    void slotClose();                           // 关闭websocket
    void slotsendDroneInfo(DroneInfo drone);    // 发送drone类型的消息

Q_SIGNALS:

private:
    QThread thread_;
    sio::client *socket_t;
};

#endif // ANALYSIS_H
