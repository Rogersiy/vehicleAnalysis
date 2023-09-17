#ifndef MONITOR_H
#define MONITOR_H

#include <QObject>
#include "qtimer.h"
#include "qudpsocket.h"
#include "qsharedmemory.h"
#include "qprocess.h"
#include "qdatetime.h"
#include "app.h"

class QUdpSocket;

class monitor : public QObject
{
    Q_OBJECT
public:
    explicit monitor(QObject *parent = nullptr);

signals:

public slots:
    void sendHearData();
    void readData();
    void killApp();
    void startApp();

private:
    QTimer *timerHeart;     //心跳定时器
    QUdpSocket *udp;        //UDP通信对象
    int count;              //计数
    bool ok;                //是否正常
};

#endif // MONITOR_H
