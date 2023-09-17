#include "monitor.h"

monitor::monitor(QObject *parent) : QObject(parent)
{
    count = 0;
    ok = false;

    //每秒钟定时询问心跳
    timerHeart = new QTimer(this);
    timerHeart->setInterval(2000);
    connect(timerHeart, SIGNAL(timeout()), this, SLOT(sendHearData()));

    //从6050端口开始,如果绑定失败则将端口加1,直到绑定成功
    udp = new QUdpSocket(this);
    int port = 6050;
    while(!udp->bind(port)) {
        port++;
    }
    connect(udp, SIGNAL(readyRead()), this, SLOT(readData()));
}


void monitor::sendHearData()
{
    udp->writeDatagram("hello", QHostAddress::LocalHost, App::TargetAppPort);

    //判断当前是否没有回复
    if (!ok) {
        count++;
    } else {
        count = 0;
        ok = false;
    }

    //如果超过规定次数没有收到心跳回复,则超时重启
    if (count >= App::TimeoutCount) {
        timerHeart->stop();

        QSharedMemory mem(App::TargetAppName);
        if (!mem.create(1)) {
            killApp();
        }

        QTimer::singleShot(1000 , this, SLOT(killApp()));
        QTimer::singleShot(3000 , this, SLOT(startApp()));
    }
}

void monitor::killApp()
{
    QProcess *p = new QProcess;
    p->start(QString("killall vehicleAnalysis"));
    timerHeart->stop();
}

void monitor::startApp()
{
    QProcess *p = new QProcess;
    qDebug() << QString("%1/%2").arg(App::TargetAppPath).arg(App::TargetAppName);
    p->start(QString("%1/%2").arg(App::TargetAppPath).arg(App::TargetAppName));

    count = 0;
    ok = true;
    timerHeart->start();

    App::ReStartCount++;
    App::ReStartLastTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    App::writeConfig();

    qDebug() << QString("已重启 %1 次").arg(App::ReStartCount);
    qDebug() << QString("最后一次重启在 %1").arg(App::ReStartLastTime);
}

void monitor::readData()
{
    QByteArray tempData;
    do {
        tempData.resize(udp->pendingDatagramSize());
        udp->readDatagram(tempData.data(), tempData.size());
        QString data = QLatin1String(tempData);
        if (data.right(2) == "OK") {
            count = 0;
            ok = true;
        }
    } while (udp->hasPendingDatagrams());
}
