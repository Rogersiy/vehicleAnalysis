#ifndef APPLIVE_H
#define APPLIVE_H

#include <QObject>
#include <iostream>

class QUdpSocket;

class AppLive : public QObject
{
    Q_OBJECT
public:
    static AppLive *Instance();
    explicit AppLive(QObject *parent = 0);

private:
    static QScopedPointer<AppLive> self;
    QUdpSocket *udpServer;
    QString appName;

private Q_SLOTS:
    void readData();

public Q_SLOTS:
    bool start(int port);
    void stop();
};

#endif // APPLIVE_H
