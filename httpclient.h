#ifndef HTTPCLIENT_H
#define HTTPCLIENT_H

#include <QObject>
#include <QThread>
#include <curl/curl.h>

#include "global.h"

class HttpClient : public QObject
{
    Q_OBJECT
public:
    explicit HttpClient();
    ~HttpClient();

public Q_SLOTS:
    void slotPushDownloadFileInfo(DownloadFileInfo downloadFileInfo);

Q_SIGNALS:

private:
    QThread thread_;
    CURL *curl;
};

#endif // HTTPCLIENT_H
