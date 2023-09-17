#include "httpclient.h"

static size_t pushDataCallback(void *ptr, size_t size, size_t nmemb, void *stream)
{
    // 获取到的body存放在ptr中，先将其转换为string格式
    std::string s((char *) ptr, size * nmemb);
    QJsonParseError json_error;
    QJsonDocument json_doc = QJsonDocument::fromJson(s.c_str(), &json_error);
    if (json_error.error != QJsonParseError::NoError){
        qWarning() << "数据推送:json parse error!";
        g_data_integrity_abnormal_count++;
    }
    QJsonObject json_object = json_doc.object();
    if (!(json_object.contains("code") && json_object["code"].toInt() == 1000)){
        qWarning() << "数据推送失败:" << QString::fromStdString(s);
        g_data_integrity_abnormal_count++;
    }
    else{
//        qDebug() << "数据推送成功!";
    }
    return size * nmemb;
}

HttpClient::HttpClient()
{
    curl = curl_easy_init();
    qDebug() << "http客户端初始化成功!";

    moveToThread(&thread_);
    thread_.start();
}

HttpClient::~HttpClient(){
    curl_easy_cleanup(this->curl);
    curl_global_cleanup();

    thread_.quit();
    thread_.wait();

    qDebug() << "http客户端退出";
}

void HttpClient::slotPushDownloadFileInfo(DownloadFileInfo downloadFileInfo){
    QJsonObject jsonobj;
    jsonobj.insert("fileIndex", downloadFileInfo.fileIndex.c_str());
    jsonobj.insert("progressInPercent", downloadFileInfo.progressInPercent);
    jsonobj.insert("downloadFileEvent", downloadFileInfo.downloadFileEvent);
    jsonobj.insert("dataLen", downloadFileInfo.dataLen);
    jsonobj.insert("data", downloadFileInfo.data.c_str());
    jsonobj.insert("deviceKey", g_serial_number.toStdString().c_str());
    QJsonDocument doc(jsonobj);
    QByteArray byte_array = doc.toJson(QJsonDocument::Compact);
    std::string formData(byte_array.data(),byte_array.length());
    //report
    string url = g_platform_data_push_address.toStdString();
    //string url = "http://192.168.100.13:8118/uavPlat/data/upload";

    CURLcode result_code;
    int error_code = 0;

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10);
        curl_easy_setopt(curl, CURLOPT_URL, url.data());
        curl_easy_setopt(curl, CURLOPT_POST, 1);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Accept: application/json");
        headers = curl_slist_append(headers, "Content-Type:application/json");
        headers = curl_slist_append(headers, "charsets: utf-8");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, (char*)formData.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, pushDataCallback);

        result_code = curl_easy_perform(curl);
        if (result_code != CURLE_OK) {
            qWarning() << "数据推送失败!";
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                    curl_easy_strerror(result_code));
        }
        curl_slist_free_all(headers);
    }
}
